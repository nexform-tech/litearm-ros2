#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Host-side link layer for litearm-stm32: port opening, non-blocking I/O,
request/response.

Responsibilities and boundaries
--------------------------------
This module is the **transport and status-cache** shell around
:mod:`litearm_ros2_control.stm32_proto`: which command gets sent when, and by whom, is
the business of :mod:`litearm_ros2_control.hw_daemon`.

Why not pyserial
----------------
``/dev/ttyACM0`` is an ordinary character device: ``os.open`` + raw termios + ``select``
is all that is needed, and CDC ignores the baud rate anyway. Sticking to the stdlib has
three practical benefits:

1. This environment (and any machine with only the base ROS dependencies) **has no
   pyserial**, and ``hw_daemon`` is started by ``ros2 launch`` — it should not fail
   because one serial library is missing.
2. The fake firmware on a pty (``fake_firmware.py``) and a real ``/dev/ttyACM0`` go
   through **the same** code path, so a hardware-free dry run really does exercise the
   production path.
3. One less rosdep dependency.

Non-blocking discipline
-----------------------
The firmware's TX is "drop when busy" (``usb_send_raw`` in ``usb_cmd.c``), so **status
frames, ACKs and ERRs can all be lost**. Two rules follow from that:

* **Never block waiting for a reply** inside the control loop —
  :meth:`Stm32Link.send` only writes bytes;
* Only startup-time configuration queries use :meth:`Stm32Link.request` (short timeout
  + retries allowed), and a timeout returns ``None`` instead of raising, leaving it to
  the caller to decide how many times to retry.

DTR/RTS are raised along the way: some real USB CDC devices only send data once they
see DTR. A pty does not support that ioctl, so failure is ignored.
"""

import errno
import fcntl
import glob
import logging
import os
import select
import struct
import termios
import time
from typing import List, Optional, Sequence, Tuple

from litearm_ros2_control import stm32_proto as proto
from litearm_ros2_control.stm32_proto import Frame, StatusFrame

log = logging.getLogger("litearm.stm32_link")

# VID:PID of the on-board USB CDC (same as arm_audit/arm_console/_proto in tools/).
DEFAULT_VID = 0x1D50
DEFAULT_PID = 0x606F

# Cache depth for non-status replies. Startup configuration queries are at most
# 7 joints × a few commands, so 32 is enough for the "fire them all off, then collect
# them one by one" scenario.
REPLY_BACKLOG = 32

# How many bytes one poll reads at most. A 153B status frame @100Hz ≈ 15KB/s, so 4KB
# per call is already far more than one control cycle produces.
READ_CHUNK = 4096


class Stm32Error(RuntimeError):
    """Base class for link-layer errors."""


class Stm32NotConnected(Stm32Error):
    """The port is not open, or the link has dropped."""


class Stm32AccessDenied(Stm32NotConnected):
    """The port exists but cannot be opened without permission.

    It gets a type of its own because **it will not fix itself**: waiting and retrying
    change nothing, someone has to add the user to the group
    (`usermod -aG dialout $USER`) and log back in. Mixing it into one infinite retry
    loop with "the board is not plugged in yet" (which does fix itself on plug-in)
    only floods the log with the same useless line.
    """

    REMEDY = ("serial port needs dialout group permission:\n"
              "    sudo usermod -aG dialout $USER\n"
              "  then **log out and back in** (group membership applies at login).\n"
              "  temporary check (lost after replug): sudo chmod 666 <device path>")


class Stm32CmdError(Stm32Error):
    """The firmware replied ``RSP_ERR`` — the command was rejected; carries
    (command number, reason code)."""

    def __init__(self, cmd: int, reason: int) -> None:
        self.cmd = cmd
        self.reason = reason
        super().__init__(
            f"Firmware rejected command 0x{cmd:02X}: ERR{{{cmd:#04x},{reason:#04x}}}"
            f" ({ERR_HINTS.get(reason, 'reason code not catalogued')})")


# Reason code cheat sheet (the branches in usb_cmd.h / usb_cmd.c). Turns an ERR into
# something readable.
ERR_HINTS = {
    0x01: "payload too short",
    0x02: "invalid or out-of-range argument",
    0x03: "not enabled, or emergency stop latched",
    0x04: "must be disabled first (enabling / enable in flight)",
    0x05: "flash save in progress, retry later",
    0x06: "link-loss rigid hold latched",
    0x07: "mask mismatch (0x32 model commit)",
    0x08: "license not activated",
}


def find_port(vendor: int = DEFAULT_VID,
              product: int = DEFAULT_PID) -> Optional[str]:
    """Scan ``/sys/class/tty`` by USB VID:PID to find a tty device node.

    pyserial's ``list_ports`` walks the same sysfs on Linux. Returns ``None`` when
    nothing is found; the caller should then suggest passing ``--port`` explicitly.
    """
    for entry in sorted(glob.glob("/sys/class/tty/*")):
        name = os.path.basename(entry)
        if not name.startswith(("ttyACM", "ttyUSB")):
            continue
        node = os.path.realpath(os.path.join(entry, "device"))
        for _ in range(6):  # walk up from the tty device to the USB interface dir
            vid_path = os.path.join(node, "idVendor")
            pid_path = os.path.join(node, "idProduct")
            if os.path.exists(vid_path) and os.path.exists(pid_path):
                try:
                    with open(vid_path, encoding="ascii") as handle:
                        vid = int(handle.read().strip(), 16)
                    with open(pid_path, encoding="ascii") as handle:
                        pid = int(handle.read().strip(), 16)
                except (OSError, ValueError):
                    break
                if (vid, pid) == (vendor, product):
                    return f"/dev/{name}"
                break
            parent = os.path.dirname(node)
            if parent == node:
                break
            node = parent
    return None


def _set_raw(fd: int) -> None:
    """Put fd into raw mode (no echo / no line discipline / no flow control).

    The baud rate is **left alone**: CDC ignores it, a pty has no such notion, and
    passing ``termios.tcsetattr`` a rate the device does not support only makes it
    fail.
    """
    attrs = termios.tcgetattr(fd)
    iflag, oflag, cflag, lflag = attrs[0], attrs[1], attrs[2], attrs[3]
    iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK
               | termios.ISTRIP | termios.INLCR | termios.IGNCR
               | termios.ICRNL | termios.IXON | termios.IXOFF | termios.IXANY)
    oflag &= ~termios.OPOST
    lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON
               | termios.ISIG | termios.IEXTEN)
    cflag &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)
    cflag |= termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[0], attrs[1], attrs[2], attrs[3] = iflag, oflag, cflag, lflag
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def _raise_dtr(fd: int) -> None:
    """Best-effort raise of DTR/RTS. On a pty this ioctl fails; just ignore it."""
    try:
        fcntl.ioctl(fd, termios.TIOCMBIS,
                    struct.pack("I", termios.TIOCM_DTR | termios.TIOCM_RTS))
    except OSError:
        pass


class Stm32Link:
    """A CDC link to the litearm-stm32 firmware.

    Every message is multiplexed over **one and the same wire**: status frames (pushed
    proactively at 100Hz) and command replies arrive interleaved, are split apart by
    :class:`~litearm_ros2_control.stm32_proto.CdcFramer`, and :meth:`poll` then updates
    :attr:`status` from the status frames and queues the replies for collection.
    """

    def __init__(self, port: Optional[str] = None, *,
                 read_timeout_s: float = 0.0) -> None:
        self.port = port
        self.read_timeout_s = float(read_timeout_s)
        self._fd: Optional[int] = None
        self._framer = proto.CdcFramer()
        self._replies: List[Frame] = []
        self._status: Optional[StatusFrame] = None
        self._status_count = 0
        # Diagnostic counters (reported out periodically by the daemon; dropped TX
        # frames are the norm on this link, not an anomaly).
        self.tx_frames = 0
        self.tx_bytes = 0
        self.rx_frames = 0
        self.crc_errors = 0
        self.errors = 0

    # ────────────────────────── Lifecycle ──────────────────────────

    @property
    def is_open(self) -> bool:
        return self._fd is not None

    def open(self, port: Optional[str] = None) -> str:
        """Open the port in raw mode and return the device path actually used."""
        if self._fd is not None:
            return self.port or ""
        path = port or self.port or find_port()
        if not path:
            raise Stm32NotConnected(
                f"no litearm-stm32 USB CDC device found"
                f" (VID:PID {DEFAULT_VID:04x}:{DEFAULT_PID:04x}); "
                f"pass --port explicitly, or check that the board is powered, "
                f"the USB cable is connected, and no other process holds the port")
        flags = os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK
        try:
            self._fd = os.open(path, flags)
        except PermissionError as exc:
            raise Stm32AccessDenied(
                f"cannot open {path}: permission denied ({exc.strerror}).\n"
                f"  {Stm32AccessDenied.REMEDY}") from exc
        try:
            _set_raw(self._fd)
            _raise_dtr(self._fd)
        except BaseException:
            os.close(self._fd)
            self._fd = None
            raise
        self.port = path
        self._framer.reset()
        self._replies.clear()
        log.info("opened %s", path)
        return path

    def close(self) -> None:
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None
            log.info("closed %s", self.port)

    # ────────────────────────── TX/RX ──────────────────────────

    def _read_available(self) -> bytes:
        """Non-blocking drain of everything currently readable (one read takes it all,
        avoiding syscall churn)."""
        assert self._fd is not None
        try:
            return os.read(self._fd, READ_CHUNK)
        except BlockingIOError:
            return b""
        except OSError as exc:
            if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                return b""
            # USB unplugged -> EIO/ENXIO/ENODEV; mark the link down and let the layer
            # above reconnect.
            self.errors += 1
            self.close()
            raise Stm32NotConnected(f"{self.port} read failed: {exc}") from exc

    def poll(self) -> int:
        """Read and dispatch all available bytes; returns the number of complete
        frames handled this call (CRC failures included).

        Non-blocking: returns 0 immediately when there is no data, never waits. The
        control loop calls this once per cycle.
        """
        if self._fd is None:
            return 0
        handled = 0
        while True:
            chunk = self._read_available()
            if not chunk:
                break
            for frame in self._framer.feed(chunk):
                handled += 1
                self.rx_frames += 1
                if not frame.crc_ok:
                    self.crc_errors += 1
                    continue
                if frame.cmd == proto.RSP_STATUS:
                    status = proto.decode_status(frame.payload)
                    if status is not None:
                        status.stamp_s = time.monotonic()
                        self._status = status
                        self._status_count += 1
                else:
                    self._replies.append(frame)
                    if len(self._replies) > REPLY_BACKLOG:
                        del self._replies[0]
        return handled

    def send(self, cmd: int, payload: bytes = b"") -> None:
        """Send one frame. **Does not wait for a reply** — for control-loop use only."""
        if self._fd is None:
            raise Stm32NotConnected("port is not open")
        data = proto.pack_frame(cmd, payload)
        try:
            os.write(self._fd, data)
        except BlockingIOError:
            # Kernel buffer full: USB CDC drops when busy as well, so count a dropped
            # frame here and carry on without blocking.
            self.errors += 1
            return
        except OSError as exc:
            self.errors += 1
            self.close()
            raise Stm32NotConnected(f"{self.port} write failed: {exc}") from exc
        self.tx_frames += 1
        self.tx_bytes += len(data)

    def request(self, cmd: int, payload: bytes = b"",
                expect: Sequence[int] = (), timeout_s: float = 0.3,
                accept_err: bool = False) -> Optional[Frame]:
        """Send one frame and wait for the expected reply. Returns ``None`` on timeout
        (**never raises**).

        For startup / configuration queries only. Inside the control loop always use
        :meth:`send`.

        ``expect`` gives the acceptable uplink command numbers; on a hit the frame is
        removed from the queue and returned. ``RSP_ERR`` counts as a hit by default too
        (that frame is returned) and is meant to be interpreted by the caller with
        :func:`~litearm_ros2_control.stm32_proto.decode_err`; with
        ``accept_err=False`` it raises :class:`Stm32CmdError` instead.
        """
        if self._fd is None:
            raise Stm32NotConnected("port is not open")
        expect = tuple(expect)
        # Drain the backlog before sending: otherwise a frame sent earlier via send()
        # whose ERR only shows up now would be taken by this request as "this command
        # was rejected" — a complete misdiagnosis (hard-failing on a stale ERR as if
        # the license were not activated, for instance).
        self.poll()
        self._replies.clear()
        self.send(cmd, payload)
        deadline = time.monotonic() + float(timeout_s)
        while True:
            self.poll()
            for index, frame in enumerate(self._replies):
                if frame.cmd in expect:
                    del self._replies[index]
                    return frame
                if frame.cmd == proto.RSP_ERR:
                    decoded = proto.decode_err(frame.payload)
                    if decoded is not None and decoded[0] == cmd:
                        del self._replies[index]
                        if accept_err:
                            return frame
                        raise Stm32CmdError(*decoded)
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return None
            select.select([self._fd], [], [], remaining)

    def command(self, cmd: int, payload: bytes = b"",
                timeout_s: float = 0.3) -> Tuple[Optional[bool], Optional[int]]:
        """Send a command and wait for ``ACK``/``ERR``; returns ``(success, reason)``.

        A timeout returns ``(None, None)`` — **that is the norm, not an anomaly**: the
        firmware's TX drops when busy, so a reply may never make it out at all. The
        caller should treat `"no reply received"` as a reason to retry, not as a
        failure.

        There is exactly one situation that needs this rather than a bare
        :meth:`send`: **when the failure reason matters**. The classic case is
        ``ENABLE``: ``ERR{0x10,0x08}`` = license not activated (retrying is pointless,
        hard failure), ``ERR{0x10,0x03}`` = feedback not ready yet (just retry).
        """
        frame = self.request(cmd, payload, expect=(proto.RSP_ACK,),
                             timeout_s=timeout_s, accept_err=True)
        if frame is None:
            return None, None
        if frame.cmd == proto.RSP_ACK:
            return True, None
        decoded = proto.decode_err(frame.payload)
        return False, (None if decoded is None else decoded[1])

    # ────────────────────────── Status ──────────────────────────

    @property
    def status(self) -> Optional[StatusFrame]:
        """Most recent status frame (pushed at 100Hz, no request needed)."""
        return self._status

    @property
    def status_count(self) -> int:
        return self._status_count

    def status_age_s(self) -> float:
        """Local age of the most recent status frame, in seconds. Returns ``inf`` when
        none has ever arrived."""
        if self._status is None:
            return float("inf")
        return time.monotonic() - self._status.stamp_s

    def wait_status(self, timeout_s: float = 2.0) -> Optional[StatusFrame]:
        """Wait for one status frame or the timeout. Used at startup to confirm the
        firmware is online and to learn the joint count."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.poll()
            if self._status is not None:
                return self._status
            select.select([self._fd], [], [], 0.01)
        return self._status

    def wait_enabled(self, timeout_s: float = 3.0) -> Optional[StatusFrame]:
        """Wait until ``flags.enabled`` is true or the timeout expires.

        The firmware's ``ENABLE`` has two stages: it first writes CMODE (replying
        ``0x03`` at that moment), and only once feedback is complete does it actually
        energize the motors (``enable_pending_poll`` in ``control_loop.c``). So an ACK
        **must not** be used to judge whether enabling succeeded — the enabled bit of
        the status frame is what counts.
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.poll()
            if self._status is not None and self._status.enabled:
                return self._status
            select.select([self._fd], [], [], 0.01)
        return self._status if (self._status and self._status.enabled) else None

    # ─────────────────────── Command wrappers ───────────────────────
    # The bare send_* wrappers never wait; only the get_* ones await a reply.

    def get_firmware(self, timeout_s: float = 0.5) -> Optional[str]:
        frame = self.request(proto.CMD_GET_FIRMWARE,
                             expect=(proto.RSP_FIRMWARE,), timeout_s=timeout_s)
        return None if frame is None else proto.decode_firmware(frame.payload)

    def get_joint_param(self, idx: int, timeout_s: float = 0.5
                        ) -> Optional[proto.JointParam]:
        frame = self.request(proto.CMD_GET_JOINT_PARAM,
                             proto.pack_get_joint_param(idx),
                             expect=(proto.RSP_JOINT_PARAM,),
                             timeout_s=timeout_s)
        return None if frame is None else proto.decode_joint_param(frame.payload)

    def get_ff_scalar(self, item: int, sub: int = 0, timeout_s: float = 0.5
                      ) -> Optional[float]:
        frame = self.request(proto.CMD_GET_FF_SCALAR,
                             proto.pack_get_ff_scalar(item, sub),
                             expect=(proto.RSP_FF_SCALAR,),
                             timeout_s=timeout_s)
        if frame is None:
            return None
        decoded = proto.decode_ff_scalar(frame.payload)
        return None if decoded is None else decoded[2]

    def get_ff_vec(self, item: int, timeout_s: float = 0.5
                   ) -> Optional[Tuple[float, ...]]:
        frame = self.request(proto.CMD_GET_FF_VEC,
                             proto.pack_get_ff_vec(item),
                             expect=(proto.RSP_FF_VEC,),
                             timeout_s=timeout_s)
        if frame is None:
            return None
        decoded = proto.decode_ff_vec(frame.payload)
        return None if decoded is None else decoded[1]

    def get_ff_mask(self, timeout_s: float = 0.5) -> Optional[int]:
        value = self.get_ff_scalar(proto.FF_SCALAR_FF_MASK, 0, timeout_s)
        return None if value is None else int(round(value))

    def enable(self) -> None:
        """Request enable. **An ACK only means "registered"** — the motors are actually
        energized once feedback is complete, see :meth:`wait_enabled`."""
        self.send(proto.CMD_ENABLE)

    def disable(self) -> None:
        self.send(proto.CMD_DISABLE)

    def emergency_stop(self) -> None:
        """Emergency stop: disables ×5 and latches it. **The arm goes limp and will
        drop**; this is not a hardware emergency stop, so do not use it lightly."""
        self.send(proto.CMD_EMERGENCY_STOP)

    def clear_faults(self) -> None:
        self.send(proto.CMD_CLEAR_FAULTS)

    def reset(self) -> None:
        self.send(proto.CMD_RESET)

    def park(self) -> None:
        """``SET_MOTION_MODE(0x20)`` with 0 = declare park.

        Once set while enabled, if commands stop arriving (the 100ms watchdog trips)
        the firmware holds position at **1.0× stiffness** instead of the fail-soft
        0.6×. This is the "declare a high-stiffness park before the PC disconnects"
        mechanism.
        ⚠ The watchdog is deliberately not kicked: after a park you stop sending
        frames, so the watchdog **will** trip — that is exactly the design.
        """
        self.send(proto.CMD_SET_MOTION_MODE, proto.pack_u8(0))

    def set_speed_percent(self, percent: int) -> None:
        """``0x21``: speed governor 0~100; only affects the speed cap of subsequent
        move_j/move_js commands."""
        self.send(proto.CMD_SET_SPEED_PERCENT, proto.pack_u8(percent))

    def set_ff_mask(self, mask: int) -> None:
        self.send(proto.CMD_SET_FF_FLAGS, proto.pack_set_ff_flags(mask))

    def set_ff_vec(self, item: int, values: Sequence[float]) -> None:
        self.send(proto.CMD_SET_FF_VEC, proto.pack_set_ff_vec(item, values))

    def set_ff_scalar(self, item: int, value: float, sub: int = 0) -> None:
        self.send(proto.CMD_SET_FF_SCALAR,
                  proto.pack_set_ff_scalar(item, sub, value))

    def move_js(self, q: Sequence[float], dq: Sequence[float],
                tau: Optional[Sequence[float]] = None) -> None:
        """Streaming joint servo. **Must be re-sent every cycle** (it does not kick the
        watchdog by itself).

        ``tau=None`` → the firmware adds its built-in feedforward; supplying tau (even
        all zeros) → the built-in feedforward is switched off entirely.
        """
        self.send(proto.CMD_MOVE_JS, proto.pack_move_js(q, dq, tau))

    def move_mit_all(self, rows: Sequence[Sequence[float]]) -> None:
        """Whole-arm MIT passthrough. **Must be re-sent every cycle**; the firmware
        adds none of its own feedforward."""
        self.send(proto.CMD_MOVE_MIT_ALL, proto.pack_move_mit_all(rows))

    # ────────────────────────── Diagnostics ──────────────────────────

    def counters(self) -> str:
        return (f"tx={self.tx_frames} frames rx={self.rx_frames} frames "
                f"crc_err={self.crc_errors} discarded={self._framer.discarded}B "
                f"other errors={self.errors}")
