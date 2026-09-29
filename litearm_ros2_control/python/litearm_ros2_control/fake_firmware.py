#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Behavioural stand-in for the litearm-stm32 firmware (USB CDC protocol over a pty).

Purpose
-------
1. **Hardware-free dry runs**: ``--dry-run`` lets the daemon speak the real protocol to
   this fake firmware instead of a board, so protocol encode/decode, status frames,
   watchdog and mode switching all get exercised end to end — far more faithful than
   "wrap a kinematic model around the protocol".
2. **Test fixture**: faults/staleness/dropped frames can be injected to force out
   branches that are hard to reproduce on real hardware.

Discipline (why it is worth taking seriously)
--------------------------------------------
* **Hand-written on the encode side**: status frames/replies are assembled directly
  with ``struct.pack``, **not by reusing ``stm32_proto``'s packers** — otherwise the
  same bug on both sides would hide itself, and the round-trip test would be proving
  its own assumption. True byte-level agreement is the job of the golden vector tests.
* **Protocol only, no dynamics**: the joint response is a first-order lag
  (``q ← q + (q_ref−q)·dt/τ``), which validates the interface and the data path; it
  **must not be used to tune gains**. On real hardware the dynamics live in the
  firmware and the feedforward model is generated from the URDF — none of that exists
  here.

The default parameters come from the firmware's ``params/defaults.c`` (7-joint full-arm
build), so the kp/kd/tau_max/limits the daemon reads back are exactly the values a real
board reports — tests can assert against them.
"""

import errno
import math
import os
import select
import struct
import threading
import time
from typing import Dict, List, Optional, Sequence, Tuple

# Factory values from the firmware's params/defaults.c (Litearm1.8.0-7J).
FW_VERSION = "Litearm1.8.0-7J"

DEFAULT_KP = (400.0, 400.0, 300.0, 300.0, 50.0, 50.0, 50.0)
DEFAULT_KD = (5.0, 5.0, 4.0, 5.0, 2.5, 2.5, 2.5)
DEFAULT_TAU_MAX = (78.0, 78.0, 21.0, 21.0, 10.0, 10.0, 10.0)
DEFAULT_Q_MIN = (-2.809547, -1.727547, -2.809547, -3.071547,
                 -2.809547, -1.553547, -1.553547)
DEFAULT_Q_MAX = (2.809547, 1.727547, 2.809547, 0.017547,
                 2.809547, 1.553547, 1.553547)
DEFAULT_VEL_MAX = (2.0, 2.0, 1.75, 1.75, 2.0, 2.0, 2.0)
DEFAULT_KD_EXTRA = (6.0, 6.0, 6.0, 6.0, 0.0, 0.0, 0.0)
DEFAULT_KI = (40.0, 40.0, 25.0, 25.0, 10.0, 10.0, 10.0)
DEFAULT_I_MAX = (20.0, 20.0, 6.0, 6.0, 4.0, 3.0, 3.0)

# Factory ff_mask = FF_MASTER|FF_G|FF_INERTIA|FF_CORIOLIS|FF_INTEGRAL|FF_QUANT
#                            |FF_VELREF|FF_FRICTION  = 0x1BF.
# ⚠ FF_INERTIA is 0x04 and FF_WALL is 0x40; one bit off and you get "wall but no
#   inertia" — cross-check against proto.FF_FACTORY_MASK in tests, never edit from
#   memory.
# There is also 0x1AF = the above minus FRICTION (the "fallback = ff_mask 431" in the
# defaults.c comment).
DEFAULT_FF_MASK = 0x1BF

# Firmware constants (litearm.h / usb_cmd.h).
SOF = 0xA5
CMD_MOVE_J = 0x01
CMD_MOVE_JS = 0x03
CMD_MOVE_MIT = 0x04
CMD_MOVE_MIT_ALL = 0x05
CMD_ZERO_G = 0x06
CMD_ENABLE = 0x10
CMD_DISABLE = 0x11
CMD_EMERGENCY_STOP = 0x12
CMD_CLEAR_FAULTS = 0x13
CMD_RESET = 0x14
CMD_SET_MOTION_MODE = 0x20
CMD_SET_SPEED_PERCENT = 0x21
CMD_SET_JOINT_PARAM = 0x22
CMD_SET_JOINT_LIMITS = 0x23
CMD_GET_JOINT_PARAM = 0x24
CMD_SET_FF_VEC = 0x26
CMD_SET_FF_FLAGS = 0x27
CMD_SET_FF_SCALAR = 0x28
CMD_GET_FF_VEC = 0x2B
CMD_GET_FF_SCALAR = 0x2C
CMD_FF_PRESET = 0x31
CMD_GET_STATUS = 0x40
CMD_GET_FIRMWARE = 0x41

RSP_STATUS = 0x40
RSP_FIRMWARE = 0x44
RSP_ACK = 0x45
RSP_ERR = 0x46
RSP_JOINT_PARAM = 0x49
RSP_FF_VEC = 0x4B
RSP_FF_SCALAR = 0x4C

MODE_INIT = 0
MODE_MOVE_J = 1
MODE_MOVE_JS = 3
MODE_MOVE_MIT = 4
MODE_MOVE_MIT_ALL = 5
MODE_EMERGENCY = 6
MODE_ZERO_G = 7

FLAG_FAULT = 1 << 0
FLAG_WATCHDOG_TRIPPED = 1 << 1
FLAG_FEEDBACK_STALE = 1 << 2
FLAG_TEMP_WARNING = 1 << 3
FLAG_POSITION_VIOLATION = 1 << 4
FLAG_OVERSPEED = 1 << 5

# Command watchdog 0.10s (params/defaults.c).
WATCHDOG_TIMEOUT_S = 0.10

# Status frame reporting period (RPT_STATUS_MS=10 in usb_cmd.c → 100Hz).
REPORT_PERIOD_S = 0.01

# Time constant of the first-order response (**a kinematic approximation only**, not
# dynamics).
JOINT_LAG_S = 0.08


def _crc16(data: bytes) -> int:
    """CRC16-CCITT-FALSE. Matches the firmware's crc16.c (independent golden vectors in
    the tests)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 \
                else (crc << 1) & 0xFFFF
    return crc


def _frame(cmd: int, payload: bytes = b"") -> bytes:
    body = bytes([SOF, cmd, len(payload)]) + payload
    crc = _crc16(body)
    return body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


class FakeFirmware:
    """Simulates a board running Litearm1.8.0-7J on a pty.

    Usage::

        fw = FakeFirmware().start()        # start() returns the path the client opens
        link.open(fw)
        link.enable(); link.wait_enabled()
        ...
        fw.stop()

    Threading model: one background thread steps at ``tick_hz`` (the firmware uses
    TIM3 at 300Hz), and on every step it takes in commands, advances the joints and
    emits status frames at 100Hz.
    """

    def __init__(self, num_joints: int = 7, *, licensed: bool = True,
                 tick_hz: float = 300.0, report_period_s: float = REPORT_PERIOD_S,
                 enable_delay_s: float = 0.05) -> None:
        self.n = int(num_joints)
        self.licensed = bool(licensed)
        self.tick_hz = float(tick_hz)
        self.report_period_s = float(report_period_s)
        # The firmware's ENABLE has two stages: write CMODE first, energize only once
        # feedback is complete. A fixed delay stands in for that here.
        self.enable_delay_s = float(enable_delay_s)

        # Joint parameters (what 0x22/0x23/0x24 read and write). Only the first n are
        # taken.
        self.kp = list(DEFAULT_KP[:self.n])
        self.kd = list(DEFAULT_KD[:self.n])
        self.tau_max = list(DEFAULT_TAU_MAX[:self.n])
        self.q_min = list(DEFAULT_Q_MIN[:self.n])
        self.q_max = list(DEFAULT_Q_MAX[:self.n])
        self.vel_max = list(DEFAULT_VEL_MAX[:self.n])
        self.ff_mask = DEFAULT_FF_MASK
        self.ff_vec: Dict[int, List[float]] = {
            1: [0.0] * self.n,          # friction (Coulomb along the error direction)
            2: list(DEFAULT_KI[:self.n]),
            3: list(DEFAULT_I_MAX[:self.n]),
            7: [1.0] * self.n,          # gravity_scale
            8: [1.0] * self.n,          # inertia_scale
            15: list(DEFAULT_KD_EXTRA[:self.n]),
        }
        self.ff_scalar: Dict[Tuple[int, int], float] = {
            (1, 0): 0.15,               # fric_db
            (3, 0): 60.0,               # friction_slew
            (4, 0): 0.0,                # payload_mass
            (7, 0): 1.0,                # friction_model = v2
            (8, 0): 0.05,               # fric_v2_eps
            (18, 0): 2.0,               # hold_kp_gain
        }
        self.speed_percent = 100

        # Motion state
        self.q = [-0.4, 1.3, -0.9, 0.0, 0.5, -0.2, 0.1][:self.n]
        self.dq = [0.0] * self.n
        self.tau = [0.0] * self.n
        self.t_mos = [32.0] * self.n
        self.t_coil = [35.0] * self.n
        self.q_ref = list(self.q)
        self.js_target_q = list(self.q)
        self.dq_ref = [0.0] * self.n
        self.tau_user = [0.0] * self.n
        self.kp_cmd = list(self.kp)
        self.kd_cmd = list(self.kd)

        self.mode = MODE_INIT
        self.enabled = False
        self.enable_pending_at: Optional[float] = None
        # Synonymous with the firmware's ctrl_mode_written: whether the motor CMODE has
        # been written since the firmware booted. While it has not, the first ENABLE is
        # guaranteed to answer 0x03 (see _cmd_enable).
        self._cmode_written = False
        self.park_requested = False
        self.joint_fault = 0
        self.seq = 0
        self.watchdog_tripped = False
        self.last_kick_s = 0.0
        self.s_js_user_ff = False
        self.err_code = [0] * self.n       # 0=disabled, 1=enabled, else=fault code

        # Fault injection (for tests)
        self._inj_feedback_stale = False
        self._inj_temp_warning = False
        self._inj_overspeed = False
        self._inj_pos_violation = False
        self._drop_replies = 0

        # Observation outlets (for test assertions)
        self.command_log: List[Tuple[int, bytes]] = []
        self.move_js_log: List[Tuple[List[float], List[float],
                                     Optional[List[float]]]] = []

        self._master: Optional[int] = None
        self._slave: Optional[int] = None
        self._port: Optional[str] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._rx = bytearray()
        self._lock = threading.Lock()

    # ────────────────────────── Lifecycle ──────────────────────────

    @property
    def port(self) -> Optional[str]:
        """The path the client opens (``/dev/pts/N``)."""
        return self._port

    def start(self) -> str:
        """Create the pty, start the background thread, return the client path."""
        master, slave = os.openpty()
        # Raw mode has to be set from our side; otherwise the default ECHO echoes the
        # client's own commands back to it (CdcFramer copes with that, but it pollutes
        # the diagnostic counters).
        self._set_raw(slave)
        self._master, self._slave = master, slave
        self._port = os.ttyname(slave)
        self.last_kick_s = time.monotonic()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="fake-firmware",
                                        daemon=True)
        self._thread.start()
        return self._port

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        for fd in (self._master, self._slave):
            if fd is not None:
                try:
                    os.close(fd)
                except OSError:
                    pass
        self._master = self._slave = None

    def __enter__(self) -> "FakeFirmware":
        self.start()
        return self

    def __exit__(self, *_exc) -> None:
        self.stop()

    @staticmethod
    def _set_raw(fd: int) -> None:
        import termios
        attrs = termios.tcgetattr(fd)
        attrs[0] &= ~(termios.ICRNL | termios.IXON)
        attrs[1] &= ~termios.OPOST
        attrs[3] &= ~(termios.ECHO | termios.ICANON | termios.ISIG)
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)

    # ─────────────────────── Fault/behaviour injection ───────────────────────

    def inject_joint_fault(self, index: int, on: bool = True) -> None:
        """Set/clear a ``joint_fault`` bit (a single dead axis disables only that
        axis in the firmware)."""
        if on:
            self.joint_fault |= 1 << index
            self.err_code[index] = 0x0D
        else:
            self.joint_fault &= ~(1 << index)
            self.err_code[index] = 1 if self.enabled else 0

    def inject_feedback_stale(self, on: bool = True) -> None:
        self._inj_feedback_stale = bool(on)

    def inject_temp_warning(self, on: bool = True) -> None:
        self._inj_temp_warning = bool(on)

    def inject_overspeed(self, on: bool = True) -> None:
        self._inj_overspeed = bool(on)

    def inject_position_violation(self, on: bool = True) -> None:
        self._inj_pos_violation = bool(on)

    def drop_replies(self, count: int = 1) -> None:
        """Drop the next ``count`` **replies** (simulating the firmware's drop-when-busy
        TX).

        Only ACK/ERR/read replies are dropped, never status frames — otherwise "no ACK
        arriving" and "no status arriving" would be conflated, and the branch under
        test could not be isolated.
        """
        self._drop_replies = int(count)

    def set_position(self, values: Sequence[float]) -> None:
        """Set the joint positions directly (test setup, bypassing the motion
        simulation)."""
        with self._lock:
            self.q = [float(v) for v in values]
            self.q_ref = list(self.q)

    def snapshot(self) -> Dict[str, object]:
        with self._lock:
            return {
                "mode": self.mode, "enabled": self.enabled,
                "q": list(self.q), "dq": list(self.dq),
                "q_ref": list(self.q_ref), "tau_user": list(self.tau_user),
                "kp_cmd": list(self.kp_cmd), "kd_cmd": list(self.kd_cmd),
                "watchdog_tripped": self.watchdog_tripped,
                "joint_fault": self.joint_fault,
                "s_js_user_ff": self.s_js_user_ff,
                "ff_mask": self.ff_mask,
            }

    # ────────────────────────── Main loop ──────────────────────────

    def _run(self) -> None:
        tick = 1.0 / self.tick_hz
        next_tick = time.monotonic()
        next_report = next_tick
        last = next_tick
        while not self._stop.is_set():
            now = time.monotonic()
            dt = max(1e-6, now - last)
            last = now
            self._receive()
            self._step(dt, now)
            if now >= next_report:
                next_report = now + self.report_period_s
                self._send(RSP_STATUS, self._build_status(), droppable=False)
            next_tick += tick
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            elif sleep_s < -tick:      # fell too far behind: re-anchor, don't race
                next_tick = time.monotonic()

    def _receive(self) -> None:
        if self._master is None:
            return
        while True:
            try:
                ready = select.select([self._master], [], [], 0)[0]
                if not ready:
                    return
                chunk = os.read(self._master, 4096)
            except (BlockingIOError, OSError) as exc:
                if isinstance(exc, OSError) and exc.errno not in (
                        errno.EAGAIN, errno.EWOULDBLOCK, errno.EIO):
                    raise
                return
            if not chunk:
                return
            self._rx += chunk
            self._drain_frames()

    def _drain_frames(self) -> None:
        while len(self._rx) >= 3:
            if self._rx[0] != SOF:
                del self._rx[0]
                continue
            length = self._rx[2]
            need = 3 + length + 2
            if len(self._rx) < need:
                return
            body = bytes(self._rx[:3 + length])
            crc = self._rx[3 + length] | (self._rx[4 + length] << 8)
            payload = bytes(self._rx[3:3 + length])
            del self._rx[:need]
            if _crc16(body) != crc:
                continue            # drop bad frames, no ERR (firmware only counts CRC)
            cmd = body[1]
            with self._lock:
                self.command_log.append((cmd, payload))
            self._handle(cmd, payload)

    # ────────────────────────── Command handling ──────────────────────────

    def _handle(self, cmd: int, payload: bytes) -> None:
        now = time.monotonic()
        with self._lock:
            if cmd == CMD_MOVE_JS:
                self._cmd_move_js(payload, now)
            elif cmd == CMD_MOVE_MIT_ALL:
                self._cmd_move_mit_all(payload, now)
            elif cmd == CMD_MOVE_J:
                self._cmd_move_j(payload, now)
            elif cmd == CMD_ENABLE:
                self._cmd_enable(now)
            elif cmd == CMD_DISABLE:
                self._cmd_disable()
            elif cmd == CMD_EMERGENCY_STOP:
                self._cmd_emergency()
            elif cmd == CMD_CLEAR_FAULTS:
                self._cmd_clear_faults()
            elif cmd == CMD_RESET:
                self._cmd_reset(now)
            elif cmd == CMD_ZERO_G:
                if len(payload) < 1:
                    self._err(cmd, 0x01)
                else:
                    self.mode = MODE_ZERO_G if payload[0] else MODE_INIT
                    self.last_kick_s = now      # 0x06 kicks the watchdog itself
                    self._ack(cmd)
            elif cmd == CMD_SET_MOTION_MODE:
                if len(payload) < 1:
                    self._err(cmd, 0x01)
                else:
                    # mode 0 = declare park; anything else clears the declaration.
                    # Deliberately does not kick the watchdog.
                    self.park_requested = payload[0] == 0
                    self._ack(cmd)
            elif cmd == CMD_SET_SPEED_PERCENT:
                if len(payload) < 1:
                    self._err(cmd, 0x01)
                else:
                    self.speed_percent = max(0, min(100, payload[0]))
                    self._ack(cmd)
            elif cmd == CMD_GET_JOINT_PARAM:
                self._cmd_get_joint_param(payload)
            elif cmd == CMD_SET_JOINT_PARAM:
                self._cmd_set_joint_param(payload)
            elif cmd == CMD_SET_JOINT_LIMITS:
                self._cmd_set_joint_limits(payload)
            elif cmd == CMD_SET_FF_FLAGS:
                if len(payload) < 4:
                    self._err(cmd, 0x01)
                else:
                    self.ff_mask = struct.unpack_from("<I", payload, 0)[0]
                    self._ack(cmd)
            elif cmd == CMD_SET_FF_VEC:
                self._cmd_set_ff_vec(payload)
            elif cmd == CMD_SET_FF_SCALAR:
                self._cmd_set_ff_scalar(payload)
            elif cmd == CMD_GET_FF_VEC:
                self._cmd_get_ff_vec(payload)
            elif cmd == CMD_GET_FF_SCALAR:
                self._cmd_get_ff_scalar(payload)
            elif cmd == CMD_FF_PRESET:
                if len(payload) < 1 or payload[0] > 2:
                    self._err(cmd, 0x02)
                else:
                    self.ff_mask = (0 if payload[0] == 0
                                    else 0x1F if payload[0] == 1
                                    else 0x1FF)
                    self._ack(cmd)
            elif cmd == CMD_GET_STATUS:
                self._send(RSP_STATUS, self._build_status(), droppable=False)
            elif cmd == CMD_GET_FIRMWARE:
                self._send(RSP_FIRMWARE, FW_VERSION.encode("ascii"))
            else:
                self._err(cmd, 0x00)    # no such command in the firmware

    # ── Motion commands ──

    def _require_enabled(self, cmd: int) -> bool:
        """The firmware's single gate for motion commands: not enabled, or emergency
        stop latched, always yields ERR{cmd,0x03}."""
        if not self.enabled or self.mode == MODE_EMERGENCY:
            self._err(cmd, 0x03)
            return False
        return True

    def _cmd_move_js(self, payload: bytes, now: float) -> None:
        n = self.n
        if len(payload) not in (8 * n, 12 * n):
            self._err(CMD_MOVE_JS, 0x01)
            return
        if not self._require_enabled(CMD_MOVE_JS):
            return
        q = list(struct.unpack_from(f"<{n}f", payload, 0))
        dq = list(struct.unpack_from(f"<{n}f", payload, 4 * n))
        tau = None
        if len(payload) == 12 * n:
            tau = list(struct.unpack_from(f"<{n}f", payload, 8 * n))
        # Supplying tau_ff switches off the entire built-in feedforward — the single
        # most important piece of firmware semantics.
        if self.s_js_user_ff and tau is None:
            self._reset_law()
        self.s_js_user_ff = tau is not None
        self.mode = MODE_MOVE_JS
        # ⚠ MOVE_JS does not behave the way you would expect (verified on real
        #   hardware, control_loop.c:2106/2227):
        #   v_lim = clamp(|dq_ref|, 0, speed_limit)   —— dq_ref **doubles as the slew
        #                                               rate limit of the position
        #                                               reference**
        #   cmd->q_ref = slew_linear(target_q, cmd->q_ref, v_lim * dt)
        #   i.e. each tick the reference advances at most v_lim·dt towards target_q.
        #   ⇒ **with dq_ref=0 the reference does not move at all, and neither does the
        #     arm**. This field is that target_q; the actual reference is self.q_ref,
        #     advanced by the slew in _step.
        # This was missed once: the fake firmware used to drive q_ref with a
        # first-order lag instead, so "dq=0 still moves" and it gave false
        # positives — on real hardware it fell apart immediately.
        self.js_target_q = list(q)
        self.dq_ref = dq
        self.tau_user = tau or [0.0] * n
        self.last_kick_s = now
        self.move_js_log.append((list(q), list(dq),
                                 None if tau is None else list(tau)))

    def _cmd_move_mit_all(self, payload: bytes, now: float) -> None:
        n = self.n
        if len(payload) < 20 * n:
            self._err(CMD_MOVE_MIT_ALL, 0x01)
            return
        if not self._require_enabled(CMD_MOVE_MIT_ALL):
            return
        # SoA read (as the firmware does). Packing as AoS makes this decode to junk.
        self.q_ref = list(struct.unpack_from(f"<{n}f", payload, 0))
        self.dq_ref = list(struct.unpack_from(f"<{n}f", payload, 4 * n))
        self.kp_cmd = list(struct.unpack_from(f"<{n}f", payload, 8 * n))
        self.kd_cmd = list(struct.unpack_from(f"<{n}f", payload, 12 * n))
        self.tau_user = list(struct.unpack_from(f"<{n}f", payload, 16 * n))
        self.mode = MODE_MOVE_MIT_ALL
        self.s_js_user_ff = False
        self.last_kick_s = now

    def _cmd_move_j(self, payload: bytes, now: float) -> None:
        n = self.n
        if len(payload) < 4 * n + 4:
            self._err(CMD_MOVE_J, 0x01)
            return
        if not self._require_enabled(CMD_MOVE_J):
            return
        self.q_ref = list(struct.unpack_from(f"<{n}f", payload, 0))
        self.dq_ref = [0.0] * n
        self.mode = MODE_MOVE_J
        self.last_kick_s = now

    # ── State control ──

    def _cmd_enable(self, now: float) -> None:
        if self.mode == MODE_EMERGENCY:
            self._err(CMD_ENABLE, 0x06)     # emergency stop latched, needs RESET
            return
        if not self.licensed:
            # The license gate is a single point: while it is not activated only
            # ENABLE is blocked, every other command is unaffected.
            self._err(CMD_ENABLE, 0x08)
            return
        if self.enabled:
            self._ack(CMD_ENABLE)           # idempotent: re-send is a no-op if enabled
            return
        if not self._cmode_written:
            # **The first ENABLE after firmware boot**: it writes the CMODE of all 7
            # motors first (switching them into MIT mode) and answers 0x03 at that
            # moment, registering the pending enable. The timing measured on real
            # hardware is in the ctrl_enable comment in control_loop.c:
            #   "ENABLE#1 -> 0x03(first CMODE write), ENABLE#2 -> ACK"
            # ⚠ This **must be reproduced in the fake firmware**: it bit us once on
            #   real hardware — if the host treats 0x03 as a failure, every cold start
            #   wastes a whole reconnect round.
            self._cmode_written = True
            self.enable_pending_at = now
            self._err(CMD_ENABLE, 0x03)
            return
        self.enable_pending_at = now
        self._ack(CMD_ENABLE)

    def _cmd_disable(self) -> None:
        self.enabled = False
        self.enable_pending_at = None
        self.err_code = [0] * self.n
        if self.mode != MODE_EMERGENCY:
            self.mode = MODE_INIT
        self._ack(CMD_DISABLE)

    def _cmd_emergency(self) -> None:
        self.enabled = False
        self.enable_pending_at = None
        self.mode = MODE_EMERGENCY
        self.err_code = [0] * self.n
        self._ack(CMD_EMERGENCY_STOP)

    def _cmd_clear_faults(self) -> None:
        # Per axis: err==1 (enabled) is left alone. The EMERGENCY latch is not
        # cleared.
        for i in range(self.n):
            if self.err_code[i] != 1:
                self.err_code[i] = 1 if self.enabled else 0
        self.joint_fault = 0
        self.q_ref = list(self.q)
        self._ack(CMD_CLEAR_FAULTS)

    def _cmd_reset(self, now: float) -> None:
        """``ctrl_reset``: **full clear, back to INIT/disabled**
        (control_loop.c:1325-1353).

        Note that it also resets ``mode`` to ``ARM_MODE_INIT``, ``enabled=false`` and
        the speed governor to 100% — it is not just "clear the fault codes". Releasing
        the emergency stop latch also relies on it.
        """
        self.enabled = False
        self.enable_pending_at = None
        self._cmode_written = False      # same as the firmware's ctrl_reset
        self.mode = MODE_INIT
        self.joint_fault = 0
        self.watchdog_tripped = False
        self.park_requested = False
        self.s_js_user_ff = False
        self.speed_percent = 100
        self.last_kick_s = now
        self.err_code = [0] * self.n
        self.tau_user = [0.0] * self.n
        self.q_ref = list(self.q)
        self._ack(CMD_RESET)

    # ── Parameters ──

    def _cmd_get_joint_param(self, payload: bytes) -> None:
        if len(payload) < 1:
            self._err(CMD_GET_JOINT_PARAM, 0x01)
            return
        idx = payload[0]
        if idx >= self.n:
            self._err(CMD_GET_JOINT_PARAM, 0x02)
            return
        # The RSP_JOINT_PARAM payload carries **no RSP prefix**: idx + 5×f32 = 21B.
        body = bytes([idx]) + struct.pack(
            "<fffff", self.kp[idx], self.kd[idx], self.tau_max[idx],
            self.q_min[idx], self.q_max[idx])
        self._send(RSP_JOINT_PARAM, body)

    def _cmd_set_joint_param(self, payload: bytes) -> None:
        if len(payload) < 13:
            self._err(CMD_SET_JOINT_PARAM, 0x01)
            return
        idx = payload[0]
        if idx >= self.n:
            self._err(CMD_SET_JOINT_PARAM, 0x02)
            return
        self.kp[idx], self.kd[idx], self.tau_max[idx] = struct.unpack_from(
            "<fff", payload, 1)
        self._ack(CMD_SET_JOINT_PARAM)

    def _cmd_set_joint_limits(self, payload: bytes) -> None:
        if len(payload) < 9:
            self._err(CMD_SET_JOINT_LIMITS, 0x01)
            return
        idx = payload[0]
        if idx >= self.n:
            self._err(CMD_SET_JOINT_LIMITS, 0x02)
            return
        q_min, q_max = struct.unpack_from("<ff", payload, 1)
        # The firmware only allows narrowing: a widening write is rejected.
        if q_min > self.q_min[idx] or q_max < self.q_max[idx]:
            self._err(CMD_SET_JOINT_LIMITS, 0x02)
            return
        self.q_min[idx], self.q_max[idx] = q_min, q_max
        self._ack(CMD_SET_JOINT_LIMITS)

    def _cmd_set_ff_vec(self, payload: bytes) -> None:
        if len(payload) < 29:
            self._err(CMD_SET_FF_VEC, 0x01)
            return
        item = payload[0]
        if item < 1 or item > 15:
            self._err(CMD_SET_FF_VEC, 0x02)
            return
        self.ff_vec[item] = list(struct.unpack_from(f"<{self.n}f", payload, 1))
        self._ack(CMD_SET_FF_VEC)

    def _cmd_set_ff_scalar(self, payload: bytes) -> None:
        if len(payload) < 6:
            self._err(CMD_SET_FF_SCALAR, 0x01)
            return
        item, sub = payload[0], payload[1]
        if item < 1 or item > 18 or item == 9:   # item 9 is read-only (ff_mask)
            self._err(CMD_SET_FF_SCALAR, 0x02)
            return
        (value,) = struct.unpack_from("<f", payload, 2)
        self.ff_scalar[(item, sub)] = value
        self._ack(CMD_SET_FF_SCALAR)

    def _cmd_get_ff_vec(self, payload: bytes) -> None:
        if len(payload) < 1:
            self._err(CMD_GET_FF_VEC, 0x01)
            return
        item = payload[0]
        if item < 1 or item > 15:
            self._err(CMD_GET_FF_VEC, 0x02)
            return
        values = list(self.ff_vec.get(item, [0.0] * self.n))
        values = (values + [0.0] * 7)[:7]     # the firmware reply is always 7 wide
        # The RSP_FF_VEC payload **does carry the RSP prefix**:
        # [0x4B, item, 7×f32] = 30B.
        self._send(RSP_FF_VEC, bytes([RSP_FF_VEC, item])
                   + struct.pack("<7f", *values))

    def _cmd_get_ff_scalar(self, payload: bytes) -> None:
        if len(payload) < 2:
            self._err(CMD_GET_FF_SCALAR, 0x01)
            return
        item, sub = payload[0], payload[1]
        if item < 1 or item > 18:
            self._err(CMD_GET_FF_SCALAR, 0x02)
            return
        if item == 9:
            value = float(self.ff_mask)
        else:
            value = self.ff_scalar.get((item, sub), 0.0)
        # The RSP_FF_SCALAR payload **does carry the RSP prefix**:
        # [0x4C, item, sub, f32] = 7B.
        self._send(RSP_FF_SCALAR, bytes([RSP_FF_SCALAR, item, sub])
                   + struct.pack("<f", value))

    # ── Replies ──

    def _ack(self, cmd: int) -> None:
        self._send(RSP_ACK, bytes([cmd]))

    def _err(self, cmd: int, reason: int) -> None:
        self._send(RSP_ERR, bytes([cmd, reason]))

    def _send(self, cmd: int, payload: bytes, droppable: bool = True) -> None:
        if self._master is None:
            return
        if droppable and self._drop_replies > 0:
            self._drop_replies -= 1
            return
        try:
            os.write(self._master, _frame(cmd, payload))
        except OSError:
            pass

    # ────────────────────────── Simulation step ──────────────────────────

    def _step(self, dt: float, now: float) -> None:
        with self._lock:
            # Two-stage enable: after registering, a short delay passes before the
            # motors are actually energized (corresponding to do_enable once feedback
            # is complete).
            if (self.enable_pending_at is not None and not self.enabled
                    and now - self.enable_pending_at >= self.enable_delay_s):
                self.enabled = True
                self.enable_pending_at = None
                self.err_code = [1] * self.n
                self.mode = MODE_INIT
                self.last_kick_s = now

            # Command watchdog: MOVE_J kicks it every tick, the other modes stay
            # alive through incoming commands.
            if self.enabled and self.mode == MODE_MOVE_J:
                self.last_kick_s = now
            # Line-for-line identical to watchdog_check(): **both branches must write
            # the flag**. When the command stream resumes (a kick every tick) the flag
            # has to be cleared, otherwise every status frame keeps lying about a
            # fail-soft state — the firmware itself had to be patched once precisely
            # because this else branch was missing.
            if not self.enabled:
                self.watchdog_tripped = False
            elif now - self.last_kick_s > WATCHDOG_TIMEOUT_S:
                if not self.watchdog_tripped:
                    # [S1] the rising edge of the link-loss hold rewrites q_ref to the
                    # **measured position** — otherwise the arm would be dragged back
                    # to "the endpoint of the last command" instead of stopping where
                    # it is.
                    self.q_ref = list(self.q)
                self.watchdog_tripped = True
            else:
                self.watchdog_tripped = False

            if not self.enabled:
                self.dq = [0.0] * self.n
                self.tau = [0.0] * self.n
                return

            # MOVE_JS: advance the reference one step towards the commanded target,
            # rate-limited by |dq_ref| (same as the firmware's slew_linear).
            if self.mode == MODE_MOVE_JS and not self.watchdog_tripped:
                for i in range(self.n):
                    limit = abs(self.dq_ref[i])
                    step = limit * dt
                    err = self.js_target_q[i] - self.q_ref[i]
                    if abs(err) <= step:
                        self.q_ref[i] = self.js_target_q[i]
                    elif step > 0.0:
                        self.q_ref[i] += math.copysign(step, err)

            alpha = min(1.0, dt / JOINT_LAG_S)
            for i in range(self.n):
                # A single-axis fault affects only that axis (G7: the joint_fault
                # bitmap → DISABLE that axis alone); the remaining axes keep
                # following.
                faulted = bool(self.joint_fault & (1 << i))
                holding = self.watchdog_tripped or faulted
                if holding:
                    # Holding: target = the frozen q_ref, tau=0, stiffness scaled by
                    # the park declaration.
                    target = self.q_ref[i]
                    kp = self.kp[i] * (1.0 if self.park_requested else 0.6)
                else:
                    target = self.q_ref[i]
                    kp = self.kp_cmd[i] if self.mode == MODE_MOVE_MIT_ALL \
                        else self.kp[i]
                q_old = self.q[i]
                q_new = q_old + (target - q_old) * alpha
                self.q[i] = q_new
                self.dq[i] = (q_new - q_old) / dt if dt > 0 else 0.0
                # tau is just a stand-in for "the torque the firmware feeds the
                # motors". It is always 0 while holding (the firmware adds no
                # feedforward then).
                self.tau[i] = 0.0 if holding else kp * (target - q_new)
                if not holding and self.mode == MODE_MOVE_JS \
                        and self.s_js_user_ff:
                    self.tau[i] += self.tau_user[i]

    def _reset_law(self) -> None:
        """When MOVE_JS switches from "with user tau_ff" back to "without", the
        firmware resets its integral/friction history."""
        self.tau_user = [0.0] * self.n

    # ────────────────────────── Status frame ──────────────────────────

    def _safety_flags(self) -> int:
        flags = 0
        if self.joint_fault or self.mode == MODE_EMERGENCY:
            flags |= FLAG_FAULT
        if self.watchdog_tripped:
            flags |= FLAG_WATCHDOG_TRIPPED
        if self._inj_feedback_stale:
            flags |= FLAG_FEEDBACK_STALE
        if self._inj_temp_warning:
            flags |= FLAG_TEMP_WARNING
        if self._inj_pos_violation:
            flags |= FLAG_POSITION_VIOLATION
        if self._inj_overspeed:
            flags |= FLAG_OVERSPEED
        return flags

    def _build_status(self) -> bytes:
        """Hand-assemble the ``6 + 21N`` status frame (**does not reuse stm32_proto**,
        see the module docstring)."""
        flags = (self._safety_flags() & 0x3F) | (self.mode << 6)
        if self.enabled:
            flags |= 1 << 9
        out = bytearray(struct.pack("<HH", flags, self.seq & 0xFFFF))
        for i in range(self.n):
            out += struct.pack("<fffff", self.q[i], self.dq[i], self.tau[i],
                               self.t_mos[i], self.t_coil[i])
            out += bytes([self.err_code[i] & 0xFF])
        out += struct.pack("<H", self.joint_fault & 0xFFFF)
        self.seq = (self.seq + 1) & 0xFFFF
        return bytes(out)
