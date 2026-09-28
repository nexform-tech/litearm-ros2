#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Byte layer of the litearm-stm32 USB CDC protocol (PC side).

Scope
-----
This module **deals only in bytes**: frame packing/splitting, serialization of each
command payload, parsing of each reply. No serial port, no threads, no state machine —
those live in :mod:`litearm_ros2_control.stm32_link`.

Source of truth
---------------
Firmware ``User/litearm/hal/usb_cmd.h`` + ``usb_cmd.c`` (Litearm1.8.0-7J). Every
constant and layout in this module has a matching line there; **any inconsistency is a
silent data corruption class of defect**, so ``test/test_stm32_proto.py`` pins them all
down with byte-for-byte golden vectors.

Wire format::

    downlink host→board:  A5 | CMD(1B) | LEN(1B) | PAYLOAD(0..255B) | CRC16-LO | CRC16-HI
    uplink   board→host:  same header (an RSP id takes the place of CMD)
    CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF), covering SOF..last byte of PAYLOAD.

**The prefix convention for reply payloads is inconsistent — the easiest pitfall in this
protocol** (copied verbatim from the firmware implementation):

==========================  ================  ===========================================
Reply                       Payload length    First payload byte
==========================  ================  ===========================================
``RSP_STATUS``    0x40      6+21N             no prefix, flags low byte first
``RSP_FIRMWARE``  0x44      variable ASCII    no prefix
``RSP_ACK``       0x45      1                 command number
``RSP_ERR``       0x46      2                 ``{command number, reason code}``
``RSP_JOINT_PARAM`` 0x49    21                **no prefix**, first byte is joint idx
``RSP_FF_VEC``    0x4B      30                **has prefix**: ``[0x4B, item, 7×f32]``
``RSP_FF_SCALAR`` 0x4C      7                 **has prefix**: ``[0x4C, item, sub, f32]``
==========================  ================  ===========================================

The ``usb_cmd.h`` comment writes 0x4C as 6B and 0x4B as 30B — the former is a **typo**
(the source declares ``uint8_t out[7]``, see the ``CMD_GET_FF_SCALAR`` branch of
``usb_cmd.c``). This module follows the source, not the comment.
"""

import struct
from typing import List, Optional, Sequence, Tuple

# ────────────────────────────── Frame layer ──────────────────────────────

SOF = 0xA5

# Downlink CMD (usb_cmd.h)
CMD_MOVE_J = 0x01
CMD_MOVE_P = 0x02
CMD_MOVE_JS = 0x03
CMD_MOVE_MIT = 0x04
CMD_MOVE_MIT_ALL = 0x05
CMD_ZERO_G = 0x06
CMD_MOVE_J_SYNC = 0x07
CMD_ENABLE = 0x10
CMD_DISABLE = 0x11
CMD_EMERGENCY_STOP = 0x12
CMD_CLEAR_FAULTS = 0x13
CMD_RESET = 0x14
CMD_ENTER_DFU = 0x15
CMD_SET_MOTION_MODE = 0x20
CMD_SET_SPEED_PERCENT = 0x21
CMD_SET_JOINT_PARAM = 0x22
CMD_SET_JOINT_LIMITS = 0x23
CMD_GET_JOINT_PARAM = 0x24
CMD_PARAM_SAVE = 0x25
CMD_SET_FF_VEC = 0x26
CMD_SET_FF_FLAGS = 0x27
CMD_SET_FF_SCALAR = 0x28
CMD_GET_GRAVITY = 0x39
CMD_FF_PRESET = 0x31
CMD_GET_FF_VEC = 0x2B
CMD_GET_FF_SCALAR = 0x2C
CMD_GET_STATUS = 0x40
CMD_GET_FIRMWARE = 0x41

# Uplink RSP
RSP_STATUS = 0x40
RSP_DETAIL = 0x41
RSP_FIRMWARE = 0x44
RSP_ACK = 0x45
RSP_ERR = 0x46
RSP_JOINT_PARAM = 0x49
RSP_FF_VEC = 0x4B
RSP_FF_SCALAR = 0x4C

# ───────────────────────── Status frame flag bits ─────────────────────────
# The firmware only writes the low 6 bits (safety bits) + mode(bit6-8) + enabled(bit9)
# + cart_busy(bit10).

ARM_FLAG_FAULT = 1 << 0
ARM_FLAG_WATCHDOG_TRIPPED = 1 << 1
ARM_FLAG_FEEDBACK_STALE = 1 << 2
ARM_FLAG_TEMP_WARNING = 1 << 3
ARM_FLAG_POSITION_VIOLATION = 1 << 4
ARM_FLAG_OVERSPEED = 1 << 5
FLAG_MODE_SHIFT = 6
FLAG_MODE_MASK = 0x7
FLAG_ENABLED = 1 << 9
FLAG_CART_BUSY = 1 << 10

# Safety bit mask (low 6 bits)
FLAG_SAFETY_MASK = 0x3F

FLAG_NAMES = {
    ARM_FLAG_FAULT: "FAULT",
    ARM_FLAG_WATCHDOG_TRIPPED: "WD_TRIPPED",
    ARM_FLAG_FEEDBACK_STALE: "FB_STALE",
    ARM_FLAG_TEMP_WARNING: "TEMP_WARN",
    ARM_FLAG_POSITION_VIOLATION: "POS_VIOL",
    ARM_FLAG_OVERSPEED: "OVERSPEED",
}

# ──────────────────────────── Motion modes ────────────────────────────
# arm_mode_t (litearm.h). mode occupies 3 bits in the status frame, so 7 is usable too.

ARM_MODE_INIT = 0
ARM_MODE_MOVE_J = 1
ARM_MODE_MOVE_P = 2
ARM_MODE_MOVE_JS = 3
ARM_MODE_MOVE_MIT = 4
ARM_MODE_MOVE_MIT_ALL = 5
ARM_MODE_EMERGENCY = 6
ARM_MODE_ZERO_G = 7

MODE_NAMES = {
    ARM_MODE_INIT: "INIT",
    ARM_MODE_MOVE_J: "MOVE_J",
    ARM_MODE_MOVE_P: "MOVE_P",
    ARM_MODE_MOVE_JS: "MOVE_JS",
    ARM_MODE_MOVE_MIT: "MOVE_MIT",
    ARM_MODE_MOVE_MIT_ALL: "MIT_ALL",
    ARM_MODE_EMERGENCY: "EMERGENCY",
    ARM_MODE_ZERO_G: "ZERO_G",
}

# ────────────────────────── Feedforward mask ff_mask ──────────────────────────
# litearm.h. Factory value = FF_MASTER|FF_G|FF_INERTIA|FF_CORIOLIS|FF_INTEGRAL
#                                     |FF_QUANT|FF_VELREF|FF_FRICTION (defaults.c,
#                                     FF_WALL off).

FF_MASTER = 1 << 0
FF_G = 1 << 1
FF_INERTIA = 1 << 2
FF_CORIOLIS = 1 << 3
FF_FRICTION = 1 << 4
FF_INTEGRAL = 1 << 5
FF_WALL = 1 << 6
FF_QUANT = 1 << 7
FF_VELREF = 1 << 8
FF_FACTORY_MASK = (FF_MASTER | FF_G | FF_INERTIA | FF_CORIOLIS
                   | FF_INTEGRAL | FF_QUANT | FF_VELREF | FF_FRICTION)

FF_BIT_NAMES = {
    FF_MASTER: "MASTER",
    FF_G: "G",
    FF_INERTIA: "INERTIA",
    FF_CORIOLIS: "CORIOLIS",
    FF_FRICTION: "FRICTION",
    FF_INTEGRAL: "INTEGRAL",
    FF_WALL: "WALL",
    FF_QUANT: "QUANT",
    FF_VELREF: "VELREF",
}

# ───────────────────────── 0x26/0x2B item table ─────────────────────────
# SET_FF_VEC and GET_FF_VEC share the same set of items (usb_cmd.h / usb_cmd.c).

FF_VEC_FRICTION = 1
FF_VEC_KI = 2
FF_VEC_I_MAX = 3
FF_VEC_WALL_STIFF = 4
FF_VEC_WALL_DAMP = 5
FF_VEC_WALL_TAU_MAX = 6
FF_VEC_GRAVITY_SCALE = 7
FF_VEC_INERTIA_SCALE = 8
FF_VEC_FRICTION_V = 9
FF_VEC_FRICTION_FC0 = 10
FF_VEC_FRICTION_FC1 = 11
FF_VEC_KD_EXTRA = 15
FF_VEC_ITEM_MAX = 15

# ──────────────────────── 0x28/0x2C item table ────────────────────────
# item 9 is rejected on the write path ("reserved"); only the read path uses it, to
# read ff_mask.

FF_SCALAR_FRIC_DB = 1
FF_SCALAR_WALL_MARGIN = 2
FF_SCALAR_FRICTION_SLEW = 3
FF_SCALAR_PAYLOAD_MASS = 4
FF_SCALAR_PAYLOAD_COM = 5
FF_SCALAR_GRAVITY_VEC = 6
FF_SCALAR_FRICTION_MODEL = 7
FF_SCALAR_FRIC_V2_EPS = 8
FF_SCALAR_FF_MASK = 9
FF_SCALAR_DRAG_GAIN = 10
FF_SCALAR_DRAG_DB = 11
FF_SCALAR_DRAG_KD_MARGIN = 12
FF_SCALAR_ZG_VEL_THR = 13
FF_SCALAR_ZG_ENGAGE_SEC = 14
FF_SCALAR_ZG_ENGAGE_KP = 15
FF_SCALAR_ZG_ENGAGE_KD = 16
FF_SCALAR_WALL_FW_KD = 17
FF_SCALAR_HOLD_KP_GAIN = 18
FF_SCALAR_ITEM_MAX = 18

# ─────────────────────────── CRC / packing ───────────────────────────


def crc16_ccitt_false(data: bytes) -> int:
    """CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF). Matches the firmware's crc16.c."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 \
                else (crc << 1) & 0xFFFF
    return crc


def pack_frame(cmd: int, payload: bytes = b"") -> bytes:
    """``A5 CMD LEN payload CRC16-LE``. LEN is one byte, so the payload caps at 255B."""
    if len(payload) > 255:
        raise ValueError(f"Payload of {len(payload)}B exceeds the 255B frame limit")
    body = bytes([SOF, cmd & 0xFF, len(payload)]) + payload
    crc = crc16_ccitt_false(body)
    return body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


class Frame:
    """One uplink frame. Frames with ``crc_ok=False`` must be discarded."""

    __slots__ = ("cmd", "payload", "crc_ok")

    def __init__(self, cmd: int, payload: bytes, crc_ok: bool) -> None:
        self.cmd = cmd
        self.payload = payload
        self.crc_ok = crc_ok

    def __repr__(self) -> str:
        return (f"Frame(cmd=0x{self.cmd:02X}, len={len(self.payload)}, "
                f"crc_ok={self.crc_ok})")


class CdcFramer:
    """Streaming frame splitter: feed it raw bytes, it emits whole frames.

    Tolerates arbitrary garbage bytes and partial frames in the stream (it re-syncs by
    self-aligning on SOF). **Use this, never "one read equals one frame"**: a 153B
    status frame gets split by USB into several 64B bulk packets, so a single ``read``
    can return half a frame, or several frames stuck together.
    """

    def __init__(self) -> None:
        self.buf = bytearray()
        self.discarded = 0

    def reset(self) -> None:
        self.buf.clear()
        self.discarded = 0

    def feed(self, data: bytes) -> List[Frame]:
        out: List[Frame] = []
        self.buf += data
        while len(self.buf) >= 3:
            if self.buf[0] != SOF:
                self.discarded += 1
                del self.buf[0]
                continue
            length = self.buf[2]
            need = 3 + length + 2
            if len(self.buf) < need:
                break
            body = bytes(self.buf[:3 + length])
            crc = self.buf[3 + length] | (self.buf[4 + length] << 8)
            del self.buf[:need]
            out.append(Frame(body[1], bytes(body[3:3 + length]),
                             crc16_ccitt_false(body) == crc))
        return out


# ─────────────────────────── Downlink packing ───────────────────────────


def _pack_f32(values: Sequence[float]) -> bytes:
    return b"".join(struct.pack("<f", float(v)) for v in values)


def pack_move_j(q: Sequence[float], speed_percent: float) -> bytes:
    """``MOVE_J(0x01)``: ``q[N] f32 + sp f32``. ``MOVE_J_SYNC(0x07)`` is byte-for-byte
    identical."""
    return _pack_f32(list(q)) + struct.pack("<f", float(speed_percent))


def pack_move_js(q: Sequence[float], dq: Sequence[float],
                 tau: Optional[Sequence[float]] = None) -> bytes:
    """``MOVE_JS(0x03)``: ``q[N] + dq[N]``, optionally ``+ tau_ff[N]``.

    ⚠ **Whether tau_ff is present decides whether the firmware's whole built-in
    feedforward chain is on** (``s_js_user_ff`` in ``control_loop.c``):
      * absent → the firmware adds G/friction/integral/kd_extra/quantization
        compensation on top of tau;
      * present → the built-in feedforward is **switched off entirely**, and tau
        carries only the values you supply.
    So "pass an all-zero tau_ff to keep things simple" is wrong: that amounts to
    turning feedforward off.
    """
    payload = _pack_f32(list(q)) + _pack_f32(list(dq))
    if tau is not None:
        payload += _pack_f32(list(tau))
    return payload


def pack_move_mit_all(rows: Sequence[Sequence[float]]) -> bytes:
    """``MOVE_MIT_ALL(0x05)``: ``q[N] dq[N] kp[N] kd[N] tau[N]`` (**SoA**).

    ``rows[i] = (q, dq, kp, kd, tau)``; payload is ``5·N·4`` bytes (7 joints = 140B).

    ⚠ The firmware reads it as SoA (``q[i] = p[i*4]``, ``dq[i] = p[4N + i*4]`` …).
    A real incident already happened once in the litearm-stm32 repository: several
    tools packed as AoS (interleaved per joint), and with q=[0.1..0.7], kp=15 actually
    sent, the firmware received q=[0.1, 0, 15, 1, 0, 0.2, 0] — J3 was commanded to
    15 rad and, once clamped, snapped straight into its limit. **Running this against
    real hardware will hit the joint limits.**
    """
    n = len(rows)
    if n == 0:
        raise ValueError("rows must not be empty")
    return b"".join(struct.pack("<f", float(rows[i][j]))
                    for j in range(5) for i in range(n))


def pack_set_ff_flags(mask: int) -> bytes:
    """``SET_FF_FLAGS(0x27)``: ``u32 ff_mask`` LE."""
    return struct.pack("<I", int(mask) & 0xFFFFFFFF)


def pack_set_ff_vec(item: int, values: Sequence[float]) -> bytes:
    """``SET_FF_VEC(0x26)``: ``item(1B) + f32[7]``."""
    values = list(values)
    if len(values) != 7:
        raise ValueError(f"SET_FF_VEC needs 7 components, got {len(values)}")
    return bytes([item & 0xFF]) + _pack_f32(values)


def pack_set_ff_scalar(item: int, sub: int, value: float) -> bytes:
    """``SET_FF_SCALAR(0x28)``: ``item(1B) + sub(1B) + f32``."""
    return bytes([item & 0xFF, sub & 0xFF]) + struct.pack("<f", float(value))


def pack_set_joint_param(idx: int, kp: float, kd: float,
                         tau_max: float) -> bytes:
    """``SET_JOINT_PARAM(0x22)``: ``idx(1B) + kp,kd,tau_max(f32)`` → RAM."""
    return bytes([idx & 0xFF]) + _pack_f32([kp, kd, tau_max])


def pack_set_joint_limits(idx: int, q_min: float, q_max: float) -> bytes:
    """``SET_JOINT_LIMITS(0x23)``: ``idx(1B) + q_min,q_max(f32)`` → RAM.

    ⚠ On the firmware side this **can only narrow the limits, never widen them**
    (``params.c``).
    """
    return bytes([idx & 0xFF]) + _pack_f32([q_min, q_max])


def pack_get_joint_param(idx: int) -> bytes:
    """``GET_JOINT_PARAM(0x24)``: ``idx(1B)``."""
    return bytes([idx & 0xFF])


def pack_get_ff_vec(item: int) -> bytes:
    """``GET_FF_VEC(0x2B)``: ``item(1B)``, item ∈ [1, 15]."""
    return bytes([item & 0xFF])


def pack_get_ff_scalar(item: int, sub: int = 0) -> bytes:
    """``GET_FF_SCALAR(0x2C)``: ``item(1B) + sub(1B)``, item ∈ [1, 18]."""
    return bytes([item & 0xFF, sub & 0xFF])


def pack_u8(value: int) -> bytes:
    """Single-byte payload (``0x20`` park / ``0x21`` speed / ``0x06`` zero-g / ``0x31``
    preset)."""
    return bytes([value & 0xFF])


# ─────────────────────────── Uplink parsing ───────────────────────────


class JointParam:
    """Parsed ``RSP_JOINT_PARAM(0x49)`` (the 0x24 branch of ``usb_cmd.c``)."""

    __slots__ = ("index", "kp", "kd", "tau_max", "q_min", "q_max")

    def __init__(self, index: int, kp: float, kd: float, tau_max: float,
                 q_min: float, q_max: float) -> None:
        self.index = index
        self.kp = kp
        self.kd = kd
        self.tau_max = tau_max
        self.q_min = q_min
        self.q_max = q_max

    def __repr__(self) -> str:
        return (f"JointParam(j{self.index + 1}: kp={self.kp:g} kd={self.kd:g} "
                f"tau_max={self.tau_max:g} q=[{self.q_min:g}, {self.q_max:g}])")


class JointStatus:
    """A single joint inside a status frame."""

    __slots__ = ("q", "dq", "tau", "t_mos", "t_coil", "err")

    def __init__(self, q: float, dq: float, tau: float, t_mos: float,
                 t_coil: float, err: int) -> None:
        self.q = q
        self.dq = dq
        self.tau = tau
        self.t_mos = t_mos
        self.t_coil = t_coil
        self.err = err

    @property
    def enabled(self) -> bool:
        """``err == 1`` means enabled (0 = disabled, anything else = a fault code)."""
        return self.err == 1

    def __repr__(self) -> str:
        return (f"JointStatus(q={self.q:.4f} dq={self.dq:.4f} tau={self.tau:.3f} "
                f"t={self.t_mos:.0f}/{self.t_coil:.0f} err={self.err})")


class StatusFrame:
    """Parsed ``RSP_STATUS(0x40)``: ``6 + 21N`` bytes (7 joints = 153B).

    Compatible with the older firmware's ``4 + 21N`` layout: back then there was no
    trailing ``joint_fault``, so ``joint_fault`` is ``None``. Only newer firmware
    writes flags bit9 (enabled).
    """

    __slots__ = ("flags", "seq", "joints", "joint_fault", "stamp_s")

    def __init__(self, flags: int, seq: int, joints: Tuple[JointStatus, ...],
                 joint_fault: Optional[int], stamp_s: float = 0.0) -> None:
        self.flags = flags
        self.seq = seq
        self.joints = joints
        self.joint_fault = joint_fault
        # Local receive time (filled in by stm32_link), not a firmware time base.
        self.stamp_s = stamp_s

    @property
    def mode(self) -> int:
        return (self.flags >> FLAG_MODE_SHIFT) & FLAG_MODE_MASK

    @property
    def mode_name(self) -> str:
        return MODE_NAMES.get(self.mode, f"?{self.mode}")

    @property
    def enabled(self) -> bool:
        return bool(self.flags & FLAG_ENABLED)

    @property
    def fault(self) -> bool:
        return bool(self.flags & ARM_FLAG_FAULT)

    @property
    def watchdog_tripped(self) -> bool:
        return bool(self.flags & ARM_FLAG_WATCHDOG_TRIPPED)

    @property
    def feedback_stale(self) -> bool:
        return bool(self.flags & ARM_FLAG_FEEDBACK_STALE)

    @property
    def temp_warning(self) -> bool:
        return bool(self.flags & ARM_FLAG_TEMP_WARNING)

    @property
    def position_violation(self) -> bool:
        return bool(self.flags & ARM_FLAG_POSITION_VIOLATION)

    @property
    def overspeed(self) -> bool:
        return bool(self.flags & ARM_FLAG_OVERSPEED)

    @property
    def cart_busy(self) -> bool:
        return bool(self.flags & FLAG_CART_BUSY)

    @property
    def safety_flags(self) -> int:
        return self.flags & FLAG_SAFETY_MASK

    def flag_names(self) -> List[str]:
        return [name for bit, name in FLAG_NAMES.items() if self.flags & bit]

    def faulted_joints(self) -> List[int]:
        """Joint indices (0-based) set in the ``joint_fault`` bitmap. Returns an empty
        list when the field is absent."""
        if self.joint_fault is None:
            return []
        return [i for i in range(len(self.joints))
                if self.joint_fault & (1 << i)]

    def __repr__(self) -> str:
        return (f"StatusFrame(mode={self.mode_name} seq={self.seq} "
                f"enabled={self.enabled} flags={'|'.join(self.flag_names()) or '-'}"
                f" joint_fault={self.joint_fault})")


def decode_status(payload: bytes) -> Optional[StatusFrame]:
    """Parse a status frame payload. Returns ``None`` when the length is not
    self-consistent (**never guess**).

    Layout (``usb_cmd_report_status`` in ``usb_cmd.c``)::

        [0:2]   flags u16 LE   (low 6 safety bits | mode<<6 | enabled<<9 | cart<<10)
        [2:4]   seq   u16 LE
        [4+21i] per joint, 21B: q,dq,tau,t_mos,t_coil (f32×5) + err (u8)
        [4+21N] joint_fault u16 LE   (6+21N layout only)
    """
    n = (len(payload) - 4) // 21
    if n < 1 or len(payload) not in (4 + n * 21, 6 + n * 21):
        return None
    flags, seq = struct.unpack_from("<HH", payload, 0)
    joints = tuple(
        JointStatus(*struct.unpack_from("<fffff", payload, 4 + i * 21),
                    payload[4 + i * 21 + 20])
        for i in range(n))
    joint_fault = None
    if len(payload) == 6 + n * 21:
        joint_fault = struct.unpack_from("<H", payload, 4 + n * 21)[0]
    return StatusFrame(flags, seq, joints, joint_fault)


def decode_joint_param(payload: bytes) -> Optional[JointParam]:
    """Parse ``RSP_JOINT_PARAM(0x49)`` (21B, **no RSP prefix**)."""
    if len(payload) != 21:
        return None
    idx = payload[0]
    kp, kd, tau_max, q_min, q_max = struct.unpack_from("<fffff", payload, 1)
    return JointParam(idx, kp, kd, tau_max, q_min, q_max)


def decode_ff_vec(payload: bytes) -> Optional[Tuple[int, Tuple[float, ...]]]:
    """Parse ``RSP_FF_VEC(0x4B)``: ``[0x4B, item, 7×f32]`` 30B (**with RSP prefix**)."""
    if len(payload) != 30 or payload[0] != RSP_FF_VEC:
        return None
    return payload[1], struct.unpack_from("<7f", payload, 2)


def decode_ff_scalar(payload: bytes) -> Optional[Tuple[int, int, float]]:
    """Parse ``RSP_FF_SCALAR(0x4C)``: ``[0x4C, item, sub, f32]`` 7B (**with RSP
    prefix**)."""
    if len(payload) != 7 or payload[0] != RSP_FF_SCALAR:
        return None
    item, sub = payload[1], payload[2]
    return item, sub, struct.unpack_from("<f", payload, 3)[0]


def decode_firmware(payload: bytes) -> Optional[str]:
    """Parse ``RSP_FIRMWARE(0x44)``: ASCII version string (no prefix), e.g.
    ``Litearm1.8.0-7J``."""
    if not payload:
        return None
    return payload.decode("ascii", errors="replace").strip("\x00").strip()


def decode_ack(payload: bytes) -> Optional[int]:
    """Parse ``RSP_ACK(0x45)``: the payload is the command number."""
    if len(payload) != 1:
        return None
    return payload[0]


def decode_err(payload: bytes) -> Optional[Tuple[int, int]]:
    """Parse ``RSP_ERR(0x46)``: ``{command number, reason code}``.

    Common reason codes: ``0x01`` payload too short / ``0x02`` invalid argument /
    ``0x03`` not enabled, or emergency stop latched / ``0x04`` must be disabled first /
    ``0x05`` flash save in progress / ``0x06`` link-loss rigid hold latched /
    ``0x07`` mask mismatch. ``0x08`` on ``ENABLE`` means the license is not activated.
    """
    if len(payload) != 2:
        return None
    return payload[0], payload[1]


def format_ff_mask(mask: int) -> str:
    """Render ff_mask as ``MASTER|G|INERTIA|...``; returns ``-`` when no bit is set."""
    names = [name for bit, name in FF_BIT_NAMES.items() if mask & bit]
    return "|".join(names) if names else "-"
