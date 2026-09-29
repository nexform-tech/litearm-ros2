#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Python-side bindings for the shared memory contract (ctypes).

The contract itself is defined in ``include/litearm_ros2_control/litearm_shm.h`` — this
module is only a consumer of it and does not redefine any layout semantics:

* Struct field order/width match the C side byte for byte, cross-checked by
  :func:`describe_layout` against the C probe (``litearm_shm_layout_probe``); see
  ``test/test_shm_layout.py``.
* The seqlock acquire/release semantics live entirely on the C side. Python performs no
  raw memory access here, only whole-struct-plus-length transfers in and out, so there
  is no need to express memory ordering in Python.

Timestamp convention: ``stamp_s`` / ``heartbeat_s`` are both ``CLOCK_MONOTONIC``
seconds, the same source as :func:`time.monotonic` on Linux, so they can be compared
directly against C++ ``steady_clock``.
"""

import ctypes
import ctypes.util
import os
from pathlib import Path
from typing import List, Optional

NUM_JOINTS = 7
"""Number of joints, matching LITEARM_SHM_NUM_JOINTS on the C side."""

DEFAULT_SHM_NAME = "/litearm_hw"
"""Default shared memory object name."""

SHM_OK = 0
SHM_TORN = 1
SHM_ERR_OPEN = -2
SHM_ERR_LAYOUT = -5
SHM_ERR_VERSION = -6

# ── LitearmState.last_error: daemon suppression reason codes ──
# Must stay in sync with LITEARM_DAEMON_* in include/litearm_ros2_control/litearm_shm.h.
DAEMON_OK = 0
DAEMON_CONNECTING = 1
DAEMON_HOLDING_STALE_COMMAND = 2
DAEMON_HOLDING_ESTOP = 3
DAEMON_HOLDING_MOTOR_FAULT = 4
DAEMON_HOLDING_FEEDBACK_STALE = 5
DAEMON_HOLDING_OVERTEMP = 6
DAEMON_DISABLED = 7
DAEMON_HOLDING_WATCHDOG = 8
DAEMON_SHUTTING_DOWN = 9
DAEMON_HOLDING_BAD_COMMAND = 10

DAEMON_STATUS_TEXT = {
    DAEMON_OK: "following ros2_control commands",
    DAEMON_CONNECTING: "hardware not connected yet (starting up, port not found, or "
                       "license not activated)",
    DAEMON_HOLDING_STALE_COMMAND: "command frame is stale: the ROS-side control loop "
                                  "stopped publishing",
    DAEMON_HOLDING_ESTOP: "soft emergency stop active",
    DAEMON_HOLDING_MOTOR_FAULT: "one or more joints report an unhealthy code",
    DAEMON_HOLDING_FEEDBACK_STALE: "joint feedback missing or timed out",
    DAEMON_HOLDING_OVERTEMP: "motor temperature reached the software protection "
                             "threshold",
    DAEMON_DISABLED: "ROS side requested disable (motors de-energized)",
    DAEMON_HOLDING_WATCHDOG: "litearm-stm32 firmware watchdog took over",
    DAEMON_SHUTTING_DOWN: "daemon is shutting down",
    DAEMON_HOLDING_BAD_COMMAND: "command frame contained non-finite numbers; rejected",
}

_ERR_TEXT = {
    SHM_OK: "ok",
    SHM_TORN: "torn read (writer is updating, retries exhausted)",
    SHM_ERR_OPEN: "cannot open shared memory segment",
    -1: "generic error",
    -3: "ftruncate failed",
    -4: "mmap failed",
    SHM_ERR_LAYOUT: "shared memory segment missing or size mismatch",
    SHM_ERR_VERSION: "shared memory layout version mismatch",
    -7: "invalid argument",
}


class ShmError(RuntimeError):
    """A shared memory operation failed."""

    def __init__(self, message: str, code: int = 0) -> None:
        super().__init__(f"{message} (code={code})")
        self.code = code


# ─────────────────────────── Struct mirrors ───────────────────────────
# Field order must match litearm_shm.h exactly; every field is a double, so there is no
# implicit padding and the cross-language layout is unambiguous.

StateFields = [
    ("position", ctypes.c_double * NUM_JOINTS),
    ("velocity", ctypes.c_double * NUM_JOINTS),
    ("effort", ctypes.c_double * NUM_JOINTS),
    ("temperature_mos", ctypes.c_double * NUM_JOINTS),
    ("temperature_coil", ctypes.c_double * NUM_JOINTS),
    ("error_code", ctypes.c_double * NUM_JOINTS),
    ("feedback_age_s", ctypes.c_double * NUM_JOINTS),
    ("feedback_received", ctypes.c_double * NUM_JOINTS),
    ("stamp_s", ctypes.c_double),
    ("heartbeat_s", ctypes.c_double),
    ("connected", ctypes.c_double),
    ("enabled", ctypes.c_double),
    ("faulted", ctypes.c_double),
    ("watchdog_tripped", ctypes.c_double),
    ("dry_run", ctypes.c_double),
    ("cycle_count", ctypes.c_double),
    ("applied_command_cycle", ctypes.c_double),
    ("command_age_s", ctypes.c_double),
    ("last_error", ctypes.c_double),
]

CommandFields = [
    ("position", ctypes.c_double * NUM_JOINTS),
    ("velocity", ctypes.c_double * NUM_JOINTS),
    ("acceleration", ctypes.c_double * NUM_JOINTS),
    ("effort", ctypes.c_double * NUM_JOINTS),
    ("kp", ctypes.c_double * NUM_JOINTS),
    ("kd", ctypes.c_double * NUM_JOINTS),
    ("enable", ctypes.c_double),
    ("estop", ctypes.c_double),
    ("stamp_s", ctypes.c_double),
    ("cycle_count", ctypes.c_double),
]

HeaderFields = [
    ("magic", ctypes.c_uint32),
    ("layout_version", ctypes.c_uint32),
    ("state_seq", ctypes.c_uint64),
    ("command_seq", ctypes.c_uint64),
    ("state_publish_count", ctypes.c_uint64),
    ("command_publish_count", ctypes.c_uint64),
    ("state_torn_reads", ctypes.c_uint64),
    ("command_torn_reads", ctypes.c_uint64),
]


class LitearmState(ctypes.Structure):
    """Joint state and health, daemon → ROS. Field semantics live in litearm_shm.h."""

    _fields_ = StateFields

    def joints(self) -> "dict[str, List[float]]":
        """Expand per joint, for convenient printing/assertion."""
        return {
            name: [getattr(self, name)[i] for i in range(NUM_JOINTS)]
            for name, _ in StateFields[:8]
        }


class LitearmCommand(ctypes.Structure):
    """MIT command, ROS → daemon. Field semantics live in litearm_shm.h."""

    _fields_ = CommandFields

    @classmethod
    def holding(
        cls,
        position: List[float],
        kp: List[float],
        kd: List[float],
        stamp_s: float = 0.0,
        cycle_count: float = 0.0,
        enable: float = 1.0,
    ) -> "LitearmCommand":
        """Build a "hold in place at high stiffness" command (dq_ref=0, tau_ff=0)."""
        command = cls()
        for index, value in enumerate(position):
            command.position[index] = float(value)
        for index in range(NUM_JOINTS):
            command.kp[index] = float(kp[index])
            command.kd[index] = float(kd[index])
        command.enable = float(enable)
        command.estop = 0.0
        command.stamp_s = float(stamp_s)
        command.cycle_count = float(cycle_count)
        return command


class LitearmHeader(ctypes.Structure):
    """Segment header diagnostics."""

    _fields_ = HeaderFields


# ─────────────────────────── Library lookup and loading ───────────────────────────


def _candidate_lib_paths() -> List[Path]:
    """List candidate paths for liblitearm_shm.so in priority order.

    Both supported run shapes have to be covered:

    * ROS install tree: ``<prefix>/lib/python3.10/site-packages/litearm_ros2_control/``
      → go up 3 levels to reach ``<prefix>/lib/``.
    * Running straight from the source tree (as the unit tests do):
      ``<ws>/src/litearm_ros2_control/python/...``
      → look for the colcon build output under ``<ws>/build/litearm_ros2_control/``.
    """
    candidates: List[Path] = []

    override = os.environ.get("LITEARM_SHM_LIB")
    if override:
        candidates.append(Path(override))

    here = Path(__file__).resolve()
    package_root = here.parents[2]          # .../litearm_ros2_control
    workspace_root = package_root.parents[1]  # .../<ws>

    # 1) ROS install tree
    candidates.append(here.parents[3] / "liblitearm_shm.so")
    # 2) colcon build tree
    candidates.append(workspace_root / "build" / package_root.name /
                      "liblitearm_shm.so")
    # 3) colcon install tree (built but never sourced)
    candidates.append(workspace_root / "install" / package_root.name / "lib" /
                      "liblitearm_shm.so")
    # 4) any AMENT_PREFIX_PATH prefix
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if prefix:
            candidates.append(Path(prefix) / "lib" / "liblitearm_shm.so")
    # 5) hand it to the dynamic linker
    found = ctypes.util.find_library("litearm_shm")
    if found:
        candidates.append(Path(found))

    return candidates


def _load_library() -> ctypes.CDLL:
    """Load liblitearm_shm.so and verify the layout size on load (guards against a
    library of the wrong vintage)."""
    errors: List[str] = []
    for path in _candidate_lib_paths():
        if not path.exists():
            errors.append(f"{path} (does not exist)")
            continue
        try:
            lib = ctypes.CDLL(str(path), use_errno=True)
        except OSError as exc:  # pragma: no cover - environment dependent
            errors.append(f"{path} ({exc})")
            continue
        _declare_signatures(lib)
        _verify_sizes(lib, path)
        return lib
    raise ShmError(
        "Could not locate liblitearm_shm.so: run colcon build and source the install "
        "space, or set the LITEARM_SHM_LIB environment variable. Tried:\n  "
        + "\n  ".join(errors)
    )


def _declare_signatures(lib: ctypes.CDLL) -> None:
    """Declare the C API signatures (without them ctypes truncates 64-bit return
    values to int)."""
    size_t = ctypes.c_size_t
    lib.litearm_shm_state_size.restype = size_t
    lib.litearm_shm_command_size.restype = size_t
    lib.litearm_shm_header_size.restype = size_t
    lib.litearm_shm_segment_size.restype = size_t

    lib.litearm_shm_open.argtypes = [
        ctypes.c_char_p, ctypes.c_int, ctypes.POINTER(ctypes.c_void_p)
    ]
    lib.litearm_shm_open.restype = ctypes.c_int
    lib.litearm_shm_close.argtypes = [ctypes.c_void_p]
    lib.litearm_shm_close.restype = None
    lib.litearm_shm_unlink.argtypes = [ctypes.c_char_p]
    lib.litearm_shm_unlink.restype = ctypes.c_int

    lib.litearm_shm_publish_state.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(LitearmState)
    ]
    lib.litearm_shm_publish_state.restype = ctypes.c_int
    lib.litearm_shm_read_state.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(LitearmState), ctypes.c_int
    ]
    lib.litearm_shm_read_state.restype = ctypes.c_int

    lib.litearm_shm_publish_command.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(LitearmCommand)
    ]
    lib.litearm_shm_publish_command.restype = ctypes.c_int
    lib.litearm_shm_read_command.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(LitearmCommand), ctypes.c_int
    ]
    lib.litearm_shm_read_command.restype = ctypes.c_int

    lib.litearm_shm_read_header.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(LitearmHeader)
    ]
    lib.litearm_shm_read_header.restype = ctypes.c_int


def _verify_sizes(lib: ctypes.CDLL, path: Path) -> None:
    """Check struct sizes at load time, so a ctypes mirror and .so of mismatched
    versions cannot silently misread each other."""
    expected = (
        ("LitearmState", ctypes.sizeof(LitearmState), lib.litearm_shm_state_size()),
        ("LitearmCommand", ctypes.sizeof(LitearmCommand),
         lib.litearm_shm_command_size()),
        ("LitearmHeader", ctypes.sizeof(LitearmHeader), lib.litearm_shm_header_size()),
    )
    for name, mirror, native in expected:
        if mirror != native:
            raise ShmError(
                f"{path}: {name} is {native} bytes on the C side, "
                f"but the Python mirror is {mirror} bytes — the shared memory layout "
                f"has drifted; re-run colcon build and update shm_bridge.py to match"
            )


_LIB: Optional[ctypes.CDLL] = None


def _lib() -> ctypes.CDLL:
    """Load the shared library lazily (so import time does not depend on the install
    space)."""
    global _LIB
    if _LIB is None:
        _LIB = _load_library()
    return _LIB


def describe_layout() -> "dict[str, object]":
    """Return a comparable snapshot of both layouts, for the cross-check test."""
    lib = _lib()
    return {
        "state_size": int(lib.litearm_shm_state_size()),
        "state_mirror_size": ctypes.sizeof(LitearmState),
        "command_size": int(lib.litearm_shm_command_size()),
        "command_mirror_size": ctypes.sizeof(LitearmCommand),
        "header_size": int(lib.litearm_shm_header_size()),
        "header_mirror_size": ctypes.sizeof(LitearmHeader),
        "segment_size": int(lib.litearm_shm_segment_size()),
        "num_joints": NUM_JOINTS,
        "state_offsets": {name: getattr(LitearmState, name).offset
                          for name, _ in StateFields},
        "command_offsets": {name: getattr(LitearmCommand, name).offset
                            for name, _ in CommandFields},
    }


# ─────────────────────────── High-level wrapper ───────────────────────────


class SharedMemory:
    """RAII wrapper around a shared memory segment.

    Typical use (daemon side)::

        with SharedMemory(create=True) as shm:
            shm.publish_state(state)

    The ROS plugin side uses ``SharedMemory(create=False)`` — the segment must already
    have been created by the daemon.
    """

    def __init__(self, name: str = DEFAULT_SHM_NAME, create: bool = False,
                 max_retries: int = 64) -> None:
        self.name = name
        self.max_retries = int(max_retries)
        self._handle = ctypes.c_void_p()
        code = _lib().litearm_shm_open(
            name.encode("utf-8"), 1 if create else 0, ctypes.byref(self._handle))
        if code != SHM_OK:
            raise ShmError(
                f"Failed to open shared memory {name!r}: "
                f"{_ERR_TEXT.get(code, 'unknown error')}", code)

    # ── Lifecycle ──
    def close(self) -> None:
        if self._handle:
            _lib().litearm_shm_close(self._handle)
            self._handle = ctypes.c_void_p()

    def __enter__(self) -> "SharedMemory":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def __del__(self) -> None:  # pragma: no cover - last-resort cleanup
        self.close()

    @staticmethod
    def unlink(name: str = DEFAULT_SHM_NAME) -> None:
        """Delete the shared memory object (idempotent)."""
        code = _lib().litearm_shm_unlink(name.encode("utf-8"))
        if code != SHM_OK:
            raise ShmError(f"Failed to delete shared memory {name!r}", code)

    # ── State block (daemon writes / ROS reads) ──
    def publish_state(self, state: LitearmState) -> None:
        code = _lib().litearm_shm_publish_state(self._handle, ctypes.byref(state))
        if code != SHM_OK:
            raise ShmError("Failed to publish state", code)

    def read_state(self, max_retries: Optional[int] = None) -> LitearmState:
        """Read the state. Raises :class:`ShmError` (code=SHM_TORN) once the torn-read
        retries are exhausted."""
        out = LitearmState()
        retries = self.max_retries if max_retries is None else int(max_retries)
        code = _lib().litearm_shm_read_state(self._handle, ctypes.byref(out), retries)
        if code != SHM_OK:
            raise ShmError(
                f"Failed to read state: {_ERR_TEXT.get(code, 'unknown error')}", code)
        return out

    def try_read_state(self, max_retries: Optional[int] = None) -> Optional[LitearmState]:
        """Read the state, returning ``None`` instead of raising on a torn read (used
        by daemon-side polling)."""
        try:
            return self.read_state(max_retries)
        except ShmError as exc:
            if exc.code == SHM_TORN:
                return None
            raise

    # ── Command block (ROS writes / daemon reads) ──
    def publish_command(self, command: LitearmCommand) -> None:
        code = _lib().litearm_shm_publish_command(self._handle, ctypes.byref(command))
        if code != SHM_OK:
            raise ShmError("Failed to publish command", code)

    def read_command(self, max_retries: Optional[int] = None) -> LitearmCommand:
        out = LitearmCommand()
        retries = self.max_retries if max_retries is None else int(max_retries)
        code = _lib().litearm_shm_read_command(self._handle, ctypes.byref(out), retries)
        if code != SHM_OK:
            raise ShmError(
                f"Failed to read command: {_ERR_TEXT.get(code, 'unknown error')}", code)
        return out

    def try_read_command(self, max_retries: Optional[int] = None) -> Optional[LitearmCommand]:
        try:
            return self.read_command(max_retries)
        except ShmError as exc:
            if exc.code == SHM_TORN:
                return None
            raise

    # ── Diagnostics ──
    def header(self) -> LitearmHeader:
        out = LitearmHeader()
        code = _lib().litearm_shm_read_header(self._handle, ctypes.byref(out))
        if code != SHM_OK:
            raise ShmError("Failed to read header", code)
        return out


__all__ = [
    "DEFAULT_SHM_NAME",
    "NUM_JOINTS",
    "SHM_OK",
    "SHM_TORN",
    "LitearmCommand",
    "LitearmHeader",
    "LitearmState",
    "SharedMemory",
    "ShmError",
    "describe_layout",
]
