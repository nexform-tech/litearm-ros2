#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""hw_daemon.py — litearm hardware daemon (litearm-stm32 backend).

Responsibilities and boundaries
-------------------------------
This process is the **sole owner of the USB CDC**: it exposes the litearm-stm32
firmware's MOVE_JS / MIT_ALL command stream to the ros2_control
``hardware_interface::SystemInterface`` plugin as a double-buffered shared
memory block.

    ros2_control process (C++, RT loop)       this process (Python, non-RT)
    ────────────────────────────────          ─────────────────────────
    read()  ← seqlock read state block    ←──  state frame parse (100Hz push)
    write() → seqlock write command block ──→  MOVE_JS(0x03) / MIT_ALL(0x05) @250Hz

Why the split works this way:

1. ros2_control's ``read()``/``write()`` are pure memcpy (lock-free, never
   blocking): the RT loop carries no Python interpreter/GIL/GC, and no serial
   write timeout can stall it.
2. The USB CDC is owned exclusively by this process.
3. **While the ROS-side process crashes/restarts, this process keeps
   holding** — once the command goes stale it switches to HOLDING and keeps
   streaming the frozen reference, so the arm does not sag when the host dies.

Division of labour vs. the pylitearm backend (the heart of the swap)
--------------------------------------------------------------------
The STM32 firmware brings its own motor control loop (300Hz), built-in
feedforward model (generated from the URDF), safety envelope and command
watchdog. **These responsibilities used to be implemented in Python in this very
file; now they have all moved down**:

    ┌ Responsibility ────────────────┬ old (pylitearm backend) ───┬ new (litearm-stm32) ──────┐
    │ Damiao MIT frame pack/unpack   │ this process (SocketCAN)   │ **firmware**              │
    │ PD gains kp/kd                 │ per-frame from cmd frame   │ **firmware params**       │
    │ FF G / fric / integ / kd_extra │ this process (pinocchio)   │ **firmware ff_mask** §1   │
    │ cmd watchdog / fail-soft hold  │ this process + pylitearm wd│ **firmware 100ms**        │
    │ safety env (lim/spd/tmp/trk)   │ this process, per cycle    │ **firmware safety_check** │
    │ joint limits / tau_max / zero  │ pylitearm litearm.yaml     │ **firmware params** §2    │
    │ ROS cmd cadence / SHM contract │ this process               │ this process (unchanged)  │
    └────────────────────────────────┴────────────────────────────┴───────────────────────────┘

§1 What selects the feedforward switches is not the "mode" but **whether this
   frame carries tau_ff**: the firmware's ``builtin_mode`` requires
   ``MOVE_JS && !s_js_user_ff``. So the default channel sends MOVE_JS **without
   tau_ff** (56B payload) and lets the firmware layer on its own feedforward;
   attach tau (even all zeros) and the whole thing switches off.

§1b **``MOVE_JS``'s ``dq_ref`` is not only velocity feedforward, it is also the
   slew-rate cap on the position reference** (verified on hardware in
   ``control_loop.c``)::

       v_lim = clampf(fabsf_(target_dq[i]), 0.0f, jp->speed_limit * gov_ratio);
       cmd->q_ref = slew_linear(target_q, cmd->q_ref, v_lim * LITEARM_CTRL_DT);

   That is, each tick the reference advances toward the command target by at most
   ``|dq_ref|·dt``. Two immediate consequences:

   * **With ``dq_ref=0`` the reference freezes and the joint does not move a
     single tick**. On the default channel, "send position with velocity 0" is
     not "position servo", it is "stay put". The driver must therefore supply
     both position and velocity (JTC supplies both).
   * The ``q_hold`` streamed under HOLDING is **ignored in the position term**
     — what the firmware freezes is its own reference. The arm thus converges
     from the "measured position" to the "firmware reference", and the
     displacement equals the tracking lag at the moment holding was entered.
     This is **correct behaviour** (it stops where it was commanded to go, not
     where it happens to be), but on the default channel ``self._q_hold`` is
     **informational** (used for logging and the exit hold stream), not a
     directly executable anchor; only under ``--mit-passthrough`` (MIT_ALL,
     where the reference slews at ``vel_max``) does it actually determine the
     position.
§2 Joint-level parameters are **all read from the firmware at startup**
   (``0x24`` / ``0x2C``); this process no longer keeps a second source of truth.
   To change a parameter, change the firmware
   (``0x22``/``0x26``/``0x27``/``0x28``), or see the "feedforward overrides"
   section.

Dual channel
------------
The default is **position mode** (MOVE_JS, without tau_ff): PD + G + friction +
integral + kd_extra + quantization compensation are all computed by the
firmware. The price is that MOVE_JS has no acceleration source (``ddq_s ≡ 0``),
hence **no M·q̈ and no C·q̇** — the most substantial difference from the
pylitearm backend.

``--mit-passthrough`` switches to **torque mode** (MIT_ALL, fully transparent):
kp/kd/effort come from the command frame on every frame, and the firmware layers
on no feedforward of its own. **This process performs no dynamics computation at
all** — ``effort`` is passed straight through, feedforward is supplied by the ROS
side or by the user. This channel exists for A/B comparison and for the "I want
to compute it myself" case.

Feedforward overrides (``--ff-*``)
----------------------------------
All five switches are **off by default** (= keep whatever the firmware currently
has; read it back and log it). They are only pushed down when given explicitly,
writing the corresponding bits into the firmware's ``ff_mask``:

    --gravity-compensation   → FF_G
    --friction-compensation  → FF_FRICTION
    --inertia-compensation   → FF_INERTIA | FF_CORIOLIS
    --integral-compensation  → FF_INTEGRAL
    --damping-compensation   → 0x26 item15 (kd_extra vector; no FF bit; 0 = off)

⚠️ In MOVE_JS mode the ``FF_INERTIA``/``FF_CORIOLIS`` bits have **no effect**
(the firmware only computes the inertia terms in MOVE_J) — setting them is
harmless, but also useless.

Safety arbitration (this process keeps the skeleton, the criteria move to the firmware)
----------------------------------------------------------------------------------------
Each cycle it arbitrates by a fixed priority, and the suppression reason is written
into the state block's ``last_error``:

    priority  condition                                      behaviour
    ────────  ─────────────────────────────────────────────  ────────────────────────────────
    1         state frame stale (> --feedback-timeout-s)     HOLDING (not connected)
    2         command frame stale (> --command-timeout-s)    HOLDING (frozen reference sent)
    3         enable=0 (**fresh command frame only**)        motors off (the arm goes limp)
    4         estop≠0                                        HOLDING (**no fw e-stop sent**)
    5         firmware FAULT / joint_fault                   HOLDING
    6         firmware FB_STALE / state frame too old        HOLDING
    7         firmware TEMP_WARN                             HOLDING
    8         command contains NaN/Inf                       reject the frame, keep holding
    9         firmware WD_TRIPPED                            still send (kick), report only
    10        normal                                         TRACKING

The **order of item 3 is deliberate**: were ``enable`` tested before staleness,
"the last frame happened to be enable=0 when the ROS side died" would drop the
arm outright. ``test_stale_enable_zero_is_ignored`` exists to pin down exactly
this branch.

Item 4 deliberately **does not map onto the firmware's 0x12 emergency stop**:
a firmware e-stop disables the motors (the arm drops), whereas the established
semantics of a ROS-side "soft e-stop" are **hold position with high stiffness**.
For a genuine emergency stop, use the hardware e-stop.

How holding is implemented
--------------------------
Under HOLDING this process does not fall back on the firmware's fail-soft hold
(that would mean stopping the stream for 100ms to let the watchdog time out, and
it comes with tau=0, no gravity feedforward, and only 0.6× stiffness). Instead it
**keeps streaming the frozen reference**:

* position mode: ``MOVE_JS(q_hold, dq=0)`` → the firmware holds with PD +
  **G(q_hold)** and nothing sags;
* torque mode: ``MIT_ALL(q_hold, 0, kp×hold_kp_gain, kd, tau=0)``, with the gains
  taken from the firmware (``hold_kp_gain`` of ``0x2C`` item18), the same source
  as the firmware's own in-position stiffening.

Exit
----
On SIGINT/SIGTERM it first **keeps streaming the frozen reference** for
``--exit-hold-s`` (default 2.0s) — during which the firmware still applies
gravity feedforward and the arm sits rock steady — then sends ``0x20 park``
(declare park: from then on, a watchdog trip holds with 1.0× stiffness instead of
0.6×) and finally closes the link.

⚠️ Holding after park has **no gravity feedforward** (the firmware's ``hold``
branch uses ``tau=0``), so there is a slight sag of ``G/kp``. This is existing
firmware-side behaviour, not something this process introduces. To avoid any sag
at all, keep the stack running or support the arm before cutting power.

A second Ctrl-C skips the remaining hold time immediately.
"""

import argparse
import errno
import fcntl
import logging
import math
import os
import signal
import sys
import threading
import time
from typing import List, Optional, Sequence, Tuple

from litearm_ros2_control import fake_firmware
from litearm_ros2_control import stm32_proto as proto
from litearm_ros2_control import shm_bridge
from litearm_ros2_control.shm_bridge import (
    DAEMON_CONNECTING,
    DAEMON_DISABLED,
    DAEMON_HOLDING_BAD_COMMAND,
    DAEMON_HOLDING_ESTOP,
    DAEMON_HOLDING_FEEDBACK_STALE,
    DAEMON_HOLDING_MOTOR_FAULT,
    DAEMON_HOLDING_OVERTEMP,
    DAEMON_HOLDING_STALE_COMMAND,
    DAEMON_HOLDING_WATCHDOG,
    DAEMON_OK,
    DAEMON_SHUTTING_DOWN,
    NUM_JOINTS,
    LitearmCommand,
    LitearmState,
    SharedMemory,
    ShmError,
)
from litearm_ros2_control.stm32_link import (Stm32AccessDenied,
                                             Stm32Error,
                                             Stm32Link,
                                             Stm32NotConnected)

log = logging.getLogger("litearm.hw_daemon")

# Tracking modes (internal state machine, deliberately named after the old
# backend so the two can be read side by side).
MODE_TRACKING = "tracking"
MODE_HOLDING = "holding"
# The "not in any mode yet" sentinel: after a successful connect the initial
# value must be this one, so that the first cycle is guaranteed to run the
# anchoring action in ``_enter(HOLDING)`` (locking the hold anchor onto the
# measured position). If the initial value were MODE_HOLDING directly, the
# ``if mode == self._mode`` guard at the top of ``_enter`` would skip the
# anchoring, ``_q_hold`` would stay [0]*7, and the very first hold frame would
# yank the arm back to zero at high stiffness.
MODE_INIT = "init"
MODE_DISABLED = "disabled"

# Hard representable range of the Damiao MIT frame (the firmware clamps to the
# same range internally; anything outside it is truncated by the ESC).
MIT_KP_MIN, MIT_KP_MAX = 0.0, 500.0
MIT_KD_MIN, MIT_KD_MAX = 0.0, 5.0

# The two-stage semantics of the firmware ENABLE: the ACK only means
# "registered"; the magnets are not really energised until the feedback is
# complete. Reason 0x03 is "feedback not ready yet" and is worth retrying;
# 0x08 is an inactive license, where retrying gains nothing.
ENABLE_REASON_NOT_READY = 0x03
ENABLE_REASON_EMERGENCY = 0x06
ENABLE_REASON_NO_LICENSE = 0x08

# Minimum interval between in-loop log lines (seconds). At a 250 Hz control rate
# "one log line per cycle" is absolutely out of the question: in the launch file
# the log sink is a terminal/pipe, i.e. blocking I/O, and while a fault persists
# that would stall exactly the cycles that most need their timing. Policy: emit
# immediately the first time, then at most once per LOG_THROTTLE_S for the same
# key; suppression counts are summarised in a single line on exit (see
# _shutdown), so no diagnostics are lost and nothing enters the loop's hot path.
LOG_THROTTLE_S = 1.0

# Wait window for the park frame to leave the port on exit (seconds). See the
# comment in _shutdown: the USB CDC has an internal transmit buffer, so closing
# the port right after writing can drop the frame — and a lost park declaration
# means the arm sags that little bit further.
PARK_FLUSH_S = 0.05

# Firmware ff_mask bits corresponding to the five --ff-* switches.
FF_FLAG_TO_BITS = {
    "gravity": proto.FF_G,
    "friction": proto.FF_FRICTION,
    "inertia": proto.FF_INERTIA | proto.FF_CORIOLIS,
    "integral": proto.FF_INTEGRAL,
}

# Factory vector for kd_extra (joint[0..6].kd_extra in the firmware's
# params/defaults.c): only J1~J4, the four load-bearing axes, carry a value.
# The firmware reverts to this on restart, but once 0x26 item15 has zeroed it
# there is no trace left in RAM — hence the fallback when
# "--damping-compensation" is switched on.
KD_EXTRA_FACTORY = (6.0, 6.0, 6.0, 6.0, 0.0, 0.0, 0.0)


def _sleep_until(next_tick: float) -> float:
    """Busy-wait until the absolute instant ``next_tick`` (coarse sleep + a
    tail-end spin), returning the instant it actually finished.

    The absolute time base prevents drift; the tail spin makes up for
    ``sleep``'s lack of precision. This process does no trajectory integration,
    so the only thing at stake is how evenly the outgoing frames are spaced.
    """
    while True:
        remain = next_tick - time.monotonic()
        if remain <= 0.0:
            break
        if remain > 0.0005:
            time.sleep(remain - 0.0003)
    return time.monotonic()


def _finite(values) -> bool:
    return all(math.isfinite(float(value)) for value in values)


class DaemonFatalError(RuntimeError):
    """Non-retryable startup failure (e.g. firmware license not activated) —
    retrying any number of times gives the same result."""


class _RetryableConnect(Exception):
    """Retryable connection failure (reply lost, feedback not ready yet, board
    just unplugged and replugged)."""


class DaemonSettings:
    """Daemon runtime parameters.

    **PC-side concepts only** (port, rates, timeouts, policy). Joint-level
    parameters (kp/kd/tau_max/limits/feedforward switches) are not part of the
    configuration: they are read from the firmware at startup — see §2 of the
    module docstring.
    """

    def __init__(self, port: str = "",
                 rate_hz: float = 250.0,
                 shm_name: str = shm_bridge.DEFAULT_SHM_NAME,
                 dry_run: bool = False,
                 mit_passthrough: bool = False,
                 command_timeout_s: float = 0.1,
                 feedback_timeout_s: float = 0.08,
                 connect_retry_s: float = 2.0,
                 arm_timeout_s: float = 5.0,
                 exit_hold_s: float = 2.0,
                 request_timeout_s: float = 0.5,
                 ff_overrides: Optional[dict] = None,
                 verbose: bool = False) -> None:
        self.port = str(port).strip()
        self.shm_name = shm_name
        self.dry_run = bool(dry_run)
        self.mit_passthrough = bool(mit_passthrough)
        self.verbose = bool(verbose)

        self.rate_hz = float(rate_hz)
        if not math.isfinite(self.rate_hz) or self.rate_hz <= 0.0:
            raise ValueError("rate_hz must be a positive finite value")

        self.command_timeout_s = _positive("command_timeout_s",
                                           command_timeout_s)
        self.feedback_timeout_s = _positive("feedback_timeout_s",
                                            feedback_timeout_s)
        self.connect_retry_s = _positive("connect_retry_s", connect_retry_s)
        self.arm_timeout_s = _positive("arm_timeout_s", arm_timeout_s)
        self.exit_hold_s = float(exit_hold_s)
        if not math.isfinite(self.exit_hold_s) or self.exit_hold_s < 0.0:
            raise ValueError("exit_hold_s must be a non-negative finite value")
        self.request_timeout_s = _positive("request_timeout_s",
                                           request_timeout_s)
        # Keep only the switches given explicitly; absent = leave the firmware
        # alone (see the module docstring).
        self.ff_overrides = {k: bool(v) for k, v in (ff_overrides or {}).items()
                             if v is not None}

        # ── filled in below by _connect() from the firmware read-back ──
        self.firmware_version = ""
        self.num_joints = NUM_JOINTS
        self.kp: List[float] = [0.0] * NUM_JOINTS
        self.kd: List[float] = [0.0] * NUM_JOINTS
        self.tau_max: List[float] = [0.0] * NUM_JOINTS
        self.q_min: List[float] = [-math.pi] * NUM_JOINTS
        self.q_max: List[float] = [math.pi] * NUM_JOINTS
        self.ff_mask = 0
        self.ff_damping: List[float] = [0.0] * NUM_JOINTS
        self.hold_kp_gain = 1.0

    def describe(self) -> str:
        channel = "MIT_ALL passthrough" if self.mit_passthrough else "MOVE_JS position mode"
        return (
            f"port={self.port or '(auto-discover)'} shm={self.shm_name} "
            f"rate={self.rate_hz:g}Hz channel={channel} "
            f"dry_run={self.dry_run} cmd_timeout={self.command_timeout_s:g}s "
            f"fb_timeout={self.feedback_timeout_s:g}s "
            f"exit_hold={self.exit_hold_s:g}s"
        )

    def describe_firmware(self) -> str:
        """The "hardware identity" block in the startup log: which board is being
        driven, which firmware version, and at which parameter values.

        The reason this line exists is **troubleshooting**: after swapping a
        board, reflashing the firmware or changing parameters, the stack must
        show at a glance what the daemon is actually talking to, rather than
        leaving you to guess.
        """
        params = "\n".join(
            f"  j{i + 1}: kp={self.kp[i]:g} kd={self.kd[i]:g} "
            f"tau_max={self.tau_max[i]:g} "
            f"q∈[{self.q_min[i]:.6f}, {self.q_max[i]:.6f}]"
            for i in range(self.num_joints))
        return (
            f"firmware version: {self.firmware_version}\n"
            f"  port: {self.port or '(auto-discover)'} · joints {self.num_joints}\n"
            f"  channel: {'MIT_ALL passthrough (kp/kd/effort applied per frame)' if self.mit_passthrough else 'MOVE_JS (firmware computes PD + built-in feedforward)'}\n"
            f"  ff_mask=0x{self.ff_mask:03X} ({proto.format_ff_mask(self.ff_mask)})"
            f" · kd_extra={self.ff_damping} · hold_kp_gain={self.hold_kp_gain:g}\n"
            f"{params}"
        )


def _positive(name: str, value) -> float:
    out = float(value)
    if not math.isfinite(out) or out <= 0.0:
        raise ValueError(f"{name} must be a positive finite value")
    return out


# ─────────────────── hardware configuration (PC-side, optional) ───────────────────
# litearm_hw.yaml holds **PC-side concepts only** (port, rates, timeouts,
# policy). The sole source of truth for joint-level parameters
# (kp/kd/tau_max/limits/feedforward switches) is the firmware parameter table;
# putting them here would be a second source of truth.
#
# Keys use a "dotted flat" form that maps one-to-one onto the YAML nesting:
#   transport.rate_hz  ←→  transport:\n  rate_hz:

HW_CONFIG_KEYS = {
    "transport.port": str,
    "transport.rate_hz": float,
    "transport.command_timeout_s": float,
    "transport.feedback_timeout_s": float,
    "transport.connect_retry_s": float,
    "transport.arm_timeout_s": float,
    "transport.request_timeout_s": float,
    "policy.mit_passthrough": bool,
    "policy.exit_hold_s": float,
    "policy.ff_overrides": dict,
}


def _flatten_config(raw: dict, prefix: str = "") -> List[Tuple[str, object]]:
    """Flatten nested YAML into a dotted flat list.

    Whether to descend is decided purely by "is this key a prefix of some known
    key" — so ``transport`` is descended into, while ``policy.ff_overrides`` is
    itself a leaf (no known key starts with it). A misspelt key (say
    ``transport.por``) is not descended into, so it ends up in leaf position and
    is caught by the unknown-key check in :func:`load_hw_config`.
    """
    out: List[Tuple[str, object]] = []
    for key, value in raw.items():
        name = f"{prefix}{key}"
        if any(known.startswith(f"{name}.") for known in HW_CONFIG_KEYS):
            if not isinstance(value, dict):
                raise ValueError(f"{name} should be a mapping, got "
                                 f"{type(value).__name__}")
            out.extend(_flatten_config(value, f"{name}."))
            continue
        out.append((name, value))
    return out


def _check_config_type(path: str, key: str, value, expected: type):
    """Type check (including the ``bool``/``int`` trap: ``True`` is an ``int``
    as well)."""
    # Writing 250 instead of 250.0 in YAML is perfectly normal; a bare
    # isinstance test would get it wrong.
    if expected is float and isinstance(value, int) and not isinstance(value,
                                                                      bool):
        return float(value)
    if expected is not bool and isinstance(value, bool):
        raise ValueError(f"{path}: {key} expects {expected.__name__}, "
                         f"got bool ({value!r})")
    if not isinstance(value, expected):
        raise ValueError(f"{path}: {key} expects {expected.__name__}, "
                         f"got {type(value).__name__} ({value!r})")
    return value


def load_hw_config(path: str) -> dict:
    """Read ``litearm_hw.yaml`` and return a dotted-flat dictionary.

    **An unknown key is an outright error**, never silently ignored: a typo in
    the config file that quietly keeps the old value is one of the hardest
    classes of problem to track down (you tweak things for ages with no effect
    and conclude the parameter is not taking).
    """
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - python3-yaml is a declared dep
        raise ValueError(
            "reading --hw-config requires PyYAML (rosdep: python3-yaml); "
            "without it, just use the command-line arguments") from exc
    try:
        with open(path, encoding="utf-8") as handle:
            raw = yaml.safe_load(handle)
    except OSError as exc:
        raise ValueError(f"cannot read hardware config file {path}: {exc}") from exc
    if raw is None:
        return {}
    if not isinstance(raw, dict):
        raise ValueError(f"{path} must be a top-level mapping (key: value), "
                         f"got {type(raw).__name__}")
    out = {}
    for key, value in _flatten_config(raw):
        if key not in HW_CONFIG_KEYS:
            raise ValueError(
                f"{path} contains unknown key {key!r}; valid keys: "
                f"{', '.join(sorted(HW_CONFIG_KEYS))}")
        out[key] = _check_config_type(path, key, value, HW_CONFIG_KEYS[key])
    return out


def _build_settings(args) -> DaemonSettings:
    """Combine the command line and the (optional) config file into
    DaemonSettings.

    Priority: **command line > config file > defaults in the code**. The command
    line must be able to override the config file, otherwise "tweak it just this
    once" means editing the file — which is exactly the thing most easily
    forgotten.
    """
    config = load_hw_config(args.hw_config) if args.hw_config else {}

    def pick(cli_value, key, fallback=None):
        if cli_value is not None:
            return cli_value
        return config.get(key, fallback)

    kwargs = {"shm_name": args.shm_name, "dry_run": args.dry_run,
              "verbose": args.verbose}
    for name, key in (
            ("port", "transport.port"),
            ("rate_hz", "transport.rate_hz"),
            ("command_timeout_s", "transport.command_timeout_s"),
            ("feedback_timeout_s", "transport.feedback_timeout_s"),
            ("connect_retry_s", "transport.connect_retry_s"),
            ("arm_timeout_s", "transport.arm_timeout_s"),
            ("request_timeout_s", "transport.request_timeout_s"),
            ("mit_passthrough", "policy.mit_passthrough"),
            ("exit_hold_s", "policy.exit_hold_s")):
        value = pick(getattr(args, name), key)
        if value is not None:
            kwargs[name] = value

    # Feedforward overrides merge **per key**: the config file supplies the
    # defaults, the keys given on the command line override them.
    overrides = dict(config.get("policy.ff_overrides") or {})
    overrides.update({key: value
                      for key, value in _ff_overrides(args).items()
                      if value is not None})
    kwargs["ff_overrides"] = overrides
    return DaemonSettings(**kwargs)


class LitearmHwDaemon:
    """Bridge daemon between shared memory and the litearm-stm32 firmware."""

    def __init__(self, settings: DaemonSettings) -> None:
        self.settings = settings
        self.shm = SharedMemory(settings.shm_name, create=True)
        self.link: Optional[Stm32Link] = None
        self._fake: Optional[fake_firmware.FakeFirmware] = None
        self._stop = False
        self._mode = MODE_INIT
        self._reason = DAEMON_CONNECTING
        # Hold anchor: None = not yet locked onto the measured position.
        # ``_hold()`` refuses to send an unanchored hold frame — there must
        # never be a "hold an arm that is not at zero, from zero" situation.
        self._q_hold: Optional[List[float]] = None
        self._cycle = 0.0
        self._applied_command_cycle = 0.0
        self._last_command_age = float("inf")
        self._last_status_age = float("inf")
        self._disabled_sent = False
        self._lock_handle = None
        # Set by the second signal → skip the remaining exit hold time. It must
        # be initialised here: when running off the main thread
        # _install_signal_handlers returns early, and without this initialisation
        # _exit_hold_stream would hit an AttributeError.
        self._exit_hold_skip = False
        # "Did we ever really connect to the hardware". Only used by _shutdown
        # for publishing the final state: after a failed connect self.link is a
        # closed object rather than None, so it cannot serve as the criterion.
        self._connected = False
        # In-loop log throttle state: key -> [last emit time, suppressed count]
        # (see _log_throttled).
        self._log_throttle: "dict[str, List[float]]" = {}
        # Diagnostics: how many motion frames this process actually sent
        # (summarised on exit).
        self.tx_motion_frames = 0

    # ──────────────────────── Small helpers ────────────────────────

    def _log_throttled(self, level: int, key: str, message: str, *args) -> None:
        """Throttled logging: the first line for a key goes out immediately,
        afterwards at most once every ``LOG_THROTTLE_S`` seconds.

        Only for **in-loop** persistent conditions (a fault that will not clear,
        a serial error, a missing anchor, …); one-off events (mode changes, a
        successful connect) keep using a plain ``log.info``. Suppression counts
        are summarised into one line in ``_shutdown`` — no diagnostic is lost,
        yet nothing writes to the terminal at 250 Hz.
        """
        now = time.monotonic()
        entry = self._log_throttle.get(key)
        if entry is not None:
            if now - entry[0] < LOG_THROTTLE_S:
                entry[1] += 1.0
                return
            entry[0] = now
        else:
            self._log_throttle[key] = [now, 0.0]
        log.log(level, message, *args)

    # ─────────────────────────── Lifecycle ───────────────────────────

    def _acquire_singleton_lock(self) -> None:
        """Single-instance lock for the daemon.

        Two daemons writing the same shared memory block would overwrite each
        other's state, and two masters fighting over the same set of motors is
        explicitly forbidden. The firmware, of course, also accepts only one
        command stream, but by then it is already too late.
        """
        path = f"/dev/shm{self.settings.shm_name}.lock"
        self._lock_handle = open(path, "w", encoding="utf-8")
        try:
            fcntl.flock(self._lock_handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            if exc.errno in (errno.EACCES, errno.EAGAIN):
                raise RuntimeError(
                    f"another litearm hardware daemon already holds {path}; "
                    f"only one control process per board is allowed. "
                    f"Stop the old process (or confirm it has exited) before "
                    f"starting another one.") from exc
            raise
        self._lock_handle.write(f"{os.getpid()}\n")
        self._lock_handle.flush()

    def _release_singleton_lock(self) -> None:
        if self._lock_handle is not None:
            try:
                fcntl.flock(self._lock_handle, fcntl.LOCK_UN)
            except OSError:
                pass
            self._lock_handle.close()
            self._lock_handle = None
            try:
                os.unlink(f"/dev/shm{self.settings.shm_name}.lock")
            except OSError:
                pass

    def _install_signal_handlers(self) -> None:
        # A non-main thread (e.g. when running embedded in a test process) has
        # no signal registration rights, so just skip: the exit semantics are
        # still driven by the ``_stop`` flag, and a standalone process is always
        # on the main thread.
        if threading.current_thread() is not threading.main_thread():
            log.debug("not the main thread, skipping signal handler installation")
            return

        def handler(signum, _frame):
            if self._stop:
                # Second Ctrl-C: skip the remaining exit hold time and finish
                # right away.
                self._exit_hold_skip = True
                log.info("signal %s again, skipping the remaining hold time",
                         signal.Signals(signum).name)
                return
            log.info("signal %s received, starting exit", signal.Signals(signum).name)
            self._stop = True

        self._exit_hold_skip = False
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, handler)

    # ───────────────── Connect & firmware identity ─────────────────

    def _connect(self) -> None:
        """Open the link, read the firmware identity and joint parameters,
        (optionally) override the feedforward, and enable.

        If any step fails, the link is closed and the whole thing is retried —
        a half-connected state is never left behind.

        Raises:
            DaemonFatalError: non-retryable (license not activated).
            _RetryableConnect: retryable (reply lost, feedback not ready, port
                absent).
        """
        if self.settings.dry_run and self._fake is None:
            # No-hardware rehearsal: start a fake firmware on a pty; the link
            # still runs the real protocol.
            self._fake = fake_firmware.FakeFirmware(
                num_joints=NUM_JOINTS)
            port = self._fake.start()
            log.info("dry-run: fake firmware started on %s (first-order "
                     "kinematic model — good for exercising the interface and "
                     "the data path, useless for tuning gains)", port)
            self.link = Stm32Link(port)
        elif self.link is None:
            self.link = Stm32Link(self.settings.port)

        try:
            self.link.open()
            self._read_firmware_identity()
            self._read_joint_params()
            self._read_ff_state()
            self._apply_ff_overrides()
            self._enable()
        except BaseException:
            self._close_link()
            raise

    def _read_firmware_identity(self) -> None:
        version = self.link.get_firmware(
            timeout_s=self.settings.request_timeout_s)
        if version is None:
            raise _RetryableConnect("timed out reading the firmware version (0x41)")
        self.settings.firmware_version = version
        status = self.link.wait_status(self.settings.request_timeout_s * 2)
        if status is None:
            raise _RetryableConnect("no state frame received (0x40)")
        count = len(status.joints)
        if count != NUM_JOINTS:
            # A joint-count mismatch is a **contract-level** error (the SHM
            # layout hard-codes 7) and cannot be worked around.
            raise DaemonFatalError(
                f"firmware reports {count} joints, this package is built for "
                f"{NUM_JOINTS} — was the single-joint bench build "
                f"(Litearm*-1J) flashed by mistake?")
        self.settings.num_joints = count

    def _read_joint_params(self) -> None:
        """Read kp/kd/tau_max/soft limits from the firmware (0x24). If the set
        comes back incomplete, the whole round is retried."""
        settings = self.settings
        for index in range(settings.num_joints):
            param = self.link.get_joint_param(
                index, timeout_s=self.settings.request_timeout_s)
            if param is None:
                raise _RetryableConnect(
                    f"timed out reading parameters for joint {index + 1} (0x24)")
            settings.kp[index] = param.kp
            settings.kd[index] = param.kd
            settings.tau_max[index] = param.tau_max
            settings.q_min[index] = param.q_min
            settings.q_max[index] = param.q_max

    def _read_ff_state(self) -> None:
        """Read ff_mask and kd_extra / hold_kp_gain.

        Failing to read them is not a failure (older firmware may not have the
        0x2B/0x2C read-back calls), but it must leave an explicit log line:
        without them there is no way to say which set of feedforward terms is
        actually in the loop.
        """
        settings = self.settings
        mask = self.link.get_ff_mask(timeout_s=self.settings.request_timeout_s)
        if mask is None:
            log.warning("firmware did not answer the ff_mask read-back "
                        "(0x2C item9) — feedforward state unknown, assuming 0; "
                        "--ff-* overrides can still be pushed explicitly")
            settings.ff_mask = 0
            return
        settings.ff_mask = mask
        damping = self.link.get_ff_vec(
            proto.FF_VEC_KD_EXTRA, timeout_s=self.settings.request_timeout_s)
        if damping is not None:
            settings.ff_damping = list(damping[:settings.num_joints])
        gain = self.link.get_ff_scalar(
            proto.FF_SCALAR_HOLD_KP_GAIN,
            timeout_s=self.settings.request_timeout_s)
        if gain is not None and math.isfinite(gain) and gain > 0.0:
            settings.hold_kp_gain = float(gain)

    def _apply_ff_overrides(self) -> None:
        """Write the explicitly given ``--ff-*`` switches into the firmware
        (none of them are written by default).

        Three design points:

        * **Only the bits mentioned explicitly are touched**, everything else
          keeps the firmware's current state — otherwise "turn gravity
          compensation on" would casually switch friction/integral off too.
        * ``kd_extra`` has no dedicated FF bit: in the firmware it is a vector
          applied unconditionally inside ``builtin_mode``, so the only way to
          switch it off is to zero the vector. Hence ``--damping-compensation``
          reads the current value back (already done above) before deciding
          between zeroing it and restoring the factory 6.0.
        * The write **must be verified by reading back**: the firmware silently
          clamps illegal values (``params.c``), so skipping the check leaves you
          believing a change went through when it did not.
        """
        settings = self.settings
        if not settings.ff_overrides:
            return
        mask = settings.ff_mask
        for name, bit in FF_FLAG_TO_BITS.items():
            if name in settings.ff_overrides:
                mask = (mask | bit) if settings.ff_overrides[name] \
                    else (mask & ~bit)
        if "damping" in settings.ff_overrides:
            if settings.ff_overrides["damping"]:
                # Switching it on: keep the firmware's current values; if they
                # have already been zeroed, restore the factory vector. The
                # factory value is 6/6/6/6/0/0/0 in the firmware's
                # params/defaults.c (only the J1~J4 load-bearing axes get one).
                target = list(settings.ff_damping)
                if all(abs(v) < 1e-9 for v in target):
                    target = [KD_EXTRA_FACTORY[i]
                              for i in range(settings.num_joints)]
            else:
                target = [0.0] * settings.num_joints
            if any(abs(a - b) > 1e-9 for a, b in
                   zip(target, settings.ff_damping)):
                self.link.set_ff_vec(proto.FF_VEC_KD_EXTRA, target)
                readback = self.link.get_ff_vec(
                    proto.FF_VEC_KD_EXTRA,
                    timeout_s=settings.request_timeout_s)
                if readback is None or any(
                        abs(a - b) > 1e-6 for a, b in
                        zip(readback[:settings.num_joints], target)):
                    raise _RetryableConnect("kd_extra read-back mismatch after write")
                settings.ff_damping = list(readback[:settings.num_joints])
                log.info("feedforward override: kd_extra → %s", settings.ff_damping)

        if mask != settings.ff_mask:
            self.link.set_ff_mask(mask)
            readback = self.link.get_ff_mask(
                timeout_s=settings.request_timeout_s)
            if readback is None or readback != mask:
                raise _RetryableConnect(
                    f"ff_mask read-back mismatch after write: "
                    f"{readback if readback is None else hex(readback)} != {hex(mask)}")
            log.info("feedforward override: ff_mask 0x%03X → 0x%03X (%s)",
                     settings.ff_mask, readback, proto.format_ff_mask(readback))
            settings.ff_mask = readback

    def _enable(self) -> None:
        """Enable, and insist the magnets really come on (**resend in place**,
        without closing the serial port).

        The firmware's ``ENABLE`` has two-stage semantics (see ``ctrl_enable``
        in ``control_loop.c``, whose comment records the timing measured on real
        hardware: "ENABLE#1 -> 0x03 (first CMODE write), ENABLE#2 -> ACK"):

        1. **The first one after firmware boot**: it must first write the CMODE
           register of all 7 motors (switching them into MIT mode); at that
           moment it replies ``0x03`` and registers ``enable_pending``;
        2. once the feedback is complete, ``enable_pending_poll`` **energises
           the magnets automatically**, and only then does a re-sent ENABLE from
           the host get an ACK.

        So ``0x03`` is **not a failure**, it is the normal first step. The
        original implementation threw it as a "retryable connection failure" —
        the layer above would then close the serial port, reconnect and re-read
        version/parameters/feedforward from scratch. That works, but every cold
        start wastes a whole round, and the log makes it look like a fault.

        Nor can the ACK be used to judge success: an ACK only means
        "registered"; you have to wait for the ``enabled`` bit in a state frame.
        """
        settings = self.settings
        deadline = time.monotonic() + settings.arm_timeout_s
        logged_first_write = False
        while True:
            ok, reason = self.link.command(
                proto.CMD_ENABLE, timeout_s=settings.request_timeout_s)

            if ok is False:
                if reason == ENABLE_REASON_NO_LICENSE:
                    raise DaemonFatalError(
                        "firmware license not activated, ENABLE rejected "
                        "(ERR{0x10,0x08}). Activate it with litearm-stm32's "
                        "tools/litearm-license and retry; until then every "
                        "motion command will be rejected.")
                if reason == ENABLE_REASON_EMERGENCY:
                    # E-stop / single-joint fault latched: resending is
                    # pointless, an explicit reset is required.
                    # Note the latch does not necessarily come from a human
                    # e-stop: the firmware's main-loop heartbeat supervision
                    # (500ms with no heartbeat from main while enabled) also
                    # calls ctrl_emergency_stop(), with the same symptoms.
                    raise _RetryableConnect(
                        "ENABLE rejected (reason code 0x06: e-stop or joint "
                        "fault latched) — this is a latched state, reconnecting "
                        "and resending will not clear it, an explicit reset is "
                        "required: scripts/firmware_reset.py --reset "
                        "--clear-faults (first run it in read-only mode, with "
                        "no switches, to inspect mode/flags/joint_fault)")
                if reason != ENABLE_REASON_NOT_READY:
                    raise _RetryableConnect(f"ENABLE rejected (reason code {reason})")
                if not logged_first_write:
                    logged_first_write = True
                    log.info("ENABLE returned 0x03 (first motor CMODE write by "
                             "the firmware / feedback not ready) — resending in "
                             "place per the firmware sequence, no reconnect")
            elif ok is None:
                # A busy firmware TX drops replies. A lost ACK does not mean
                # enable failed, so this is not treated as a failure — it is
                # left to the wait_enabled call below, which judges by state
                # frames.
                log.warning("no ENABLE reply received (dropped when the "
                            "firmware TX is busy), falling back to the state "
                            "frames to judge the enable result")

            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise _RetryableConnect(
                    f"no enabled bit within {settings.arm_timeout_s:g}s of "
                    f"ENABLE — motor feedback not ready or a hardware fault")
            if self.link.wait_enabled(min(0.4, remaining)) is not None:
                return

    def _close_link(self) -> None:
        if self.link is not None:
            try:
                self.link.close()
            except Exception:
                log.debug("exception while closing the link", exc_info=True)
        if self._fake is not None:
            try:
                self._fake.stop()
            except Exception:
                log.debug("exception while stopping the fake firmware", exc_info=True)
            self._fake = None
        if self.settings.dry_run:
            # On a dry-run restart, bring up a clean fake firmware again (the
            # semantics of power-cycling).
            self.link = None

    # ────────────────────── Mode arbitration ──────────────────────

    def _evaluate(self, cmd: Optional[LitearmCommand], now: float,
                  status) -> Tuple[str, int]:
        """Arbitrate this cycle's tracking mode by fixed priority, returning
        ``(mode, reason_code)``."""
        # 0) The link / state frame itself is unusable — nothing else matters.
        if status is None:
            return MODE_HOLDING, DAEMON_CONNECTING
        age = float("inf") if cmd is None else now - float(cmd.stamp_s)
        self._last_command_age = age
        status_age = self.link.status_age_s()
        self._last_status_age = status_age

        # 1) State frame stale: the firmware is running but we can no longer
        #    hear it (hung serial port, board reset). Worse than a stale
        #    command — there is no feedback left at all, so any action would be
        #    sent blind.
        if status_age > self.settings.feedback_timeout_s:
            return MODE_HOLDING, DAEMON_HOLDING_FEEDBACK_STALE

        # 2) Command frame stale (a torn seqlock read counts as stale) — the
        #    highest priority. The moment the ROS-side control loop stops, this
        #    switches to holding; it is the core of "host dies, arm does not
        #    drop".
        if not math.isfinite(age) or age > self.settings.command_timeout_s:
            return MODE_HOLDING, DAEMON_HOLDING_STALE_COMMAND

        # 3) An explicit disable request (honoured only while the command frame
        #    is fresh, so that enable=0 in a stale frame cannot disable the
        #    motors by accident).
        if float(cmd.enable) == 0.0:
            return MODE_DISABLED, DAEMON_DISABLED

        # 4) Soft e-stop. Deliberately **not** mapped onto the firmware's 0x12
        #    e-stop (that disables the motors and the arm drops) — the
        #    established semantics are "hold position with high stiffness".
        if float(cmd.estop) != 0.0:
            return MODE_HOLDING, DAEMON_HOLDING_ESTOP

        # 5) Firmware-side fault: the FAULT master flag or the joint_fault
        #    bitmap (G7 axis loss).
        if status.fault or status.faulted_joints():
            return MODE_HOLDING, DAEMON_HOLDING_MOTOR_FAULT

        # 6) Feedback stale (the firmware's 80ms criterion).
        if status.feedback_stale:
            return MODE_HOLDING, DAEMON_HOLDING_FEEDBACK_STALE

        # 7) Over-temperature warning.
        if status.temp_warning:
            return MODE_HOLDING, DAEMON_HOLDING_OVERTEMP

        # 8) Command numeric sanity: a non-finite value rejects this frame
        #    outright (the previous frame's hold is kept).
        if not (_finite(cmd.position) and _finite(cmd.velocity)
                and _finite(cmd.effort) and _finite(cmd.kp)
                and _finite(cmd.kd)):
            return MODE_HOLDING, DAEMON_HOLDING_BAD_COMMAND

        # 9) The firmware watchdog took over at some point (this process was
        #    stuck for >100ms). **Frames are still sent** — sending one is
        #    itself the kick and the flag clears on its own; this only makes it
        #    visible to the layer above, without creating a deadlock.
        if status.watchdog_tripped:
            return MODE_TRACKING, DAEMON_HOLDING_WATCHDOG

        return MODE_TRACKING, DAEMON_OK

    def _enter(self, mode: str) -> None:
        """One-shot action on a mode change: on entering HOLDING, lock the hold
        anchor onto the measured position."""
        if mode == self._mode:
            return
        if mode == MODE_HOLDING:
            status = self.link.status
            if status is None:
                return
            measured = [float(j.q) for j in status.joints]
            self._q_hold = [
                min(self.settings.q_max[i],
                    max(self.settings.q_min[i], measured[i]))
                for i in range(self.settings.num_joints)]
            log.info("entering HOLDING, holding at %s",
                     [round(value, 4) for value in self._q_hold])
        elif mode == MODE_TRACKING:
            log.info("entering TRACKING, following ros2_control commands "
                     "(channel: %s)",
                     "MIT_ALL passthrough" if self.settings.mit_passthrough
                     else "MOVE_JS")
        elif mode == MODE_DISABLED:
            self._disabled_sent = False
        self._mode = mode

    # ────────────────────── Per-cycle dispatch ──────────────────────

    def _status_position(self) -> List[float]:
        status = self.link.status
        if status is None:
            return [0.0] * self.settings.num_joints
        return [float(j.q) for j in status.joints]

    def _track(self, cmd: LitearmCommand) -> None:
        """Forward the command frame to the firmware.

        The MOVE_JS path **never carries tau_ff**: attaching it would make the
        firmware switch its entire built-in feedforward off (``builtin_mode``
        requires ``!s_js_user_ff``). The ``effort`` command interface is
        therefore deliberately ignored on this channel — use
        ``--mit-passthrough`` if you want it.
        """
        settings = self.settings
        count = settings.num_joints
        q_ref = [float(cmd.position[i]) for i in range(count)]
        dq_ref = [float(cmd.velocity[i]) for i in range(count)]

        if not settings.mit_passthrough:
            self.link.move_js(q_ref, dq_ref)
        else:
            kp = [min(MIT_KP_MAX, max(MIT_KP_MIN, float(cmd.kp[i])))
                  for i in range(count)]
            kd = [min(MIT_KD_MAX, max(MIT_KD_MIN, float(cmd.kd[i])))
                  for i in range(count)]
            tau = [float(cmd.effort[i]) for i in range(count)]
            self.link.move_mit_all(
                [(q_ref[i], dq_ref[i], kp[i], kd[i], tau[i])
                 for i in range(count)])
        self.tx_motion_frames += 1
        self._applied_command_cycle = float(cmd.cycle_count)

    def _hold(self) -> None:
        """Keep streaming hold frames (rather than "stop sending and let the
        firmware watchdog take over").

        Stopping would make the firmware drop into fail-soft after 100ms —
        ``tau=0``, a stiffness of only 0.6×, and **no gravity feedforward**, so
        it sags under load. Continuing to send frames lets the firmware hold the
        arm with normal stiffness + gravity feedforward, which is exactly the
        "host dies, arm does not drop" effect we are after.

        ⚠ The position semantics differ per channel (see §1b of the module
        docstring):

        * default (MOVE_JS): ``dq_ref=0`` → the firmware reference freezes at
          **its own** position, and ``self._q_hold`` is used only for logging
          and the exit hold stream. Measured on hardware this gives a
          convergence displacement on the order of the tracking lag (~0.01 rad
          on the real arm), not zero.
        * ``--mit-passthrough`` (MIT_ALL): the reference slews at ``vel_max``
          toward ``self._q_hold``, so there the anchor is a **real** anchor —
          which is why the "anchor onto the measured position, never onto zero"
          defence is a hard requirement on that channel.
        """
        settings = self.settings
        if self._q_hold is None:
            # Defence: the hold anchor can only be locked onto the measured
            # position by _enter(HOLDING). Better to send no frame at all (the
            # firmware keeps the previous command) than to send a "zero
            # position + high stiffness" frame — that would drag an arm that is
            # not at zero straight back to zero. This branch is hit every cycle,
            # so it must be throttled.
            self._log_throttled(logging.ERROR, "hold-anchor",
                                "hold anchor not locked yet, skipping this "
                                "cycle's hold frame (refusing to send a zero "
                                "position command)")
            return
        zeros = [0.0] * settings.num_joints
        if not settings.mit_passthrough:
            self.link.move_js(self._q_hold, zeros)
        else:
            # On the torque channel there is no firmware hold hand-off to fall
            # back on, so this process replicates the firmware's same-named
            # semantics: kp × hold_kp_gain (0x2C item18, factory 2.0), tau = 0.
            kp = [min(MIT_KP_MAX, settings.kp[i] * settings.hold_kp_gain)
                  for i in range(settings.num_joints)]
            kd = [min(MIT_KD_MAX, settings.kd[i])
                  for i in range(settings.num_joints)]
            self.link.move_mit_all(
                [(self._q_hold[i], 0.0, kp[i], kd[i], 0.0)
                 for i in range(settings.num_joints)])
        self.tx_motion_frames += 1

    def _tick(self, now: float) -> None:
        self.link.poll()
        status = self.link.status
        cmd = self.shm.try_read_command()
        mode, reason = self._evaluate(cmd, now, status)
        self._reason = reason

        if mode == MODE_DISABLED:
            if not self._disabled_sent:
                log.warning("ROS side requested disable: the motors will go "
                            "limp, support the arm")
                self.link.disable()
                self._disabled_sent = True
            self._enter(mode)
            return

        self._enter(mode)
        if mode == MODE_TRACKING:
            assert cmd is not None
            self._track(cmd)
        elif mode == MODE_HOLDING:
            self._hold()

    def _publish(self, connected: bool, now: float) -> None:
        """Publish the latest state to shared memory."""
        state = LitearmState()
        state.stamp_s = now
        state.heartbeat_s = now
        state.connected = 1.0 if connected else 0.0
        state.dry_run = 1.0 if self.settings.dry_run else 0.0
        state.cycle_count = self._cycle
        state.applied_command_cycle = self._applied_command_cycle
        state.command_age_s = (self._last_command_age
                               if math.isfinite(self._last_command_age) else -1.0)
        state.last_error = float(self._reason)

        status = self.link.status if self.link is not None else None
        if status is not None:
            count = min(self.settings.num_joints, len(status.joints))
            joints = status.joints
            # ctypes arrays support slice assignment: one whole-block copy,
            # instead of writing element by element.
            state.position[:count] = [float(j.q) for j in joints[:count]]
            state.velocity[:count] = [float(j.dq) for j in joints[:count]]
            state.effort[:count] = [float(j.tau) for j in joints[:count]]
            state.temperature_mos[:count] = [float(j.t_mos)
                                             for j in joints[:count]]
            state.temperature_coil[:count] = [float(j.t_coil)
                                              for j in joints[:count]]
            state.error_code[:count] = [float(j.err) for j in joints[:count]]
            # ⚠ The firmware's state frame has no per-joint feedback age (it
            # only gives a global FB_STALE flag), so what is published here is
            # the **age of the whole frame**. For per-joint criteria use
            # error_code and flags; do not threshold this field axis by axis.
            age = self.link.status_age_s()
            state.feedback_age_s[:count] = [age] * count
            state.feedback_received[:count] = [1.0] * count
            state.enabled = 1.0 if status.enabled else 0.0
            state.faulted = 1.0 if (status.fault or status.faulted_joints()) \
                else 0.0
            state.watchdog_tripped = 1.0 if status.watchdog_tripped else 0.0
        self.shm.publish_state(state)

    # ─────────────────────────── Main loop ───────────────────────────

    def run(self) -> int:
        self._acquire_singleton_lock()
        self._install_signal_handlers()
        settings = self.settings
        period = 1.0 / settings.rate_hz
        log.info("litearm hardware daemon starting: %s", settings.describe())

        self._connected = False      # _shutdown uses it for the final state (see below)
        connected = False
        exit_code = 0
        next_connect_attempt = 0.0
        next_tick = time.monotonic()
        try:
            while not self._stop:
                now = time.monotonic()

                if not connected:
                    # The heartbeat must keep flowing even while disconnected,
                    # so that the ROS side can tell "the daemon never started"
                    # apart from "the daemon is up but cannot reach the
                    # hardware".
                    if now >= next_connect_attempt:
                        try:
                            self._connect()
                            connected = True
                            self._connected = True
                            self._reason = DAEMON_HOLDING_STALE_COMMAND
                            # Force the anchoring action in _enter to run next
                            # cycle: the hold anchor must come from the measured
                            # position (see the MODE_INIT comment). Re-anchor on
                            # reconnect too, since the arm may have been moved
                            # while the link was down.
                            self._mode = MODE_INIT
                            self._q_hold = None
                            log.info("hardware connected (firmware %s, %s)",
                                     settings.firmware_version,
                                     settings.port or "auto-discover")
                            log.info("firmware parameters: %s",
                                     settings.describe_firmware())
                        except DaemonFatalError as exc:
                            log.error("%s", exc)
                            exit_code = 2
                            break
                        except Stm32AccessDenied as exc:
                            # Insufficient permissions **do not fix
                            # themselves**: waiting and retrying change nothing;
                            # a human has to add the group and log in again.
                            # Treat as fatal, do not spam the log.
                            log.error("%s", exc)
                            exit_code = 2
                            break
                        except (_RetryableConnect, Stm32Error, OSError) as exc:
                            log.error("failed to connect to the hardware, "
                                      "retrying in %.1fs: %s",
                                      settings.connect_retry_s, exc)
                            self._close_link()
                            self._connected = False
                            next_connect_attempt = now + settings.connect_retry_s
                    self._publish(connected=False, now=now)
                    self._cycle += 1.0
                    next_tick += period
                    if next_tick < now - period:  # re-anchor if too far behind
                        next_tick = now + period
                    _sleep_until(next_tick)
                    continue

                try:
                    self._tick(now)
                except Stm32NotConnected as exc:
                    self._log_throttled(logging.ERROR, "link-io",
                                        "serial link dropped, reconnecting: %s", exc)
                    self._close_link()
                    connected = False
                    self._connected = False
                    next_connect_attempt = now + settings.connect_retry_s
                except Stm32Error as exc:
                    # Firmware-side error (including rejected commands) — do
                    # not exit; switch to holding, publish the reason and let
                    # the ROS side decide. Throttled: this is hit every cycle
                    # while the fault persists.
                    self._log_throttled(logging.ERROR, "cycle-error",
                                        "cycle error, switching to hold: %s", exc)
                    self._reason = DAEMON_HOLDING_MOTOR_FAULT
                except OSError as exc:
                    self._log_throttled(logging.ERROR, "serial-io",
                                        "serial error, reconnecting: %s", exc)
                    self._close_link()
                    connected = False
                    next_connect_attempt = now + settings.connect_retry_s

                self._publish(connected=connected, now=now)
                self._cycle += 1.0
                next_tick += period
                if next_tick < now - period:
                    next_tick = now + period
                _sleep_until(next_tick)
        finally:
            self._shutdown()
        return exit_code

    # ──────────────────────────── Exit ────────────────────────────

    def _exit_hold_stream(self) -> None:
        """The exit hold stream: "buy" the caller a final stretch of time to
        wind the stack down.

        The launch file's Ctrl-C hits every process at once, and
        ros2_control_node needs time both to stop the controllers and to send
        its last few commands. Throughout that stretch we keep sending the
        frozen reference at the normal rate, so the firmware holds the arm with
        normal stiffness **+ gravity feedforward**; the moment the stream stops,
        the firmware's ``hold`` branch uses ``tau=0`` and the arm sags slightly
        by ``G/kp``.

        A second signal (or the timeout) ends it immediately, so a human
        e-stop is not held up.
        """
        seconds = self.settings.exit_hold_s
        if seconds <= 0.0 or self.link is None or self.link.status is None:
            return
        q_hold = self._q_hold if self._q_hold is not None \
            else self._status_position()
        self._q_hold = list(q_hold)
        log.info("exit hold stream %.1fs (press Ctrl-C again to end it now)",
                 seconds)
        zeros = [0.0] * self.settings.num_joints
        period = 1.0 / self.settings.rate_hz
        deadline = time.monotonic() + seconds
        next_tick = time.monotonic()
        while time.monotonic() < deadline and not self._exit_hold_skip:
            try:
                if not self.settings.mit_passthrough:
                    self.link.move_js(self._q_hold, zeros)
                else:
                    kp = [min(MIT_KP_MAX,
                              self.settings.kp[i] * self.settings.hold_kp_gain)
                          for i in range(self.settings.num_joints)]
                    kd = [min(MIT_KD_MAX, self.settings.kd[i])
                          for i in range(self.settings.num_joints)]
                    self.link.move_mit_all(
                        [(self._q_hold[i], 0.0, kp[i], kd[i], 0.0)
                         for i in range(self.settings.num_joints)])
            except OSError:
                break
            next_tick += period
            _sleep_until(next_tick)

    def _shutdown(self) -> None:
        """The exit sequence: hold stream → park declaration → close the link.

        ``0x20 park`` is a **declaration**: from then on, once the commands stop
        and the watchdog trips, the firmware holds with 1.0× stiffness (rather
        than fail-soft's 0.6×). The firmware-side comment calls it "the
        high-stiffness park declaration before PC disconnect", and this function
        is exactly the path it was designed for.

        ⚠ Holding after park **has no gravity feedforward** (the firmware's
        ``hold`` branch uses ``tau=0``), so the arm sags slightly by ``G/kp``.
        That is existing firmware behaviour; to avoid any sag at all, keep the
        stack running.
        """
        self._reason = DAEMON_SHUTTING_DOWN
        try:
            # ⚠ This must test "did we really connect", not
            # `self.link is not None`: after a failed connect self.link is a
            # **closed object rather than None**, and using the latter would
            # publish connected=1.0 on its deathbed — whereupon the ROS plugin's
            # on_configure would take that as "daemon ready" and carry on
            # activating. Seen on real hardware: no permission on the serial
            # port → daemon exits → the plugin still reports "daemon ready
            # (status: daemon exiting)" and activates, on a frozen set of joint
            # positions.
            self._publish(connected=self._connected, now=time.monotonic())
        except Exception:
            log.debug("failed to publish state before exiting", exc_info=True)

        self._exit_hold_stream()

        if self.link is not None and self.link.is_open:
            log.info("exit: PARK high-stiffness hold declaration — SUPPORT THE ARM")
            try:
                self.link.park()
                # Give this frame time to actually leave the port. The USB CDC
                # has an internal transmit buffer, so "write, then close the
                # port immediately" can drop the frame — and the park
                # declaration is the only basis for "hold at 1.0× stiffness
                # after PC disconnect instead of fail-soft 0.6×"; lose it and
                # the arm sags that little bit further.
                deadline = time.monotonic() + PARK_FLUSH_S
                while time.monotonic() < deadline:
                    self.link.poll()
                    time.sleep(0.005)
            except Exception:
                log.exception("park failed on exit")
        self._close_link()
        self.shm.close()
        self._release_singleton_lock()
        suppressed = {key: entry[1] for key, entry in self._log_throttle.items()
                      if entry[1] > 0}
        if suppressed:
            # Summary of the in-loop log throttling: everything suppressed was a
            # repeated "fault persists" line, spelled out here in one go
            # (without it, an operator has no way to know how long a fault
            # lasted or how many times it occurred).
            log.info("in-loop log throttling summary: %s",
                     ", ".join(f"{key} suppressed {int(count)} times"
                               for key, count in suppressed.items()))
        log.info("daemon exited (motion frames sent: %d)", self.tx_motion_frames)


# ────────────────────────────── CLI ──────────────────────────────


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="litearm_hw_daemon",
        description="litearm hardware daemon: owns the USB CDC exclusively and "
                    "provides a command/state channel to ros2_control over "
                    "shared memory (litearm-stm32 firmware)")
    parser.add_argument("--port", default=None,
                        help="USB CDC device path of the litearm-stm32 "
                             "(default: auto-discovered by VID:PID 1d50:606f)")
    parser.add_argument("--shm-name", default=shm_bridge.DEFAULT_SHM_NAME,
                        help="POSIX shared memory object name")
    # ⚠ The default=None values below are **deliberate**: None means "not given
    #   on the command line", so the value from --hw-config can fill it in.
    #   Spelling out a concrete default makes "not given" and "given as the
    #   default" indistinguishable, and silently overrides the config file. The
    #   real defaults live in DaemonSettings.
    parser.add_argument("--hw-config", default=None,
                        help="optional litearm_hw.yaml: PC-side parameters only "
                             "(port/rates/timeouts/policy); joint-level "
                             "parameters are always read from the firmware. "
                             "The command line takes precedence over this file")
    parser.add_argument("--rate-hz", type=float, default=None,
                        help="command send rate (the firmware control loop runs "
                             "at 300Hz, the watchdog at 100ms)")
    parser.add_argument("--command-timeout-s", type=float, default=None,
                        help="command frame staleness threshold; past it the "
                             "daemon switches to holding")
    parser.add_argument("--feedback-timeout-s", type=float, default=None,
                        help="state frame staleness threshold (the firmware "
                             "reports at 100Hz)")
    parser.add_argument("--connect-retry-s", type=float, default=None,
                        help="retry interval after a failed connect")
    parser.add_argument("--arm-timeout-s", type=float, default=None,
                        help="timeout for waiting for the magnets to really come "
                             "on (the enabled bit) after ENABLE")
    parser.add_argument("--request-timeout-s", type=float, default=None,
                        help="per-reply timeout for startup configuration "
                             "queries (a busy firmware TX drops replies)")
    parser.add_argument("--exit-hold-s", type=float, default=None,
                        help="seconds to keep streaming the hold reference "
                             "before exiting (0 = park and exit straight away)")
    parser.add_argument("--mit-passthrough", action=argparse.BooleanOptionalAction,
                        default=None,
                        help="switch to the MIT_ALL passthrough channel: "
                             "kp/kd/effort take effect per frame and the "
                             "firmware layers on no feedforward of its own "
                             "(default: MOVE_JS position mode)")
    parser.add_argument("--dry-run", action="store_true",
                        help="no-hardware mode: start a fake firmware on a pty; "
                             "the link still runs the real protocol")
    # The five feedforward switches are **tri-state** (BooleanOptionalAction +
    # default=None): omitted = leave the firmware alone; the positive switch =
    # set the bit; --no-xxx = clear it.
    # Without --no-xxx there is no way to run the "back to pure PD" A/B
    # comparison.
    ff = argparse.BooleanOptionalAction
    parser.add_argument("--gravity-compensation", action=ff, default=None,
                        help="override the FF_G bit of the firmware ff_mask "
                             "(omitted = leave the firmware alone)")
    parser.add_argument("--friction-compensation", action=ff, default=None,
                        help="override the FF_FRICTION bit of the firmware ff_mask")
    parser.add_argument("--inertia-compensation", action=ff, default=None,
                        help="override the FF_INERTIA|FF_CORIOLIS bits of the "
                             "firmware ff_mask (⚠ in MOVE_JS mode the firmware "
                             "computes no inertia terms, so setting them is "
                             "useless)")
    parser.add_argument("--integral-compensation", action=ff, default=None,
                        help="override the FF_INTEGRAL bit of the firmware ff_mask")
    parser.add_argument("--damping-compensation", action=ff, default=None,
                        help="override the firmware kd_extra vector (no "
                             "dedicated FF bit; off = zero it, on = restore the "
                             "factory 6/6/6/6/0/0/0)")
    parser.add_argument("--verbose", action="store_true",
                        help="pass through the link layer's verbose logging")
    parser.add_argument("--log-level", default="INFO",
                        choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    return parser.parse_args(list(argv) if argv is not None else None)


def _ff_overrides(args: argparse.Namespace) -> dict:
    return {
        "gravity": args.gravity_compensation,
        "friction": args.friction_compensation,
        "inertia": args.inertia_compensation,
        "integral": args.integral_compensation,
        "damping": args.damping_compensation,
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
    if args.verbose:
        logging.getLogger("litearm.stm32_link").setLevel(logging.DEBUG)

    try:
        settings = _build_settings(args)
    except (ValueError, ShmError) as exc:
        log.error("invalid arguments: %s", exc)
        return 2

    try:
        daemon = LitearmHwDaemon(settings)
    except RuntimeError as exc:
        log.error("%s", exc)
        return 3
    except ShmError as exc:
        log.error("shared memory initialisation failed: %s", exc)
        return 3

    return daemon.run()


if __name__ == "__main__":
    sys.exit(main())
