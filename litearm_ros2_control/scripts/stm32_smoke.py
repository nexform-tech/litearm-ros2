#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""stm32_smoke.py — smoke acceptance of the litearm-stm32 link (the whole
thing runs without hardware).

The order it runs in is the first thing you should do on real hardware:

    open the port → read the firmware version → read the 7 joint parameters
    (0x24) → read ff_mask (0x2C item9) → ENABLE → wait for the enabled bit →
    [optional] a small single-axis MOVE_JS → stop publishing and let the
    watchdog hold position → PARK → DISABLE → print the link counters

Four ways to use it::

    # fully read-only: **not one word that would change firmware state** is
    # sent (no ENABLE, no motion, no park). Plug the board in and run it; the
    # arm will not move at all. Use this to check "is the board there / which
    # firmware version / are the parameters right".
    scripts/stm32_smoke.py --read-only

    # no hardware: bring up a fake firmware and run the whole flow (motion and
    # watchdog included)
    scripts/stm32_smoke.py --fake

    # real hardware: it will ENABLE (energise the motors) but **not move**
    # (it goes only as far as enable/status/park/disable)
    scripts/stm32_smoke.py
    scripts/stm32_smoke.py --move   # additionally allow 0.03 rad of motion (make sure the arm is supported!)

    # against a fake firmware started elsewhere (tools/bench_fake_fw.py, say)
    scripts/stm32_smoke.py --port /dev/pts/7

On real hardware **look at the license first**: while the firmware is not
activated ``ENABLE`` replies ``ERR{0x10,0x08}``, which this script reports
directly with guidance; the other commands are unaffected.
"""

import argparse
import sys
import time
from pathlib import Path

# Make sure this package is importable when run straight from the source tree.
_SOURCE_PYTHON = Path(__file__).resolve().parents[1] / "python"
if _SOURCE_PYTHON.is_dir() and str(_SOURCE_PYTHON) not in sys.path:
    sys.path.insert(0, str(_SOURCE_PYTHON))

from litearm_ros2_control import fake_firmware, stm32_proto as proto  # noqa: E402
from litearm_ros2_control.stm32_link import (ERR_HINTS,  # noqa: E402
                                             Stm32Error, Stm32Link,
                                             find_port)

NUM_JOINTS = 7

# Maximum motion amplitude allowed on real hardware (rad). A smoke test is not
# a calibration, and 0.03 rad is already enough to see whether the data path
# is connected, while even hitting something would not cause serious trouble.
MAX_AMPLITUDE_RAD = 0.2
DEFAULT_AMPLITUDE_RAD = 0.03
# Rate limit for the first motion on real hardware. Slowness is the first line
# of defence: at 0.05 rad/s, 0.03 rad takes 0.6s, so any anomaly has enough
# time to be seen or interrupted by an emergency stop.
DEFAULT_SPEED_RAD_S = 0.05
MAX_SPEED_RAD_S = 2.0


class Report:
    """Collect the checks and give the verdict at the end (its return code)."""

    def __init__(self) -> None:
        self.failures = []

    def check(self, title: str, ok: bool, detail: str = "") -> bool:
        mark = "✔" if ok else "✘"
        line = f"  {mark} {title}"
        if detail:
            line += f": {detail}"
        print(line, flush=True)
        if not ok:
            self.failures.append(title)
        return ok

    def done(self) -> int:
        print()
        if self.failures:
            print(f"{len(self.failures)} checks failed: "
                  f"{', '.join(self.failures)}")
            return 1
        print("all checks passed")
        return 0


def _spin(link: Stm32Link, seconds: float, period_s: float = 0.005) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        link.poll()
        time.sleep(period_s)


def _firmware_path() -> Path:
    """Firmware source path (optional) — used to compare the read-back
    parameters against defaults.c."""
    override = Path.home() / "wkspace" / "litearm-stm32"
    return override


def _measure_report_rate(link: Stm32Link, seconds: float) -> tuple:
    """Passively measure the status frame rate: returns (measured Hz, frames
    received, unique seq count).

    Only polls, sends nothing — the firmware reports at 100Hz on its own.
    It counts **seq changes** rather than "how many polls": the same frame may
    be read several times from the buffer, and counting frames would make the
    rate look too high.
    """
    seen = set()
    frames = 0
    deadline = time.monotonic() + seconds
    prev_seq = None
    while time.monotonic() < deadline:
        before = link.status_count
        link.poll()
        frames += link.status_count - before
        status = link.status
        if status is not None and status.seq != prev_seq:
            prev_seq = status.seq
            seen.add(status.seq)
        time.sleep(0.002)
    elapsed = seconds
    return len(seen) / elapsed if elapsed > 0 else 0.0, frames, len(seen)


def _report_params_and_ff(link: Stm32Link, report: Report, status,
                         firmware_root: Path) -> bool:
    """Print the firmware parameters and feedforward switches (shared by both
    paths). Returns whether everything was available."""
    print("\n[2] joint parameters (all read from the firmware, 0x24)")
    params = []
    for index in range(len(status.joints)):
        param = link.get_joint_param(index)
        if param is None:
            report.check(f"joint {index + 1} parameters read back", False,
                         "timeout")
            return False
        params.append(param)
    for param in params:
        print(f"      j{param.index + 1}: kp={param.kp:g} kd={param.kd:g} "
              f"tau_max={param.tau_max:g} "
              f"q∈[{param.q_min:.6f}, {param.q_max:.6f}]")
    if not report.check("all 7 joint parameters present",
                        len(params) == NUM_JOINTS):
        return False

    # Compare against the firmware's factory table (only done when the
    # litearm-stm32 sources are present — not a hard dependency).
    defaults_c = firmware_root / "User" / "litearm" / "params" / "defaults.c"
    if defaults_c.exists():
        expected = {
            "kp": fake_firmware.DEFAULT_KP[:len(params)],
            "kd": fake_firmware.DEFAULT_KD[:len(params)],
            "tau_max": fake_firmware.DEFAULT_TAU_MAX[:len(params)],
        }
        for field, values in expected.items():
            actual = [getattr(item, field) for item in params]
            report.check(f"{field} matches the firmware factory table "
                         f"(defaults.c)",
                         all(abs(a - e) < 1e-6
                             for a, e in zip(actual, values)),
                         f"{actual}")
    else:
        print(f"      (skipping the factory table comparison: {defaults_c} is "
              f"not present)")

    print("\n[3] built-in feedforward switches (0x2C item9)")
    mask = link.get_ff_mask(timeout_s=1.0)
    if not report.check("read ff_mask", mask is not None,
                        proto.format_ff_mask(mask)
                        if mask is not None else "timeout"):
        return True
    print(f"      ff_mask=0x{mask:03X} · friction_model="
          f"{link.get_ff_scalar(proto.FF_SCALAR_FRICTION_MODEL)}"
          f" · fric_db={link.get_ff_scalar(proto.FF_SCALAR_FRIC_DB)}"
          f" · hold_kp_gain={link.get_ff_scalar(proto.FF_SCALAR_HOLD_KP_GAIN)}"
          f" · payload_mass={link.get_ff_scalar(proto.FF_SCALAR_PAYLOAD_MASS)}")
    damping = link.get_ff_vec(proto.FF_VEC_KD_EXTRA)
    if damping is not None:
        print(f"      kd_extra={[round(v, 2) for v in damping]}")
    gravity_scale = link.get_ff_vec(proto.FF_VEC_GRAVITY_SCALE)
    if gravity_scale is not None:
        print(f"      gravity_scale={[round(v, 3) for v in gravity_scale]}")
    report.check("FF_MASTER is set (master switch of the built-in "
                 "feedforward)",
                 bool(mask & proto.FF_MASTER),
                 "with MASTER off, position mode adds no feedforward either")
    return True


def run_read_only(link: Stm32Link, report: Report, *,
                  measure_s: float, firmware_root: Path) -> None:
    """**Zero-side-effect** precheck: no ENABLE, no motion, no park.

    The difference from :func:`run` is not that it "does a few steps less" but
    that it **sends not one word that would change firmware state**: ``run()``
    ENABLEs even in the mode where the motors do not move (energising them),
    and that is already an operation with physical consequences. So use this
    path to check "is the board there, which firmware version, are the
    parameters read correctly".
    """
    print("\n[1] firmware identity")
    version = link.get_firmware(timeout_s=1.0)
    report.check("read firmware version (0x41)", version is not None,
                 version or "timeout")
    if version is None:
        report.check("link is up (status frame received)", False,
                     "no reply at all — is the port right? is the board "
                     "running?")
        return

    status = link.wait_status(2.0)
    if not report.check("status frame received (0x40)", status is not None):
        return
    print(f"      joints {len(status.joints)} · mode={status.mode_name} "
          f"· enabled={status.enabled} · flags={status.flag_names() or 'none'}")
    if not report.check("motors not enabled (the premise of a read-only "
                        "precheck)", not status.enabled,
                        "already enabled means another process is in control, "
                        "stop it before checking"):
        return
    print(f"      q   = {[round(j.q, 4) for j in status.joints]}")
    print(f"      dq  = {[round(j.dq, 4) for j in status.joints]}")
    print(f"      err = {[j.err for j in status.joints]} (0=disabled 1=enabled)")
    print(f"      temp = {[(round(j.t_mos, 1), round(j.t_coil, 1)) for j in status.joints]}")
    print(f"      joint_fault = {status.joint_fault}")
    # ⚠ The reverse trap: **"all zeros" is not a pose, it is "no feedback read
    # yet"**. While not enabled the firmware sends a zero-torque probe frame
    # every 3 ticks, and a motor "only reports state once it receives a frame"
    # — so the feedback has to build up. At this moment q/dq/temps all sit at
    # their initial value of 0, which looks like an arm whose seven axes
    # happen to be exactly at zero. This note is here because we **really hit
    # it**: the first read-only precheck printed all zeros as if it were a
    # pose, and it still passed the "no fault bits" check. A real arm cannot
    # have all seven axes at exactly 0.0000 and a temperature of exactly 0.0.
    all_zero = (all(j.q == 0.0 for j in status.joints)
                and all(j.dq == 0.0 for j in status.joints)
                and all(j.t_mos == 0.0 and j.t_coil == 0.0
                        for j in status.joints))
    report.check("motors have reported feedback (these q are not initial "
                 "zeros)", not all_zero,
                 "" if not all_zero else
                 "q/dq/temps are **all 0** = no motor feedback has been "
                 "received yet, this is not a real pose. Check the motor "
                 "power supply and the CAN wiring between the board and the "
                 "arm, then retry shortly")
    if all_zero:
        return
    report.check("no fault bits",
                 not status.fault and not status.faulted_joints(),
                 "with fault bits set, find the cause first, do not rush to "
                 "enable")

    if not _report_params_and_ff(link, report, status, firmware_root):
        return

    print(f"\n[4] status frame rate (passively measured for {measure_s:g}s, "
          f"sending no commands)")
    hz, frames, unique = _measure_report_rate(link, measure_s)
    report.check("report rate close to 100Hz", 70.0 <= hz <= 130.0,
                 f"measured {hz:.1f} Hz ({unique} seqs / {measure_s:g}s, "
                 f"{frames} frames polled in total)")
    report.check("no CRC errors", link.crc_errors == 0,
                 f"crc_err={link.crc_errors}, {link._framer.discarded}B "
                 f"discarded outside frames")

    print(f"\n[5] link counters\n      {link.counters()}")


def run(link: Stm32Link, report: Report, *, allow_motion: bool,
        amplitude: float, joint: int, move_speed: float,
        firmware_root: Path, on_fake: bool) -> None:
    print("\n[1] firmware identity")
    version = link.get_firmware(timeout_s=1.0)
    report.check("read firmware version (0x41)", version is not None,
                 version or "timeout")
    if version is None:
        report.check("link is up (status frame received)", False,
                     "no reply at all — is the port right? is the board "
                     "running?")
        return

    status = link.wait_status(2.0)
    if not report.check("status frame received (0x40)", status is not None):
        return
    print(f"      joints {len(status.joints)} · mode={status.mode_name} "
          f"· enabled={status.enabled} · flags={status.flag_names() or 'none'}")
    report.check("safe state right after power-up (motors not enabled)",
                 not status.enabled,
                 "if it is already enabled at power-up, the firmware is not "
                 "running this repo's startup sequence")

    if not _report_params_and_ff(link, report, status, firmware_root):
        return

    print("\n[4] enable (0x10)")
    # The firmware's ENABLE is **two-stage**: the first one after startup has
    # to write the CMODE register of all 7 motors, and at that moment it
    # replies 0x03 and registers the pending enable; once every feedback is
    # in, enable_pending_poll energises the magnets automatically, and the
    # host only gets the ACK when it resends ENABLE. The timing measured on
    # real hardware is in ctrl_enable in control_loop.c:
    #   "ENABLE#1 -> 0x03 (first CMODE write), ENABLE#2 -> ACK"
    # So 0x03 is **not a failure** and must be resent in place (not by closing
    # the serial port and reconnecting).
    deadline = time.monotonic() + 8.0
    told_first_write = False
    enabled = None
    while time.monotonic() < deadline:
        ok, reason = link.command(proto.CMD_ENABLE, timeout_s=1.0)
        if ok is False and reason == 0x08:
            report.check("ENABLE rejected by the license gate", False,
                         "the firmware is not activated. Activate it with "
                         "tools/litearm-license and retry; until then every "
                         "motion command is rejected")
            return
        if ok is False and reason == 0x06:
            report.check("ENABLE rejected (emergency stop / joint fault "
                         "latched)", False,
                         "clear the faults or reset first (0x13 / 0x14)")
            return
        if ok is False and reason == 0x03:
            if not told_first_write:
                told_first_write = True
                print("      ENABLE replied 0x03 (the firmware's first write to "
                      "the motor CMODE / feedback not ready) — resending in "
                      "place, as the firmware sequence requires")
        elif ok is False:
            report.check("ENABLE accepted", False,
                         f"reason code {reason} "
                         f"({ERR_HINTS.get(reason, 'see usb_cmd.h')})")
            return
        if link.wait_enabled(1.0) is not None:
            enabled = link.status
            break
    report.check("ENABLE accepted and the magnets really energised",
                 enabled is not None,
                 "the ACK only means registered; energising waits for every "
                 "feedback (wait_enabled)")
    if enabled is None:
        return
    print(f"      after energising mode={link.status.mode_name} "
          f"err={[j.err for j in link.status.joints]}")

    print("\n[5] motion (MOVE_JS, no tau_ff → the firmware adds its built-in "
          "feedforward)")
    if not allow_motion:
        print("  - skipped (on real hardware the motors move only with --move; "
              "--fake allows it automatically)")
    else:
        start_q = [j.q for j in link.status.joints]
        target = list(start_q)
        target[joint] += amplitude
        # ⚠ **A non-zero velocity is mandatory**: in the firmware's MOVE_JS the
        #   dq_ref doubles as the slew limit of the position reference
        #   (control_loop.c: v_lim = clamp(|dq_ref|, 0, speed_limit);
        #    cmd->q_ref = slew_linear(target, q_ref, v_lim*dt)),
        #   so "position only, velocity 0" freezes the reference and the arm
        #   does not move a single step.
        #   Here we give a very small velocity towards the target — for the
        #   first slow single-axis move, "slow" is exactly what is wanted.
        speed = min(move_speed, max(1e-3, amplitude / 0.5))
        dq = [0.0] * NUM_JOINTS
        print(f"      target j{joint + 1}: {start_q[joint]:+.4f} → "
              f"{target[joint]:+.4f} rad (rate {speed:.3f} rad/s, other axes "
              f"held)")
        deadline = time.monotonic() + max(1.2, 2.0 * amplitude / speed + 0.5)
        while time.monotonic() < deadline:
            link.poll()
            # The velocity command is recomputed from the difference between
            # the **measured** position and the target (the same thing a real
            # JTC does); once there, it goes to zero and the reference stops
            # with it.
            for i, jnt in enumerate(link.status.joints):
                error = target[i] - jnt.q
                dq[i] = max(-speed, min(speed, error / max(0.004, 1e-3)))
            link.move_js(target, dq)
            time.sleep(0.004)
        link.poll()
        moved = link.status.joints[joint].q
        report.check(f"joint {joint + 1} reached the target",
                     abs(moved - target[joint]) < 0.02,
                     f"{start_q[joint]:.4f} → {moved:.4f} "
                     f"(target {target[joint]:.4f})")
        report.check("mode switched to MOVE_JS",
                     firmware_mode_ok(link, proto.ARM_MODE_MOVE_JS),
                     link.status.mode_name)
        moved_all = [j.q for j in link.status.joints]
        drift = [moved_all[i] - start_q[i] for i in range(NUM_JOINTS)]
        print(f"      per-axis displacement (rad) = {[round(v, 4) for v in drift]}")
        # ⚠ Known exception: **an axis in "controlled return" keeps moving**.
        # A load-bearing joint that has lost torque stops at a sagging
        # equilibrium point that may lie outside the soft limits; on enable the
        # firmware clamps the reference inside the limits and brings it back, a
        # return that takes about 3s (the grace period of
        # `ctrl_axis_returning`), while `wait_enabled` returns as soon as the
        # enabled bit is set (~0.3s) — so that axis is still travelling back
        # during the motion phase. This is by design, not "the other axes
        # moving randomly".
        others = [(i, drift[i]) for i in range(NUM_JOINTS)
                  if i != joint and abs(drift[i]) > 0.02]
        if not others:
            report.check("the other axes barely moved", True)
        else:
            print(f"      other axes that moved: "
                  f"{[f'j{i+1} {v:+.4f}' for i, v in others]} (expected if it "
                  f"is a controlled return; returns happen in the first ~3s "
                  f"after enabling)")
            worst_other = max(abs(v) for _i, v in others)
            report.check("displacement of the other axes is bounded (a return "
                         "or lag, not random motion)", worst_other < 0.25,
                         f"max {worst_other:.4f} rad")

    print("\n[6] stop publishing → firmware watchdog holds position (100ms)")
    held_at = [j.q for j in link.status.joints]
    _spin(link, 0.4)
    report.check("WD_TRIPPED is set", link.status.watchdog_tripped is True,
                 "still not tripped after 0.4s of silence — is the watchdog "
                 "not working?")
    drift = [link.status.joints[i].q - held_at[i] for i in range(len(held_at))]
    worst = max(abs(v) for v in drift)
    if on_fake:
        # The fake firmware implements the hold as "freeze q_ref", so not a
        # single position should move.
        report.check("no position drift while holding", worst < 0.02,
                     f"max drift {worst:.5f} rad")
    else:
        # ⚠ On real hardware it **will sag**, and that is not a fault: the
        # firmware's fail-soft hold is tau=0 with a stiffness of only 0.6×
        # (`hold_kp_scale`) and **no gravity feedforward** — a load-bearing
        # joint keeps sagging to the equilibrium point of kp·0.6·Δq = G
        # (documented §4.4: measured J2 8~13.5°, J4 −9°).
        # So this does not assert "no drift" (that would make the first run on
        # real hardware report a scary but wrong failure); it asserts "the sag
        # is bounded, it is not a fall", and prints the per-axis sag as data.
        report.check("sag is bounded (not a fall)", worst < 0.6,
                     f"max sag {worst:.4f} rad = {worst * 57.2958:.2f}° "
                     f"(expected behaviour: fail-soft with no gravity "
                     f"feedforward)")
        print(f"      per-axis sag (rad) = "
              f"{[round(v, 4) for v in drift]}")

    # Commands resume → kick → the flag clears on its own.
    link.move_js([j.q for j in link.status.joints], [0.0] * NUM_JOINTS)
    _spin(link, 0.1)
    report.check("WD_TRIPPED clears automatically once commands resume",
                 link.status.watchdog_tripped is False)

    print("\n[7] PARK declaration (0x20 mode=0)")
    link.park()
    _spin(link, 0.1)
    report.check("after PARK the watchdog holds position at 1.0× stiffness",
                 link.status.watchdog_tripped is True,
                 "PARK is a high-stiffness declaration made before the PC "
                 "disconnects; it only takes effect once the watchdog trips")

    print("\n[8] disable (0x11)")
    link.disable()
    _spin(link, 0.2)
    report.check("motors disabled", link.status.enabled is False,
                 "the arm goes limp and drops — support it first on real "
                 "hardware")
    report.check("mode back to INIT",
                 link.status.mode == proto.ARM_MODE_INIT,
                 link.status.mode_name)

    print("\n[9] link counters")
    print(f"      {link.counters()}")
    report.check("no CRC errors", link.crc_errors == 0)


def firmware_mode_ok(link: Stm32Link, expected: int) -> bool:
    return link.status is not None and link.status.mode == expected


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="litearm-stm32 link smoke acceptance (--fake runs the "
                    "whole thing with no hardware)")
    parser.add_argument("--port", default="",
                        help="USB CDC device path; empty = auto-discover by "
                             "VID:PID")
    parser.add_argument("--read-only", action="store_true",
                        help="zero-side-effect precheck: no ENABLE, no motion, "
                             "no park (plug the board in and run it; the arm "
                             "will not move at all)")
    parser.add_argument("--fake", action="store_true",
                        help="start a fake firmware and run against it (no "
                             "hardware, motion allowed)")
    parser.add_argument("--move", action="store_true",
                        help="explicitly allow motor motion on real hardware "
                             "(by default it stops at enable/status/park/"
                             "disable)")
    parser.add_argument("--amplitude", type=float, default=DEFAULT_AMPLITUDE_RAD,
                        help=f"motion amplitude in rad (max {MAX_AMPLITUDE_RAD})")
    parser.add_argument("--joint", type=int, default=0,
                        help="which axis to move (0-based, default 0)")
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED_RAD_S,
                        help=f"motion rate in rad/s (max {MAX_SPEED_RAD_S}); "
                             f"slow is safer than fast, ≤0.05 is recommended "
                             f"for the first real-hardware run")
    parser.add_argument("--measure-s", type=float, default=2.0,
                        help="how long to measure the status frame rate in "
                             "read-only mode (seconds)")
    args = parser.parse_args(argv)

    if args.read_only and args.fake:
        print("--read-only and --fake are mutually exclusive (a fake firmware "
              "is a zero-risk path by definition)", file=sys.stderr)
        return 2
    if args.read_only and args.move:
        print("--read-only and --move contradict each other, do not pass both",
              file=sys.stderr)
        return 2
    if not (0.0 < args.amplitude <= MAX_AMPLITUDE_RAD):
        print(f"--amplitude must be within (0, {MAX_AMPLITUDE_RAD}]",
              file=sys.stderr)
        return 2
    if not (0 <= args.joint < NUM_JOINTS):
        print(f"--joint must be within [0, {NUM_JOINTS - 1}]", file=sys.stderr)
        return 2
    if args.measure_s <= 0.0:
        print("--measure-s must be positive", file=sys.stderr)
        return 2
    if not (0.0 < args.speed <= MAX_SPEED_RAD_S):
        print(f"--speed must be within (0, {MAX_SPEED_RAD_S}]", file=sys.stderr)
        return 2

    firmware = None
    port = args.port
    allow_motion = args.move
    if args.fake:
        firmware = fake_firmware.FakeFirmware()
        port = firmware.start()
        allow_motion = True
        print(f"fake firmware started: {port} (first-order kinematics model, "
              f"not usable for tuning gains)")
    elif not port:
        port = find_port() or ""
        if not port:
            print("litearm-stm32 not found (VID:PID 1d50:606f). Specify it "
                  "with --port, or use --fake for a no-hardware rehearsal.",
                  file=sys.stderr)
            return 3

    link = Stm32Link(port)
    report = Report()
    try:
        link.open()
        print(f"opened {port}")
        if args.read_only:
            print("mode: read-only (no ENABLE / no motion / no park)")
            run_read_only(link, report, measure_s=args.measure_s,
                          firmware_root=_firmware_path())
        else:
            run(link, report, allow_motion=allow_motion,
                amplitude=args.amplitude, joint=args.joint,
                move_speed=args.speed,
                firmware_root=_firmware_path(), on_fake=args.fake)
    except (Stm32Error, OSError) as exc:
        # Stm32AccessDenied's str() carries its own recovery path (add the
        # dialout group + log in again).
        print(f"link error: {exc}", file=sys.stderr)
        return 3
    finally:
        link.close()
        if firmware is not None:
            firmware.stop()
    return report.done()


if __name__ == "__main__":
    sys.exit(main())
