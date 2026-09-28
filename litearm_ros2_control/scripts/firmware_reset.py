#!/usr/bin/env python3
"""Inspect and clear firmware fault / emergency-stop latches (manual entry
point).

Why this is needed
————————————————————————————————————————————————————————————
When the firmware rejects ``ENABLE`` with ``ERR{0x10,0x06}`` (emergency stop or
joint fault latched), ``hw_daemon`` **only** does "close the serial port → wait
2s → reconnect and resend ENABLE" and can never clear it — this is a **latched
state** and needs an explicit ``RESET(0x14)`` (clears EMERGENCY) /
``CLEAR_FAULTS(0x13)`` (clears motor-side error codes). Before this, only
``Stm32Link`` wrapped those two commands and no script exposed them, so once
latched you had to dig through the code and do it by hand (real hardware got
stuck like that for a whole round on 2026-09-21: the plugin waited 10s, decided
"daemon not ready" → FATAL → configure failed → ``ros2_control_node`` threw →
the whole launch went down with it).

The latch does not only come from a human hitting the emergency stop — the
firmware's own main-loop heartbeat supervision (main goes 500ms without a
heartbeat while enabled) calls ``ctrl_emergency_stop()`` too, and the symptoms
are exactly like "someone pressed the emergency stop": ``mode=EMERGENCY`` +
``FAULT`` + every axis flagged. So read the status first, do not guess.

This script does exactly three things: **read the status → clear explicitly →
compare before and after**. It **sends no motion command and does not enable
the motors**; in fact it refuses to act when ``enabled=True`` (something else
is in control) — enabling is the daemon's job, here we only open the latch.

Usage
————————————————————————————————————————————————————————————
::

    scripts/firmware_reset.py                          # read-only: status + advice (no side effects)
    scripts/firmware_reset.py --reset                  # reset the state machine (clear the EMERGENCY latch)
    scripts/firmware_reset.py --reset --clear-faults   # reset + clear fault bits (the usual combination)
    scripts/firmware_reset.py --fake                   # no-hardware rehearsal: fake firmware + inject one emergency stop

Exit codes: 0 = target state reached / 1 = still not clean after acting (with a
recheck and advice) / 2 = usage error / 3 = device cannot be opened or the
safety gate blocked it.

Once the actions have finished, just restart the stack as prompted:
``ros2 launch litearm_manipulation manipulation.launch.xml dry_run:=false start_rviz:=true``
"""

import argparse
import sys
import time
from pathlib import Path

# Make sure this package is importable when run straight from the source tree
# (the script may also be installed under lib/ and run from there, in which
# case the ament index is used).
_SOURCE_PYTHON = Path(__file__).resolve().parents[1] / "python"
if _SOURCE_PYTHON.is_dir() and str(_SOURCE_PYTHON) not in sys.path:
    sys.path.insert(0, str(_SOURCE_PYTHON))

from litearm_ros2_control import fake_firmware, stm32_proto as proto  # noqa: E402
from litearm_ros2_control.stm32_link import Stm32Link, find_port  # noqa: E402

# Time given to the firmware after a command to flush the new state into a
# status frame (status frames run at 100Hz, and 0.6s is enough to cover the
# three steps "command handling + disable debounce + state machine switch").
SETTLE_S = 0.6
# Upper bound on waiting for a **new** status frame.
STATUS_TIMEOUT_S = 2.0


def snapshot(link: Stm32Link):
    """Fetch a **new** status frame.

    ``link.status`` is a snapshot taken at the moment of the last ``poll()``,
    so reading it directly returns a stale frame (this project has already
    paid for that once: enabled=True was still printed after disabling). So
    poll first, and only accept it as new once seq has actually changed.
    """
    seq0 = link.status.seq if link.status is not None else None
    deadline = time.monotonic() + STATUS_TIMEOUT_S
    while time.monotonic() < deadline:
        link.poll()
        status = link.status
        if status is not None and (seq0 is None or status.seq != seq0):
            return status
        time.sleep(0.01)
    return link.status


def describe(status) -> str:
    flags = [name for bit, name in proto.FLAG_NAMES.items() if status.flags & bit]
    lines = [f"  mode={status.mode_name} · enabled={status.enabled} · "
             f"flags={flags if flags else ['(none)']}"]
    if status.joint_fault is not None:
        lines.append(f"  joint_fault=0x{status.joint_fault:02X}"
                     f" ({bin(status.joint_fault).count('1')} axes flagged)")
    lines.append(f"  q  = [{', '.join(f'{j.q:+.4f}' for j in status.joints)}]")
    lines.append(f"  err= {[j.err for j in status.joints]} (0=disabled 1=enabled)")
    return "\n".join(lines)


def is_locked(status) -> bool:
    """Is it still in the state where "ENABLE is rejected with 0x06"?"""
    return (status.mode == proto.ARM_MODE_EMERGENCY or status.fault
            or bool(status.joint_fault))


def advise(status) -> str:
    if status.mode == proto.ARM_MODE_EMERGENCY or status.fault:
        return ("advice: --reset (0x14, clear the EMERGENCY latch first); if "
                "the recheck still shows FAULT, add --clear-faults")
    if status.joint_fault:
        return ("advice: --clear-faults (0x13; mode is normal but axes are "
                "flagged as faulty)")
    return "status is clean, nothing to do — you can bring up the stack"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="inspect/clear the fault and emergency-stop latches of the "
                    "litearm-stm32 (sends no motion command, does not enable "
                    "the motors)")
    parser.add_argument("--port", default="",
                        help="USB CDC device path; empty = auto-discover by "
                             "VID:PID")
    parser.add_argument("--reset", action="store_true",
                        help="send RESET(0x14): reset the state machine, clear "
                             "the EMERGENCY latch")
    parser.add_argument("--clear-faults", action="store_true",
                        help="send CLEAR_FAULTS(0x13): clear motor/joint-side "
                             "fault codes")
    parser.add_argument("--fake", action="store_true",
                        help="start a fake firmware and inject one emergency "
                             "stop, rehearsing the whole flow (no hardware)")
    args = parser.parse_args(argv)

    firmware = None
    if args.fake:
        firmware = fake_firmware.FakeFirmware()
        port = firmware.start()
        print(f"fake firmware started: {port}")
    else:
        port = args.port or find_port() or ""
        if not port:
            print("litearm-stm32 not found (VID:PID 1d50:606f). Specify it "
                  "with --port, or use --fake for a no-hardware rehearsal.",
                  file=sys.stderr)
            return 3

    link = Stm32Link(port)
    try:
        link.open()
        print(f"opened {port}")

        if args.fake:
            # rehearsal: inject an emergency stop first to reproduce the
            # latched state where "ENABLE replies 0x06" on real hardware
            link.emergency_stop()
            time.sleep(SETTLE_S)
            print("(rehearsal) injected one emergency stop — ENABLE is now "
                  "rejected with 0x06")

        before = snapshot(link)
        print("\nstatus before:")
        if before is None:
            print("  no status frame received (is the firmware running?)",
                  file=sys.stderr)
            return 3
        print(describe(before))

        if before.enabled:
            print("\nrefusing to act: the status frame shows enabled=True — "
                  "something else is in control (the daemon or another tool). "
                  "Stop it before clearing faults, otherwise the next frame of "
                  "commands overwrites what you just cleared.", file=sys.stderr)
            return 3

        if not (args.reset or args.clear_faults):
            print(f"\n{advise(before)}")
            return 0

        if args.reset:
            link.reset()
            print("\nsent RESET(0x14)")
        if args.clear_faults:
            link.clear_faults()
            print("sent CLEAR_FAULTS(0x13)")
        time.sleep(SETTLE_S)

        after = snapshot(link)
        print("\nstatus after:")
        if after is None:
            print("  no status frame received", file=sys.stderr)
            return 1
        print(describe(after))

        if is_locked(after):
            print(f"\nstill latched: {advise(after)}")
            return 1
        print("\nlatches/faults cleared — you can bring up the stack now")
        return 0
    finally:
        link.close()
        if firmware is not None:
            firmware.stop()


if __name__ == "__main__":
    sys.exit(main())
