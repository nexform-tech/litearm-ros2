#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""enable_probe.py — a one-shot enable-sequence probe with full observation
(stage B only).

It only does "ENABLE (resent if needed) → observe → DISABLE" and **sends no
motion commands at all**. The point is to nail down, on real hardware, the two
stages of the firmware ENABLE semantics:
  ENABLE#1 → 0x03 (first write to CMODE) / ENABLE#2 → ACK (see ctrl_enable in
  control_loop.c)
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from litearm_ros2_control import stm32_proto as proto  # noqa: E402
from litearm_ros2_control.stm32_link import Stm32Link  # noqa: E402


def snap(link, label):
    # ⚠ poll first: link.status is only updated inside poll(), so reading it
    # directly returns a stale frame. We really did hit this bug — after
    # DISABLE we read an old frame from while the magnets were energising,
    # which looked like "disable failed".
    link.poll()
    st = link.status
    flags = ",".join(st.flag_names()) or "-"
    print(f"  {label:12s} flags=0x{st.flags:04X} [{flags}] enabled={st.enabled} "
          f"mode={st.mode_name} err={[j.err for j in st.joints]}")
    print(f"               q={[round(j.q, 4) for j in st.joints]}")


def watch(link, seconds, label):
    print(f"\n=== {label} ({seconds:g}s, sending no commands at all) ===")
    t0 = time.monotonic()
    last = None
    while time.monotonic() - t0 < seconds:
        link.poll()
        st = link.status
        if st is not None and st.seq != last:
            last = st.seq
            flags = ",".join(st.flag_names()) or "-"
            print(f"  t={time.monotonic() - t0:4.1f}s enabled={st.enabled} "
                  f"mode={st.mode_name} err={[j.err for j in st.joints]} "
                  f"J4={st.joints[3].q:+.4f} flags={flags}")
        time.sleep(0.25)


def main():
    link = Stm32Link()
    link.open()
    try:
        link.wait_status(2.0)
        print("=== baseline ===")
        snap(link, "baseline")

        print("\n=== ENABLE #1 (firmware is expected to reply 0x03: the first "
              "write to the CMODE register) ===")
        ok, reason = link.command(proto.CMD_ENABLE, timeout_s=3.0)
        print(f"  reply ok={ok} reason={reason}")
        watch(link, 4.0, "after ENABLE#1")

        if not link.status.enabled:
            print("\n=== still not energised → ENABLE #2 (ACK expected) ===")
            ok, reason = link.command(proto.CMD_ENABLE, timeout_s=3.0)
            print(f"  reply ok={ok} reason={reason}")
            watch(link, 4.0, "after ENABLE#2")

        print("\n=== conclusion ===")
        snap(link, "final")

        print("\n=== wrap-up: DISABLE (back to the state we found it in) ===")
        link.disable()
        time.sleep(0.6)
        snap(link, "after")
        if link.status.enabled:
            print("  ⚠ enabled is still true after DISABLE — the arm is still "
                  "powered, check it by hand!")
        else:
            print("  ✔ confirmed disabled")
    finally:
        link.close()


if __name__ == "__main__":
    main()
