#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""enable_probe.py — 一次性、带完整观测的使能时序探针（阶段 B 专用）。

只做「ENABLE（必要时重发）→ 观察 → DISABLE」，**完全不发运动命令**。
目的是把固件 ENABLE 的两段语义在真机上问清楚：
  ENABLE#1 → 0x03（首写 CMODE）/ ENABLE#2 → ACK（见 control_loop.c 的 ctrl_enable）
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from litearm_ros2_control import stm32_proto as proto  # noqa: E402
from litearm_ros2_control.stm32_link import Stm32Link  # noqa: E402


def snap(link, label):
    # ⚠ 必须先 poll：link.status 只在 poll() 里更新，直接读会拿到陈旧帧。
    # 这个 bug 真踩过 —— DISABLE 之后读到的是加磁中的旧帧，看起来像"失能失败"。
    link.poll()
    st = link.status
    flags = ",".join(st.flag_names()) or "-"
    print(f"  {label:12s} flags=0x{st.flags:04X} [{flags}] enabled={st.enabled} "
          f"mode={st.mode_name} err={[j.err for j in st.joints]}")
    print(f"               q={[round(j.q, 4) for j in st.joints]}")


def watch(link, seconds, label):
    print(f"\n=== {label}（{seconds:g}s，不发任何命令）===")
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
        print("=== 基线 ===")
        snap(link, "baseline")

        print("\n=== ENABLE #1（固件预期回 0x03：首次写 CMODE 寄存器）===")
        ok, reason = link.command(proto.CMD_ENABLE, timeout_s=3.0)
        print(f"  应答 ok={ok} reason={reason}")
        watch(link, 4.0, "ENABLE#1 之后")

        if not link.status.enabled:
            print("\n=== 仍未加磁 → ENABLE #2（预期 ACK）===")
            ok, reason = link.command(proto.CMD_ENABLE, timeout_s=3.0)
            print(f"  应答 ok={ok} reason={reason}")
            watch(link, 4.0, "ENABLE#2 之后")

        print("\n=== 结论 ===")
        snap(link, "final")

        print("\n=== 收尾：DISABLE（回到我们发现它时的状态）===")
        link.disable()
        time.sleep(0.6)
        snap(link, "after")
        if link.status.enabled:
            print("  ⚠ DISABLE 后 enabled 仍为真 —— 臂还带电，请人工确认！")
        else:
            print("  ✔ 已确认失能")
    finally:
        link.close()


if __name__ == "__main__":
    main()
