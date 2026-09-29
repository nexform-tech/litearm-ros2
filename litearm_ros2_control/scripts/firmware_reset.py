#!/usr/bin/env python3
"""固件故障 / 急停锁存的**查看与解除**（人工入口）。

为什么需要它
————————————————————————————————————————————————————————————
``ENABLE`` 被固件拒 ``ERR{0x10,0x06}``（急停或关节故障锁存）时，``hw_daemon``
**只会**"关串口 → 等 2s → 重连重发 ENABLE"，永远解不开 —— 因为这是**锁存态**，
必须显式 ``RESET(0x14)``（清 EMERGENCY）/ ``CLEAR_FAULTS(0x13)``（清电机侧错码）。
此前这两个命令只有 ``Stm32Link`` 有封装、没有任何脚本暴露，一锁就得翻代码手搓
（真机 2026-09-21 就这么卡过一整轮：插件 10s 判"守护进程未就绪" → FATAL →
configure 失败 → ``ros2_control_node`` 抛异常 → 整个 launch 陪葬）。

锁存的来源**不止人为急停** —— 固件自己的主循环心跳监督（使能中 main 500ms
无心跳）就会调 ``ctrl_emergency_stop()``，现象与"有人按了急停"完全一样：
``mode=EMERGENCY`` + ``FAULT`` + 全轴被标。所以先读状态，别猜。

本脚本只做三件事：**读状态 → 明确解除 → 复查前后对照**。
它**不发任何运动命令、不使能电机**；``enabled=True``（有别的东西在控制它）时
反而拒绝动作 —— 使能是守护进程的职责，这里只管把锁存打开。

用法
————————————————————————————————————————————————————————————
::

    scripts/firmware_reset.py                          # 只读：状态 + 建议（零副作用）
    scripts/firmware_reset.py --reset                  # 复位状态机（解 EMERGENCY 锁存）
    scripts/firmware_reset.py --reset --clear-faults   # 复位 + 清故障位（常见组合）
    scripts/firmware_reset.py --fake                   # 无硬件演练：假固件 + 注入一次急停

退出码：0 = 目标状态已达成 / 1 = 动作后仍不干净（附复查与建议）/
2 = 用法错 / 3 = 打不开设备或被安全闸门拦住。

动作完成后按提示重启栈即可：
``ros2 launch litearm_manipulation manipulation.launch.xml dry_run:=false start_rviz:=true``
"""

import argparse
import sys
import time
from pathlib import Path

# 源码树直接跑时保证能 import 到本包（脚本也可能被装到 lib/ 下运行，此时走 ament 索引）。
_SOURCE_PYTHON = Path(__file__).resolve().parents[1] / "python"
if _SOURCE_PYTHON.is_dir() and str(_SOURCE_PYTHON) not in sys.path:
    sys.path.insert(0, str(_SOURCE_PYTHON))

from litearm_ros2_control import fake_firmware, stm32_proto as proto  # noqa: E402
from litearm_ros2_control.stm32_link import Stm32Link, find_port  # noqa: E402

# 命令之后给固件把新状态刷进状态帧的时间（状态帧 100Hz，0.6s 足够覆盖
# "命令处理 + 失能去抖 + 状态机切换"这三步）。
SETTLE_S = 0.6
# 等一帧**新**状态帧的上限。
STATUS_TIMEOUT_S = 2.0


def snapshot(link: Stm32Link):
    """取一份**新**状态帧。

    ``link.status`` 是上一次 ``poll()`` 那一刻的快照，直接读会拿到陈旧帧
    （本项目已为此付过一次代价：失能后仍打印 enabled=True）。所以先 poll，
    并且等 seq 真的变了才算新帧。
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
             f"flags={flags if flags else ['(无)']}"]
    if status.joint_fault is not None:
        lines.append(f"  joint_fault=0x{status.joint_fault:02X}"
                     f"（{bin(status.joint_fault).count('1')} 轴被标）")
    lines.append(f"  q  = [{', '.join(f'{j.q:+.4f}' for j in status.joints)}]")
    lines.append(f"  err= {[j.err for j in status.joints]} (0=失能 1=使能)")
    return "\n".join(lines)


def is_locked(status) -> bool:
    """还处于"ENABLE 会被拒 0x06"的状态吗。"""
    return (status.mode == proto.ARM_MODE_EMERGENCY or status.fault
            or bool(status.joint_fault))


def advise(status) -> str:
    if status.mode == proto.ARM_MODE_EMERGENCY or status.fault:
        return ("建议：--reset（0x14，先解 EMERGENCY 锁存）；复查若仍有 FAULT "
                "再加 --clear-faults")
    if status.joint_fault:
        return "建议：--clear-faults（0x13，mode 正常但有轴被标故障）"
    return "状态干净，无需动作 —— 可直接起栈"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="查看/解除 litearm-stm32 的故障与急停锁存（不发运动命令、不使能电机）")
    parser.add_argument("--port", default="",
                        help="USB CDC 设备路径；留空则按 VID:PID 自动发现")
    parser.add_argument("--reset", action="store_true",
                        help="发 RESET(0x14)：复位状态机，解 EMERGENCY 锁存")
    parser.add_argument("--clear-faults", action="store_true",
                        help="发 CLEAR_FAULTS(0x13)：清电机/关节侧故障码")
    parser.add_argument("--fake", action="store_true",
                        help="起一块假固件并注入一次急停，演练整个流程（无硬件）")
    args = parser.parse_args(argv)

    firmware = None
    if args.fake:
        firmware = fake_firmware.FakeFirmware()
        port = firmware.start()
        print(f"假固件已起：{port}")
    else:
        port = args.port or find_port() or ""
        if not port:
            print("未找到 litearm-stm32（VID:PID 1d50:606f）。用 --port 指定，"
                  "或 --fake 跑无硬件演练。", file=sys.stderr)
            return 3

    link = Stm32Link(port)
    try:
        link.open()
        print(f"已打开 {port}")

        if args.fake:
            # 演练：先注入急停，复现真机那种"ENABLE 回 0x06"的锁存态
            link.emergency_stop()
            time.sleep(SETTLE_S)
            print("（演练）已注入一次急停 —— 此时 ENABLE 会被拒 0x06")

        before = snapshot(link)
        print("\n动作前状态：")
        if before is None:
            print("  没有收到状态帧（固件在跑吗？）", file=sys.stderr)
            return 3
        print(describe(before))

        if before.enabled:
            print("\n拒绝动作：状态帧显示 enabled=True —— 有别的东西正在控制它"
                  "（守护进程或别的工具）。先停掉它再清故障，"
                  "否则清了也会被下一帧命令覆盖。", file=sys.stderr)
            return 3

        if not (args.reset or args.clear_faults):
            print(f"\n{advise(before)}")
            return 0

        if args.reset:
            link.reset()
            print("\n已发 RESET(0x14)")
        if args.clear_faults:
            link.clear_faults()
            print("已发 CLEAR_FAULTS(0x13)")
        time.sleep(SETTLE_S)

        after = snapshot(link)
        print("\n动作后状态：")
        if after is None:
            print("  没有收到状态帧", file=sys.stderr)
            return 1
        print(describe(after))

        if is_locked(after):
            print(f"\n仍未解除：{advise(after)}")
            return 1
        print("\n已解除锁存/故障 —— 现在可以起栈了")
        return 0
    finally:
        link.close()
        if firmware is not None:
            firmware.stop()


if __name__ == "__main__":
    sys.exit(main())
