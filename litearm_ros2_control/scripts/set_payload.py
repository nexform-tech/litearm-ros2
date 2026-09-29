#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""set_payload.py — 把末端负载（质量/质心）写进固件的重力前馈参数。

为什么需要它
------------
固件 `ff_mask` 的 `FF_G` 出厂常开（defaults.c 整臂档 = 0x1BF），但
`payload_mass / payload_com` 出厂是 **0.0 / {0,0,0}**。而动力学模型只到
`ee_link`（`litearm_id.urdf`，ee_link 惯量清零、质量并进 link7）⇒ 末端挂的
夹爪/工具重量**不在模型里**，不设 payload 时 `G(q)` 少补这一项。

三个默认值怎么来的
------------------
按 `litearm_manipulation/urdf/litearm_gripper.urdf.xacro` 的三个惯量块
（gripper_base_link + 两个指节）合成，再经 `gripper_mount_joint` 的
`xyz=(0,0,0.0641) rpy=(0,0,π/2)` 折算到 **ee_link 系**：

    mass = 0.506299067530095 + 2 × 0.035453736732405 = 0.577206541 kg
    com  = [-0.000032, -0.000069, +0.055239] m（ee_link 系）

x/y 两项 ≈ 0.03~0.07 mm（装配公差以下，可忽略），z = +55.24 mm 是主项
（夹爪整体挂在前方接近轴上）。**与开口无关**：两指用同一 mimic 反号同步
（multiplier=-0.5），指组合质心不随开合移动 —— 实测 0/43.5/87 mm 三档逐位相同。

⚠ 注意 ee_link 的关节原点在固件真源 `litearm_id.urdf` 与 ROS 侧 `litearm.urdf`
  里**逐字节相同**（`xyz=-0.05374985299 6.976277736e-07 1.773739393e-06`），
  所以这里算出的 com 可以直接写进固件，不需要再换系。

性质 / 边界
-----------
* **只写 RAM**（`0x28`），掉电丢。要跨上电保留需另发 `0x25` 存 flash，而固件
  要求**先失能** ⇒ 那一步会让臂失力下垂，必须先把臂支撑好（本脚本不做）。
* `0x28` 在固件里**没有使能门控**（只校验长度 → `params_ff_scalar()`），但
  **改的是重力前馈力矩**：FF_G 生效时臂的持位力矩随之变化，下一次起栈/进入
  MOVE_JS 跟踪时会看到一次小幅沉降变化。写入本身不发任何运动帧。
* **同一根 CDC 线复用**（状态帧 100 Hz + 命令应答）⇒ 起栈时不要跑本脚本：
  两个 reader 会互相吃掉对方的字节。脚本会先检查并拒绝。

用法::

    # 只读预览（默认）：读回 firmware/payload/com/ff_mask，一个写命令都不发
    scripts/set_payload.py

    # 真正写入（改固件状态）
    scripts/set_payload.py --apply

    # 称过真夹爪后自定义
    scripts/set_payload.py --mass 0.61 --com 0 0 0.0552 --apply
"""

import argparse
import os
import sys
from pathlib import Path

_SOURCE_PYTHON = Path(__file__).resolve().parents[1] / "python"
if _SOURCE_PYTHON.is_dir() and str(_SOURCE_PYTHON) not in sys.path:
    sys.path.insert(0, str(_SOURCE_PYTHON))

from litearm_ros2_control import stm32_proto as proto  # noqa: E402
from litearm_ros2_control.stm32_link import (  # noqa: E402
    Stm32Error, Stm32Link, find_port)

# LiteGrip 夹爪的合质量/合质心（见模块 docstring 的推导与出处）。
DEFAULT_MASS_KG = 0.577206541
DEFAULT_COM_M = (-0.000032, -0.000069, 0.055239)

# float32 往返的容差：固件按 IEEE754 单精度存，0.577 量级的 eps ≈ 6e-8。
TOL = 1e-5

# 会独占同一根 CDC 线的进程（出现任一即拒绝写入）。
BUSY_PROC_NAMES = ("ros2_control_node", "litearm_hw_daemon")


def find_busy_processes() -> list:
    """扫 /proc 找正在占用这条 CDC 链路的进程（名字匹配）。"""
    hits = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            with open(f"/proc/{entry}/cmdline", "rb") as handle:
                cmdline = handle.read().decode("utf-8", "replace")
        except OSError:
            continue
        for name in BUSY_PROC_NAMES:
            if name in cmdline:
                hits.append((entry, cmdline.replace("\x00", " ").strip()))
                break
    return hits


def read_payload(link: Stm32Link) -> tuple:
    """读回 payload_mass 与 payload_com 三分量；缺项为 None。"""
    mass = link.get_ff_scalar(proto.FF_SCALAR_PAYLOAD_MASS)
    com = tuple(link.get_ff_scalar(proto.FF_SCALAR_PAYLOAD_COM, sub=i)
                for i in range(3))
    return mass, com


def show(tag: str, mass, com) -> None:
    mass_s = "超时" if mass is None else f"{mass:.9f} kg"
    com_s = " ".join("超时" if v is None else f"{v:+.6f}" for v in com)
    print(f"  {tag}  payload_mass = {mass_s}")
    print(f"  {tag}  payload_com  = [{com_s}] m (ee_link 系)")


def write_payload(link: Stm32Link, mass: float, com) -> bool:
    """写 mass + com 三分量，再读回逐项核对。返回是否一致。"""
    link.set_ff_scalar(proto.FF_SCALAR_PAYLOAD_MASS, mass)
    for index, value in enumerate(com):
        link.set_ff_scalar(proto.FF_SCALAR_PAYLOAD_COM, value, sub=index)
    print("  → 已发 0x28 ×4（item4 payload_mass / item5 payload_com ×3），等读回…")
    # 固件每帧都在上报状态，读回要穿过在途帧，给足超时并重试一次。
    for attempt in (1, 2):
        back_mass, back_com = read_payload(link)
        if back_mass is not None and all(v is not None for v in back_com):
            break
        print(f"    （读回超时，重试 {attempt}/2）")
    else:
        print("  ✗ 读回全部超时：链路或固件未响应")
        return False

    show("写后", back_mass, back_com)
    ok_mass = abs(back_mass - mass) <= TOL
    ok_com = all(abs(back_com[i] - com[i]) <= TOL for i in range(3))
    if ok_mass and ok_com:
        return True
    print("  ✗ 读回与写入不一致 —— 常见原因：0x04 须先失能 / 0x02 非法参数"
          " / 端口被别的 reader 抢字节")
    return False


def main() -> int:
    parser = argparse.ArgumentParser(
        description="把末端负载（质量/质心）写进固件重力前馈参数（0x28，RAM）")
    parser.add_argument("--mass", type=float, default=DEFAULT_MASS_KG,
                        help=f"负载质量 kg（默认 LiteGrip 夹爪 {DEFAULT_MASS_KG}）")
    parser.add_argument("--com", type=float, nargs=3, default=list(DEFAULT_COM_M),
                        metavar=("X", "Y", "Z"),
                        help="质心，ee_link 系，m（默认夹爪合质心）")
    parser.add_argument("--port", default=None, help="显式指定串口（默认自动扫描）")
    parser.add_argument("--apply", action="store_true",
                        help="真正写入；不给则只读预览（推荐先跑一次只读）")
    args = parser.parse_args()

    print("=== set_payload：末端负载 → 固件重力前馈参数 ===")
    print(f"  目标  payload_mass = {args.mass:.9f} kg")
    print(f"  目标  payload_com  = [{args.com[0]:+.6f} {args.com[1]:+.6f}"
          f" {args.com[2]:+.6f}] m (ee_link 系)")

    busy = find_busy_processes()
    if busy:
        print("\n✗ 有进程正在独占这根 CDC 线，拒绝操作（两个 reader 会互相吃字节）：")
        for pid, cmd in busy:
            print(f"    PID {pid}: {cmd[:110]}")
        print("  → 先停栈（停栈前确认臂已被支撑/落稳），再跑本脚本。")
        if args.apply:
            return 2

    port = args.port or find_port()
    if not port:
        print("\n✗ 未找到 litearm-stm32 的 USB CDC 设备；用 --port 显式指定。")
        return 2

    link = Stm32Link(port)
    try:
        link.open()
    except Stm32Error as exc:
        print(f"\n✗ 打不开 {port}：{exc}")
        return 2

    try:
        print(f"\n  端口 = {link.port}")
        firmware = link.get_firmware(timeout_s=1.0)
        print(f"  固件 = {firmware if firmware else '（读不到）'}")

        mask = link.get_ff_mask(timeout_s=1.0)
        if mask is None:
            print("  ff_mask = （读不到）⇒ FF_G 是否生效未知，本脚本先不继续")
            return 2
        ff_g = bool(mask & proto.FF_G)
        print(f"  ff_mask = 0x{mask:03X}（{proto.format_ff_mask(mask)}）"
              f"  ⇒ FF_G {'已开，payload 会参与 G(q)' if ff_g else '★关着，写 payload 也不会生效'}")
        if not ff_g:
            print("    ⚠ 要让它生效：起栈时别传 gravity_compensation:=false，"
                  "或显式 gravity_compensation:=true")

        status = link.wait_status(timeout_s=1.0)
        if status is None:
            print("  状态 = （1s 内没等到状态帧）")
        else:
            taus = " ".join(f"{j.tau:+.2f}" for j in status.joints)
            print(f"  状态 = mode={status.mode_name} enabled={status.enabled} "
                  f"fault={status.fault} wd_tripped={status.watchdog_tripped} "
                  f"joint_fault={status.joint_fault}")
            print(f"  逐关节 tau(N·m) = [{taus}]")
            print("    （这是「同姿态对比 tau」判据的基线；未使能/未跟踪时该读数无意义）")

        print("\n  ── 写前现状 ──")
        before_mass, before_com = read_payload(link)
        show("before", before_mass, before_com)

        if not args.apply:
            print("\n  只读预览结束（未发任何写命令）。要写入请加 --apply")
            print("  ⚠ 写入只进 RAM，掉电丢；跨上电保留需另发 0x25 存 flash"
                  "（固件要求先失能 ⇒ 务必先支撑臂）")
            return 0

        print("\n  ── 写入（改固件状态）──")
        ok = write_payload(link, args.mass, args.com)
        print()
        if ok:
            print("  ✓ 写入并通过读回核对")
            print("  → 下次起栈进入 MOVE_JS 跟踪时，这一项才开始改变持位力矩；")
            print("    预期看到 J2/J3 等轴的持位力矩减小（夹爪重量被补上了）。")
            print("  ⚠ 只在 RAM：现在断电会丢。要保留：支撑好臂 → 失能 → 0x25 存 flash")
            return 0
        print("  ✗ 写入未通过读回核对（详情见上）")
        return 1
    finally:
        link.close()


if __name__ == "__main__":
    sys.exit(main())
