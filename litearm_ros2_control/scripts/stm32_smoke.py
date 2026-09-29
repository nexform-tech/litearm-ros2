#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""stm32_smoke.py — litearm-stm32 链路的冒烟验收（无硬件可全跑）。

跑的顺序就是真机上第一件该做的事：

    打开端口 → 读固件版本 → 读 7 关节参数（0x24）→ 读 ff_mask（0x2C item9）
    → ENABLE → 等 enabled 位 → [可选] 单轴小幅 MOVE_JS → 停发看门狗持位
    → PARK → DISABLE → 打印链路计数

四种用法::

    # 完全只读：**一个会改变固件状态的字都不发**（不 ENABLE、不动、不 park）
    # 板子插上就能跑，臂完全不会动。查"板子在不在/固件哪版/参数对不对"用它。
    scripts/stm32_smoke.py --read-only

    # 无硬件：自己起一块假固件，全流程跑通（含运动与看门狗）
    scripts/stm32_smoke.py --fake

    # 真机：会 ENABLE（电机加磁）但**不动**（只到 enable/状态/park/disable）
    scripts/stm32_smoke.py
    scripts/stm32_smoke.py --move   # 再显式允许动 0.03 rad（确认臂已被支撑！）

    # 对着别处起的假固件（tools/bench_fake_fw.py 之类）
    scripts/stm32_smoke.py --port /dev/pts/7

真机上**先看 license**：固件未激活时 ``ENABLE`` 回 ``ERR{0x10,0x08}``，
本脚本会直接报出来并给指引，其余命令不受影响。
"""

import argparse
import sys
import time
from pathlib import Path

# 源码树直接跑时保证能 import 到本包。
_SOURCE_PYTHON = Path(__file__).resolve().parents[1] / "python"
if _SOURCE_PYTHON.is_dir() and str(_SOURCE_PYTHON) not in sys.path:
    sys.path.insert(0, str(_SOURCE_PYTHON))

from litearm_ros2_control import fake_firmware, stm32_proto as proto  # noqa: E402
from litearm_ros2_control.stm32_link import (ERR_HINTS,  # noqa: E402
                                             Stm32Error, Stm32Link,
                                             find_port)

NUM_JOINTS = 7

# 真机上允许的最大运动幅度（rad）。冒烟不是标定，0.03 rad 已经足够看出
# 数据通路是否贯通，同时即使撞上什么也不至于出大事。
MAX_AMPLITUDE_RAD = 0.2
DEFAULT_AMPLITUDE_RAD = 0.03
# 真机首次运动的速率上限。慢是安全的第一道防线：0.05 rad/s 时 0.03 rad 要走 0.6s，
# 任何异常都有足够时间被看见/被急停打断。
DEFAULT_SPEED_RAD_S = 0.05
MAX_SPEED_RAD_S = 2.0


class Report:
    """收集检查项，最后一次性给出结论（返回码即结论）。"""

    def __init__(self) -> None:
        self.failures = []

    def check(self, title: str, ok: bool, detail: str = "") -> bool:
        mark = "✔" if ok else "✘"
        line = f"  {mark} {title}"
        if detail:
            line += f"：{detail}"
        print(line, flush=True)
        if not ok:
            self.failures.append(title)
        return ok

    def done(self) -> int:
        print()
        if self.failures:
            print(f"失败 {len(self.failures)} 项：{'，'.join(self.failures)}")
            return 1
        print("全部通过")
        return 0


def _spin(link: Stm32Link, seconds: float, period_s: float = 0.005) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        link.poll()
        time.sleep(period_s)


def _firmware_path() -> Path:
    """固件源码路径（可选）——用于把读回的参数与 defaults.c 对照。"""
    override = Path.home() / "wkspace" / "litearm-stm32"
    return override


def _measure_report_rate(link: Stm32Link, seconds: float) -> tuple:
    """被动测状态帧节拍：返回 (测得 Hz, 收到帧数, 唯一 seq 数)。

    只 poll 不发任何命令——固件本来就以 100Hz 主动上报。
    用 **seq 变化**计数而不是"poll 到几次"：同一帧可能在缓冲里被读到多次，
    按帧计数会把节拍算高。
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
    """打印固件参数与前馈开关（两条路径共用）。返回是否齐备。"""
    print("\n[2] 关节参数（全部从固件读，0x24）")
    params = []
    for index in range(len(status.joints)):
        param = link.get_joint_param(index)
        if param is None:
            report.check(f"关节 {index + 1} 参数读回", False, "超时")
            return False
        params.append(param)
    for param in params:
        print(f"      j{param.index + 1}: kp={param.kp:g} kd={param.kd:g} "
              f"tau_max={param.tau_max:g} "
              f"q∈[{param.q_min:.6f}, {param.q_max:.6f}]")
    if not report.check("7 关节参数齐备", len(params) == NUM_JOINTS):
        return False

    # 与固件出厂表对照（只在 litearm-stm32 源码在场时做——不是硬依赖）。
    defaults_c = firmware_root / "User" / "litearm" / "params" / "defaults.c"
    if defaults_c.exists():
        expected = {
            "kp": fake_firmware.DEFAULT_KP[:len(params)],
            "kd": fake_firmware.DEFAULT_KD[:len(params)],
            "tau_max": fake_firmware.DEFAULT_TAU_MAX[:len(params)],
        }
        for field, values in expected.items():
            actual = [getattr(item, field) for item in params]
            report.check(f"{field} 与固件出厂表一致（defaults.c）",
                         all(abs(a - e) < 1e-6
                             for a, e in zip(actual, values)),
                         f"{actual}")
    else:
        print(f"      （跳过出厂表对照：{defaults_c} 不在）")

    print("\n[3] 内置前馈开关（0x2C item9）")
    mask = link.get_ff_mask(timeout_s=1.0)
    if not report.check("读 ff_mask", mask is not None,
                        proto.format_ff_mask(mask)
                        if mask is not None else "超时"):
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
    report.check("FF_MASTER 已置位（内置前馈总开关）",
                 bool(mask & proto.FF_MASTER),
                 "MASTER 关掉时位置模式也不会叠任何前馈")
    return True


def run_read_only(link: Stm32Link, report: Report, *,
                  measure_s: float, firmware_root: Path) -> None:
    """**零副作用**预检：不 ENABLE、不动、不 park。

    与 :func:`run` 的区别不是"少做几步"，而是**一个会改变固件状态的字都不发**：
    ``run()`` 即使在不动电机的模式下也会 ENABLE（电机加磁），那已经是有物理后果
    的操作了。所以查"板子在不在、固件哪版、参数读得对不对"用这条路径。
    """
    print("\n[1] 固件身份")
    version = link.get_firmware(timeout_s=1.0)
    report.check("读固件版本 (0x41)", version is not None, version or "超时")
    if version is None:
        report.check("链路可用（收到状态帧）", False,
                     "收不到任何应答——端口对不对？板子跑起来了？")
        return

    status = link.wait_status(2.0)
    if not report.check("收到状态帧 (0x40)", status is not None):
        return
    print(f"      关节数 {len(status.joints)} · mode={status.mode_name} "
          f"· enabled={status.enabled} · flags={status.flag_names() or '无'}")
    if not report.check("电机未使能（只读预检的前提）", not status.enabled,
                        "已使能说明有别的进程在控制它，先停掉再查"):
        return
    print(f"      q   = {[round(j.q, 4) for j in status.joints]}")
    print(f"      dq  = {[round(j.dq, 4) for j in status.joints]}")
    print(f"      err = {[j.err for j in status.joints]} (0=失能 1=使能)")
    print(f"      温度 = {[(round(j.t_mos, 1), round(j.t_coil, 1)) for j in status.joints]}")
    print(f"      joint_fault = {status.joint_fault}")
    # ⚠ 反向陷阱：**"全零"不是姿态，是"还没读到反馈"**。
    # 固件在未使能时每 3 拍发零力矩探针帧，电机"收帧才回状态"——所以反馈有
    # 建立过程。此刻 q/dq/temps 全停在初值 0，看起来像一台七轴恰好全在零位的臂。
    # 写这条是因为**真踩过**：第一次只读预检就是这么把全零当姿态打出来的，
    # 而且"无故障位"照样判了通过。真实臂不可能七轴恰好 0.0000、温度恰好 0.0。
    all_zero = (all(j.q == 0.0 for j in status.joints)
                and all(j.dq == 0.0 for j in status.joints)
                and all(j.t_mos == 0.0 and j.t_coil == 0.0
                        for j in status.joints))
    report.check("电机已回报反馈（这些 q 不是初值零）", not all_zero,
                 "" if not all_zero else
                 "q/dq/温度**全为 0** = 还没收到任何电机反馈，不是真实姿态。"
                 "检查电机供电与板子↔臂的 CAN 线，稍候重试")
    if all_zero:
        return
    report.check("无故障位", not status.fault and not status.faulted_joints(),
                 "有故障位时先查因，别急着使能")

    if not _report_params_and_ff(link, report, status, firmware_root):
        return

    print(f"\n[4] 状态帧节拍（被动测 {measure_s:g}s，不发任何命令）")
    hz, frames, unique = _measure_report_rate(link, measure_s)
    report.check("上报节拍接近 100Hz", 70.0 <= hz <= 130.0,
                 f"实测 {hz:.1f} Hz（{unique} 个 seq / {measure_s:g}s，"
                 f"共 poll 到 {frames} 帧）")
    report.check("无 CRC 错误", link.crc_errors == 0,
                 f"crc_err={link.crc_errors}，帧外丢弃 {link._framer.discarded}B")

    print(f"\n[5] 链路计数\n      {link.counters()}")


def run(link: Stm32Link, report: Report, *, allow_motion: bool,
        amplitude: float, joint: int, move_speed: float,
        firmware_root: Path, on_fake: bool) -> None:
    print("\n[1] 固件身份")
    version = link.get_firmware(timeout_s=1.0)
    report.check("读固件版本 (0x41)", version is not None, version or "超时")
    if version is None:
        report.check("链路可用（收到状态帧）", False,
                     "收不到任何应答——端口对不对？板子跑起来了？")
        return

    status = link.wait_status(2.0)
    if not report.check("收到状态帧 (0x40)", status is not None):
        return
    print(f"      关节数 {len(status.joints)} · mode={status.mode_name} "
          f"· enabled={status.enabled} · flags={status.flag_names() or '无'}")
    report.check("上电即安全态（电机未使能）", not status.enabled,
                 "若上电就已使能，说明固件不是本仓的启动流程")

    if not _report_params_and_ff(link, report, status, firmware_root):
        return

    print("\n[4] 使能（0x10）")
    # 固件的 ENABLE 是**两段**的：启动后第一次要先写 7 台电机的 CMODE 寄存器，
    # 那一刻回 0x03 并登记待使能；反馈齐了由 enable_pending_poll 自动加磁，
    # 主机重发 ENABLE 才拿到 ACK。真机实测时序见 control_loop.c 的 ctrl_enable：
    #   "ENABLE#1 -> 0x03(首写 CMODE), ENABLE#2 -> ACK"
    # 所以 0x03 **不是失败**，必须原地重发（不是关串口重连）。
    deadline = time.monotonic() + 8.0
    told_first_write = False
    enabled = None
    while time.monotonic() < deadline:
        ok, reason = link.command(proto.CMD_ENABLE, timeout_s=1.0)
        if ok is False and reason == 0x08:
            report.check("ENABLE 被 license 门禁拒绝", False,
                         "固件未激活。用 tools/litearm-license 激活后重试；"
                         "在此之前所有运动命令都会被拒")
            return
        if ok is False and reason == 0x06:
            report.check("ENABLE 被拒（急停/关节故障锁存）", False,
                         "须先清故障或复位（0x13 / 0x14）")
            return
        if ok is False and reason == 0x03:
            if not told_first_write:
                told_first_write = True
                print("      ENABLE 回 0x03（固件首次写电机 CMODE / 反馈未就绪）"
                      "——按固件时序原地重发")
        elif ok is False:
            report.check("ENABLE 被接受", False,
                         f"原因码 {reason}（{ERR_HINTS.get(reason, '见 usb_cmd.h')}）")
            return
        if link.wait_enabled(1.0) is not None:
            enabled = link.status
            break
    report.check("ENABLE 被接受并真正加磁", enabled is not None,
                 "ACK 只表示登记；加磁要等反馈齐（wait_enabled）")
    if enabled is None:
        return
    print(f"      加磁后 mode={link.status.mode_name} "
          f"err={[j.err for j in link.status.joints]}")

    print("\n[5] 运动（MOVE_JS，不带 tau_ff → 固件叠内置前馈）")
    if not allow_motion:
        print("  - 跳过（真机上需要 --move 才动电机；--fake 会自动允许）")
    else:
        start_q = [j.q for j in link.status.joints]
        target = list(start_q)
        target[joint] += amplitude
        # ⚠ **必须给非零速度**：固件 MOVE_JS 的 dq_ref 同时是位置参考的 slew 上限
        #   （control_loop.c: v_lim = clamp(|dq_ref|, 0, speed_limit);
        #    cmd->q_ref = slew_linear(target, q_ref, v_lim*dt)），
        #   所以"只给位置、速度给 0"参考会冻结、臂一步都不动。
        #   这里朝目标方向给一个很小的速度 —— 低速单轴第一步要的就是"慢"。
        speed = min(move_speed, max(1e-3, amplitude / 0.5))
        dq = [0.0] * NUM_JOINTS
        print(f"      目标 j{joint + 1}: {start_q[joint]:+.4f} → {target[joint]:+.4f} rad"
              f"（速率 {speed:.3f} rad/s，其余轴保持）")
        deadline = time.monotonic() + max(1.2, 2.0 * amplitude / speed + 0.5)
        while time.monotonic() < deadline:
            link.poll()
            # 速度指令方向按**实测**与目标的差重算（同真实 JTC 的做法），
            # 到位后自动归零、参考随之停住。
            for i, jnt in enumerate(link.status.joints):
                error = target[i] - jnt.q
                dq[i] = max(-speed, min(speed, error / max(0.004, 1e-3)))
            link.move_js(target, dq)
            time.sleep(0.004)
        link.poll()
        moved = link.status.joints[joint].q
        report.check(f"关节 {joint + 1} 跟到目标", abs(moved - target[joint]) < 0.02,
                     f"{start_q[joint]:.4f} → {moved:.4f}（目标 {target[joint]:.4f}）")
        report.check("模式切到 MOVE_JS",
                     firmware_mode_ok(link, proto.ARM_MODE_MOVE_JS),
                     link.status.mode_name)
        moved_all = [j.q for j in link.status.joints]
        drift = [moved_all[i] - start_q[i] for i in range(NUM_JOINTS)]
        print(f"      逐轴位移(rad) = {[round(v, 4) for v in drift]}")
        # ⚠ 已知例外：**处于"受控回程"中的轴会继续动**。
        # 承重关节失力后停在下垂平衡点，那里可能落在软限位外；使能时固件把参考
        # 钳到限位内并把它带回来，整个回程约 3s（`ctrl_axis_returning` 的宽限期），
        # 而 `wait_enabled` 在 enabled 位置位（~0.3s）就返回了 —— 所以运动段期间
        # 该轴仍在往回走。这是设计内行为，不是"别的轴乱动"。
        others = [(i, drift[i]) for i in range(NUM_JOINTS)
                  if i != joint and abs(drift[i]) > 0.02]
        if not others:
            report.check("其余轴基本没动", True)
        else:
            print(f"      其余动了的轴：{[f'j{i+1} {v:+.4f}' for i, v in others]}"
                  f"（若为受控回程则属预期；回程发生在使能后的前 ~3s）")
            worst_other = max(abs(v) for _i, v in others)
            report.check("其余轴位移有界（回程或滞后，不是乱动）", worst_other < 0.25,
                         f"最大 {worst_other:.4f} rad")

    print("\n[6] 停发 → 固件看门狗持位（100ms）")
    held_at = [j.q for j in link.status.joints]
    _spin(link, 0.4)
    report.check("WD_TRIPPED 已置位", link.status.watchdog_tripped is True,
                 "停发 0.4s 仍未触发——看门狗没在工作？")
    drift = [link.status.joints[i].q - held_at[i] for i in range(len(held_at))]
    worst = max(abs(v) for v in drift)
    if on_fake:
        # 假固件把持位实现成"冻结 q_ref"，所以位置一条都不该动。
        report.check("持位期间位置不漂移", worst < 0.02,
                     f"最大漂移 {worst:.5f} rad")
    else:
        # ⚠ 真机上**必然下沉**，这不是故障：固件 fail-soft 持位是 tau=0 且刚度
        # 只有 0.6×（`hold_kp_scale`），**不叠重力前馈** —— 承重关节会一直沉到
        # kp·0.6·Δq = G 的平衡点（文档 §4.4 实测 J2 8~13.5°、J4 −9°）。
        # 所以这里不断言"不漂移"（那会让真机首跑报一个吓人但错误的失败），
        # 而是断言"下沉是有界的、不是坠落"，并把逐轴下沉量打出来当数据看。
        report.check("下沉有界（不是坠落）", worst < 0.6,
                     f"最大下沉 {worst:.4f} rad = {worst * 57.2958:.2f}°"
                     f"（预期行为：fail-soft 无重力前馈）")
        print(f"      逐轴下沉(rad) = "
              f"{[round(v, 4) for v in drift]}")

    # 命令恢复 → kick → 标志自动清除。
    link.move_js([j.q for j in link.status.joints], [0.0] * NUM_JOINTS)
    _spin(link, 0.1)
    report.check("命令恢复后 WD_TRIPPED 自动清除",
                 link.status.watchdog_tripped is False)

    print("\n[7] PARK 声明（0x20 mode=0）")
    link.park()
    _spin(link, 0.1)
    report.check("PARK 后看门狗以 1.0× 刚度持位",
                 link.status.watchdog_tripped is True,
                 "PARK 是「PC 断开前的高刚度声明」，靠看门狗触发才生效")

    print("\n[8] 失能（0x11）")
    link.disable()
    _spin(link, 0.2)
    report.check("电机已失能", link.status.enabled is False,
                 "臂会失力下坠——真机上请先支撑")
    report.check("模式回 INIT",
                 link.status.mode == proto.ARM_MODE_INIT,
                 link.status.mode_name)

    print("\n[9] 链路计数")
    print(f"      {link.counters()}")
    report.check("无 CRC 错误", link.crc_errors == 0)


def firmware_mode_ok(link: Stm32Link, expected: int) -> bool:
    return link.status is not None and link.status.mode == expected


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="litearm-stm32 链路冒烟验收（--fake 可无硬件全跑）")
    parser.add_argument("--port", default="",
                        help="USB CDC 设备路径；留空则按 VID:PID 自动发现")
    parser.add_argument("--read-only", action="store_true",
                        help="零副作用预检：不 ENABLE、不动、不 park "
                             "（板子插上就能跑，臂完全不会动）")
    parser.add_argument("--fake", action="store_true",
                        help="起一块假固件并对着它跑（无硬件，允许运动）")
    parser.add_argument("--move", action="store_true",
                        help="真机上显式允许动电机（默认只到 enable/状态/park/disable）")
    parser.add_argument("--amplitude", type=float, default=DEFAULT_AMPLITUDE_RAD,
                        help=f"运动幅度 rad（上限 {MAX_AMPLITUDE_RAD}）")
    parser.add_argument("--joint", type=int, default=0,
                        help="动哪一轴（0 基，默认 0）")
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED_RAD_S,
                        help=f"运动速率 rad/s（上限 {MAX_SPEED_RAD_S}）；"
                             f"慢比快安全，首次真机建议 ≤0.05")
    parser.add_argument("--measure-s", type=float, default=2.0,
                        help="只读模式下测状态帧节拍的时长（秒）")
    args = parser.parse_args(argv)

    if args.read_only and args.fake:
        print("--read-only 与 --fake 互斥（假固件本来就是零风险路径）",
              file=sys.stderr)
        return 2
    if args.read_only and args.move:
        print("--read-only 与 --move 是反义，别同时给", file=sys.stderr)
        return 2
    if not (0.0 < args.amplitude <= MAX_AMPLITUDE_RAD):
        print(f"--amplitude 必须在 (0, {MAX_AMPLITUDE_RAD}] 内", file=sys.stderr)
        return 2
    if not (0 <= args.joint < NUM_JOINTS):
        print(f"--joint 必须在 [0, {NUM_JOINTS - 1}] 内", file=sys.stderr)
        return 2
    if args.measure_s <= 0.0:
        print("--measure-s 必须为正", file=sys.stderr)
        return 2
    if not (0.0 < args.speed <= MAX_SPEED_RAD_S):
        print(f"--speed 必须在 (0, {MAX_SPEED_RAD_S}] 内", file=sys.stderr)
        return 2

    firmware = None
    port = args.port
    allow_motion = args.move
    if args.fake:
        firmware = fake_firmware.FakeFirmware()
        port = firmware.start()
        allow_motion = True
        print(f"假固件已起：{port}（一阶运动学模型，不能用来整定增益）")
    elif not port:
        port = find_port() or ""
        if not port:
            print("未找到 litearm-stm32（VID:PID 1d50:606f）。"
                  "用 --port 指定，或 --fake 跑无硬件演练。", file=sys.stderr)
            return 3

    link = Stm32Link(port)
    report = Report()
    try:
        link.open()
        print(f"已打开 {port}")
        if args.read_only:
            print("模式：只读（不发 ENABLE / 不发运动 / 不发 park）")
            run_read_only(link, report, measure_s=args.measure_s,
                          firmware_root=_firmware_path())
        else:
            run(link, report, allow_motion=allow_motion,
                amplitude=args.amplitude, joint=args.joint,
                move_speed=args.speed,
                firmware_root=_firmware_path(), on_fake=args.fake)
    except (Stm32Error, OSError) as exc:
        # Stm32AccessDenied 的 str() 自带恢复路径（dialout 加组 + 重登录）。
        print(f"链路错误：{exc}", file=sys.stderr)
        return 3
    finally:
        link.close()
        if firmware is not None:
            firmware.stop()
    return report.done()


if __name__ == "__main__":
    sys.exit(main())
