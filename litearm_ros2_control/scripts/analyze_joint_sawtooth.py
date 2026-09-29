#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""analyze_joint_sawtooth.py — 判定 /joint_states 位置曲线的"锯齿"是哪一种成因。

两种候选，用数据区分（可以同时存在）
------------------------------------
A. **采样率失配**：数据只在**固件的上报节拍**上更新（100Hz，``usb_cmd.c`` 的
   ``RPT_STATUS_MS=10``），而采样/发布比它快（改频前是 250Hz）⇒ 中间那些采样点
   只能重复上一个值。**判据是相对量**：

     · **变化间隔 / 实测采样周期 > 1.4** ← 主判据（采样快于数据更新）
     · 旁证：游程**上限** —— A 的保持时长被上报周期卡住（250/100 ⇒ ≤3）；
       量化保持随速度变化、**没有上限**（真数据里见过 39）
     · 旁证：重复样本占比 ≈ 1 − 上报率/采样率（250Hz 时 = 60%）

   ⚠ 判据**不能**写成"间隔 ≈ 10ms"这种绝对形式：采样率本身就是 100Hz 时，
   10ms 正是采样周期，那样子把**已经修好**的数据误判成失配（踩过，已加回归用例）。
   ⚠ 重复占比**单独分不开 A 与 B**：慢速运动在任何采样率下都会出现重复值。
     所以它只作旁证，不参与判定。
B. **量化**：位置是 16bit 覆盖电机量程（±12.5 rad，DM6248P 是 ±12.566）⇒
   步长 **3.81e-4 rad ≈ 0.022°**。特征：**最小的非零增量**就落在该步长上
   （且更小的增量根本不存在）。

判据的取向：A 决定"**什么时候**变"，B 决定"**变多少**"。看**慢速**运动最容易分辨
—— 快速运动时机械惯量会把台阶抹平，两者都看不出来。

--selftest 在**合成数据**上验证本脚本能认出 A、能认出 B、也能认出"其实光滑"，
并含一条"改频后 100/100 不得误报 A"的回归。**先信这个，再信它给出的结论。**

用法
----
    python3 analyze_joint_sawtooth.py <bag 路径>
    python3 analyze_joint_sawtooth.py --selftest
"""

import argparse
import sys
from collections import Counter

import numpy as np

# 时间轴上的候选：固件上报 100Hz / ROS 侧 250Hz
FIRMWARE_REPORT_HZ = 100.0
ROS_SAMPLE_HZ = 250.0
# 位置量化步长：16bit 覆盖 ±12.5 rad（DM4310/DM4340）；DM6248P 是 ±12.566，差 0.5%
POSITION_STEP_RAD = 2 * 12.5 / 65536.0

# 判定阈值
INTERVAL_TOL = 0.25          # 间隔与其名义值差 25% 以内算"对上了"
REPEAT_MIN = 0.35            # 重复样本占比超过它才算"有明显重复"
SMOOTH_REPEAT_MAX = 0.10     # 低于它算"基本没有重复"
STEP_FRACTION_MIN = 0.55     # 最小非零增量落在步长 ±20% 内算"量化解得上"


def _runs_and_changes(x):
    """返回 (变化点下标数组, 游程长度数组)。用**精确相等**判定重复 —— 同一帧被
    复制出来的值逐位相同，所以相等就是相等，不需要容差。"""
    if x.size < 2:
        return np.array([], dtype=int), np.array([], dtype=int)
    changed = np.flatnonzero(np.diff(x) != 0.0) + 1  # 每个"新值"的第一个样本下标
    starts = np.concatenate(([0], changed))
    runs = np.diff(np.concatenate((starts, [x.size])))
    return changed, runs


def analyze_series(t, x):
    """纯函数：给 (时间, 位置) 算出判据用到的全部统计量。"""
    t = np.asarray(t, dtype=float)
    x = np.asarray(x, dtype=float)
    order = np.argsort(t, kind="stable")
    t, x = t[order], x[order]

    n = x.size
    span = float(t[-1] - t[0]) if n > 1 else 0.0
    rate = (n - 1) / span if span > 0 else float("nan")

    changed, runs = _runs_and_changes(x)
    repeats = int(np.sum(runs - 1))            # 被重复的样本数
    repeat_frac = repeats / n if n else 0.0

    # 变化的间隔：用"新值出现时刻"的差
    dt_changes = np.diff(t[changed]) if changed.size > 1 else np.array([])
    dt_median = float(np.median(dt_changes)) if dt_changes.size else float("nan")
    dt_mode = Counter(np.round(dt_changes, 4)).most_common(1)[0][0] if dt_changes.size else float("nan")

    # 增量分布（只看非零）
    dx_all = np.diff(x)
    dx = np.abs(dx_all[dx_all != 0.0])
    dx_min = float(dx.min()) if dx.size else float("nan")
    dx_median = float(np.median(dx)) if dx.size else float("nan")
    # 非零增量是否落在量化为步长的整数倍上
    if dx.size:
        k = dx / POSITION_STEP_RAD
        on_grid = float(np.mean(np.abs(k - np.round(k)) < 0.2))
    else:
        on_grid = float("nan")

    max_run = int(runs.max()) if runs.size else 0
    return {
        "n": n, "span": span, "rate": rate,
        "changes": int(changed.size), "repeats": repeats, "repeat_frac": repeat_frac,
        "run_hist": Counter(runs.tolist()), "max_run": max_run,
        "dt_changes_median": dt_median, "dt_changes_mode": dt_mode,
        "dx_min": dx_min, "dx_median": dx_median, "dx_on_grid": on_grid,
    }


def verdict(s):
    """把统计量翻成人话。返回 (标签列表, 说明行列表)。"""
    tags, lines = [], []
    if not np.isfinite(s["rate"]) or s["n"] < 10:
        return ["样本不足"], ["样本太少，判不了"]
    # ★ 恒定通道（值一次都没变）要单独报，否则判据退化：全是"重复样本"会把
    #   一条**静止**的通道误判成 A 采样率失配（实测踩到：mocked 夹爪 1218×1）。
    if s["changes"] == 0:
        return ["恒定（无运动）"], [
            f"  整段值一次都没变（{s['n']} 个样本同一游程）——"
            f" 这不是锯齿，是这条通道根本没动；判据在这里不适用"]

    # ── A：采样率失配 ──
    # ★ 判据必须**相对于实测采样周期**，不能用绝对毫秒：
    #   采样降到 100Hz 之后，"变化间隔 = 10ms"就是采样周期本身，不是失配的证据。
    #   （我第一版写成绝对 ≈10ms，会在修好之后误报 —— 记录在此以免改回去。）
    sample_dt = 1.0 / s["rate"] if np.isfinite(s["rate"]) and s["rate"] > 0 else float("nan")
    report_dt = 1.0 / FIRMWARE_REPORT_HZ
    a_hit = False
    if np.isfinite(s["dt_changes_mode"]) and np.isfinite(sample_dt):
        ratio = s["dt_changes_mode"] / sample_dt
        lines.append(f"  变化间隔：众数 {s['dt_changes_mode'] * 1000:.2f} ms"
                     f" / 中位 {s['dt_changes_median'] * 1000:.2f} ms"
                     f"（采样周期 {sample_dt * 1000:.2f} ms ⇒ 比值 {ratio:.2f}）")
        if ratio > 1.4:
            a_hit = True
            tags.append("A 采样率失配")
            lines.append(f"    ⇒ 变化间隔是采样周期的 **{ratio:.1f}×**：采样快于数据更新，"
                         f"中间那些采样点只能重复上一个值 ⇒ A 成立")
        else:
            lines.append("    ⇒ 变化间隔≈采样周期 ⇒ 每个采样点都带新数据，无失配")
        if abs(s["dt_changes_mode"] - report_dt) <= INTERVAL_TOL * report_dt:
            lines.append(f"    （间隔同时对上固件上报率 {FIRMWARE_REPORT_HZ:.0f}Hz 的 "
                         f"{report_dt * 1000:.1f}ms —— 那就是「数据更新的节拍」）")
    # ⚠ 重复占比**单独分不开 A 与 B**：慢速运动在任何采样率下都会出现重复值（量化亦然）。
    #   能分开的只有上面那个"变化间隔 / 采样周期"的比值 —— A 的比值 >1，B 的 ≈1。
    #   所以这里只把占比连同上报率推算的理论值一起**报出来**，作为旁证，不参与判定。
    theoretical = max(0.0, 1.0 - FIRMWARE_REPORT_HZ / max(s["rate"], 1e-9))
    lines.append(f"  重复样本占比 {s['repeat_frac'] * 100:.1f}%"
                 f"（若采样率 {s['rate']:.0f}Hz 快于上报 {FIRMWARE_REPORT_HZ:.0f}Hz，"
                 f"理论值 = {theoretical * 100:.0f}%）")
    top_runs = ", ".join(f"{k}×{v}" for k, v in sorted(s["run_hist"].items())[:5])
    # ★ 游程**上限**是分开 A 与"量化保持"的关键旁证：A 的保持时长被上报周期卡住
    #   （250/100 ⇒ 最多 3），而量化保持随速度变化、**没有上限**（真数据里见过 39）。
    #   所以 max_run 很大时，那些长保持是"运动太慢"造成的，不是上报率造成的。
    lines.append(f"  游程长度分布（前几项）：{top_runs}；最长 {s['max_run']} —— "
                 f"A 成立时最长为 ceil(采样/上报)+1；若最长远大于它，长保持是"
                 f"**运动太慢**造成的，不是上报率")

    # ── B：量化 ──
    if np.isfinite(s["dx_min"]):
        lines.append(f"  最小非零增量 {s['dx_min']:.3e} rad"
                     f"（16bit/±12.5rad 的量化步长 = {POSITION_STEP_RAD:.3e}）")
        if abs(s["dx_min"] - POSITION_STEP_RAD) <= 0.2 * POSITION_STEP_RAD:
            tags.append("B 量化")
            lines.append("    ⇒ 最小增量就落在量化步长上 ⇒ B 成立")
        elif s["dx_min"] > 3 * POSITION_STEP_RAD:
            lines.append("    ⇒ 最小增量远大于量化步长 ⇒ 本次数据看不出量化"
                         "（动得太快，或没抓到慢速段）")
        if np.isfinite(s["dx_on_grid"]):
            lines.append(f"  非零增量落在量化网格上的比例 {s['dx_on_grid'] * 100:.0f}%")

    if not tags:
        tags = ["光滑"]
    return sorted(set(tags)), lines


# ────────────────────────── 合成数据自证 ──────────────────────────

def _synth_smooth(n=2500, hz=ROS_SAMPLE_HZ, jitter=2e-4, seed=0):
    rng = np.random.default_rng(seed)
    t = np.arange(n) / hz + rng.uniform(-jitter, jitter, n)
    t = np.sort(t)
    x = 0.3 * np.sin(2 * np.pi * 0.15 * t)          # 慢速平滑运动
    return t, x


def _synth_time_staircase(n=2500, seed=1):
    """模拟 A：真实位置以 100Hz 更新，被 250Hz 采样 ⇒ 重复 ~2.5 次。"""
    t, x_true = _synth_smooth(n, seed=seed)
    # 按 100Hz 的节拍对真值取样，再"保持"到下一次更新（零阶保持 = 真实链路行为）
    grid = np.floor(t * FIRMWARE_REPORT_HZ) / FIRMWARE_REPORT_HZ
    _, idx = np.unique(grid, return_index=True)
    x = x_true[np.clip(np.searchsorted(grid, grid), 0, None)]
    _ = idx
    return t, x


def _synth_aligned(n=1000, hz=100.0, seed=3):
    """模拟**改频之后**：上报与采样同为 100Hz ⇒ 每个采样点都带新数据。
    这条用来防"判据修好之后误报 A"——采样周期本身变成 10ms 时，
    绝对间隔判据会把正常数据判成失配。"""
    rng = np.random.default_rng(seed)
    t = np.arange(n) / hz + rng.uniform(-2e-4, 2e-4, n)
    t = np.sort(t)
    return t, 0.3 * np.sin(2 * np.pi * 0.15 * t)


def _synth_quantized(n=2500, seed=2):
    """模拟 B：250Hz 更新，但位置被量化到 16bit 步长。"""
    t, x = _synth_smooth(n, seed=seed)
    return t, np.round(x / POSITION_STEP_RAD) * POSITION_STEP_RAD


def selftest():
    cases = [
        ("光滑（无锯齿）", _synth_smooth(), ["光滑"]),
        ("A 采样率失配（100Hz 零阶保持 + 250Hz 采样）", _synth_time_staircase(), ["A 采样率失配"]),
        ("B 量化（250Hz 更新 + 16bit 步长）", _synth_quantized(), ["B 量化"]),
        # ★ 回归：改频后（100/100）**不得**被判成 A。这条是因为第一版判据用绝对
        #   间隔（≈10ms）写的，而 100Hz 采样下 10ms 正是采样周期 ⇒ 会误报。
        ("改频后 100/100 对齐（不得误报 A）", _synth_aligned(), ["光滑"]),
    ]
    ok = True
    for name, (t, x), (expect,) in cases:
        s = analyze_series(t, x)
        tags, lines = verdict(s)
        hit = expect in tags
        ok &= hit
        print(f"\n── {name} ──")
        for ln in lines:
            print(ln)
        print(f"  判定：{tags}   期望含 {expect!r} ⇒ {'✓' if hit else '✗ 失败'}")

    # A 的量化判据要能区分"有重复"与"没重复"
    s_a = analyze_series(*_synth_time_staircase())
    s_s = analyze_series(*_synth_smooth())
    print(f"\n── 关键分辨力检查 ──")
    print(f"  A 的重复样本占比 {s_a['repeat_frac'] * 100:.1f}%  vs  光滑 {s_s['repeat_frac'] * 100:.1f}%")
    print(f"  A 的变化间隔众数 {s_a['dt_changes_mode'] * 1000:.2f} ms"
          f"  vs  光滑 {s_s['dt_changes_mode'] * 1000:.2f} ms")
    sep = (s_a["repeat_frac"] > 3 * max(s_s["repeat_frac"], 1e-6)
           and abs(s_a["dt_changes_mode"] - 1 / FIRMWARE_REPORT_HZ) < 0.3 / FIRMWARE_REPORT_HZ)
    print(f"  ⇒ {'✓ 能分开' if sep else '✗ 分不开，判据不可信'}")
    ok &= sep

    print(f"\n{'✓ selftest 全部通过' if ok else '✗ selftest 有失败项'}")
    return 0 if ok else 1


# ────────────────────────── 读包 ──────────────────────────

def read_bag(path):
    """返回 {话题: [(t, msg), ...]}。t 用**消息自带的 header.stamp**（不是包的接收
    时刻）—— 判别采样率必须用数据自己的时间戳。"""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=""),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                            output_serialization_format="cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = {}
    while reader.has_next():
        topic, data, _recv = reader.read_next()
        if topic not in types:
            continue
        msg = deserialize_message(data, get_message(types[topic]))
        stamp = msg.header.stamp
        out.setdefault(topic, []).append((stamp.sec + stamp.nanosec * 1e-9, msg))
    return out


def _joint_series(entries, joint):
    """从 /joint_states 取出某个关节的 (t, position)。"""
    ts, xs = [], []
    for t, msg in entries:
        if joint in msg.name:
            i = msg.name.index(joint)
            ts.append(t)
            xs.append(msg.position[i])
    return np.asarray(ts), np.asarray(xs)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", nargs="?", help="rosbag 目录")
    ap.add_argument("--selftest", action="store_true", help="在合成数据上自证判据")
    ap.add_argument("--joints", default="", help="只看这些关节（逗号分隔）")
    args = ap.parse_args()

    if args.selftest:
        return selftest()
    if not args.bag:
        ap.error("给一个 bag 路径，或用 --selftest")

    data = read_bag(args.bag)
    js = data.get("/joint_states")
    if not js:
        sys.exit(f"包里没有 /joint_states（有：{list(data)}）")
    joints = [j for j in js[0][1].name if not args.joints or j in args.joints.split(",")]

    print(f"包：{args.bag}")
    print(f"/joint_states：{len(js)} 条，"
          f"{js[0][0]:.3f} → {js[-1][0]:.3f} s（跨度 {js[-1][0] - js[0][0]:.1f} s）")
    print(f"关节：{', '.join(joints)}\n")

    for joint in joints:
        t, x = _joint_series(js, joint)
        if x.size < 10:
            print(f"── {joint}：样本不足（{x.size}）\n")
            continue
        s = analyze_series(t, x)
        tags, lines = verdict(s)
        print(f"── {joint} ──  样本 {s['n']}，实测平均 {s['rate']:.1f} Hz")
        for ln in lines:
            print(ln)
        print(f"  判定：{', '.join(tags)}\n")

    # 参考（命令）侧对照：用来定位锯齿出在命令侧还是反馈侧
    for topic, entries in data.items():
        if topic == "/joint_states" or "controller_state" not in topic:
            continue
        print(f"── 参考（命令）侧对照：{topic}（{len(entries)} 条）──")
        # ⚠ JointTrajectoryControllerState 的 joint_names 在**顶层**，
        #   不在 reference 下面（reference 只是个 JointTrajectoryPoint）。
        names = list(getattr(entries[0][1], "joint_names", []) or [])
        if not names:
            print("  （这条消息里没有 joint_names，跳过）\n")
            continue
        for joint in joints:
            if joint not in names:
                continue
            i = names.index(joint)
            ts = np.array([t for t, _ in entries])
            xs = np.array([m.reference.positions[i] for _, m in entries])
            s = analyze_series(ts, xs)
            print(f"  {joint}: 重复样本 {s['repeat_frac'] * 100:.1f}%，"
                  f"变化间隔众数 {s['dt_changes_mode'] * 1000:.2f} ms")
        print("  ⇒ 这一侧若**几乎没有重复**而 /joint_states 重复很多"
              " ⇒ 锯齿出在**反馈**路径（固件上报率），不在命令侧\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
