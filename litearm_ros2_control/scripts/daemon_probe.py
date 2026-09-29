#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""daemon_probe.py — 只读探针：判定「臂下坠 / 来回摇」是**调度空档**还是**固件拒帧**。

为什么不用串口
--------------
栈在跑时打开 ``/dev/ttyACM0`` 会**抢走守护进程的字节**，反而制造出正要诊断的那个
看门狗跳闸。本脚本只读共享内存 + 读日志，全程不碰串口。

判据
----
固件 `control_loop.c` 的 ``hold = watchdog_tripped() || g_arm.drop_hold``，而
``watchdog_tripped`` 只在 ``enabled`` 时评估、门限 100 ms（``watchdog_timeout_s``）。
一掉进 ``hold`` 就是 fail-soft 持位 —— ``kp×0.6`` / **``τ=0``** / 无重力前馈
⇒ 臂下沉。所以

    「臂掉了一下」  ≡  「连续 >100 ms 没有被固件**接受**的命令帧」

这有**两条完全不同的成因**，处置也不同，必须先分开：

======================  ====================================================  ==========================
成因                    机制                                                  判据 / 修法
======================  ====================================================  ==========================
**调度空档**（H1）      守护进程被抢占 >100 ms ⇒ **根本没发帧**                `cycle_count` 出现 >100 ms 不推进
                                                                              ⇒ 给守护进程 `chrt`
**固件拒帧**（门禁）    帧发了也送到了，但被 `ctrl_accept_*` 拒绝 —— 拒绝发生    日志里 `被拒[0x03/0x02×N]`
                        在 `watchdog_kick()` **之前** ⇒ 同样不喂看门狗       ⇒ 持位帧改发实测位置
======================  ====================================================  ==========================

``cycle_count`` 是守护进程**自己每周期写**的（`_publish`），所以「它不推进」=
「守护进程真的停了那么久」，与链路拥塞、固件行为都无关 —— 这是本探针的主判据。

用法
----
::

    source /opt/ros/humble/setup.bash
    source install/setup.bash          # 让 shm_bridge 找到 liblitearm_shm.so
    python3 scripts/daemon_probe.py 8                     # 采共享内存 8 s
    python3 scripts/daemon_probe.py 0 --log <launch.log>  # 只扫日志

⚠ 要抓**启动窗**或**运动末**，必须在窗口**之内**跑本脚本（起栈 / 发 goal 之前
就把它挂上）。事后只能扫日志，而日志看不到 `cycle_count`。
"""

import argparse
import re
import sys
import time
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from litearm_ros2_control.shm_bridge import (  # noqa: E402
    DAEMON_SHUTTING_DOWN, DAEMON_STATUS_TEXT, SharedMemory, ShmError)

# 固件命令看门狗门限（`params/defaults.c` 的 `watchdog_timeout_s = 0.10`）。
# 也是守护进程侧 `command_timeout_s` 的默认值 —— 两边同值，会**同时**跳。
FIRMWARE_WATCHDOG_S = 0.10

# 采样周期：要在 100 ms 内分辨出"停转"，2 ms 有两个数量级余量。
SAMPLE_PERIOD_S = 0.002

REJECT_RE = re.compile(r"被拒\[([^\]]*)\]")
REJECT_ITEM_RE = re.compile(r"(0x[0-9a-f]{2})/(0x[0-9a-f]{2})×(\d+)")
TIMESTAMP_RE = re.compile(r"\[(?:INFO|WARN|ERROR)\] \[([0-9]+\.[0-9]+)\]")
LINK_LINE_RE = re.compile(r"链路计数（[0-9]+s）")
MISMATCH_RE = re.compile(r"守护进程未跟随命令")
RETRY_RE = re.compile(r"原地重试")


# ────────────────────────── 共享内存采样 ──────────────────────────


def sample_shm(name, seconds):
    """采样守护进程的 `cycle_count` 节拍。返回统计字典。

    `cycle_count` 在 `_publish()` 里每周期都写（连接与否都写），所以它的推进
    就等价于"守护进程这一拍跑到了"。
    """
    stats = {
        "samples": 0, "torn": 0, "ticks": 0,
        "stalls": [], "errors": Counter(), "watchdog_edges": 0,
        "connected": None, "enabled": None, "dry_run": None,
        "command_age_max": 0.0, "command_age_last": float("nan"),
        "last_error_last": None, "elapsed": 0.0,
    }
    try:
        shm = SharedMemory(name, create=False)
    except ShmError as exc:
        return {"fatal": f"打不开共享内存 {name!r}：{exc}\n"
                         f"  ⇒ 守护进程没在跑？（或需要 source install/setup.bash）"}

    with shm:
        t0 = time.monotonic()
        last_cycle = None
        last_change = t0
        prev_watchdog = None
        while True:
            now = time.monotonic()
            if now - t0 >= seconds:
                break
            state = shm.try_read_state()
            if state is None:
                stats["torn"] += 1
            else:
                cycle = float(state.cycle_count)
                stats["samples"] += 1
                stats["connected"] = bool(state.connected)
                stats["enabled"] = bool(state.enabled)
                stats["dry_run"] = bool(state.dry_run)
                stats["command_age_last"] = float(state.command_age_s)
                if stats["command_age_last"] == stats["command_age_last"]:
                    stats["command_age_max"] = max(
                        stats["command_age_max"], stats["command_age_last"])
                stats["errors"][int(state.last_error)] += 1
                stats["last_error_last"] = int(state.last_error)
                watchdog = bool(state.watchdog_tripped)
                if prev_watchdog is False and watchdog:
                    stats["watchdog_edges"] += 1
                prev_watchdog = watchdog
                if last_cycle is None or cycle != last_cycle:
                    if last_cycle is not None:
                        # 上一拍到现在才推进 ⇒ 这段就是"停转时长"。
                        gap = now - last_change
                        if gap > 0.02:          # 20 ms 以下算正常抖动，不记
                            stats["stalls"].append((last_change - t0, gap))
                    stats["ticks"] += 1
                    last_change = now
                    last_cycle = cycle
            time.sleep(SAMPLE_PERIOD_S)
        stats["elapsed"] = time.monotonic() - t0
        # 尾部那段"开口区间"也要记：采样结束时若 `cycle_count` 已经 >100 ms 没动，
        # 那不是"正常抖动"而是**生产者已经停了**（活着的 250 Hz 守护进程，
        # 退出时这个间隔 ≤ 一个周期）。漏掉它就会把"守护进程死了/被杀"报成"无停转"。
        if last_cycle is not None:
            gap = time.monotonic() - last_change
            if gap > 0.02:
                stats["stalls"].append((last_change - t0, gap))
    return stats


def report_shm(name, stats):
    if "fatal" in stats:
        print(f"✗ {stats['fatal']}")
        return None

    elapsed = stats["elapsed"] or 1e-9
    print(f"── 共享内存 {name}：采样 {elapsed:.1f}s"
          f"（{stats['samples']} 次读到，{stats['torn']} 次撕裂重试）")
    if stats["ticks"] <= 1 and elapsed > 1.0:
        print(f"   ✗ **生产者没在发布**：整个窗口 `cycle_count` 只动了 "
              f"{stats['ticks']} 次 ⇒ 守护进程**没在跑**（已退出/没启动），"
              f"不是在「被抢占」。")
        print("     下面的「最长停转」只是它停止发布之后的残留，别当成调度空档。")
    print(f"   守护进程节拍     : {stats['ticks'] / elapsed:.1f} Hz"
          f"（名义 250；Δcycle_count/Δt）")
    print(f"   connected={stats['connected']} enabled={stats['enabled']} "
          f"dry_run={stats['dry_run']}")
    print(f"   command_age      : 末值 {stats['command_age_last']:.4f}s "
          f"· 峰值 {stats['command_age_max']:.4f}s")
    seen = ", ".join(f"{code}({DAEMON_STATUS_TEXT.get(code, '未收录')})"
                     f"×{count}" for code, count in stats["errors"].most_common())
    print(f"   last_error 出现过 : {seen or '（还没读到）'}")
    print(f"   watchdog_tripped 上升沿: {stats['watchdog_edges']}")

    stalls = sorted(stats["stalls"], key=lambda item: -item[1])[:5]
    if not stalls:
        print("   最长停转         : 无（>20 ms 的空档一个都没有）")
    else:
        print("   最长停转（前 5） :")
        for offset, gap in stalls:
            mark = "  ⚠ >100ms ⇒ 固件必然跳闸" if gap > FIRMWARE_WATCHDOG_S else ""
            print(f"       t+{offset:7.2f}s 停了 {gap * 1000:7.1f} ms{mark}")
    return stats


# ─────────────────────────── 日志扫描 ───────────────────────────


def scan_log(path):
    text = Path(path).read_text(encoding="utf-8", errors="replace")
    lines = text.splitlines()
    rejects = Counter()
    reject_lines = 0
    mismatch_times = []
    link_lines = 0
    retries = 0
    for line in lines:
        if LINK_LINE_RE.search(line):
            link_lines += 1
        if RETRY_RE.search(line):
            retries += 1
        if MISMATCH_RE.search(line):
            match = TIMESTAMP_RE.search(line)
            if match:
                mismatch_times.append(float(match.group(1)))
        found = REJECT_RE.search(line)
        if found:
            reject_lines += 1
            for cmd, reason, count in REJECT_ITEM_RE.findall(found.group(1)):
                rejects[(int(cmd, 16), int(reason, 16))] += int(count)
    return {"path": str(path), "rejects": rejects, "reject_lines": reject_lines,
            "mismatch_times": mismatch_times, "link_lines": link_lines,
            "retries": retries}


def report_log(stats):
    print(f"── 日志 {stats['path']}")
    if stats["rejects"]:
        print(f"   固件拒帧         : {stats['reject_lines']} 行汇总，"
              f"去重后逐码计数（**这就是「帧被拒、不喂看门狗」的直接证据**）：")
        for (cmd, reason), count in stats["rejects"].most_common():
            tag = ""
            if (cmd, reason) == (0x03, 0x02):
                tag = ("  ← MOVE_JS 被固件门禁拒（dq 全 0 且目标离实测 > 5 mrad）；"
                       "修法 = 持位帧发实测位置")
            elif cmd == 0x10:
                tag = "  ← ENABLE 的 0x03 是正常路径（首写 CMODE），不必管"
            print(f"       {cmd:#04x}/{reason:#04x} ×{count}{tag}")
    else:
        print("   固件拒帧         : 无（本窗口没有帧被拒）")

    times = stats["mismatch_times"]
    if not times:
        print("   未跟随命令告警   : 无")
    else:
        gaps = [b - a for a, b in zip(times, times[1:])]
        window = times[-1] - times[0]
        period = (sum(gaps) / len(gaps)) if gaps else float("nan")
        print(f"   未跟随命令告警   : {len(times)} 条 · 窗口 {window:.1f}s "
              f"(t={times[0]:.2f}…{times[-1]:.2f}) · 平均间隔 {period:.3f}s")
        print(f"                      平均间隔 ≈1/摆动频率 ≈{1 / period:.2f} Hz"
              if period == period and period > 0 else "")
    if stats["retries"]:
        print(f"   启动期原地重试   : {stats['retries']} 条"
              f"（>0 ⇒ 走过 `_poll_sliced` 那条保活路径）")
    print(f"   链路计数行       : {stats['link_lines']} 条"
          f"（每 5 s 一条 ⇒ 本日志覆盖约 {stats['link_lines'] * 5}s）")


# ─────────────────────────── 结论 ───────────────────────────


def verdict(shm_stats, log_stats):
    print("\n── 结论")
    rejects = (log_stats or {}).get("rejects", {})
    motion = {code: n for code, n in rejects.items()
              if code[0] in (0x01, 0x02, 0x03, 0x04, 0x05, 0x07)}

    if shm_stats is None and not rejects:
        print("   两个证据都没有 ⇒ 本窗口没有下坠的条件，换到窗口内再采一次。")
        return

    if shm_stats is not None:
        stalls = shm_stats.get("stalls", [])
        long_stalls = [gap for _offset, gap in stalls
                       if gap > FIRMWARE_WATCHDOG_S]
        producer_dead = (shm_stats.get("ticks", 0) <= 1
                         and shm_stats.get("elapsed", 0.0) > 1.0)
        if producer_dead:
            # 生产者已退出 ⇒ 那些"停转"只是残留。报成调度空档会把人引到
            # `chrt` 上去，而真正该看的是它为什么退出（`last_error`）。
            code = shm_stats.get("last_error_last")
            why = ("正常退出流程" if code == DAEMON_SHUTTING_DOWN
                   else f"`last_error={code}`"
                        f"（{DAEMON_STATUS_TEXT.get(code, '未收录')}）")
            print(f"   ✗ **调度空档不成立**：守护进程**根本没在发布** —— {why}。")
            print("     ⇒ 采样窗是空转的；把探针挂在窗口**内**再采一次"
                  "（起栈/发 goal 之前就启动它）。")
        elif long_stalls:
            print(f"   ✔ **调度空档成立**：`cycle_count` 有 {len(long_stalls)} 次停转 "
                  f">100 ms（最长 {max(long_stalls) * 1000:.0f} ms）")
            print("     ⇒ 守护进程**根本没发帧**，与固件行为无关。修法：给守护进程加 "
                  "`chrt`\n"
                  "       （`launch/litearm_control.launch.py` 的 daemon prefix，"
                  "权限已具备）。")
        else:
            print("   ✗ **调度空档不成立**：守护进程节拍正常、无 >100 ms 停转。")
            print("     ⇒ 跳闸不是因为它停转；去看是不是**帧被固件拒了**（下一段）。")

    if motion:
        print(f"   ✔ **固件拒帧成立**：{motion}")
        print("     ⇒ 帧发了也送到了，但被 `ctrl_accept_*` 拒 ⇒ 不喂看门狗。"
              "修法：持位帧\n"
              "       发**实测位置**（`hw_daemon._hold_target`），别发进入持位时的旧锚点。")
    elif rejects:
        print(f"   · 有拒帧但都是非运动命令：{rejects}（ENABLE 的 0x03 属正常路径）")
    elif shm_stats is not None:
        print("   ✗ **固件拒帧不成立**：日志里没有任何运动帧被拒。")


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="只读诊断探针：调度空档 vs 固件拒帧")
    parser.add_argument("seconds", nargs="?", type=float, default=8.0,
                        help="共享内存采样秒数（0 = 只扫日志）")
    parser.add_argument("--shm-name", default="/litearm_hw")
    parser.add_argument("--log", default=None,
                        help="launch.log / 守护进程日志路径；给了就一并扫")
    args = parser.parse_args(argv)

    shm_stats = None
    if args.seconds > 0:
        shm_stats = report_shm(args.shm_name,
                               sample_shm(args.shm_name, args.seconds))
    log_stats = None
    if args.log:
        log_stats = scan_log(args.log)
        report_log(log_stats)
    if shm_stats is not None or log_stats is not None:
        verdict(shm_stats, log_stats)
    return 0


if __name__ == "__main__":
    sys.exit(main())
