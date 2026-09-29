#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""record_joint_states.py — 录一段 /joint_states 供"位置曲线为什么是锯齿"分析用。

为什么单独写一个而不是直接 `ros2 bag record /joint_states`
------------------------------------------------------------
要判"锯齿是**采样率失配**还是**量化**"，光有位置不够，还得有**参照**：
控制器侧的参考（``~/controller_state``）是**命令**，/joint_states 是**反馈**。
两者一起录下来才能定位锯齿出现在哪一段 —— 参考光滑而反馈呈台阶 ⇒ 锯齿在反馈侧；
两者都台阶 ⇒ 在命令侧。所以这里按需带上那个话题。

用法
----
    # 终端 A：录（默认等你在终端 B 里手动动臂，Ctrl-C 结束）
    python3 record_joint_states.py

    # 指定时长/输出目录/只录反馈
    python3 record_joint_states.py --duration 60 --out ~/wkspace/ros2_ws/bags
    python3 record_joint_states.py --no-reference

⚠ 域必须对：本栈默认 ROS_DOMAIN_ID=42 / ROS_LOCALHOST_ONLY=1，见《真机启停指令》§0。
"""

import argparse
import os
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime

# 反馈（必录）与参考（有就录，用来定位锯齿在哪一段）
FEEDBACK_TOPIC = "/joint_states"
REFERENCE_CANDIDATES = (
    "/joint_trajectory_controller/controller_state",   # 控制器的参考位置/速度
    "/joint_trajectory_controller/joint_trajectory",   # 正在执行的那条轨迹
)

PROGRESS_PERIOD_S = 2.0

# 分析脚本的名字（写死，别用 __file__ 拼 —— 之前拼错过，把使用者指到一个不存在的文件名）
ANALYZER = "analyze_joint_sawtooth.py"


def _topics_now():
    """当前可见话题集合（取不到就返回空 —— 不因此中断录制）。"""
    try:
        out = subprocess.run(["ros2", "topic", "list"], capture_output=True,
                             text=True, timeout=10.0).stdout
        return {line.strip() for line in out.splitlines() if line.strip()}
    except Exception:
        return set()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=os.path.expanduser("~/wkspace/ros2_ws/bags"),
                    help="输出目录（默认 ~/wkspace/ros2_ws/bags）")
    ap.add_argument("--name", default=None, help="包名（默认 joint-states-<时间戳>）")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="录制秒数；0 = 一直录到 Ctrl-C（默认）")
    ap.add_argument("--no-reference", action="store_true",
                    help="只录 /joint_states，不带控制器参考")
    args = ap.parse_args()

    if shutil.which("ros2") is None:
        sys.exit("找不到 ros2 —— 先 source /opt/ros/humble/setup.bash 与 install/setup.bash")

    topics = _topics_now()
    if FEEDBACK_TOPIC not in topics:
        print(f"⚠ 当前话题列表里没有 {FEEDBACK_TOPIC}。")
        print("  栈起了吗？另开终端确认：ros2 topic list | grep joint_states")
        print("  ⚠ 域要对：export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1")
        sys.exit(2)

    record = [FEEDBACK_TOPIC]
    if not args.no_reference:
        for cand in REFERENCE_CANDIDATES:
            if cand in topics:
                record.append(cand)
                print(f"✓ 找到参考话题 {cand}（用来分辨锯齿出在命令侧还是反馈侧）")
                break
        else:
            print("· 没有控制器参考话题，本次只录反馈（仍能判采样率失配与量化）")

    name = args.name or f"joint-states-{datetime.now():%Y%m%d-%H%M%S}"
    path = os.path.join(os.path.expanduser(args.out), name)
    os.makedirs(os.path.dirname(path), exist_ok=True)

    # ⚠ 话题是**位置参数**：Humble 的 ros2 bag record **没有** --topics
    #   （写成 --topics 会得到 `ros2: error: unrecognized arguments: --topics`，
    #    而且**瞬间退出**——之前踩过：把它当成"录完了"，白折腾两次）
    cmd = ["ros2", "bag", "record", "-o", path, *record]
    print("\n" + "=" * 72)
    print("开始录制。**现在去手动动臂** —— 让每个关节都走过一段平滑的、慢速的运动")
    print("（慢速很关键：速度快时机械惯量会把台阶抹平，看不到真形状）。")
    print("建议：单关节、小幅度、来回几次；每个主要关节都轮到。")
    print("录完按 Ctrl-C。")
    print("=" * 72 + "\n")
    print("$ " + " ".join(cmd) + "\n")

    start = time.monotonic()
    proc = subprocess.Popen(cmd)
    stopped = False

    def _stop(signum, frame):
        nonlocal stopped
        stopped = True
        proc.send_signal(signal.SIGINT)

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    try:
        while proc.poll() is None:
            if args.duration > 0 and time.monotonic() - start >= args.duration:
                proc.send_signal(signal.SIGINT)
                break
            time.sleep(0.2)
    finally:
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:  # pragma: no cover
            proc.kill()
            proc.wait()

    elapsed = time.monotonic() - start

    # ★ 必须**验一下真的录到了**：bag 命令失败时是瞬间退出（返回码可能仍是 0），
    #   以前只看"进程结束了"就报成功 ⇒ 空目录也被当成录好了，白跑两次。
    #   metadata.yaml 是 rosbag2 一开始就写的，它不在就一定没录成。
    meta = os.path.join(path, "metadata.yaml")
    db3 = [f for f in os.listdir(path) if f.endswith(".db3")] if os.path.isdir(path) else []
    if not os.path.exists(meta) or not db3:
        print(f"\n✗ 录制**没有成功**：{path}")
        print(f"  metadata.yaml={'有' if os.path.exists(meta) else '缺'}，.db3={len(db3)} 个"
              f"，耗时 {elapsed:.1f} s")
        print("  常见原因：① bag 命令行参数不对（子进程会打 ros2 的 usage 错误）；")
        print("           ② 没有可订阅的发布者 —— 话题名或 ROS 域不对"
              "（本栈要 export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1）")
        return 1

    size = 0
    for root, _dirs, files in os.walk(path):
        size += sum(os.path.getsize(os.path.join(root, f)) for f in files)

    print(f"\n✓ 录制结束：{elapsed:.1f} s，{size / 1e6:.2f} MB")
    print(f"  包路径：{path}")
    print(f"  话题：{' '.join(record)}")
    print("\n下一步（分析）：")
    print(f"  python3 {ANALYZER} {path}")
    print("⚠ 想先确认判据可信，可先跑 `python3 {0} --selftest`（它在合成数据上自证）。"
          .format(ANALYZER))
    return 0


if __name__ == "__main__":
    sys.exit(main())
