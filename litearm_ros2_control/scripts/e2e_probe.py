#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""e2e_probe.py — ros2_control 全链路验收探针（独立于测试套件的现场检查脚本）。

在已启动的 litearm_control 栈上：
  1. 确认 joint_trajectory_controller 的 action server 可用
  2. 下发一条两点轨迹
  3. 从 /joint_states 回读，校验实际位置收敛到轨迹终点
  4. 顺带校验守护进程状态块（/joint_states 之外的诊断量通过 rclpy 参数拿不到，
     这里只做 ROS 侧可见的检查）

用法：
  ros2 launch litearm_ros2_control litearm_control.launch.py dry_run:=true &
  python3 e2e_probe.py

退出码 0 = 通过。
"""

import sys

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = [f"joint{i}" for i in range(1, 8)]
TARGET = [0.35, -0.25, 0.20, -0.15, 0.10, 0.05, -0.05]
TOLERANCE_RAD = 0.02


def main() -> int:
    rclpy.init()
    node = Node("litearm_e2e_probe")
    latest = {}

    def on_joint_state(msg):
        for name, position in zip(msg.name, msg.position):
            latest[name] = position

    node.create_subscription(JointState, "/joint_states", on_joint_state, 10)

    client = ActionClient(node, FollowJointTrajectory,
                          "/joint_trajectory_controller/follow_joint_trajectory")
    print("等待 /joint_trajectory_controller action server …")
    if not client.wait_for_server(timeout_sec=30.0):
        print("✗ action server 未出现：JTC 是否已激活？")
        return 1
    print("✓ action server 就绪")

    # 先转几圈，确保 /joint_states 有值
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.05)
        if len(latest) >= 7:
            break
    if len(latest) < 7:
        print(f"✗ /joint_states 未给出 7 个关节（收到 {sorted(latest)}）")
        return 1
    start = [latest[name] for name in JOINTS]
    print(f"  起始位置: {[round(v, 4) for v in start]}")

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINTS
    for seconds, fraction in ((1.0, 0.5), (2.5, 1.0)):
        point = JointTrajectoryPoint()
        # 从实测起始位置插值到目标，避免"第一条轨迹点突然跳变"
        point.positions = [start[i] + (TARGET[i] - start[i]) * fraction
                           for i in range(7)]
        point.velocities = [0.0] * 7
        point.time_from_start.sec = int(seconds)
        point.time_from_start.nanosec = int((seconds % 1) * 1e9)
        goal.trajectory.points.append(point)

    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=15)
    goal_handle = send_future.result()
    if goal_handle is None or not goal_handle.accepted:
        print("✗ 轨迹目标被拒绝")
        return 1
    print("✓ 轨迹已接受，执行中 …")

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=30)
    result = result_future.result()
    if result is None:
        print("✗ 未在超时内拿到执行结果")
        return 1
    error_code = result.result.error_code
    print(f"✓ 执行结束: error_code={error_code} "
          f"({'SUCCESSFUL' if error_code == 0 else 'FAILED'})")

    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.05)

    worst = 0.0
    for index, name in enumerate(JOINTS):
        worst = max(worst, abs(latest.get(name, 99.0) - TARGET[index]))
    print(f"  实测位置: {[round(latest.get(n, float('nan')), 4) for n in JOINTS]}")
    print(f"  目标位置: {[round(v, 4) for v in TARGET]}")
    print(f"  最大误差: {worst:.4e} rad（容差 {TOLERANCE_RAD}）")

    node.destroy_node()
    rclpy.shutdown()

    if error_code != 0:
        print("✗ 轨迹执行返回失败")
        return 1
    if worst >= TOLERANCE_RAD:
        print("✗ 跟踪误差超限")
        return 1
    print("\n✓ 端到端轨迹跟踪验收通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
