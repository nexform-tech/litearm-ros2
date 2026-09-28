#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""e2e_probe.py — end-to-end acceptance probe for ros2_control (a field check
script that is independent of the test suite).

Against an already started litearm_control stack:
  1. confirm the action server of joint_trajectory_controller is available
  2. send a two-point trajectory
  3. read back from /joint_states and check that the actual position converges
     to the end of the trajectory
  4. additionally check the daemon status block (diagnostics beyond
     /joint_states are not reachable through rclpy parameters, so this only
     does the checks visible from the ROS side)

Usage:
  ros2 launch litearm_ros2_control litearm_control.launch.py dry_run:=true &
  python3 e2e_probe.py

Exit code 0 = pass.
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
    print("waiting for the /joint_trajectory_controller action server …")
    if not client.wait_for_server(timeout_sec=30.0):
        print("✗ action server did not appear: is JTC activated?")
        return 1
    print("✓ action server ready")

    # spin a few times first to be sure /joint_states has values
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.05)
        if len(latest) >= 7:
            break
    if len(latest) < 7:
        print(f"✗ /joint_states did not provide 7 joints (got {sorted(latest)})")
        return 1
    start = [latest[name] for name in JOINTS]
    print(f"  start position: {[round(v, 4) for v in start]}")

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINTS
    for seconds, fraction in ((1.0, 0.5), (2.5, 1.0)):
        point = JointTrajectoryPoint()
        # interpolate from the measured start position to the target so the
        # first trajectory point does not jump
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
        print("✗ trajectory goal rejected")
        return 1
    print("✓ trajectory accepted, executing …")

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=30)
    result = result_future.result()
    if result is None:
        print("✗ no execution result within the timeout")
        return 1
    error_code = result.result.error_code
    print(f"✓ execution finished: error_code={error_code} "
          f"({'SUCCESSFUL' if error_code == 0 else 'FAILED'})")

    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.05)

    worst = 0.0
    for index, name in enumerate(JOINTS):
        worst = max(worst, abs(latest.get(name, 99.0) - TARGET[index]))
    print(
        f"  measured position: {[round(latest.get(n, float('nan')), 4) for n in JOINTS]}")
    print(f"  target position: {[round(v, 4) for v in TARGET]}")
    print(f"  max error: {worst:.4e} rad (tolerance {TOLERANCE_RAD})")

    node.destroy_node()
    rclpy.shutdown()

    if error_code != 0:
        print("✗ trajectory execution reported a failure")
        return 1
    if worst >= TOLERANCE_RAD:
        print("✗ tracking error exceeds the limit")
        return 1
    print("\n✓ end-to-end trajectory tracking acceptance passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
