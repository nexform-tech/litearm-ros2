#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""demo_motion.py — 无硬件演示用的关节摆动（经 joint_trajectory_controller）。

用途
----
配合 ``litearm_demo.launch.py``（或手动起来的 dry-run 控制栈）产生一段**看得见**
的连续运动，用来确认「JTC → 硬件接口 → 共享内存 → 守护进程 → 关节状态回读」这条
链路真的通了。没有它，demo 起来后机械臂是静止的，看不出任何东西。

安全设计
--------
* **不需要真实硬件**：走 dry-run 守护进程，唯一"动"的是模拟关节状态。
* **以当前位形为摆动中心**（不是零位），从任意姿态启动都不会先来一次大幅回零。
* **中心点会做安全内缩**：若当前位置离某个限位太近（放不下目标振幅），就把该轴
  的摆动中心朝行程内部挪，并用 ``--ramp`` 段的平滑引导从当前位置过渡过去。
  没有这一步，像 J4 这种「行程几乎全在负侧、零位距上限仅 1°」的轴会永远动不了。
* **振幅按实测关节限位逐轴收紧**：从 ``/robot_state_publisher`` 的
  ``robot_description`` 参数解析 URDF 限位，每轴只用到可用余量的
  ``--margin-fraction``（默认 60%）。解析不到就退回内置保守振幅并明确告警。
* 单轴可用余量不足 ``--min-amplitude`` 时该轴保持不动，避免"贴着限位蹭"。
* 速度由周期决定（默认 12s 一个整周期），峰值约 ``amp·2π/T``，量级 0.2~0.3 rad/s；
  两端各有 ``--ramp`` 秒的升余弦包络，起停速度为零。

关于 JTC 的位置容差
-------------------
joint_trajectory_controller 会按 ``constraints.<joint>.trajectory`` 检查
"参考位置 - 实测位置"，超差就**中止整条轨迹**。本脚本的默认参数（周期 12s、
振幅 ≤0.45 rad）在 dry-run 的一阶滞后模型下误差约 0.05~0.1 rad，落在
``litearm_controllers.yaml`` 按固件 ``following_error``（0.25~0.35 rad）设的
容差内。若你手工把周期调得很短（例如 4s），误差会超过容差，
JTC 会报 "State tolerances failed" 并中止 —— 那是这套参数组合的预期行为，
不是链路故障。

索引约定
--------
**一切按关节名索引，不按下标。** 实测 ``/joint_states`` 的 ``name`` 顺序是
``[joint2, joint3, joint5, joint6, joint1, joint4, joint7]``（DDS 不保证顺序），
按下标比对会静默错位。

用法::

    python3 demo_motion.py                      # 无限循环，Ctrl-C 停
    python3 demo_motion.py --cycles 2            # 只跑 2 轮后退出
    python3 demo_motion.py --scale 0.5           # 振幅减半
    python3 demo_motion.py --period 15           # 放慢到 15s 一个周期
    python3 demo_motion.py --ramp 5              # 起停更缓慢
"""

import argparse
import math
import sys
import time
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = [f"joint{i}" for i in range(1, 8)]

# 内置的保守振幅（rad）。解析不到 URDF 限位时用它，量级远小于本臂最小限位 1.55。
FALLBACK_AMPLITUDE = {
    "joint1": 0.45, "joint2": 0.30, "joint3": 0.35, "joint4": 0.20,
    "joint5": 0.40, "joint6": 0.30, "joint7": 0.45,
}

# 各轴相位错开，运动看起来更自然（也更容易一眼看出"多条轴都在动"）
PHASE_OFFSET = {
    "joint1": 0.0, "joint2": 0.7, "joint3": 1.4, "joint4": 2.1,
    "joint5": 2.8, "joint6": 3.5, "joint7": 4.2,
}


def _parse_joint_limits(urdf_text: str) -> Dict[str, Tuple[float, float]]:
    """从 URDF 文本里取 revolute 关节的 (lower, upper)。"""
    limits: Dict[str, Tuple[float, float]] = {}
    root = ET.fromstring(urdf_text)
    for joint in root.findall("joint"):
        name = joint.get("name")
        if name not in JOINTS:
            continue
        limit = joint.find("limit")
        if joint.get("type") == "continuous" or limit is None:
            limits[name] = (-math.pi, math.pi)
            continue
        limits[name] = (float(limit.get("lower", -math.pi)),
                        float(limit.get("upper", math.pi)))
    return limits


def _smoothstep(t: float, duration: float) -> Tuple[float, float]:
    """升余弦平滑：返回 (s(t), s'(t))，s(0)=s'(0)=0，s(duration)=1，s'(duration)=0。"""
    if duration <= 0.0:
        return (1.0, 0.0) if t > 0.0 else (0.0, 0.0)
    if t <= 0.0:
        return 0.0, 0.0
    if t >= duration:
        return 1.0, 0.0
    return (0.5 * (1.0 - math.cos(math.pi * t / duration)),
            0.5 * math.pi / duration * math.sin(math.pi * t / duration))


class DemoMotion(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("litearm_demo_motion")
        self.args = args
        self.live: Dict[str, float] = {}
        self.clamped: List[str] = []
        self.frozen: List[str] = []
        self.shifted: List[str] = []
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.client = ActionClient(
            self, FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory")

    def _on_joint_state(self, msg: JointState) -> None:
        # 按名字索引：DDS 不保证 name 与 position 的语义顺序符合我们的期望，
        # 实测顺序是 [joint2, joint3, joint5, joint6, joint1, joint4, joint7]。
        for name, position in zip(msg.name, msg.position):
            self.live[name] = position

    # ── 读取 ──

    def wait_for_joint_states(self, timeout: float = 30.0) -> Optional[Dict[str, float]]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(name in self.live for name in JOINTS):
                return {name: self.live[name] for name in JOINTS}
        return None

    def fetch_limits(self) -> Optional[Dict[str, Tuple[float, float]]]:
        """从 /robot_state_publisher 的 robot_description 参数解析关节限位。

        参数是一整段 URDF 文本。拿不到就返回 None，由调用方退回内置振幅。
        只查这个节点是有意的：限位的唯一真相是 URDF，而 RSP 恰好把它作为
        参数持有，不需要额外起服务或读文件。
        """
        if not self.args.use_urdf_limits:
            return None
        node_name = "/robot_state_publisher"
        known = [n.lstrip("/") for n in self.get_node_names()]
        if node_name.lstrip("/") not in known:
            self.get_logger().debug(f"未发现 {node_name}，跳过 URDF 限位解析")
            return None
        try:
            from rcl_interfaces.srv import GetParameters
            client = self.create_client(GetParameters, f"{node_name}/get_parameters")
            if not client.wait_for_service(timeout_sec=3.0):
                raise RuntimeError("get_parameters 服务不可用")
            request = GetParameters.Request()
            request.names = ["robot_description"]
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            result = future.result()
            if result is None or not result.values:
                raise RuntimeError("未取到 robot_description")
            text = result.values[0].string_value
            if not text:
                raise RuntimeError("robot_description 为空")
            return _parse_joint_limits(text)
        except Exception as exc:  # 取不到就用内置振幅，不值得让演示失败
            self.get_logger().warn(f"解析 URDF 关节限位失败（{exc}），改用内置振幅")
            return None

    # ── 规划摆动参数 ──

    def plan_swing(self, current: Dict[str, float],
                   limits: Optional[Dict[str, Tuple[float, float]]]
                   ) -> Tuple[Dict[str, float], Dict[str, float]]:
        """决定每轴的摆动中心与振幅，返回 (center, amplitude)。

        中心的内缩规则：希望摆到 center±amp 都不出限位，即需要
        ``margin(center) = min(upper-center, center-lower) >= amp/margin_fraction``。
        把 center 钳进 [lower+need, upper-need] 即满足；这个区间为空时说明行程
        本身放不下目标振幅，退到行程中点并用行程的一半作为振幅上限。

        这样哪怕上电位形贴着某个限位（本臂 J4 零位距上限只有 1°），该轴也能
        参与摆动，代价只是开场多一段从当前位置到中心的平滑引导。
        """
        center: Dict[str, float] = {}
        amplitude: Dict[str, float] = {}
        fraction = max(1e-3, self.args.margin_fraction)
        need_min = self.args.min_amplitude / fraction

        for name in JOINTS:
            desired = FALLBACK_AMPLITUDE[name] * self.args.scale
            now = current[name]
            if limits is None:
                center[name] = now
                amplitude[name] = desired
                continue
            lower, upper = limits[name]
            need = desired / fraction
            safe_low, safe_high = lower + need, upper - need
            if safe_low <= safe_high:
                chosen = min(safe_high, max(safe_low, now))
                amp = desired
            else:
                # 行程放不下 desired：退到中点，振幅压到"可用余量的 fraction"
                chosen = 0.5 * (lower + upper)
                amp = max(0.0, min(upper - chosen, chosen - lower) * fraction)
            if abs(chosen - now) > 1e-3:
                self.shifted.append(name)
            allowed = max(0.0, min(upper - chosen, chosen - lower) * fraction)
            if allowed < self.args.min_amplitude:
                center[name] = chosen
                amplitude[name] = 0.0
                self.frozen.append(name)
                continue
            center[name] = chosen
            amplitude[name] = min(amp, allowed)
            if desired > 0.0 and amplitude[name] < desired - 1e-9:
                self.clamped.append(name)
        return center, amplitude

    # ── 轨迹构造 ──

    def build_trajectory(self, start: Dict[str, float], center: Dict[str, float],
                         amplitude: Dict[str, float], phase: float) -> JointTrajectory:
        """一轮往返（``periods`` 个整周期）的轨迹。

        位置由三部分叠加：

            pos(t) = center + amp·e(t)·sin(ωt+φ) + (start − center)·(1 − s(t))

        * ``e`` 是摆动包络，``s`` 是引导包络，两者都是升余弦，两端值/导数均为 0。
        * 于是 t=0 时 pos=start、vel=0；t=total 时 pos=center、vel=0。
        * 首轮 ``start`` 取实测位形（从真实位置平滑起飞），后续轮次 start=center，
          因此可以无缝连发。

        速度里必须带上包络导数项（``amp·e'·sin`` 与 ``(start−center)·s'``）：
        在引导段它们的量级和主项相当，漏掉会让 MIT 的 dq_ref 明显偏掉。
        """
        period = self.args.period
        total = period * self.args.periods
        step = self.args.sample_period
        steps = max(2, int(round(total / step)))
        omega = 2.0 * math.pi / period
        ramp = min(self.args.ramp, total / 3.0)

        trajectory = JointTrajectory()
        trajectory.joint_names = list(JOINTS)
        for index in range(steps + 1):
            t = total * index / steps
            # 摆动包络：引导段结束后升到 1，末段再落回 0
            if t < ramp:
                e, de = _smoothstep(t, ramp)
            elif t > total - ramp:
                e, de = _smoothstep(total - t, ramp)
                de = -de
            else:
                e, de = 1.0, 0.0
            s, ds = _smoothstep(t, ramp)

            point = JointTrajectoryPoint()
            for name in JOINTS:
                angle = omega * t + phase + PHASE_OFFSET[name]
                lead = (start[name] - center[name]) * (1.0 - s)
                lead_vel = (start[name] - center[name]) * (-ds)
                point.positions.append(
                    center[name] + amplitude[name] * e * math.sin(angle) + lead)
                point.velocities.append(
                    amplitude[name] * (de * math.sin(angle)
                                       + e * omega * math.cos(angle))
                    + lead_vel)
            sec = int(t)
            point.time_from_start = Duration(sec=sec, nanosec=int((t - sec) * 1e9))
            trajectory.points.append(point)
        return trajectory

    # ── 执行 ──

    def send(self, trajectory: JointTrajectory) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = Duration(sec=2, nanosec=0)

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=20)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("轨迹目标被拒绝")
            return False
        result_future = handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=1.0)
        wrapped = result_future.result()
        if wrapped is None:
            self.get_logger().error("未拿到执行结果")
            return False
        code = wrapped.result.error_code
        if code != 0:
            self.get_logger().error(
                f"轨迹执行失败: error_code={code}"
                + ("（-4 = JTC 按 constraints.<joint>.trajectory 判定跟踪超差而中止；"
                   "把 --period 调大或减小 --scale 可缓解）" if code == -4 else ""))
            return False
        return True

    def run(self) -> int:
        args = self.args
        self.get_logger().info("等待 /joint_trajectory_controller …")
        if not self.client.wait_for_server(timeout_sec=60.0):
            self.get_logger().error(
                "/joint_trajectory_controller 不可用。请先起控制栈：\n"
                "  ros2 launch litearm_ros2_control litearm_demo.launch.py")
            return 1

        current = self.wait_for_joint_states()
        if current is None:
            self.get_logger().error("/joint_states 超时未给出全部 7 个关节")
            return 1

        limits = self.fetch_limits()
        center, amplitude = self.plan_swing(current, limits)

        self.get_logger().info(f"当前位形: {[round(current[n], 4) for n in JOINTS]}")
        self.get_logger().info(f"摆动中心: {[round(center[n], 4) for n in JOINTS]}")
        self.get_logger().info(f"振幅:     {[round(amplitude[n], 4) for n in JOINTS]}")
        if limits is None:
            self.get_logger().warn("未取到 URDF 关节限位，使用内置保守振幅（未做限位收紧）")
        if self.shifted:
            self.get_logger().info(
                f"以下轴中心已内缩以留出摆幅（开场由 {args.ramp:g}s 引导段过渡）: "
                f"{self.shifted}")
        if self.clamped:
            self.get_logger().warn(f"以下轴振幅被限位收紧: {self.clamped}")
        if self.frozen:
            self.get_logger().warn(
                f"以下轴可用余量不足 {args.min_amplitude} rad，保持不动: {self.frozen}")
        if all(value == 0.0 for value in amplitude.values()):
            self.get_logger().error("所有轴振幅都是 0，无法演示。检查起始位形是否贴着限位。")
            return 1
        self.get_logger().info(
            f"周期 {args.period:g}s × {args.periods} 个整周期/轮，Ctrl-C 停止")

        cycle = 0
        start = dict(current)  # 首轮从实测位形起飞，之后从中心起飞
        try:
            while rclpy.ok():
                # 相邻轮次相位差 π：来回镜像摆动，视觉上更容易看出在动
                phase = (cycle % 2) * math.pi
                trajectory = self.build_trajectory(start, center, amplitude, phase)
                self.get_logger().info(
                    f"第 {cycle + 1} 轮摆动（{args.period * args.periods:g}s）…")
                if not self.send(trajectory):
                    return 1
                cycle += 1
                start = dict(center)  # 上一轮终点就在 center，无缝衔接
                if args.cycles and cycle >= args.cycles:
                    break
        except KeyboardInterrupt:
            self.get_logger().info("收到中断，停止演示")
        self.get_logger().info(f"演示结束，共 {cycle} 轮")
        return 0


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scale", type=float, default=1.0,
                        help="振幅整体缩放（默认 1.0）")
    parser.add_argument("--period", type=float, default=12.0,
                        help="单个摆动周期秒数（默认 12；调小会让 JTC 跟踪超差而中止）")
    parser.add_argument("--periods", type=int, default=2,
                        help="每轮包含的整周期数（默认 2）")
    parser.add_argument("--ramp", type=float, default=3.0,
                        help="起停/引导段的升余弦时长秒数（默认 3）")
    parser.add_argument("--sample-period", type=float, default=0.1,
                        help="轨迹采样间隔秒数（默认 0.1）")
    parser.add_argument("--margin-fraction", type=float, default=0.6,
                        help="可用限位余量的使用比例（默认 0.6）")
    parser.add_argument("--min-amplitude", type=float, default=0.03,
                        help="低于该振幅(rad)的轴保持不动（默认 0.03）")
    parser.add_argument("--cycles", type=int, default=0,
                        help="摆动轮数，0 = 无限循环直到 Ctrl-C")
    parser.add_argument("--no-urdf-limits", dest="use_urdf_limits",
                        action="store_false", default=True,
                        help="不解析 URDF 限位，直接用内置振幅")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    if args.period <= 0 or args.periods < 1 or args.scale <= 0 or args.ramp <= 0:
        print("--period/--ramp/--scale 必须 >0，--periods 必须 >=1", file=sys.stderr)
        return 2
    rclpy.init()
    node = DemoMotion(args)
    try:
        return node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
