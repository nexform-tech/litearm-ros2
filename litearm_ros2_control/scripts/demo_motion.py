#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""demo_motion.py — joint swinging for the no-hardware demo (through
joint_trajectory_controller).

Purpose
-------
Works with ``litearm_demo.launch.py`` (or a manually started dry-run control
stack) to produce **visible** continuous motion, confirming that the chain
"JTC → hardware interface → shared memory → daemon → joint state read back" is
really connected. Without it the arm just sits still once the demo comes up
and there is nothing to see.

Safety design
-------------
* **No real hardware needed**: this goes through the dry-run daemon and the
  only thing that "moves" is the simulated joint state.
* **The swing centre is the current configuration** (not the zero pose), so
  starting from any pose never begins with a big move back to zero.
* **The centre is pulled in safely**: if the current position is too close to a
  limit (no room for the target amplitude), that axis' swing centre is moved
  inward along the travel and the ``--ramp`` segment smoothly leads from the
  current position to it. Without this step an axis like J4 — whose travel is
  almost entirely on the negative side and whose zero pose is only 1° from the
  upper limit — could never move.
* **The amplitude is tightened per axis against the measured joint limits**:
  the URDF limits are parsed from the ``robot_description`` parameter of
  ``/robot_state_publisher``, and each axis only uses ``--margin-fraction`` of
  the available margin (60% by default). If parsing fails, it falls back to the
  built-in conservative amplitudes and warns explicitly.
* When the available margin of a single axis is below ``--min-amplitude``, that
  axis stays put, avoiding "rubbing against the limit".
* The speed follows from the period (12s per full cycle by default) with a peak
  of about ``amp·2π/T``, in the 0.2~0.3 rad/s range; both ends have a ``--ramp``
  second raised-cosine envelope, so the start/stop velocity is zero.

About the JTC position tolerance
--------------------------------
joint_trajectory_controller checks "reference position - measured position"
against ``constraints.<joint>.trajectory`` and **aborts the whole trajectory**
when the tolerance is exceeded. With the default parameters of this script
(period 12s, amplitude ≤0.45 rad) the error under the dry-run first-order lag
model is about 0.05~0.1 rad, inside the tolerance that
``litearm_controllers.yaml`` sets from the firmware ``following_error``
(0.25~0.35 rad). If you manually set a very short period (4s, say), the error
exceeds the tolerance and JTC reports "State tolerances failed" and aborts —
that is the expected behaviour of this parameter combination, not a broken
chain.

Indexing convention
-------------------
**Everything is indexed by joint name, never by index.** The measured ``name``
order of ``/joint_states`` is
``[joint2, joint3, joint5, joint6, joint1, joint4, joint7]`` (DDS does not
guarantee an order), and comparing by index misaligns silently.

Usage::

    python3 demo_motion.py                      # loop forever, Ctrl-C stops
    python3 demo_motion.py --cycles 2            # exit after 2 rounds
    python3 demo_motion.py --scale 0.5           # halve the amplitude
    python3 demo_motion.py --period 15           # slow down to 15s per cycle
    python3 demo_motion.py --ramp 5              # slower start/stop
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

# Built-in conservative amplitudes (rad). Used when the URDF limits cannot be
# parsed; far smaller than this arm's smallest limit of 1.55.
FALLBACK_AMPLITUDE = {
    "joint1": 0.45, "joint2": 0.30, "joint3": 0.35, "joint4": 0.20,
    "joint5": 0.40, "joint6": 0.30, "joint7": 0.45,
}

# Stagger the axis phases so the motion looks more natural (and it is easier to
# see at a glance that "several axes are moving")
PHASE_OFFSET = {
    "joint1": 0.0, "joint2": 0.7, "joint3": 1.4, "joint4": 2.1,
    "joint5": 2.8, "joint6": 3.5, "joint7": 4.2,
}


def _parse_joint_limits(urdf_text: str) -> Dict[str, Tuple[float, float]]:
    """Extract (lower, upper) of the revolute joints from the URDF text."""
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
    """Raised-cosine smoothing: returns (s(t), s'(t)).

    s(0)=s'(0)=0, s(duration)=1, s'(duration)=0.
    """
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
        # Index by name: DDS does not guarantee that the name and position
        # order matches what we expect; the measured order is [joint2, joint3,
        # joint5, joint6, joint1, joint4, joint7].
        for name, position in zip(msg.name, msg.position):
            self.live[name] = position

    # ── reading ──

    def wait_for_joint_states(self, timeout: float = 30.0) -> Optional[Dict[str, float]]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(name in self.live for name in JOINTS):
                return {name: self.live[name] for name in JOINTS}
        return None

    def fetch_limits(self) -> Optional[Dict[str, Tuple[float, float]]]:
        """Parse the joint limits from the robot_description parameter of
        /robot_state_publisher.

        The parameter is a whole URDF text. Returns None when it cannot be
        obtained, and the caller falls back to the built-in amplitudes.
        Querying only this node is intentional: the single source of truth for
        the limits is the URDF, and RSP happens to hold it as a parameter, so
        there is no need to start an extra service or read a file.
        """
        if not self.args.use_urdf_limits:
            return None
        node_name = "/robot_state_publisher"
        known = [n.lstrip("/") for n in self.get_node_names()]
        if node_name.lstrip("/") not in known:
            self.get_logger().debug(
                f"{node_name} not found, skipping URDF limit parsing")
            return None
        try:
            from rcl_interfaces.srv import GetParameters
            client = self.create_client(GetParameters, f"{node_name}/get_parameters")
            if not client.wait_for_service(timeout_sec=3.0):
                raise RuntimeError("the get_parameters service is unavailable")
            request = GetParameters.Request()
            request.names = ["robot_description"]
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            result = future.result()
            if result is None or not result.values:
                raise RuntimeError("robot_description was not retrieved")
            text = result.values[0].string_value
            if not text:
                raise RuntimeError("robot_description is empty")
            return _parse_joint_limits(text)
        except Exception as exc:  # fall back to the built-in amplitudes; not
            # worth failing the demo over
            self.get_logger().warn(
                f"failed to parse the URDF joint limits ({exc}), falling back "
                f"to the built-in amplitudes")
            return None

    # ── planning the swing parameters ──

    def plan_swing(self, current: Dict[str, float],
                   limits: Optional[Dict[str, Tuple[float, float]]]
                   ) -> Tuple[Dict[str, float], Dict[str, float]]:
        """Decide the swing centre and amplitude of every axis; returns
        (center, amplitude).

        Centre pull-in rule: we want center±amp to stay inside the limits, i.e.
        ``margin(center) = min(upper-center, center-lower) >= amp/margin_fraction``.
        Clamping center into [lower+need, upper-need] satisfies that; when the
        interval is empty, the travel itself cannot hold the target amplitude,
        so fall back to the midpoint of the travel and cap the amplitude at
        half the travel.

        That way even a pose sitting against a limit (this arm's J4 zero pose
        is only 1° from the upper limit) lets the axis take part in the swing;
        the only cost is one extra smooth lead segment from the current
        position to the centre.
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
                # the travel cannot hold desired: fall back to the midpoint and
                # squeeze the amplitude to "the fraction of available margin"
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

    # ── trajectory construction ──

    def build_trajectory(self, start: Dict[str, float], center: Dict[str, float],
                         amplitude: Dict[str, float], phase: float) -> JointTrajectory:
        """The trajectory of one round trip (``periods`` full cycles).

        The position is the sum of three parts:

            pos(t) = center + amp·e(t)·sin(ωt+φ) + (start − center)·(1 − s(t))

        * ``e`` is the swing envelope and ``s`` the lead envelope; both are
          raised cosines with value/derivative 0 at either end.
        * So pos=start and vel=0 at t=0, and pos=center and vel=0 at t=total.
        * The first round takes ``start`` from the measured configuration (a
          smooth take-off from the real position) and later rounds use
          start=center, so they can be sent back to back without a seam.

        The velocity must carry the envelope derivative terms
        (``amp·e'·sin`` and ``(start−center)·s'``): during the lead segment they
        are comparable in size to the main term, and leaving them out skews
        the MIT dq_ref noticeably.
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
            # swing envelope: rises to 1 after the lead segment and falls back
            # to 0 over the final segment
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

    # ── execution ──

    def send(self, trajectory: JointTrajectory) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = Duration(sec=2, nanosec=0)

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=20)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("trajectory goal rejected")
            return False
        result_future = handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=1.0)
        wrapped = result_future.result()
        if wrapped is None:
            self.get_logger().error("no execution result received")
            return False
        code = wrapped.result.error_code
        if code != 0:
            self.get_logger().error(
                f"trajectory execution failed: error_code={code}"
                + (" (-4 = JTC aborted because "
                   "constraints.<joint>.trajectory judged the tracking error "
                   "too large; increasing --period or reducing --scale "
                   "mitigates it)" if code == -4 else ""))
            return False
        return True

    def run(self) -> int:
        args = self.args
        self.get_logger().info("waiting for /joint_trajectory_controller …")
        if not self.client.wait_for_server(timeout_sec=60.0):
            self.get_logger().error(
                "/joint_trajectory_controller is unavailable. Bring up the "
                "control stack first:\n"
                "  ros2 launch litearm_ros2_control litearm_demo.launch.py")
            return 1

        current = self.wait_for_joint_states()
        if current is None:
            self.get_logger().error(
                "/joint_states did not provide all 7 joints before the timeout")
            return 1

        limits = self.fetch_limits()
        center, amplitude = self.plan_swing(current, limits)

        self.get_logger().info(
            f"current configuration: {[round(current[n], 4) for n in JOINTS]}")
        self.get_logger().info(
            f"swing centre:  {[round(center[n], 4) for n in JOINTS]}")
        self.get_logger().info(
            f"amplitude:     {[round(amplitude[n], 4) for n in JOINTS]}")
        if limits is None:
            self.get_logger().warn(
                "URDF joint limits unavailable, using the built-in "
                "conservative amplitudes (no limit tightening applied)")
        if self.shifted:
            self.get_logger().info(
                f"centre pulled in on these axes to leave room for the swing "
                f"(the opening uses the {args.ramp:g}s lead segment): "
                f"{self.shifted}")
        if self.clamped:
            self.get_logger().warn(
                f"amplitude tightened by the limits on these axes: "
                f"{self.clamped}")
        if self.frozen:
            self.get_logger().warn(
                f"available margin below {args.min_amplitude} rad on these "
                f"axes, staying put: {self.frozen}")
        if all(value == 0.0 for value in amplitude.values()):
            self.get_logger().error(
                "every axis has zero amplitude, nothing to demo. Check whether "
                "the starting configuration is hard against a limit.")
            return 1
        self.get_logger().info(
            f"period {args.period:g}s × {args.periods} full cycles/round, "
            f"Ctrl-C to stop")

        cycle = 0
        start = dict(current)  # the first round takes off from the measured
        # configuration, later ones from the centre
        try:
            while rclpy.ok():
                # consecutive rounds differ in phase by π: the swing mirrors
                # back and forth, which is easier to see moving
                phase = (cycle % 2) * math.pi
                trajectory = self.build_trajectory(start, center, amplitude, phase)
                self.get_logger().info(
                    f"swing round {cycle + 1} "
                    f"({args.period * args.periods:g}s)…")
                if not self.send(trajectory):
                    return 1
                cycle += 1
                start = dict(center)  # the previous round ended at center, so
                # this one continues seamlessly
                if args.cycles and cycle >= args.cycles:
                    break
        except KeyboardInterrupt:
            self.get_logger().info("interrupted, stopping the demo")
        self.get_logger().info(f"demo finished, {cycle} rounds in total")
        return 0


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scale", type=float, default=1.0,
                        help="overall amplitude scale factor (default 1.0)")
    parser.add_argument("--period", type=float, default=12.0,
                        help="seconds per swing cycle (default 12; a smaller "
                             "value makes JTC abort on tracking error)")
    parser.add_argument("--periods", type=int, default=2,
                        help="number of full cycles per round (default 2)")
    parser.add_argument("--ramp", type=float, default=3.0,
                        help="raised-cosine duration in seconds for the "
                             "start/stop and lead segments (default 3)")
    parser.add_argument("--sample-period", type=float, default=0.1,
                        help="trajectory sample interval in seconds "
                             "(default 0.1)")
    parser.add_argument("--margin-fraction", type=float, default=0.6,
                        help="fraction of the available limit margin to use "
                             "(default 0.6)")
    parser.add_argument("--min-amplitude", type=float, default=0.03,
                        help="axes below this amplitude (rad) stay put "
                             "(default 0.03)")
    parser.add_argument("--cycles", type=int, default=0,
                        help="number of swing rounds, 0 = loop forever until "
                             "Ctrl-C")
    parser.add_argument("--no-urdf-limits", dest="use_urdf_limits",
                        action="store_false", default=True,
                        help="do not parse the URDF limits, use the built-in "
                             "amplitudes directly")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    if args.period <= 0 or args.periods < 1 or args.scale <= 0 or args.ramp <= 0:
        print("--period/--ramp/--scale must be >0, --periods must be >=1",
              file=sys.stderr)
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
