#!/usr/bin/env python3
"""litearm_node — ROS2 (Humble) bridge for the LiteArm robotic arm.

Publishes:
    /joint_states            sensor_msgs/JointState     (q / dq / tau)
    /litearm/tcp_pose        geometry_msgs/PoseStamped  (TCP pose in base)
    /tf                      base_link -> tool0

Subscribes:
    /litearm/cmd_joint       sensor_msgs/JointState     (position -> movej)
    /litearm/stop            std_msgs/Empty             (emergency stop)

Services:
    /litearm/movej           litearm_ros2/Movej
    /litearm/movel           litearm_ros2/Movel
    /litearm/fk              litearm_ros2/Fk
    /litearm/ik              litearm_ros2/Ik
    /litearm/get_state       litearm_ros2/GetState
    /litearm/request_stop    std_srvs/Trigger
    /litearm/clear_stop      std_srvs/Trigger
    /litearm/enable          std_srvs/Trigger
    /litearm/disable         std_srvs/Trigger
"""
from __future__ import annotations

from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
import tf2_ros

import litearm
import rclpy
from rclpy.node import Node

from litearm_ros2.bridge import LiteArmBridge
from litearm_interfaces.srv import Fk, GetState, Ik, Movej, Movel


class LiteArmNode(Node):
    def __init__(self, endpoint: str = None, arm_id: str = None, loop_hz: float = None,
                 joint_names=None, base_frame: str = None,
                 tcp_frame: str = None, cmd_speed: float = None) -> None:
        super().__init__("litearm_node")
        self._declare_params()
        # command-line values win; else fall back to ROS parameters
        endpoint = endpoint or self._param("endpoint", "tcp/127.0.0.1:7447")
        arm_id = arm_id or self._param("arm_id", "armA")
        loop_hz = float(loop_hz if loop_hz is not None else self._param("loop_hz", 50.0))
        cmd_speed = float(cmd_speed if cmd_speed is not None else self._param("cmd_speed", 0.5))
        jn = self._param("joint_names", "joint0 joint1 joint2 joint3 joint4 joint5 joint6")
        joint_names = joint_names or (jn.split() if isinstance(jn, str) else list(jn))
        base_frame = base_frame or self._param("base_frame", "base_link")
        tcp_frame = tcp_frame or self._param("tcp_frame", "tool0")

        self.loop_hz = loop_hz
        self.base_frame = base_frame
        self.tcp_frame = tcp_frame
        self.cmd_speed = cmd_speed
        self._joint_names = joint_names or [f"joint{i}" for i in range(7)]

        arm = litearm.Arm(endpoint=endpoint, arm_id=arm_id)
        self.bridge = LiteArmBridge(arm, joint_names=self._joint_names)

        # publishers
        self.pub_joint = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_pose = self.create_publisher(PoseStamped, "/litearm/tcp_pose", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # subscribers
        self.create_subscription(JointState, "/litearm/cmd_joint", self._on_cmd_joint, 10)
        self.create_subscription(Empty, "/litearm/stop", self._on_stop, 10)

        # services
        self.create_service(Movej, "/litearm/movej", self._srv_movej)
        self.create_service(Movel, "/litearm/movel", self._srv_movel)
        self.create_service(Fk, "/litearm/fk", self._srv_fk)
        self.create_service(Ik, "/litearm/ik", self._srv_ik)
        self.create_service(GetState, "/litearm/get_state", self._srv_get_state)
        self.create_service(Trigger, "/litearm/request_stop", self._srv_request_stop)
        self.create_service(Trigger, "/litearm/clear_stop", self._srv_clear_stop)
        self.create_service(Trigger, "/litearm/enable", self._srv_enable)
        self.create_service(Trigger, "/litearm/disable", self._srv_disable)

        self.timer = self.create_timer(1.0 / self.loop_hz, self._publish_state)

    def _declare_params(self) -> None:
        self.declare_parameter("endpoint", "tcp/127.0.0.1:7447")
        self.declare_parameter("arm_id", "armA")
        self.declare_parameter("loop_hz", 50.0)
        self.declare_parameter("cmd_speed", 0.5)
        self.declare_parameter("joint_names", "joint0 joint1 joint2 joint3 joint4 joint5 joint6")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tcp_frame", "tool0")

    def _param(self, name: str, default):
        try:
            return self.get_parameter(name).get_parameter_value().get_value()
        except Exception:
            return default

    # ── publishers ───────────────────────────────────────────────────────────

    def _publish_state(self) -> None:
        js = self.bridge.read_joint_state()
        if js is None:
            return
        stamp = self.get_clock().now().to_msg()
        msg = JointState()
        msg.header.stamp = stamp
        msg.header.frame_id = self.base_frame
        msg.name = js["name"]
        msg.position = js["position"]
        msg.velocity = js["velocity"]
        msg.effort = js["effort"]
        self.pub_joint.publish(msg)

        pose = self.bridge.read_tcp_pose()
        if pose is None:
            return
        pmsg = PoseStamped()
        pmsg.header.stamp = stamp
        pmsg.header.frame_id = self.base_frame
        _fill_pose(pmsg.pose, pose)
        self.pub_pose.publish(pmsg)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.tcp_frame
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = pose[0:3]
        t.transform.rotation.x, t.transform.rotation.y = pose[3], pose[4]
        t.transform.rotation.z, t.transform.rotation.w = pose[5], pose[6]
        self.tf_broadcaster.sendTransform(t)

    # ── subscribers ──────────────────────────────────────────────────────────

    def _on_cmd_joint(self, msg: JointState) -> None:
        if not msg.position:
            return
        ok, message = self.bridge.movej(msg.position, speed=self.cmd_speed)
        if not ok:
            self.get_logger().warn(f"movej failed: {message}")

    def _on_stop(self, _msg: Empty) -> None:
        ok, message = self.bridge.request_stop()
        self.get_logger().info(f"emergency stop: ok={ok} ({message})")

    # ── services ─────────────────────────────────────────────────────────────

    def _srv_movej(self, req, resp):
        ok, message = self.bridge.movej(req.q_target, speed=req.speed, settle_s=req.settle_s)
        resp.success, resp.message = ok, message
        return resp

    def _srv_movel(self, req, resp):
        ok, message = self.bridge.movel(_pose_to_xyz_quat(req.pose), speed=req.speed,
                                        settle_s=req.settle_s)
        resp.success, resp.message = ok, message
        return resp

    def _srv_fk(self, req, resp):
        pose, ok, message = self.bridge.fk(req.q)
        resp.success, resp.message = ok, message
        if pose is not None:
            _fill_pose(resp.pose, pose)
        return resp

    def _srv_ik(self, req, resp):
        q, ok, message = self.bridge.ik(_pose_to_xyz_quat(req.pose), q_seed=req.q_seed)
        resp.success, resp.message = ok, message
        resp.q = q or []
        return resp

    def _srv_get_state(self, req, resp):
        state = self.bridge.get_state()
        if state is None:
            resp.message = "no state yet"
            return resp
        resp.q, resp.dq, resp.tau = state["q"], state["dq"], state["tau"]
        resp.state = state["state"]
        resp.fault = state["fault"]
        resp.message = "ok"
        return resp

    def _srv_request_stop(self, req, resp):
        ok, message = self.bridge.request_stop()
        resp.success, resp.message = ok, message
        return resp

    def _srv_clear_stop(self, req, resp):
        ok, message = self.bridge.clear_stop()
        resp.success, resp.message = ok, message
        return resp

    def _srv_enable(self, req, resp):
        ok, message = self.bridge.enable()
        resp.success, resp.message = ok, message
        return resp

    def _srv_disable(self, req, resp):
        ok, message = self.bridge.disable()
        resp.success, resp.message = ok, message
        return resp


def _pose_to_xyz_quat(pose: Pose):
    return (pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)


def _fill_pose(pose: Pose, xyz_quat) -> None:
    pose.position.x, pose.position.y, pose.position.z = xyz_quat[0:3]
    pose.orientation.x, pose.orientation.y = xyz_quat[3], xyz_quat[4]
    pose.orientation.z, pose.orientation.w = xyz_quat[5], xyz_quat[6]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LiteArmNode(endpoint=None, arm_id=None)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.bridge.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
