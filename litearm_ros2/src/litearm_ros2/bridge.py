"""ROS-agnostic bridge logic: LiteArm (litearm-python) <-> ROS message data.

No ``rclpy``/``rospy`` imports on purpose — every method works on plain dicts
and tuples so it can be unit-tested on any machine.
"""
from __future__ import annotations

from typing import Dict, List, Optional, Tuple

from .pose_utils import litearm_pose_to_xyz_quat, xyz_quat_to_litearm_pose


class LiteArmBridge:
    """Adapt a litearm-python ``Arm`` to ROS-friendly plain data."""

    def __init__(self, arm, joint_names: Optional[List[str]] = None) -> None:
        self.arm = arm
        self.joint_names = joint_names or [f"joint{i}" for i in range(7)]

    # ── Reading ──────────────────────────────────────────────────────────────

    def read_joint_state(self) -> Optional[Dict]:
        """sensor_msgs/JointState fields, or None before the first broadcast."""
        state = self.arm.get_state()
        if state is None:
            return None
        return {
            "name": list(self.joint_names),
            "position": [float(v) for v in state.get("q", [])],
            "velocity": [float(v) for v in state.get("dq", [])],
            "effort": [float(v) for v in state.get("tau", [])],
        }

    def read_tcp_pose(self) -> Optional[Tuple[float, float, float, float, float, float, float]]:
        """TCP pose as (x, y, z, qx, qy, qz, qw), or None on error."""
        try:
            pose = self.arm.get_tcp_pose()
        except Exception:
            return None
        return litearm_pose_to_xyz_quat(pose)

    def get_state(self) -> Optional[Dict]:
        state = self.arm.get_state()
        if state is None:
            return None
        return {
            "q": [float(v) for v in state.get("q", [])],
            "dq": [float(v) for v in state.get("dq", [])],
            "tau": [float(v) for v in state.get("tau", [])],
            "state": state.get("state", ""),
            "fault": bool(state.get("fault")),
        }

    # ── Commands ─────────────────────────────────────────────────────────────

    def movej(self, q, speed: float = 0.5, settle_s: float = 0.5) -> Tuple[bool, str]:
        try:
            self.arm.movej([float(v) for v in q], speed=speed, settle_s=settle_s)
            return True, "ok"
        except Exception as exc:
            return False, str(exc)

    def movel(self, xyz_quat, speed: float = 0.5, settle_s: float = 0.5) -> Tuple[bool, str]:
        try:
            pose = xyz_quat_to_litearm_pose(xyz_quat)
            self.arm.movel(pose, speed=speed, settle_s=settle_s)
            return True, "ok"
        except Exception as exc:
            return False, str(exc)

    def fk(self, q):
        """Return (pose_xyz_quat, success, message)."""
        try:
            pos, rot = self.arm.fk([float(v) for v in q])
            pose = litearm_pose_to_xyz_quat([pos, rot])
            return pose, True, "ok"
        except Exception as exc:
            return None, False, str(exc)

    def ik(self, xyz_quat, q_seed=None):
        """Return (q, success, message)."""
        try:
            pose = xyz_quat_to_litearm_pose(xyz_quat)
            q, success = self.arm.ik(pose, q_seed=list(q_seed) if q_seed else None)
            return [float(v) for v in q], bool(success), "ok"
        except Exception as exc:
            return None, False, str(exc)

    def request_stop(self) -> Tuple[bool, str]:
        try:
            self.arm.request_stop()
            return True, "ok"
        except Exception as exc:
            return False, str(exc)

    def clear_stop(self) -> Tuple[bool, str]:
        try:
            self.arm.clear_stop()
            return True, "ok"
        except Exception as exc:
            return False, str(exc)

    def enable(self) -> Tuple[bool, str]:
        try:
            self.arm.enable()
            return True, "ok"
        except Exception as exc:
            return False, str(exc)

    def disable(self) -> Tuple[bool, str]:
        try:
            self.arm.disable()
            return True, "ok"
        except Exception as exc:
            return False, str(exc)

    # ── Lifecycle ────────────────────────────────────────────────────────────

    def close(self) -> None:
        self.arm.close()
