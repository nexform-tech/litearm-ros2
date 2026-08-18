"""Unit tests for the ROS-agnostic LiteArmBridge (no ROS/rclpy needed)."""
from __future__ import annotations

import pytest

from litearm_ros2.bridge import LiteArmBridge


@pytest.fixture
def bridge(fake_arm):
    return LiteArmBridge(fake_arm)


def test_read_joint_state(bridge, fake_arm):
    fake_arm.q = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
    fake_arm.dq = [0.0] * 7
    js = bridge.read_joint_state()
    assert js["name"] == [f"joint{i}" for i in range(7)]
    assert js["position"] == pytest.approx([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
    assert len(js["velocity"]) == 7
    assert len(js["effort"]) == 7


def test_read_tcp_pose(bridge):
    pose = bridge.read_tcp_pose()
    assert len(pose) == 7
    assert pose[:3] == pytest.approx((0.3, 0.0, 0.4))
    # identity rotation
    assert pose[3:] == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_get_state(bridge, fake_arm):
    fake_arm.state = "motion"
    fake_arm.fault = False
    s = bridge.get_state()
    assert s["q"] == [0.0] * 7
    assert s["state"] == "motion"
    assert s["fault"] is False


def test_movej(bridge, fake_arm):
    ok, msg = bridge.movej([0.5] * 7)
    assert ok is True
    assert fake_arm.calls[-1][0] == "movej"


def test_movej_error(bridge, fake_arm):
    def boom(*a, **kw):
        raise RuntimeError("motor fault")
    fake_arm.movej = boom
    ok, msg = bridge.movej([0.1] * 7)
    assert ok is False
    assert "motor fault" in msg


def test_movel(bridge, fake_arm):
    ok, msg = bridge.movel((0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0))
    assert ok is True
    assert fake_arm.calls[-1][0] == "movel"


def test_fk(bridge, fake_arm):
    pose, ok, msg = bridge.fk([0.0] * 7)
    assert ok is True
    assert len(pose) == 7
    assert pose[:3] == pytest.approx((0.3, 0.0, 0.4))


def test_ik(bridge, fake_arm):
    q, ok, msg = bridge.ik((0.3, 0.0, 0.4, 1.0, 0.0, 0.0, 0.0), q_seed=[0.0] * 7)
    assert ok is True
    assert len(q) == 7


def test_stop_lifecycle(bridge, fake_arm):
    ok, _ = bridge.request_stop()
    assert ok and fake_arm.stopped
    ok, _ = bridge.clear_stop()
    assert ok and not fake_arm.stopped


def test_enable_disable(bridge, fake_arm):
    bridge.disable()
    assert not fake_arm.enabled
    bridge.enable()
    assert fake_arm.enabled


def test_close(bridge, fake_arm):
    bridge.close()
    assert fake_arm.calls[-1][0] == "close"
