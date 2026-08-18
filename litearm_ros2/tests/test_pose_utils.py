"""Tests for the pure-python pose conversions."""
from __future__ import annotations

import math

import pytest

from litearm_ros2.pose_utils import (
    litearm_pose_to_xyz_quat,
    quat_to_rot_matrix,
    rot_matrix_to_quat,
    xyz_quat_to_litearm_pose,
)


def _close(a, b, tol=1e-9):
    return abs(a - b) < tol


def test_identity_rotation():
    R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    q = rot_matrix_to_quat(R)
    assert _close(q[0], 0) and _close(q[1], 0) and _close(q[2], 0) and _close(q[3], 1)
    R2 = quat_to_rot_matrix(q)
    for r1, r2 in zip(R, R2):
        for a, b in zip(r1, r2):
            assert _close(a, b)


def test_90_deg_about_z():
    # Rz(90°): x -> y
    R = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    q = rot_matrix_to_quat(R)
    # quaternion (0,0,sin45,cos45)
    assert _close(q[0], 0.0) and _close(q[1], 0.0)
    assert _close(q[2], math.sin(math.pi / 4), 1e-9)
    assert _close(q[3], math.cos(math.pi / 4), 1e-9)
    R2 = quat_to_rot_matrix(q)
    for r1, r2 in zip(R, R2):
        for a, b in zip(r1, r2):
            assert _close(a, b, 1e-9)


def test_roundtrip_random_quat():
    raw = (0.2, -0.3, 0.4, 0.7)  # not unit-length
    norm = math.sqrt(sum(v * v for v in raw))
    expected = tuple(v / norm for v in raw)
    R = quat_to_rot_matrix(raw)
    q2 = rot_matrix_to_quat(R)
    for a, b in zip(q2, expected):
        assert _close(a, b, 1e-6)


def test_xyz_quat_to_litearm_pose():
    pose = xyz_quat_to_litearm_pose([1, 2, 3, 0, 0, 0, 1])
    assert pose[0] == [1.0, 2.0, 3.0]
    assert pose[1] == [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]


def test_litearm_pose_to_xyz_quat():
    out = litearm_pose_to_xyz_quat([[1, 2, 3], [[1, 0, 0], [0, 1, 0], [0, 0, 1]]])
    assert out == (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)


def test_roundtrip_pose():
    xyz_quat = (0.1, 0.2, 0.3, 0.1, 0.2, 0.3, 0.9)  # quaternion gets normalized
    q = (0.1, 0.2, 0.3, 0.9)
    norm = math.sqrt(sum(v * v for v in q))
    expected = (0.1, 0.2, 0.3) + tuple(v / norm for v in q)
    pose = xyz_quat_to_litearm_pose(xyz_quat)
    back = litearm_pose_to_xyz_quat(pose)
    for a, b in zip(back, expected):
        assert _close(a, b, 1e-6)


def test_xyz_quat_wrong_length():
    with pytest.raises(ValueError):
        xyz_quat_to_litearm_pose([1, 2, 3, 0, 0, 0])
