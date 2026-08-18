"""Pure-python pose conversions between litearm and ROS conventions.

No numpy and no ROS imports so the module is unit-testable on any machine.
litearm pose  = [position(3), rotation(3x3 row-major matrix)]
ROS pose      = (x, y, z, qx, qy, qz, qw)
"""
from __future__ import annotations

import math
from typing import List, Sequence, Tuple


def rot_matrix_to_quat(R: Sequence[Sequence[float]]) -> Tuple[float, float, float, float]:
    """3x3 row-major rotation matrix -> (qx, qy, qz, qw)."""
    m00, m01, m02 = R[0]
    m10, m11, m12 = R[1]
    m20, m21, m22 = R[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    return _normalize_quat(x, y, z, w)


def quat_to_rot_matrix(q: Sequence[float]) -> List[List[float]]:
    """(qx, qy, qz, qw) -> 3x3 row-major rotation matrix."""
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]


def xyz_quat_to_litearm_pose(xyz_quat: Sequence[float]) -> List:
    """(x, y, z, qx, qy, qz, qw) -> litearm [position(3), rotation(3x3)]."""
    if len(xyz_quat) != 7:
        raise ValueError(f"xyz+quat must have 7 elements, got {len(xyz_quat)}")
    x, y, z, qx, qy, qz, qw = (float(v) for v in xyz_quat)
    return [[x, y, z], quat_to_rot_matrix([qx, qy, qz, qw])]


def litearm_pose_to_xyz_quat(pose: Sequence) -> Tuple[float, float, float, float, float, float, float]:
    """litearm [position(3), rotation(3x3)] -> (x, y, z, qx, qy, qz, qw)."""
    pos, rot = pose
    x, y, z = (float(v) for v in pos)
    qx, qy, qz, qw = rot_matrix_to_quat(rot)
    return x, y, z, qx, qy, qz, qw


def _normalize_quat(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    return x / n, y / n, z / n, w / n
