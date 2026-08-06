"""二维位姿数学工具。"""

import math
from typing import Tuple


def normalize_angle(angle: float) -> float:
    """将角度归一化到 [-pi, pi]。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> float:
    """将四元数转为 yaw（rad）。"""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """将 yaw（rad）转为 (qx, qy, qz, qw)。"""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))
