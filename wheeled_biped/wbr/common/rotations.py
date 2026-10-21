"""Rotation helpers. Quaternions are (w, x, y, z) as in MuJoCo and upkie."""
from __future__ import annotations

import math

import numpy as np


def quat_to_matrix(q) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(R: np.ndarray) -> np.ndarray:
    m = R
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def rot_x(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def pitch_frame_in_parent(R_frame_to_parent: np.ndarray) -> float:
    """Port of upkie's compute_pitch_frame_in_parent (BaseOrientation.h).

    Angle from the parent z-axis to the frame z-axis, positive when the frame leans forward
    (its x-axis points down). Returns radians.
    """
    sagittal = R_frame_to_parent[:, 0].copy()
    sagittal /= np.linalg.norm(sagittal)
    heading = sagittal - sagittal[2] * np.array([0.0, 0.0, 1.0])
    n = np.linalg.norm(heading)
    if n < 1e-12:
        return math.copysign(math.pi / 2, -sagittal[2])
    heading /= n
    if R_frame_to_parent[2, 2] < 0.0:
        heading *= -1.0
    sign = 1.0 if sagittal[2] < 0.0 else -1.0
    cos_pitch = float(np.clip(sagittal.dot(heading), -1.0, 1.0))
    return sign * math.acos(cos_pitch)


def roll_pitch_yaw(R: np.ndarray):
    """ZYX Euler angles (roll about x, pitch about y, yaw about z) of a rotation matrix."""
    pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def quat_to_rpy(q):
    return roll_pitch_yaw(quat_to_matrix(q))


def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi
