"""Port of upkie's spine IMU frame observer (upkie/cpp/observers, Apache-2.0), used by the MCU as its IMU convention adapter.

BaseOrientation : pitch of the base w.r.t. the world vertical (positive = leaning forward) and
                  body angular velocity, from an IMU quaternion (orientation of the IMU in the ARS frame)

The other upkie observers (wheel contact, floor contact, wheel odometry) served only Plan 1 and were removed
with Plans 1-3 on 2026-11-11 (git history).
"""
from __future__ import annotations

import numpy as np

from wbr.common.rotations import pitch_frame_in_parent, quat_to_matrix


class BaseOrientation:
    """R_base_to_world = R_ars_to_world @ R_imu_to_ars @ R_base_to_imu (upkie BaseOrientation.h).

    Defaults are upkie's real-robot values: the pi3hat IMU is mounted rotated (R_base_to_imu =
    diag(-1, 1, -1)) and reports its orientation in the ARS frame (x forward, y right, z down:
    R_ars_to_world = diag(1, -1, -1)).  The MuJoCo plans pass identity matrices because their IMU
    sensor already reports the base orientation in the world frame.
    """

    def __init__(self, rotation_base_to_imu: np.ndarray | None = None, rotation_ars_to_world: np.ndarray | None = None):
        self.R_base_to_imu = np.diag([-1.0, 1.0, -1.0]) if rotation_base_to_imu is None else np.asarray(rotation_base_to_imu, float)
        self.R_ars_to_world = np.diag([1.0, -1.0, -1.0]) if rotation_ars_to_world is None else np.asarray(rotation_ars_to_world, float)
        self.pitch = 0.0
        self.angular_velocity = np.zeros(3)
        self.rotation_base_to_world = np.eye(3)

    def update(self, quat_imu_in_ars, angular_velocity_imu) -> dict:
        R_imu_to_ars = quat_to_matrix(quat_imu_in_ars)
        self.rotation_base_to_world = self.R_ars_to_world @ R_imu_to_ars @ self.R_base_to_imu
        self.pitch = pitch_frame_in_parent(self.rotation_base_to_world)
        self.angular_velocity = self.R_base_to_imu.T @ np.asarray(angular_velocity_imu, dtype=float)
        return {"pitch": self.pitch, "angular_velocity": self.angular_velocity,
                "rotation_base_to_world": self.rotation_base_to_world}
