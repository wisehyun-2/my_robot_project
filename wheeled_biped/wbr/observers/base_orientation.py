"""IMU frame adapter of the MCU: base orientation from the pi3hat's quaternion.  Port of upkie's spine observer
BaseOrientation (upkie/cpp/observers/BaseOrientation.h, Apache-2.0).

BaseOrientation : pitch of the base w.r.t. the world vertical (positive = leaning forward) and
                  body angular velocity, from an IMU quaternion (orientation of the IMU in the ARS frame)
"""
from __future__ import annotations

import numpy as np

from wbr.common.rotations import pitch_frame_in_parent, quat_to_matrix


class BaseOrientation:
    """R_base_to_world = R_ars_to_world @ R_imu_to_ars @ R_base_to_imu.

    rotation_base_to_imu: the board's mount on the robot (v5: diag(-1, -1, 1), description.json; there is no default
    mount: None = identity, the board axes along the base axes).  rotation_ars_to_world: the pi3hat reports its orientation
    in the ARS frame (x forward, y right, z down), so the default is diag(1, -1, -1).
    """

    def __init__(self, rotation_base_to_imu: np.ndarray | None = None, rotation_ars_to_world: np.ndarray | None = None):
        self.R_base_to_imu = np.eye(3) if rotation_base_to_imu is None else np.asarray(rotation_base_to_imu, float)
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
