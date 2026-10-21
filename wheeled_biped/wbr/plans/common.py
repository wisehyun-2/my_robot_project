"""Shared pieces of the plan assemblies: sensor noise/latency emulation, logging, base placement."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List

import numpy as np

from wbr.common.filters import DelayLine
from wbr.common.rotations import matrix_to_quat, quat_to_matrix, rot_y


@dataclass
class SensorNoise:
    """Noise levels of the design report (sec. 8.2): pitch 0.2 deg, gyro 0.5 deg/s, 14-bit encoders."""
    enabled: bool = False
    pitch_std: float = math.radians(0.2)
    gyro_std: float = math.radians(0.5)
    accel_std: float = 0.05
    encoder_bits: int = 14
    joint_velocity_std: float = math.radians(0.5)
    seed: int = 0

    def __post_init__(self):
        self.rng = np.random.default_rng(self.seed)

    def apply(self, obs: dict, dt: float) -> dict:
        if not self.enabled:
            return obs
        q = obs["imu"]["orientation"]
        R = quat_to_matrix(q)
        R = R @ rot_y(self.rng.normal(0.0, self.pitch_std))
        obs["imu"]["orientation"] = matrix_to_quat(R)
        obs["imu"]["angular_velocity"] = obs["imu"]["angular_velocity"] + self.rng.normal(0.0, self.gyro_std, 3)
        obs["imu"]["linear_acceleration"] = obs["imu"]["linear_acceleration"] + self.rng.normal(0.0, self.accel_std, 3)
        quantum = 2 * math.pi / (2 ** self.encoder_bits)
        for s in obs["servo"].values():
            s["position"] = round(s["position"] / quantum) * quantum
            s["velocity"] = s["velocity"] + self.rng.uniform(-1, 1) * quantum / dt + self.rng.normal(0.0, self.joint_velocity_std)
        return obs


class ObservationDelay:
    """Delays the whole observation dictionary by an integer number of control steps."""

    def __init__(self, delay_steps: int):
        self.line = None
        self.delay_steps = delay_steps

    def push(self, obs: dict) -> dict:
        if self.delay_steps <= 0:
            return obs
        if self.line is None:
            self.line = DelayLine(self.delay_steps, obs)
        return self.line.push(obs)


class Logger:
    def __init__(self):
        self.rows: Dict[str, List[float]] = {}

    def log(self, **kw):
        for k, v in kw.items():
            self.rows.setdefault(k, []).append(float(v))

    def arrays(self) -> Dict[str, np.ndarray]:
        return {k: np.asarray(v) for k, v in self.rows.items()}

    def __getitem__(self, k):
        return np.asarray(self.rows[k])


def placed_base_pose(wheel_offset_in_base: np.ndarray, wheel_radius: float, pitch: float, yaw: float = 0.0,
                     xy=(0.0, 0.0)):
    """Base position/quaternion such that the wheels touch the floor when the base is pitched by `pitch`
    (positive = leaning forward, rotation about +y) and yawed by `yaw`."""
    c, s = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    R = Rz @ rot_y(pitch)
    wheel_world = R @ wheel_offset_in_base
    pos = np.array([xy[0] - wheel_world[0], xy[1] - wheel_world[1], -wheel_world[2] + wheel_radius])
    return pos, matrix_to_quat(R)
