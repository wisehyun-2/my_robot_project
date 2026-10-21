"""Ports of the upkie spine observers (upkie/cpp/observers, Apache-2.0).

BaseOrientation : pitch of the base w.r.t. the world vertical (positive = leaning forward) and
                  body angular velocity, from an IMU quaternion (orientation of the base in the world)
WheelContact    : contact detection from the apparent inertia |torque| / |acceleration| of a wheel
FloorContact    : robot-level contact flag (any wheel contact, or large upper-leg torques)
WheelOdometry   : ground position/velocity integrated from wheel velocities while in contact

All update() methods take plain dictionaries shaped like the MuJoCo sim observation.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict

import numpy as np

from wbr.common.filters import low_pass_filter
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


@dataclass
class WheelContactParams:
    liftoff_inertia: float = 0.001            # m^2 kg (hysteresis low)
    touchdown_inertia: float = 0.004          # m^2 kg (hysteresis high)
    min_touchdown_acceleration: float = 2.0   # rad/s^2
    min_touchdown_torque: float = 0.015       # N m (upkie spine_backend defaults)
    cutoff_period: float = 0.2                # s


class WheelContact:
    def __init__(self, params: WheelContactParams):
        self.p = params
        self.abs_acceleration = 0.0
        self.abs_torque = 0.0
        self.contact = False
        self.inertia = 0.0
        self.velocity = 0.0

    def observe(self, torque: float, velocity: float, dt: float) -> None:
        prev = self.velocity
        self.velocity = low_pass_filter(self.velocity, self.p.cutoff_period, velocity, dt)
        new_acc = (self.velocity - prev) / dt
        self.abs_acceleration = low_pass_filter(self.abs_acceleration, self.p.cutoff_period, abs(new_acc), dt)
        self.abs_torque = low_pass_filter(self.abs_torque, self.p.cutoff_period, abs(torque), dt)
        if not self.contact and (self.abs_acceleration < self.p.min_touchdown_acceleration
                                 or self.abs_torque < self.p.min_touchdown_torque):
            return
        self.inertia = self.abs_torque / (self.abs_acceleration + 1e-4)
        if self.inertia < self.p.liftoff_inertia:
            self.contact = False
        elif self.inertia > self.p.touchdown_inertia:
            self.contact = True


@dataclass
class FloorContactParams:
    dt: float = 1e-3
    upper_leg_torque_threshold: float = 10.0
    wheel_contact: WheelContactParams = field(default_factory=WheelContactParams)
    upper_leg_joints: tuple = ("left_hip", "left_knee", "right_hip", "right_knee")
    wheel_joints: tuple = ("left_wheel", "right_wheel")


class FloorContact:
    def __init__(self, params: FloorContactParams):
        self.p = params
        self.wheels = {w: WheelContact(params.wheel_contact) for w in params.wheel_joints}
        self.upper_leg_torque = 0.0
        self.contact = False

    def update(self, servo: Dict[str, dict]) -> dict:
        any_contact = False
        for w, wc in self.wheels.items():
            wc.observe(servo[w]["torque"], servo[w]["velocity"], self.p.dt)
            any_contact = any_contact or wc.contact
        sq = sum(servo[j]["torque"] ** 2 for j in self.p.upper_leg_joints)
        self.upper_leg_torque = low_pass_filter(self.upper_leg_torque, 0.01, math.sqrt(sq), self.p.dt)
        self.contact = any_contact or (self.upper_leg_torque > self.p.upper_leg_torque_threshold)
        out = {"contact": self.contact, "upper_leg_torque": self.upper_leg_torque}
        for w, wc in self.wheels.items():
            out[w] = {"contact": wc.contact, "inertia": wc.inertia, "abs_torque": wc.abs_torque,
                      "abs_acceleration": wc.abs_acceleration}
        return out


class WheelOdometry:
    def __init__(self, signed_radius: Dict[str, float], dt: float):
        self.signed_radius = signed_radius
        self.dt = dt
        self.position = 0.0
        self.velocity = 0.0

    def reset(self) -> None:
        self.position = 0.0
        self.velocity = 0.0

    def update(self, floor_contact: dict, servo: Dict[str, dict]) -> dict:
        if floor_contact["contact"]:
            vsum, n = 0.0, 0
            for w, r in self.signed_radius.items():
                if floor_contact[w]["contact"]:
                    vsum += r * servo[w]["velocity"]
                    n += 1
            self.velocity = vsum / n if n else 0.0
            self.position += self.velocity * self.dt
        return {"position": self.position, "velocity": self.velocity}
