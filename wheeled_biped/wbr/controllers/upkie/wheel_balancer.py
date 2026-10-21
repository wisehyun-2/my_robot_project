"""Port of upkie's WheelBalancer (upkie/cpp/controllers/WheelBalancer.cpp, Apache-2.0).

Balance by PI feedback on the base pitch and the ground position, output a ground velocity
that is converted to wheel velocity targets (wheels in velocity mode). The legs are held in
position mode with kp/kd scales that stiffen while turning.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from wbr.common.filters import clamp, low_pass_filter

K_AIR_RETURN_PERIOD = 1.0       # s
K_MAX_INTEGRAL_VELOCITY = 10.0  # m/s
K_MAX_TARGET_DISTANCE = 1.0     # m
K_GAIN_SCALE = 2.0
K_TURNING_GAIN_SCALE = 2.0


@dataclass
class WheelBalancerParams:
    contact_radius: float = 0.1524     # half wheel base [m]
    dt: float = 1e-3
    fall_pitch: float = 1.0            # rad
    max_ground_velocity: float = 2.0   # m/s
    pitch_damping: float = 1.8
    pitch_stiffness: float = 20.0
    position_damping: float = 0.7
    position_stiffness: float = 1.6
    stiff_yaw_velocity: float = 0.1
    wheel_radius: float = 0.05         # m (upkie_description tire cylinder)
    gain_scale: float = 1.0            # extra multiplier on all four PI gains (model adaptation)


class WheelBalancer:
    def __init__(self, params: WheelBalancerParams):
        self.p = params
        self.reset()

    def reset(self) -> None:
        self.ground_velocity = 0.0
        self.integral_velocity = 0.0
        self.target_ground_position = 0.0
        self.target_yaw_velocity = 0.0
        self.fallen = False

    def read(self, observation: dict, target_ground_velocity: float, target_yaw_velocity: float) -> None:
        p = self.p
        self.target_yaw_velocity = target_yaw_velocity
        pitch = observation["base_orientation"]["pitch"]
        self.fallen = abs(pitch) > p.fall_pitch
        if self.fallen:
            self.ground_velocity = 0.0
            return
        ground_position = observation["wheel_odometry"]["position"]
        floor_contact = observation["floor_contact"]["contact"]
        target_pitch = 0.0
        e_pos = self.target_ground_position - ground_position
        e_pitch = target_pitch - pitch
        kp_pos, kp_pitch = p.gain_scale * p.position_damping, p.gain_scale * p.pitch_damping
        ki_pos, ki_pitch = p.gain_scale * p.position_stiffness, p.gain_scale * p.pitch_stiffness
        dt = p.dt
        if floor_contact:
            self.integral_velocity += (ki_pos * e_pos + ki_pitch * e_pitch) * dt
            self.integral_velocity = clamp(self.integral_velocity, -K_MAX_INTEGRAL_VELOCITY, K_MAX_INTEGRAL_VELOCITY)
            self.target_ground_position += target_ground_velocity * dt
            self.target_ground_position = clamp(self.target_ground_position,
                                                ground_position - K_MAX_TARGET_DISTANCE,
                                                ground_position + K_MAX_TARGET_DISTANCE)
        else:
            self.integral_velocity = low_pass_filter(self.integral_velocity, K_AIR_RETURN_PERIOD, 0.0, dt)
            self.target_ground_position = low_pass_filter(self.target_ground_position, K_AIR_RETURN_PERIOD,
                                                          ground_position, dt)
        # non-minimum phase trick (see WheelBalancer class documentation)
        trick_velocity = -target_ground_velocity
        self.ground_velocity = trick_velocity - (kp_pos * e_pos + kp_pitch * e_pitch) - self.integral_velocity
        self.ground_velocity = clamp(self.ground_velocity, -p.max_ground_velocity, p.max_ground_velocity)

    def write(self) -> dict:
        """Wheel velocity targets [rad/s] (left_wheeled convention: +left = forward) and leg gain scales."""
        p = self.p
        ref_wheel_velocity = self.ground_velocity / p.wheel_radius
        left = +ref_wheel_velocity
        right = -ref_wheel_velocity
        yaw_to_wheel = p.contact_radius / p.wheel_radius
        left += yaw_to_wheel * self.target_yaw_velocity
        right += yaw_to_wheel * self.target_yaw_velocity
        turning = 1.0 if abs(self.target_yaw_velocity) > p.stiff_yaw_velocity else 0.0
        scale = K_GAIN_SCALE + K_TURNING_GAIN_SCALE * turning
        return {"left_wheel_velocity": left, "right_wheel_velocity": right, "leg_kp_scale": scale, "leg_kd_scale": scale,
                "ground_velocity": self.ground_velocity}

    def step(self, observation: dict, target_ground_velocity: float, target_yaw_velocity: float) -> dict:
        self.read(observation, target_ground_velocity, target_yaw_velocity)
        return self.write()
