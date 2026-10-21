"""Servo action mapping of upkie's UpkieGyropod wrapper (upkie/envs/upkie_gyropod.py, Apache-2.0).

Converts (ground velocity, yaw velocity) into wheel velocity targets (velocity mode, max torque =
wheel effort limit) and keeps hips/knees in position mode, low-pass filtered toward the neutral
configuration with a configurable kp/kd gain scale.
"""
from __future__ import annotations

import math
from typing import Dict

from wbr.common.filters import clamp_abs, low_pass_filter
from wbr.common.servo import ServoCommand


class GyropodMapper:
    def __init__(self, wheel_radius: float, wheel_base: float, dt: float, left_wheeled: bool = True,
                 right_sign: float | None = None, yaw_sign: float = 1.0,
                 upper_leg_joints=("left_hip", "left_knee", "right_hip", "right_knee"),
                 wheel_joints=("left_wheel", "right_wheel"), wheel_max_torque: float = 1.7,
                 leg_max_torque: float = 16.0, leg_gain_scale: float = 1.0,
                 max_ground_velocity: float = 3.0, max_yaw_velocity: float = 1.0,
                 neutral_leg_positions: Dict[str, float] | None = None, leg_cutoff_period: float = 1.0):
        self.wheel_radius = wheel_radius
        self.contact_radius = 0.5 * wheel_base
        self.dt = dt
        self.left_sign = 1.0 if left_wheeled else -1.0
        self.right_sign = -self.left_sign if right_sign is None else right_sign   # upkie: mirrored wheel motors
        self.yaw_sign = yaw_sign       # +1 reproduces upkie's code; -1 for same-sign wheel joints
        self.upper_leg_joints = tuple(upper_leg_joints)
        self.wheel_joints = tuple(wheel_joints)
        self.wheel_max_torque = wheel_max_torque
        self.leg_max_torque = leg_max_torque
        self.leg_gain_scale = leg_gain_scale
        self.max_ground_velocity = max_ground_velocity
        self.max_yaw_velocity = max_yaw_velocity
        self.neutral = dict(neutral_leg_positions or {j: 0.0 for j in self.upper_leg_joints})
        self.leg_cutoff_period = leg_cutoff_period
        self.leg_positions = dict(self.neutral)

    def reset(self, current_leg_positions: Dict[str, float] | None = None) -> None:
        if current_leg_positions:
            self.leg_positions = {j: current_leg_positions[j] for j in self.upper_leg_joints}
        else:
            self.leg_positions = dict(self.neutral)

    def set_neutral(self, neutral: Dict[str, float]) -> None:
        self.neutral.update(neutral)

    def servo_commands(self, ground_velocity: float, yaw_velocity: float,
                       leg_kp_scale: float | None = None, leg_kd_scale: float | None = None) -> Dict[str, ServoCommand]:
        ground_velocity = clamp_abs(ground_velocity, self.max_ground_velocity)
        yaw_velocity = clamp_abs(yaw_velocity, self.max_yaw_velocity)
        # differential drive on the forward velocity of each wheel, then per-wheel joint signs:
        #   yaw_sign = +1 reproduces upkie (left wheel faster for +yaw: clockwise), -1 gives REP-103 (CCW)
        dv = self.yaw_sign * self.contact_radius * yaw_velocity
        left = self.left_sign * (ground_velocity + dv) / self.wheel_radius
        right = self.right_sign * (ground_velocity - dv) / self.wheel_radius
        kp = self.leg_gain_scale if leg_kp_scale is None else leg_kp_scale
        kd = self.leg_gain_scale if leg_kd_scale is None else leg_kd_scale
        cmds: Dict[str, ServoCommand] = {}
        for j in self.upper_leg_joints:
            self.leg_positions[j] = low_pass_filter(self.leg_positions[j], self.leg_cutoff_period, self.neutral[j], self.dt)
            cmds[j] = ServoCommand(position=self.leg_positions[j], velocity=0.0, kp_scale=kp, kd_scale=kd,
                                   maximum_torque=self.leg_max_torque)
        cmds[self.wheel_joints[0]] = ServoCommand(position=math.nan, velocity=left, kp_scale=0.0, kd_scale=1.0,
                                                  maximum_torque=self.wheel_max_torque)
        cmds[self.wheel_joints[1]] = ServoCommand(position=math.nan, velocity=right, kp_scale=0.0, kd_scale=1.0,
                                                  maximum_torque=self.wheel_max_torque)
        return cmds
