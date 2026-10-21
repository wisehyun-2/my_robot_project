"""Servo command/model emulating a moteus-style joint controller (as in upkie's Bullet spine).

tau = clamp(tau_ff + kp * kp_scale * (q* - q) + kd * kd_scale * (qd* - qd), +-max_torque)
A NaN position target disables the position term (velocity mode).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Dict


@dataclass
class ServoCommand:
    position: float = math.nan   # rad (NaN -> no position feedback)
    velocity: float = 0.0        # rad/s
    feedforward_torque: float = 0.0
    kp_scale: float = 1.0
    kd_scale: float = 1.0
    maximum_torque: float = 1.0  # N*m (upkie default_action::kMaximumTorque)

    def copy(self) -> "ServoCommand":
        return replace(self)


@dataclass
class ServoProperties:
    kp: float = 20.0             # N*m/rad   (upkie Bullet spine torque_control_kp)
    kd: float = 1.0              # N*m*s/rad (torque_control_kd)
    max_torque: float = 16.0     # hardware limit


class ServoModel:
    """Torque computation for a set of named servos."""

    def __init__(self, properties: Dict[str, ServoProperties]):
        self.props = properties

    def torque(self, name: str, cmd: ServoCommand, q: float, qd: float) -> float:
        p = self.props[name]
        tau = cmd.feedforward_torque
        tau += p.kd * cmd.kd_scale * (cmd.velocity - qd)
        if not math.isnan(cmd.position):
            tau += p.kp * cmd.kp_scale * (cmd.position - q)
        tau_max = min(cmd.maximum_torque, p.max_torque)
        return max(-tau_max, min(tau_max, tau))


def torque_command(tau: float, tau_max: float) -> ServoCommand:
    """Pure torque mode (DIABLO/LQI6 style): feedforward only."""
    return ServoCommand(position=math.nan, velocity=0.0, feedforward_torque=tau, kp_scale=0.0, kd_scale=0.0,
                        maximum_torque=tau_max)
