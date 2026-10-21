"""upkie's control code running inside the DIABLO-style MCU (Plan 3).

* balance: upkie WheelBalancer (PI on base pitch and ground position -> ground velocity), optionally
  the upkie MPC balancer, both unchanged apart from the wheel geometry parameters
* observers: upkie FloorContact (apparent wheel inertia) and WheelOdometry
* actuation: upkie servo semantics -- wheels in velocity mode (kd loop), hips in position mode
  holding the five-bar IK pose for the commanded leg length (with the kp/kd scales of the
  wheel balancer), i.e. the upkie "gyropod" mapping.  No torque-mode VMC.
"""
from __future__ import annotations

import math
from typing import Dict

from wbr.common.servo import ServoCommand
from wbr.controllers.lqi6.fivebar import FiveBarLeg
from wbr.controllers.upkie.gyropod import GyropodMapper
from wbr.controllers.upkie.wheel_balancer import WheelBalancer, WheelBalancerParams
from wbr.observers.upkie_observers import FloorContact, FloorContactParams, WheelOdometry


class UpkieLaw:
    def __init__(self, leg: FiveBarLeg, dt: float, wheel_radius: float, track: float, tau_wheel_max: float,
                 tau_hip_max: float, L_stand: float, theta_eq=None, balancer: str = "pid", gain_scale: float = 1.0,
                 leg_gain_scale: float = 2.0, mpc_leg_length: float = 0.3, max_ground_velocity: float = 2.0,
                 use_sim_contact: bool = False, gravity_feedforward_N: float = 0.0):
        self.leg = leg
        self.gravity_ff = gravity_feedforward_N   # static leg force per leg sent as servo feedforward torque
        self.dt = dt
        self.R = wheel_radius
        self.track = track
        self.theta_eq = theta_eq or (lambda L0: 0.0)
        self.balancer_kind = balancer
        self.balancer = WheelBalancer(WheelBalancerParams(dt=dt, wheel_radius=wheel_radius, contact_radius=track / 2,
                                                          gain_scale=gain_scale, max_ground_velocity=max_ground_velocity))
        self.mpc = None
        if balancer == "mpc":
            from wbr.controllers.upkie.mpc_balancer import MPCBalancer
            self.mpc = MPCBalancer(leg_length=mpc_leg_length, max_ground_velocity=max_ground_velocity)
            self.mpc_period = max(1, int(round(0.005 / dt)))
        self.use_sim_contact = use_sim_contact
        self.fc_params = FloorContactParams(dt=dt, upper_leg_joints=("left_hip", "left_crank", "right_hip", "right_crank"),
                                            upper_leg_torque_threshold=6.0)
        self.floor_contact = FloorContact(self.fc_params)
        self.odometry = WheelOdometry({"left_wheel": wheel_radius, "right_wheel": wheel_radius}, dt)
        self.mapper = GyropodMapper(wheel_radius, track, dt, left_wheeled=True, right_sign=+1.0, yaw_sign=-1.0,
                                    upper_leg_joints=("left_hip", "left_crank", "right_hip", "right_crank"),
                                    wheel_joints=("left_wheel", "right_wheel"), wheel_max_torque=tau_wheel_max,
                                    leg_max_torque=tau_hip_max, leg_gain_scale=leg_gain_scale,
                                    max_ground_velocity=max_ground_velocity, max_yaw_velocity=2.0,
                                    neutral_leg_positions=self.leg_targets({"left": L_stand, "right": L_stand}),
                                    leg_cutoff_period=0.2)
        self.k = 0
        self.ground_velocity = 0.0

    def leg_targets(self, L_ref_side: Dict[str, float]) -> Dict[str, float]:
        out = {}
        for side in ("left", "right"):
            L0 = L_ref_side[side]
            qh, qc = self.leg.inverse(L0, self.theta_eq(L0))
            out[f"{side}_hip"] = qh
            out[f"{side}_crank"] = qc
        return out

    def reset(self) -> None:
        self.balancer.reset()
        self.odometry.reset()
        self.floor_contact = FloorContact(self.fc_params)
        self.mapper.reset()
        if self.mpc is not None:
            self.mpc.reset()
        self.k = 0
        self.ground_velocity = 0.0

    def compute(self, est, refs: dict, mcu) -> dict:
        obs = mcu.last_obs
        servo = obs["servo"]
        fc = self.floor_contact.update(servo)
        if self.use_sim_contact and "sim" in obs:
            for w in ("left_wheel", "right_wheel"):
                fc[w]["contact"] = obs["sim"]["wheel_contact"][w]
            fc["contact"] = any(obs["sim"]["wheel_contact"].values())
        wo = self.odometry.update(fc, servo)
        observation = {"base_orientation": {"pitch": est.pitch - refs.get("pitch_ref", 0.0),
                                            "angular_velocity": [est.roll_rate, est.pitch_rate, est.yaw_rate]},
                       "wheel_odometry": wo, "floor_contact": fc}
        v_ref = refs["v_ref"] if refs["balance"] else 0.0
        yaw_ref = refs["yaw_rate_ref"] if refs["balance"] else 0.0
        if self.mpc is not None:
            if self.k % self.mpc_period == 0:
                self.ground_velocity = self.mpc.step(v_ref, observation, self.mpc_period * self.dt)
            kp_scale = kd_scale = 2.0 + (2.0 if abs(yaw_ref) > 0.1 else 0.0)   # upkie follow_joystick turning gains
        else:
            out = self.balancer.step(observation, v_ref, yaw_ref)
            self.ground_velocity = out["ground_velocity"]
            kp_scale, kd_scale = out["leg_kp_scale"], out["leg_kd_scale"]
        self.k += 1
        if not refs["balance"]:
            self.ground_velocity = 0.0
        self.mapper.set_neutral(self.leg_targets(refs["L_ref_side"]))
        cmds = self.mapper.servo_commands(self.ground_velocity, yaw_ref, kp_scale, kd_scale)
        if self.gravity_ff > 0.0:   # upkie servo API: feedforward_torque (gravity compensation of the leg load)
            for side in ("left", "right"):
                qh, qc = servo[f"{side}_hip"]["position"], servo[f"{side}_crank"]["position"]
                _, _, th_b = self.leg.forward(qh, qc)
                tau_h, tau_c = self.leg.vmc_torques(self.gravity_ff * math.cos(th_b + est.pitch), 0.0, qh, qc)
                cmds[f"{side}_hip"].feedforward_torque = tau_h
                cmds[f"{side}_crank"].feedforward_torque = tau_c
        return {"servo": cmds, "ground_velocity": self.ground_velocity, "contact": float(fc["contact"]),
                "odom_position": wo["position"], "odom_velocity": wo["velocity"], "T": 0.0, "Tp": 0.0}
