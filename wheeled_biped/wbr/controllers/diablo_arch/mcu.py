"""DIABLO-style real-time controller ("MCU program") for the DIABLO-lite robot.

Mirrors the DIABLO architecture (arXiv 2407.21500, diablo_ros2): a 1 kHz loop on the real-time
board reads the IMU and the six motors, estimates the body/leg state, runs a mode state machine
(CAR / TRANSITION_UP / STAND / TRANSITION_DOWN), applies the balance law (pluggable: the LQI6 of the
design report for Plan 2, upkie's wheel balancer for Plan 3), the leg length controller (PD + weight
feed-forward through the five-bar VMC), yaw rate control (differential wheel torque), roll control
(leg length difference) and an anti-split PD between the legs, with command timeouts (host link).
The host (ROS 2) only provides MotionCtrl set-points; the robot stands even if the host dies.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Optional

import numpy as np

from wbr.common.filters import Ramp, clamp, clamp_abs
from wbr.common.rotations import quat_to_matrix, roll_pitch_yaw
from wbr.common.servo import ServoCommand, torque_command
from wbr.controllers.diablo_arch.host_link import HostLink, MotionCtrl, RobotMode
from wbr.controllers.lqi6.fivebar import FiveBarLeg, LegGeometry

LEFT_JOINTS = ("left_hip", "left_crank", "left_wheel")
RIGHT_JOINTS = ("right_hip", "right_crank", "right_wheel")


@dataclass
class McuParams:
    dt: float = 1e-3
    wheel_radius: float = 0.065
    track: float = 0.38                 # distance between wheel centers
    mass_total: float = 6.0
    mass_wheels: float = 1.0
    L_min: float = 0.12
    L_max: float = 0.26
    L_stand: float = 0.198
    height_gain: float = 0.06           # m per unit of the `up` command
    kp_leg: float = 400.0               # N/m
    kd_leg: float = 10.0                # N s/m
    kp_yaw: float = 2.5                 # N m per rad/s of yaw rate error
    ki_yaw: float = 4.0                 # N m per rad
    kp_roll: float = 0.5                # m of leg-length difference per rad of roll error
    ki_roll: float = 0.3                # m per rad s
    kd_roll: float = 0.03
    roll_int_limit: float = 0.02        # m
    allow_transform_down: bool = False  # DIABLO-lite has no auxiliary wheels: refuse CAR (review B3)
    disconnect_off_s: float = 0.0       # > 0: all motors off this long after the host link dropped (report 4.5);
                                        # 0 keeps DIABLO's behaviour (balance on the spot indefinitely)
    arm_pitch: float = math.radians(3)  # arm() only accepts an upright robot
    kp_split: float = 15.0              # N m/rad on the left-right leg angle difference (anti-split)
    kd_split: float = 0.8
    max_forward: float = 1.0            # m/s (DIABLO normal speed mode)
    max_yaw_rate: float = 2.0           # rad/s
    fall_pitch: float = math.radians(34)  # design report FAULT threshold
    transition_time: float = 1.0        # s for stand-up / sit-down leg ramps
    tau_hip_max: float = 12.0
    tau_wheel_max: float = 5.0
    leg_rate: float = 0.25              # m/s leg length ramp


@dataclass
class Estimate:
    pitch: float = 0.0          # body pitch about +y (positive = leaning forward)
    pitch_rate: float = 0.0
    roll: float = 0.0
    roll_rate: float = 0.0
    yaw_rate: float = 0.0
    theta_v: Dict[str, float] = field(default_factory=dict)      # world leg angle per side
    theta_v_dot: Dict[str, float] = field(default_factory=dict)
    theta_v_body: Dict[str, float] = field(default_factory=dict)
    L0: Dict[str, float] = field(default_factory=dict)
    L0_dot: Dict[str, float] = field(default_factory=dict)
    v_wheel: Dict[str, float] = field(default_factory=dict)      # ground speed of each wheel
    v: float = 0.0
    theta_v_mean: float = 0.0
    theta_v_dot_mean: float = 0.0
    L0_mean: float = 0.0
    position: float = 0.0


class DiabloMcu:
    def __init__(self, law, params: McuParams | None = None, leg: FiveBarLeg | None = None, link: HostLink | None = None):
        self.p = params or McuParams()
        self.leg = leg or FiveBarLeg(LegGeometry())
        self.law = law
        self.link = link or HostLink()
        self.reset(RobotMode.STAND)

    # ---------------------------------------------------------------- state
    def reset(self, mode: RobotMode = RobotMode.STAND, armed: bool = True) -> None:
        """armed=True starts balancing immediately (test harness); the real program starts disarmed and
        calls arm() once the robot is held upright (|pitch| < arm_pitch)."""
        self.mode = mode
        self.mode_time = 0.0
        self.t = 0.0
        self.fault = False
        self.armed = armed
        self.roll_int = 0.0
        self.est = Estimate()
        self.L_ref = Ramp(self.p.leg_rate, value=self.p.L_stand if mode == RobotMode.STAND else self.p.L_min)
        self.L_target = self.L_ref.value
        self.v_ref = 0.0
        self.yaw_rate_ref = 0.0
        self.yaw_int = 0.0
        self.roll_ref = 0.0
        self.pitch_ref = 0.0
        self.position = 0.0
        self.last_obs: dict = {}
        self.law.reset()
        self.link.reset()
        self.telemetry: dict = {}
        self.events = []

    def arm(self, force: bool = False) -> bool:
        """Enable balancing; refused unless the robot is upright (or force=True)."""
        if force or abs(self.est.pitch) < self.p.arm_pitch:
            if not self.armed:
                self.events.append((self.t, "armed"))
            self.armed = True
            self.fault = False
        return self.armed

    def disarm(self, reason: str = "disarm") -> None:
        if self.armed:
            self.events.append((self.t, reason))
        self.armed = False

    # ------------------------------------------------------------- estimator
    def estimate(self, obs: dict) -> Estimate:
        e = self.est
        R = quat_to_matrix(obs["imu"]["orientation"])
        roll, pitch, _ = roll_pitch_yaw(R)
        gyro = np.asarray(obs["imu"]["angular_velocity"], dtype=float)
        e.pitch, e.pitch_rate = pitch, float(gyro[1])
        e.roll, e.roll_rate = roll, float(gyro[0])
        e.yaw_rate = float((R @ gyro)[2])          # world-frame yaw rate (review B11)
        servo = obs["servo"]
        for side in ("left", "right"):
            qh, qc = servo[f"{side}_hip"]["position"], servo[f"{side}_crank"]["position"]
            qdh, qdc = servo[f"{side}_hip"]["velocity"], servo[f"{side}_crank"]["velocity"]
            _, L0, th_b = self.leg.forward(qh, qc)
            L0_dot, th_b_dot = self.leg.virtual_rates(qh, qc, qdh, qdc)
            e.theta_v_body[side] = th_b
            e.theta_v[side] = th_b + pitch
            e.theta_v_dot[side] = th_b_dot + e.pitch_rate
            e.L0[side], e.L0_dot[side] = L0, L0_dot
            shank_rate_world = qdc + e.pitch_rate          # shank orientation = crank orientation
            e.v_wheel[side] = self.p.wheel_radius * (servo[f"{side}_wheel"]["velocity"] + shank_rate_world)
        e.v = 0.5 * (e.v_wheel["left"] + e.v_wheel["right"])
        e.theta_v_mean = 0.5 * (e.theta_v["left"] + e.theta_v["right"])
        e.theta_v_dot_mean = 0.5 * (e.theta_v_dot["left"] + e.theta_v_dot["right"])
        e.L0_mean = 0.5 * (e.L0["left"] + e.L0["right"])
        self.position += e.v * self.p.dt
        e.position = self.position
        return e

    # ------------------------------------------------------------- commands
    def apply_command(self, cmd: MotionCtrl, dt: float) -> None:
        p = self.p
        if cmd.mode_mark:
            if cmd.mode.stand_mode and self.mode == RobotMode.CAR:
                self._set_mode(RobotMode.TRANSITION_UP)
            elif not cmd.mode.stand_mode and self.mode == RobotMode.STAND:
                if p.allow_transform_down:
                    self._set_mode(RobotMode.TRANSITION_DOWN)
                elif not getattr(self, "_down_refused", False):
                    self.events.append((self.t, "transform down ignored (no auxiliary wheels on DIABLO-lite)"))
                    self._down_refused = True
        self.v_ref = clamp_abs(cmd.forward, p.max_forward)
        self.yaw_rate_ref = clamp_abs(cmd.left, p.max_yaw_rate)
        if cmd.mode.height_ctrl_mode:
            self.L_target = clamp(self.L_target + cmd.up * dt, p.L_min, p.L_max)
        else:
            self.L_target = clamp(p.L_stand + p.height_gain * cmd.up, p.L_min, p.L_max)
        if cmd.mode.roll_ctrl_mode:
            self.roll_ref = clamp_abs(self.roll_ref + cmd.roll * dt, 0.3)
        else:
            self.roll_ref = clamp_abs(cmd.roll, 0.3)
        if cmd.mode.pitch_ctrl_mode:
            self.pitch_ref = clamp_abs(self.pitch_ref + cmd.pitch * dt, 0.3)
        else:
            self.pitch_ref = clamp_abs(cmd.pitch, 0.3)

    def _set_mode(self, mode: RobotMode) -> None:
        self.events.append((self.t, f"{self.mode.name} -> {mode.name}"))
        self.mode = mode
        self.mode_time = 0.0

    # ------------------------------------------------------------------ loop
    def tick(self, obs: dict, host_cmd: Optional[MotionCtrl] = None) -> Dict[str, ServoCommand]:
        p = self.p
        dt = p.dt
        self.t += dt
        self.mode_time += dt
        self.last_obs = obs
        e = self.estimate(obs)
        cmd = self.link.poll(self.t) if host_cmd is None else host_cmd
        self.apply_command(cmd, dt)

        # mode state machine ---------------------------------------------------------------
        if self.mode == RobotMode.TRANSITION_UP:
            self.L_ref.step(p.L_stand, dt)
            if self.mode_time > p.transition_time:
                self._set_mode(RobotMode.STAND)
            balance = True
        elif self.mode == RobotMode.TRANSITION_DOWN:
            self.L_ref.step(p.L_min, dt)
            if self.mode_time > p.transition_time:
                self._set_mode(RobotMode.CAR)
            balance = True
        elif self.mode == RobotMode.STAND:
            self.L_ref.step(self.L_target, dt)
            balance = True
        else:  # CAR / others: no balancing, legs retracted, wheels free
            self.L_ref.step(p.L_min, dt)
            balance = False

        if abs(e.pitch) > p.fall_pitch and not self.fault:
            self.fault = True
            self.events.append((self.t, "FAULT: pitch"))
        if p.disconnect_off_s > 0 and not self.link.in_control and (self.t - self.link.last_rx_time) > p.disconnect_off_s:
            self.disarm("motors off: host link lost for %.1f s" % p.disconnect_off_s)
        if self.fault or not self.armed:
            # safe state (report 4.5): every joint torque-free, no leg force / feed-forward either
            balance = False
            self.yaw_int = 0.0
            self.roll_int = 0.0
            self.telemetry = {"mode": int(self.mode), "fault": self.fault, "armed": self.armed, "L_ref": self.L_ref.value,
                              "v_ref": 0.0, "in_control": self.link.in_control, "tau_yaw": 0.0, "dL": 0.0, "T": 0.0, "Tp": 0.0,
                              "tau_split": 0.0, "d_theta": 0.0}
            self.law.compute(e, dict(v_ref=0.0, yaw_rate_ref=0.0, pitch_ref=0.0, L_ref=self.L_ref.value,
                                     L_ref_side={"left": self.L_ref.value, "right": self.L_ref.value}, balance=False,
                                     tau_yaw=0.0), self)
            return {j: torque_command(0.0, 0.0) for j in LEFT_JOINTS + RIGHT_JOINTS}

        # yaw (differential wheel torque / velocity) and roll (leg-length difference) terms --
        if balance:
            yaw_err = self.yaw_rate_ref - e.yaw_rate
            self.yaw_int = clamp_abs(self.yaw_int + yaw_err * dt, 0.5)
            tau_yaw = p.kp_yaw * yaw_err + p.ki_yaw * self.yaw_int
        else:
            tau_yaw = 0.0
            self.yaw_int = 0.0
        # roll: geometric feed-forward (leg-length difference ~ track * tan(roll)) + PI on the error (review B8)
        roll_err = self.roll_ref - e.roll
        self.roll_int = clamp_abs(self.roll_int + roll_err * dt, p.roll_int_limit / max(p.ki_roll, 1e-6))
        dL = clamp_abs(p.track * math.tan(self.roll_ref) + p.kp_roll * roll_err + p.ki_roll * self.roll_int
                       - p.kd_roll * e.roll_rate, 0.05)
        L_ref_side = {"left": clamp(self.L_ref.value + 0.5 * dL, p.L_min, p.L_max),
                      "right": clamp(self.L_ref.value - 0.5 * dL, p.L_min, p.L_max)}
        v_ref = self.v_ref if balance else 0.0
        self.telemetry = {"mode": int(self.mode), "fault": self.fault, "L_ref": self.L_ref.value, "v_ref": v_ref,
                          "in_control": self.link.in_control, "tau_yaw": tau_yaw, "dL": dL}

        # balance law --------------------------------------------------------------------
        refs = dict(v_ref=v_ref, yaw_rate_ref=self.yaw_rate_ref if balance else 0.0, pitch_ref=self.pitch_ref,
                    L_ref=self.L_ref.value, L_ref_side=L_ref_side, balance=balance, tau_yaw=tau_yaw)
        law_out = self.law.compute(e, refs, self)
        if "servo" in law_out:                       # the law produced full servo commands (Plan 3)
            self.telemetry.update({k: v for k, v in law_out.items() if k != "servo"})
            return dict(law_out["servo"])

        T, Tp = (law_out["T"], law_out["Tp"]) if balance else (0.0, 0.0)
        F_ff = 0.5 * (p.mass_total - p.mass_wheels) * 9.81
        # anti-split: keep both legs at the same angle (DIABLO leg_split = 0)
        d_theta = e.theta_v_body["left"] - e.theta_v_body["right"]
        d_theta_dot = e.theta_v_dot["left"] - e.theta_v_dot["right"]
        tau_split = p.kp_split * d_theta + p.kd_split * d_theta_dot
        out: Dict[str, ServoCommand] = {}
        for side, sign in (("left", -1.0), ("right", +1.0)):
            F = p.kp_leg * (L_ref_side[side] - e.L0[side]) - p.kd_leg * e.L0_dot[side] + F_ff / max(math.cos(e.theta_v[side]), 0.5)
            Tp_side = 0.5 * Tp + sign * 0.5 * tau_split
            tau_hip, tau_crank = self.leg.vmc_torques(F, Tp_side, obs["servo"][f"{side}_hip"]["position"],
                                                      obs["servo"][f"{side}_crank"]["position"])
            tau_wheel = 0.5 * T + sign * 0.5 * tau_yaw
            for name, tau, lim in ((f"{side}_hip", tau_hip, p.tau_hip_max), (f"{side}_crank", tau_crank, p.tau_hip_max),
                                   (f"{side}_wheel", tau_wheel, p.tau_wheel_max)):
                if not math.isfinite(tau):          # never send NaN to the motors (review B13)
                    tau = 0.0
                out[name] = torque_command(clamp_abs(tau, lim), lim)
        self.telemetry.update({"T": T, "Tp": Tp, "tau_split": tau_split, "d_theta": d_theta, "armed": self.armed})
        return out
