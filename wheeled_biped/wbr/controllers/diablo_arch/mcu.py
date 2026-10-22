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
from wbr.observers.upkie_observers import BaseOrientation

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
    yaw_int_limit: float = 0.5          # rad, clamp of the yaw-rate error integral (ki_yaw * limit = max steady torque)
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
    leg_joint: str = "crank"            # name suffix of the second leg joint ("crank" five-bar, "knee" serial leg)
    F_ff_N: Optional[float] = None      # static leg force feed-forward per leg; None: 0.5 (mass_total - mass_wheels) g
    friction_comp_Nm: float = 0.0       # hip stiction compensation: +fc * tanh((Tp - Tp_ff) / width) in the direction the
                                        # balance law wants the virtual leg to move (not the sign of the total torque, which
                                        # includes the static holding torque)
    friction_comp_wheel_Nm: float = 0.0  # wheel stiction compensation: +fcw * tanh(T / width)
    friction_comp_width: float = 0.02   # N m
    vel_window_s: float = 0.0           # > 0: joint velocities from quantised-position differences over this window
                                        # instead of the driver's 1-step differentiated velocity (noise 1/N, delay N/2 steps)
    friction_comp_rate_gate: float = 0.0  # rad/s: the stiction compensation fades as 1 / (1 + (pitch_rate / gate)^2); it is
                                        # a static-posture aid and must not add high-frequency loop gain (review r1 M1:
                                        # the relay term sustained a 13 Hz limit cycle at >= 12 ms latency).  0 = no gate
    # real-robot IMU convention (review r1 M3): the pi3hat IMU reports its orientation in the ARS frame (z down) and is
    # mounted rotated on the torso; both None = the IMU already reports the base orientation in the world frame
    imu_rotation_base_to_imu: Optional[np.ndarray] = None
    imu_rotation_ars_to_world: Optional[np.ndarray] = None
    # sensor-data guards (review r1 M2 / m12): non-finite samples are held, then FAULT; a time stamp that stops
    # advancing (stale driver data) is held up to stale_fault_s, then FAULT
    max_bad_samples: int = 5
    stale_fault_s: float = 0.05
    fall_roll: float = math.radians(34)
    friction_comp_static_tau: float = 0.0  # > 0 (s): the compensation acts on (Tp - Tp_ff) and T minus their own low-pass
                                        # with this time constant.  A static holding-torque change (payload, mass error,
                                        # slope) is then not taken for an intended motion; comparing with the fixed Tp_ff
                                        # added a constant +-2 fc (0.16 N m) to the hip torque in those cases (Plan 4).
                                        # 0 keeps the comparison with Tp_ff / 0 (Plans 2-3)


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
    v_ref: float = 0.0          # the MCU's own current velocity reference (shaped v_filt): telemetry for the host


class DiabloMcu:
    def __init__(self, law, params: McuParams | None = None, leg: FiveBarLeg | None = None, link: HostLink | None = None):
        self.p = params or McuParams()
        self.leg = leg or FiveBarLeg(LegGeometry())
        self.law = law
        self.link = link or HostLink()
        self.left_joints = ("left_hip", f"left_{self.p.leg_joint}", "left_wheel")
        self.right_joints = ("right_hip", f"right_{self.p.leg_joint}", "right_wheel")
        p = self.p
        self.imu = (BaseOrientation(p.imu_rotation_base_to_imu, p.imu_rotation_ars_to_world)
                    if (p.imu_rotation_base_to_imu is not None or p.imu_rotation_ars_to_world is not None) else None)
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
        self.est_valid = False          # becomes True after the first estimate() (arm() needs a measurement)
        self.off_latched = False        # motors switched off after a host-link loss (disconnect_off_s)
        self.ever_connected = False
        self._down_refused = False
        self.est = Estimate()
        self.L_ref = Ramp(self.p.leg_rate, value=self.p.L_stand if mode == RobotMode.STAND else self.p.L_min)
        self.L_target = self.L_ref.value
        self.v_ref = 0.0
        self.yaw_rate_ref = 0.0
        self.yaw_int = 0.0
        self.roll_ref = 0.0
        self.pitch_ref = 0.0
        self.position = 0.0
        self.fc_hip_lp = 0.0            # low-pass of (Tp - Tp_ff) and T for friction_comp_static_tau
        self.fc_wheel_lp = 0.0
        self.last_obs: dict = {}
        self._pos_hist: Dict[str, list] = {}
        self.bad_samples = 0
        self.stale_samples = 0
        self.last_obs_time: Optional[float] = None
        self.est_hold = False
        self.servo_valid: Dict[str, dict] = {}       # last valid (filtered) servo sample: the VMC never reads raw NaNs
        self._rejected_seen = int(getattr(self.link, "rejected", 0))
        self.law.reset()
        self.link.reset()
        self.telemetry: dict = {}
        self.events = []
        self.last_bad_channel: Optional[str] = None

    def arm(self, force: bool = False) -> bool:
        """Enable balancing; refused before the first measurement, while the motors are latched off after a
        link loss, or unless the robot is upright (force=True bypasses all three)."""
        if force or (self.est_valid and not self.off_latched and abs(self.est.pitch) < self.p.arm_pitch):
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
    def _filtered_servo(self, servo: Dict[str, dict]) -> Dict[str, dict]:
        """Replace the driver velocities by position differences over vel_window_s (Plan 4 jitter fix: the 14-bit
        encoder differentiated over one 1 kHz step has +-0.38 rad/s quantisation noise which the LQI6 theta_dot gain
        turned into 0.1 N m rms wheel torque chatter; over 5 ms the noise is 5x lower at 2.5 ms extra delay)."""
        n = int(round(self.p.vel_window_s / self.p.dt))
        if n <= 1:
            # a COPY: the held sample (servo_valid) must never alias the caller's dict, or an entry point that updates
            # one observation dict in place hands a NaN position straight to the VMC at 200 Hz (review r3 m1)
            return {name: dict(s) for name, s in servo.items()}
        out = {}
        for name, s in servo.items():
            h = self._pos_hist.setdefault(name, [])
            h.append(s["position"])
            if len(h) > n + 1:
                del h[0]
            if len(h) >= 2:
                vel = (h[-1] - h[0]) / ((len(h) - 1) * self.p.dt)
            else:
                vel = s["velocity"]
            out[name] = {"position": s["position"], "velocity": vel, "torque": s["torque"]}
        return out

    @staticmethod
    def _invalid_channel(obs: dict) -> Optional[str]:
        """Name of the first invalid channel of the sample (non-finite, or a non-unit quaternion), None when the
        sample is valid: finite unit quaternion and gyro, finite joint positions / velocities.  The FAULT event
        names the channel for the real-robot diagnosis (review r3 n1)."""
        imu = obs["imu"]
        q = np.asarray(imu["orientation"], dtype=float)
        if not (np.all(np.isfinite(q)) and abs(float(np.linalg.norm(q)) - 1.0) < 0.1):
            return "imu.orientation"
        if not np.all(np.isfinite(np.asarray(imu["angular_velocity"], dtype=float))):
            return "imu.angular_velocity"
        for name, s in obs["servo"].items():
            if not math.isfinite(s["position"]):
                return f"servo.{name}.position"
            if not math.isfinite(s["velocity"]):
                return f"servo.{name}.velocity"
        return None

    @classmethod
    def _sample_valid(cls, obs: dict) -> bool:
        return cls._invalid_channel(obs) is None

    def _hold_history(self) -> None:
        """A held sample still advances the position-difference window by one step (repeat the last position), so
        that the velocity estimate keeps its time base: skipping the step made the next difference span two or more
        control periods while being divided by one, a 10x velocity spike after the 10 ms start-up delay line."""
        n = int(round(self.p.vel_window_s / self.p.dt))
        if n <= 1:
            return
        for h in self._pos_hist.values():
            if h:
                h.append(h[-1])
                if len(h) > n + 1:
                    del h[0]

    def estimate(self, obs: dict) -> Estimate:
        e = self.est
        # review r1 M2 / m12: a non-finite or stale sample must never reach the integrators.  The last estimate is
        # held for up to max_bad_samples cycles (dropped packet) or stale_fault_s (frozen driver data), then FAULT.
        t_obs = obs.get("time")
        stale = t_obs is not None and self.last_obs_time is not None and float(t_obs) <= self.last_obs_time
        if stale:
            self.stale_samples += 1
        else:
            self.stale_samples = 0
            if t_obs is not None:
                self.last_obs_time = float(t_obs)
        bad = self._invalid_channel(obs)
        if bad is not None:
            self._hold_history()
            self.bad_samples += 1
            self.est_hold = True
            self.last_bad_channel = bad
            if self.bad_samples > self.p.max_bad_samples and not self.fault:
                self.fault = True
                self.events.append((self.t, f"FAULT: non-finite sensor data for {self.bad_samples} cycles ({bad})"))
            return e
        if stale:
            self._hold_history()
            self.est_hold = True
            if self.stale_samples * self.p.dt > self.p.stale_fault_s and not self.fault:
                self.fault = True
                self.events.append((self.t, f"FAULT: sensor data stale for {self.stale_samples * self.p.dt * 1e3:.0f} ms"))
            return e
        self.bad_samples = 0
        self.est_hold = False
        gyro_raw = np.asarray(obs["imu"]["angular_velocity"], dtype=float)
        quat = np.asarray(obs["imu"]["orientation"], dtype=float)
        quat = quat / np.linalg.norm(quat)           # unit quaternion before use (review r2 n2)
        if self.imu is not None:        # real-robot convention (ARS frame + IMU mounting) -> base frame (review r1 M3)
            bo = self.imu.update(quat, gyro_raw)
            R = bo["rotation_base_to_world"]
            gyro = np.asarray(bo["angular_velocity"], dtype=float)
        else:
            R = quat_to_matrix(quat)
            gyro = gyro_raw
        roll, pitch, _ = roll_pitch_yaw(R)
        e.pitch, e.pitch_rate = pitch, float(gyro[1])
        e.roll, e.roll_rate = roll, float(gyro[0])
        e.yaw_rate = float((R @ gyro)[2])          # world-frame yaw rate (review B11)
        self.est_valid = True
        servo = self._filtered_servo(obs["servo"])
        self.servo_valid = servo
        for side in ("left", "right"):
            qh, qc = servo[f"{side}_hip"]["position"], servo[f"{side}_{self.p.leg_joint}"]["position"]
            qdh, qdc = servo[f"{side}_hip"]["velocity"], servo[f"{side}_{self.p.leg_joint}"]["velocity"]
            _, L0, th_b = self.leg.forward(qh, qc)
            L0_dot, th_b_dot = self.leg.virtual_rates(qh, qc, qdh, qdc)
            e.theta_v_body[side] = th_b
            e.theta_v[side] = th_b + pitch
            e.theta_v_dot[side] = th_b_dot + e.pitch_rate
            e.L0[side], e.L0_dot[side] = L0, L0_dot
            shank_rate_world = self.leg.shank_rate(qdh, qdc) + e.pitch_rate   # wheel carrier rate (leg specific)
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
        if not all(math.isfinite(float(x)) for x in (cmd.forward, cmd.left, cmd.up, cmd.roll, cmd.pitch, cmd.leg_split)):
            return                                    # never let a non-finite reference into the ramps / integrators
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
        rejected = int(getattr(self.link, "rejected", 0))
        if rejected > self._rejected_seen:            # review r2 M2: corrupt host packets are dropped by the link
            if self._rejected_seen == 0 or rejected // 50 > self._rejected_seen // 50:   # r3 n2: 1st, then every
                self.events.append((self.t, f"host packet rejected (non-finite field), {rejected} so far"))   # 50th
            self._rejected_seen = rejected
        self.apply_command(cmd, dt)
        if hasattr(self.law, "set_reference_shaping"):   # host-shaped reference (stage-2 MPC) -> plain tracking;
            self.law.set_reference_shaping(not getattr(cmd, "host_shaped", False))   # timeout / fallback -> own shaping

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
        if abs(e.roll) > p.fall_roll and not self.fault:
            self.fault = True
            self.events.append((self.t, "FAULT: roll"))
        if self.link.in_control:
            self.ever_connected = True
            if self.off_latched:            # the host is back: allow re-arming (arm() still needs an upright robot)
                self.off_latched = False
                self.events.append((self.t, "host link back: motors may be re-armed"))
        elif (p.disconnect_off_s > 0 and self.ever_connected and not self.off_latched
              and (self.t - self.link.last_rx_time) > p.disconnect_off_s):
            self.off_latched = True
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
            return {j: torque_command(0.0, 0.0) for j in self.left_joints + self.right_joints}

        # yaw (differential wheel torque / velocity) and roll (leg-length difference) terms --
        if balance:
            yaw_err = self.yaw_rate_ref - e.yaw_rate
            self.yaw_int = clamp_abs(self.yaw_int + yaw_err * dt, p.yaw_int_limit)
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
        ctrl = getattr(self.law, "ctrl", None)       # telemetry: the reference the balancer is actually tracking
        e.v_ref = float(getattr(ctrl, "v_filt", v_ref)) if ctrl is not None else float(v_ref)
        if getattr(ctrl, "nonfinite_steps", 0) > p.max_bad_samples and not self.fault:
            self.fault = True                         # the balance law has been holding its output on bad inputs
            self.events.append((self.t, "FAULT: non-finite balance-law input"))
        if "servo" in law_out:                       # the law produced full servo commands (Plan 3)
            self.telemetry.update({k: v for k, v in law_out.items() if k != "servo"})
            return dict(law_out["servo"])

        T, Tp = (law_out["T"], law_out["Tp"]) if balance else (0.0, 0.0)
        Tp_ff = law_out.get("Tp_ff", 0.0) if balance else 0.0
        # stiction compensation (Plan 4): only a quasi-static aid -- faded out with the body pitch rate so that it adds
        # no gain to the balance loop (review r1 M1)
        fc_gate = 1.0 if p.friction_comp_rate_gate <= 0.0 else 1.0 / (1.0 + (e.pitch_rate / p.friction_comp_rate_gate) ** 2)
        if balance and p.friction_comp_Nm > 0.0:        # break the hip stiction in the intended direction
            dev = Tp - Tp_ff
            if p.friction_comp_static_tau > 0.0:
                self.fc_hip_lp += (dev - self.fc_hip_lp) * dt / p.friction_comp_static_tau
                dev -= self.fc_hip_lp
            Tp += fc_gate * p.friction_comp_Nm * 2.0 * math.tanh(dev / p.friction_comp_width)
        if balance and p.friction_comp_wheel_Nm > 0.0:
            dev = T
            if p.friction_comp_static_tau > 0.0:
                self.fc_wheel_lp += (dev - self.fc_wheel_lp) * dt / p.friction_comp_static_tau
                dev -= self.fc_wheel_lp
            T += fc_gate * p.friction_comp_wheel_Nm * 2.0 * math.tanh(dev / p.friction_comp_width)
        F_ff = p.F_ff_N if p.F_ff_N is not None else 0.5 * (p.mass_total - p.mass_wheels) * 9.81
        # anti-split: keep both legs at the same angle (DIABLO leg_split = 0)
        d_theta = e.theta_v_body["left"] - e.theta_v_body["right"]
        d_theta_dot = e.theta_v_dot["left"] - e.theta_v_dot["right"]
        tau_split = p.kp_split * d_theta + p.kd_split * d_theta_dot
        out: Dict[str, ServoCommand] = {}
        for side, sign in (("left", -1.0), ("right", +1.0)):
            F = p.kp_leg * (L_ref_side[side] - e.L0[side]) - p.kd_leg * e.L0_dot[side] + F_ff / max(math.cos(e.theta_v[side]), 0.5)
            Tp_side = 0.5 * Tp + sign * 0.5 * tau_split
            sv = self.servo_valid if self.servo_valid else obs["servo"]        # held sample on invalid data (r2 M1)
            tau_hip, tau_crank = self.leg.vmc_torques(F, Tp_side, sv[f"{side}_hip"]["position"],
                                                      sv[f"{side}_{p.leg_joint}"]["position"])
            tau_wheel = 0.5 * T + sign * 0.5 * tau_yaw
            for name, tau, lim in ((f"{side}_hip", tau_hip, p.tau_hip_max), (f"{side}_{p.leg_joint}", tau_crank, p.tau_hip_max),
                                   (f"{side}_wheel", tau_wheel, p.tau_wheel_max)):
                if not math.isfinite(tau):          # never send NaN to the motors (review B13); and never silently:
                    tau = 0.0                       # a non-finite command is a FAULT with an event (review r1 M2)
                    if not self.fault:
                        self.fault = True
                        self.events.append((self.t, f"FAULT: non-finite torque command on {name}"))
                out[name] = torque_command(clamp_abs(tau, lim), lim)
        self.telemetry.update({"T": T, "Tp": Tp, "tau_split": tau_split, "d_theta": d_theta, "armed": self.armed})
        return out
