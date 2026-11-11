"""DIABLO-style real-time controller ("MCU program") of the serial-leg wheel biped (Plan 4 harness, v5 robot).

Mirrors the DIABLO architecture (arXiv 2407.21500, diablo_ros2): a 1 kHz loop on the real-time
board reads the IMU and the six motors, estimates the body/leg state, runs a mode state machine
(CAR / TRANSITION_UP / STAND / TRANSITION_DOWN), applies the balance law (pluggable: the LQI8 law, or the
v5_mpc law), the leg length controller (PD + weight feed-forward through the serial-leg VMC), yaw rate
control (differential wheel torque), roll control (leg length difference) and an anti-split PD between
the legs, with command timeouts (host link).
The host (ROS 2) only provides MotionCtrl set-points; the robot stands even if the host dies.
(Until 2026-11-11 it also ran Plans 2 / 3 on the DIABLO-lite five-bar legs; those were removed.)
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, Optional

import numpy as np

from wbr.common.filters import Ramp, clamp, clamp_abs
from wbr.common.rotations import quat_to_matrix, roll_pitch_yaw
from wbr.common.servo import ServoCommand, torque_command
from wbr.controllers.diablo_arch.host_link import HostLink, MotionCtrl, RobotMode
from wbr.controllers.lqi8.serial_leg import SerialLeg
from wbr.observers.body_attitude import BodyAttitude
from wbr.observers.upkie_observers import BaseOrientation


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
    kw_yaw: float = 0.0                 # N m per rad/s of (v_right - v_left) / track error (11-07; 0 = off).  A wheel stopped by
                                        # a sharp step turned the robot before the gyro saw it (the leg splits first), and the
                                        # gyro gain cannot go up: 3 x kp_yaw shakes the legs.  This term acts on the wheels' own
                                        # speed, like a locked differential: the free wheel slows down at once and the stopped
                                        # one gets the torque.  A wheel spinning on the step fools it: only the gyro sees that
    yaw_ref_rate: float = 3.0           # rad/s^2, limits the yaw reference while kw_yaw is on: a step would reach the wheels at
                                        # (kp_yaw + kw_yaw) x step (1 rad/s: 1.2 -> 5 N m, overshoot 50 %, legs split 5.7 deg)
    attitude_tau: float = 0.0           # s; > 0: roll and pitch from the MCU's own complementary filter on the raw IMU sample, with the
                                        # robot's acceleration (encoders, gyro, IMU position) taken out: wbr.observers.body_attitude.
                                        # 0 keeps the attitude the IMU delivers
    imu_position: Optional[np.ndarray] = None   # m, IMU position from the hip midpoint in the base frame (attitude_tau)
    yaw_offset_tau: float = 0.0         # s; > 0 and the IMU sample has "bias_rps": yaw rate from the raw gyro (pi3hat rate - bias) minus
                                        # its own offset, learned with this time constant while the wheels say the robot does not turn.
                                        # The pi3hat's z bias state is pulled by a zero-yaw-rate pseudo measurement: after a 10 s turn at
                                        # 1 rad/s its rate reads 0.13 rad/s low and the yaw loop keeps the robot turning at that rate
    kp_roll: float = 0.5                # m of leg-length difference per rad of roll error
    ki_roll: float = 0.3                # m per rad s
    kd_roll: float = 0.03
    roll_int_limit: float = 0.02        # m
    allow_transform_down: bool = False  # the robot has no auxiliary wheels (unlike DIABLO): refuse CAR (review B3)
    disconnect_off_s: float = 0.0       # > 0: all motors off this long after the host link dropped (report 4.5);
                                        # 0 keeps DIABLO's behaviour (balance on the spot indefinitely)
    arm_pitch: float = math.radians(3)  # arm() only accepts an upright robot
    arm_leg_angle: float = math.radians(20)  # ... whose legs hang under it: |world leg angle| below this on both sides (arm_refusal)
    kp_split: float = 15.0              # N m/rad on the left-right leg angle difference (anti-split)
    kd_split: float = 0.8
    max_forward: float = 1.0            # m/s (DIABLO normal speed mode)
    max_yaw_rate: float = 2.0           # rad/s
    fall_pitch: float = math.radians(34)  # design report FAULT threshold
    transition_time: float = 1.0        # s for stand-up / sit-down leg ramps
    tau_hip_max: float = 12.0
    tau_wheel_max: float = 5.0
    leg_rate: float = 0.25              # m/s leg length ramp
    leg_joint: str = "knee"             # name suffix of the second leg joint (serial hip-knee leg)
    F_ff_N: Optional[float] = None      # static leg force feed-forward per leg; None: 0.5 (mass_total - mass_wheels) g
    leg_force: Optional[Callable[[float, float], float]] = None     # static force per leg from (L0, world lean), replaces F_ff_N / cos(lean)
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
    wheel_reaction_ff: bool = False     # serial leg only: the knee also cancels the wheel motor's reaction on the shank
                                        # (SerialLeg.wheel_reaction_ratio), which otherwise moves the leg length: forward
                                        # wheel torque stretches a knee-forward leg.  Uses the commanded wheel torque, like
                                        # the hip term that vmc_torques adds for Tp.  False keeps Plans 2-4 as they were
    # compliant roll (11-07): the leg-length PD only holds the MEAN length, so a leg pressed by a bump under one wheel
    # gets out of the way instead of pushing the body over (with one length PD per leg the body tilted by
    # h / (track + kp_roll)); the body is levelled by a differential force from the IMU roll, and soft walls keep each leg
    # inside its range because no length reference does that any more.  False keeps the leg-length-difference roll above
    roll_compliant: bool = False
    kp_roll_force: float = 0.0          # N per rad of roll error, +- on each leg
    ki_roll_force: float = 0.0          # N per rad s
    kd_roll_force: float = 0.0          # N s per rad, on the roll rate (no relative leg damping: a skyhook)
    roll_force_limit: float = 0.0       # N, |differential force|: at the static leg force a wheel is just unloaded
    roll_force_int_limit: float = 0.0   # N
    roll_ref_rate: float = 0.5          # rad/s: the force loop has no leg-length reference to smooth a roll command step
    kp_leg_limit: float = 0.0           # N/m, soft wall of one leg beyond L_max or below L_min - L_min_margin
    kd_leg_limit: float = 0.0           # N s/m, only while going further out
    L_min_margin: float = 0.02          # m: a bump under one wheel may shorten that leg below the commanded range
    L_max_margin: float = 0.0           # m: ... and lengthen the other one above it (the straight knee is at L1 + L2)
    mean_follows_range: bool = False    # compliant roll: when the leg-length difference needs it, the mean length goes down so that both legs stay in their range
    # like a car's suspension (11-07): over a speed bump the height control holds the LONGER leg, so the leg on the bump folds without being pushed back
    # and the body does not rise; with even legs this is the mean-length control.  uneven_blend: the leg difference over which it changes between the two
    suspension: bool = False
    suspension_blend: float = 0.005     # m
    suspension_share: float = 0.0       # m; > 0: above this leg difference the longer leg starts to take part, so that the shorter one only reaches its end at the full difference
    kd_leg_own: float = 0.0             # N s/m on each leg's own speed against the held length: a car's damper, it stops the wheel hopping
    # slower over uneven ground (11-07): the speed command is limited while the legs differ in length.  > 0: the limit at uneven_d1
    uneven_speed: float = 0.0           # m/s
    uneven_d0: float = 0.01             # m, leg-length difference where the limit starts to come down from max_forward
    uneven_d1: float = 0.04             # m, ... and reaches uneven_speed
    uneven_hold: float = 0.5            # s, the limit stays this long after the legs are even again
    uneven_rise: float = 1.0            # m/s^2, then it rises at this rate
    uneven_brake: float = 0.0           # m/s^2, the speed reference's rate under the limit; 0: the law's normal rate
    uneven_brake_tau: float = 0.0       # s, low-pass on the reference while it comes down (rounds the braking onset)
    uneven_lean: float = 0.0            # lean of the virtual leg per (a / g) under the limit: a balancing robot has to lean back before it can brake
    uneven_hold_int: bool = False       # no position / tilt integration while driving with uneven legs: a robot delayed by a bump would speed up again to catch its position reference


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
    def __init__(self, law, params: McuParams | None = None, leg: SerialLeg | None = None, link: HostLink | None = None):
        self.p = params or McuParams()
        if leg is None:
            raise ValueError("DiabloMcu needs the robot's leg kinematics (SerialLeg)")
        self.leg = leg
        self.law = law
        self.link = link or HostLink()
        self.left_joints = ("left_hip", f"left_{self.p.leg_joint}", "left_wheel")
        self.right_joints = ("right_hip", f"right_{self.p.leg_joint}", "right_wheel")
        p = self.p
        self.imu = (BaseOrientation(p.imu_rotation_base_to_imu, p.imu_rotation_ars_to_world)
                    if (p.imu_rotation_base_to_imu is not None or p.imu_rotation_ars_to_world is not None) else None)
        self.attitude = (BodyAttitude(p.attitude_tau, p.imu_position if p.imu_position is not None else np.zeros(3), p.dt)
                         if p.attitude_tau > 0.0 else None)
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
        self.roll_force_int = 0.0
        self.roll_ref_limited = 0.0
        self.est_valid = False          # becomes True after the first estimate() (arm() needs a measurement)
        self.home_state: Dict[str, int] = {}     # moteus register 0x00c per servo, when the sample carries "home_state"
        self.arm_refused: Optional[str] = None   # why the last arm() refused (arm_refusal)
        self.last_used_time: Optional[float] = None   # time stamp of the last sample the estimator integrated (_sample_dt)
        self._no_bias_event = False
        self.off_latched = False        # motors switched off after a host-link loss (disconnect_off_s)
        self.ever_connected = False
        self._down_refused = False
        self.est = Estimate()
        self.L_ref = Ramp(self.p.leg_rate, value=self.p.L_stand if mode == RobotMode.STAND else self.p.L_min)
        self.L_target = self.L_ref.value
        self.v_ref = 0.0
        self.yaw_rate_ref = 0.0
        self.yaw_int = 0.0
        self.yaw_ref_limited = 0.0
        self.yaw_offset: Optional[float] = None     # offset of the raw yaw gyro (yaw_offset_tau)
        self.v_limit = self.p.max_forward           # speed limit over uneven ground (uneven_speed)
        self.v_limit_hold = 0.0
        self.wheel_yaw_lp = 0.0
        if self.attitude is not None:
            self.attitude.reset()
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
        self.leg_force_cmd: Dict[str, float] = {}    # the force each leg was given in the last period (static + length PD + roll
                                                     # + range wall + damper): what the leg loops put on the hip / knee motors,
                                                     # read by a balance law that plans inside the motor limits (v5_mpc)
        self._rejected_seen = int(getattr(self.link, "rejected", 0))
        self.law.reset()
        self.link.reset()
        self.telemetry: dict = {}
        self.events = []
        self.last_bad_channel: Optional[str] = None

    def arm(self, force: bool = False) -> bool:
        """Enable balancing; refused before the first measurement, while the motors are latched off after a
        link loss, unless the robot is upright, or while the joints do not read like legs under it (arm_refusal;
        force=True bypasses all of them)."""
        self.arm_refused = None if force else self.arm_refusal()
        if self.arm_refused is None:
            if not self.armed:
                self.events.append((self.t, "armed"))
            self.armed = True
            self.fault = False
        return self.armed

    def arm_refusal(self) -> Optional[str]:
        """Why arm() would refuse now, None when it would arm.

        Besides the pitch, the hips and knees must read like legs under the body.  A moteus whose encoder sits on the rotor
        (8:1) knows the output angle only within +-22.5 deg after power-on: an un-homed knee then reads |sigma2| <= 22.5 deg,
        a leg of at least 2 L1 cos(11.25 deg) = 490 mm on the v5 robot whatever its real length (above the range's 485 mm),
        and an un-homed hip turns the leg by a multiple of 45 deg.  Armed like that, the controller folds the legs at full
        torque (v5 simulation: fall in 0.24 s).  So both legs must be inside the range the leg control allows
        (L_min - L_min_margin .. L_max + L_max_margin) and within arm_leg_angle of vertical, and a servo sample that carries
        "home_state" (moteus register 0x00c) must say 2 = output referenced for the hips and knees."""
        p, e = self.p, self.est
        if not self.est_valid:
            return "no measurement yet"
        if self.off_latched:
            return "motors latched off after a host-link loss"
        if abs(e.pitch) >= p.arm_pitch:
            return f"body pitch {math.degrees(e.pitch):+.1f} deg (limit {math.degrees(p.arm_pitch):.1f})"
        for name in ("left_hip", f"left_{p.leg_joint}", "right_hip", f"right_{p.leg_joint}"):
            hs = self.home_state.get(name)
            if hs is not None and hs != 2:
                return f"{name} not referenced to the output (home state {hs}, needs 2: d nearest / d exact)"
        low, high = p.L_min - p.L_min_margin, p.L_max + p.L_max_margin
        for side in ("left", "right"):
            L0, th = e.L0[side], e.theta_v[side]
            if not low <= L0 <= high:
                return f"{side} leg reads {1e3 * L0:.1f} mm, outside {1e3 * low:.0f}..{1e3 * high:.0f} mm (encoder zero / homing)"
            if abs(th) >= p.arm_leg_angle:
                return f"{side} leg angle {math.degrees(th):+.1f} deg (limit {math.degrees(p.arm_leg_angle):.0f}; hip zero / homing)"
        return None

    def disarm(self, reason: str = "disarm") -> None:
        if self.armed:
            self.events.append((self.t, reason))
        self.armed = False

    # ------------------------------------------------------------- estimator
    def _filtered_servo(self, servo: Dict[str, dict]) -> Dict[str, dict]:
        """Replace the driver velocities by position differences over vel_window_s (Plan 4 jitter fix: the 14-bit
        encoder differentiated over one 1 kHz step has +-0.38 rad/s quantisation noise which the LQI8 theta_dot gain
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

    def _invalid_channel(self, obs: dict) -> Optional[str]:
        """Name of the first invalid channel of the sample (non-finite, or a non-unit quaternion), None when the
        sample is valid: finite unit quaternion and gyro, finite joint positions / velocities.  The FAULT event
        names the channel for the real-robot diagnosis (review r3 n1).  The channels the configured estimator
        integrates are checked too: the accelerometer for the attitude filter (one NaN made its low-pass NaN for
        good, the attitude then ran on the gyro alone without a FAULT) and the board's gyro bias for the raw yaw
        path (a NaN made the attitude NaN; a sample that stops carrying it would switch the path off silently)."""
        imu = obs["imu"]
        q = np.asarray(imu["orientation"], dtype=float)
        if not (np.all(np.isfinite(q)) and abs(float(np.linalg.norm(q)) - 1.0) < 0.1):
            return "imu.orientation"
        if not np.all(np.isfinite(np.asarray(imu["angular_velocity"], dtype=float))):
            return "imu.angular_velocity"
        if self.attitude is not None:
            f = imu.get("linear_acceleration")
            if f is None or not np.all(np.isfinite(np.asarray(f, dtype=float))):
                return "imu.linear_acceleration"
        if self.p.yaw_offset_tau > 0.0:
            b = imu.get("bias_rps")
            if (b is None and self.yaw_offset is not None) or (b is not None and not np.all(np.isfinite(np.asarray(b, dtype=float)))):
                return "imu.bias_rps"
        for name, s in obs["servo"].items():
            if not math.isfinite(s["position"]):
                return f"servo.{name}.position"
            if not math.isfinite(s["velocity"]):
                return f"servo.{name}.velocity"
        return None

    def _sample_valid(self, obs: dict) -> bool:
        return self._invalid_channel(obs) is None

    def _sample_dt(self, t_obs) -> float:
        """Time since the last sample the estimator integrated, from the samples' own time stamps, for the gyro paths
        (attitude filter, yaw offset).  A sample the loop never saw (sample age jitter, a late read) is then integrated
        over its real interval: with the fixed dt the attitude filter lost that piece of angle, and with samples 0-5 ms
        old at random the robot fell.  The nominal dt without a time stamp, at the first sample and when the interval
        equals it (runs stay bit-identical), at most stale_fault_s after a long gap.  The position stays on the fixed
        dt: its velocity comes from position differences over a window counted in ticks (vel_window_s), so the two
        cancel when the real period is off; with dt_s alone the position read 23 % long at 1.25 x the period."""
        p = self.p
        last, self.last_used_time = self.last_used_time, (None if t_obs is None else float(t_obs))
        if t_obs is None or last is None:
            return p.dt
        gap = float(t_obs) - last
        if gap <= 0.0 or abs(gap - p.dt) <= 1e-9:
            return p.dt
        return min(gap, max(p.stale_fault_s, p.dt))

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
        dt_s = self._sample_dt(t_obs)
        for name, s in obs["servo"].items():
            if "home_state" in s:
                self.home_state[name] = int(s["home_state"])
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
        p = self.p
        raw_yaw = p.yaw_offset_tau > 0.0 and "bias_rps" in obs["imu"]
        if p.yaw_offset_tau > 0.0 and not raw_yaw and not self._no_bias_event:
            self._no_bias_event = True
            self.events.append((self.t, "yaw rate from the board's rate: the IMU sample has no bias_rps (request_attitude_detail)"))
        if raw_yaw:                                 # the yaw axis without the board's bias state, offset learned below
            bias = np.asarray(obs["imu"]["bias_rps"], dtype=float)
            bias_z = float((self.imu.R_base_to_imu.T @ bias)[2] if self.imu is not None else bias[2])
            if self.yaw_offset is None:
                self.yaw_offset = -bias_z
            gyro = np.array(gyro, dtype=float)
            gyro[2] -= bias_z + self.yaw_offset
        if self.attitude is not None:
            f = np.asarray(obs["imu"]["linear_acceleration"], dtype=float)
            up = self.attitude.update(gyro, self.imu.R_base_to_imu.T @ f if self.imu is not None else f, R[2, :], dt_s)   # R[2, :]: the
            roll, pitch = self.attitude.roll, self.attitude.pitch                                       # world's up in the base frame
            yaw_rate = float(up @ gyro)
        else:
            roll, pitch, _ = roll_pitch_yaw(R)
            yaw_rate = float((R @ gyro)[2])
        e.pitch, e.pitch_rate = pitch, float(gyro[1])
        e.roll, e.roll_rate = roll, float(gyro[0])
        e.yaw_rate = yaw_rate                      # world-frame yaw rate (review B11)
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
        self.position += e.v * self.p.dt            # on the same tick base as the velocity window (vel_window_s), not dt_s
        e.position = self.position
        if raw_yaw:
            self._learn_yaw_offset(e, dt_s)
        if self.attitude is not None:               # the hip midpoint's velocity for the next sample
            th, L = e.theta_v_mean, e.L0_mean
            Ld = 0.5 * (e.L0_dot["left"] + e.L0_dot["right"])
            thd = e.theta_v_dot_mean
            self.attitude.set_hip_velocity(e.v + Ld * math.sin(th) + L * thd * math.cos(th), Ld * math.cos(th) - L * thd * math.sin(th), e.yaw_rate, dt_s)
        return e

    def _learn_yaw_offset(self, e: Estimate, dt: float) -> None:
        """The raw yaw gyro's offset is learned against the wheels while they show no turn (no yaw command, so no slip is expected).
        It starts from the board's own bias, which a robot at rest has converged on."""
        p = self.p
        wheel_yaw = (e.v_wheel["right"] - e.v_wheel["left"]) / p.track
        self.wheel_yaw_lp += (wheel_yaw - self.wheel_yaw_lp) * min(dt / 0.1, 1.0)
        if self.yaw_rate_ref == 0.0 and abs(self.wheel_yaw_lp) < 0.1:
            self.yaw_offset += (e.yaw_rate - self.wheel_yaw_lp) * dt / p.yaw_offset_tau

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
                    self.events.append((self.t, "transform down ignored (no auxiliary wheels on this robot)"))
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
            self.yaw_ref_limited = 0.0
            self.roll_int = 0.0
            self.roll_force_int = 0.0
            self.leg_force_cmd = {}             # no leg force in the safe state
            self.telemetry ={"mode": int(self.mode), "fault": self.fault, "armed": self.armed, "L_ref": self.L_ref.value,
                              "v_ref": 0.0, "in_control": self.link.in_control, "tau_yaw": 0.0, "dL": 0.0, "T": 0.0, "Tp": 0.0,
                              "tau_split": 0.0, "d_theta": 0.0}
            self.law.compute(e, dict(v_ref=0.0, yaw_rate_ref=0.0, pitch_ref=0.0, L_ref=self.L_ref.value,
                                     L_ref_side={"left": self.L_ref.value, "right": self.L_ref.value}, balance=False,
                                     tau_yaw=0.0), self)
            return {j: torque_command(0.0, 0.0) for j in self.left_joints + self.right_joints}

        # yaw (differential wheel torque / velocity) and roll (leg-length difference) terms --
        if balance:
            yaw_ref = self.yaw_rate_ref
            if p.kw_yaw != 0.0:
                self.yaw_ref_limited += clamp_abs(self.yaw_rate_ref - self.yaw_ref_limited, p.yaw_ref_rate * dt)
                yaw_ref = self.yaw_ref_limited
            yaw_err = yaw_ref - e.yaw_rate
            self.yaw_int = clamp_abs(self.yaw_int + yaw_err * dt, p.yaw_int_limit)
            tau_yaw = p.kp_yaw * yaw_err + p.ki_yaw * self.yaw_int
            if p.kw_yaw != 0.0:
                tau_yaw += p.kw_yaw * (yaw_ref - (e.v_wheel["right"] - e.v_wheel["left"]) / p.track)
        else:
            tau_yaw = 0.0
            self.yaw_int = 0.0
            self.yaw_ref_limited = 0.0
        # roll: geometric feed-forward (leg-length difference ~ track * tan(roll)) + PI on the error (review B8)
        compliant = p.roll_compliant and balance
        F_roll = 0.0
        leg_limit: Dict[str, float] = {}
        if compliant:
            self.roll_ref_limited += clamp_abs(self.roll_ref - self.roll_ref_limited, p.roll_ref_rate * dt)
            roll_err = self.roll_ref_limited - e.roll
            leg_limit = {s: self._leg_limit_force(e.L0[s], e.L0_dot[s]) for s in ("left", "right")}
            F_wanted = p.kp_roll_force * roll_err + self.roll_force_int - p.kd_roll_force * e.roll_rate
            F_roll = clamp_abs(F_wanted, p.roll_force_limit)
            if F_roll == F_wanted and not any(leg_limit.values()):      # conditional integration: not against a limit
                self.roll_force_int = clamp_abs(self.roll_force_int + p.ki_roll_force * roll_err * dt, p.roll_force_int_limit)
            dL = 0.0
        else:
            self.roll_force_int = 0.0
            self.roll_ref_limited = self.roll_ref
            roll_err = self.roll_ref - e.roll
            self.roll_int = clamp_abs(self.roll_int + roll_err * dt, p.roll_int_limit / max(p.ki_roll, 1e-6))
            dL = clamp_abs(p.track * math.tan(self.roll_ref) + p.kp_roll * roll_err + p.ki_roll * self.roll_int
                           - p.kd_roll * e.roll_rate, 0.05)
        L_ref_side = {"left": clamp(self.L_ref.value + 0.5 * dL, p.L_min, p.L_max),
                      "right": clamp(self.L_ref.value - 0.5 * dL, p.L_min, p.L_max)}
        v_ref = self.v_ref if balance else 0.0
        limited = hold_int = False
        if p.uneven_speed > 0.0 and balance:
            limit = self._uneven_limit(abs(e.L0["left"] - e.L0["right"]), dt)
            limited = abs(v_ref) > limit
            hold_int = p.uneven_hold_int and limit < p.max_forward and abs(v_ref) > 0.02
            v_ref = clamp_abs(v_ref, limit)
        self.telemetry = {"mode": int(self.mode), "fault": self.fault, "L_ref": self.L_ref.value, "v_ref": v_ref,
                          "in_control": self.link.in_control, "tau_yaw": tau_yaw, "dL": dL}

        # balance law --------------------------------------------------------------------
        refs = dict(v_ref=v_ref, yaw_rate_ref=self.yaw_rate_ref if balance else 0.0, pitch_ref=self.pitch_ref,
                    L_ref=self.L_ref.value, L_ref_side=L_ref_side, balance=balance, tau_yaw=tau_yaw,
                    v_brake=p.uneven_brake if limited else 0.0, v_lean=p.uneven_lean if limited else 0.0, v_brake_tau=p.uneven_brake_tau if limited else 0.0,
                    hold_integrators=hold_int)
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
        L_mean_dot = 0.5 * (e.L0_dot["left"] + e.L0_dot["right"])
        L_mean_ref = self.L_ref.value
        L_held, L_held_dot = e.L0_mean, L_mean_dot
        if compliant:
            d = e.L0["left"] - e.L0["right"]
            if p.suspension:                    # the longer leg (blend: no step in the force when the legs cross)
                w = min(abs(d) / p.suspension_blend, 1.0)
                L_held += 0.5 * abs(d) * w
                L_held_dot += 0.5 * math.copysign(1.0, d) * (e.L0_dot["left"] - e.L0_dot["right"]) * w
            if p.mean_follows_range:
                spread = abs(d) if p.suspension else 0.5 * abs(d)     # how far below the held length the shorter leg is
                low, high = p.L_min - p.L_min_margin, p.L_max + p.L_max_margin
                if p.suspension and p.suspension_share > 0.0 and spread > p.suspension_share and high - low > p.suspension_share:
                    L_mean_ref += (spread - p.suspension_share) * max(high - L_mean_ref, 0.0) / (high - low - p.suspension_share)
                lo, hi = low + spread, high if p.suspension else high - spread
                L_mean_ref = clamp(L_mean_ref, lo, hi) if lo <= hi else 0.5 * (lo + hi)
        out: Dict[str, ServoCommand] = {}
        for side, sign in (("left", -1.0), ("right", +1.0)):
            if compliant:                       # positive roll error: left leg longer, right leg shorter
                F = p.kp_leg * (L_mean_ref - L_held) - p.kd_leg * L_held_dot - sign * F_roll + leg_limit[side] - p.kd_leg_own * (e.L0_dot[side] - L_held_dot)
            else:
                F = p.kp_leg * (L_ref_side[side] - e.L0[side]) - p.kd_leg * e.L0_dot[side]
            F += p.leg_force(e.L0[side], e.theta_v[side]) if p.leg_force is not None else F_ff / max(math.cos(e.theta_v[side]), 0.5)
            self.leg_force_cmd[side] = F
            Tp_side = 0.5 * Tp + sign * 0.5 * tau_split
            sv = self.servo_valid if self.servo_valid else obs["servo"]        # held sample on invalid data (r2 M1)
            tau_hip, tau_crank = self.leg.vmc_torques(F, Tp_side, sv[f"{side}_hip"]["position"],
                                                      sv[f"{side}_{p.leg_joint}"]["position"])
            tau_wheel = 0.5 * T + sign * 0.5 * tau_yaw
            if p.wheel_reaction_ff and math.isfinite(tau_wheel):
                tau_crank += clamp_abs(tau_wheel, p.tau_wheel_max) * self.leg.wheel_reaction_ratio(
                    sv[f"{side}_hip"]["position"], sv[f"{side}_{p.leg_joint}"]["position"])
            for name, tau, lim in ((f"{side}_hip", tau_hip, p.tau_hip_max), (f"{side}_{p.leg_joint}", tau_crank, p.tau_hip_max),
                                   (f"{side}_wheel", tau_wheel, p.tau_wheel_max)):
                if not math.isfinite(tau):          # never send NaN to the motors (review B13); and never silently:
                    tau = 0.0                       # a non-finite command is a FAULT with an event (review r1 M2)
                    if not self.fault:
                        self.fault = True
                        self.events.append((self.t, f"FAULT: non-finite torque command on {name}"))
                out[name] = torque_command(clamp_abs(tau, lim), lim)
        self.telemetry.update({"T": T, "Tp": Tp, "tau_split": tau_split, "d_theta": d_theta, "armed": self.armed, "F_roll": F_roll})
        return out

    def _uneven_limit(self, diff: float, dt: float) -> float:
        """Speed limit from the leg-length difference: comes down at once, goes back up after uneven_hold."""
        p = self.p
        s = clamp((diff - p.uneven_d0) / max(p.uneven_d1 - p.uneven_d0, 1e-6), 0.0, 1.0)
        now = p.max_forward + s * (p.uneven_speed - p.max_forward)
        if now <= self.v_limit:
            self.v_limit = now
            self.v_limit_hold = p.uneven_hold
        elif self.v_limit_hold > 0.0:
            self.v_limit_hold -= dt
        else:
            self.v_limit = min(now, self.v_limit + p.uneven_rise * dt)
        return self.v_limit

    def _leg_limit_force(self, L: float, L_dot: float) -> float:
        """Soft wall of one leg's range for the compliant roll: extension force beyond the range, damped only outwards."""
        p = self.p
        high = p.L_max + p.L_max_margin
        if L > high:
            return -p.kp_leg_limit * (L - high) - p.kd_leg_limit * max(L_dot, 0.0)
        low = p.L_min - p.L_min_margin
        if L < low:
            return p.kp_leg_limit * (low - L) - p.kd_leg_limit * min(L_dot, 0.0)
        return 0.0
