"""Plan 4 -- upkie hardware (serial hip-knee legs, qdd100 / mj5208) + the LQI6 controller designed on
the serial-leg three-body plant (plant_serial.py), inside the DIABLO-style host / MCU split.

Runtime structure:
  MCU program (DiabloMcu, `rate_hz` = 1 kHz or 200 Hz): estimator -> LQI6 balance law (T, Tp) -> leg
      length PD + weight feed-forward -> serial-leg VMC -> torque commands; mode machine, FAULT, host
      link timeouts (unchanged from Plan 2, only the leg kinematics and the plant differ)
  host  (HostEmulator at 50 Hz, or the stage-2 MPC reference generator): MotionCtrl set-points
upkie's joints are mirrored left/right; a sign adapter (UPKIE_JOINT_SIGNS) converts observations and
torque commands so that the MCU sees the symmetric "u-convention" of the leg model.

Robustness knobs used by the acceptance battery: mass_scale (all masses/inertias of the model, the
controller keeps the nominal parameters), friction (floor + tires), slope_deg (floor pitched about +y,
positive = uphill in +x), payload (extra point mass on the torso), pitch_bias_deg (IMU offset).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional

import mujoco
import numpy as np

from wbr import MODELS_DIR
from wbr.common.rotations import matrix_to_quat, quat_to_matrix, roll_pitch_yaw
from wbr.common.servo import ServoCommand, ServoModel, ServoProperties
from wbr.controllers.diablo_arch.host_link import HostEmulator, HostLink, HostLinkParams, RobotMode
from wbr.controllers.diablo_arch.mcu import DiabloMcu, McuParams
from wbr.controllers.lqi6.law import LQI6Law
from wbr.controllers.lqi6.plant_serial import SerialPlant, SerialPlantParams
from wbr.controllers.lqi6.serial_leg import UPKIE_JOINT_SIGNS, SerialLeg, SerialLegGeometry
from wbr.plans.common import Logger, ObservationDelay, SensorNoise, placed_base_pose
from wbr.sim.mujoco_sim import MujocoSim, SimConfig

JOINTS = ["left_hip", "left_knee", "left_wheel", "right_hip", "right_knee", "right_wheel"]
WHEELS = ["left_wheel", "right_wheel"]
WHEEL_RADIUS = 0.056        # BOM 112 mm RC tire (models/upkie_plan4); upkie_description's 0.05 is used by Plan 1
TRACK = 2 * 0.1984          # wheel centre distance (URDF)
TAU_WHEEL_MAX = 1.7         # mj5208 (URDF effort)
TAU_LEG_MAX = 16.0          # qdd100


@dataclass
class Plan4Config:
    rate_hz: float = 1000.0          # MCU / balancer rate (1000 or 200)
    host_hz: float = 50.0
    latency_ms: float = 0.0
    noise: bool = False
    seed: int = 0
    pitch_bias_deg: float = 0.0
    host_timeout_ms: float = 500.0
    scene: str = "scene.xml"
    model_dir: str = "upkie_plan4"   # upkie_description with the BOM 112 mm tires (docs/plan4_model.md)
    start_mode: RobotMode = RobotMode.STAND
    lqi_kwargs: Optional[dict] = None
    L_stand: Optional[float] = None  # default: plant params L0 (0.31 m)
    kp_leg: float = 800.0            # N/m   leg length PD (per leg)
    kd_leg: float = 40.0             # N s/m
    kp_yaw: float = 0.4              # N m per rad/s
    ki_yaw: float = 0.8
    yaw_int_limit: float = 1.0       # rad: ki_yaw * limit = 0.8 N m steady differential torque against the tire scrub
    kw_yaw: float = 0.0              # N m per rad/s of wheel-speed-difference yaw rate error (see McuParams)
    yaw_ref_rate: float = 3.0        # rad/s^2, see McuParams
    yaw_offset_tau: float = 0.0      # s, see McuParams (the v5 robot's pi3hat chain: wbr.plans.plan_v5)
    attitude_tau: float = 0.0        # s, see McuParams (same)
                                     # (sized with upkie_description's 40 mm cylinder tire, whose two rim contacts scrub
                                     # when yawing: 0.5 rad capped it at 0.4 N m and the yaw rate stalled at 0.84 rad/s
                                     # with mass x1.15 / friction 1.0; kept for the ellipsoid tire of models/upkie_plan4)
    kp_split: float = 60.0           # anti-split PD (N m/rad): upkie's light serial legs must be held together
    kd_split: float = 3.0            # firmly, otherwise differential wheel torque splits the legs instead of yawing
    friction_comp_Nm: float = 0.08   # hip stiction compensation per hip (model frictionloss 0.1 Nm)
    friction_comp_wheel_Nm: float = 0.016   # wheel stiction compensation per wheel (model 0.02 Nm)
    vel_window_s: float = 0.005      # MCU joint velocities from quantised-position differences over 5 ms (wheel-jitter fix:
                                     # the 1-step differentiated 14-bit encoder noise became 0.1 N m rms wheel torque chatter)
    friction_comp_static_tau: float = 1.0   # s, see McuParams: a payload / mass error changes the static hip torque;
                                     # comparing with the fixed Tp_ff then added a constant 0.16 N m (2 x 0.08)
    predict_ms: float = 0.0          # LQI state-predictor horizon (design constant = nominal latency, not latency_ms);
                                     # 10 ms: latency tolerance 15 -> 30 ms but the push pitch peaks double, so off
                                     # by default (docs/plan4_lqi6.md 3.5)
    wheel_reaction_ff: bool = False  # knee cancels the wheel motor's reaction on the shank (McuParams); off here so that
                                     # the upkie results stay as judged, on for the CAD robots (wbr.bench.ake_cad)
    roll_compliant: bool = False     # legs yield to a bump, the IMU roll levels the body (McuParams); off here, on for the CAD robots
    kp_roll_force: float = 0.0       # N/rad, gains and limits of the compliant roll: wbr.bench.ake_cad.outer_loop_gains
    ki_roll_force: float = 0.0
    kd_roll_force: float = 0.0
    roll_force_limit: Optional[float] = None   # None: the static leg force, where a wheel is just unloaded
    roll_force_int_limit: float = 0.0
    kp_leg_limit: float = 0.0
    kd_leg_limit: float = 0.0
    friction_comp_rate_gate: float = 0.1    # rad/s: the stiction compensation fades with the body pitch rate (review r1
                                     # M1: the relay term sustained a 13 Hz limit cycle from 12 ms latency; with the
                                     # gate 12 ms recovers cleanly, standing quality unchanged); 0 = off
    real_imu_convention: bool = True # feed the MCU the IMU sample the way the upkie spine reports it (ARS frame, IMU
                                     # mounted rotated 180 deg about y) so that the MCU's frame adapter runs in every test
    mass_scale: float = 1.0
    friction: Optional[float] = None
    slope_deg: float = 0.0
    payload: Optional[tuple] = None  # (mass [kg], x ahead of the axle [m], z above the axle [m] or None = torso COM height)
    L_min_margin: float = 0.02          # see McuParams
    L_max_margin: float = 0.0           # see McuParams
    mean_follows_range: bool = False    # see McuParams
    uneven_speed: float = 0.0           # see McuParams
    uneven_d0: float = 0.01             # see McuParams
    uneven_d1: float = 0.04             # see McuParams
    uneven_brake: float = 0.0           # see McuParams
    uneven_lean: float = 0.0            # see McuParams
    uneven_hold_int: bool = False       # see McuParams
    uneven_brake_tau: float = 0.0       # see McuParams
    suspension: bool = False            # see McuParams
    suspension_share: float = 0.0       # see McuParams
    kd_leg_own: float = 0.0             # see McuParams
    floor_humps: tuple = ()             # curved speed bumps, each (height [m], side +1 left / -1 right / 0 both wheels, x start [m], length [m], half width [m])
    floor_bump: Optional[tuple] = None  # bump under one wheel: (height [m], side +1 left / -1 right, x start [m], length of
                                     # each ramp [m], length of the top [m], half width [m]), centred on that tire's track
    drop_height: float = 0.0005
    params_json: Optional[str] = None
    host: str = "ramp"               # "ramp": plain MotionCtrl host; "mpc": stage-2 reference generator


class Plan4UpkieLQI6:
    def __init__(self, cfg: Plan4Config | None = None, law=None):
        self.cfg = cfg or self.default_config()
        c = self.cfg
        self.dt = 1.0 / c.rate_hz
        robot = self._robot_constants()
        self.wheel_radius, self.track = robot["wheel_radius"], robot["track"]
        self.params = SerialPlantParams.from_json(c.params_json or MODELS_DIR / c.model_dir / "plan4_params.json")
        p = self.params
        if c.L_stand is not None:
            p.L0 = c.L_stand
        self.plant = SerialPlant(p)
        self.leg = SerialLeg(SerialLegGeometry(L1=p.extra["L1"], L2=p.extra["L2"], knee_sign=p.extra["knee_sign"]))
        self.signs = dict(robot["joint_signs"])
        self.hip_above_base = p.hip_above_base

        servo = ServoModel({j: ServoProperties(20.0, 1.0, robot["tau_wheel_max"] if "wheel" in j else robot["tau_leg_max"]) for j in JOINTS})
        self.sim = MujocoSim(SimConfig(xml_path=MODELS_DIR / c.model_dir / c.scene, joint_names=JOINTS, wheel_joints=WHEELS,
                                       base_body="base", imu_site=robot["imu_site"], control_dt=self.dt,
                                       wheel_geoms={"left_wheel": "left_tire", "right_wheel": "right_tire"},
                                       spec_edit=self._spec_edit if self._needs_spec_edit() else None,
                                       model_edit=self._model_edit), servo)
        self.standing_base_z = None

        # controller -------------------------------------------------------------------------
        # LQI6 weights of docs/plan4_lqi6.md: stronger velocity integral (position holding against joint stiction),
        # more hip authority, wheel torque penalised for the 1.7 Nm mj5208; velocity reference shaped by a 2 m/s^2
        # ramp + 0.3 s pre-filter with the integrator frozen while the reference moves (overshoot < 10 %).
        # q_att: 7th state = integral of the static body tilt e_att (encoders + actuator torques, IMU-bias free), so
        # that the TRUE body pitch goes to zero with an IMU bias / payload / slope (items 16, 17: -1.37 / +0.95 deg
        # before); the wheel-channel gains change by < 5 %, the hip-channel theta_dot / v / z1 gains drop by 24-50 %
        # (z2 takes over part of the static hip work): push pitch peaks +1..1.5 deg, transient items still pass
        # q_int 0.2 / q_pos 0.5: velocity-integral + position-error integral hold the position against constant
        # disturbance torques (5 deg slope, payload, IMU bias) without a 10-20 cm offset; r_wheel 0.2 keeps the
        # 10 deg tilt recovery below the 1.36 Nm wheel margin and short of the slip limit on mu = 0.5 floors;
        # freeze_tol 0.05 lets the integrators resume once the shaped reference is within 0.05 m/s of the target
        # (uphill velocity steps otherwise keep a 0.15 m/s error).  docs/plan4_lqi6.md
        lqi_kwargs = dict(table_L0=(0.22, 0.34, 0.005), q_int=0.2, q_v=0.5, r_hip_frac=0.3, r_wheel_frac=0.2,
                          v_ramp_rate=2.0, v_ref_tau=0.3, freeze_int_on_ramp=True, freeze_tol=0.05, q_att=0.2, q_pos=0.5)
        if c.predict_ms > 0.0:
            lqi_kwargs["pred_steps"] = int(round(c.predict_ms * 1e-3 * c.rate_hz))
        # (the stage-2 MPC host flags MotionCtrl.host_shaped: the MCU then switches the LQI6 to plain tracking and
        #  falls back to this shaping on a link timeout or when the host's QP fails)
        lqi_kwargs.update(self._lqi_design())
        lqi_kwargs.update(c.lqi_kwargs or {})
        self.law = law or LQI6Law(self.plant, Ts=self.dt, theta_eq=self.plant.theta_eq, Tp_ff=self.plant.Tp_eq(), **lqi_kwargs)
        self.link = HostLink(HostLinkParams(timeout_ms=c.host_timeout_ms))
        self.host = self._make_host()
        mcu_params = McuParams(dt=self.dt, wheel_radius=self.wheel_radius, track=self.track, mass_total=p.extra["total_mass"],
                               mass_wheels=p.m_w, L_min=p.L_min, L_max=p.L_max, L_stand=p.L0, kp_leg=c.kp_leg, kd_leg=c.kd_leg,
                               kp_yaw=c.kp_yaw, ki_yaw=c.ki_yaw, yaw_int_limit=c.yaw_int_limit, kw_yaw=c.kw_yaw,
                               yaw_ref_rate=c.yaw_ref_rate, yaw_offset_tau=c.yaw_offset_tau, attitude_tau=c.attitude_tau,
                               imu_position=robot.get("imu_position"),
                               kp_split=c.kp_split, kd_split=c.kd_split,
                               tau_hip_max=robot["tau_leg_max"], tau_wheel_max=robot["tau_wheel_max"],
                               leg_joint="knee", F_ff_N=self._static_leg_force(), leg_force=self._leg_force_law(), friction_comp_Nm=c.friction_comp_Nm,
                               friction_comp_wheel_Nm=c.friction_comp_wheel_Nm,
                               friction_comp_static_tau=c.friction_comp_static_tau, vel_window_s=c.vel_window_s,
                               friction_comp_rate_gate=c.friction_comp_rate_gate,
                               wheel_reaction_ff=c.wheel_reaction_ff,
                               roll_compliant=c.roll_compliant, kp_roll_force=c.kp_roll_force, ki_roll_force=c.ki_roll_force,
                               kd_roll_force=c.kd_roll_force,
                               roll_force_limit=self._static_leg_force() if c.roll_force_limit is None else c.roll_force_limit,
                               roll_force_int_limit=c.roll_force_int_limit, kp_leg_limit=c.kp_leg_limit, kd_leg_limit=c.kd_leg_limit,
                               L_min_margin=c.L_min_margin, L_max_margin=c.L_max_margin, mean_follows_range=c.mean_follows_range, uneven_speed=c.uneven_speed,
                               uneven_d0=c.uneven_d0, uneven_d1=c.uneven_d1, uneven_brake=c.uneven_brake, uneven_lean=c.uneven_lean, uneven_hold_int=c.uneven_hold_int, uneven_brake_tau=c.uneven_brake_tau, suspension=c.suspension, suspension_share=c.suspension_share, kd_leg_own=c.kd_leg_own,
                               imu_rotation_base_to_imu=robot["R_base_to_imu"] if c.real_imu_convention else None,
                               imu_rotation_ars_to_world=self.R_ARS_TO_WORLD if c.real_imu_convention else None)
        self.mcu = DiabloMcu(self.law, mcu_params, self.leg, self.link)
        self.noise = self._make_noise()
        self.delay = ObservationDelay(int(math.ceil(c.latency_ms * 1e-3 * c.rate_hz - 1e-9)), dt=self.dt)   # never round a latency away
        self.up_cmd = 0.0
        self.reset()

    # ------------------------------------------------------------------ what differs from robot to robot (plan_v5.PlanV5)
    torso_body = "torso"            # the body a push acts on

    def _leg_force_law(self):
        """McuParams.leg_force: None keeps F_ff / cos(lean) (upkie's verdicts were made with it)."""
        return None

    @staticmethod
    def default_config():
        return Plan4Config()

    def _robot_constants(self) -> dict:
        """The module constants are read here, at construction: wbr.bench.ake_sizing.patched_plan4 patches them for scaled robots."""
        return dict(wheel_radius=WHEEL_RADIUS, track=TRACK, tau_wheel_max=TAU_WHEEL_MAX, tau_leg_max=TAU_LEG_MAX, joint_signs=UPKIE_JOINT_SIGNS,
                    imu_site="base", R_base_to_imu=self.R_BASE_TO_IMU)

    def _lqi_design(self) -> dict:
        return {}

    def _needs_spec_edit(self) -> bool:
        return bool(self.cfg.payload or self.cfg.floor_bump or self.cfg.floor_humps)

    def _make_noise(self) -> SensorNoise:
        c = self.cfg
        return SensorNoise(enabled=c.noise, seed=c.seed, pitch_bias=math.radians(c.pitch_bias_deg))

    def _static_leg_force(self) -> float:
        """Static force along one virtual leg: the body weight plus the part of the leg weight carried at
        the leg COM height (c / L0 ~ 0.42 for upkie), see docs/plan4_model.md; a massless-leg VMC with the
        full robot weight would over-extend the leg by ~7 mm."""
        p = self.params
        leg = p.leg.at(p.L0)
        return 0.5 * (p.m_b + (leg["c"] / p.L0) * leg["m_l"]) * p.g

    def _make_host(self):
        """Host process: plain MotionCtrl publisher (stage 1) or the MPC reference generator (stage 2).  Both expose
        set_command(**fields) / tick(t) / enabled, the MPC host additionally violations, solve_times_ms,
        fallback_used and force_fail (wbr.controllers.mpc_host.reference_mpc.MpcHost)."""
        if self.cfg.host == "mpc":
            from wbr.controllers.mpc_host.reference_mpc import MpcHost
            return MpcHost(self.link, self, publish_hz=self.cfg.host_hz)
        return HostEmulator(self.link, publish_hz=self.cfg.host_hz)

    # ------------------------------------------------------------------ model variants
    def _axle_in_base(self, L0: float) -> np.ndarray:
        """Wheel axle position in the base frame for the equilibrium posture at leg length L0."""
        qh, qk = self.leg.inverse(L0, self.plant.theta_eq(L0))
        p_axle, _, _ = self.leg.forward(qh, qk)
        return np.array([p_axle[0], 0.0, -self.hip_above_base + p_axle[1]])

    def _spec_edit(self, spec) -> None:
        if self.cfg.payload:
            self._add_payload(spec)
        if self.cfg.floor_bump:
            self._add_floor_bump(spec)
        for i, hump in enumerate(self.cfg.floor_humps):
            self._add_floor_hump(spec, i, *hump)

    def _add_floor_bump(self, spec) -> None:
        """Ramp up, flat top, ramp down: three boxes of floor material under one wheel track (a ramp length near 0 = a step)."""
        h, side, x0, ramp, top, half_width = self.cfg.floor_bump
        floor = next(g for g in spec.geoms if g.name == "floor")
        thickness = 0.05

        def box(name, x_a, z_a, x_b, z_b):
            a = math.atan2(z_b - z_a, x_b - x_a)
            g = spec.worldbody.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX,
                                        size=[0.5 * math.hypot(x_b - x_a, z_b - z_a), half_width, thickness],
                                        pos=[0.5 * (x_a + x_b) + thickness * math.sin(a), side * 0.5 * self.track,
                                             0.5 * (z_a + z_b) - thickness * math.cos(a)],
                                        quat=[math.cos(-0.5 * a), 0.0, math.sin(-0.5 * a), 0.0])
            g.contype, g.conaffinity = floor.contype, floor.conaffinity
            g.friction = floor.friction
            g.condim = floor.condim

        box("bump_up", x0, 0.0, x0 + ramp, h)
        box("bump_top", x0 + ramp, h, x0 + ramp + top, h)
        box("bump_down", x0 + ramp + top, h, x0 + 2 * ramp + top, 0.0)

    def _add_floor_hump(self, spec, index: int, h: float, side: float, x0: float, length: float, half_width: float) -> None:
        """A curved speed bump, z = h/2 (1 - cos(2 pi s / length)): thin boxes of floor material, each following one chord of the curve."""
        floor = next(g for g in spec.geoms if g.name == "floor")
        thickness = 0.05
        n = max(16, int(math.ceil(length / 0.03)))
        z = lambda s: 0.5 * h * (1.0 - math.cos(2.0 * math.pi * s / length))
        y = side * 0.5 * self.track
        for k in range(n):
            xa, xb = x0 + length * k / n, x0 + length * (k + 1) / n
            za, zb = z(length * k / n), z(length * (k + 1) / n)
            a = math.atan2(zb - za, xb - xa)
            g = spec.worldbody.add_geom(name=f"hump{index}_{k}", type=mujoco.mjtGeom.mjGEOM_BOX,
                                        size=[0.5 * math.hypot(xb - xa, zb - za) + 0.002, half_width, thickness],
                                        pos=[0.5 * (xa + xb) + thickness * math.sin(a), y, 0.5 * (za + zb) - thickness * math.cos(a)],
                                        quat=[math.cos(-0.5 * a), 0.0, math.sin(-0.5 * a), 0.0])
            g.contype, g.conaffinity = floor.contype, floor.conaffinity
            g.friction = floor.friction
            g.condim = floor.condim
            g.rgba = [0.75, 0.6, 0.2, 1.0]

    def _add_payload(self, spec) -> None:
        mass, x_ahead, z_above = self.cfg.payload
        axle = self._axle_in_base(self.params.L0)
        torso = next(b for b in spec.bodies if b.name == "torso")
        # torso frame = base frame shifted by (0, 0, -0.1); torso-group COM is 0.119 m below the base origin
        z = (-0.119 + 0.1) if z_above is None else (axle[2] + z_above + 0.1)
        body = torso.add_body(name="payload", pos=[axle[0] + x_ahead, 0.0, z])
        body.mass = mass
        body.ipos = [0.0, 0.0, 0.0]
        body.inertia = [1e-5, 1e-5, 1e-5]
        body.explicitinertial = True
        g = body.add_geom(name="payload_vis", type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.03, 0, 0], rgba=[1, 0.2, 0.2, 0.8])
        g.contype = 0
        g.conaffinity = 0
        g.mass = 0.0

    def _model_edit(self, m) -> None:
        c = self.cfg
        if c.mass_scale != 1.0:
            # robot bodies only: the item-17 payload is a test load defined as exactly 0.3 kg
            scale = np.full(m.nbody, c.mass_scale)
            scale[0] = 1.0
            pid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "payload")
            if pid >= 0:
                scale[pid] = 1.0
            m.body_mass[:] *= scale
            m.body_inertia[:] *= scale[:, None]
            # recompute the derived constants (body_subtreemass = sim.total_mass(), used for the push force
            # W = total mass x g; invweight0 of the constraint solver) as the compiler would for these masses;
            # without this the subtree mass stayed at the nominal 5.34 kg for every mass scale
            mujoco.mj_setConst(m, mujoco.MjData(m))
        if c.friction is not None:
            for name in ("floor", "left_tire", "right_tire"):
                gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name)
                m.geom_friction[gid, 0] = c.friction
        if c.slope_deg != 0.0:
            a = -math.radians(c.slope_deg)       # plane normal tilted towards -x: floor rises with +x
            gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
            m.geom_quat[gid] = [math.cos(a / 2), 0.0, math.sin(a / 2), 0.0]
            # the compiler flagged the (identity-pose) floor as "same frame as the world body"; with the flag set
            # MuJoCo ignores geom_quat and the floor stayed flat (items 12 / 13 ran on level ground before)
            m.geom_sameframe[gid] = 0

    # ------------------------------------------------------------------ sign adapter
    def _obs_to_mcu(self, obs: dict) -> dict:
        o = {k: v for k, v in obs.items() if k != "sim"}      # the MCU never sees simulator ground truth (review r1 m13)
        o["servo"] = {n: {"position": self.signs[n] * s["position"], "velocity": self.signs[n] * s["velocity"],
                          "torque": self.signs[n] * s["torque"]} for n, s in obs["servo"].items()}
        return o

    def _cmds_from_mcu(self, cmds: Dict[str, ServoCommand]) -> Dict[str, ServoCommand]:
        out = {}
        for n, cmd in cmds.items():
            sg = self.signs[n]
            c2 = cmd.copy()
            c2.feedforward_torque = sg * cmd.feedforward_torque
            c2.velocity = sg * cmd.velocity
            if not math.isnan(cmd.position):
                c2.position = sg * cmd.position
            out[n] = c2
        return out

    # ------------------------------------------------------------------ posture / API
    def _leg_pose(self, L0: float, theta_v: float) -> Dict[str, float]:
        qh, qk = self.leg.inverse(L0, theta_v)
        return {"left_hip": qh * self.signs["left_hip"], "left_knee": qk * self.signs["left_knee"],
                "right_hip": qh * self.signs["right_hip"], "right_knee": qk * self.signs["right_knee"]}

    def reset(self, pitch: float = 0.0, yaw: float = 0.0, joint_positions: Optional[Dict[str, float]] = None,
              base_linvel=None, mode: Optional[RobotMode] = None) -> None:
        p = self.mcu.p
        L0 = float(np.clip(p.L_stand + p.height_gain * self.up_cmd, p.L_min, p.L_max))
        if joint_positions is None:
            joint_positions = self._leg_pose(L0, self.plant.theta_eq(L0))
        qh = joint_positions["left_hip"] * self.signs["left_hip"]
        qk = joint_positions["left_knee"] * self.signs["left_knee"]
        p_axle, _, _ = self.leg.forward(qh, qk)
        wheel_offset = np.array([p_axle[0], 0.0, -self.hip_above_base + p_axle[1]])
        pos, quat = placed_base_pose(wheel_offset, self.wheel_radius, pitch, yaw)
        slope = math.radians(self.cfg.slope_deg)
        pos[2] += self.cfg.drop_height + self.wheel_radius * (1.0 / math.cos(slope) - 1.0)
        self.sim.reset(base_pos=pos, base_quat=quat, joint_positions=joint_positions, base_linvel=base_linvel)
        if self.standing_base_z is None:
            self.standing_base_z = float(pos[2])
        self.mcu.reset(mode or self.cfg.start_mode)
        self.host = self._make_host()
        self.host.set_command(forward=0.0, left=0.0, up=self.up_cmd, stand_mode=True)
        self.delay = ObservationDelay(self.delay.delay_steps, dt=self.dt)
        self.log = Logger()
        self.k = 0
        self.observation = self._observe()

    # upkie's pi3hat IMU: mounted rotated 180 deg about y on the torso (imu body quat 0 0 1 0) and reporting its
    # orientation in the ARS frame (x forward, y right, z down).  The simulator's IMU sensors sit on the base site
    # (world-frame base orientation, base-frame rates); the sample is re-expressed the way the spine reports it and
    # the MCU runs the same BaseOrientation adapter as the real robot (review r1 M3).
    R_ARS_TO_WORLD = np.diag([1.0, -1.0, -1.0])
    R_BASE_TO_IMU = np.diag([-1.0, 1.0, -1.0])

    def _observe(self) -> dict:
        raw = self.sim.observe()
        noisy = self.noise.apply(raw, self.dt)            # noise / bias in the base frame, then the IMU convention
        if self.cfg.real_imu_convention:
            imu = noisy["imu"]
            R_bw = quat_to_matrix(imu["orientation"])
            imu["orientation"] = matrix_to_quat(self.R_ARS_TO_WORLD.T @ R_bw @ self.R_BASE_TO_IMU.T)
            imu["angular_velocity"] = self.R_BASE_TO_IMU @ np.asarray(imu["angular_velocity"], dtype=float)
            imu["linear_acceleration"] = self.R_BASE_TO_IMU @ np.asarray(imu["linear_acceleration"], dtype=float)
        obs = self.delay.push(noisy)
        obs["sim"] = raw["sim"]
        return obs

    def set_height_offset(self, dh: float) -> None:
        self.up_cmd = dh / self.mcu.p.height_gain
        self.host.set_command(up=self.up_cmd)

    def step(self, target_ground_velocity: float = 0.0, target_yaw_velocity: float = 0.0) -> dict:
        obs = self.observation
        self.host.set_command(forward=target_ground_velocity, left=target_yaw_velocity)
        self.host.tick(self.sim.time)
        cmds = self.mcu.tick(self._obs_to_mcu(obs))
        self.sim.set_commands(self._cmds_from_mcu(cmds))
        self.sim.step()
        self.k += 1
        self.observation = self._observe()
        o = self.observation
        e, tm = self.mcu.est, self.mcu.telemetry
        base = o["sim"]["base"]
        sv = o["servo"]
        roll_true, pitch_true, _ = roll_pitch_yaw(quat_to_matrix(base["orientation"]))     # ground truth (acceptance criteria)
        self.log.log(t=self.sim.time, pitch=e.pitch, pitch_true=pitch_true, pitch_rate=e.pitch_rate, roll=e.roll, roll_true=roll_true, theta_v=e.theta_v_mean,
                     L0=e.L0_mean, L0_left=e.L0["left"], L0_right=e.L0["right"], L_ref=tm.get("L_ref", 0.0),
                     ground_vel=e.v, ground_pos=e.position,
                     x=base["position"][0], y=base["position"][1], z=base["position"][2], vx=base["linear_velocity"][0],
                     yaw_rate=base["angular_velocity"][2], T=tm.get("T", 0.0), Tp=tm.get("Tp", 0.0),
                     tau_lw=sv["left_wheel"]["torque"], tau_rw=sv["right_wheel"]["torque"],
                     tau_lh=sv["left_hip"]["torque"], tau_lk=sv["left_knee"]["torque"],
                     tau_rh=sv["right_hip"]["torque"], tau_rk=sv["right_knee"]["torque"],
                     qd_lw=sv["left_wheel"]["velocity"], qd_rw=sv["right_wheel"]["velocity"],
                     qd_lh=sv["left_hip"]["velocity"], qd_lk=sv["left_knee"]["velocity"],
                     qd_rh=sv["right_hip"]["velocity"], qd_rk=sv["right_knee"]["velocity"],
                     mode=tm.get("mode", 0), in_control=float(tm.get("in_control", False)),
                     v_cmd=target_ground_velocity, w_cmd=target_yaw_velocity)
        o["base_orientation"] = {"pitch": e.pitch, "angular_velocity": np.array([e.roll_rate, e.pitch_rate, e.yaw_rate])}
        return o

    @property
    def fallen(self) -> bool:
        base = self.observation["sim"]["base"]
        true_pitch = roll_pitch_yaw(quat_to_matrix(base["orientation"]))[1]      # ground truth, not the estimate (n5)
        return abs(true_pitch) > 1.0 or base["position"][2] < 0.6 * self.standing_base_z

    def push(self, force_x: float, duration: float) -> None:
        """Horizontal push applied at the robot's total COM (acceptance criteria definition)."""
        self.sim.push([force_x, 0.0, 0.0], duration, body=self.torso_body, point=self.sim.com())

    def run(self, seconds: float, v=0.0, w=0.0) -> bool:
        for _ in range(int(round(seconds / self.dt))):
            self.step(v, w)
            if self.fallen:
                return False
        return True
