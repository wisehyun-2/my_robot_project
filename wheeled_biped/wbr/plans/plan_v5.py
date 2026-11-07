"""The v5 robot's closed loop: its own URDF model (wbr.models.build_v5), the DIABLO-style MCU with the LQI6 designed on that model,
and the IMU the way the real board delivers it.

Same runtime structure as Plan 4 (the class is shared), but nothing of upkie's: model, joint signs, tyre, track, motor limits, IMU
mount come from description.json, written from the Fusion model; the gains from wbr.bench.ake_cad.v5_gains.

IMU chain:
    MuJoCo site "imu" at the chip on the pi3hat, raw axes of the board (+x CAN / HDMI = the robot's rear, +y USB = its right,
    +z Pi -> hat = up; the front is the rocker switch side):
    gyro and specific force there, with the lever arm to the rest of the robot -> sensor noise -> the firmware's attitude filter
    (wbr.observers.pi3hat_ars) -> quaternion IMU -> ARS frame and bias-corrected rate -> sample delay -> MCU (BaseOrientation with
    this robot's R_base_to_imu).
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from wbr.common.rotations import quat_to_matrix, roll_pitch_yaw, rot_y
from wbr.models import build_v5 as BV
from wbr.observers.pi3hat_ars import Pi3HatAttitude
from wbr.plans import plan4_upkie_lqi6 as P4
from wbr.plans.common import SensorNoise


@dataclass
class PlanV5Config(P4.Plan4Config):
    model_dir: str = BV.OUT_DIR.name
    yaw_offset_tau: float = 20.0    # s: yaw rate from the raw gyro, offset learned against the wheels at standstill (McuParams)
    attitude_tau: float = 5.0       # s: the MCU's complementary filter with the IMU position in it (wbr.observers.body_attitude)
    imu_model: str = "pi3hat"       # "pi3hat": the firmware filter on the simulated chip; "ideal": true attitude + white noise
    imu_position_mm: Optional[tuple] = None     # study: another IMU position in the base frame, instead of the chip on the hat
    pitch_noise_deg: float = 0.0    # extra white noise on the filter's output (the design report's 0.2 deg, on top of the real chain)
    exact_leg_force: bool = True    # static leg force from the plant at the actual lean (SerialPlant.static_leg_force), False: F_ff / cos(lean)
    L_min_margin: float = 0.046     # soft wall of one leg 46 mm below L_min: 326 mm, above the Fusion interference check's lowest pose (286 mm)
    L_max_margin: float = 0.0       # soft wall at L_max = 485 mm, 15 mm from straight (500 mm); a wall too near straight lets the knee flip through it
    mean_follows_range: bool = True # the mean leg length goes down when the difference needs it (a tall bump under one wheel)
    suspension: bool = True         # over a speed bump the leg on it folds, the body keeps its height (McuParams.suspension)
    suspension_share: float = 0.05  # m: above this leg difference the other leg takes part, so neither leg reaches its end suddenly
    kd_leg_own: float = 30.0        # N s/m: each leg's own damper; the leg torque peak at a sharp 4 cm step at 1 m/s 12.4 -> 8.7 N m
    uneven_speed: float = 0.2       # m/s while the legs differ by uneven_d1 or more
    uneven_d0: float = 0.005        # m, the speed limit starts as soon as one leg starts to rise
    uneven_d1: float = 0.03         # m
    uneven_brake: float = 0.8       # m/s^2: gentle
    uneven_brake_tau: float = 0.2   # s: no jerk at the start of the braking
    uneven_lean: float = 1.0        # the leg leans back with the braking, so the speed follows the reference
    uneven_hold_int: bool = True    # no chasing of the position reference after a bump

    @classmethod
    def from_model(cls, **kw) -> "PlanV5Config":
        """The committed model with its gains (description.json)."""
        desc = json.loads((BV.OUT_DIR / BV.DESCRIPTION_FILE).read_text(encoding="utf-8"))
        args = dict(desc["plan_overrides"], params_json=str(BV.OUT_DIR / BV.PARAMS_FILE))
        args.update(kw)
        return cls(**args)


class PlanV5(P4.Plan4UpkieLQI6):
    torso_body = "base"

    @staticmethod
    def default_config():
        return PlanV5Config.from_model()

    def _leg_force_law(self):
        return self.plant.static_leg_force if self.cfg.exact_leg_force else None

    def _robot_constants(self) -> dict:
        d = json.loads((P4.MODELS_DIR / self.cfg.model_dir / BV.DESCRIPTION_FILE).read_text(encoding="utf-8"))
        self.description = d
        self.R_base_to_imu = np.array(d["imu"]["R_base_to_imu"])
        pos = np.array(d["imu"]["position_in_base"]) if self.cfg.imu_position_mm is None else np.array(self.cfg.imu_position_mm) / 1000.0
        return dict(wheel_radius=d["wheel_radius"], track=d["track"], tau_wheel_max=d["tau_wheel_max"], tau_leg_max=d["tau_leg_max"],
                    joint_signs=d["joint_signs"], imu_site="imu", R_base_to_imu=self.R_base_to_imu, imu_position=pos)

    def _lqi_design(self) -> dict:
        """LQR torque weights from this robot's weight and size, as shares of the motor peak (the plant's tau_*_max), and the gain table on the plant's leg-length grid."""
        from wbr.bench.ake_cad import LQR_R_HIP, LQR_R_HIP_ARM, LQR_R_WHEEL
        p = self.params
        m = p.extra["total_mass"]
        grid = p.leg.L0
        return dict(r_wheel_frac=LQR_R_WHEEL * m * p.g * p.R / p.tau_wheel_max, r_hip_frac=LQR_R_HIP * m * p.g * LQR_R_HIP_ARM / p.tau_hip_max,
                    table_L0=(float(grid[0]), float(grid[-1]), float(grid[1] - grid[0])))

    def _needs_spec_edit(self) -> bool:
        return super()._needs_spec_edit() or self.cfg.imu_position_mm is not None

    def _spec_edit(self, spec) -> None:
        super()._spec_edit(spec)
        if self.cfg.imu_position_mm is not None:
            site = next(s for s in spec.sites if s.name == "imu")
            site.pos = [x / 1000.0 for x in self.cfg.imu_position_mm]

    def _add_payload(self, spec) -> None:
        """(mass, x ahead of the axle, z above the axle or None = height of the body's COM) on the base, unknown to the controller."""
        mass, x_ahead, z_above = self.cfg.payload
        axle = self._axle_in_base(self.params.L0)
        base = next(b for b in spec.bodies if b.name == "base")
        z = self.params.b_z if z_above is None else axle[2] + z_above
        body = base.add_body(name="payload", pos=[axle[0] + x_ahead, 0.0, z])
        body.mass = mass
        body.ipos = [0.0, 0.0, 0.0]
        body.inertia = [1e-5, 1e-5, 1e-5]
        body.explicitinertial = True
        g = body.add_geom(name="payload_vis", type=P4.mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.03, 0, 0], rgba=[1, 0.2, 0.2, 0.8])
        g.contype = 0
        g.conaffinity = 0
        g.mass = 0.0

    def _make_noise(self) -> SensorNoise:
        """Gyro, accelerometer and encoder noise of the design report; the attitude noise comes out of the filter."""
        c = self.cfg
        if c.imu_model == "ideal":
            return SensorNoise(enabled=c.noise, seed=c.seed)
        return SensorNoise(enabled=c.noise, seed=c.seed, pitch_std=math.radians(c.pitch_noise_deg))

    def reset(self, *args, **kw) -> None:
        self._ars = None
        super().reset(*args, **kw)

    def _board_start_attitude(self, R_wi: np.ndarray) -> np.ndarray:
        """IMU frame -> ARS frame with the yaw of a board that started level (the firmware's sign: the hat side up is roll +180 deg).
        Its heading zero is the board's own x axis, whichever way the board is mounted on the robot."""
        yaw = roll_pitch_yaw(R_wi)[2]
        c, s = math.cos(yaw), math.sin(yaw)
        q = P4.matrix_to_quat(self.R_ARS_TO_WORLD.T @ np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]]) @ R_wi)
        return -q if q[1] < 0.0 else q

    def _observe(self) -> dict:
        raw = self.sim.observe()
        c = self.cfg
        imu = raw["imu"]
        R_wi = quat_to_matrix(imu["orientation"])       # true IMU frame -> world
        R_bi = self.R_base_to_imu
        if c.pitch_bias_deg != 0.0:                     # the board sits this far off its nominal mount, about the base y axis
            R_mount = R_bi @ rot_y(math.radians(c.pitch_bias_deg)) @ R_bi.T
            R_wi = R_wi @ R_mount
            imu["angular_velocity"] = R_mount.T @ imu["angular_velocity"]
            imu["linear_acceleration"] = R_mount.T @ imu["linear_acceleration"]
        imu["orientation"] = np.array([1.0, 0.0, 0.0, 0.0])     # (SensorNoise rotates it by the extra noise only)
        noisy = self.noise.apply(raw, self.dt)
        R_extra = quat_to_matrix(noisy["imu"]["orientation"])
        if c.imu_model == "ideal":
            noisy["imu"]["orientation"] = P4.matrix_to_quat(self.R_ARS_TO_WORLD.T @ R_wi @ R_extra)
        else:
            if self._ars is None:                       # a board that has been on for a while, at the robot's start attitude
                self._ars = Pi3HatAttitude(self.dt)
                self._ars.start_converged(self._board_start_attitude(R_wi))
            self._ars.process(noisy["imu"]["angular_velocity"], noisy["imu"]["linear_acceleration"])
            noisy["imu"]["orientation"] = P4.matrix_to_quat(quat_to_matrix(self._ars.quaternion) @ R_extra)
            noisy["imu"]["angular_velocity"] = self._ars.rate_rps
            noisy["imu"]["bias_rps"] = self._ars.bias_rps
        obs = self.delay.push(noisy)
        obs["sim"] = raw["sim"]
        return obs
