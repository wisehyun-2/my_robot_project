"""Plan 2 -- DIABLO architecture (6 kg DIABLO-lite five-bar robot) + the LQI6 controller of the
design report.  MCU loop at 1 kHz (torque mode), host MotionCtrl commands at 50 Hz with the OSDK
timeout / heartbeat semantics.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np

from wbr import MODELS_DIR
from wbr.common.servo import ServoModel, ServoProperties
from wbr.controllers.diablo_arch.host_link import HostEmulator, HostLink, HostLinkParams, RobotMode
from wbr.controllers.diablo_arch.mcu import DiabloMcu, McuParams
from wbr.controllers.lqi6.fivebar import FiveBarLeg, LegGeometry
from wbr.controllers.lqi6.law import LQI6Law
from wbr.controllers.lqi6.plant6 import Plant6, diablo_lite_params
from wbr.plans.common import Logger, ObservationDelay, SensorNoise, placed_base_pose
from wbr.sim.mujoco_sim import MujocoSim, SimConfig

JOINTS = ["left_hip", "left_crank", "left_wheel", "right_hip", "right_crank", "right_wheel"]
WHEELS = ["left_wheel", "right_wheel"]


@dataclass
class Plan2Config:
    mcu_hz: float = 1000.0
    host_hz: float = 50.0
    latency_ms: float = 0.0
    noise: bool = False
    seed: int = 0
    host_timeout_ms: float = 500.0
    scene: str = "scene.xml"
    start_mode: RobotMode = RobotMode.STAND
    lqi_kwargs: Optional[dict] = None


class Plan2DiabloLQI6:
    def __init__(self, cfg: Plan2Config | None = None, law=None):
        self.cfg = cfg or Plan2Config()
        c = self.cfg
        self.dt = 1.0 / c.mcu_hz
        from wbr.models.build_diablo_lite import DiabloLiteParams
        self.params = DiabloLiteParams()
        p = self.params
        servo = ServoModel({j: ServoProperties(20.0, 1.0, p.tau_hip if "wheel" not in j else p.tau_wheel) for j in JOINTS})
        self.sim = MujocoSim(SimConfig(xml_path=MODELS_DIR / "diablo_lite" / c.scene, joint_names=JOINTS, wheel_joints=WHEELS,
                                       base_body="base", imu_site="imu", control_dt=self.dt,
                                       wheel_geoms={"left_wheel": "left_tire", "right_wheel": "right_tire"}), servo)
        # body pitch inertia about the COM in the base frame: body_inertia is expressed in the principal
        # frame (body_iquat), which for this torso is rotated 90 deg about x (review B1)
        import mujoco
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, self.sim.model.body_iquat[self.sim.base_bid])
        R = R.reshape(3, 3)
        I_body = R @ np.diag(self.sim.model.body_inertia[self.sim.base_bid]) @ R.T
        Im = float(I_body[1, 1])
        self.body_inertia_base = I_body
        self.plant = Plant6(**diablo_lite_params(model_inertia_yy=Im))
        self.leg = FiveBarLeg(LegGeometry(p.L1, p.L2, p.theta1_nom, p.theta0_nom))
        self.theta_eq_table = self._equilibrium_table()
        self.law = law or LQI6Law(self.plant, Ts=self.dt, theta_eq=self.theta_eq, **(c.lqi_kwargs or {}))
        self.link = HostLink(HostLinkParams(timeout_ms=c.host_timeout_ms))
        self.host = HostEmulator(self.link, publish_hz=c.host_hz)
        mcu_params = McuParams(dt=self.dt, wheel_radius=p.wheel_radius, track=2 * p.wheel_y, mass_total=p.total_mass,
                               mass_wheels=2 * p.m_wheel, L_stand=self.plant.L0, tau_hip_max=p.tau_hip, tau_wheel_max=p.tau_wheel)
        self.mcu = DiabloMcu(self.law, mcu_params, self.leg, self.link)
        self.noise = SensorNoise(enabled=c.noise, seed=c.seed)
        self.delay = ObservationDelay(int(round(c.latency_ms * 1e-3 * c.mcu_hz)))
        self.sim.reset()
        self.up_cmd = 0.0
        self.reset()

    # ------------------------------------------------------- equilibrium posture
    def _set_leg_pose(self, L0: float, theta_v: float) -> Dict[str, float]:
        qh, qc = self.leg.inverse(L0, theta_v)
        return {"left_hip": qh, "left_crank": qc, "right_hip": qh, "right_crank": qc,
                "left_knee": qc - qh, "left_rod": qh - qc, "right_knee": qc - qh, "right_rod": qh - qc}

    def _com_offset(self, L0: float, theta_v: float) -> float:
        # x of the total COM relative to the wheel axle, base level, legs at (L0, theta_v)
        import mujoco
        m, d = self.sim.model, self.sim.data
        mujoco.mj_resetDataKeyframe(m, d, 0)
        for j, q in self._set_leg_pose(L0, theta_v).items():
            d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]] = q
        mujoco.mj_forward(m, d)
        return float(d.subtree_com[self.sim.base_bid][0] - self.sim.site_position("left_axle")[0])

    def _equilibrium_table(self):
        grid = np.linspace(0.12, 0.26, 15)
        thetas = []
        for L0 in grid:
            lo, hi = -0.4, 0.4
            for _ in range(40):
                mid = 0.5 * (lo + hi)
                if self._com_offset(L0, mid) > 0:   # COM ahead of the axle -> move the hip back
                    hi = mid
                else:
                    lo = mid
            thetas.append(0.5 * (lo + hi))
        return grid, np.array(thetas)

    def theta_eq(self, L0: float) -> float:
        g, t = self.theta_eq_table
        return float(np.interp(L0, g, t))

    # ------------------------------------------------------------------ API
    def reset(self, pitch: float = 0.0, yaw: float = 0.0, joint_positions: Optional[Dict[str, float]] = None,
              base_linvel=None, mode: Optional[RobotMode] = None) -> None:
        if joint_positions is None:
            L0 = float(np.clip(self.mcu.p.L_stand + self.mcu.p.height_gain * self.up_cmd, 0.12, 0.26))
            joint_positions = self._set_leg_pose(L0, self.theta_eq(L0))
        p_axle, _, _ = self.leg.forward(joint_positions["left_hip"], joint_positions["left_crank"])
        wheel_offset = np.array([p_axle[0], 0.0, p_axle[1]])
        pos, quat = placed_base_pose(wheel_offset, self.params.wheel_radius, pitch, yaw)
        self.sim.reset(base_pos=pos, base_quat=quat, joint_positions=joint_positions, base_linvel=base_linvel)
        self.mcu.reset(mode or self.cfg.start_mode)
        self.host = HostEmulator(self.link, publish_hz=self.cfg.host_hz)
        self.host.set_command(forward=0.0, left=0.0, up=self.up_cmd, stand_mode=True)
        self.delay = ObservationDelay(self.delay.delay_steps)
        self.log = Logger()
        self.k = 0
        self.observation = self._observe()

    def _observe(self) -> dict:
        raw = self.sim.observe()
        obs = self.delay.push(self.noise.apply(raw, self.dt))
        obs["sim"] = raw["sim"]
        return obs

    def set_height_offset(self, dh: float) -> None:
        self.up_cmd = dh / self.mcu.p.height_gain
        self.host.set_command(up=self.up_cmd)

    def step(self, target_ground_velocity: float = 0.0, target_yaw_velocity: float = 0.0) -> dict:
        obs = self.observation
        self.host.set_command(forward=target_ground_velocity, left=target_yaw_velocity)
        self.host.tick(self.sim.time)
        cmds = self.mcu.tick(obs)
        self.sim.set_commands(cmds)
        self.sim.step()
        self.k += 1
        self.observation = self._observe()
        o = self.observation
        e, tm = self.mcu.est, self.mcu.telemetry
        base = o["sim"]["base"]
        self.log.log(t=self.sim.time, pitch=e.pitch, pitch_rate=e.pitch_rate, roll=e.roll, theta_v=e.theta_v_mean,
                     L0=e.L0_mean, L_ref=tm.get("L_ref", 0.0), ground_vel=e.v, ground_pos=e.position,
                     x=base["position"][0], y=base["position"][1], z=base["position"][2], vx=base["linear_velocity"][0],
                     yaw_rate=base["angular_velocity"][2], T=tm.get("T", 0.0), Tp=tm.get("Tp", 0.0),
                     tau_lw=o["servo"]["left_wheel"]["torque"], tau_rw=o["servo"]["right_wheel"]["torque"],
                     tau_lh=o["servo"]["left_hip"]["torque"], tau_lc=o["servo"]["left_crank"]["torque"],
                     mode=tm.get("mode", 0), in_control=float(tm.get("in_control", False)),
                     v_cmd=target_ground_velocity, w_cmd=target_yaw_velocity)
        o["base_orientation"] = {"pitch": e.pitch, "angular_velocity": np.array([e.roll_rate, e.pitch_rate, e.yaw_rate])}
        return o

    @property
    def fallen(self) -> bool:
        return abs(self.mcu.est.pitch) > 1.0 or self.observation["sim"]["base"]["position"][2] < 0.08

    def push(self, force_x: float, duration: float) -> None:
        self.sim.push([force_x, 0.0, 0.0], duration)

    def run(self, seconds: float, v=0.0, w=0.0) -> bool:
        for _ in range(int(round(seconds / self.dt))):
            self.step(v, w)
            if self.fallen:
                return False
        return True
