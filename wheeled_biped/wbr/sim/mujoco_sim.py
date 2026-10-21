"""MuJoCo simulation backend shared by the three plans.

The sim exposes a servo-level interface: every control step the caller provides one
ServoCommand per joint; the moteus-like PD of ServoModel is evaluated at every physics
substep (as the real servo loop runs much faster than the control loop), the resulting
torque is applied through a <motor> actuator named like the joint, and an observation
dictionary is returned (servo positions/velocities/torques, IMU, ground-truth base state,
wheel contacts).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import mujoco
import numpy as np

from wbr.common.servo import ServoCommand, ServoModel


@dataclass
class SimConfig:
    xml_path: Path
    joint_names: List[str]
    wheel_joints: List[str]
    base_body: str
    imu_site: str = "imu"
    wheel_geoms: Dict[str, str] = field(default_factory=dict)   # wheel joint -> tire geom name
    control_dt: float = 1e-3
    timestep: Optional[float] = None
    stand_keyframe: str = "stand"


class MujocoSim:
    def __init__(self, cfg: SimConfig, servo_model: ServoModel):
        self.cfg = cfg
        self.model = mujoco.MjModel.from_xml_path(str(cfg.xml_path))
        if cfg.timestep is not None:
            self.model.opt.timestep = cfg.timestep
        self.data = mujoco.MjData(self.model)
        self.servo_model = servo_model
        m = self.model
        self.joint_names = list(cfg.joint_names)
        self.jnt_id = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in self.joint_names}
        for n, j in self.jnt_id.items():
            if j < 0:
                raise ValueError(f"joint {n} not found in {cfg.xml_path}")
        self.qpos_adr = {n: int(m.jnt_qposadr[j]) for n, j in self.jnt_id.items()}
        self.dof_adr = {n: int(m.jnt_dofadr[j]) for n, j in self.jnt_id.items()}
        self.act_id = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in self.joint_names}
        for n, a in self.act_id.items():
            if a < 0:
                raise ValueError(f"actuator {n} not found (actuators must be named like their joint)")
        self.base_bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, cfg.base_body)
        self.imu_sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, cfg.imu_site)
        self.floor_gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self.wheel_gid = {w: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, g) for w, g in cfg.wheel_geoms.items()}
        # free joint of the base
        self.free_qadr = None
        for j in range(m.njnt):
            if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE and m.jnt_bodyid[j] == self.base_bid:
                self.free_qadr = int(m.jnt_qposadr[j]); self.free_dadr = int(m.jnt_dofadr[j])
        if self.free_qadr is None:
            raise ValueError("base body has no free joint")
        self.n_substeps = max(1, int(round(cfg.control_dt / m.opt.timestep)))
        self.control_dt = self.n_substeps * m.opt.timestep
        self.sensor_ids = {s: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, s) for s in ("imu_quat", "imu_gyro", "imu_accel")}
        self.key_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, cfg.stand_keyframe)
        self._push_until = -1.0
        self._push_force = np.zeros(3)
        self._push_body = self.base_bid
        self.last_torques = {n: 0.0 for n in self.joint_names}
        self.commands: Dict[str, ServoCommand] = {n: ServoCommand() for n in self.joint_names}
        self.reset()

    # ------------------------------------------------------------------ state
    def reset(self, base_pos=None, base_quat=None, joint_positions: Optional[Dict[str, float]] = None,
              base_linvel=None, base_angvel=None):
        m, d = self.model, self.data
        mujoco.mj_resetData(m, d)
        if self.key_id >= 0:
            mujoco.mj_resetDataKeyframe(m, d, self.key_id)
        if base_pos is not None:
            d.qpos[self.free_qadr:self.free_qadr + 3] = base_pos
        if base_quat is not None:
            d.qpos[self.free_qadr + 3:self.free_qadr + 7] = base_quat
        if joint_positions:
            for n, q in joint_positions.items():
                if n in self.qpos_adr:
                    d.qpos[self.qpos_adr[n]] = q
                else:   # passive joints (e.g. five-bar knee/rod) are addressed by name
                    j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
                    if j < 0:
                        raise KeyError(n)
                    d.qpos[m.jnt_qposadr[j]] = q
        d.qvel[:] = 0.0
        if base_linvel is not None:
            d.qvel[self.free_dadr:self.free_dadr + 3] = base_linvel
        if base_angvel is not None:
            d.qvel[self.free_dadr + 3:self.free_dadr + 6] = base_angvel
        d.ctrl[:] = 0.0
        d.xfrc_applied[:] = 0.0
        self._push_until = -1.0
        self.commands = {n: ServoCommand() for n in self.joint_names}
        mujoco.mj_forward(m, d)

    @property
    def time(self) -> float:
        return float(self.data.time)

    def joint_position(self, name: str) -> float:
        return float(self.data.qpos[self.qpos_adr[name]])

    def joint_velocity(self, name: str) -> float:
        return float(self.data.qvel[self.dof_adr[name]])

    # --------------------------------------------------------------- stepping
    def set_commands(self, commands: Dict[str, ServoCommand]) -> None:
        self.commands.update(commands)

    def push(self, force_xyz, duration: float, body: Optional[str] = None) -> None:
        """Apply a constant world-frame force on the base (or a named body) for `duration` seconds."""
        self._push_force = np.asarray(force_xyz, dtype=float)
        self._push_until = self.data.time + duration
        self._push_body = self.base_bid if body is None else mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body)

    def step(self) -> None:
        """Advance one control period, evaluating the servo PD at every physics substep."""
        m, d = self.model, self.data
        for _ in range(self.n_substeps):
            for n in self.joint_names:
                q = d.qpos[self.qpos_adr[n]]; qd = d.qvel[self.dof_adr[n]]
                tau = self.servo_model.torque(n, self.commands[n], q, qd)
                d.ctrl[self.act_id[n]] = tau
                self.last_torques[n] = tau
            if d.time < self._push_until:
                d.xfrc_applied[self._push_body, :3] = self._push_force
            else:
                d.xfrc_applied[self._push_body, :3] = 0.0
            mujoco.mj_step(m, d)

    # ------------------------------------------------------------ observation
    def wheel_contacts(self) -> Dict[str, bool]:
        d = self.data
        out = {w: False for w in self.wheel_gid}
        for i in range(d.ncon):
            c = d.contact[i]
            for w, g in self.wheel_gid.items():
                if (c.geom1 == g and c.geom2 == self.floor_gid) or (c.geom2 == g and c.geom1 == self.floor_gid):
                    out[w] = True
        return out

    def observe(self) -> dict:
        m, d = self.model, self.data
        servo = {}
        for n in self.joint_names:
            servo[n] = {
                "position": float(d.qpos[self.qpos_adr[n]]),
                "velocity": float(d.qvel[self.dof_adr[n]]),
                "torque": float(d.actuator_force[self.act_id[n]]),
            }
        quat = d.sensordata[m.sensor_adr[self.sensor_ids["imu_quat"]]:m.sensor_adr[self.sensor_ids["imu_quat"]] + 4].copy()
        gyro = d.sensordata[m.sensor_adr[self.sensor_ids["imu_gyro"]]:m.sensor_adr[self.sensor_ids["imu_gyro"]] + 3].copy()
        accel = d.sensordata[m.sensor_adr[self.sensor_ids["imu_accel"]]:m.sensor_adr[self.sensor_ids["imu_accel"]] + 3].copy()
        vel6 = np.zeros(6)
        mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, self.base_bid, vel6, 0)
        base = {
            "position": d.xpos[self.base_bid].copy(),
            "orientation": d.xquat[self.base_bid].copy(),
            "linear_velocity": vel6[3:].copy(),
            "angular_velocity": vel6[:3].copy(),
        }
        return {
            "time": float(d.time),
            "servo": servo,
            "imu": {"orientation": quat, "angular_velocity": gyro, "linear_acceleration": accel},
            "sim": {"base": base, "wheel_contact": self.wheel_contacts()},
        }

    # --------------------------------------------------------------- geometry
    def wheel_forward_signs(self) -> Dict[str, float]:
        """+1 if a positive wheel joint velocity rolls the robot forward (+x of the base), else -1."""
        m, d = self.model, self.data
        R_base = d.xmat[self.base_bid].reshape(3, 3)
        out = {}
        for w in self.cfg.wheel_joints:
            j = self.jnt_id[w]
            axis_world = d.xmat[m.jnt_bodyid[j]].reshape(3, 3) @ m.jnt_axis[j]
            left_axis = R_base[:, 1]
            out[w] = 1.0 if float(axis_world @ left_axis) > 0 else -1.0
        return out

    def body_position(self, name: str) -> np.ndarray:
        return self.data.xpos[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)].copy()

    def site_position(self, name: str) -> np.ndarray:
        return self.data.site_xpos[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, name)].copy()

    def total_mass(self) -> float:
        return float(self.model.body_subtreemass[self.base_bid])

    def com(self) -> np.ndarray:
        return self.data.subtree_com[self.base_bid].copy()
