"""Plan 1 -- Upkie on its own description, upkie control stack re-architected for ROS 2.

Runtime structure (mirrors upkie):
  spine loop @1 kHz : servo observation -> observers (BaseOrientation, FloorContact, WheelOdometry)
                      -> controller pipeline (WheelBalancer PI, runs at the spine rate like upkie's
                      wheel_balancer pipeline) -> servo commands -> MuJoCo servo PD
  agent loop  @200 Hz: optional MPC balancer producing the ground velocity (upkie's mpc_balancer)
Inputs are (target ground velocity [m/s], target yaw velocity [rad/s]).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Optional

import numpy as np

from wbr import MODELS_DIR
from wbr.common.servo import ServoModel, ServoProperties
from wbr.observers.upkie_observers import BaseOrientation, FloorContact, FloorContactParams, WheelOdometry
from wbr.plans.common import Logger, ObservationDelay, SensorNoise, placed_base_pose
from wbr.sim.mujoco_sim import MujocoSim, SimConfig

JOINTS = ["left_hip", "left_knee", "left_wheel", "right_hip", "right_knee", "right_wheel"]
WHEELS = ["left_wheel", "right_wheel"]
WHEEL_RADIUS = 0.05
WHEEL_BASE = 2 * 0.1524


@dataclass
class Plan1Config:
    balancer: str = "pid"          # "pid" (WheelBalancer PI) or "mpc"
    spine_hz: float = 1000.0
    agent_hz: float = 200.0        # MPC rate
    latency_ms: float = 0.0        # observation delay
    noise: bool = False
    seed: int = 0
    servo_kp: float = 20.0         # Bullet spine torque_control_kp
    servo_kd: float = 1.0
    gain_scale: float = 1.0        # WheelBalancer PI gain multiplier
    mpc_leg_length: float = 0.58
    use_sim_contact: bool = False  # bypass the apparent-inertia contact observer with ground truth
    scene: str = "scene.xml"
    yaw_sign: float = -1.0         # upkie's yaw_velocity is clockwise-positive; -1 makes +w = CCW (ROS REP-103)
    drop_height: float = 0.01      # spawn height above touchdown (upkie's Bullet spine drops the robot at reset)
    pitch_offset: str = "model"    # "model": subtract the model's equilibrium pitch (IMU zero at the balance point,
                                   # recomputed when the legs crouch); "none": raw pitch like upkie's code


class Plan1Upkie:
    def __init__(self, cfg: Plan1Config | None = None):
        from wbr.controllers.upkie.gyropod import GyropodMapper
        from wbr.controllers.upkie.wheel_balancer import WheelBalancer, WheelBalancerParams

        self.cfg = cfg or Plan1Config()
        c = self.cfg
        self.dt = 1.0 / c.spine_hz
        servo = ServoModel({
            "left_hip": ServoProperties(c.servo_kp, c.servo_kd, 16.0), "left_knee": ServoProperties(c.servo_kp, c.servo_kd, 16.0),
            "right_hip": ServoProperties(c.servo_kp, c.servo_kd, 16.0), "right_knee": ServoProperties(c.servo_kp, c.servo_kd, 16.0),
            "left_wheel": ServoProperties(c.servo_kp, c.servo_kd, 1.7), "right_wheel": ServoProperties(c.servo_kp, c.servo_kd, 1.7),
        })
        self.sim = MujocoSim(SimConfig(xml_path=MODELS_DIR / "upkie" / c.scene, joint_names=JOINTS, wheel_joints=WHEELS,
                                       base_body="base", imu_site="base", control_dt=self.dt,
                                       wheel_geoms={"left_wheel": "left_tire", "right_wheel": "right_tire"}), servo)
        # the MuJoCo IMU sensor reports the base orientation in the world frame: identity mounting /
        # ARS rotations (the real pi3hat path uses BaseOrientation's default upkie rotations)
        self.base_orientation = BaseOrientation(rotation_base_to_imu=np.eye(3), rotation_ars_to_world=np.eye(3))
        self.floor_contact = FloorContact(FloorContactParams(dt=self.dt))
        signs = self.sim.wheel_forward_signs()
        self.wheel_odometry = WheelOdometry({w: signs[w] * WHEEL_RADIUS for w in WHEELS}, self.dt)
        self.balancer = WheelBalancer(WheelBalancerParams(dt=self.dt, wheel_radius=WHEEL_RADIUS, contact_radius=WHEEL_BASE / 2,
                                                          gain_scale=c.gain_scale))
        self.mapper = GyropodMapper(WHEEL_RADIUS, WHEEL_BASE, self.dt, left_wheeled=(signs["left_wheel"] > 0))
        self.mpc = None
        if c.balancer == "mpc":
            from wbr.controllers.upkie.mpc_balancer import MPCBalancer
            self.mpc = MPCBalancer(leg_length=c.mpc_leg_length)
            # the MPC does not drive without a contact flag and the apparent-inertia detector needs wheel
            # torque to trigger: ground-truth contact in simulation (see report)
            c.use_sim_contact = True
        self.agent_period = max(1, int(round(c.spine_hz / c.agent_hz)))
        self.noise = SensorNoise(enabled=c.noise, seed=c.seed)
        self.delay = ObservationDelay(int(round(c.latency_ms * 1e-3 * c.spine_hz)), dt=1.0 / c.spine_hz)
        # wheel axle offset in the base frame (standing keyframe)
        self.sim.reset()
        base_pos = self.sim.body_position("base")
        self.wheel_offset = self.sim.body_position("left_wheel_tire") - base_pos
        self.wheel_offset[1] = 0.0
        self.pitch_eq = self._equilibrium_pitch() if c.pitch_offset == "model" else 0.0   # IMU zero (see Plan1Config)
        self.pitch_eq_target = self.pitch_eq
        self.log = Logger()
        self.k = 0
        self.observation: dict = {}
        self.reset()

    # ------------------------------------------------------------------ API
    def reset(self, pitch: float = 0.0, yaw: float = 0.0, joint_positions: Optional[Dict[str, float]] = None,
              base_linvel=None) -> None:
        pos, quat = placed_base_pose(self.wheel_offset, WHEEL_RADIUS, pitch, yaw)
        pos[2] += self.cfg.drop_height
        self.sim.reset(base_pos=pos, base_quat=quat, joint_positions=joint_positions, base_linvel=base_linvel)
        self.balancer.reset()
        self.wheel_odometry.reset()
        self.floor_contact = FloorContact(FloorContactParams(dt=self.dt))
        self.mapper.reset({j: self.sim.joint_position(j) for j in self.mapper.upper_leg_joints})
        if self.mpc is not None:
            self.mpc.reset()
        self.delay = ObservationDelay(self.delay.delay_steps, dt=self.delay.dt)
        self.log = Logger()
        self.k = 0
        self.ground_velocity_cmd = 0.0
        self.observation = self._observe()

    def _observe(self) -> dict:
        raw = self.sim.observe()
        truth = raw["sim"]
        obs = self.delay.push(self.noise.apply(raw, self.dt))
        servo = obs["servo"]
        bo = self.base_orientation.update(obs["imu"]["orientation"], obs["imu"]["angular_velocity"])
        bo = dict(bo)
        bo["pitch_raw"] = bo["pitch"]
        if hasattr(self, "pitch_eq_target"):   # follow the crouch at the leg filter's pace (1 s cutoff, review N3)
            from wbr.common.filters import low_pass_filter
            self.pitch_eq = low_pass_filter(self.pitch_eq, self.mapper.leg_cutoff_period, self.pitch_eq_target, self.dt)
        bo["pitch"] = bo["pitch"] - getattr(self, "pitch_eq", 0.0)
        fc = self.floor_contact.update(servo)
        if self.cfg.use_sim_contact:
            for w in WHEELS:
                fc[w]["contact"] = truth["wheel_contact"][w]
            fc["contact"] = any(truth["wheel_contact"].values())
        wo = self.wheel_odometry.update(fc, servo)
        return {"time": obs["time"], "servo": servo, "imu": obs["imu"], "base_orientation": bo, "floor_contact": fc,
                "wheel_odometry": wo, "sim": truth}

    def control_step(self, obs: dict, target_ground_velocity: float, target_yaw_velocity: float) -> tuple:
        """One control cycle (balancer + servo mapping) on an observation: (servo commands, info).

        Shared by step() and the ROS 2 spine/balancer nodes so that the yaw convention (cfg.yaw_sign:
        +w = counter-clockwise, REP-103) and the leg gain scales are applied in exactly one place.
        """
        w = self.cfg.yaw_sign * target_yaw_velocity
        if self.mpc is not None:
            if self.k % self.agent_period == 0:
                self.ground_velocity_cmd = self.mpc.step(target_ground_velocity, obs, self.agent_period * self.dt)
            ground_velocity = self.ground_velocity_cmd
            scale = 2.0 + (2.0 if abs(w) > 0.1 else 0.0)   # upkie follow_joystick: gain_scale + turning_gain_scale
            cmds = self.mapper.servo_commands(ground_velocity, w, scale, scale)
        else:
            out = self.balancer.step(obs, target_ground_velocity, w)
            ground_velocity = out["ground_velocity"]
            cmds = self.mapper.servo_commands(ground_velocity, w, out["leg_kp_scale"], out["leg_kd_scale"])
        return cmds, {"ground_velocity": ground_velocity}

    def step(self, target_ground_velocity: float = 0.0, target_yaw_velocity: float = 0.0) -> dict:
        obs = self.observation
        cmds, info = self.control_step(obs, target_ground_velocity, target_yaw_velocity)
        ground_velocity = info["ground_velocity"]
        self.sim.set_commands(cmds)
        self.sim.step()
        self.k += 1
        self.observation = self._observe()
        o = self.observation
        base = o["sim"]["base"]
        self.log.log(t=self.sim.time, pitch=o["base_orientation"]["pitch"], pitch_rate=o["base_orientation"]["angular_velocity"][1],
                     ground_pos=o["wheel_odometry"]["position"], ground_vel=o["wheel_odometry"]["velocity"],
                     x=base["position"][0], y=base["position"][1], z=base["position"][2],
                     vx=base["linear_velocity"][0], yaw_rate=base["angular_velocity"][2],
                     contact=float(o["floor_contact"]["contact"]), ground_velocity_cmd=ground_velocity,
                     tau_lw=o["servo"]["left_wheel"]["torque"], tau_rw=o["servo"]["right_wheel"]["torque"],
                     tau_lh=o["servo"]["left_hip"]["torque"], tau_lk=o["servo"]["left_knee"]["torque"],
                     v_cmd=target_ground_velocity, w_cmd=target_yaw_velocity)
        return o

    def _equilibrium_pitch(self, joint_positions: Optional[Dict[str, float]] = None) -> float:
        """Base pitch (about the wheel axle) that puts the total COM above the contact point."""
        import mujoco
        m = self.sim.model
        d = mujoco.MjData(m)          # scratch data: never touch the running simulation state

        def com_offset(pitch):
            mujoco.mj_resetDataKeyframe(m, d, 0)
            for j, q in (joint_positions or {}).items():
                d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]] = q
            mujoco.mj_forward(m, d)
            axle_bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "left_wheel_tire")
            wheel_offset = d.xpos[axle_bid] - d.xpos[self.sim.base_bid]
            wheel_offset[1] = 0.0
            pos, quat = placed_base_pose(wheel_offset, WHEEL_RADIUS, pitch, 0.0)
            d.qpos[self.sim.free_qadr:self.sim.free_qadr + 3] = pos
            d.qpos[self.sim.free_qadr + 3:self.sim.free_qadr + 7] = quat
            mujoco.mj_forward(m, d)
            return float(d.subtree_com[self.sim.base_bid][0] - d.xpos[axle_bid][0])

        lo, hi = -0.2, 0.2
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            if com_offset(mid) > 0:
                hi = mid
            else:
                lo = mid
        return 0.5 * (lo + hi)

    # ------------------------------------------------------------ leg height
    def _leg_length(self, hip: float, knee: float) -> tuple:
        """Vertical hip->axle distance and axle x offset (base frame) for symmetric leg angles."""
        import mujoco
        m = self.sim.model
        d = mujoco.MjData(m)
        mujoco.mj_resetDataKeyframe(m, d, 0)
        for j, q in (("left_hip", hip), ("left_knee", knee), ("right_hip", -hip), ("right_knee", -knee)):
            d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]] = q
        mujoco.mj_forward(m, d)
        base = d.xpos[self.sim.base_bid]
        axle = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "left_wheel_tire")]
        return float(base[2] - axle[2]), float(axle[0] - base[0])

    def set_height_offset(self, dh: float) -> None:
        """Crouch the legs (dh <= 0, metres) while balancing, through hip/knee position targets."""
        dh = min(0.0, dh)
        if getattr(self, "_last_dh", None) == dh:   # cached: the IK search is expensive
            return
        self._last_dh = dh
        if not hasattr(self, "_leg_len0"):
            self._leg_len0 = self._leg_length(0.0, 0.0)[0]
            # knee sign that keeps the axle under the hip (symmetric crouch)
            self._knee_sign = min((+1.0, -1.0), key=lambda s: abs(self._leg_length(0.3, s * 0.6)[1]))
        target = self._leg_len0 + dh
        lo, hi = 0.0, 1.2
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            if self._leg_length(mid, self._knee_sign * 2 * mid)[0] > target:
                lo = mid
            else:
                hi = mid
        a = 0.5 * (lo + hi)
        neutral = {"left_hip": a, "left_knee": self._knee_sign * 2 * a,
                   "right_hip": -a, "right_knee": -self._knee_sign * 2 * a}
        self.mapper.set_neutral(neutral)
        if self.cfg.pitch_offset == "model":   # the IMU zero moves with the leg masses: tracked as slowly as the legs
            self.pitch_eq_target = self._equilibrium_pitch(neutral)

    @property
    def fallen(self) -> bool:
        return abs(self.observation["base_orientation"]["pitch"]) > 1.0 or self.observation["sim"]["base"]["position"][2] < 0.2

    def push(self, force_x: float, duration: float) -> None:
        self.sim.push([force_x, 0.0, 0.0], duration, body="torso")

    def run(self, seconds: float, v=0.0, w=0.0) -> bool:
        n = int(round(seconds / self.dt))
        for _ in range(n):
            self.step(v, w)
            if self.fallen:
                return False
        return True
