"""DIABLO-lite: a 6 kg wheeled biped with DIABLO's leg architecture (Plans 2 and 3).

Leg mechanism (from DDTRobot/DiabloSTLSDF model.sdf, left leg, hip frame x forward / z up):
  two coaxial hip motors per leg.  Motor1 drives the thigh (L1 = 0.14 m, hip -> knee).  Motor0
  drives a crank (Lc = 0.09 m); a rod (Lr = 0.14 m) links the crank tip to the shank 0.09 m below
  the knee, forming a parallelogram (crank // shank segment, rod // thigh).  Hence the shank
  (L2 = 0.14 m, knee -> wheel axle) always has the absolute orientation of the crank: each motor
  sets the absolute angle of one link.  Motor2 is the wheel.
Nominal pose: thigh -45 deg (down-forward), crank/shank +45 deg (down-backward), wheel axle
0.198 m below the hip; joint coordinates are zero at this pose, positive about the +y axis.

Mass budget (6.0 kg total): torso box 2.0 + hip motors 4 x 0.45 = 3.8 kg; per leg thigh 0.25,
shank 0.25, crank 0.05, rod 0.05; wheel + motor 0.5 each (R = 0.065 m, like the design report's
5 kg-class five-bar robot).  Torque limits: hip motors 12 N m, wheels 5 N m (RMD-L-9015 class).

Outputs models/diablo_lite/diablo_lite.xml (+ scene.xml) and diablo_lite.urdf (tree form for ROS 2:
the knee and rod joints are passive and published from the crank/hip angles).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, asdict
from pathlib import Path

import numpy as np

from wbr import MODELS_DIR

OUT_DIR = MODELS_DIR / "diablo_lite"
JOINTS = ["left_hip", "left_crank", "left_wheel", "right_hip", "right_crank", "right_wheel"]
PASSIVE = ["left_knee", "left_rod", "right_knee", "right_rod"]


@dataclass
class DiabloLiteParams:
    L1: float = 0.14          # thigh
    L2: float = 0.14          # shank (knee -> wheel axle)
    Lc: float = 0.09          # crank
    Lr: float = 0.14          # rod
    rod_attach: float = 0.09  # distance knee -> rod attachment along the shank (= L2 - 0.05)
    theta1_nom: float = -math.pi / 4   # thigh absolute angle at q = 0 (about +y, 0 = straight down)
    theta0_nom: float = +math.pi / 4   # crank / shank absolute angle at q = 0
    wheel_radius: float = 0.065
    wheel_width: float = 0.03
    hip_y: float = 0.14       # leg plane lateral offset
    wheel_y: float = 0.19     # wheel center lateral offset (track = 0.38 m)
    m_torso: float = 2.0
    m_motor: float = 0.45     # per hip motor (4)
    m_thigh: float = 0.25
    m_shank: float = 0.25
    m_crank: float = 0.05
    m_rod: float = 0.05
    m_wheel: float = 0.5
    torso_size: tuple = (0.10, 0.11, 0.09)   # half sizes
    torso_z: float = 0.10                    # torso box center above the hip axis
    imu_z: float = 0.05
    tau_hip: float = 12.0
    tau_wheel: float = 5.0
    hip_range: tuple = (-1.2, 1.0)
    crank_range: tuple = (-1.0, 1.2)
    armature_hip: float = 0.002
    armature_wheel: float = 0.0004
    friction_hip: float = 0.05
    friction_wheel: float = 0.02

    @property
    def total_mass(self) -> float:
        return self.m_torso + 4 * self.m_motor + 2 * (self.m_thigh + self.m_shank + self.m_crank + self.m_rod + self.m_wheel)


def quat_y(angle: float) -> str:
    return f"{math.cos(angle / 2):.9f} 0 {math.sin(angle / 2):.9f} 0"


def leg_xml(p: DiabloLiteParams, side: str) -> str:
    sy = 1.0 if side == "left" else -1.0
    hip_y = sy * p.hip_y
    wheel_dy = sy * (p.wheel_y - p.hip_y)
    rel_knee = p.theta0_nom - p.theta1_nom     # shank relative to thigh at q = 0
    rel_rod = p.theta1_nom - p.theta0_nom      # rod relative to crank at q = 0
    n = side
    return f"""
      <body name="{n}_thigh" pos="0 {hip_y:.4f} 0" quat="{quat_y(p.theta1_nom)}">
        <joint name="{n}_hip" axis="0 1 0" range="{p.hip_range[0]} {p.hip_range[1]}" armature="{p.armature_hip}" frictionloss="{p.friction_hip}" damping="0.01"/>
        <geom name="{n}_thigh_geom" type="capsule" fromto="0 0 0 0 0 {-p.L1:.4f}" size="0.012" mass="{p.m_thigh}" class="leg"/>
        <body name="{n}_shank" pos="0 0 {-p.L1:.4f}" quat="{quat_y(rel_knee)}">
          <joint name="{n}_knee" axis="0 1 0" armature="0.0001" damping="0.005"/>
          <geom name="{n}_shank_geom" type="capsule" fromto="0 0 0 0 0 {-p.L2:.4f}" size="0.012" mass="{p.m_shank}" class="leg"/>
          <site name="{n}_rod_attach" pos="0 {-sy * 0.02:.4f} {-p.rod_attach:.4f}" size="0.004"/>
          <body name="{n}_wheel" pos="0 {wheel_dy:.4f} {-p.L2:.4f}">
            <joint name="{n}_wheel" axis="0 1 0" armature="{p.armature_wheel}" frictionloss="{p.friction_wheel}" damping="0.001"/>
            <geom name="{n}_tire" type="cylinder" zaxis="0 1 0" size="{p.wheel_radius} {p.wheel_width / 2}" mass="{p.m_wheel}" class="wheel"/>
            <site name="{n}_axle" pos="0 0 0" size="0.004"/>
          </body>
        </body>
      </body>
      <body name="{n}_crank" pos="0 {hip_y - sy * 0.02:.4f} 0" quat="{quat_y(p.theta0_nom)}">
        <joint name="{n}_crank" axis="0 1 0" range="{p.crank_range[0]} {p.crank_range[1]}" armature="{p.armature_hip}" frictionloss="{p.friction_hip}" damping="0.01"/>
        <geom name="{n}_crank_geom" type="capsule" fromto="0 0 0 0 0 {-p.Lc:.4f}" size="0.009" mass="{p.m_crank}" class="leg"/>
        <body name="{n}_rod" pos="0 0 {-p.Lc:.4f}" quat="{quat_y(rel_rod)}">
          <joint name="{n}_rod" axis="0 1 0" armature="0.0001" damping="0.005"/>
          <geom name="{n}_rod_geom" type="capsule" fromto="0 0 0 0 0 {-p.Lr:.4f}" size="0.007" mass="{p.m_rod}" class="leg"/>
          <site name="{n}_rod_tip" pos="0 0 {-p.Lr:.4f}" size="0.004"/>
        </body>
      </body>"""


def mjcf(p: DiabloLiteParams) -> str:
    stand_z = p.L1 * math.cos(p.theta1_nom) + p.L2 * math.cos(p.theta0_nom) + p.wheel_radius
    tz = p.torso_size
    return f"""<mujoco model="diablo_lite">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.0005" integrator="implicitfast" cone="elliptic" impratio="10" noslip_iterations="2"/>
  <default>
    <geom condim="3" friction="1.0 0.005 0.0001"/>
    <default class="leg"><geom contype="0" conaffinity="0" rgba="0.75 0.75 0.78 1"/></default>
    <default class="wheel"><geom contype="1" conaffinity="1" rgba="0.15 0.15 0.15 1" priority="1" friction="1.0 0.005 0.0001"/></default>
    <default class="body"><geom contype="1" conaffinity="1"/></default>
  </default>
  <worldbody>
    <body name="base" pos="0 0 {stand_z:.4f}">
      <freejoint name="base_freejoint"/>
      <site name="base" pos="0 0 0" size="0.005"/>
      <site name="imu" pos="0 0 {p.imu_z}" size="0.005"/>
      <geom name="torso" type="box" pos="0 0 {p.torso_z}" size="{tz[0]} {tz[1]} {tz[2]}" mass="{p.m_torso}" class="body" rgba="0.3 0.45 0.8 1"/>
      <geom name="left_motors" type="cylinder" zaxis="0 1 0" pos="0 {p.hip_y - 0.03:.4f} 0" size="0.045 0.03" mass="{2 * p.m_motor}" class="leg" rgba="0.2 0.2 0.2 1"/>
      <geom name="right_motors" type="cylinder" zaxis="0 1 0" pos="0 {-(p.hip_y - 0.03):.4f} 0" size="0.045 0.03" mass="{2 * p.m_motor}" class="leg" rgba="0.2 0.2 0.2 1"/>
      {leg_xml(p, "left")}
      {leg_xml(p, "right")}
    </body>
  </worldbody>
  <equality>
    <connect name="left_loop" site1="left_rod_tip" site2="left_rod_attach" solref="0.001 1" solimp="0.99 0.999 0.0001"/>
    <connect name="right_loop" site1="right_rod_tip" site2="right_rod_attach" solref="0.001 1" solimp="0.99 0.999 0.0001"/>
  </equality>
  <actuator>
    <motor name="left_hip" joint="left_hip" ctrlrange="-{p.tau_hip} {p.tau_hip}"/>
    <motor name="left_crank" joint="left_crank" ctrlrange="-{p.tau_hip} {p.tau_hip}"/>
    <motor name="left_wheel" joint="left_wheel" ctrlrange="-{p.tau_wheel} {p.tau_wheel}"/>
    <motor name="right_hip" joint="right_hip" ctrlrange="-{p.tau_hip} {p.tau_hip}"/>
    <motor name="right_crank" joint="right_crank" ctrlrange="-{p.tau_hip} {p.tau_hip}"/>
    <motor name="right_wheel" joint="right_wheel" ctrlrange="-{p.tau_wheel} {p.tau_wheel}"/>
  </actuator>
  <sensor>
    <framequat name="imu_quat" objtype="site" objname="imu"/>
    <gyro name="imu_gyro" site="imu"/>
    <accelerometer name="imu_accel" site="imu"/>
  </sensor>
  <keyframe>
    <key name="stand" qpos="0 0 {stand_z:.4f} 1 0 0 0 0 0 0 0 0 0 0 0 0 0"/>
  </keyframe>
</mujoco>
"""


SCENE_XML = """<mujoco model="diablo_lite_scene">
  <include file="diablo_lite.xml"/>
  <statistic center="0 0 0.2" extent="1.0"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="140" elevation="-20" offwidth="1280" offheight="720"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
  </asset>
  <worldbody>
    <light pos="0 0 3.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane" friction="1.0 0.005 0.0001" condim="3"/>
    <camera name="track" mode="trackcom" pos="1.1 -1.1 0.45" xyaxes="0.707 0.707 0 -0.25 0.25 0.94"/>
    <camera name="side" mode="trackcom" pos="0 -1.8 0.3" xyaxes="1 0 0 0 0.2 0.98"/>
  </worldbody>
</mujoco>
"""


def urdf(p: DiabloLiteParams) -> str:
    """Tree URDF (loop closure not representable): base -> thigh -> shank -> wheel, base -> crank -> rod."""
    def rpy_y(a):
        return f"0 {a:.6f} 0"

    def inertial(m, length, r):
        ixx = m * (3 * r * r + length * length) / 12.0
        izz = m * r * r / 2.0
        return (f'<inertial><origin xyz="0 0 {-length / 2:.4f}"/><mass value="{m}"/>'
                f'<inertia ixx="{ixx:.3e}" ixy="0" ixz="0" iyy="{ixx:.3e}" iyz="0" izz="{izz:.3e}"/></inertial>')

    def capsule_visual(length, r):
        return (f'<visual><origin xyz="0 0 {-length / 2:.4f}"/><geometry><cylinder radius="{r}" length="{length}"/></geometry>'
                f'<material name="grey"/></visual>')

    tz = p.torso_size
    m_body = p.m_torso + 4 * p.m_motor
    out = [f'<?xml version="1.0"?>\n<robot name="diablo_lite">\n  <material name="grey"><color rgba="0.75 0.75 0.78 1"/></material>'
           f'\n  <material name="blue"><color rgba="0.3 0.45 0.8 1"/></material>\n  <material name="dark"><color rgba="0.15 0.15 0.15 1"/></material>']
    out.append(f'''  <link name="base_link">
    <inertial><origin xyz="0 0 {p.m_torso * p.torso_z / m_body:.4f}"/><mass value="{m_body}"/>
      <inertia ixx="{m_body * 0.012:.3e}" ixy="0" ixz="0" iyy="{m_body * 0.012:.3e}" iyz="0" izz="{m_body * 0.015:.3e}"/></inertial>
    <visual><origin xyz="0 0 {p.torso_z}"/><geometry><box size="{2 * tz[0]} {2 * tz[1]} {2 * tz[2]}"/></geometry><material name="blue"/></visual>
    <collision><origin xyz="0 0 {p.torso_z}"/><geometry><box size="{2 * tz[0]} {2 * tz[1]} {2 * tz[2]}"/></geometry></collision>
  </link>
  <link name="imu_link"/>
  <joint name="imu_joint" type="fixed"><parent link="base_link"/><child link="imu_link"/><origin xyz="0 0 {p.imu_z}"/></joint>''')
    for side in ("left", "right"):
        sy = 1.0 if side == "left" else -1.0
        n = side
        out.append(f'''  <link name="{n}_thigh">{inertial(p.m_thigh, p.L1, 0.012)}{capsule_visual(p.L1, 0.012)}</link>
  <joint name="{n}_hip" type="revolute"><parent link="base_link"/><child link="{n}_thigh"/>
    <origin xyz="0 {sy * p.hip_y:.4f} 0" rpy="{rpy_y(p.theta1_nom)}"/><axis xyz="0 1 0"/>
    <limit lower="{p.hip_range[0]}" upper="{p.hip_range[1]}" effort="{p.tau_hip}" velocity="30"/></joint>
  <link name="{n}_shank">{inertial(p.m_shank, p.L2, 0.012)}{capsule_visual(p.L2, 0.012)}</link>
  <joint name="{n}_knee" type="revolute"><parent link="{n}_thigh"/><child link="{n}_shank"/>
    <origin xyz="0 0 {-p.L1:.4f}" rpy="{rpy_y(p.theta0_nom - p.theta1_nom)}"/><axis xyz="0 1 0"/>
    <limit lower="-2.5" upper="2.5" effort="0" velocity="30"/></joint>
  <link name="{n}_wheel">
    <inertial><mass value="{p.m_wheel}"/><inertia ixx="{p.m_wheel * (3 * p.wheel_radius**2 + p.wheel_width**2) / 12:.3e}" ixy="0" ixz="0" iyy="{p.m_wheel * p.wheel_radius**2 / 2:.3e}" iyz="0" izz="{p.m_wheel * (3 * p.wheel_radius**2 + p.wheel_width**2) / 12:.3e}"/></inertial>
    <visual><origin rpy="1.5707963 0 0"/><geometry><cylinder radius="{p.wheel_radius}" length="{p.wheel_width}"/></geometry><material name="dark"/></visual>
    <collision><origin rpy="1.5707963 0 0"/><geometry><cylinder radius="{p.wheel_radius}" length="{p.wheel_width}"/></geometry></collision>
  </link>
  <joint name="{n}_wheel" type="continuous"><parent link="{n}_shank"/><child link="{n}_wheel"/>
    <origin xyz="0 {sy * (p.wheel_y - p.hip_y):.4f} {-p.L2:.4f}"/><axis xyz="0 1 0"/><limit effort="{p.tau_wheel}" velocity="60"/></joint>
  <link name="{n}_crank">{inertial(p.m_crank, p.Lc, 0.009)}{capsule_visual(p.Lc, 0.009)}</link>
  <joint name="{n}_crank" type="revolute"><parent link="base_link"/><child link="{n}_crank"/>
    <origin xyz="0 {sy * (p.hip_y - 0.02):.4f} 0" rpy="{rpy_y(p.theta0_nom)}"/><axis xyz="0 1 0"/>
    <limit lower="{p.crank_range[0]}" upper="{p.crank_range[1]}" effort="{p.tau_hip}" velocity="30"/></joint>
  <link name="{n}_rod">{inertial(p.m_rod, p.Lr, 0.007)}{capsule_visual(p.Lr, 0.007)}</link>
  <joint name="{n}_rod" type="revolute"><parent link="{n}_crank"/><child link="{n}_rod"/>
    <origin xyz="0 0 {-p.Lc:.4f}" rpy="{rpy_y(p.theta1_nom - p.theta0_nom)}"/><axis xyz="0 1 0"/>
    <limit lower="-2.5" upper="2.5" effort="0" velocity="30"/></joint>''')
    out.append("</robot>\n")
    return "\n".join(out)


def build(out_dir: Path = OUT_DIR, params: DiabloLiteParams | None = None, verbose: bool = True) -> Path:
    import mujoco
    p = params or DiabloLiteParams()
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "diablo_lite.xml").write_text(mjcf(p), encoding="utf-8")
    (out_dir / "scene.xml").write_text(SCENE_XML, encoding="utf-8")
    (out_dir / "diablo_lite.urdf").write_text(urdf(p), encoding="utf-8")
    import json
    (out_dir / "params.json").write_text(json.dumps(asdict(p), indent=1), encoding="utf-8")
    m = mujoco.MjModel.from_xml_path(str(out_dir / "scene.xml"))
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    if verbose:
        tip = d.site_xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "left_rod_tip")]
        att = d.site_xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "left_rod_attach")]
        print(f"wrote {out_dir / 'diablo_lite.xml'}; total mass {float(m.body_subtreemass[1]):.3f} kg "
              f"(budget {p.total_mass:.3f}); loop closure residual {np.linalg.norm(tip - att):.2e} m")
        print("COM", np.round(d.subtree_com[1], 4), "axle", np.round(d.site_xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, 'left_axle')], 4))
    return out_dir / "scene.xml"


if __name__ == "__main__":
    build()
