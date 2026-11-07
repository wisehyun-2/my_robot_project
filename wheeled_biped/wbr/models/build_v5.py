"""URDF and MuJoCo model of the v5 robot, from the Fusion model only: no upkie file, mesh, mass or joint convention goes in.

Inputs (written by robot_cad_v5/fusion/v5run.py):
    robot_cad_v5/calc/mass_properties.json     save_mass_json(): mass, COM, inertia of each link at the nominal pose
    robot_cad_v5/calc/urdf_data.json           urdf_export(): joint axes, tyre, IMU position and axes
    robot_cad_v5/export/links_stl/*.stl        urdf_export(): one mesh per link
Outputs (models/wheelbiped_v5):
    wheelbiped_v5.urdf + meshes/               the robot description
    wheelbiped_v5.xml, scene.xml               MuJoCo model compiled from that URDF (free joint, motors, IMU sensors, contact)
    plant_params.json                          parameters of the controller's plant, computed on this model
    description.json                           what the plan needs besides: tyre, track, motor limits, joint signs, IMU mount

Frames: all link frames are parallel to the base frame at q = 0, which is the straight-leg pose.  Base frame: origin at the
midpoint of the hip axes (the DIABLO paper's body frame), x forward, y left, z up.  The robot's front is the rocker switch side
(user, 2026-11-07), which is the CAD's -x: the base axes are the CAD axes turned 180 deg about z and the knees point backwards.  The
part names follow the robot (user, 2026-11-08): _L parts are its left = CAD -y, shell_front is on the switch side.  All six joint
axes are +y, so a positive joint angle is the controller's u-convention on both sides (JOINT_SIGNS all +1): upkie's mirrored joints
and its sign table do not exist here.  The real motors get the same signs through moteus' output sign.

    python -m wbr.models.build_v5
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import mujoco
import numpy as np

from wbr import MODELS_DIR
from wbr.bench.ake_sizing import AKE60_8, Motor
from wbr.controllers.lqi6.plant_serial import SerialPlantParams

REPO = Path(__file__).resolve().parents[3]
CAD_DIR = REPO / "robot_cad_v5"
MASS_JSON = CAD_DIR / "calc" / "mass_properties.json"
URDF_JSON = CAD_DIR / "calc" / "urdf_data.json"
STL_DIR = CAD_DIR / "export" / "links_stl"
OUT_DIR = MODELS_DIR / "wheelbiped_v5"
NAME = "wheelbiped_v5"
PARAMS_FILE = "plant_params.json"
DESCRIPTION_FILE = "description.json"

JOINTS = ["left_hip", "left_knee", "left_wheel", "right_hip", "right_knee", "right_wheel"]
JOINT_SIGNS = {j: 1.0 for j in JOINTS}
SIDES = (("left", 1.0, "L"), ("right", -1.0, "R"))     # robot side, sign of its y, suffix of its parts in the CAD
R_ROBOT_FROM_CAD = np.diag([-1.0, -1.0, 1.0])           # front = the switch side = CAD -x, 180 deg about z
KNEE_SIGN = +1.0                                        # SerialLegGeometry: the knees are behind the hip -> axle line
MOTOR: Motor = AKE60_8                  # all six joints
IMU_CHIP = "BMI088"                     # the footprint the firmware tries first
# AKE60-8: armature and backdrive torque from the catalogue (ake_sizing.Motor); viscous damping is not published: assumed
DAMPING = {"hip": 0.02, "knee": 0.02, "wheel": 0.001}       # N m s/rad
TIRE_FRICTION = [1.0, 0.005, 0.0001]    # sliding, torsional, rolling; condim 4 (one contact point of the rounded tyre)
MESH_TRIANGLES = {"base": 30000, "thigh": 8000, "shank": 8000, "wheel": 8000}
# Leg range [m] from the link length (links 250 mm, user 2026-11-07).  The same distances from the straight leg as on the 175 mm robot (310 and
# 335 of 350 mm), so the standing knee moment arm stays small: 98 mm at 460 mm against 139 mm at the 175 mm robot's knee angle.  The lowest pose is set
# by the knee: this moment arm gives 10 N m (80 % of the motor's peak) standing with 5 kg on top.
STAND_FROM_STRAIGHT = 0.040
TOP_FROM_STRAIGHT = 0.015
KNEE_ARM_LOWEST = 0.167
GRID_STEP = 0.005
GRID_BELOW_LOWEST = 0.060       # the plant and gain tables reach below the commanded range: one leg folds further over a bump


def leg_range(L1: float) -> dict:
    """Standing, lowest and highest commanded leg length and the leg-length grid of the plant and gain tables, thigh = shank = L1."""
    top = 2.0 * L1 - 0.005
    low = 2.0 * math.sqrt(L1 * L1 - KNEE_ARM_LOWEST * KNEE_ARM_LOWEST)
    grid = np.round(np.arange(round(low - GRID_BELOW_LOWEST, 3), top + 0.5 * GRID_STEP, GRID_STEP), 4)
    return dict(L_stand=2.0 * L1 - STAND_FROM_STRAIGHT, L_min=low, L_max=2.0 * L1 - TOP_FROM_STRAIGHT, grid=grid)

SCENE_XML = """<mujoco model="wheelbiped_v5_scene">
  <include file="{name}.xml"/>
  <statistic center="0 0 0.3" extent="1.2"/>
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
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane" friction="1.0 0.005 0.0001" condim="3" contype="1" conaffinity="3"/>
    <camera name="track" mode="trackcom" pos="1.2 -1.2 0.55" xyaxes="0.707 0.707 0 -0.25 0.25 0.94"/>
    <camera name="side" mode="trackcom" pos="0 -2.0 0.4" xyaxes="1 0 0 0 0.2 0.98"/>
  </worldbody>
</mujoco>
"""


def ry(t: float) -> np.ndarray:
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rpy_of(R: np.ndarray) -> tuple:
    """URDF rpy (R = Rz(y) Ry(p) Rx(r))."""
    p = math.atan2(-R[2, 0], math.hypot(R[0, 0], R[1, 0]))
    return math.atan2(R[2, 1], R[2, 2]), p, math.atan2(R[1, 0], R[0, 0])


class V5Geometry:
    """Link frames at the nominal pose (robot axes, metres) and everything derived from the two Fusion JSON files."""

    def __init__(self, mass_json: Path = MASS_JSON, urdf_json: Path = URDF_JSON):
        self.cad = json.loads(Path(mass_json).read_text(encoding="utf-8"))
        self.urdf = json.loads(Path(urdf_json).read_text(encoding="utf-8"))
        pose = self.cad["pose"]
        self.L1, self.L2 = pose["L1"] / 1000.0, pose["L2"] / 1000.0
        self.R = pose["R_wheel"] / 1000.0
        self.Z_hip = pose["Z_hip"] / 1000.0
        self.L0 = pose["L0"] / 1000.0
        self.kx_cad, kz = pose["knee"][0] / 1000.0, pose["knee"][1] / 1000.0
        kx = -self.kx_cad
        self.t1 = math.atan2(-kx, self.Z_hip - kz)      # thigh, about +y: u(t) = Ry(t) (0, 0, -1)
        self.t2 = math.atan2(kx, kz - self.R)           # shank and wheel
        jy = self.urdf["joint_y"]
        self.y_hip, self.y_knee, self.y_wheel = jy["hip"] / 1000.0, jy["knee"] / 1000.0, jy["wheel"] / 1000.0
        self.tyre_width = self.urdf["tyre"]["width"] / 1000.0
        self.tyre_y = self.urdf["tyre"]["centre_y"] / 1000.0
        self.frames = {"base": (np.array([0.0, 0.0, self.Z_hip]), 0.0, "torso")}
        for side, s, k in SIDES:
            self.frames[f"{side}_thigh"] = (np.array([0.0, s * self.y_hip, self.Z_hip]), self.t1, f"femur_{k}")
            self.frames[f"{side}_shank"] = (np.array([kx, s * self.y_knee, kz]), self.t2, f"tibia_{k}")
            self.frames[f"{side}_wheel_link"] = (np.array([0.0, s * self.y_wheel, self.R]), self.t2, f"wheel_{k}")
            # data from before the names followed the switch side (hub version 10 or earlier) would mirror the robot without a word
            assert s * self.cad["links"][f"wheel_{k}"]["com"][1] < 0.0, ("export names and CAD side disagree: re-export from a renamed model", k)
        self._check_joints(kz)

    def _check_joints(self, kz: float) -> None:
        """The joints as Fusion measured them, in CAD axes."""
        want = {"hip": (0.0, self.Z_hip), "knee": (self.kx_cad, kz), "wheel": (0.0, self.R)}
        for name, j in self.urdf["joints"].items():
            kind = name.split("_")[0]
            o = np.array(j["origin"]) / 1000.0
            assert abs(o[0] - want[kind][0]) < 1e-4 and abs(o[2] - want[kind][1]) < 1e-4, (name, o, want[kind])
            assert abs(j["axis"][1]) > 0.999999, (name, j["axis"])

    def inertial(self, link: str):
        """(mass, COM, inertia about the COM) in the link frame at q = 0."""
        o, t, cad_name = self.frames[link]
        c = self.cad["links"][cad_name]
        R = ry(t)
        A = R_ROBOT_FROM_CAD
        com = R.T @ (A @ np.array(c["com"]) / 1000.0 - o)
        return float(c["mass"]), com, R.T @ (A @ (np.array(c["inertia"]) * 1e-6) @ A.T) @ R

    def imu(self, chip: str = IMU_CHIP):
        """(position in the base frame [m], R_base_to_imu = v_imu = R v_base) from the measured chip and raw IMU axes."""
        d = self.urdf["imu"]
        A = R_ROBOT_FROM_CAD @ np.column_stack([d["axes"][a] for a in "xyz"])       # IMU axes in robot axes
        pos = R_ROBOT_FROM_CAD @ np.array(d["chips"][chip]) / 1000.0 - self.frames["base"][0]
        return pos, A.T


# ----------------------------------------------------------------------------- meshes
def read_stl_robot(path: Path) -> np.ndarray:
    """Triangles [n, 3, 3] of a Fusion STL in robot axes, metres (Fusion X, Y, Z = CAD y, z, x; millimetres)."""
    with open(path, "rb") as f:
        f.read(80)
        n = int(np.frombuffer(f.read(4), dtype="<u4")[0])
        d = np.frombuffer(f.read(), dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]), count=n)
    return d["v"].reshape(-1, 3, 3)[:, :, [2, 0, 1]].astype(float) / 1000.0 @ R_ROBOT_FROM_CAD


def write_stl(path: Path, tri: np.ndarray) -> None:
    e1, e2 = tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]
    nrm = np.cross(e1, e2)
    nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-20)
    rec = np.zeros(len(tri), dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    rec["n"] = nrm
    rec["v"] = tri.reshape(-1, 9)
    with open(path, "wb") as f:
        f.write(b"wheelbiped_v5 link mesh from Fusion".ljust(80, b" "))
        f.write(np.array([len(tri)], dtype="<u4").tobytes())
        f.write(rec.tobytes())


def simplify(tri: np.ndarray, target: int) -> np.ndarray:
    """Fewer triangles for the visual mesh when fast_simplification can be imported; otherwise the mesh as it is."""
    if len(tri) <= target:
        return tri
    try:
        import fast_simplification
    except Exception:
        return tri
    pts, inv = np.unique(np.round(tri.reshape(-1, 3), 7), axis=0, return_inverse=True)
    faces = inv.reshape(-1, 3)
    p2, f2 = fast_simplification.simplify(pts.astype(np.float32), faces.astype(np.int32), target_count=target)
    return p2[f2].astype(float)


def link_meshes(g: V5Geometry, out: Path) -> dict:
    """Writes meshes/<link>.stl (link frame at q = 0) and returns the vertices of each link, for the collision shapes."""
    out.mkdir(parents=True, exist_ok=True)
    verts = {}
    for link, (o, t, cad_name) in g.frames.items():
        src = STL_DIR / f"{cad_name}.stl"
        tri = (read_stl_robot(src) - o) @ ry(t)      # row vectors: v_link = R^T (v - o)
        verts[link] = tri.reshape(-1, 3)
        kind = "base" if link == "base" else link.split("_")[1]
        write_stl(out / f"{link}.stl", simplify(tri, MESH_TRIANGLES[kind]))
    return verts


def collision_shapes(g: V5Geometry, verts: dict) -> dict:
    """Simple shapes for the contacts with the floor, sized from the link meshes: {link: [(kind, size, origin xyz, rpy)]}.
    The tyre is a cylinder here; the MuJoCo model turns it into the rounded ellipsoid of the real profile."""
    shapes = {}
    v = verts["base"]
    lo, hi = v.min(axis=0), v.max(axis=0)
    shapes["base"] = [("box", hi - lo, 0.5 * (lo + hi), (0.0, 0.0, 0.0))]
    for side, s, _k in SIDES:
        v = verts[f"{side}_thigh"]
        mid = (v[:, 2] < -0.25 * g.L1) & (v[:, 2] > -0.75 * g.L1)
        hx = float(np.abs(v[mid, 0]).max())
        y0, y1 = float(v[:, 1].min()), float(v[:, 1].max())
        shapes[f"{side}_thigh"] = [("box", (2 * hx, y1 - y0, g.L1), (0.0, 0.5 * (y0 + y1), -0.5 * g.L1), (0.0, 0.0, 0.0))]
        v = verts[f"{side}_shank"]
        mid = (v[:, 2] < -0.25 * g.L2) & (v[:, 2] > -0.75 * g.L2)
        hx = float(np.abs(v[mid, 0]).max())
        y0, y1 = float(v[mid, 1].min()), float(v[mid, 1].max())
        beam = ("box", (2 * hx, y1 - y0, g.L2), (0.0, 0.5 * (y0 + y1), -0.5 * g.L2), (0.0, 0.0, 0.0))
        rad = np.hypot(v[:, 0], v[:, 2] + g.L2)      # wheel motor housing around the axle: what touches a tall bump
        near = v[:, 2] < -g.L2                       # the part beyond the axle (the beam comes from the knee side)
        r_h = float(rad[near].max())
        y0, y1 = float(v[near, 1].min()), float(v[near, 1].max())
        housing = ("cylinder", (r_h, y1 - y0), (0.0, 0.5 * (y0 + y1), -g.L2), (math.pi / 2, 0.0, 0.0))
        shapes[f"{side}_shank"] = [beam, housing]
        shapes[f"{side}_wheel_link"] = [("cylinder", (g.R, g.tyre_width), (0.0, s * (g.tyre_y - g.y_wheel), 0.0), (math.pi / 2, 0.0, 0.0))]
    return shapes


# ----------------------------------------------------------------------------- URDF
def _xyz(v) -> str:
    return " ".join(f"{float(x):.7g}" for x in v)


def urdf_text(g: V5Geometry, shapes: dict, with_meshes: bool = True, motor: Motor = MOTOR) -> str:
    def link(name: str) -> list:
        m, c, I = g.inertial(name)
        out = [f'  <link name="{name}">',
               f'    <inertial><origin xyz="{_xyz(c)}" rpy="0 0 0"/><mass value="{m:.7g}"/>'
               f'<inertia ixx="{I[0, 0]:.7g}" ixy="{I[0, 1]:.7g}" ixz="{I[0, 2]:.7g}" iyy="{I[1, 1]:.7g}" iyz="{I[1, 2]:.7g}" izz="{I[2, 2]:.7g}"/></inertial>']
        if with_meshes:
            out.append(f'    <visual><geometry><mesh filename="meshes/{name}.stl"/></geometry><material name="body"/></visual>')
        for kind, size, xyz, rpy in shapes[name]:
            geom = f'<box size="{_xyz(size)}"/>' if kind == "box" else f'<cylinder radius="{size[0]:.7g}" length="{size[1]:.7g}"/>'
            out.append(f'    <collision><origin xyz="{_xyz(xyz)}" rpy="{_xyz(rpy)}"/><geometry>{geom}</geometry></collision>')
        out.append("  </link>")
        return out

    def joint(name: str, kind: str, parent: str, child: str, xyz) -> list:
        lim = f'<limit effort="{motor.peak:g}" velocity="{motor.w0:.4g}"' + (' lower="-3.14159" upper="3.14159"/>' if kind == "revolute" else "/>")
        return [f'  <joint name="{name}" type="{kind}">', f'    <origin xyz="{_xyz(xyz)}" rpy="0 0 0"/><parent link="{parent}"/><child link="{child}"/>',
                f'    <axis xyz="0 1 0"/>{lim}', "  </joint>"]

    imu_pos, R_bi = g.imu()
    out = [f'<robot name="{NAME}">',
           "  <!-- Generated by wbr/models/build_v5.py from the Fusion model (robot_cad_v5). q = 0: straight legs. Base frame: midpoint of",
           "       the hip axes, x forward, y left, z up; every joint axis is +y. Masses, COMs and inertias: mass_properties.json. -->",
           '  <material name="body"><color rgba="0.25 0.25 0.27 1"/></material>']
    out += link("base")
    # the raw IMU axes of pi3hat docs/reference.md: +x CAN / HDMI side, +y USB side, +z from the Pi to the hat
    out += ['  <link name="imu"/>', '  <joint name="imu_fix" type="fixed">',
            f'    <origin xyz="{_xyz(imu_pos)}" rpy="{_xyz(rpy_of(R_bi.T))}"/><parent link="base"/><child link="imu"/>', "  </joint>"]
    for side, s, _k in SIDES:
        out += link(f"{side}_thigh") + link(f"{side}_shank") + link(f"{side}_wheel_link")
        out += joint(f"{side}_hip", "revolute", "base", f"{side}_thigh", (0.0, s * g.y_hip, 0.0))
        out += joint(f"{side}_knee", "revolute", f"{side}_thigh", f"{side}_shank", (0.0, s * (g.y_knee - g.y_hip), -g.L1))
        out += joint(f"{side}_wheel", "continuous", f"{side}_shank", f"{side}_wheel_link", (0.0, s * (g.y_wheel - g.y_knee), -g.L2))
    out.append("</robot>\n")
    return "\n".join(out)


# ----------------------------------------------------------------------------- MuJoCo model
def compile_spec(urdf_path: Path, g: V5Geometry, motor: Motor = MOTOR, limits: bool = True, payload=None):
    """The URDF plus what a simulation needs on top of it: free joint, motors, IMU sensors, contact settings, stand keyframe."""
    text = urdf_path.read_text(encoding="utf-8")
    ext = ('<mujoco><compiler meshdir="." discardvisual="false" fusestatic="true" balanceinertia="false" '
           'boundmass="0" boundinertia="0" strippath="false" autolimits="true"/></mujoco>')
    text = text.replace(f'<robot name="{NAME}">', f'<robot name="{NAME}">\n  {ext}', 1)
    patched = urdf_path.with_name(urdf_path.stem + "_compile.urdf")
    patched.write_text(text, encoding="utf-8")
    try:
        spec = mujoco.MjSpec.from_file(str(patched))
    finally:
        patched.unlink()
    spec.modelname = NAME
    spec.option.timestep = 0.0005
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.option.impratio = 10.0
    spec.option.noslip_iterations = 2
    bodies = {b.name: b for b in spec.bodies}
    base = bodies["base"]
    base.add_freejoint(name="base_freejoint")
    big = 1e4
    for j in spec.joints:
        kind = j.name.split("_")[-1]
        if kind in DAMPING:
            j.armature = motor.armature
            j.damping = [DAMPING[kind], 0.0, 0.0]
            j.frictionloss = motor.backdrive
            j.actfrcrange = [-motor.peak, motor.peak] if limits else [-big, big]
            j.actfrclimited = mujoco.mjtLimited.mjLIMITED_TRUE
    for g_ in spec.geoms:
        visual = g_.contype == 0 and g_.conaffinity == 0
        g_.group = 2 if visual else 3
        if not visual:
            g_.contype, g_.conaffinity = 2, 0       # robot parts touch the floor and the bumps, not each other
    for side, _s, _k in SIDES:
        for g_ in bodies[f"{side}_wheel_link"].geoms:
            if g_.type == mujoco.mjtGeom.mjGEOM_CYLINDER and g_.contype != 0:
                half = float(g_.size[1])
                g_.name = f"{side}_tire"
                g_.type = mujoco.mjtGeom.mjGEOM_ELLIPSOID
                g_.size = [float(g_.size[0]), float(g_.size[0]), half]
                g_.friction = TIRE_FRICTION
                g_.condim = 4
                g_.priority = 1
    for jn in JOINTS:
        a = spec.add_actuator(name=jn, target=jn, trntype=mujoco.mjtTrn.mjTRN_JOINT)
        a.ctrlrange = [-motor.peak, motor.peak] if limits else [-big, big]
        a.ctrllimited = mujoco.mjtLimited.mjLIMITED_TRUE
    # the IMU: a site on the base at the chip, with the raw IMU axes.  Quaternion of the site frame in the world
    imu_pos, R_bi = g.imu()
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, R_bi.T.flatten())
    base.add_site(name="imu", pos=imu_pos.tolist(), quat=q.tolist())
    spec.add_sensor(name="imu_quat", type=mujoco.mjtSensor.mjSENS_FRAMEQUAT, objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    spec.add_sensor(name="imu_gyro", type=mujoco.mjtSensor.mjSENS_GYRO, objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    spec.add_sensor(name="imu_accel", type=mujoco.mjtSensor.mjSENS_ACCELEROMETER, objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    if payload is not None:
        mp, size, pos = payload
        a, b_, c = size
        pb = base.add_body(name="payload", pos=list(pos))
        pb.explicitinertial = True
        pb.mass = mp
        pb.ipos = [0.0, 0.0, 0.0]
        pb.iquat = [1.0, 0.0, 0.0, 0.0]
        pb.inertia = [mp * (b_ * b_ + c * c) / 12, mp * (a * a + c * c) / 12, mp * (a * a + b_ * b_) / 12]
        pb.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[a / 2, b_ / 2, c / 2], contype=0, conaffinity=0, rgba=[0.9, 0.6, 0.2, 0.6], mass=0.0)
    key = spec.add_key(name="stand")
    model = spec.compile()
    data = mujoco.MjData(model)
    qpos0 = np.array(data.qpos)
    qpos0[2] = g.L1 + g.L2 + g.R        # straight legs, tyre on the floor
    key.qpos = qpos0.tolist()
    spec.compile()
    return spec


def _write_xml(spec, path: Path, payload) -> None:
    """(fusestatic merges the payload body into the base at the compile, so the written model has it in the base's inertial.)"""
    xml = spec.to_xml()
    if payload is not None:
        assert '<body name="payload"' not in xml, "the payload body should have been merged into the base"
    path.write_text(xml, encoding="utf-8")


def build(out_dir: Path = OUT_DIR, payload_kg: float = 0.0, payload_size=(0.20, 0.15, 0.10), with_meshes: bool = False, motor: Motor = MOTOR,
          limits: bool = True, verbose: bool = False, mass_json: Path = MASS_JSON, urdf_json: Path = URDF_JSON) -> SerialPlantParams:
    """Writes the model into out_dir and returns the controller's plant parameters.  with_meshes=False: a simulation-only copy
    without the visual meshes (same dynamics).  payload_kg: a box on the top plate, known to the controller's plant."""
    from wbr.models.upkie_params import build_params
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    g = V5Geometry(mass_json, urdf_json)
    verts = link_meshes(g, out_dir / "meshes" if with_meshes else out_dir / "_meshes_tmp")
    shapes = collision_shapes(g, verts)
    if not with_meshes:
        for f in (out_dir / "_meshes_tmp").glob("*.stl"):
            f.unlink()
        (out_dir / "_meshes_tmp").rmdir()
    urdf = out_dir / f"{NAME}.urdf"
    urdf.write_text(urdf_text(g, shapes, with_meshes, motor), encoding="utf-8")
    payload = None
    if payload_kg > 0:
        top = g.cad["pose"]["top_plate_z"] / 1000.0 + payload_size[2] / 2
        payload = (payload_kg, tuple(payload_size), (0.0, 0.0, top - g.Z_hip))
    spec = compile_spec(urdf, g, motor, limits, payload)
    _write_xml(spec, out_dir / f"{NAME}.xml", payload)
    (out_dir / "scene.xml").write_text(SCENE_XML.format(name=NAME), encoding="utf-8")
    params = build_params(out_dir / "scene.xml", verbose=verbose, signs=JOINT_SIGNS, name="wheelbiped_v5", knee_sign=KNEE_SIGN, **leg_range(g.L1))
    params.tau_wheel_max = motor.peak
    params.tau_hip_max = motor.peak
    params.to_json(out_dir / PARAMS_FILE)
    imu_pos, R_bi = g.imu()
    desc = dict(robot=NAME, wheel_radius=g.R, track=2.0 * g.tyre_y, tau_wheel_max=motor.peak, tau_leg_max=motor.peak, joint_signs=JOINT_SIGNS,
                knee_sign=KNEE_SIGN, front="rocker switch side (CAD -x)",
                imu=dict(chip=IMU_CHIP, position_in_base=imu_pos.tolist(), R_base_to_imu=R_bi.tolist()),
                payload_kg=payload_kg)
    (out_dir / DESCRIPTION_FILE).write_text(json.dumps(desc, indent=1), encoding="utf-8")
    return params


def main() -> None:
    from wbr.bench import ake_cad as AC
    params = build(OUT_DIR, with_meshes=True, verbose=True)
    desc = json.loads((OUT_DIR / DESCRIPTION_FILE).read_text(encoding="utf-8"))
    desc["plan_overrides"] = AC.v5_gains(OUT_DIR / "scene.xml", params)
    (OUT_DIR / DESCRIPTION_FILE).write_text(json.dumps(desc, indent=1), encoding="utf-8")
    m = mujoco.MjModel.from_xml_path(str(OUT_DIR / "scene.xml"))
    print(f"wrote {OUT_DIR}: nbody {m.nbody}, nq {m.nq}, nu {m.nu}, total mass {m.body_subtreemass[1]:.4f} kg")
    print(json.dumps(desc["plan_overrides"], indent=1))


if __name__ == "__main__":
    main()
