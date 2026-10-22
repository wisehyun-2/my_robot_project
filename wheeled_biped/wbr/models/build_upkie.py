"""Build the MuJoCo model of Upkie from the upkie_description URDF (Plan 1).

Steps: patch package:// mesh paths, let MuJoCo parse the URDF into an MjSpec, add a free joint,
joint dynamics (rotor armature / friction), torque actuators named like the joints, an IMU site
with quaternion/gyro/accelerometer sensors, name the tire collision geoms, compute the standing
height and write models/upkie/upkie.xml plus a scene file with floor, lights and a tracking camera.
"""
from __future__ import annotations

import math
import re
from pathlib import Path

import mujoco
import numpy as np

from wbr import MODELS_DIR

URDF_SRC = MODELS_DIR / "upkie_description" / "urdf" / "upkie.urdf"
OUT_DIR = MODELS_DIR / "upkie"
JOINTS = ["left_hip", "left_knee", "left_wheel", "right_hip", "right_knee", "right_wheel"]
TORQUE_LIMITS = {"hip": 16.0, "knee": 16.0, "wheel": 1.7}
TIRE_RADIUS = 0.05  # collision cylinder radius in the URDF (upkie.model.Model reads the same value)

SCENE_XML = """<mujoco model="upkie_scene">
  <include file="upkie.xml"/>
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
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane" friction="1.0 0.005 0.0001" condim="3"/>
    <camera name="track" mode="trackcom" pos="1.2 -1.2 0.55" xyaxes="0.707 0.707 0 -0.25 0.25 0.94"/>
    <camera name="side" mode="trackcom" pos="0 -2.0 0.4" xyaxes="1 0 0 0 0.2 0.98"/>
  </worldbody>
</mujoco>
"""


def patch_urdf(text: str, meshdir: str) -> str:
    text = text.replace("package://upkie_description/", "")
    ext = (f'<mujoco><compiler meshdir="{meshdir}" discardvisual="false" fusestatic="false" '
           f'balanceinertia="true" boundmass="0.001" boundinertia="1e-7" strippath="false"/></mujoco>')
    return re.sub(r"(<robot[^>]*>)", r"\1\n  " + ext, text, count=1)


def build(out_dir: Path = OUT_DIR, verbose: bool = True, tire_radius: float = TIRE_RADIUS, tire_shape: str = "cylinder") -> Path:
    """tire_radius: collision cylinder radius.  upkie_description models a 100 mm tire (0.05); the BOM of the real
    robot lists 112 mm RC tires (0.056) and upkie's spine configuration uses 0.06.  Plan 4 builds a second model
    directory (models/upkie_plan4) with the BOM value -- see docs/plan4_model.md.
    tire_shape: "cylinder" (upkie_description: a 40 mm wide flat cylinder, i.e. two contact points at the rims that scrub
    against each other when turning in place) or "ellipsoid" (rounded RC tire profile, one contact point; Plan 4)."""
    out_dir.mkdir(parents=True, exist_ok=True)
    patched = out_dir / "upkie_patched.urdf"
    patched.write_text(patch_urdf(URDF_SRC.read_text(encoding="utf-8"), "../upkie_description"), encoding="utf-8")
    spec = mujoco.MjSpec.from_file(str(patched))
    spec.modelname = "upkie"

    # options
    spec.option.timestep = 0.0005
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.option.impratio = 10.0
    spec.option.noslip_iterations = 2

    bodies = {b.name: b for b in spec.bodies}
    base = bodies["base"]
    base.add_freejoint(name="base_freejoint")

    # joints: rotor armature and friction (qdd100 geared 6:1 on hips/knees, mj5208 direct drive on wheels)
    for j in spec.joints:
        if j.name in ("left_hip", "left_knee", "right_hip", "right_knee"):
            j.armature = 0.005
            j.damping = [0.02, 0.0, 0.0]
            j.frictionloss = 0.1
        elif j.name in ("left_wheel", "right_wheel"):
            j.armature = 0.0003
            j.damping = [0.001, 0.0, 0.0]
            j.frictionloss = 0.02

    # geoms: meshes are visual only; name tire collision cylinders; tire friction
    # (MuJoCo's URDF importer marks visual geoms with contype = conaffinity = 0)
    for g in spec.geoms:
        g.group = 2 if (g.contype == 0 and g.conaffinity == 0) else 3
        if g.type != mujoco.mjtGeom.mjGEOM_MESH and float(np.max(np.abs(g.size))) <= 0.0:
            g.size = [1e-4, 1e-4, 1e-4]   # zero-size visual of the URDF virtual links
    for side in ("left", "right"):
        tire = bodies[f"{side}_wheel_tire"]
        for g in tire.geoms:
            if g.type == mujoco.mjtGeom.mjGEOM_CYLINDER and g.contype != 0:
                g.name = f"{side}_tire"
                half_width = float(g.size[1])
                if tire_shape == "ellipsoid":
                    g.type = mujoco.mjtGeom.mjGEOM_ELLIPSOID
                    g.size = [tire_radius, tire_radius, half_width]
                else:
                    g.size = [tire_radius, half_width, 0.0]
                g.friction = [1.0, 0.005, 0.0001]
                g.condim = 3
                g.priority = 1

    # actuators
    for jn in JOINTS:
        kind = jn.split("_")[1]
        lim = TORQUE_LIMITS[kind]
        a = spec.add_actuator(name=jn, target=jn, trntype=mujoco.mjtTrn.mjTRN_JOINT)
        a.ctrlrange = [-lim, lim]
        a.ctrllimited = True

    # IMU site on the imu body and sensors
    imu = bodies["imu"]
    imu.add_site(name="imu", pos=[0, 0, 0])
    base.add_site(name="base", pos=[0, 0, 0])
    s = spec.add_sensor(name="imu_quat", type=mujoco.mjtSensor.mjSENS_FRAMEQUAT,
                        objtype=mujoco.mjtObj.mjOBJ_SITE, objname="base")
    s = spec.add_sensor(name="imu_gyro", type=mujoco.mjtSensor.mjSENS_GYRO,
                        objtype=mujoco.mjtObj.mjOBJ_SITE, objname="base")
    s = spec.add_sensor(name="imu_accel", type=mujoco.mjtSensor.mjSENS_ACCELEROMETER,
                        objtype=mujoco.mjtObj.mjOBJ_SITE, objname="base")

    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    # standing height: tire bottom at z = 0 with straight legs
    tire_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "left_wheel_tire")
    z_tire = float(data.xpos[tire_bid][2])
    base_z = -z_tire + tire_radius
    key = spec.add_key(name="stand")
    qpos0 = np.array(data.qpos)
    qpos0[2] = base_z
    key.qpos = qpos0.tolist()
    model = spec.compile()
    xml = spec.to_xml()
    (out_dir / "upkie.xml").write_text(xml, encoding="utf-8")
    (out_dir / "scene.xml").write_text(SCENE_XML, encoding="utf-8")
    if verbose:
        print(f"wrote {out_dir / 'upkie.xml'} ({len(xml)} bytes); nbody={model.nbody} njnt={model.njnt} nu={model.nu}")
        print(f"standing base height = {base_z:.4f} m, total mass = {float(model.body_subtreemass[1]):.3f} kg")
    return out_dir / "scene.xml"


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=None, help="output directory (default models/upkie)")
    ap.add_argument("--tire-radius", type=float, default=TIRE_RADIUS)
    ap.add_argument("--tire-shape", default="cylinder", choices=("cylinder", "ellipsoid"))
    args = ap.parse_args()
    build(Path(args.out) if args.out else OUT_DIR, tire_radius=args.tire_radius, tire_shape=args.tire_shape)
