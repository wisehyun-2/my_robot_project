"""Motor check of the v2 CAD robot (robot_cad_v2): AKE60-8 on all six joints, CAD link lengths, masses, COMs and
inertias, 150 mm wheel; with and without a 5 kg payload on the top plate.

The scenarios, controller (Plan 4 LQI6 re-identified on the model, MCU 1 kHz, noise + 2 ms latency) and motor model
(peak torque, voltage line, reflected rotor inertia, backdrive) are those of ake_sizing.run_case; only the model is
built differently (build_cad_model):

* upkie_description legs are stretched to the CAD lengths L1 = L2 = 175 mm (the knee stator and ankle bodies are
  moved along the hip->knee / knee->axle directions);
* each body group (base / thigh / shank / wheel, grouped like upkie_params by the last joint on the path) gets the
  CAD mass, COM and inertia tensor on one carrier body, the other bodies of the group become massless (the compiler's
  boundmass, 1 g each);
* lateral geometry (11-05): with lateral = "cad" (default) the group COMs sit at the CAD y and the tires at the CAD
  tyre centre (pose.tyre_centre_y: 173.5 inboard for v2-v4, 270 outboard for v5), and the MCU gets that track; the
  joint axes are all along y, so the sagittal model is the same.  lateral = "upkie" keeps upkie's lateral positions
  (tire centres 305 mm apart whatever the CAD says: the results up to 11-02 were made that way - yaw inertia 8 % low,
  roll 4 %);
* the payload is a 5 kg box (200 x 150 x 100 mm) on the top plate, its COM 50 mm above the plate.

    python -m wbr.bench.ake_cad                      # both cases -> results/ake_cad/
    python -m wbr.bench.ake_cad --only robot --quick
    python -m wbr.bench.ake_cad --cad ../robot_cad_v3/calc/mass_properties.json --out results/ake_cad_v3   # v3 (6061 legs)
    python -m wbr.bench.ake_cad --cad ../robot_cad_v5/calc/mass_properties.json --out results/ake_cad_v5   # v5 (outboard wheels)
    python -m wbr.bench.ake_cad --lateral upkie --outer-gains upkie --wheel-reaction off ...  # the set-up before 11-05

Outer loops (11-05, user decision): the yaw PI and the leg-length PD are scaled from the upkie tuning to the CAD robot
(outer_loop_gains; `--outer-gains upkie` keeps the Plan 4 defaults).  A scenario also counts as not recovered when a
knee ends on the wrong side of full extension (ake_sizing.knee_flipped).

Knee coupling (11-06, user decision): the knee also cancels the wheel motor's reaction on the shank
(McuParams.wheel_reaction_ff; `--wheel-reaction off` = the results before 11-06).

Bumps under one wheel (11-07): the legs yield and the IMU roll levels the body (McuParams.roll_compliant; `--roll leg-length`
= the results before), and on a sharp step the wheels hold their speed difference (McuParams.kw_yaw; `--yaw-wheel off`).

Own model (11-07): a CAD folder with calc/urdf_data.json (v5) no longer goes through the stretched upkie_description above.
_build_from_urdf uses wbr.models.build_v5 and wbr.plans.plan_v5 instead: the robot's own URDF, IMU axes and chip position,
the pi3hat filter and the MCU attitude filter (docs/robot_v5_model.md).  `--model upkie` = the old route on the same CAD;
folders without that file (v2-v4) stay on the old route.
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
import time
from dataclasses import dataclass, field
from multiprocessing import Pool
from pathlib import Path

import mujoco
import numpy as np

from wbr import MODELS_DIR, RESULTS_DIR
from wbr.bench import ake_sizing as AS

CAD_JSON = Path(__file__).resolve().parents[3] / "robot_cad_v2" / "calc" / "mass_properties.json"
OUT_DIR = RESULTS_DIR / "ake_cad"
MASSLESS = 1e-4
CARRIER = {"base": "torso", "left_hip": "left_femur", "left_knee": "left_tibia", "left_wheel": "left_wheel_tire",
           "right_hip": "right_femur", "right_knee": "right_tibia", "right_wheel": "right_wheel_tire"}
CAD_LINK = {"base": "torso", "left_hip": "femur_L", "left_knee": "tibia_L", "left_wheel": "wheel_L",
            "right_hip": "femur_R", "right_knee": "tibia_R", "right_wheel": "wheel_R"}


@dataclass
class CadCase(AS.Case):
    payload_kg: float = 0.0
    payload_size: tuple = (0.20, 0.15, 0.10)
    cad_json: str = str(CAD_JSON)
    lateral: str = "cad"            # "cad": CAD lateral COMs / tyre centre / track; "upkie": upkie's (results before 11-05)
    track_m: float | None = None    # set by build_cad_model (lateral = "cad"): tyre centre distance for the MCU
    outer_gains: str = "scaled"     # "scaled": yaw PI and leg-length PD scaled from the upkie tuning to this robot
                                    # (build_cad_model -> plan_overrides); "upkie": Plan 4 defaults (results before 11-05)
    wheel_reaction_ff: bool = True  # knee cancels the wheel motor's reaction on the shank (False: results before 11-06)
    roll_compliant: bool = True     # a leg pressed by a bump yields, the IMU roll levels the body (False: results before 11-07)
    yaw_wheel_damping: bool = True  # the wheels hold their speed difference on a sharp bump (False: compliant roll results of 11-07)
    plan_overrides: dict = field(default_factory=dict)   # Plan4Config fields for ake_sizing.make_plan
    model: str = "urdf"             # "urdf": the robot's own URDF when the CAD folder has urdf_data.json (v5; wbr.models.build_v5);
                                    # "upkie": upkie_description stretched to the CAD lengths and masses (v2-v4, v5 before 11-07)
    params_file: str = "plan4_params.json"      # plant parameters in the model folder, set by build_cad_model

    def plan_classes(self):
        """(config class, plan class) of a robot with its own URDF; None: Plan 4 with patched module constants."""
        if self.params_file == "plan4_params.json":
            return None
        from wbr.plans.plan_v5 import PlanV5, PlanV5Config
        return PlanV5Config, PlanV5


def _anchors(m, d):
    jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
    out = {}
    for side in ("left", "right"):
        out[side] = tuple(np.array(d.xanchor[jid(f"{side}_{k}")]) for k in ("hip", "knee", "wheel"))
    return out


def _straight(spec):
    m = spec.compile()
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "stand"))
    mujoco.mj_forward(m, d)
    return m, d


def _stretch(spec, B, L1, L2):
    """Move the knee stator / ankle base bodies so that hip->knee = L1 and knee->axle = L2 (sagittal plane)."""
    for stage, (child, parent, i0, i1, L) in enumerate((("knee_stator", "femur", 0, 1, L1),
                                                         ("ankle_base", "tibia", 1, 2, L2))):
        m, d = _straight(spec)
        anc = _anchors(m, d)
        for side in ("left", "right"):
            a, b = anc[side][i0], anc[side][i1]
            v = b - a
            v[1] = 0.0
            cur = float(np.linalg.norm(v))
            dw = v * (L / cur - 1.0)
            pid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"{side}_{parent}")
            Rp = d.xmat[pid].reshape(3, 3)
            body = B[f"{side}_{child}"]
            body.pos = (np.array(body.pos) + Rp.T @ dw).tolist()
    m, d = _straight(spec)
    anc = _anchors(m, d)
    h, k, w = anc["left"]
    return float(np.linalg.norm((h - k)[[0, 2]])), float(np.linalg.norm((k - w)[[0, 2]]))


def _place_tires(spec, B, y_tyre):
    """Move the tire bodies (tire geom, contact / wheel-centre markers) along the wheel axis to y = +-y_tyre."""
    m, d = _straight(spec)
    for side, sgn in (("left", 1.0), ("right", -1.0)):
        gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"{side}_tire")
        dw = np.array([0.0, sgn * y_tyre - d.geom_xpos[gid][1], 0.0])
        pid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"{side}_wheel_hub")       # parent of the tire body
        body = B[f"{side}_wheel_tire"]
        body.pos = (np.array(body.pos) + d.xmat[pid].reshape(3, 3).T @ dw).tolist()


def _principal(I):
    """3x3 inertia -> (diag, quaternion of the principal frame)."""
    vals, vecs = np.linalg.eigh(I)
    if np.linalg.det(vecs) < 0:
        vecs[:, 0] = -vecs[:, 0]
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, vecs.flatten())
    return np.maximum(vals, 1e-9), q


def _build_from_urdf(case: CadCase, mdir: Path, urdf_json: Path):
    """The robot's own model: URDF from the Fusion model, no upkie file (wbr.models.build_v5)."""
    from wbr.models import build_v5 as BV
    params = BV.build(mdir, payload_kg=case.payload_kg, payload_size=case.payload_size, with_meshes=False, motor=case.motor("hip"),
                      limits=case.limits, mass_json=Path(case.cad_json), urdf_json=urdf_json)
    case.params_file = BV.PARAMS_FILE
    case.diameter_mm = 2000.0 * params.R
    case.track_m = json.loads((mdir / BV.DESCRIPTION_FILE).read_text(encoding="utf-8"))["track"]
    params.extra["cad_L1_L2"] = (params.extra["L1"], params.extra["L2"])
    case.plan_overrides = v5_gains(mdir / "scene.xml", params, wheel_reaction_ff=case.wheel_reaction_ff, roll_compliant=case.roll_compliant,
                                   yaw_wheel_damping=case.yaw_wheel_damping, motor=case.motor("hip"))
    return params


def build_cad_model(case: CadCase, mdir: Path):
    urdf_json = Path(case.cad_json).with_name("urdf_data.json")
    if case.model == "urdf" and urdf_json.exists():
        return _build_from_urdf(case, mdir, urdf_json)
    import wbr.models.build_upkie as bu
    from wbr.controllers.lqi6.serial_leg import SerialLeg, SerialLegGeometry
    from wbr.models.upkie_params import _group_bodies, build_params, set_leg_posture
    cad = json.loads(Path(case.cad_json).read_text(encoding="utf-8"))
    pose = cad["pose"]
    R = pose["R_wheel"] / 1000.0
    case.diameter_mm = 2000.0 * R
    mesh_abs = str(MODELS_DIR / "upkie_description").replace("\\", "/")
    orig = bu.patch_urdf
    bu.patch_urdf = lambda text, meshdir: orig(text, mesh_abs)
    try:
        bu.build(mdir, verbose=False, tire_radius=R, tire_shape="ellipsoid", tire_condim=4)
    finally:
        bu.patch_urdf = orig
    spec = mujoco.MjSpec.from_file(str(mdir / "upkie.xml"))
    spec.meshdir = mesh_abs
    B = {b.name: b for b in spec.bodies}
    L1, L2 = _stretch(spec, B, pose["L1"] / 1000.0, pose["L2"] / 1000.0)
    if case.lateral not in ("cad", "upkie"):
        raise ValueError(f"lateral must be 'cad' or 'upkie', not {case.lateral!r}")
    cad_lateral = case.lateral == "cad"
    if cad_lateral:
        _place_tires(spec, B, pose["tyre_centre_y"] / 1000.0)
        case.track_m = 2.0 * pose["tyre_centre_y"] / 1000.0

    # CAD pose in MuJoCo: virtual leg vertical at L0, axle at x = 0, z = R (same as the CAD nominal pose)
    m = spec.compile()
    d = mujoco.MjData(m)
    leg = SerialLeg(SerialLegGeometry(L1=L1, L2=L2, knee_sign=-1.0))
    set_leg_posture(m, d, leg, pose["L0"] / 1000.0, 0.0, wheel_radius=R)
    groups = _group_bodies(m)
    jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
    hip_mid = 0.5 * (d.xanchor[jid("left_hip")] + d.xanchor[jid("right_hip")])
    cad_hip = np.array([0.0, 0.0, pose["Z_hip"] / 1000.0])
    names = {i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(m.nbody)}
    # this route keeps the CAD axes, left = +y; v5 names its parts from the rocker switch side, so its +y parts are _R
    v5_names = cad["links"]["wheel_L"]["com"][1] < 0.0
    for grp, carrier in CARRIER.items():
        link = cad["links"][CAD_LINK[grp].translate(str.maketrans("LR", "RL")) if v5_names else CAD_LINK[grp]]
        cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, carrier)
        mass = link["mass"]
        com_cad = np.array(link["com"]) / 1000.0
        # COM relative to the hip midpoint as in the CAD; lateral = "upkie" keeps upkie's lateral group COM
        cur = sum(m.body_mass[b] * d.xipos[b] for b in groups[grp]) / sum(m.body_mass[b] for b in groups[grp])
        com_w = hip_mid + (com_cad - cad_hip)
        if not cad_lateral:
            com_w[1] = cur[1]
        Rb = d.xmat[cid].reshape(3, 3)
        I_w = np.array(link["inertia"]) * 1e-6                       # kg mm^2 -> kg m^2, robot axes = world axes
        diag, quat = _principal(Rb.T @ I_w @ Rb)
        for b in groups[grp]:
            body = B[names[b]]
            if b == cid:
                body.mass = mass
                body.ipos = (Rb.T @ (com_w - d.xpos[cid])).tolist()
                body.inertia = diag.tolist()
                body.iquat = quat.tolist()
            else:
                body.mass = MASSLESS
                body.inertia = [1e-9, 1e-9, 1e-9]
        if grp == "base" and case.payload_kg > 0:
            top = pose["top_plate_z"] / 1000.0 + case.payload_size[2] / 2
            p_w = hip_mid + np.array([0.0, 0.0, top - cad_hip[2]])
            pb = B["torso"].add_body(name="payload", pos=(Rb.T @ (p_w - d.xpos[cid])).tolist())
            a, b_, c = case.payload_size
            mp = case.payload_kg
            pb.explicitinertial = True
            pb.mass = mp
            pb.inertia = [mp * (b_ * b_ + c * c) / 12, mp * (a * a + c * c) / 12, mp * (a * a + b_ * b_) / 12]
            pb.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[a / 2, b_ / 2, c / 2], contype=0, conaffinity=0,
                        rgba=[0.9, 0.6, 0.2, 0.6], mass=0.0)

    for side in ("left", "right"):                                      # tire touches only the floor
        for g in B[f"{side}_wheel_tire"].geoms:
            if g.name == f"{side}_tire":
                g.contype, g.conaffinity = 2, 0
    big = 1e4
    for j in spec.joints:
        kind = j.name.split("_")[-1] if j.name.startswith(("left_", "right_")) else None
        if kind in ("hip", "knee", "wheel"):
            mtr = case.motor(kind)
            j.armature = mtr.armature
            j.frictionloss = mtr.backdrive
            j.actfrcrange = [-mtr.peak, mtr.peak] if case.limits else [-big, big]
    for a in spec.actuators:
        mtr = case.motor(a.name.split("_")[-1])
        a.ctrlrange = [-mtr.peak, mtr.peak] if case.limits else [-big, big]
    spec.compile()
    xml = spec.to_xml()
    if case.payload_kg > 0:      # MjSpec does not write the added body's inertial: insert it
        a, b_, c = case.payload_size
        mp = case.payload_kg
        inertial = '<inertial pos="0 0 0" mass="%g" diaginertia="%g %g %g"/>' % (
            mp, mp * (b_ * b_ + c * c) / 12, mp * (a * a + c * c) / 12, mp * (a * a + b_ * b_) / 12)
        i = xml.find('<body name="payload"')
        j = xml.find('>', i) + 1
        xml = xml[:j] + "\n          " + inertial + xml[j:]
    (mdir / "upkie.xml").write_text(xml, encoding="utf-8")
    scene = (mdir / "scene.xml").read_text(encoding="utf-8")
    (mdir / "scene.xml").write_text(scene.replace('<geom name="floor"', '<geom name="floor" contype="1" conaffinity="3"', 1),
                                    encoding="utf-8")
    params = build_params(mdir / "scene.xml", verbose=False)
    k_m = params.extra["total_mass"] / AS.UPKIE_MASS
    params.tau_wheel_max = 1.7 * k_m * R / AS.UPKIE_R
    params.tau_hip_max = 16.0 * k_m
    params.to_json(mdir / "plan4_params.json")
    params.extra["cad_L1_L2"] = (L1, L2)
    if case.outer_gains not in ("scaled", "upkie"):
        raise ValueError(f"outer_gains must be 'scaled' or 'upkie', not {case.outer_gains!r}")
    case.plan_overrides = outer_loop_gains(mdir / "scene.xml", params.m_b) if case.outer_gains == "scaled" else {}
    case.plan_overrides["wheel_reaction_ff"] = case.wheel_reaction_ff
    case.plan_overrides["roll_compliant"] = case.roll_compliant
    case.plan_overrides.update(compliant_roll_gains(mdir / "scene.xml", params.m_b, case.plan_overrides.get("kp_leg", 800.0),
                                                    case.plan_overrides.get("kd_leg", 40.0)))
    if case.yaw_wheel_damping:
        case.plan_overrides.update(yaw_wheel_damping_gains(case.plan_overrides))
    return params


def outer_loop_gains(xml_path, m_b: float) -> dict:
    """Plan 4's yaw PI and leg-length PD (tuned on the 5.3 kg upkie model) scaled to this robot so that the loops keep
    their upkie bandwidth (11-05, user decision):
      yaw   kp, ki x (upkie yaw plant gain) / (this robot's), plant gain = W / (2 R Izz) - a 10.9 kg robot on Ø202
            wheels 0.54 m apart has 1/4 of upkie's, with the defaults a 1 rad/s command took 1.3 s to 90 % and stalled
            at 0.5 rad/s with 5 kg on top;
      leg   kp, kd x m_b / (upkie m_b): the leg-length mode sqrt(2 kp / m_b) stays at 23.9 rad/s, clear of the
            balance poles.  With the defaults (17 rad/s, 12.5 with 5 kg) the wheel-torque reaction on the shank
            stretched the leg by 2 cm on a 15 deg slope and through full extension when stopping downhill with 5 kg:
            the knee flipped backward.
    A payload case is scaled with its own mass / inertia, like its LQI gains (the controller knows the payload)."""
    import wbr.plans.plan4_upkie_lqi6 as p4
    base = p4.Plan4Config()
    ky = AS.UPKIE_YAW_GAIN / AS.yaw_plant_gain(xml_path)["g"]
    kl = m_b / AS.UPKIE_BODY_MASS
    return dict(kp_yaw=base.kp_yaw * ky, ki_yaw=base.ki_yaw * ky, kp_leg=base.kp_leg * kl, kd_leg=base.kd_leg * kl)


# compliant roll (11-07): roll mode bandwidth and damping, tuned on v5 with one-wheel bumps (TEST 2-7, 2-8): 1600 N/rad and
# 100 N s/rad per leg there.  Hip shaking under latency sets in at the same delay with or without it (11-07 sweep).
ROLL_WN = 34.0                  # rad/s
ROLL_ZETA = 1.06
ROLL_KI_RATIO = 0.6             # 1/s: ki = ratio x kp takes a one-sided load off the roll in about 2 s


def compliant_roll_gains(xml_path, m_b: float, kp_leg: float, kd_leg: float, wn: float = ROLL_WN) -> dict:
    """Gains of McuParams.roll_compliant from the robot's roll plant (a payload case gets its own roll inertia).
    Soft walls: 8 x and 3 x the leg-length gains, so that a full differential force presses about 3 mm into a wall."""
    g = AS.roll_plant_gain(xml_path)["g"]
    kp = wn ** 2 / g
    return dict(kp_roll_force=kp, ki_roll_force=ROLL_KI_RATIO * kp, kd_roll_force=2.0 * ROLL_ZETA * wn / g,
                roll_force_int_limit=0.3 * m_b * 9.81, kp_leg_limit=8.0 * kp_leg, kd_leg_limit=3.0 * kd_leg)


# yaw on a sharp bump (11-07): tuned on v5 with one-wheel steps (TEST 2-11 to 2-13): 8 N m s/rad and 9.6 N m/rad there.
YAW_KW_RATIO = 5.0              # kw_yaw = ratio x kp_yaw.  The same gain on the gyro (kp_yaw x 3) goes unstable with the legs
YAW_KI_RATIO = 3.0              # ki_yaw x ratio: the heading comes back with (kp + kw) / ki = 1 s, x 1 leaves it 3 s.  x 4 is
                                # faster but a wheel stuck at a 4 cm step at 0.2-0.4 m/s then takes the hips to their peak


def yaw_wheel_damping_gains(overrides: dict) -> dict:
    """McuParams.kw_yaw and the stiffer heading on top of the yaw PI of this case (a payload case keeps its own scale)."""
    import wbr.plans.plan4_upkie_lqi6 as p4
    base = p4.Plan4Config()
    return dict(kw_yaw=YAW_KW_RATIO * overrides.get("kp_yaw", base.kp_yaw), ki_yaw=YAW_KI_RATIO * overrides.get("ki_yaw", base.ki_yaw))


# The v5 robot's outer loops and LQR weights, set from its own plant (no reference to another robot).  The numbers came from the
# design work on the bench robots and were checked on v5 (TEST/README.md); here they are bandwidths and torque scales.
LEG_WN = 23.93                  # rad/s, leg-length mode sqrt(2 kp / m_b): clear of the balance poles
LEG_ZETA = 0.598
YAW_WC = 10.06                  # rad/s, g kp_yaw (g = yaw plant gain W / (2 R Izz))
YAW_KI = 20.11                  # 1/s^2, g ki_yaw before YAW_KI_RATIO
FRICTION_COMP_RATIO = 0.8       # stiction compensation / the motor's backdrive torque
# Split PD (11-07, N m/rad and N m s/rad): a leg hit by a sharp step swings at its hip, and the damping term then takes the other hip to
# its limit (60 / 3: 11 N m at 1 m/s; damping 2: 8 N m).  The stiffness only goes up: the split mode is close to neutral with the robot's
# weight on it (peak split in a turn: 2.4 deg at 80, 3.4 at 60, 5.7 at 50, the legs run away at 30).
SPLIT_KP = 80.0                 # at 175 mm links; scaled with the link length (the split mode's negative stiffness goes with weight x leg length)
SPLIT_KD = 2.0
SPLIT_LINK = 0.175
LEG_WALL_SCALE = 0.5            # soft walls of the leg range against compliant_roll_gains: the knee peak on a 14-20 cm bump 8-12 -> 6-10 N m
LQR_R_WHEEL = 0.116             # LQR torque weight of the wheels: this share of m g R
LQR_R_HIP = 0.296               # LQR torque weight of the hips: this share of m g LQR_R_HIP_ARM
LQR_R_HIP_ARM = 0.31            # m, fixed, not the leg length: the hips drive the torso, whose inertia does not grow with the links.  With L0 here the hip loop
                                # got faster with longer links and a 13-14 Hz hip shake set in at 10 ms latency from 25 cm (8 of 28 scenarios recovered, 11-07)
V5_ROLL_WN = 17.0               # rad/s, half of ROLL_WN: the printed links' lateral flex (a sway mode the rigid model lacks).  With a hinge standing in for it, 34 rad/s
                                # shook the knees from 25 cm links (standing knee peak 10 N m, rms 4.1 -> 6.0); 17 rad/s does not (11-07)


def v5_gains(xml_path, params, wheel_reaction_ff: bool = True, roll_compliant: bool = True, yaw_wheel_damping: bool = True, motor=None) -> dict:
    """Plan overrides of a robot with its own model: yaw, leg length, roll and stiction compensation from its plant and motor."""
    motor = motor or AS.AKE60_8
    g = AS.yaw_plant_gain(xml_path)["g"]
    m_b = params.m_b
    kp_leg, kd_leg = 0.5 * m_b * LEG_WN ** 2, m_b * LEG_ZETA * LEG_WN
    k_link = params.extra["L1"] / SPLIT_LINK
    out = dict(kp_yaw=YAW_WC / g, ki_yaw=YAW_KI / g, kp_leg=kp_leg, kd_leg=kd_leg, kp_split=SPLIT_KP * k_link, kd_split=SPLIT_KD * k_link, wheel_reaction_ff=wheel_reaction_ff, roll_compliant=roll_compliant,
               friction_comp_Nm=FRICTION_COMP_RATIO * motor.backdrive, friction_comp_wheel_Nm=FRICTION_COMP_RATIO * motor.backdrive)
    out.update(compliant_roll_gains(xml_path, m_b, kp_leg, kd_leg, wn=V5_ROLL_WN))
    out["kp_leg_limit"] *= LEG_WALL_SCALE
    out["kd_leg_limit"] *= LEG_WALL_SCALE
    if yaw_wheel_damping:
        out.update(yaw_wheel_damping_gains(out))
    return out


CASES = [CadCase("robot", hip_motor="AKE60-8", knee_motor="AKE60-8", wheel_motor="AKE60-8"),
         CadCase("payload5", hip_motor="AKE60-8", knee_motor="AKE60-8", wheel_motor="AKE60-8", payload_kg=5.0)]


def _run(case: CadCase):
    AS.build_model = build_cad_model          # run_case looks it up at call time (also in pool workers)
    try:
        return case.tag, AS.run_case(case)
    except Exception as exc:
        import traceback
        return case.tag, {"error": repr(exc), "trace": traceback.format_exc()}


def static_report(case: CadCase) -> dict:
    """Model facts of a case (no simulation): masses, link lengths, Plan 4 plant parameters."""
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        p = build_cad_model(case, Path(tmp) / "model")
        m = mujoco.MjModel.from_xml_path(str(Path(tmp) / "model" / "scene.xml"))
        d = mujoco.MjData(m)
        mujoco.mj_resetDataKeyframe(m, d, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "stand"))
        mujoco.mj_forward(m, d)
        tire_track = float(abs(d.geom_xpos[m.geom("left_tire").id][1] - d.geom_xpos[m.geom("right_tire").id][1]))
    L1, L2 = p.extra["cad_L1_L2"]
    leg = p.leg.at(p.L0)
    return dict(tag=case.tag, total_mass=float(m.body_subtreemass[1]), L1=L1, L2=L2, R=case.radius, m_b=p.m_b,
                b_z=p.b_z, b_x=p.b_x, I_b=p.I_b, m_w=p.m_w, I_w=p.I_w, J_w=p.J_w,
                m_l=leg["m_l"], c=leg["c"], d=leg["d"], I_l=leg["I_l"], lateral=case.lateral, tire_track=tire_track,
                mcu_track=case.track_m, outer_gains=case.outer_gains, plan_overrides=dict(case.plan_overrides))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", nargs="*")
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--out", type=Path, default=OUT_DIR)
    ap.add_argument("--static", action="store_true", help="only the model facts (masses, standing torques)")
    ap.add_argument("--cad", type=Path, default=CAD_JSON,
                    help="mass_properties.json of the CAD robot (default robot_cad_v2; v3: robot_cad_v3/calc/...)")
    ap.add_argument("--lateral", choices=("cad", "upkie"), default="cad",
                    help="lateral COMs / tyre centre / track from the CAD (default) or upkie's (results before 11-05)")
    ap.add_argument("--outer-gains", choices=("scaled", "upkie"), default="scaled",
                    help="yaw PI / leg-length PD scaled to the robot (default) or the upkie tuning (results before 11-05)")
    ap.add_argument("--wheel-reaction", choices=("on", "off"), default="on",
                    help="knee cancels the wheel motor's reaction on the shank (default) or not (results before 11-06)")
    ap.add_argument("--roll", choices=("compliant", "leg-length"), default="compliant",
                    help="legs yield to a bump, IMU roll levels the body (default), or a leg-length difference (results before 11-07)")
    ap.add_argument("--model", choices=("urdf", "upkie"), default="urdf",
                    help="the robot's own URDF when the CAD folder has one (default; v5) or upkie_description stretched to the CAD (v5 results before 11-07)")
    ap.add_argument("--yaw-wheel", choices=("on", "off"), default="on",
                    help="the wheels hold their speed difference on a sharp bump (default) or only the gyro PI (compliant roll results of 11-07)")
    args = ap.parse_args(argv)
    cases = [dataclasses.replace(c, cad_json=str(args.cad), lateral=args.lateral, outer_gains=args.outer_gains,
                                 wheel_reaction_ff=args.wheel_reaction == "on", roll_compliant=args.roll == "compliant",
                                 yaw_wheel_damping=args.yaw_wheel == "on", model=args.model) for c in CASES
             if not args.only or c.tag in args.only]
    args.out.mkdir(parents=True, exist_ok=True)
    stat = {c.tag: static_report(dataclasses.replace(c)) for c in cases}
    (args.out / "static.json").write_text(json.dumps(stat, indent=1), encoding="utf-8")
    for t, s in stat.items():
        print(t, json.dumps({k: (round(v, 4) if isinstance(v, float) else v) for k, v in s.items()}))
    if args.static:
        return
    for c in cases:
        c.quick = args.quick
    t0 = time.time()
    with Pool(len(cases), maxtasksperchild=1) as pool:
        for tag, res in pool.imap_unordered(_run, cases):
            (args.out / f"{tag}.json").write_text(json.dumps(res, indent=1), encoding="utf-8")
            print(f"{tag}: {'ERROR ' + res['error'] if 'error' in res else 'done'} ({time.time() - t0:.0f} s)")
            if "trace" in res:
                print(res["trace"])
    results = {f.stem: json.loads(f.read_text(encoding="utf-8")) for f in sorted(args.out.glob("*.json"))
               if f.stem != "static"}
    (args.out / "summary.md").write_text(AS.summary_markdown(results), encoding="utf-8")
    print(f"wrote {args.out / 'summary.md'}")


if __name__ == "__main__":
    main()
