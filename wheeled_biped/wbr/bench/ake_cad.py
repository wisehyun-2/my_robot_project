"""Motor check of the v2 CAD robot (robot_cad_v2): AKE60-8 on all six joints, CAD link lengths, masses, COMs and
inertias, 150 mm wheel; with and without a 5 kg payload on the top plate.

The scenarios, controller (Plan 4 LQI6 re-identified on the model, MCU 1 kHz, noise + 2 ms latency) and motor model
(peak torque, voltage line, reflected rotor inertia, backdrive) are those of ake_sizing.run_case; only the model is
built differently (build_cad_model):

* upkie_description legs are stretched to the CAD lengths L1 = L2 = 175 mm (the knee stator and ankle bodies are
  moved along the hip->knee / knee->axle directions);
* each body group (base / thigh / shank / wheel, grouped like upkie_params by the last joint on the path) gets the
  CAD mass, COM (sagittal x, z; the lateral position stays upkie's) and inertia tensor on one carrier body, the other
  bodies of the group become massless (1e-4 kg);
* the payload is a 5 kg box (200 x 150 x 100 mm) on the top plate, its COM 50 mm above the plate.

    python -m wbr.bench.ake_cad                      # both cases -> results/ake_cad/
    python -m wbr.bench.ake_cad --only robot --quick
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
import time
from dataclasses import dataclass
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


def _principal(I):
    """3x3 inertia -> (diag, quaternion of the principal frame)."""
    vals, vecs = np.linalg.eigh(I)
    if np.linalg.det(vecs) < 0:
        vecs[:, 0] = -vecs[:, 0]
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, vecs.flatten())
    return np.maximum(vals, 1e-9), q


def build_cad_model(case: CadCase, mdir: Path):
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
    for grp, carrier in CARRIER.items():
        link = cad["links"][CAD_LINK[grp]]
        cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, carrier)
        mass = link["mass"]
        com_cad = np.array(link["com"]) / 1000.0
        # sagittal COM relative to the hip midpoint as in the CAD; lateral position = upkie's group COM
        cur = sum(m.body_mass[b] * d.xipos[b] for b in groups[grp]) / sum(m.body_mass[b] for b in groups[grp])
        com_w = hip_mid + (com_cad - cad_hip)
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
    return params


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
    L1, L2 = p.extra["cad_L1_L2"]
    return dict(tag=case.tag, total_mass=float(m.body_subtreemass[1]), L1=L1, L2=L2, R=case.radius, m_b=p.m_b,
                b_z=p.b_z, b_x=p.b_x, I_b=p.I_b, m_w=p.m_w, I_w=p.I_w, J_w=p.J_w)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", nargs="*")
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--out", type=Path, default=OUT_DIR)
    ap.add_argument("--static", action="store_true", help="only the model facts (masses, standing torques)")
    args = ap.parse_args(argv)
    cases = [c for c in CASES if not args.only or c.tag in args.only]
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
