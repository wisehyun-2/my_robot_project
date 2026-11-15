"""Motor check of the CAD robot (robot_cad_v5): AKE60-8 on all six joints, the robot's own URDF from the Fusion model
(wbr.models.build_v5: links, joints, masses, COMs, inertias, IMU axes and chip position) and its closed loop
(wbr.plans.plan_v5: the pi3hat filter and the MCU attitude filter, docs/robot_v5_model.md); with and without a 5 kg
payload on the top plate (a 200 x 150 x 100 mm box, its COM 50 mm above the plate).

The scenarios, controller (LQI8 designed on the model, MCU 1 kHz, noise + 2 ms latency) and motor model (peak torque,
voltage line, reflected rotor inertia, backdrive) are those of ake_sizing.run_case.  A scenario also counts as not
recovered when a knee ends on the wrong side of full extension (ake_sizing.knee_flipped).

    python -m wbr.bench.ake_cad                      # both cases, robot_cad_v5 -> results/ake_cad_v5/
    python -m wbr.bench.ake_cad --only robot --quick
    python -m wbr.bench.ake_cad --cad <mass_properties.json> --out results/<folder>   # another CAD export with urdf_data.json

The knee cancels the wheel motor's reaction on the shank (11-06, user decision); over a bump under one wheel the legs yield
and the IMU roll levels the body, and on a sharp step the wheels hold their speed difference (McuParams.kw_yaw, 11-07).
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import time
from dataclasses import dataclass, field
from multiprocessing import Pool
from pathlib import Path

import mujoco

from wbr import RESULTS_DIR
from wbr.bench import ake_sizing as AS

CAD_JSON = Path(__file__).resolve().parents[3] / "robot_cad_v5" / "calc" / "mass_properties.json"
OUT_DIR = RESULTS_DIR / "ake_cad_v5"


@dataclass
class CadCase(AS.Case):
    payload_kg: float = 0.0
    payload_size: tuple = (0.20, 0.15, 0.10)
    cad_json: str = str(CAD_JSON)
    track_m: float | None = None    # set by build_cad_model: tyre centre distance
    plan_overrides: dict = field(default_factory=dict)   # PlanV5Config fields for ake_sizing.make_plan
    params_file: str = "plant_params.json"      # plant parameters in the model folder (wbr.models.build_v5.PARAMS_FILE)

    def plan_classes(self):
        """(config class, plan class) of the robot: its own URDF and IMU chain (wbr.plans.plan_v5)."""
        from wbr.plans.plan_v5 import PlanV5, PlanV5Config
        return PlanV5Config, PlanV5


def build_cad_model(case: CadCase, mdir: Path):
    """The robot's own model in mdir: URDF from the Fusion model (wbr.models.build_v5), and its plan overrides in case."""
    from wbr.models import build_v5 as BV
    urdf_json = Path(case.cad_json).with_name("urdf_data.json")
    if not urdf_json.exists():
        raise FileNotFoundError(f"{urdf_json}: a CAD robot needs its urdf_data.json (robot_cad_v5: v5run.urdf_export())")
    params = BV.build(mdir, payload_kg=case.payload_kg, payload_size=case.payload_size, with_meshes=False, motor=case.motor("hip"),
                      limits=case.limits, mass_json=Path(case.cad_json), urdf_json=urdf_json)
    case.params_file = BV.PARAMS_FILE
    case.diameter_mm = 2000.0 * params.R
    case.track_m = json.loads((mdir / BV.DESCRIPTION_FILE).read_text(encoding="utf-8"))["track"]
    params.extra["cad_L1_L2"] = (params.extra["L1"], params.extra["L2"])
    case.plan_overrides = v5_gains(mdir / "scene.xml", params, motor=case.motor("hip"))
    return params


# compliant roll (11-07): roll mode bandwidth and damping, tuned on v5 with one-wheel bumps (TEST 2-7, 2-8): 1600 N/rad and
# 100 N s/rad per leg there.  Hip shaking under latency sets in at the same delay with or without it (11-07 sweep).
ROLL_WN = 34.0                  # rad/s
ROLL_ZETA = 1.06
ROLL_KI_RATIO = 0.6             # 1/s: ki = ratio x kp takes a one-sided load off the roll in about 2 s


def compliant_roll_gains(xml_path, m_b: float, kp_leg: float, kd_leg: float, wn: float = ROLL_WN) -> dict:
    """Gains of the compliant roll (McuParams.kp_roll_force ...) from the robot's roll plant (a payload case gets its own roll inertia).
    Soft walls: 8 x and 3 x the leg-length gains, so that a full differential force presses about 3 mm into a wall."""
    g = AS.roll_plant_gain(xml_path)["g"]
    kp = wn ** 2 / g
    return dict(kp_roll_force=kp, ki_roll_force=ROLL_KI_RATIO * kp, kd_roll_force=2.0 * ROLL_ZETA * wn / g,
                roll_force_int_limit=0.3 * m_b * 9.81, kp_leg_limit=8.0 * kp_leg, kd_leg_limit=3.0 * kd_leg)


# yaw on a sharp bump (11-07): tuned on v5 with one-wheel steps (TEST 2-11 to 2-13): 8 N m s/rad and 9.6 N m/rad there.
YAW_KW_RATIO = 5.0              # kw_yaw = ratio x kp_yaw.  The same gain on the gyro (kp_yaw x 3) goes unstable with the legs
YAW_KI_RATIO = 3.0              # ki_yaw x ratio: the heading comes back with (kp + kw) / ki = 1 s, x 1 leaves it 3 s.  x 4 is
                                # faster but a wheel stuck at a 4 cm step at 0.2-0.4 m/s then takes the hips to their peak


def yaw_wheel_damping_gains(kp_yaw: float, ki_yaw: float) -> dict:
    """McuParams.kw_yaw and the stiffer heading on top of the yaw PI of this case (a payload case keeps its own scale)."""
    return dict(kw_yaw=YAW_KW_RATIO * kp_yaw, ki_yaw=YAW_KI_RATIO * ki_yaw)


# The v5 robot's outer loops and LQR weights, set from its own plant (no reference to another robot).  The numbers came from the
# design work on the bench robots and were checked on v5 (v5_lqi/TEST/README.md); here they are bandwidths and torque scales.
LEG_WN = 23.93                  # rad/s, leg-length mode sqrt(2 kp / m_b): clear of the balance poles
LEG_ZETA = 0.598
YAW_WC = 10.06                  # rad/s, g kp_yaw (g = yaw plant gain W / (2 R Izz))
YAW_KI = 20.11                  # 1/s^2, g ki_yaw before YAW_KI_RATIO
FRICTION_COMP_RATIO = 0.8       # stiction compensation / the motor's backdrive torque
# Split PD (11-07, N m/rad and N m s/rad): a leg hit by a sharp step swings at its hip, and the damping term then takes the other hip to
# its limit, so the damping is kept low.  The stiffness only goes up: the split mode is close to neutral with the robot's weight on it
# (too soft and the legs run away in a turn).
SPLIT_KP = 114.28571428571429   # at 250 mm links; scaled with the link length (the split mode's negative stiffness goes with weight x leg length)
SPLIT_KD = 2.857142857142857
SPLIT_LINK = 0.25
LEG_WALL_SCALE = 0.5            # soft walls of the leg range against compliant_roll_gains: the knee peak on a 14-20 cm bump 8-12 -> 6-10 N m
LQR_R_WHEEL = 0.116             # LQR torque weight of the wheels: this share of m g R
LQR_R_HIP = 0.296               # LQR torque weight of the hips: this share of m g LQR_R_HIP_ARM
LQR_R_HIP_ARM = 0.31            # m, fixed, not the leg length: the hips drive the torso, whose inertia does not grow with the links.  With L0 here the hip loop
                                # got faster with longer links and a 13-14 Hz hip shake set in at 10 ms latency from 25 cm (8 of 28 scenarios recovered, 11-07)
V5_ROLL_WN = 17.0               # rad/s, half of ROLL_WN: the printed links' lateral flex (a sway mode the rigid model lacks).  With a hinge standing in for it, 34 rad/s
                                # shook the knees from 25 cm links (standing knee peak 10 N m, rms 4.1 -> 6.0); 17 rad/s does not (11-07)


def v5_gains(xml_path, params, motor=None) -> dict:
    """Plan overrides of a robot with its own model: yaw, leg length, roll and stiction compensation from its plant and motor."""
    motor = motor or AS.AKE60_8
    g = AS.yaw_plant_gain(xml_path)["g"]
    m_b = params.m_b
    kp_leg, kd_leg = 0.5 * m_b * LEG_WN ** 2, m_b * LEG_ZETA * LEG_WN
    k_link = params.extra["L1"] / SPLIT_LINK
    out = dict(kp_yaw=YAW_WC / g, ki_yaw=YAW_KI / g, kp_leg=kp_leg, kd_leg=kd_leg, kp_split=SPLIT_KP * k_link, kd_split=SPLIT_KD * k_link,
               friction_comp_Nm=FRICTION_COMP_RATIO * motor.backdrive, friction_comp_wheel_Nm=FRICTION_COMP_RATIO * motor.backdrive)
    out.update(compliant_roll_gains(xml_path, m_b, kp_leg, kd_leg, wn=V5_ROLL_WN))
    out["kp_leg_limit"] *= LEG_WALL_SCALE
    out["kd_leg_limit"] *= LEG_WALL_SCALE
    out.update(yaw_wheel_damping_gains(out["kp_yaw"], out["ki_yaw"]))
    return out


CASES = [CadCase("robot", hip_motor="AKE60-8", knee_motor="AKE60-8", wheel_motor="AKE60-8"),
         CadCase("payload5", hip_motor="AKE60-8", knee_motor="AKE60-8", wheel_motor="AKE60-8", payload_kg=5.0)]


def _run(case: CadCase):
    try:
        return case.tag, AS.run_case(case)
    except Exception as exc:
        import traceback
        return case.tag, {"error": repr(exc), "trace": traceback.format_exc()}


def static_report(case: CadCase) -> dict:
    """Model facts of a case (no simulation): masses, link lengths, plant parameters."""
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
                m_l=leg["m_l"], c=leg["c"], d=leg["d"], I_l=leg["I_l"], tire_track=tire_track,
                mcu_track=case.track_m, plan_overrides=dict(case.plan_overrides))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", nargs="*")
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--out", type=Path, default=OUT_DIR)
    ap.add_argument("--static", action="store_true", help="only the model facts (masses, standing torques)")
    ap.add_argument("--cad", type=Path, default=CAD_JSON,
                    help="mass_properties.json of the CAD robot, urdf_data.json next to it (default robot_cad_v5)")
    args = ap.parse_args(argv)
    cases = [dataclasses.replace(c, cad_json=str(args.cad)) for c in CASES if not args.only or c.tag in args.only]
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
