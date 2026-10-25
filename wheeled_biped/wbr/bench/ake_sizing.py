"""The user's robot with CubeMars AKE motors: MuJoCo check of the motor torques and of the usable wheel diameter,
docs/ake_motor_sizing.md.

Robot (the values saved in the knee-torque calculator, 2026-10-24): upkie_description leg geometry (Plan 4 builder:
ellipsoid tire, condim 4), torso 2.5 kg with the 1.8 kg battery centred so that the torso COM is on the hip line,
upkie link / wheel masses, hip and knee = AKE50-8 KV110 (0.19 kg), wheel = AKE60-8 KV80 (0.26 kg).  Motor model:
peak torque, the voltage line of the torque-speed curve (no-load speed at the rated 24 V, winding resistance) while
motoring, reflected rotor inertia (rotor inertia x gear^2) as joint armature, backdrive torque as joint friction.
Controller: Plan 4 (MCU 1 kHz, LQI6, noise + 2 ms latency) with the plant re-identified from this model; the LQI6
torque budget is the Plan 4 tuning scaled by mass and wheel radius (the validated closed loop), the actuators clip at
the real motor limits.  Plan 4 itself is not modified: its model folder, wheel radius and torque limits are patched for
the duration of one case.  Besides the joint torques and speeds every step records the tire normal forces and the
wheel slip (wheel-centre speed along the floor - rim speed), to see when a large wheel leaves the floor and spins.

    python -m wbr.bench.ake_sizing                       # standard cases (16, ~70 s each, parallel) -> results/ake_sizing/
    python -m wbr.bench.ake_sizing --only d150 d400
    python -m wbr.bench.ake_sizing --diameter 180 --tag my_case [--seed 1] [--volt 22.2] [--no-limits]
    python -m wbr.bench.ake_sizing --clearance [150 250 300]   # tire-to-leg gaps -> results/ake_sizing/clearance.md

In upkie the wheel sits inboard of the shank, in the same lateral band as the thigh link and the knee-motor bracket on
the thigh (a cylinder of ~50 mm radius around the knee axis, which stays 175 mm = shank length from the axle): from a
250 mm wheel diameter the tire overlaps the bracket in every posture (the thigh link from ~300 mm), the wheel locks and
the robot slides.  The simulation therefore lets the tire touch only the floor, i.e. it assumes a build in which the
wheel clears the leg; `--clearance` reports the gaps of the upkie layout.
"""
from __future__ import annotations

import argparse
import contextlib
import dataclasses
import json
import math
import tempfile
import time
from dataclasses import dataclass
from multiprocessing import Pool, cpu_count
from pathlib import Path
from typing import Dict, List

import mujoco
import numpy as np

from wbr import MODELS_DIR, RESULTS_DIR

G = 9.81
JN = ["left_hip", "left_knee", "left_wheel", "right_hip", "right_knee", "right_wheel"]
OUT_DIR = RESULTS_DIR / "ake_sizing"
UPKIE_MASS = 5.33922            # Plan 4 model, the reference of the LQI6 tuning
UPKIE_R = 0.056
KNEE_REACH_DEG = 60.0           # tire clearance: smallest knee angle checked besides the Plan 4 leg lengths


@dataclass(frozen=True)
class Motor:
    """CubeMars catalogue values (motor side kt, output side torques / speed)."""
    name: str
    peak: float                 # N m, output
    rated: float                # N m, output
    kt_motor: float             # N m / A, motor side
    gear: float
    no_load_rpm: float          # output, at the rated voltage
    resistance: float           # ohm (catalogue phase / line value)
    mass: float                 # kg
    rotor_gcm2: float           # rotor inertia, g cm^2
    backdrive: float            # N m, output
    rated_volt: float = 24.0

    @property
    def kt(self) -> float:
        return self.kt_motor * self.gear

    @property
    def w0(self) -> float:
        return self.no_load_rpm * 2 * math.pi / 60

    @property
    def armature(self) -> float:
        return self.rotor_gcm2 * 1e-7 * self.gear ** 2

    def available(self, tau: float, w: float, volt: float) -> float:
        """Torque available at output speed w for a command of sign(tau): the peak, or the voltage line while motoring."""
        if tau * w > 0:
            ke = self.rated_volt / self.w0
            return min(self.peak, max(0.0, self.kt * (volt - ke * abs(w)) / self.resistance))
        return self.peak


AKE50_8 = Motor("AKE50-8 KV110", peak=7.5, rated=2.5, kt_motor=0.104, gear=8.0, no_load_rpm=320, resistance=0.792,
                mass=0.190, rotor_gcm2=153.81, backdrive=0.09)
AKE60_8 = Motor("AKE60-8 KV80", peak=12.5, rated=5.0, kt_motor=0.13, gear=8.0, no_load_rpm=240, resistance=0.577,
                mass=0.260, rotor_gcm2=647.39, backdrive=0.12)          # backdrive torque not published: assumed


@dataclass
class Case:
    tag: str
    diameter_mm: float = 112.0
    seed: int = 0
    volt: float = 24.0
    limits: bool = True          # False: motor torque / speed limits off (diagnosis)
    torso_kg: float = 2.5        # motors excluded
    battery_kg: float = 1.8      # centred over the hip
    quick: bool = False

    @property
    def radius(self) -> float:
        return self.diameter_mm / 2000.0

    def motor(self, kind: str) -> Motor:
        return AKE60_8 if kind == "wheel" else AKE50_8


STANDARD_CASES = [Case(f"d{d}", diameter_mm=float(d)) for d in (112, 125, 135, 150, 200, 250, 300, 400, 450, 500, 600)]
STANDARD_CASES += [Case(f"d{d}_seed1", diameter_mm=float(d), seed=1) for d in (112, 150, 400)]
STANDARD_CASES += [Case(f"d{d}_v22", diameter_mm=float(d), volt=22.2) for d in (125, 150)]     # 6S LiPo nominal


# ----------------------------------------------------------------------------- model
def build_model(case: Case, mdir: Path):
    """Plan 4 upkie model with the user's masses and the AKE motors, in `mdir` (scene.xml, upkie.xml, plan4_params.json)."""
    import wbr.models.build_upkie as bu
    from wbr.models.upkie_params import build_params
    R = case.radius
    mesh_abs = str(MODELS_DIR / "upkie_description").replace("\\", "/")
    orig = bu.patch_urdf
    bu.patch_urdf = lambda text, meshdir: orig(text, mesh_abs)       # absolute mesh path: the model lives outside models/
    try:
        bu.build(mdir, verbose=False, tire_radius=R, tire_shape="ellipsoid", tire_condim=4)
    finally:
        bu.patch_urdf = orig
    spec = mujoco.MjSpec.from_file(str(mdir / "upkie.xml"))
    spec.meshdir = mesh_abs
    B = {b.name: b for b in spec.bodies}

    def set_mass(name: str, mass: float) -> None:
        b = B[name]
        ratio = mass / b.mass
        b.mass = mass
        b.inertia = [float(x) * ratio for x in b.inertia]

    # torso: battery centred over the hip, Raspberry Pi + printed frame in the torso body -> torso COM on the hip line
    B["battery"].pos = [0.0, float(B["battery"].pos[1]), float(B["battery"].pos[2])]
    set_mass("battery", case.battery_kg)
    set_mass("torso", case.torso_kg - case.battery_kg - (B["base"].mass + B["imu"].mass + B["handle"].mass))
    for side in ("left", "right"):
        set_mass(f"{side}_hip_qdd100_stator", AKE50_8.mass - B[f"{side}_hip_qdd100_rotor"].mass)
        set_mass(f"{side}_knee_qdd100_stator", AKE50_8.mass - B[f"{side}_knee_qdd100_rotor"].mass)
        set_mass(f"{side}_ankle_mj5208_stator", AKE60_8.mass - B[f"{side}_ankle_mj5208_rotor"].mass)
        tire = B[f"{side}_wheel_tire"]                  # same tire mass for every diameter, inertia ~ R^2
        tire.inertia = [float(x) * (R / UPKIE_R) ** 2 for x in tire.inertia]
        for g in tire.geoms:                            # tire touches only the floor (see module doc)
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
    (mdir / "upkie.xml").write_text(spec.to_xml(), encoding="utf-8")
    scene = (mdir / "scene.xml").read_text(encoding="utf-8")
    (mdir / "scene.xml").write_text(scene.replace('<geom name="floor"', '<geom name="floor" contype="1" conaffinity="3"', 1),
                                    encoding="utf-8")
    params = build_params(mdir / "scene.xml", verbose=False)
    k_m = params.extra["total_mass"] / UPKIE_MASS
    params.tau_wheel_max = 1.7 * k_m * R / UPKIE_R             # Plan 4 tuning scaled: the validated closed loop
    params.tau_hip_max = 16.0 * k_m
    params.to_json(mdir / "plan4_params.json")
    return params


def tire_clearance(diameter_mm: float, samples: int = 60) -> dict:
    """Smallest gap [mm] between the left tire and each left-leg part over the knee range, from the MuJoCo collision
    shapes (mj_geomDistance, which ignores the tire-floor-only contact filter of build_model; negative = overlap, capped
    at 100 mm), for the Plan 4 leg lengths and for knee angles >= KNEE_REACH_DEG (covers every balanced posture of the
    user's robot, 62-180 deg at the hip limit), plus every part's lateral span |y| [mm]."""
    from wbr.models.upkie_params import L_MAX, L_MIN
    with tempfile.TemporaryDirectory() as tmp:
        build_model(Case("clearance", diameter_mm), Path(tmp) / "model")
        m = mujoco.MjModel.from_xml_path(str(Path(tmp) / "model" / "scene.xml"))
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key("stand").id)
    mujoco.mj_kinematics(m, d)
    tire = m.geom("left_tire").id
    hip, knee = m.joint("left_hip").id, m.joint("left_knee")

    def span(g):
        R = d.geom_xmat[g].reshape(3, 3)
        c, h = d.geom_xpos[g] + R @ m.geom_aabb[g, :3], np.abs(R) @ m.geom_aabb[g, 3:]
        return [1000 * (abs(c[1]) - h[1]), 1000 * (abs(c[1]) + h[1])]

    parts: Dict[int, str] = {}
    out = {"diameter_mm": diameter_mm, "tire_span_mm": span(tire), "parts": {}}
    for g in range(m.ngeom):
        body = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) or ""
        if g != tire and (m.geom_contype[g] or m.geom_conaffinity[g]) and body.startswith("left_") and "wheel" not in body:
            parts[g] = name = body[len("left_"):]
            p = out["parts"].setdefault(name, {"span_mm": span(g), "plan4": None, "reach": None})
            s = span(g)
            p["span_mm"] = [min(p["span_mm"][0], s[0]), max(p["span_mm"][1], s[1])]
    fromto = np.zeros(6)
    for q in np.linspace(0.0, m.jnt_range[knee.id][0], samples):
        d.qpos[knee.qposadr[0]] = q
        mujoco.mj_kinematics(m, d)
        leg = float(np.hypot(*(d.xanchor[hip] - d.xpos[m.geom_bodyid[tire]])[[0, 2]]))
        bands = [b for b, inside in (("plan4", L_MIN <= leg <= L_MAX), ("reach", 180 - abs(math.degrees(q)) >= KNEE_REACH_DEG))
                 if inside]
        for g, name in parts.items():
            gap = 1000 * mujoco.mj_geomDistance(m, d, tire, g, 0.1, fromto)
            p = out["parts"][name]
            for band in bands:
                if p[band] is None or gap < p[band]:
                    p[band] = gap
    return out


@contextlib.contextmanager
def patched_plan4(case: Case, root: Path):
    import wbr.plans.plan4_upkie_lqi6 as p4
    saved = p4.MODELS_DIR, p4.WHEEL_RADIUS, p4.TAU_WHEEL_MAX, p4.TAU_LEG_MAX
    try:
        p4.MODELS_DIR, p4.WHEEL_RADIUS = root, case.radius
        p4.TAU_WHEEL_MAX, p4.TAU_LEG_MAX = (AKE60_8.peak, AKE50_8.peak) if case.limits else (1e4, 1e4)
        yield p4
    finally:
        p4.MODELS_DIR, p4.WHEEL_RADIUS, p4.TAU_WHEEL_MAX, p4.TAU_LEG_MAX = saved


def available(case: Case, kind: str, tau: float, w: float) -> float:
    return case.motor(kind).available(tau, w, case.volt) if case.limits else 1e4


def make_plan(p4, case: Case, model_dir: str, params_json: Path, slope: float = 0.0):
    cfg = p4.Plan4Config(model_dir=model_dir, params_json=str(params_json), latency_ms=2.0, noise=True, seed=case.seed,
                         slope_deg=slope, friction_comp_Nm=0.8 * AKE50_8.backdrive,
                         friction_comp_wheel_Nm=0.8 * AKE60_8.backdrive)
    plan = p4.Plan4UpkieLQI6(cfg)
    sm = plan.sim.servo_model
    servo_torque = sm.torque

    def limited(name, cmd, q, qd):                      # torque-speed limit on top of the servo model
        tau = servo_torque(name, cmd, q, qd)
        lim = available(case, name.split("_")[1], tau, qd)
        return max(-lim, min(lim, tau))

    sm.torque = limited
    plan.slope = math.radians(slope)
    plan.down, plan.down_why, plan.rec = False, "", []
    aid = [plan.sim.act_id[j] for j in JN]
    dof = [plan.sim.dof_adr[j] for j in JN]
    m = plan.sim.model
    tires = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"{s}_tire") for s in ("left", "right")]
    tire_body = [int(m.geom_bodyid[g]) for g in tires]
    ca, sa = math.cos(plan.slope), math.sin(plan.slope)
    f6, vel = np.zeros(6), np.zeros(6)
    step = plan.step

    def wheel_contact(d):
        """Normal force on each tire, and slip = wheel-centre speed along the floor - rim speed (rolling: 0)."""
        fn = [0.0, 0.0]
        for c in range(d.ncon):
            con = d.contact[c]
            for i, g in enumerate(tires):
                if g in (con.geom1, con.geom2):
                    mujoco.mj_contactForce(m, d, c, f6)
                    fn[i] += f6[0]
        slip = []
        for b in tire_body:
            mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, b, vel, 0)       # world frame: angular, linear
            slip.append(vel[3] * ca + vel[5] * sa - vel[1] * case.radius)
        return fn + slip

    def recording_step(v=0.0, w=0.0):
        o = step(v, w)
        d = plan.sim.data
        plan.rec.append(np.r_[d.time, d.actuator_force[aid], d.qvel[dof], plan.observation["sim"]["base"]["linear_velocity"][0],
                              wheel_contact(d)])
        return o

    plan.step = recording_step
    return plan


def is_down(plan) -> bool:
    """Fallen: true pitch over 1 rad or the base below half its standing height above the (possibly inclined) floor."""
    from wbr.common.rotations import quat_to_matrix, roll_pitch_yaw
    b = plan.observation["sim"]["base"]
    pitch = roll_pitch_yaw(quat_to_matrix(b["orientation"]))[1]
    h = (b["position"][2] - b["position"][0] * math.tan(plan.slope)) * math.cos(plan.slope)
    if abs(pitch) > 1.0 or h < 0.5 * plan.standing_base_z:
        plan.down_why = f"t={plan.sim.time:.2f} s, pitch {math.degrees(pitch):.0f} deg, height {h:.3f} m"
        return True
    return False


def run_for(plan, seconds: float, v: float = 0.0, w: float = 0.0) -> bool:
    for _ in range(int(round(seconds / plan.dt))):
        plan.step(v, w)
        if is_down(plan):
            plan.down = True
            return False
    return True


def fresh(plan, dh: float = 0.0, pitch: float = 0.0) -> None:
    plan.set_height_offset(dh)
    plan.reset(pitch=pitch)
    plan.mcu.L_ref.value = plan.mcu.p.L_stand + dh
    plan.down, plan.down_why, plan.rec = False, "", []


def stats(plan, case: Case, t_from: float = 0.0, steady_from: float | None = None, v_window=None) -> dict:
    Rr = np.array(plan.rec)
    t = Rr[:, 0] - Rr[0, 0]
    sel = t >= t_from
    if not sel.any():                                   # fell before the analysed window: use the whole run
        sel = np.ones_like(t, dtype=bool)
    tau, w = Rr[sel, 1:7], Rr[sel, 7:13]
    pt = np.asarray(plan.log["pitch_true"])
    n = int(round(1.0 / plan.dt))
    out = dict(alive=not plan.down, recovered=bool(not plan.down and np.max(np.abs(pt[-n:])) < math.radians(3)),
               pitch_peak_deg=float(math.degrees(np.max(np.abs(pt)))))
    if plan.down:
        out["down_why"] = plan.down_why
    for kind, cols in (("hip", [0, 3]), ("knee", [1, 4]), ("wheel", [2, 5])):
        T, W = tau[:, cols], w[:, cols]
        lim = np.array([[available(case, kind, a, b) for a, b in zip(rt, rw)] for rt, rw in zip(T, W)])
        out[f"{kind}_peak"] = float(np.abs(T).max())
        out[f"{kind}_rms"] = float(np.sqrt((T ** 2).mean(axis=0)).max())
        out[f"{kind}_speed_peak"] = float(np.abs(W).max())
        out[f"{kind}_sat_pct"] = float(100 * np.mean(np.abs(T) >= 0.98 * lim))
    s = sel & (t >= 0.1)                                # skip the drop onto the floor after a reset
    if not s.any():
        s = sel
    out["wheel_air_ms"] = float(1000 * plan.dt * (Rr[s, 14:16] < 1.0).sum(axis=0).max())       # tire normal force < 1 N
    out["wheel_slip_max"] = float(np.abs(Rr[s, 16:18]).max())
    if steady_from is not None:
        s = t >= steady_from
        out["steady_mean"] = {JN[i]: float(Rr[s, 1 + i].mean()) for i in range(6)} if s.any() else None
    if v_window is not None:
        s = (t >= v_window[0]) & (t < v_window[1])
        out["v_mean"] = float(Rr[s, 13].mean()) if s.any() else None
    return out


# ----------------------------------------------------------------------------- scenarios
def run_case(case: Case) -> dict:
    t0 = time.perf_counter()
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        params = build_model(case, root / "model")
        with patched_plan4(case, root) as p4:
            plan = make_plan(p4, case, "model", root / "model" / "plan4_params.json")
            q = case.quick
            S: Dict[str, dict] = {}
            Ls, Lmin, Lmax = plan.mcu.p.L_stand, plan.mcu.p.L_min, plan.mcu.p.L_max
            for L0 in ([Lmin, Ls] if q else [Lmin, 0.27, Ls, Lmax]):
                fresh(plan, dh=L0 - Ls); run_for(plan, 5.0); S[f"stand_L{L0:.3f}"] = stats(plan, case, 0.0, 2.0)
            for deg in ([10, 20] if q else [5, 10, 15, 20, 25]):
                fresh(plan, pitch=math.radians(deg)); run_for(plan, 6.0); S[f"tilt_{deg}"] = stats(plan, case)
            for frac in ([0.5, 1.0] if q else [0.25, 0.5, 0.75, 1.0, 1.5]):           # 0.1 s push, fraction of the weight
                fresh(plan); run_for(plan, 1.0); plan.push(frac * plan.sim.total_mass() * G, 0.1); run_for(plan, 6.0)
                S[f"push_{frac}"] = stats(plan, case, 1.0)
            for V in ([0.8] if q else [0.5, 0.8, 1.0]):
                fresh(plan); run_for(plan, 0.5) and run_for(plan, 3.0, V) and run_for(plan, 3.0)
                S[f"velocity_{V}"] = stats(plan, case, v_window=(2.5, 3.5))
            fresh(plan); run_for(plan, 0.5) and run_for(plan, 2.5, 0.0, 1.0); S["yaw_1.0"] = stats(plan, case)
            fresh(plan); run_for(plan, 1.0)
            for L0 in (Lmin, Lmax, Ls):                                                  # full leg-length range
                plan.set_height_offset(L0 - Ls)
                if not run_for(plan, 2.0):
                    break
            S["height_full_range"] = stats(plan, case)
            for slope in ([10] if q else [5, 10, 15]):
                sp = make_plan(p4, case, "model", root / "model" / "plan4_params.json", slope)
                fresh(sp); run_for(sp, 6.0); S[f"slope_{slope}_stand"] = stats(sp, case, 0.0, 3.0)
                for name, V in (("up", 0.5), ("down", -0.5)):
                    fresh(sp); run_for(sp, 2.0) and run_for(sp, 4.0, V) and run_for(sp, 2.0)
                    S[f"slope_{slope}_{name}_0.5"] = stats(sp, case, 2.0)
            total = plan.sim.total_mass()
    return dict(case=dataclasses.asdict(case), radius=case.radius, total_mass=total,
                plant=dict(m_b=params.m_b, b_x=params.b_x, J_w=params.J_w, tau_budget_wheel=params.tau_wheel_max,
                           tau_budget_hip=params.tau_hip_max),
                motors={"hip_knee": dataclasses.asdict(AKE50_8), "wheel": dataclasses.asdict(AKE60_8)},
                scen=S, wall_s=time.perf_counter() - t0)


def _run(case: Case):
    try:
        return case.tag, run_case(case)
    except Exception as exc:          # keep the other cases running; the error is reported in the summary
        return case.tag, {"error": repr(exc)}


# ----------------------------------------------------------------------------- report
def fmt(v, nd=2):
    return "-" if v is None else f"{v:.{nd}f}"


def _worst(S: dict, key: str):
    name = max(S, key=lambda n: S[n][key])
    return S[name][key], name


def summary_markdown(results: Dict[str, dict]) -> str:
    ok = {t: r for t, r in sorted(results.items(), key=lambda kv: (kv[1].get("radius", 0), kv[0])) if "error" not in r}
    lines = ["# AKE motor check of the user's robot (`python -m wbr.bench.ake_sizing`)", ""]
    for t, r in results.items():
        if "error" in r:
            lines.append(f"- ERROR {t}: {r['error']}")
    lines += ["| motor | peak / rated [N m] | no-load [rpm] (rad/s) | gear | mass [kg] | reflected inertia [kg m^2] | backdrive [N m] |",
              "|---|---|---|---|---|---|---|"]
    for role, m in (("hip, knee", AKE50_8), ("wheel", AKE60_8)):
        lines.append(f"| {m.name} ({role}) | {m.peak} / {m.rated} | {m.no_load_rpm:.0f} ({m.w0:.1f}) | {m.gear:.0f}:1 | {m.mass} | "
                     f"{m.armature:.2e} | {m.backdrive} |")
    lines += ["", "## Per wheel diameter", "",
              "| case | wheel [mm] | seed | V | mass [kg] | not recovered | wheel peak [N m] (scenario) | wheel RMS max | wheel sat % max | "
              "wheel speed max [rad/s] | v at 1.0 m/s cmd | hip peak (scenario) | hip RMS max | knee peak (scenario) | knee static max | knee RMS max | "
              "tire off the floor max [ms] (scenario) | slip max [m/s] |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for t, r in ok.items():
        S, c = r["scen"], r["case"]
        bad = [n for n, s in S.items() if not s["recovered"]]
        wp, wps = _worst(S, "wheel_peak")
        hp, hps = _worst(S, "hip_peak")
        kp, kps = _worst(S, "knee_peak")
        ks = max((abs(s["steady_mean"]["left_knee"]) for n, s in S.items() if n.startswith("stand_L") and s["steady_mean"]),
                 default=None)
        rec = {n: s for n, s in S.items() if s["recovered"]}             # a fall lifts the wheels anyway
        air, airs = _worst(rec, "wheel_air_ms") if rec else (None, "-")
        lines.append(f"| {t} | {c['diameter_mm']:.0f} | {c['seed']} | {c['volt']:.1f} | {r['total_mass']:.3f} | {', '.join(bad) if bad else 'none'} | "
                     f"{fmt(wp)} ({wps}) | {fmt(_worst(S, 'wheel_rms')[0])} | {fmt(_worst(S, 'wheel_sat_pct')[0], 1)} | "
                     f"{fmt(_worst(S, 'wheel_speed_peak')[0], 1)} | {fmt(S['velocity_1.0'].get('v_mean') if 'velocity_1.0' in S else None)} | "
                     f"{fmt(hp)} ({hps}) | {fmt(_worst(S, 'hip_rms')[0])} | {fmt(kp)} ({kps}) | {fmt(ks)} | {fmt(_worst(S, 'knee_rms')[0])} | "
                     f"{fmt(air, 0)}{f' ({airs})' if air else ''} | {fmt(_worst(rec, 'wheel_slip_max')[0] if rec else None)} |")
    fails = [(t, n, s) for t, r in ok.items() for n, s in r["scen"].items() if not s["recovered"]]
    if fails:
        lines += ["", "## Scenarios not recovered", "",
                  "| case | scenario | why | wheel peak / sat % / speed | knee peak / sat % | hip peak / sat % |", "|---|---|---|---|---|---|"]
        for t, n, s in fails:
            lines.append(f"| {t} | {n} | {s.get('down_why') or 'pitch above 3 deg in the last second'} | "
                         f"{fmt(s['wheel_peak'])} / {fmt(s['wheel_sat_pct'], 1)} / {fmt(s['wheel_speed_peak'], 1)} | "
                         f"{fmt(s['knee_peak'])} / {fmt(s['knee_sat_pct'], 1)} | {fmt(s['hip_peak'])} / {fmt(s['hip_sat_pct'], 1)} |")
    return "\n".join(lines) + "\n"


def clearance_markdown(rows: List[dict]) -> str:
    from wbr.models.upkie_params import L_MAX, L_MIN

    def gap(v):
        return "-" if v is None else "≥ 100" if v >= 99.9 else f"**{v:.0f}**" if v < 0 else f"{v:.0f}"

    near = {n for r in rows for n, p in r["parts"].items() if p["reach"] is not None and p["reach"] < 99.9}
    names = sorted(near, key=lambda n: rows[0]["parts"][n]["span_mm"][0])
    t = rows[0]["tire_span_mm"]
    lines = ["# Tire clearance to the leg, upkie layout (`python -m wbr.bench.ake_sizing --clearance`)", "",
             f"Left tire lateral span |y| {t[0]:.0f}..{t[1]:.0f} mm: the wheel is inboard of the shank. Smallest gap [mm] "
             f"between the tire and each part from the MuJoCo collision shapes, Plan 4 leg lengths {L_MIN}..{L_MAX} m / "
             f"knee angle >= {KNEE_REACH_DEG:.0f} deg; **negative = overlap**. The simulation itself lets the tire pass "
             "through the robot (tire-floor contact only).", "",
             "| part | lateral span |y| [mm] | " + " | ".join(f"Ø{r['diameter_mm']:.0f}" for r in rows) + " |",
             "|---|---|" + "---|" * len(rows)]
    for n in names:
        s = rows[0]["parts"][n]["span_mm"]
        cells = [f"{gap(r['parts'][n]['plan4'])} / {gap(r['parts'][n]['reach'])}" for r in rows]
        lines.append(f"| {n} | {s[0]:.0f}..{s[1]:.0f} | " + " | ".join(cells) + " |")
    return "\n".join(lines) + "\n"


def main(argv: List[str] | None = None) -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", nargs="*", help="tags of the standard cases to run")
    ap.add_argument("--tag", help="run one custom case with this tag")
    ap.add_argument("--diameter", type=float, default=112.0, help="wheel diameter [mm] of the custom case")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--volt", type=float, default=24.0)
    ap.add_argument("--no-limits", action="store_true", help="motor torque / speed limits off (diagnosis)")
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--jobs", type=int, default=max(1, cpu_count() - 2))
    ap.add_argument("--out", type=Path, default=OUT_DIR)
    ap.add_argument("--clearance", type=float, nargs="*", help="only report the tire-to-leg gaps for these diameters [mm]")
    args = ap.parse_args(argv)
    if args.clearance is not None:
        rows = [tire_clearance(D) for D in (args.clearance or (112, 150, 200, 240, 250, 260, 300, 400))]
        args.out.mkdir(parents=True, exist_ok=True)
        (args.out / "clearance.md").write_text(clearance_markdown(rows), encoding="utf-8")
        print(clearance_markdown(rows))
        return
    if args.tag:
        cases = [Case(args.tag, args.diameter, args.seed, args.volt, limits=not args.no_limits)]
    else:
        cases = [c for c in STANDARD_CASES if not args.only or c.tag in args.only]
    for c in cases:
        c.quick = args.quick
    args.out.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    with Pool(min(args.jobs, len(cases)), maxtasksperchild=1) as pool:
        for tag, res in pool.imap_unordered(_run, cases):
            (args.out / f"{tag}.json").write_text(json.dumps(res, indent=1), encoding="utf-8")
            print(f"{tag}: {'ERROR ' + res['error'] if 'error' in res else 'done'} ({time.time() - t0:.0f} s)")
    results = {f.stem: json.loads(f.read_text(encoding="utf-8")) for f in sorted(args.out.glob("*.json"))}
    (args.out / "summary.md").write_text(summary_markdown(results), encoding="utf-8")
    print(f"wrote {args.out / 'summary.md'}")


if __name__ == "__main__":
    main()
