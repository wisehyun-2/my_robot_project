"""CubeMars AKE motor model and the scenario runner of the CAD robot's motor check (wbr.bench.ake_cad) and of the staged
tests (wbr.bench.stage_tests).

Motor model: peak torque, the voltage line of the torque-speed curve (no-load speed at the rated 24 V, winding resistance)
while motoring, reflected rotor inertia (rotor inertia x gear^2) as joint armature, backdrive torque as joint friction.
make_plan puts the torque-speed limit on top of the plan's servo model and records, besides the joint torques and speeds,
the tire normal forces and the wheel slip (wheel-centre speed along the floor - rim speed) every step; run_case runs the
scenario set of the motor check (stand, tilt, push, velocity, yaw, height range, slopes) on the robot built by
ake_cad.build_cad_model.  yaw_plant_gain / roll_plant_gain: the plants the outer loops are scaled with (ake_cad.v5_gains).
"""
from __future__ import annotations

import contextlib
import dataclasses
import math
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict

import mujoco
import numpy as np

G = 9.81
JN = ["left_hip", "left_knee", "left_wheel", "right_hip", "right_knee", "right_wheel"]


def yaw_plant_gain(xml_path) -> dict:
    """Yaw plant of the differential wheel torque in the straight-leg keyframe: tau_yaw is split -+ tau_yaw / 2 on the
    wheels, i.e. a yaw moment tau_yaw W / (2 R) on the yaw inertia Izz -> g = W / (2 R Izz)."""
    m = mujoco.MjModel.from_xml_path(str(xml_path))
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "stand"))
    mujoco.mj_forward(m, d)
    mass = float(m.body_mass[1:].sum())
    com = sum(m.body_mass[b] * d.xipos[b] for b in range(1, m.nbody)) / mass
    Izz = 0.0
    for b in range(1, m.nbody):
        Rm = d.ximat[b].reshape(3, 3)
        r = d.xipos[b] - com
        Izz += float((Rm @ np.diag(m.body_inertia[b]) @ Rm.T)[2, 2] + m.body_mass[b] * (r[0] ** 2 + r[1] ** 2))
    gl, gr = (mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"{s}_tire") for s in ("left", "right"))
    W = float(abs(d.geom_xpos[gl][1] - d.geom_xpos[gr][1]))
    R = float(m.geom_size[gl][0])
    return dict(W=W, R=R, Izz=Izz, g=W / (2 * R * Izz))


def roll_plant_gain(xml_path) -> dict:
    """Roll plant of a differential leg force in the straight-leg keyframe: +-F on the legs is a roll moment F W on the roll
    inertia Ixx of the whole robot about its COM -> g = W / Ixx [rad/s^2 per N]."""
    m = mujoco.MjModel.from_xml_path(str(xml_path))
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "stand"))
    mujoco.mj_forward(m, d)
    mass = float(m.body_mass[1:].sum())
    com = sum(m.body_mass[b] * d.xipos[b] for b in range(1, m.nbody)) / mass
    Ixx = 0.0
    for b in range(1, m.nbody):
        Rm = d.ximat[b].reshape(3, 3)
        r = d.xipos[b] - com
        Ixx += float((Rm @ np.diag(m.body_inertia[b]) @ Rm.T)[0, 0] + m.body_mass[b] * (r[1] ** 2 + r[2] ** 2))
    gl, gr = (mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"{s}_tire") for s in ("left", "right"))
    W = float(abs(d.geom_xpos[gl][1] - d.geom_xpos[gr][1]))
    return dict(W=W, Ixx=Ixx, g=W / Ixx)


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
MOTORS = {"AKE50-8": AKE50_8, "AKE60-8": AKE60_8}


@dataclass
class Case:
    tag: str
    diameter_mm: float = 0.0     # wheel diameter: set by the model builder (ake_cad.build_cad_model)
    seed: int = 0
    volt: float = 24.0
    limits: bool = True          # False: motor torque / speed limits off (diagnosis)
    quick: bool = False
    hip_motor: str = "AKE60-8"   # keys of MOTORS
    knee_motor: str = "AKE60-8"
    wheel_motor: str = "AKE60-8"

    @property
    def radius(self) -> float:
        return self.diameter_mm / 2000.0

    def motor(self, kind: str) -> Motor:
        return MOTORS[{"hip": self.hip_motor, "knee": self.knee_motor, "wheel": self.wheel_motor}[kind]]


# ----------------------------------------------------------------------------- model
def build_model(case: Case, mdir: Path):
    """The robot of `case` in `mdir` (scene.xml, description.json, plant_params.json): wbr.bench.ake_cad.build_cad_model."""
    from wbr.bench.ake_cad import build_cad_model
    return build_cad_model(case, mdir)


@contextlib.contextmanager
def patched_runtime(case: Case, root: Path):
    """wbr.plans.closed_loop.MODELS_DIR -> root for the duration: the plan reads its model folder (make_plan's model_dir)
    there."""
    import wbr.plans.closed_loop as cl
    saved = cl.MODELS_DIR
    try:
        cl.MODELS_DIR = root
        yield cl
    finally:
        cl.MODELS_DIR = saved


def available(case: Case, kind: str, tau: float, w: float) -> float:
    return case.motor(kind).available(tau, w, case.volt) if case.limits else 1e4


def make_plan(cl, case: Case, model_dir: str, params_json: Path, slope: float = 0.0):
    kw = dict(model_dir=model_dir, params_json=str(params_json), latency_ms=2.0, noise=True, seed=case.seed,
              slope_deg=slope, friction_comp_Nm=0.8 * case.motor("hip").backdrive,
              friction_comp_wheel_Nm=0.8 * case.motor("wheel").backdrive)
    # the robot's gains (ake_cad.v5_gains); the staged tests (v5_lqi/TEST, v5_mpc/TEST) also set lqi_kwargs / mpc_kwargs, the
    # latency and the disturbances here - an override replaces the default above
    kw.update(getattr(case, "plan_overrides", None) or {})
    cfg_cls, plan_cls = case.plan_classes()         # the CAD robot with its own URDF: wbr.plans.plan_v5 (ake_cad.CadCase)
    plan = plan_cls(cfg_cls(**kw))
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


def knee_flipped(plan) -> bool:
    """True when a knee ends on the other side of full extension than the designed posture: the leg went through the
    singular straight pose and came back bent the other way.  The robot may keep standing like that (tilted: the
    controller's plant has the knee on the designed side), so the pitch criterion alone can miss it (11-05: the 5 kg
    case stopping on the 15 deg slope, pitch still below 3 deg when the 2 s window ended)."""
    m, d = plan.sim.model, plan.sim.data
    side = -plan.leg.g.knee_sign                        # sign of the knee angle (u-convention) in the designed posture
    for name in ("left_knee", "right_knee"):
        q = d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)]] * plan.signs[name]
        if q * side < 0.0:
            return True
    return False


def stats(plan, case: Case, t_from: float = 0.0, steady_from: float | None = None, v_window=None) -> dict:
    Rr = np.array(plan.rec)
    t = Rr[:, 0] - Rr[0, 0]
    sel = t >= t_from
    if not sel.any():                                   # fell before the analysed window: use the whole run
        sel = np.ones_like(t, dtype=bool)
    tau, w = Rr[sel, 1:7], Rr[sel, 7:13]
    pt = np.asarray(plan.log["pitch_true"])
    n = int(round(1.0 / plan.dt))
    flipped = knee_flipped(plan)
    out = dict(alive=not plan.down,
               recovered=bool(not plan.down and not flipped and np.max(np.abs(pt[-n:])) < math.radians(3)),
               pitch_peak_deg=float(math.degrees(np.max(np.abs(pt)))), knee_flipped=flipped,
               # longest mean leg length of the run and what is left to full extension (L1 + L2, singular)
               leg_max_mm=float(1000 * np.max(plan.log["L0"])),
               leg_margin_mm=float(1000 * (plan.leg.g.L0_max - np.max(plan.log["L0"]))))
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
        with patched_runtime(case, root) as cl:
            params_json = root / "model" / case.params_file
            plan = make_plan(cl, case, "model", params_json)
            q = case.quick
            S: Dict[str, dict] = {}
            Ls, Lmin, Lmax = plan.mcu.p.L_stand, plan.mcu.p.L_min, plan.mcu.p.L_max
            mid = 0.27 if Lmin < 0.27 < Ls else 0.5 * (Lmin + Ls)       # (a robot with longer links: 0.27 m is below its range)
            for L0 in ([Lmin, Ls] if q else [Lmin, mid, Ls, Lmax]):
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
                sp = make_plan(cl, case, "model", params_json, slope)
                fresh(sp); run_for(sp, 6.0); S[f"slope_{slope}_stand"] = stats(sp, case, 0.0, 3.0)
                for name, V in (("up", 0.5), ("down", -0.5)):
                    fresh(sp); run_for(sp, 2.0) and run_for(sp, 4.0, V) and run_for(sp, 2.0)
                    S[f"slope_{slope}_{name}_0.5"] = stats(sp, case, 2.0)
            total = plan.sim.total_mass()
    return dict(case=dataclasses.asdict(case), radius=case.radius, total_mass=total,
                plant=dict(m_b=params.m_b, b_x=params.b_x, J_w=params.J_w, tau_budget_wheel=params.tau_wheel_max,
                           tau_budget_hip=params.tau_hip_max),
                motors={k: dataclasses.asdict(case.motor(k)) for k in ("hip", "knee", "wheel")},
                scen=S, wall_s=time.perf_counter() - t0)


# ----------------------------------------------------------------------------- report
def fmt(v, nd=2):
    return "-" if v is None else f"{v:.{nd}f}"


def _worst(S: dict, key: str):
    name = max(S, key=lambda n: S[n][key])
    return S[name][key], name


def summary_markdown(results: Dict[str, dict]) -> str:
    ok = {t: r for t, r in sorted(results.items(), key=lambda kv: (kv[1].get("radius", 0), kv[0])) if "error" not in r}
    lines = ["# AKE motor check of the CAD robot (`python -m wbr.bench.ake_cad`)", ""]
    for t, r in results.items():
        if "error" in r:
            lines.append(f"- ERROR {t}: {r['error']}")
    lines += ["| motor | peak / rated [N m] | no-load [rpm] (rad/s) | gear | mass [kg] | reflected inertia [kg m^2] | backdrive [N m] |",
              "|---|---|---|---|---|---|---|"]
    for m in MOTORS.values():
        lines.append(f"| {m.name} | {m.peak} / {m.rated} | {m.no_load_rpm:.0f} ({m.w0:.1f}) | {m.gear:.0f}:1 | {m.mass} | "
                     f"{m.armature:.2e} | {m.backdrive} |")
    lines += ["", "## Per wheel diameter", "",
              "| case | hip / knee / wheel motor | wheel [mm] | seed | V | mass [kg] | not recovered | wheel peak [N m] (scenario) | wheel RMS max | wheel sat % max | "
              "wheel speed max [rad/s] | v at 1.0 m/s cmd | hip peak (scenario) | hip RMS max | knee peak (scenario) | knee static max | knee RMS max | "
              "tire off the floor max [ms] (scenario) | slip max [m/s] | longest leg [mm] (scenario) |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
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
        mot = " / ".join(c.get(f"{k}_motor", d) for k, d in (("hip", "AKE50-8"), ("knee", "AKE50-8"), ("wheel", "AKE60-8")))
        legs = {n: s["leg_max_mm"] for n, s in S.items() if s.get("leg_max_mm") is not None}    # (results before 11-05: none)
        leg = "-" if not legs else "%.0f (%s)" % (max(legs.values()), max(legs, key=legs.get))
        lines.append(f"| {t} | {mot} | {c['diameter_mm']:.0f} | {c['seed']} | {c['volt']:.1f} | {r['total_mass']:.3f} | {', '.join(bad) if bad else 'none'} | "
                     f"{fmt(wp)} ({wps}) | {fmt(_worst(S, 'wheel_rms')[0])} | {fmt(_worst(S, 'wheel_sat_pct')[0], 1)} | "
                     f"{fmt(_worst(S, 'wheel_speed_peak')[0], 1)} | {fmt(S['velocity_1.0'].get('v_mean') if 'velocity_1.0' in S else None)} | "
                     f"{fmt(hp)} ({hps}) | {fmt(_worst(S, 'hip_rms')[0])} | {fmt(kp)} ({kps}) | {fmt(ks)} | {fmt(_worst(S, 'knee_rms')[0])} | "
                     f"{fmt(air, 0)}{f' ({airs})' if air else ''} | {fmt(_worst(rec, 'wheel_slip_max')[0] if rec else None)} | "
                     f"{leg} |")
    fails = [(t, n, s) for t, r in ok.items() for n, s in r["scen"].items() if not s["recovered"]]
    if fails:
        lines += ["", "## Scenarios not recovered", "",
                  "| case | scenario | why | wheel peak / sat % / speed | knee peak / sat % | hip peak / sat % |", "|---|---|---|---|---|---|"]
        for t, n, s in fails:
            why = s.get("down_why") or ("knee flipped through full extension" if s.get("knee_flipped")
                                        else "pitch above 3 deg in the last second")
            lines.append(f"| {t} | {n} | {why} | "
                         f"{fmt(s['wheel_peak'])} / {fmt(s['wheel_sat_pct'], 1)} / {fmt(s['wheel_speed_peak'], 1)} | "
                         f"{fmt(s['knee_peak'])} / {fmt(s['knee_sat_pct'], 1)} | {fmt(s['hip_peak'])} / {fmt(s['hip_sat_pct'], 1)} |")
    return "\n".join(lines) + "\n"
