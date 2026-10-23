"""Motor torque sizing of a mass-scaled Plan 2 robot (DIABLO-lite five-bar + LQI6), docs/plan2_15kg_torque_sizing.md.

The DIABLO-lite model is rebuilt in a temporary folder with the same geometry and every mass, rotor
inertia and joint friction multiplied by k = mass / 6 kg.  The Plan 2 stack (1 kHz MCU, LQI6, IMU/encoder
noise + 2 ms latency) runs a scenario battery and the true actuator torques and joint speeds of all six
motors are recorded.  The force-like MCU gains (leg PD, anti-split, yaw PI) are scaled by k; the LQI6
weights are normalised to the design torque budget (default 12 k / 5 k N m, i.e. the 6 kg tuning), so a
pure mass scaling reproduces the 6 kg closed loop and every torque scales by k.

    python -m wbr.bench.torque_sizing                    # standard cases -> results/torque_sizing/*.json + summary.md
    python -m wbr.bench.torque_sizing --only demand_15kg  # a subset of the standard cases
    python -m wbr.bench.torque_sizing --mass 15 --clamp-hip 20 --clamp-wheel 8 --tag my_case   # one custom case

clamp 0 = actuators unclamped (measures the demand); clamp > 0 = the motor limit to verify.  Plan 2 itself
is not modified: the model parameters and the model folder are patched for the duration of one case.
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

import numpy as np

from wbr import RESULTS_DIR

G = 9.81
JN = ["left_hip", "left_crank", "left_wheel", "right_hip", "right_crank", "right_wheel"]
HIP = [0, 1, 3, 4]
WHEEL = [2, 5]
SCALED_FIELDS = ("m_torso", "m_motor", "m_thigh", "m_shank", "m_crank", "m_rod", "m_wheel",
                 "armature_hip", "armature_wheel", "friction_hip", "friction_wheel")
SCALED_GAINS = ("kp_leg", "kd_leg", "kp_split", "kd_split", "kp_yaw", "ki_yaw")
OUT_DIR = RESULTS_DIR / "torque_sizing"


@dataclass
class Case:
    tag: str
    mass: float = 15.0
    seed: int = 0
    design_hip: float = 0.0       # 0: 12 k N m (6 kg tuning scaled)
    design_wheel: float = 0.0     # 0: 5 k N m
    clamp_hip: float = 0.0        # 0: unclamped
    clamp_wheel: float = 0.0
    wheel_radius: float = 0.065
    quick: bool = False


STANDARD_CASES = [
    Case("demand_6kg", mass=6.0),
    Case("demand_15kg"),
    Case("demand_15kg_seed1", seed=1),
    Case("clamp_15kg_12_5", clamp_hip=12.0, clamp_wheel=5.0),
    Case("clamp_15kg_15_6", clamp_hip=15.0, clamp_wheel=6.0),
    Case("clamp_15kg_20_8", clamp_hip=20.0, clamp_wheel=8.0),
    Case("clamp_15kg_design20_8", design_hip=20.0, design_wheel=8.0, clamp_hip=20.0, clamp_wheel=8.0),
]


# ----------------------------------------------------------------------------- model / plan
def scaled_params_class(case: Case):
    """DiabloLiteParams subclass whose defaults are the mass-scaled robot (Plan 2 instantiates it without arguments)."""
    from wbr.models.build_diablo_lite import DiabloLiteParams
    b0 = DiabloLiteParams()
    k = case.mass / b0.total_mass
    defaults = {n: getattr(b0, n) * k for n in SCALED_FIELDS}
    defaults.update(wheel_radius=case.wheel_radius, tau_hip=case.design_hip or b0.tau_hip * k,
                    tau_wheel=case.design_wheel or b0.tau_wheel * k)
    return dataclasses.make_dataclass("ScaledDiabloLiteParams", [(n, float, v) for n, v in defaults.items()],
                                      bases=(DiabloLiteParams,)), k


@contextlib.contextmanager
def scaled_plan2(case: Case):
    """Patch the DIABLO-lite parameters and the model folder seen by Plan 2 for the duration of one case."""
    import wbr.models.build_diablo_lite as bdl
    import wbr.plans.plan2_diablo_lqi6 as p2
    cls, k = scaled_params_class(case)
    saved = bdl.DiabloLiteParams, p2.MODELS_DIR
    with tempfile.TemporaryDirectory() as tmp:
        try:
            bdl.build(Path(tmp) / "diablo_lite", cls(), verbose=False)
            bdl.DiabloLiteParams, p2.MODELS_DIR = cls, Path(tmp)
            yield k
        finally:
            bdl.DiabloLiteParams, p2.MODELS_DIR = saved


def make_plan(case: Case, k: float, slope_deg: float = 0.0):
    import mujoco
    from wbr.plans.plan2_diablo_lqi6 import Plan2Config, Plan2DiabloLQI6
    plan = Plan2DiabloLQI6(Plan2Config(latency_ms=2.0, noise=True, seed=case.seed))
    mp = plan.mcu.p
    for name in SCALED_GAINS:
        setattr(mp, name, getattr(mp, name) * k)
    ch, cw = case.clamp_hip or 1e4, case.clamp_wheel or 1e4
    mp.tau_hip_max, mp.tau_wheel_max = ch, cw
    m = plan.sim.model
    for j in JN:
        lim = cw if "wheel" in j else ch
        plan.sim.servo_model.props[j].max_torque = lim
        m.actuator_ctrlrange[plan.sim.act_id[j]] = [-lim, lim]
    if slope_deg:                         # floor pitched about +y, rising with +x (as in Plan 4)
        a = -math.radians(slope_deg)
        gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        m.geom_quat[gid] = [math.cos(a / 2), 0.0, math.sin(a / 2), 0.0]
        m.geom_sameframe[gid] = 0
    plan.slope = math.radians(slope_deg)
    plan.down = False
    plan.rec = []
    aid = [plan.sim.act_id[j] for j in JN]
    dof = [plan.sim.dof_adr[j] for j in JN]
    step = plan.step

    def recording_step(v=0.0, w=0.0):
        o = step(v, w)
        d = plan.sim.data
        plan.rec.append(np.r_[d.time, d.actuator_force[aid], d.qvel[dof]])
        return o

    plan.step = recording_step
    return plan


def is_down(plan) -> bool:
    """Plan 2's own check uses the world z of the base, which is wrong on a slope: use the height above the floor."""
    base = plan.observation["sim"]["base"]["position"]
    return abs(plan.mcu.est.pitch) > 1.0 or (base[2] - base[0] * math.tan(plan.slope)) * math.cos(plan.slope) < 0.08


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
    plan.mcu.L_ref.value = plan.mcu.p.L_stand + dh     # start the leg-length ramp at the posture the robot is placed in
    plan.down = False
    plan.rec = []


def stats(plan, t_from: float = 0.0, steady_from: float | None = None) -> dict:
    R = np.array(plan.rec)
    t = R[:, 0] - R[0, 0]
    sel = t >= t_from
    tau, w = R[sel, 1:7], R[sel, 7:13]
    power = np.abs(tau * w)
    L = plan.log
    n = int(round(1.0 / plan.dt))
    out = dict(alive=not plan.down,
               recovered=bool(not plan.down and np.max(np.abs(L["pitch"][-n:])) < math.radians(3)),
               pitch_peak_deg=float(math.degrees(np.max(np.abs(L["pitch"])))),
               hip_peak=float(np.abs(tau[:, HIP]).max()), wheel_peak=float(np.abs(tau[:, WHEEL]).max()),
               hip_rms=float(np.sqrt((tau[:, HIP] ** 2).mean(axis=0)).max()),
               wheel_rms=float(np.sqrt((tau[:, WHEEL] ** 2).mean(axis=0)).max()),
               hip_speed_peak=float(np.abs(w[:, HIP]).max()), wheel_speed_peak=float(np.abs(w[:, WHEEL]).max()),
               hip_power_peak=float(power[:, HIP].max()), wheel_power_peak=float(power[:, WHEEL].max()),
               per_motor_peak={JN[i]: float(np.abs(tau[:, i]).max()) for i in range(6)},
               per_motor_rms={JN[i]: float(np.sqrt((tau[:, i] ** 2).mean())) for i in range(6)})
    if steady_from is not None:
        s = t >= steady_from
        out["steady_mean"] = {JN[i]: float(R[s, 1 + i].mean()) for i in range(6)}
    return out


# ----------------------------------------------------------------------------- scenarios
def run_case(case: Case) -> dict:
    t0 = time.perf_counter()
    with scaled_plan2(case) as k:
        plan = make_plan(case, k)
        q = case.quick
        S: Dict[str, dict] = {}
        L_stand = plan.mcu.p.L_stand
        # static postures over the leg-length range (holding torque = steady mean)
        for L0 in ([0.12, 0.198, 0.26] if q else [0.12, 0.14, 0.16, 0.18, 0.198, 0.22, 0.24, 0.26]):
            fresh(plan, dh=L0 - L_stand)
            run_for(plan, 5.0)
            S[f"stand_L{L0:.3f}"] = stats(plan, 0.0, steady_from=2.0)
        for deg in ([10, 20] if q else [5, 10, 15, 20, 25]):
            fresh(plan, pitch=math.radians(deg))
            run_for(plan, 6.0)
            S[f"tilt_{deg}"] = stats(plan)
        for frac in ([0.5, 1.0] if q else [0.25, 0.5, 0.75, 1.0, 1.5, 2.0]):     # 0.1 s push, fraction of the weight
            fresh(plan)
            run_for(plan, 1.0)
            plan.push(frac * plan.sim.total_mass() * G, 0.1)
            run_for(plan, 6.0)
            S[f"push_{frac}"] = stats(plan, 1.0)
        for V in ([0.8] if q else [0.5, 0.8, 1.0]):
            fresh(plan)
            run_for(plan, 0.5) and run_for(plan, 3.0, V) and run_for(plan, 3.0)
            S[f"velocity_{V}"] = stats(plan)
        for W in ([1.0] if q else [1.0, 2.0]):
            fresh(plan)
            run_for(plan, 0.5) and run_for(plan, 2.5, 0.0, W)
            S[f"yaw_{W}"] = stats(plan)
        # full leg-length range while balancing: stand -> lowest -> highest -> stand (leg ramp 0.25 m/s)
        fresh(plan)
        run_for(plan, 1.0)
        for L0 in (0.12, 0.26, L_stand):
            plan.set_height_offset(L0 - L_stand)
            if not run_for(plan, 2.0):
                break
        S["height_full_range"] = stats(plan)
        for L0 in (0.14, 0.24):                                                    # driving crouched / tall
            fresh(plan, dh=L0 - L_stand)
            run_for(plan, 1.0) and run_for(plan, 3.0, 0.8) and run_for(plan, 3.0)
            S[f"velocity_0.8_L{L0:.2f}"] = stats(plan)
        for slope in ([10] if q else [5, 10, 15]):
            sp = make_plan(case, k, slope)
            fresh(sp)
            run_for(sp, 6.0)
            S[f"slope_{slope}_stand"] = stats(sp, 0.0, steady_from=3.0)
            for name, V in (("up", 0.5), ("down", -0.5)):
                fresh(sp)
                run_for(sp, 2.0) and run_for(sp, 4.0, V) and run_for(sp, 2.0)
                S[f"slope_{slope}_{name}_0.5"] = stats(sp, 2.0)
        # analytic static check: VMC torques for the per-leg weight (wheels excluded) at the equilibrium leg angle
        F_leg = 0.5 * (plan.sim.total_mass() - 2 * plan.params.m_wheel) * G
        analytic = {}
        for L0 in np.linspace(0.12, 0.26, 8):
            th = plan.theta_eq(float(L0))
            qh, qc = plan.leg.inverse(float(L0), th)
            analytic[f"{L0:.3f}"] = plan.leg.vmc_torques(F_leg / math.cos(th), 0.0, qh, qc)
        res = dict(case=dataclasses.asdict(case), k=k, total_mass=plan.sim.total_mass(), params=dataclasses.asdict(plan.params),
                   design_hip=plan.params.tau_hip, design_wheel=plan.params.tau_wheel, scen=S,
                   analytic_static_hip_crank=analytic)
    res["wall_s"] = time.perf_counter() - t0
    return res


def _run(case: Case):
    try:
        return case.tag, run_case(case)
    except Exception as exc:          # keep the other cases running; the error is reported in the summary
        return case.tag, {"error": repr(exc)}


# ----------------------------------------------------------------------------- report
def fmt(v, nd=2):
    return "-" if v is None else f"{v:.{nd}f}"


def envelope(res: dict) -> dict:
    """Worst case over the scenarios the robot recovers from (a scenario that fails unclamped is a controller limit)."""
    S = {n: s for n, s in res["scen"].items() if s["recovered"]}
    out = {}
    for key in ("hip_peak", "wheel_peak", "hip_speed_peak", "wheel_speed_peak", "hip_power_peak", "wheel_power_peak"):
        name = max(S, key=lambda n: S[n][key])
        out[key] = (S[name][key], name)
    out["excluded"] = sorted(set(res["scen"]) - set(S))
    return out


def summary_markdown(results: Dict[str, dict]) -> str:
    ok = {t: r for t, r in results.items() if "error" not in r}
    lines = ["# Plan 2 mass-scaled torque sizing (`python -m wbr.bench.torque_sizing`)", ""]
    for t, r in results.items():
        if "error" in r:
            lines.append(f"- ERROR {t}: {r['error']}")
    demand = [t for t, r in ok.items() if not r["case"]["clamp_hip"] and not r["case"]["clamp_wheel"]]
    for t in demand:
        r = ok[t]
        lines += ["", f"## Demand: {t} ({r['total_mass']:.1f} kg, LQI6 budget {r['design_hip']:.1f} / {r['design_wheel']:.2f} N m, "
                      f"seed {r['case']['seed']}, unclamped)", "",
                  "| scenario | recovered | pitch peak [deg] | hip peak / RMS [N m] | wheel peak / RMS [N m] | hip / wheel speed peak [rad/s] | hip / wheel power peak [W] |",
                  "|---|---|---|---|---|---|---|"]
        for n, s in r["scen"].items():
            lines.append(f"| {n} | {'O' if s['recovered'] else 'X'} | {fmt(s['pitch_peak_deg'], 1)} | {fmt(s['hip_peak'])} / {fmt(s['hip_rms'])} | "
                         f"{fmt(s['wheel_peak'])} / {fmt(s['wheel_rms'])} | {fmt(s['hip_speed_peak'], 1)} / {fmt(s['wheel_speed_peak'], 1)} | "
                         f"{fmt(s['hip_power_peak'], 0)} / {fmt(s['wheel_power_peak'], 0)} |")
        e = envelope(r)
        lines += ["", "Envelope over recovered scenarios: " + ", ".join(f"{k} {v[0]:.2f} ({v[1]})" for k, v in e.items() if k != "excluded")
                  + (f"; excluded (not recovered): {', '.join(e['excluded'])}" if e["excluded"] else "")]
    if demand:
        lines += ["", "## Static holding torque per hip motor (steady mean, stand_L*) vs. analytic VMC (per-leg weight incl. legs)", "",
                  "| L0 [m] | " + " | ".join(f"{t} sim / analytic" for t in demand) + " |", "|---|" + "---|" * len(demand)]
        for n in [n for n in ok[demand[0]]["scen"] if n.startswith("stand_L")]:
            L0 = float(n[len("stand_L"):])
            cells = []
            for t in demand:
                sm = ok[t]["scen"][n]["steady_mean"]
                a = ok[t]["analytic_static_hip_crank"]
                key = min(a, key=lambda x: abs(float(x) - L0))
                cells.append(f"{max(abs(sm['left_hip']), abs(sm['left_crank'])):.2f} / {abs(a[key][0]):.2f} (L0 {key})")
            lines.append(f"| {L0:.3f} | " + " | ".join(cells) + " |")
        lines += ["", "## Wheel slope holding torque (steady mean) vs. m g R sin(a) / 2", "", "| case | slope | sim [N m] | analytic [N m] |", "|---|---|---|---|"]
        for t in demand:
            r = ok[t]
            for n, s in r["scen"].items():
                if n.endswith("_stand") and n.startswith("slope_"):
                    a = math.radians(float(n.split("_")[1]))
                    ana = r["total_mass"] * G * r["case"]["wheel_radius"] * math.sin(a) / 2
                    lines.append(f"| {t} | {math.degrees(a):.0f} deg | {abs(s['steady_mean']['left_wheel']):.2f} | {ana:.2f} |")
    clamped = [t for t in ok if t not in demand]
    if clamped:
        keys = ["tilt_20", "tilt_25", "push_1.5", "push_2.0", "yaw_1.0", "height_full_range", "slope_15_up_0.5"]
        lines += ["", "## Verification with motor limits", "",
                  "| case | LQI6 budget hip / wheel | clamp hip / wheel | not recovered | " + " | ".join(f"{k}: pitch pk / hip pk / wheel pk" for k in keys) + " |",
                  "|---|---|---|---|" + "---|" * len(keys)]
        for t in clamped:
            r = ok[t]
            bad = [n for n, s in r["scen"].items() if not s["recovered"]]
            cells = []
            for k_ in keys:
                s = r["scen"].get(k_)
                cells.append("-" if s is None else f"{fmt(s['pitch_peak_deg'], 1)} / {fmt(s['hip_peak'], 1)} / {fmt(s['wheel_peak'], 1)}")
            lines.append(f"| {t} | {r['design_hip']:.1f} / {r['design_wheel']:.2f} | {r['case']['clamp_hip']:.0f} / {r['case']['clamp_wheel']:.0f} | "
                         f"{', '.join(bad) if bad else 'none'} | " + " | ".join(cells) + " |")
    return "\n".join(lines) + "\n"


def main(argv: List[str] | None = None) -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", nargs="*", help="tags of the standard cases to run")
    ap.add_argument("--tag", help="run one custom case with this tag (uses --mass, --seed, --design-*, --clamp-*)")
    ap.add_argument("--mass", type=float, default=15.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--design-hip", type=float, default=0.0)
    ap.add_argument("--design-wheel", type=float, default=0.0)
    ap.add_argument("--clamp-hip", type=float, default=0.0)
    ap.add_argument("--clamp-wheel", type=float, default=0.0)
    ap.add_argument("--wheel-radius", type=float, default=0.065)
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--jobs", type=int, default=max(1, cpu_count() - 2))
    ap.add_argument("--out", type=Path, default=OUT_DIR)
    args = ap.parse_args(argv)
    if args.tag:
        cases = [Case(args.tag, args.mass, args.seed, args.design_hip, args.design_wheel, args.clamp_hip, args.clamp_wheel,
                      args.wheel_radius)]
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
    results = {}                          # the summary covers every case stored in the output folder
    for f in sorted(args.out.glob("*.json")):
        results[f.stem] = json.loads(f.read_text(encoding="utf-8"))
    (args.out / "summary.md").write_text(summary_markdown(results), encoding="utf-8")
    print(f"wrote {args.out / 'summary.md'}")


if __name__ == "__main__":
    main()
