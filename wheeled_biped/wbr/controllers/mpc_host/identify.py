"""Identification / validation of the MPC host's prediction model against the Plan 4 simulation.

    python -m wbr.controllers.mpc_host.identify [--quick]

Writes models/upkie_plan4/plan4_closed_loop.json (host-rate reduced models for 1 kHz / 200 Hz, limits, observer,
fit errors) and results/plan4_mpc/model_fit.png, results/plan4_mpc/model_fit.json.

Validation runs (the inner loop exactly as in MPC mode: Plan4Config(host="ramp", lqi_kwargs=dict(v_ramp_rate=4.0,
v_ref_tau=0.0, freeze_int_on_ramp=False))):
  * velocity step 0 -> 0.5 m/s (held 2.5 s) -> 0, through the stage-1 HostEmulator + the MCU's 4 m/s^2 ramp; the
    model is driven by the reference the LQI6 actually used (law.ctrl.v_filt, logged every MCU step)
  * push 0.5 W (0.1 s at the total COM) while standing; the model is driven by the same force (input F)
  under three conditions: A = specification (noise off, latency 0), B = acceptance (noise on, latency 10 ms),
  C = B without joint dry friction (frictionloss 0, compensation off): isolates the model's main error source.
Errors are rms over the response window (step: 1.5 s .. end, push: push .. end) of model - simulation for v_base (the
judged base velocity vx), the TRUE body pitch (minus its pre-manoeuvre mean) and the total wheel torque T.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

from wbr import MODELS_DIR, RESULTS_DIR
from wbr.controllers.mpc_host import model as M
from wbr.controllers.mpc_host import reference_mpc as R

# the inner loop the host sees: MotionCtrl.host_shaped = True -> LQI6 plain tracking (4 m/s^2 ramp, no pre-filter) with
# every integrator frozen while the low-passed reference acceleration exceeds 0.1 m/s^2 (LQI6.freeze_int_on_accel_plain)
INNER_MPC_MODE = dict(host_shaped=True, plain_ramp_rate=4.0, integrators="frozen while |a_ref_lp| > 0.1 m/s^2")
OUT_JSON = MODELS_DIR / "upkie_plan4" / "plan4_closed_loop.json"
OUT_DIR = RESULTS_DIR / "plan4_mpc"


def make_plan(rate: float, noise: bool, latency_ms: float, friction: bool = True, seed: int = 0):
    from wbr.plans.plan4_upkie_lqi6 import Plan4Config, Plan4UpkieLQI6
    cfg = Plan4Config(rate_hz=rate, host="ramp", noise=noise, latency_ms=latency_ms, seed=seed)
    if not friction:
        cfg.friction_comp_Nm = 0.0
        cfg.friction_comp_wheel_Nm = 0.0
    plan = Plan4UpkieLQI6(cfg)
    if not friction:
        plan.sim.model.dof_frictionloss[:] = 0.0
        plan.mcu.p.friction_comp_Nm = 0.0
        plan.mcu.p.friction_comp_wheel_Nm = 0.0
    return plan


def run_step(plan, V: float = 0.5, t0: float = 1.5, t_on: float = 2.5, t_off: float = 3.0) -> dict:
    plan.reset()
    plan.host.set_command(host_shaped=True)       # the MCU tracks the host reference plainly (MPC-mode inner loop)
    r_log, F_log = [], []
    for T, v in ((t0, 0.0), (t_on, V), (t_off, 0.0)):
        for _ in range(int(round(T / plan.dt))):
            plan.step(v, 0.0)
            r_log.append(plan.law.ctrl.v_filt)
            F_log.append(0.0)
    return dict(L=plan.log, r=np.array(r_log), F=np.array(F_log), t_event=t0, dt=plan.dt)


def run_push(plan, frac: float = 0.5, t0: float = 1.5, T: float = 4.0) -> dict:
    plan.reset()
    plan.host.set_command(host_shaped=True)
    r_log, F_log = [], []
    for _ in range(int(round(t0 / plan.dt))):
        plan.step(0.0, 0.0)
        r_log.append(plan.law.ctrl.v_filt)
        F_log.append(0.0)
    W = plan.sim.total_mass() * 9.81
    plan.push(frac * W, 0.1)
    n_push = int(round(0.1 / plan.dt))
    for k in range(int(round(T / plan.dt))):
        plan.step(0.0, 0.0)
        r_log.append(plan.law.ctrl.v_filt)
        F_log.append(frac * W if k < n_push else 0.0)
    return dict(L=plan.log, r=np.array(r_log), F=np.array(F_log), t_event=t0, dt=plan.dt)


def compare(model: M.LTIModel, run: dict) -> dict:
    """Simulate the MCU-rate model with the logged r (and F) and compare with the simulation (model row k+1 is the
    state after MCU step k, i.e. log row k)."""
    U = np.stack([run["r"], run["F"], np.zeros_like(run["r"])], axis=1)
    Y, _ = model.simulate(U)
    Y = Y[1:]
    L = run["L"]
    n = len(Y)
    t = L["t"][:n]
    pre = t < run["t_event"]
    sel = t >= run["t_event"]
    vx = L["vx"][:n]
    pt = L["pitch_true"][:n] - L["pitch_true"][:n][pre].mean()
    T = L["T"][:n] - L["T"][:n][pre].mean()
    ym = dict(v_base=Y[:, model.iy("v_base")], pitch=Y[:, model.iy("pitch")], T=Y[:, model.iy("T")])
    rms = lambda a: float(np.sqrt(np.mean(a ** 2)))
    out = dict(rms_v=rms(ym["v_base"][sel] - vx[sel]), rms_pitch_deg=math.degrees(rms(ym["pitch"][sel] - pt[sel])),
               rms_T=rms(ym["T"][sel] - T[sel]),
               peak_v_sim=float(np.abs(vx[sel]).max()), peak_v_model=float(np.abs(ym["v_base"][sel]).max()),
               peak_pitch_sim_deg=float(math.degrees(np.abs(pt[sel]).max())),
               peak_pitch_model_deg=float(math.degrees(np.abs(ym["pitch"][sel]).max())))
    out["trace"] = dict(t=t[::5].tolist(), vx=vx[::5].tolist(), pitch=np.degrees(pt[::5]).tolist(), T=T[::5].tolist(),
                        r=run["r"][:n][::5].tolist(), v_model=ym["v_base"][::5].tolist(),
                        pitch_model=np.degrees(ym["pitch"][::5]).tolist(), T_model=ym["T"][::5].tolist())
    return out


def compare_reduced(red: M.LTIModel, run: dict, m: int) -> dict:
    """Host-rate reduced model driven by the period-averaged reference (what the host can represent with its ZOH)."""
    L = run["L"]
    r = run["r"]
    n_h = len(r) // m
    r_h = r[:n_h * m].reshape(n_h, m).mean(axis=1)
    U = np.stack([r_h, np.zeros(n_h)], axis=1)
    Y, _ = red.simulate(U)
    t = L["t"][:n_h * m:m]                     # host instants (start of each period, row k*m is after step k*m)
    idx = np.arange(n_h) * m
    vx = L["vx"][idx]
    pre = t < run["t_event"]
    pt = L["pitch_true"][idx] - L["pitch_true"][idx][pre].mean()
    sel = t >= run["t_event"]
    v_m = Y[:, red.iy("v_base")]
    p_m = Y[:, red.iy("pitch")]
    rms = lambda a: float(np.sqrt(np.mean(a ** 2)))
    return dict(rms_v=rms(v_m[sel] - vx[sel]), rms_pitch_deg=math.degrees(rms(p_m[sel] - pt[sel])),
                trace=dict(t=t.tolist(), v_model=v_m.tolist(), pitch_model=np.degrees(p_m).tolist()))


def order_table(plan, period: float = 0.02) -> list:
    full = M.plan_closed_loop(plan)
    m = int(round(period / full.Ts))
    lifted = M.lift(full, m).select(["r", "Tf"], ["v_base", "pitch", "v", "theta", "T"])
    hsv = M.hankel_singular_values(lifted.select(["r"], ["v_base", "pitch"]))
    N = 150
    U = np.zeros((N, 2))
    U[5:, 0] = 1.0
    Yf, _ = lifted.simulate(U)
    rows = []
    for order in (2, 3, 4, 5, 6, 7):
        red = M.reduce(lifted, order, input_scale=[1.0, 0.05], output_scale=[0.1, math.radians(1.0), 1e3, 1e3, 1e3])
        Y, _ = red.simulate(U)
        rows.append(dict(order=order, step_rms_v_pct=float(np.sqrt(np.mean((Y[:, 0] - Yf[:, 0]) ** 2)) * 100),
                         step_rms_pitch_deg=float(np.degrees(np.sqrt(np.mean((Y[:, 1] - Yf[:, 1]) ** 2))))))
    return dict(hsv=hsv[:10].tolist(), rows=rows, full_order_mcu=full.n)


def mass_sensitivity(plan, cfg: R.MpcConfig) -> list:
    """Per-unit-acceleration peaks with the plant masses scaled (controller nominal, as in the criteria matrix)."""
    import copy
    from wbr.controllers.lqi6.plant_serial import SerialPlant
    rows = []
    K = M._gains_no_side_effect(plan.law.ctrl, plan.params.L0)
    d = int(round(plan.cfg.latency_ms * 1e-3 / plan.law.ctrl.Ts))
    for ms in (0.85, 1.0, 1.15):
        p = copy.deepcopy(plan.params)
        p.m_w *= ms
        p.I_w *= ms                                  # rigid wheel bodies (the rotor armature J_w is a joint property, not scaled)
        p.m_b *= ms
        p.I_b *= ms
        p.leg.m_l = p.leg.m_l * ms
        p.leg.I_l = p.leg.I_l * ms
        opts = M.lqi6_options(plan.law.ctrl, plan.params.L0)
        cl = M.lqi6_closed_loop(SerialPlant(p), M.select_gains(K, opts), plan.params.L0, plan.law.ctrl.Ts, d,
                                h_base=plan.hip_above_base, **M.model_kwargs(opts))
        full = cl.select(["r"], ["T", "pitch", "theta"])
        Ts = full.Ts
        t = np.arange(int(round(3.0 / Ts))) * Ts
        r = np.cumsum(np.minimum(t * 20.0, 1.0)) * Ts
        Y, _ = full.simulate(r[:, None])
        rows.append(dict(mass_scale=ms, wheel_torque_peak_Nm=float(np.abs(Y[:, 0]).max() / 2),
                         pitch_peak_deg=float(np.degrees(np.abs(Y[:, 1]).max())),
                         lean_peak_deg=float(np.degrees(np.abs(Y[:, 2]).max()))))
    return rows


def plot(fit: dict, path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(3, 2, figsize=(12, 9), sharex="col")
    for col, key in enumerate(("step", "push")):
        A = fit["1000"]["A"][key]
        B = fit["1000"]["B"][key]
        tr, trB = A["trace"], B["trace"]
        ax[0, col].plot(tr["t"], tr["vx"], color="0.6", lw=1.0, label="sim (A: noise off, latency 0)")
        ax[0, col].plot(tr["t"], tr["v_model"], "C0", lw=1.6, label="full-order model (A)")
        ax[0, col].plot(trB["t"], trB["vx"], color="C3", lw=0.6, alpha=0.6, label="sim (B: noise on, latency 10 ms)")
        ax[0, col].plot(trB["t"], trB["v_model"], "C1--", lw=1.4, label="full-order model (B)")
        if key == "step":
            ax[0, col].plot(tr["t"], tr["r"], "k:", lw=1.0, label="reference r (LQI6 v_filt)")
            red = fit["1000"]["B"]["step_reduced"]["trace"]
            ax[0, col].plot(red["t"], red["v_model"], "C2.", ms=3, label="reduced 5-state host model (B, 50 Hz)")
        ax[0, col].set_ylabel("v_base [m/s]")
        ax[0, col].legend(fontsize=7, loc="best")
        ax[1, col].plot(tr["t"], tr["pitch"], color="0.6", lw=1.0, label="sim A")
        ax[1, col].plot(tr["t"], tr["pitch_model"], "C0", lw=1.6, label="model A")
        ax[1, col].plot(trB["t"], trB["pitch"], color="C3", lw=0.6, alpha=0.6, label="sim B")
        ax[1, col].plot(trB["t"], trB["pitch_model"], "C1--", lw=1.4, label="model B")
        ax[1, col].set_ylabel("body pitch (true) [deg]")
        ax[1, col].legend(fontsize=7)
        ax[2, col].plot(tr["t"], tr["T"], color="0.6", lw=1.0, label="sim A (telemetry T)")
        ax[2, col].plot(tr["t"], tr["T_model"], "C0", lw=1.6, label="model A")
        ax[2, col].set_ylabel("wheel torque T (total) [Nm]")
        ax[2, col].set_xlabel("t [s]")
        ax[2, col].legend(fontsize=7)
        title = ("velocity step 0 -> 0.5 -> 0 m/s (manoeuvre model: integrators frozen)" if key == "step"
                 else "push 0.5 W (0.1 s) at t = 1.5 s (stand-still model: integrators active)")
        ax[0, col].set_title(f"{title}: rms v {A['rms_v']:.3f} / {B['rms_v']:.3f} m/s, pitch {A['rms_pitch_deg']:.2f} / "
                             f"{B['rms_pitch_deg']:.2f} deg (A / B)", fontsize=9)
    fig.suptitle("Plan 4 MPC host: LQI6 closed-loop prediction model vs MuJoCo (1 kHz MCU, inner loop in MPC mode)", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="1 kHz only")
    args = ap.parse_args(argv)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    cfg = R.MpcConfig()
    fit = {}
    models = {}
    for rate in ((1000.0,) if args.quick else (1000.0, 200.0)):
        key = f"{rate:.0f}"
        fit[key] = {}
        for cond, (noise, lat, fric) in (("A", (False, 0.0, True)), ("B", (True, 10.0, True)), ("C", (True, 10.0, False))):
            plan = make_plan(rate, noise, lat, friction=fric)
            full = M.plan_closed_loop(plan)                     # manoeuvre regime: integrators frozen (MPC prediction)
            full_rest = M.plan_closed_loop(plan, regime="rest")  # stand-still regime: integrators active (push at rest)
            step = run_step(plan)
            push = run_push(plan)
            entry = dict(step=compare(full, step), push=compare(full_rest, push), push_manoeuvre_model=compare(full, push))
            entry["push_manoeuvre_model"].pop("trace", None)
            mdl = R.prediction_model(plan, 0.02, cfg.order)
            entry["step_reduced"] = compare_reduced(mdl["red"], step, mdl["m"])
            if cond != "B":                     # keep the json small: traces only for the figure conditions
                entry["step_reduced"].pop("trace", None)
            if cond == "C":
                for k in ("step", "push"):
                    entry[k].pop("trace", None)
            fit[key][cond] = entry
            print(f"{key} Hz {cond}: step rms v {entry['step']['rms_v']:.4f} pitch {entry['step']['rms_pitch_deg']:.3f} deg "
                  f"T {entry['step']['rms_T']:.3f} | push (rest model) rms v {entry['push']['rms_v']:.4f} pitch {entry['push']['rms_pitch_deg']:.3f}"
                  f" (manoeuvre model {entry['push_manoeuvre_model']['rms_v']:.4f}) | "
                  f"reduced step rms v {entry['step_reduced']['rms_v']:.4f} ({time.time() - t0:.0f} s)")
        # host model as used by the MPC at the acceptance condition (latency 10 ms)
        plan = make_plan(rate, True, 10.0)
        mdl = R.prediction_model(plan, 0.02, cfg.order)
        lim = R.derive_limits(plan, cfg)
        obs = R.prediction_observer(mdl["red"], cfg.obs_sigma_v, cfg.obs_sigma_w, cfg.obs_sigma_f, cfg.obs_sigma_z)
        models[key] = dict(reduced=mdl["red"].to_json(), dc=mdl["dc"], mcu_steps_per_period=mdl["m"], limits=lim,
                           observer=dict(L=obs["L"].tolist(), poles_abs=np.abs(obs["observer_poles"]).tolist(),
                                         slow_eig=[obs["slow_eig"].real, obs["slow_eig"].imag]),
                           eig_abs=np.abs(np.linalg.eigvals(mdl["red"].A)).tolist(),
                           order_table=order_table(plan), mass_sensitivity=mass_sensitivity(plan, cfg))
    for key in fit:
        for cond in fit[key]:
            for k in list(fit[key][cond]):
                if isinstance(fit[key][cond][k], dict):
                    fit[key][cond][k] = {kk: vv for kk, vv in fit[key][cond][k].items()}
    plot(fit, OUT_DIR / "model_fit.png")
    summary = {key: {cond: {k: {kk: vv for kk, vv in v.items() if kk != "trace"} for k, v in fit[key][cond].items()}
                     for cond in fit[key]} for key in fit}
    (OUT_DIR / "model_fit.json").write_text(json.dumps(summary, indent=1), encoding="utf-8")
    out = dict(description="Plan 4 MPC host prediction model: LQI6 closed loop (plan.plant.linearize + MCU gain table, "
                           "ZOH at the MCU period, 10 ms observation latency), lifted to the 20 ms host period, balanced "
                           "truncation (DC matched) on the (r, Tf) -> (v_base, pitch, v, theta, T) channels",
               inner_loop=INNER_MPC_MODE, host_period_s=0.02, config={k: getattr(cfg, k) for k in ("order", "horizon_s")},
               models=models, fit=summary)
    OUT_JSON.write_text(json.dumps(out, indent=1, default=float), encoding="utf-8")
    print("wrote", OUT_JSON, "and", OUT_DIR / "model_fit.png", f"({time.time() - t0:.0f} s)")


if __name__ == "__main__":
    main()
