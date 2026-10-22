"""Stage-1 ramp host vs stage-2 MPC host on the acceptance velocity step (items 8 / 19).

    python -m wbr.controllers.mpc_host.compare_hosts [--rate 1000] [--seed 0] [--mass 1.0] [--friction 0.75]

Same plan factory as the acceptance battery (noise on, 10 ms latency); profile 0.5 s stand, 2.5 s at 0.5 m/s, 3 s at
0.  Writes results/plan4_mpc/ramp_vs_mpc[_<rate>Hz].png / .json.  The MPC panel also shows the host's online
prediction (period-averaged telemetry speed vs. the host model's prediction for the same instant).
"""
from __future__ import annotations

import argparse
import json
import math

import numpy as np

from wbr import RESULTS_DIR
from wbr.bench import acceptance as A

KEYS = ("rise_s", "overshoot_pct", "ss_err", "pitch_peak", "stop_disp", "settled", "wheel_peak", "wheel_rms10")


def run(host: str, rate: float, seed: int, mass: float, friction: float) -> dict:
    plan = A.make_plan(rate, seed, mass, friction, host)
    ref = []
    orig_step = plan.step

    def step(v=0.0, w=0.0):                         # record the reference the LQI6 actually used
        o = orig_step(v, w)
        ref.append(plan.law.ctrl.v_filt)
        return o

    plan.step = step
    res = A.scenario_velocity(plan, V=0.5)
    L = plan.log
    out = dict(metrics={k: res.get(k) for k in KEYS}, t=L["t"].tolist(), vx=L["vx"].tolist(),
               pitch=np.degrees(L["pitch_true"]).tolist(), x=L["x"].tolist(), r=ref,
               tau_w=(0.5 * (np.abs(L["tau_lw"]) + np.abs(L["tau_rw"]))).tolist(), v_cmd=L["v_cmd"].tolist())
    if host == "mpc":
        tr = plan.host.trace
        out["host"] = {k: list(map(float, tr[k])) for k in ("t", "target", "r", "a", "v_avg", "v_pred_prev", "solve_ms")}
        out["stats"] = plan.host.stats()
    return out


def plot(ramp: dict, mpc: dict, path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(5, 1, figsize=(10, 13), sharex=True, gridspec_kw=dict(height_ratios=[1.4, 1, 0.8, 0.8, 1]))
    rows = []
    for d, name, c in ((ramp, "stage 1 ramp host", "C0"), (mpc, "stage 2 MPC host", "C3")):
        t = np.asarray(d["t"])
        m = d["metrics"]
        rows.append(f"{name}: rise {m['rise_s']:.2f} s, overshoot {m['overshoot_pct']:.1f} %, pitch {m['pitch_peak']:.2f} deg, "
                    f"stop {m['stop_disp']:.3f} m, wheel peak {m['wheel_peak']:.2f} Nm")
        ax[0].plot(t, d["vx"], c, lw=0.9, label=f"{name}: v_base")
        ax[0].plot(t, d["r"], c, lw=1.4, ls="--", alpha=0.8, label=f"{name}: reference used by the LQI6")
        ax[1].plot(t, d["pitch"], c, lw=0.8, label=name)
        ax[2].plot(t, d["x"], c, lw=1.2, label=name)
        ax[3].plot(t, d["tau_w"], c, lw=0.6, alpha=0.8, label=name)
    ax[0].plot(ramp["t"], ramp["v_cmd"], "k:", lw=1.0, label="target (user command)")
    ax[0].axhline(0.55, color="0.5", lw=0.6, ls=":")
    ax[0].set_ylabel("velocity [m/s]")
    ax[0].legend(fontsize=7, loc="upper right", ncol=2)
    nl = chr(10)
    ax[0].set_title(nl.join(rows) + nl + "(stage 1: 2 m/s^2 ramp + 0.3 s pre-filter, integrator frozen while ramping; "
                    "stage 2: plain inner loop + MPC; dotted = item 19 limits)", fontsize=8)
    ax[1].axhline(2.0, color="0.5", lw=0.6, ls=":")
    ax[1].axhline(-2.0, color="0.5", lw=0.6, ls=":")
    ax[1].set_ylabel("true body pitch [deg]")
    ax[1].legend(fontsize=7, loc="lower right")
    ax[2].set_ylabel("base x [m]")
    ax[2].legend(fontsize=7, loc="lower right")
    ax[3].axhline(1.36, color="0.5", lw=0.6, ls=":")
    ax[3].set_ylabel("|wheel torque| mean [Nm]")
    ax[3].legend(fontsize=7, loc="upper right")
    h = mpc.get("host")
    if h:
        th = np.asarray(h["t"])
        ax[4].plot(th[1:], np.asarray(h["v_avg"])[1:], "C3.", ms=2.5, label="telemetry est.v, 20 ms average (aligned one period back)")
        ax[4].plot(th[1:], np.asarray(h["v_pred_prev"])[:-1], "k-", lw=0.9, label="host prediction for the same instant")
        ax[4].step(th, np.asarray(h["r"]), "C1", lw=1.0, where="post", label="published reference r (50 Hz)")
        ax[4].set_ylabel("MPC host [m/s]")
        ax[4].legend(fontsize=7, loc="upper right")
    ax[4].set_xlabel("t [s]")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=float, default=1000.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--mass", type=float, default=1.0)
    ap.add_argument("--friction", type=float, default=0.75)
    args = ap.parse_args(argv)
    out_dir = RESULTS_DIR / "plan4_mpc"
    out_dir.mkdir(parents=True, exist_ok=True)
    ramp = run("ramp", args.rate, args.seed, args.mass, args.friction)
    mpc = run("mpc", args.rate, args.seed, args.mass, args.friction)
    suffix = "" if args.rate == 1000.0 and args.mass == 1.0 and args.friction == 0.75 else f"_{args.rate:.0f}Hz_m{args.mass}_mu{args.friction}"
    plot(ramp, mpc, out_dir / f"ramp_vs_mpc{suffix}.png")
    summary = dict(rate=args.rate, seed=args.seed, mass_scale=args.mass, friction=args.friction,
                   ramp=ramp["metrics"], mpc=mpc["metrics"], mpc_stats=mpc.get("stats"))
    (out_dir / f"ramp_vs_mpc{suffix}.json").write_text(json.dumps(summary, indent=1, default=float), encoding="utf-8")
    for name, d in (("ramp", ramp), ("mpc", mpc)):
        print(name, " ".join(f"{k} {v:.3f}" if isinstance(v, float) else f"{k} {v}" for k, v in d["metrics"].items()))


if __name__ == "__main__":
    main()
