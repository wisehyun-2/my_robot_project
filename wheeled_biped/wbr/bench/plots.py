"""Figures from results/bench/summary.json -> results/figures/*.png"""
from __future__ import annotations

import json
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from wbr import RESULTS_DIR

LABELS = {"plan1_pid": "Plan 1: upkie PI", "plan1_mpc": "Plan 1: upkie MPC", "plan2_lqi6": "Plan 2: DIABLO + LQI6",
          "plan3_pid": "Plan 3: DIABLO + upkie PI", "plan3_mpc": "Plan 3: DIABLO + upkie MPC",
          "plan4_lqi6": "Plan 4: upkie HW + LQI6 (1 kHz)", "plan4_lqi6_200hz": "Plan 4: upkie HW + LQI6 (200 Hz)",
          "plan4_lqi6_mpc": "Plan 4: upkie HW + LQI6 + MPC host"}
COLORS = {"plan1_pid": "tab:blue", "plan1_mpc": "tab:cyan", "plan2_lqi6": "tab:red", "plan3_pid": "tab:green", "plan3_mpc": "tab:olive",
          "plan4_lqi6": "tab:purple", "plan4_lqi6_200hz": "tab:pink", "plan4_lqi6_mpc": "tab:brown"}


def main():
    summary = json.loads((RESULTS_DIR / "bench" / "summary.json").read_text(encoding="utf-8"))
    fig_dir = RESULTS_DIR / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)
    base = {k: v for k, v in summary.items() if "@g" not in k}

    # 1. time responses: 10 deg tilt, 0.5 W push, 0.8 m/s velocity step
    fig, axes = plt.subplots(3, 2, figsize=(13, 10))
    for key, s in base.items():
        c, lab = COLORS.get(key, None), LABELS.get(key, key)
        tr = s["tilt"].get("10", {}).get("traces") or s["tilt"].get(10, {}).get("traces")
        if tr:
            t = np.array(tr["t"]) - tr["t"][0]
            axes[0, 0].plot(t, np.degrees(tr["pitch"]), color=c, label=lab)
            axes[0, 1].plot(t, tr["vx"], color=c, label=lab)
        tr = s["push"].get("0.5", {}).get("traces") or s["push"].get(0.5, {}).get("traces")
        if tr:
            t = np.array(tr["t"]) - tr["t"][0]
            axes[1, 0].plot(t, np.degrees(tr["pitch"]), color=c, label=lab)
            axes[1, 1].plot(t, tr["vx"], color=c, label=lab)
        tr = s.get("velocity", {}).get("traces")
        if tr:
            t = np.array(tr["t"]) - tr["t"][0]
            axes[2, 0].plot(t, np.degrees(tr["pitch"]), color=c, label=lab)
            axes[2, 1].plot(t, tr["vx"], color=c, label=lab)
    titles = [("10 deg initial tilt: pitch [deg]", "10 deg initial tilt: velocity [m/s]"),
              ("0.5 W push for 0.1 s at t=1 s: pitch [deg]", "0.5 W push: velocity [m/s]"),
              ("0.8 m/s velocity step (t=0.5..3 s): pitch [deg]", "0.8 m/s velocity step: velocity [m/s]")]
    for i, (a, b) in enumerate(titles):
        axes[i, 0].set_title(a); axes[i, 1].set_title(b)
        axes[i, 0].grid(alpha=0.3); axes[i, 1].grid(alpha=0.3)
    axes[0, 0].legend(fontsize=8)
    axes[2, 0].set_xlabel("t [s]"); axes[2, 1].set_xlabel("t [s]")
    fig.tight_layout()
    fig.savefig(fig_dir / "time_responses.png", dpi=130)

    # 2. robustness bars
    fig, axes = plt.subplots(1, 4, figsize=(15, 5))
    keys = list(base.keys())
    names = [LABELS.get(k, k).replace(": ", "\n") for k in keys]
    cols = [COLORS.get(k, "gray") for k in keys]
    axes[0].bar(names, [base[k]["max_tilt_deg"] for k in keys], color=cols); axes[0].set_title("max recoverable tilt [deg]")
    axes[1].bar(names, [base[k]["max_push_frac_W"] for k in keys], color=cols); axes[1].set_title("max push (0.1 s) [x robot weight]")
    axes[2].bar(names, [base[k]["max_latency_ms"] or 0 for k in keys], color=cols); axes[2].set_title("latency tolerance [ms]")
    axes[3].bar(names, [base[k].get("stand", {}).get("wheel_torque_rms", 0) for k in keys], color=cols); axes[3].set_title("rest wheel torque RMS [Nm]")
    for a in axes:
        a.tick_params(axis="x", labelsize=7)
        plt.setp(a.get_xticklabels(), rotation=45, ha="right", rotation_mode="anchor")   # 7+ variants: avoid overlap
        a.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(fig_dir / "robustness.png", dpi=130)

    # 3. gain-scale sweep for the upkie PI on both platforms
    fig, ax = plt.subplots(figsize=(7, 4))
    for plan in ("plan1_pid", "plan3_pid"):
        pts = [(1.0, summary[plan]["max_push_frac_W"])] if plan in summary else []
        for key, s in summary.items():
            if key.startswith(plan + "@g"):
                pts.append((s["gain_scale"], s["max_push_frac_W"]))
        if pts:
            pts.sort()
            ax.plot([p[0] for p in pts], [p[1] for p in pts], "o-", color=COLORS[plan], label=LABELS[plan])
    ax.set_xlabel("wheel balancer PI gain scale"); ax.set_ylabel("max push [x W]"); ax.grid(alpha=0.3); ax.legend()
    fig.tight_layout()
    fig.savefig(fig_dir / "pi_gain_sweep.png", dpi=130)
    print("figures written to", fig_dir)


if __name__ == "__main__":
    main()
