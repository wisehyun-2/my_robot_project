"""Run the test battery on every plan variant and write results/bench/*.json + summary.md.

    python -m wbr.bench.run_bench [--plans plan1_pid plan2_lqi6 ...] [--jobs N] [--quick]
"""
from __future__ import annotations

import argparse
import json
import time
from multiprocessing import Pool, cpu_count

from wbr import RESULTS_DIR
from wbr.bench.battery import LATENCIES, PUSHES, TILTS, run_task

BASELINE = ["plan1_pid", "plan1_mpc", "plan2_lqi6", "plan3_pid", "plan3_mpc"]
GAIN_VARIANTS = {"plan1_pid": [0.5, 1.5, 2.0], "plan3_pid": [0.5, 1.5, 2.0, 3.0]}


def build_tasks(plans, quick=False):
    tasks = []
    tilts = TILTS if not quick else [10, 20, 30]
    pushes = PUSHES if not quick else [0.5, 1.0, 1.5]
    lats = LATENCIES if not quick else [0, 20, 40]
    for name in plans:
        tasks.append((name, "stand", None, 1.0))
        tasks += [(name, "tilt", a, 1.0) for a in tilts]
        tasks += [(name, "push", f, 1.0) for f in pushes]
        tasks.append((name, "velocity", None, 1.0))
        tasks.append((name, "yaw", None, 1.0))
        tasks.append((name, "height", None, 1.0))
        tasks.append((name, "link_loss", None, 1.0))
        tasks += [(name, "latency", ms, 1.0) for ms in lats]
        for g in GAIN_VARIANTS.get(name, []):
            tasks += [(name, "push", f, g) for f in pushes]
            tasks += [(name, "tilt", a, g) for a in tilts]
            tasks.append((name, "stand", None, g))
    return tasks


def aggregate(results: dict) -> dict:
    """results[(name, scenario, arg, gain)] -> per (name, gain) summary."""
    summary = {}
    for (name, scenario, arg, gain), r in results.items():
        key = f"{name}" if gain == 1.0 else f"{name}@g{gain}"
        s = summary.setdefault(key, {"plan": name, "gain_scale": gain, "tilt": {}, "push": {}, "latency": {}})
        if scenario in ("tilt", "push", "latency"):
            s[scenario][arg] = r
        else:
            s[scenario] = r
    for s in summary.values():
        ok_t = [a for a, r in s["tilt"].items() if r.get("ok")]
        al_t = [a for a, r in s["tilt"].items() if r.get("alive")]
        s["max_tilt_deg"] = max(ok_t) if ok_t else 0.0
        s["max_tilt_alive_deg"] = max(al_t) if al_t else 0.0
        ok_p = [f for f, r in s["push"].items() if r.get("ok")]
        al_p = [f for f, r in s["push"].items() if r.get("alive")]
        s["max_push_frac_W"] = max(ok_p) if ok_p else 0.0
        s["max_push_alive_frac_W"] = max(al_p) if al_p else 0.0
        s["max_push_N"] = max(s["push"][f]["force_N"] for f in ok_p) if ok_p else 0.0
        ok_l = [ms for ms, r in s["latency"].items() if r.get("ok")]
        al_l = [ms for ms, r in s["latency"].items() if r.get("alive")]
        s["max_latency_ms"] = max(ok_l) if ok_l else None
        s["max_latency_alive_ms"] = max(al_l) if al_l else None
    return summary


def fmt(v, nd=2):
    if v is None:
        return "-"
    if isinstance(v, bool):
        return "O" if v else "X"
    if isinstance(v, float):
        return f"{v:.{nd}f}"
    return str(v)


def summary_table(summary: dict) -> str:
    rows = ["| variant | max tilt settled / alive [deg] | max push settled / alive [xW] | latency tol. settled / alive [ms] | stand: pitch rms [deg] / drift [m] / wheel tau rms [Nm] | 10 deg tilt settle [s] / disp [m] | 0.5 W push: pitch peak [deg] / settle [s] | 0.8 m/s step: rise [s] / overshoot [%] / pitch peak [deg] | yaw 1 rad/s -> [rad/s] | max crouch | link loss (alive / v after 6 s) | RT factor |",
            "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for key, s in sorted(summary.items()):
        st = s.get("stand", {})
        t10 = s["tilt"].get(10, {})
        p05 = s["push"].get(0.5, {})
        vel = s.get("velocity", {})
        yaw = s.get("yaw", {})
        h = s.get("height", {})
        ll = s.get("link_loss", {})
        rows.append("| %s | %s / %s | %s / %s | %s / %s | %s / %s / %s | %s / %s | %s / %s | %s / %s / %s | %s | %s | %s | %s |" % (
            key, fmt(s["max_tilt_deg"], 0), fmt(s["max_tilt_alive_deg"], 0), fmt(s["max_push_frac_W"]), fmt(s["max_push_alive_frac_W"]),
            fmt(s["max_latency_ms"], 0), fmt(s["max_latency_alive_ms"], 0),
            fmt(st.get("pitch_rms_deg")), fmt(st.get("drift_m"), 3), fmt(st.get("wheel_torque_rms"), 3),
            fmt(t10.get("settle_s")), fmt(t10.get("disp_m")), fmt(p05.get("pitch_peak_deg"), 1), fmt(p05.get("settle_s")),
            fmt(vel.get("rise_s")), fmt(vel.get("overshoot_pct"), 0), fmt(vel.get("pitch_peak_deg"), 1),
            fmt(yaw.get("yaw_rate_mean")), (f"{fmt(h.get('max_crouch_cm'), 0)} cm (pitch {fmt(h.get('pitch_peak_deg'), 1)} deg)" if h.get("ok") else "X"),
            (fmt(ll.get("ok")) + " / " + fmt(ll.get("v_end"))) if ll.get("ok") is not None else "n/a", fmt(st.get("realtime_factor"), 1)))
    return "\n".join(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--plans", nargs="*", default=BASELINE)
    ap.add_argument("--jobs", type=int, default=max(1, cpu_count() - 2))
    ap.add_argument("--quick", action="store_true")
    args = ap.parse_args()
    out_dir = RESULTS_DIR / "bench"
    out_dir.mkdir(parents=True, exist_ok=True)
    tasks = build_tasks(args.plans, args.quick)
    print(f"{len(tasks)} tasks on {args.jobs} processes")
    t0 = time.time()
    results = {}
    with Pool(args.jobs) as pool:
        for i, (key, r) in enumerate(pool.imap_unordered(run_task, tasks)):
            results[key] = r
            if "error" in r:
                print("ERROR", key, r["error"])
            if (i + 1) % 20 == 0:
                print(f"  {i + 1}/{len(tasks)} done ({time.time() - t0:.0f} s)")
    print(f"finished in {time.time() - t0:.0f} s")
    summary = aggregate(results)
    serial = {"|".join(str(x) for x in k): v for k, v in results.items()}
    (out_dir / "raw_results.json").write_text(json.dumps(serial, indent=1), encoding="utf-8")
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=1, default=str), encoding="utf-8")
    table = summary_table(summary)
    (out_dir / "summary.md").write_text(table + "\n", encoding="utf-8")
    print(table)


if __name__ == "__main__":
    main()
