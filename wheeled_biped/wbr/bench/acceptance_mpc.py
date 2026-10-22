"""Stage-2 (MPC host) items of the Plan 4 acceptance battery that are not plain scenarios (docs/합격기준.md 22, 23).

    run_mpc_item(item, task) -> dict          called by wbr.bench.acceptance.run_task for scenarios "mpc_timing" / "mpc_fallback"

22 (record)        QP solve time on the development PC: velocity step 0 -> 0.5 -> 0 m/s and a 0.5 W push with host="mpc";
                   p50 / p99 / max of time.perf_counter() around every QP solve, QP failures (no solution from DAQP nor
                   OSQP), constraint violations of the MPC's own plan.
23 (must-survive)  forced solver failure (MpcHost.force_fail = True on every host the plan creates): the host must switch
                   to the ramp fallback and items 1 (stand, here 10 s), 8 (velocity step) and 11 (link loss) must pass
                   with the stage-1 criteria.  The per-item verdicts are returned as item1_pass / item8_pass /
                   item11_pass and passes_1_8_11.

Also: constraint_check(rate, ...) runs items 8 / 9 / 13 / 19 with the MPC host and counts the plan's constraint
violations (criterion 21 covers 8, 9, 13 and 19; the battery's item 21 only runs the velocity step).
"""
from __future__ import annotations

import math
from typing import Dict, List

import numpy as np

from wbr.bench import acceptance as A


def _host_stats(hosts: List) -> dict:
    times = np.concatenate([np.asarray(h.solve_times_ms, dtype=float) for h in hosts]) if hosts else np.zeros(0)
    times = times[np.isfinite(times)]
    st0 = hosts[0].stats() if hosts else {}
    return dict(n_solves=int(times.size),
                solve_p50_ms=float(np.percentile(times, 50)) if times.size else None,
                solve_p99_ms=float(np.percentile(times, 99)) if times.size else None,
                solve_max_ms=float(times.max()) if times.size else None,
                solve_mean_ms=float(times.mean()) if times.size else None,
                qp_failures=int(sum(h.qp_failures for h in hosts)),
                solver_failures=int(sum(getattr(h, "solver_failures", 0) for h in hosts)),
                constraint_violations=int(sum(h.violations for h in hosts)),
                fallback_used=int(any(h.fallback_used for h in hosts)),
                n_var=st0.get("n_var"), n_ineq=st0.get("n_ineq"), horizon_steps=st0.get("horizon_steps"),
                a_max=st0.get("a_max"), j_max=st0.get("j_max"), model_ok=int(all(getattr(h, "model_ok", True) for h in hosts)))


def _capture_hosts(plan, force_fail: bool = False) -> List:
    """Wrap plan._make_host so that every host the plan creates (the scenarios call plan.reset()) is recorded and,
    for item 23, has force_fail set before its first tick."""
    hosts: List = []
    make = type(plan)._make_host

    def _make_host():
        h = make(plan)
        if force_fail:
            h.force_fail = True
        hosts.append(h)
        return h

    plan._make_host = _make_host
    plan.host = _make_host()
    plan.host.set_command(forward=0.0, left=0.0, up=plan.up_cmd, stand_mode=True)
    return hosts


def _verdict(item_id: int, res: dict) -> bool:
    it = A._item(item_id)
    ev = A.evaluate(it, res)
    return bool(all(c["passed"] for c in ev.values()))


def run_timing(task) -> dict:
    plan = A.make_plan(task.rate, task.seed, task.mass_scale, task.friction, "mpc")
    hosts = _capture_hosts(plan)
    vel = A.scenario_velocity(plan, V=0.5)
    push = A.scenario_push(plan, frac=0.5)
    out = _host_stats(hosts[1:] if len(hosts) > 1 else hosts)      # hosts[0] is the never-ticked construction host
    out.update(alive=int(bool(vel["alive"]) and bool(push["alive"])), vel_rise_s=vel["rise_s"], vel_overshoot_pct=vel["overshoot_pct"],
               push_settle_s=push["settle_s"])
    return out


def run_fallback(task) -> dict:
    plan = A.make_plan(task.rate, task.seed, task.mass_scale, task.friction, "mpc")
    hosts = _capture_hosts(plan, force_fail=True)
    stand = A.scenario_stand(plan, T=10.0)
    vel = A.scenario_velocity(plan, V=0.5)
    link = A.scenario_link_loss(plan, V=0.5)
    st = _host_stats(hosts[1:] if len(hosts) > 1 else hosts)
    p1, p8, p11 = _verdict(1, stand), _verdict(8, vel), _verdict(11, link)
    out = dict(alive=int(bool(stand["alive"]) and bool(vel["alive"]) and bool(link["alive"])),
               fallback_used=int(all(h.fallback_used for h in hosts[1:])) if len(hosts) > 1 else 0,
               qp_failures=st["qp_failures"], item1_pass=int(p1), item8_pass=int(p8), item11_pass=int(p11),
               passes_1_8_11=int(p1 and p8 and p11))
    for name, res in (("stand", stand), ("vel", vel), ("link", link)):
        for k, v in res.items():
            if isinstance(v, (bool, int, float)) or v is None:
                out[f"{name}_{k}"] = v
    events = [e for h in hosts[1:] for e in h.events[:2]]
    out["events"] = [f"{t:.3f} {msg}" for t, msg in events[:6]]
    return out


def run_constraints(task) -> dict:
    """Item 21: items 8, 9, 13 and 19 with the MPC host at the task's combination; worst wheel peak, summed violations."""
    res = constraint_check(task.rate, task.seed, task.mass_scale, task.friction)
    return dict(wheel_peak=max(r["wheel_peak"] for r in res.values()),
                constraint_violations=int(sum(r["constraint_violations"] for r in res.values())),
                qp_failures=int(sum(r["qp_failures"] for r in res.values())),
                alive=int(all(bool(r["alive"]) for r in res.values())),
                per_item={k: v for k, v in res.items()})


def run_mpc_item(item, task) -> dict:
    if item.scenario == "mpc_timing":
        return run_timing(task)
    if item.scenario == "mpc_fallback":
        return run_fallback(task)
    if item.scenario == "mpc_constraints":
        return run_constraints(task)
    raise KeyError(item.scenario)


# ------------------------------------------------------------------------------------------- criterion 21 helper
def constraint_check(rate: float, seed: int = 0, mass_scale: float = 1.0, friction: float = 0.75) -> Dict[int, dict]:
    """Items 8, 9, 13, 19 with the MPC host: wheel torque peak and constraint violations of the host's plan."""
    out = {}
    for item_id, scen, extra, params in ((8, "velocity", {}, dict(V=0.5)), (9, "yaw", {}, dict(W=1.0)),
                                         (13, "slope_drive", dict(slope_deg=5.0), dict(V=0.5)),
                                         (19, "velocity", {}, dict(V=0.5))):
        plan = A.make_plan(rate, seed, mass_scale, friction, "mpc", **extra)
        hosts = _capture_hosts(plan)
        res = A.SCENARIOS[scen](plan, **params)
        st = _host_stats(hosts[1:] if len(hosts) > 1 else hosts)
        out[item_id] = dict(wheel_peak=res["wheel_peak"], constraint_violations=st["constraint_violations"],
                            qp_failures=st["qp_failures"], alive=res["alive"])
    return out


if __name__ == "__main__":
    import json
    import sys
    rate = float(sys.argv[1]) if len(sys.argv) > 1 else 1000.0
    print(json.dumps(constraint_check(rate), indent=1, default=float))
