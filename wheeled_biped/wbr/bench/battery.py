"""Test battery shared by the three plans (same spirit as the SIL harness of the design report).

Every scenario runs with IMU/encoder noise and a 2 ms observation latency unless stated otherwise.
A run "recovers" when the robot is still up at the end, its pitch stays below 3 deg over the last
second and its ground velocity is within 0.3 m/s of the reference.
"""
from __future__ import annotations

import math
import time
from typing import Callable, Dict

import numpy as np

G = 9.81
DEFAULT_LATENCY_MS = 2.0


def make_plan(name: str, latency_ms: float = DEFAULT_LATENCY_MS, noise: bool = True, seed: int = 0,
              gain_scale: float = 1.0):
    if name.startswith("plan1"):
        from wbr.plans.plan1_upkie import Plan1Config, Plan1Upkie
        bal = "mpc" if "mpc" in name else "pid"
        return Plan1Upkie(Plan1Config(balancer=bal, latency_ms=latency_ms, noise=noise, seed=seed, gain_scale=gain_scale))
    if name.startswith("plan2"):
        from wbr.plans.plan2_diablo_lqi6 import Plan2Config, Plan2DiabloLQI6
        return Plan2DiabloLQI6(Plan2Config(latency_ms=latency_ms, noise=noise, seed=seed))
    if name.startswith("plan3"):
        from wbr.plans.plan3_diablo_upkie import Plan3Config, Plan3DiabloUpkie
        bal = "mpc" if "mpc" in name else "pid"
        return Plan3DiabloUpkie(Plan3Config(balancer=bal, latency_ms=latency_ms, noise=noise, seed=seed, gain_scale=gain_scale))
    raise ValueError(name)


def weight(plan) -> float:
    return plan.sim.total_mass() * G


def recovered(plan, v_end: float = 0.0) -> bool:
    if plan.fallen:
        return False
    L = plan.log
    n = int(round(1.0 / plan.dt))
    return bool(np.max(np.abs(L["pitch"][-n:])) < math.radians(3) and np.max(np.abs(L["vx"][-n:] - v_end)) < 0.3)


def traces(plan, decimate: int = 10) -> Dict[str, list]:
    L = plan.log.arrays()
    keys = [k for k in ("t", "pitch", "vx", "x", "yaw_rate", "theta_v", "L0", "tau_lw", "tau_rw", "tau_lh", "ground_vel") if k in L]
    return {k: L[k][::decimate].tolist() for k in keys}


# ----------------------------------------------------------------------------- scenarios
def scenario_stand(plan, T: float = 6.0) -> dict:
    plan.reset()
    t0 = time.perf_counter()
    plan.run(T)
    wall = time.perf_counter() - t0
    L = plan.log
    n = int(round(2.0 / plan.dt))
    tail = slice(-n, None)
    out = dict(ok=recovered(plan), pitch_rms_deg=float(math.degrees(np.sqrt(np.mean(L["pitch"][tail] ** 2)))),
               drift_m=float(abs(L["x"][-1] - L["x"][-n])), wheel_torque_rms=float(np.sqrt(np.mean(L["tau_lw"][tail] ** 2))),
               realtime_factor=float(T / wall))
    if "tau_lh" in L.rows:
        out["hip_torque_rms"] = float(np.sqrt(np.mean(L["tau_lh"][tail] ** 2)))
    return out


def settle_time(L, t_start: float, v_ref: float = 0.0) -> float | None:
    """Last time the (20 ms moving-average) pitch exceeds 1 deg or |v - v_ref| exceeds 0.1 m/s, relative to
    t_start (None if never settled).  The moving average keeps single noisy samples from resetting the clock."""
    n = max(1, int(round(0.02 / (L["t"][1] - L["t"][0]))))
    kernel = np.ones(n) / n
    pitch = np.convolve(L["pitch"], kernel, mode="same")
    vx = np.convolve(L["vx"], kernel, mode="same")
    bad = (np.abs(pitch) > math.radians(1)) | (np.abs(vx - v_ref) > 0.1)
    after = L["t"] >= t_start
    idx = np.where(bad & after)[0]
    if not len(idx):
        return 0.0
    if idx[-1] >= len(L["t"]) - 2:
        return None
    return float(L["t"][idx[-1]] - t_start)


def scenario_tilt(plan, deg: float, T: float = 8.0) -> dict:
    plan.reset(pitch=math.radians(deg))
    plan.run(T)
    L = plan.log
    ok = recovered(plan)
    return dict(ok=ok, alive=not plan.fallen, deg=deg, settle_s=settle_time(L, L["t"][0]) if ok else None,
                disp_m=float(abs(L["x"][-1])) if ok else None, wheel_torque_peak=float(np.max(np.abs(L["tau_lw"]))),
                hip_torque_peak=float(np.max(np.abs(L["tau_lh"]))) if "tau_lh" in L.rows else None,
                traces=traces(plan) if deg == 10 else None)


def scenario_push(plan, frac: float, T: float = 8.0) -> dict:
    plan.reset()
    plan.run(1.0)
    W = weight(plan)
    plan.push(frac * W, 0.1)
    plan.run(T)
    L = plan.log
    ok = recovered(plan)
    after = L["t"] >= 1.0
    return dict(ok=ok, alive=not plan.fallen, frac=frac, force_N=frac * W,
                pitch_peak_deg=float(math.degrees(np.max(np.abs(L["pitch"][after])))),
                settle_s=settle_time(L, 1.0) if ok else None,
                disp_m=float(abs(L["x"][-1] - L["x"][after][0])) if ok else None,
                v_peak=float(np.max(np.abs(L["vx"][after]))), traces=traces(plan) if abs(frac - 0.5) < 1e-9 else None)


def scenario_velocity(plan, V: float = 0.8) -> dict:
    plan.reset()
    plan.run(0.5)
    for _ in range(int(round(2.5 / plan.dt))):
        plan.step(V, 0.0)
        if plan.fallen:
            break
    ok1 = not plan.fallen
    plan.run(3.0)
    L = plan.log
    t, v = L["t"], L["vx"]
    seg = (t >= 0.5) & (t < 3.0)
    rise = None
    if v[seg].max() > 0.9 * V:
        rise = float(t[seg][np.argmax(v[seg] > 0.9 * V)] - 0.5)
    return dict(ok=ok1 and recovered(plan), rise_s=rise, overshoot_pct=float((v[seg].max() - V) / V * 100),
                v_ss=float(np.mean(v[(t >= 2.0) & (t < 3.0)])), pitch_peak_deg=float(math.degrees(np.max(np.abs(L["pitch"])))),
                leg_peak_deg=float(math.degrees(np.max(np.abs(L["theta_v"])))) if "theta_v" in L.rows else None,
                stop_disp_m=float(abs(L["x"][-1] - L["x"][t >= 3.0][0])), traces=traces(plan))


def scenario_yaw(plan, W_cmd: float = 1.0) -> dict:
    plan.reset()
    plan.run(0.5)
    for _ in range(int(round(2.0 / plan.dt))):
        plan.step(0.0, W_cmd)
        if plan.fallen:
            break
    L = plan.log
    n = int(round(1.0 / plan.dt))
    return dict(ok=not plan.fallen, yaw_rate_mean=float(np.mean(L["yaw_rate"][-n:])), cmd=W_cmd,
                pitch_peak_deg=float(math.degrees(np.max(np.abs(L["pitch"])))))


def scenario_height(plan) -> dict:
    """Crouch by 2 / 4 / 6 cm while balancing (then back up); report the largest crouch that survives."""
    out = dict(ok=False, max_crouch_cm=0.0, runs={})
    for dh in (-0.02, -0.04, -0.06):
        plan.reset()
        plan.run(1.0)
        plan.set_height_offset(dh)
        ok1 = plan.run(2.5)
        plan.set_height_offset(0.0)
        ok2 = plan.run(2.5)
        L = plan.log
        after = L["t"] >= 1.0
        ok = ok1 and ok2 and recovered(plan)
        out["runs"][f"{-dh * 100:.0f}cm"] = dict(ok=ok, pitch_peak_deg=float(math.degrees(np.max(np.abs(L["pitch"][after])))),
                                                 z_min=float(L["z"].min()), z_end=float(L["z"][-1]))
        if ok:
            out["ok"] = True
            out["max_crouch_cm"] = -dh * 100
            out["pitch_peak_deg"] = out["runs"][f"{-dh * 100:.0f}cm"]["pitch_peak_deg"]
        plan.set_height_offset(0.0)
        if not ok:
            break
    return out


def scenario_link_loss(plan) -> dict:
    if not hasattr(plan, "host"):
        return dict(ok=None, note="no host link in this plan")
    plan.reset()
    plan.run(1.0)
    for _ in range(int(round(1.0 / plan.dt))):
        plan.step(0.5, 0.0)
    plan.host.enabled = False
    ok = plan.run(6.0)
    L = plan.log
    plan.host.enabled = True
    return dict(ok=ok, settled=recovered(plan), v_end=float(L["vx"][-1]), released=not plan.link.in_control,
                events=[(round(t, 3), e) for t, e in plan.link.events])


def scenario_latency(name: str, ms: float, gain_scale: float = 1.0) -> dict:
    plan = make_plan(name, latency_ms=ms, noise=False, gain_scale=gain_scale)
    plan.reset(pitch=math.radians(3))
    plan.run(6.0)
    return dict(ok=recovered(plan), alive=not plan.fallen, latency_ms=ms)


TILTS = [5, 10, 15, 20, 25, 30, 35, 40]
PUSHES = [0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.5, 3.0]
LATENCIES = [0, 5, 10, 15, 20, 30, 40, 60, 80]


def run_task(task: tuple) -> tuple:
    """(plan_name, scenario, arg, gain_scale) -> (key, result); used by the multiprocessing pool."""
    name, scenario, arg, gain_scale = task
    key = (name, scenario, arg, gain_scale)
    try:
        if scenario == "latency":
            return key, scenario_latency(name, arg, gain_scale)
        plan = make_plan(name, gain_scale=gain_scale)
        fn: Dict[str, Callable] = {
            "stand": lambda: scenario_stand(plan), "tilt": lambda: scenario_tilt(plan, arg),
            "push": lambda: scenario_push(plan, arg), "velocity": lambda: scenario_velocity(plan),
            "yaw": lambda: scenario_yaw(plan), "height": lambda: scenario_height(plan),
            "link_loss": lambda: scenario_link_loss(plan),
        }[scenario]
        return key, fn()
    except Exception as exc:  # keep the pool alive, report the failure
        return key, dict(ok=False, error=repr(exc))
