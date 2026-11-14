"""Scenarios and metrics of the acceptance criteria (docs/합격기준.md), used by the v5 staged tests (wbr.bench.stage_tests).

Every scenario runs on a plan (wbr.plans.plan_v5) with sensor noise and latency as the caller set them.  Pitch criteria use
the TRUE body pitch of the simulator.  Check: one pass / fail criterion with its margin.

Until 2026-11-14 this module was also Plan 4's acceptance battery on the upkie model (items 1-24 with upkie's motor limits,
the mass x friction matrix, the stage-2 MPC items, results/acceptance/); the upkie model, the battery and its results
were removed that day (git history).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

G = 9.81
FLAG_METRICS = {"alive", "settled", "motors_on"}     # 0 / 1 pass flags (no margin ratio)
SETTLE_HOLD_S = 1.0          # a recovery counts as settled only if the last violation is >= 1 s before the log ends (r1 m4)


@dataclass
class Check:
    metric: str
    op: str            # "<=" or ">="
    limit: float
    unit: str = ""
    note: str = ""

    def passes(self, value) -> Optional[bool]:
        if value is None or (isinstance(value, float) and math.isnan(value)):
            return False
        return (value <= self.limit) if self.op == "<=" else (value >= self.limit)

    def margin(self, value) -> Optional[float]:
        """measured / limit (<= checks) or limit / measured (>= checks); 1.0 = at the limit.  Pass / fail flags
        (alive, settled, ... >= 1) and zero limits (no saturation, 0 violations) have no ratio: 0 when passed,
        inf when failed (before, a passed flag showed x1.00 and a passed zero limit x inf, hiding the real
        tightest metric of the item)."""
        if value is None:
            return None
        if self.metric in FLAG_METRICS or self.limit == 0:
            return 0.0 if self.passes(value) else float("inf")
        if self.op == "<=":
            return value / self.limit if self.limit else float("inf")
        return (self.limit / value) if value else float("inf")


# ----------------------------------------------------------------------------- metrics helpers
def moving_average(x: np.ndarray, dt: float, win: float) -> np.ndarray:
    n = max(1, int(round(win / dt)))
    k = np.ones(n) / n
    padded = np.pad(x, (n // 2, n - 1 - n // 2), mode="edge")
    return np.convolve(padded, k, mode="valid")


def moving_rms_max(x: np.ndarray, dt: float, win: float = 10.0) -> float:
    n = max(1, int(round(win / dt)))
    if len(x) <= n:
        return float(np.sqrt(np.mean(x ** 2)))
    c = np.cumsum(np.concatenate([[0.0], x ** 2]))
    return float(np.sqrt(np.max((c[n:] - c[:-n]) / n)))


def settle_time_true(L, t_start: float, v_ref: float = 0.0, dt: Optional[float] = None):
    """Last time the 20 ms moving-average TRUE pitch exceeds 1 deg or |v - v_ref| exceeds 0.1 m/s after t_start
    (None = never settled, or the last violation is less than SETTLE_HOLD_S before the end of the log: r1 m4)."""
    t = L["t"]
    dt = dt or float(t[1] - t[0])
    pitch = moving_average(L["pitch_true"], dt, 0.02)
    vx = moving_average(L["vx"], dt, 0.02)
    bad = (np.abs(pitch) > math.radians(1)) | (np.abs(vx - v_ref) > 0.1)
    idx = np.where(bad & (t >= t_start))[0]
    if not len(idx):
        return 0.0
    if idx[-1] >= len(t) - 2 or (t[-1] - t[idx[-1]]) < SETTLE_HOLD_S:
        return None
    return float(t[idx[-1]] - t_start)


def torque_metrics(plan, t_from: float = 0.0) -> dict:
    L = plan.log
    sel = L["t"] >= t_from
    wheels = np.stack([L["tau_lw"][sel], L["tau_rw"][sel]])
    legs = np.stack([L["tau_lh"][sel], L["tau_lk"][sel], L["tau_rh"][sel], L["tau_rk"][sel]])
    wspeed = np.stack([L["qd_lw"][sel], L["qd_rw"][sel]])
    lspeed = np.stack([L["qd_lh"][sel], L["qd_lk"][sel], L["qd_rh"][sel], L["qd_rk"][sel]])
    # per motor (r1 m6): the worst leg joint's rms.  (The wheel saturation duty / time against upkie's 1.7 N m wheel
    # motor were dropped on 2026-11-14; the staged tests judge the peaks against the AKE60-8 instead.)
    return dict(wheel_peak=float(np.abs(wheels).max()),
                wheel_rms10=float(max(moving_rms_max(w, plan.dt) for w in wheels)),
                leg_peak=float(np.abs(legs).max()), leg_rms=float(np.sqrt(np.mean(legs ** 2, axis=1)).max()),
                wheel_speed_peak=float(np.abs(wspeed).max()), leg_speed_peak=float(np.abs(lspeed).max()))


# ----------------------------------------------------------------------------- scenarios
STAND_SKIP_S = 2.0     # the reset transient (robot released 0.5 mm above the floor) is excluded from the stand metrics


def scenario_stand(plan, T: float = 60.0, **_) -> dict:
    plan.reset()
    alive = plan.run(T)
    L = plan.log
    sel = L["t"] >= STAND_SKIP_S
    if sel.sum() < 2:          # fell before the end of the reset window: nothing to judge, every metric fails
        return dict(alive=False, pitch_ma_max=float("inf"), pitch_rms=float("inf"), drift_max=float("inf"),
                    drift_max_from_t0=float(np.abs(L["x"] - L["x"][0]).max()), drift_end=float("inf"), pitch_mean=float("nan"),
                    pitch_ma_max_incl_transient=float("inf"), wheel_peak=float("inf"), wheel_rms10=float("inf"),
                    leg_peak=float("inf"), leg_rms=float("inf"), wheel_speed_peak=float("inf"), leg_speed_peak=float("inf"))
    pitch = L["pitch_true"][sel]
    ma = moving_average(pitch, plan.dt, 0.02)
    x_sel = L["x"][sel]
    out = dict(alive=alive, pitch_ma_max=float(math.degrees(np.abs(ma).max())), pitch_rms=float(math.degrees(np.sqrt(np.mean(pitch ** 2)))),
               # drift (criteria section 3): max |x(t) - x(0)| searched over t >= 2 s (the reset transient is excluded from
               # the SEARCH, not from the reference point -- the 2 s reference point measured 1-2 cm too much on items
               # 16 / 17, review r2 m3).  The old definition is kept as drift_max_from_t2 for comparison.
               drift_max=float(np.abs(x_sel - L["x"][0]).max()), drift_max_from_t0=float(np.abs(L["x"] - L["x"][0]).max()),
               drift_max_from_t2=float(np.abs(x_sel - x_sel[0]).max()),
               drift_end=float(L["x"][-1] - L["x"][0]),
               pitch_mean=float(math.degrees(pitch.mean())),
               pitch_ma_max_incl_transient=float(math.degrees(np.abs(moving_average(L["pitch_true"], plan.dt, 0.02)).max())))
    out.update(torque_metrics(plan, t_from=STAND_SKIP_S))
    return out


def scenario_tilt(plan, deg: float, T: float = 8.0, **_) -> dict:
    plan.reset(pitch=math.radians(deg))
    alive = plan.run(T)
    L = plan.log
    st = settle_time_true(L, L["t"][0]) if alive else None
    out = dict(alive=alive, settled=int(st is not None), settle_s=st, disp=float(np.abs(L["x"] - L["x"][0]).max()),
               disp_end=float(abs(L["x"][-1] - L["x"][0])),
               pitch_peak=float(math.degrees(np.abs(L["pitch_true"]).max())))
    out.update(torque_metrics(plan))
    return out


def scenario_push(plan, frac: float, T: float = 8.0, **_) -> dict:
    plan.reset()
    plan.run(1.0)
    W = plan.sim.total_mass() * G
    plan.push(frac * W, 0.1)
    alive = plan.run(T)
    L = plan.log
    after = L["t"] >= 1.0
    st = settle_time_true(L, 1.0) if alive else None
    out = dict(alive=alive, settled=int(st is not None), settle_s=st, force_N=frac * W,
               pitch_peak=float(math.degrees(np.abs(L["pitch_true"][after]).max())),
               disp=float(np.abs(L["x"][after] - L["x"][after][0]).max()), disp_end=float(abs(L["x"][-1] - L["x"][after][0])),
               v_peak=float(np.abs(L["vx"][after]).max()))
    out.update(torque_metrics(plan, t_from=1.0))
    return out


def scenario_yaw(plan, W: float = 1.0, **_) -> dict:
    plan.reset()
    plan.run(0.5)
    alive = True
    for _ in range(int(round(2.0 / plan.dt))):
        plan.step(0.0, W)
        if plan.fallen:
            alive = False
            break
    L = plan.log
    n = int(round(1.0 / plan.dt))
    yaw_mean = float(np.mean(L["yaw_rate"][-n:]))
    out = dict(alive=alive, yaw_rate_mean=yaw_mean, yaw_err_pct=float(abs(yaw_mean - W) / W * 100),
               pitch_peak=float(math.degrees(np.abs(L["pitch_true"]).max())))
    out.update(torque_metrics(plan, t_from=0.5))
    return out


def scenario_height(plan, dh: float = -0.04, **_) -> dict:
    plan.reset()
    plan.run(1.0)
    plan.set_height_offset(dh)
    ok1 = plan.run(2.5)
    plan.set_height_offset(0.0)
    ok2 = ok1 and plan.run(2.5)
    L = plan.log
    after = L["t"] >= 1.0
    out = dict(alive=ok1 and ok2, pitch_peak=float(math.degrees(np.abs(L["pitch_true"][after]).max())),
               L0_min=float(L["L0"].min()), settled=int(ok2 and settle_time_true(L, 3.5) is not None))
    out.update(torque_metrics(plan, t_from=1.0))
    return out
