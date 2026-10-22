"""Plan 4 acceptance battery: automatic pass / fail against docs/합격기준.md.

    python -m wbr.bench.acceptance [--rate 1000|200|both] [--matrix full|reduced|nominal] [--seeds 5]
                                   [--host ramp|mpc] [--reference] [--jobs N] [--no-cache] [--tag NAME]
                                   [--items 1 2 ...]

Every scenario runs with sensor noise (pitch 0.2 deg, gyro 0.5 deg/s, 14-bit encoders) and a 10 ms
observation latency.  "settle" items run over the mass x friction matrix and all seeds, "survive" and
"record" items at the nominal combination only.  Pitch criteria use the TRUE body pitch of the
simulator (the logged estimate contains the 0.2 deg sensor noise).  Results: results/acceptance/
plan4[_mpc]_<rate>Hz.md / .json; per-run results are cached under results/acceptance/cache/<tag>/.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import time
from dataclasses import dataclass, field
from multiprocessing import Pool, cpu_count
from pathlib import Path
from typing import Callable, Dict, List, Optional

import numpy as np

from wbr import RESULTS_DIR

G = 9.81
LATENCY_MS = 10.0
WHEEL_TAU_MAX, LEG_TAU_MAX = 1.7, 16.0
WHEEL_PEAK_LIMIT, WHEEL_RMS_LIMIT = 0.8 * WHEEL_TAU_MAX, 0.2          # 1.36 Nm, 0.2 Nm (50 % of ~0.4 Nm continuous)
LEG_PEAK_LIMIT, LEG_RMS_LIMIT = 0.8 * LEG_TAU_MAX, 4.0
WHEEL_SPEED_LIMIT, LEG_SPEED_LIMIT = 0.8 * 111.0, 0.8 * 28.8
SAT_DUTY_LIMIT = 0.05

NOMINAL = (1.0, 0.75)
MATRIX_FULL = [(ms, mu) for ms in (0.85, 1.0, 1.15) for mu in (0.5, 0.75, 1.0)]
MATRIX_REDUCED = [(1.0, 0.75), (0.85, 0.5), (1.15, 1.0)]


# ----------------------------------------------------------------------------- criteria table
FLAG_METRICS = {"alive", "settled", "motors_on", "fallback_used", "passes_1_8_11", "passes_8_transient", "mpc_resumed",
                "model_ok"}     # 0 / 1 pass flags (no margin ratio)
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


@dataclass
class Item:
    id: int
    name: str
    grade: str                    # "settle" | "survive" | "record" | "note"
    scenario: str
    params: dict = field(default_factory=dict)
    checks: List[Check] = field(default_factory=list)
    torque_margin: bool = True    # apply the wheel / leg torque-margin checks
    wheel_peak_check: bool = True  # False where the criteria allow saturation ("포화 허용")
    duration_s: float = 8.0


TORQUE_CHECKS = [
    Check("wheel_peak", "<=", WHEEL_PEAK_LIMIT, "Nm", "바퀴 피크 ≤ 80 %"),
    Check("wheel_rms10", "<=", WHEEL_RMS_LIMIT, "Nm", "바퀴 10 s 이동 rms"),
    Check("wheel_sat_duty", "<=", SAT_DUTY_LIMIT, "", "포화 지속 비율"),
    Check("leg_peak", "<=", LEG_PEAK_LIMIT, "Nm", "다리 피크 ≤ 80 %"),
    Check("leg_rms", "<=", LEG_RMS_LIMIT, "Nm", "다리 rms"),
    Check("wheel_speed_peak", "<=", WHEEL_SPEED_LIMIT, "rad/s", "바퀴 속도 ≤ 80 %"),
    Check("leg_speed_peak", "<=", LEG_SPEED_LIMIT, "rad/s", "다리 속도 ≤ 80 %"),
]

STAND_CHECKS = [
    Check("alive", ">=", 1, "", "생존(2 s 이후 전복 포함)"),
    Check("pitch_ma_max", "<=", 1.0, "deg", "20 ms 이동평균 |피치|"),
    Check("pitch_rms", "<=", 0.5, "deg"),
    Check("drift_max", "<=", 0.05, "m", "max |x − x0|"),
    Check("wheel_rms10", "<=", WHEEL_RMS_LIMIT, "Nm"),
    Check("wheel_sat_duty", "<=", 0.0, "", "포화 없음"),
]

ITEMS: List[Item] = [
    Item(1, "정지 60 s", "settle", "stand", dict(T=60.0), STAND_CHECKS, duration_s=60),
    Item(2, "초기 기울기 10°", "settle", "tilt", dict(deg=10), [Check("settled", ">=", 1, "", "정착"), Check("settle_s", "<=", 2.0, "s"),
                                                            Check("disp", "<=", 0.3, "m")]),
    Item(3, "초기 기울기 15°", "settle", "tilt", dict(deg=15), [Check("settled", ">=", 1), Check("settle_s", "<=", 3.0, "s"),
                                                            Check("disp", "<=", 0.5, "m"), Check("wheel_sat_s", "<=", 0.3, "s", "포화 지속")],
         wheel_peak_check=False),
    Item(4, "초기 기울기 20°", "survive", "tilt", dict(deg=20), [Check("alive", ">=", 1)], torque_margin=False),
    Item(5, "밀기 0.5 W", "settle", "push", dict(frac=0.5), [Check("settled", ">=", 1), Check("settle_s", "<=", 3.0, "s"),
                                                         Check("pitch_peak", "<=", 8.0, "deg"), Check("disp", "<=", 0.5, "m")]),
    Item(6, "밀기 0.75 W", "settle", "push", dict(frac=0.75), [Check("settled", ">=", 1), Check("settle_s", "<=", 4.0, "s"),
                                                           Check("pitch_peak", "<=", 12.0, "deg")], wheel_peak_check=False),
    Item(7, "밀기 1.0 W", "survive", "push", dict(frac=1.0), [Check("alive", ">=", 1)], torque_margin=False),
    Item(8, "속도 스텝 0.5 m/s", "settle", "velocity", dict(V=0.5, slope=0.0),
         [Check("rise_s", "<=", 1.5, "s", "90 % 도달"), Check("overshoot_pct", "<=", 20.0, "%"), Check("ss_err", "<=", 0.05, "m/s", "1 s 평균"),
          Check("pitch_peak", "<=", 5.0, "deg"), Check("stop_disp", "<=", 0.5, "m"), Check("settled", ">=", 1)]),
    Item(9, "요 1 rad/s", "settle", "yaw", dict(W=1.0), [Check("yaw_err_pct", "<=", 10.0, "%", "마지막 1 s 평균"), Check("pitch_peak", "<=", 2.0, "deg"),
                                                     Check("alive", ">=", 1)]),
    Item(10, "4 cm 웅크림 후 복귀", "settle", "height", dict(dh=-0.04), [Check("alive", ">=", 1), Check("pitch_peak", "<=", 3.0, "deg"),
                                                                    Check("settled", ">=", 1)]),
    Item(11, "링크 끊김(0.5 m/s 주행 중)", "settle", "link_loss", dict(V=0.5), [Check("alive", ">=", 1), Check("stop_time", "<=", 3.0, "s", "|v| < 0.1"),
                                                                           Check("motors_on", ">=", 1), Check("settled", ">=", 1)]),
    Item(12, "경사 5° 정지", "settle", "slope_stand", dict(T=60.0, slope=5.0),
         [Check("alive", ">=", 1), Check("disp_max", "<=", 0.10, "m"), Check("pitch_dev_ma_max", "<=", 1.0, "deg", "평형 오프셋 제거 후"),
          Check("pitch_dev_rms", "<=", 0.5, "deg"), Check("wheel_rms10", "<=", WHEEL_RMS_LIMIT, "Nm"),
          Check("wheel_sat_duty", "<=", 0.0, "", "포화 없음")], duration_s=60),
    Item(13, "경사 5° 주행 0.5 m/s(오르기·내리기)", "settle", "slope_drive", dict(V=0.5, slope=5.0),
         [Check("rise_s", "<=", 1.5, "s"), Check("overshoot_pct", "<=", 20.0, "%"), Check("ss_err", "<=", 0.05, "m/s"),
          Check("pitch_peak", "<=", 5.0, "deg"), Check("stop_disp", "<=", 0.5, "m"), Check("settled", ">=", 1)]),
    Item(15, "지연 스윕(3° 기울기, 무노이즈; 강건 여유도 함께 기록)", "record", "latency_sweep", dict(),
         [Check("max_latency_ms", ">=", 25.0, "ms", "3° 기울기 정착(기준)"),
          Check("max_latency_robust_ms", ">=", 0.0, "ms", "노이즈·2 mm 낙하·0.5 W 밀기 20 s, 마지막 5 s 피치 rms < 0.5°(기록, 검토 r1 M1)")],
         torque_margin=False),
    Item(16, "IMU 피치 바이어스 +1°", "settle", "stand", dict(T=60.0, pitch_bias_deg=1.0), STAND_CHECKS, duration_s=60),
    Item(17, "무게중심 전방 오프셋(0.3 kg, +10 cm)", "settle", "stand", dict(T=60.0, payload=(0.3, 0.10, None)), STAND_CHECKS, duration_s=60),
    # review r1 (stage 2 M1): the criteria's disturbance items all run with the target exactly 0 (the host idle); this
    # record-only item pushes the robot while it cruises so that the stage-2 host (telemetry correction) and the
    # stage-1 ramp host can be compared.  Not part of the user's criteria: it never flips the verdict.
    Item(24, "주행 중 밀기(0.3 m/s 순항, 0.75 W; 기록, 검토 r1)", "record", "drive_push", dict(V=0.3, frac=0.75),
         [Check("alive", ">=", 1), Check("settled", ">=", 1), Check("settle_s", "<=", 4.0, "s", "6번과 같은 4 s(참고)"),
          Check("v_back", "<=", 0.1, "m/s", "후진 속도(참고)")], torque_margin=False),
]

STAGE2_ITEMS: List[Item] = [
    Item(19, "MPC 속도 스텝 0.5 m/s", "settle", "velocity", dict(V=0.5, slope=0.0),
         [Check("pitch_peak", "<=", 2.0, "deg"), Check("overshoot_pct", "<=", 10.0, "%"), Check("rise_s", "<=", 1.5, "s"),
          Check("stop_disp", "<=", 0.3, "m"), Check("settled", ">=", 1)]),
    Item(21, "MPC 제약 준수(8·9·13·19)", "settle", "mpc_constraints", dict(),
         [Check("constraint_violations", "<=", 0, "", "가속도·속도 제약 위반(QP 해 자체 검사)"),
          Check("packet_violations", "<=", 0, "", "송신 패킷의 가속도·저크 위반(독립 계수, 검토 r1 m4)"),
          Check("wheel_peak", "<=", WHEEL_PEAK_LIMIT, "Nm"), Check("alive", ">=", 1)], torque_margin=False),
    Item(22, "MPC 풀이 시간", "record", "mpc_timing", dict(), [Check("solve_p99_ms", "<=", 5.0, "ms"), Check("qp_failures", "<=", 0, "")],
         torque_margin=False),
    Item(23, "MPC 폴백(솔버 강제 실패)", "survive", "mpc_fallback", dict(),
         [Check("alive", ">=", 1), Check("fallback_used", ">=", 1), Check("passes_1_8_11", ">=", 1, "", "1·8·11 통과"),
          Check("passes_8_transient", ">=", 1, "", "기동 중 0.1 s QP 실패(0.8·1.2·3.04 s) 뒤에도 8번 통과(검토 r1 M2)"),
          Check("mpc_resumed", ">=", 1, "", "일시 실패 뒤 MPC 복귀")],
         torque_margin=False),
]


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
    sat = np.abs(wheels) >= 0.999 * WHEEL_TAU_MAX
    # per motor (r1 m6): the worst wheel's saturation duty, the worst leg joint's rms
    return dict(wheel_peak=float(np.abs(wheels).max()),
                wheel_rms10=float(max(moving_rms_max(w, plan.dt) for w in wheels)),
                wheel_sat_duty=float(sat.mean(axis=1).max()), wheel_sat_s=float(sat.any(axis=0).sum() * plan.dt),
                leg_peak=float(np.abs(legs).max()), leg_rms=float(np.sqrt(np.mean(legs ** 2, axis=1)).max()),
                wheel_speed_peak=float(np.abs(wspeed).max()), leg_speed_peak=float(np.abs(lspeed).max()))


# ----------------------------------------------------------------------------- plan factory
def make_plan(rate: float, seed: int, mass_scale: float, friction: float, host: str = "ramp", **extra):
    from wbr.plans.plan4_upkie_lqi6 import Plan4Config, Plan4UpkieLQI6
    cfg = Plan4Config(rate_hz=rate, noise=True, latency_ms=LATENCY_MS, seed=seed, mass_scale=mass_scale, friction=friction,
                      host=host, **extra)
    return Plan4UpkieLQI6(cfg)


def make_reference_plan(seed: int, **extra):
    """upkie's own PI balancer on the same hardware (Plan 1, 1 kHz): shown as a reference column, not judged."""
    from wbr.plans.plan1_upkie import Plan1Config, Plan1Upkie
    plan = Plan1Upkie(Plan1Config(balancer="pid", noise=True, latency_ms=LATENCY_MS, seed=seed))
    _augment_reference_log(plan)
    return plan


def _augment_reference_log(plan) -> None:
    """Plan 1 logs fewer signals than Plan 4 (no true pitch, no right-leg torques, no joint velocities, no leg length),
    so every reference run ended in KeyError('pitch_true').  The missing judged signals are added from the simulator
    after each step; Plan 1 itself is not modified."""
    import mujoco
    from wbr.common.rotations import quat_to_matrix, roll_pitch_yaw
    sim, base_step = plan.sim, plan.step
    m, d = sim.model, sim.data
    hip_j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "left_hip")
    axle_b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "left_wheel_tire")
    short = {"left_hip": "lh", "left_knee": "lk", "left_wheel": "lw", "right_hip": "rh", "right_knee": "rk", "right_wheel": "rw"}

    def step(target_ground_velocity: float = 0.0, target_yaw_velocity: float = 0.0):
        o = base_step(target_ground_velocity, target_yaw_velocity)
        have = plan.log.rows
        extra = {"pitch_true": roll_pitch_yaw(quat_to_matrix(d.xquat[sim.base_bid]))[1],
                 "L0": float(np.hypot(*(d.xanchor[hip_j][[0, 2]] - d.xpos[axle_b][[0, 2]])))}
        for n, k in short.items():
            if f"tau_{k}" not in have or len(have[f"tau_{k}"]) < len(have["t"]):
                extra[f"tau_{k}"] = float(d.actuator_force[sim.act_id[n]])
            extra[f"qd_{k}"] = float(d.qvel[sim.dof_adr[n]])
        plan.log.log(**extra)
        return o
    plan.step = step


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
                    pitch_ma_max_incl_transient=float("inf"), wheel_peak=float("inf"), wheel_rms10=float("inf"), wheel_sat_duty=1.0,
                    wheel_sat_s=float("inf"), leg_peak=float("inf"), leg_rms=float("inf"), wheel_speed_peak=float("inf"),
                    leg_speed_peak=float("inf"))
    pitch = L["pitch_true"][sel]
    ma = moving_average(pitch, plan.dt, 0.02)
    x_sel = L["x"][sel]
    out = dict(alive=alive, pitch_ma_max=float(math.degrees(np.abs(ma).max())), pitch_rms=float(math.degrees(np.sqrt(np.mean(pitch ** 2)))),
               # drift: largest excursion from the position at the end of the reset transient (same 2 s exclusion as the
               # pitch metrics; the excursion from t = 0 is kept as drift_max_from_t0 for information)
               drift_max=float(np.abs(x_sel - x_sel[0]).max()), drift_max_from_t0=float(np.abs(L["x"] - L["x"][0]).max()),
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


def _velocity_profile(plan, V: float, hook=None) -> dict:
    """0.5 s stand, 2.5 s at V, 3 s at 0 (same timing as the benchmark battery).  hook(t) is called before every
    step (acceptance_mpc: transient QP failure injection)."""
    plan.reset()
    plan.run(0.5)
    alive = True
    for _ in range(int(round(2.5 / plan.dt))):
        if hook is not None:
            hook(plan.sim.time)
        plan.step(V, 0.0)
        if plan.fallen:
            alive = False
            break
    if alive:
        for _ in range(int(round(3.0 / plan.dt))):
            if hook is not None:
                hook(plan.sim.time)
            plan.step(0.0, 0.0)
            if plan.fallen:
                alive = False
                break
    L = plan.log
    t, v = L["t"], L["vx"]
    seg = (t >= 0.5) & (t < 3.0)
    rise = None
    sgn = 1.0 if V >= 0 else -1.0
    if len(v[seg]) and (sgn * v[seg]).max() > 0.9 * abs(V):
        rise = float(t[seg][np.argmax(sgn * v[seg] > 0.9 * abs(V))] - 0.5)
    ss = (t >= 2.0) & (t < 3.0)
    out = dict(alive=alive, rise_s=rise, overshoot_pct=float(((sgn * v[seg]).max() - abs(V)) / abs(V) * 100) if len(v[seg]) else None,
               ss_err=float(abs(np.mean(v[ss]) - V)) if ss.any() else None,
               pitch_peak=float(math.degrees(np.abs(L["pitch_true"][t >= 0.5]).max())),
               # stopping distance = largest excursion after the stop command (r1 M3; the end point hid a 0.2 m
               # roll-back), stop_back = how far the robot came back from that point (record)
               stop_disp=float(np.abs(L["x"][t >= 3.0] - L["x"][t >= 3.0][0]).max()) if (t >= 3.0).any() else None,
               stop_end=float(abs(L["x"][-1] - L["x"][t >= 3.0][0])) if (t >= 3.0).any() else None,
               stop_back=float(np.abs(L["x"][t >= 3.0] - L["x"][t >= 3.0][0]).max() - abs(L["x"][-1] - L["x"][t >= 3.0][0]))
               if (t >= 3.0).any() else None,
               settled=int(alive and settle_time_true(L, 3.0) is not None))
    out.update(torque_metrics(plan, t_from=0.5))
    return out


def scenario_velocity(plan, V: float = 0.5, **_) -> dict:
    return _velocity_profile(plan, V)


def scenario_slope_drive(plan, V: float = 0.5, **_) -> dict:
    """Uphill then downhill on the inclined floor; worst case of the two for every metric."""
    up = _velocity_profile(plan, V)
    down = _velocity_profile(plan, -V)
    out = {}
    for k in set(up) | set(down):
        a, b = up.get(k), down.get(k)
        if k in ("alive", "settled"):
            out[k] = int(bool(a) and bool(b))
        elif a is None or b is None:
            out[k] = None
        else:
            out[k] = max(a, b)
    out["uphill"], out["downhill"] = {k: v for k, v in up.items()}, {k: v for k, v in down.items()}
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


def scenario_link_loss(plan, V: float = 0.5, **_) -> dict:
    plan.reset()
    plan.run(1.0)
    for _ in range(int(round(3.0 / plan.dt))):       # cruise at V before the link drops (r1 m5: 1 s reached only 0.38 m/s)
        plan.step(V, 0.0)
    plan.host.enabled = False
    t_drop = plan.sim.time
    alive = plan.run(6.0)
    L = plan.log
    after = L["t"] >= t_drop
    slow = np.abs(moving_average(L["vx"], plan.dt, 0.02)) < 0.1
    # first time after the drop from which |v| stays below 0.1 m/s
    idx = np.where(after)[0]
    stop_time = None
    for i in idx:
        if slow[i:].all():
            stop_time = float(L["t"][i] - t_drop)
            break
    out = dict(alive=alive, stop_time=stop_time, v_end=float(L["vx"][-1]), released=int(not plan.link.in_control),
               motors_on=int(plan.mcu.armed and not plan.mcu.off_latched), settled=int(alive and settle_time_true(L, t_drop) is not None))
    out.update(torque_metrics(plan, t_from=t_drop))
    plan.host.enabled = True
    return out


def scenario_slope_stand(plan, T: float = 60.0, **_) -> dict:
    plan.reset()
    alive = plan.run(T)
    L = plan.log
    pitch = L["pitch_true"]
    n10 = int(round(10.0 / plan.dt))
    offset = float(pitch[-n10:].mean())
    dev = pitch - offset
    ma = moving_average(dev, plan.dt, 0.02)
    out = dict(alive=alive, disp_max=float(np.abs(L["x"] - L["x"][0]).max()), pitch_offset_deg=math.degrees(offset),
               pitch_dev_ma_max=float(math.degrees(np.abs(ma[int(5.0 / plan.dt):]).max())),   # skip the 5 s initial transient
               pitch_dev_rms=float(math.degrees(np.sqrt(np.mean(dev[int(5.0 / plan.dt):] ** 2)))))
    out.update(torque_metrics(plan))
    return out


def scenario_latency_sweep(rate: float, host: str = "ramp") -> dict:
    """Item 15 (record).  Criterion: 3 deg tilt, no noise, nominal friction, latency 0..80 ms -> the largest latency
    with a settled recovery, stopping at the first failure (monotonic reading, r1 n6).  Also recorded (r1 M1): the
    ROBUST margin -- noise on, 2 mm drop start, 0.5 W push at 1 s, 20 s -- the largest latency with no fall, a settled
    recovery and a quiet tail (last 5 s true-pitch rms < 0.5 deg), which catches the large-amplitude limit cycle the
    3 deg test cannot see."""
    from wbr.plans.plan4_upkie_lqi6 import Plan4Config, Plan4UpkieLQI6
    results, best = {}, None
    for ms in (0, 5, 10, 15, 20, 25, 30, 40, 60, 80):
        plan = Plan4UpkieLQI6(Plan4Config(rate_hz=rate, noise=False, latency_ms=float(ms), host=host, friction=NOMINAL[1]))
        plan.reset(pitch=math.radians(3))
        alive = plan.run(6.0)
        st = settle_time_true(plan.log, 0.0) if alive else None
        results[ms] = dict(alive=alive, settled=st is not None, settle_s=st)
        if st is not None:
            best = ms
        else:
            break
    robust, robust_runs = None, {}
    for ms in (0, 5, 8, 10, 12, 15, 20, 25, 30, 40):
        plan = Plan4UpkieLQI6(Plan4Config(rate_hz=rate, noise=True, latency_ms=float(ms), host=host, friction=NOMINAL[1],
                                          drop_height=0.002))
        plan.reset()
        alive = plan.run(1.0)
        if alive:
            plan.push(0.5 * plan.sim.total_mass() * G, 0.1)
            alive = plan.run(19.0)
        L = plan.log
        st = settle_time_true(L, 1.0) if alive else None
        n5 = int(5.0 / plan.dt)
        tail_rms = float(math.degrees(np.sqrt(np.mean(L["pitch_true"][-n5:] ** 2)))) if alive else None
        ok = bool(alive and st is not None and tail_rms < 0.5)
        robust_runs[ms] = dict(alive=alive, settled=st is not None, settle_s=st, tail_pitch_rms_deg=tail_rms, ok=ok)
        if ok:
            robust = ms
        else:
            break
    return dict(max_latency_ms=best, max_latency_robust_ms=robust, runs=results, robust_runs=robust_runs, alive=True)


def scenario_drive_push(plan, V: float = 0.3, frac: float = 0.75, T: float = 6.0, **_) -> dict:
    """Item 24 (record, review r1 stage 2 M1): push while cruising at V with the host still commanding V.  Settle time
    relative to V after the push, largest backward speed, excursion."""
    plan.reset()
    alive = plan.run(0.5)
    if alive:
        alive = plan.run(2.5, v=V)
    t_push = plan.sim.time
    if alive:
        plan.push(frac * plan.sim.total_mass() * G, 0.1)
        alive = plan.run(T, v=V)
    L = plan.log
    after = L["t"] >= t_push
    st = settle_time_true(L, t_push, v_ref=V) if alive else None
    sgn = 1.0 if V >= 0 else -1.0
    out = dict(alive=alive, settled=int(st is not None), settle_s=st,
               v_back=float(max(0.0, -(sgn * L["vx"][after]).min())) if after.any() else None,
               pitch_peak=float(math.degrees(np.abs(L["pitch_true"][after]).max())) if after.any() else None,
               disp=float(np.abs(L["x"][after] - L["x"][after][0]).max()) if after.any() else None)
    out.update(torque_metrics(plan, t_from=t_push))
    return out


SCENARIOS: Dict[str, Callable] = {"drive_push": scenario_drive_push, "stand": scenario_stand, "tilt": scenario_tilt, "push": scenario_push, "velocity": scenario_velocity,
                                  "yaw": scenario_yaw, "height": scenario_height, "link_loss": scenario_link_loss,
                                  "slope_stand": scenario_slope_stand, "slope_drive": scenario_slope_drive}


# ----------------------------------------------------------------------------- tasks
@dataclass(frozen=True)
class Task:
    item_id: int
    rate: float
    mass_scale: float
    friction: float
    seed: int
    host: str = "ramp"
    reference: bool = False


def _item(item_id: int) -> Item:
    for it in ITEMS + STAGE2_ITEMS:
        if it.id == item_id:
            return it
    raise KeyError(item_id)


def run_task(task: Task) -> dict:
    it = _item(task.item_id)
    t0 = time.time()
    try:
        if it.scenario == "latency_sweep":
            res = scenario_latency_sweep(task.rate, task.host)
        elif it.scenario in ("mpc_timing", "mpc_fallback", "mpc_constraints"):
            from wbr.bench.acceptance_mpc import run_mpc_item
            res = run_mpc_item(it, task)
        else:
            extra = {k: v for k, v in it.params.items() if k in ("pitch_bias_deg", "payload")}
            if "slope" in it.params and it.params["slope"]:
                extra["slope_deg"] = it.params["slope"]
            if task.reference:
                plan = make_reference_plan(task.seed)
            else:
                plan = make_plan(task.rate, task.seed, task.mass_scale, task.friction, task.host, **extra)
            params = {k: v for k, v in it.params.items() if k not in ("pitch_bias_deg", "payload", "slope", "constraint_log")}
            res = SCENARIOS[it.scenario](plan, **params)
            if it.params.get("constraint_log") and hasattr(plan, "host") and hasattr(plan.host, "violations"):
                res["constraint_violations"] = int(plan.host.violations)
    except Exception as exc:  # keep the pool alive
        res = dict(alive=False, error=repr(exc))
    res["wall_s"] = time.time() - t0
    return res


def evaluate(it: Item, res: dict) -> dict:
    checks = list(it.checks)
    if it.torque_margin and it.grade == "settle":
        for c in TORQUE_CHECKS:
            if c.metric == "wheel_peak" and not it.wheel_peak_check:
                continue
            if any(c.metric == k.metric for k in checks):
                continue
            checks.append(c)
    out = {}
    for c in checks:
        v = res.get(c.metric)
        out[c.metric] = dict(value=v, limit=c.limit, op=c.op, passed=bool(c.passes(v)) if "error" not in res else False, margin=c.margin(v))
    return out


# ----------------------------------------------------------------------------- cache
def code_hash() -> str:
    """Hash of every source file of the wbr package and of the Plan 4 model directory (r1 m8: a model-XML change
    alone must invalidate the cache).  Viewer / video / plotting modules are excluded."""
    root = Path(__file__).resolve().parents[1]
    h = hashlib.sha1()
    skip = ("viewer", "video.py", "plots.py", "run_bench.py", "battery.py", "compare_hosts.py", "regression.py")
    for p in sorted(root.rglob("*.py")):
        rel = p.relative_to(root).as_posix()
        if any(s in rel for s in skip) or "__pycache__" in rel:
            continue
        h.update(rel.encode()); h.update(p.read_bytes())
    for p in sorted((root.parent / "models" / "upkie_plan4").glob("*")):
        if p.is_file() and p.suffix in (".xml", ".json"):
            h.update(p.name.encode()); h.update(p.read_bytes())
    return h.hexdigest()[:10]


def task_key(task: Task, chash: str) -> str:
    return hashlib.sha1(json.dumps([task.__dict__, chash], sort_keys=True).encode()).hexdigest()[:16]


# ----------------------------------------------------------------------------- reporting
def fmt(v, nd=2):
    if v is None:
        return "-"
    if isinstance(v, bool):
        return "O" if v else "X"
    if isinstance(v, (int, np.integer)):
        return str(int(v))
    if isinstance(v, float):
        return f"{v:.{nd}f}"
    return str(v)


def regression_row(summary: dict, stage1: dict) -> dict:
    """Item 20 of the criteria (r1 m7, computed automatically): every stage-1 item 1-17 must still pass with the MPC
    host, and the margin of the item's tightest metric must not be more than 10 % worse than in plan4_<rate>Hz.json."""
    s1 = {it["id"]: it for it in stage1.get("items", [])}
    flips, worse, compared = [], [], 0

    def tightest(it):
        ms = [w["margin"] for w in it["worst"].values() if w["margin"] is not None and math.isfinite(w["margin"])]
        return max(ms) if ms else None

    for it in summary["items"]:
        ref = s1.get(it["id"])
        if ref is None or it["id"] >= 19 or it["grade"] not in ("settle", "survive"):
            continue
        compared += 1
        if ref["passed"] and not it["passed"]:
            flips.append(it["id"])
        m1, m2 = tightest(ref), tightest(it)
        if m1 is not None and m2 is not None and m1 > 0 and m2 > 1.10 * m1:
            worse.append(dict(id=it["id"], stage1=m1, stage2=m2))
    worst = {"regression_flips": dict(margin=float("inf") if flips else 0.0, value=len(flips), limit=0, op="<=", passed=not flips,
                                     case=("항목 " + ", ".join(map(str, flips))) if flips else "-"),
             "margin_worse_10pct": dict(margin=float("inf") if worse else 0.0, value=len(worse), limit=0, op="<=", passed=not worse,
                                        case=", ".join(f"{w['id']}: ×{w['stage1']:.2f}→×{w['stage2']:.2f}" for w in worse) or "-")}
    failures = [dict(case=f"항목 {w['id']}", metric="margin_worse_10pct", value=w["stage2"], limit=1.1 * w["stage1"], op="<=") for w in worse]
    failures += [dict(case=f"항목 {i}", metric="regression_flips", value=1, limit=0, op="<=") for i in flips]
    return dict(id=20, name="회귀 없음(1~17: 1단계 통과 유지, 가장 빠듯한 지표 여유율 악화 ≤ 10 %)", grade="settle",
                passed=not flips and not worse, n_runs=compared, worst=worst, failures=failures, errors=[], reference=None)


def summarize(items: List[Item], runs: Dict[Task, dict], rate: float, reference: Dict[int, dict] | None,
              stage1: dict | None = None) -> dict:
    summary = {"rate_hz": rate, "items": [], "passed": True}
    for it in items:
        tasks = [t for t in runs if t.item_id == it.id and t.rate == rate and not t.reference]
        if not tasks:
            continue
        evals = {t: evaluate(it, runs[t]) for t in tasks}
        item_pass = all(all(c["passed"] for c in ev.values()) for ev in evals.values()) and all("error" not in runs[t] for t in tasks)
        worst = {}
        for t, ev in evals.items():
            for metric, c in ev.items():
                m = c["margin"]
                if m is None:
                    m = float("inf") if not c["passed"] else 0.0
                if metric not in worst or m > worst[metric]["margin"]:
                    worst[metric] = dict(margin=m, value=c["value"], limit=c["limit"], op=c["op"], passed=c["passed"],
                                         case=f"m×{t.mass_scale:.2f} μ{t.friction:.2f} s{t.seed}")
        failures = [dict(case=f"m×{t.mass_scale:.2f} μ{t.friction:.2f} seed {t.seed}", metric=metric, value=c["value"], limit=c["limit"], op=c["op"])
                    for t, ev in evals.items() for metric, c in ev.items() if not c["passed"]]
        errors = [dict(case=f"m×{t.mass_scale:.2f} μ{t.friction:.2f} seed {t.seed}", error=runs[t]["error"]) for t in tasks if "error" in runs[t]]
        summary["items"].append(dict(id=it.id, name=it.name, grade=it.grade, passed=item_pass, n_runs=len(tasks), worst=worst,
                                     failures=failures, errors=errors, reference=(reference or {}).get(it.id)))
        # docs/합격기준.md 5: only the required grades (필수-정착 / 필수-생존) decide the verdict; "record" items (15)
        # keep their own pass / fail in the tables but do not flip the overall result
        if it.grade in ("settle", "survive") and not item_pass:
            summary["passed"] = False
    if stage1 is not None:                           # stage 2: item 20 from the stage-1 summary of the same rate
        row = regression_row(summary, stage1)
        summary["items"].append(row)
        summary["items"].sort(key=lambda it: it["id"])
        if not row["passed"]:
            summary["passed"] = False
    return summary


GRADE_KO = {"settle": "필수-정착", "survive": "필수-생존", "record": "기록", "note": "명시"}


def summary_markdown(summary: dict, matrix_name: str, seeds: int, host: str) -> str:
    lines = [f"# Plan 4 합격 판정 — {summary['rate_hz']:.0f} Hz, 호스트 {host}, 매트릭스 {matrix_name}, 시드 {seeds}개",
             "", f"**종합: {'합격' if summary['passed'] else '불합격'}**  (노이즈 ON, 지연 {LATENCY_MS:.0f} ms, 판정 피치 = 시뮬 실제 피치; "
                 "종합 판정은 필수 항목만, 기록 항목은 자체 통과/실패만 표시)", "",
             "| # | 항목 | 등급 | 판정 | 실행 수 | 가장 빠듯한 지표 (측정 / 기준, 여유율) | 최악 조합 |", "|---|---|---|---|---|---|---|"]
    for it in summary["items"]:
        w = it["worst"]
        if w:
            metric, info = max(w.items(), key=lambda kv: kv[1]["margin"] if kv[1]["margin"] is not None else 0)
            tight = f"{metric}: {fmt(info['value'], 3)} {info['op']} {fmt(info['limit'], 3)} (×{fmt(info['margin'])})"
            case = info["case"]
        else:
            tight, case = "-", "-"
        ref = it.get("reference")
        lines.append(f"| {it['id']} | {it['name']} | {GRADE_KO[it['grade']]} | {'통과' if it['passed'] else '**실패**'} | {it['n_runs']} | {tight} | {case} |")
    fails = [(it, f) for it in summary["items"] for f in it["failures"]]
    if fails:
        lines += ["", "## 실패 목록", "", "| # | 항목 | 조합 | 지표 | 측정 | 기준 |", "|---|---|---|---|---|---|"]
        for it, f in fails[:200]:
            lines.append(f"| {it['id']} | {it['name']} | {f['case']} | {f['metric']} | {fmt(f['value'], 3)} | {f['op']} {fmt(f['limit'], 3)} |")
        if len(fails) > 200:
            lines.append(f"| … | {len(fails) - 200}건 더 | | | | |")
    errs = [(it, e) for it in summary["items"] for e in it["errors"]]
    if errs:
        lines += ["", "## 오류", ""] + [f"- {it['id']} {it['name']} {e['case']}: `{e['error']}`" for it, e in errs]
    lines += ["", "## 항목별 전체 지표(최악값)", ""]
    for it in summary["items"]:
        lines.append(f"- **{it['id']} {it['name']}**: " + ", ".join(f"{m} {fmt(i['value'], 3)}{'' if i['passed'] else ' ✗'}" for m, i in it["worst"].items()))
        if it.get("reference"):
            r = it["reference"]
            lines.append("  - 참고(plan1_pid, upkie 기본 PI, 1 kHz, 공칭): " + ", ".join(f"{k} {fmt(v, 3)}" for k, v in r.items()
                                                                         if isinstance(v, (int, float)) and k not in ("wall_s",)))
    return "\n".join(lines) + "\n"


# ----------------------------------------------------------------------------- main
def build_tasks(items: List[Item], rates, matrix, seeds, host: str) -> List[Task]:
    tasks = []
    for it in items:
        for rate in rates:
            if it.grade == "settle":
                for ms, mu in matrix:
                    for s in seeds:
                        tasks.append(Task(it.id, rate, ms, mu, s, host))
            elif it.grade == "survive":            # r1 m1: must-survive items on every noise seed (nominal combination)
                for s in seeds:
                    tasks.append(Task(it.id, rate, NOMINAL[0], NOMINAL[1], s, host))
            else:                                   # record items: deterministic sweeps / timing, one run
                tasks.append(Task(it.id, rate, NOMINAL[0], NOMINAL[1], 0, host))
    return tasks


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rate", default="both", choices=("1000", "200", "both"))
    ap.add_argument("--matrix", default="full", choices=("full", "reduced", "nominal"))
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--host", default="ramp", choices=("ramp", "mpc"))
    ap.add_argument("--stage2", action="store_true", help="add the stage-2 MPC items 19-23")
    ap.add_argument("--reference", action="store_true", help="also run plan1_pid at nominal as a reference column")
    ap.add_argument("--jobs", type=int, default=max(1, cpu_count() - 2))
    ap.add_argument("--no-cache", action="store_true")
    ap.add_argument("--tag", default="default")
    ap.add_argument("--items", type=int, nargs="*", default=None)
    ap.add_argument("--out", default=str(RESULTS_DIR / "acceptance"))
    args = ap.parse_args(argv)
    import sys
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")     # Windows cp949 console

    rates = [1000.0, 200.0] if args.rate == "both" else [float(args.rate)]
    matrix = {"full": MATRIX_FULL, "reduced": MATRIX_REDUCED, "nominal": [NOMINAL]}[args.matrix]
    seeds = list(range(args.seeds))
    items = list(ITEMS) + (list(STAGE2_ITEMS) if args.stage2 or args.host == "mpc" else [])
    if args.items:
        items = [it for it in items if it.id in args.items]
    tasks = build_tasks(items, rates, matrix, seeds, args.host)
    if args.reference:
        tasks += [Task(it.id, 1000.0, NOMINAL[0], NOMINAL[1], 0, "ramp", True) for it in items
                  if it.scenario in ("stand", "tilt", "push", "velocity", "yaw", "height") and "payload" not in it.params and "pitch_bias_deg" not in it.params]
    out_dir = Path(args.out)
    cache_dir = out_dir / "cache" / args.tag
    cache_dir.mkdir(parents=True, exist_ok=True)
    chash = code_hash()
    runs: Dict[Task, dict] = {}
    todo = []
    for t in tasks:
        f = cache_dir / f"{task_key(t, chash)}.json"
        if f.exists() and not args.no_cache:
            runs[t] = json.loads(f.read_text(encoding="utf-8"))
        else:
            todo.append(t)
    print(f"{len(tasks)} runs ({len(todo)} to compute, {len(tasks) - len(todo)} cached) on {args.jobs} processes; code hash {chash}")
    t0 = time.time()
    if todo:
        with Pool(args.jobs) as pool:
            for i, (t, r) in enumerate(zip(todo, pool.imap(run_task, todo))):
                runs[t] = r
                (cache_dir / f"{task_key(t, chash)}.json").write_text(json.dumps(r, default=float), encoding="utf-8")
                if "error" in r:
                    print("ERROR", t, r["error"])
                if (i + 1) % 25 == 0:
                    print(f"  {i + 1}/{len(todo)} done ({time.time() - t0:.0f} s)")
    print(f"finished in {time.time() - t0:.0f} s")
    reference = {t.item_id: runs[t] for t in tasks if t.reference}
    tag = "plan4_mpc" if args.host == "mpc" else "plan4"
    all_pass = True
    for rate in rates:
        s1_path = out_dir / f"plan4_{rate:.0f}Hz.json"
        stage1 = json.loads(s1_path.read_text(encoding="utf-8")) if (args.host == "mpc" and s1_path.exists()) else None
        summary = summarize(items, runs, rate, reference, stage1)
        all_pass &= summary["passed"]
        md = summary_markdown(summary, args.matrix, len(seeds), args.host)
        (out_dir / f"{tag}_{rate:.0f}Hz.md").write_text(md, encoding="utf-8")
        (out_dir / f"{tag}_{rate:.0f}Hz.json").write_text(json.dumps(summary, indent=1, default=float), encoding="utf-8")
        print(md.split("\n## ")[0])
    print("ALL PASS" if all_pass else "NOT PASSED")
    return 0 if all_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
