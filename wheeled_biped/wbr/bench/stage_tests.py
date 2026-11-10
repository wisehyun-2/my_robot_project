"""Staged balance-controller tests of the v5 robot, shared by both controller folders: v5_lqi/TEST (LQI8) and
v5_mpc/TEST (MPC).  The scenarios, metrics, limits and the report are the same for both, so their results compare
one to one; only the stage table differs (register()).

The robot is the v5 CAD robot with its own URDF (wbr.models.build_v5 from robot_cad_v5/calc: links, joints, masses, inertias, IMU
pose as in Fusion; AKE60-8 on all six joints) and the IMU chain of the real board (wbr.plans.plan_v5: chip at its place on the hat,
pi3hat firmware filter, the MCU's complementary filter).  A stage only changes the balance law inside the MCU; the four
other controllers (height, yaw, roll, split) are the same in every stage and in both folders.  The LQI folder's stages:

    lqr              6 states [theta, theta_dot, v - v_ref, phi, phi_dot, x - x_ref]      (no integral action)
    lqr_ipos         + z3 = integral of the position error                                 (q_pos)
    lqr_ipos_itilt   + z2 = integral of the static body tilt                               (q_att) = the full controller

and the MPC folder's the same states as the MPC's prediction model (mpc, mpc_ipos, mpc_ipos_itilt).

Scenarios and metric definitions are those of the acceptance battery (wbr/bench/acceptance.py, docs/합격기준.md):
sensor noise on, the sensor -> torque latency below, true body pitch judged.  Limits that depend on the motors use the
AKE60-8 (peak 12.5 N m, rated 5 N m).

Environment variables: WBR_TEST_CAD (mass_properties.json), WBR_TEST_SEEDS ("0,1,2"), WBR_TEST_LATENCY_MS,
WBR_TEST_RATE (1000 or 200), WBR_TEST_LONG=1 (60 s stand tests as in the acceptance criteria instead of 30 s).
"""
from __future__ import annotations

import argparse
import atexit
import contextlib
import dataclasses
import functools
import json
import math
import os
import shutil
import tempfile
import time
import unittest
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional

import numpy as np

from wbr.bench import acceptance as ACC
from wbr.bench import ake_cad as AC
from wbr.bench import ake_sizing as AS
from wbr.bench.acceptance import Check

REPO = Path(__file__).resolve().parents[3]          # my_robot_project/ (this file: wheeled_biped/wbr/bench/)
CAD_JSON = Path(os.environ.get("WBR_TEST_CAD", REPO / "robot_cad_v5" / "calc" / "mass_properties.json"))
SEEDS = [int(s) for s in os.environ.get("WBR_TEST_SEEDS", "0").split(",") if s.strip() != ""]
LATENCY_MS = float(os.environ.get("WBR_TEST_LATENCY_MS", "10"))     # acceptance criteria: 10 ms sensor -> torque
RATE_HZ = float(os.environ.get("WBR_TEST_RATE", "1000"))
LONG = os.environ.get("WBR_TEST_LONG", "0") not in ("", "0")
STAND_T = 60.0 if LONG else 30.0            # undisturbed stand (criteria item 1: 60 s)
DISTURBED_T = 60.0 if LONG else 30.0        # stand under a constant disturbance (items 12 / 16 / 17: 60 s)
G = 9.81

MOTOR = AS.MOTORS["AKE60-8"]
PEAK_LIMIT = 0.8 * MOTOR.peak               # 10 N m: 80 % of the peak, as the criteria do for upkie's motors
LEG_RMS_LIMIT = MOTOR.rated                 # 5 N m continuous
WHEEL_RMS_LIMIT = 0.5 * MOTOR.rated         # 2.5 N m (criteria: half the continuous torque)

# constant disturbances of the integrator tests (criteria items 12 / 16 / 17, the payload scaled to this robot:
# 0.3 kg on the 5.3 kg upkie -> 0.5 kg on the 11 kg robot)
SLOPE_DEG = 5.0
IMU_BIAS_DEG = 1.0
PAYLOAD = (0.5, 0.10, None)                 # mass [kg], ahead of the axle [m], height = the body's COM


@dataclass(frozen=True)
class Stage:
    """One stage of a controller folder.  overrides: Plan config fields of this stage (the LQI folder sets lqi_kwargs,
    the MPC folder mpc_kwargs); plan_classes: () -> (config class, plan class), None = the robot's default (PlanV5, LQI8)."""
    key: str
    title: str
    overrides: dict = field(default_factory=dict)
    plan_classes: Optional[Callable] = None


STAGES: Dict[str, Stage] = {}       # filled by the folder's TEST/stage_common.py (register)


def register(*stages: Stage) -> None:
    for s in stages:
        STAGES[s.key] = s

# checks applied to every scenario: the robot stays up, the knees stay on their side of full extension with a margin,
# the motors stay inside 80 % of their peak and inside their continuous rating
COMMON_CHECKS = [
    Check("alive", ">=", 1, "", "넘어지지 않음"),
    Check("knee_flipped", "<=", 0, "", "무릎이 완전 신장 반대쪽으로 넘어가지 않음"),
    Check("leg_margin_mm", ">=", 5.0, "mm", "완전 신장까지 남은 다리 길이"),
    Check("wheel_peak", "<=", PEAK_LIMIT, "N m", "바퀴 토크 피크 ≤ AKE60 피크의 80 %"),
    Check("leg_peak", "<=", PEAK_LIMIT, "N m", "hip·무릎 토크 피크 ≤ AKE60 피크의 80 %"),
    Check("leg_rms", "<=", LEG_RMS_LIMIT, "N m", "hip·무릎 토크 rms ≤ 정격"),
    Check("wheel_rms10", "<=", WHEEL_RMS_LIMIT, "N m", "바퀴 토크 10 s rms ≤ 정격의 50 %"),
]


# ----------------------------------------------------------------------------- robot / plan factory
class Robot:
    """The CAD robot built once per process in a temporary folder."""

    def __init__(self):
        if not CAD_JSON.exists():
            raise FileNotFoundError(f"{CAD_JSON} (robot_cad_v5: v5run.save_mass_json(); or set WBR_TEST_CAD)")
        self.root = Path(tempfile.mkdtemp(prefix="wbr_stage_test_"))
        atexit.register(shutil.rmtree, self.root, True)
        self.case = AC.CadCase("robot", hip_motor="AKE60-8", knee_motor="AKE60-8", wheel_motor="AKE60-8", cad_json=str(CAD_JSON))
        self.params = AC.build_cad_model(self.case, self.root / "model")

    @contextlib.contextmanager
    def plan(self, stage: str, seed: int = 0, slope: float = 0.0, **cfg):
        """A Plan 4 closed loop of this robot with the balance law of `stage`.  The module patch of ake_sizing (model
        folder, wheel radius, motor limits, track) stays active inside the with-block: plan.reset() reads the wheel
        radius when it places the robot on the floor."""
        st = STAGES[stage]
        case = dataclasses.replace(self.case, seed=seed)
        case.plan_overrides = dict(self.case.plan_overrides, **{k: (dict(v) if isinstance(v, dict) else v) for k, v in st.overrides.items()},
                                   latency_ms=LATENCY_MS, rate_hz=RATE_HZ, **cfg)
        if st.plan_classes is not None:     # another balance law in the same harness (v5_mpc)
            case.plan_classes = st.plan_classes
        with AS.patched_plan4(case, self.root) as p4:
            yield AS.make_plan(p4, case, "model", self.root / "model" / self.case.params_file, slope)


@functools.lru_cache(maxsize=1)
def robot() -> Robot:
    return Robot()


# ----------------------------------------------------------------------------- metrics
def _extras(plan, out: dict) -> dict:
    """Leg extension / knee side and the motor torques (acceptance.torque_metrics; its saturation figures refer to
    upkie's 1.7 N m wheel motor and are dropped)."""
    L0 = np.maximum(plan.log["L0_left"], plan.log["L0_right"])  # the longer leg: the legs differ under a roll or a bump
    out["knee_flipped"] = int(AS.knee_flipped(plan))
    out["leg_max_mm"] = float(1000 * np.max(L0))
    out["leg_margin_mm"] = float(1000 * (plan.leg.g.L0_max - np.max(L0)))
    for k in ("wheel_sat_duty", "wheel_sat_s", "wheel_speed_peak", "leg_speed_peak"):
        out.pop(k, None)
    out["alive"] = int(bool(out.get("alive", True)))
    return out


def stand(stage: str, seed: int = 0, T: float = STAND_T, **cfg) -> dict:
    with robot().plan(stage, seed, **cfg) as plan:
        return _extras(plan, ACC.scenario_stand(plan, T=T))


def tilt(stage: str, seed: int = 0, deg: float = 10.0) -> dict:
    with robot().plan(stage, seed) as plan:
        return _extras(plan, ACC.scenario_tilt(plan, deg=deg))


def push(stage: str, seed: int = 0, frac: float = 0.5) -> dict:
    with robot().plan(stage, seed) as plan:
        return _extras(plan, ACC.scenario_push(plan, frac=frac))


STOP_TAIL_S = 5.0       # time simulated after a stop command (the acceptance battery uses 3 s)


def _run_cmd(plan, seconds: float, v: float = 0.0, w: float = 0.0) -> bool:
    for _ in range(int(round(seconds / plan.dt))):
        plan.step(v, w)
        if plan.fallen:
            return False
    return True


def _drive_until(plan, x_end: float, V: float, T_max: float = 14.0) -> bool:
    """Drive at V until the robot is past x_end (it slows down by itself over uneven ground, McuParams.uneven_speed)."""
    for _ in range(int(round(T_max / plan.dt))):
        if plan.log["x"][-1] >= x_end:
            return True
        plan.step(V)
        if plan.fallen:
            return False
    return True


def velocity(stage: str, seed: int = 0, V: float = 0.5, slope: float = 0.0) -> dict:
    """Criteria item 8: 0.5 s stand, 2.5 s at V, then stop.  Same metrics as acceptance._velocity_profile, but the
    tail after the stop is STOP_TAIL_S long and the time to settle after the stop is reported (stop_settle_s): the
    battery's 3 s tail with its 1 s hold only tells whether the robot settles within 2 s, and this robot needs
    2.1-2.2 s (it overshoots the stop by 25-29 cm and rolls back at 0.13-0.16 m/s)."""
    with robot().plan(stage, seed, slope=slope) as plan:
        plan.reset()
        plan.run(0.5)
        alive = _run_cmd(plan, 2.5, V) and _run_cmd(plan, STOP_TAIL_S)
        L = plan.log
        t, v, x = L["t"], L["vx"], L["x"]
        sgn = 1.0 if V >= 0 else -1.0
        seg, ss, stop = (t >= 0.5) & (t < 3.0), (t >= 2.0) & (t < 3.0), t >= 3.0
        rise = float(t[seg][np.argmax(sgn * v[seg] > 0.9 * abs(V))] - 0.5) if seg.any() and (sgn * v[seg]).max() > 0.9 * abs(V) else None
        st = ACC.settle_time_true(L, 3.0) if alive and stop.any() else None
        out = dict(alive=alive, rise_s=rise,
                   overshoot_pct=float(((sgn * v[seg]).max() - abs(V)) / abs(V) * 100) if seg.any() else None,
                   ss_err=float(abs(np.mean(v[ss]) - V)) if ss.any() else None,
                   pitch_peak=float(math.degrees(np.abs(L["pitch_true"][t >= 0.5]).max())),
                   stop_disp=float(np.abs(x[stop] - x[stop][0]).max()) if stop.any() else None,
                   stop_end=float(abs(x[-1] - x[stop][0])) if stop.any() else None,
                   stop_settle_s=st, settled=int(st is not None))
        out.update(ACC.torque_metrics(plan, t_from=0.5))
        return _extras(plan, out)


def yaw(stage: str, seed: int = 0, W: float = 1.0) -> dict:
    with robot().plan(stage, seed) as plan:
        return _extras(plan, ACC.scenario_yaw(plan, W=W))


def height(stage: str, seed: int = 0, dh: float = -0.04) -> dict:
    """4 cm crouch and back (criteria item 10), plus how well the leg length follows."""
    with robot().plan(stage, seed) as plan:
        out = ACC.scenario_height(plan, dh=dh)
        L, t = plan.log["L0"], plan.log["t"]
        L_stand = plan.mcu.p.L_stand
        low = (t >= 2.5) & (t < 3.5)                      # crouched from 1.0 s to 3.5 s: the last second of it
        out["crouch_err_mm"] = float(1000 * abs(L[low].mean() - (L_stand + dh)))
        out["return_err_mm"] = float(1000 * abs(L[t >= t[-1] - 1.0].mean() - L_stand))
        return _extras(plan, out)


def roll(stage: str, seed: int = 0, ref_deg: float = 3.0) -> dict:
    """Roll reference step: the roll controller leans the body by a leg-length difference."""
    with robot().plan(stage, seed) as plan:
        plan.reset()
        plan.run(1.0)
        plan.host.set_command(roll=math.radians(ref_deg))
        alive = plan.run(4.0)
        plan.host.set_command(roll=0.0)
        L = plan.log
        t, r = L["t"], np.degrees(L["roll_true"])
        end = t >= t[-1] - 1.0
        out = dict(alive=alive, roll_end_deg=float(r[end].mean()), roll_err_deg=float(abs(r[end].mean() - ref_deg)),
                   roll_peak_deg=float(np.abs(r).max()), pitch_peak=float(math.degrees(np.abs(L["pitch_true"][t >= 1.0]).max())))
        out.update(ACC.torque_metrics(plan, t_from=1.0))
        return _extras(plan, out)


def turn(stage: str, seed: int = 0, V: float = 0.5, W: float = 1.0) -> dict:
    """Driving and turning together: the differential wheel torque must yaw the robot, not split the legs."""
    with robot().plan(stage, seed) as plan:
        split = []
        base_step = plan.step

        def step(v=0.0, w=0.0):
            o = base_step(v, w)
            split.append(plan.mcu.telemetry.get("d_theta", 0.0))
            return o

        plan.step = step
        plan.reset()
        plan.run(1.0)
        alive = _run_cmd(plan, 3.0, V, W) and _run_cmd(plan, STOP_TAIL_S)
        L = plan.log
        t = L["t"]
        ss = (t >= 3.0) & (t < 4.0)
        yaw_mean = float(L["yaw_rate"][ss].mean()) if ss.any() else float("nan")
        st = ACC.settle_time_true(L, 4.0) if alive else None
        out = dict(alive=alive, yaw_rate_mean=yaw_mean, yaw_err_pct=float(abs(yaw_mean - W) / W * 100),
                   split_peak_deg=float(np.degrees(np.abs(split)).max()),
                   pitch_peak=float(math.degrees(np.abs(L["pitch_true"][t >= 1.0]).max())),
                   stop_settle_s=st, settled=int(st is not None))
        out.update(ACC.torque_metrics(plan, t_from=1.0))
        return _extras(plan, out)


def _leg_error_mm(plan) -> np.ndarray:
    return 1000.0 * (plan.log["L0"] - plan.log["L_ref"])


def leg_on_slope(stage: str, seed: int = 0, slope: float = 10.0, T: float = 6.0, **cfg) -> dict:
    """Standing on a slope the wheels hold a constant torque; the knee must keep its reaction on the shank out of the
    leg length (McuParams.wheel_reaction_ff).  leg_err_mm: |L0 - L_ref| over the last second."""
    with robot().plan(stage, seed, slope=slope, **cfg) as plan:
        plan.reset()
        alive = plan.run(T)
        t = plan.log["t"]
        end = t >= t[-1] - 1.0
        out = dict(alive=alive, leg_err_mm=float(abs(_leg_error_mm(plan)[end].mean())),
                   wheel_hold_Nm=float(0.5 * plan.log["T"][end].mean()))
        out.update(ACC.torque_metrics(plan, t_from=ACC.STAND_SKIP_S))
        return _extras(plan, out)


def leg_under_push(stage: str, seed: int = 0, frac: float = 0.5, **cfg) -> dict:
    """A push makes the hip and the wheels work hard for a moment; the knee must keep their reactions out of the leg
    length.  leg_dev_mm: largest |L0 - L_ref| after the push, measured from its value just before the push."""
    with robot().plan(stage, seed, **cfg) as plan:
        plan.reset()
        plan.run(1.0)
        before = float(_leg_error_mm(plan)[-200:].mean())
        n = len(plan.log["t"])
        plan.push(frac * plan.sim.total_mass() * G, 0.1)
        alive = plan.run(2.0)
        out = dict(alive=alive, leg_dev_mm=float(np.abs(_leg_error_mm(plan)[n:] - before).max()))
        out.update(ACC.torque_metrics(plan, t_from=1.0))
        return _extras(plan, out)


def leg_over_bump(stage: str, seed: int = 0, h: float = 0.04, V: float = 0.5, side: float = 1.0, **cfg) -> dict:
    """One wheel drives over a bump (30 cm ramps, 50 cm top, 14 cm wide): the body stays level because the pressed leg
    yields (McuParams.roll_compliant).  roll_top_deg: |mean roll| on the top; yield_pct: leg-length difference there as a
    share of the bump height."""
    x0, ramp, top = 0.4, 0.3, 0.5
    with robot().plan(stage, seed, floor_bump=(h, side, x0, ramp, top, 0.07), **cfg) as plan:
        plan.reset()
        alive = _run_cmd(plan, 1.0) and _drive_until(plan, 2.0, V) and _run_cmd(plan, 3.0)
        L = plan.log
        x = L["x"]
        roll = np.degrees(L["roll_true"])
        on = (x >= x0 - 0.2) & (x <= x0 + 2 * ramp + top + 0.2)
        flat = (x >= x0 + ramp + 0.15) & (x <= x0 + ramp + top - 0.15)
        out = dict(alive=alive, roll_peak_deg=float(np.abs(roll[on]).max()), roll_top_deg=float(abs(roll[flat].mean())),
                   yield_pct=float(100 * abs((L["L0_left"][flat] - L["L0_right"][flat]).mean()) / h),
                   pitch_peak=float(math.degrees(np.abs(L["pitch_true"][L["t"] >= 1.0]).max())))
        out.update(ACC.torque_metrics(plan, t_from=1.0))
        return _extras(plan, out)


def heading_over_step(stage: str, seed: int = 0, h: float = 0.04, V: float = 0.5, side: float = 1.0, **cfg) -> dict:
    """One wheel hits a sharp step (15 mm ramp, 50 cm top, 14 cm wide): it stops for a moment and must not turn the robot
    (McuParams.kw_yaw).  heading_peak_deg: largest |heading|, heading_end_deg: after 3 s of standing, y_end_cm: where it ends."""
    with robot().plan(stage, seed, floor_bump=(h, side, 0.4, 0.015, 0.5, 0.07), **cfg) as plan:
        plan.reset()
        alive = _run_cmd(plan, 1.0) and _drive_until(plan, 2.0, V) and _run_cmd(plan, 3.0)
        L = plan.log
        s = L["t"] >= 1.0
        heading = np.degrees(np.cumsum(L["yaw_rate"]) * plan.dt)
        out = dict(alive=alive, heading_peak_deg=float(np.abs(heading[s]).max()), heading_end_deg=float(abs(heading[-1])),
                   y_end_cm=float(100 * abs(L["y"][-1])), roll_peak_deg=float(np.abs(np.degrees(L["roll_true"][s])).max()),
                   pitch_peak=float(math.degrees(np.abs(L["pitch_true"][s]).max())))
        out.update(ACC.torque_metrics(plan, t_from=1.0))
        return _extras(plan, out)


def hump_length(h: float) -> float:
    """Curved bump: 10 x its height, at least 0.8 m (steepest slope 17 deg for the tall ones)."""
    return max(0.8, 10.0 * h)


def hump_over(stage: str, seed: int = 0, h: float = 0.10, V: float = 0.3, side: float = 1.0, half_width: float = 0.15, **cfg) -> dict:
    """One wheel over a curved speed bump, z = h/2 (1 - cos(2 pi s / length)).  The legs make up the height while their range lasts, the mean
    length goes down to give them all of it; beyond that the body tilts.  roll_top_deg: mean |roll| on the middle fifth of the bump."""
    length = hump_length(h)
    x0 = 0.6
    with robot().plan(stage, seed, floor_humps=((h, side, x0, length, half_width),), **cfg) as plan:
        plan.reset()
        alive = _run_cmd(plan, 1.0) and _drive_until(plan, x0 + length + 0.4, V, T_max=30.0)
        alive = alive and _run_cmd(plan, 2.5, V)
        v_after = float(np.mean(plan.log["vx"][-int(round(0.5 / plan.dt)):]))
        alive = alive and _run_cmd(plan, 2.0)
        L = plan.log
        x = L["x"]
        s = L["t"] >= 1.0
        on = (x >= x0 - 0.1) & (x <= x0 + length + 0.2)
        top = (x >= x0 + 0.4 * length) & (x <= x0 + 0.6 * length)
        roll = np.degrees(L["roll_true"])
        heading = np.degrees(np.cumsum(L["yaw_rate"]) * plan.dt)
        w = int(round(0.1 / plan.dt))
        vs = np.convolve(L["vx"], np.ones(w) / w, mode="same")
        zs = np.convolve(L["z"], np.ones(w) / w, mode="same")
        out = dict(alive=alive, roll_peak_deg=float(np.abs(roll[on]).max()), roll_top_deg=float(np.abs(roll[top]).mean()), pitch_peak=float(math.degrees(np.abs(L["pitch_true"][s]).max())),
                   rise_mm=float(1000 * (L["z"][on].max() - L["z"][(L["t"] > 0.5) & (L["t"] < 1.0)].mean())), heave_peak=float(np.abs(np.gradient(np.gradient(zs, plan.dt), plan.dt)[on][w:-w]).max()),
                   brake_peak=float(-np.gradient(vs, plan.dt)[on].min()),
                   leg_diff_mm=float(1000 * np.abs(L["L0_left"][s] - L["L0_right"][s]).max()), leg_shortest_mm=float(1000 * np.minimum(L["L0_left"][s], L["L0_right"][s]).min()),
                   heading_peak_deg=float(np.abs(heading[s]).max()), y_end_cm=float(100 * abs(L["y"][-1])), v_top=float(L["vx"][top].mean()), v_after=v_after)
        out.update(ACC.torque_metrics(plan, t_from=1.0))
        return _extras(plan, out)


def _estimate_error_deg(plan, signal: str, truth: str) -> float:
    """Largest error of the MCU's attitude after the start, over 50 ms (the sensor -> MCU delay of a fast swing is not an estimate error)."""
    L = plan.log
    n = int(round(0.05 / plan.dt))
    e = np.degrees(L[signal] - L[truth])[L["t"] >= 0.3]
    return float(np.abs(np.convolve(e, np.ones(n) / n, mode="valid")).max())


def imu_drive(stage: str, seed: int = 0, V: float = 1.0, W: float = 0.0, T: float = 3.0, **cfg) -> dict:
    """Start, drive and stop: the accelerometer must not be taken for gravity (the board's own attitude reads 1.5 deg off at 1 m/s and
    2 deg when also turning at 2 rad/s, wbr.observers.body_attitude)."""
    with robot().plan(stage, seed, **cfg) as plan:
        plan.reset()
        alive = _run_cmd(plan, 1.0) and _run_cmd(plan, T, V, W) and _run_cmd(plan, 4.0)
        out = dict(alive=alive, pitch_err_deg=_estimate_error_deg(plan, "pitch", "pitch_true"), roll_err_deg=_estimate_error_deg(plan, "roll", "roll_true"),
                   pitch_peak=float(math.degrees(np.abs(plan.log["pitch_true"][plan.log["t"] >= 1.0]).max())))
        out.update(ACC.torque_metrics(plan, t_from=1.0))
        return _extras(plan, out)


def imu_long_turn(stage: str, seed: int = 0, W: float = 2.0, T: float = 30.0, **cfg) -> dict:
    """A long turn, then stop: the board's z bias state has run away during the turn (zero-yaw-rate pseudo measurement) and must not
    keep the robot turning (McuParams.yaw_offset_tau).  yaw_err_pct: the turn itself, last second."""
    with robot().plan(stage, seed, **cfg) as plan:
        plan.reset()
        alive = _run_cmd(plan, 1.0) and _run_cmd(plan, T, 0.0, W) and _run_cmd(plan, 4.0)
        L = plan.log
        t = L["t"]
        turning = (t >= 1.0 + T - 1.0) & (t < 1.0 + T)
        out = dict(alive=alive, yaw_err_pct=float(100 * abs(L["yaw_rate"][turning].mean() - W) / W),
                   yaw_rate_end=float(abs(L["yaw_rate"][t >= t[-1] - 1.0].mean())))
        out.update(ACC.torque_metrics(plan, t_from=1.0))
        return _extras(plan, out)


def roll_at_leg_limit(stage: str, seed: int = 0, deg: float = 6.0, dh: float = 0.02, **cfg) -> dict:
    """A roll command the leg range cannot give at a tall stance: each leg stays inside its range, the knee does not flip."""
    with robot().plan(stage, seed, **cfg) as plan:
        plan.set_height_offset(dh)
        plan.reset()
        plan.mcu.L_ref.value = plan.mcu.p.L_stand + dh
        alive = plan.run(1.0)
        plan.host.set_command(roll=math.radians(deg))
        alive = alive and plan.run(4.0)
        L = plan.log
        out = dict(alive=alive, roll_end_deg=float(np.degrees(L["roll_true"][L["t"] >= L["t"][-1] - 1.0].mean())))
        out.update(ACC.torque_metrics(plan, t_from=1.0))
        return _extras(plan, out)


def roll_under_side_load(stage: str, seed: int = 0, force: float = 20.0, lever: float = 0.15, T: float = 8.0, **cfg) -> dict:
    """A constant downward force beside the body centre (2 kg, 15 cm to the left): the roll integral takes it out."""
    with robot().plan(stage, seed, **cfg) as plan:
        plan.reset()
        alive = plan.run(1.0)
        plan.sim.push([0.0, 0.0, -force], T, body=plan.torso_body, point=np.asarray(plan.sim.com()) + np.array([0.0, lever, 0.0]))
        alive = alive and plan.run(T)
        L = plan.log
        roll = np.degrees(L["roll_true"])
        out = dict(alive=alive, roll_peak_deg=float(np.abs(roll[L["t"] >= 1.0]).max()),
                   roll_end_deg=float(abs(roll[L["t"] >= L["t"][-1] - 1.0].mean())))
        out.update(ACC.torque_metrics(plan, t_from=1.0))
        return _extras(plan, out)


END_WINDOW_S = 5.0      # the end values of a disturbed stand are means over the last 5 s (the robot wanders +-2 cm slowly)


@functools.lru_cache(maxsize=None)      # stage 4 judges the same three runs twice (position, then tilt)
def disturbed_stand(stage: str, seed: int = 0, kind: str = "slope", T: float = DISTURBED_T) -> dict:
    """Standing under a constant disturbance the controller does not know: a 5 deg slope, a +1 deg IMU pitch bias or
    a payload ahead of the axle.  What the integrators are for:
      offset_end_cm   where the robot ends up (|mean of the last 5 s - start|)      -> position integral
      pitch_end_deg   how far the TRUE body pitch ends from level (last 5 s)        -> static-tilt integral
    plus the largest excursion on the way (criteria items 12 / 16 / 17: <= 10 cm)."""
    slope, cfg = 0.0, {}
    if kind == "slope":
        slope = SLOPE_DEG
    elif kind == "imu_bias":
        cfg["pitch_bias_deg"] = IMU_BIAS_DEG
    elif kind == "payload":
        cfg["payload"] = PAYLOAD
    else:
        raise ValueError(kind)
    with robot().plan(stage, seed, slope=slope, **cfg) as plan:
        plan.reset()
        alive = plan.run(T)
        L = plan.log
        t = L["t"] - L["t"][0]
        x = L["x"] - L["x"][0]
        pitch = np.degrees(L["pitch_true"])
        end = late = t >= t[-1] - END_WINDOW_S
        out = dict(alive=alive, offset_end_cm=float(100 * abs(x[end].mean())), offset_end_signed_cm=float(100 * x[end].mean()),
                   excursion_max_cm=float(100 * np.abs(x[t >= ACC.STAND_SKIP_S]).max()) if (t >= ACC.STAND_SKIP_S).any() else float("inf"),
                   pitch_end_deg=float(abs(pitch[end].mean())), pitch_end_signed_deg=float(pitch[end].mean()),
                   pitch_rms_end_deg=float(np.sqrt(np.mean((pitch[late] - pitch[late].mean()) ** 2))),
                   speed_end_cm_s=float(100 * abs(L["vx"][end].mean())))
        out.update(ACC.torque_metrics(plan, t_from=ACC.STAND_SKIP_S))
        return _extras(plan, out)


# ----------------------------------------------------------------------------- judging / report
RECORDS: List[dict] = []            # every judged check of this process (run_stages.py writes the report from it)


def judge(test_id: str, title: str, stage: str, seed: int, metrics: dict, checks: List[Check], common: bool = True,
          group: str | None = None) -> List[str]:
    """Evaluate the checks (+ COMMON_CHECKS) on the metrics, record them and return the failure messages."""
    fails = []
    for c in list(checks) + (COMMON_CHECKS if common else []):
        v = metrics.get(c.metric)
        ok = bool(c.passes(v))
        RECORDS.append(dict(group=group or STAGES[stage].title, test=test_id, title=title, stage=stage, seed=seed, metric=c.metric,
                            value=v, op=c.op, limit=c.limit, unit=c.unit, note=c.note, ok=ok))
        if not ok:
            shown = "없음" if v is None else (f"{v:.3f}" if isinstance(v, float) else str(v))
            fails.append(f"{c.metric} = {shown} (기준 {c.op} {c.limit:g} {c.unit}) {c.note}".rstrip())
    return fails


class StageTestCase(unittest.TestCase):
    """Base class: STAGE selects the balance law, GROUP is the heading in the report; check() judges one scenario
    for every seed."""
    STAGE = "lqr"
    GROUP = None
    maxDiff = None

    def check(self, test_id: str, title: str, scenario, checks: List[Check], common: bool = True, **kw) -> dict:
        last = {}
        for seed in SEEDS:
            with self.subTest(seed=seed):
                m = scenario(self.STAGE, seed, **kw)
                last = m
                fails = judge(test_id, title, self.STAGE, seed, m, checks, common, self.GROUP)
                shown = ", ".join(f"{c.metric} {m.get(c.metric):.3g}" for c in checks if isinstance(m.get(c.metric), (int, float)))
                print(f"\n    [{test_id}] {title} (시드 {seed}): {shown}", end="")
                self.assertFalse(fails, f"\n[{test_id}] {title} — {STAGES[self.STAGE].title}, 시드 {seed}\n  " + "\n  ".join(fails))
        return last


# ----------------------------------------------------------------------------- the tests of each stage (mix-ins)
# Limits: docs/합격기준.md where the item exists there (item number in the title); the deviations are listed in
# README of each folder's TEST/ (v5_lqi/TEST/README.md).  A later stage inherits the mix-ins of the earlier ones, so adding an integrator must not break
# what already worked.
DISTURBANCES = (("slope", f"경사 {SLOPE_DEG:g}°"), ("imu_bias", f"IMU 피치 오차 +{IMU_BIAS_DEG:g}°"),
                ("payload", f"앞쪽 짐 {PAYLOAD[0]:g} kg (+{100 * PAYLOAD[1]:g} cm)"))
OFFSET_LIMIT_CM = 3.0           # position integral: where the robot ends up
EXCURSION_LIMIT_CM = 10.0       # criteria items 12 / 16 / 17: largest excursion on the way
TILT_LIMIT_DEG = 0.5            # static-tilt integral: true body pitch at the end
STOP_SETTLE_LIMIT_S = 3.0       # settled this long after a stop command (the acceptance battery implies 2 s: README)
LEG_SLOPE_LIMIT_MM = 1.0        # knee coupling compensation (seeds 0-4, all stages): measured 0.04-0.12 with it, 3.7-3.8 without
LEG_PUSH_LIMIT_MM = 3.0         # measured 0.7-1.6 with it, 5.7-6.8 without
BUMP_ROLL_PEAK_DEG = 0.5        # compliant roll, 4 cm bump under one wheel: leg-length roll 2.4-2.7
BUMP_ROLL_TOP_DEG = 0.2         # leg-length roll 2.0-2.2
BUMP_YIELD_PCT = 90.0           # leg-length roll 48-53 %
STEP_HEADING_END_DEG = 2.0      # sharp step: 3 s after the stop.  Gyro PI alone 3.1-3.3
STEP_Y_END_CM = 15.0            # gyro PI alone 12 (2 cm) to 40 (4 cm, 0.5 m/s)
# 4 cm with the knees trailing (11-07, user: limits from this direction's measurements, about 15 % over the worst of seeds 0-4; the cause is the knee-side
# geometry, docs/robot_v5_model.md sec. 6).  Gyro PI alone 40-60 deg, 3-4 deg, 23-39 cm
STEP_12 = dict(heading_deg=15.5, end_deg=3.1, y_cm=15.0)
STEP_13 = dict(heading_deg=15.0, y_cm=17.5, pitch_deg=7.5)
HUMP_SPEED_TOP = 0.3            # m/s on a 10 cm bump with 0.5 m/s commanded (limit 0.2)
HUMP_SPEED_AFTER = 0.8          # m/s 2.5 s after a bump, 1 m/s commanded
HUMP_BRAKE = 1.2                # m/s^2: slowing down gently
IMU_PITCH_ERR_DEG = 0.5         # the MCU's attitude filter; the board's attitude alone: 1.5 driving, 2.1 turning
IMU_ROLL_ERR_DEG = 0.5          # the board's attitude alone: 1.0
IMU_YAW_END = 0.01              # rad/s; the board's rate alone: 0.7


class BalanceTests:
    """1. The balance law alone: stand, recover from a tilt and a push, follow a velocity step."""

    def test_1_1_stand(self):
        self.check("1-1", f"정지 {STAND_T:g} s (기준 1번)", stand,
                   [Check("pitch_ma_max", "<=", 1.0, "deg", "20 ms 이동평균 |피치|"), Check("pitch_rms", "<=", 0.5, "deg"),
                    Check("drift_max", "<=", 0.05, "m", "max |x(t) − x(0)|, t ≥ 2 s")])

    def test_1_2_tilt_10deg(self):
        self.check("1-2", "초기 기울기 10° 복귀 (기준 2번)", tilt,
                   [Check("settled", ">=", 1, "", "정착"), Check("settle_s", "<=", 2.0, "s"), Check("disp", "<=", 0.3, "m", "이동")])

    def test_1_3_push_half_weight(self):
        self.check("1-3", "밀기 0.5 W, 0.1 s (기준 5번)", push,
                   [Check("settled", ">=", 1, "", "정착"), Check("settle_s", "<=", 3.0, "s"), Check("pitch_peak", "<=", 8.0, "deg"),
                    Check("disp", "<=", 0.5, "m", "이동")])

    def test_1_4_velocity_step(self):
        self.check("1-4", "속도 스텝 0 → 0.5 m/s → 0 (기준 8번)", velocity,
                   [Check("rise_s", "<=", 1.5, "s", "90 % 도달"), Check("overshoot_pct", "<=", 20.0, "%"),
                    Check("ss_err", "<=", 0.05, "m/s", "2.0~3.0 s 평균"), Check("pitch_peak", "<=", 5.0, "deg"),
                    Check("stop_disp", "<=", 0.5, "m", "정지 명령 뒤 최대 이탈"), Check("settled", ">=", 1, "", "정지 뒤 정착"),
                    Check("stop_settle_s", "<=", STOP_SETTLE_LIMIT_S, "s", "정지 명령 뒤 정착 시간")])


class OuterLoopTests:
    """2. The other four controllers of the paper's Fig. 6 on top of the balance law: height, yaw, roll, split angle."""

    def test_2_1_height_crouch(self):
        self.check("2-1", "높이: 4 cm 웅크림 후 복귀 (기준 10번)", height,
                   [Check("pitch_peak", "<=", 3.0, "deg"), Check("settled", ">=", 1, "", "복귀 후 정착"),
                    Check("crouch_err_mm", "<=", 3.0, "mm", "웅크린 길이 오차"), Check("return_err_mm", "<=", 3.0, "mm", "복귀 길이 오차")])

    def test_2_2_yaw_rate(self):
        self.check("2-2", "요: 1 rad/s, 2 s (기준 9번)", yaw,
                   [Check("yaw_err_pct", "<=", 10.0, "%", "마지막 1 s 평균"), Check("pitch_peak", "<=", 2.0, "deg")])

    def test_2_3_roll_reference(self):
        self.check("2-3", "롤: 기준 3°", roll,
                   [Check("roll_err_deg", "<=", 0.5, "deg", "마지막 1 s 평균의 오차"), Check("pitch_peak", "<=", 2.0, "deg")])

    def test_2_4_turn_while_driving(self):
        self.check("2-4", "다리 벌어짐: 0.5 m/s로 가며 1 rad/s 회전", turn,
                   [Check("yaw_err_pct", "<=", 10.0, "%"), Check("split_peak_deg", "<=", 5.0, "deg", "좌우 다리각 차 최대"),
                    Check("pitch_peak", "<=", 5.0, "deg"), Check("stop_settle_s", "<=", STOP_SETTLE_LIMIT_S, "s", "정지 명령 뒤 정착 시간")])

    def test_2_5_leg_length_on_slope(self):
        self.check("2-5", "무릎 결합 보상: 경사 10° 정지", leg_on_slope,
                   [Check("leg_err_mm", "<=", LEG_SLOPE_LIMIT_MM, "mm", "마지막 1 s 평균 |L0 − L_ref|")])

    def test_2_6_leg_length_under_push(self):
        self.check("2-6", "무릎 결합 보상: 밀기 0.5 W, 0.1 s", leg_under_push,
                   [Check("leg_dev_mm", "<=", LEG_PUSH_LIMIT_MM, "mm", "밀기 뒤 다리 길이 오차 변화 최대")])

    def _bump_checks(self):
        return [Check("roll_peak_deg", "<=", BUMP_ROLL_PEAK_DEG, "deg", "턱 구간 롤 최대"),
                Check("roll_top_deg", "<=", BUMP_ROLL_TOP_DEG, "deg", "턱 위 평균 롤"),
                Check("yield_pct", ">=", BUMP_YIELD_PCT, "%", "턱 위 좌우 다리 길이 차 / 턱 높이"),
                Check("pitch_peak", "<=", 3.0, "deg")]

    def test_2_7_bump_under_one_wheel(self):
        self.check("2-7", "순응 롤: 왼쪽 바퀴만 턱 4 cm, 0.5 m/s", leg_over_bump, self._bump_checks(), h=0.04, V=0.5)

    def test_2_8_bump_fast_other_side(self):
        self.check("2-8", "순응 롤: 오른쪽 바퀴만 턱 4 cm, 1 m/s", leg_over_bump, self._bump_checks(), h=0.04, V=1.0, side=-1.0)

    def test_2_9_roll_at_the_leg_limit(self):
        self.check("2-9", "다리 길이 한계: 기립 +20 mm에서 롤 6° 명령", roll_at_leg_limit,
                   [Check("roll_end_deg", ">=", 3.0, "deg", "한계 안에서 낼 수 있는 롤")])

    def test_2_10_roll_under_side_load(self):
        self.check("2-10", "한쪽 짐: 왼쪽 15 cm에 20 N", roll_under_side_load,
                   [Check("roll_end_deg", "<=", 0.1, "deg", "마지막 1 s 평균"), Check("roll_peak_deg", "<=", 1.0, "deg")])

    def _step_checks(self, heading_deg: float, end_deg: float = STEP_HEADING_END_DEG, y_cm: float = STEP_Y_END_CM, pitch_deg: float = 6.0):
        return [Check("heading_peak_deg", "<=", heading_deg, "deg", "방향 틀어짐 최대"),
                Check("heading_end_deg", "<=", end_deg, "deg", "정지 3 s 뒤 방향"),
                Check("y_end_cm", "<=", y_cm, "cm", "옆 이동"), Check("roll_peak_deg", "<=", 1.0, "deg"),
                Check("pitch_peak", "<=", pitch_deg, "deg")]

    def test_2_11_sharp_step_slow(self):
        self.check("2-11", "방향: 왼쪽 바퀴만 날카로운 턱 2 cm, 0.3 m/s", heading_over_step, self._step_checks(6.0), h=0.02, V=0.3)

    def test_2_12_sharp_step(self):
        self.check("2-12", "방향: 왼쪽 바퀴만 날카로운 턱 4 cm, 0.5 m/s", heading_over_step, self._step_checks(**STEP_12), h=0.04, V=0.5)

    def test_2_13_sharp_step_fast_other_side(self):
        self.check("2-13", "방향: 오른쪽 바퀴만 날카로운 턱 4 cm, 1 m/s", heading_over_step, self._step_checks(**STEP_13), h=0.04, V=1.0, side=-1.0)

    def test_2_17_curved_bump_10cm(self):
        self.check("2-17", "곡선 턱 10 cm, 왼쪽 바퀴, 0.3 m/s", hump_over,
                   [Check("roll_peak_deg", "<=", 1.0, "deg", "턱 구간 롤 최대"), Check("pitch_peak", "<=", 4.0, "deg"), Check("heading_peak_deg", "<=", 8.0, "deg"),
                    Check("rise_mm", "<=", 22.0, "mm", "몸통 상승(턱 높이의 절반이면 평균 길이 유지)"), Check("heave_peak", "<=", 1.5, "m/s^2", "몸통 수직 가속도")], h=0.10)

    def test_2_18_curved_bump_14cm(self):
        self.check("2-18", "곡선 턱 14 cm, 오른쪽 바퀴, 0.3 m/s", hump_over,
                   [Check("roll_peak_deg", "<=", 1.5, "deg", "턱 구간 롤 최대"), Check("pitch_peak", "<=", 4.0, "deg"), Check("heading_peak_deg", "<=", 8.0, "deg")], h=0.14, side=-1.0)

    def test_2_19_curved_bump_20cm(self):
        """Beyond the legs' range (135 mm between 200 and 335 mm) the body tilts: it must only stay up.  One knee touches its wall, so the peak limit here is the motor's."""
        self.check("2-19", "곡선 턱 20 cm, 왼쪽 바퀴, 0.3 m/s", hump_over,
                   [Check("alive", ">=", 1, "", "넘어지지 않음"), Check("knee_flipped", "<=", 0, "", "무릎이 완전 신장 반대쪽으로 넘어가지 않음"), Check("leg_margin_mm", ">=", 5.0, "mm", "완전 신장까지 남은 다리 길이"),
                    Check("roll_peak_deg", "<=", 8.0, "deg", "턱 구간 롤 최대"), Check("roll_top_deg", "<=", 7.0, "deg", "턱 위 평균 롤"), Check("pitch_peak", "<=", 5.0, "deg"),
                    Check("leg_peak", "<=", MOTOR.peak, "N m", "hip·무릎 토크 피크 ≤ AKE60 피크"), Check("leg_rms", "<=", LEG_RMS_LIMIT, "N m", "hip·무릎 토크 rms ≤ 정격")], common=False, h=0.20)

    def test_2_20_slows_down_on_a_curved_bump(self):
        self.check("2-20", "감속: 곡선 턱 10 cm, 명령 0.5 m/s", hump_over,
                   [Check("v_top", "<=", HUMP_SPEED_TOP, "m/s", "턱 위 속도"), Check("brake_peak", "<=", HUMP_BRAKE, "m/s^2", "감속도 최대"), Check("roll_peak_deg", "<=", 1.5, "deg"),
                    Check("pitch_peak", "<=", 4.0, "deg")], h=0.10, V=0.5)

    def test_2_21_speed_comes_back_after_a_curved_bump(self):
        self.check("2-21", "감속 복귀: 곡선 턱 10 cm, 명령 1 m/s", hump_over,
                   [Check("v_after", ">=", HUMP_SPEED_AFTER, "m/s", "턱을 지나 2.5 s 뒤 속도"), Check("brake_peak", "<=", HUMP_BRAKE, "m/s^2", "감속도 최대"), Check("roll_peak_deg", "<=", 1.5, "deg"),
                    Check("pitch_peak", "<=", 4.0, "deg")], h=0.10, V=1.0)

    def test_2_14_imu_while_driving(self):
        self.check("2-14", "IMU: 1 m/s 출발·정지의 자세 추정", imu_drive,
                   [Check("pitch_err_deg", "<=", IMU_PITCH_ERR_DEG, "deg", "추정 − 실제 피치, 50 ms 평균의 최대"), Check("pitch_peak", "<=", 3.2, "deg", "실제 피치 최대")])

    def test_2_15_imu_while_turning(self):
        self.check("2-15", "IMU: 1 m/s로 가며 2 rad/s 회전의 자세 추정", imu_drive,
                   [Check("pitch_err_deg", "<=", IMU_PITCH_ERR_DEG, "deg", "추정 − 실제 피치"), Check("roll_err_deg", "<=", IMU_ROLL_ERR_DEG, "deg", "추정 − 실제 롤")], W=2.0, T=6.0)

    def test_2_16_imu_after_a_long_turn(self):
        self.check("2-16", "IMU: 2 rad/s로 30 s 회전 뒤 정지", imu_long_turn,
                   [Check("yaw_err_pct", "<=", 5.0, "%", "회전 끝의 요 속도 오차"), Check("yaw_rate_end", "<=", IMU_YAW_END, "rad/s", "정지 뒤 남은 요 속도")])


class PositionIntegralTests:
    """3. The position integral: under a constant disturbance the robot comes back to where it started."""

    def test_3_position_under_disturbance(self):
        for i, (kind, label) in enumerate(DISTURBANCES, 1):
            self.check(f"3-{i}", f"{label} 정지 {DISTURBED_T:g} s: 위치", disturbed_stand,
                       [Check("offset_end_cm", "<=", OFFSET_LIMIT_CM, "cm", "마지막 5 s 평균 위치 − 출발 위치"),
                        Check("excursion_max_cm", "<=", EXCURSION_LIMIT_CM, "cm", "최대 이탈, t ≥ 2 s (기준 12·16·17번)")], kind=kind)


class TiltIntegralTests:
    """4. The static-tilt integral: under a constant disturbance the TRUE body pitch comes back to level."""

    def test_4_tilt_under_disturbance(self):
        for i, (kind, label) in enumerate(DISTURBANCES, 1):
            self.check(f"4-{i}", f"{label} 정지 {DISTURBED_T:g} s: 몸통 기울기", disturbed_stand,
                       [Check("pitch_end_deg", "<=", TILT_LIMIT_DEG, "deg", "마지막 5 s 평균 실제 피치"),
                        Check("pitch_rms_end_deg", "<=", 0.5, "deg", "마지막 5 s 흔들림")], common=False, kind=kind)


def report_markdown(records: List[dict] | None = None) -> str:
    """Markdown table of every judged check, grouped by stage and test."""
    records = RECORDS if records is None else records
    lines = ["# 단계별 통과 시험 결과", "",
             f"로봇: `{CAD_JSON.parent.parent.name}` ({CAD_JSON.name}), 제어 주기 {RATE_HZ:g} Hz, 센서→토크 지연 {LATENCY_MS:g} ms, "
             f"센서 노이즈 켬, 시드 {SEEDS}, 정지 시험 {STAND_T:g} s / 외란 정지 {DISTURBED_T:g} s.", ""]
    groups = []
    for r in records:
        if r["group"] not in groups:
            groups.append(r["group"])
    for g in groups:
        rs = [r for r in records if r["group"] == g]
        n_fail = sum(not r["ok"] for r in rs)
        lines += [f"## {g} — {'통과' if n_fail == 0 else f'실패 {n_fail}건'}", "",
                  "| 시험 | 시드 | 지표 | 측정 | 기준 | 판정 |", "|---|---|---|---|---|---|"]
        cell = lambda s: str(s).replace("|", "\\|")          # a bare | would split the table cell
        for r in rs:
            v = r["value"]
            shown = "—" if v is None else (f"{v:.3f}" if isinstance(v, float) else str(v))
            lines.append(f"| {r['test']} {cell(r['title'])} | {r['seed']} | {r['metric']}{' (' + cell(r['note']) + ')' if r['note'] else ''} | {shown} | "
                         f"{r['op']} {r['limit']:g} {r['unit']} | {'통과' if r['ok'] else '**실패**'} |")
        lines.append("")
    return "\n".join(lines)


def run_stages(modules: Dict[int, str], titles: Dict[int, str], out_default: Path, controller: str, doc: str = "",
               argv=None) -> int:
    """The runner of a folder's TEST/run_stages.py: the stages in order, stop at the first failing one (unless
    --keep-going), then write <out>/stage_report.md and .json.  modules: stage number -> test module name (importable:
    the folder's TEST directory is on sys.path)."""
    ap = argparse.ArgumentParser(description=doc, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stage", type=int, nargs="*", choices=sorted(modules), help="stages to run (default: all, in order)")
    ap.add_argument("--keep-going", action="store_true", help="do not stop at the first failing stage")
    ap.add_argument("--out", type=Path, default=out_default, help="report folder")
    args = ap.parse_args(argv)
    stages = sorted(args.stage) if args.stage else sorted(modules)
    summary = []
    t0 = time.time()
    for n in stages:
        print(f"\n===== {controller} 단계 {n}: {titles[n]} =====")
        suite = unittest.defaultTestLoader.loadTestsFromName(modules[n])
        t1 = time.time()
        res = unittest.TextTestRunner(verbosity=2).run(suite)
        ok = res.wasSuccessful()
        summary.append(dict(stage=n, title=titles[n], passed=ok, tests=res.testsRun, failures=len(res.failures), errors=len(res.errors),
                            seconds=round(time.time() - t1, 1)))
        if not ok and not args.keep_going:
            print(f"\n단계 {n} 실패: 이후 단계는 실행하지 않았습니다(--keep-going으로 계속 실행).")
            break
    args.out.mkdir(parents=True, exist_ok=True)
    head = [f"# 단계별 통과 시험 요약 — {controller}", "", "| 단계 | 내용 | 판정 | 시험 수 | 실패 | 오류 | 시간 [s] |", "|---|---|---|---|---|---|---|"]
    for s in summary:
        head.append(f"| {s['stage']} | {s['title']} | {'통과' if s['passed'] else '**실패**'} | {s['tests']} | {s['failures']} | {s['errors']} | {s['seconds']} |")
    not_run = [n for n in stages if n not in [s["stage"] for s in summary]]
    if not_run:
        head.append("")
        head.append("실행하지 않은 단계: " + ", ".join(str(n) for n in not_run))
    md = "\n".join(head) + "\n\n" + report_markdown().split("\n", 2)[2]
    (args.out / "stage_report.md").write_text(md, encoding="utf-8")
    (args.out / "stage_report.json").write_text(json.dumps(dict(controller=controller, summary=summary, records=RECORDS), indent=1, default=float,
                                                           ensure_ascii=False), encoding="utf-8")
    print(f"\n보고서: {args.out / 'stage_report.md'}  (전체 {time.time() - t0:.0f} s)")
    for s in summary:
        print(f"  단계 {s['stage']} {s['title']}: {'통과' if s['passed'] else '실패'}")
    return 0 if all(s["passed"] for s in summary) and not not_run else 1
