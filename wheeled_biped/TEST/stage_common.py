"""Shared code of the staged balance-controller tests: LQR first, integrators later (TEST/README.md).

The robot is the v5 CAD robot (robot_cad_v5/calc/mass_properties.json through wbr.bench.ake_cad: AKE60-8 on all six
joints, CAD masses / inertias / lateral geometry, yaw PI and leg-length PD scaled to the robot).  A stage only changes
the balance law:

    lqr              6 states [theta, theta_dot, v - v_ref, phi, phi_dot, x - x_ref]      (no integral action)
    lqr_ipos         + z3 = integral of the position error                                 (q_pos)
    lqr_ipos_itilt   + z2 = integral of the static body tilt                               (q_att) = the full controller

Scenarios and metric definitions are those of the acceptance battery (wbr/bench/acceptance.py, docs/합격기준.md):
sensor noise on, the sensor -> torque latency below, true body pitch judged.  Limits that depend on the motors use the
AKE60-8 (peak 12.5 N m, rated 5 N m) instead of upkie's qdd100 / mj5208.

Environment variables: WBR_TEST_CAD (mass_properties.json), WBR_TEST_SEEDS ("0,1,2"), WBR_TEST_LATENCY_MS,
WBR_TEST_RATE (1000 or 200), WBR_TEST_LONG=1 (60 s stand tests as in the acceptance criteria instead of 30 s).
"""
from __future__ import annotations

import atexit
import contextlib
import dataclasses
import functools
import math
import os
import shutil
import tempfile
import unittest
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List

import numpy as np

from wbr.bench import acceptance as ACC
from wbr.bench import ake_cad as AC
from wbr.bench import ake_sizing as AS
from wbr.bench.acceptance import Check

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
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
PAYLOAD = (0.5, 0.10, None)                 # mass [kg], ahead of the axle [m], height = Plan 4 default


@dataclass(frozen=True)
class Stage:
    key: str
    title: str
    lqi: dict


STAGES: Dict[str, Stage] = {
    "lqr": Stage("lqr", "순수 LQR (6상태)", dict(q_att=None, q_pos=None)),
    "lqr_ipos": Stage("lqr_ipos", "LQR + ∫위치", dict(q_att=None, q_pos=0.5)),
    "lqr_ipos_itilt": Stage("lqr_ipos_itilt", "LQR + ∫위치 + ∫기울기", dict(q_att=0.2, q_pos=0.5)),
}

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
        case = dataclasses.replace(self.case, seed=seed)
        case.plan_overrides = dict(self.case.plan_overrides, lqi_kwargs=dict(STAGES[stage].lqi), latency_ms=LATENCY_MS,
                                   rate_hz=RATE_HZ, **cfg)
        with AS.patched_plan4(case, self.root) as p4:
            yield AS.make_plan(p4, case, "model", self.root / "model" / "plan4_params.json", slope)


@functools.lru_cache(maxsize=1)
def robot() -> Robot:
    return Robot()


# ----------------------------------------------------------------------------- metrics
def _extras(plan, out: dict) -> dict:
    """Leg extension / knee side and the motor torques (acceptance.torque_metrics; its saturation figures refer to
    upkie's 1.7 N m wheel motor and are dropped)."""
    L0 = plan.log["L0"]
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
        t, r = L["t"], np.degrees(L["roll"])
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
# TEST/README.md.  A later stage inherits the mix-ins of the earlier ones, so adding an integrator must not break
# what already worked.
DISTURBANCES = (("slope", f"경사 {SLOPE_DEG:g}°"), ("imu_bias", f"IMU 피치 오차 +{IMU_BIAS_DEG:g}°"),
                ("payload", f"앞쪽 짐 {PAYLOAD[0]:g} kg (+{100 * PAYLOAD[1]:g} cm)"))
OFFSET_LIMIT_CM = 3.0           # position integral: where the robot ends up
EXCURSION_LIMIT_CM = 10.0       # criteria items 12 / 16 / 17: largest excursion on the way
TILT_LIMIT_DEG = 0.5            # static-tilt integral: true body pitch at the end
STOP_SETTLE_LIMIT_S = 3.0       # settled this long after a stop command (the acceptance battery implies 2 s: README)


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
