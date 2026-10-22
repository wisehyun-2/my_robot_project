"""Stage-2 host process of Plan 4: MPC velocity-reference generator above the MCU's LQI6 balancer.

    MpcHost(link, plan, publish_hz=50)      drop-in replacement of HostEmulator (wbr.controllers.diablo_arch.host_link)

Every publish period (1 / publish_hz, 20 ms) the host solves one small QP (qpmpc condensing, qpsolvers + DAQP with a
warm start, OSQP as backup) and sends the FIRST element of the optimal velocity-reference trajectory as
``MotionCtrl.forward``; ``left`` (yaw rate) and ``up`` (height, clipped to the leg-length range) are passed through,
``pitch`` is never sent.  The MPC has no integrator: the inner LQI6 keeps its own velocity integral.

Prediction model (``wbr.controllers.mpc_host.model``): the stage-1 LQI6 closed loop built from the plant
linearisation and the MCU's own gain table (integrator, ZOH at the MCU period, observation latency, optional rate
low-pass), lifted to the host period and reduced by balanced truncation (5 states) to the r -> (v_base, pitch) axes.

Prediction state: the reference-induced closed-loop state, propagated with the references the host publishes (exact
for the linear closed loop by superposition).  While a manoeuvre is in progress (target != 0 or less than obs_hold_s
after it returned to 0) a steady-state Kalman correction of the slowest closed-loop mode (the LQI6 integrator mode)
uses the period-averaged telemetry speed (``plan.mcu.est.v``): the joint dry friction of the robot makes the inner
integrator wind up and pay back more than the linear model says.  At stand-still the reference stays exactly 0, so
disturbances (pushes, tilts) are left to the inner loop as in stage 1 (criterion 20).  Telemetry is also used to
resynchronise after start-up or a host outage.

Desired response: a critically damped second-order reference model of the target (omega 3.5 rad/s speeding up,
10 rad/s slowing down); the MPC makes the predicted v_base follow it.

QP (decision variable u_k = Delta a_k, the reference-acceleration increment per period, k = 0..N-1, N = 75):
    xi_k = [x_k (reduced closed loop), s_k (horizon displacement), r_{k-1}, a_{k-1}],  a_k = a_{k-1} + u_k,
    r_k = r_{k-1} + T a_k
    min  sum_k w_v (v_k - v_d,k)^2 + w_p (p_k - p_ss)^2 + w_r (r_k - v_d,k)^2 + w_a a_k^2 + w_u u_k^2  + terminal
    s.t. |r_k| <= v_max,  |a_k| <= a_max,  |u_k| <= j_max T
a_max / j_max are derived from the wheel-torque margin (1.36 Nm per wheel), the pitch limit and the leg-lean limit
through the closed-loop model (``derive_limits``, docs/plan4_mpc_host.md section 3).

Fallback (QP failed / no solver): immediately, with an event, the explicit unconstrained optimum of the same QP (one
precomputed gain row, no solver) through the acceleration / jerk / velocity ramp limiter; the stage-1 shaping
(``RampFallback``, 2 m/s^2 + 0.3 s pre-filter) is the last resort (fallback_mode "ramp" selects it directly).
"""
from __future__ import annotations

import copy
import hashlib
import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from wbr.controllers.diablo_arch.host_link import MotionCtrl
from wbr.controllers.mpc_host import model as M
from wbr.controllers.mpc_host.fallback import RampFallback


# ------------------------------------------------------------------------------------------------ configuration
@dataclass
class MpcConfig:
    horizon_s: float = 1.5           # prediction horizon (1-2 s)
    order: int = 0                   # reduced model order; 0 = smallest order whose unit-step error is below
                                     # order_tol_v (velocity, fraction) and order_tol_pitch_deg (docs: order table)
    order_tol_v: float = 0.01
    order_tol_pitch_deg: float = 0.1
    wheel_damping_eq: Optional[float] = None    # N m s/rad: optional equivalent wheel damping of the prediction plant
                                     # (None = nominal 0.002; an identified 0.011 fits the cruise but spoils the stop)
    v_max: float = 1.0               # m/s (MCU max_forward)
    a_max: Optional[float] = None    # m/s^2, None = derive_limits()
    j_max: Optional[float] = None    # m/s^3, None = derive_limits()
    # cost weights (v in m/s, pitch in rad, a in m/s^2, u in m/s^2 per period)
    w_v: float = 1.0 / 0.05 ** 2
    w_p: float = 1.0 / math.radians(1.0) ** 2
    w_r: float = 1.0 / 0.5 ** 2
    w_a: float = 1.0 / 1.0 ** 2
    w_u: float = 1.0 / 1.0 ** 2
    w_x_up: float = 0.0              # displacement (horizon-relative) weight while speeding up  [1/m^2]
    w_x_down: float = 0.0            # ... while slowing down / stopping
    w_terminal: float = 20.0         # multiplies the stage weights at the end of the horizon
    # desired response (reference model, critically damped 2nd order): the MPC makes the predicted v_base follow it
    ref_omega_up: float = 3.5        # rad/s while speeding up   (90 % rise of the model 3.89 / omega = 1.11 s)
    ref_omega_down: float = 10.0     # rad/s while slowing down  (stopping distance of the model 2 V / omega)
    solver: str = "daqp"
    backup_solver: str = "osqp"
    # limit derivation (derive_limits)
    wheel_peak_limit: float = 0.8 * 1.7      # Nm per wheel (criteria: 80 % of the mj5208 peak)
    wheel_torque_reserve: float = 0.30       # Nm: wheel torque already used by the stand-still noise response
    mass_factor: float = 1.15                # +15 % mass variant of the criteria matrix
    pitch_limit_deg: float = 2.0             # criterion 19 (velocity step 0 -> 0.5 m/s)
    pitch_limit_general_deg: float = 5.0     # criteria 8 / 13 (any velocity change, here the full-range reversal)
    pitch_reserve_deg: float = 0.5           # stand-still pitch noise (true pitch, 20 ms moving average)
    step_v: float = 0.5                      # velocity step of criterion 19
    pitch_step_binds: bool = False           # the step-pitch bound is reported but not used for a_max: with the LQI6
                                             # body-tilt integrator (q_att) the step's pitch peak is set by the velocity
                                             # change, not by a (1.3 deg at 0.25 m/s^2, 2.0 deg at 1 m/s^2); limiting a
                                             # would only stretch the rise.  Criterion 19's pitch is handled in the cost.
    lean_limit_deg: float = 32.0             # plant: ankle housing touches the floor at 32 deg forward lean (plan4_model 4)
    lean_fraction: float = 0.8
    mcu_ramp_fraction: float = 0.8           # keep below the MCU's own ramp (4 m/s^2) so that the host ZOH model holds
    jerk_rise_s: float = 0.15                # time to build up a_max (>= 2.5 x the 58 ms closed-loop time constant)
    # fallback when the QP fails / no solver: "explicit" = the unconstrained optimum of the same QP (one precomputed
    # gain row, no solver) passed through the acceleration / jerk / velocity ramp limiter, with the stage-1 shaping as
    # the last resort; "ramp" = the stage-1 shaping (2 m/s^2 ramp + 0.3 s pre-filter) only.  With the MPC-mode inner
    # loop (integrator not frozen) the pure ramp overshoots ~30 % (docs/plan4_mpc_host.md section 5).
    fallback_mode: str = "explicit"
    fallback_rate: float = 2.0
    fallback_tau: float = 0.3
    fallback_hold_s: float = 1.0             # stay in fallback at least this long before retrying the MPC
    # telemetry correction of the prediction state (slow-mode Kalman observer, active only while manoeuvring)
    use_observer: bool = True
    obs_sigma_v: float = 0.01                # m/s   noise of the period-averaged telemetry speed (est.v)
    obs_sigma_z: float = 0.002               # m     per period: random walk of the LQI6 velocity integrator z
    obs_sigma_w: float = 0.0                 # m/s   per period: slowest reduced mode (alternative direction, 0 = off)
    obs_sigma_f: float = 0.0                 # N m   per period: white external wheel torque (0 = off)
    # known joint dry friction (identify.py): torque -F tanh(v_plan / v_s) on the wheels in the prediction
    friction_ff_Nm: float = 0.0             # 0 = off (tried 0.02-0.04 N m: overshoot -0.3 %, stop +2 cm)
    friction_v_s: float = 0.05
    handover: bool = True                    # when the correction stops at stand-still, hand the residual (inner
                                             # integrator pay-back) to the LQI6: prediction state -> 0, r -> 0
    obs_hold_s: float = 0.5                  # keep correcting this long after the target returned to zero (longer
                                             # cancels the inner integrator's pay-back and stretches the stop: 2 s -> +9 cm)
    # bookkeeping
    resync_gap_s: float = 0.25               # host outage longer than this -> resynchronise from telemetry
    violation_tol: float = 1e-3              # relative tolerance of the constraint-violation counter
    keep_trace: bool = True


# ------------------------------------------------------------------------------------------------ caches
_MODEL_CACHE: Dict[tuple, dict] = {}
_LIMIT_CACHE: Dict[tuple, dict] = {}
_QP_CACHE: Dict[tuple, "ReferenceQP"] = {}


def _plan_key(plan, L0: float) -> tuple:
    ctrl = plan.law.ctrl
    K = M._gains_no_side_effect(ctrl, L0)
    d = int(round(plan.cfg.latency_ms * 1e-3 / ctrl.Ts))
    h = hashlib.sha1(np.round(K, 10).tobytes()).hexdigest()[:12]
    lpf = float(getattr(plan.cfg, "rate_lpf_tau", 0.0) or 0.0)
    return (round(ctrl.Ts, 9), d, round(L0, 6), h, round(lpf, 9), round(float(ctrl.v_ramp_rate), 6))


def prediction_model(plan, period: float, order: int, L0: Optional[float] = None,
                     wheel_damping: Optional[float] = None, tol_v: float = 0.01, tol_pitch_deg: float = 0.1) -> dict:
    """Reduced host-rate model (cached).  Returns dict(red=LTIModel (r -> v_base, pitch, v, theta, T), full=lifted
    full-order LTIModel, m=MCU steps per host period, dc=steady-state gains)."""
    L0 = float(plan.params.L0 if L0 is None else L0)
    key = _plan_key(plan, L0) + (round(period, 9), int(order), wheel_damping, tol_v, tol_pitch_deg)
    if key in _MODEL_CACHE:
        return _MODEL_CACHE[key]
    full_mcu = M.plan_closed_loop(plan, L0=L0, wheel_damping=wheel_damping)
    rho = float(np.abs(np.linalg.eigvals(full_mcu.A)).max())
    if not rho < 1.0 - 1e-9:
        # the inner loop itself is unstable (e.g. latency >= 20 ms): no meaningful prediction model
        out = dict(red=None, full=None, m=None, dc={}, L0=L0, stable=False, spectral_radius=rho)
        _MODEL_CACHE[key] = out
        return out
    m = int(round(period / full_mcu.Ts))
    if m < 1 or abs(m * full_mcu.Ts - period) > 1e-9:
        raise ValueError(f"host period {period} s is not a multiple of the MCU period {full_mcu.Ts} s")
    lifted = M.lift(full_mcu, m).select(["r", "Tf"], ["v_base", "pitch", "v", "theta", "T"])
    # balanced truncation weighted on the tracked outputs (velocity 0.1 m/s ~ pitch 1 deg) and on the inputs
    # (reference 1 m/s ~ external wheel torque 0.05 N m); v, theta, T are carried along (docs: order table)
    scales = dict(input_scale=[1.0, 0.05], output_scale=[0.1, math.radians(1.0), 1e3, 1e3, 1e3])
    order_errors = {}
    if order and order > 0:
        red = M.reduce(lifted, order, **scales)
    else:                                  # automatic: the inner loop may change (gain table, extra integrators)
        U = np.zeros((150, 2))
        U[5:, 0] = 1.0
        Yf, _ = lifted.simulate(U)
        red = None
        for o in range(3, 11):
            cand = M.reduce(lifted, o, **scales)
            Y, _ = cand.simulate(U)
            ev = float(np.sqrt(np.mean((Y[:, 0] - Yf[:, 0]) ** 2)))
            ep = float(np.degrees(np.sqrt(np.mean((Y[:, 1] - Yf[:, 1]) ** 2))))
            order_errors[o] = (ev, ep)
            red = cand
            if ev <= tol_v and ep <= tol_pitch_deg:
                break
    red.info["order_errors"] = {str(k): v for k, v in order_errors.items()}
    # direction of the LQI6 velocity integrator z (and z_att) in the reduced coordinates: process-noise direction of
    # the host's observer (joint dry friction winds this integrator up more than the linear model predicts)
    dirs = {}
    for name in ("i_z", "i_z_att"):
        i = full_mcu.info.get(name)
        if i is not None and getattr(red, "proj", None) is not None:
            dirs[name[2:]] = red.proj[:, i].copy()
    red.info["observer_dirs"] = list(dirs)
    red.obs_dirs = dirs
    out = dict(red=red, full=lifted, m=m, dc=dict(zip(lifted.outputs, lifted.dc_gain()[:, 0])), L0=L0, stable=True,
               spectral_radius=rho)
    _MODEL_CACHE[key] = out
    return out


def prediction_observer(red: M.LTIModel, sigma_v: float, sigma_w: float = 0.0, sigma_f: float = 0.0,
                        sigma_z: float = 0.0) -> dict:
    """Steady-state Kalman gain for the measured wheel speed v (period-averaged telemetry est.v).

    Process noise (no disturbance STATE is added, so the host has no integral action):
      sigma_z on the direction of the MCU's LQI6 velocity integrator z (projection of the full-order state onto the
              reduced coordinates): the joint dry friction of the robot makes z wind up and pay back more than the
              linear model predicts -- this is the physical error the correction has to absorb;
      sigma_w on the slowest reduced mode (basis normalised to 1 m/s of velocity), optional;
      sigma_f along the external wheel-torque input Tf, optional."""
    import scipy.linalg as la
    A = red.A
    n = A.shape[0]
    C = red.C[red.iy("v")][None, :]
    Q = 1e-14 * np.eye(n)
    w, V = np.linalg.eig(A)
    i = int(np.argmax(np.abs(w)))
    if sigma_w > 0:
        v = V[:, i]
        basis = [np.real(v), np.imag(v)] if abs(np.imag(w[i])) > 1e-9 else [np.real(v)]
        Vs = np.stack(basis, axis=1)
        gains = C @ Vs
        Vs = Vs / np.where(np.abs(gains) > 1e-9, np.abs(gains), 1.0)
        Q = Q + (sigma_w ** 2) * (Vs @ Vs.T)
    if sigma_f > 0 and "Tf" in red.inputs:
        bf = red.B[:, red.iu("Tf")]
        Q = Q + (sigma_f ** 2) * np.outer(bf, bf)
    dirs = getattr(red, "obs_dirs", {}) or {}
    if sigma_z > 0 and "z" in dirs:
        Q = Q + (sigma_z ** 2) * np.outer(dirs["z"], dirs["z"])
    R = np.array([[sigma_v ** 2]])
    P = la.solve_discrete_are(A.T, C.T, Q, R)
    L = (P @ C.T @ np.linalg.inv(C @ P @ C.T + R))[:, 0]
    poles = np.linalg.eigvals(A - np.outer(A @ L, C[0]))
    return dict(L=L, AL=A @ L, slow_eig=complex(w[i]), observer_poles=poles, C=C[0], sigma_v=sigma_v, sigma_w=sigma_w,
                sigma_f=sigma_f, sigma_z=sigma_z)


def scurve(v_span: float, a: float, j: float, dt: float, t_hold: float = 1.5) -> np.ndarray:
    """Jerk-limited (time-optimal, discrete) velocity profile 0 -> v_span with |a| <= a, |da/dt| <= j, then held."""
    out, v, acc = [], 0.0, 0.0
    for _ in range(int(1e6)):
        dv = v_span - v
        stop_dv = acc * abs(acc) / (2.0 * j)              # velocity gained while bringing acc back to 0
        acc_t = a if dv - stop_dv > 1e-12 else 0.0
        acc += max(-j * dt, min(j * dt, acc_t - acc))
        v = min(v_span, v + acc * dt)
        out.append(v)
        if v >= v_span - 1e-12 and abs(acc) < j * dt:
            break
    return np.concatenate([out, np.full(int(round(t_hold / dt)), v_span)])


def derive_limits(plan, cfg: MpcConfig, L0: Optional[float] = None) -> dict:
    """a_max / j_max from the closed-loop model (docs/plan4_mpc_host.md section 3).

    Two manoeuvres at constant reference acceleration a with the onset jerk j = a / jerk_rise_s, simulated on the
    full-order closed loop (5 ms resolution):
      full : velocity reversal -v_max -> +v_max (2 m/s; a longer acceleration is impossible because |r| <= v_max)
      step : 0 -> 0.5 m/s (criterion 19)
    and each limit is found by bisection on a:
        a_T     (full) : peak T/2 x mass factor 1.15   <= 1.36 Nm - stand-still reserve 0.30 Nm
        a_p,gen (full) : peak true body pitch            <= 5 deg (criteria 8 / 13) - stand-still reserve 0.5 deg
        a_p     (step) : peak true body pitch            <= 2 deg (criterion 19) - 0.5 deg   (reported; binds only with
                         pitch_step_binds, see MpcConfig)
        a_lean  (full) : peak leg lean                   <= 0.8 x 32 deg (ankle housing contact, plan4_model.md 4)
        a_mcu          = 0.8 x MCU ramp rate             (keeps the host's ZOH model valid)
    a_max = min(...), j_max = a_max / jerk_rise_s."""
    L0 = float(plan.params.L0 if L0 is None else L0)
    key = _plan_key(plan, L0) + (cfg.wheel_peak_limit, cfg.wheel_torque_reserve, cfg.mass_factor, cfg.pitch_limit_deg,
                                 cfg.pitch_limit_general_deg, cfg.pitch_reserve_deg, cfg.lean_limit_deg, cfg.lean_fraction,
                                 cfg.mcu_ramp_fraction, cfg.jerk_rise_s, plan.law.ctrl.v_ramp_rate, cfg.wheel_damping_eq,
                                 cfg.v_max, cfg.step_v)
    if key in _LIMIT_CACHE:
        return _LIMIT_CACHE[key]
    full_mcu = M.plan_closed_loop(plan, L0=L0, wheel_damping=cfg.wheel_damping_eq)
    m = max(1, int(round(0.005 / full_mcu.Ts)))
    full = M.lift(full_mcu, m).select(["r"], ["T", "pitch", "theta", "v_base"])
    dt = full.Ts

    def peaks(a: float, v_span: float) -> dict:
        r = scurve(v_span, a, a / cfg.jerk_rise_s, dt)
        Y, _ = full.simulate(r[:, None])
        return dict(T_w=float(np.abs(Y[:, 0]).max() / 2.0), pitch_deg=float(np.degrees(np.abs(Y[:, 1]).max())),
                    lean_deg=float(np.degrees(np.abs(Y[:, 2]).max())))

    v_full, v_step = 2.0 * cfg.v_max, cfg.step_v
    tests = dict(torque=(v_full, "T_w", (cfg.wheel_peak_limit - cfg.wheel_torque_reserve) / cfg.mass_factor),
                 pitch_step=(v_step, "pitch_deg", cfg.pitch_limit_deg - cfg.pitch_reserve_deg),
                 pitch_full=(v_full, "pitch_deg", cfg.pitch_limit_general_deg - cfg.pitch_reserve_deg),
                 lean=(v_full, "lean_deg", cfg.lean_fraction * cfg.lean_limit_deg))
    a_mcu = cfg.mcu_ramp_fraction * float(plan.law.ctrl.v_ramp_rate)
    cands = {}
    for name, (span, f, budget) in tests.items():
        lo, hi = 0.02, a_mcu
        if peaks(hi, span)[f] <= budget:
            cands[name] = hi
            continue
        if peaks(lo, span)[f] > budget:
            cands[name] = lo
            continue
        for _ in range(14):
            mid = 0.5 * (lo + hi)
            if peaks(mid, span)[f] <= budget:
                lo = mid
            else:
                hi = mid
        cands[name] = lo
    cands["mcu_ramp"] = a_mcu
    binding = {k: v for k, v in cands.items() if cfg.pitch_step_binds or k != "pitch_step"}
    a_max = float(min(binding.values()))
    grid = (0.25, 0.5, 1.0, 1.5, 2.0, 3.0)
    out = dict(a_max=a_max, j_max=a_max / cfg.jerk_rise_s, binding=min(binding, key=binding.get), candidates=cands,
               budgets={k: v[2] for k, v in tests.items()},
               at_a_max=dict(full=peaks(a_max, v_full), step=peaks(a_max, v_step)),
               peaks_full=[dict(a=a, **peaks(a, v_full)) for a in grid],
               peaks_step=[dict(a=a, **peaks(a, v_step)) for a in grid],
               rigid_body_Nm_per_wheel=float(0.5 * (plan.params.extra["total_mass"] * plan.params.R + plan.params.I_w / plan.params.R)))
    _LIMIT_CACHE[key] = out
    return out


# ------------------------------------------------------------------------------------------------ reference model
class ReferenceModel:
    """Desired velocity response to the target: critically damped second order (omega_up while speeding up,
    omega_down while slowing down), state (v_d, a_d), exact ZOH discretisation at the host period.  It only shapes
    the target; it has no integral action."""

    def __init__(self, period: float, omega_up: float, omega_down: float, horizon_steps: int):
        from wbr.controllers.lqi6.lqi6 import c2d_zoh
        self.period = period
        self.N = int(horizon_steps)
        self.models = {}
        for name, w in (("up", omega_up), ("down", omega_down)):
            Ac = np.array([[0.0, 1.0], [-w * w, -2.0 * w]])
            Bc = np.array([[0.0], [w * w]])
            Ad, Bd = c2d_zoh(Ac, Bc, period)
            # prediction of v_d at k = 0..N for a constant target: v_k = L[k] s0 + l[k] v*
            L = np.zeros((self.N + 1, 2))
            l = np.zeros(self.N + 1)
            Ak = np.eye(2)
            bk = np.zeros(2)
            for k in range(self.N + 1):
                L[k] = Ak[0]
                l[k] = bk[0]
                bk = Ad @ bk + Bd[:, 0]
                Ak = Ad @ Ak
            self.models[name] = (Ad, Bd[:, 0], L, l)
        self.reset(0.0)

    def reset(self, v: float = 0.0) -> None:
        self.s = np.array([float(v), 0.0])

    def mode(self, target: float) -> str:
        v = self.s[0]
        return "down" if (target * v < 0.0 or abs(target) < abs(v) - 1e-9) else "up"

    def predict(self, target: float) -> np.ndarray:
        _, _, L, l = self.models[self.mode(target)]
        return L @ self.s + l * target

    def step(self, target: float) -> float:
        Ad, Bd, _, _ = self.models[self.mode(target)]
        self.s = Ad @ self.s + Bd * target
        return float(self.s[0])


# ------------------------------------------------------------------------------------------------ QP
class ReferenceQP:
    """Condensed QP of the reference MPC, built with qpmpc (MPCProblem / MPCQP: prediction and constraint matrices)
    and solved with qpsolvers (DAQP, OSQP backup).  The output-weighted cost is assembled from qpmpc's prediction
    matrices (qpmpc's own cost only has a scalar state weight).

    MPC state xi_k = [x_k (reduced closed loop), s_k (displacement since the solve instant), r_{k-1}, a_{k-1}],
    input u_k = Delta a_k.  s is the horizon-relative displacement (reset to 0 at every solve: no integrator state).
    Stage outputs z_k = [v_base, pitch, r_k, a_k, s_k] with desired values [v_d, p_ss(v_d), v_d, 0, s_d]
    (v_d, s_d from the reference model).  Two weight sets: "up" (speeding up) and "down" (slowing down / stopping)."""

    NZ = 5

    def __init__(self, red: M.LTIModel, horizon_steps: int, v_max: float, a_max: float, j_max: float, cfg: MpcConfig,
                 dc: dict):
        from qpmpc import MPCProblem, MPCQP
        self.cfg = cfg
        self.N = N = int(horizon_steps)
        self.T = T = red.Ts
        self.nr = nr = red.n
        self.v_max, self.a_max, self.j_max = float(v_max), float(a_max), float(j_max)
        self.dc_pitch = float(dc.get("pitch", 0.0))
        iv, ip = red.iy("v_base"), red.iy("pitch")
        ir = red.iu("r")
        Ar, Br = red.A, red.B[:, ir]
        Cv, Cp = red.C[iv], red.C[ip]
        Dv, Dp = float(red.D[iv, ir]), float(red.D[ip, ir])
        i_s, i_r, i_a = nr, nr + 1, nr + 2
        nx = nr + 3
        # r_k = r_{k-1} + T a_{k-1} + T u_k ;  a_k = a_{k-1} + u_k
        A = np.zeros((nx, nx))
        B = np.zeros((nx, 1))
        A[:nr, :nr] = Ar
        A[:nr, i_r] = Br
        A[:nr, i_a] = T * Br
        B[:nr, 0] = T * Br
        A[i_s, :nr] = T * Cv                         # s_{k+1} = s_k + T v_base,k
        A[i_s, i_s] = 1.0
        A[i_s, i_r] = T * Dv
        A[i_s, i_a] = T * T * Dv
        B[i_s, 0] = T * T * Dv
        A[i_r, i_r], A[i_r, i_a], B[i_r, 0] = 1.0, T, T
        A[i_a, i_a], B[i_a, 0] = 1.0, 1.0
        # stage constraints C xi_k + D u_k <= e : r_k and a_k boxes (u boxes go to lb / ub)
        Cc = np.zeros((4, nx))
        Dc = np.zeros((4, 1))
        Cc[0, i_r], Cc[0, i_a], Dc[0, 0] = 1.0, T, T
        Cc[1], Dc[1] = -Cc[0], -Dc[0]
        Cc[2, i_a], Dc[2, 0] = 1.0, 1.0
        Cc[3], Dc[3] = -Cc[2], -Dc[2]
        ec = np.array([v_max, v_max, a_max, a_max])
        prob = MPCProblem(transition_state_matrix=A, transition_input_matrix=B, ineq_state_matrix=Cc,
                          ineq_input_matrix=Dc, ineq_vector=ec, nb_timesteps=N, terminal_cost_weight=1.0,
                          stage_state_cost_weight=None, stage_input_cost_weight=1.0,
                          initial_state=np.zeros(nx), goal_state=np.zeros(nx))
        mqp = MPCQP(prob)
        self.problem, self.mpc_qp = prob, mqp
        self.A, self.B = A, B
        self.G = np.asarray(mqp.G, dtype=float)                 # (4N, N)
        self.e_stack = np.tile(ec, N)
        self.CPhi = np.asarray(mqp.C @ mqp.Phi, dtype=float)    # h(x0) = e - C Phi x0
        Phi, Psi = np.asarray(mqp.Phi), np.asarray(mqp.Psi)    # xi_k, k = 0..N-1
        # stage outputs z_k = H xi_k + E u_k
        H = np.zeros((self.NZ, nx))
        E = np.zeros((self.NZ, 1))
        H[0, :nr], H[0, i_r], H[0, i_a], E[0, 0] = Cv, Dv, Dv * T, Dv * T     # feed-through of the reduced model
        H[1, :nr], H[1, i_r], H[1, i_a], E[1, 0] = Cp, Dp, Dp * T, Dp * T
        H[2, i_r], H[2, i_a], E[2, 0] = 1.0, T, T
        H[3, i_a], E[3, 0] = 1.0, 1.0
        H[4, i_s] = 1.0
        Hb = np.kron(np.eye(N), H)
        Eb = np.kron(np.eye(N), E)
        self.Fz = Hb @ Phi                                       # (NZ N, nx)
        self.Mz = Hb @ Psi + Eb                                  # (NZ N, N)
        # terminal outputs on xi_N: [v_base (with r_{N-1}), r_{N-1}, a_{N-1}, s_N]
        Ht = np.zeros((4, nx))
        Ht[0, :nr], Ht[0, i_r] = Cv, Dv
        Ht[1, i_r] = 1.0
        Ht[2, i_a] = 1.0
        Ht[3, i_s] = 1.0
        self.Ft = Ht @ np.asarray(mqp.phi_last)
        self.Mt = Ht @ np.asarray(mqp.psi_last)
        # known external wheel torque sequence Tf_k (joint dry friction feed-forward): xi += Gamma Tf
        self.has_tf = "Tf" in red.inputs
        if self.has_tf:
            itf = red.iu("Tf")
            bxi = np.zeros(nx)
            bxi[:nr] = red.B[:, itf]
            bxi[i_s] = T * float(red.D[iv, itf])
            Gam = np.zeros((nx, N))
            Gam_list = []
            for k in range(N):
                Gam_list.append(Gam.copy())            # xi_k depends on Tf_0..Tf_{k-1}
                Gam = A @ Gam
                Gam[:, k] += bxi
            Gb = np.vstack(Gam_list)                   # (nx N, N)
            Ef = np.zeros((self.NZ, 1))
            Ef[0, 0], Ef[1, 0] = float(red.D[iv, itf]), float(red.D[ip, itf])
            self.Zf = Hb @ Gb + np.kron(np.eye(N), Ef)  # (NZ N, N)
            self.Tf_t = Ht @ Gam                       # terminal (4, N)
        self.weights = {}
        for mode in ("up", "down"):
            w_x = cfg.w_x_up if mode == "up" else cfg.w_x_down
            w = np.array([cfg.w_v, cfg.w_p, cfg.w_r, cfg.w_a, w_x])
            Wz = np.tile(w, N)
            Wz[-self.NZ:] *= cfg.w_terminal
            Wt = cfg.w_terminal * np.array([cfg.w_v, cfg.w_r, cfg.w_a, w_x])
            P = self.Mz.T @ (Wz[:, None] * self.Mz) + self.Mt.T @ (Wt[:, None] * self.Mt) + cfg.w_u * np.eye(N)
            self.weights[mode] = (Wz, Wt, 2.0 * 0.5 * (P + P.T))
        # unconstrained optimum u_0 = g . q (first row of -P^-1): the explicit fallback law (no solver needed)
        self.g0 = {mode: -np.linalg.solve(self.weights[mode][2], np.eye(N)[0]) for mode in self.weights}
        self.lb = -self.j_max * T * np.ones(N)
        self.ub = +self.j_max * T * np.ones(N)
        self.n_var, self.n_ineq = N, self.G.shape[0]

    def x0(self, x_r: np.ndarray, r_prev: float, a_prev: float) -> np.ndarray:
        return np.concatenate([np.asarray(x_r, float), [0.0, r_prev, a_prev]])

    def build(self, x0: np.ndarray, v_target: float, v_des: Optional[np.ndarray] = None, mode: str = "up",
              tf: Optional[np.ndarray] = None):
        """v_des: desired v_base at the stage instants k = 0..N (reference model, N + 1 values); None = the constant
        target.  tf: known external wheel torque over the horizon (N values) or None.  Returns (P, q, h)."""
        N, T = self.N, self.T
        if v_des is None:
            vd = np.full(N + 1, float(v_target))
        else:
            vd = np.asarray(v_des, dtype=float)
            if vd.size == N:
                vd = np.append(vd, vd[-1])
        sd = np.concatenate([[0.0], np.cumsum(vd[:-1]) * T])         # desired displacement s_d,k, k = 0..N
        z_ref = np.empty(self.NZ * N)
        z_ref[0::5] = vd[:N]
        z_ref[1::5] = self.dc_pitch * vd[:N]
        z_ref[2::5] = vd[:N]
        z_ref[3::5] = 0.0
        z_ref[4::5] = sd[:N]
        t_ref = np.array([v_target, v_target, 0.0, sd[N]])
        Wz, Wt, P = self.weights[mode]
        fz = self.Fz @ x0 - z_ref
        ft = self.Ft @ x0 - t_ref
        if tf is not None and self.has_tf:
            fz = fz + self.Zf @ tf
            ft = ft + self.Tf_t @ tf
        q = 2.0 * (self.Mz.T @ (Wz * fz) + self.Mt.T @ (Wt * ft))
        h = self.e_stack - self.CPhi @ x0
        return P, q, h

    def solve(self, x0: np.ndarray, v_target: float, warm: Optional[np.ndarray] = None, solver: str = "daqp",
              v_des: Optional[np.ndarray] = None, mode: str = "up", tf: Optional[np.ndarray] = None):
        import qpsolvers
        P, q, h = self.build(x0, v_target, v_des, mode, tf)
        kwargs = {}
        G = self.G
        if solver == "osqp":
            import scipy.sparse as sps
            if not hasattr(self, "_sparse"):
                self._sparse = {"G": sps.csc_matrix(self.G)}
            key = id(P)
            if key not in self._sparse:
                self._sparse[key] = sps.csc_matrix(P)
            P, G = self._sparse[key], self._sparse["G"]
            kwargs = dict(eps_abs=1e-6, eps_rel=1e-6, polish=False, max_iter=20000, verbose=False)
        problem = qpsolvers.Problem(P, q, G, h, lb=self.lb, ub=self.ub)
        t0 = time.perf_counter()
        sol = qpsolvers.solve_problem(problem, solver=solver, initvals=warm, **kwargs)
        dt_ms = (time.perf_counter() - t0) * 1e3
        ok = bool(sol.found) and sol.x is not None and np.all(np.isfinite(sol.x))
        return ok, (np.asarray(sol.x, float) if ok else None), dt_ms

    def explicit_first_move(self, x0: np.ndarray, v_target: float, v_des: Optional[np.ndarray] = None, mode: str = "up",
                            tf: Optional[np.ndarray] = None) -> float:
        """First input u_0 of the unconstrained optimum (the same cost, no constraints, no iterative solver)."""
        _, q, _ = self.build(x0, v_target, v_des, mode, tf)
        return float(self.g0[mode] @ q)

    def trajectory(self, x0: np.ndarray, U: np.ndarray, tf: Optional[np.ndarray] = None) -> Dict[str, np.ndarray]:
        """Planned v_k, p_k, r_k, a_k, s_k (k = 0..N-1) for a solution U."""
        z = self.Fz @ x0 + self.Mz @ U
        if tf is not None and self.has_tf:
            z = z + self.Zf @ tf
        z = z.reshape(self.N, self.NZ)
        return dict(v=z[:, 0], pitch=z[:, 1], r=z[:, 2], a=z[:, 3], s=z[:, 4], u=U)


def reference_qp(plan, cfg: MpcConfig, period: float) -> Tuple[Optional["ReferenceQP"], dict, dict]:
    mdl = prediction_model(plan, period, cfg.order, wheel_damping=cfg.wheel_damping_eq, tol_v=cfg.order_tol_v,
                           tol_pitch_deg=cfg.order_tol_pitch_deg)
    if not mdl.get("stable", True):
        return None, mdl, {}
    lim = derive_limits(plan, cfg)
    a_max = cfg.a_max if cfg.a_max is not None else lim["a_max"]
    j_max = cfg.j_max if cfg.j_max is not None else (a_max / cfg.jerk_rise_s if cfg.a_max is not None else lim["j_max"])
    N = int(round(cfg.horizon_s / period))
    wkey = (cfg.w_v, cfg.w_p, cfg.w_r, cfg.w_a, cfg.w_u, cfg.w_x_up, cfg.w_x_down, cfg.w_terminal)
    key = _plan_key(plan, mdl["L0"]) + (round(period, 9), cfg.order, N, cfg.v_max, round(a_max, 9), round(j_max, 9),
                                        cfg.wheel_damping_eq) + wkey
    if key not in _QP_CACHE:
        _QP_CACHE[key] = ReferenceQP(mdl["red"], N, cfg.v_max, a_max, j_max, cfg, mdl["dc"])
    return _QP_CACHE[key], mdl, dict(lim, a_used=a_max, j_used=j_max)


# ------------------------------------------------------------------------------------------------ host
class MpcHost:
    """Host process with the MPC reference generator (same interface as HostEmulator plus the stage-2 hooks).

    Attributes: enabled, cmd (requested MotionCtrl: forward = TARGET velocity), sent (last published MotionCtrl),
    violations, solve_times_ms, qp_failures, fallback_used, force_fail, events, trace."""

    def __init__(self, link, plan, publish_hz: float = 50.0, horizon_s: Optional[float] = None,
                 config: Optional[MpcConfig] = None):
        self.link = link
        self.plan = plan
        self.cfg = copy.deepcopy(config) if config is not None else MpcConfig()
        if horizon_s is not None:
            self.cfg.horizon_s = float(horizon_s)
        self.period = 1.0 / float(publish_hz)
        try:
            self.qp, self.model, self.limits = reference_qp(plan, self.cfg, self.period)
        except Exception as exc:          # unsupported inner-loop structure etc.: never take the robot down with us
            self.qp, self.model, self.limits = None, dict(red=None, stable=False, error=repr(exc)), {}
        self.model_ok = self.qp is not None
        self.red = self.model["red"]
        self.cmd = MotionCtrl()
        self.sent = MotionCtrl()
        self.enabled = True
        self.violations = 0
        self.solve_times_ms: List[float] = []
        self.qp_failures = 0                 # solves without any solution (-> fallback)
        self.solver_failures = 0             # individual solver failures (primary failed, backup tried)
        self.fallback_used = False
        self.force_fail = False
        self.events: List[tuple] = []
        self.trace: Dict[str, list] = {k: [] for k in ("t", "target", "r", "a", "mode", "solve_ms", "v_meas", "v_pred",
                                                        "pitch_meas", "pitch_pred", "x_meas", "v_avg", "v_pred_prev")}
        self._y_meas = None
        self._tf0 = 0.0
        self.fallback = RampFallback(self.period, self.cfg.fallback_rate, self.cfg.fallback_tau, self.cfg.v_max)
        self.mode = "mpc"
        self.next_pub = 0.0
        self.last_sent = -1e9
        self.last_tick = None
        self._r, self._a = 0.0, 0.0
        if not self.model_ok:
            # inner closed loop unstable at this configuration: the MPC cannot predict anything; run the model-free
            # stage-1 shaping permanently (the robot is not expected to balance in this case anyway)
            self.mode = "fallback"
            self.fallback_used = True
            self._fallback_since = 0.0
            why = self.model.get("error") or "inner closed loop unstable (spectral radius %.4f)" % self.model.get(
                "spectral_radius", float("nan"))
            self.events.append((0.0, f"MPC disabled: {why}; stage-1 ramp shaping only"))
            self._need_sync = True
            return
        self.refmodel = ReferenceModel(self.period, self.cfg.ref_omega_up, self.cfg.ref_omega_down, self.qp.N)
        self._fallback_since = -1e9
        self.next_pub = 0.0
        self.last_sent = -1e9
        self.last_tick = None
        self._x = np.zeros(self.red.n)       # reference-induced closed-loop state (reduced coordinates)
        self._r = 0.0                        # last published reference
        self._a = 0.0                        # its rate of change (m/s^2)
        self._U = None                       # last solution (warm start)
        self._need_sync = True
        self._ir = self.red.iu("r")
        self._Br = self.red.B[:, self._ir]
        self._Dr = self.red.D[:, self._ir]
        self._x_ss_per_v = np.linalg.solve(np.eye(self.red.n) - self.red.A, self._Br)
        key = ("obs", id(self.red), self.cfg.obs_sigma_v, self.cfg.obs_sigma_w, self.cfg.obs_sigma_f, self.cfg.obs_sigma_z)
        if key not in _MODEL_CACHE:
            _MODEL_CACHE[key] = prediction_observer(self.red, self.cfg.obs_sigma_v, self.cfg.obs_sigma_w, self.cfg.obs_sigma_f,
                                                    self.cfg.obs_sigma_z)
        self.observer = _MODEL_CACHE[key]
        self._iv = self.red.iy("v")
        self._acc_v, self._acc_n = 0.0, 0     # telemetry speed averaged over the publish period
        self._v_pred_prev = None             # predicted wheel speed at the previous publish instant
        self._t_target_zero = -1e9           # time the target velocity last became zero
        self._was_active = False
        self._last_target = 0.0
        self.corrections = 0

    # ------------------------------------------------------------------ interface
    def set_command(self, **fields) -> None:
        for k, v in fields.items():
            if hasattr(self.cmd, k):
                setattr(self.cmd, k, v)
            elif hasattr(self.cmd.mode, k):
                setattr(self.cmd.mode, k, v)
            else:
                raise KeyError(k)

    def tick(self, t: float) -> None:
        if not self.enabled:
            return
        if self.last_tick is not None and (t - self.last_tick) > self.cfg.resync_gap_s:
            self._need_sync = True                       # the host was dead / paused
        self.last_tick = t
        e = self._est()
        if self.model_ok and e is not None and math.isfinite(e.v):
            self._acc_v += float(e.v)
            self._acc_n += 1
        if t >= self.next_pub - 1e-9:
            if self._need_sync:
                self._resync(t)
            self._update(t)
            self._send(t)
            self.next_pub += self.period                 # fixed publish grid (no drift from float time)
            if self.next_pub <= t:
                self.next_pub = t + self.period
        elif (t - self.last_sent) * 1e3 >= self.link.p.heartbeat_ms:
            self._send(t)

    # ------------------------------------------------------------------ internals
    def _est(self):
        mcu = getattr(self.plan, "mcu", None)
        return None if mcu is None else mcu.est

    def _resync(self, t: float) -> None:
        """Start-up / after an outage: assume the MCU tracks steadily and take the measured ground speed as the current
        reference (telemetry only: est.v)."""
        if not self.model_ok:
            self._r, self._a = 0.0, 0.0
            self.fallback.reset(0.0)
            self._need_sync = False
            return
        e = self._est()
        v = float(e.v) if (e is not None and math.isfinite(e.v)) else 0.0
        if self.last_sent < -1e8 and abs(v) < 0.05:      # first packet with the robot at rest: start from zero
            v = 0.0
        v = max(-self.cfg.v_max, min(self.cfg.v_max, v))
        self._r, self._a = v, 0.0
        self._x = self._x_ss_per_v * v
        self._U = None
        self.fallback.reset(v)
        self.refmodel.reset(v)
        self._v_pred_prev = None
        self._acc_v, self._acc_n = 0.0, 0
        self._need_sync = False
        if self.last_sent > -1e8:
            self.events.append((t, f"resync from telemetry (v = {v:+.3f} m/s)"))

    def _target(self) -> float:
        return max(-self.cfg.v_max, min(self.cfg.v_max, float(self.cmd.forward)))

    def _correct(self, t: float, v_star: float) -> None:
        """Telemetry correction of the slow mode, only while a manoeuvre is in progress (target != 0, or less than
        obs_hold_s after it returned to 0): at stand-still the reference stays exactly 0 and disturbances are left to
        the inner loop (stage-1 behaviour).  The period-averaged est.v (latency d plus the half-period average ~ one
        host period) is compared with the prediction made one period earlier."""
        if v_star == 0.0 and self._last_target != 0.0:
            self._t_target_zero = t
        self._last_target = v_star
        y = self._acc_v / self._acc_n if self._acc_n else None
        self._acc_v, self._acc_n = 0.0, 0
        active = self.cfg.use_observer and (v_star != 0.0 or (t - self._t_target_zero) < self.cfg.obs_hold_s)
        if self.cfg.handover and v_star == 0.0 and not active and self._was_active:
            # end of the manoeuvre: what is left in the prediction state is the LQI6 integrator's pay-back of the
            # stop; the host does not fight it (no correction any more), so the plan restarts from rest
            self._x = np.zeros_like(self._x)
            self._U = None
            self.events.append((t, "handover to the inner loop (manoeuvre finished)"))
        self._was_active = active
        self._y_meas = y
        if active and y is not None and self._v_pred_prev is not None:
            err = y - self._v_pred_prev
            self._x = self._x + self.observer["AL"] * err
            self.corrections += 1

    def _update(self, t: float) -> None:
        v_star = self._target()
        if not self.model_ok:
            r_prev = self._r
            self._r = self.fallback.step(v_star)
            self._a = (self._r - r_prev) / self.period
            return
        self._correct(t, v_star)
        r_prev, a_prev = self._r, self._a
        x0 = self.qp.x0(self._x, r_prev, a_prev)
        v_des = self.refmodel.predict(v_star)
        tf_now = self._friction(v_des[:1])
        self._tf0 = 0.0 if tf_now is None else float(tf_now[0])
        r_new, solve_ms, ok = None, float("nan"), False
        if self.mode == "fallback":
            # stay in the ramp fallback at least fallback_hold_s and until it is steady; then retry the MPC
            if (t - self._fallback_since) >= self.cfg.fallback_hold_s and self.fallback.steady and not self.force_fail \
                    and abs(self._a) < 1e-6:
                self.refmodel.reset(r_prev)
                v_des = self.refmodel.predict(v_star)
                ok, U, solve_ms = self._solve(x0, v_star, t, v_des)
                if ok:
                    self.mode = "mpc"
                    self.events.append((t, "MPC resumed"))
                    r_new = self._accept(x0, U, r_prev, a_prev)
            if r_new is None:
                r_new = self._fallback_step(x0, v_star, v_des, r_prev, a_prev, t)
        else:
            ok, U, solve_ms = self._solve(x0, v_star, t, v_des)
            if ok:
                r_new = self._accept(x0, U, r_prev, a_prev)
            else:
                self.mode = "fallback"
                self.fallback_used = True
                self._fallback_since = t
                self.fallback.reset(r_prev)
                kind = ("explicit unconstrained MPC law + ramp limiter" if self.cfg.fallback_mode == "explicit"
                        else "stage-1 ramp shaping")
                self.events.append((t, f"fallback engaged: {kind} (QP failed or unavailable)"))
                r_new = self._fallback_step(x0, v_star, v_des, r_prev, a_prev, t)
        self.refmodel.step(v_star)
        if abs(self._r) < 1e-9 and abs(self._a) < 1e-9:
            self._r, self._a = 0.0, 0.0                   # exact zero at stand-still (no solver round-off)
        self._v_pred_prev = float(self.red.C[self._iv] @ self._x + self._Dr[self._iv] * self._r
                                  + (self.red.D[self._iv, self.red.iu("Tf")] * self._tf0 if "Tf" in self.red.inputs else 0.0))
        # propagate the prediction state with the reference that is applied now (and the assumed friction torque)
        x_next = self.red.A @ self._x + self._Br * self._r
        if self._tf0 != 0.0:
            x_next = x_next + self.red.B[:, self.red.iu("Tf")] * self._tf0
        if self.cfg.keep_trace:
            self._log(t, v_star, solve_ms)
        self._x = x_next

    def _fallback_step(self, x0, v_star, v_des, r_prev, a_prev, t) -> float:
        """Fallback reference for this period.  "explicit": first move of the unconstrained optimum, limited like the
        MPC's constraints (|Delta a| <= j_max T, |a| <= a_max, |r| <= v_max); "ramp" (or a non-finite explicit move):
        the stage-1 shaping."""
        q = self.qp
        if self.cfg.fallback_mode == "explicit":
            try:
                mode = self.refmodel.mode(v_star)
                tf = self._friction(v_des[:q.N]) if v_des is not None else None
                u0 = q.explicit_first_move(x0, v_star, v_des, mode, tf)
            except Exception:                         # no linear algebra either: last resort below
                u0 = float("nan")
            if math.isfinite(u0):
                u0 = max(-q.j_max * self.period, min(q.j_max * self.period, u0))
                a_new = max(-q.a_max, min(q.a_max, a_prev + u0))
                r_new = r_prev + self.period * a_new
                r_new = max(-q.v_max, min(q.v_max, r_new))
                self._a = (r_new - r_prev) / self.period
                self._r = r_new
                self.fallback.reset(r_new)            # keeps the last-resort shaper continuous
                return r_new
            self.events.append((t, "explicit fallback not finite: stage-1 ramp shaping"))
        r_new = self.fallback.step(v_star)
        self._a = (r_new - r_prev) / self.period
        self._r = r_new
        return r_new

    def _friction(self, v_plan: np.ndarray) -> Optional[np.ndarray]:
        """Known joint dry friction on the wheels, opposite to the planned motion (identified magnitude)."""
        F = self.cfg.friction_ff_Nm
        if F <= 0.0 or not self.qp.has_tf:
            return None
        return -F * np.tanh(np.asarray(v_plan, dtype=float) / self.cfg.friction_v_s)

    def _solve(self, x0, v_star, t, v_des=None):
        mode = self.refmodel.mode(v_star)
        tf = self._friction(v_des[:self.qp.N]) if v_des is not None else None
        if self.force_fail:
            self.qp_failures += 1
            self.events.append((t, "QP failed (forced)"))
            return False, None, float("nan")
        warm = None
        if self._U is not None:
            warm = np.concatenate([self._U[1:], [0.0]])
        ok, U, ms_total = False, None, 0.0
        for solver in (self.cfg.solver, self.cfg.backup_solver):
            ms = float("nan")
            try:
                ok, U, ms = self.qp.solve(x0, v_star, warm=warm, solver=solver, v_des=v_des, mode=mode, tf=tf)
            except Exception as exc:              # solver missing / DLL error: try the backup, then fall back
                ok, U = False, None
                self.events.append((t, f"QP solver {solver} unavailable: {exc!r}"))
            if math.isfinite(ms):
                ms_total += ms
            if ok:
                if solver != self.cfg.solver:
                    self.events.append((t, f"QP solved by the backup solver {solver}"))
                break
            self.solver_failures += 1
            self.events.append((t, f"QP infeasible / failed ({solver})"))
        self.solve_times_ms.append(ms_total)          # time of the whole solve (primary + backup if needed)
        if not ok:
            self.qp_failures += 1                     # no solution from any solver -> fallback
        return ok, U, ms_total

    def _accept(self, x0, U, r_prev, a_prev) -> float:
        self._U = U
        a_new = a_prev + float(U[0])
        r_new = r_prev + self.period * a_new
        self._check_violations(x0, U, r_prev, a_prev, r_new, a_new)
        self._r, self._a = r_new, a_new
        return r_new

    def _check_violations(self, x0, U, r_prev, a_prev, r_new, a_new) -> None:
        tol = self.cfg.violation_tol
        q = self.qp
        bad = 0
        if abs(r_new) > q.v_max * (1 + tol) + 1e-9:
            bad += 1
        if abs(a_new) > q.a_max * (1 + tol) + 1e-9:
            bad += 1
        if abs(a_new - a_prev) > q.j_max * self.period * (1 + tol) + 1e-9:
            bad += 1
        traj = q.trajectory(x0, U)                      # the whole planned trajectory must respect the limits too
        if np.any(np.abs(traj["r"]) > q.v_max * (1 + tol) + 1e-9) or np.any(np.abs(traj["a"]) > q.a_max * (1 + tol) + 1e-9) \
                or np.any(np.abs(U) > q.j_max * self.period * (1 + tol) + 1e-9):
            bad += 1
        self.violations += bad

    def _leg_up(self) -> float:
        """Pass `up` through, clipped so that the commanded leg length stays in [L_min, L_max] (position mode)."""
        up = float(self.cmd.up)
        mcu = getattr(self.plan, "mcu", None)
        if mcu is None or self.cmd.mode.height_ctrl_mode:
            return up
        p = mcu.p
        lo = (p.L_min - p.L_stand) / p.height_gain
        hi = (p.L_max - p.L_stand) / p.height_gain
        return max(lo, min(hi, up))

    def _send(self, t: float) -> None:
        out = copy.deepcopy(self.cmd)
        if self.mode == "fallback":
            # QP failed / unavailable: hand the raw target to the MCU, which applies its own stage-1 shaping (ramp +
            # pre-filter + integrator freeze) because host_shaped is False
            out.forward = float(self._target())
            out.host_shaped = False
        else:
            out.forward = float(self._r)
            out.host_shaped = True                        # the MCU tracks the MPC reference plainly
        out.up = self._leg_up()
        out.pitch = 0.0                                   # the host never sends a pitch reference
        out.mode.pitch_ctrl_mode = False
        self.sent = out
        self.link.receive(out, t)
        self.last_sent = t

    def _log(self, t: float, v_star: float, solve_ms: float) -> None:
        e = self._est()
        tr = self.trace
        tr["t"].append(t)
        tr["target"].append(v_star)
        tr["r"].append(self._r)
        tr["a"].append(self._a)
        tr["mode"].append(0 if self.mode == "mpc" else 1)
        tr["solve_ms"].append(solve_ms)
        y = self.red.C @ self._x + self._Dr * self._r
        tr["v_pred"].append(float(y[self.red.iy("v")]))
        tr["pitch_pred"].append(float(y[self.red.iy("pitch")]))
        tr["v_meas"].append(float(e.v) if e is not None else float("nan"))
        tr["pitch_meas"].append(float(e.pitch) if e is not None else float("nan"))
        tr["x_meas"].append(float(e.position) if e is not None else float("nan"))
        tr["v_avg"].append(float(self._y_meas) if self._y_meas is not None else float("nan"))
        tr["v_pred_prev"].append(float(self._v_pred_prev) if self._v_pred_prev is not None else float("nan"))

    # ------------------------------------------------------------------ reporting
    def stats(self) -> dict:
        st = np.asarray(self.solve_times_ms, dtype=float)
        st = st[np.isfinite(st)]
        if not self.model_ok:
            return dict(n_solves=0, solve_p50_ms=None, solve_p99_ms=None, solve_max_ms=None, qp_failures=int(self.qp_failures),
                        solver_failures=0, violations=0, fallback_used=True, n_var=None, n_ineq=None, horizon_steps=None,
                        a_max=None, j_max=None, order=None, model_ok=False)
        return dict(n_solves=int(st.size), solve_p50_ms=float(np.percentile(st, 50)) if st.size else None,
                    solve_p99_ms=float(np.percentile(st, 99)) if st.size else None,
                    solve_max_ms=float(st.max()) if st.size else None, qp_failures=int(self.qp_failures),
                    solver_failures=int(self.solver_failures),
                    violations=int(self.violations), fallback_used=bool(self.fallback_used),
                    n_var=int(self.qp.n_var), n_ineq=int(self.qp.n_ineq), horizon_steps=int(self.qp.N),
                    a_max=float(self.qp.a_max), j_max=float(self.qp.j_max), order=int(self.red.n))
