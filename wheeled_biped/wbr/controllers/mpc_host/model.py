"""Prediction model of the stage-2 MPC host: the stage-1 LQI6 closed loop, reduced to the velocity / pitch axes.

Construction (docs/plan4_mpc_host.md, section 2):

1. plant  ``plan.plant.linearize(L0)`` -> A (6x6, [theta, theta_dot, x, x_dot, phi, phi_dot]), B (6x2, [T, Tp]).
   The wheel position x does not enter the controller, so the 5-state plant s = [theta, theta_dot, v, phi, phi_dot]
   is discretised with a zero-order hold at the MCU period Ts (the same c2d as ``LQI6.design``).
2. controller exactly as ``LQI6.step`` executes it in MPC mode while the reference moves (no pre-filter, every
   integrator frozen, ``lqi6_options`` -> use_int = False):
       u_k = -K_5 [theta_m, theta_dot_m, v_m - r_k, phi_m, phi_dot_m]
   where the measured state m_k = s_{k-d} is the plant state delayed by the observation latency d (steps), kept in a
   delay chain.  K_5 = the first five columns of ``ctrl.gains(L0)`` (the MCU's own gain table, ``select_gains``).
   The integrator variants (use_int / att_gM / use_pos) remain available for the stand-still regime
   (``regime="rest"``), used only to compare the model with push responses.
3. inputs  r (the velocity reference seen by the LQI6), F (horizontal force at the total COM, used to validate the
   push response) and Tf (external wheel torque, e.g. joint dry friction: the process-noise direction of the host's
   observer); outputs v (wheel ground speed), v_base (torso origin, = the judged ``vx``), pitch (body, IMU sign),
   theta (virtual-leg angle deviation), T (total wheel torque), Tp (total hip torque deviation).
4. ``lift``: the host holds r over its period (m MCU steps): A_h = A^m, B_h = sum A^i B.
5. ``reduce``: square-root balanced truncation with singular-perturbation DC matching (steady-state gains kept
   exactly), on the (r, Tf) channels (the MPC never sees F).

No integrator is added by the host: the only integrator in the prediction is the LQI6's own (part of the plant of
the MPC).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Sequence, Tuple

import numpy as np
import scipy.linalg as la

from wbr.controllers.lqi6.lqi6 import c2d_zoh

OUTPUTS = ("v", "v_base", "pitch", "theta", "T", "Tp")


@dataclass
class LTIModel:
    """x+ = A x + B u,  y = C x + D u  (discrete time, period Ts)."""
    A: np.ndarray
    B: np.ndarray
    C: np.ndarray
    D: np.ndarray
    Ts: float
    inputs: Tuple[str, ...]
    outputs: Tuple[str, ...]
    info: Dict = field(default_factory=dict)

    @property
    def n(self) -> int:
        return self.A.shape[0]

    def iy(self, name: str) -> int:
        return self.outputs.index(name)

    def iu(self, name: str) -> int:
        return self.inputs.index(name)

    def select(self, inputs: Sequence[str] | None = None, outputs: Sequence[str] | None = None) -> "LTIModel":
        iu = [self.iu(n) for n in (inputs or self.inputs)]
        iy = [self.iy(n) for n in (outputs or self.outputs)]
        return LTIModel(self.A, self.B[:, iu], self.C[iy, :], self.D[np.ix_(iy, iu)], self.Ts,
                        tuple(self.inputs[i] for i in iu), tuple(self.outputs[i] for i in iy), dict(self.info))

    def simulate(self, U: np.ndarray, x0: np.ndarray | None = None) -> Tuple[np.ndarray, np.ndarray]:
        """U: (N, nu) -> Y (N, ny) (output at the start of each step, before the step's input acts on the state,
        except for the direct feed-through D) and the final state."""
        U = np.atleast_2d(np.asarray(U, dtype=float))
        if U.shape[0] == 1 and U.shape[1] != len(self.inputs):
            U = U.T
        x = np.zeros(self.n) if x0 is None else np.asarray(x0, dtype=float).copy()
        Y = np.zeros((U.shape[0], self.C.shape[0]))
        for k, u in enumerate(U):
            Y[k] = self.C @ x + self.D @ u
            x = self.A @ x + self.B @ u
        return Y, x

    def dc_gain(self) -> np.ndarray:
        return self.C @ np.linalg.solve(np.eye(self.n) - self.A, self.B) + self.D

    def to_json(self) -> dict:
        return dict(A=self.A.tolist(), B=self.B.tolist(), C=self.C.tolist(), D=self.D.tolist(), Ts=self.Ts,
                    inputs=list(self.inputs), outputs=list(self.outputs), info=self.info)

    @classmethod
    def from_json(cls, obj: dict) -> "LTIModel":
        return cls(np.asarray(obj["A"], float), np.asarray(obj["B"], float), np.asarray(obj["C"], float),
                   np.asarray(obj["D"], float), float(obj["Ts"]), tuple(obj["inputs"]), tuple(obj["outputs"]),
                   dict(obj.get("info", {})))


# ----------------------------------------------------------------------------------------------- full closed loop
def push_input_column(plant, L0: float, total_mass_with_wheels: float | None = None) -> np.ndarray:
    """Continuous-time input column (state order [theta, theta_dot, x, x_dot, phi, phi_dot]) of a horizontal force
    applied at the robot's total COM, a material point of the torso (acceptance-criteria push).  Generalised force
    Q = J^T F with J = d p / d (x, theta, phi) of that torso point at the equilibrium."""
    P = plant.params(L0)
    R, m_w, I_w, m_l, c, d, I_l, m_b, b_x, b_z, I_b, L0v = P[:12]
    th = plant.theta_eq(L0)
    m_tot = total_mass_with_wheels or (m_w + m_l + m_b)
    z_leg = -d * np.sin(th) + c * np.cos(th)
    z_body = L0v * np.cos(th) + b_z
    z_com = (m_l * z_leg + m_b * z_body) / m_tot          # above the axle (wheels at the axle height)
    o_z = z_com - L0v * np.cos(th)                       # application point relative to the hip (world = body frame)
    J = np.array([1.0, L0v * np.cos(th), -o_z])          # d p_x / d (x, theta, phi)
    M = plant.mass_matrix(th, 0.0, L0)
    qdd = np.linalg.solve(M, J)                          # per newton
    col = np.zeros(6)
    col[1], col[3], col[5] = qdd[1], qdd[0], qdd[2]
    return col


def lqi6_closed_loop(plant, K: np.ndarray, L0: float, Ts: float, delay_steps: int = 0, h_base: float = 0.164,
                     rate_lpf_tau: float = 0.0, wheel_radius: float | None = None, att_gM: float | None = None,
                     use_pos: bool = False, pred: tuple | None = None, use_int: bool = True) -> LTIModel:
    """Discrete closed loop plant + LQI6 at the MCU period Ts, exactly as ``LQI6.step`` executes it in MPC mode
    (no pre-filter).  Feedback vector xs = [theta, theta_dot, v - r, phi, phi_dot, (z), (z_att), (z_pos)],
    u = -K xs with K = 2 x len(xs).  use_int=False drops the velocity integrator z: the manoeuvre regime of the
    stage-2 inner loop, whose integrators are all frozen while the host reference accelerates (lqi6_options).

    State X = [s_k (5), z, (z_att), (z_pos), (u_{k-1}, ..., u_{k-N} : 2 N), (f1, f2), s_{k-1} .. s_{k-d} (5 d)]
      z_k     = z_{k-1} + Ts (e_v - r)
      z_att_k = z_att_{k-1} + Ts (e_theta + e_phi + (Tp_{k-1} - T_{k-1}) / gM)          (q_att, att_gM = g M(L0))
      z_pos_k = z_pos_{k-1} + Ts z_k                                                   (q_pos)
      pred = (P = Ad^N, G = [Bd, Ad Bd, ...]): the feedback uses x_hat = P e + G [u_{k-1}; ...; u_{k-N}] for the five
      balance states (LQI6 pred_steps); the integrators keep the measured signals.
    The measured e = s_{k-d} (observation latency d), with the optional encoder-rate low-pass rate_lpf_tau
    (body-frame leg rate f1 = LPF(theta_dot + phi_dot), wheel-carrier rate f2 = LPF(v / R + phi_dot)).
    Inputs [r, F, Tf], outputs OUTPUTS.  Saturations / clamps / conditional integration are not modelled.
    """
    A6, B6 = plant.linearize(L0)
    keep = [0, 1, 3, 4, 5]
    A5, B5 = A6[np.ix_(keep, keep)], B6[keep, :]
    BF5 = push_input_column(plant, L0)[keep][:, None]
    Ad, Bd = c2d_zoh(A5, np.hstack([B5, BF5]), Ts)
    Bu, BF = Bd[:, :2], Bd[:, 2:]
    BT = Bd[:, 0:1]                                  # external wheel torque (friction, ...): enters like T, not through K
    R = float(plant.p.R if wheel_radius is None else wheel_radius)
    K = np.asarray(K, dtype=float)
    use_att = att_gM is not None
    N_pred = 0 if pred is None else int(np.asarray(pred[1]).shape[1] // 2)
    N_u = max(N_pred, 1 if use_att else 0)
    nK = 5 + int(use_int) + int(use_att) + int(use_pos)
    if K.shape != (2, nK):
        raise ValueError(f"gain matrix {K.shape} does not match the LQI6 options (expected (2, {nK}))")
    if use_pos and not use_int:
        raise ValueError("z_pos integrates z: use_pos requires use_int")
    d = int(delay_steps)
    nf = 2 if rate_lpf_tau > 0.0 else 0
    # state layout
    idx = 5
    i_z = idx if use_int else None; idx += int(use_int)
    i_za = idx if use_att else None; idx += int(use_att)
    i_zp = idx if use_pos else None; idx += int(use_pos)
    i_U = idx; idx += 2 * N_u
    i_f = idx; idx += nf
    i_d = idx
    n = i_d + 5 * d
    e = lambda i: np.eye(1, n, i)[0]
    # delayed raw measurement m = S X
    S = np.zeros((5, n))
    if d == 0:
        S[:, :5] = np.eye(5)
    else:
        S[:, i_d + 5 * (d - 1):i_d + 5 * d] = np.eye(5)
    # estimate (rates possibly low-passed) Se X
    Se = S.copy()
    F1 = F2 = None
    if nf:
        a = Ts / (rate_lpf_tau + Ts)
        F1 = (1 - a) * e(i_f) + a * (S[1] + S[4])
        F2 = (1 - a) * e(i_f + 1) + a * (S[2] / R + S[4])
        Se[1] = F1 - S[4]
        Se[2] = R * (F2 - S[4])
    # rows (coefficient over X, coefficient of r)
    Z = (e(i_z) + Ts * Se[2], -Ts) if use_int else None
    rows = []
    if pred is not None:
        P, G = np.asarray(pred[0], float), np.asarray(pred[1], float)
        EU = np.zeros((2 * N_pred, n))
        EU[:, i_U:i_U + 2 * N_pred] = np.eye(2 * N_pred)
        XH = P @ Se + G @ EU
        rows += [(XH[0], 0.0), (XH[1], 0.0), (XH[2], -1.0), (XH[3], 0.0), (XH[4], 0.0)]
    else:
        rows += [(Se[0], 0.0), (Se[1], 0.0), (Se[2], -1.0), (Se[3], 0.0), (Se[4], 0.0)]
    if use_int:
        rows.append(Z)
    ZA = ZP = None
    if use_att:
        ZA = (e(i_za) + Ts * (Se[0] + Se[3] + (e(i_U + 1) - e(i_U)) / float(att_gM)), 0.0)
        rows.append(ZA)
    if use_pos:
        ZP = (e(i_zp) + Ts * Z[0], Ts * Z[1])
        rows.append(ZP)
    Xs = np.stack([r_[0] for r_ in rows])            # xs = Xs X + xr r
    xr = np.array([r_[1] for r_ in rows])
    Cu = -K @ Xs                                     # u = Cu X + Du r
    Du = (-K @ xr)[:, None]
    A = np.zeros((n, n))
    B = np.zeros((n, 3))
    A[:5, :] = Ad @ np.eye(5, n) + Bu @ Cu
    B[:5, 0:1] = Bu @ Du
    B[:5, 1:2] = BF
    B[:5, 2:3] = BT
    if use_int:
        A[i_z, :], B[i_z, 0] = Z
    if use_att:
        A[i_za, :], B[i_za, 0] = ZA
    if use_pos:
        A[i_zp, :], B[i_zp, 0] = ZP
    if N_u:
        A[i_U:i_U + 2, :] = Cu                        # u_{k} becomes u_{k-1} of the next step
        B[i_U:i_U + 2, 0] = Du[:, 0]
        for j in range(1, N_u):
            A[i_U + 2 * j:i_U + 2 * j + 2, i_U + 2 * (j - 1):i_U + 2 * j] = np.eye(2)
    if nf:
        A[i_f, :] = F1
        A[i_f + 1, :] = F2
    if d >= 1:
        A[i_d:i_d + 5, :5] = np.eye(5)
        for i in range(1, d):
            A[i_d + 5 * i:i_d + 5 + 5 * i, i_d + 5 * (i - 1):i_d + 5 + 5 * (i - 1)] = np.eye(5)
    th_eq = plant.theta_eq(L0)
    C = np.zeros((len(OUTPUTS), n))
    D = np.zeros((len(OUTPUTS), 3))
    C[0, 2] = 1.0                                             # v
    C[1, 2], C[1, 1], C[1, 4] = 1.0, L0 * np.cos(th_eq), -h_base   # v_base = v + L0 cos(th) th_dot - h_b phi_dot
    C[2, 3] = -1.0                                            # pitch = -phi
    C[3, 0] = 1.0                                             # theta
    C[4, :], D[4, 0] = Cu[0], Du[0, 0]                        # T
    C[5, :], D[5, 0] = Cu[1], Du[1, 0]                        # Tp
    return LTIModel(A, B, C, D, Ts, ("r", "F", "Tf"), OUTPUTS,
                    dict(L0=float(L0), Ts_mcu=float(Ts), delay_steps=d, h_base=float(h_base), K=K.tolist(),
                         rate_lpf_tau=float(rate_lpf_tau), att_gM=None if att_gM is None else float(att_gM),
                         use_pos=bool(use_pos), pred_steps=N_pred, i_z=None if i_z is None else int(i_z), use_int=bool(use_int),
                         i_z_att=None if i_za is None else int(i_za)))


def lift(model: LTIModel, m: int) -> LTIModel:
    """Input held for m steps (host ZOH): x_{k+1} = A^m x_k + sum_{i<m} A^i B u_k; outputs sampled at the start."""
    A, B = model.A, model.B
    Am = np.eye(model.n)
    S = np.zeros_like(B)
    for _ in range(m):
        S = A @ S + B
        Am = A @ Am
    info = dict(model.info)
    info["lift"] = int(m)
    return LTIModel(Am, S, model.C.copy(), model.D.copy(), model.Ts * m, model.inputs, model.outputs, info)


def _psd_sqrt_factor(W: np.ndarray) -> np.ndarray:
    w, V = np.linalg.eigh(0.5 * (W + W.T))
    w = np.clip(w, 0.0, None)
    return V * np.sqrt(w)


def hankel_singular_values(model: LTIModel) -> np.ndarray:
    Wc = la.solve_discrete_lyapunov(model.A, model.B @ model.B.T)
    Wo = la.solve_discrete_lyapunov(model.A.T, model.C.T @ model.C)
    Lc, Lo = _psd_sqrt_factor(Wc), _psd_sqrt_factor(Wo)
    return np.linalg.svd(Lo.T @ Lc, compute_uv=False)


def reduce(model: LTIModel, order: int, input_scale: Sequence[float] | None = None,
           output_scale: Sequence[float] | None = None, dc_match: bool = True) -> LTIModel:
    """Square-root balanced truncation (discrete time) with optional singular-perturbation DC matching.
    input_scale / output_scale: typical magnitudes used to weight the channels in the Gramians (the reduced model is
    returned in the original units)."""
    su = np.ones(model.B.shape[1]) if input_scale is None else np.asarray(input_scale, float)
    sy = np.ones(model.C.shape[0]) if output_scale is None else np.asarray(output_scale, float)
    A, B, C, D = model.A, model.B * su, model.C / sy[:, None], model.D * su / sy[:, None]
    Wc = la.solve_discrete_lyapunov(A, B @ B.T)
    Wo = la.solve_discrete_lyapunov(A.T, C.T @ C)
    Lc, Lo = _psd_sqrt_factor(Wc), _psd_sqrt_factor(Wo)
    U, s, Vt = np.linalg.svd(Lo.T @ Lc)
    s_safe = np.maximum(s, 1e-14 * s[0])
    Tm = Lc @ Vt.T / np.sqrt(s_safe)                 # balancing transformation (columns)
    Ti = (U / np.sqrt(s_safe)).T @ Lo.T
    keep = len(s) if order >= len(s) else order
    # drop numerically unobservable / uncontrollable directions entirely
    nz = int(np.sum(s > 1e-12 * s[0]))
    Tm, Ti = Tm[:, :nz], Ti[:nz, :]
    Ab, Bb, Cb = Ti @ A @ Tm, Ti @ B, C @ Tm
    r = min(keep, nz)
    A11, A12, A21, A22 = Ab[:r, :r], Ab[:r, r:], Ab[r:, :r], Ab[r:, r:]
    B1, B2, C1, C2 = Bb[:r], Bb[r:], Cb[:, :r], Cb[:, r:]
    if dc_match and r < nz:
        X = np.linalg.solve(np.eye(nz - r) - A22, np.hstack([A21, B2]))
        Ar = A11 + A12 @ X[:, :r]
        Br = B1 + A12 @ X[:, r:]
        Cr = C1 + C2 @ X[:, :r]
        Dr = D + C2 @ X[:, r:]
    else:
        Ar, Br, Cr, Dr = A11, B1, C1, D
    info = dict(model.info)
    info.update(reduced_order=int(r), hsv=s[:min(len(s), 12)].tolist(), dc_match=bool(dc_match))
    red = LTIModel(Ar, Br / su, Cr * sy[:, None], Dr / su * sy[:, None], model.Ts, model.inputs, model.outputs, info)
    red.proj = Ti[:r, :]                      # x_reduced ~ proj @ x_full (balanced coordinates, not serialised)
    return red


# ----------------------------------------------------------------------------------------------- convenience
def plant_with_damping(plan, wheel_damping: float | None = None):
    """The plan's nominal plant, optionally with the wheel viscous damping c_w replaced by an identified equivalent
    value (joint dry friction under sensor noise acts like extra damping during manoeuvres, see identify.py)."""
    if wheel_damping is None:
        return plan.plant
    import copy
    from wbr.controllers.lqi6.plant_serial import SerialPlant
    p = copy.deepcopy(plan.params)
    p.c_w = float(wheel_damping)
    return SerialPlant(p)


def plan_closed_loop(plan, L0: float | None = None, latency_ms: float | None = None,
                     wheel_damping: float | None = None, regime: str = "manoeuvre") -> LTIModel:
    """Full-order MCU-rate closed loop of a Plan4UpkieLQI6 instance (design knowledge only: plant parameters, the
    LQI6 gain table, the MCU period and the configured sensing latency -- no simulator state).  The controller gains
    are always the MCU's own (designed on the nominal plant); wheel_damping only changes the plant of the prediction."""
    L0 = float(plan.params.L0) if L0 is None else float(L0)
    ctrl = plan.law.ctrl
    K = _gains_no_side_effect(ctrl, L0)
    lat = plan.cfg.latency_ms if latency_ms is None else latency_ms
    d = int(round(lat * 1e-3 / ctrl.Ts))
    lpf = float(getattr(plan.cfg, "rate_lpf_tau", 0.0) or 0.0)
    opts = lqi6_options(ctrl, L0, regime)
    cl = lqi6_closed_loop(plant_with_damping(plan, wheel_damping), select_gains(K, opts), L0, ctrl.Ts, d,
                          h_base=plan.hip_above_base, rate_lpf_tau=lpf, **model_kwargs(opts))
    cl.info["wheel_damping"] = float(plan.params.c_w if wheel_damping is None else wheel_damping)
    return cl


def lqi6_options(ctrl, L0: float, regime: str = "manoeuvre") -> dict:
    """The LQI6 structure of the MCU's controller at L0 as it acts DURING A MANOEUVRE -- the regime the prediction
    model is for.  z_att is gated on the low-passed reference acceleration (always frozen while the reference moves),
    and with the plain-mode acceleration freeze (LQI6.freeze_int_on_accel_plain, Plan 4 stage 2) z and z_pos are
    frozen as well: the prediction is then the pure state-feedback loop (review r1 stage 2 M4: the model that kept
    integrating z_att over-predicted the step pitch peak, 1.8 deg vs 1.0 deg).  gain_cols = the columns of the MCU
    gain matrix the model uses (select_gains); pass the rest with model_kwargs()."""
    if regime not in ("manoeuvre", "rest"):
        raise ValueError(regime)
    # "rest": stand-still regime (target 0, reference not moving): every designed integrator is active -- used to
    # compare the model with a push response, never for the MPC prediction
    frozen = bool(getattr(ctrl, "freeze_int_on_accel_plain", False)) and regime == "manoeuvre"
    gated_att = hasattr(ctrl, "att_accel_thresh") and regime == "manoeuvre"
    use_int = not frozen
    use_att = getattr(ctrl, "i_att", None) is not None and not frozen and not gated_att
    use_pos = getattr(ctrl, "i_pos", None) is not None and not frozen
    opts = dict(att_gM=float(ctrl.att_gM(L0)) if use_att else None, use_pos=use_pos, pred=None, use_int=use_int)
    if getattr(ctrl, "pred_steps", 0) > 0:
        if hasattr(ctrl, "_aux") and getattr(ctrl, "aux_table", None) is not None:
            P, G, _ = ctrl._aux(L0)
        else:
            P, G = ctrl.pred
        opts["pred"] = (np.array(P), np.array(G))
    cols = [0, 1, 2, 3, 4] + ([5] if use_int else []) + ([int(ctrl.i_att)] if use_att else []) + ([int(ctrl.i_pos)] if use_pos else [])
    opts["gain_cols"] = cols
    return opts


def select_gains(K: np.ndarray, opts: dict) -> np.ndarray:
    """Columns of the MCU gain matrix used by the prediction model (lqi6_options)."""
    cols = opts.get("gain_cols")
    K = np.asarray(K, dtype=float)
    return K if cols is None else K[:, cols]


def model_kwargs(opts: dict) -> dict:
    """lqi6_options() entries that lqi6_closed_loop() takes as keyword arguments."""
    return {k: v for k, v in opts.items() if k != "gain_cols"}


def _gains_no_side_effect(ctrl, L0: float) -> np.ndarray:
    """ctrl.gains() caches K / L0_last on the controller; evaluate the table without touching the MCU's object."""
    if ctrl.table is not None:
        grid, Ks = ctrl.table
        L0c = float(np.clip(L0, grid[0], grid[-1]))
        return np.array([[np.interp(L0c, grid, Ks[:, i, j]) for j in range(Ks.shape[2])] for i in range(Ks.shape[1])])
    K_saved, L_saved = ctrl.K, ctrl.L0_last
    try:
        return ctrl.design(L0).copy()
    finally:
        ctrl.K, ctrl.L0_last = K_saved, L_saved
