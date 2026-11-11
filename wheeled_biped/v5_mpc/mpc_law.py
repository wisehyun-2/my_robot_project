"""MPC balance law of the v5 robot: the only balance controller of v5_mpc (no LQI8 anywhere in this folder).

Where it sits: inside the MCU, in the balance-law slot where v5_lqi has the LQI8 (DiabloMcu.law).  Same inputs (the
MCU's estimate, the velocity command, the yaw loop's torque), same outputs (total wheel torque T, total hip torque Tp).
The four other controllers (height, yaw, roll, split), the VMC and the knee's coupling term are the MCU's own and the
same as in v5_lqi, so the two folders differ in the balance controller only.

Prediction model: the v5 serial-leg plant linearised at the current leg length (wbr.controllers.lqi8.plant_serial: the
robot model, shared with the LQI8 the way the URDF is), error states

    e = [theta, theta_dot, v - v_r, phi, phi_dot, z1 (, z2) (, z3)],   u = [T, Tp]   (Tp beyond the static hip torque)

    theta  virtual-leg angle from its equilibrium      phi  body pitch, COM behind the hip > 0     v  ground speed
    z1 = int(v - v_r)  position error   z2 = int(static body tilt)  (q_att)   z3 = int(z1)  (q_pos)

What makes it an MPC and not an LQR:
  1. preview with lead: a new velocity command starts the reference t_lead later (the ramp towards it at v_ramp_rate,
     smoothed by v_ref_tau); the MPC sees that reference over its horizon (v_r and its slope enter the exact
     discretisation), so the robot leans and starts rolling with the reference instead of after it.  z1 is held while
     the reference moves, as in the LQI8 (without the hold the position it lost during the start is caught up with a
     14-38 % velocity overshoot, whatever the lead: linear-model scan 11-10);
  2. constraints: every motor's torque over the first n_con moves -- wheel (T/2 +- tau_yaw/2), hip and knee (through
     the leg's VMC map at the current joint angles, with the static leg force, the split torque and the knee's
     wheel-reaction term of the other loops) -- within u_limit of the motor peak.  The QP (DAQP) is solved only when the
     unconstrained optimum breaks one of them; otherwise the explicit solution is the answer (same result, no solver).
Horizon: blocks of Ts, Ts, 2 Ts, 4 Ts, then n_steps x dt_mpc (1 kHz: 0.31 s, 34 moves), stage costs weighted by the
block length, terminal cost = the DARE solution at Ts.  The fine first blocks make the unconstrained first move the
LQR feedback of the control period itself (a uniform 10 ms grid made the hip gains 30-40 % softer).
Cost: Bryson weights, by default the numbers of the v5 LQI8, so a comparison is about the structure and not the tuning.
The condensed matrices are built on the leg-length grid at start-up and blended linearly at run time (no Riccati solve
in the loop); the MPC is re-solved every control period.
"""
from __future__ import annotations

import collections
import math
import time
from typing import Optional

import numpy as np
import scipy.linalg as la

_TABLES: dict = {}          # (model, weights, blocks) -> grid tables: plans built in one process share them


def static_gM(plant, theta_eq):
    """L0 -> g dSum(m x)/dtheta at the level-body equilibrium (legs + body, wheels at the axle): the leg-angle stiffness
    of the COM that turns the static body tilt into z2's rate (e_att = theta + phi + (Tp - T) / (g M))."""
    p = plant.p

    def gM(L0: float) -> float:
        leg = p.leg.at(L0)
        th = theta_eq(L0)
        return p.g * (leg["m_l"] * (leg["c"] * np.cos(th) - leg["d"] * np.sin(th)) + p.m_b * L0 * np.cos(th))
    return gM


class BalanceMPC:
    def __init__(self, plant, Ts: float = 1e-3, *, dt_mpc: float = 0.01, n_steps: int = 30, fine_blocks=(1, 1, 2, 4),
                 n_con: int = 10, q_theta: float = 0.3, q_thetad: float = 3.0, q_v: float = 0.5, q_phi: float = 0.15,
                 q_phid: float = 3.0, q_int: float = 0.2, q_att: Optional[float] = None, q_pos: Optional[float] = None,
                 att_gM=None, r_wheel_frac: float = 0.2, r_hip_frac: float = 0.3, table_L0=(0.37, 0.485, 0.005),
                 v_ramp_rate: float = 2.0, t_lead: float = 0.15, v_ref_tau: float = 0.1, preview: bool = True,
                 int_limit: float = 1.0, att_limit: float = 0.2, pos_limit: float = 0.5, freeze_z_on_ramp: bool = True,
                 freeze_tol: float = 0.05, att_accel_tau: float = 0.1, att_accel_thresh: float = 0.1, freeze_hold_s: float = 0.1,
                 solver: str = "daqp"):
        """Weights q_* are Bryson (largest acceptable value), r_*_frac shares of the plant's motor peaks.
        v_ramp_rate: acceleration of the velocity reference [m/s^2]; t_lead: the reference starts this long after the
        command (what the preview looks ahead into); v_ref_tau: first-order filter on the reference while speeding up
        (0 = off).  freeze_z_on_ramp: hold z1 while the reference moves (the LQI8's rule; off: catch-up overshoot)."""
        if q_att is not None and att_gM is None:
            raise ValueError("q_att needs att_gM (L0 -> g M(L0))")
        self.plant, self.Ts, self.dt_mpc = plant, Ts, dt_mpc
        self.d = np.array([min(f * Ts, dt_mpc) for f in fine_blocks] + [dt_mpc] * int(n_steps))     # block lengths [s]
        self.t_start = np.concatenate([[0.0], np.cumsum(self.d)[:-1]])                             # block start times
        self.N = len(self.d)
        self.n_con = min(int(n_con), self.N)
        self.q_att, self.q_pos, self.att_gM = q_att, q_pos, att_gM
        q = [q_theta, q_thetad, q_v, q_phi, q_phid, q_int] + ([q_att] if q_att is not None else []) + ([q_pos] if q_pos is not None else [])
        self.n = len(q)
        self.i_att = 6 if q_att is not None else None
        self.i_pos = (7 if q_att is not None else 6) if q_pos is not None else None
        self.Q = np.diag([1.0 / x ** 2 for x in q])
        self.R = np.diag([1.0 / (r_wheel_frac * plant.tau_wheel_max) ** 2, 1.0 / (r_hip_frac * plant.tau_hip_max) ** 2])
        self.v_ramp_rate, self.v_ref_tau, self.preview = v_ramp_rate, v_ref_tau, preview
        self.n_lead = int(round(t_lead / Ts))
        self.int_limit, self.att_limit, self.pos_limit = int_limit, att_limit, pos_limit
        self.freeze_z_on_ramp, self.freeze_tol = freeze_z_on_ramp, freeze_tol
        self.att_accel_tau, self.att_accel_thresh = att_accel_tau, att_accel_thresh
        self._freeze_hold_steps = int(round(freeze_hold_s / Ts))
        self.solver = solver
        self._build_tables(*table_L0)
        self.qp_calls = self.qp_failures = 0
        self.qp_time_max_ms = 0.0
        self.reset()

    def reset(self) -> None:
        self.z = self.z_att = self.z_pos = 0.0
        self.v_ramp = self.v_filt = 0.0
        self._cmd = collections.deque([0.0] * (self.n_lead + 1), maxlen=self.n_lead + 1)   # [-1] now, [0] t_lead ago
        self._a_ref_lp = 0.0
        self._freeze_countdown = 0
        self.u_prev = np.zeros(2)
        self._last_out = (0.0, 0.0)
        self.nonfinite_steps = 0
        self.constrained = False                # the last step's first move sat on a constraint (QP solution)

    # ------------------------------------------------------------------ design (start-up)
    def _model(self, L0: float):
        """Continuous error model at L0: (Aa, Ba, Fa, Ga): de/dt = Aa e + Ba u + Fa v_r + Ga a_r."""
        A, B = self.plant.linearize(L0)
        keep = [0, 1, 3, 4, 5]                  # [theta, theta_dot, v, phi, phi_dot]: the position x is z1 here
        A5, B5 = A[np.ix_(keep, keep)], B[keep, :]
        n = self.n
        Aa, Ba = np.zeros((n, n)), np.zeros((n, 2))
        Aa[:5, :5], Ba[:5, :] = A5, B5
        Aa[5, 2] = 1.0                          # z1' = v - v_r
        if self.i_att is not None:              # z2' = theta + phi + (Tp - T) / (g M)
            gM = self.att_gM(L0)
            Aa[self.i_att, 0] = Aa[self.i_att, 3] = 1.0
            Ba[self.i_att, :] = [-1.0 / gM, 1.0 / gM]
        if self.i_pos is not None:              # z3' = z1
            Aa[self.i_pos, 5] = 1.0
        Fa, Ga = np.zeros(n), np.zeros(n)
        Fa[:5] = A5[:, 2]                       # e5 = s5 - v_r e_v: the plant's own speed terms act on v_r too
        Ga[2] = -1.0                            # d(v - v_r)/dt = ... - a_r
        return Aa, Ba, Fa, Ga

    def _disc(self, model, d: float):
        """Exact discretisation over d with u held and v_r ramping at a_r: (Phi, Gamma, P_v, P_a)."""
        Aa, Ba, Fa, Ga = model
        n = self.n
        M = np.zeros((n + 4, n + 4))            # [e, v_r, a_r, u]: v_r' = a_r, a_r' = 0, u' = 0
        M[:n, :n], M[:n, n], M[:n, n + 1], M[:n, n + 2:] = Aa, Fa, Ga, Ba
        M[n, n + 1] = 1.0
        E = la.expm(M * d)
        return E[:n, :n], E[:n, n + 2:], E[:n, n], E[:n, n + 1]

    def _design(self, L0: float):
        """Condensed horizon at L0: H, and Kx, Kv, Ka with U* = -(Kx e0 + Kv vr + Ka ar) (unconstrained optimum)."""
        n, N = self.n, self.N
        model = self._model(L0)
        cache = {}
        blocks = []
        for d in self.d:
            k = round(float(d), 9)
            if k not in cache:
                cache[k] = self._disc(model, float(d))
            blocks.append(cache[k])
        Phi1, Gam1, _, _ = cache.get(round(self.Ts, 9)) or self._disc(model, self.Ts)
        P = la.solve_discrete_are(Phi1, Gam1, self.Q, self.R)
        Sx = np.zeros((n * N, n))
        Su = np.zeros((n * N, 2 * N))
        Sv = np.zeros((n * N, N))
        Sa = np.zeros((n * N, N))
        X = np.eye(n)
        Tm = []
        for i, (Phi, _, _, _) in enumerate(blocks):     # block row i: e_{i+1}
            Tm = [Phi @ T for T in Tm] + [np.eye(n)]    # Tm[j] = Phi_i ... Phi_{j+1}
            X = Phi @ X
            r = slice(i * n, (i + 1) * n)
            Sx[r] = X
            for j, Tj in enumerate(Tm):
                _, Gj, Pvj, Paj = blocks[j]
                Su[r, 2 * j:2 * j + 2] = Tj @ Gj
                Sv[r, j] = Tj @ Pvj
                Sa[r, j] = Tj @ Paj
        wq = self.d / self.Ts                   # stage costs in units of one control period
        Qb = la.block_diag(*([self.Q * wq[j] for j in range(N - 1)] + [P]))
        W = Su.T @ Qb
        H = W @ Su + la.block_diag(*[self.R * wq[j] for j in range(N)])
        H = 0.5 * (H + H.T)
        cf = la.cho_factor(H)
        return H, la.cho_solve(cf, W @ Sx), la.cho_solve(cf, W @ Sv), la.cho_solve(cf, W @ Sa)

    def _build_tables(self, L_min: float, L_max: float, step: float) -> None:
        grid = np.arange(L_min, L_max + 0.5 * step, step)
        A0, B0 = self.plant.linearize(float(grid[0]))
        A1, B1 = self.plant.linearize(float(grid[-1]))
        gM = tuple(round(self.att_gM(float(L)), 9) for L in (grid[0], grid[-1])) if self.att_gM is not None else None
        key = (np.round(np.concatenate([A0.ravel(), B0.ravel(), A1.ravel(), B1.ravel()]), 9).tobytes(), tuple(np.diag(self.Q)),
               tuple(np.diag(self.R)), tuple(self.d.round(9)), self.Ts, tuple(grid.round(6)), gM)
        if key not in _TABLES:
            rows = [self._design(float(L)) for L in grid]
            _TABLES[key] = (grid, *(np.array([r[k] for r in rows]) for k in range(4)))
        self.grid, self.H_t, self.Kx_t, self.Kv_t, self.Ka_t = _TABLES[key]

    def first_move_gain(self, L0: float) -> np.ndarray:
        """The unconstrained feedback of the first move (2 x n) at the grid point nearest L0 (checks against an LQR)."""
        i = int(np.argmin(np.abs(self.grid - L0)))
        return self.Kx_t[i, :2]

    # ------------------------------------------------------------------ run time
    def _blend(self, L0: float):
        g = self.grid
        x = (min(max(L0, g[0]), g[-1]) - g[0]) / (g[1] - g[0])
        i = min(int(x), len(g) - 2)
        return i, x - i

    def _filter(self, filt: float, ramp: float, brake_tau: float, dt: float) -> float:
        if self.v_ref_tau > 0.0 and abs(ramp) > abs(filt):          # optional smoothing while speeding up
            return filt + (ramp - filt) * (1.0 - math.exp(-dt / self.v_ref_tau))
        if brake_tau > 0.0 and abs(ramp) < abs(filt):               # gentle braking over uneven ground (MCU)
            return filt + (ramp - filt) * (1.0 - math.exp(-dt / brake_tau))
        return ramp

    def _reference(self, v_cmd: float, rate: float, brake_tau: float):
        """Advance the reference one control period (towards the command of t_lead ago), then preview it: vr[j] at the
        start of block j (j = 0..N; vr[0] = the reference tracked now), the command of each block from the buffer."""
        Ts = self.Ts
        self._cmd.append(v_cmd)
        target = self._cmd[0]
        dv = float(np.clip(target - self.v_ramp, -rate * Ts, rate * Ts))
        self.v_ramp += dv
        self.v_filt = self._filter(self.v_filt, self.v_ramp, brake_tau, Ts)
        if not self.preview:
            return np.full(self.N + 1, self.v_filt), dv
        vr = np.empty(self.N + 1)
        ramp, filt = self.v_ramp, self.v_filt
        vr[0] = filt
        nl = self.n_lead
        for j, (t0, d) in enumerate(zip(self.t_start, self.d)):
            k = int(round((t0 + d) / Ts))                    # the reference at the end of this block follows the command
            tgt = self._cmd[k] if k <= nl else v_cmd         # of (t0 + d - t_lead): _cmd[k] is (nl - k) periods old
            ramp += min(max(tgt - ramp, -rate * d), rate * d)
            filt = self._filter(filt, ramp, brake_tau, d)
            vr[j + 1] = filt
        return vr, dv

    def step(self, theta, theta_dot, v, phi, phi_dot, L0, v_cmd, rate: float = 0.0, brake_tau: float = 0.0,
             hold: bool = False, limits=None):
        """One control period.  rate > 0 replaces the reference acceleration (MCU: braking over uneven ground);
        hold: integrators held (uneven ground); limits: (G, h) with G u <= h for every one of the first n_con moves,
        u = [T, Tp].  Returns (T, Tp)."""
        if not all(math.isfinite(float(x)) for x in (theta, theta_dot, v, phi, phi_dot, L0, v_cmd)):
            self.nonfinite_steps += 1           # hold the last output; the MCU raises FAULT when this persists
            return self._last_out
        self.nonfinite_steps = 0
        vr, dv = self._reference(v_cmd, rate if rate > 0.0 else self.v_ramp_rate, brake_tau)
        ar = np.diff(vr) / self.d
        ev = v - self.v_filt

        # integrators: the rules of the LQI8 of v5_lqi (z2 only quasi-statically, nothing integrates while saturated)
        saved = (self.z, self.z_att, self.z_pos)
        ramping = abs(dv) > 1e-9 or abs(self.v_filt - self.v_ramp) > self.freeze_tol
        self._a_ref_lp += (dv / self.Ts - self._a_ref_lp) * self.Ts / self.att_accel_tau
        accelerating = abs(self._a_ref_lp) > self.att_accel_thresh
        if accelerating:
            self._freeze_countdown = self._freeze_hold_steps
        elif self._freeze_countdown > 0:
            self._freeze_countdown -= 1
            accelerating = True
        att_ramping = accelerating or abs(self.v_filt - self.v_ramp) > self.freeze_tol
        frozen = hold or (self.freeze_z_on_ramp and ramping)
        if not frozen:
            self.z = float(np.clip(self.z + ev * self.Ts, -self.int_limit, self.int_limit))
        e = [theta, theta_dot, ev, phi, phi_dot, self.z]
        if self.i_att is not None:
            if not frozen and not att_ramping:
                e_att = theta + phi + (self.u_prev[1] - self.u_prev[0]) / self.att_gM(L0)
                self.z_att = float(np.clip(self.z_att + e_att * self.Ts, -self.att_limit, self.att_limit))
            e.append(self.z_att)
        if self.i_pos is not None:
            if not frozen and abs(self.z) < self.int_limit:
                self.z_pos = float(np.clip(self.z_pos + self.z * self.Ts, -self.pos_limit, self.pos_limit))
            e.append(self.z_pos)
        e = np.array(e)

        u = self._solve(e, vr[:-1], ar, *self._blend(L0), limits)
        if self.constrained:                    # conditional integration: no wind-up against a torque limit
            self.z, self.z_att, self.z_pos = saved
        self.u_prev = u
        self._last_out = (float(u[0]), float(u[1]))
        return self._last_out

    def _solve(self, e, vr, ar, i, w, limits) -> np.ndarray:
        nc2 = 2 * self.n_con
        Kx, Kv, Ka = self.Kx_t, self.Kv_t, self.Ka_t
        U_head = -((1.0 - w) * (Kx[i, :nc2] @ e + Kv[i, :nc2] @ vr + Ka[i, :nc2] @ ar)
                   + w * (Kx[i + 1, :nc2] @ e + Kv[i + 1, :nc2] @ vr + Ka[i + 1, :nc2] @ ar))
        self.constrained = False
        if limits is None:
            return U_head[:2]
        G, h = limits
        if np.all(G @ U_head.reshape(-1, 2).T <= h[:, None] + 1e-9):
            return U_head[:2]                   # the unconstrained optimum keeps every limit: no QP
        import qpsolvers
        mix = lambda T: (1.0 - w) * T[i] + w * T[i + 1]
        U_unc = -(mix(Kx) @ e + mix(Kv) @ vr + mix(Ka) @ ar)
        H = mix(self.H_t)
        f = -H @ U_unc
        Gb = np.kron(np.eye(self.n_con), G)
        Gq = np.hstack([Gb, np.zeros((Gb.shape[0], 2 * (self.N - self.n_con)))])
        hq = np.tile(h, self.n_con)
        lim = np.tile([2.0 * self.plant.tau_wheel_max, 2.0 * self.plant.tau_hip_max], self.N)
        t0 = time.perf_counter()
        try:
            U = qpsolvers.solve_qp(H, f, Gq, hq, lb=-lim, ub=lim, solver=self.solver)
        except Exception:
            U = None
        self.qp_calls += 1
        self.qp_time_max_ms = max(self.qp_time_max_ms, 1e3 * (time.perf_counter() - t0))
        self.constrained = True
        if U is None:                           # infeasible / solver error: the unconstrained move, projected into the limits
            self.qp_failures += 1
            u = U_head[:2].copy()
            for _ in range(4):
                viol = G @ u - h
                k = int(np.argmax(viol))
                if viol[k] <= 0.0:
                    break
                g = G[k]
                u -= g * viol[k] / max(float(g @ g), 1e-12)
            return u
        return np.asarray(U[:2], dtype=float)


class MpcLaw:
    """MCU adapter (DiabloMcu law interface): estimate -> plant states, the other loops' torques -> motor limits."""

    def __init__(self, plant, Ts: float, theta_eq=None, Tp_ff: float = 0.0, u_limit: float = 0.75, **mpc_kwargs):
        """u_limit: share of each motor's peak the MPC plans within (the MCU adds stiction compensation on top);
        0 = no constraints (the explicit unconstrained MPC)."""
        self.theta_eq = theta_eq or (lambda L0: 0.0)
        if mpc_kwargs.get("q_att") is not None and mpc_kwargs.get("att_gM") is None:
            mpc_kwargs["att_gM"] = static_gM(plant, self.theta_eq)
        self.ctrl = BalanceMPC(plant, Ts, **mpc_kwargs)
        self.Tp_ff = Tp_ff
        self.u_limit = u_limit

    def reset(self) -> None:
        self.ctrl.reset()

    def limits(self, est, refs: dict, mcu):
        """(G, h): every motor torque the balance torques reach, G [T, Tp] <= h, at the joint angles of now.
        Per side (MCU signs: left -1, right +1): wheel T/2 + s tau_yaw/2; hip and knee through the VMC map of
        (F, Tp/2 + s tau_split/2) plus the knee's wheel-reaction term.  F is the force the MCU's leg loops gave that leg
        in the last period (DiabloMcu.leg_force_cmd: static + length PD + roll + range wall + damper); before the first
        period, the static force.  A constraint the other loops alone already break is left out (the balance torques
        cannot fix it).
        The static force alone is not enough (2026-11-11, test 2-19): with a leg on its range wall the leg loop puts
        10.5-12.5 N m on that knee while the static force gives 8.6-9.4, just under the 9.4 limit; the row then left
        the balance torques almost no room, pushed the hip torque the wrong way and the body pitched to 10 deg."""
        p = mcu.p
        servo = mcu.servo_valid if getattr(mcu, "servo_valid", None) else mcu.last_obs["servo"]
        F_cmd = getattr(mcu, "leg_force_cmd", None) or {}
        tau_yaw = refs.get("tau_yaw", 0.0)
        d_theta = est.theta_v_body["left"] - est.theta_v_body["right"]
        d_theta_dot = est.theta_v_dot["left"] - est.theta_v_dot["right"]
        tau_split = p.kp_split * d_theta + p.kd_split * d_theta_dot
        F_ff = p.F_ff_N if p.F_ff_N is not None else 0.5 * (p.mass_total - p.mass_wheels) * 9.81
        Lw, Lh = self.u_limit * p.tau_wheel_max, self.u_limit * p.tau_hip_max
        rows, rhs = [], []

        def add(coef, off, L):
            if abs(off) >= L:
                return
            rows.extend([coef, [-coef[0], -coef[1]]])
            rhs.extend([L - off, L + off])

        for side, s in (("left", -1.0), ("right", +1.0)):
            qh, qk = servo[f"{side}_hip"]["position"], servo[f"{side}_{p.leg_joint}"]["position"]
            F = F_cmd.get(side)                 # what the leg loops really put on this leg (one period old)
            if F is None or not math.isfinite(F):
                F = p.leg_force(est.L0[side], est.theta_v[side]) if p.leg_force is not None else F_ff / max(math.cos(est.theta_v[side]), 0.5)
            a_h, a_k = mcu.leg.vmc_torques(1.0, 0.0, qh, qk)
            b_h, b_k = mcu.leg.vmc_torques(0.0, 1.0, qh, qk)
            c_k = mcu.leg.wheel_reaction_ratio(qh, qk) if p.wheel_reaction_ff else 0.0
            tp_off = 0.5 * self.Tp_ff + 0.5 * s * tau_split          # this side's hip torque beyond u[1] / 2
            add([0.5, 0.0], 0.5 * s * tau_yaw, Lw)
            add([0.0, 0.5 * b_h], a_h * F + b_h * tp_off, Lh)
            add([0.5 * c_k, 0.5 * b_k], a_k * F + b_k * tp_off + c_k * 0.5 * s * tau_yaw, Lh)
        if not rows:
            return None
        return np.array(rows, dtype=float), np.array(rhs, dtype=float)

    def compute(self, est, refs: dict, mcu) -> dict:
        if not refs.get("balance", True):       # CAR / transitions / FAULT: start again from rest
            self.ctrl.reset()
            return {"T": 0.0, "Tp": 0.0}
        phi = -(est.pitch - refs.get("pitch_ref", 0.0))
        theta = est.theta_v_mean - self.theta_eq(est.L0_mean)
        lim = self.limits(est, refs, mcu) if self.u_limit > 0.0 else None
        T, Tp = self.ctrl.step(theta, est.theta_v_dot_mean, est.v, phi, -est.pitch_rate, est.L0_mean, refs["v_ref"],
                               rate=refs.get("v_brake", 0.0), brake_tau=refs.get("v_brake_tau", 0.0),
                               hold=bool(refs.get("hold_integrators", False)), limits=lim)
        return {"T": T, "Tp": Tp + self.Tp_ff, "Tp_ff": self.Tp_ff, "mpc_constrained": float(self.ctrl.constrained)}
