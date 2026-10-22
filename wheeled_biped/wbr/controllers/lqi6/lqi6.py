"""LQI6 -- the controller of the design report (section 8, "MineLQI6"): 6-state LQR with a velocity
integral designed from the three-body plant linearisation (Bryson weights, R scaled with the torque
limits).  Gains are designed on a leg-length grid at start-up and interpolated at run time (table_L0);
with table_L0=None the controller re-designs online whenever the leg length changes by > 1 cm.  State xs = [theta, theta_dot, v - v_ref, phi, phi_dot, int(v - v_ref)],
outputs u = [T, Tp] = -K xs (total wheel torque, total hip torque).
"""
from __future__ import annotations

import math

import numpy as np
import scipy.linalg as la

from wbr.controllers.lqi6.plant6 import Plant6


def c2d_zoh(A: np.ndarray, B: np.ndarray, Ts: float):
    n, m = A.shape[0], B.shape[1]
    M = np.zeros((n + m, n + m))
    M[:n, :n] = A
    M[:n, n:] = B
    E = la.expm(M * Ts)
    return E[:n, :n], E[:n, n:]


class LQI6:
    def __init__(self, plant: Plant6, Ts: float = 1e-3, q_theta=0.3, q_thetad=3.0, q_v=1.0, q_phi=0.15, q_phid=3.0,
                 q_int=1.0, r_wheel_frac=0.25, r_hip_frac=0.15, v_ramp_rate=1.0, int_limit=1.0,
                 reschedule_dL=0.01, table_L0=(0.10, 0.28, 0.01), a_ff=0.0, v_ref_tau=0.0, v_ref_order=1,
                 freeze_int_on_ramp=False, v_ref_tau_brake=0.0, q_att=None, q_pos=None, att_gM=None,
                 att_limit=0.2, pos_limit=0.5, pred_steps=0, freeze_tol=1e-3, quiet_boost_att=1.0,
                 quiet_boost_pos=1.0, plain_ramp_rate=4.0, att_accel_tau=0.1, att_accel_thresh=0.1,
                 freeze_int_on_accel_plain=True):
        """table_L0 = (min, max, step): gains are designed once on this leg-length grid and interpolated
        at run time (no Riccati solve inside the 1 kHz loop, as a Teensy-class MCU would do it);
        None keeps the online re-design of the report.

        Optional extra integral states (Plan 4; None = off, the 6-state LQI of the report):
          q_att  z2 = int(e_att),  e_att = theta + phi + (Tp - T) / (g M(L0)): the static body tilt seen by the
                 encoders and the actuator torques.  At rest the total COM satisfies sum(m x) = T / g and the hip
                 holds the body moment, which gives  phi_true = d(theta_body) + (dTp - T) / (g M) with
                 M = m_l c + m_b L0 (att_gM(L0) = g M) -- independent of an IMU pitch bias (it cancels in
                 theta + phi), of a body COM offset (payload) and of a slope.  Driving it to zero levels the body.
          q_pos  z3 = int(z1) = int(position error): constant force / torque disturbances (slope, payload, IMU
                 bias) are absorbed without a steady position offset (z1 alone ends displaced by T_d / K_z).
        pred_steps > 0: state predictor over a FIXED horizon of that many control periods (the nominal sensor ->
                 torque latency of the target system, a design constant -- not the actual latency of a run):
                 x_hat(k) = Ad^N x_meas(k) + sum_{j=1..N} Ad^(j-1) Bd u(k-j) on the 5 balance states with the last N
                 outputs (finite-spectrum assignment).  The feedback acts on x_hat; the integrators keep integrating the
                 measured (delayed) signals, so a constant model error cannot bias their steady state.
        Plan 4 uses q_att and q_pos; pred_steps is kept (default off) for the trade-off studies of
        docs/plan4_lqi6.md (position-offset-free disturbance rejection, latency compensation)."""
        self.plant = plant
        self.Ts = Ts
        self.pred_steps = int(pred_steps)
        self.q_att, self.q_pos, self.att_gM = q_att, q_pos, att_gM
        self.att_limit, self.pos_limit = att_limit, pos_limit
        if q_att is not None and att_gM is None:
            raise ValueError("q_att needs att_gM (L0 -> g M(L0))")
        q_diag = [1 / q_theta**2, 1 / q_thetad**2, 1 / q_v**2, 1 / q_phi**2, 1 / q_phid**2, 1 / q_int**2]
        if q_att is not None:
            q_diag.append(1 / q_att**2)
        if q_pos is not None:
            q_diag.append(1 / q_pos**2)
        self.n = len(q_diag)
        self.i_att = 6 if q_att is not None else None
        self.i_pos = (7 if q_att is not None else 6) if q_pos is not None else None
        self.Q = np.diag(q_diag)
        self.R = np.diag([1 / (r_wheel_frac * plant.tau_wheel_max)**2, 1 / (r_hip_frac * plant.tau_hip_max)**2])
        self.v_ramp_rate = v_ramp_rate
        self.a_ff = a_ff                 # wheel torque feed-forward per m/s^2 of reference acceleration (N m s^2/m)
        self.v_ref_tau = v_ref_tau       # pre-filter time constant on the ramped reference (s); 0 = off
        self.v_ref_order = v_ref_order   # 1: first order, 2: two cascaded first-order stages (smoother onset)
        self.freeze_int_on_ramp = freeze_int_on_ramp   # do not integrate while the reference is still ramping
        self.freeze_tol = freeze_tol     # m/s: the reference counts as 'ramping' until the pre-filter is this close to the ramp
        self.v_ref_tau_brake = v_ref_tau_brake   # pre-filter time constant while slowing down (0 = follow the ramp)
        # reference shaping can be switched off at run time when the host flags that it shaped the reference itself
        # (MotionCtrl.host_shaped, stage-2 MPC): then a fast ramp without pre-filter / integrator freeze is used
        self._shaped = (v_ramp_rate, v_ref_tau, freeze_int_on_ramp)
        self._plain = (plain_ramp_rate, 0.0, False)
        self.shaping_on = True
        # integration of the slow static-tilt / position integrators is `quiet_boost` times faster while the robot stands
        # still near its equilibrium (|v| < 0.1 m/s, |theta|, |phi| < 1.5 deg, reference constant): a payload or an IMU
        # bias is then taken up in a few seconds instead of tens of seconds, without touching the dynamic response
        self.quiet_boost_att = quiet_boost_att
        self.quiet_boost_pos = quiet_boost_pos
        # the static-tilt integrator is frozen while the (low-passed) reference acceleration exceeds att_accel_thresh:
        # a commanded acceleration a leans the robot by ~a/g and loads the hips, which e_att would read as a static
        # tilt (stage-2 item 19).  A host-shaped reference is piecewise constant (new value every host period), so
        # the acceleration is low-passed (att_accel_tau) instead of looking at single-step changes; cruising at
        # constant speed and disturbance recoveries keep integrating (slope item 13, push item 5)
        self.att_accel_tau = att_accel_tau
        self.att_accel_thresh = att_accel_thresh
        self._a_ref_lp = 0.0
        # plain tracking of a host-shaped reference (stage-2 MPC): freeze ALL integrators while the low-passed
        # reference acceleration is above att_accel_thresh, exactly like the stage-1 ramp freeze.  Otherwise z winds
        # up during every manoeuvre and pays back afterwards: a 0.2 m roll-back after each stop and a 10-30 %
        # overshoot when the host hands over mid-manoeuvre (review r1 stage 2 M2 / M3).  The MPC prediction model
        # is the integrator-free loop accordingly (mpc_host.model.lqi6_options).
        self.freeze_int_on_accel_plain = freeze_int_on_accel_plain
        self._last_out = (0.0, 0.0)
        self.int_limit = int_limit
        self.reschedule_dL = reschedule_dL
        self.K = None
        self.L0_last = None
        self.table = None
        self.pred = None                 # (P = Ad^N, G = [Ad^0 Bd, ..., Ad^(N-1) Bd]) at the last designed L0
        self.aux_table = None            # (P, G, gM) on the gain-table grid (only with the Plan 4 extensions)
        if table_L0 is not None:
            self.build_table(*table_L0)
        self.reset()

    def set_reference_shaping(self, on: bool) -> None:
        """Stage-1 shaping (ramp + pre-filter + integrator freeze) on, or plain tracking (host-shaped reference)."""
        if on == self.shaping_on:
            return
        self.shaping_on = on
        self.v_ramp_rate, self.v_ref_tau, self.freeze_int_on_ramp = self._shaped if on else self._plain
        self.v_filt = self.v_filt1 = self.v_ramp      # continuity of the reference seen by the loop

    def reset(self) -> None:
        self.shaping_on = True                 # review r1 n7: a reset restores the MCU's own shaping
        self.v_ramp_rate, self.v_ref_tau, self.freeze_int_on_ramp = self._shaped
        self._last_out = (0.0, 0.0)
        self.z = 0.0
        self.z_att = 0.0
        self.z_pos = 0.0
        self.u_prev = np.zeros(2)
        self.U = np.zeros(2 * max(self.pred_steps, 1))     # u(k-1), u(k-2), ... for the predictor
        self.v_ramp = 0.0
        self.v_filt = 0.0
        self.v_filt1 = 0.0
        self._a_ref_lp = 0.0

    def build_table(self, L_min: float, L_max: float, step: float) -> None:
        grid = np.arange(L_min, L_max + 0.5 * step, step)
        Ks, aux = [], []
        for L0 in grid:
            try:
                Ks.append(self.design(float(L0)).copy())
                aux.append(self.pred)
            except (np.linalg.LinAlgError, ValueError):
                if not Ks:
                    raise
                Ks.append(Ks[-1])
                aux.append(aux[-1])
        self.table = (grid, np.array(Ks))        # (n_grid, 2, n)
        if self.pred_steps > 0 or self.att_gM is not None:
            Ps = np.array([a[0] for a in aux]) if self.pred_steps > 0 else None
            Gs = np.array([a[1] for a in aux]) if self.pred_steps > 0 else None
            gMs = np.array([self.att_gM(float(L)) for L in grid]) if self.att_gM is not None else None
            self.aux_table = (Ps, Gs, gMs)

    def design(self, L0: float) -> np.ndarray:
        A, B = self.plant.linearize(L0)
        keep = [0, 1, 3, 4, 5]
        A5 = A[np.ix_(keep, keep)]
        B5 = B[keep, :]
        n = self.n
        Aa = np.zeros((n, n))
        Aa[:5, :5] = A5
        Aa[5, 2] = 1.0
        Ba = np.zeros((n, 2))
        Ba[:5, :] = B5
        if self.i_att is not None:              # z2' = theta + phi + (Tp - T) / (g M)
            gM = self.att_gM(L0)
            Aa[self.i_att, 0] = Aa[self.i_att, 3] = 1.0
            Ba[self.i_att, :] = [-1.0 / gM, 1.0 / gM]
        if self.i_pos is not None:              # z3' = z1
            Aa[self.i_pos, 5] = 1.0
        Ad, Bd = c2d_zoh(Aa, Ba, self.Ts)
        P = la.solve_discrete_are(Ad, Bd, self.Q, self.R)
        self.K = np.linalg.solve(self.R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
        self.L0_last = L0
        if self.pred_steps > 0:
            Ad5, Bd5 = c2d_zoh(A5, B5, self.Ts)
            N = self.pred_steps
            G = np.zeros((5, 2 * N))
            Apow = np.eye(5)
            for j in range(N):                   # column block j <-> u(k-1-j): Ad^j Bd
                G[:, 2 * j:2 * j + 2] = Apow @ Bd5
                Apow = Apow @ Ad5
            self.pred = (Apow, G)                # Apow = Ad^N
        return self.K

    def _aux(self, L0: float):
        """(P, G, gM) at L0: linear blend of the two neighbouring grid points (no per-element np.interp)."""
        if self.aux_table is None or self.table is None:
            gM = self.att_gM(L0) if self.att_gM is not None else None
            return (self.pred[0], self.pred[1], gM) if self.pred is not None else (None, None, gM)
        grid = self.table[0]
        x = (min(max(L0, grid[0]), grid[-1]) - grid[0]) / (grid[1] - grid[0])
        i = min(int(x), len(grid) - 2)
        w = x - i
        Ps, Gs, gMs = self.aux_table
        P = None if Ps is None else (1 - w) * Ps[i] + w * Ps[i + 1]
        G = None if Gs is None else (1 - w) * Gs[i] + w * Gs[i + 1]
        gM = None if gMs is None else (1 - w) * gMs[i] + w * gMs[i + 1]
        return P, G, gM

    def gains(self, L0: float) -> np.ndarray:
        if self.table is not None:
            grid, Ks = self.table
            L0 = float(np.clip(L0, grid[0], grid[-1]))
            self.K = np.array([[np.interp(L0, grid, Ks[:, i, j]) for j in range(self.n)] for i in range(2)])
            self.L0_last = L0
            return self.K
        if self.K is None or abs(L0 - self.L0_last) > self.reschedule_dL:
            try:
                self.design(L0)
            except (np.linalg.LinAlgError, ValueError):
                if self.K is None:
                    raise
        return self.K

    def step(self, theta, theta_dot, v, phi, phi_dot, L0, v_ref):
        """Returns (T, Tp): total wheel torque and total hip torque."""
        if not all(math.isfinite(float(x)) for x in (theta, theta_dot, v, phi, phi_dot, L0, v_ref)):
            return self._last_out          # never integrate a non-finite sample (review r1 M2): hold the last output
        K = self.gains(L0)
        dv = float(np.clip(v_ref - self.v_ramp, -self.v_ramp_rate * self.Ts, self.v_ramp_rate * self.Ts))
        self.v_ramp += dv
        if self.v_ref_tau > 0.0 and abs(self.v_ramp) > abs(self.v_filt):
            # pre-filter only while speeding up (limits the velocity overshoot); braking follows the ramp
            # directly so that the stopping distance is not stretched
            if self.v_ref_order >= 2:
                self.v_filt1 += (self.v_ramp - self.v_filt1) * self.Ts / self.v_ref_tau
                self.v_filt += (self.v_filt1 - self.v_filt) * self.Ts / self.v_ref_tau
            else:
                self.v_filt += (self.v_ramp - self.v_filt) * self.Ts / self.v_ref_tau
        elif self.v_ref_tau_brake > 0.0 and abs(self.v_ramp) < abs(self.v_filt):
            self.v_filt += (self.v_ramp - self.v_filt) * self.Ts / self.v_ref_tau_brake
            self.v_filt1 = self.v_filt
        else:
            self.v_filt = self.v_filt1 = self.v_ramp
        ev = v - self.v_filt
        z_prev, za_prev, zp_prev = self.z, self.z_att, self.z_pos
        ramping = abs(dv) > 1e-9 or abs(self.v_filt - self.v_ramp) > self.freeze_tol
        self._a_ref_lp += (dv / self.Ts - self._a_ref_lp) * self.Ts / self.att_accel_tau
        att_ramping = abs(self._a_ref_lp) > self.att_accel_thresh or abs(self.v_filt - self.v_ramp) > self.freeze_tol
        frozen = (self.freeze_int_on_ramp and ramping) or \
                 (self.freeze_int_on_accel_plain and not self.shaping_on and abs(self._a_ref_lp) > self.att_accel_thresh)
        if not frozen:
            self.z = float(np.clip(self.z + ev * self.Ts, -self.int_limit, self.int_limit))
        Pm = Gm = gM = None
        if self.pred_steps > 0 or self.i_att is not None:
            Pm, Gm, gM = self._aux(L0)
        if self.pred_steps > 0:      # feedback on the predicted current state, integrators on the measurement
            xh = Pm @ np.array([theta, theta_dot, v, phi, phi_dot]) + Gm @ self.U
            xs = [xh[0], xh[1], xh[2] - self.v_filt, xh[3], xh[4], self.z]
        else:
            xs = [theta, theta_dot, ev, phi, phi_dot, self.z]
        quiet = (not att_ramping) and abs(v) < 0.1 and abs(theta) < 0.026 and abs(phi) < 0.026
        if self.i_att is not None:
            # static body tilt (bias-free: the IMU cancels in theta + phi), previous outputs (D-term).  Only valid
            # quasi-statically: never while the reference moves (a commanded acceleration shows up as hip torque and
            # would be read as a tilt -- stage-2 item 19); disturbance recoveries keep integrating (push item 5 needs
            # the tilt correction right after the slip), faster while standing still
            if not frozen and not att_ramping:
                e_att = theta + phi + (self.u_prev[1] - self.u_prev[0]) / gM
                gain = self.quiet_boost_att if quiet else 1.0
                self.z_att = float(np.clip(self.z_att + gain * e_att * self.Ts, -self.att_limit, self.att_limit))
            xs.append(self.z_att)
        if self.i_pos is not None:
            if not frozen and abs(self.z) < self.int_limit:   # conditional integration: not while z1 is clamped
                gain = self.quiet_boost_pos if quiet else 1.0
                self.z_pos = float(np.clip(self.z_pos + gain * self.z * self.Ts, -self.pos_limit, self.pos_limit))
            xs.append(self.z_pos)
        xs = np.array(xs)
        u = -K @ xs
        # conditional integration: do not wind up while an actuator is saturated
        if abs(u[0]) > 2 * self.plant.tau_wheel_max or abs(u[1]) > 2 * self.plant.tau_hip_max:
            self.z, self.z_att, self.z_pos = z_prev, za_prev, zp_prev
            xs[5] = self.z
            if self.i_att is not None:
                xs[self.i_att] = self.z_att
            if self.i_pos is not None:
                xs[self.i_pos] = self.z_pos
            u = -K @ xs
        self.u_prev = u.copy()
        T_out = float(u[0] + self.a_ff * dv / self.Ts)
        self._last_out = (T_out, float(u[1]))
        if self.pred_steps > 0:      # input history of the predictor (as applied: clamped to the actuator range)
            self.U[2:] = self.U[:-2].copy()
            self.U[0] = min(max(T_out, -2 * self.plant.tau_wheel_max), 2 * self.plant.tau_wheel_max)
            self.U[1] = min(max(float(u[1]), -2 * self.plant.tau_hip_max), 2 * self.plant.tau_hip_max)
        return T_out, float(u[1])


if __name__ == "__main__":
    from wbr.controllers.lqi6.plant6 import diablo_lite_params
    plant = Plant6(**diablo_lite_params())
    A, B = plant.linearize()
    print("open-loop eigenvalues:", np.round(np.linalg.eigvals(A), 2))
    ctrl = LQI6(plant)
    for L0 in (0.14, 0.198, 0.26):
        K = ctrl.design(L0)
        print(f"L0={L0:.3f}  K_T  = {np.round(K[0], 2)}\n           K_Tp = {np.round(K[1], 2)}")
