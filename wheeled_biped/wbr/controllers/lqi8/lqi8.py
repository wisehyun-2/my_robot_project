"""LQI8 -- discrete LQR with up to 8 states, designed from the three-body plant linearisation (Bryson weights, R scaled
with the torque limits).  Gains are designed on a leg-length grid at start-up and interpolated at run time (table_L0);
with table_L0=None the controller re-designs online whenever the leg length changes by > 1 cm.
State xs = [theta, theta_dot, v - v_ref, phi, phi_dot, z1 (, z2, z3)], z1 = int(v - v_ref) (the position error),
z2 = int(static body tilt) (q_att), z3 = int(z1) (q_pos); outputs u = [T, Tp] = -K xs (total wheel torque, total hip torque).
With q_att = q_pos = None it is the 6-state controller of the design report (section 8, "MineLQI6"), as Plan 2 runs it;
the v5 robot runs all 8 states (as Plan 4 did on the upkie model, removed 2026-11-14).  Named LQI6 until 2026-11-10.
"""
from __future__ import annotations

import math

import numpy as np
import scipy.linalg as la

from typing import TYPE_CHECKING

if TYPE_CHECKING:                     # the plant: linearize(L0) -> A, B (6 x 6, 6 x 2); L0, tau_wheel_max, tau_hip_max
    from wbr.controllers.lqi8.plant_serial import SerialPlant


def c2d_zoh(A: np.ndarray, B: np.ndarray, Ts: float):
    n, m = A.shape[0], B.shape[1]
    M = np.zeros((n + m, n + m))
    M[:n, :n] = A
    M[:n, n:] = B
    E = la.expm(M * Ts)
    return E[:n, :n], E[:n, n:]


class LQI8:
    def __init__(self, plant: "SerialPlant", Ts: float = 1e-3, q_theta=0.3, q_thetad=3.0, q_v=1.0, q_phi=0.15, q_phid=3.0,
                 q_int=1.0, r_wheel_frac=0.25, r_hip_frac=0.15, v_ramp_rate=1.0, int_limit=1.0,
                 reschedule_dL=0.01, table_L0=(0.10, 0.28, 0.01), v_ref_tau=0.0,
                 freeze_int_on_ramp=False, v_ref_tau_brake=0.0, q_att=None, q_pos=None, att_gM=None,
                 att_limit=0.2, pos_limit=0.5, freeze_tol=1e-3, att_accel_tau=0.1, att_accel_thresh=0.1,
                 freeze_hold_s=0.1):
        """table_L0 = (min, max, step): gains are designed once on this leg-length grid and interpolated
        at run time (no Riccati solve inside the 1 kHz loop, as a Teensy-class MCU would do it);
        None keeps the online re-design of the report (the unit tests use it).

        Optional extra integral states (None = off, the 6-state LQI of the report; v5_lqi/TEST stages 1-3):
          q_att  z2 = int(e_att),  e_att = theta + phi + (Tp - T) / (g M(L0)): the static body tilt seen by the
                 encoders and the actuator torques.  At rest the total COM satisfies sum(m x) = T / g and the hip
                 holds the body moment, which gives  phi_true = d(theta_body) + (dTp - T) / (g M) with
                 M = m_l c + m_b L0 (att_gM(L0) = g M) -- independent of an IMU pitch bias (it cancels in
                 theta + phi), of a body COM offset (payload) and of a slope.  Driving it to zero levels the body.
          q_pos  z3 = int(z1) = int(position error): constant force / torque disturbances (slope, payload, IMU
                 bias) are absorbed without a steady position offset (z1 alone ends displaced by T_d / K_z).
        The v5 robot runs both (docs/plan4_lqi8.md).  Removed on 2026-11-14 as unused by v5 (git history): the state
        predictor (pred_steps), the plain tracking of a host-shaped reference (Plan 4's MPC host), the reference
        acceleration feed-forward (a_ff), the second-order pre-filter (v_ref_order) and the faster integration while
        standing still (quiet_boost_*)."""
        self.plant = plant
        self.Ts = Ts
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
        self.v_ramp_rate_base = v_ramp_rate    # the MCU's own ramp; v_ramp_rate may be lowered at run time (law.py: uneven ground)
        self.v_ramp_rate = v_ramp_rate
        self.v_ref_tau = v_ref_tau       # pre-filter time constant on the ramped reference (s); 0 = off
        self.freeze_int_on_ramp = freeze_int_on_ramp   # do not integrate while the reference is still ramping
        self.freeze_tol = freeze_tol     # m/s: the reference counts as 'ramping' until the pre-filter is this close to the ramp
        self.v_ref_tau_brake = v_ref_tau_brake   # pre-filter time constant while slowing down (0 = follow the ramp)
        # the static-tilt integrator is frozen while the (low-passed) reference acceleration exceeds att_accel_thresh:
        # a commanded acceleration a leans the robot by ~a/g and loads the hips, which e_att would read as a static
        # tilt.  The acceleration is low-passed (att_accel_tau) instead of looking at single-step changes; cruising at
        # constant speed and disturbance recoveries keep integrating
        self.att_accel_tau = att_accel_tau
        self.att_accel_thresh = att_accel_thresh
        self._a_ref_lp = 0.0
        # r2 m4: the low-passed reference acceleration passes through zero when the manoeuvre changes sign (speed up ->
        # brake); without a hold-off the integrators ran for 20-40 ms on a 0.2 m/s error there
        self._freeze_hold_steps = int(round(freeze_hold_s / Ts))
        self._freeze_countdown = 0
        self.hold_integrators = False    # set by the MCU over uneven ground: the robot must not chase the moving position reference there
        self._last_out = (0.0, 0.0)
        self.int_limit = int_limit
        self.reschedule_dL = reschedule_dL
        self.K = None
        self.L0_last = None
        self.table = None
        self.gM_table = None             # att_gM on the gain-table grid (with q_att)
        if table_L0 is not None:
            self.build_table(*table_L0)
        self.reset()

    def reset(self) -> None:
        self.hold_integrators = False
        self.v_ramp_rate = self.v_ramp_rate_base
        self._last_out = (0.0, 0.0)
        self.nonfinite_steps = 0
        self._freeze_countdown = 0
        self.z = 0.0
        self.z_att = 0.0
        self.z_pos = 0.0
        self.u_prev = np.zeros(2)
        self.v_ramp = 0.0
        self.v_filt = 0.0
        self._a_ref_lp = 0.0

    def build_table(self, L_min: float, L_max: float, step: float) -> None:
        grid = np.arange(L_min, L_max + 0.5 * step, step)
        Ks = []
        for L0 in grid:
            try:
                Ks.append(self.design(float(L0)).copy())
            except (np.linalg.LinAlgError, ValueError):
                if not Ks:
                    raise
                Ks.append(Ks[-1])
        self.table = (grid, np.array(Ks))        # (n_grid, 2, n)
        if self.att_gM is not None:
            self.gM_table = np.array([self.att_gM(float(L)) for L in grid])

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
        return self.K

    def _gM(self, L0: float) -> float:
        """att_gM at L0: linear blend of the two neighbouring grid points of the gain table (no per-element np.interp)."""
        if self.gM_table is None or self.table is None:
            return self.att_gM(L0)
        grid = self.table[0]
        x = (min(max(L0, grid[0]), grid[-1]) - grid[0]) / (grid[1] - grid[0])
        i = min(int(x), len(grid) - 2)
        w = x - i
        return (1 - w) * self.gM_table[i] + w * self.gM_table[i + 1]

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
            self.nonfinite_steps += 1      # never integrate a non-finite sample (review r1 M2): hold the last output;
            return self._last_out          # the MCU raises FAULT when this persists (review r2 M2)
        self.nonfinite_steps = 0
        K = self.gains(L0)
        dv = float(np.clip(v_ref - self.v_ramp, -self.v_ramp_rate * self.Ts, self.v_ramp_rate * self.Ts))
        self.v_ramp += dv
        if self.v_ref_tau > 0.0 and abs(self.v_ramp) > abs(self.v_filt):
            # pre-filter only while speeding up (limits the velocity overshoot); braking follows the ramp
            # directly so that the stopping distance is not stretched
            self.v_filt += (self.v_ramp - self.v_filt) * self.Ts / self.v_ref_tau
        elif self.v_ref_tau_brake > 0.0 and abs(self.v_ramp) < abs(self.v_filt):
            self.v_filt += (self.v_ramp - self.v_filt) * self.Ts / self.v_ref_tau_brake
        else:
            self.v_filt = self.v_ramp
        ev = v - self.v_filt
        z_prev, za_prev, zp_prev = self.z, self.z_att, self.z_pos
        ramping = abs(dv) > 1e-9 or abs(self.v_filt - self.v_ramp) > self.freeze_tol
        self._a_ref_lp += (dv / self.Ts - self._a_ref_lp) * self.Ts / self.att_accel_tau
        accelerating = abs(self._a_ref_lp) > self.att_accel_thresh
        if accelerating:
            self._freeze_countdown = self._freeze_hold_steps
        elif self._freeze_countdown > 0:
            self._freeze_countdown -= 1
            accelerating = True                      # hold-off across the sign change of the reference acceleration
        att_ramping = accelerating or abs(self.v_filt - self.v_ramp) > self.freeze_tol
        frozen = (self.freeze_int_on_ramp and ramping) or self.hold_integrators
        if not frozen:
            self.z = float(np.clip(self.z + ev * self.Ts, -self.int_limit, self.int_limit))
        xs = [theta, theta_dot, ev, phi, phi_dot, self.z]
        if self.i_att is not None:
            # static body tilt (bias-free: the IMU cancels in theta + phi), previous outputs (D-term).  Only valid
            # quasi-statically: never while the reference moves (a commanded acceleration shows up as hip torque and
            # would be read as a tilt); disturbance recoveries keep integrating (a push needs the tilt correction right
            # after the slip)
            if not frozen and not att_ramping:
                e_att = theta + phi + (self.u_prev[1] - self.u_prev[0]) / self._gM(L0)
                self.z_att = float(np.clip(self.z_att + e_att * self.Ts, -self.att_limit, self.att_limit))
            xs.append(self.z_att)
        if self.i_pos is not None:
            if not frozen and abs(self.z) < self.int_limit:   # conditional integration: not while z1 is clamped
                self.z_pos = float(np.clip(self.z_pos + self.z * self.Ts, -self.pos_limit, self.pos_limit))
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
        T_out = float(u[0])
        self._last_out = (T_out, float(u[1]))
        return T_out, float(u[1])


if __name__ == "__main__":            # the v5 robot's plant (models/wheelbiped_v5), the 6-state form
    from wbr import MODELS_DIR
    from wbr.controllers.lqi8.plant_serial import SerialPlant, SerialPlantParams
    plant = SerialPlant(SerialPlantParams.from_json(MODELS_DIR / "wheelbiped_v5" / "plant_params.json"))
    grid = plant.p.leg.L0
    A, B = plant.linearize(plant.L0)
    print("open-loop eigenvalues:", np.round(np.linalg.eigvals(A), 2))
    ctrl = LQI8(plant, table_L0=(float(grid[0]), float(grid[-1]), float(grid[1] - grid[0])))
    for L0 in (float(grid[0]), plant.L0, float(grid[-1])):
        K = ctrl.design(L0)
        print(f"L0={L0:.3f}  K_T  = {np.round(K[0], 2)}\n           K_Tp = {np.round(K[1], 2)}")
