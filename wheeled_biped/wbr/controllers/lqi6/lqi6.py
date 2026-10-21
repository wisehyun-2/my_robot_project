"""LQI6 -- the controller of the design report (section 8, "MineLQI6"): 6-state LQR with a velocity
integral designed online from the three-body plant linearisation (Bryson weights, R scaled with the
torque limits), re-designed whenever the leg length changes by more than 1 cm (gain scheduling
without polynomial fits).  State xs = [theta, theta_dot, v - v_ref, phi, phi_dot, int(v - v_ref)],
outputs u = [T, Tp] = -K xs (total wheel torque, total hip torque).
"""
from __future__ import annotations

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
                 reschedule_dL=0.01, table_L0=(0.10, 0.28, 0.01)):
        """table_L0 = (min, max, step): gains are designed once on this leg-length grid and interpolated
        at run time (no Riccati solve inside the 1 kHz loop, as a Teensy-class MCU would do it);
        None keeps the online re-design of the report."""
        self.plant = plant
        self.Ts = Ts
        self.Q = np.diag([1 / q_theta**2, 1 / q_thetad**2, 1 / q_v**2, 1 / q_phi**2, 1 / q_phid**2, 1 / q_int**2])
        self.R = np.diag([1 / (r_wheel_frac * plant.tau_wheel_max)**2, 1 / (r_hip_frac * plant.tau_hip_max)**2])
        self.v_ramp_rate = v_ramp_rate
        self.int_limit = int_limit
        self.reschedule_dL = reschedule_dL
        self.K = None
        self.L0_last = None
        self.table = None
        if table_L0 is not None:
            self.build_table(*table_L0)
        self.reset()

    def reset(self) -> None:
        self.z = 0.0
        self.v_ramp = 0.0

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
        self.table = (grid, np.array(Ks))        # (n, 2, 6)

    def design(self, L0: float) -> np.ndarray:
        A, B = self.plant.linearize(L0)
        keep = [0, 1, 3, 4, 5]
        A5 = A[np.ix_(keep, keep)]
        B5 = B[keep, :]
        Aa = np.zeros((6, 6))
        Aa[:5, :5] = A5
        Aa[5, 2] = 1.0
        Ba = np.zeros((6, 2))
        Ba[:5, :] = B5
        Ad, Bd = c2d_zoh(Aa, Ba, self.Ts)
        P = la.solve_discrete_are(Ad, Bd, self.Q, self.R)
        self.K = np.linalg.solve(self.R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
        self.L0_last = L0
        return self.K

    def gains(self, L0: float) -> np.ndarray:
        if self.table is not None:
            grid, Ks = self.table
            L0 = float(np.clip(L0, grid[0], grid[-1]))
            self.K = np.array([[np.interp(L0, grid, Ks[:, i, j]) for j in range(6)] for i in range(2)])
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
        K = self.gains(L0)
        self.v_ramp += float(np.clip(v_ref - self.v_ramp, -self.v_ramp_rate * self.Ts, self.v_ramp_rate * self.Ts))
        ev = v - self.v_ramp
        z_prev = self.z
        self.z = float(np.clip(self.z + ev * self.Ts, -self.int_limit, self.int_limit))
        xs = np.array([theta, theta_dot, ev, phi, phi_dot, self.z])
        u = -K @ xs
        # conditional integration: do not wind up while an actuator is saturated
        if abs(u[0]) > 2 * self.plant.tau_wheel_max or abs(u[1]) > 2 * self.plant.tau_hip_max:
            self.z = z_prev
            xs[5] = self.z
            u = -K @ xs
        return float(u[0]), float(u[1])


if __name__ == "__main__":
    from wbr.controllers.lqi6.plant6 import diablo_lite_params
    plant = Plant6(**diablo_lite_params())
    A, B = plant.linearize()
    print("open-loop eigenvalues:", np.round(np.linalg.eigvals(A), 2))
    ctrl = LQI6(plant)
    for L0 in (0.14, 0.198, 0.26):
        K = ctrl.design(L0)
        print(f"L0={L0:.3f}  K_T  = {np.round(K[0], 2)}\n           K_Tp = {np.round(K[1], 2)}")
