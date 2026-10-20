"""
kollarcik_lqi.py -- port of Adam Kollarcik, "Modeling and Control of Two-Legged Wheeled Robot", CTU Prague master's
thesis 2021 (the Ascento-type 4-motor "Stoupento" prototype, predecessor of CTU's SK8O).  Code:
https://gitlab.fel.cvut.cz/kollaada/master-thesis-by-adam-kollarcik  (Matlab/3Dpendulum/model.m, PlanarModel/stoupentoController.m)

Their balance law (thesis sec. 3.1, model.m 'KpendII'):
  * 3D wheeled inverted pendulum (pendulum = everything but the wheels), states [xdot, phidot, psidot, phi] + two
    integrators (speed error, yaw-rate error); hips are NOT in the loop (PD position control holds the leg pose,
    like Ascento); one fixed K for all leg lengths.
  * discrete LQR, Ts = 1 ms (dlqr on the ZOH model), weights in motor-angle coordinates
      Q = diag(5*(2/r)^2 [xdot], 1 [phidot], 15*(d/r)^2 [psidot], 1 [phi], 1e-4*(d/r)^2 [int psidot], 1e-4*(2/r)^2 [int xdot]),
      R = diag(1000, 1000) per wheel  ->  planar (symmetric) part: Q = diag(5*(2/r)^2, 1, 1, 1e-4*(2/r)^2), R_total = 500
  * per-wheel torque saturated at +-2 Nm (their eX8108 1:16.5 motors; wheels slip above ~0.5 Nm on the real robot)
  * their pendulum parameters: mb 4.0 kg, mw 0.1, J 7.35e-4, r 0.07 m, l 0.2907 m, Ipy 0.01186, viscous b 0.01

Two ways to run it (meta.json): 'kollarcik_lqi' = their Q/R + design procedure on THIS robot's parameters;
'kollarcik_lqi_native' = their robot's parameters -> their exact planar K, output scale searched.
"""
import numpy as np
import scipy.linalg as la


class PyController:
    out_type = 0                      # wheel torque
    has_scale = True
    name = "Kollarcik (CTU 2021): 4-state dLQR + speed integrator, THEIR Q/R designed on this robot, fixed K, 1 kHz"

    def __init__(self, Ts, native=0, ref_in_state=0):
        self.Ts = Ts; self.native = int(native); self.ref_in_state = int(ref_in_state); self.scale = 1.0
        self.K = None; self.z = 0.0; self.umax = 2.0
        if self.native:
            self.name = "Kollarcik (CTU 2021): their exact planar K (mb 4 kg, l 0.29 m, r 0.07 m), fixed, 1 kHz"
        if self.ref_in_state:
            self.name += " + speed reference also subtracted from the xdot state (error form)"

    def set_scale(self, s): self.scale = float(s)
    def set_param(self, name, v):
        if name == "native": self.native = int(v); self.__init__(self.Ts, self.native, self.ref_in_state)
        if name == "ref_in_state": self.ref_in_state = int(v); self.__init__(self.Ts, self.native, self.ref_in_state)

    def design(self, mb, mw, J, r, l, Ipy, calpha):
        g = 9.81
        M = np.array([[mb + 2 * mw + 2 * J / r**2, mb * l], [mb * l, Ipy + mb * l**2]])
        D = np.array([[2 * calpha / r**2, -2 * calpha / r], [-2 * calpha / r, 2 * calpha]])
        Gq = np.array([[0.0, 0.0], [0.0, -mb * l * g]])          # d(-G)/d[x, phi]: pendulum gravity term
        B2 = np.array([1 / r, -1.0])                               # total wheel torque (model.m B per wheel, summed)
        Mi = np.linalg.inv(M)
        A = np.zeros((3, 3)); B = np.zeros((3, 1))                # states [xdot, phidot, phi]
        A[:2, :2] = -Mi @ D; A[:2, 2] = (-Mi @ Gq)[:, 1]; A[2, 1] = 1.0
        B[:2, 0] = Mi @ B2
        # augment with the speed-error integrator, exact ZOH at Ts, dlqr
        Aa = np.zeros((4, 4)); Aa[:3, :3] = A; Ba = np.zeros((4, 1)); Ba[:3] = B
        n = 4; blk = np.zeros((n + 1, n + 1)); blk[:n, :n] = Aa; blk[:n, n:] = Ba; Z = la.expm(blk * self.Ts)
        Ad, Bd = Z[:n, :n], Z[:n, n:]
        Ad[3, 0] += self.Ts                                        # z_{k+1} = z_k + Ts (xdot - ref): discrete integrator as in model.m (-1 ... 1 rows)
        Q = np.diag([5 * (2 / r)**2, 1.0, 1.0, 1e-4 * (2 / r)**2]); R = np.array([[500.0]])
        P = la.solve_discrete_are(Ad, Bd, Q, R)
        self.K = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)[0]

    def init(self, p):
        self.z = 0.0
        if self.native:
            self.design(4.0, 0.1, 7.35e-4, 0.07, 0.2907, 0.0118625, 0.01); self.umax = 2.0
        else:
            self.design(p.M_b, p.m_w, p.I_w, p.r, p.l, p.I_b, 0.01); self.umax = p.tau_max

    def step(self, theta, theta_dot, x_dot, psi_dot, leg_len, r, v_ref, yaw_ref, t):
        self.z += (x_dot - v_ref) * self.Ts
        xs = x_dot - v_ref if self.ref_in_state else x_dot          # thesis eq. 3.19: reference enters only via the integrator
        u = -self.scale * float(self.K @ np.array([xs, theta_dot, theta, self.z]))        # total wheel torque
        w = float(np.clip(u / 2, -self.umax, self.umax))                                   # per wheel, +-2 Nm (theirs)
        return (w, w)


def make(Ts, **kw):
    return PyController(Ts, **kw)
