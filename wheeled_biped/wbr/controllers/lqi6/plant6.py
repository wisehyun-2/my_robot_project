"""Three-body planar wheel-leg plant used to design the LQI6 gains (port of
balancing_robot_design/sim6/plant6.py from the design report, section 8.2).

States X = [theta, theta_dot, x, x_dot, phi, phi_dot]  (theta: virtual-leg angle from vertical,
hip displaced forward for theta > 0; phi: body pitch, body COM at -l sin(phi) from the hip, i.e.
phi > 0 = COM behind the hip; x: wheel position), inputs U = [T, Tp] (total wheel torque, total
hip torque acting on the body / reacting on the leg).  The leg length L0 is a quasi-static
parameter.  Equations (Nm, Pm: leg->body force; N, P: wheel->leg force):

  Nm = M (x'' + (L+Lm)(th'' cos th - th'^2 sin th) - l (phi'' cos phi - phi'^2 sin phi))
  Pm = M g + M ((L+Lm)(-th'^2 cos th - th'' sin th) - l (phi'^2 cos phi + phi'' sin phi))
  N  = Nm + mp (x'' + L (th'' cos th - th'^2 sin th))
  P  = Pm + mp g + mp L (-th'^2 cos th - th'' sin th)
  (1) x''  = (T - N R) / (Iw/R + mw R)
  (2) (P L + Pm Lm) sin th - (N L + Nm Lm) cos th - T + Tp - Ip th'' = 0
  (3) Tp + Nm l cos phi + Pm l sin phi - Im phi'' = 0
"""
from __future__ import annotations

import functools

import numpy as np
import sympy as sp


def _build():
    th, thd, thdd, x, xd, xdd, ph, phd, phdd = sp.symbols("th thd thdd x xd xdd ph phd phdd")
    T, Tp, F = sp.symbols("T Tp F")
    R, L, Lm, l, mw, mp, M, Iw, Ip, Im, g = sp.symbols("R L Lm l mw mp M Iw Ip Im g", positive=True)
    Nm = M * (xdd + (L + Lm) * (thdd * sp.cos(th) - thd**2 * sp.sin(th)) - l * (phdd * sp.cos(ph) - phd**2 * sp.sin(ph))) - F
    Pm = M * g + M * ((L + Lm) * (-thd**2 * sp.cos(th) - thdd * sp.sin(th)) - l * (phd**2 * sp.cos(ph) + phdd * sp.sin(ph)))
    N = Nm + mp * (xdd + L * (thdd * sp.cos(th) - thd**2 * sp.sin(th)))
    P = Pm + mp * g + mp * L * (-thd**2 * sp.cos(th) - thdd * sp.sin(th))
    e1 = xdd - (T - N * R) / (Iw / R + mw * R)
    e2 = (P * L + Pm * Lm) * sp.sin(th) - (N * L + Nm * Lm) * sp.cos(th) - T + Tp - Ip * thdd
    e3 = Tp + Nm * l * sp.cos(ph) + Pm * l * sp.sin(ph) - Im * phdd
    M_acc, rhs = sp.linear_eq_to_matrix([e1, e2, e3], [xdd, thdd, phdd])
    params = (R, L, Lm, l, mw, mp, M, Iw, Ip, Im, g)
    args = (th, thd, x, xd, ph, phd, T, Tp, F) + params
    fM = sp.lambdify(args, M_acc, "numpy")
    fr = sp.lambdify(args, rhs, "numpy")

    def f(*a):
        acc = np.linalg.solve(np.array(fM(*a), dtype=float), np.array(fr(*a), dtype=float).ravel())
        return acc[0], acc[1], acc[2]

    return f


@functools.lru_cache(maxsize=1)
def _funcs():
    return _build()


class Plant6:
    """Three-body wheel-leg plant (parameter names follow the RoboMaster MATLAB scripts)."""

    def __init__(self, name, R, mw, mp, M, Iw, Im, l, L0, L_ratio=0.5, Ip_fn=None, tau_wheel_max=10.0,
                 tau_hip_max=30.0, b_x=0.3):
        self.name = name
        self.R, self.mw, self.mp, self.M, self.Iw, self.Im, self.l = R, mw, mp, M, Iw, Im, l
        self.L0, self.L_ratio = L0, L_ratio
        self.Ip_fn = Ip_fn or (lambda L0: mp * L0**2 / 12.0)
        self.tau_wheel_max, self.tau_hip_max, self.b_x = tau_wheel_max, tau_hip_max, b_x
        self.g = 9.81
        self.f = _funcs()

    def params(self, L0=None):
        L0 = self.L0 if L0 is None else L0
        L = self.L_ratio * L0
        Lm = L0 - L
        return (self.R, L, Lm, self.l, self.mw, self.mp, self.M, self.Iw, self.Ip_fn(L0), self.Im, self.g)

    def deriv(self, X, T, Tp, F_dist=0.0, L0=None):
        th, thd, x, xd, ph, phd = X
        T_eff = T - self.b_x * xd * self.R
        xdd, thdd, phdd = self.f(th, thd, x, xd, ph, phd, T_eff, Tp, F_dist, *self.params(L0))
        return np.array([thd, thdd, xd, xdd, phd, phdd], dtype=float)

    def linearize(self, L0=None, eps=1e-6):
        X0 = np.zeros(6)
        A = np.zeros((6, 6))
        B = np.zeros((6, 2))
        for i in range(6):
            d = np.zeros(6)
            d[i] = eps
            A[:, i] = (self.deriv(X0 + d, 0, 0, L0=L0) - self.deriv(X0 - d, 0, 0, L0=L0)) / (2 * eps)
        for j, (dT, dTp) in enumerate(((eps, 0), (0, eps))):
            B[:, j] = (self.deriv(X0, dT, dTp, L0=L0) - self.deriv(X0, -dT, -dTp, L0=L0)) / (2 * eps)
        return A, B


# parameter sets ------------------------------------------------------------------------------
# five-bar 5 kg class of the design report (assumed values)
SMALL5 = dict(name="five_bar_5kg", R=0.065, mw=1.3, mp=0.6, M=3.0, Iw=2 * 0.7 * 0.65 * 0.065**2, Im=0.05, l=0.02,
              L0=0.16, L_ratio=0.5, tau_wheel_max=5.0, tau_hip_max=12.0)


def diablo_lite_params(model_inertia_yy: float | None = None) -> dict:
    """Plant parameters of the 6 kg DIABLO-lite MuJoCo model (wbr.models.build_diablo_lite)."""
    from wbr.models.build_diablo_lite import DiabloLiteParams
    p = DiabloLiteParams()
    body = p.m_torso + 4 * p.m_motor
    L0 = p.L1 * np.cos(p.theta1_nom) + p.L2 * np.cos(p.theta0_nom)
    Iw = 2 * 0.5 * p.m_wheel * p.wheel_radius**2
    Im = model_inertia_yy if model_inertia_yy is not None else 0.0234   # body pitch inertia about its COM (base frame)
    l = p.m_torso * p.torso_z / body
    mp = 2 * (p.m_thigh + p.m_shank + p.m_crank + p.m_rod)
    return dict(name="diablo_lite_6kg", R=p.wheel_radius, mw=2 * p.m_wheel, mp=mp, M=body, Iw=Iw, Im=Im, l=l, L0=float(L0),
                L_ratio=0.5, tau_wheel_max=p.tau_wheel, tau_hip_max=p.tau_hip, b_x=0.1)
