"""
plant6.py -- the three-body planar wheel-leg model used by the RoboMaster / HEU / Skythinker lineage of
six-motor (five-bar) balancing robots.  Derived symbolically (sympy) from the same three equations every one of
those projects uses in MATLAB (sys_calc.m / chassis_calc_heu.m / model_LQR.m):

  Nm = M (x'' + (L+Lm)(th'' cos th - th'^2 sin th) - l (phi'' cos phi - phi'^2 sin phi))
  Pm = M g + M ((L+Lm)(-th'^2 cos th - th'' sin th) - l (phi'^2 cos phi + phi'' sin phi))
  N  = Nm + mp (x'' + L (th'' cos th - th'^2 sin th))
  P  = Pm + mp g + mp L (-th'^2 cos th - th'' sin th)
  (1) x''  = (T - N R) / (Iw/R + mw R)
  (2) (P L + Pm Lm) sin th - (N L + Nm Lm) cos th - T + Tp - Ip th'' = 0
  (3) Tp + Nm l cos phi + Pm l sin phi - Im phi'' = 0

States X = [theta, theta_dot, x, x_dot, phi, phi_dot]  (theta: virtual-leg angle from vertical, hip displaced
forward for theta > 0; phi: body pitch, body COM at -l sin(phi) from the hip; x: wheel position),
inputs U = [T, Tp] (wheel torque, hip torque acting on the body / reacting on the leg).
Leg length L0 = L + Lm (L: axle -> leg COM, Lm: leg COM -> hip) is a quasi-static parameter (set by the leg-length
force F through the VMC, which these controllers handle with a separate PD).  Ip scales with L0.
"""
import functools
import numpy as np
import sympy as sp

_sym = {}


def _build():
    th, thd, thdd, x, xd, xdd, ph, phd, phdd = sp.symbols("th thd thdd x xd xdd ph phd phdd")
    T, Tp, F = sp.symbols("T Tp F")
    R, L, Lm, l, mw, mp, M, Iw, Ip, Im, g = sp.symbols("R L Lm l mw mp M Iw Ip Im g", positive=True)
    # external horizontal push F on the body COM: the leg->body force becomes Nm = M*a_body - F
    Nm = M * (xdd + (L + Lm) * (thdd * sp.cos(th) - thd**2 * sp.sin(th)) - l * (phdd * sp.cos(ph) - phd**2 * sp.sin(ph))) - F
    Pm = M * g + M * ((L + Lm) * (-thd**2 * sp.cos(th) - thdd * sp.sin(th)) - l * (phd**2 * sp.cos(ph) + phdd * sp.sin(ph)))
    N = Nm + mp * (xdd + L * (thdd * sp.cos(th) - thd**2 * sp.sin(th)))
    P = Pm + mp * g + mp * L * (-thd**2 * sp.cos(th) - thdd * sp.sin(th))
    e1 = xdd - (T - N * R) / (Iw / R + mw * R)
    e2 = (P * L + Pm * Lm) * sp.sin(th) - (N * L + Nm * Lm) * sp.cos(th) - T + Tp - Ip * thdd
    e3 = Tp + Nm * l * sp.cos(ph) + Pm * l * sp.sin(ph) - Im * phdd
    # linear in the accelerations: M_acc * [xdd, thdd, phdd] = rhs   (no symbolic simplification: solve numerically)
    M_acc, rhs = sp.linear_eq_to_matrix([e1, e2, e3], [xdd, thdd, phdd])
    params = (R, L, Lm, l, mw, mp, M, Iw, Ip, Im, g)
    args = (th, thd, x, xd, ph, phd, T, Tp, F) + params
    fM = sp.lambdify(args, M_acc, "numpy"); fr = sp.lambdify(args, rhs, "numpy")

    def f(*a):
        acc = np.linalg.solve(np.array(fM(*a), dtype=float), np.array(fr(*a), dtype=float).ravel())
        return acc[0], acc[1], acc[2]

    return f


@functools.lru_cache(maxsize=1)
def _funcs():
    return _build()


class Plant6:
    """Three-body wheel-leg plant.  Parameters follow the MATLAB scripts' names."""

    def __init__(self, name, R, mw, mp, M, Iw, Im, l, L0, L_ratio=0.5, Ip_fn=None, tau_wheel_max=10.0, tau_hip_max=30.0,
                 b_x=0.3):
        self.name = name; self.R = R; self.mw = mw; self.mp = mp; self.M = M; self.Iw = Iw; self.Im = Im; self.l = l
        self.L0 = L0; self.L_ratio = L_ratio       # L = L_ratio * L0 (leg COM fraction from the axle)
        self.Ip_fn = Ip_fn or (lambda L0: mp * L0**2 / 12.0)
        self.tau_wheel_max = tau_wheel_max; self.tau_hip_max = tau_hip_max; self.b_x = b_x
        self.g = 9.81
        self.f = _funcs()

    def params(self, L0=None):
        L0 = self.L0 if L0 is None else L0
        L = self.L_ratio * L0; Lm = L0 - L
        return (self.R, L, Lm, self.l, self.mw, self.mp, self.M, self.Iw, self.Ip_fn(L0), self.Im, self.g)

    def deriv(self, X, T, Tp, F_dist=0.0, L0=None):
        th, thd, x, xd, ph, phd = X
        # horizontal disturbance force on the hip: add as an equivalent torque pair? keep simple: push acts on the
        # body COM -> approximate as extra wheel torque -F*R (force on the ground contact) + hip moment F*L0
        T_eff = T - self.b_x * xd * self.R          # viscous rolling/bearing friction as a wheel torque
        xdd, thdd, phdd = self.f(th, thd, x, xd, ph, phd, T_eff, Tp, F_dist, *self.params(L0))
        return np.array([thd, thdd, xd, xdd, phd, phdd], dtype=float)

    def linearize(self, L0=None, eps=1e-6):
        """A, B by central finite differences of the nonlinear derivative at the upright equilibrium."""
        X0 = np.zeros(6); A = np.zeros((6, 6)); B = np.zeros((6, 2))
        for i in range(6):
            d = np.zeros(6); d[i] = eps
            A[:, i] = (self.deriv(X0 + d, 0, 0, L0=L0) - self.deriv(X0 - d, 0, 0, L0=L0)) / (2 * eps)
        for j, (dT, dTp) in enumerate(((eps, 0), (0, eps))):
            B[:, j] = (self.deriv(X0, dT, dTp, L0=L0) - self.deriv(X0, -dT, -dTp, L0=L0)) / (2 * eps)
        return A, B


# ---------------------------------------------------------------- parameter sets
# USTB Reborn (JerryGong0911) documented parameters: R 0.076, m_w 0.47, m_p 1.874, M 6.975, I_w 0.001974, I_M 0.143, l 0.0353
# leg: L = 0.1576 L0 + 0.0272, L_M = 0.8424 L0 - 0.0272 (-> L_ratio ~ 0.16..0.3), I_p = 0.0328 L0 + 0.0216
RM_CLASS = dict(name="RM_class_USTB_Reborn", R=0.076, mw=0.47, mp=1.874, M=6.975, Iw=0.001974, Im=0.143, l=0.0353, L0=0.20,
                L_ratio=0.1576 + 0.0272 / 0.20, Ip_fn=lambda L0: 0.0328 * L0 + 0.0216, tau_wheel_max=6.0, tau_hip_max=30.0)
# LiuDingchuan (Webots) documented: R 0.05, mw 6 (3*2), mp 1.616 (0.808*2), M 12, Iw 0.0075161, Im 0.78714333*0.062665311, l 0
LIU_CLASS = dict(name="Liu_Webots_15kg", R=0.05, mw=6.0, mp=1.616, M=12.0, Iw=0.0075161, Im=0.0493, l=0.0, L0=0.25,
                 L_ratio=0.5, tau_wheel_max=10.0, tau_hip_max=40.0)
# Skythinker616 sys_calc.m (0.6 kg desktop robot): R 0.02, mw 0.251327, mp 0.0248, M 0.325, Iw 5.02655e-5,
# Ip (9.66667e-7 + 7.9975e-6)*2 (constant), Im 3.38542e-4, l 0, L = Lm = L0/2, L0 0.04..0.14 (nominal 0.07);
# torque limits (2804-class gimbal motors) and friction are assumptions.
SKY_CLASS = dict(name="Sky_0.6kg_desktop", R=0.02, mw=0.251327, mp=0.0248, M=0.325, Iw=5.02655e-5, Im=0.000338542, l=0.0,
                 L0=0.07, L_ratio=0.5, Ip_fn=lambda L0: (9.66667e-07 + 7.9975e-06) * 2, tau_wheel_max=0.2, tau_hip_max=0.3,
                 b_x=0.02)
# upkie (upkie/upkie): 6 motors (hip + knee qdd100, wheel mj5208 per leg), ~5.4 kg total, wheel radius 0.06 m, COM height
# ~0.58 m (MPC default leg_length).  Mass split is an estimate (no URDF in the repo): wheels 0.8, legs 1.2, torso 3.4 kg;
# serial leg at its nominal bent pose ~0.40 m axle->hip, torso COM 0.12 m above the hip; mj5208 1.7 Nm, qdd100 16 Nm.
UPKIE_CLASS = dict(name="upkie_5.4kg", R=0.06, mw=0.8, mp=1.2, M=3.4, Iw=2 * 0.5 * 0.4 * 0.06**2, Im=0.06, l=0.12, L0=0.40,
                   L_ratio=0.5, tau_wheel_max=1.7, tau_hip_max=16.0)
# our target classes with five-bar legs (assumed)
SMALL5 = dict(name="five_bar_5kg", R=0.065, mw=1.3, mp=0.6, M=3.0, Iw=2 * 0.7 * 0.65 * 0.065**2, Im=0.05, l=0.02, L0=0.16,
              L_ratio=0.5, tau_wheel_max=5.0, tau_hip_max=12.0)
LARGE10 = dict(name="five_bar_10kg", R=0.0825, mw=1.3, mp=1.2, M=6.5, Iw=2 * 0.7 * 0.65 * 0.0825**2, Im=0.15, l=0.03, L0=0.25,
               L_ratio=0.5, tau_wheel_max=5.0, tau_hip_max=24.0)

if __name__ == "__main__":
    import scipy.linalg as la, control as ct
    p = Plant6(**RM_CLASS)
    A, B = p.linearize()
    print("open-loop eigenvalues (RM class, L0 = 0.20):", np.round(np.linalg.eigvals(A), 2))
    # --- validation against USTB Reborn's published gain polynomials (k[12][3], quadratic in leg length)
    Q = np.diag([160, 160, 200, 120, 2000, 50]); Rm = np.diag([1, 0.25])
    k = np.array([[175.9120, -170.6678, -17.1254], [35.0040, -31.1551, -3.6713], [20.3988, -21.8621, -6.6372], [16.7956, -19.7366, -10.9749],
                  [101.1676, -109.3629, 43.8122], [18.3027, -19.2733, 8.2075], [-161.4978, 56.6788, 27.0371], [-1.5529, -14.1453, 11.7354],
                  [-36.9927, 11.8450, 6.5878], [-31.0895, 2.3105, 10.9002], [-188.9519, 156.6850, 49.1255], [-31.0208, 26.7038, 7.4055]])
    for L0 in (0.15, 0.20, 0.30):
        pp = Plant6(**{**RM_CLASS, "L0": L0, "L_ratio": 0.1576 + 0.0272 / L0}); pp.b_x = 0.0
        A, B = pp.linearize()
        sysd = ct.c2d(ct.ss(A, B, np.eye(6), np.zeros((6, 2))), 1e-3)
        P = la.solve_discrete_are(np.asarray(sysd.A), np.asarray(sysd.B), Q, Rm)
        K = np.linalg.solve(Rm + sysd.B.T @ P @ sysd.B, sysd.B.T @ P @ sysd.A)
        Kj = (k[:, 0] * L0**2 + k[:, 1] * L0 + k[:, 2]).reshape(2, 6)
        print(f"L0={L0}: my dLQR K row T  = {np.round(K[0], 2)}\n         USTB poly K row T = {np.round(Kj[0], 2)}\n"
              f"         my dLQR K row Tp = {np.round(K[1], 2)}\n         USTB poly K row Tp= {np.round(Kj[1], 2)}")
