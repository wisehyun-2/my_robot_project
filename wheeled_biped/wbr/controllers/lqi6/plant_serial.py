"""Three-body planar plant of a wheeled biped with SERIAL hip-knee legs (upkie, Plan 4), derived
from the Lagrangian -- no parallelogram assumption (docs/plan4_model.md).

Bodies (both legs lumped into one, all masses/inertias are totals):
  wheel  mass m_w, inertia I_w about the axle (tires, hubs, rotors + reflected rotor armature),
         rolls without slipping: position x, angle x / R
  leg    the two serial legs held at a fixed length L0 (knee position controlled by the leg PD),
         treated as one rigid body: mass m_l(L0), COM at (d, c) in the leg frame (c along the
         virtual leg from the axle towards the hip, d perpendicular, positive forward when the
         leg is vertical), inertia I_l(L0) about its COM.  These depend on the knee angle, i.e.
         on L0, and come from a table computed on the MuJoCo model (wbr.models.upkie_params).
  body   torso + batteries + hip stators: mass m_b, COM at (b_x, b_z) from the hip in the body
         frame, inertia I_b about its COM.

Generalized coordinates q = (x, theta, phi):
  x      wheel position
  theta  virtual-leg angle from the vertical, positive when the hip is AHEAD of the wheel
         (rotation about +y)
  phi    body pitch, positive when the body leans BACKWARDS (rotation about -y), so that
         phi = -IMU_pitch; the body COM sits at hip + (b_x cos phi - b_z sin phi, b_x sin phi + b_z cos phi)
Inputs u = (T, Tp): total wheel torque (positive drives forward) and total hip torque applied on
the leg about +y (reaction -Tp on the body).  The hip joint coordinate is q_hip = theta + phi.

Generalized forces: Q_x = T / R - c_w xdot / R^2,  Q_theta = -T + Tp - c_h (thetadot + phidot),
Q_phi = Tp - c_h (thetadot + phidot)   (c_w, c_h: wheel / hip joint viscous damping of the model).

The serial structure enters through (i) the L0-dependent lumped leg parameters, (ii) the hip
torque acting on the body-thigh joint q_hip = theta + phi (both coordinates receive +Tp, the wheel
reaction -T acts on the leg only), (iii) the hip rotor armature J_h on q_hip_dot.  With L0 fixed
the shank rotates with the virtual leg, so the wheel torque reaction is a pure torque -T on the leg.

Equilibrium (phi = 0): the total COM is above the axle,  m_l (d cos th + c sin th) + m_b (L0 sin th + b_x) = 0,
and the static hip torque is Tp_eq = m_b g b_x (= dV/dphi = dV/dtheta there).
Linearisation: M(q_eq) ddq = -(H_V) dq - D dq + E du  with H_V the Hessian of the potential, D the
damping matrix and E = [[1/R, 0], [-1, 1], [0, 1]].  State order of the design report:
X = [theta, theta_dot, x, x_dot, phi, phi_dot].
"""
from __future__ import annotations

import functools
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict

import numpy as np
import sympy as sp

G = 9.81

PARAM_NAMES = ("R", "m_w", "I_w", "m_l", "c", "d", "I_l", "m_b", "b_x", "b_z", "I_b", "L0", "J_h", "c_h", "c_w", "g")


@functools.lru_cache(maxsize=1)
def _symbolic():
    """Lagrangian derivation -> lambdified mass matrix, potential gradient/Hessian and full dynamics."""
    x, th, ph, xd, thd, phd, xdd, thdd, phdd = sp.symbols("x th ph xd thd phd xdd thdd phdd")
    T, Tp = sp.symbols("T Tp")
    P = sp.symbols(" ".join(PARAM_NAMES), real=True)
    R, m_w, I_w, m_l, c, d, I_l, m_b, b_x, b_z, I_b, L0, J_h, c_h, c_w, g = P
    q = sp.Matrix([x, th, ph])
    qd = sp.Matrix([xd, thd, phd])
    qdd = sp.Matrix([xdd, thdd, phdd])

    # positions (x, z) ---------------------------------------------------------------------
    p_leg = sp.Matrix([x + d * sp.cos(th) + c * sp.sin(th), R - d * sp.sin(th) + c * sp.cos(th)])
    p_hip = sp.Matrix([x + L0 * sp.sin(th), R + L0 * sp.cos(th)])
    p_body = p_hip + sp.Matrix([b_x * sp.cos(ph) - b_z * sp.sin(ph), b_x * sp.sin(ph) + b_z * sp.cos(ph)])

    def vel(p):
        return p.jacobian(q) * qd

    v_leg, v_body = vel(p_leg), vel(p_body)
    KE = (sp.Rational(1, 2) * (m_w + I_w / R**2) * xd**2
          + sp.Rational(1, 2) * m_l * (v_leg.T * v_leg)[0] + sp.Rational(1, 2) * I_l * thd**2
          + sp.Rational(1, 2) * m_b * (v_body.T * v_body)[0] + sp.Rational(1, 2) * I_b * phd**2
          + sp.Rational(1, 2) * J_h * (thd + phd)**2)
    V = g * (m_l * p_leg[1] + m_b * p_body[1])
    Lag = KE - V
    # generalized forces
    Q = sp.Matrix([T / R - c_w * xd / R**2, -T + Tp - c_h * (thd + phd), Tp - c_h * (thd + phd)])
    # Euler-Lagrange: d/dt(dL/dqd) - dL/dq = Q
    eqs = []
    for i in range(3):
        dL_dqd = sp.diff(Lag, qd[i])
        ddt = sum(sp.diff(dL_dqd, qd[j]) * qdd[j] + sp.diff(dL_dqd, q[j]) * qd[j] for j in range(3))
        eqs.append(sp.simplify(ddt - sp.diff(Lag, q[i]) - Q[i]))
    M_acc, rhs = sp.linear_eq_to_matrix(eqs, [xdd, thdd, phdd])
    Mq = sp.hessian(KE, qd)                       # mass matrix M(q)
    gradV = sp.Matrix([sp.diff(V, qi) for qi in q])
    HV = sp.hessian(V, q)
    args_dyn = (x, th, ph, xd, thd, phd, T, Tp) + P
    args_q = (th, ph) + P
    return dict(f_M=sp.lambdify(args_dyn, M_acc, "numpy"), f_rhs=sp.lambdify(args_dyn, rhs, "numpy"),
                f_Mq=sp.lambdify(args_q, Mq, "numpy"), f_gradV=sp.lambdify(args_q, gradV, "numpy"),
                f_HV=sp.lambdify(args_q, HV, "numpy"),
                latex=dict(KE=sp.latex(KE), V=sp.latex(V), Q=sp.latex(Q), Mq=sp.latex(sp.simplify(Mq))))


@dataclass
class LegTable:
    """Lumped parameters of the two legs as a function of the virtual leg length L0 (from MuJoCo)."""
    L0: np.ndarray
    m_l: np.ndarray
    c: np.ndarray
    d: np.ndarray
    I_l: np.ndarray

    def at(self, L0: float) -> Dict[str, float]:
        g = self.L0
        L0 = min(max(float(L0), float(g[0])), float(g[-1]))
        i = int(np.searchsorted(g, L0, side="right")) - 1
        i = min(max(i, 0), len(g) - 2)
        w = (L0 - g[i]) / (g[i + 1] - g[i])
        return {k: float(getattr(self, k)[i] * (1 - w) + getattr(self, k)[i + 1] * w) for k in ("m_l", "c", "d", "I_l")}


@dataclass
class SerialPlantParams:
    R: float                 # wheel radius
    m_w: float               # both wheels
    I_w: float               # both wheels about the axle (incl. rotor armature)
    m_b: float               # body (torso group)
    b_x: float               # body COM from the hip, body frame (forward)
    b_z: float               # body COM above the hip
    I_b: float               # body pitch inertia about its COM
    J_h: float               # hip rotor armature (both hips)
    c_h: float               # hip joint viscous damping (both hips), N m s / rad
    c_w: float               # wheel joint viscous damping (both wheels)
    hip_above_base: float    # not used by the plant; geometry note (base origin - hip, z)
    leg: LegTable
    L0: float                # standing length
    L_min: float
    L_max: float
    tau_wheel_max: float = 1.7   # per wheel (R weight of the LQR, torque clamp of the MCU)
    tau_hip_max: float = 16.0    # per joint
    name: str = "upkie_serial"
    g: float = G
    extra: dict = field(default_factory=dict)

    # ------------------------------------------------------------- (de)serialisation
    def to_json(self, path: Path) -> None:
        obj = {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in self.__dict__.items() if k != "leg"}
        obj["leg"] = {k: getattr(self.leg, k).tolist() for k in ("L0", "m_l", "c", "d", "I_l")}
        Path(path).write_text(json.dumps(obj, indent=1), encoding="utf-8")

    @classmethod
    def from_json(cls, path: Path) -> "SerialPlantParams":
        obj = json.loads(Path(path).read_text(encoding="utf-8"))
        leg = LegTable(**{k: np.asarray(v, dtype=float) for k, v in obj.pop("leg").items()})
        return cls(leg=leg, **obj)


class SerialPlant:
    """Plant object with the Plant6 interface used by LQI6 (linearize(L0) -> A, B; L0, tau_*_max)."""

    def __init__(self, params: SerialPlantParams):
        self.p = params
        self.L0 = params.L0
        self.tau_wheel_max = params.tau_wheel_max
        self.tau_hip_max = params.tau_hip_max
        self.g = params.g
        self.name = params.name
        self.sym = _symbolic()

    # ------------------------------------------------------------------- parameters
    def params(self, L0: float | None = None) -> tuple:
        p = self.p
        L0 = self.L0 if L0 is None else float(L0)
        leg = p.leg.at(L0)
        return (p.R, p.m_w, p.I_w, leg["m_l"], leg["c"], leg["d"], leg["I_l"], p.m_b, p.b_x, p.b_z, p.I_b, L0,
                p.J_h, p.c_h, p.c_w, p.g)

    def com_x(self, theta: float, L0: float | None = None, phi: float = 0.0) -> float:
        """Horizontal offset of the total COM (legs + body) from the axle."""
        P = self.params(L0)
        R, m_w, I_w, m_l, c, d, I_l, m_b, b_x, b_z, I_b, L0v, *_ = P
        x_leg = d * np.cos(theta) + c * np.sin(theta)
        x_body = L0v * np.sin(theta) + b_x * np.cos(phi) - b_z * np.sin(phi)
        return float((m_l * x_leg + m_b * x_body) / (m_l + m_b))

    def theta_eq(self, L0: float | None = None) -> float:
        """Equilibrium virtual-leg angle (body level): total COM above the axle.  Interpolated from a table
        on the leg-length grid (the exact bisection is theta_eq_exact; the table error is < 1e-4 rad)."""
        if not hasattr(self, "_theta_table"):
            grid = self.p.leg.L0
            self._theta_table = (grid, np.array([self.theta_eq_exact(float(L)) for L in grid]))
        L0 = self.L0 if L0 is None else float(L0)
        g, t = self._theta_table
        return float(np.interp(L0, g, t))

    def theta_eq_exact(self, L0: float | None = None) -> float:
        """Exact equilibrium angle by bisection on com_x (monotonic over [-1, 1] rad)."""
        lo, hi = -1.0, 1.0            # bisection (no scipy.optimize dependency); com_x is monotonic here
        f_lo = self.com_x(lo, L0)
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            f_mid = self.com_x(mid, L0)
            if (f_mid > 0) == (f_lo > 0):
                lo, f_lo = mid, f_mid
            else:
                hi = mid
        return 0.5 * (lo + hi)

    def Tp_eq(self, L0: float | None = None) -> float:
        """Static hip torque at the equilibrium (body level): dV/dphi = m_b g b_x."""
        P = self.params(L0)
        th = self.theta_eq_exact(L0)     # the interpolated table value is ~3e-5 rad off between grid points
        gradV = np.asarray(self.sym["f_gradV"](th, 0.0, *P), dtype=float).ravel()
        assert abs(gradV[1] - gradV[2]) < 1e-4 * max(1.0, abs(gradV[2])), "equilibrium inconsistency"
        return float(gradV[2])      # = m_b g b_x at phi = 0

    def com_height(self, L0: float | None = None) -> float:
        P = self.params(L0)
        R, m_w, I_w, m_l, c, d, I_l, m_b, b_x, b_z, I_b, L0v, *_ = P
        th = self.theta_eq(L0)
        z_leg = -d * np.sin(th) + c * np.cos(th)
        z_body = L0v * np.cos(th) + b_z
        return float((m_l * z_leg + m_b * z_body) / (m_l + m_b))

    # --------------------------------------------------------------------- dynamics
    def deriv(self, X, T: float, Tp: float, L0: float | None = None) -> np.ndarray:
        """Nonlinear state derivative, X = [theta, theta_dot, x, x_dot, phi, phi_dot]."""
        th, thd, x, xd, ph, phd = X
        P = self.params(L0)
        M = np.asarray(self.sym["f_M"](x, th, ph, xd, thd, phd, T, Tp, *P), dtype=float)
        r = np.asarray(self.sym["f_rhs"](x, th, ph, xd, thd, phd, T, Tp, *P), dtype=float).ravel()
        xdd, thdd, phdd = np.linalg.solve(M, r)
        return np.array([thd, thdd, xd, xdd, phd, phdd])

    def mass_matrix(self, theta: float, phi: float = 0.0, L0: float | None = None) -> np.ndarray:
        return np.asarray(self.sym["f_Mq"](theta, phi, *self.params(L0)), dtype=float)

    def linearize(self, L0: float | None = None):
        """A (6x6), B (6x2) at the equilibrium (theta_eq(L0), phi = 0, T = 0, Tp = Tp_eq)."""
        P = self.params(L0)
        R, c_h, c_w = P[0], P[13], P[14]
        th = self.theta_eq(L0)
        Mq = np.asarray(self.sym["f_Mq"](th, 0.0, *P), dtype=float)
        HV = np.asarray(self.sym["f_HV"](th, 0.0, *P), dtype=float)
        D = np.array([[c_w / R**2, 0.0, 0.0], [0.0, c_h, c_h], [0.0, c_h, c_h]])
        E = np.array([[1.0 / R, 0.0], [-1.0, 1.0], [0.0, 1.0]])
        Minv = np.linalg.inv(Mq)
        # (q, qd) ordering with q = (x, theta, phi)
        A_q = np.zeros((6, 6))
        A_q[:3, 3:] = np.eye(3)
        A_q[3:, :3] = -Minv @ HV
        A_q[3:, 3:] = -Minv @ D
        B_q = np.zeros((6, 2))
        B_q[3:, :] = Minv @ E
        # reorder to [theta, theta_dot, x, x_dot, phi, phi_dot]
        idx = [1, 4, 0, 3, 2, 5]
        return A_q[np.ix_(idx, idx)], B_q[idx, :]

    def equilibrium_state(self, L0: float | None = None) -> np.ndarray:
        return np.array([self.theta_eq(L0), 0.0, 0.0, 0.0, 0.0, 0.0])


if __name__ == "__main__":
    from wbr import MODELS_DIR
    plant = SerialPlant(SerialPlantParams.from_json(MODELS_DIR / "upkie_plan4" / "plan4_params.json"))
    for L0 in (0.25, 0.28, 0.31, 0.33):
        A, B = plant.linearize(L0)
        print(f"L0={L0:.2f} theta_eq={np.degrees(plant.theta_eq(L0)):+.2f} deg  Tp_eq={plant.Tp_eq(L0):+.3f} Nm  "
              f"eig(A)={np.round(np.sort_complex(np.linalg.eigvals(A)), 3)}")
