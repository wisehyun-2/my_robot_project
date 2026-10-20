"""
Two-Wheeled Inverted Pendulum (TWIP) model for a direct-drive self-balancing robot.

Longitudinal (pitch) dynamics, both wheels lumped, torque input = sum of wheel torques.
States: x = [x_w, x_w_dot, theta, theta_dot]
  x_w     : wheel (axle) position along ground [m]
  theta   : body pitch from vertical, + = leaning forward [rad]
Input:  tau = tau_L + tau_R  (direct-drive: wheel torque == motor torque) [N m]

Equations of motion (Lagrange, no slip), with
  a11 = M_b + 2 m_w + 2 I_w / r^2
  a12 = a21 = M_b l cos(theta)
  a22 = I_b + M_b l^2
  a11 * xdd + a12 * thdd - M_b l sin(th) thd^2 + b_x xd = tau / r
  a21 * xdd + a22 * thdd - M_b g l sin(th)                = -tau
(The reaction torque of the direct-drive stator acts on the body.)

Yaw dynamics (decoupled, small-signal):
  I_z psi_dd = (tau_R - tau_L) * (d / 2) / r
"""
from dataclasses import dataclass, asdict
import numpy as np

G = 9.81


@dataclass
class TWIPParams:
    name: str = "baseline"
    M_b: float = 7.0      # body mass (chassis + battery + computer), kg
    m_w: float = 1.2      # mass of ONE wheel incl. hub-motor rotor+stator moving with wheel, kg
    r: float = 0.0825     # wheel radius, m  (6.5 in hub motor ~165 mm dia)
    l: float = 0.25       # axle -> body COM distance, m
    I_b: float = 0.20     # body inertia about its COM (pitch axis), kg m^2
    I_w_factor: float = 0.7   # I_w = factor * m_w r^2 (0.5 solid disk .. 1.0 thin ring)
    b_x: float = 0.5      # viscous friction on translation, N s/m (rolling + bearing)
    d: float = 0.40       # track width (wheel-to-wheel), m
    I_z: float = 0.25     # yaw inertia of the whole robot, kg m^2
    tau_max: float = 8.0  # peak torque per wheel available from motor, N m
    Kt: float = 0.5       # torque constant per motor, N m / A (phase, peak-of-sine convention as drivers report)

    @property
    def I_w(self):
        return self.I_w_factor * self.m_w * self.r ** 2

    @property
    def M_tot(self):
        return self.M_b + 2 * self.m_w

    def mass_matrix(self, th):
        a11 = self.M_b + 2 * self.m_w + 2 * self.I_w / self.r ** 2
        a12 = self.M_b * self.l * np.cos(th)
        a22 = self.I_b + self.M_b * self.l ** 2
        return np.array([[a11, a12], [a12, a22]])

    def f(self, X, tau, F_dist=0.0):
        """Nonlinear dynamics. F_dist: horizontal force on the body COM [N]."""
        xw, xd, th, thd = X
        s, c = np.sin(th), np.cos(th)
        a11 = self.M_b + 2 * self.m_w + 2 * self.I_w / self.r ** 2
        a12 = self.M_b * self.l * c
        a22 = self.I_b + self.M_b * self.l ** 2
        r1 = tau / self.r + self.M_b * self.l * s * thd ** 2 - self.b_x * xd + F_dist
        r2 = -tau + self.M_b * G * self.l * s + F_dist * self.l * c
        det = a11 * a22 - a12 * a12
        xdd = (a22 * r1 - a12 * r2) / det
        thdd = (-a12 * r1 + a11 * r2) / det
        return np.array([xd, xdd, thd, thdd])

    def linearize(self):
        """Continuous-time A, B about upright equilibrium; also disturbance matrix Bd (force on COM)."""
        M = self.mass_matrix(0.0)
        Minv = np.linalg.inv(M)
        # acc = Minv @ ([ -b_x xd + tau/r + F ], [ M_b g l th - tau + F l ])
        A = np.zeros((4, 4))
        B = np.zeros((4, 1))
        Bd = np.zeros((4, 1))
        A[0, 1] = 1.0
        A[2, 3] = 1.0
        # d(acc)/d(xd)
        dacc_dxd = Minv @ np.array([-self.b_x, 0.0])
        # d(acc)/d(th)
        dacc_dth = Minv @ np.array([0.0, self.M_b * G * self.l])
        # d(acc)/d(tau)
        dacc_dtau = Minv @ np.array([1.0 / self.r, -1.0])
        dacc_dF = Minv @ np.array([1.0, self.l])
        A[1, 1], A[3, 1] = dacc_dxd
        A[1, 2], A[3, 2] = dacc_dth
        B[1, 0], B[3, 0] = dacc_dtau
        Bd[1, 0], Bd[3, 0] = dacc_dF
        return A, B, Bd

    def unstable_pole(self):
        A, _, _ = self.linearize()
        ev = np.linalg.eigvals(A)
        return float(np.max(ev.real))

    def summary(self):
        A, B, _ = self.linearize()
        ev = np.linalg.eigvals(A)
        return {
            **asdict(self),
            "I_w": self.I_w,
            "M_tot": self.M_tot,
            "open_loop_poles": np.sort_complex(ev).tolist(),
            "unstable_pole_rad_s": self.unstable_pole(),
            "unstable_time_constant_ms": 1000.0 / self.unstable_pole(),
            "static_hold_torque_10deg_Nm": self.M_b * G * self.l * np.sin(np.deg2rad(10)),
            "static_hold_torque_20deg_Nm": self.M_b * G * self.l * np.sin(np.deg2rad(20)),
            "slope_hold_torque_10deg_total_Nm": self.M_tot * G * np.sin(np.deg2rad(10)) * self.r,
        }


# Parameter sets ---------------------------------------------------------------
# 1) Repo-1-like small robot: 105 mm wheels (r = 0.0525 from controller_main.h), light body.
REPO1_LIKE = TWIPParams(name="repo1_like_105mm_wheels", M_b=4.0, m_w=0.6, r=0.0525, l=0.18,
                        I_b=0.06, b_x=0.3, d=0.30, I_z=0.08, tau_max=3.0, Kt=0.25)
# 2) Planned robot: ~10 kg, 6.5 in direct-drive hub motors, COM 25 cm above axle.
BASELINE = TWIPParams(name="planned_10kg_6p5in_hub", M_b=7.0, m_w=1.2, r=0.0825, l=0.25,
                      I_b=0.20, b_x=0.5, d=0.40, I_z=0.25, tau_max=8.0, Kt=0.5)
# 3) Heavier, taller (DIABLO-ish mass class, legs extended)
HEAVY = TWIPParams(name="heavy_15kg_8in_hub", M_b=12.0, m_w=1.6, r=0.10, l=0.35,
                   I_b=0.60, b_x=0.8, d=0.45, I_z=0.6, tau_max=15.0, Kt=0.8)

if __name__ == "__main__":
    import json
    for p in (REPO1_LIKE, BASELINE, HEAVY):
        s = p.summary()
        print(json.dumps({k: (v if not isinstance(v, list) else [str(c) for c in v]) for k, v in s.items()}, indent=1))
