"""Kinematics and virtual model control (VMC) of the DIABLO-lite parallelogram leg.

Hip frame: x forward, z up, rotations about +y.  A link of length L pointing "down" rotated by
the absolute angle theta has direction u(theta) = (-sin theta, -cos theta) in (x, z).
    wheel axle position  p = L1 u(theta1) + L2 u(theta0)
with theta1 = theta1_nom + q_hip (thigh, Motor1) and theta0 = theta0_nom + q_crank (shank, set by
the crank of Motor0 through the parallelogram).  The virtual leg is the hip -> axle segment:
length L0 = |p|, angle theta_v = atan2(-p_x, -p_z) (positive when the hip is ahead of the wheel,
i.e. the same convention as the three-body plant of the design report).

VMC: a leg force F along the virtual leg (positive = extension, pushing the wheel away from the
hip) and a hip torque Tp applied on the leg about +y (positive rotates the hip forward over the
wheel; its reaction on the body is -Tp) are mapped to motor torques through tau = J^T F_cart with
F_cart = F u(theta_v) + (Tp / L0) u'(theta_v).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class LegGeometry:
    L1: float = 0.14
    L2: float = 0.14
    theta1_nom: float = -math.pi / 4
    theta0_nom: float = math.pi / 4

    @property
    def L0_max(self) -> float:
        return self.L1 + self.L2


def u(theta: float) -> np.ndarray:
    return np.array([-math.sin(theta), -math.cos(theta)])


def du(theta: float) -> np.ndarray:
    return np.array([-math.cos(theta), math.sin(theta)])


class FiveBarLeg:
    def __init__(self, geom: LegGeometry | None = None):
        self.g = geom or LegGeometry()

    def angles(self, q_hip: float, q_crank: float):
        return self.g.theta1_nom + q_hip, self.g.theta0_nom + q_crank

    def forward(self, q_hip: float, q_crank: float):
        """Axle position p (x, z) in the hip frame, virtual leg length L0 and angle theta_v."""
        t1, t0 = self.angles(q_hip, q_crank)
        p = self.g.L1 * u(t1) + self.g.L2 * u(t0)
        L0 = float(np.linalg.norm(p))
        theta_v = math.atan2(-p[0], -p[1])
        return p, L0, theta_v

    def jacobian(self, q_hip: float, q_crank: float) -> np.ndarray:
        """d p / d [q_hip, q_crank] (2 x 2)."""
        t1, t0 = self.angles(q_hip, q_crank)
        J = np.zeros((2, 2))
        J[:, 0] = self.g.L1 * du(t1)
        J[:, 1] = self.g.L2 * du(t0)
        return J

    def virtual_rates(self, q_hip: float, q_crank: float, qd_hip: float, qd_crank: float):
        """Time derivatives (L0_dot, theta_v_dot) from the joint rates."""
        p, L0, theta_v = self.forward(q_hip, q_crank)
        pd = self.jacobian(q_hip, q_crank) @ np.array([qd_hip, qd_crank])
        L0_dot = float(u(theta_v) @ pd)
        theta_v_dot = float(du(theta_v) @ pd) / max(L0, 1e-3)
        return L0_dot, theta_v_dot

    def inverse(self, L0: float, theta_v: float):
        """Joint angles (q_hip, q_crank) placing the axle at length L0 and angle theta_v (knee forward)."""
        L1, L2 = self.g.L1, self.g.L2
        L0 = min(max(L0, abs(L1 - L2) + 1e-4), L1 + L2 - 1e-4)
        # triangle hip - knee - axle: angle at the hip between the virtual leg and the thigh
        cos_a = (L1 * L1 + L0 * L0 - L2 * L2) / (2 * L1 * L0)
        a = math.acos(max(-1.0, min(1.0, cos_a)))
        t1 = theta_v - a                          # thigh rotated toward the front (knee forward)
        p = L0 * u(theta_v) - L1 * u(t1)
        t0 = math.atan2(-p[0], -p[1])
        return t1 - self.g.theta1_nom, t0 - self.g.theta0_nom

    def vmc_torques(self, F: float, Tp: float, q_hip: float, q_crank: float):
        """Motor torques (tau_hip, tau_crank) realising leg force F and hip torque Tp (see module doc)."""
        p, L0, theta_v = self.forward(q_hip, q_crank)
        F_cart = F * u(theta_v) + (Tp / max(L0, 1e-3)) * du(theta_v)
        tau = self.jacobian(q_hip, q_crank).T @ F_cart
        return float(tau[0]), float(tau[1])


if __name__ == "__main__":
    leg = FiveBarLeg()
    p, L0, th = leg.forward(0.0, 0.0)
    print("nominal axle", np.round(p, 4), "L0", round(L0, 4), "theta_v", round(th, 4))
    q = leg.inverse(0.15, 0.1)
    print("inverse(0.15, 0.1) ->", np.round(q, 4), "check", leg.forward(*q)[1:])
    J = leg.jacobian(0.1, -0.2)
    eps = 1e-6
    Jn = np.column_stack([(leg.forward(0.1 + eps, -0.2)[0] - leg.forward(0.1 - eps, -0.2)[0]) / (2 * eps),
                          (leg.forward(0.1, -0.2 + eps)[0] - leg.forward(0.1, -0.2 - eps)[0]) / (2 * eps)])
    print("jacobian error", np.abs(J - Jn).max())
    print("static support torques for F = 29.4 N at nominal:", leg.vmc_torques(29.4, 0.0, 0.0, 0.0))
