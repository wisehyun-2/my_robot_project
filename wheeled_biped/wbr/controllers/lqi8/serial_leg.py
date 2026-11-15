"""Kinematics and virtual model control (VMC) of a serial hip-knee leg (the v5 robot: L1 = L2 = 250 mm, knees behind).

Conventions (the DIABLO-style MCU program is written for them):

Hip frame: x forward, z up, rotations about +y.  A link of length L pointing "down" rotated by the
absolute angle t has direction u(t) = (-sin t, -cos t) in (x, z), i.e. a positive angle swings the
link end backwards (hip ahead of the link end).
    thigh angle   t1 = q_hip                 (q = 0: thigh straight down)
    shank angle   t2 = q_hip + q_knee        (knee joint angle is the thigh->shank relative angle)
    axle position p  = L1 u(t1) + L2 u(t2)
The virtual leg is the hip -> axle segment: length L0 = |p|, angle theta_v = atan2(-p_x, -p_z)
(positive when the hip is ahead of the wheel, the convention of the three-body plant).

Joint sign convention: the "u-convention" above is what the controller sees.  The plan assembly multiplies the
model's joint positions/velocities/torques by the robot's joint signs before they enter the MCU and the commanded
torques by the same signs on the way out (the v5 model's joint axes are all +y: every sign is +1, description.json).

VMC: a leg force F along the virtual leg (positive = extension) and a hip torque Tp applied on the
leg about +y (positive rotates the hip forward over the wheel; reaction -Tp on the body) are mapped
to joint torques through tau = J^T F_cart with F_cart = F u(theta_v) + (Tp / L0) u'(theta_v).

Singularities: at full extension (q_knee = 0, L0 = L1 + L2) the Jacobian loses rank -- the leg
length cannot be controlled and the knee bending direction is undefined.  ``inverse`` therefore
clips L0 to L0_max - 1 mm and the standing length keeps the knee bent (L_stand < L0_max).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class SerialLegGeometry:
    L1: float                   # hip -> knee [m]
    L2: float                   # knee -> wheel axle [m]
    knee_sign: float = +1.0     # +1: knee behind the hip->axle line (u-convention: thigh angle > theta_v), -1: knee in front

    @property
    def L0_max(self) -> float:
        return self.L1 + self.L2

    @property
    def L0_min(self) -> float:
        return abs(self.L1 - self.L2)


def u(theta: float) -> np.ndarray:
    return np.array([-math.sin(theta), -math.cos(theta)])


def du(theta: float) -> np.ndarray:
    return np.array([-math.cos(theta), math.sin(theta)])


class SerialLeg:
    """Hip-knee serial leg with the MCU's leg interface (forward / jacobian / virtual_rates / inverse /
    vmc_torques / shank_rate)."""

    def __init__(self, geom: SerialLegGeometry):
        self.g = geom

    # ------------------------------------------------------------------ kinematics
    def angles(self, q_hip: float, q_knee: float):
        return q_hip, q_hip + q_knee

    def forward(self, q_hip: float, q_knee: float):
        """Axle position p (x, z) in the hip frame, virtual leg length L0 and angle theta_v."""
        t1, t2 = self.angles(q_hip, q_knee)
        p = self.g.L1 * u(t1) + self.g.L2 * u(t2)
        L0 = float(np.linalg.norm(p))
        theta_v = math.atan2(-p[0], -p[1])
        return p, L0, theta_v

    def jacobian(self, q_hip: float, q_knee: float) -> np.ndarray:
        """d p / d [q_hip, q_knee] (2 x 2)."""
        t1, t2 = self.angles(q_hip, q_knee)
        J = np.zeros((2, 2))
        J[:, 0] = self.g.L1 * du(t1) + self.g.L2 * du(t2)
        J[:, 1] = self.g.L2 * du(t2)
        return J

    def virtual_rates(self, q_hip: float, q_knee: float, qd_hip: float, qd_knee: float):
        """Time derivatives (L0_dot, theta_v_dot) from the joint rates."""
        p, L0, theta_v = self.forward(q_hip, q_knee)
        pd = self.jacobian(q_hip, q_knee) @ np.array([qd_hip, qd_knee])
        L0_dot = float(u(theta_v) @ pd)
        theta_v_dot = float(du(theta_v) @ pd) / max(L0, 1e-3)
        return L0_dot, theta_v_dot

    def shank_rate(self, qd_hip: float, qd_knee: float) -> float:
        """Angular rate of the shank (wheel carrier) in the hip frame, about +y."""
        return qd_hip + qd_knee

    def inverse(self, L0: float, theta_v: float):
        """Joint angles (q_hip, q_knee) placing the axle at length L0 and angle theta_v (knee side =
        geometry.knee_sign)."""
        L1, L2 = self.g.L1, self.g.L2
        L0 = min(max(L0, self.g.L0_min + 1e-4), self.g.L0_max - 1e-3)
        cos_a = (L1 * L1 + L0 * L0 - L2 * L2) / (2 * L1 * L0)   # angle at the hip between virtual leg and thigh
        a = math.acos(max(-1.0, min(1.0, cos_a)))
        t1 = theta_v + self.g.knee_sign * a
        p = L0 * u(theta_v) - L1 * u(t1)
        t2 = math.atan2(-p[0], -p[1])
        q_knee = t2 - t1
        q_knee = (q_knee + math.pi) % (2 * math.pi) - math.pi
        return t1, q_knee

    # ------------------------------------------------------------------------- VMC
    def vmc_torques(self, F: float, Tp: float, q_hip: float, q_knee: float):
        """Joint torques (tau_hip, tau_knee) realising leg force F and hip torque Tp (see module doc)."""
        p, L0, theta_v = self.forward(q_hip, q_knee)
        F_cart = F * u(theta_v) + (Tp / max(L0, 1e-3)) * du(theta_v)
        tau = self.jacobian(q_hip, q_knee).T @ F_cart
        return float(tau[0]), float(tau[1])

    def wheel_reaction_ratio(self, q_hip: float, q_knee: float) -> float:
        """Knee torque per N m of wheel motor torque that keeps the wheel motor's reaction on the shank out of the
        leg length: tau_knee += ratio * tau_wheel.

        The wheel motor applies tau_wheel on the wheel and -tau_wheel on the shank (both about +y).  With the virtual
        leg angle held, the thigh and the shank turn apart from it by d(alpha) = (L2 cos(beta) / L0) d(q_knee) and
        d(beta) = (L1 cos(alpha) / L0) d(q_knee), so the virtual work gives the torque on the knee coordinate
            Q_knee = tau_knee - (L2 cos(beta) / L0) tau_hip - (L1 cos(alpha) / L0) tau_wheel.
        The hip term is what ``vmc_torques`` already adds to tau_knee for Tp; this is the wheel term.  L1 = L2: 1/2."""
        _, L0, theta_v = self.forward(q_hip, q_knee)
        return self.g.L1 * math.cos(q_hip - theta_v) / max(L0, 1e-3)


if __name__ == "__main__":
    leg = SerialLeg(SerialLegGeometry(L1=0.25, L2=0.25, knee_sign=+1.0))      # the v5 leg
    for L0 in (0.33, 0.40, 0.46, 0.485):
        q = leg.inverse(L0, 0.0)
        p, L0b, th = leg.forward(*q)
        print(f"L0={L0:.3f}: q_hip={q[0]:+.4f} q_knee={q[1]:+.4f}  -> L0={L0b:.4f} theta_v={th:+.2e}  "
              f"static torques for F=24.5 N: {np.round(leg.vmc_torques(24.5, 0.0, *q), 3)}")
    J = leg.jacobian(0.3, -0.9)
    eps = 1e-6
    Jn = np.column_stack([(leg.forward(0.3 + eps, -0.9)[0] - leg.forward(0.3 - eps, -0.9)[0]) / (2 * eps),
                          (leg.forward(0.3, -0.9 + eps)[0] - leg.forward(0.3, -0.9 - eps)[0]) / (2 * eps)])
    print("jacobian error", np.abs(J - Jn).max())
