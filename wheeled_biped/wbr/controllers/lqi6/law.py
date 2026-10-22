"""LQI6 balance law plugged into the DIABLO-style MCU (Plan 2).

Converts the MCU estimate into the plant6 state convention of the design report:
  theta = world virtual-leg angle (positive: hip ahead of the wheel)  -> est.theta_v_mean
  phi   = body pitch with the COM behind the hip for phi > 0          -> -est.pitch
  v     = wheel ground speed                                           -> est.v
and returns the total wheel torque T and total hip torque Tp (u = -K xs).
"""
from __future__ import annotations

import numpy as np

from wbr.controllers.lqi6.lqi6 import LQI6
from wbr.controllers.lqi6.plant6 import Plant6


class LQI6Law:
    def __init__(self, plant, Ts: float = 1e-3, theta_eq=None, Tp_ff: float = 0.0, **lqi_kwargs):
        # theta_eq: callable L0 -> equilibrium leg angle of the real robot (body level, total COM
        # above the contact point); zero for the symmetric plant of the report.
        # Tp_ff: constant static hip torque (plant convention) holding the body level when its COM is
        # not above the hip (Plan 4: m_b g b_x); a constant does not change the linearised plant.
        if lqi_kwargs.get("q_att") is not None and lqi_kwargs.get("att_gM") is None:
            lqi_kwargs = dict(lqi_kwargs, att_gM=self.static_gM(plant, theta_eq or (lambda L0: 0.0)))
        self.ctrl = LQI6(plant, Ts=Ts, **lqi_kwargs)
        self.ctrl.design(plant.L0)
        self.theta_eq = theta_eq or (lambda L0: 0.0)
        self.Tp_ff = Tp_ff

    @staticmethod
    def static_gM(plant, theta_eq):
        """L0 -> g dSum(m x)/dtheta at the level-body equilibrium of the serial-leg plant (legs + body, wheels at
        the axle): the leg-angle stiffness of the COM used by the static body-tilt estimate of LQI6 (q_att)."""
        p = plant.p

        def gM(L0: float) -> float:
            leg = p.leg.at(L0)
            th = theta_eq(L0)
            return p.g * (leg["m_l"] * (leg["c"] * np.cos(th) - leg["d"] * np.sin(th)) + p.m_b * L0 * np.cos(th))
        return gM

    def reset(self) -> None:
        self.ctrl.reset()

    def set_reference_shaping(self, on: bool) -> None:
        self.ctrl.set_reference_shaping(on)

    def compute(self, est, refs: dict, mcu) -> dict:
        if not refs.get("balance", True):      # CAR / transitions / FAULT: freeze the integrator and ramp
            self.ctrl.reset()
            return {"T": 0.0, "Tp": 0.0}
        phi = -(est.pitch - refs.get("pitch_ref", 0.0))
        phi_dot = -est.pitch_rate
        theta = est.theta_v_mean - self.theta_eq(est.L0_mean)
        T, Tp = self.ctrl.step(theta, est.theta_v_dot_mean, est.v, phi, phi_dot, est.L0_mean,
                               refs["v_ref"] if refs.get("balance", True) else 0.0)
        return {"T": T, "Tp": Tp + self.Tp_ff, "Tp_ff": self.Tp_ff, "K_T": self.ctrl.K[0].tolist(), "K_Tp": self.ctrl.K[1].tolist()}
