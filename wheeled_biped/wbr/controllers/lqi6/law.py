"""LQI6 balance law plugged into the DIABLO-style MCU (Plan 2).

Converts the MCU estimate into the plant6 state convention of the design report:
  theta = world virtual-leg angle (positive: hip ahead of the wheel)  -> est.theta_v_mean
  phi   = body pitch with the COM behind the hip for phi > 0          -> -est.pitch
  v     = wheel ground speed                                           -> est.v
and returns the total wheel torque T and total hip torque Tp (u = -K xs).
"""
from __future__ import annotations

from wbr.controllers.lqi6.lqi6 import LQI6
from wbr.controllers.lqi6.plant6 import Plant6


class LQI6Law:
    def __init__(self, plant: Plant6, Ts: float = 1e-3, theta_eq=None, **lqi_kwargs):
        # theta_eq: callable L0 -> equilibrium leg angle of the real robot (body level, total COM
        # above the contact point); zero for the symmetric plant of the report.
        self.ctrl = LQI6(plant, Ts=Ts, **lqi_kwargs)
        self.ctrl.design(plant.L0)
        self.theta_eq = theta_eq or (lambda L0: 0.0)

    def reset(self) -> None:
        self.ctrl.reset()

    def compute(self, est, refs: dict, mcu) -> dict:
        if not refs.get("balance", True):      # CAR / transitions / FAULT: freeze the integrator and ramp
            self.ctrl.reset()
            return {"T": 0.0, "Tp": 0.0}
        phi = -(est.pitch - refs.get("pitch_ref", 0.0))
        phi_dot = -est.pitch_rate
        theta = est.theta_v_mean - self.theta_eq(est.L0_mean)
        T, Tp = self.ctrl.step(theta, est.theta_v_dot_mean, est.v, phi, phi_dot, est.L0_mean,
                               refs["v_ref"] if refs.get("balance", True) else 0.0)
        return {"T": T, "Tp": Tp, "K_T": self.ctrl.K[0].tolist(), "K_Tp": self.ctrl.K[1].tolist()}
