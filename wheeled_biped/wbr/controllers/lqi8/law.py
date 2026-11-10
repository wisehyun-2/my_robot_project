"""LQI8 balance law plugged into the DIABLO-style MCU (Plan 4 harness, v5 robot).

Converts the MCU estimate into the plant state convention of the design report (plant_serial):
  theta = world virtual-leg angle (positive: hip ahead of the wheel)  -> est.theta_v_mean
  phi   = body pitch with the COM behind the hip for phi > 0          -> -est.pitch
  v     = wheel ground speed                                           -> est.v
and returns the total wheel torque T and total hip torque Tp (u = -K xs).
"""
from __future__ import annotations

import numpy as np

from wbr.controllers.lqi8.lqi8 import LQI8


class LQI8Law:
    def __init__(self, plant, Ts: float = 1e-3, theta_eq=None, Tp_ff: float = 0.0, **lqi_kwargs):
        # theta_eq: callable L0 -> equilibrium leg angle of the real robot (body level, total COM
        # above the contact point); zero for the symmetric plant of the report.
        # Tp_ff: constant static hip torque (plant convention) holding the body level when its COM is
        # not above the hip (Plan 4: m_b g b_x); a constant does not change the linearised plant.
        if lqi_kwargs.get("q_att") is not None and lqi_kwargs.get("att_gM") is None:
            lqi_kwargs = dict(lqi_kwargs, att_gM=self.static_gM(plant, theta_eq or (lambda L0: 0.0)))
        self.ctrl = LQI8(plant, Ts=Ts, **lqi_kwargs)
        self.ctrl.design(plant.L0)
        self.theta_eq = theta_eq or (lambda L0: 0.0)
        self.Tp_ff = Tp_ff

    @staticmethod
    def static_gM(plant, theta_eq):
        """L0 -> g dSum(m x)/dtheta at the level-body equilibrium of the serial-leg plant (legs + body, wheels at
        the axle): the leg-angle stiffness of the COM used by the static body-tilt estimate of LQI8 (q_att)."""
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
        if self.ctrl.shaping_on:                # slowing down over uneven ground (McuParams.uneven_brake, uneven_lean)
            brake = refs.get("v_brake", 0.0)
            self.ctrl.v_ramp_rate = brake if brake > 0.0 else self.ctrl._shaped[0]
            self.ctrl.v_ref_tau_brake = refs.get("v_brake_tau", 0.0)
            theta -= refs.get("v_lean", 0.0) * self.ctrl._a_ref_lp / 9.81
        self.ctrl.hold_integrators = bool(refs.get("hold_integrators", False))
        T, Tp = self.ctrl.step(theta, est.theta_v_dot_mean, est.v, phi, phi_dot, est.L0_mean,
                               refs["v_ref"] if refs.get("balance", True) else 0.0)
        return {"T": T, "Tp": Tp + self.Tp_ff, "Tp_ff": self.Tp_ff, "K_T": self.ctrl.K[0].tolist(), "K_Tp": self.ctrl.K[1].tolist()}
