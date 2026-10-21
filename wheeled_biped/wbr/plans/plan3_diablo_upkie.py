"""Plan 3 -- DIABLO architecture (6 kg DIABLO-lite) + upkie's control code (wheel balancer PI or MPC,
upkie observers, velocity-mode wheels and position-mode legs) inside the DIABLO MCU structure."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from wbr.controllers.diablo_arch.upkie_law import UpkieLaw
from wbr.plans.plan2_diablo_lqi6 import Plan2Config, Plan2DiabloLQI6


@dataclass
class Plan3Config(Plan2Config):
    balancer: str = "pid"       # upkie WheelBalancer ("pid") or MPC ("mpc")
    gain_scale: float = 1.0     # multiplier on the four PI gains of the wheel balancer
    leg_gain_scale: float = 2.0
    use_sim_contact: bool = False
    mpc_leg_length: float = 0.30
    gravity_feedforward: bool = True   # static leg load as servo feedforward torque (upkie servo API field)


class Plan3DiabloUpkie(Plan2DiabloLQI6):
    def __init__(self, cfg: Plan3Config | None = None):
        cfg = cfg or Plan3Config()
        self._cfg3 = cfg
        super().__init__(cfg, law=_Pending())
        # the law needs the leg geometry and the equilibrium table computed by the base class
        p = self.params
        # the MPC does not drive without a contact flag and the apparent-inertia detector needs wheel
        # torque to trigger: ground-truth contact in simulation for the MPC variant (see report)
        use_sim_contact = cfg.use_sim_contact or cfg.balancer == "mpc"
        self.law = UpkieLaw(self.leg, self.dt, p.wheel_radius, 2 * p.wheel_y, p.tau_wheel, p.tau_hip, self.plant.L0,
                            theta_eq=self.theta_eq, balancer=cfg.balancer, gain_scale=cfg.gain_scale,
                            leg_gain_scale=cfg.leg_gain_scale, mpc_leg_length=cfg.mpc_leg_length,
                            use_sim_contact=use_sim_contact,
                            gravity_feedforward_N=(0.5 * (p.total_mass - 2 * p.m_wheel) * 9.81) if cfg.gravity_feedforward else 0.0)
        self.mcu.law = self.law
        self.reset()


class _Pending:
    """Placeholder law used while the base class initialises (replaced right after)."""

    def reset(self):
        pass

    def compute(self, est, refs, mcu):
        return {"T": 0.0, "Tp": 0.0}
