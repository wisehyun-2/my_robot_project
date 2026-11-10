"""The v5 robot with the MPC balance law: the controller of this folder, and nothing else (no LQI8, no MPC host).

    MCU (1 kHz):  estimator -> MPC (T, Tp) -> height / yaw / roll / split loops -> serial-leg VMC -> joint torques
    host:         plain MotionCtrl set-points (HostEmulator), no reference generator

    v5_mpc/mpc_law.py      BalanceMPC (prediction model, preview, motor-torque constraints, QP) + MpcLaw (MCU adapter)
    v5_mpc/controller.py   PlanV5Mpc: the v5 closed loop of wbr.plans.plan_v5 with the MpcLaw in the MCU's balance slot

The robot model, the simulator, the IMU chain, the MCU and its four other loops are the same objects as in v5_lqi.

    python -c "from v5_mpc.controller import make_plan; p = make_plan(); p.reset(); print(p.run(2.0))"     (from wheeled_biped/)
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from wbr.bench.stage_tests import Stage
from wbr.plans.plan_v5 import PlanV5, PlanV5Config

from v5_mpc.mpc_law import MpcLaw

NAME = "MPC"

# MPC weights and settings; the q_* / r_* rules are the v5 LQI8's (v5_lqi), so the two differ in structure only
MPC_DEFAULTS = dict(
    dt_mpc=0.01, n_steps=30,        # 0.3 s horizon + DARE terminal cost
    n_con=10,                       # motor-torque constraints on the first 0.1 s of moves
    u_limit=0.75,                   # of each motor's peak (AKE60-8: 9.4 of 12.5 N m; the tests judge 10)
    q_theta=0.3, q_thetad=3.0, q_v=0.5, q_phi=0.15, q_phid=3.0, q_int=0.2,
    q_att=0.2, q_pos=0.5,           # the full controller: z2 static tilt, z3 position integral (None = off)
    v_ramp_rate=2.0,                # m/s^2 reference ramp (as the LQI8) ...
    t_lead=0.15, v_ref_tau=0.1,     # ... starting 0.15 s after the command, 0.1 s filter, previewed over the horizon
    freeze_z_on_ramp=True,          # z1 held while the reference moves (the LQI8's rule)
)


@dataclass
class PlanV5MpcConfig(PlanV5Config):
    mpc_kwargs: Optional[dict] = None      # overrides of MPC_DEFAULTS (BalanceMPC / MpcLaw arguments)


class PlanV5Mpc(PlanV5):
    """PlanV5 with the MPC in place of the LQI8 (Plan4LQI8._make_law)."""

    @staticmethod
    def default_config():
        return PlanV5MpcConfig.from_model()

    def _make_law(self, lqi_kwargs: dict):
        c = self.cfg
        if c.lqi_kwargs:
            raise ValueError("v5_mpc has no LQI8: use mpc_kwargs")
        if c.host != "ramp":
            raise ValueError("v5_mpc: the MPC in the MCU is the only controller (no MPC host)")
        design = self._lqi_design()        # torque weights from the robot's weight and size + the plant's leg-length grid (same rule as v5_lqi)
        kw = dict(MPC_DEFAULTS, r_wheel_frac=design["r_wheel_frac"], r_hip_frac=design["r_hip_frac"], table_L0=design["table_L0"])
        kw.update(getattr(c, "mpc_kwargs", None) or {})
        return MpcLaw(self.plant, Ts=self.dt, theta_eq=self.plant.theta_eq, Tp_ff=self.plant.Tp_eq(), **kw)


def _classes():
    return PlanV5MpcConfig, PlanV5Mpc


# the stages of v5_mpc/TEST: the same states as v5_lqi's stages, here as the MPC's prediction model
STAGES = (
    Stage("mpc", "MPC (6상태)", dict(mpc_kwargs=dict(q_att=None, q_pos=None)), _classes),
    Stage("mpc_ipos", "MPC + ∫위치", dict(mpc_kwargs=dict(q_att=None, q_pos=0.5)), _classes),
    Stage("mpc_ipos_itilt", "MPC + ∫위치 + ∫기울기", dict(mpc_kwargs=dict(q_att=0.2, q_pos=0.5)), _classes),
)
FULL_STAGE = "mpc_ipos_itilt"


def make_plan(stage: str = FULL_STAGE, **cfg) -> PlanV5Mpc:
    """The committed v5 model (models/wheelbiped_v5) with the MPC of `stage`; cfg: further PlanV5MpcConfig fields."""
    st = {s.key: s for s in STAGES}[stage]
    kw = dict(st.overrides)
    if "mpc_kwargs" in cfg:
        kw["mpc_kwargs"] = dict(kw["mpc_kwargs"], **cfg.pop("mpc_kwargs"))
    return PlanV5Mpc(PlanV5MpcConfig.from_model(**kw, **cfg))
