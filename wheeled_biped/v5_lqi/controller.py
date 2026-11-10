"""The v5 robot with the LQI8 balance law: the controller of this folder, and nothing else.

    MCU (1 kHz):  estimator -> LQI8 (T, Tp) -> height / yaw / roll / split loops -> serial-leg VMC -> joint torques
    host:         plain MotionCtrl set-points (wbr.controllers.diablo_arch.host_link.HostEmulator), no reference generator

Where the code is (shared with Plans 2 / 4, so it stays in wbr):
    wbr/controllers/lqi8/lqi8.py    LQI8: discrete LQR on [theta, theta_dot, v - v_ref, phi, phi_dot, z1 (, z2, z3)],
                                    gain table on the leg-length grid, velocity ramp + pre-filter, integrator freeze
    wbr/controllers/lqi8/law.py     the MCU adapter (estimate -> plant states, + static hip torque)
    wbr/plans/plan_v5.py            the v5 closed loop (its URDF, IMU chain, gains); Plan4LQI8._make_law builds the LQI8 (default)

    python -c "from v5_lqi.controller import make_plan; p = make_plan(); p.reset(); print(p.run(2.0))"     (from wheeled_biped/)
"""
from __future__ import annotations

from wbr.bench.stage_tests import Stage
from wbr.plans.plan_v5 import PlanV5, PlanV5Config

NAME = "LQI8"

# the stages of v5_lqi/TEST (lqi_kwargs of Plan4Config; q = Bryson weight, None = that integral state off)
STAGES = (
    Stage("lqr", "순수 LQR (6상태)", dict(lqi_kwargs=dict(q_att=None, q_pos=None))),
    Stage("lqr_ipos", "LQR + ∫위치", dict(lqi_kwargs=dict(q_att=None, q_pos=0.5))),
    Stage("lqr_ipos_itilt", "LQR + ∫위치 + ∫기울기", dict(lqi_kwargs=dict(q_att=0.2, q_pos=0.5))),
)
FULL_STAGE = "lqr_ipos_itilt"       # the controller in use (user 11-07: no pure LQR)


def make_plan(stage: str = FULL_STAGE, **cfg) -> PlanV5:
    """The committed v5 model (models/wheelbiped_v5, gains from description.json) with the LQI8 of `stage`.
    cfg: further PlanV5Config fields (noise, seed, latency_ms, floor_humps, ...)."""
    st = {s.key: s for s in STAGES}[stage]
    if cfg.get("host", "ramp") != "ramp":
        raise ValueError("v5_lqi: the LQI8 is the only controller here (no MPC host)")
    return PlanV5(PlanV5Config.from_model(**st.overrides, **cfg))
