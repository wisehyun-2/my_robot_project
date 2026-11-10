"""v5_mpc: the MPC balance law of the v5 robot (2026-11-10).

The MPC must (1) give the LQR feedback of the control period when no constraint is active (its first move against the
LQI8 of v5_lqi with the same weights), (2) keep the motor-torque constraints when they are active, (3) start the
reference t_lead after the command and preview it, (4) sit in the MCU of the v5 closed loop instead of the LQI8.
(1)-(3) need no simulator; (4) needs MuJoCo and is skipped when its DLL cannot be loaded.
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

WB = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WB))

from wbr.controllers.lqi8.lqi8 import LQI8  # noqa: E402
from wbr.controllers.lqi8.plant_serial import SerialPlant, SerialPlantParams  # noqa: E402
from v5_mpc.mpc_law import BalanceMPC, static_gM  # noqa: E402

try:
    import mujoco  # noqa: F401
    HAVE_MUJOCO = True
except Exception:        # Windows application control blocks the DLL now and then (memory: windows-env-constraints)
    HAVE_MUJOCO = False


def _setup():
    p = SerialPlantParams.from_json(str(WB / "models" / "wheelbiped_v5" / "plant_params.json"))
    plant = SerialPlant(p)
    m = p.extra["total_mass"]
    W = dict(q_theta=0.3, q_thetad=3.0, q_v=0.5, q_phi=0.15, q_phid=3.0, q_int=0.2, q_att=0.2, q_pos=0.5,
             r_wheel_frac=0.116 * m * p.g * p.R / p.tau_wheel_max, r_hip_frac=0.296 * m * p.g * 0.31 / p.tau_hip_max)
    g = p.leg.L0
    table = (float(g[0]), float(g[-1]), float(g[1] - g[0]))
    return p, plant, W, table, static_gM(plant, plant.theta_eq)


class TestBalanceMPC(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.p, cls.plant, cls.W, cls.table, gM = _setup()
        cls.gM = staticmethod(gM)          # a plain function, not a bound method, through self
        cls.mpc = BalanceMPC(cls.plant, 1e-3, table_L0=cls.table, att_gM=gM, **cls.W)

    def test_first_move_is_the_lqr_of_the_control_period(self):
        K_lqr = LQI8(self.plant, Ts=1e-3, table_L0=self.table, att_gM=self.gM, **self.W).gains(self.p.L0)
        K_mpc = self.mpc.first_move_gain(self.p.L0)
        ratio = K_mpc / K_lqr
        # wheel row: within 1 %; hip row: within 17 % (the horizon's coarse 10 ms blocks after the first 8 ms)
        np.testing.assert_allclose(ratio[0], 1.0, atol=0.01)
        np.testing.assert_allclose(ratio[1], 1.0, atol=0.17)

    def test_unconstrained_step_is_the_explicit_solution(self):
        m = self.mpc
        m.reset()
        e = np.array([0.02, 0.1, 0.05, -0.01, 0.2])
        T, Tp = m.step(*e[:5], self.p.L0, 0.0)
        i = int(np.argmin(np.abs(m.grid - self.p.L0)))
        e_full = np.array(list(e) + [m.z, m.z_att, m.z_pos])
        u = -(m.Kx_t[i, :2] @ e_full)            # reference zero over the horizon: no preview terms
        if abs(m.grid[i] - self.p.L0) < 1e-9:
            np.testing.assert_allclose([T, Tp], u, rtol=1e-9, atol=1e-9)
        self.assertFalse(m.constrained)

    def test_constraints_hold(self):
        m = self.mpc
        m.reset()
        G = np.array([[1.0, 0.0], [-1.0, 0.0], [0.0, 1.0], [0.0, -1.0]])
        h = np.array([4.0, 4.0, 6.0, 6.0])
        y = (0.0, 0.8, 0.3, 0.0, 0.5)
        T0, Tp0 = m.step(*y, self.p.L0, 0.0)            # unconstrained: beyond the box
        self.assertGreater(abs(T0), 4.0)
        m.reset()
        T, Tp = m.step(*y, self.p.L0, 0.0, limits=(G, h))
        self.assertTrue(m.constrained)
        self.assertLessEqual(abs(T), 4.0 + 1e-6)
        self.assertLessEqual(abs(Tp), 6.0 + 1e-6)
        self.assertEqual(m.qp_failures, 0)

    def test_reference_starts_after_the_lead(self):
        m = BalanceMPC(self.plant, 1e-3, table_L0=self.table, att_gM=self.gM, t_lead=0.15, v_ref_tau=0.0, **self.W)
        v = []
        for k in range(400):
            m.step(0.0, 0.0, 0.0, 0.0, 0.0, self.p.L0, 0.5)
            v.append(m.v_filt)
        v = np.array(v)
        self.assertEqual(v[:150].max(), 0.0)                       # nothing before t_lead
        self.assertAlmostEqual(v[150 + 100], min(0.5, 2.0 * 0.101), places=3)   # then the 2 m/s^2 ramp
        self.assertAlmostEqual(v[-1], 0.5, places=6)


@unittest.skipUnless(HAVE_MUJOCO, "MuJoCo DLL not loadable on this machine right now")
class TestPlanV5Mpc(unittest.TestCase):
    def test_mpc_replaces_the_lqi8_and_stands(self):
        from v5_mpc.controller import make_plan
        from v5_mpc.mpc_law import MpcLaw
        plan = make_plan(noise=True, seed=0, latency_ms=10.0)
        self.assertIsInstance(plan.mcu.law, MpcLaw)
        plan.reset()
        self.assertTrue(plan.run(3.0))
        self.assertLess(np.degrees(np.abs(plan.log["pitch_true"][plan.log["t"] > 1.0]).max()), 1.0)

    def test_no_lqi_settings_in_the_mpc_folder(self):
        from v5_mpc.controller import PlanV5Mpc, PlanV5MpcConfig
        with self.assertRaises(ValueError):
            PlanV5Mpc(PlanV5MpcConfig.from_model(lqi_kwargs=dict(q_att=None)))


if __name__ == "__main__":
    unittest.main()
