"""Unit tests of the ROS-free core that need no MuJoCo (run: python -m unittest discover -s tests -v).

2026-11-11: Plans 1-3 were removed; their tests (five-bar leg, Plan 1-3 smoke runs, the Plan 1 crouch, the benchmark's
settle_time) went with them, the LQI8 tests now run on the v5 robot's plant, and the two MCU behaviour tests that used
the Plan 2 robot (transform-down refusal, motors off after a host-link loss) moved to tests/test_plan4.py.
"""
from __future__ import annotations

import functools
import math
import unittest

import numpy as np

from wbr import MODELS_DIR
from wbr.common.rotations import pitch_frame_in_parent, quat_to_matrix, matrix_to_quat, rot_y, roll_pitch_yaw
from wbr.common.servo import ServoCommand, ServoModel, ServoProperties, torque_command
from wbr.common.filters import Ramp, DelayLine, low_pass_filter
from wbr.controllers.diablo_arch.host_link import HostEmulator, HostLink, HostLinkParams, MotionCtrl
from wbr.controllers.lqi8.lqi8 import LQI8, c2d_zoh
from wbr.controllers.lqi8.plant_serial import SerialPlant, SerialPlantParams


@functools.lru_cache(maxsize=1)
def v5_plant() -> SerialPlant:
    """The v5 robot's three-body plant (models/wheelbiped_v5, written by wbr.models.build_v5)."""
    return SerialPlant(SerialPlantParams.from_json(MODELS_DIR / "wheelbiped_v5" / "plant_params.json"))


def v5_table(plant: SerialPlant) -> tuple:
    grid = plant.p.leg.L0
    return float(grid[0]), float(grid[-1]), float(grid[1] - grid[0])


class RotationTests(unittest.TestCase):
    def test_pitch_convention_positive_when_leaning_forward(self):
        R = rot_y(0.2)  # base x-axis tilted down -> leaning forward
        self.assertAlmostEqual(pitch_frame_in_parent(R), 0.2, places=9)
        self.assertAlmostEqual(pitch_frame_in_parent(rot_y(-0.3)), -0.3, places=9)
        self.assertAlmostEqual(roll_pitch_yaw(rot_y(0.25))[1], 0.25, places=9)

    def test_quat_roundtrip(self):
        q = np.array([0.9, 0.1, -0.3, 0.2]); q /= np.linalg.norm(q)
        q2 = matrix_to_quat(quat_to_matrix(q))
        self.assertTrue(np.allclose(q, q2, atol=1e-9) or np.allclose(q, -q2, atol=1e-9))


class ServoTests(unittest.TestCase):
    def test_moteus_like_pd_and_clamp(self):
        m = ServoModel({"j": ServoProperties(kp=20.0, kd=1.0, max_torque=16.0)})
        tau = m.torque("j", ServoCommand(position=0.1, velocity=0.0, kp_scale=2.0, kd_scale=1.0, maximum_torque=16.0), 0.0, 0.5)
        self.assertAlmostEqual(tau, 20 * 2 * 0.1 - 0.5)
        tau = m.torque("j", ServoCommand(position=math.nan, velocity=3.0, maximum_torque=1.7), 0.0, 0.0)
        self.assertAlmostEqual(tau, 1.7)  # velocity mode, clamped
        self.assertAlmostEqual(m.torque("j", torque_command(-4.0, 12.0), 1.0, 1.0), -4.0)


class FilterTests(unittest.TestCase):
    def test_ramp_and_delay(self):
        r = Ramp(1.0, max_abs=0.7)
        for _ in range(500):
            v = r.step(2.0, 1e-3)
        self.assertAlmostEqual(v, 0.5, places=6)
        for _ in range(1000):
            v = r.step(2.0, 1e-3)
        self.assertAlmostEqual(v, 0.7, places=6)
        d = DelayLine(3, 0.0)
        out = [d.push(i) for i in range(6)]
        self.assertEqual(out, [0.0, 0.0, 0.0, 0, 1, 2])
        self.assertAlmostEqual(low_pass_filter(0.0, 1.0, 1.0, 0.001), 0.001)


class LQI8Tests(unittest.TestCase):
    """The 6-state form (q_att = q_pos = None) on the v5 plant; the 8-state weights the robot runs are checked with
    MuJoCo in tests/test_plan4.py and tests/test_robot_v5.py."""

    def test_closed_loop_stable_for_all_leg_lengths(self):
        plant = v5_plant()
        table = v5_table(plant)
        ctrl = LQI8(plant, table_L0=table)
        for L0 in (table[0], plant.L0, table[1]):
            A, B = plant.linearize(L0)
            keep = [0, 1, 3, 4, 5]
            Aa = np.zeros((6, 6)); Aa[:5, :5] = A[np.ix_(keep, keep)]; Aa[5, 2] = 1.0
            Ba = np.zeros((6, 2)); Ba[:5, :] = B[keep, :]
            Ad, Bd = c2d_zoh(Aa, Ba, 1e-3)
            K = ctrl.design(L0)
            eig = np.linalg.eigvals(Ad - Bd @ K)
            self.assertLess(np.max(np.abs(eig)), 1.0, f"unstable closed loop at L0={L0}")
            # open loop must be unstable (two positive real poles: body and leg)
            self.assertGreaterEqual(int(np.sum(np.real(np.linalg.eigvals(A)) > 1.0)), 2)

    def test_ramp_and_integrator(self):
        plant = v5_plant()
        ctrl = LQI8(plant, table_L0=v5_table(plant))
        T, Tp = ctrl.step(0.0, 0.0, 0.0, 0.0, 0.0, plant.L0, 0.0)
        self.assertAlmostEqual(T, 0.0); self.assertAlmostEqual(Tp, 0.0)
        for _ in range(100):
            ctrl.step(0.0, 0.0, 0.0, 0.0, 0.0, plant.L0, 1.0)
        self.assertAlmostEqual(ctrl.v_ramp, 0.1, places=9)  # 1 m/s^2 ramp for 0.1 s

    def test_gain_table_matches_direct_design(self):
        plant = v5_plant()
        table = v5_table(plant)
        ctrl = LQI8(plant, table_L0=table)
        direct = LQI8(plant, table_L0=None)
        grid = plant.p.leg.L0
        mids = 0.5 * (grid[:-1] + grid[1:])
        for L0 in mids[:: max(1, len(mids) // 5)]:              # grid mid-points (interpolation error)
            K_table = ctrl.gains(float(L0))
            K_direct = direct.design(float(L0))
            self.assertLess(np.max(np.abs(K_table - K_direct)) / np.max(np.abs(K_direct)), 0.02)


class HostLinkTests(unittest.TestCase):
    def test_timeout_releases_control_and_heartbeat_keeps_it(self):
        link = HostLink(HostLinkParams(timeout_ms=500, heartbeat_ms=180))
        host = HostEmulator(link, publish_hz=50)
        host.set_command(forward=0.7)
        t = 0.0
        for _ in range(1000):   # 1 s with the host alive
            host.tick(t)
            cmd = link.poll(t)
            t += 1e-3
        self.assertTrue(link.in_control)
        self.assertAlmostEqual(cmd.forward, 0.7)
        host.enabled = False     # host dies
        for _ in range(400):
            cmd = link.poll(t); t += 1e-3
        self.assertTrue(link.in_control)          # still within the 500 ms timeout
        for _ in range(200):
            cmd = link.poll(t); t += 1e-3
        self.assertFalse(link.in_control)         # released
        self.assertAlmostEqual(cmd.forward, 0.0)  # disconnect program: zero velocity

    def test_mode_mark_message_keeps_fields(self):
        msg = MotionCtrl(mode_mark=True)
        msg.mode.stand_mode = True
        self.assertTrue(msg.mode.stand_mode)
        self.assertEqual(msg.up, 0.0)   # same defaults as motion_msgs/MotionCtrl

    def test_received_packet_is_a_copy(self):
        link = HostLink(HostLinkParams())
        host = HostEmulator(link, publish_hz=50)
        host.set_command(forward=0.5)
        host.tick(0.0)
        host.set_command(forward=0.0)      # edited after the send: must not leak into the MCU
        self.assertAlmostEqual(link.poll(0.001).forward, 0.5)


if __name__ == "__main__":
    unittest.main()
