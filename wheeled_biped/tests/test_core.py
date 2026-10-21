"""Unit tests of the ROS-free core (run: python -m unittest discover -s tests -v)."""
from __future__ import annotations

import math
import unittest

import numpy as np

from wbr.common.rotations import pitch_frame_in_parent, quat_to_matrix, matrix_to_quat, rot_y, roll_pitch_yaw
from wbr.common.servo import ServoCommand, ServoModel, ServoProperties, torque_command
from wbr.common.filters import Ramp, DelayLine, low_pass_filter
from wbr.controllers.diablo_arch.host_link import HostEmulator, HostLink, HostLinkParams, MotionCtrl
from wbr.controllers.lqi6.fivebar import FiveBarLeg
from wbr.controllers.lqi6.lqi6 import LQI6, c2d_zoh
from wbr.controllers.lqi6.plant6 import Plant6, diablo_lite_params


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


class FiveBarTests(unittest.TestCase):
    def setUp(self):
        self.leg = FiveBarLeg()

    def test_nominal_pose(self):
        p, L0, th = self.leg.forward(0.0, 0.0)
        self.assertAlmostEqual(L0, 0.14 * math.sqrt(2), places=9)
        self.assertAlmostEqual(th, 0.0, places=9)

    def test_inverse_matches_forward(self):
        for L0, th in ((0.15, 0.1), (0.25, -0.2), (0.198, 0.0), (0.13, 0.3)):
            qh, qc = self.leg.inverse(L0, th)
            _, L0b, thb = self.leg.forward(qh, qc)
            self.assertAlmostEqual(L0, L0b, places=7)
            self.assertAlmostEqual(th, thb, places=7)

    def test_jacobian_finite_difference(self):
        q = (0.17, -0.22)
        J = self.leg.jacobian(*q)
        eps = 1e-6
        for i in range(2):
            dq = np.zeros(2); dq[i] = eps
            num = (self.leg.forward(*(np.array(q) + dq))[0] - self.leg.forward(*(np.array(q) - dq))[0]) / (2 * eps)
            self.assertTrue(np.allclose(J[:, i], num, atol=1e-6))

    def test_vmc_static_support(self):
        # pushing the axle straight down at the nominal pose: hip and crank torques are equal and opposite
        th, tc = self.leg.vmc_torques(30.0, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(th, -tc, places=9)
        self.assertGreater(th, 0.0)
        # a hip torque Tp alone produces equal torques on both motors (sum = Tp)
        th, tc = self.leg.vmc_torques(0.0, 2.0, 0.0, 0.0)
        self.assertAlmostEqual(th + tc, 2.0, places=6)


class LQI6Tests(unittest.TestCase):
    def test_closed_loop_stable_for_all_leg_lengths(self):
        plant = Plant6(**diablo_lite_params())
        ctrl = LQI6(plant)
        for L0 in (0.13, 0.198, 0.26):
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
        plant = Plant6(**diablo_lite_params())
        ctrl = LQI6(plant)
        T, Tp = ctrl.step(0.0, 0.0, 0.0, 0.0, 0.0, plant.L0, 0.0)
        self.assertAlmostEqual(T, 0.0); self.assertAlmostEqual(Tp, 0.0)
        for _ in range(100):
            ctrl.step(0.0, 0.0, 0.0, 0.0, 0.0, plant.L0, 1.0)
        self.assertAlmostEqual(ctrl.v_ramp, 0.1, places=9)  # 1 m/s^2 ramp for 0.1 s


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

    def test_gain_table_matches_direct_design(self):
        plant = Plant6(**diablo_lite_params())
        ctrl = LQI6(plant)
        direct = LQI6(plant, table_L0=None)
        for L0 in (0.105, 0.145, 0.205, 0.255, 0.275):      # grid mid-points (interpolation error)
            K_table = ctrl.gains(L0)
            K_direct = direct.design(L0)
            self.assertLess(np.max(np.abs(K_table - K_direct)) / np.max(np.abs(K_direct)), 0.02)


class McuSafetyTests(unittest.TestCase):
    def test_transform_down_refused_and_fault_cuts_all_torques(self):
        from wbr.plans.plan2_diablo_lqi6 import Plan2DiabloLQI6
        from wbr.controllers.diablo_arch.host_link import RobotMode
        plan = Plan2DiabloLQI6()
        plan.run(0.5)
        plan.host.set_command(mode_mark=True, stand_mode=False)   # DIABLO teleop 'v'/'b' keys before 'z'
        plan.run(0.3)
        self.assertEqual(plan.mcu.mode, RobotMode.STAND)
        self.assertTrue(any("transform down ignored" in e for _, e in plan.mcu.events))
        plan.host.set_command(mode_mark=False)
        plan.reset(pitch=math.radians(40))      # beyond the 34 deg FAULT threshold
        plan.step()
        self.assertTrue(plan.mcu.fault)
        cmds = plan.mcu.tick(plan.observation)
        self.assertTrue(all(c.feedforward_torque == 0.0 and c.maximum_torque == 0.0 for c in cmds.values()))


class PlanSmokeTests(unittest.TestCase):
    def test_plan1_plan2_plan3_stand_two_seconds(self):
        from wbr.plans.plan1_upkie import Plan1Upkie
        from wbr.plans.plan2_diablo_lqi6 import Plan2DiabloLQI6
        from wbr.plans.plan3_diablo_upkie import Plan3DiabloUpkie
        for cls in (Plan1Upkie, Plan2DiabloLQI6, Plan3DiabloUpkie):
            plan = cls()
            self.assertTrue(plan.run(2.0), f"{cls.__name__} fell while standing")
            self.assertLess(abs(plan.log["pitch"][-1]), math.radians(5))



class ReviewPass2Tests(unittest.TestCase):
    def test_arm_refused_before_measurement_and_when_tilted(self):
        from wbr.plans.plan2_diablo_lqi6 import Plan2DiabloLQI6
        plan = Plan2DiabloLQI6()
        plan.reset(pitch=math.radians(10))
        plan.mcu.reset(armed=False)
        self.assertFalse(plan.mcu.arm())            # no measurement yet (review N1)
        plan.step()                                 # one tick -> estimate valid, robot tilted 10 deg
        self.assertFalse(plan.mcu.arm())
        plan.reset(pitch=0.0)
        plan.mcu.reset(armed=False)
        plan.step()
        self.assertTrue(plan.mcu.arm())             # upright and measured

    def test_disconnect_off_only_after_a_connection_and_latched(self):
        from wbr.plans.plan2_diablo_lqi6 import Plan2DiabloLQI6
        plan = Plan2DiabloLQI6()
        plan.mcu.p.disconnect_off_s = 1.0
        plan.host.enabled = False                   # never connected: must not switch off at boot (review N2)
        plan.mcu.reset()
        plan.link.reset()
        plan.run(2.0)
        self.assertTrue(plan.mcu.armed)
        plan.host.enabled = True
        plan.run(0.5)                               # connects
        plan.host.enabled = False
        plan.run(2.0)                               # 0.5 s timeout + 1.0 s -> motors off, latched
        self.assertTrue(plan.mcu.off_latched)
        self.assertFalse(plan.mcu.armed)
        n_events = len(plan.mcu.events)
        plan.run(0.5)
        self.assertEqual(len(plan.mcu.events), n_events)   # no event flood while latched

    def test_settle_time_reports_unsettled_tail(self):
        from wbr.bench.battery import settle_time
        t = np.arange(0, 8.0, 1e-3)
        L = {"t": t, "pitch": np.full_like(t, math.radians(1.5)), "vx": np.zeros_like(t)}
        self.assertIsNone(settle_time(L, 0.0))      # never within 1 deg (review N4)
        L["pitch"] = np.where(t < 2.0, math.radians(5), 0.0)
        self.assertAlmostEqual(settle_time(L, 0.0), 2.0, delta=0.03)

    def test_plan1_survives_4cm_crouch(self):
        from wbr.plans.plan1_upkie import Plan1Upkie
        plan = Plan1Upkie()
        plan.run(1.0)
        plan.set_height_offset(-0.04)
        self.assertTrue(plan.run(4.0))              # alive while crouching (review N3: smoothed IMU zero)
        self.assertLess(abs(plan.log["pitch"][-1]), math.radians(5))


if __name__ == "__main__":
    unittest.main()
