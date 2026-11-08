"""The v5 robot's own model and IMU chain: nothing of upkie's may be left in them (2026-11-07).

Fusion -> URDF -> MuJoCo: masses, COM, inertia, joints and the IMU pose must be the CAD's.  IMU: the pi3hat's axes and frames
(docs/reference.md of mjbots/pi3hat), its firmware filter, and the MCU's attitude filter with the IMU position in it.
The robot's front is the rocker switch side, the CAD's -x: robot axes = CAD axes turned 180 deg about z (BV.R_ROBOT_FROM_CAD).
"""
from __future__ import annotations

import copy
import json
import math
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from wbr.common.rotations import quat_to_matrix, rot_y, roll_pitch_yaw
from wbr.controllers.diablo_arch.mcu import DiabloMcu
from wbr.controllers.lqi6.serial_leg import SerialLeg, SerialLegGeometry
from wbr.models import build_v5 as BV
from wbr.models.upkie_params import set_leg_posture
from wbr.observers.body_attitude import BodyAttitude
from wbr.observers.pi3hat_ars import G, Pi3HatAttitude, quat_from_euler, steady_covariance
from wbr.observers.upkie_observers import BaseOrientation
from wbr.plans import plan4_upkie_lqi6 as P4
from wbr.plans.plan_v5 import PlanV5, PlanV5Config

R_ARS_TO_WORLD = np.diag([1.0, -1.0, -1.0])
CAD = json.loads(BV.MASS_JSON.read_text(encoding="utf-8"))
URDF = BV.OUT_DIR / f"{BV.NAME}.urdf"
A = BV.R_ROBOT_FROM_CAD


def model() -> tuple:
    m = mujoco.MjModel.from_xml_path(str(BV.OUT_DIR / "scene.xml"))
    return m, mujoco.MjData(m)


class ModelIsTheCadTests(unittest.TestCase):
    def test_no_upkie_in_the_description(self):
        text = URDF.read_text(encoding="utf-8").lower()
        self.assertNotIn("upkie", text)
        xml = (BV.OUT_DIR / f"{BV.NAME}.xml").read_text(encoding="utf-8").lower()
        self.assertNotIn("upkie", xml)
        self.assertNotIn("qdd100", text + xml)

    def test_urdf_links_and_joint_axes(self):
        root = ET.parse(URDF).getroot()
        self.assertEqual(root.attrib["name"], "wheelbiped_v5")
        joints = {j.attrib["name"]: j for j in root.findall("joint")}
        for name in BV.JOINTS:
            self.assertEqual(joints[name].find("axis").attrib["xyz"], "0 1 0", name)     # all six +y: no mirrored joints
        self.assertEqual(joints["imu_fix"].attrib["type"], "fixed")
        self.assertEqual({lk.attrib["name"] for lk in root.findall("link")},
                         {"base", "imu", "left_thigh", "left_shank", "left_wheel_link", "right_thigh", "right_shank", "right_wheel_link"})

    def test_mass_com_inertia_are_the_cads_at_the_nominal_pose(self):
        m, d = model()
        p = CAD["pose"]
        leg = SerialLeg(SerialLegGeometry(L1=p["L1"] / 1000, L2=p["L2"] / 1000, knee_sign=BV.KNEE_SIGN))
        set_leg_posture(m, d, leg, p["L0"] / 1000, 0.0, wheel_radius=p["R_wheel"] / 1000, signs=BV.JOINT_SIGNS)
        robot = CAD["links"]["robot"]
        self.assertAlmostEqual(float(m.body_subtreemass[1]), robot["mass"], delta=1e-4)
        np.testing.assert_allclose(1000 * d.subtree_com[1], A @ np.array(robot["com"]), atol=1e-2)
        com = d.subtree_com[1]
        I = np.zeros((3, 3))
        for b in range(1, m.nbody):
            R = d.ximat[b].reshape(3, 3)
            r = d.xipos[b] - com
            I += R @ np.diag(m.body_inertia[b]) @ R.T + m.body_mass[b] * (r @ r * np.eye(3) - np.outer(r, r))
        np.testing.assert_allclose(I * 1e6, A @ np.array(robot["inertia"]) @ A.T, atol=2.0)      # kg mm^2 of about 7e5

    def test_joint_anchors_and_tyre_are_the_cads(self):
        m, d = model()
        p = CAD["pose"]
        leg = SerialLeg(SerialLegGeometry(L1=p["L1"] / 1000, L2=p["L2"] / 1000, knee_sign=BV.KNEE_SIGN))
        set_leg_posture(m, d, leg, p["L0"] / 1000, 0.0, wheel_radius=p["R_wheel"] / 1000, signs=BV.JOINT_SIGNS)
        jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
        for side, s in (("left", 1.0), ("right", -1.0)):
            np.testing.assert_allclose(1000 * d.xanchor[jid(f"{side}_hip")][[0, 2]], [0.0, p["Z_hip"]], atol=1e-3)
            np.testing.assert_allclose(1000 * d.xanchor[jid(f"{side}_knee")][[0, 2]], [-p["knee"][0], p["knee"][1]], atol=1e-3)     # the knees point backwards
            self.assertLess(d.xanchor[jid(f"{side}_knee")][0], 0.0)
            np.testing.assert_allclose(1000 * d.xanchor[jid(f"{side}_wheel")][[0, 2]], [0.0, p["R_wheel"]], atol=1e-3)
            g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"{side}_tire")
            self.assertAlmostEqual(1000 * d.geom_xpos[g][1], s * p["tyre_centre_y"], delta=1e-3)
            self.assertAlmostEqual(1000 * m.geom_size[g][0], p["R_wheel"], delta=1e-6)

    def test_imu_site_is_the_chip_on_the_hat(self):
        m, d = model()
        p = CAD["pose"]
        leg = SerialLeg(SerialLegGeometry(L1=p["L1"] / 1000, L2=p["L2"] / 1000, knee_sign=BV.KNEE_SIGN))
        set_leg_posture(m, d, leg, p["L0"] / 1000, 0.0, wheel_radius=p["R_wheel"] / 1000, signs=BV.JOINT_SIGNS)
        data = json.loads(BV.URDF_JSON.read_text(encoding="utf-8"))["imu"]
        s = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "imu")
        np.testing.assert_allclose(1000 * d.site_xpos[s], A @ np.array(data["chips"][BV.IMU_CHIP]), atol=1e-3)
        # docs/reference.md: +x CAN / HDMI, +y USB, +z Pi -> hat: on the v5 deck the robot's rear, right, up
        np.testing.assert_allclose(d.site_xmat[s].reshape(3, 3), np.diag([-1.0, -1.0, 1.0]), atol=1e-9)
        for a, want in zip("xyz", np.eye(3)):       # measured in Fusion: along the CAD axes
            np.testing.assert_allclose(data["axes"][a], want, atol=1e-9)

    def test_imu_is_on_the_centre_line(self):
        pos, _ = BV.V5Geometry().imu()
        self.assertLess(abs(pos[0]), 0.001)
        self.assertLess(abs(pos[1]), 0.003)


class PlanHasNoUpkieValuesTests(unittest.TestCase):
    def setUp(self):
        self.plan = PlanV5(PlanV5Config.from_model())

    def test_robot_constants(self):
        p = self.plan
        self.assertEqual(set(p.signs.values()), {1.0})
        self.assertAlmostEqual(p.wheel_radius, CAD["pose"]["R_wheel"] / 1000, places=9)
        self.assertAlmostEqual(p.track, 2 * CAD["pose"]["tyre_centre_y"] / 1000, places=9)
        self.assertEqual(p.mcu.p.tau_hip_max, 12.5)
        self.assertEqual(p.mcu.p.tau_wheel_max, 12.5)
        self.assertAlmostEqual(p.leg.g.L1, CAD["pose"]["L1"] / 1000, places=9)
        self.assertAlmostEqual(p.leg.g.L2, CAD["pose"]["L2"] / 1000, places=9)
        self.assertAlmostEqual(p.leg.g.L1, 0.25, places=9)       # user 11-07
        self.assertAlmostEqual(p.mcu.p.L_stand, 0.46, places=9)   # 40 mm from straight, as on the 175 mm robot
        self.assertAlmostEqual(p.mcu.p.L_max, 0.485, places=9)
        self.assertLess(p.mcu.p.L_min, p.mcu.p.L_stand)
        self.assertEqual(p.hip_above_base, 0.0)         # the base frame is the hip midpoint (the DIABLO paper's body frame)
        self.assertAlmostEqual(p.params.m_b, CAD["links"]["torso"]["mass"], places=5)

    def test_imu_mount_is_this_robots_not_upkies(self):
        np.testing.assert_allclose(self.plan.mcu.imu.R_base_to_imu, np.diag([-1.0, -1.0, 1.0]))
        self.assertFalse(np.allclose(self.plan.mcu.imu.R_base_to_imu, np.diag([-1.0, 1.0, -1.0])))

    def test_upkies_mount_constant_would_read_the_standing_robot_upside_down(self):
        q = quat_from_euler(math.pi, 0.0)                # the hat side up on a level robot, what the board reports
        R = BaseOrientation(np.diag([-1.0, 1.0, -1.0]), R_ARS_TO_WORLD).update(q, np.zeros(3))["rotation_base_to_world"]
        self.assertAlmostEqual(abs(roll_pitch_yaw(R)[0]), math.pi, places=6)
        R = BaseOrientation(np.diag([-1.0, -1.0, 1.0]), R_ARS_TO_WORLD).update(q, np.zeros(3))["rotation_base_to_world"]
        np.testing.assert_allclose(roll_pitch_yaw(R)[:2], [0.0, 0.0], atol=1e-9)

    def test_a_forward_lean_reads_as_positive_pitch_through_the_rotated_board(self):
        # the base pitched forward about its +y = the board's -y: the mount must give the sign back
        R_bi = np.diag([-1.0, -1.0, 1.0])
        tilt = math.radians(5.0)
        R_wi = rot_y(tilt) @ R_bi.T
        q = P4.matrix_to_quat(R_ARS_TO_WORLD.T @ R_wi)
        R = BaseOrientation(R_bi, R_ARS_TO_WORLD).update(q, np.zeros(3))["rotation_base_to_world"]
        self.assertAlmostEqual(roll_pitch_yaw(R)[1], tilt, places=9)

    def test_knees_are_on_the_rear_side_in_the_controller(self):
        self.assertEqual(self.plan.leg.g.knee_sign, BV.KNEE_SIGN)
        qh, qk = self.plan.leg.inverse(self.plan.mcu.p.L_stand, 0.0)
        self.assertGreater(qh, 0.0)     # the thigh swings its end backwards
        self.assertLess(qk, 0.0)

    def test_controller_sees_the_true_tilt_with_the_right_signs(self):
        for pitch in (math.radians(6), math.radians(-6)):
            plan = PlanV5(PlanV5Config.from_model(latency_ms=0.0))
            plan.reset(pitch=pitch)
            plan.step(0.0)
            self.assertAlmostEqual(plan.mcu.est.pitch, pitch, delta=math.radians(0.3))

    def test_stands_with_the_whole_chain(self):
        plan = PlanV5(PlanV5Config.from_model(noise=True, seed=1, latency_ms=10.0))
        self.assertTrue(plan.run(3.0))
        n = int(1.0 / plan.dt)
        self.assertLess(abs(np.degrees(plan.log["pitch_true"][-n:].mean())), 1.0)
        self.assertLess(abs(np.degrees(plan.log["roll_true"][-n:].mean())), 0.1)


class FirmwareFilterTests(unittest.TestCase):
    """wbr.observers.pi3hat_ars against what docs/reference.md and fw/attitude_reference.h say."""

    def test_hat_side_up_at_rest_reads_roll_180(self):
        # "the raspberry pi must be inverted to achieve a pitch and roll of 0"
        f = Pi3HatAttitude(1e-3)
        for _ in range(200):
            f.process(np.zeros(3), np.array([0.0, 0.0, G]))         # raw axes: +z from the Pi to the hat: 1 g up
        R = quat_to_matrix(f.quaternion)
        roll = math.atan2(R[2, 1], R[2, 2])
        self.assertAlmostEqual(abs(roll), math.pi, delta=1e-3)
        np.testing.assert_allclose(R_ARS_TO_WORLD @ R, np.eye(3), atol=1e-3)

    def test_board_inverted_reads_level(self):
        f = Pi3HatAttitude(1e-3)
        for _ in range(200):
            f.process(np.zeros(3), np.array([0.0, 0.0, -G]))
        np.testing.assert_allclose(quat_to_matrix(f.quaternion), np.eye(3), atol=1e-3)

    def test_acceleration_reads_as_tilt_slowly(self):
        f = Pi3HatAttitude(1e-3)
        f.start_converged(quat_from_euler(math.pi, 0.0))
        for _ in range(500):
            f.process(np.zeros(3), np.array([2.0, 0.0, G]))         # 2 m/s^2 forward, the body level, for 0.5 s
        pitch = roll_pitch_yaw(R_ARS_TO_WORLD @ quat_to_matrix(f.quaternion))[1]
        self.assertLess(math.degrees(pitch), -1.0)                  # a false tilt of 1.6 deg ...
        self.assertGreater(math.degrees(pitch), -2.5)               # ... far from the 11.5 deg of the acceleration itself

    def test_large_acceleration_is_rejected(self):
        f = Pi3HatAttitude(1e-3)
        f.start_converged(quat_from_euler(math.pi, 0.0))
        for _ in range(500):
            f.process(np.zeros(3), np.array([8.0, 0.0, G]))         # |a| 12.7, more than 2 m/s^2 off 1 g
        pitch = roll_pitch_yaw(R_ARS_TO_WORLD @ quat_to_matrix(f.quaternion))[1]
        self.assertLess(abs(math.degrees(pitch)), 0.01)

    def test_yaw_rate_is_pulled_to_zero_by_the_bias_state(self):
        f = Pi3HatAttitude(1e-3)
        f.start_converged(quat_from_euler(math.pi, 0.0))
        for _ in range(5000):
            f.process(np.array([0.0, 0.0, 2.0]), np.array([0.0, 0.0, G]))
        self.assertLess(f.rate_rps[2], 1.95)                        # what the MCU must not feed its yaw loop with

    def test_steady_covariance_table(self):
        self.assertEqual(steady_covariance(1e-3).shape, (7, 7))
        self.assertEqual(steady_covariance(2.5e-3).shape, (7, 7))


class BodyAttitudeTests(unittest.TestCase):
    """The MCU's complementary filter: the robot's own acceleration and the IMU's lever arm are taken out."""

    def _level_run(self, f_of, r=(0.0, 0.0, 0.0), gyro=(0.0, 0.0, 0.0), vx=None, T=3.0, yaw_rate=0.0):
        a = BodyAttitude(5.0, r, 1e-3)
        up = np.array([0.0, 0.0, 1.0])
        for k in range(int(T * 1000)):
            t = k * 1e-3
            a.set_hip_velocity(0.0 if vx is None else vx(t), 0.0, yaw_rate)
            a.update(np.array(gyro, dtype=float), f_of(t), up)
        return a

    def test_at_rest_it_finds_the_tilt(self):
        tilt = math.radians(4.0)
        a = BodyAttitude(0.5, np.zeros(3), 1e-3)
        f = G * np.array([-math.sin(tilt), 0.0, math.cos(tilt)])
        for _ in range(4000):
            a.set_hip_velocity(0.0, 0.0, 0.0)
            a.update(np.zeros(3), f, np.array([0.0, 0.0, 1.0]))
        self.assertAlmostEqual(a.pitch, tilt, delta=math.radians(0.02))

    def test_forward_acceleration_is_not_a_tilt(self):
        a = self._level_run(lambda t: np.array([1.5, 0.0, G]), vx=lambda t: 1.5 * t)
        self.assertLess(abs(math.degrees(a.pitch)), 0.05)

    def test_without_the_encoders_it_would_be(self):
        a = self._level_run(lambda t: np.array([1.5, 0.0, G]))
        self.assertGreater(abs(math.degrees(a.pitch)), 3.0)

    def test_lever_arm_of_a_spin_is_taken_out(self):
        r = np.array([0.0, -0.07, 0.0])
        w = np.array([0.0, 0.0, 2.0])
        f = np.array([0.0, 0.0, G]) + np.cross(w, np.cross(w, r))      # centripetal at the IMU
        known = self._level_run(lambda t: f, r=r, gyro=w, T=5.0)
        ignored = self._level_run(lambda t: f, gyro=w, T=5.0)
        self.assertLess(abs(math.degrees(known.roll)) + abs(math.degrees(known.pitch)), 0.02)
        self.assertGreater(abs(math.degrees(ignored.roll)) + abs(math.degrees(ignored.pitch)), 0.2)

    def test_gyro_is_trusted_during_a_shake(self):
        a = BodyAttitude(0.5, np.zeros(3), 1e-3)
        for k in range(1000):
            s = 10.0 * math.sin(2 * math.pi * 12 * k * 1e-3)
            a.set_hip_velocity(0.0, 0.0, 0.0)
            a.update(np.zeros(3), np.array([s, 0.0, G]), np.array([0.0, 0.0, 1.0]))
        self.assertLess(abs(math.degrees(a.pitch)), 2.0)


class YawGyroTests(unittest.TestCase):
    def test_robot_stops_turning_after_a_long_turn(self):
        plan = PlanV5(PlanV5Config.from_model(latency_ms=10.0))
        self.assertTrue(plan.run(0.5))
        self.assertTrue(plan.run(5.0, 0.0, 2.0))
        self.assertTrue(plan.run(3.0))
        n = int(1.0 / plan.dt)
        self.assertLess(abs(float(plan.log["yaw_rate"][-n:].mean())), 0.01)
        self.assertLess(plan._ars.bias_rps[2], -0.05)               # the board's own bias state did run away

    def test_board_rate_alone_keeps_it_turning(self):
        plan = PlanV5(PlanV5Config.from_model(latency_ms=10.0, yaw_offset_tau=0.0))
        self.assertTrue(plan.run(0.5))
        self.assertTrue(plan.run(5.0, 0.0, 2.0))
        self.assertTrue(plan.run(3.0))
        n = int(1.0 / plan.dt)
        self.assertGreater(abs(float(plan.log["yaw_rate"][-n:].mean())), 0.05)

    def test_offset_is_learned_at_standstill(self):
        plan = PlanV5(PlanV5Config.from_model(latency_ms=0.0, yaw_offset_tau=1.0))
        self.assertTrue(plan.run(0.2))
        obs = copy.deepcopy(plan._obs_to_mcu(plan.observation))
        mcu: DiabloMcu = plan.mcu
        before = mcu.yaw_offset
        for k in range(1000):
            o = copy.deepcopy(obs)
            o["time"] = obs["time"] + (k + 1) * plan.dt
            o["imu"]["angular_velocity"] = np.array(o["imu"]["angular_velocity"]) + np.array([0.0, 0.0, 0.05])    # a gyro offset
            mcu.estimate(o)
        self.assertAlmostEqual(mcu.yaw_offset - before, 0.05, delta=0.02)


class StaticLegForceTests(unittest.TestCase):
    """The leg's feed-forward holds its length at any lean: on a slope the lean differs by 10 deg between uphill and downhill."""

    def test_formula_is_half_of_dV_dL0(self):
        plan = PlanV5(PlanV5Config.from_model())
        p, t = plan.plant, plan.plant.p
        L, th, e = plan.mcu.p.L_stand, math.radians(6.0), 1e-4

        def V(L0):
            leg = t.leg.at(L0)
            return t.g * (leg["m_l"] * (-leg["d"] * math.sin(th) + leg["c"] * math.cos(th)) + t.m_b * L0 * math.cos(th))
        self.assertAlmostEqual(p.static_leg_force(L, th), 0.5 * (V(L + e) - V(L - e)) / (2 * e), delta=0.05)

    def test_v5_uses_it_and_plan4_does_not(self):
        plan = PlanV5(PlanV5Config.from_model())
        self.assertIsNotNone(plan.mcu.p.leg_force)
        L = plan.mcu.p.L_stand
        self.assertLess(plan.mcu.p.leg_force(L, math.radians(9.0)), plan.mcu.p.leg_force(L, math.radians(-1.0)))      # d' > 0, leg COM on the rear side
        self.assertIsNone(P4.Plan4UpkieLQI6().mcu.p.leg_force)

    def test_leg_holds_its_length_standing(self):
        plan = PlanV5(PlanV5Config.from_model(latency_ms=10.0))
        self.assertTrue(plan.run(4.0))
        n = int(1.0 / plan.dt)
        err = 1000 * (plan.log["L0"][-n:] - plan.log["L_ref"][-n:]).mean()
        self.assertLess(abs(err), 0.5)


class UnevenGroundTests(unittest.TestCase):
    def test_speed_limit_follows_the_leg_difference(self):
        mcu = PlanV5(PlanV5Config.from_model()).mcu
        dt = mcu.p.dt
        self.assertEqual(mcu._uneven_limit(0.0, dt), mcu.p.max_forward)
        self.assertAlmostEqual(mcu._uneven_limit(0.5 * (mcu.p.uneven_d0 + mcu.p.uneven_d1), dt), 0.5 * (mcu.p.max_forward + mcu.p.uneven_speed), places=9)
        self.assertAlmostEqual(mcu._uneven_limit(0.10, dt), mcu.p.uneven_speed, places=9)
        for _ in range(int(0.4 / dt)):                          # the hold: the limit stays after the legs are even again
            self.assertAlmostEqual(mcu._uneven_limit(0.0, dt), mcu.p.uneven_speed, places=9)
        for _ in range(int(1.0 / dt)):
            v = mcu._uneven_limit(0.0, dt)
        self.assertEqual(v, mcu.p.max_forward)

    def test_plan4_has_no_limit(self):
        p = P4.Plan4UpkieLQI6().mcu.p
        self.assertEqual(p.uneven_speed, 0.0)
        self.assertFalse(p.mean_follows_range)
        self.assertEqual(p.L_min_margin, 0.02)

    def test_speed_bump_the_leg_gives_way_and_the_body_stays_low(self):
        plan = PlanV5(PlanV5Config.from_model(latency_ms=10.0, floor_humps=((0.10, 1.0, 0.6, 1.0, 0.15),)))
        self.assertTrue(plan.run(1.0))
        z0 = float(plan.log["z"][-1])
        n = 0
        while plan.log["x"][-1] < 1.1 and n < 12000:
            plan.step(0.5)
            n += 1
        self.assertFalse(plan.fallen)
        n1 = int(0.2 / plan.dt)
        self.assertLess(float(plan.log["vx"][-n1:].mean()), 0.3)     # on the top, 0.5 m/s commanded
        self.assertLess(abs(math.degrees(plan.log["roll_true"][-n1:].mean())), 1.0)
        left, right = 1000 * plan.log["L0_left"][-1], 1000 * plan.log["L0_right"][-1]
        self.assertGreater(right - left, 90.0)                       # the leg on the bump took it ...
        self.assertLess(right, 1000 * plan.mcu.p.L_max)
        self.assertLess(1000 * (float(plan.log["z"].max()) - z0), 22.0)     # ... and the body rose by a fifth of the bump (mean-length control: half)


class SplitPdTests(unittest.TestCase):
    def test_gains(self):
        plan = PlanV5(PlanV5Config.from_model())
        k = plan.leg.g.L1 / 0.175           # with the link length (11-07)
        self.assertAlmostEqual(plan.mcu.p.kp_split, 80.0 * k, places=9)
        self.assertAlmostEqual(plan.mcu.p.kd_split, 2.0 * k, places=9)
        self.assertEqual((P4.Plan4Config().kp_split, P4.Plan4Config().kd_split), (60.0, 3.0))      # upkie's verdicts stand

    def test_turning_does_not_open_the_legs(self):
        plan = PlanV5(PlanV5Config.from_model(latency_ms=10.0))
        self.assertTrue(plan.run(0.5))
        split = []
        for _ in range(int(3.0 / plan.dt)):
            plan.step(0.5, 1.0)
            split.append(abs(plan.mcu.telemetry.get("d_theta", 0.0)))
        self.assertLess(math.degrees(max(split)), 3.5)


class MountErrorTests(unittest.TestCase):
    def test_a_board_mounted_off_level_reads_as_a_tilt(self):
        plan = PlanV5(PlanV5Config.from_model(latency_ms=0.0, pitch_bias_deg=1.0, attitude_tau=0.5))
        plan.reset()
        plan.step(0.0)
        self.assertAlmostEqual(math.degrees(plan.mcu.est.pitch), 1.0, delta=0.2)


def _power_on_read(q: float, ratio: float = 0.125) -> float:
    """What a moteus with its encoder on the rotor reads after power-on: the output angle within +-22.5 deg (8:1)."""
    h = math.pi * ratio
    return (q + h) % (2 * h) - h


class BringUpGuardTests(unittest.TestCase):
    """Step 0 of the bring-up (docs/bringup_step0.md): what the MCU must refuse, or survive, on the real robot."""

    def _at_rest(self):
        plan = PlanV5(PlanV5Config.from_model(latency_ms=0.0))
        plan.reset()
        plan.mcu.reset(armed=False)
        return plan, plan._obs_to_mcu(plan.observation)

    def _feed(self, plan, obs, n, edit=None):
        t0 = obs["time"]
        for k in range(n):
            o = copy.deepcopy(obs)
            o["time"] = t0 + k * plan.dt
            if edit is not None:
                edit(k, o)
            plan.mcu.estimate(o)

    def test_arm_refuses_legs_read_before_homing(self):
        # every un-homed knee reads |sigma2| <= 22.5 deg, a leg of at least 490 mm, above the 485 mm range
        for L0 in (0.372, 0.42, 0.46, 0.485):
            plan, obs = self._at_rest()
            qh, qk = plan.leg.inverse(L0, plan.plant.theta_eq(L0))
            for side in ("left", "right"):
                obs["servo"][f"{side}_hip"]["position"] = _power_on_read(qh)
                obs["servo"][f"{side}_knee"]["position"] = _power_on_read(qk)
            plan.mcu.estimate(obs)
            self.assertGreater(plan.mcu.est.L0["left"], 0.49)
            self.assertFalse(plan.mcu.arm(), L0)
            self.assertIn("leg reads", plan.mcu.arm_refused)

    def test_arm_refuses_a_hip_read_45_deg_off(self):
        plan, obs = self._at_rest()
        for side in ("left", "right"):
            obs["servo"][f"{side}_hip"]["position"] = _power_on_read(obs["servo"][f"{side}_hip"]["position"])
        plan.mcu.estimate(obs)
        self.assertFalse(plan.mcu.arm())
        self.assertIn("leg angle", plan.mcu.arm_refused)

    def test_arm_needs_the_legs_referenced_to_the_output(self):
        plan, obs = self._at_rest()
        for s in obs["servo"].values():
            s["home_state"] = 1                     # moteus 0x00c: rotor only, before d nearest
        self._feed(plan, obs, 1)
        self.assertFalse(plan.mcu.arm())
        self.assertIn("home state 1", plan.mcu.arm_refused)
        for name, s in obs["servo"].items():
            s["home_state"] = 1 if name.endswith("wheel") else 2     # the wheels need no output reference
        obs["time"] += plan.dt
        self._feed(plan, obs, 1)
        self.assertTrue(plan.mcu.arm())
        self.assertIsNone(plan.mcu.arm_refused)

    def test_the_standing_robot_arms(self):
        plan, obs = self._at_rest()
        plan.mcu.estimate(obs)
        self.assertTrue(plan.mcu.arm(), plan.mcu.arm_refused)

    def test_a_nan_in_the_new_imu_channels_is_a_fault(self):
        for channel, bad in (("linear_acceleration", [math.nan, 0.0, G]), ("bias_rps", [0.0, 0.0, math.nan])):
            plan, obs = self._at_rest()

            def edit(k, o, channel=channel, bad=bad):
                if k >= 10:
                    o["imu"][channel] = np.array(bad)
            self._feed(plan, obs, 20, edit)
            self.assertTrue(plan.mcu.fault, channel)
            self.assertTrue(any(f"imu.{channel}" in m for _, m in plan.mcu.events), plan.mcu.events)
            self.assertTrue(np.all(np.isfinite(plan.mcu.attitude.f_lp)))
            self.assertTrue(math.isfinite(plan.mcu.est.pitch))

    def test_a_missing_accelerometer_is_a_fault_not_a_crash(self):
        plan, obs = self._at_rest()
        del obs["imu"]["linear_acceleration"]
        self._feed(plan, obs, 10)
        self.assertTrue(plan.mcu.fault)
        self.assertTrue(any("imu.linear_acceleration" in m for _, m in plan.mcu.events), plan.mcu.events)

    def test_the_bias_channel_lost_is_a_fault_and_never_sent_is_reported(self):
        plan, obs = self._at_rest()

        def drop(k, o):
            if k >= 5:
                del o["imu"]["bias_rps"]
        self._feed(plan, obs, 15, drop)
        self.assertTrue(plan.mcu.fault)
        self.assertTrue(any("imu.bias_rps" in m for _, m in plan.mcu.events), plan.mcu.events)
        plan, obs = self._at_rest()
        del obs["imu"]["bias_rps"]
        self._feed(plan, obs, 15)
        self.assertFalse(plan.mcu.fault)
        self.assertEqual(sum("no bias_rps" in m for _, m in plan.mcu.events), 1)

    def test_a_skipped_sample_is_integrated_over_its_interval(self):
        # the body turns at 0.5 rad/s about +y; the second run sees only every other sample (random sample age, late reads)
        def pitch_after(step):
            plan, obs = self._at_rest()
            w = plan.mcu.imu.R_base_to_imu @ np.array([0.0, 0.5, 0.0])
            obs["imu"]["angular_velocity"] = w
            t0 = obs["time"]
            for k in range(0, 101, step):
                o = copy.deepcopy(obs)
                o["time"] = t0 + k * plan.dt
                plan.mcu.estimate(o)
            return plan.mcu.est.pitch
        every, skipped = pitch_after(1), pitch_after(2)
        self.assertAlmostEqual(every, 0.5 * 0.101, delta=0.002)
        self.assertAlmostEqual(skipped, every, delta=5e-4)       # fixed dt: half of it

    def test_steady_covariance_at_the_slow_rate(self):
        self.assertEqual(steady_covariance(5e-3).shape, (7, 7))        # WBR_TEST_RATE=200
        P = steady_covariance(1e-2)                                     # not in the table: computed
        self.assertTrue(np.all(np.linalg.eigvalsh(P) > 0.0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
