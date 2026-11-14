"""The closed-loop runtime (wbr.plans.plan4_lqi8 + the DIABLO-style MCU) on the v5 model: the model's kinematics and plant
against MuJoCo, the gain table the plan runs, closed-loop smoke runs, the MCU's sensor / command guards and faults, and
the mechanisms of the wheel-reaction feed-forward, the compliant roll and the yaw wheel damping.

Moved here on 2026-11-14 from tests/test_plan4.py and tests/test_plan4_safety.py, which ran the same checks on the upkie
model of Plan 4 (removed that day).  Left out with the upkie model: its mirrored joint signs and IMU mount (the v5 ones are
in tests/test_robot_v5.py), the joint ranges of upkie's URDF (the v5 model's are +-pi), the stage-2 MPC host and the
acceptance battery.  What the mechanisms do on slopes, bumps and steps is judged by the staged tests (v5_lqi/TEST 2-5,
2-7, 2-8, 2-11 to 2-13).  The checks without MuJoCo are in tests/test_leg_plant.py.

Needs the mujoco package (a Windows PC with Smart App Control on blocks its DLLs: run these on the Ubuntu laptop).
Run from wheeled_biped/:  python -m unittest tests.test_runtime_v5
"""
from __future__ import annotations

import copy
import functools
import math
import unittest

import mujoco
import numpy as np

from wbr.common.servo import ServoModel, ServoProperties
from wbr.controllers.lqi8.lqi8 import LQI8, c2d_zoh
from wbr.controllers.lqi8.plant_serial import SerialPlant, SerialPlantParams
from wbr.controllers.lqi8.serial_leg import SerialLeg, SerialLegGeometry
from wbr.models import build_v5 as BV
from wbr.models.plant_params import set_leg_posture
from wbr.plans.plan_v5 import PlanV5, PlanV5Config

SCENE = BV.OUT_DIR / "scene.xml"
JOINTS = ("left_hip", "left_knee", "left_wheel", "right_hip", "right_knee", "right_wheel")
WHEELS = ("left_wheel", "right_wheel")
SIGNS = dict(BV.JOINT_SIGNS)


def make_plan(**cfg) -> PlanV5:
    """The committed v5 model with its gains (description.json); noise off and no latency unless cfg sets them."""
    return PlanV5(PlanV5Config.from_model(**cfg))


@functools.lru_cache(maxsize=1)
def v5_params() -> SerialPlantParams:
    return SerialPlantParams.from_json(BV.OUT_DIR / BV.PARAMS_FILE)


@functools.lru_cache(maxsize=1)
def v5_plant() -> SerialPlant:
    return SerialPlant(v5_params())


def v5_leg() -> SerialLeg:
    p = v5_params()
    return SerialLeg(SerialLegGeometry(L1=p.extra["L1"], L2=p.extra["L2"], knee_sign=p.extra["knee_sign"]))


def mean_last(plan, key: str, seconds: float = 0.2) -> float:
    n = max(1, int(round(seconds / plan.dt)))
    return float(np.mean(plan.log[key][-n:]))


def augmented_discrete(ctrl, L0: float):
    """Discrete closed-loop model of an LQI8 instance, rebuilt here from the plant linearisation and the documented
    integral states: xs = [theta, theta_dot, v, phi, phi_dot, int v] (+ optional z_att' = theta + phi + (Tp - T) /
    (g M(L0)) and z_pos' = int v when the controller has them, ctrl.n = 7 or 8)."""
    A, B = ctrl.plant.linearize(L0)
    n = getattr(ctrl, "n", 6)
    keep = [0, 1, 3, 4, 5]
    Aa = np.zeros((n, n))
    Aa[:5, :5] = A[np.ix_(keep, keep)]
    Aa[5, 2] = 1.0
    Ba = np.zeros((n, 2))
    Ba[:5, :] = B[keep, :]
    i_att, i_pos = getattr(ctrl, "i_att", None), getattr(ctrl, "i_pos", None)
    if i_att is not None:
        gM = ctrl.att_gM(L0)
        Aa[i_att, 0] = Aa[i_att, 3] = 1.0
        Ba[i_att, :] = [-1.0 / gM, 1.0 / gM]
    if i_pos is not None:
        Aa[i_pos, 5] = 1.0
    return c2d_zoh(Aa, Ba, ctrl.Ts)


# ----------------------------------------------------------------------------- the model's kinematics
class ModelKinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.m = mujoco.MjModel.from_xml_path(str(SCENE))
        cls.d = mujoco.MjData(cls.m)
        cls.base = mujoco.mj_name2id(cls.m, mujoco.mjtObj.mjOBJ_BODY, "base")

    def jid(self, name: str) -> int:
        return mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, name)

    def pose(self, which: str) -> None:
        m, d, p = self.m, self.d, v5_params()
        if which == "keyframe":            # straight legs, base level
            mujoco.mj_resetDataKeyframe(m, d, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "stand"))
            mujoco.mj_forward(m, d)
        else:                              # standing posture, base level
            set_leg_posture(m, d, v5_leg(), p.L0, v5_plant().theta_eq(p.L0), wheel_radius=p.R, signs=SIGNS)

    def axle_in_base(self, side: str) -> np.ndarray:
        R = self.d.xmat[self.base].reshape(3, 3)
        return R.T @ (self.d.xanchor[self.jid(f"{side}_wheel")] - self.d.xpos[self.base])

    def test_signed_joint_axes_are_plus_y(self):
        # u-convention = rotation about +y for every joint (leg end backwards / wheel rolling forward)
        for which in ("keyframe", "standing"):
            self.pose(which)
            for n in JOINTS:
                axis = SIGNS[n] * self.d.xaxis[self.jid(n)]
                np.testing.assert_allclose(axis, [0.0, 1.0, 0.0], atol=1e-9, err_msg=f"{which}: {n}")

    def test_positive_u_convention_moves_the_axle_backwards(self):
        for which in ("keyframe", "standing"):
            for side, other in (("left", "right"), ("right", "left")):
                for joint in ("hip", "knee"):
                    name = f"{side}_{joint}"
                    self.pose(which)
                    a0, b0 = self.axle_in_base(side), self.axle_in_base(other)
                    self.d.qpos[self.m.jnt_qposadr[self.jid(name)]] += 0.1 * SIGNS[name]
                    mujoco.mj_forward(self.m, self.d)
                    a1, b1 = self.axle_in_base(side), self.axle_in_base(other)
                    self.assertLess(a1[0] - a0[0], -0.01, f"{which}: +0.1 rad (u-convention) on {name} must move the axle back")
                    self.assertLess(np.abs(b1 - b0).max(), 1e-12, f"{which}: {name} moved the other leg")

    def test_serial_leg_kinematics_match_the_model(self):
        p, leg = v5_params(), v5_leg()
        for L0, th in ((p.L0, v5_plant().theta_eq(p.L0)), (0.40, 0.1), (0.48, -0.2)):
            qh, qk = set_leg_posture(self.m, self.d, leg, L0, th, wheel_radius=p.R, signs=SIGNS)
            p_fk, _, _ = leg.forward(qh, qk)
            for side in ("left", "right"):
                rel = self.d.xanchor[self.jid(f"{side}_wheel")] - self.d.xanchor[self.jid(f"{side}_hip")]
                np.testing.assert_allclose(rel[[0, 2]], p_fk, atol=1e-9, err_msg=f"{side} L0={L0}")

    def test_wheel_signs_roll_forward(self):
        m, d = self.m, self.d
        for name in WHEELS:
            self.pose("standing")
            d.qvel[:] = 0.0
            d.qvel[m.jnt_dofadr[self.jid(name)]] = SIGNS[name] * 1.0
            mujoco.mj_forward(m, d)
            v = np.zeros(6)
            mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, m.jnt_bodyid[self.jid(name)], v, 0)
            # rolling forward (+x) on the floor = spin about +y (contact point at rest: v_x = omega_y R)
            np.testing.assert_allclose(v[:3], [0.0, 1.0, 0.0], atol=1e-9, err_msg=name)
        from wbr.sim.mujoco_sim import MujocoSim, SimConfig
        sim = MujocoSim(SimConfig(xml_path=SCENE, joint_names=list(JOINTS), wheel_joints=list(WHEELS), base_body="base",
                                  imu_site="imu"), ServoModel({j: ServoProperties() for j in JOINTS}))
        self.assertEqual(sim.wheel_forward_signs(), {w: SIGNS[w] for w in WHEELS})


# ----------------------------------------------------------------------------- the plant against MuJoCo
class PlantAgainstMujocoTests(unittest.TestCase):
    def test_theta_eq_matches_mujoco_com_bisection(self):
        p, plant, leg = v5_params(), v5_plant(), v5_leg()
        m = mujoco.MjModel.from_xml_path(str(SCENE))
        d = mujoco.MjData(m)
        base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base")
        wheels = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in WHEELS]

        def com_ahead_of_axle(theta: float) -> float:
            set_leg_posture(m, d, leg, p.L0, theta, wheel_radius=p.R, signs=SIGNS)
            axle = 0.5 * (d.xanchor[wheels[0]] + d.xanchor[wheels[1]])
            return float(d.subtree_com[base][0] - axle[0])

        lo, hi = -0.3, 0.3
        self.assertLess(com_ahead_of_axle(lo), 0.0)
        self.assertGreater(com_ahead_of_axle(hi), 0.0)
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            if com_ahead_of_axle(mid) > 0.0:
                hi = mid
            else:
                lo = mid
        self.assertLess(abs(math.degrees(plant.theta_eq(p.L0) - 0.5 * (lo + hi))), 0.1)

    def test_plant_matches_mujoco_at_the_standing_length(self):
        from wbr.models.verify_plan4_plant import MjPlanar, check_L0
        p = v5_params()
        r = check_L0(MjPlanar(p), v5_plant(), p.L0)
        up = r["unstable_poles"]
        self.assertIsNotNone(up["rel_err"], f"unstable pole count differs: plant {up['plant']} mujoco {up['mujoco']}")
        self.assertEqual(len(up["rel_err"]), 2)
        self.assertLess(max(up["rel_err"]), 0.10, f"unstable poles plant {up['plant']} mujoco {up['mujoco']}")
        for name, b in r["B_check"].items():
            self.assertLess(b["rel_err"], 0.10, f"B check {name}: mujoco {b['mujoco']} plant {b['plant']}")
        # the static hip torque is small on v5 (the body COM is almost above the hips: 0.05 N m in the plant, 0.18 N m
        # in MuJoCo at 0.46 m, results/wheelbiped_v5_model), so it is compared with the body's weight moment, not as a ratio
        self.assertLess(abs(r["Tp_eq_mujoco"] - r["Tp_eq_plant"]), 0.02 * p.m_b * p.g * p.L0)


# ----------------------------------------------------------------------------- the LQI8 the plan runs
class RuntimeGainTests(unittest.TestCase):
    RATES = (1000.0, 200.0)

    @classmethod
    def setUpClass(cls):
        cls.ctrl = {rate: make_plan(rate_hz=rate).law.ctrl for rate in cls.RATES}
        p = v5_params()
        grid = [float(x) for x in p.leg.L0]
        mids = [0.5 * (a + b) for a, b in zip(grid[:-1], grid[1:]) if p.L_min <= 0.5 * (a + b) <= p.L_max]
        cls.mids = mids[::max(1, len(mids) // 4)]

    def assertClosedLoopStable(self, ctrl, label: str):
        p = v5_params()
        for L0 in (p.L_min, p.L0, p.L_max):
            Ad, Bd = augmented_discrete(ctrl, L0)
            for kind, K in (("table", ctrl.gains(L0).copy()), ("design", ctrl.design(L0).copy())):
                rho = float(np.abs(np.linalg.eigvals(Ad - Bd @ K)).max())
                self.assertLess(rho, 1.0, f"{label}, L0={L0}: {kind} gains unstable (rho={rho:.5f})")
        for L0 in np.linspace(p.L_min, p.L_max, 9):        # run-time leg range, interpolated table gains
            Ad, Bd = augmented_discrete(ctrl, float(L0))
            rho = float(np.abs(np.linalg.eigvals(Ad - Bd @ ctrl.gains(float(L0)))).max())
            self.assertLess(rho, 1.0, f"{label}, L0={L0:.4f}: table gains unstable (rho={rho:.5f})")

    def assertTableMatchesDesign(self, ctrl, label: str):
        direct = copy.copy(ctrl)                            # same weights and states, online design instead of the table
        direct.table = None
        for L0 in self.mids:                                # grid mid-points (interpolation error)
            K_table = ctrl.gains(L0).copy()
            K_direct = direct.design(L0)
            err = np.abs(K_table - K_direct).max() / np.abs(K_direct).max()
            self.assertLess(err, 0.02, f"{label}, L0={L0}")

    def test_closed_loop_stable_with_the_plan_weights(self):
        for rate in self.RATES:
            ctrl = self.ctrl[rate]
            self.assertIsInstance(ctrl, LQI8)
            self.assertAlmostEqual(ctrl.Ts, 1.0 / rate, places=12)
            self.assertClosedLoopStable(ctrl, f"plan default {rate:.0f} Hz")

    def test_gain_table_matches_direct_design(self):
        for rate in self.RATES:
            self.assertTableMatchesDesign(self.ctrl[rate], f"plan default {rate:.0f} Hz")

    def test_extra_integral_states_keep_the_loop_stable(self):
        # the optional LQI8 states (static body tilt z_att, position z_pos) with other positive weights must still give
        # a stabilisable design model and a stabilising, well-interpolated gain table
        for rate in self.RATES:
            ctrl = make_plan(rate_hz=rate, lqi_kwargs=dict(q_att=0.05, q_pos=0.5)).law.ctrl
            self.assertEqual(ctrl.n, 8)
            self.assertClosedLoopStable(ctrl, f"q_att + q_pos {rate:.0f} Hz")
            self.assertTableMatchesDesign(ctrl, f"q_att + q_pos {rate:.0f} Hz")


# ----------------------------------------------------------------------------- closed-loop smoke runs (noise off)
class SmokeTests(unittest.TestCase):
    def assertUpright(self, plan, max_deg: float, window_s: float = 0.5):
        n = int(round(window_s / plan.dt))
        peak = math.degrees(np.abs(plan.log["pitch_true"][-n:]).max())
        self.assertLess(peak, max_deg, f"|pitch_true| peak {peak:.2f} deg over the last {window_s} s")
        self.assertFalse(plan.fallen)
        self.assertFalse(plan.mcu.fault)

    def test_default_config_stands_two_seconds(self):
        plan = make_plan()
        self.assertFalse(plan.fallen)
        self.assertTrue(plan.run(2.0))
        self.assertUpright(plan, 3.0, window_s=1.0)

    def test_200hz_config_stands_two_seconds(self):
        plan = make_plan(rate_hz=200.0)
        self.assertAlmostEqual(plan.dt, 5e-3)
        self.assertAlmostEqual(plan.mcu.p.dt, 5e-3)
        self.assertAlmostEqual(plan.sim.control_dt, 5e-3)
        self.assertTrue(plan.run(2.0))
        self.assertUpright(plan, 3.0, window_s=1.0)

    def test_recovers_from_10_degree_tilt(self):
        plan = make_plan()
        plan.reset(pitch=math.radians(10))
        self.assertTrue(plan.run(4.0))
        self.assertLess(abs(math.degrees(plan.log["pitch_true"][-1])), 2.0)
        self.assertFalse(plan.mcu.fault)

    def test_crouch_4cm(self):
        plan = make_plan()
        self.assertTrue(plan.run(0.5))
        L0_before = plan.log["L0"][-1]
        plan.set_height_offset(-0.04)
        self.assertTrue(plan.run(2.0))
        self.assertGreaterEqual(L0_before - plan.log["L0"][-1], 0.03)
        self.assertUpright(plan, 3.0)

    def test_survives_push(self):
        plan = make_plan()
        self.assertTrue(plan.run(0.5))
        plan.push(0.3 * plan.sim.total_mass() * 9.81, 0.1)
        self.assertTrue(plan.run(3.0))
        self.assertUpright(plan, 3.0)

    def test_drive_and_turn_directions(self):
        plan = make_plan()
        plan.run(0.3)
        x0 = plan.log["x"][-1]
        self.assertTrue(plan.run(1.5, v=0.3))
        L, n = plan.log, int(round(0.5 / plan.dt))
        self.assertGreater(L["vx"][-1], 0.15)
        self.assertGreater(L["x"][-1] - x0, 0.1)
        self.assertLess(abs(np.mean(L["ground_vel"][-n:] - L["vx"][-n:])), 0.05)     # MCU odometry: sign and scale
        plan.reset()
        plan.run(0.3)
        self.assertTrue(plan.run(2.0, w=1.0))
        self.assertGreater(np.mean(plan.log["yaw_rate"][-n:]), 0.25)     # w > 0 turns counter-clockwise (REP-103)

    def test_fallen_and_fault(self):
        plan = make_plan()
        self.assertTrue(plan.run(0.2))
        self.assertFalse(plan.fallen)
        plan.reset(pitch=math.radians(70))
        for _ in range(5):
            plan.step()
        self.assertTrue(plan.fallen)
        self.assertTrue(plan.mcu.fault)
        # beyond the FAULT pitch (34 deg) but not "fallen" yet: every joint is commanded torque-free
        plan.reset(pitch=plan.mcu.p.fall_pitch + math.radians(6))
        plan.step()
        self.assertTrue(plan.mcu.fault)
        self.assertTrue(any("FAULT" in e for _, e in plan.mcu.events))
        self.assertFalse(plan.fallen)
        cmds = plan._cmds_from_mcu(plan.mcu.tick(plan._obs_to_mcu(plan.observation)))
        self.assertEqual(set(cmds), set(JOINTS))
        self.assertTrue(all(c.feedforward_torque == 0.0 and c.maximum_torque == 0.0 for c in cmds.values()))
        self.assertFalse(plan.run(1.0))                 # the torque-free robot falls and is detected

    def test_transform_down_refused(self):
        from wbr.controllers.diablo_arch.host_link import RobotMode
        plan = make_plan()
        plan.run(0.5)
        plan.host.set_command(mode_mark=True, stand_mode=False)   # DIABLO teleop 'v'/'b' keys before 'z'
        plan.run(0.3)
        self.assertEqual(plan.mcu.mode, RobotMode.STAND)          # no auxiliary wheels: CAR is refused (review B3)
        self.assertTrue(any("transform down ignored" in e for _, e in plan.mcu.events))

    def test_balances_on_without_the_host(self):
        """A lost host link stops the driving and the turning, the robot keeps balancing (ros2_ws/README.md 4.4)."""
        plan = make_plan()
        self.assertTrue(plan.run(0.5, v=0.3))
        plan.host.enabled = False
        self.assertTrue(plan.run(3.0, v=0.3))       # the commands no longer reach the MCU
        self.assertFalse(plan.link.in_control)
        self.assertTrue(plan.mcu.armed)
        self.assertLess(abs(mean_last(plan, "vx", 0.5)), 0.05)
        self.assertUpright(plan, 3.0)

    def test_slope_edits_the_model(self):
        plan = make_plan(slope_deg=5.0)
        gid = mujoco.mj_name2id(plan.sim.model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        normal = plan.sim.data.geom_xmat[gid].reshape(3, 3)[:, 2]
        self.assertAlmostEqual(normal[0], -math.sin(math.radians(5.0)), places=6)     # floor rises with +x


# ----------------------------------------------------------------------------- MCU guards (noise on, 10 ms latency)
def guarded_plan(rate: float = 1000.0) -> PlanV5:
    plan = make_plan(rate_hz=rate, noise=True, latency_ms=10.0)
    plan.reset()
    return plan


def all_torques_finite(plan) -> bool:
    L = plan.log
    return all(np.all(np.isfinite(L[k])) for k in ("tau_lw", "tau_rw", "tau_lh", "tau_lk", "tau_rh", "tau_rk"))


class SensorGuardTests(unittest.TestCase):
    def _inject(self, plan, key, n_steps, joint=None):
        """Corrupt the observation the MCU is about to see for n_steps consecutive steps."""
        for _ in range(n_steps):
            if key == "gyro":
                plan.observation["imu"]["angular_velocity"] = np.array([float("nan"), 0.0, 0.0])
            else:
                plan.observation["servo"][joint]["position"] = float("nan")
            plan.step(0.0, 0.0)

    def test_single_nan_sample_is_held_without_fault(self):
        for rate in (1000.0, 200.0):
            plan = guarded_plan(rate)
            self.assertTrue(plan.run(1.0))
            self._inject(plan, "gyro", 1)
            self._inject(plan, "enc", 1, joint="left_wheel")
            self.assertTrue(plan.run(1.0), f"{rate} Hz: fell after a single NaN sample")
            self.assertFalse(plan.mcu.fault, f"{rate} Hz: a single bad sample must be held, not faulted")
            self.assertTrue(all_torques_finite(plan))
            self.assertTrue(math.isfinite(plan.mcu.est.pitch) and math.isfinite(plan.law.ctrl.z))
            self.assertLess(abs(math.degrees(mean_last(plan, "pitch_true"))), 1.0)

    def test_persistent_nan_faults_with_event_and_zero_torque(self):
        for rate in (1000.0, 200.0):
            plan = guarded_plan(rate)
            self.assertTrue(plan.run(1.0))
            self._inject(plan, "gyro", 8)                  # > max_bad_samples (5)
            self.assertTrue(plan.mcu.fault, f"{rate} Hz")
            msgs = [msg for _, msg in plan.mcu.events if "non-finite sensor" in msg]
            self.assertTrue(msgs and "imu.angular_velocity" in msgs[0], plan.mcu.events)   # names the channel (r3 n1)
            self.assertEqual(plan.mcu.last_bad_channel, "imu.angular_velocity")
            self.assertTrue(all_torques_finite(plan))
            plan.step(0.0, 0.0)
            cmds = plan.mcu.tick(plan._obs_to_mcu(plan.observation))
            self.assertTrue(all(c.feedforward_torque == 0.0 for c in cmds.values()))   # safe state: torque-free

    def test_stale_time_stamp_faults(self):
        plan = guarded_plan(1000.0)
        self.assertTrue(plan.run(0.5))
        obs = plan._obs_to_mcu(plan.observation)
        for _ in range(40):                                  # 40 ms of frozen data: held, no fault yet
            plan.mcu.tick(obs)
        self.assertFalse(plan.mcu.fault)
        self.assertTrue(plan.mcu.est_hold)
        for _ in range(20):                                  # > stale_fault_s = 50 ms
            plan.mcu.tick(obs)
        self.assertTrue(plan.mcu.fault)
        self.assertTrue(any("stale" in msg for _, msg in plan.mcu.events), plan.mcu.events)

    def test_hip_and_knee_nan_are_held_without_fault(self):
        """Review r2 M1: a NaN hip / knee position must not reach the VMC."""
        for rate in (1000.0, 200.0):
            for joint in ("left_hip", "right_knee"):
                plan = guarded_plan(rate)
                self.assertTrue(plan.run(0.5))
                self._inject(plan, "enc", 1, joint=joint)
                self.assertTrue(plan.run(1.0), f"{rate} Hz {joint}: fell after one NaN position")
                self.assertFalse(plan.mcu.fault, f"{rate} Hz {joint}: a single NaN position must be held")
                self.assertTrue(all_torques_finite(plan))
                self._inject(plan, "enc", 8, joint=joint)
                self.assertTrue(plan.mcu.fault)                  # persistent -> FAULT
                self.assertTrue(all_torques_finite(plan))

    def test_held_samples_keep_the_velocity_time_base(self):
        """Stale / invalid samples advance the 5 ms position window: after 9 held ticks the next valid sample's
        velocity is dp / (n dt), not dp / dt."""
        plan = guarded_plan(1000.0)
        obs = plan._obs_to_mcu(plan.observation)
        plan.mcu.tick(obs)                                     # first valid sample
        for _ in range(9):
            plan.mcu.tick(obs)                                 # same time stamp: held
        obs2 = copy.deepcopy(obs)
        obs2["time"] = obs["time"] + 0.010
        obs2["servo"]["left_wheel"]["position"] += 0.010       # 10 mrad in 10 ms = 1 rad/s true
        plan.mcu.tick(obs2)
        v = plan.mcu.est.v_wheel["left"] / plan.mcu.p.wheel_radius
        self.assertLess(abs(v), 3.0, f"windowed wheel velocity {v:.2f} rad/s: time base lost")

    def test_startup_delay_line_does_not_trip_the_guards(self):
        for rate in (1000.0, 200.0):
            plan = guarded_plan(rate)
            self.assertTrue(plan.run(0.5))
            self.assertFalse(plan.mcu.fault)
            self.assertEqual(plan.mcu.bad_samples, 0)

    def test_held_sample_does_not_alias_the_callers_dict(self):
        """Review r3 m1: the held servo sample must be a copy at both rates."""
        for rate in (1000.0, 200.0):
            plan = guarded_plan(rate)
            self.assertTrue(plan.run(0.5))
            obs = plan._obs_to_mcu(plan.observation)
            plan.mcu.tick(obs)                                           # valid sample -> held as servo_valid
            self.assertIsNot(plan.mcu.servo_valid, obs["servo"], f"{rate} Hz: servo_valid aliases the input")
            self.assertIsNot(plan.mcu.servo_valid["left_hip"], obs["servo"]["left_hip"])
            for joint in ("left_hip", "right_knee"):
                obs["servo"][joint]["position"] = float("nan")           # in-place corruption of the SAME dict
                obs["time"] += plan.mcu.p.dt
                cmds = plan.mcu.tick(obs)
                self.assertFalse(plan.mcu.fault, f"{rate} Hz {joint}: in-place NaN reached the VMC")
                self.assertTrue(all(math.isfinite(c.feedforward_torque) for c in cmds.values()))
                self.assertEqual(plan.mcu.last_bad_channel, f"servo.{joint}.position")
                obs["servo"][joint]["position"] = plan.mcu.servo_valid[joint]["position"]   # repaired sample
                obs["time"] += plan.mcu.p.dt
                plan.mcu.tick(obs)
                self.assertEqual(plan.mcu.bad_samples, 0)


MOTION_FIELDS = ("forward", "left", "up", "roll", "pitch", "leg_split")


class HostCommandGuardTests(unittest.TestCase):
    """Review r2 M2: the MCU must not act on a MotionCtrl with a non-finite field (both rates, all six fields; r3 n3)."""

    def _nan_packets(self, plan, field, n):
        """n garbage packets, one right before each control step."""
        from wbr.controllers.diablo_arch.host_link import MotionCtrl
        for _ in range(n):
            plan.link.receive(MotionCtrl(**{field: float("nan")}), plan.sim.time)
            plan.step(0.0, 0.0)

    def test_single_nan_packet_is_rejected_and_the_robot_stands(self):
        for rate in (1000.0, 200.0):
            for field in MOTION_FIELDS:
                plan = guarded_plan(rate)
                self.assertTrue(plan.run(0.5))
                self._nan_packets(plan, field, 1)
                self.assertEqual(plan.link.rejected, 1, f"{rate} Hz {field}")
                self.assertTrue(plan.run(1.0), f"{rate} Hz: fell after a NaN {field} packet")
                self.assertFalse(plan.mcu.fault, field)
                for v in (plan.mcu.L_ref.value, plan.mcu.v_ref, plan.mcu.yaw_rate_ref, plan.mcu.roll_ref, plan.mcu.pitch_ref):
                    self.assertTrue(math.isfinite(v), f"{rate} Hz {field}: non-finite reference")
                self.assertTrue(all_torques_finite(plan))
                self.assertTrue(any("rejected" in msg for _, msg in plan.link.events), field)
                self.assertTrue(any("rejected" in msg for _, msg in plan.mcu.events), field)
                self.assertLess(abs(math.degrees(mean_last(plan, "pitch_true"))), 1.0)

    def test_sustained_nan_packets_fall_back_to_the_disconnect_program(self):
        for rate in (1000.0, 200.0):
            for field in ("forward", "pitch"):                    # the two fields that toppled the robot in round 2
                plan = guarded_plan(rate)
                self.assertTrue(plan.run(0.5))
                plan.host.enabled = False                         # only our garbage packets reach the link
                n = int(round(0.7 * rate))
                self._nan_packets(plan, field, n)                 # 0.7 s of garbage, one packet per control step
                self.assertGreaterEqual(plan.link.rejected, n)
                self.assertFalse(plan.link.in_control)            # 0.5 s timeout -> disconnect program
                self.assertFalse(plan.mcu.fault)
                self.assertTrue(plan.run(0.5), f"{rate} Hz {field}: fell under sustained garbage")
                self.assertLess(abs(math.degrees(mean_last(plan, "pitch_true"))), 1.5)
                self.assertTrue(all_torques_finite(plan))

    def test_rejected_packet_events_are_bounded(self):
        """Review r3 n2: one event for the first rejected packet, then one per 50 (the link's rule), not one per packet."""
        plan = guarded_plan(1000.0)
        self.assertTrue(plan.run(0.2))
        plan.host.enabled = False
        self._nan_packets(plan, "forward", 120)
        n_ev = sum(1 for _, msg in plan.mcu.events if "rejected" in msg)
        self.assertGreaterEqual(n_ev, 1)
        self.assertLessEqual(n_ev, 3)                             # 1st, 50th, 100th

    def test_law_input_fault_reaches_the_mcu(self):
        """The last line of defence: a non-finite balance-law input behind every sensor / command guard holds the
        output for max_bad_samples steps and then raises the MCU FAULT (both rates)."""
        for rate in (1000.0, 200.0):
            plan = guarded_plan(rate)
            self.assertTrue(plan.run(0.5))
            good = plan.law.theta_eq
            plan.law.theta_eq = lambda L0: float("nan")           # corrupts theta inside LQI8Law.compute
            for _ in range(3):
                plan.step(0.0, 0.0)
            self.assertFalse(plan.mcu.fault, f"{rate} Hz: faulted before max_bad_samples")
            self.assertEqual(plan.law.ctrl.nonfinite_steps, 3)
            for _ in range(5):
                plan.step(0.0, 0.0)
            self.assertTrue(plan.mcu.fault, f"{rate} Hz: no FAULT after 8 non-finite law inputs")
            self.assertTrue(any("balance-law" in msg for _, msg in plan.mcu.events), plan.mcu.events)
            self.assertTrue(all_torques_finite(plan))
            plan.law.theta_eq = good


class ImuGuardTests(unittest.TestCase):
    def test_mcu_never_sees_ground_truth(self):
        plan = guarded_plan(1000.0)
        self.assertNotIn("sim", plan._obs_to_mcu(plan.observation))

    def test_roll_fault(self):
        # the roll comes from the MCU's attitude filter: put its state at 45 deg of roll (a sample with a rolled quaternion
        # would only come through with the filter's 5 s time constant)
        plan = make_plan()
        plan.run(0.2)
        a = math.radians(45.0)
        plan.mcu.attitude.up = np.array([0.0, math.sin(a), math.cos(a)])     # the world's up in the base frame, rolled
        obs = plan._obs_to_mcu(plan.observation)
        plan.mcu.tick(obs)
        self.assertGreater(abs(plan.mcu.est.roll), plan.mcu.p.fall_roll)
        self.assertTrue(plan.mcu.fault)
        self.assertTrue(any("roll" in msg for _, msg in plan.mcu.events))


# ----------------------------------------------------------------------------- mechanisms of the outer loops
def spy_vmc(plan, record: list):
    """Wrap plan.leg.vmc_torques so that every call's (F, (tau_hip, tau_knee)) lands in record; returns the original."""
    vmc = plan.leg.vmc_torques

    def spy(F, Tp, q_hip, q_knee):
        out = vmc(F, Tp, q_hip, q_knee)
        record.append((F, out))
        return out

    plan.leg.vmc_torques = spy
    return vmc


class WheelReactionTests(unittest.TestCase):
    """The knee cancels the wheel motor's reaction on the shank (DiabloMcu, 11-06)."""

    def test_knee_gets_the_shank_share_of_the_wheel_torque(self):
        plan = make_plan()
        for _ in range(400):
            plan.step(0.3)
        record = []
        vmc = spy_vmc(plan, record)
        cmds = plan.mcu.tick(plan._obs_to_mcu(plan.observation))
        plan.leg.vmc_torques = vmc
        self.assertEqual(len(record), 2)                # left, then right
        sv = plan.mcu.servo_valid
        for (F, (tau_hip, tau_knee)), side in zip(record, ("left", "right")):
            wheel = cmds[f"{side}_wheel"].feedforward_torque
            self.assertGreater(abs(wheel), 1e-3)
            ratio = plan.leg.wheel_reaction_ratio(sv[f"{side}_hip"]["position"], sv[f"{side}_knee"]["position"])
            self.assertAlmostEqual(cmds[f"{side}_hip"].feedforward_torque, tau_hip, places=9)      # the hip has no such term
            self.assertAlmostEqual(cmds[f"{side}_knee"].feedforward_torque, tau_knee + ratio * wheel, places=9)


class CompliantRollTests(unittest.TestCase):
    """Compliant roll: the legs have no spring against each other, the IMU roll levels the body."""

    def test_legs_have_no_spring_against_each_other(self):
        # each leg's own damper (kd_leg_own, a gain) is off here: a step in the knee reading is a step in that leg's rate
        plan = make_plan(kd_leg_own=0.0)
        for _ in range(50):
            plan.step(0.0)
        record = []
        vmc = spy_vmc(plan, record)
        obs = copy.deepcopy(plan._obs_to_mcu(plan.observation))
        obs["servo"]["left_knee"]["position"] += 0.02          # one leg 2 mm longer, the other 2 mm shorter (no roll)
        obs["servo"]["right_knee"]["position"] -= 0.02
        plan.mcu.tick(obs)
        plan.leg.vmc_torques = vmc
        (F_left, _), (F_right, _) = record
        # a length spring per leg (kp_leg 1.5 kN/m) would give about 6 N here; only the static force of each leg's own
        # length and lean differs (SerialPlant.static_leg_force: 0.34 N)
        self.assertLess(abs(F_left - F_right), 1.0)

    def test_roll_command_is_reached_with_the_leg_difference_of_the_track(self):
        plan = make_plan()
        self.assertTrue(plan.run(0.5))
        plan.host.set_command(roll=0.05)
        self.assertTrue(plan.run(3.0))
        n = int(round(0.5 / plan.dt))
        self.assertAlmostEqual(float(np.mean(plan.log["roll"][-n:])), 0.05, delta=0.005)
        diff = float(np.mean(plan.log["L0_left"][-n:] - plan.log["L0_right"][-n:]))
        d = plan.sim.data
        tires = abs(d.geom("left_tire").xpos[1] - d.geom("right_tire").xpos[1])     # the model's, not the MCU's track constant
        self.assertAlmostEqual(diff, tires * math.tan(0.05), delta=0.15 * tires * 0.05)

    def test_soft_walls_only_act_outside_the_range_and_damp_outwards(self):
        mcu = make_plan().mcu
        p = mcu.p
        self.assertGreater(p.L_min_margin, 0.0)
        self.assertGreater(p.kp_leg_limit, 0.0)
        low = p.L_min - p.L_min_margin
        self.assertEqual(mcu._leg_limit_force(0.5 * (p.L_min + p.L_max), 1.0), 0.0)
        self.assertEqual(mcu._leg_limit_force(p.L_min - 0.5 * p.L_min_margin, -1.0), 0.0)   # the margin below L_min is free
        high = p.L_max + p.L_max_margin
        self.assertAlmostEqual(mcu._leg_limit_force(high + 0.01, 0.0), -0.01 * p.kp_leg_limit)
        self.assertAlmostEqual(mcu._leg_limit_force(high + 0.01, 0.1), -0.01 * p.kp_leg_limit - 0.1 * p.kd_leg_limit)
        self.assertAlmostEqual(mcu._leg_limit_force(high + 0.01, -0.1), -0.01 * p.kp_leg_limit)
        self.assertAlmostEqual(mcu._leg_limit_force(low - 0.01, -0.1), 0.01 * p.kp_leg_limit + 0.1 * p.kd_leg_limit)


class YawWheelDampingTests(unittest.TestCase):
    """McuParams.kw_yaw: the differential wheel torque also acts on the wheels' own speed difference."""

    def test_on_for_v5(self):
        self.assertGreater(make_plan().mcu.p.kw_yaw, 0.0)

    def test_wheel_speed_difference_reaches_the_differential_torque(self):
        plan = make_plan(kw_yaw=2.0, vel_window_s=0.0)      # no position window: the driver velocity goes straight in
        for _ in range(50):
            plan.step(0.0)
        obs = copy.deepcopy(plan._obs_to_mcu(plan.observation))
        plan.mcu.tick(copy.deepcopy(obs))
        before = plan.mcu.telemetry["tau_yaw"]
        obs["time"] += plan.dt
        obs["servo"]["left_wheel"]["velocity"] += 1.0       # left wheel faster: the robot turns right, torque to turn back left
        plan.mcu.tick(obs)
        p = plan.mcu.p
        self.assertAlmostEqual(plan.mcu.telemetry["tau_yaw"] - before, p.kw_yaw * p.wheel_radius / p.track, delta=0.05 * p.kw_yaw * p.wheel_radius / p.track)

    def test_yaw_reference_is_ramped(self):
        plan = make_plan(yaw_ref_rate=3.0)
        for _ in range(50):
            plan.step(0.0)
        for _ in range(100):
            plan.step(0.0, 1.0)
        self.assertAlmostEqual(plan.mcu.yaw_ref_limited, 3.0 * 0.1, delta=0.03)


if __name__ == "__main__":
    unittest.main()
