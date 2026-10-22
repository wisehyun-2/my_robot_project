"""Unit tests of Plan 4: upkie serial hip-knee legs + LQI6 + DIABLO-style MCU.

Run from the project root:  python -m unittest discover -s tests -v   (this module: ~20 s)

The tests check properties that must hold whatever the controller gains are: kinematic identities of the
serial leg, sign conventions of the MuJoCo model, the derived three-body plant against MuJoCo, closed-loop
stability with the weights the plan actually runs (read from Plan4UpkieLQI6().law.ctrl), invariants of the
velocity reference shaping, short closed-loop smoke runs (noise off) and the acceptance-battery helpers.
"""
from __future__ import annotations

import copy
import functools
import math
import unittest
from types import SimpleNamespace

import mujoco
import numpy as np

from wbr import MODELS_DIR
from wbr.common.servo import ServoCommand, ServoModel, ServoProperties, torque_command
from wbr.controllers.lqi6.lqi6 import LQI6, c2d_zoh
from wbr.controllers.lqi6.plant_serial import SerialPlant, SerialPlantParams
from wbr.controllers.lqi6.serial_leg import UPKIE_JOINT_SIGNS, SerialLeg, SerialLegGeometry, du, u
from wbr.models.upkie_params import set_leg_posture

PLAN4_DIR = MODELS_DIR / "upkie_plan4"
SCENE = PLAN4_DIR / "scene.xml"
JOINTS = ("left_hip", "left_knee", "left_wheel", "right_hip", "right_knee", "right_wheel")
WHEELS = ("left_wheel", "right_wheel")
L_STAND = 0.31          # standing virtual leg length of the model documentation (docs/plan4_model.md)


# ----------------------------------------------------------------------------- shared fixtures
@functools.lru_cache(maxsize=1)
def plan4_params() -> SerialPlantParams:
    return SerialPlantParams.from_json(PLAN4_DIR / "plan4_params.json")


@functools.lru_cache(maxsize=1)
def plan4_plant() -> SerialPlant:
    return SerialPlant(plan4_params())


def plan4_leg(knee_sign: float | None = None) -> SerialLeg:
    p = plan4_params()
    ks = p.extra["knee_sign"] if knee_sign is None else knee_sign
    return SerialLeg(SerialLegGeometry(L1=p.extra["L1"], L2=p.extra["L2"], knee_sign=ks))


def make_plan(**cfg):
    from wbr.plans.plan4_upkie_lqi6 import Plan4Config, Plan4UpkieLQI6
    return Plan4UpkieLQI6(Plan4Config(**cfg))


def plan4_lqi6(rate_hz: float) -> LQI6:
    """The LQI6 instance the Plan 4 assembly runs at this rate (its weights may be retuned at any time)."""
    ctrl = getattr(make_plan(rate_hz=rate_hz).law, "ctrl", None)
    if isinstance(ctrl, LQI6):
        return ctrl
    # fallback if the plan stops exposing its LQI6: the documented Plan 4 weights
    return LQI6(plan4_plant(), Ts=1.0 / rate_hz, q_int=0.3, r_hip_frac=0.3, r_wheel_frac=0.2, table_L0=(0.22, 0.34, 0.005))


def augmented_discrete(ctrl, L0: float):
    """Discrete closed-loop model of an LQI6 instance, rebuilt here from the plant linearisation and the documented
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


# ----------------------------------------------------------------------------- serial leg
class SerialLegTests(unittest.TestCase):
    POSTURES = ((-0.5127, 0.8847), (0.3, -0.9), (-1.0, 1.6), (0.15, 0.45), (0.9, -1.8))

    def test_forward_inverse_round_trip_both_knee_sides(self):
        for ks in (-1.0, +1.0):                      # -1: Plan 4 (knee in front), +1: knee behind
            leg = plan4_leg(ks)
            for L0 in (0.20, 0.25, 0.31, 0.335):
                for th in (-0.3, -0.0607, 0.0, 0.25):
                    qh, qk = leg.inverse(L0, th)
                    p, L0b, thb = leg.forward(qh, qk)
                    self.assertAlmostEqual(L0b, L0, places=9)
                    self.assertAlmostEqual(thb, th, places=9)
                    self.assertAlmostEqual(float(np.linalg.norm(p)), L0, places=9)
                    # du(theta_v) points "backwards" relative to the virtual leg: the knee is in front of the
                    # hip -> axle line (component < 0) for knee_sign -1 and behind it for +1
                    knee = leg.g.L1 * u(qh)
                    self.assertEqual(float(np.sign(knee @ du(th))), ks, f"knee side, ks={ks} L0={L0} th={th}")
                    self.assertEqual(float(np.sign(qk)), -ks)

    def test_jacobian_and_virtual_rates_match_finite_differences(self):
        leg = plan4_leg()
        eps = 1e-6
        for qh, qk in self.POSTURES:
            J = leg.jacobian(qh, qk)
            Jn = np.column_stack([(leg.forward(qh + eps, qk)[0] - leg.forward(qh - eps, qk)[0]) / (2 * eps),
                                  (leg.forward(qh, qk + eps)[0] - leg.forward(qh, qk - eps)[0]) / (2 * eps)])
            self.assertLess(np.abs(J - Jn).max(), 1e-8, f"jacobian at q=({qh}, {qk})")
            for qdh, qdk in ((0.7, -1.3), (-2.0, 0.4)):
                Ld, thd = leg.virtual_rates(qh, qk, qdh, qdk)
                _, La, tha = leg.forward(qh + eps * qdh, qk + eps * qdk)
                _, Lb, thb = leg.forward(qh - eps * qdh, qk - eps * qdk)
                self.assertAlmostEqual(Ld, (La - Lb) / (2 * eps), places=6)
                self.assertAlmostEqual(thd, (tha - thb) / (2 * eps), places=6)
                self.assertAlmostEqual(leg.shank_rate(qdh, qdk), qdh + qdk)    # shank angle t2 = q_hip + q_knee

    def test_vmc_pure_leg_force_and_pure_hip_torque(self):
        leg = plan4_leg()
        for L0 in (0.24, 0.31, 0.335):
            for th in (0.0, -0.0607, 0.2):
                q = leg.inverse(L0, th)
                tau_h, tau_k = leg.vmc_torques(25.0, 0.0, *q)
                self.assertAlmostEqual(tau_h, 0.0, places=9)        # the hip turns the whole leg: no work along it
                self.assertGreater(abs(tau_k), 0.5)                  # the knee carries the leg force
                tau_h, _ = leg.vmc_torques(0.0, 1.7, *q)
                self.assertAlmostEqual(tau_h, 1.7, places=9)        # a pure hip torque is the hip joint torque
        tau_h, tau_k = leg.vmc_torques(25.0, 0.0, *leg.inverse(L_STAND, 0.0))
        self.assertLess(tau_k, 0.0)       # knee in front (q_knee > 0): extending = straightening = negative torque

    def test_vmc_is_power_consistent_with_virtual_rates(self):
        leg = plan4_leg()
        rng = np.random.default_rng(1)
        for qh, qk in self.POSTURES:
            for _ in range(3):
                F, Tp = rng.uniform(-40.0, 40.0), rng.uniform(-3.0, 3.0)
                qd = rng.uniform(-3.0, 3.0, 2)
                tau = np.array(leg.vmc_torques(F, Tp, qh, qk))
                Ld, thd = leg.virtual_rates(qh, qk, *qd)
                self.assertAlmostEqual(float(tau @ qd), F * Ld + Tp * thd, places=9)

    def test_inverse_clips_at_the_singular_postures(self):
        leg = plan4_leg()
        g = leg.g
        for L0 in (g.L0_max - 1e-4, g.L0_max, g.L0_max + 0.02):        # full extension: q_knee = 0 is singular
            qh, qk = leg.inverse(L0, 0.05)
            self.assertTrue(math.isfinite(qh) and math.isfinite(qk))
            _, L0b, thb = leg.forward(qh, qk)
            self.assertAlmostEqual(L0b, g.L0_max - 1e-3, places=9)
            self.assertAlmostEqual(thb, 0.05, places=9)
            self.assertGreater(abs(qk), 0.1)                            # the knee stays bent ...
            self.assertGreater(abs(np.linalg.det(leg.jacobian(qh, qk))), 1e-3)   # ... and the Jacobian full rank
        qh, qk = leg.inverse(0.0, 0.0)                                  # folded leg (L0 < |L1 - L2|)
        self.assertTrue(math.isfinite(qh) and math.isfinite(qk))
        self.assertAlmostEqual(leg.forward(qh, qk)[1], g.L0_min + 1e-4, places=6)
        self.assertLess(plan4_params().L0, g.L0_max - 0.01)            # standing length keeps away from it

    def test_standing_posture_within_joint_limits(self):
        p, plant, leg = plan4_params(), plan4_plant(), plan4_leg()
        for L0 in (p.L_min, p.L0, L_STAND, p.L_max):
            q = leg.inverse(L0, plant.theta_eq(L0))
            self.assertTrue(leg.within_limits(*q, margin=0.2), f"L0={L0}: q={q}")
        qh, qk = leg.inverse(L_STAND, plant.theta_eq(L_STAND))
        self.assertAlmostEqual(qh, -0.513, delta=0.005)                 # docs/plan4_model.md: q = (-0.513, +0.885)
        self.assertAlmostEqual(qk, 0.885, delta=0.005)
        self.assertFalse(leg.within_limits(1.3, 0.0))
        self.assertFalse(leg.within_limits(0.0, -2.6))
        self.assertFalse(leg.within_limits(-1.2, 0.5, margin=0.1))


# ----------------------------------------------------------------------------- MuJoCo joint signs
class UpkieJointSignTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.m = mujoco.MjModel.from_xml_path(str(SCENE))
        cls.d = mujoco.MjData(cls.m)
        cls.base = mujoco.mj_name2id(cls.m, mujoco.mjtObj.mjOBJ_BODY, "base")

    def jid(self, name: str) -> int:
        return mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, name)

    def pose(self, which: str) -> None:
        m, d = self.m, self.d
        if which == "keyframe":            # straight legs, base level
            mujoco.mj_resetDataKeyframe(m, d, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "stand"))
            mujoco.mj_forward(m, d)
        else:                              # Plan 4 standing posture, base level
            set_leg_posture(m, d, plan4_leg(), L_STAND, plan4_plant().theta_eq(L_STAND), wheel_radius=plan4_params().R)

    def axle_in_base(self, side: str) -> np.ndarray:
        R = self.d.xmat[self.base].reshape(3, 3)
        return R.T @ (self.d.xanchor[self.jid(f"{side}_wheel")] - self.d.xpos[self.base])

    def test_signed_joint_axes_are_plus_y(self):
        # u-convention = rotation about +y for every joint (leg end backwards / wheel rolling forward)
        for which in ("keyframe", "standing"):
            self.pose(which)
            for n in JOINTS:
                axis = UPKIE_JOINT_SIGNS[n] * self.d.xaxis[self.jid(n)]
                np.testing.assert_allclose(axis, [0.0, 1.0, 0.0], atol=1e-9, err_msg=f"{which}: {n}")

    def test_positive_u_convention_moves_the_axle_backwards(self):
        for which in ("keyframe", "standing"):
            for side, other in (("left", "right"), ("right", "left")):
                for joint in ("hip", "knee"):
                    name = f"{side}_{joint}"
                    self.pose(which)
                    a0, b0 = self.axle_in_base(side), self.axle_in_base(other)
                    self.d.qpos[self.m.jnt_qposadr[self.jid(name)]] += 0.1 * UPKIE_JOINT_SIGNS[name]
                    mujoco.mj_forward(self.m, self.d)
                    a1, b1 = self.axle_in_base(side), self.axle_in_base(other)
                    self.assertLess(a1[0] - a0[0], -0.01, f"{which}: +0.1 rad (u-convention) on {name} must move the axle back")
                    self.assertLess(np.abs(b1 - b0).max(), 1e-12, f"{which}: {name} moved the other leg")

    def test_serial_leg_kinematics_match_the_model(self):
        p, leg = plan4_params(), plan4_leg()
        for L0, th in ((L_STAND, plan4_plant().theta_eq(L_STAND)), (0.25, 0.1), (0.335, -0.2)):
            qh, qk = set_leg_posture(self.m, self.d, leg, L0, th, wheel_radius=p.R)
            p_fk, _, _ = leg.forward(qh, qk)
            for side in ("left", "right"):
                rel = self.d.xanchor[self.jid(f"{side}_wheel")] - self.d.xanchor[self.jid(f"{side}_hip")]
                np.testing.assert_allclose(rel[[0, 2]], p_fk, atol=1e-9, err_msg=f"{side} L0={L0}")

    def test_wheel_signs_roll_forward(self):
        m, d = self.m, self.d
        for name in WHEELS:
            self.pose("standing")
            d.qvel[:] = 0.0
            d.qvel[m.jnt_dofadr[self.jid(name)]] = UPKIE_JOINT_SIGNS[name] * 1.0
            mujoco.mj_forward(m, d)
            v = np.zeros(6)
            mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, m.jnt_bodyid[self.jid(name)], v, 0)
            # rolling forward (+x) on the floor = spin about +y (contact point at rest: v_x = omega_y R)
            np.testing.assert_allclose(v[:3], [0.0, 1.0, 0.0], atol=1e-9, err_msg=name)
        from wbr.sim.mujoco_sim import MujocoSim, SimConfig
        sim = MujocoSim(SimConfig(xml_path=SCENE, joint_names=list(JOINTS), wheel_joints=list(WHEELS), base_body="base",
                                  imu_site="base"), ServoModel({j: ServoProperties() for j in JOINTS}))
        self.assertEqual(sim.wheel_forward_signs(), {w: UPKIE_JOINT_SIGNS[w] for w in WHEELS})


# ----------------------------------------------------------------------------- three-body plant
class SerialPlantTests(unittest.TestCase):
    def test_theta_eq_matches_mujoco_com_bisection(self):
        p, plant, leg = plan4_params(), plan4_plant(), plan4_leg()
        m = mujoco.MjModel.from_xml_path(str(SCENE))
        d = mujoco.MjData(m)
        base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base")
        wheels = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in WHEELS]

        def com_ahead_of_axle(theta: float) -> float:
            set_leg_posture(m, d, leg, L_STAND, theta, wheel_radius=p.R)
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
        th_mujoco = 0.5 * (lo + hi)
        self.assertLess(abs(math.degrees(plant.theta_eq(L_STAND) - th_mujoco)), 0.1)
        self.assertAlmostEqual(math.degrees(plant.theta_eq(L_STAND)), -3.48, delta=0.05)     # docs/plan4_model.md

    def test_theta_eq_table_matches_exact_bisection(self):
        plant = plan4_plant()
        for L0 in (0.2425, 0.2575, 0.2875, L_STAND, 0.3125, 0.3275):
            self.assertLess(abs(plant.theta_eq(L0) - plant.theta_eq_exact(L0)), 1e-4, f"L0={L0}")
            self.assertLess(abs(plant.com_x(plant.theta_eq_exact(L0), L0)), 1e-9)    # total COM above the axle
        ths = [plant.theta_eq(L0) for L0 in (0.25, 0.28, 0.31, 0.33)]
        self.assertTrue(all(a < b for a, b in zip(ths, ths[1:])))                  # docs: -8.00 / -5.79 / -3.48 / -1.54 deg

    def test_static_hip_torque_is_the_body_weight_moment(self):
        p, plant = plan4_params(), plan4_plant()
        for L0 in (0.25, L_STAND, 0.33):            # points of the leg-table grid
            self.assertAlmostEqual(plant.Tp_eq(L0), p.m_b * p.g * p.b_x, places=12)
        self.assertAlmostEqual(plant.Tp_eq(L_STAND), -0.314, delta=0.002)          # docs/plan4_model.md

    def test_static_hip_torque_between_grid_points(self):
        """Tp_eq() off the 5 mm theta_eq grid (e.g. L_stand = 0.3125): the static hip torque is evaluated at the
        exact bisection equilibrium, so the equilibrium consistency check must hold between grid points."""
        p, plant = plan4_params(), plan4_plant()
        self.assertAlmostEqual(plant.Tp_eq(0.3125), p.m_b * p.g * p.b_x, places=9)

    def test_linearization_poles_and_input_signs(self):
        A, B = plan4_plant().linearize(L_STAND)
        eig = np.linalg.eigvals(A)
        unstable = eig[eig.real > 0.5]
        self.assertEqual(len(unstable), 2, f"open-loop poles {np.sort_complex(eig)}")
        self.assertLess(np.abs(unstable.imag).max(), 1e-9)                         # real, divergent modes
        slow, fast = np.sort(unstable.real)
        self.assertAlmostEqual(slow / 6.13, 1.0, delta=0.02)                      # docs/plan4_model.md (MuJoCo 6.125)
        self.assertAlmostEqual(fast / 9.77, 1.0, delta=0.02)                      # 9.772 with the rotor armature on the
        #                                                                           wheel joint rate (review r1 m9)
        self.assertGreater(B[3, 0], 0.0)     # x_ddot per wheel torque: positive T drives forward
        self.assertGreater(B[5, 1], 0.0)     # phi_ddot per hip torque: reaction -Tp pitches the body back (phi > 0)
        np.testing.assert_array_equal(A[[0, 2, 4]], np.eye(6)[[1, 3, 5]])         # kinematic rows
        np.testing.assert_array_equal(B[[0, 2, 4]], 0.0)

    def test_linearization_matches_the_nonlinear_model(self):
        plant = plan4_plant()
        X0, Tp0 = plant.equilibrium_state(L_STAND), plant.Tp_eq(L_STAND)
        np.testing.assert_allclose(plant.deriv(X0, 0.0, Tp0, L_STAND), 0.0, atol=1e-9)    # it is an equilibrium
        A, B = plant.linearize(L_STAND)
        eps = 1e-6
        f = lambda X, T, Tp: plant.deriv(X, T, Tp, L_STAND)
        An = np.column_stack([(f(X0 + eps * e, 0.0, Tp0) - f(X0 - eps * e, 0.0, Tp0)) / (2 * eps) for e in np.eye(6)])
        Bn = np.column_stack([(f(X0, eps, Tp0) - f(X0, -eps, Tp0)) / (2 * eps), (f(X0, 0.0, Tp0 + eps) - f(X0, 0.0, Tp0 - eps)) / (2 * eps)])
        np.testing.assert_allclose(An, A, atol=1e-6 * np.abs(A).max())
        np.testing.assert_allclose(Bn, B, atol=1e-6 * np.abs(B).max())

    def test_plant_matches_mujoco_at_the_standing_length(self):
        from wbr.models.verify_plan4_plant import MjPlanar, check_L0
        r = check_L0(MjPlanar(plan4_params()), plan4_plant(), L_STAND)
        up = r["unstable_poles"]
        self.assertIsNotNone(up["rel_err"], f"unstable pole count differs: plant {up['plant']} mujoco {up['mujoco']}")
        self.assertEqual(len(up["rel_err"]), 2)
        self.assertLess(max(up["rel_err"]), 0.10, f"unstable poles plant {up['plant']} mujoco {up['mujoco']}")
        for name, b in r["B_check"].items():
            self.assertLess(b["rel_err"], 0.10, f"B check {name}: mujoco {b['mujoco']} plant {b['plant']}")
        self.assertAlmostEqual(r["Tp_eq_mujoco"] / r["Tp_eq_plant"], 1.0, delta=0.10)


# ----------------------------------------------------------------------------- LQI6 on the serial plant
class Plan4LQI6Tests(unittest.TestCase):
    RATES = (1000.0, 200.0)

    @classmethod
    def setUpClass(cls):
        cls.ctrl = {rate: plan4_lqi6(rate) for rate in cls.RATES}

    def assertClosedLoopStable(self, ctrl, label: str):
        for L0 in (0.25, L_STAND, 0.33):
            Ad, Bd = augmented_discrete(ctrl, L0)
            for kind, K in (("table", ctrl.gains(L0).copy()), ("design", ctrl.design(L0).copy())):
                rho = float(np.abs(np.linalg.eigvals(Ad - Bd @ K)).max())
                self.assertLess(rho, 1.0, f"{label}, L0={L0}: {kind} gains unstable (rho={rho:.5f})")
        for L0 in np.linspace(0.24, 0.335, 9):              # run-time leg range, interpolated table gains
            Ad, Bd = augmented_discrete(ctrl, float(L0))
            rho = float(np.abs(np.linalg.eigvals(Ad - Bd @ ctrl.gains(float(L0)))).max())
            self.assertLess(rho, 1.0, f"{label}, L0={L0:.4f}: table gains unstable (rho={rho:.5f})")

    def assertTableMatchesDesign(self, ctrl, label: str):
        direct = copy.copy(ctrl)                            # same weights and states, online design instead of the table
        direct.table = None
        for L0 in (0.2525, 0.2825, 0.3125, 0.3275):        # grid mid-points (interpolation error)
            K_table = ctrl.gains(L0).copy()
            K_direct = direct.design(L0)
            err = np.abs(K_table - K_direct).max() / np.abs(K_direct).max()
            self.assertLess(err, 0.02, f"{label}, L0={L0}")

    def test_closed_loop_stable_with_the_plan_weights(self):
        for rate in self.RATES:
            ctrl = self.ctrl[rate]
            self.assertAlmostEqual(ctrl.Ts, 1.0 / rate, places=12)
            self.assertClosedLoopStable(ctrl, f"plan default {rate:.0f} Hz")

    def test_gain_table_matches_direct_design(self):
        for rate in self.RATES:
            self.assertTableMatchesDesign(self.ctrl[rate], f"plan default {rate:.0f} Hz")

    def test_extra_integral_states_keep_the_loop_stable(self):
        # optional LQI6 states (static body tilt z_att, position z_pos): any positive weights must still give a
        # stabilisable design model and a stabilising, well-interpolated gain table
        import inspect
        if not {"q_att", "q_pos"} <= set(inspect.signature(LQI6.__init__).parameters):
            self.skipTest("LQI6 has no q_att / q_pos options")
        for rate in self.RATES:
            ctrl = make_plan(rate_hz=rate, lqi_kwargs=dict(q_att=0.05, q_pos=0.5)).law.ctrl
            self.assertEqual(ctrl.n, 8)
            self.assertClosedLoopStable(ctrl, f"q_att + q_pos {rate:.0f} Hz")
            self.assertTableMatchesDesign(ctrl, f"q_att + q_pos {rate:.0f} Hz")

    @staticmethod
    def shape(ctrl: LQI6, v_ref: float, seconds: float, L0: float = L_STAND) -> np.ndarray:
        """Step the controller with the robot held at the equilibrium (zero state); rows (v_ramp, v_filt, z)."""
        n = int(round(seconds / ctrl.Ts))
        out = np.zeros((n, 3))
        for k in range(n):
            ctrl.step(0.0, 0.0, 0.0, 0.0, 0.0, L0, v_ref)
            out[k] = (ctrl.v_ramp, ctrl.v_filt, ctrl.z)
        return out

    def test_reference_shaping_ramp_prefilter_and_frozen_integrator(self):
        Ts = 1e-3
        ctrl = LQI6(plan4_plant(), Ts=Ts, q_int=0.3, r_hip_frac=0.3, r_wheel_frac=0.2, table_L0=None,
                    v_ramp_rate=2.0, v_ref_tau=0.3, freeze_int_on_ramp=True, v_ref_tau_brake=0.0)
        ramp, filt, z = self.shape(ctrl, 0.5, 3.0).T
        # 2 m/s^2 ramp: 0.2 m/s after 0.1 s, 0.5 m/s from 0.25 s on, never beyond the set-point
        self.assertAlmostEqual(ramp[int(round(0.1 / Ts)) - 1], 0.2, places=9)
        self.assertEqual(ramp[-1], 0.5)
        self.assertLessEqual(ramp.max(), 0.5)
        # pre-filtered reference: lags the ramp, monotonic, never overshoots, converges
        self.assertLess(filt[0], ramp[0])
        self.assertTrue(np.all(np.diff(filt) >= 0.0))
        self.assertLessEqual(filt.max(), 0.5)
        self.assertAlmostEqual(filt[-1], 0.5, delta=1e-3)
        # integrator frozen while the reference still moves (ramp step or filter lag > 1 mm/s) ...
        moving = np.r_[True, np.abs(np.diff(ramp)) > 1e-9] | (np.abs(filt - ramp) > 1e-3)
        self.assertTrue(moving[:int(round(1.0 / Ts))].all())
        self.assertTrue(np.all(z[moving] == 0.0))
        self.assertLess(z[-1], 0.0)            # ... and integrating again once settled (v = 0 < v_ref)
        # braking: back to 0 along the ramp without the filter (the stopping distance is not stretched)
        ramp_d, filt_d, _ = self.shape(ctrl, 0.0, 0.5).T
        np.testing.assert_array_equal(filt_d, ramp_d)
        n_brake = int(round(0.5 / (2.0 * Ts)))
        np.testing.assert_allclose(np.diff(np.r_[ramp[-1], ramp_d])[:n_brake - 1], -2.0 * Ts, atol=1e-12)
        self.assertEqual(ramp_d[-1], 0.0)

    def test_shaped_reference_never_overshoots_whatever_the_settings(self):
        variants = [self.ctrl[rate] for rate in self.RATES]           # the plan's own settings
        variants.append(LQI6(plan4_plant(), Ts=1e-3, table_L0=None, v_ramp_rate=2.0, v_ref_tau=0.15, v_ref_order=2))
        variants.append(LQI6(plan4_plant(), Ts=1e-3, table_L0=None, v_ramp_rate=4.0))       # ramp only
        for ctrl in variants:
            ctrl.reset()
            filt = self.shape(ctrl, 0.5, 3.0)[:, 1]
            ctrl.reset()
            self.assertLessEqual(filt.max(), 0.5)
            self.assertTrue(np.all(np.diff(filt) >= 0.0))
            self.assertGreater(filt[-1], 0.45)


# ----------------------------------------------------------------------------- closed-loop smoke runs
class Plan4SmokeTests(unittest.TestCase):
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
        self.assertTrue(plan.fallen)

    def test_robustness_knobs_edit_the_model(self):
        plan = make_plan(slope_deg=5.0)
        gid = mujoco.mj_name2id(plan.sim.model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        normal = plan.sim.data.geom_xmat[gid].reshape(3, 3)[:, 2]
        self.assertAlmostEqual(normal[0], -math.sin(math.radians(5.0)), places=6)     # floor rises with +x
        nominal = make_plan().sim.total_mass()
        self.assertAlmostEqual(make_plan(mass_scale=1.15).sim.total_mass(), 1.15 * nominal, places=6)


# ----------------------------------------------------------------------------- sign adapter
class Plan4SignAdapterTests(unittest.TestCase):
    def test_adapter_round_trip(self):
        plan = make_plan()
        self.assertEqual(plan.signs, UPKIE_JOINT_SIGNS)
        rng = np.random.default_rng(3)
        raw = {"time": 0.0, "servo": {n: {"position": float(rng.uniform(-1, 1)), "velocity": float(rng.uniform(-5, 5)),
                                          "torque": float(rng.uniform(-2, 2))} for n in JOINTS}}
        snapshot = copy.deepcopy(raw)
        seen = plan._obs_to_mcu(raw)
        self.assertEqual(raw, snapshot)                                 # the raw observation is not modified
        for n in JOINTS:
            for k in ("position", "velocity", "torque"):
                self.assertEqual(seen["servo"][n][k], UPKIE_JOINT_SIGNS[n] * raw["servo"][n][k])
        # MCU-side commands built from the MCU-side observation map back onto the raw values
        cmds = {n: ServoCommand(position=seen["servo"][n]["position"], velocity=seen["servo"][n]["velocity"],
                                feedforward_torque=seen["servo"][n]["torque"], kp_scale=0.5, kd_scale=0.25, maximum_torque=1.5)
                for n in JOINTS}
        back = plan._cmds_from_mcu(cmds)
        for n in JOINTS:
            self.assertEqual(back[n].feedforward_torque, raw["servo"][n]["torque"])
            self.assertEqual(back[n].position, raw["servo"][n]["position"])
            self.assertEqual(back[n].velocity, raw["servo"][n]["velocity"])
            self.assertEqual((back[n].kp_scale, back[n].kd_scale, back[n].maximum_torque), (0.5, 0.25, 1.5))
            self.assertIsNot(back[n], cmds[n])
            self.assertEqual(cmds[n].feedforward_torque, seen["servo"][n]["torque"])     # input not modified
        out = plan._cmds_from_mcu({"left_hip": torque_command(1.2, 16.0)})             # torque mode: NaN position kept
        self.assertTrue(math.isnan(out["left_hip"].position))
        self.assertEqual(out["left_hip"].feedforward_torque, UPKIE_JOINT_SIGNS["left_hip"] * 1.2)

    def test_mcu_sees_the_standing_posture_in_u_convention(self):
        plan = make_plan()
        L0 = plan.mcu.p.L_stand
        qh, qk = plan.leg.inverse(L0, plan.plant.theta_eq(L0))
        raw = plan.observation["servo"]
        seen = plan._obs_to_mcu(plan.observation)["servo"]
        for side in ("left", "right"):
            self.assertAlmostEqual(seen[f"{side}_hip"]["position"], qh, places=9)
            self.assertAlmostEqual(seen[f"{side}_knee"]["position"], qk, places=9)
        self.assertAlmostEqual(raw["left_hip"]["position"], -raw["right_hip"]["position"], places=9)     # mirrored joints
        self.assertAlmostEqual(raw["left_knee"]["position"], -raw["right_knee"]["position"], places=9)
        if abs(L0 - L_STAND) < 1e-9:
            self.assertAlmostEqual(seen["left_hip"]["position"], -0.513, delta=0.002)   # docs/plan4_model.md
            self.assertAlmostEqual(seen["left_knee"]["position"], 0.885, delta=0.002)
        plan.step()
        e = plan.mcu.est
        self.assertAlmostEqual(e.L0_mean, L0, delta=1e-3)
        self.assertAlmostEqual(e.theta_v_mean, plan.plant.theta_eq(L0), delta=math.radians(0.2))
        self.assertAlmostEqual(e.theta_v["left"], e.theta_v["right"], delta=1e-6)


# ----------------------------------------------------------------------------- acceptance helpers
class AcceptanceHelperTests(unittest.TestCase):
    def test_moving_average_keeps_edge_values(self):
        from wbr.bench.acceptance import moving_average
        dt = 1e-3
        np.testing.assert_allclose(moving_average(np.full(100, 2.5), dt, 0.02), 2.5, rtol=0, atol=1e-12)
        x = np.r_[np.zeros(500), np.ones(500)]
        y = moving_average(x, dt, 0.02)
        self.assertEqual(len(y), len(x))
        self.assertEqual(y[0], 0.0)
        self.assertAlmostEqual(y[-1], 1.0, places=12)               # zero padding would give ~0.5 at the end
        self.assertTrue(np.all(y[:480] == 0.0))
        np.testing.assert_allclose(y[520:], 1.0, atol=1e-12)
        self.assertTrue(0.0 < y[500] < 1.0)                         # the step is spread over the 20 ms window
        self.assertTrue(np.all(np.diff(y) >= -1e-15))
        np.testing.assert_array_equal(moving_average(x, dt, 0.0), x)   # window shorter than a sample: identity

    def test_moving_rms_max(self):
        from wbr.bench.acceptance import moving_rms_max
        dt = 1e-3
        for n in (500, 20000):                                      # shorter and longer than the 10 s window
            self.assertAlmostEqual(moving_rms_max(np.full(n, -0.3), dt), 0.3, places=9)
        x = np.zeros(30000)
        x[12000:22000] = 2.0                                        # one 10 s burst
        self.assertAlmostEqual(moving_rms_max(x, dt), 2.0, places=9)
        self.assertAlmostEqual(moving_rms_max(x, dt, win=20.0), math.sqrt(2.0), places=9)

    def test_settle_time_true(self):
        from wbr.bench.acceptance import settle_time_true
        t = np.arange(0.0, 6.0, 1e-3)
        zeros = np.zeros_like(t)
        never = {"t": t, "pitch_true": np.full_like(t, math.radians(1.5)), "vx": zeros}
        self.assertIsNone(settle_time_true(never, 0.0))
        always = {"t": t, "pitch_true": np.full_like(t, math.radians(0.5)), "vx": np.full_like(t, 0.05)}
        self.assertEqual(settle_time_true(always, 0.0), 0.0)
        until2 = {"t": t, "pitch_true": np.where(t < 2.0, math.radians(5.0), 0.0), "vx": zeros}
        self.assertAlmostEqual(settle_time_true(until2, 0.0), 2.0, delta=0.03)
        self.assertAlmostEqual(settle_time_true(until2, 1.0), 1.0, delta=0.03)      # relative to t_start
        self.assertEqual(settle_time_true(until2, 3.0), 0.0)
        spike = dict(until2, pitch_true=until2["pitch_true"].copy())
        spike["pitch_true"][4000] = math.radians(10.0)              # one noisy sample: averaged out (0.5 deg)
        self.assertAlmostEqual(settle_time_true(spike, 0.0), 2.0, delta=0.03)
        vel = {"t": t, "pitch_true": zeros, "vx": np.where(t < 1.5, 0.0, 0.5)}
        self.assertAlmostEqual(settle_time_true(vel, 0.0, v_ref=0.5), 1.5, delta=0.03)
        self.assertIsNone(settle_time_true(vel, 0.0, v_ref=0.0))

    def test_torque_metrics_on_a_synthetic_log(self):
        from wbr.bench.acceptance import WHEEL_TAU_MAX, torque_metrics
        dt, n = 1e-3, 1000
        lw = np.full(n, 0.5)
        lw[:100] = WHEEL_TAU_MAX                                    # left wheel saturated for 100 ms
        rw = np.zeros(n)
        rw[50:150] = -WHEEL_TAU_MAX                                 # right wheel for 100 ms, 50 ms overlap
        log = {"t": np.arange(n) * dt, "tau_lw": lw, "tau_rw": rw,
               "tau_lh": np.full(n, 2.0), "tau_lk": np.full(n, -3.0), "tau_rh": np.full(n, 1.0), "tau_rk": np.zeros(n),
               "qd_lw": np.full(n, 10.0), "qd_rw": np.full(n, -12.0),
               "qd_lh": np.zeros(n), "qd_lk": np.full(n, 0.5), "qd_rh": np.full(n, -1.5), "qd_rk": np.zeros(n)}
        plan = SimpleNamespace(log=log, dt=dt)                      # duck-typed plan
        m = torque_metrics(plan)
        self.assertAlmostEqual(m["wheel_sat_duty"], 200 / 2000)    # fraction of (wheel, sample) pairs at the limit
        self.assertAlmostEqual(m["wheel_sat_s"], 0.150)            # time with at least one wheel saturated
        self.assertAlmostEqual(m["wheel_peak"], WHEEL_TAU_MAX)
        self.assertAlmostEqual(m["wheel_rms10"], math.sqrt((100 * WHEEL_TAU_MAX ** 2 + 900 * 0.25) / 1000))
        self.assertAlmostEqual(m["leg_peak"], 3.0)
        self.assertAlmostEqual(m["leg_rms"], 3.0)                 # worst single leg motor (review r1 m6), not pooled
        self.assertAlmostEqual(m["wheel_speed_peak"], 12.0)
        self.assertAlmostEqual(m["leg_speed_peak"], 1.5)
        late = torque_metrics(plan, t_from=0.5)
        self.assertEqual(late["wheel_sat_duty"], 0.0)
        self.assertEqual(late["wheel_sat_s"], 0.0)
        self.assertAlmostEqual(late["wheel_peak"], 0.5)

    def test_check_margin_semantics(self):
        from wbr.bench.acceptance import Check
        le, ge = Check("x", "<=", 2.0), Check("y", ">=", 2.0)
        self.assertAlmostEqual(le.margin(1.0), 0.5)                 # measured / limit
        self.assertAlmostEqual(le.margin(2.0), 1.0)
        self.assertAlmostEqual(le.margin(3.0), 1.5)
        self.assertAlmostEqual(ge.margin(4.0), 0.5)                 # limit / measured
        self.assertAlmostEqual(ge.margin(2.0), 1.0)
        self.assertAlmostEqual(ge.margin(1.0), 2.0)
        self.assertEqual(ge.margin(0.0), float("inf"))
        checks = [le, ge, Check("wheel_rms10", "<=", 0.2), Check("max_latency_ms", ">=", 25.0),
                  Check("wheel_sat_duty", "<=", 0.0), Check("settled", ">=", 1)]
        for c in checks:
            self.assertIsNone(c.margin(None))
            self.assertFalse(c.passes(None))
            self.assertFalse(c.passes(float("nan")))
            for v in (0.0, 0.05, 0.199, 0.2, 0.25, 1, 1.0, 1.5, 2.0, 3.0, 24.0, 25.0, 30.0):
                # margin <= 1 <=> passes (the summary reports the largest margin of an item as its tightest metric:
                # a passed check must never look worse than the limit, e.g. 0 <= 0 saturation or a passed flag)
                self.assertEqual(c.passes(v), c.margin(v) <= 1.0, f"{c.metric} {c.op} {c.limit}: value {v}, margin {c.margin(v)}")

    def test_evaluate_adds_the_torque_checks_once(self):
        from wbr.bench.acceptance import TORQUE_CHECKS, Check, Item, evaluate
        res = {"settled": 1, "wheel_peak": 1.0, "wheel_rms10": 0.3, "wheel_sat_duty": 0.0, "wheel_sat_s": 0.0, "leg_peak": 2.0,
               "leg_rms": 1.0, "wheel_speed_peak": 10.0, "leg_speed_peak": 1.0}
        it = Item(id=99, name="synthetic", grade="settle", scenario="tilt",
                  checks=[Check("settled", ">=", 1), Check("wheel_rms10", "<=", 0.5)], wheel_peak_check=False)
        ev = evaluate(it, res)
        self.assertEqual(set(ev), {"settled"} | {c.metric for c in TORQUE_CHECKS if c.metric != "wheel_peak"})
        self.assertEqual(ev["wheel_rms10"]["limit"], 0.5)          # the item's own limit wins over the default 0.2
        self.assertTrue(all(c["passed"] for c in ev.values()))
        survive = Item(id=98, name="synthetic", grade="survive", scenario="tilt", checks=[Check("alive", ">=", 1)])
        self.assertEqual(set(evaluate(survive, {"alive": 1})), {"alive"})
        self.assertFalse(any(c["passed"] for c in evaluate(it, dict(res, error="boom")).values()))


if __name__ == "__main__":
    unittest.main()
