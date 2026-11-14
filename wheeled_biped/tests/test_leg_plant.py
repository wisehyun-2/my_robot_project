"""Serial leg, three-body plant, LQI8 reference shaping and the acceptance metrics on the v5 robot's parameters, without
MuJoCo (this module runs where the mujoco package cannot be loaded).

Moved here on 2026-11-14 from tests/test_plan4.py and tests/test_plan4_safety.py, which ran the same checks on the upkie
model of Plan 4 (removed that day).  The upkie-specific numbers (docs/plan4_model.md) are replaced by properties that hold
for any robot or by the v5 robot's plant check (results/wheelbiped_v5_model).  The checks that need the simulator are in
tests/test_runtime_v5.py.

Run from wheeled_biped/:  python -m unittest tests.test_leg_plant
"""
from __future__ import annotations

import functools
import json
import math
import unittest
from types import SimpleNamespace

import numpy as np

from wbr import MODELS_DIR, RESULTS_DIR
from wbr.controllers.lqi8.law import LQI8Law
from wbr.controllers.lqi8.lqi8 import LQI8
from wbr.controllers.lqi8.plant_serial import SerialPlant, SerialPlantParams
from wbr.controllers.lqi8.serial_leg import SerialLeg, SerialLegGeometry, du, u

V5_DIR = MODELS_DIR / "wheelbiped_v5"
PLANT_CHECK = RESULTS_DIR / "wheelbiped_v5_model" / "linearization.json"   # python -m wbr.models.verify_plan4_plant


@functools.lru_cache(maxsize=1)
def v5_params() -> SerialPlantParams:
    return SerialPlantParams.from_json(V5_DIR / "plant_params.json")


@functools.lru_cache(maxsize=1)
def v5_plant() -> SerialPlant:
    return SerialPlant(v5_params())


def v5_leg(knee_sign: float | None = None) -> SerialLeg:
    p = v5_params()
    ks = p.extra["knee_sign"] if knee_sign is None else knee_sign
    return SerialLeg(SerialLegGeometry(L1=p.extra["L1"], L2=p.extra["L2"], knee_sign=ks))


def leg_lengths() -> tuple:
    """Lowest, a middle, standing and highest commanded leg length of the v5 robot."""
    p = v5_params()
    return (p.L_min, 0.5 * (p.L_min + p.L0), p.L0, p.L_max)


# ----------------------------------------------------------------------------- serial leg
class SerialLegTests(unittest.TestCase):
    POSTURES = ((-0.5127, 0.8847), (0.3, -0.9), (-1.0, 1.6), (0.15, 0.45), (0.9, -1.8), (0.4027, -0.8054))

    def test_forward_inverse_round_trip_both_knee_sides(self):
        for ks in (-1.0, +1.0):                      # -1: knee in front, +1: knee behind (v5)
            leg = v5_leg(ks)
            for L0 in (0.30, 0.37, 0.43, 0.46, 0.485):
                for th in (-0.3, -0.06, 0.0, 0.061, 0.25):
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
        leg = v5_leg()
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
        leg = v5_leg()
        for L0 in leg_lengths():
            for th in (0.0, 0.035, 0.13):
                q = leg.inverse(L0, th)
                tau_h, tau_k = leg.vmc_torques(25.0, 0.0, *q)
                self.assertAlmostEqual(tau_h, 0.0, places=9)        # the hip turns the whole leg: no work along it
                self.assertGreater(abs(tau_k), 0.5)                  # the knee carries the leg force
                tau_h, _ = leg.vmc_torques(0.0, 1.7, *q)
                self.assertAlmostEqual(tau_h, 1.7, places=9)        # a pure hip torque is the hip joint torque
        qh, qk = leg.inverse(v5_params().L0, 0.0)
        _, tau_k = leg.vmc_torques(25.0, 0.0, qh, qk)
        self.assertLess(tau_k * qk, 0.0)      # extending = straightening the knee: the torque turns q_knee towards 0

    def test_vmc_is_power_consistent_with_virtual_rates(self):
        leg = v5_leg()
        rng = np.random.default_rng(1)
        for qh, qk in self.POSTURES:
            for _ in range(3):
                F, Tp = rng.uniform(-40.0, 40.0), rng.uniform(-3.0, 3.0)
                qd = rng.uniform(-3.0, 3.0, 2)
                tau = np.array(leg.vmc_torques(F, Tp, qh, qk))
                Ld, thd = leg.virtual_rates(qh, qk, *qd)
                self.assertAlmostEqual(float(tau @ qd), F * Ld + Tp * thd, places=9)

    def test_inverse_clips_at_the_singular_postures(self):
        leg = v5_leg()
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
        p = v5_params()
        self.assertLess(p.L0, g.L0_max - 0.01)                          # standing length keeps away from it
        self.assertLess(p.L_max, g.L0_max)

    def test_wheel_reaction_ratio_is_the_shank_share_of_the_knee_motion(self):
        eps = 1e-5
        for L1, L2, ks in ((0.25, 0.25, +1.0), (0.175, 0.175, -1.0), (0.168, 0.175, -1.0), (0.20, 0.15, +1.0)):
            leg = SerialLeg(SerialLegGeometry(L1=L1, L2=L2, knee_sign=ks))
            for L0 in (0.8 * (L1 + L2), 0.92 * (L1 + L2)):
                for th in (0.0, -0.06, 0.2):
                    qa, qb = leg.inverse(L0 - eps, th), leg.inverse(L0 + eps, th)     # virtual leg angle held
                    shank = (sum(qb) - sum(qa)) / (qb[1] - qa[1])                     # d(q_hip + q_knee) / d(q_knee)
                    q = leg.inverse(L0, th)
                    ratio = leg.wheel_reaction_ratio(*q)
                    self.assertAlmostEqual(ratio, shank, places=6)
                    # the shank and the thigh shares add up to one; vmc_torques gives the thigh share for Tp
                    self.assertAlmostEqual(ratio + leg.vmc_torques(0.0, 1.0, *q)[1], 1.0, places=9)
                    if L1 == L2:
                        self.assertAlmostEqual(ratio, 0.5, places=9)


# ----------------------------------------------------------------------------- three-body plant
class SerialPlantTests(unittest.TestCase):
    def test_theta_eq_table_matches_exact_bisection(self):
        plant, p = v5_plant(), v5_params()
        grid = p.leg.L0
        mids = [0.5 * (a + b) for a, b in zip(grid[:-1], grid[1:]) if p.L_min <= 0.5 * (a + b) <= p.L_max]
        for L0 in mids[::3] + [p.L0]:
            self.assertLess(abs(plant.theta_eq(L0) - plant.theta_eq_exact(L0)), 1e-4, f"L0={L0}")
            self.assertLess(abs(plant.com_x(plant.theta_eq_exact(L0), L0)), 1e-9)    # total COM above the axle
        ths = [plant.theta_eq(L0) for L0 in leg_lengths()]
        # knees behind: the leg's mass is behind the hip -> axle line, so the hip leans forward over the wheel, less so
        # the straighter the leg (+7.4 deg at the lowest length, +2.0 deg at the highest)
        self.assertTrue(all(t > 0.0 for t in ths), [math.degrees(t) for t in ths])
        self.assertTrue(all(a > b for a, b in zip(ths, ths[1:])), [math.degrees(t) for t in ths])

    def test_static_hip_torque_is_the_body_weight_moment(self):
        p, plant = v5_params(), v5_plant()
        for L0 in [float(x) for x in p.leg.L0 if p.L_min <= x <= p.L_max][::4]:     # points of the leg-table grid
            self.assertAlmostEqual(plant.Tp_eq(L0), p.m_b * p.g * p.b_x, places=12)

    def test_static_hip_torque_between_grid_points(self):
        """Tp_eq() off the 5 mm theta_eq grid: the static hip torque is evaluated at the exact bisection equilibrium,
        so the equilibrium consistency check must hold between grid points."""
        p, plant = v5_params(), v5_plant()
        self.assertAlmostEqual(plant.Tp_eq(p.L0 + 0.0025), p.m_b * p.g * p.b_x, places=9)

    def test_linearization_poles_and_input_signs(self):
        p = v5_params()
        A, B = v5_plant().linearize(p.L0)
        eig = np.linalg.eigvals(A)
        unstable = eig[eig.real > 0.5]
        self.assertEqual(len(unstable), 2, f"open-loop poles {np.sort_complex(eig)}")
        self.assertLess(np.abs(unstable.imag).max(), 1e-9)                         # real, divergent modes
        if PLANT_CHECK.exists():                                                    # the plant of the plant check
            rec = next(r for r in json.loads(PLANT_CHECK.read_text(encoding="utf-8")) if abs(r["L0"] - p.L0) < 1e-6)
            np.testing.assert_allclose(np.sort(unstable.real), np.sort(rec["unstable_poles"]["plant"]), rtol=0.005)
        self.assertGreater(B[3, 0], 0.0)     # x_ddot per wheel torque: positive T drives forward
        self.assertGreater(B[5, 1], 0.0)     # phi_ddot per hip torque: reaction -Tp pitches the body back (phi > 0)
        np.testing.assert_array_equal(A[[0, 2, 4]], np.eye(6)[[1, 3, 5]])         # kinematic rows
        np.testing.assert_array_equal(B[[0, 2, 4]], 0.0)

    def test_linearization_matches_the_nonlinear_model(self):
        plant, p = v5_plant(), v5_params()
        L = float(p.leg.L0[np.argmin(np.abs(p.leg.L0 - p.L0))])        # a grid point (theta_eq is exact there)
        X0, Tp0 = plant.equilibrium_state(L), plant.Tp_eq(L)
        np.testing.assert_allclose(plant.deriv(X0, 0.0, Tp0, L), 0.0, atol=1e-9)    # it is an equilibrium
        A, B = plant.linearize(L)
        eps = 1e-6
        f = lambda X, T, Tp: plant.deriv(X, T, Tp, L)
        An = np.column_stack([(f(X0 + eps * e, 0.0, Tp0) - f(X0 - eps * e, 0.0, Tp0)) / (2 * eps) for e in np.eye(6)])
        Bn = np.column_stack([(f(X0, eps, Tp0) - f(X0, -eps, Tp0)) / (2 * eps), (f(X0, 0.0, Tp0 + eps) - f(X0, 0.0, Tp0 - eps)) / (2 * eps)])
        np.testing.assert_allclose(An, A, atol=1e-6 * np.abs(A).max())
        np.testing.assert_allclose(Bn, B, atol=1e-6 * np.abs(B).max())


# ----------------------------------------------------------------------------- LQI8 reference shaping and guards
def shape(ctrl: LQI8, v_ref: float, seconds: float, L0: float) -> np.ndarray:
    """Step the controller with the robot held at the equilibrium (zero state); rows (v_ramp, v_filt, z)."""
    n = int(round(seconds / ctrl.Ts))
    out = np.zeros((n, 3))
    for k in range(n):
        ctrl.step(0.0, 0.0, 0.0, 0.0, 0.0, L0, v_ref)
        out[k] = (ctrl.v_ramp, ctrl.v_filt, ctrl.z)
    return out


# the runtime's LQI8 settings (wbr.plans.plan4_lqi8.Plan4LQI8.__init__) with an online design instead of the gain table
RUNTIME_LQI = dict(table_L0=None, q_int=0.2, q_v=0.5, r_hip_frac=0.3, r_wheel_frac=0.2, v_ramp_rate=2.0, v_ref_tau=0.3,
                   freeze_int_on_ramp=True, freeze_tol=0.05, q_att=0.2, q_pos=0.5)


def runtime_lqi8(Ts: float = 1e-3) -> LQI8:
    """The LQI8 the way the runtime builds it (Plan4LQI8._make_law: LQI8Law fills in the static tilt's g M(L0))."""
    plant = v5_plant()
    return LQI8Law(plant, Ts=Ts, theta_eq=plant.theta_eq, Tp_ff=plant.Tp_eq(), **RUNTIME_LQI).ctrl


class LQI8ReferenceTests(unittest.TestCase):
    def test_reference_shaping_ramp_prefilter_and_frozen_integrator(self):
        Ts, L0 = 1e-3, v5_params().L0
        ctrl = LQI8(v5_plant(), Ts=Ts, q_int=0.3, r_hip_frac=0.3, r_wheel_frac=0.2, table_L0=None,
                    v_ramp_rate=2.0, v_ref_tau=0.3, freeze_int_on_ramp=True, v_ref_tau_brake=0.0)
        ramp, filt, z = shape(ctrl, 0.5, 3.0, L0).T
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
        ramp_d, filt_d, _ = shape(ctrl, 0.0, 0.5, L0).T
        np.testing.assert_array_equal(filt_d, ramp_d)
        n_brake = int(round(0.5 / (2.0 * Ts)))
        np.testing.assert_allclose(np.diff(np.r_[ramp[-1], ramp_d])[:n_brake - 1], -2.0 * Ts, atol=1e-12)
        self.assertEqual(ramp_d[-1], 0.0)

    def test_shaped_reference_never_overshoots_whatever_the_settings(self):
        plant, L0 = v5_plant(), v5_params().L0
        variants = [runtime_lqi8(1e-3), runtime_lqi8(5e-3),                                       # the runtime's settings
                    LQI8(plant, Ts=1e-3, table_L0=None, v_ramp_rate=2.0, v_ref_tau=0.15),
                    LQI8(plant, Ts=1e-3, table_L0=None, v_ramp_rate=4.0)]                    # ramp only
        for ctrl in variants:
            ctrl.reset()
            filt = shape(ctrl, 0.5, 3.0, L0)[:, 1]
            ctrl.reset()
            self.assertLessEqual(filt.max(), 0.5)
            self.assertTrue(np.all(np.diff(filt) >= 0.0))
            self.assertGreater(filt[-1], 0.45)

    def test_reset_restores_the_mcus_own_ramp(self):
        ctrl = runtime_lqi8()
        self.assertEqual(ctrl.v_ramp_rate_base, 2.0)
        ctrl.v_ramp_rate = 0.8                  # the law lowers it over uneven ground (McuParams.uneven_brake, law.py)
        ctrl.reset()
        self.assertEqual(ctrl.v_ramp_rate, ctrl.v_ramp_rate_base)

    def test_tilt_integrator_waits_while_the_reference_accelerates(self):
        """z_att (the static tilt) must not integrate while the reference accelerates (the lean of the acceleration would
        read as a tilt), nor for the 0.1 s hold-off after it (r2 m4); z (the velocity integral) is frozen by the ramp."""
        L0 = v5_params().L0
        ctrl = runtime_lqi8()
        ctrl.reset()
        for _ in range(100):                    # a 0.5 m/s step: the 2 m/s^2 ramp runs for 0.25 s
            ctrl.step(0.02, 0.0, 0.0, 0.0, 0.0, L0, 0.5)
        self.assertEqual(ctrl.z_att, 0.0)
        self.assertEqual(ctrl.z, 0.0)

    def test_law_counts_nonfinite_inputs(self):
        L0 = v5_params().L0
        ctrl = runtime_lqi8()
        ctrl.reset()
        for _ in range(3):
            ctrl.step(float("nan"), 0.0, 0.0, 0.0, 0.0, L0, 0.0)
        self.assertEqual(ctrl.nonfinite_steps, 3)
        ctrl.step(0.0, 0.0, 0.0, 0.0, 0.0, L0, 0.0)
        self.assertEqual(ctrl.nonfinite_steps, 0)
        from wbr.common.filters import Ramp
        r = Ramp(1.0, value=0.3)
        self.assertEqual(r.step(float("nan"), 0.01), 0.3)        # NaN target ignored
        self.assertEqual(r.step(float("inf"), 0.01), 0.3)


# ----------------------------------------------------------------------------- host emulator
class HostTests(unittest.TestCase):
    def test_emulator_publishes_on_a_fixed_grid(self):
        from wbr.controllers.diablo_arch.host_link import HostEmulator, HostLink, HostLinkParams
        link = HostLink(HostLinkParams())
        host = HostEmulator(link, publish_hz=50)
        times = []
        orig = link.receive

        def rec(cmd, t):
            times.append(t)
            return orig(cmd, t)

        link.receive = rec
        host.set_command(forward=0.3)
        for k in range(1, 2001):
            host.tick(k * 1e-3)
        self.assertIn(len(times), (100, 101))                # first packet at the first tick, then every 20.000 ms
        self.assertLess(np.abs(np.diff(times)[1:] - 0.02).max(), 1e-9)


# ----------------------------------------------------------------------------- acceptance metrics (wbr.bench.acceptance)
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

    def test_settle_requires_a_hold(self):
        from wbr.bench.acceptance import settle_time_true
        t = np.arange(0.0, 6.0, 1e-3)
        late = {"t": t, "pitch_true": np.where(t < 5.5, math.radians(5.0), 0.0), "vx": np.zeros_like(t)}
        self.assertIsNone(settle_time_true(late, 0.0))        # last violation 0.5 s before the end: not settled
        ok = {"t": t, "pitch_true": np.where(t < 4.9, math.radians(5.0), 0.0), "vx": np.zeros_like(t)}
        self.assertAlmostEqual(settle_time_true(ok, 0.0), 4.9, delta=0.03)

    def test_torque_metrics_on_a_synthetic_log(self):
        from wbr.bench.acceptance import torque_metrics
        dt, n = 1e-3, 1000
        lw = np.full(n, 0.5)
        lw[:100] = 10.0                                             # left wheel at 10 N m for 100 ms
        rw = np.zeros(n)
        rw[50:150] = -10.0                                          # right wheel for 100 ms, 50 ms overlap
        log = {"t": np.arange(n) * dt, "tau_lw": lw, "tau_rw": rw,
               "tau_lh": np.full(n, 2.0), "tau_lk": np.full(n, -3.0), "tau_rh": np.full(n, 1.0), "tau_rk": np.zeros(n),
               "qd_lw": np.full(n, 10.0), "qd_rw": np.full(n, -12.0),
               "qd_lh": np.zeros(n), "qd_lk": np.full(n, 0.5), "qd_rh": np.full(n, -1.5), "qd_rk": np.zeros(n)}
        plan = SimpleNamespace(log=log, dt=dt)                      # duck-typed plan
        m = torque_metrics(plan)
        self.assertEqual(set(m), {"wheel_peak", "wheel_rms10", "leg_peak", "leg_rms", "wheel_speed_peak", "leg_speed_peak"})
        self.assertAlmostEqual(m["wheel_peak"], 10.0)
        self.assertAlmostEqual(m["wheel_rms10"], math.sqrt((100 * 10.0 ** 2 + 900 * 0.25) / 1000))
        self.assertAlmostEqual(m["leg_peak"], 3.0)
        self.assertAlmostEqual(m["leg_rms"], 3.0)                 # worst single leg motor (review r1 m6), not pooled
        self.assertAlmostEqual(m["wheel_speed_peak"], 12.0)
        self.assertAlmostEqual(m["leg_speed_peak"], 1.5)
        late = torque_metrics(plan, t_from=0.5)
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
        checks = [le, ge, Check("wheel_rms10", "<=", 2.5), Check("leg_margin_mm", ">=", 5.0),
                  Check("knee_flipped", "<=", 0), Check("alive", ">=", 1)]
        for c in checks:
            self.assertIsNone(c.margin(None))
            self.assertFalse(c.passes(None))
            self.assertFalse(c.passes(float("nan")))
            for v in (0.0, 0.05, 0.199, 0.2, 0.25, 1, 1.0, 1.5, 2.0, 2.5, 3.0, 4.9, 5.0, 30.0):
                # margin <= 1 <=> passes (the report shows the largest margin of a test as its tightest metric:
                # a passed check must never look worse than the limit, e.g. a 0 <= 0 limit or a passed flag)
                self.assertEqual(c.passes(v), c.margin(v) <= 1.0, f"{c.metric} {c.op} {c.limit}: value {v}, margin {c.margin(v)}")


if __name__ == "__main__":
    unittest.main()
