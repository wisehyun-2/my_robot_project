"""Plan 4 review round 1 regression tests: sensor-data guards (NaN / stale -> hold, then FAULT), the real-robot IMU
convention adapter, the integrator-free MPC prediction model, acceptance metric definitions, the host publish grid
and the MPC specification checks."""
import math
import unittest

import numpy as np

from wbr.controllers.diablo_arch.host_link import HostEmulator, HostLink, HostLinkParams
from wbr.plans.plan4_upkie_lqi6 import Plan4Config, Plan4UpkieLQI6


def make_plan(rate=1000.0, **kw):
    cfg = Plan4Config(rate_hz=rate, noise=True, latency_ms=10.0, **kw)
    plan = Plan4UpkieLQI6(cfg)
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
            plan = make_plan(rate)
            self.assertTrue(plan.run(1.0))
            self._inject(plan, "gyro", 1)
            self._inject(plan, "enc", 1, joint="left_wheel")
            self.assertTrue(plan.run(1.0), f"{rate} Hz: fell after a single NaN sample")
            self.assertFalse(plan.mcu.fault, f"{rate} Hz: a single bad sample must be held, not faulted")
            self.assertTrue(all_torques_finite(plan))
            self.assertTrue(math.isfinite(plan.mcu.est.pitch) and math.isfinite(plan.law.ctrl.z))
            self.assertLess(abs(math.degrees(plan.log["pitch_true"][-1])), 1.0)

    def test_persistent_nan_faults_with_event_and_zero_torque(self):
        for rate in (1000.0, 200.0):
            plan = make_plan(rate)
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
        plan = make_plan(1000.0)
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
        """Review r2 M1: a NaN hip / knee position must not reach the VMC (it used the raw observation and produced a
        NaN torque -> immediate FAULT -> fall)."""
        for rate in (1000.0, 200.0):
            for joint in ("left_hip", "right_knee"):
                plan = make_plan(rate)
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
        velocity is dp / (n dt), not dp / dt (the 10x start-up spike that broke the 10 deg tilt wheel peak)."""
        import copy
        plan = make_plan(1000.0)
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
            plan = make_plan(rate)
            self.assertTrue(plan.run(0.5))
            self.assertFalse(plan.mcu.fault)
            self.assertEqual(plan.mcu.bad_samples, 0)

    def test_held_sample_does_not_alias_the_callers_dict(self):
        """Review r3 m1: at 200 Hz the 5 ms window is one step and the held servo sample used to be the caller's
        dict itself, so an entry point that updates ONE observation dict in place handed a NaN hip position straight
        to the VMC (immediate FAULT, the round-2 M1 symptom).  The held sample must be a copy at both rates."""
        for rate in (1000.0, 200.0):
            plan = make_plan(rate)
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
        """n garbage packets, one right before each control step; the steps are logged like any other (the standing
        checks below therefore see the state AFTER the injection)."""
        from wbr.controllers.diablo_arch.host_link import MotionCtrl
        for _ in range(n):
            plan.link.receive(MotionCtrl(**{field: float("nan")}), plan.sim.time)
            plan.step(0.0, 0.0)

    def test_single_nan_packet_is_rejected_and_the_robot_stands(self):
        for rate in (1000.0, 200.0):
            for field in MOTION_FIELDS:
                plan = make_plan(rate)
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
                self.assertLess(abs(math.degrees(plan.log["pitch_true"][-1])), 1.0)

    def test_sustained_nan_packets_fall_back_to_the_disconnect_program(self):
        for rate in (1000.0, 200.0):
            for field in ("forward", "pitch"):                    # the two fields that toppled the robot in round 2
                plan = make_plan(rate)
                self.assertTrue(plan.run(0.5))
                plan.host.enabled = False                         # only our garbage packets reach the link
                n = int(round(0.7 * rate))
                self._nan_packets(plan, field, n)                 # 0.7 s of garbage, one packet per control step
                self.assertGreaterEqual(plan.link.rejected, n)
                self.assertFalse(plan.link.in_control)            # 0.5 s timeout -> disconnect program
                self.assertFalse(plan.mcu.fault)
                self.assertTrue(plan.run(0.5), f"{rate} Hz {field}: fell under sustained garbage")
                self.assertLess(abs(math.degrees(plan.log["pitch_true"][-1])), 1.5)
                self.assertTrue(all_torques_finite(plan))

    def test_rejected_packet_events_are_bounded(self):
        """Review r3 n2: one event for the first rejected packet, then one per 50 (the link's rule), not one per packet."""
        plan = make_plan(1000.0)
        self.assertTrue(plan.run(0.2))
        plan.host.enabled = False
        self._nan_packets(plan, "forward", 120)
        n_ev = sum(1 for _, msg in plan.mcu.events if "rejected" in msg)
        self.assertGreaterEqual(n_ev, 1)
        self.assertLessEqual(n_ev, 3)                             # 1st, 50th, 100th

    def test_law_faults_after_persistent_nonfinite_inputs(self):
        plan = make_plan(1000.0)
        ctrl = plan.law.ctrl
        ctrl.reset()
        for _ in range(3):
            ctrl.step(float("nan"), 0.0, 0.0, 0.0, 0.0, plan.params.L0, 0.0)
        self.assertEqual(ctrl.nonfinite_steps, 3)
        ctrl.step(0.0, 0.0, 0.0, 0.0, 0.0, plan.params.L0, 0.0)
        self.assertEqual(ctrl.nonfinite_steps, 0)
        from wbr.common.filters import Ramp
        r = Ramp(1.0, value=0.3)
        self.assertEqual(r.step(float("nan"), 0.01), 0.3)        # NaN target ignored
        self.assertEqual(r.step(float("inf"), 0.01), 0.3)

    def test_law_input_fault_reaches_the_mcu(self):
        """The last line of defence: a non-finite balance-law input behind every sensor / command guard holds the
        output for max_bad_samples steps and then raises the MCU FAULT (both rates)."""
        for rate in (1000.0, 200.0):
            plan = make_plan(rate)
            self.assertTrue(plan.run(0.5))
            good = plan.law.theta_eq
            plan.law.theta_eq = lambda L0: float("nan")           # corrupts theta inside LQI6Law.compute
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


class ImuConventionTests(unittest.TestCase):
    def test_adapter_round_trip_equals_world_frame_estimate(self):
        """The simulator feeds the MCU the IMU the way upkie's spine reports it (ARS frame, IMU rotated 180 deg about
        y); the MCU's BaseOrientation adapter must recover exactly the world-frame base estimate (review r1 M3)."""
        import mujoco
        from wbr.controllers.diablo_arch.mcu import DiabloMcu, McuParams
        plan = Plan4UpkieLQI6(Plan4Config(rate_hz=1000.0, noise=False, latency_ms=0.0, real_imu_convention=True))
        plan.reset(pitch=0.1, yaw=0.4)
        plan.sim.data.qvel[3:6] = [0.2, 0.3, 1.0]           # body-frame angular velocity of the free joint
        mujoco.mj_forward(plan.sim.model, plan.sim.data)
        obs_conv = plan._observe()                            # ARS / IMU-frame sample
        plan_w = Plan4UpkieLQI6(Plan4Config(rate_hz=1000.0, noise=False, latency_ms=0.0, real_imu_convention=False))
        raw = plan.sim.observe()                              # world-frame sample of the same state
        self.assertIsNone(plan_w.mcu.imu)
        self.assertIsNotNone(plan.mcu.imu)
        e1 = plan.mcu.estimate(plan._obs_to_mcu(obs_conv))
        e2 = plan_w.mcu.estimate(plan_w._obs_to_mcu(raw))
        for k in ("pitch", "roll", "pitch_rate", "roll_rate", "yaw_rate"):
            self.assertAlmostEqual(getattr(e1, k), getattr(e2, k), places=9, msg=k)
        self.assertAlmostEqual(abs(e1.pitch), 0.1, delta=0.02)
        self.assertAlmostEqual(abs(e1.yaw_rate), 1.0, delta=0.05)
        # the converted sample really is in another frame (not a no-op)
        self.assertGreater(np.abs(np.asarray(obs_conv["imu"]["orientation"]) - np.asarray(raw["imu"]["orientation"])).max(), 0.5)

    def test_mcu_never_sees_ground_truth(self):
        plan = make_plan(1000.0)
        self.assertNotIn("sim", plan._obs_to_mcu(plan.observation))

    def test_roll_fault(self):
        from wbr.controllers.diablo_arch.mcu import RobotMode
        plan = make_plan(1000.0)
        plan.run(0.2)
        obs = plan._obs_to_mcu(plan.observation)
        from wbr.common.rotations import matrix_to_quat, rot_x
        R_roll = rot_x(math.radians(45.0))
        # express the rolled base the way the plan's IMU emulation would
        obs["imu"]["orientation"] = matrix_to_quat(plan.R_ARS_TO_WORLD.T @ R_roll @ plan.R_BASE_TO_IMU.T)
        obs["time"] = plan.observation["time"] + 1.0
        plan.mcu.tick(obs)
        self.assertTrue(plan.mcu.fault)
        self.assertTrue(any("roll" in msg for _, msg in plan.mcu.events))


class PredictionModelTests(unittest.TestCase):
    def test_manoeuvre_model_is_integrator_free_and_stable(self):
        from wbr.controllers.mpc_host import model as M
        plan = Plan4UpkieLQI6(Plan4Config(rate_hz=1000.0, noise=False, latency_ms=10.0))
        opts = M.lqi6_options(plan.law.ctrl, plan.params.L0)
        self.assertFalse(opts["use_int"])
        self.assertIsNone(opts["att_gM"])
        self.assertFalse(opts["use_pos"])
        self.assertEqual(opts["gain_cols"], [0, 1, 2, 3, 4])
        K = M._gains_no_side_effect(plan.law.ctrl, plan.params.L0)
        self.assertEqual(K.shape, (2, 8))
        self.assertEqual(M.select_gains(K, opts).shape, (2, 5))
        cl = M.plan_closed_loop(plan)
        self.assertIsNone(cl.info["i_z"])
        self.assertLess(np.abs(np.linalg.eigvals(cl.A)).max(), 1.0)
        # a unit velocity reference settles near v = 1 without an integrator (the viscous joint damping of the plant
        # needs a steady torque, hence a small steady error that the MCU integrator removes once the manoeuvre is over)
        dc = cl.dc_gain()
        self.assertAlmostEqual(float(dc[cl.iy("v"), cl.iu("r")]), 1.0, delta=0.05)
        with self.assertRaises(ValueError):
            M.lqi6_closed_loop(plan.plant, K, plan.params.L0, plan.law.ctrl.Ts, 10, use_int=False)   # wrong K shape

    def test_plain_mode_freezes_integrators_while_accelerating(self):
        from wbr.controllers.lqi6.lqi6 import LQI6
        plan = Plan4UpkieLQI6(Plan4Config(rate_hz=1000.0, noise=False, latency_ms=0.0))
        ctrl: LQI6 = plan.law.ctrl
        ctrl.reset()
        ctrl.set_reference_shaping(False)
        # host-shaped reference accelerating at 0.8 m/s^2 with a constant velocity error: once the low-passed reference
        # acceleration (tau 0.1 s) is above the threshold (~15 ms) z must not move
        for k in range(50):
            r = 0.8 * k * ctrl.Ts
            ctrl.step(0.0, 0.0, r - 0.05, 0.0, 0.0, plan.params.L0, r)
        z0 = ctrl.z
        for k in range(50, 300):
            r = 0.8 * k * ctrl.Ts
            ctrl.step(0.0, 0.0, r - 0.05, 0.0, 0.0, plan.params.L0, r)
        self.assertAlmostEqual(ctrl.z, z0, places=12)
        # constant reference afterwards: the integrator resumes once the low-passed acceleration (tau 0.1 s) has
        # decayed below the threshold and the 0.1 s hold-off (r2 m4) has elapsed
        for _ in range(800):
            ctrl.step(0.0, 0.0, r - 0.05, 0.0, 0.0, plan.params.L0, r)
        self.assertLess(ctrl.z, z0 - 1e-3)
        ctrl.reset()
        self.assertTrue(ctrl.shaping_on)                     # reset restores the MCU's own shaping (n7)


class AcceptanceDefinitionTests(unittest.TestCase):
    def test_settle_requires_a_hold(self):
        from wbr.bench.acceptance import settle_time_true
        t = np.arange(0.0, 6.0, 1e-3)
        late = {"t": t, "pitch_true": np.where(t < 5.5, math.radians(5.0), 0.0), "vx": np.zeros_like(t)}
        self.assertIsNone(settle_time_true(late, 0.0))        # last violation 0.5 s before the end: not settled
        ok = {"t": t, "pitch_true": np.where(t < 4.9, math.radians(5.0), 0.0), "vx": np.zeros_like(t)}
        self.assertAlmostEqual(settle_time_true(ok, 0.0), 4.9, delta=0.03)

    def test_packet_violation_counter(self):
        from wbr.bench.acceptance_mpc import packet_violations
        T, a_max, j_max = 0.02, 1.0, 5.0
        pk = [(k * T, min(0.5, 0.5 * a_max * (k * T) ** 2) if k < 30 else 0.5, True) for k in range(60)]
        # a smooth parabola-then-constant: |a| <= 0.6 (k<30) fine, but the acceleration step at k = 30 exceeds j_max T
        n = packet_violations(pk, a_max, j_max, T)
        self.assertGreaterEqual(n, 1)
        ramp = [(k * T, 0.5 * a_max * (k * T) ** 2 * 0 + min(0.5, 0.5 * k * T), True) for k in range(60)]   # a = 0.5 <= a_max
        self.assertEqual(packet_violations(ramp, a_max, j_max, T), 1)   # only the kink where the ramp stops (jerk 25 > 5)
        self.assertEqual(packet_violations([(0.0, 0.0, False), (0.02, 0.5, False)], a_max, j_max, T), 0)   # fallback packets
        # velocity bound (r2 m9 / r3 n1): every packet of a host-shaped sequence, including its first one
        fast = [(k * T, 1.2, True) for k in range(3)]
        self.assertEqual(packet_violations(fast, a_max, j_max, T), 0)
        self.assertEqual(packet_violations(fast, a_max, j_max, T, v_max=1.0), 3)
        self.assertEqual(packet_violations([(0.0, 0.0, True), (T, 0.0, True), (2 * T, 1.2, False), (3 * T, 1.2, True),
                                            (4 * T, 1.2, True)], a_max, j_max, T, v_max=1.0), 2)   # new sequence at 3T

    def test_stand_items_check_alive_and_stop_distance_is_the_excursion(self):
        from wbr.bench import acceptance as A
        self.assertIn("alive", [c.metric for c in A.STAND_CHECKS])
        it12 = A._item(12)
        self.assertIn("alive", [c.metric for c in it12.checks])
        self.assertIn("wheel_sat_duty", [c.metric for c in it12.checks])
        self.assertIn("packet_violations", [c.metric for c in A._item(21).checks])
        self.assertIn("passes_8_transient", [c.metric for c in A._item(23).checks])
        self.assertEqual(A._item(24).grade, "record")


class MpcHostTests(unittest.TestCase):
    def test_command_latency_is_a_pure_reference_delay(self):
        """Review r3 m1: the real robot's host -> MCU command latency is a delay of the reference, not of the inner
        loop's feedback (modelled that way, 10 ms made the inner loop 'unstable' and switched the MPC off)."""
        from wbr.controllers.mpc_host import reference_mpc as RM
        for rate in (1000.0, 200.0):
            plan = Plan4UpkieLQI6(Plan4Config(rate_hz=rate, noise=False, latency_ms=10.0))
            cfg0, cfg = RM.MpcConfig(), RM.MpcConfig(model_extra_latency_ms=20.0)
            m0 = RM.prediction_model(plan, 0.02, cfg0.order)
            m1 = RM.prediction_model(plan, 0.02, cfg.order, extra_latency_ms=20.0)
            m2 = RM.prediction_model(plan, 0.02, cfg.order, extra_latency_ms=10.0)
            self.assertTrue(m1["stable"] and m2["stable"], f"{rate} Hz: delayed prediction model reported unstable")
            self.assertEqual(m1["full"].n, m0["full"].n + int(round(0.02 * rate)))
            # 20 ms = one host period: the lifted full-order response is the undelayed one shifted by one period
            U = np.zeros((40, 2))
            U[2:, 0] = 0.5
            Y0, _ = m0["full"].simulate(U)
            Y1, _ = m1["full"].simulate(U)
            self.assertLess(np.abs(Y1[1:, :2] - Y0[:-1, :2]).max(), 1e-9, f"{rate} Hz")
            lim0, lim = RM.derive_limits(plan, cfg0), RM.derive_limits(plan, cfg)
            self.assertAlmostEqual(lim["a_max"], lim0["a_max"], delta=0.01 * lim0["a_max"], msg=f"{rate} Hz: a_max moved")
            q0, _, _ = RM.reference_qp(plan, cfg0, 0.02)
            q1, _, _ = RM.reference_qp(plan, cfg, 0.02)
            self.assertIsNot(q0, q1)                              # the QP cache distinguishes the two models

    def test_handover_waits_until_the_robot_has_stopped(self):
        """Review r3 m2: a stop from 0.8 m/s is not over 1.0 s after the target returned to zero.  The hand-over to the
        inner loop (prediction dropped, braking direction no longer from the prediction) used to happen while the body
        still rolled at 0.17-0.37 m/s and a few sent packets then exceeded a_max."""
        from wbr.bench.acceptance_mpc import _capture_packets, packet_violations
        plan = Plan4UpkieLQI6(Plan4Config(rate_hz=200.0, noise=True, latency_ms=10.0, host="mpc"))
        plan.reset()
        packets = _capture_packets(plan)
        self.assertTrue(plan.run(0.5))
        self.assertTrue(plan.run(2.5, v=0.8))
        self.assertTrue(plan.run(3.5, v=0.0))
        st = plan.host.stats()
        self.assertTrue(plan.host.model_ok)
        self.assertEqual(packet_violations(packets, st["a_max"], st["j_max"], plan.host.period, a_dec=st["a_dec_max"],
                                           v_max=st["v_max"]), 0)
        t_h = [t for t, msg in plan.host.events if "handover" in msg]
        self.assertTrue(t_h, plan.host.events)
        L, tr = plan.log, plan.host.trace
        y_at_handover = float(np.interp(t_h[-1], tr["t"], np.nan_to_num(np.asarray(tr["v_avg"], float))))
        v_at_handover = float(np.interp(t_h[-1], L["t"], L["vx"]))
        self.assertLessEqual(abs(y_at_handover), plan.host.cfg.obs_hold_v + 1e-6, f"telemetry speed {y_at_handover:.3f} at hand-over")
        # the torso still rocks a little relative to the wheels at that moment (base speed != wheel speed); before the
        # fix the hand-over happened at 0.17-0.37 m/s
        self.assertLess(abs(v_at_handover), 0.12, f"handed over at {t_h[-1]:.2f} s while rolling at {v_at_handover:.3f} m/s")
        self.assertGreater(t_h[-1], 3.0 + plan.host.cfg.obs_hold_s + 0.5 * plan.host.period)   # later than the fixed 1.0 s
        self.assertLess(abs(L["vx"][-1]), 0.05)


class HostTests(unittest.TestCase):
    def test_emulator_publishes_on_a_fixed_grid(self):
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

    def test_mpc_host_rejects_out_of_spec_configuration(self):
        from wbr.controllers.mpc_host.reference_mpc import MpcHost
        plan = Plan4UpkieLQI6(Plan4Config(rate_hz=1000.0, noise=False, latency_ms=10.0))
        with self.assertRaises(ValueError):
            MpcHost(plan.link, plan, publish_hz=25.0)
        with self.assertRaises(ValueError):
            MpcHost(plan.link, plan, publish_hz=50.0, horizon_s=3.0)

    def test_target_deadband_and_send_fields(self):
        plan = Plan4UpkieLQI6(Plan4Config(rate_hz=1000.0, noise=False, latency_ms=10.0, host="mpc"))
        plan.reset()
        plan.host.set_command(forward=1e-4, roll=0.2, leg_split=0.1)
        self.assertEqual(plan.host._target(), 0.0)
        plan.run(0.1)
        sent = plan.host.sent
        self.assertEqual(sent.roll, 0.0)
        self.assertEqual(sent.leg_split, 0.0)
        self.assertEqual(sent.pitch, 0.0)
        self.assertFalse(sent.mode.pitch_ctrl_mode)


if __name__ == "__main__":
    unittest.main()
