"""
upkie_mpc.py -- runs upkie's ACTUAL Python MPC balancer (upkie/controllers/mpc_balancer.py, v13.1.0, Apache-2.0)
in the SIL loop.  The module is loaded straight from the cloned repository; only its three tiny upkie-internal
imports (logger, clamp_abs, low_pass_filter) are stubbed so that the heavy `upkie` package (gymnasium, PyBullet)
is not needed.  qpmpc + proxsuite are the real solver stack upkie uses.

Structure (theirs): WheeledInvertedPendulum model (length = leg_length), N = 50 steps of 20 ms (1 s horizon),
|ground accel| <= 10 m/s^2, stage weights 1e-3, terminal 1.0; the first planned acceleration is integrated into a
commanded ground velocity (|v| <= 3 m/s) which is sent to the moteus wheel velocity loops.  Runs at 200 Hz.
"""
import importlib.util, os, sys, types
import numpy as np

UPKIE_ROOT = os.environ.get("UPKIE_ROOT", "/tmp/claude-0/-home-user/43117b7d-358d-559a-9210-4aefc030835e/scratchpad/upkie/upkie")


def _stub_modules():
    class _Log:
        def warning(self, *a, **k): pass
        def info(self, *a, **k): pass
        def error(self, *a, **k): pass
        def debug(self, *a, **k): pass
    logging_mod = types.ModuleType("upkie.logging"); logging_mod.logger = _Log()
    clamp_mod = types.ModuleType("upkie.utils.clamp")
    def clamp_abs(value, bound): return max(-bound, min(bound, value))
    clamp_mod.clamp_abs = clamp_abs
    filters_mod = types.ModuleType("upkie.utils.filters")
    def low_pass_filter(prev_output, cutoff_period, new_input, dt):
        alpha = min(1.0, dt / cutoff_period)
        return prev_output + alpha * (new_input - prev_output)
    filters_mod.low_pass_filter = low_pass_filter
    pkg = types.ModuleType("upkie"); utils = types.ModuleType("upkie.utils")
    for name, mod in (("upkie", pkg), ("upkie.utils", utils), ("upkie.logging", logging_mod),
                      ("upkie.utils.clamp", clamp_mod), ("upkie.utils.filters", filters_mod)):
        sys.modules.setdefault(name, mod)


def _load_mpc_balancer():
    _stub_modules()
    path = os.path.join(UPKIE_ROOT, "upkie", "controllers", "mpc_balancer.py")
    spec = importlib.util.spec_from_file_location("upkie_mpc_balancer_src", path)
    mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
    return mod.MPCBalancer


class PyController:
    """Same duck-type interface as sil_harness.Controller (init/step/out_type/name/Ts)."""
    out_type = 1          # wheel velocity reference
    has_scale = False
    name = "upkie MPCBalancer (qpmpc + ProxQP, N=50 x 20 ms, 200 Hz) -> moteus wheel velocity"

    def __init__(self, Ts, leg_length=None):
        self.Ts = Ts
        self.leg_length = leg_length
        self.MPCBalancer = _load_mpc_balancer()
        self.bal = None
        self.wheel_r = 0.06
        self.x = 0.0

    def init(self, p):
        self.wheel_r = p.r; self.x = 0.0
        self.bal = self.MPCBalancer(leg_length=self.leg_length if self.leg_length else p.l)
        self.bal.reset()

    def set_param(self, name, v):
        if name == "leg_length": self.leg_length = v

    def step(self, theta, theta_dot, x_dot, psi_dot, leg_len, r, v_ref, yaw_ref, t):
        self.x += x_dot * self.Ts
        obs = {"floor_contact": {"contact": True},
               "base_orientation": {"pitch": float(theta), "angular_velocity": [0.0, float(theta_dot), 0.0]},
               "wheel_odometry": {"position": float(self.x), "velocity": float(x_dot)}}
        v = self.bal.step(target_ground_velocity=float(v_ref), spine_observation=obs, dt=self.Ts)
        w = v / self.wheel_r
        return (w, w)


def make(Ts, **kw):
    return PyController(Ts, **kw)
