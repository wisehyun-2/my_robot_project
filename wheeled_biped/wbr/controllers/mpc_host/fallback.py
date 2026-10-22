"""Fallback reference shaper of the MPC host: the stage-1 shaping (2 m/s^2 ramp + 0.3 s first-order pre-filter that
acts only while speeding up; braking follows the ramp), executed by the host at its publish rate.

Since review round 1 the host does NOT send this shaped reference: on a QP failure it hands the raw target to the
MCU with ``host_shaped = False`` and the MCU runs the same stage-1 shaping itself (``LQI6`` shaped mode).  The host
keeps this shaper only to propagate its internal reference state while in fallback (resume decision), and as the
reference generator when the prediction model is unavailable (unstable inner loop).
"""
from __future__ import annotations

import math


class RampFallback:
    def __init__(self, period: float, rate: float = 2.0, tau: float = 0.3, v_max: float = 1.0):
        self.period = float(period)
        self.rate = float(rate)
        self.tau = float(tau)
        self.v_max = float(v_max)
        self.alpha = 1.0 - math.exp(-self.period / self.tau) if self.tau > 0 else 1.0   # exact first-order step
        self.reset(0.0)

    def reset(self, value: float = 0.0) -> None:
        self.v_ramp = float(value)
        self.v_filt = float(value)

    def step(self, target: float) -> float:
        target = max(-self.v_max, min(self.v_max, float(target)))
        dv = max(-self.rate * self.period, min(self.rate * self.period, target - self.v_ramp))
        self.v_ramp += dv
        if self.tau > 0 and abs(self.v_ramp) > abs(self.v_filt):
            self.v_filt += (self.v_ramp - self.v_filt) * self.alpha      # speeding up: pre-filter
        else:
            self.v_filt = self.v_ramp                                     # braking / steady: follow the ramp
        return self.v_filt

    @property
    def steady(self) -> bool:
        return abs(self.v_filt - self.v_ramp) < 1e-4
