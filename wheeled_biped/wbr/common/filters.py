"""Filters and clamps (ports of upkie/utils/filters.py and clamp.py, Apache-2.0)."""
from __future__ import annotations

import math


def clamp(value: float, lower: float, upper: float) -> float:
    return lower if value < lower else upper if value > upper else value


def clamp_abs(value: float, bound: float) -> float:
    return clamp(value, -bound, bound)


def low_pass_filter(prev_output: float, cutoff_period: float, new_input: float, dt: float) -> float:
    """First-order low-pass filter (same semantics as upkie: alpha = dt / cutoff_period)."""
    if cutoff_period <= 2.0 * dt:   # upkie: FilterError (Nyquist)
        raise ValueError(f"low_pass_filter: cutoff_period {cutoff_period} s must exceed 2 * dt = {2 * dt} s")
    alpha = dt / cutoff_period
    return prev_output + alpha * (new_input - prev_output)


def abs_bounded_derivative_filter(prev_output: float, new_input: float, dt: float, max_derivative: float) -> float:
    """Rate limiter: output moves toward the input at most max_derivative * dt per step."""
    delta = clamp_abs(new_input - prev_output, max_derivative * dt)
    return prev_output + delta


class Ramp:
    """Rate-limited reference (the velocity ramp of the design report, 1 m/s^2 by default)."""

    def __init__(self, max_rate: float, max_abs: float = math.inf, value: float = 0.0):
        self.max_rate = max_rate
        self.max_abs = max_abs
        self.value = value

    def reset(self, value: float = 0.0) -> None:
        self.value = value

    def step(self, target: float, dt: float) -> float:
        if not math.isfinite(target):                 # a NaN target would poison the ramp for good (review r2 M2)
            return self.value
        target = clamp_abs(target, self.max_abs)
        self.value = abs_bounded_derivative_filter(self.value, target, dt, self.max_rate)
        return self.value


class DelayLine:
    """Fixed-latency FIFO used to emulate sensor/actuation delays (in control steps)."""

    def __init__(self, delay_steps: int, initial):
        self.delay_steps = max(0, int(delay_steps))
        self.buffer = [initial] * self.delay_steps   # value pushed at step k comes out at step k + delay_steps

    def push(self, value):
        self.buffer.append(value)
        return self.buffer.pop(0)
