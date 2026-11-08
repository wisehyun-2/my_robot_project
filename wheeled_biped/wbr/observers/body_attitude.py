"""Body attitude on the MCU: a complementary filter on the raw IMU sample with the robot's own acceleration taken out.

The DIABLO paper's estimator (sec. III-C): "a complementary filter based state estimator ... utilizing information acquired from
the IMU, motor encoder, and motor drivers".  The pi3hat's attitude alone is not enough here: its filter takes the accelerometer for
gravity, so 2 m/s^2 of forward acceleration on a level body reads 1.6 deg of false pitch after 0.5 s, and the hips then really tilt
the body (wbr.observers.pi3hat_ars).  This filter integrates the gyro and pulls the result towards

    f - a_imu = g * up        (f: accelerometer, a_imu: the acceleration of the IMU point, known from the encoders and the gyro)

    a_imu = a_hip + alpha x r + omega x (omega x r)       r: IMU position from the hip midpoint (this is where the IMU's place enters)
    a_hip = time derivative of (v + d/dt (L0 sin theta), d/dt (L0 cos theta)), plus the centripetal yaw_rate * v sideways

Everything is in the base frame (x forward, y left, z up); the state is the world's up direction seen from the base.
The pi3hat library delivers the raw sample: gyro = rate_dps - bias_dps, accelerometer = accel_mps2 + 9.81 * conj(q) (0, 0, -1), q the
board's attitude (IMU -> ARS frame, z down): the firmware's OrientationToAccel, the world's UP seen from the IMU, (0, 0, +1) with the hat
side up at rest.  The physical down direction there would flip the sign.
"""
from __future__ import annotations

import math

import numpy as np

G = 9.81


class BodyAttitude:
    def __init__(self, tau: float, imu_position, dt: float, deriv_tau: float = 0.02, reject: float = 1.0, quiet: float = 3.0):
        self.tau = float(tau)                       # s, how slowly the accelerometer corrects the gyro integral
        self.r = np.asarray(imu_position, dtype=float)
        self.dt = float(dt)
        self.deriv_tau = float(deriv_tau)           # s, low-pass of the two derivatives and of the accelerometer (same delay)
        self.reject = float(reject)                 # m/s^2, the corrected sample is not used this far from 1 g
        self.quiet = float(quiet)                   # m/s^2, nor while the robot's own acceleration is above this: a push shakes the hips at
                                                    # 10 m/s^2 and the two signals are never in phase, so the gyro alone carries the attitude
        self.reset()

    def reset(self) -> None:
        self.up = None
        self.gyro_lp = np.zeros(3)
        self.f_lp = None
        self.alpha = np.zeros(3)
        self.vh_lp = None
        self.a_hip = np.zeros(2)                    # forward, vertical (heading frame)
        self.lateral = 0.0

    @property
    def roll(self) -> float:
        return math.atan2(self.up[1], self.up[2])

    @property
    def pitch(self) -> float:
        return math.atan2(-self.up[0], math.hypot(self.up[1], self.up[2]))

    def set_hip_velocity(self, vx: float, vz: float, yaw_rate: float, dt: float | None = None) -> None:
        """Hip midpoint velocity in the heading frame, from the encoders and the leg angle: called once per sample, after the legs.
        dt: the time since the previous sample (the nominal dt when None)."""
        dt = self.dt if dt is None else dt
        v = np.array([vx, vz])
        if self.vh_lp is None:
            self.vh_lp = v.copy()
        self.a_hip = (v - self.vh_lp) / self.deriv_tau
        self.vh_lp = self.vh_lp + self.a_hip * dt
        self.lateral = yaw_rate * vx

    def update(self, gyro: np.ndarray, f: np.ndarray, start_up: np.ndarray, dt: float | None = None) -> np.ndarray:
        """gyro, f in the base frame; start_up: the up direction to start from (the board's attitude).  Returns the up direction.
        dt: the time since the previous sample, from the time stamps (the nominal dt when None): a sample the loop skipped is
        integrated over its real interval, with the fixed dt its piece of angle was lost."""
        dt = self.dt if dt is None else dt
        k = dt / self.deriv_tau
        if self.up is None:
            self.up = np.asarray(start_up, dtype=float) / np.linalg.norm(start_up)
            self.gyro_lp = gyro.copy()
            self.f_lp = G * self.up
        self.alpha = (gyro - self.gyro_lp) / self.deriv_tau
        self.gyro_lp = self.gyro_lp + self.alpha * dt
        self.f_lp = self.f_lp + k * (f - self.f_lp)
        self.up = self.up - dt * np.cross(gyro, self.up)
        self.up /= np.linalg.norm(self.up)
        # the robot's acceleration in the base frame: heading frame -> base (Rx(-roll) Ry(-pitch))
        sp, cp = math.sin(self.pitch), math.cos(self.pitch)
        sr, cr = math.sin(self.roll), math.cos(self.roll)
        ax, ay, az = self.a_hip[0], self.lateral, self.a_hip[1]
        x1, z1 = cp * ax - sp * az, sp * ax + cp * az
        a_base = np.array([x1, cr * ay + sr * z1, -sr * ay + cr * z1])
        a_imu = a_base + np.cross(self.alpha, self.r) + np.cross(self.gyro_lp, np.cross(self.gyro_lp, self.r))
        m = self.f_lp - a_imu
        norm = float(np.linalg.norm(m))
        if abs(norm - G) < self.reject and float(np.linalg.norm(a_imu)) < self.quiet:
            self.up = self.up + (dt / self.tau) * (m / norm - self.up)
            self.up /= np.linalg.norm(self.up)
        return self.up
