"""The pi3hat attitude reference, ported from the firmware (mjbots/pi3hat fw/attitude_reference.h, fw/ukf_filter.h, Apache-2.0).

The board's attitude comes out of this filter and not from a perfect sensor, so the simulation runs the same filter on the
gyro and accelerometer samples at the IMU's place on the robot.  7 states: quaternion (IMU -> ARS frame, x forward, y right,
z down) + gyro bias.  The measurement is the normalised accelerometer sample, skipped when |a| is more than 2 m/s^2 from 1 g,
plus a pseudo measurement that pulls the world-frame yaw rate towards 0.  Same process / measurement noise and the same
14-point sigma set as the firmware, in double precision.  The accelerometer sets the attitude only slowly: 1 kHz in, about
16 s time constant, so short accelerations hardly reach it but a long turn does.

    python -m wbr.observers.pi3hat_ars       # the converged covariance of a robot at rest: pi3hat_ars_steady.json
"""
from __future__ import annotations

import functools
import json
import math
from pathlib import Path

import numpy as np

G = 9.81
N_STATES = 7
STEADY_FILE = Path(__file__).with_name("pi3hat_ars_steady.json")


def quat_mul(a, b):
    """Hamilton product of two quaternion arrays [..., 4] (w, x, y, z)."""
    aw, ax, ay, az = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    bw, bx, by, bz = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return np.stack((aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw), axis=-1)


def quat_from_euler(roll: float, pitch: float, yaw: float = 0.0) -> np.ndarray:
    """fw/quaternion.h FromEuler: yaw * pitch * roll."""
    def axis(angle, i):
        q = np.zeros(4)
        q[0] = math.cos(0.5 * angle)
        q[i] = math.sin(0.5 * angle)
        return q
    return quat_mul(quat_mul(axis(yaw, 3), axis(pitch, 2)), axis(roll, 1))


def accel_to_orientation(accel) -> np.ndarray:
    """AttitudeReference::AccelToOrientation: the start attitude from one accelerometer sample."""
    n = -np.asarray(accel, dtype=float) / np.linalg.norm(accel)
    return quat_from_euler(math.atan2(-n[0], n[2]), math.atan2(n[1], math.hypot(n[0], n[2])))


class Pi3HatAttitude:
    """AttitudeReference + UkfFilter.  process(rate, accel) once per sample; quaternion, rate_rps, bias_rps as the firmware."""

    def __init__(self, dt: float, process_noise_gyro=5e-4 ** 2, process_noise_bias=7e-5 ** 2, measurement_noise_accel=0.5 ** 2,
                 initial_noise_attitude=1.0, initial_noise_bias=0.1 ** 2, accelerometer_reject=2.0, zero_yaw_noise=10.0 ** 2):
        self.dt = float(dt)
        self.Q = np.diag([process_noise_gyro] * 4 + [process_noise_bias] * 3)
        self.R_accel = measurement_noise_accel * np.eye(3)
        self.R_yaw = float(zero_yaw_noise)
        self.reject = float(accelerometer_reject)
        self.x = np.array([1.0, 0, 0, 0, 0, 0, 0])
        self.P = np.diag([initial_noise_attitude] * 4 + [initial_noise_bias] * 3)
        self.initialized = False
        self.gyro = np.zeros(3)
        self.accel = np.zeros(3)

    def start_converged(self, quaternion, gyro_bias=(0.0, 0.0, 0.0), P=None) -> None:
        """A board that has been powered for a while: the attitude known (IMU -> ARS frame), the bias known, the covariance at
        rest.  Not from accel_to_orientation: the firmware's start formula swaps the tilt axes (its filter then converges from a
        large covariance), so it is only right on the level."""
        self.x[:4] = np.asarray(quaternion, dtype=float)
        self.x[4:] = -np.asarray(gyro_bias, dtype=float)    # the filter adds its bias state to the gyro sample
        self.P = np.array(steady_covariance(self.dt) if P is None else P, dtype=float)
        self.initialized = True

    # ------------------------------------------------------------------ UkfFilter
    def _sigma(self) -> np.ndarray:
        try:
            L = np.linalg.cholesky(N_STATES * self.P)
        except np.linalg.LinAlgError:           # the firmware's single-precision llt() carries on; so does this
            L = np.linalg.cholesky(N_STATES * self.P + 1e-12 * np.eye(N_STATES))
        return np.concatenate((self.x + L.T, self.x - L.T), axis=0)     # rows = sigma points (columns of L)

    def _update_state(self) -> None:
        s = self._sigma()
        q = s[:, :4] / np.linalg.norm(s[:, :4], axis=1, keepdims=True)
        rate = self.gyro + s[:, 4:]
        d = np.empty((s.shape[0], 4))
        d[:, 0] = 1.0
        d[:, 1:] = 0.5 * rate * self.dt
        d /= np.linalg.norm(d, axis=1, keepdims=True)
        n = quat_mul(q, d)
        n /= np.linalg.norm(n, axis=1, keepdims=True)
        xh = np.concatenate((n, s[:, 4:]), axis=1)
        self.x = xh.mean(axis=0)
        c = xh - self.x
        P = c.T @ c / xh.shape[0] + self.dt * self.Q
        self.P = 0.5 * (P + P.T)

    def _update_measurement(self, s: np.ndarray, yh: np.ndarray, y: np.ndarray, R: np.ndarray) -> None:
        ybar = yh.mean(axis=0)
        dy = yh - ybar
        Py = dy.T @ dy / s.shape[0] + R
        Pxy = (s - self.x).T @ dy / s.shape[0]
        K = Pxy @ np.linalg.inv(Py)
        self.x = self.x + K @ (y - ybar)
        P = self.P - K @ Py @ K.T
        self.P = 0.5 * (P + P.T)

    # ------------------------------------------------------------- AttitudeReference
    def process(self, rate_rps, accel_mps2) -> None:
        self.gyro = np.asarray(rate_rps, dtype=float)
        self.accel = np.asarray(accel_mps2, dtype=float)
        norm = float(np.linalg.norm(self.accel))
        if not self.initialized:
            self.initialized = True
            self.x[:4] = accel_to_orientation(self.accel)
        self._update_state()
        if abs(norm - G) < self.reject:
            s = self._sigma()
            q = s[:, :4] / np.linalg.norm(s[:, :4], axis=1, keepdims=True)
            w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
            # attitude.conjugated().Rotate((0, 0, -1)): gravity direction in the body frame
            yh = -np.stack((2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)), axis=1)
            self._update_measurement(s, yh, self.accel / norm, self.R_accel)
        # zero yaw rate pseudo measurement: attitude.conjugated().Rotate(gyro + bias).z, the quaternion not normalised
        s = self._sigma()
        w, x, y, z = s[:, 0], s[:, 1], s[:, 2], s[:, 3]
        r = self.gyro + s[:, 4:]
        yh = (2 * (x * z + w * y) * r[:, 0] + 2 * (y * z - w * x) * r[:, 1] + (w * w - x * x - y * y + z * z) * r[:, 2])[:, None]
        self._update_measurement(s, yh, np.zeros(1), np.array([[self.R_yaw]]))

    @property
    def quaternion(self) -> np.ndarray:
        """IMU frame -> ARS frame (x forward, y right, z down); unit length, as the MCU normalises it anyway."""
        return self.x[:4] / np.linalg.norm(self.x[:4])

    @property
    def rate_rps(self) -> np.ndarray:
        return self.gyro + self.x[4:]

    @property
    def bias_rps(self) -> np.ndarray:
        return self.x[4:].copy()


def steady_covariance(dt: float) -> np.ndarray:
    """Covariance of a board at rest, hat side up, POWER_ON_S after power-on (pi3hat_ars_steady.json, python -m wbr.observers.pi3hat_ars).
    The accelerometer's weight keeps falling for many minutes (2 m/s^2 for 0.5 s reads 2.9 deg after 10 s, 1.6 after 60 s, 0.9
    after 15 min): a robot that is armed a minute after power-on is the realistic and the less favourable case.
    The table has TABLE_RATES; any other rate is computed the same way on first use (about 1 s per 200 Hz)."""
    hz = int(round(1.0 / dt))
    table = json.loads(STEADY_FILE.read_text(encoding="utf-8"))
    if str(hz) in table:
        return np.array(table[str(hz)])
    return _rest_covariance(hz).copy()


POWER_ON_S = 60
TABLE_RATES = (1000, 400, 200)      # Hz: the plans' MCU rates (1000 or 200, WBR_TEST_RATE) and 400


@functools.lru_cache(maxsize=None)
def _rest_covariance(rate: int) -> np.ndarray:
    f = Pi3HatAttitude(1.0 / rate)
    accel = np.array([0.0, 0.0, G])
    for _ in range(rate * POWER_ON_S):
        f.process(np.zeros(3), accel)
    return f.P.copy()


def main() -> None:
    table = json.loads(STEADY_FILE.read_text(encoding="utf-8")) if STEADY_FILE.exists() else {}
    for rate in TABLE_RATES:
        P = _rest_covariance(rate)
        print(rate, "Hz, sqrt P diag", np.round(np.sqrt(np.diag(P)), 7))
        table[str(rate)] = P.tolist()
    STEADY_FILE.write_text(json.dumps(table), encoding="utf-8")
    print("wrote", STEADY_FILE)


if __name__ == "__main__":
    main()
