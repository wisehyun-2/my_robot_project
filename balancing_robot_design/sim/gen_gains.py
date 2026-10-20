"""
gen_gains.py  --  print LQR / LQI gains (and the C constants for the Teensy firmware) for a parameter set.

usage:  python3 gen_gains.py [--M_b 7 --m_w 1.2 --r 0.0825 --l 0.25 --I_b 0.2 --tau_max 5 --fs 1000]
The weights are the ones validated in analysis_controllers.py (PM ~72 deg, delay margin ~40 ms).
"""
import argparse, json
import numpy as np
import scipy.linalg as la
import control as ct
from twip_model import TWIPParams, BASELINE

ap = argparse.ArgumentParser()
for k, v in BASELINE.__dict__.items():
    if isinstance(v, (int, float)):
        ap.add_argument(f"--{k}", type=float, default=v)
ap.add_argument("--fs", type=float, default=1000.0, help="control loop rate [Hz]")
ap.add_argument("--th_w", type=float, default=0.2, help="Bryson weight: tolerated pitch [rad]")
ap.add_argument("--v_w", type=float, default=1.0, help="Bryson weight: tolerated speed error [m/s]")
args = ap.parse_args()

p = TWIPParams(**{k: getattr(args, k) for k in BASELINE.__dict__ if isinstance(BASELINE.__dict__[k], (int, float))})
Ts = 1.0 / args.fs
A, B, _ = p.linearize()
Q = np.diag([1 / 1.0 ** 2, 1 / args.v_w ** 2, 1 / args.th_w ** 2, 1 / 2.0 ** 2])
R = np.array([[1 / (0.5 * p.tau_max) ** 2]])

# continuous LQR (position-holding form) for margins
P = la.solve_continuous_are(A, B, Q, R); K = np.linalg.solve(R, B.T @ P)
gm, pm, wgc, wpc = ct.margin(ct.ss(A, B, K, np.zeros((1, 1))))

# discrete LQI (velocity-tracking form, used by the firmware)
Aa = np.block([[A, np.zeros((4, 1))], [np.array([[0, 1, 0, 0]]), np.zeros((1, 1))]])
Ba = np.vstack([B, [[0.0]]])
Qa = np.block([[Q, np.zeros((4, 1))], [np.zeros((1, 4)), np.array([[1.0]])]]); Qa[0, 0] = 0.0
sysd = ct.c2d(ct.ss(Aa, Ba, np.eye(5), np.zeros((5, 1))), Ts)
Pd = la.solve_discrete_are(sysd.A, sysd.B, Qa, R)
Kd = np.linalg.solve(R + sysd.B.T @ Pd @ sysd.B, sysd.B.T @ Pd @ sysd.A).ravel()

print(f"robot: M_b={p.M_b} kg  m_w={p.m_w} kg  r={p.r} m  l={p.l} m  I_b={p.I_b} kg m^2  tau_max={p.tau_max} Nm/wheel")
print(f"unstable pole {p.unstable_pole():.2f} rad/s;  continuous LQR K={np.round(K.ravel(),3)};  PM={pm:.0f} deg @ {wpc:.1f} rad/s -> delay margin {np.deg2rad(pm)/wpc*1e3:.0f} ms")
print(f"discrete LQI @ {args.fs:.0f} Hz:  K = [k_x, k_v, k_theta, k_thetadot, k_int] = {np.round(Kd,3)}")
print("\n// paste into teensy_balance.ino  (u_total = -(K_V*(v - v_ref) + K_TH*theta + K_THD*theta_dot + K_I*z))")
print(f"constexpr float K_V   = {Kd[1]:.4f}f;")
print(f"constexpr float K_TH  = {Kd[2]:.4f}f;")
print(f"constexpr float K_THD = {Kd[3]:.4f}f;")
print(f"constexpr float K_I   = {Kd[4]:.4f}f;")
print("\n// equivalent cascade-PID gains (if you insist on the PID structure):")
print(f"//   inner  tau = Kp_th*(theta - theta_ref) + Kd_th*theta_dot      Kp_th={-Kd[2]:.3f} Nm/rad, Kd_th={-Kd[3]:.3f} Nm s/rad")
print(f"//   outer  theta_ref = -(Kp_v*(v - v_ref) + Ki_v*int)             Kp_v={Kd[1]/Kd[2]:.4f} rad/(m/s), Ki_v={Kd[4]/Kd[2]:.4f} rad/m")
