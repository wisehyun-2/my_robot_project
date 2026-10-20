"""
sizing_options.py -- for each candidate motor (per-wheel peak torque, continuous torque, no-load speed, Kt),
compute on the planned 10 kg robot: max recoverable tilt, max survivable push, 1 m/s step feasibility,
peak phase current, and the LQI gains/delay margin with that torque limit.
"""
import json, os
import numpy as np
import scipy.linalg as la
import control as ct
from twip_model import TWIPParams, BASELINE, G

HERE = os.path.dirname(os.path.abspath(__file__)); RES = os.path.join(HERE, "..", "results")
DT = 5e-4; TS = 1e-3; LAT = 2e-3

# per-wheel numbers. Kt in Nm/A (vendor), v_nl no-load speed [m/s] at r = 0.0825
SMALL = TWIPParams(name="small_5kg", M_b=3.2, m_w=0.7, r=0.065, l=0.16, I_b=0.07, b_x=0.3, d=0.30, I_z=0.08, tau_max=3.0, Kt=0.5)
CANDIDATES_SMALL = {
    # name: (peak Nm, continuous Nm, no-load rpm, Kt, mass kg)   per wheel, incl. driver
    "RMD-L-7015-23T (gearless, CAN, 360 g)":         (3.8, 1.0, 805, 0.28, 0.36),
    "RMD-L-9015-35T (gearless, CAN, 650 g)":         (5.0, 1.67, 500, 0.45, 0.65),
    "RMD-L-9025-35T (gearless, CAN, 880 g)":         (4.58, 2.42, 490, 0.76, 0.88),
    "Waveshare DDSM115 (gearless hub, RS485, 770 g)": (2.0, 0.96, 200, 0.75, 0.77),
    "iPower GM110-8 + moteus c1 (gimbal direct)":   (2.0, 0.9, 130, 0.8, 0.55),
    "mj5208 + moteus c1 (gimbal direct)":           (1.7, 0.4, 600, 0.03, 0.21),
    "CyberGear 7.75:1 (geared fallback, 320 g)":    (12.0, 4.0, 300, 0.87, 0.32),
}
CANDIDATES = {
    "RMD-L-9015-35T (gearless, CAN, 650 g)":         (5.0, 1.67, 500, 0.45, 0.65),
    "RMD-L-9025-35T (gearless, CAN, 880 g)":         (4.58, 2.42, 490, 0.76, 0.88),
    "LK MF9025 V2 (gearless, CAN/RS485)":            (4.5, 2.42, 300, 0.7, 0.9),
    "Hoverboard 6.5in hub + ODrive S1 (20 A pk)":    (10.0, 2.5, 600, 0.52, 2.9),
    "Hoverboard 6.5in hub + ODrive S1 (12 A pk)":    (6.0, 2.5, 600, 0.52, 2.9),
    "CyberGear 7.75:1 (geared fallback)":            (12.0, 4.0, 300, 0.87, 0.32),
    "CubeMars AK70-10 (geared fallback)":            (24.0, 8.3, 475, 1.1, 0.52),
}


def design(p, Ts):
    A, B, _ = p.linearize()
    Q = np.diag([0, 1, 25, 0.25]); R = np.array([[1 / (0.5 * p.tau_max) ** 2]])
    Aa = np.block([[A, np.zeros((4, 1))], [np.array([[0, 1, 0, 0]]), np.zeros((1, 1))]]); Ba = np.vstack([B, [[0.0]]])
    Qa = np.block([[Q, np.zeros((4, 1))], [np.zeros((1, 4)), np.array([[1.0]])]])
    sysd = ct.c2d(ct.ss(Aa, Ba, np.eye(5), np.zeros((5, 1))), Ts)
    Pd = la.solve_discrete_are(sysd.A, sysd.B, Qa, R)
    K = np.linalg.solve(R + sysd.B.T @ Pd @ sysd.B, sysd.B.T @ Pd @ sysd.A).ravel()
    Qc = np.diag([1, 1, 25, 0.25]); Pc = la.solve_continuous_are(A, B, Qc, R); Kc = np.linalg.solve(R, B.T @ Pc)
    gm, pm, wgc, wpc = ct.margin(ct.ss(A, B, Kc, np.zeros((1, 1))))
    return K, float(np.deg2rad(pm) / wpc * 1e3)


def simulate(p, K, v_nl, T=3.0, x0=None, push=None, v_ref=0.0, ramp=1.0):
    n = int(round(T / DT)); sub = int(round(TS / DT)); d = int(round(LAT / DT))
    x = np.zeros(4) if x0 is None else np.array(x0, float); fifo = [0.0] * (d + 1); u = 0.0; z = 0.0; vr = 0.0
    tmax = 2 * p.tau_max; peak_tau = 0.0; peak_v = 0.0
    for k in range(n):
        t = k * DT
        if k % sub == 0:
            vr += float(np.clip(v_ref - vr, -ramp * TS, ramp * TS))
            ev = x[1] - vr; z = float(np.clip(z + ev * TS, -1, 1))
            u = float(np.clip(-(K[1] * ev + K[2] * x[2] + K[3] * x[3] + K[4] * z), -tmax, tmax))
        fifo.append(u); tau = fifo.pop(0)
        frac = max(0.0, 1 - abs(x[1]) / v_nl)
        if x[1] != 0 and np.sign(tau) == np.sign(x[1]):
            tau = float(np.clip(tau, -tmax * frac, tmax * frac))
        F = push[2] if (push and push[0] <= t < push[0] + push[1]) else 0.0
        f = p.f
        k1 = f(x, tau, F); k2 = f(x + 0.5 * DT * k1, tau, F); k3 = f(x + 0.5 * DT * k2, tau, F); k4 = f(x + DT * k3, tau, F)
        x = x + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        peak_tau = max(peak_tau, abs(tau)); peak_v = max(peak_v, abs(x[1]))
        if abs(x[2]) > np.deg2rad(70):
            return False, peak_tau, peak_v
    return abs(x[2]) < np.deg2rad(3), peak_tau, peak_v


out = {}
import sys
which = sys.argv[1] if len(sys.argv) > 1 else "large"
base, cands = (SMALL, CANDIDATES_SMALL) if which == "small" else (BASELINE, CANDIDATES)
print(f"== robot class: {which}  (M_b={base.M_b} kg, r={base.r} m, l={base.l} m) ==")
for name, (pk, cont, rpm, Kt, mass) in cands.items():
    p = TWIPParams(**{**base.__dict__, "name": name, "tau_max": pk, "m_w": mass, "Kt": Kt})
    v_nl = rpm * 2 * np.pi / 60 * p.r
    K, dm = design(p, TS)
    tilt = max([a for a in np.arange(5, 46, 2.5) if simulate(p, K, v_nl, x0=[0, 0, np.deg2rad(a), 0])[0]], default=0)
    push = max([F for F in range(20, 201, 10) if simulate(p, K, v_nl, push=(0.3, 0.1, float(F)))[0]], default=0)
    ok1, pt1, pv1 = simulate(p, K, v_nl, T=5.0, v_ref=1.0)
    ok15, pt15, pv15 = simulate(p, K, v_nl, T=5.0, v_ref=1.5)
    out[name] = dict(peak_Nm=pk, cont_Nm=cont, no_load_m_s=round(v_nl, 2), mass_kg=mass, Kt=Kt,
                     peak_phase_A=round(pk / Kt, 1), cont_A=round(cont / Kt, 1),
                     max_tilt_deg=float(tilt), max_push_N=float(push),
                     step_1m_s=dict(ok=bool(ok1), peak_Nm_per_wheel=round(pt1 / 2, 2), peak_speed=round(pv1, 2)),
                     step_1p5m_s=dict(ok=bool(ok15), peak_Nm_per_wheel=round(pt15 / 2, 2), peak_speed=round(pv15, 2)),
                     slope_15deg_cont_Nm=round(0.5 * p.M_tot * G * np.sin(np.deg2rad(15)) * p.r, 2),
                     delay_margin_ms=round(dm, 0), K_lqi_1kHz=[round(float(k), 3) for k in K])
    print(f"{name:45s} tilt {tilt:5.1f} deg  push {push:4.0f} N  1 m/s {'ok' if ok1 else 'X '} ({pt1/2:.2f} Nm)  "
          f"1.5 m/s {'ok' if ok15 else 'X '}  Ipk {pk/Kt:5.1f} A  DM {dm:.0f} ms")
json.dump(out, open(os.path.join(RES, f"sizing_options_{which}.json"), "w"), indent=1)
