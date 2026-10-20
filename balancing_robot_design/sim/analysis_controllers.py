"""
Controller comparison for a direct-drive two-wheeled balancing robot.

Produces (into ../figures and ../results):
  1. Model summary (unstable pole, required static torques) for three parameter sets
  2. LQR / LQI design, closed-loop poles, gains, Bode margins at plant input
  3. Cascade PID (angle PD + velocity PI) shown to be a restricted state feedback
  4. Stability map  (loop rate x total latency)  -> quantifies the "balance loop in ROS2 at 100 Hz" question
  5. Nonlinear simulations with torque saturation, ZOH, latency, IMU noise:
       - tilt recovery, velocity step, push disturbance
       - torque mode (direct drive) vs velocity-mode cascade (repo-1 / ODrive style)
"""
import json
import os
import numpy as np
import scipy.linalg as la
import control as ct
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from twip_model import TWIPParams, REPO1_LIKE, BASELINE, HEAVY, G

HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, "..", "figures")
RES = os.path.join(HERE, "..", "results")
os.makedirs(FIG, exist_ok=True)
os.makedirs(RES, exist_ok=True)
rng = np.random.default_rng(0)

# ----------------------------------------------------------------------------- helpers

def lqr_gain(A, B, Q, R):
    P = la.solve_continuous_are(A, B, Q, R)
    K = np.linalg.solve(R, B.T @ P)
    return K, P


def dlqr_gain(Ad, Bd, Q, R):
    P = la.solve_discrete_are(Ad, Bd, Q, R)
    K = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
    return K, P


def margins_at_input(A, B, K):
    """Loop transfer function broken at the plant input: L(s) = K (sI-A)^-1 B  (SISO)."""
    sys = ct.ss(A, B, K, np.zeros((1, 1)))
    gm, pm, wgc, wpc = ct.margin(sys)
    return dict(gain_margin_dB=20 * np.log10(gm) if np.isfinite(gm) else np.inf,
                phase_margin_deg=float(pm), crossover_rad_s=float(wpc),
                delay_margin_ms=float(np.deg2rad(pm) / wpc * 1e3) if wpc > 0 else np.inf)


# ----------------------------------------------------------------------------- simulation core

def simulate(p: TWIPParams, controller, Ts=1e-3, latency=0.0, T=4.0, x0=None,
             push=None, v_ref_fn=None, noise=False, tau_limit=None, dt_int=1e-4,
             vel_mode=None):
    """
    Fixed-step RK4 of the nonlinear TWIP with a sampled-data controller.
      controller(xmeas, v_ref, t) -> u_total  (torque, N m) OR wheel speed command (vel_mode)
      Ts       : controller period
      latency  : total sensor->actuator delay (s), applied as a FIFO on the actuator command
      push     : (t_start, duration, force N) horizontal push on COM
      vel_mode : None  -> torque command directly to wheels (direct drive, FOC torque mode)
                 dict(Kp, Ki, bw_meas) -> controller outputs wheel speed, a PI speed loop in
                 the driver makes torque (ODrive/hub-driver "velocity mode")
    """
    tau_limit = 2 * p.tau_max if tau_limit is None else tau_limit
    n = int(round(T / dt_int))
    x = np.zeros(4) if x0 is None else np.array(x0, float)
    sub = int(round(Ts / dt_int))
    delay_steps = int(round(latency / dt_int))
    fifo = [0.0] * (delay_steps + 1)
    u_cmd = 0.0
    tau_applied = 0.0
    t_hist, x_hist, u_hist = [], [], []
    vel_int = 0.0
    w_meas_f = 0.0
    th_bias = np.deg2rad(0.0)
    for k in range(n):
        t = k * dt_int
        v_ref = v_ref_fn(t) if v_ref_fn else 0.0
        if k % sub == 0:
            xm = x.copy()
            if noise:
                # pitch estimate noise ~0.2 deg rms (complementary filter), gyro 0.5 deg/s rms,
                # wheel speed from 16384-cnt encoder differentiated at Ts
                xm[2] += rng.normal(0, np.deg2rad(0.2)) + th_bias
                xm[3] += rng.normal(0, np.deg2rad(0.5))
                q = 2 * np.pi / 16384 * p.r
                xm[1] += rng.uniform(-q, q) / Ts
            u_cmd = float(controller(xm, v_ref, t))
        F = 0.0
        if push and push[0] <= t < push[0] + push[1]:
            F = push[2]
        # actuator path with latency
        fifo.append(u_cmd)
        u_delayed = fifo.pop(0)
        if vel_mode is None:
            tau_applied = float(np.clip(u_delayed, -tau_limit, tau_limit))
        else:
            # driver speed loop at integration rate (drivers run >= 8 kHz), measured speed low-passed
            w = x[1] / p.r
            a = dt_int * 2 * np.pi * vel_mode["bw_meas"]
            w_meas_f += a * (w - w_meas_f)
            e = u_delayed - w_meas_f
            vel_int += e * dt_int
            tau_unsat = vel_mode["Kp"] * e + vel_mode["Ki"] * vel_int
            tau_applied = float(np.clip(tau_unsat, -tau_limit, tau_limit))
            if tau_applied != tau_unsat:  # anti-windup
                vel_int -= e * dt_int
        # RK4
        f = p.f
        k1 = f(x, tau_applied, F)
        k2 = f(x + 0.5 * dt_int * k1, tau_applied, F)
        k3 = f(x + 0.5 * dt_int * k2, tau_applied, F)
        k4 = f(x + dt_int * k3, tau_applied, F)
        x = x + dt_int / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        t_hist.append(t); x_hist.append(x.copy()); u_hist.append(tau_applied)
        if abs(x[2]) > np.deg2rad(60):
            break
    return np.array(t_hist), np.array(x_hist), np.array(u_hist)


def fell(xh, T_total, dt_int=1e-4):
    return len(xh) < int(round(T_total / dt_int)) - 1 or np.max(np.abs(xh[-2000:, 2])) > np.deg2rad(2.0)


# ----------------------------------------------------------------------------- controllers

def make_lqr(p: TWIPParams, Q=None, R=None, integral=False, Ts=None):
    A, B, _ = p.linearize()
    if Q is None:
        # Bryson-style: x 1 m, xdot 1 m/s, theta 0.2 rad (~11 deg), thetadot 2 rad/s
        # -> crossover ~30 rad/s (4-5x unstable pole), PM ~72 deg, delay margin ~40 ms (see lqr_design_baseline.json)
        Q = np.diag([1 / 1.0 ** 2, 1 / 1.0 ** 2, 1 / 0.2 ** 2, 1 / 2.0 ** 2])
    if R is None:
        R = np.array([[1 / (0.5 * p.tau_max) ** 2]])   # 'typical' torque ~ half of one wheel's peak
    if integral:
        # augment with z = int (xdot - v_ref) dt   -> velocity tracking w/o steady-state error
        Aa = np.block([[A, np.zeros((4, 1))], [np.array([[0, 1, 0, 0]]), np.zeros((1, 1))]])
        Ba = np.vstack([B, [[0.0]]])
        Qa = np.block([[Q, np.zeros((4, 1))], [np.zeros((1, 4)), np.array([[1 / 1.0 ** 2]])]])
        Qa[0, 0] = 0.0  # don't hold position when tracking velocity
        if Ts:
            Ad, Bd = ct.c2d(ct.ss(Aa, Ba, np.eye(5), np.zeros((5, 1))), Ts).A, ct.c2d(ct.ss(Aa, Ba, np.eye(5), np.zeros((5, 1))), Ts).B
            K, _ = dlqr_gain(Ad, Bd, Qa, R)
        else:
            K, _ = lqr_gain(Aa, Ba, Qa, R)
        K = K.ravel()
        state = {"z": 0.0}

        def ctrl(xm, v_ref, t, _Ts=Ts or 1e-3):
            state["z"] += (xm[1] - v_ref) * _Ts
            state["z"] = float(np.clip(state["z"], -2, 2))
            return -(K[1] * (xm[1] - v_ref) + K[2] * xm[2] + K[3] * xm[3] + K[4] * state["z"])
        return ctrl, K, (Aa, Ba)
    else:
        if Ts:
            sysd = ct.c2d(ct.ss(A, B, np.eye(4), np.zeros((4, 1))), Ts)
            K, _ = dlqr_gain(sysd.A, sysd.B, Q, R)
        else:
            K, _ = lqr_gain(A, B, Q, R)
        K = K.ravel()

        def ctrl(xm, v_ref, t):
            return -(K @ xm)
        return ctrl, K, (A, B)


def make_cascade_pid(p: TWIPParams, kp_th, kd_th, kp_v, ki_v, Ts=1e-3, use_wheel_feedback=True):
    """Outer PI on wheel velocity -> theta_ref ; inner PD on pitch -> torque.   (all gains positive)
         theta_ref = -(kp_v (xdot - v_ref) + ki_v int(xdot - v_ref))      # faster than wanted -> lean back
         u         =  kp_th (theta - theta_ref) + kd_th thetadot            # torque drives wheels under the body
       => u = kp_th theta + kd_th thetadot + kp_th kp_v (xdot - v_ref) + kp_th ki_v int(...)
       i.e. EXACTLY a state feedback u = -K x with K = -[0, kp_th kp_v, kp_th, kd_th, kp_th ki_v]:
       the cascade is a point in LQR's gain space (minus the x gain); LQR just chooses the point optimally.
       use_wheel_feedback=False reproduces repo 1 (no wheel speed closed, stick -> theta_ref only)."""
    st = {"iv": 0.0}

    def ctrl(xm, v_ref, t):
        if use_wheel_feedback:
            ev = xm[1] - v_ref
            st["iv"] = float(np.clip(st["iv"] + ev * Ts, -1.0, 1.0))
            th_ref = -(kp_v * ev + ki_v * st["iv"])
        else:
            th_ref = kp_v * v_ref
        th_ref = float(np.clip(th_ref, -np.deg2rad(12), np.deg2rad(12)))
        return kp_th * (xm[2] - th_ref) + kd_th * xm[3]
    return ctrl


# ----------------------------------------------------------------------------- 1. model summaries
summ = {}
for p in (REPO1_LIKE, BASELINE, HEAVY):
    s = p.summary()
    s["open_loop_poles"] = [complex(c) for c in s["open_loop_poles"]]
    summ[p.name] = {k: (v if not isinstance(v, list) else [str(c) for c in v]) for k, v in s.items()}
    print(f"[{p.name}] unstable pole = {s['unstable_pole_rad_s']:.2f} rad/s  (tau = {s['unstable_time_constant_ms']:.0f} ms), "
          f"hold 10deg = {s['static_hold_torque_10deg_Nm']:.2f} Nm, hold 20deg = {s['static_hold_torque_20deg_Nm']:.2f} Nm, "
          f"slope 10deg = {s['slope_hold_torque_10deg_total_Nm']:.2f} Nm total")
json.dump(summ, open(os.path.join(RES, "model_summary.json"), "w"), indent=1)

# ----------------------------------------------------------------------------- 2. LQR design (baseline)
p = BASELINE
A, B, Bd = p.linearize()
ctrl_lqr, K_lqr, _ = make_lqr(p)
ctrl_lqi, K_lqi, (Aa, Ba) = make_lqr(p, integral=True, Ts=1e-3)
cl_poles = np.linalg.eigvals(A - B @ K_lqr[None, :])
marg = margins_at_input(A, B, K_lqr[None, :])
print("\nLQR gains K = [k_x, k_xdot, k_theta, k_thetadot] =", np.round(K_lqr, 3))
print("LQI gains (Ts=1ms) K = [k_x(0), k_xdot, k_theta, k_thetadot, k_int] =", np.round(K_lqi, 3))
print("closed-loop poles:", np.round(cl_poles, 2))
print("margins at plant input:", {k: round(v, 2) for k, v in marg.items()})

# discrete LQR for several sample rates -> how much do gains change?  (answers 'do I need 1 kHz?')
gain_vs_rate = {}
for fs in (100, 200, 500, 1000, 2000):
    _, Kd, _ = make_lqr(p, Ts=1.0 / fs)
    gain_vs_rate[fs] = Kd.tolist()
    print(f"  dLQR @ {fs:5d} Hz: K = {np.round(Kd, 3)}")

# cascade PID equivalent gains derived from the LQR point (so both are 'well tuned')
kp_th, kd_th = -K_lqr[2], -K_lqr[3]          # positive numbers
kp_v = K_lqr[1] / K_lqr[2]                    # positive: theta_ref per (m/s) of speed error
ctrl_pid = make_cascade_pid(p, kp_th=kp_th, kd_th=kd_th, kp_v=kp_v, ki_v=0.6 * kp_v)
print(f"\nCascade PID equivalent: inner Kp_theta={kp_th:.2f} Nm/rad, Kd_theta={kd_th:.3f} Nm s/rad, "
      f"outer Kp_v={kp_v:.4f} rad/(m/s)  (theta_ref per m/s), Ki_v={0.6*kp_v:.4f}")

# ----------------------------------------------------------------------------- 3. Bode plot
w = np.logspace(-1, 3, 600)
L = ct.ss(A, B, K_lqr[None, :], np.zeros((1, 1)))
mag, ph, _ = ct.frequency_response(L, w)
fig, ax = plt.subplots(2, 1, figsize=(7, 6), sharex=True)
ax[0].semilogx(w, 20 * np.log10(mag.squeeze())); ax[0].axhline(0, color="k", lw=0.5)
ax[0].set_ylabel("|L| dB"); ax[0].set_title(f"Loop at plant input, LQR, {p.name}\nPM={marg['phase_margin_deg']:.0f} deg @ {marg['crossover_rad_s']:.1f} rad/s -> delay margin {marg['delay_margin_ms']:.0f} ms")
ax[1].semilogx(w, np.rad2deg(np.unwrap(ph.squeeze()))); ax[1].set_ylabel("phase deg"); ax[1].set_xlabel("rad/s")
for Td in (0.005, 0.02, 0.05):
    ax[1].semilogx(w, np.rad2deg(np.unwrap(ph.squeeze())) - np.rad2deg(w * Td), "--", lw=0.8, label=f"+{Td*1e3:.0f} ms delay")
ax[1].legend(); ax[1].grid(True, which="both", alpha=0.3); ax[0].grid(True, which="both", alpha=0.3)
fig.tight_layout(); fig.savefig(os.path.join(FIG, "bode_lqr_baseline.png"), dpi=130); plt.close(fig)

# ----------------------------------------------------------------------------- 4. stability map: loop rate x latency
DT_MAP = 5e-4

def stability_map(p, make_ctrl, rates, latencies, T=3.0, label=""):
    res = np.zeros((len(rates), len(latencies)), dtype=bool)
    for i, fs in enumerate(rates):
        Ts = 1.0 / fs
        for j, Td in enumerate(latencies):
            ctrl = make_ctrl(Ts)
            t, xh, uh = simulate(p, ctrl, Ts=Ts, latency=Td, T=T, x0=[0, 0, np.deg2rad(3), 0], noise=False, dt_int=DT_MAP)
            res[i, j] = not fell(xh, T, DT_MAP)
    return res

rates = [50, 100, 200, 500, 1000]
latencies_ms = [0, 5, 10, 15, 20, 25, 30, 40, 50, 60, 80, 100]
maps = {}
for pp in (REPO1_LIKE, BASELINE, HEAVY):
    def mk(Ts, pp=pp):
        # redesign discrete LQR at each rate (fair: best gains for that rate)
        c, _, _ = make_lqr(pp, Ts=Ts)
        return c
    m = stability_map(pp, mk, rates, [l / 1e3 for l in latencies_ms])
    maps[pp.name] = m
    # critical latency per rate
    crit = {}
    for i, fs in enumerate(rates):
        ok = [latencies_ms[j] for j in range(len(latencies_ms)) if m[i, j]]
        crit[fs] = max(ok) if ok else None
    print(f"[{pp.name}] max tolerated latency (ms) vs loop rate (LQR redesigned per rate): {crit}")
    json.dump({"rates_hz": rates, "latency_ms": latencies_ms, "stable": m.tolist(), "max_latency_ms_per_rate": crit},
              open(os.path.join(RES, f"stability_map_{pp.name}.json"), "w"), indent=1)

fig, axs = plt.subplots(1, 3, figsize=(13, 3.8))
for ax, pp in zip(axs, (REPO1_LIKE, BASELINE, HEAVY)):
    m = maps[pp.name]
    ax.imshow(m, cmap="RdYlGn", aspect="auto", origin="lower", vmin=0, vmax=1)
    ax.set_xticks(range(len(latencies_ms))); ax.set_xticklabels(latencies_ms, fontsize=8)
    ax.set_yticks(range(len(rates))); ax.set_yticklabels(rates)
    ax.set_xlabel("total sensor->torque latency [ms]"); ax.set_ylabel("control loop rate [Hz]")
    ax.set_title(f"{pp.name}\n(unstable pole {pp.unstable_pole():.1f} rad/s)", fontsize=9)
fig.suptitle("Balances from 3 deg tilt? (green=yes). LQR torque control, no noise", fontsize=10)
fig.tight_layout(); fig.savefig(os.path.join(FIG, "stability_map_rate_vs_latency.png"), dpi=130); plt.close(fig)

# ----------------------------------------------------------------------------- 5. velocity-mode cascade (repo-1 / ODrive style) latency tolerance
# The balance controller outputs a WHEEL SPEED reference; the motor driver's own speed loop makes torque:
#     tau = Kp_v (w_ref - w_meas) + Ki_v int(w_ref - w_meas)
# With a P-only speed loop this is algebraically a torque-mode state feedback (k_theta = Kp_v kth, ...,
# k_xdot = -Kp_v/r, i.e. the speed loop *resists* wheel motion = wrong sign vs LQR unless the outer loop
# adds +wheel-speed feedback).  With an integrator (ODrive default vel_integrator_gain > 0) the speed loop
# removes the pitch stiffness entirely (tau no longer depends on theta at steady state).
# driver speed-measurement bandwidth: hall sensors (ODrive 3.6 + hoverboard motor) ~8 Hz ; 14-16 bit encoder ~50 Hz+
pr = REPO1_LIKE
Ar, Br, _ = pr.linearize()
_, K_r, _ = make_lqr(pr)
def mk_velmode(pp, bw_meas, with_integrator, with_wheel_fb):
    Kp_v = 0.3 * pp.tau_max               # Nm per rad/s of speed error in the driver
    vel_mode = dict(Kp=Kp_v, Ki=(10.0 * Kp_v if with_integrator else 0.0), bw_meas=bw_meas)
    # outer gains chosen so that the P-only case reproduces the LQR torque law exactly
    kth = -K_r[2] / Kp_v; kthd = -K_r[3] / Kp_v
    kv = (-K_r[1] / Kp_v + 1.0 / pp.r) if with_wheel_fb else 0.0   # (+) feedback of wheel speed cancels the loop's resistance
    def make(Ts):
        def ctrl(xm, v_ref, t):
            return kth * xm[2] + kthd * xm[3] + kv * (xm[1] - v_ref)
        return ctrl
    return make, vel_mode

velmode_results = {}
for label, bw, integ, wfb in [("P-only speed loop, outer PD(theta)+wheel-speed fb, hall-sensor speed (8 Hz)", 8.0, False, True),
                              ("P-only speed loop, outer PD(theta)+wheel-speed fb, encoder speed (50 Hz)", 50.0, False, True),
                              ("PI speed loop (ODrive default), outer PD(theta) only  [repo-1 structure]", 50.0, True, False),
                              ("P-only speed loop, outer PD(theta) only (no wheel fb)", 50.0, False, False)]:
    make, vm = mk_velmode(pr, bw, integ, wfb)
    row = {}
    for fs in (100, 500):
        Ts = 1 / fs
        okl = []
        for Td_ms in latencies_ms:
            t, xh, uh = simulate(pr, make(Ts), Ts=Ts, latency=Td_ms / 1e3, T=3.0,
                                 x0=[0, 0, np.deg2rad(3), 0], vel_mode=vm, dt_int=DT_MAP)
            if not fell(xh, 3.0, DT_MAP):
                okl.append(Td_ms)
        row[fs] = max(okl) if okl else None
    velmode_results[label] = row
    print(f"[repo1_like, VELOCITY-mode: {label}] max tolerated latency ms @100/500 Hz: {row}")
# reference: torque mode, same robot, same LQR law
row = {}
for fs in (100, 500):
    okl = [Td for Td in latencies_ms if not fell(simulate(pr, make_lqr(pr, Ts=1/fs)[0], Ts=1/fs, latency=Td/1e3, T=3.0,
                                                          x0=[0, 0, np.deg2rad(3), 0], dt_int=DT_MAP)[1], 3.0, DT_MAP)]
    row[fs] = max(okl) if okl else None
velmode_results["TORQUE mode (direct drive), same LQR law"] = row
print(f"[repo1_like, TORQUE mode] max tolerated latency ms @100/500 Hz: {row}")
json.dump(velmode_results, open(os.path.join(RES, "velocity_mode_latency_tolerance.json"), "w"), indent=1)

# ----------------------------------------------------------------------------- 6. time-domain comparisons (baseline, with noise)
T = 4.0
scen = {}
cases = {
    "LQR torque, 1 kHz, 1 ms latency":   dict(ctrl=lambda: make_lqr(p, Ts=1e-3)[0], Ts=1e-3, lat=1e-3),
    "LQR torque, 500 Hz, 3 ms latency":  dict(ctrl=lambda: make_lqr(p, Ts=2e-3)[0], Ts=2e-3, lat=3e-3),
    "LQR torque, 100 Hz, 15 ms latency": dict(ctrl=lambda: make_lqr(p, Ts=1e-2)[0], Ts=1e-2, lat=15e-3),
    "LQR torque, 100 Hz, 40 ms latency (PC + Python I/O path)": dict(ctrl=lambda: make_lqr(p, Ts=1e-2)[0], Ts=1e-2, lat=40e-3),
    "Cascade PID torque (LQR-equivalent gains), 1 kHz, 1 ms": dict(ctrl=lambda: make_cascade_pid(p, kp_th, kd_th, kp_v, 0.6 * kp_v), Ts=1e-3, lat=1e-3),
    "Cascade PID, NO wheel-speed feedback (repo-1 structure), 1 kHz": dict(ctrl=lambda: make_cascade_pid(p, kp_th, kd_th, kp_v, 0.6 * kp_v, use_wheel_feedback=False), Ts=1e-3, lat=1e-3),
}
scenarios = {
    "tilt_recovery_12deg": dict(x0=[0, 0, np.deg2rad(12), 0]),
    "push_40N_100ms": dict(x0=None, push=(0.5, 0.1, 40.0)),
}
fig, axs = plt.subplots(2, 3, figsize=(14, 7))
for r_i, (sname, skw) in enumerate(scenarios.items()):
    scen[sname] = {}
    for cname, ckw in cases.items():
        t, xh, uh = simulate(p, ckw["ctrl"](), Ts=ckw["Ts"], latency=ckw["lat"], T=T, noise=True, dt_int=2e-4, **skw)
        ok = not fell(xh, T, 2e-4)
        peak_tau = float(np.max(np.abs(uh)))
        peak_v = float(np.max(np.abs(xh[:, 1])))
        settle = None
        if ok:
            idx = np.where(np.abs(xh[:, 2]) > np.deg2rad(1.0))[0]
            settle = float(t[idx[-1]]) if len(idx) else 0.0
        scen[sname][cname] = dict(stable=bool(ok), peak_torque_total_Nm=round(peak_tau, 2),
                                  peak_torque_per_wheel_Nm=round(peak_tau / 2, 2), peak_speed_m_s=round(peak_v, 2),
                                  settle_1deg_s=settle, rms_torque_total_Nm=round(float(np.sqrt(np.mean(uh ** 2))), 3))
        lab = cname + ("" if ok else "  [FELL]")
        axs[r_i, 0].plot(t, np.rad2deg(xh[:, 2]), label=lab, lw=1)
        axs[r_i, 1].plot(t, xh[:, 1], lw=1)
        axs[r_i, 2].plot(t, uh / 2, lw=0.8)
    axs[r_i, 0].set_ylabel(f"{sname}\npitch [deg]"); axs[r_i, 1].set_ylabel("speed [m/s]"); axs[r_i, 2].set_ylabel("torque per wheel [Nm]")
    axs[r_i, 0].legend(fontsize=6); axs[r_i, 0].set_ylim(-15, 15); axs[r_i, 1].set_ylim(-2.5, 2.5)
    for a in axs[r_i]:
        a.grid(alpha=0.3); a.set_xlabel("t [s]")
fig.suptitle(f"{p.name}: M_b={p.M_b} kg, r={p.r} m, l={p.l} m, tau_max={p.tau_max} Nm/wheel; IMU noise on", fontsize=10)
fig.tight_layout(); fig.savefig(os.path.join(FIG, "time_response_comparison.png"), dpi=130); plt.close(fig)
json.dump(scen, open(os.path.join(RES, "time_response_metrics.json"), "w"), indent=1)
print("\nTime-domain metrics:")
for s, d in scen.items():
    for c, m in d.items():
        print(f"  {s:22s} | {c:52s} | {m}")

# ----------------------------------------------------------------------------- 7. velocity tracking with LQI (1 m/s step) + torque needed
ctrl, Kq, _ = make_lqr(p, integral=True, Ts=1e-3)
t, xh, uh = simulate(p, ctrl, Ts=1e-3, latency=1e-3, T=5.0, noise=True, dt_int=2e-4,
                     v_ref_fn=lambda tt: 1.0 if 0.5 <= tt < 3.0 else 0.0)
fig, axs = plt.subplots(3, 1, figsize=(7, 7), sharex=True)
axs[0].plot(t, xh[:, 1]); axs[0].plot(t, [1.0 if 0.5 <= tt < 3.0 else 0.0 for tt in t], "k--", lw=0.8); axs[0].set_ylabel("speed m/s")
axs[1].plot(t, np.rad2deg(xh[:, 2])); axs[1].set_ylabel("pitch deg")
axs[2].plot(t, uh / 2); axs[2].set_ylabel("torque/wheel Nm"); axs[2].set_xlabel("t s")
for a in axs: a.grid(alpha=0.3)
fig.suptitle("LQI velocity step 0 -> 1 m/s -> 0  (1 kHz, direct-drive torque mode)")
fig.tight_layout(); fig.savefig(os.path.join(FIG, "lqi_velocity_step.png"), dpi=130); plt.close(fig)
vel_metrics = dict(peak_torque_per_wheel_Nm=float(np.max(np.abs(uh)) / 2), peak_lean_deg=float(np.rad2deg(np.max(np.abs(xh[:, 2])))),
                   rise_time_90pct_s=float(t[np.argmax(xh[:, 1] > 0.9)] - 0.5))
print("LQI 1 m/s step:", vel_metrics)
json.dump(vel_metrics, open(os.path.join(RES, "lqi_velocity_step.json"), "w"), indent=1)

# ----------------------------------------------------------------------------- 8. torque / current sizing table
sizing = {}
for pp in (REPO1_LIKE, BASELINE, HEAVY):
    c, _, _ = make_lqr(pp, Ts=1e-3)
    row = {}
    for ang in (5, 10, 15, 20):
        t, xh, uh = simulate(pp, c, Ts=1e-3, latency=1e-3, T=3.0, x0=[0, 0, np.deg2rad(ang), 0], dt_int=DT_MAP)
        row[f"recover_{ang}deg_peak_Nm_per_wheel"] = None if fell(xh, 3.0, DT_MAP) else round(float(np.max(np.abs(uh)) / 2), 2)
        row[f"recover_{ang}deg_peak_speed_m_s"] = None if fell(xh, 3.0, DT_MAP) else round(float(np.max(np.abs(xh[:, 1]))), 2)
    row["accel_1m_s2_Nm_per_wheel"] = round(0.5 * (pp.M_tot + 2 * pp.I_w / pp.r ** 2) * 1.0 * pp.r, 2)
    row["slope_10deg_hold_Nm_per_wheel"] = round(0.5 * pp.M_tot * G * np.sin(np.deg2rad(10)) * pp.r, 2)
    row["slope_15deg_hold_Nm_per_wheel"] = round(0.5 * pp.M_tot * G * np.sin(np.deg2rad(15)) * pp.r, 2)
    row["speed_1p5m_s_wheel_rpm"] = round(1.5 / pp.r * 60 / (2 * np.pi), 1)
    sizing[pp.name] = row
    print(f"[{pp.name}] sizing:", row)
json.dump(sizing, open(os.path.join(RES, "torque_sizing.json"), "w"), indent=1)

json.dump(dict(K_lqr=K_lqr.tolist(), K_lqi_1kHz=K_lqi.tolist(), closed_loop_poles=[str(c) for c in cl_poles],
               margins=marg, gain_vs_rate=gain_vs_rate,
               cascade_equiv=dict(kp_th=float(kp_th), kd_th=float(kd_th), kp_v=float(kp_v), ki_v=float(0.6 * kp_v))),
          open(os.path.join(RES, "lqr_design_baseline.json"), "w"), indent=1)
print("\nDone. Figures in", FIG)
