"""
Linear MPC (OSQP) vs saturated LQR on the same nonlinear TWIP, with a torque-limited
direct-drive motor and a realistic SPEED limit (direct-drive hub motors top out ~200-250 rpm).

Questions answered:
  1. Is the recoverable tilt / push larger with MPC than with LQR + clipping?   (-> no: physics-limited)
  2. The direct-drive trap: near the motor's no-load speed there is no forward torque left, so the robot
     cannot lean back to decelerate. Does MPC with a torque-speed envelope + speed constraint avoid it,
     and does "LQR + clipped/ramped reference" already get most of that benefit?
  3. Cost of one MPC solve (OSQP on this PC) -> extrapolate to Teensy 4.1 with TinyMPC poster numbers.
"""
import json, os, time
import numpy as np
import scipy.sparse as sp
import scipy.linalg as la
import control as ct
import osqp
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from twip_model import TWIPParams, BASELINE, G

HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, "..", "figures"); RES = os.path.join(HERE, "..", "results")
os.makedirs(FIG, exist_ok=True); os.makedirs(RES, exist_ok=True)

# --- torque- and speed-limited version of the planned robot (e.g. DDT M1505A: 3 Nm rated, ~7.5 Nm stall,
#     no-load 250 rpm -> r = 0.0825 m: 2.1 m/s no-load)
p = TWIPParams(**{**BASELINE.__dict__, "name": "planned_10kg_speed_torque_limited", "tau_max": 5.0})
TAU_MAX_TOTAL = 2 * p.tau_max     # both wheels
V_NL = 2.1                         # m/s  no-load speed of the wheel motor -> torque envelope
V_MAX = 1.5                        # m/s  speed limit we want enforced (~70 % of no-load keeps torque margin)
TH_MAX = np.deg2rad(45)
TS = 0.01                          # MPC / LQR update period (100 Hz)
N = 30                             # horizon = 0.3 s (~2 unstable time constants)
LAT = 0.002                        # 2 ms compute + CAN latency
DT = 5e-4
W_SLACK = 1e4                      # soft speed-constraint weight

A, B, _ = p.linearize()
sysd = ct.c2d(ct.ss(A, B, np.eye(4), np.zeros((4, 1))), TS)
Ad, Bd = np.asarray(sysd.A), np.asarray(sysd.B)
Q = np.diag([0.0, 1 / 1.0 ** 2, 1 / 0.2 ** 2, 1 / 2.0 ** 2])      # velocity-tracking form (no position hold)
R = np.array([[1 / (0.5 * p.tau_max) ** 2]])
P_inf = la.solve_discrete_are(Ad, Bd, Q, R)
K_d = np.linalg.solve(R + Bd.T @ P_inf @ Bd, Bd.T @ P_inf @ Ad).ravel()
nx, nu = 4, 1


class MPC:
    """z = [x_0..x_N (4 each), u_0..u_{N-1}, s_0..s_N (speed slack)]"""
    def __init__(self, envelope=True):
        nX, nU, nS = (N + 1) * nx, N * nu, N + 1
        self.nX, self.nU, self.nS = nX, nU, nS
        self.P = sp.block_diag([sp.kron(sp.eye(N), Q), P_inf, sp.kron(sp.eye(N), R), W_SLACK * sp.eye(nS)], format="csc")
        # dynamics
        Ax = sp.kron(sp.eye(N + 1), -sp.eye(nx)) + sp.kron(sp.eye(N + 1, k=-1), Ad)
        Bu = sp.kron(sp.vstack([sp.csc_matrix((1, N)), sp.eye(N)]), Bd)
        Aeq = sp.hstack([Ax, Bu, sp.csc_matrix((nX, nS))])
        # hard box: theta, torque
        Abox = sp.hstack([sp.eye(nX + nU), sp.csc_matrix((nX + nU, nS))])
        xmin = np.array([-np.inf, -np.inf, -TH_MAX, -np.inf]); xmax = -xmin
        lbox = np.hstack([np.kron(np.ones(N + 1), xmin), -TAU_MAX_TOTAL * np.ones(N)])
        ubox = np.hstack([np.kron(np.ones(N + 1), xmax), TAU_MAX_TOTAL * np.ones(N)])
        # torque-speed envelope (back-EMF), linear:  -tau_max <= tau_k + c v_k <= tau_max,  c = tau_max/v_nl
        c_env = TAU_MAX_TOTAL / V_NL if envelope else 0.0
        Sv = sp.kron(sp.eye(N, N + 1), sp.csc_matrix(np.array([[0, c_env, 0, 0]])))
        Aenv = sp.hstack([Sv, sp.eye(N), sp.csc_matrix((N, nS))])
        lenv = -TAU_MAX_TOTAL * np.ones(N); uenv = TAU_MAX_TOTAL * np.ones(N)
        # soft speed limit:  v_k - s_k <= V_MAX ,  -v_k - s_k <= V_MAX ,  s_k >= 0
        Ev = sp.kron(sp.eye(N + 1), sp.csc_matrix(np.array([[0, 1, 0, 0]])))
        Asp = sp.hstack([Ev, sp.csc_matrix((nS, nU)), -sp.eye(nS)])
        Asn = sp.hstack([-Ev, sp.csc_matrix((nS, nU)), -sp.eye(nS)])
        Ass = sp.hstack([sp.csc_matrix((nS, nX + nU)), sp.eye(nS)])
        self.A = sp.vstack([Aeq, Abox, Aenv, Asp, Asn, Ass], format="csc")
        self.l_tail = np.hstack([lbox, lenv, -np.inf * np.ones(nS), -np.inf * np.ones(nS), np.zeros(nS)])
        self.u_tail = np.hstack([ubox, uenv, V_MAX * np.ones(nS), V_MAX * np.ones(nS), np.inf * np.ones(nS)])
        self.q = np.zeros(nX + nU + nS)
        self.prob = osqp.OSQP()
        l = np.hstack([np.zeros(nX), self.l_tail]); u = np.hstack([np.zeros(nX), self.u_tail])
        self.prob.setup(self.P, self.q, self.A, l, u, warm_starting=True, verbose=False,
                        eps_abs=1e-4, eps_rel=1e-4, max_iter=400, polishing=False)
        self.times, self.iters, self.fallbacks = [], [], 0

    def set_ref(self, v_ref):
        xr = np.array([0, float(np.clip(v_ref, -V_MAX, V_MAX)), 0, 0])
        self.q = np.hstack([np.kron(np.ones(N), -Q @ xr), -P_inf @ xr, np.zeros(self.nU + self.nS)])

    def __call__(self, x, v_ref, t):
        self.set_ref(v_ref)
        leq = np.hstack([-x, np.zeros(N * nx)])
        l = np.hstack([leq, self.l_tail]); u = np.hstack([leq, self.u_tail])
        upd = getattr(self.prob, "update_data_vec", None)
        if upd is not None:
            upd(q=self.q, l=l, u=u)
        else:
            self.prob.update(q=self.q, l=l, u=u)
        t0 = time.perf_counter()
        res = self.prob.solve()
        self.times.append(time.perf_counter() - t0)
        self.iters.append(int(res.info.iter))
        if "solved" not in str(res.info.status):
            self.fallbacks += 1
            return lqr_clipped_ref(x, v_ref, t)
        return float(res.x[self.nX])


def lqr_raw_ref(x, v_ref, t):
    return float(np.clip(-K_d @ (x - np.array([0, v_ref, 0, 0])), -TAU_MAX_TOTAL, TAU_MAX_TOTAL))


_ramp = {"v": 0.0}
def lqr_clipped_ref(x, v_ref, t, rate=1.0):
    """practical LQR: reference clipped to V_MAX and ramped at `rate` m/s^2 (what the Teensy firmware does)"""
    target = float(np.clip(v_ref, -V_MAX, V_MAX))
    _ramp["v"] += float(np.clip(target - _ramp["v"], -rate * TS, rate * TS))
    return float(np.clip(-K_d @ (x - np.array([0, _ramp["v"], 0, 0])), -TAU_MAX_TOTAL, TAU_MAX_TOTAL))


def simulate(ctrl, T=3.0, x0=None, push=None, v_ref_fn=None, Ts=TS, lat=LAT):
    """Same integrator as analysis_controllers.simulate; adds the motor torque-speed envelope (back-EMF):
       torque available in the direction of motion falls linearly to zero at the no-load speed."""
    _ramp["v"] = 0.0
    n = int(round(T / DT)); sub = int(round(Ts / DT)); dsteps = int(round(lat / DT))
    x = np.zeros(4) if x0 is None else np.array(x0, float)
    fifo = [0.0] * (dsteps + 1); u_cmd = 0.0
    th, xh, uh = [], [], []
    for k in range(n):
        t = k * DT
        v_ref = v_ref_fn(t) if v_ref_fn else 0.0
        if k % sub == 0:
            u_cmd = ctrl(x.copy(), v_ref, t)
        fifo.append(u_cmd); ud = fifo.pop(0)
        tau = float(np.clip(ud, -TAU_MAX_TOTAL, TAU_MAX_TOTAL))
        frac = max(0.0, 1.0 - abs(x[1]) / V_NL)
        if x[1] != 0 and np.sign(tau) == np.sign(x[1]):
            tau = float(np.clip(tau, -TAU_MAX_TOTAL * frac, TAU_MAX_TOTAL * frac))
        F = push[2] if (push and push[0] <= t < push[0] + push[1]) else 0.0
        f = p.f
        k1 = f(x, tau, F); k2 = f(x + 0.5 * DT * k1, tau, F); k3 = f(x + 0.5 * DT * k2, tau, F); k4 = f(x + DT * k3, tau, F)
        x = x + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        th.append(t); xh.append(x.copy()); uh.append(tau)
        if abs(x[2]) > np.deg2rad(70):
            break
    th, xh, uh = np.array(th), np.array(xh), np.array(uh)
    ok = len(th) >= n - 1 and np.max(np.abs(xh[-2000:, 2])) < np.deg2rad(3)   # did not fall, settled within 3 deg
    return th, xh, uh, ok


out = {"setup": dict(robot=p.__dict__, tau_max_total=TAU_MAX_TOTAL, v_max=V_MAX, v_no_load=V_NL, Ts=TS, N=N, latency=LAT,
                     K_dlqr_100Hz=K_d.tolist())}

# ---------------------------------------------------------------- 1. recoverable initial tilt
tilts = np.arange(5, 46, 2.5)
rec = {"LQR_sat_100Hz": [], "MPC_100Hz": []}
for ang in tilts:
    _, _, _, ok1 = simulate(lqr_raw_ref, x0=[0, 0, np.deg2rad(ang), 0])
    m = MPC(); _, _, _, ok2 = simulate(m, x0=[0, 0, np.deg2rad(ang), 0])
    rec["LQR_sat_100Hz"].append(bool(ok1)); rec["MPC_100Hz"].append(bool(ok2))
max_tilt = {k: float(max([a for a, o in zip(tilts, v) if o], default=0)) for k, v in rec.items()}
print("max recoverable tilt [deg]:", max_tilt)
out["max_recoverable_tilt_deg"] = max_tilt

# ---------------------------------------------------------------- 2. survivable push (0.1 s)
forces = np.arange(20, 201, 10)
surv = {"LQR_sat_100Hz": [], "MPC_100Hz": []}
for F in forces:
    _, _, _, ok1 = simulate(lqr_raw_ref, push=(0.3, 0.1, float(F)))
    m = MPC(); _, _, _, ok2 = simulate(m, push=(0.3, 0.1, float(F)))
    surv["LQR_sat_100Hz"].append(bool(ok1)); surv["MPC_100Hz"].append(bool(ok2))
max_push = {k: float(max([f for f, o in zip(forces, v) if o], default=0)) for k, v in surv.items()}
print("max survivable push [N for 0.1 s]:", max_push)
out["max_survivable_push_N_100ms"] = max_push

# ---------------------------------------------------------------- 3. velocity steps near the speed ceiling
out["velocity_steps"] = {}
fig, axs = plt.subplots(3, 3, figsize=(14, 8), sharex=True)
m_last = None
for col, vstep in enumerate((1.0, 1.5, 2.0)):
    vref = (lambda vs: (lambda t: vs if 0.5 <= t < 3.5 else 0.0))(vstep)
    runs = {
        "LQR, raw reference": simulate(lqr_raw_ref, T=7.0, v_ref_fn=vref),
        "LQR, ref clipped to 1.5 + ramp 1 m/s^2": simulate(lqr_clipped_ref, T=7.0, v_ref_fn=vref),
    }
    m = MPC(); runs["MPC (soft |v|<=1.5, torque-speed envelope)"] = simulate(m, T=7.0, v_ref_fn=vref)
    m_last = m
    out["velocity_steps"][f"{vstep}"] = {k: dict(stable=bool(v[3]), peak_speed=float(np.max(np.abs(v[1][:, 1]))),
                                                 peak_pitch_deg=float(np.rad2deg(np.max(np.abs(v[1][:, 2])))),
                                                 peak_torque_per_wheel=float(np.max(np.abs(v[2])) / 2)) for k, v in runs.items()}
    print(f"velocity step {vstep} m/s:")
    for k, v in out["velocity_steps"][f"{vstep}"].items():
        print(f"    {k:45s} {v}")
    for k, (tt, xx, uu, ok) in runs.items():
        axs[0, col].plot(tt, xx[:, 1], label=f"{k}{' (FELL)' if not ok else ''}", lw=1)
        axs[1, col].plot(tt, np.rad2deg(xx[:, 2]), lw=1)
        axs[2, col].plot(tt, uu / 2, lw=0.8)
    tt = runs["LQR, raw reference"][0]
    axs[0, col].plot(tt, [vref(t) for t in tt], "k--", lw=0.8); axs[0, col].axhline(V_NL, color="r", lw=0.6, ls=":"); axs[0, col].axhline(V_MAX, color="g", lw=0.6, ls=":")
    axs[0, col].set_ylabel("speed m/s"); axs[0, col].legend(fontsize=6); axs[0, col].set_title(f"step to {vstep} m/s  (no-load {V_NL} m/s)", fontsize=9); axs[0, col].set_ylim(-0.6, 2.4)
    axs[1, col].set_ylabel("pitch deg"); axs[1, col].set_ylim(-30, 30)
    axs[2, col].set_ylabel("torque/wheel Nm"); axs[2, col].set_xlabel("t s")
    for a in axs[:, col]: a.grid(alpha=0.3)
fig.suptitle(f"Velocity steps, tau_max {p.tau_max} Nm/wheel, motor no-load speed {V_NL} m/s, 100 Hz update, 2 ms latency", fontsize=10)
fig.tight_layout(); fig.savefig(os.path.join(FIG, "mpc_vs_lqr_velocity_step.png"), dpi=130); plt.close(fig)

m = m_last
solve_ms = np.array(m.times) * 1e3
out["osqp_solve_time_ms_on_this_pc"] = dict(mean=float(solve_ms.mean()), p95=float(np.percentile(solve_ms, 95)), max=float(solve_ms.max()),
                                            iters_mean=float(np.mean(m.iters)), iters_p95=float(np.percentile(m.iters, 95)), iters_max=int(np.max(m.iters)),
                                            fallbacks=int(m.fallbacks), n_vars=int(m.nX + m.nU + m.nS), n_constraints=int(m.A.shape[0]))
print("OSQP solve time on this PC [ms]:", out["osqp_solve_time_ms_on_this_pc"])

# ---------------------------------------------------------------- 4. tilt recovery traces at the limit
ang = max_tilt["LQR_sat_100Hz"]
tl, xl, ul, okl = simulate(lqr_raw_ref, x0=[0, 0, np.deg2rad(ang), 0])
m = MPC(); tm, xm, um, okm = simulate(m, x0=[0, 0, np.deg2rad(ang), 0])
fig, axs = plt.subplots(3, 1, figsize=(7, 7.5), sharex=True)
axs[0].plot(tl, np.rad2deg(xl[:, 2]), label=f"LQR + clip{' (fell)' if not okl else ''}"); axs[0].plot(tm, np.rad2deg(xm[:, 2]), label=f"MPC{' (fell)' if not okm else ''}"); axs[0].set_ylabel("pitch deg"); axs[0].legend(fontsize=8)
axs[1].plot(tl, xl[:, 1]); axs[1].plot(tm, xm[:, 1]); axs[1].axhline(V_NL, color="r", lw=0.6, ls=":"); axs[1].set_ylabel("speed m/s")
axs[2].plot(tl, ul / 2); axs[2].plot(tm, um / 2); axs[2].set_ylabel("torque/wheel Nm"); axs[2].set_xlabel("t s")
for a in axs: a.grid(alpha=0.3)
fig.suptitle(f"Recovery from {ang:.1f} deg initial tilt (torque {p.tau_max} Nm/wheel, no-load speed {V_NL} m/s)", fontsize=10)
fig.tight_layout(); fig.savefig(os.path.join(FIG, "mpc_vs_lqr_tilt_recovery.png"), dpi=130); plt.close(fig)

# ---------------------------------------------------------------- 5. Teensy extrapolation from the TinyMPC ICRA-2024 poster (Teensy 4.1, 600 MHz)
# poster (https://a2r-lab.org/files/TinyMPC_ICRA_Poster.pdf): time per ADMM iteration on Teensy 4.1
#   vs horizon (12-state problem): N=12: 72 us, N=16: 80 us, N=30: 152 us, N=40: 195 us
#   vs state size (default horizon): 4 states: 15 us, 8 states: 38 us, 12 states: 63 us
it_typ, it_p95 = float(np.mean(m_last.iters)), float(np.percentile(m_last.iters, 95))
est = dict(iters_typ=it_typ, iters_p95=it_p95,
           teensy_ms_typ_conservative_152us=it_typ * 152e-3, teensy_ms_p95_conservative_152us=it_p95 * 152e-3,
           teensy_ms_typ_optimistic_45us=it_typ * 45e-3, teensy_ms_p95_optimistic_45us=it_p95 * 45e-3,
           note="152 us/iter = poster N=30 12-state; 45 us/iter = poster 4-state figure scaled to N=30")
out["teensy_mpc_estimate"] = est
print("Teensy MPC estimate:", est)
json.dump(out, open(os.path.join(RES, "mpc_vs_lqr.json"), "w"), indent=1, default=str)
print("done")
