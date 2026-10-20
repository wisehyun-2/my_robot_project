"""
sil_harness.py -- software-in-the-loop benchmark for balance controllers written in C.

Each controller in controllers/*.c is compiled to a shared library (gcc -shared -fPIC) exposing the ctrl_api.h
interface and driven by the nonlinear two-wheeled-inverted-pendulum model (sim/twip_model.py) with:
  * controller period Ts (the project's own loop rate), sensor->actuator latency, ZOH
  * torque-mode output (direct drive) or wheel-velocity output through a simulated driver speed loop
  * IMU noise, leg-length (hip) changes that move the body COM up/down
Benchmarks: tilt recovery, push rejection, velocity step, latency tolerance, leg-length change, torque chatter.

usage: python3 sil_harness.py [--robot small|large] [--ctrl name ...] [--out results.json]
"""
import argparse, ctypes, json, os, subprocess, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim"))
from twip_model import TWIPParams, G  # noqa: E402

ROBOTS = {
    "small": TWIPParams(name="small_5kg", M_b=3.2, m_w=0.65, r=0.065, l=0.16, I_b=0.07, b_x=0.3, d=0.30, I_z=0.08, tau_max=5.0, Kt=0.45),
    "large": TWIPParams(name="large_10kg", M_b=7.0, m_w=0.65, r=0.0825, l=0.25, I_b=0.20, b_x=0.5, d=0.40, I_z=0.25, tau_max=5.0, Kt=0.45),    # Kollarcik (CTU 2021) Stoupento: pendulum 4.0 kg, wheels 0.3 kg, r 70 mm, l 0.29 m, Ipy 0.0119, I_w 7.35e-4, eX8108 1:16.5 +-2 Nm
    "kollarcik": TWIPParams(name="kollarcik_ctu_4.6kg", M_b=4.0, m_w=0.3, r=0.07, l=0.29, I_b=0.0119, I_w_factor=0.5, b_x=0.3, d=0.29, I_z=0.03, tau_max=2.0, Kt=0.1),
}
DT = 5e-4


class CtrlIn(ctypes.Structure):
    _fields_ = [(n, ctypes.c_float) for n in ("theta", "theta_dot", "x_dot", "psi_dot", "leg_len", "wheel_r", "v_ref", "yaw_rate_ref", "t")]


class CtrlParams(ctypes.Structure):
    _fields_ = [(n, ctypes.c_float) for n in ("M_b", "m_w", "r", "l", "I_b", "I_w", "tau_max")]


class Controller:
    def __init__(self, c_path, Ts, extra_cflags=()):
        so = os.path.splitext(c_path)[0] + ".so"
        if not os.path.exists(so) or os.path.getmtime(so) < os.path.getmtime(c_path):
            cmd = ["gcc", "-O2", "-shared", "-fPIC", "-o", so, c_path, "-lm", *extra_cflags]
            subprocess.check_call(cmd)
        self.lib = ctypes.CDLL(so)
        self.lib.ctrl_init.argtypes = [ctypes.c_float, ctypes.POINTER(CtrlParams)]
        self.lib.ctrl_step.argtypes = [ctypes.POINTER(CtrlIn), ctypes.POINTER(ctypes.c_float * 2)]
        self.lib.ctrl_name.restype = ctypes.c_char_p
        self.Ts = Ts
        self.out_type = self.lib.ctrl_output_type()
        self.name = self.lib.ctrl_name().decode()
        self.has_scale = hasattr(self.lib, "ctrl_set_scale")
        if self.has_scale:
            self.lib.ctrl_set_scale.argtypes = [ctypes.c_float]
        if hasattr(self.lib, "ctrl_set_param"):
            self.lib.ctrl_set_param.argtypes = [ctypes.c_char_p, ctypes.c_float]

    def set_scale(self, s):
        self.lib.ctrl_set_scale(ctypes.c_float(s))

    def set_param(self, name, v):
        self.lib.ctrl_set_param(name.encode(), ctypes.c_float(v))

    def init(self, p):
        pr = CtrlParams(p.M_b, p.m_w, p.r, p.l, p.I_b, p.I_w, p.tau_max)
        self.lib.ctrl_init(ctypes.c_float(self.Ts), ctypes.byref(pr))

    def step(self, theta, theta_dot, x_dot, psi_dot, leg_len, r, v_ref, yaw_ref, t):
        inp = CtrlIn(theta, theta_dot, x_dot, psi_dot, leg_len, r, v_ref, yaw_ref, t)
        out = (ctypes.c_float * 2)()
        self.lib.ctrl_step(ctypes.byref(inp), ctypes.byref(out))
        return float(out[0]), float(out[1])


def make_controller(nm, META, cdir=None):
    """Build the controller named `nm` from controllers/meta.json: a compiled C controller (default) or, when the
    entry has "py": "<module>", a Python controller module exposing make(Ts, **kw) with the same duck-type interface."""
    cdir = cdir or os.path.join(HERE, "controllers")
    m = META[nm]
    if m.get("py"):
        import importlib.util
        spec = importlib.util.spec_from_file_location(m["py"], os.path.join(cdir, m["py"] + ".py"))
        mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
        ctrl = mod.make(m.get("Ts", 1e-3), **m.get("py_kwargs", {}))
    else:
        ctrl = Controller(os.path.join(cdir, m.get("src", nm) + ".c"), Ts=m.get("Ts", 1e-3))
    for k, v in m.get("params", {}).items():
        ctrl.set_param(k, v)
    if hasattr(ctrl, "lib") and m.get("params"):
        ctrl.name = ctrl.lib.ctrl_name().decode()          # name may depend on the parameters (e.g. wheelleg native)
    return ctrl


def simulate(p, ctrl, T=4.0, x0=None, push=None, v_ref_fn=None, latency=0.002, noise=True, leg_fn=None,
             sign=+1.0, vel_loop=None, seed=0):
    """vel_loop: dict(Kp, Ki, bw_meas) for CTRL_OUT_WHEELVEL controllers (driver speed loop made of torque)."""
    rng = np.random.default_rng(seed)
    n = int(round(T / DT)); sub = max(1, int(round(ctrl.Ts / DT))); dsteps = int(round(latency / DT))
    x = np.zeros(4) if x0 is None else np.array(x0, float)
    fifo = [(0.0, 0.0)] * (dsteps + 1); cmd = (0.0, 0.0)
    tmax = p.tau_max
    vel_int = [0.0, 0.0]; w_f = [0.0, 0.0]
    ctrl.init(p)
    pl = TWIPParams(**p.__dict__)
    hist_t, hist_x, hist_u = [], [], []
    for k in range(n):
        t = k * DT
        l_now = leg_fn(t) if leg_fn else p.l
        if l_now != pl.l:
            pl.l = l_now; pl.I_b = p.I_b * (l_now / p.l) ** 2
        v_ref = v_ref_fn(t) if v_ref_fn else 0.0
        if k % sub == 0:
            th, thd, xd = x[2], x[3], x[1]
            if noise:
                th += rng.normal(0, np.deg2rad(0.2)); thd += rng.normal(0, np.deg2rad(0.5))
                q = 2 * np.pi / 16384 * p.r; xd += rng.uniform(-q, q) / ctrl.Ts
            # sign flips the OUTPUT only: a linear controller is odd, so flipping inputs and outputs together is the
            # identity; what differs between projects is the motor/IMU polarity, i.e. the sign of the output alone.
            cmd = ctrl.step(th, thd, xd, 0.0, l_now, p.r, v_ref, 0.0, t)
            cmd = (sign * cmd[0], sign * cmd[1])
        fifo.append(cmd); cL, cR = fifo.pop(0)
        if ctrl.out_type == 0:
            tL = float(np.clip(cL, -tmax, tmax)); tR = float(np.clip(cR, -tmax, tmax))
        else:
            w = x[1] / p.r
            a = DT * 2 * np.pi * vel_loop["bw_meas"]
            taus = []
            for i, c in enumerate((cL, cR)):
                w_f[i] += a * (w - w_f[i])
                e = c - w_f[i]
                vel_int[i] += e * DT
                tu = vel_loop["Kp"] * e + vel_loop["Ki"] * vel_int[i]
                ts = float(np.clip(tu, -tmax, tmax))
                if ts != tu: vel_int[i] -= e * DT
                taus.append(ts)
            tL, tR = taus
        tau = tL + tR
        F = push[2] if (push and push[0] <= t < push[0] + push[1]) else 0.0
        f = pl.f
        k1 = f(x, tau, F); k2 = f(x + 0.5 * DT * k1, tau, F); k3 = f(x + 0.5 * DT * k2, tau, F); k4 = f(x + DT * k3, tau, F)
        x = x + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        hist_t.append(t); hist_x.append(x.copy()); hist_u.append(tau)
        if abs(x[2]) > np.deg2rad(60) or abs(x[1]) > 6.0:
            break
    hist_t, hist_x, hist_u = np.array(hist_t), np.array(hist_x), np.array(hist_u)
    settled = len(hist_t) >= n - 1 and np.max(np.abs(hist_x[-2000:, 2])) < np.deg2rad(3) and np.max(np.abs(hist_x[-2000:, 1] - (v_ref_fn(T) if v_ref_fn else 0))) < 0.3
    return dict(ok=bool(settled), t=hist_t, x=hist_x, u=hist_u)


def benchmark(p, ctrl, sign=+1.0, vel_loop=None, latency=0.002):
    kw = dict(sign=sign, vel_loop=vel_loop)
    res = {}
    # 1 tilt recovery
    tilts = np.arange(2.5, 41, 2.5)
    ok_t = [a for a in tilts if simulate(p, ctrl, x0=[0, 0, np.deg2rad(a), 0], latency=latency, **kw)["ok"]]
    res["max_tilt_deg"] = float(max(ok_t)) if ok_t else 0.0
    # 2 push
    forces = range(5, 161, 5)
    ok_f = [F for F in forces if simulate(p, ctrl, push=(0.5, 0.1, float(F)), latency=latency, **kw)["ok"]]
    res["max_push_N_100ms"] = float(max(ok_f)) if ok_f else 0.0
    # 3 velocity step 0.8 m/s (0.5 .. 3.0 s)
    r = simulate(p, ctrl, T=7.0, v_ref_fn=lambda t: 0.8 if 0.5 <= t < 3.0 else 0.0, latency=latency, **kw)
    if r["ok"]:
        v = r["x"][:, 1]; t = r["t"]
        i90 = np.argmax(v > 0.72)
        res["vstep_rise_s"] = float(t[i90] - 0.5) if v.max() > 0.72 else None
        res["vstep_overshoot_pct"] = float((v.max() - 0.8) / 0.8 * 100)
        res["vstep_peak_pitch_deg"] = float(np.rad2deg(np.max(np.abs(r["x"][:, 2]))))
        res["vstep_ok"] = True
    else:
        res["vstep_ok"] = False
    # 4 latency tolerance (3 deg tilt)
    lats = [0, 5, 10, 15, 20, 25, 30, 40, 50, 60, 80]
    ok_l = [L for L in lats if simulate(p, ctrl, x0=[0, 0, np.deg2rad(3), 0], latency=L / 1e3, **kw)["ok"]]
    res["max_latency_ms"] = float(max(ok_l)) if ok_l else None
    # 5 leg length change while balancing: l -> 1.4 l over 1 s, hold, back
    def leg_fn(t):
        if t < 1.0: return p.l
        if t < 2.0: return p.l * (1 + 0.4 * (t - 1.0))
        if t < 3.0: return p.l * 1.4
        if t < 4.0: return p.l * (1.4 - 0.4 * (t - 3.0))
        return p.l
    r = simulate(p, ctrl, T=5.5, leg_fn=leg_fn, latency=latency, **kw)
    res["leg_change_ok"] = r["ok"]; res["leg_change_peak_pitch_deg"] = float(np.rad2deg(np.max(np.abs(r["x"][:, 2])))) if len(r["x"]) else None
    # 6 torque chatter (noise) at rest
    r = simulate(p, ctrl, T=3.0, latency=latency, **kw)
    res["rest_torque_rms_Nm"] = float(np.sqrt(np.mean(r["u"][-4000:] ** 2))) if r["ok"] else None
    res["rest_pitch_rms_deg"] = float(np.rad2deg(np.sqrt(np.mean(r["x"][-4000:, 2] ** 2)))) if r["ok"] else None
    return res


def pick_sign(p, ctrl, vel_loop):
    """velocity-mode projects have unknown sign conventions; keep the sign that balances (if any)."""
    for s in (+1.0, -1.0):
        if simulate(p, ctrl, x0=[0, 0, np.deg2rad(2), 0], noise=False, sign=s, vel_loop=vel_loop)["ok"]:
            return s
    return None


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="small")
    ap.add_argument("--ctrl", nargs="*", default=None, help="controller base names (default: all in controllers/)")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    p = ROBOTS[args.robot]
    cdir = os.path.join(HERE, "controllers")
    META = json.load(open(os.path.join(HERE, "controllers", "meta.json")))
    names = args.ctrl or [k for k in META.keys()]
    results = {}
    for nm in names:
        m = META.get(nm, {})
        ctrl = make_controller(nm, META, cdir)
        vel_loop = m.get("vel_loop") if ctrl.out_type == 1 else None
        if m.get("scale_search"):
            best = None
            for s_ in np.logspace(m["scale_search"][0], m["scale_search"][1], m["scale_search"][2]):
                ctrl.set_scale(float(s_))
                sgn = pick_sign(p, ctrl, vel_loop)
                if sgn is None:
                    continue
                # score: survive the largest push, then the largest tilt (cheap sweeps)
                sc = 0.0
                for F in (10, 20, 30, 40, 60, 80):
                    if simulate(p, ctrl, push=(0.5, 0.1, float(F)), latency=m.get("latency", 0.002), sign=sgn, vel_loop=vel_loop)["ok"]:
                        sc = F
                    else:
                        break
                if best is None or sc > best[0]:
                    best = (sc, float(s_), sgn)
            if best is None:
                results[nm] = dict(name=ctrl.name, Ts=ctrl.Ts, note="no output scale makes it balance on this robot")
                print(f"[{nm}] {ctrl.name}: no scale balances"); continue
            ctrl.set_scale(best[1]); m["autosign"] = True
            print(f"[{nm}] best output scale {best[1]:.4g} (push score {best[0]} N)")
        sign = +1.0
        if ctrl.out_type == 1 or m.get("autosign"):
            sign = pick_sign(p, ctrl, vel_loop)
            if sign is None:
                results[nm] = dict(name=ctrl.name, Ts=ctrl.Ts, note="does not balance with either sign on this robot")
                print(f"[{nm}] {ctrl.name}: does NOT balance (either sign)"); continue
        res = benchmark(p, ctrl, sign=sign, vel_loop=vel_loop, latency=m.get("latency", 0.002))
        res.update(name=ctrl.name, Ts=ctrl.Ts, sign=sign, output="torque" if ctrl.out_type == 0 else "wheel-velocity",
                   scale=(best[1] if m.get("scale_search") else None), params=m.get("params"))
        results[nm] = res
        print(f"[{nm}] {ctrl.name}\n    {json.dumps({k: v for k, v in res.items() if k != 'name'})}")
    if args.out:
        json.dump(results, open(args.out, "w"), indent=1)
