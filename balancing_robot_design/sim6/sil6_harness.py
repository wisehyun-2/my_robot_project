"""
sil6_harness.py -- benchmark of six-motor wheel-leg balance controllers on the three-body plant (plant6.py).
Same test battery as sil/sil_harness.py: tilt recovery, push, 0.8 m/s step, latency tolerance, leg-length change,
rest chatter.  Sign conventions of each project (theta, phi, T, Tp) are unknown in the model frame, so the harness
searches the 16 sign combinations (plus an optional output scale) and keeps the one that balances best.

usage: python3 sil6_harness.py --plant rm|liu|small|large [--ctrl names] [--out results.json] [--scale]
"""
import argparse, json, os, sys, itertools
import numpy as np
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
from plant6 import Plant6, RM_CLASS, LIU_CLASS, SKY_CLASS, SMALL5, LARGE10   # noqa: E402
from controllers6 import REGISTRY                                           # noqa: E402

PLANTS = {"rm": RM_CLASS, "liu": LIU_CLASS, "sky": SKY_CLASS, "small": SMALL5, "large": LARGE10}
DT = 1e-3
# output scales tried when a law does not balance as published (robot mass / leg length differ from its native robot)
SCALES = [1.0, 0.5, 2.0, 0.25, 4.0, 0.125, 8.0, 0.0625, 16.0, 32.0, 64.0, 128.0]


def simulate(plant, ctrl, signs=(1, 1, 1, 1), T=3.0, X0=None, push=None, v_ref_fn=None, latency=0.002, noise=False,
             L0_fn=None, seed=0):
    sth, sph, sT, sTp = signs
    rng = np.random.default_rng(seed)
    n = int(round(T / DT)); sub = max(1, int(round(ctrl.Ts / DT))); dsteps = int(round(latency / DT))
    X = np.zeros(6) if X0 is None else np.array(X0, float)
    fifo = [(0.0, 0.0)] * (dsteps + 1); cmd = (0.0, 0.0)
    Tmax = 2 * plant.tau_wheel_max; Tpmax = 2 * plant.tau_hip_max
    ctrl.init(plant)
    th, xh, uh = [], [], []
    for k in range(n):
        t = k * DT
        L0 = L0_fn(t) if L0_fn else plant.L0
        v_ref = v_ref_fn(t) if v_ref_fn else 0.0
        if k % sub == 0:
            Xm = X.copy()
            if noise:
                Xm[0] += rng.normal(0, np.deg2rad(0.1)); Xm[1] += rng.normal(0, np.deg2rad(0.5))
                Xm[4] += rng.normal(0, np.deg2rad(0.2)); Xm[5] += rng.normal(0, np.deg2rad(0.5))
                Xm[3] += rng.uniform(-1, 1) * 2 * np.pi / 16384 * plant.R / ctrl.Ts
            Xs = Xm.copy(); Xs[0] *= sth; Xs[1] *= sth; Xs[4] *= sph; Xs[5] *= sph
            Tc, Tpc = ctrl.step(Xs, L0, v_ref, t)
            cmd = (sT * Tc, sTp * Tpc)
        fifo.append(cmd); Tc, Tpc = fifo.pop(0)
        Tc = float(np.clip(Tc, -Tmax, Tmax)); Tpc = float(np.clip(Tpc, -Tpmax, Tpmax))
        F = push[2] if (push and push[0] <= t < push[0] + push[1]) else 0.0
        f = lambda Y: plant.deriv(Y, Tc, Tpc, F, L0)
        k1 = f(X); k2 = f(X + 0.5 * DT * k1); k3 = f(X + 0.5 * DT * k2); k4 = f(X + DT * k3)
        X = X + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        th.append(t); xh.append(X.copy()); uh.append((Tc, Tpc))
        if abs(X[0]) > np.deg2rad(60) or abs(X[4]) > np.deg2rad(60) or abs(X[3]) > 6:
            break
    th, xh, uh = np.array(th), np.array(xh), np.array(uh)
    vend = v_ref_fn(T) if v_ref_fn else 0.0
    ok = len(th) >= n - 1 and np.max(np.abs(xh[-1000:, 0])) < np.deg2rad(3) and np.max(np.abs(xh[-1000:, 4])) < np.deg2rad(3) \
        and np.max(np.abs(xh[-1000:, 3] - vend)) < 0.3
    return dict(ok=bool(ok), t=th, x=xh, u=uh)


def weight(plant):
    return (plant.M + plant.mp + plant.mw) * plant.g


def vstep(plant):
    return 0.8 if plant.R >= 0.05 else 0.3        # desktop-size robots (Skythinker, R = 2 cm) get 0.3 m/s


def sign_search(plant, ctrl, scale):
    """at a given output scale, return (push_score, signs) of the sign combination that balances a 2 deg leg tilt
    and survives the largest push in a cheap sweep, or None"""
    best = None
    ctrl.set_scale(scale)
    for signs in itertools.product((1, -1), repeat=4):
        r = simulate(plant, ctrl, signs, T=2.5, X0=[np.deg2rad(2), 0, 0, 0, 0, 0])
        if not r["ok"]:
            continue
        sc = 0
        for frac in (0.1, 0.2, 0.4, 0.6, 0.8, 1.0):           # push = fraction of the robot weight for 100 ms
            if simulate(plant, ctrl, signs, T=3.0, push=(0.5, 0.1, frac * weight(plant)))["ok"]:
                sc = frac
            else:
                break
        if best is None or sc > best[0]:
            best = (sc, signs)
    return best


def sign_scale_search(plant, ctrl, scales):
    """return (score, scale, signs) over all scales (largest push wins; ties -> scale closest to 1)"""
    best = None
    for s in scales:
        r = sign_search(plant, ctrl, s)
        if r is None:
            continue
        sc, signs = r
        if best is None or sc > best[0] or (sc == best[0] and abs(np.log(s)) < abs(np.log(best[1]))):
            best = (sc, s, signs)
    return best


def benchmark(plant, ctrl, signs, latency=0.002):
    res = {}
    tilts = np.arange(2.5, 41, 2.5)
    ok_t = [a for a in tilts if simulate(plant, ctrl, signs, X0=[np.deg2rad(a), 0, 0, 0, 0, 0], latency=latency, noise=True)["ok"]]
    res["max_leg_tilt_deg"] = float(max(ok_t)) if ok_t else 0.0
    ok_b = [a for a in tilts if simulate(plant, ctrl, signs, X0=[0, 0, 0, 0, np.deg2rad(a), 0], latency=latency, noise=True)["ok"]]
    res["max_body_tilt_deg"] = float(max(ok_b)) if ok_b else 0.0
    W = weight(plant); res["weight_N"] = float(W)
    ok_f = [f for f in np.arange(0.1, 2.01, 0.1) if simulate(plant, ctrl, signs, push=(0.5, 0.1, f * W), latency=latency, noise=True)["ok"]]
    res["max_push_frac_W_100ms"] = float(max(ok_f)) if ok_f else 0.0
    res["max_push_N_100ms"] = float(max(ok_f) * W) if ok_f else 0.0
    V = vstep(plant); res["vstep_mps"] = V
    r = simulate(plant, ctrl, signs, T=7.0, v_ref_fn=lambda t: V if 0.5 <= t < 3.0 else 0.0, latency=latency, noise=True)
    if r["ok"]:
        v = r["x"][:, 3]; t = r["t"]
        res["vstep_rise_s"] = float(t[np.argmax(v > 0.9 * V)] - 0.5) if v.max() > 0.9 * V else None
        res["vstep_overshoot_pct"] = float((v.max() - V) / V * 100)
        res["vstep_peak_body_deg"] = float(np.rad2deg(np.max(np.abs(r["x"][:, 4]))))
        res["vstep_peak_leg_deg"] = float(np.rad2deg(np.max(np.abs(r["x"][:, 0]))))
        res["vstep_ok"] = True
    else:
        res["vstep_ok"] = False
    lats = [0, 5, 10, 15, 20, 30, 40, 60, 80]
    ok_l = [L for L in lats if simulate(plant, ctrl, signs, X0=[np.deg2rad(3), 0, 0, 0, 0, 0], latency=L / 1e3)["ok"]]
    res["max_latency_ms"] = float(max(ok_l)) if ok_l else None
    L0 = plant.L0
    def L0_fn(t):
        if t < 1.0: return L0
        if t < 2.0: return L0 * (1 + 0.4 * (t - 1))
        if t < 3.0: return L0 * 1.4
        if t < 4.0: return L0 * (1.4 - 0.4 * (t - 3))
        return L0
    r = simulate(plant, ctrl, signs, T=5.5, L0_fn=L0_fn, latency=latency, noise=True)
    res["leg_change_ok"] = r["ok"]; res["leg_change_peak_body_deg"] = float(np.rad2deg(np.max(np.abs(r["x"][:, 4])))) if len(r["x"]) else None
    r = simulate(plant, ctrl, signs, T=3.0, latency=latency, noise=True)
    res["rest_T_rms_Nm"] = float(np.sqrt(np.mean(r["u"][-2000:, 0] ** 2))) if r["ok"] else None
    res["rest_Tp_rms_Nm"] = float(np.sqrt(np.mean(r["u"][-2000:, 1] ** 2))) if r["ok"] else None
    # fixed scenarios
    r = simulate(plant, ctrl, signs, T=5.0, X0=[np.deg2rad(10), 0, 0, 0, 0, 0], latency=latency, noise=True)
    if r["ok"]:
        idx = np.where((np.abs(r["x"][:, 0]) > np.deg2rad(1)) | (np.abs(r["x"][:, 4]) > np.deg2rad(1)))[0]
        res["tilt10_settle_s"] = float(r["t"][idx[-1]]) if len(idx) else 0.0
        res["tilt10_disp_m"] = float(abs(r["x"][-1, 2])); res["tilt10_peak_T"] = float(np.max(np.abs(r["u"][:, 0]))); res["tilt10_peak_Tp"] = float(np.max(np.abs(r["u"][:, 1])))
    r = simulate(plant, ctrl, signs, T=5.0, push=(0.5, 0.1, 0.4 * W), latency=latency, noise=True)   # 0.4 W (RM: 36 N)
    if r["ok"]:
        idx = np.where((np.abs(r["x"][:, 0]) > np.deg2rad(1)) | (np.abs(r["x"][:, 4]) > np.deg2rad(1)))[0]
        res["push40_settle_s"] = float(r["t"][idx[-1]] - 0.5) if len(idx) else 0.0
        res["push40_disp_m"] = float(abs(r["x"][-1, 2])); res["push40_peak_v"] = float(np.max(np.abs(r["x"][:, 3])))
    else:
        res["push40_settle_s"] = None
    return res


def run_one(args):
    """Two results per controller:
         asis : the law exactly as published (scale 1), only the sensor/motor sign conventions searched
         best : the output scale also searched (SCALES) -- how the law does once its gains are re-scaled to this robot"""
    plant_key, nm, do_scale = args
    plant = Plant6(**PLANTS[plant_key]); ctrl = REGISTRY[nm]()
    out = dict(name=ctrl.name, Ts=ctrl.Ts)
    r1 = sign_search(plant, ctrl, 1.0)
    if r1 is None:
        out["asis"] = dict(ok=False, note="does not balance as published for any sign convention on this plant")
    else:
        sc, signs = r1; ctrl.set_scale(1.0)
        res = benchmark(plant, ctrl, signs); res.update(ok=True, scale=1.0, signs=list(signs), search_push_N=sc)
        out["asis"] = res
    if do_scale and nm != "mine_lqi6":
        best = sign_scale_search(plant, ctrl, SCALES)
        if best is None:
            out["best"] = dict(ok=False, note="does not balance for any sign/scale combination on this plant")
        else:
            sc, s, signs = best
            if s == 1.0 and r1 is not None:
                out["best"] = dict(out["asis"])
            else:
                ctrl.set_scale(s)
                res = benchmark(plant, ctrl, signs); res.update(ok=True, scale=s, signs=list(signs), search_push_N=sc)
                out["best"] = res
    return nm, out


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--plant", default="rm"); ap.add_argument("--ctrl", nargs="*", default=None)
    ap.add_argument("--out", default=None); ap.add_argument("--scale", action="store_true", help="also search an output scale")
    ap.add_argument("--jobs", type=int, default=4)
    a = ap.parse_args()
    names = a.ctrl or list(REGISTRY.keys())
    with Pool(a.jobs) as pool:
        out = dict(pool.map(run_one, [(a.plant, nm, a.scale) for nm in names]))
    for nm, r in out.items():
        print(f"[{nm}] {r.get('name')}")
        for key in ("asis", "best"):
            if key in r:
                print(f"    {key}: {json.dumps(r[key])}")
    if a.out:
        json.dump(out, open(a.out, "w"), indent=1)
