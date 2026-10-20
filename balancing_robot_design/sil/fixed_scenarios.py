"""fixed_scenarios.py -- same two scenarios for every controller that balances: 10 deg tilt and 40 N x 0.1 s push.
Reports settling time (|pitch|<1 deg), peak speed and run-off displacement, so that 'max push' can be interpreted."""
import json, os, sys, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
from sil_harness import Controller, ROBOTS, simulate, make_controller
from make_report import LABEL, ORDER
META = json.load(open(os.path.join(HERE, "controllers", "meta.json")))
out_md = []
for robot in ("small", "large"):
    res = json.load(open(os.path.join(HERE, f"results_{robot}.json")))
    rows = []
    for nm in ORDER + ["fern_lqr3_retuned_softpi"]:
        r = res.get(nm, {});
        if "max_tilt_deg" not in r: continue
        m = META[nm]; c = make_controller(nm, META)
        if r.get("scale"): c.set_scale(r["scale"])
        vl = m.get("vel_loop") if c.out_type == 1 else None
        p = ROBOTS[robot]; cells = [LABEL.get(nm, nm)]
        for skw in (dict(x0=[0, 0, np.deg2rad(10), 0], T=5.0), dict(push=(0.5, 0.1, 40.0), T=5.0)):
            o = simulate(p, c, latency=m.get("latency", 0.002), sign=r.get("sign", 1.0), vel_loop=vl, **skw)
            if not o["ok"]:
                cells.append("실패"); continue
            th = np.abs(o["x"][:, 2]); idx = np.where(th > np.deg2rad(1.0))[0]
            settle = o["t"][idx[-1]] - (0.5 if "push" in skw else 0.0) if len(idx) else 0.0
            disp = abs(o["x"][-1, 0]); vpk = np.max(np.abs(o["x"][:, 1])); tpk = np.max(np.abs(o["u"])) / 2
            cells.append(f"{settle:.2f} s / {vpk:.2f} m/s / {disp:.2f} m / {tpk:.2f} Nm")
        rows.append("| " + " | ".join(cells) + " |")
    out_md.append(f"**{'5 kg급' if robot=='small' else '10 kg급'} — 고정 시나리오** (정착시간(1°) / 최대 속도 / 최종 변위 / 최대 토크·휠)\n\n| 제어기 | 10° 기울임 복귀 | 40 N × 0.1 s 푸시 |\n|---|---|---|\n" + "\n".join(rows) + "\n")
md = "\n".join(out_md); open(os.path.join(HERE, "results_fixed.md"), "w").write(md); print(md)
