"""
make_report.py -- turn results_small.json / results_large.json into markdown tables and plot time responses
for the report (제어기_비교보고서.md).  usage: python3 make_report.py
"""
import json, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from sil_harness import Controller, ROBOTS, simulate  # noqa: E402

LABEL = {
    "mine_lqi": "**내 제어기: LQI 1 kHz (온라인 리카티)**",
    "fern_lqr3": "A1 fern: 3상태 LQR 500 Hz (그들의 Q/R)",
    "fern_lqr3_ff2": "A1 fern + 중력 FF2 0.5",
    "fern_lqr3_retuned": "A1 fern 구조 + 내 Q/R (속도 PI 0.3/0.6)",
    "fern_lqr3_retuned_softpi": "A1 fern 구조 + 내 Q/R (속도 PI 0.15/0.3)",
    "lzu_lqr4": "A2 LZU_ASCENTO: 4상태 LQR 1 kHz",
    "zephyr_lqr4": "A3 zephyr_ascento: 4상태 LQR 1 kHz (코드 부호)",
    "zephyr_lqr4_vflip": "A3 zephyr_ascento (v·x 부호 반전)",
    "pow_lqr4": "A4 Pow_WBR: 4상태 LQR 125 Hz",
    "mushibo_lqr4": "A5 MuShibo: 4항 P + 소토크 PI ~1 kHz",
    "wl1_cascade": "A6 WL1: 캐스케이드 PID 100/20 Hz",
    "venus_cascade": "A7 Venus-1: 캐스케이드 PID 1 kHz",
    "sonic_pid": "A8 SonicRobot: PID→속도모드 100 Hz",
    "ryung_pid": "A9 Ryung: 3중 PID→속도모드, 실제 배치(100 Hz, 30 ms)",
    "ryung_pid_on_mcu": "A9 Ryung: 같은 코드, MCU 가정(100 Hz, 2 ms)",
}
LABEL_EN = {"mine_lqi": "mine: LQI 1 kHz", "fern_lqr3": "A1 fern LQR3 500 Hz", "lzu_lqr4": "A2 LZU LQR4 1 kHz",
            "pow_lqr4": "A4 Pow_WBR LQR4 125 Hz", "mushibo_lqr4": "A5 MuShibo 4P ~1 kHz", "wl1_cascade": "A6 WL1 cascade PID",
            "venus_cascade": "A7 Venus-1 cascade PID", "ryung_pid_on_mcu": "A9 Ryung PID (MCU)", "zephyr_lqr4_vflip": "A3 zephyr LQR4 (vflip)",
            "zephyr_lqr4": "A3 zephyr LQR4", "sonic_pid": "A8 SonicRobot PID", "ryung_pid": "A9 Ryung PID (PC)", "fern_lqr3_ff2": "A1 fern +FF2"}
ORDER = ["mine_lqi", "fern_lqr3", "fern_lqr3_ff2", "fern_lqr3_retuned", "fern_lqr3_retuned_softpi", "lzu_lqr4", "zephyr_lqr4", "zephyr_lqr4_vflip", "pow_lqr4",
         "mushibo_lqr4", "wl1_cascade", "venus_cascade", "sonic_pid", "ryung_pid", "ryung_pid_on_mcu"]


def f(v, fmt="{:.2f}", none="—"):
    if v is None:
        return none
    if isinstance(v, bool):
        return "O" if v else "X"
    try:
        return fmt.format(v)
    except Exception:
        return str(v)


def table(res, title):
    lines = [f"**{title}**", "",
             "| 제어기 | 최대 복귀 틸트 [°] | 최대 푸시 [N·0.1 s] | 0.8 m/s 스텝: 상승 s / 오버슈트 % / 최대 기울기 ° | 지연 허용 [ms] | 다리 1.4× 변화: 안정 / 최대 기울기 ° | 정지 토크 RMS [Nm] / 기울기 RMS [°] | 출력 스케일 |",
             "|---|---|---|---|---|---|---|---|"]
    for k in ORDER:
        r = res.get(k)
        if r is None:
            continue
        lab = LABEL.get(k, k)
        if "note" in r and "max_tilt_deg" not in r:
            lines.append(f"| {lab} | 균형 불가 ({r['note']}) | | | | | | |")
            continue
        vs = (f"{f(r.get('vstep_rise_s'))} / {f(r.get('vstep_overshoot_pct'), '{:.0f}')} / {f(r.get('vstep_peak_pitch_deg'), '{:.1f}')}"
              if r.get("vstep_ok") else "실패")
        lines.append(f"| {lab} | {f(r['max_tilt_deg'], '{:.1f}')} | {f(r['max_push_N_100ms'], '{:.0f}')} | {vs} | {f(r['max_latency_ms'], '{:.0f}')} | "
                     f"{f(r['leg_change_ok'])} / {f(r.get('leg_change_peak_pitch_deg'), '{:.1f}')} | {f(r.get('rest_torque_rms_Nm'), '{:.3f}')} / {f(r.get('rest_pitch_rms_deg'), '{:.3f}')} | {f(r.get('scale'), '{:.3g}')} |")
    return "\n".join(lines)


def plot_responses(robot, names, out_png):
    p = ROBOTS[robot]
    META = json.load(open(os.path.join(HERE, "controllers", "meta.json")))
    res = json.load(open(os.path.join(HERE, f"results_{robot}.json")))
    fig, axs = plt.subplots(2, 3, figsize=(15, 7))
    scen = [("tilt 10 deg", dict(x0=[0, 0, np.deg2rad(10), 0], T=3.0)),
            ("push 30 N x 0.1 s", dict(push=(0.5, 0.1, 30.0), T=4.0))]
    for row, (sname, skw) in enumerate(scen):
        for nm in names:
            r = res.get(nm, {})
            if "max_tilt_deg" not in r:
                continue
            m = META[nm]
            c = Controller(os.path.join(HERE, "controllers", m.get("src", nm) + ".c"), Ts=m.get("Ts", 1e-3))
            for k, v in m.get("params", {}).items():
                c.set_param(k, v)
            if r.get("scale"):
                c.set_scale(r["scale"])
            vl = m.get("vel_loop") if c.out_type == 1 else None
            out = simulate(p, c, latency=m.get("latency", 0.002), sign=r.get("sign", 1.0), vel_loop=vl, **skw)
            lab = LABEL_EN.get(nm, nm)
            axs[row, 0].plot(out["t"], np.rad2deg(out["x"][:, 2]), lw=1, label=lab)
            axs[row, 1].plot(out["t"], out["x"][:, 1], lw=1)
            axs[row, 2].plot(out["t"], out["u"] / 2, lw=0.8)
        axs[row, 0].set_ylabel(f"{sname}\npitch [deg]"); axs[row, 1].set_ylabel("speed [m/s]"); axs[row, 2].set_ylabel("torque/wheel [Nm]")
        axs[row, 0].set_ylim(-15, 15); axs[row, 1].set_ylim(-1.5, 1.5)
        for a in axs[row]:
            a.grid(alpha=0.3); a.set_xlabel("t [s]")
    axs[0, 0].legend(fontsize=6)
    fig.suptitle(f"SIL time responses, {p.name}", fontsize=10)
    fig.tight_layout(); fig.savefig(out_png, dpi=130); plt.close(fig)


if __name__ == "__main__":
    out = []
    for robot, title in (("small", "5 kg급 로봇 (r 65 mm, l 0.16 m, 5 Nm/휠)"), ("large", "10 kg급 로봇 (r 82.5 mm, l 0.25 m, 5 Nm/휠)")):
        path = os.path.join(HERE, f"results_{robot}.json")
        if os.path.exists(path):
            out.append(table(json.load(open(path)), title)); out.append("")
    md = "\n".join(out)
    open(os.path.join(HERE, "results_tables.md"), "w").write(md)
    print(md)
    figdir = os.path.join(HERE, "..", "figures")
    sel = ["mine_lqi", "fern_lqr3", "lzu_lqr4", "pow_lqr4", "mushibo_lqr4", "wl1_cascade", "venus_cascade", "ryung_pid_on_mcu"]
    for robot in ("small", "large"):
        if os.path.exists(os.path.join(HERE, f"results_{robot}.json")):
            plot_responses(robot, sel, os.path.join(figdir, f"sil_responses_{robot}.png"))
            print("plotted", robot)
