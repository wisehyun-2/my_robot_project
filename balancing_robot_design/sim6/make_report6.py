"""
make_report6.py -- results6_<plant>.json -> markdown tables (results6_tables.md) and time-response plots
(figures/sil6_responses_<plant>.png) for section 8 of 제어기_비교보고서.md.   usage: python3 make_report6.py
"""
import json, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
from plant6 import Plant6                                  # noqa: E402
from controllers6 import REGISTRY                          # noqa: E402
from sil6_harness import PLANTS, simulate, weight          # noqa: E402

LABEL = {
    "mine_lqi6": "**내 제어기: LQI6 1 kHz (온라인 리카티, 6상태+속도적분)**",
    "jerry": "B1 JerryGong0911 RM2024_Reborn(USTB): 6상태 LQR, K(L) 2차 다항식",
    "sky": "B2 Skythinker616: 6상태 dLQR 5 ms, K(L0) 3차 (0.6 kg 로봇용)",
    "hnust": "B3 HNUST-Dust wbr_control: 10상태/4출력 LQR, K(L) 3차",
    "qdu": "B4 QDU-Robomaster Wheelleg: 10상태/4출력 LQR, K(L_l,L_r) 2변수 2차",
    "multivac": "B5 Multivac11: 6상태 LQR(다리별), K(L0) 3차",
    "zhouzzw": "B6 Zhouzzw: 6상태 LQR(다리별), K(L) 3차, Q=diag(16000,…,70000,…)",
    "at": "B8 lyz1202dt ATWheelLeg: 6상태 LQR 500 Hz, 17점 게인 표",
    "ljw": "B9 LJW0401 (코드 그대로): 바퀴 T = 0.4×LQR 1행, 고관절 = CyberGear 위치 루프",
    "ljw_fullK": "B9' LJW0401 MATLAB K를 2×6 전체 상태 피드백으로 (가정; 펌웨어는 이렇게 안 씀)",
    "upkie_wb": "U1 upkie WheelBalancer(스파인 1 kHz, 속도 PI→바퀴 속도루프 kd 1.0), 다리 = 위치 서보 kp 20/kd 1",
    "upkie_wb_soft": "U1 upkie WheelBalancer, 무른 바퀴 속도 루프(kd 0.2)",
    "upkie_mpc": "U3 upkie MPCBalancer(qpmpc+ProxQP 200 Hz, 실제 Python 코드), 바퀴 속도루프 kd 1.0, 다리 = 위치 서보",
}
LABEL_EN = {"mine_lqi6": "mine: LQI6 1 kHz", "jerry": "B1 USTB Reborn LQR", "sky": "B2 Skythinker dLQR 5 ms", "hnust": "B3 HNUST LQR",
            "qdu": "B4 QDU LQR", "multivac": "B5 Multivac LQR", "zhouzzw": "B6 Zhouzzw LQR", "at": "B8 ATWheelLeg LQR 500 Hz",
            "ljw": "B9 LJW (as coded)", "ljw_fullK": "B9' LJW full K",
            "upkie_wb": "U1 upkie WheelBalancer", "upkie_wb_soft": "U1 upkie WB soft drive", "upkie_mpc": "U3 upkie MPC"}
ORDER = ["mine_lqi6", "jerry", "sky", "hnust", "qdu", "multivac", "zhouzzw", "at", "ljw", "ljw_fullK", "upkie_wb", "upkie_wb_soft", "upkie_mpc"]   # liu (B7) dropped from the report
PLANT_TITLE = {"rm": "RM급 5절링크 로봇 (USTB Reborn 파라미터: 몸체 7.0 kg, 다리 1.9 kg, 바퀴 0.47 kg, r 76 mm, L0 0.20 m, 휠 6 Nm, 고관절 30 Nm)",
               "liu": "15 kg급 Webots 모델 (몸체 12 kg, 다리 1.6 kg, 바퀴 6 kg, r 50 mm, L0 0.25 m, l = 0)",
               "sky": "Skythinker 탁상 로봇 (몸체 0.33 kg, 다리 0.025 kg, 바퀴 0.25 kg, r 20 mm, L0 0.07 m, 휠 0.2 Nm, 고관절 0.3 Nm)",
               "small": "5 kg급 5절링크 가정 (몸체 3.0 kg, 다리 0.6 kg, 바퀴 1.3 kg, r 65 mm, L0 0.16 m, 휠 5 Nm, 고관절 12 Nm)",
               "large": "10 kg급 5절링크 가정 (몸체 6.5 kg, 다리 1.2 kg, 바퀴 1.3 kg, r 82.5 mm, L0 0.25 m, 휠 5 Nm, 고관절 24 Nm)",
               "upkie": "upkie 추정 모델 (몸체 3.4 kg, 다리 1.2 kg, 바퀴 0.8 kg, r 60 mm, L0 0.40 m, l 0.12 m, 휠 1.7 Nm, 고관절 16 Nm; 질량 배분은 추정)"}


def f(v, fmt="{:.2f}", none="—"):
    if v is None:
        return none
    if isinstance(v, bool):
        return "O" if v else "X"
    try:
        return fmt.format(v)
    except Exception:
        return str(v)


def row(lab, r):
    if not r.get("ok"):
        return f"| {lab} | 균형 불가 | | | | | | | |"
    vs = (f"{f(r.get('vstep_rise_s'))} / {f(r.get('vstep_overshoot_pct'), '{:.0f}')} / {f(r.get('vstep_peak_body_deg'), '{:.1f}')} / {f(r.get('vstep_peak_leg_deg'), '{:.1f}')}"
          if r.get("vstep_ok") else "실패")
    sg = "".join("+" if s > 0 else "−" for s in r.get("signs", [1, 1, 1, 1]))
    return (f"| {lab} | {f(r['max_leg_tilt_deg'], '{:.1f}')} / {f(r['max_body_tilt_deg'], '{:.1f}')} | {f(r['max_push_N_100ms'], '{:.0f}')} ({f(r['max_push_frac_W_100ms'], '{:.1f}')} W) | {vs} | "
            f"{f(r['max_latency_ms'], '{:.0f}')} | {f(r['leg_change_ok'])} / {f(r.get('leg_change_peak_body_deg'), '{:.2f}')} | "
            f"{f(r.get('rest_T_rms_Nm'), '{:.3f}')} / {f(r.get('rest_Tp_rms_Nm'), '{:.3f}')} | {f(r.get('scale'), '{:.3g}')} | {sg} |")


def row_fixed(lab, r):
    if not r.get("ok"):
        return f"| {lab} | 균형 불가 | |"
    a = (f"{f(r.get('tilt10_settle_s'))} s / {f(r.get('tilt10_disp_m'))} m / {f(r.get('tilt10_peak_T'), '{:.1f}')} / {f(r.get('tilt10_peak_Tp'), '{:.1f}')} Nm"
         if r.get("tilt10_settle_s") is not None else "실패")
    b = (f"{f(r.get('push40_settle_s'))} s / {f(r.get('push40_peak_v'))} m/s / {f(r.get('push40_disp_m'))} m"
         if r.get("push40_settle_s") is not None else "실패")
    return f"| {lab} | {a} | {b} |"


def tables(plant_key, res):
    W = None
    for r in res.values():
        for k in ("asis", "best"):
            if r.get(k, {}).get("weight_N"):
                W = r[k]["weight_N"]
    V = 0.8 if PLANTS[plant_key]["R"] >= 0.05 else 0.3
    out = [f"**{PLANT_TITLE[plant_key]}** — 로봇 무게 W = {f(W, '{:.0f}')} N, 속도 스텝 {V} m/s", ""]
    for key, ttl in (("asis", "(a) 공개된 게인 그대로 (출력 스케일 1, 센서·모터 부호만 탐색)"), ("best", "(b) 출력 스케일까지 탐색 (이 로봇에 맞게 게인 크기 재조정)")):
        out += [f"*{ttl}*", "",
                "| 제어기 | 최대 복귀 틸트 다리/몸체 [°] | 최대 푸시 [N·0.1 s] | 속도 스텝: 상승 s / 오버슈트 % / 몸체 ° / 다리 ° | 지연 허용 [ms] | 다리 1.4× 변화: 안정 / 몸체 ° | 정지 토크 RMS 휠/고관절 [Nm] | 스케일 | 부호 θφ/T·Tp |",
                "|---|---|---|---|---|---|---|---|---|"]
        for k in ORDER:
            if k in res and key in res[k]:
                out.append(row(LABEL.get(k, k), res[k][key]))
        out.append("")
    out += ["*고정 시나리오* (정착시간(1°) / 최종 변위 / 최대 토크 휠·고관절 ; 0.4 W 푸시: 정착 / 최대 속도 / 변위) — (b) 설정", "",
            "| 제어기 | 10° 다리 기울임 복귀 | 0.4 W × 0.1 s 푸시 |", "|---|---|---|"]
    for k in ORDER:
        if k in res:
            r = res[k].get("best", res[k].get("asis", {}))
            out.append(row_fixed(LABEL.get(k, k), r))
    out.append("")
    return "\n".join(out)


def plot_responses(plant_key, names, out_png):
    plant = Plant6(**PLANTS[plant_key]); W = weight(plant)
    res = json.load(open(os.path.join(HERE, f"results6_{plant_key}.json")))
    fig, axs = plt.subplots(2, 4, figsize=(17, 7))
    scen = [("leg tilt 10 deg", dict(X0=[np.deg2rad(10), 0, 0, 0, 0, 0], T=3.0)), (f"push 0.4 W = {0.4*W:.0f} N x 0.1 s", dict(push=(0.5, 0.1, 0.4 * W), T=4.0))]
    for rowi, (sname, skw) in enumerate(scen):
        for nm in names:
            r = res.get(nm, {}).get("best", res.get(nm, {}).get("asis", {}))
            if not r.get("ok"):
                continue
            c = REGISTRY[nm](); c.set_scale(r.get("scale", 1.0))
            out = simulate(plant, c, tuple(r.get("signs", (1, 1, 1, 1))), latency=0.002, noise=True, **skw)
            lab = LABEL_EN.get(nm, nm)
            axs[rowi, 0].plot(out["t"], np.rad2deg(out["x"][:, 0]), lw=1, label=lab)
            axs[rowi, 1].plot(out["t"], np.rad2deg(out["x"][:, 4]), lw=1)
            axs[rowi, 2].plot(out["t"], out["x"][:, 3], lw=1)
            axs[rowi, 3].plot(out["t"], out["u"][:, 1], lw=0.8)
        axs[rowi, 0].set_ylabel(f"{sname}\nleg angle theta [deg]"); axs[rowi, 1].set_ylabel("body pitch phi [deg]")
        axs[rowi, 2].set_ylabel("speed [m/s]"); axs[rowi, 3].set_ylabel("hip torque Tp [Nm]")
        axs[rowi, 0].set_ylim(-15, 15); axs[rowi, 1].set_ylim(-10, 10)
        for a in axs[rowi]:
            a.grid(alpha=0.3); a.set_xlabel("t [s]")
    axs[0, 0].legend(fontsize=6)
    fig.suptitle(f"SIL time responses, six-motor plant: {plant.name}", fontsize=10)
    fig.tight_layout(); fig.savefig(out_png, dpi=130); plt.close(fig)


if __name__ == "__main__":
    out = []
    for pk in ("rm", "liu", "sky", "small", "large", "upkie"):
        path = os.path.join(HERE, f"results6_{pk}.json")
        if os.path.exists(path):
            out.append(tables(pk, json.load(open(path)))); out.append("")
    md = "\n".join(out)
    open(os.path.join(HERE, "results6_tables.md"), "w").write(md); print(md)
    figdir = os.path.join(HERE, "..", "figures")
    sel = ["mine_lqi6", "jerry", "hnust", "qdu", "multivac", "zhouzzw", "at", "ljw", "upkie_wb", "upkie_mpc"]
    for pk in ("rm", "small", "large"):
        if os.path.exists(os.path.join(HERE, f"results6_{pk}.json")):
            plot_responses(pk, sel, os.path.join(figdir, f"sil6_responses_{pk}.png")); print("plotted", pk)
