"""v5 hand calculations with the links printed in PETG FDM (user 11-01: compute with PETG; 11-02: the links will be
PETG): fork thigh and shank (closed boxes + Warren ribs, plan A spar), the 6061 hub inserts clamped to the links by a
6061 backing ring (11-02), the knee's outboard 61807 support and the printed motor plates, for the outboard wheel
(tyre centre y 270 since the 11-02 wheel shift).

    python calc/strength_v5.py        # -> calc/strength_v5.md

Loads, load cases and formulas are those of robot_cad_v4/calc/strength_v4.py (= v3 strength.py): contact 60 N static
per wheel, design 300 N, check 600 N (SF 2), lateral 0.5 x, motor torque 12.5 / 25 N m; stresses at the check load,
deflections at the design load.  Geometry: fusion/legs_v5.py and the v4 params (no Fusion needed: a stub stands in for
adsk, as in manufacturing/explain).  Part volumes: Fusion report() of 11-02 (hub WheelBiped_v5 version 6).
"""
import math
import os
import sys
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
V5 = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(os.path.dirname(V5), 'robot_cad_v4', 'fusion'))
sys.path.insert(0, os.path.join(V5, 'fusion'))
for _m in ('adsk', 'adsk.core', 'adsk.fusion'):
    sys.modules.setdefault(_m, mock.MagicMock())
import params as P            # noqa: E402
import legs_v5 as LV          # noqa: E402

# ----------------------------------------------------------------------------- materials
# PETG (FDM, the values v2 / v3 used): E 2000, tensile / flexural 45 within a layer, bearing 50; across the layers
# (build direction) taken as 0.6 x (assumption - FDM layer bonding, measure a test print); HDT about 70 C
E_PETG, NU_PETG = 2000.0, 0.35
S_PETG, C_PETG, S_PETG_Z, HDT_PETG = 45.0, 50.0, 0.6 * 45.0, 70.0
MAT = {   # E, G, strength (MPa), density (g/cm3)
    'PETG': (E_PETG, E_PETG / (2 * (1 + NU_PETG)), S_PETG, 1.27),
    'PA12': (1700.0, 1700.0 / (2 * 1.39), 48.0, 1.01),          # HP MJF PA12 sheet (v5 README)
    '6061-T6': (69000.0, 26000.0, 276.0, 2.70)}
E_SUS = 193000.0
AS_M3, AS_M25 = 5.03, 3.39
PROOF = {'8.8': 600.0, '12.9': 970.0}
P_AL = 260.0                  # 6061 limiting pressure under heads / washers (v3)
PULL_M3, PULL_M25 = 1000.0, 600.0   # heat-set insert pull-out in PETG, order of magnitude (v3: supplier tests); here
                              # pulled along the build direction (y), so expect less
BRG_61807 = (4.36, 3.35)      # C, C0 kN (catalogue value of a 61807; check the bought bearing)

F_STATIC = 60.0
F_DES, F_CHK = 5 * F_STATIC, 10 * F_STATIC
FL_DES, FL_CHK = P.LATERAL * F_DES, P.LATERAL * F_CHK
T_DES, T_CHK = P.MOTOR_PEAK * 1000.0, 2 * P.MOTOR_PEAK * 1000.0      # N mm
PHI = P.PHI
YTC = 2 * LV.Y_MSH - P.tyre_centre_y()                               # 270 outboard tyre centre plane
LINKS3 = (('thigh', (0.0, P.Z_HIP), (P.KNEE_X, P.KNEE_Z)), ('shank', (P.KNEE_X, P.KNEE_Z), (0.0, P.R_WHEEL)))
LEN = {n: math.hypot(b[0] - a[0], b[1] - a[1]) for n, a, b in LINKS3}
# Fusion report() 11-02 (hub version 6), one side: volume cm3 of the thigh / shank PETG body and the closed cells
# (outer shell - body: 1081.1 - 464.2, 532.8 - 256.6); 6061 backing ring 4.6 cm3 (12.3 g) per joint
VOL = {'thigh': 464.2, 'shank': 256.6}
CELLS = {'thigh': 1081.1 - 464.2, 'shank': 532.8 - 256.6}
ROBOT_PETG = 10.928           # robot_mass() 11-02 with the PETG legs (model material PETG print, solid)
PRELOAD_M3 = 1000.0           # flange bolts ⑥ into the 6061 ring: low tightening (about 0.7 N m) on the PETG wall
out = []


def h(t):
    out.append('\n## ' + t + '\n')


def table(head, rows):
    out.append('| ' + ' | '.join(head) + ' |')
    out.append('| ' + ' | '.join('---' for _ in head) + ' |')
    for r in rows:
        out.append('| ' + ' | '.join(str(c) for c in r) + ' |')


def ok(v, lim):
    return '통과' if v <= lim else '**초과**'


# ----------------------------------------------------------------------------- sections
def box(yin, yout, h_, t):
    """Closed box y yin .. yout, in-plane depth h_, all walls t (v4 strength_v4.box) + a Warren rib crossed by the
    section (rib / sin 45 across the cavity: area and lateral bending only)."""
    assert yout - yin > 2 * t + 1.0, (yin, yout)  # every v5 box has a cavity (tine 26, spar 25 wide at least)
    r = [(yin, yin + t, -h_ / 2, h_ / 2), (yout - t, yout, -h_ / 2, h_ / 2),
         (yin + t, yout - t, -h_ / 2, -h_ / 2 + t), (yin + t, yout - t, h_ / 2 - t, h_ / 2)]
    tw = LV.RIB * math.sqrt(2.0)
    r.append((yin + t, yout - t, -tw / 2, tw / 2))           # the rib as a web across the cavity (thickness tw)
    wy, wn = (yout - yin) - t, h_ - t
    Am, tt = wy * wn, t
    J = 4 * Am ** 2 / (2 * wn / t + 2 * wy / t)
    A = sum((b - a) * (d - c) for a, b, c, d in r)
    yc = sum((b - a) * (d - c) * (a + b) / 2 for a, b, c, d in r) / A
    Iy = sum((d - c) * (b - a) ** 3 / 12 + (b - a) * (d - c) * ((a + b) / 2 - yc) ** 2 for a, b, c, d in r)
    In = sum((b - a) * (d - c) ** 3 / 12 + (b - a) * (d - c) * ((c + d) / 2) ** 2 for a, b, c, d in r)
    return dict(A=A, yc=yc, Iy=Iy, In=In, J=J, Am=Am, t=tt, yin=yin, yout=yout, h=h_)


def combine(boxes, composite=True):
    """One section from one or two boxes side by side in y (the fork below the crotch).  composite: the two boxes
    bend together about their common centroid (the knee end joins them through the shank head and the two bearings);
    otherwise each bends about its own centroid (lower bound).  Torsion: the two cells add (no frame action)."""
    A = sum(b['A'] for b in boxes)
    yc = sum(b['A'] * b['yc'] for b in boxes) / A
    if composite:
        Iy = sum(b['Iy'] + b['A'] * (b['yc'] - yc) ** 2 for b in boxes)
        cy = max(max(yc - b['yin'], b['yout'] - yc) for b in boxes)
    else:
        Iy = sum(b['Iy'] for b in boxes)
        cy = max(max(b['yc'] - b['yin'], b['yout'] - b['yc']) for b in boxes)
    J = sum(b['J'] for b in boxes)
    tau = max(b['J'] / J / (2 * b['Am'] * b['t']) for b in boxes)
    return dict(A=A, yc=yc, Iy=Iy, In=sum(b['In'] for b in boxes), J=J, tau=tau, cy=cy,
                cn=max(b['h'] for b in boxes) / 2, n=len(boxes), face=min(b['h'] for b in boxes))


def thigh_sec(s, composite=True):
    """Thigh at s from the hip: one box Y_TF .. SPAR[1], or below the crotch the inner tine and the outer spar (the
    slot between them grown by the walls; its half-round end from CROTCH[0] to CROTCH_FACE)."""
    h_ = 2 * LV.th_half(s)
    y0, y1 = LV.Y_TF, LV.SPAR[1]
    s_c, r_c = LV.CROTCH
    if s <= s_c:
        return combine([box(y0, y1, h_, LV.WALL)], composite)
    yc = (LV.Y_TO + LV.SPAR[0]) / 2
    w = 2 * math.sqrt(max(r_c ** 2 - (LV.CROTCH_FACE - s) ** 2, 0.0)) if s < LV.CROTCH_FACE else LV.SPAR[0] - LV.Y_TO
    return combine([box(y0, yc - w / 2, h_, LV.WALL), box(yc + w / 2, y1, h_, LV.WALL)], composite)


def shank_sec(s, composite=True):
    """Shank at s from the knee: base layer Y_SI .. Y_SO, with the head pad to Y_SH where the r 70 pad covers the
    whole depth (beyond that the partial pad is left out)."""
    h_ = 2 * LV.sh_half(s)
    full_pad = s < LV.PAD_R and math.sqrt(LV.PAD_R ** 2 - s ** 2) >= h_ / 2
    return combine([box(LV.Y_SI, LV.Y_SH if full_pad else LV.Y_SO, h_, LV.WALL)], composite)


def contact_moment(S, y_s, F, Fl, sgn, ytc=YTC):
    """Moment of the contact forces (vertical F, lateral sgn Fl at the tyre centre plane on the floor) about the
    point (S[0], y_s, S[1]): (Mx, My, Mz) in N mm (v4)."""
    dx, dy, dz = -S[0], ytc - y_s, -S[1]
    fy, fz = sgn * Fl, F
    return dy * fz - dz * fy, -dx * fz, dx * fy


def joint_moment(S, y_s, F, Fl, ytc=YTC):
    """Out-of-plane moment at a joint (the leg-plane part (Mx, Mz)), worse lateral sign."""
    return max(math.hypot(*[m for i, m in enumerate(contact_moment(S, y_s, F, Fl, sg, ytc)) if i != 1])
               for sg in (1, -1))


def legs3(mat, composite=True, n=200):
    """v4 legs3 on the v5 sections: max von Mises at the check load, v2-metric link-end lateral deflection, in-plane
    deflection, lateral contact displacement (design load, bending + twist) and its twist share, face plate
    compression."""
    E, G, _S, _rho = MAT[mat]
    secs = {'thigh': lambda s: thigh_sec(s, composite), 'shank': lambda s: shank_sec(s, composite)}
    res, delta, twist = {}, {'+': 0.0, '-': 0.0}, {'+': 0.0, '-': 0.0}
    sig_face, face_b = 0.0, None
    for name, a, b in LINKS3:
        L = LEN[name]
        u = ((b[0] - a[0]) / L, (b[1] - a[1]) / L)
        nv = (-u[1], u[0])
        d_v2 = d_in = vm_max = 0.0
        where = None
        for i in range(n + 1):
            s = L * i / n
            S = (a[0] + u[0] * s, a[1] + u[1] * s)
            sec = secs[name](s)
            if i < n:
                sm = s + L / n / 2
                Sm = (a[0] + u[0] * sm, a[1] + u[1] * sm)
                sc = secs[name](sm)
                for sg, k in ((1, '+'), (-1, '-')):
                    Mx, _My, Mz = contact_moment(Sm, sc['yc'], F_DES, FL_DES, sg)
                    mx_, _my, mz_ = contact_moment(Sm, sc['yc'], 0.0, 1.0, sg)
                    T, Mb = Mx * u[0] + Mz * u[1], Mx * nv[0] + Mz * nv[1]
                    t1, mb1 = mx_ * u[0] + mz_ * u[1], mx_ * nv[0] + mz_ * nv[1]
                    db, dt = Mb * mb1 / (E * sc['Iy']) * L / n, T * t1 / (G * sc['J']) * L / n
                    delta[k] += db + dt
                    twist[k] += dt
                Mb2 = (FL_DES * Sm[1] + F_DES * abs(sc['yc'] - YTC)) * math.cos(PHI)
                d_v2 += Mb2 * (L - sm) * L / n / (E * sc['Iy'])
                d_in += T_DES * (L - sm) * L / n / (E * sc['In'])
            for sg in (1, -1):
                Mx, _My, Mz = contact_moment(S, sec['yc'], F_CHK, FL_CHK, sg)
                T, Mb = Mx * u[0] + Mz * u[1], Mx * nv[0] + Mz * nv[1]
                s_out, s_in, s_ax = abs(Mb) * sec['cy'] / sec['Iy'], T_CHK * sec['cn'] / sec['In'], F_CHK / sec['A']
                tau = abs(T) * sec['tau']
                vm = math.sqrt((s_out + s_in + s_ax) ** 2 + 3 * tau ** 2)
                if vm > vm_max:
                    vm_max, where = vm, s
                if s_out + s_ax > sig_face:
                    sig_face = s_out + s_ax
                    face_b = sec['face']
        res[name] = dict(vm=vm_max, at=where, d_v2=d_v2, d_in=d_in)
    k = '+' if abs(delta['+']) >= abs(delta['-']) else '-'
    return res, abs(delta[k]), abs(twist[k]), sig_face, face_b


def plate_buckling(t, b, E, nu, k=8.0):
    """Compressive buckling stress of a face plate strip of width b between the edge walls (k = 8: the ribs cut it
    into triangles, v4)."""
    return k * math.pi ** 2 * E / (12 * (1 - nu ** 2)) * (t / b) ** 2


def annular(b, a, t, M, E=E_PETG, nu=NU_PETG, outer='clamped'):
    """Max radial bending stress of an annular plate (inner edge r = b fixed to a rigid boss tilted by the moment M,
    outer edge r = a clamped or simply supported), n = 1 Fourier mode (v4 strength_v4.annular)."""
    f = [lambda r: r, lambda r: r ** 3, lambda r: 1 / r, lambda r: r * math.log(r)]
    d1 = [lambda r: 1.0, lambda r: 3 * r ** 2, lambda r: -1 / r ** 2, lambda r: math.log(r) + 1]
    d2 = [lambda r: 0.0, lambda r: 6 * r, lambda r: 2 / r ** 3, lambda r: 1 / r]
    D = E * t ** 3 / (12 * (1 - nu ** 2))
    mr_row = lambda r: [-D * (d2[j](r) + nu * (d1[j](r) / r - f[j](r) / r ** 2)) for j in range(4)]
    rows = [[g(b) for g in f], [g(b) for g in d1], [g(a) for g in f],
            [g(a) for g in d1] if outer == 'clamped' else mr_row(a)]
    C = _solve4(rows, [b, 1.0, 0.0, 0.0])
    w = lambda r: sum(c * g(r) for c, g in zip(C, f))
    w1 = lambda r: sum(c * g(r) for c, g in zip(C, d1))
    w2 = lambda r: sum(c * g(r) for c, g in zip(C, d2))
    Mr = lambda r: -D * (w2(r) + nu * (w1(r) / r - w(r) / r ** 2))
    Mrt = lambda r: -D * (1 - nu) * (w1(r) / r - w(r) / r ** 2)
    lap = lambda r: w2(r) + w1(r) / r - w(r) / r ** 2
    V = lambda r, hh=1e-6: -D * (lap(r + hh) - lap(r - hh)) / (2 * hh) - Mrt(r) / r
    scale = M / (math.pi * b * (Mr(b) - b * V(b)))
    return max(abs(6 * Mr(b + (a - b) * i / 100) * scale / t ** 2) for i in range(101))


def _solve4(A, y):
    n = len(y)
    M = [list(map(float, A[i])) + [float(y[i])] for i in range(n)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(M[r][c]))
        M[c], M[p] = M[p], M[c]
        for r in range(c + 1, n):
            k = M[r][c] / M[c][c]
            M[r] = [M[r][j] - k * M[c][j] for j in range(n + 1)]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (M[r][n] - sum(M[r][j] * x[j] for j in range(r + 1, n))) / M[r][r]
    return x


def bolt_circle_max(M, n, r):
    return 2 * M / (n * r)


def ring_peak(F, width, t, base, E=E_PETG):
    """Peak pressure under a SUS clamp ring next to one screw: ring = beam on an elastic printed foundation (v4)."""
    beta = (E / base / (4 * E_SUS * t ** 3 / 12)) ** 0.25
    return F * beta / (2 * width), math.pi / beta


def main():
    out.append('# v5 강도 손계산 — 링크를 PETG로 (`python calc/strength_v5.py`)\n')
    out.append('사용자 결정: 11-01 허벅지·정강이를 MJF PA12 대신 **PETG(FDM)** 로 계산, 11-02 **PETG로 출력하기로 함**.  형상은 '
               'v5 (`fusion/legs_v5.py`, 허브 버전 6): 바깥 막대 y %.1f ~ %.1f, 갈래 틈 %.1f, 허벅지 상자 폭 %.1f, 정강이 기본층 '
               'y %.0f ~ %.0f(무릎 판과 틈 %.1f), 바퀴는 바깥쪽(타이어 중심면 y %.1f, 11-02 바퀴 묶음 2.5 mm 바깥으로), hub 인서트는 '
               '6061 받침 링으로 PETG 벽을 끼워 고정(11-02).  하중·식은 v4(`robot_cad_v4/calc/strength_v4.py`)와 같다: 정지 %.0f N/바퀴, '
               '설계 %.0f N, 검토 %.0f N(안전계수 2), 옆 %.0f / %.0f N, 관절 토크 설계 12.5 / 검토 25 N·m.  응력은 검토 하중, 처짐은 '
               '설계 하중.\n'
               % (LV.SPAR[0], LV.SPAR[1], LV.SPAR[0] - LV.Y_TO, LV.SPAR[1] - LV.Y_TF, LV.Y_SI, LV.Y_SO, LV.KNEE_GAP, YTC,
                  F_STATIC, F_DES, F_CHK, FL_DES, FL_CHK))
    out.append('재료: PETG — 탄성 %.0f MPa, 층 안 인장·굽힘 %.0f MPa, 지압 %.0f MPa(v2·v3 값), **층 사이(출력 높이 방향)는 0.6배인 '
               '%.0f MPa로 가정**(시험 출력으로 확인), 밀도 %.2f, HDT 약 %.0f °C.  링크는 안쪽 면을 바닥에 놓고(y가 출력 높이) '
               '출력한다고 본다: 링크 굽힘·비틀림의 면판 응력은 층 안, 인서트를 뽑는 힘은 층 사이로 간다.  비교로 MJF PA12(탄성 1700, '
               '인장 48)도 같은 단면으로 계산했다.\n'
               % (E_PETG, S_PETG, C_PETG, S_PETG_Z, MAT['PETG'][3], HDT_PETG))

    # ------------------------------------------------------------------ 1. link sections
    h('1. 링크 — 갈래 허벅지와 정강이 상자')
    out.append('단면은 링크 축을 따라 200곳에서 잡았다.  허벅지는 크로치(s %.1f)까지 상자 하나(y %.0f ~ %.0f, 벽 %.1f), 그 아래는 '
               '안쪽 갈래와 바깥 막대 두 상자다.  두 상자가 **함께 굽는다고 본 값(무릎 끝이 정강이 머리와 두 베어링으로 이어짐)**과 '
               '**따로 굽는다고 본 값(아래 한계)** 을 둘 다 냈다.  비틀림은 두 칸을 더하기만 했다(틀 작용 무시, 보수적).  정강이는 '
               'r 70 패드가 깊이 전체를 덮는 곳까지(무릎에서 약 %.0f mm)만 패드를 넣었다.  리브는 단면마다 하나가 대각선으로 걸린다고 '
               '보고 옆 굽힘에만 넣었다(v4와 같음).  "링크 끝 옆 처짐"은 v2부터 쓴 지표(옆 굽힘만, 한계 1 mm), "접지점 옆 변위"는 '
               '굽힘 + 비틀림 전체다.\n'
               % (LV.CROTCH[0], LV.Y_TF, LV.SPAR[1], LV.WALL,
                  max(s for s in range(0, 71) if math.sqrt(LV.PAD_R ** 2 - s ** 2) >= LV.sh_half(s))))
    rows, chosen = [], {}
    for mat, comp, nm in (('PETG', True, '**PETG, 갈래 함께 굽음**'), ('PETG', False, 'PETG, 갈래 따로 굽음 (아래 한계)'),
                          ('PA12', True, '참고: MJF PA12, 갈래 함께 굽음')):
        res, d, tw, sf, fb = legs3(mat, comp)
        E, G, lim, _r = MAT[mat]
        vm = max(res['thigh']['vm'], res['shank']['vm'])
        sb = plate_buckling(LV.WALL, fb, E, 0.35)
        fine = max(res['thigh']['d_v2'], res['shank']['d_v2']) <= 1.0 and vm <= lim and sf <= sb
        worst = 'thigh' if res['thigh']['vm'] >= res['shank']['vm'] else 'shank'
        rows.append((nm, '%.2f / %.2f' % (res['thigh']['d_v2'], res['shank']['d_v2']), '%.1f (%.1f)' % (d, tw),
                     '%.0f (%s s %.0f) / %.0f' % (vm, {'thigh': '허벅지', 'shank': '정강이'}[worst], res[worst]['at'], lim),
                     '%.0f / %.0f (폭 %.0f)' % (sf, sb, fb), '통과' if fine else '**초과**'))
        chosen[(mat, comp)] = (res, d, tw, sf, sb, vm)
    table(('단면', '링크 끝 옆 처짐 허벅지 / 정강이 [mm] (한계 1)', '접지점 옆 변위 [mm] (비틀림)', '검토 응력 / 강도 [MPa]',
           '면판 압축 / 좌굴 [MPa]', '판정'), rows)
    res, d, tw, sf, sb, vm = chosen[('PETG', True)]
    res2, d2, tw2, _sf2, _sb2, vm2 = chosen[('PETG', False)]
    out.append('\n- PETG(탄성 2000)는 MJF PA12(1700)보다 조금 단단해서 같은 형상이면 처짐이 약 15 % 작다.  강도는 비슷하다(45 / 48).')
    out.append('- **정강이 옆 처짐 %.2f mm가 v2부터 쓴 링크 하나 1 mm 기준을 넘는다**: 정강이는 무릎 패드(r 70) 밖에서 폭이 '
               '%.0f mm뿐이다(v4 정강이는 바깥으로 %.0f까지 넓혔지만, v5는 그쪽에 타이어가 있다).  대신 허벅지가 폭 %.0f 상자라 '
               '매우 단단해서(%.2f mm) 접지점 전체 옆 변위는 %.1f mm로 v4 · v2보다 작다.  링크 하나 기준을 지킬지, 접지점 변위로 '
               '볼지는 결정할 것 — 지키려면 정강이 보를 키워야 한다(자세 스윕 다시 필요).'
               % (res['shank']['d_v2'], LV.Y_SO - LV.Y_SI, P.TIBIA_WIDE_OUT - P.Y_TIBIA_IN, LV.SPAR[1] - LV.Y_TF,
                  res['thigh']['d_v2'], d))
    out.append('- 갈래를 따로 굽는다고 보면 접지점 옆 변위 %.1f → %.1f mm, 검토 응력 %.0f → %.0f MPa — 실제는 그 사이다.  크로치 '
               '아래 두 상자를 잇는 것은 무릎 끝(모터 + 61807)뿐이라 함께 굽음은 위쪽 값이다.' % (d, d2, vm, vm2))
    out.append('- 비교: v4 SLA 레진 상자(바퀴 안쪽) 링크 끝 처짐 0.87 / 0.94 mm, 접지점 옆 변위 10.5 mm, v2 PETG 0.95 / 1.00 · 12.8 mm.')
    m_petg = {k: v * MAT['PETG'][3] for k, v in VOL.items()}
    m_pa12 = {k: v * MAT['PA12'][3] for k, v in VOL.items()}
    dm = 2 * sum(m_petg[k] - m_pa12[k] for k in VOL)
    out.append('- 질량(Fusion 부피, 한쪽): 허벅지 %.1f cm³ → PETG **%.0f g** (PA12였다면 %.0f), 정강이 %.1f cm³ → **%.0f g** (%.0f), '
               '6061 받침 링 12 g × 2.  벽 3.5 · 리브 2.0은 노즐 0.4로 둘레선만으로 채워지므로 꽉 찬 밀도로 셌다.  로봇(PETG 다리 모델) '
               '**%.2f kg** (PA12였다면 약 %.2f kg).  닫힌 칸은 허벅지 %.0f cm³ · 정강이 %.0f cm³ — FDM이라 빈칸 그대로다(11-02 배출 '
               '구멍 없앰).'
               % (VOL['thigh'], m_petg['thigh'], m_pa12['thigh'], VOL['shank'], m_petg['shank'], m_pa12['shank'],
                  ROBOT_PETG, ROBOT_PETG - dm / 1000, CELLS['thigh'], CELLS['shank']))

    # ------------------------------------------------------------------ 2. joints
    h('2. 관절부')
    S_H, S_K, S_W = (0.0, P.Z_HIP), (P.KNEE_X, P.KNEE_Z), (0.0, P.R_WHEEL)
    y_out = {j: P.y_output(j) for j in ('hip', 'knee', 'wheel')}
    W_boss = math.pi * P.HUB_BOSS_D ** 3 / 32 - 6 * (math.pi / 4 * 3.4 ** 2) * 10 ** 2 / (P.HUB_BOSS_D / 2) / 2
    WA = math.pi / 4 * (7.0 ** 2 - 3.2 ** 2)                    # DIN 125 M3 washer
    a_head = math.pi / 4 * (4.5 ** 2 - 2.8 ** 2)                # M2.5 head on the hip plate
    I_fl = math.pi / 4 * (LV.INSERT_FL[0] ** 4 - 14.0 ** 4)     # flange annulus r 14 .. 24 on the link pocket floor
    rows = []
    M = {}
    for joint, S in (('hip', S_H), ('knee', S_K), ('wheel', S_W)):
        for tag, F, Fl in (('검토', F_CHK, FL_CHK), ('설계', F_DES, FL_DES)):
            M[(joint, tag)] = joint_moment(S, y_out[joint], F, Fl)
    M4 = {(j, t): joint_moment(S, y_out[j], F, Fl, ytc=P.tyre_centre_y())        # v4: the inboard tyre
          for j, S in (('hip', S_H), ('knee', S_K)) for t, F, Fl in (('검토', F_CHK, FL_CHK),)}
    for joint in ('hip', 'knee'):
        for tag in ('검토', '설계'):
            m = M[(joint, tag)]
            fb = bolt_circle_max(m, 6, P.AKE_PCD_OUT / 2)
            sb = m / W_boss
            rows.append(('%s 출력면 옆 모멘트 %s %.0f N·m%s' % (joint, tag, m / 1000, ' (모터가 다 받을 때)' if joint == 'knee' else ''),
                         '② M3 12.9 인장 %.0f N (보증 %.0f), 와셔 면압 %.0f MPa (6061 %.0f), 6061 Ø28 보스 굽힘 %.0f MPa (276)'
                         % (fb, AS_M3 * PROOF['12.9'], fb / WA, P_AL, sb),
                         ok(max(fb / (AS_M3 * PROOF['12.9']), fb / WA / P_AL, sb / 276.0), 1.0)))
            fi = bolt_circle_max(m, 6, LV.FLANGE_BOLT_R)
            fb6 = PRELOAD_M3 + fi                         # conservative: the whole external load on top of the preload
            ri, ro = LV.RING[0], LV.RING[1]
            A_r = math.pi * (ro ** 2 - ri ** 2) - 6 * math.pi / 4 * LV.LG.M3_CLEAR ** 2
            pw = m * ro / (math.pi / 4 * (ro ** 4 - ri ** 4)) + 6 * PRELOAD_M3 / A_r
            rows.append(('%s ⑥ 플랜지 → 받침 링 (PETG 벽 %.0f mm를 끼움), %s' % (joint, LV.WALL_J, tag),
                         'M3 × 16 12.9 6개 (r %.0f, 6061 링 탭 6 mm 물림) 하나에 외력 %.0f N + 예압 %.0f = %.0f N (보증 %.0f); '
                         '받침 링 밑 PETG 벽 압축 %.0f MPa (%.0f, 층 사이로 눌림)'
                         % (LV.FLANGE_BOLT_R, fi, PRELOAD_M3, fb6, AS_M3 * PROOF['12.9'], pw, C_PETG),
                         ok(max(fb6 / (AS_M3 * PROOF['12.9']), pw / C_PETG), 1.0)))
    for tag in ('검토', '설계'):
        m = M[('hip', tag)]
        ft = bolt_circle_max(m, 6, P.AKE_PCD_RING / 2)
        web = 289.0 * m / 133000.0
        rows.append(('hip 고정링 ① M2.5 12.9 · 6061 hip 판, %s' % tag,
                     '인장 %.0f N (보증 %.0f), 머리 면압 %.0f MPa (%.0f); hip 판 웹 5 mm (v3 식 비례: 한 단면이 다 받으면) %.0f MPa (276)'
                     % (ft, AS_M25 * PROOF['12.9'], ft / a_head, P_AL, web),
                     ok(max(ft / (AS_M25 * PROOF['12.9']), ft / a_head / P_AL, web / 276.0), 1.0)))
    for joint in ('knee', 'wheel'):
        for tag in ('검토', '설계'):
            m = M[(joint, tag)]
            b, a, t = P.STATOR_RECESS_D / 2, P.CAN_CLEAR_R, P.KNEE_PLATE_T
            sc, ss, sg_ = annular(b, a, t, m), annular(b, a, t, m, outer='simple'), annular(b, a, t - 2.0, m, outer='simple')
            ft = bolt_circle_max(m, 6, P.AKE_PCD_RING / 2)
            pk, spread = ring_peak(ft, (P.RING_STATOR[0] - P.RING_STATOR[1]) / 2, P.RING_T, t - P.RING_T - 1.7)
            rows.append(('%s PETG 모터 판 %.0f mm + 고정링 ① M2.5 8.8, %s: %.0f N·m%s'
                         % (joint, t, tag, m / 1000, ' (모터가 다 받을 때)' if joint == 'knee' else ''),
                         '판 굽힘 바깥 고정 %.0f / 단순 지지 %.0f (리드 홈 자리 %.0f mm: %.0f) MPa; 나사 인장 %.0f N (보증 %.0f), '
                         'SUS 링 밑 PETG 면압 %.0f MPa' % (sc, ss, t - 2.0, sg_, ft, AS_M25 * PROOF['8.8'], pk),
                         ok(max(sg_ / S_PETG, ft / (AS_M25 * PROOF['8.8']), pk / C_PETG), 1.0)))
    f = T_CHK / (6 * LV.FLANGE_BOLT_R)
    rows.append(('⑥ 토크 25 N·m (검토)', '예압 마찰(μ 0.2)로 %.0f N·m; 마찰을 빼도 볼트 하나에 옆 힘 %.0f N → PETG 벽 구멍 지압 '
                 '%.1f MPa (Ø3.4 × %.0f)' % (0.2 * 6 * PRELOAD_M3 * LV.FLANGE_BOLT_R / 1000, f, f / (3.4 * LV.WALL_J),
                                            LV.WALL_J), ok(f / (3.4 * LV.WALL_J), C_PETG)))
    f2 = T_CHK / (6 * P.AKE_PCD_RING / 2)
    web = P.KNEE_PLATE_T - P.POCKET - P.RING_T - 1.7
    rows.append(('고정링 → PETG 모터 판, 토크 25 N·m (검토)', 'M2.5 6 (r 18.25): 개당 %.0f N → 구멍 지압 %.1f MPa (웹 %.1f mm)'
                 % (f2, f2 / (2.5 * web), web), ok(f2 / (2.5 * web), C_PETG)))
    table(('부위', '응력·힘', '판정'), rows)

    # 61807: the knee moment as a couple of the motor output bearing and the 61807
    y_mb = y_out['knee'] - 4.0                    # AKE60 output bearing, assumed 4 mm inside the output face
    y_b = LV.Y_SH - LV.BRG[2] / 2
    dd = y_b - y_mb
    rows = []
    for tag, F, Fl in (('검토', F_CHK, FL_CHK), ('설계', F_DES, FL_DES)):
        m = joint_moment(S_K, y_mb, F, Fl)
        R = m / dd
        p_seat = R / (LV.BRG[1] * LV.BRG[2])
        lever = LV.SPAR[0] - y_b
        W_tube = math.pi * ((2 * LV.CAP_R[1]) ** 4 - (2 * LV.CAP_R[0]) ** 4) / (32 * 2 * LV.CAP_R[1])
        m_cap = R * (LV.SPAR[1] - LV.CAP_T - y_b)
        f_cap = bolt_circle_max(m_cap, 3, LV.CAP_BOLT_R)
        p_hole = 6 * m_cap / (2 * LV.CAP_R[1] * (LV.SPAR[1] - LV.SPAR[0]) ** 2)
        rows.append(('무릎 %s: 모멘트 %.0f N·m를 모터 베어링과 61807이 짝힘으로 (간격 %.0f mm)' % (tag, m / 1000, dd),
                     '61807 반경 하중 %.2f kN (C0 %.2f kN), 정강이 자리 PETG 면압 %.1f MPa; 캡 관 굽힘 %.0f MPa (6061, 팔 %.1f mm); '
                     '캡 관이 막대 구멍에 꼭 맞음(Ø35 H7/g6, 11-02) → 구멍 면압 %.0f MPa (%.0f), 캡 M2.5 인서트 3개는 예압만 '
                     '(관이 놀면 인발 %.0f N, 어림 한계 %.0f)'
                     % (R / 1000, BRG_61807[1], p_seat, R * lever / W_tube, lever, p_hole, C_PETG, f_cap, PULL_M25),
                     '61807·자리·관 %s, 캡 구멍 %s' % (ok(max(R / 1000 / BRG_61807[1], p_seat / C_PETG), 1.0),
                                                   ok(p_hole, C_PETG) if LV.CAP_FIT == 0 else ok(f_cap, PULL_M25))))
    out.append('\n**무릎 바깥 지지 (61807)** — 모터 출력 베어링이 모멘트를 못 받는다고(핀) 보면 무릎 옆 모멘트는 두 베어링의 짝힘이 '
               '된다.  AKE60 출력 베어링 자리는 비공개라 출력면 안쪽 4 mm로 잡았다.\n')
    table(('경우', '힘·응력', '판정'), rows)

    # ------------------------------------------------------------------ 3. verdict
    mh, mk = M[('hip', '검토')], M[('knee', '검토')]
    h('3. 정리')
    out.append('- **링크 몸통(PETG 상자) 강도는 통과**: 검토 응력 %.0f MPa < %.0f, 면판 좌굴 여유 %.1f배.  처짐은 허벅지 %.2f mm로 '
               '아주 단단하고 **정강이 %.2f mm는 v2의 링크 하나 1 mm 기준을 넘는다**(1절); 접지점 전체 옆 변위는 %.1f mm(갈래 따로 '
               '굽으면 %.1f)로 v4(10.5) · v2(12.8)보다 작다.  정강이는 바깥에 타이어, 안쪽에 허벅지가 있어 폭 30을 못 넘고, 면 벽을 '
               '7 mm로 두껍게 해도 1.30 mm(+106 g/개)라 1 mm는 이 폭으로 못 맞춘다 — **기준을 접지점 변위로 볼지 결정할 것**.'
               % (vm, S_PETG, sb / sf, res['thigh']['d_v2'], res['shank']['d_v2'], d, d2))
    out.append('- **⑥ hub 인서트 고정은 통과 (11-02 다시 설계)**: 열압입 인서트 대신 M3 × 16 12.9 6개가 플랜지 → PETG 벽 %.0f mm → '
               '6061 받침 링(Ø%.0f / Ø%.0f × %.0f, 링크 바깥 면 쪽 Ø%.0f 구멍으로 넣음)의 탭에 물린다.  PETG는 눌리기만 한다.  hip 검토 '
               '하중에서 볼트 하나 %.1f kN(예압 1 kN 포함, 보증 4.9 kN), 벽 압축 %.0f MPa (50).  예전 인서트 방식은 인서트 하나 '
               '3.1 kN 인발(한계 약 1 kN)이었다.  조임은 약 0.7 N·m + 나사 고정제, 처음 몇 주 다시 조인다(PETG 크리프).'
               % (LV.WALL_J, 2 * LV.RING[1], 2 * LV.RING[0], LV.RING[2], 2 * LV.BORE_R,
                  (PRELOAD_M3 + bolt_circle_max(mh, 6, LV.FLANGE_BOLT_R)) / 1000,
                  mh * LV.RING[1] / (math.pi / 4 * (LV.RING[1] ** 4 - LV.RING[0] ** 4))
                  + 6 * PRELOAD_M3 / (math.pi * (LV.RING[1] ** 2 - LV.RING[0] ** 2) - 6 * math.pi / 4 * LV.LG.M3_CLEAR ** 2)))
    out.append('- **hip 출력 ② M3 12.9는 검토 하중에서 보증을 넘는다**(%.0f N > %.0f, 바퀴를 바깥으로 돌려 hip 옆 모멘트가 %.0f N·m가 '
               '됨, v4 %.0f).  ① M2.5 12.9도 경계, hip 판 웹은 한 단면이 다 받는다고 보면 %.0f MPa로 6061 항복을 넘는다.  링크 재료와 '
               '무관한 바깥 바퀴 + AKE60 출력부(M3 6개, PCD 20)의 한계라 다리 설계로는 못 고친다(설계 하중에서는 통과).'
               % (bolt_circle_max(mh, 6, 10), AS_M3 * PROOF['12.9'], mh / 1000, M4[('hip', '검토')] / 1000, 289.0 * mh / 133000))
    out.append('- 바퀴 PETG 모터 판·고정링은 통과.  **무릎은 모터가 옆 모멘트를 다 받는다고 보면 검토 하중에서 PETG 판(리드 홈 '
               '자리 7 mm)과 고정링 M2.5 8.8이 넘는다**.  실제로는 61807이 짝힘으로 나눠 받아 통과 쪽이며, 11-02 캡 관을 막대 구멍에 '
               '꼭 맞게 해 61807 쪽 길이 단단해졌다(실제 몫은 두 베어링 강성에 달림).  61807과 정강이 자리·캡 관·캡 구멍은 통과.')

    # ------------------------------------------------------------------ 4. heat, print
    h('4. 열 · 출력 메모 (PETG FDM)')
    out.append('- PETG는 약 70 °C에서 물러진다.  무릎·바퀴 모터가 PETG 판에 붙는다(v2와 같음).  v2 계산: 짐 5 kg에 L0 0.240 m면 무릎 '
               '정지 토크가 정격의 144 % → 모터 외피가 PETG 연화 온도에 가까워질 수 있음.  실물에서 모터 링 면 온도를 재서 50 °C 밑인지 '
               '확인할 것.')
    out.append('- 출력 방향(11-02): **정강이는 안쪽 면(y %.0f, 평평)을 바닥에** — 바퀴 모터 칸(Ø74) 안, 바퀴 판 밑에만 받침.  '
               '**허벅지는 바깥 면(y %.1f, 평평: Ø%.0f hip 구멍과 무릎 캡 자리만)을 바닥에** — 무릎 하우징이 안쪽 면보다 4 mm 나와 있어 '
               '그쪽은 바닥으로 못 쓴다.  허벅지 안쪽 갈래는 갈래 틈(%.1f mm) 위에 떠 있어 틈 안에 받침이 필요하다(무릎 쪽이 열려 '
               '있어 빼기 쉬움).  닫힌 칸의 천장은 리브 칸 위를 브리지로 건넌다 — 칸 폭이 넓은 곳(약 30~40 mm)은 첫 층이 처질 수 '
               '있으니 브리지 설정(냉각 최대, 느리게)을 쓴다.'
               % (LV.Y_SI, LV.SPAR[1], 2 * LV.BORE_R, LV.SPAR[0] - LV.Y_TO))
    out.append('- 링크에 남은 열압입 인서트는 컵 기둥 M2.5와 무릎 캡 M2.5뿐이다(⑥ 인서트 없앰).  벽 3.5는 둘레선(0.45 × 8)으로 채우고 '
               '채움 100 %%로 출력한다.  PETG는 오래 눌리면 줄어든다(크리프) — 조임 토크를 낮게(M3 0.7~1.0 N·m, M2.5 0.5 N·m), '
               '나사 고정제를 쓰고 처음 몇 주 다시 조인다.')
    with open(os.path.join(HERE, 'strength_v5.md'), 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(out) + '\n')
    print('\n'.join(out))


if __name__ == '__main__':
    main()
