"""v5 hand calculations with every part printed (user 11-02: the links in PETG; 11-09: all parts in PLA / PETG, strength
not considered; 11-15: no metal parts): fork thigh and shank (closed boxes + Warren ribs, plan A spar), the hub inserts
clamped to the links by a backing ring (11-02), the knee's outboard 61807 support and the printed motor plates, for the
outboard wheel (tyre centre y 270 since the 11-02 wheel shift).  Only PETG properties are in this file: the parts the
BOM allows in PLA are computed with them too.

    python calc/strength_v5.py        # -> calc/strength_v5.md

Loads, load cases and formulas are those of ../v4/calc/strength_v4.py (= v3 strength.py; from robot_cad_v4/calc): contact 60 N static
per wheel, design 300 N, check 600 N (SF 2), lateral 0.5 x, motor torque 12.5 / 25 N m; stresses at the check load,
deflections at the design load.  Where v4 compared a machined part with a metal limit, the same stress is compared here
with the PETG value (within a layer or across the layers, by the print orientation of BOM.md 2).  Geometry: fusion/legs_v5.py and the v4 params lengthened by fusion/link_length.py (no Fusion needed: a
stub stands in for adsk, as in manufacturing/explain).  Part volumes: Fusion report() of 11-07 (links 250 mm, ribs 3.0 x 9 / 10).
"""
import math
import os
import sys
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
V5 = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(V5, 'v4', 'fusion'))
sys.path.insert(0, os.path.join(V5, 'fusion'))
for _m in ('adsk', 'adsk.core', 'adsk.fusion'):
    sys.modules.setdefault(_m, mock.MagicMock())
import params as P            # noqa: E402
import link_length as LL      # noqa: E402
LL.lengthen(P)                # before anything reads them at import (v5: links 250 mm, user 11-07)
import v5_params as VP  # noqa: E402
VP.apply(P)                   # v5-only values (11-08)
import legs_v5 as LV          # noqa: E402

# ----------------------------------------------------------------------------- materials
# PETG (FDM, the values v2 / v3 used): E 2000, tensile / flexural 45 within a layer, bearing 50; across the layers
# (build direction) taken as 0.6 x (assumption - FDM layer bonding, measure a test print); HDT about 70 C
E_PETG, NU_PETG = 2000.0, 0.35
S_PETG, C_PETG, S_PETG_Z, HDT_PETG = 45.0, 50.0, 0.6 * 45.0, 70.0
MAT = {'PETG': (E_PETG, E_PETG / (2 * (1 + NU_PETG)), S_PETG, 1.27)}   # E, G, strength (MPa), density (g/cm3)
AS_M3, AS_M25 = 5.03, 3.39
PROOF = {'8.8': 600.0, '12.9': 970.0}   # bought steel screws (BOM.md 4)
PULL_M25 = 600.0              # heat-set M2.5 insert pull-out in PETG, order of magnitude (v3: supplier tests); here
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
# Fusion report() 11-07 (links 250 mm, ribs 3.0), one side: volume cm3 of the thigh / shank PETG body and the closed cells
# (outer shell - body: 1945.8 - 651.6, 758.9 - 331.3); backing ring 4.6 cm3 per joint
VOL = {'thigh': 651.6, 'shank': 331.3}
VOL_RING = 4.6
CELLS = {'thigh': 1945.8 - 651.6, 'shank': 758.9 - 331.3}
ROBOT_MODEL = 11.597          # model mass, the 'robot' value of calc/mass_properties.json (hub version 11): the hub inserts,
                              # backing rings, knee caps, hip plates, spine bars, rails, clamp rings and magnet spacers
                              # are still at their metal density there (README 5)
BASE_175 = dict(vm=18.4, end=(0.18, 1.90), contact=8.6, contact_sep=12.2, m_hip=185, mass=10.931)   # hub version 9, 175 mm links, ribs 2.0
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


def ring_peak(F, width, t, base, E=E_PETG, E_ring=E_PETG):
    """Peak pressure under a clamp ring next to one screw: ring = beam on an elastic printed foundation (v4; the ring
    is printed as well, so its modulus is the PETG one)."""
    beta = (E / base / (4 * E_ring * t ** 3 / 12)) ** 0.25
    return F * beta / (2 * width), math.pi / beta


def flange_wall_pressure(m):
    """Compression of the PETG wall under the backing ring by the joint moment m alone (N mm -> MPa): the ring annulus
    as a rigid footprint.  The screws' preload is left out: ⑥ runs in a hand-tapped printed thread, tightened lightly."""
    ri, ro = LV.RING[0], LV.RING[1]
    return m * ro / (math.pi / 4 * (ro ** 4 - ri ** 4))


def main():
    out.append('# v5 강도 손계산 — 모든 부품을 출력 (`python calc/strength_v5.py`)\n')
    out.append('사용자 결정: 11-02 허벅지·정강이를 **PETG(FDM)로 출력**, 11-07 **링크 25 cm + 리브 3.0**, 11-09 **모든 부품을 PLA·PETG로 출력하고 '
               '강도는 보지 않는다**, 11-15 금속 부품은 없다.  형상은 '
               'v5 (`fusion/legs_v5.py`, 링크 %.0f mm): 바깥 막대 y %.1f ~ %.1f, 갈래 틈 %.1f, 허벅지 상자 폭 %.1f, 정강이 기본층 '
               'y %.0f ~ %.0f(무릎 판과 틈 %.1f), 바퀴는 바깥쪽(타이어 중심면 y %.1f, 11-02 바퀴 묶음 2.5 mm 바깥으로), hub 인서트는 '
               '받침 링으로 PETG 벽을 끼워 고정(11-02).  하중·식은 v4(`v4/calc/strength_v4.py`)와 같다: 정지 %.0f N/바퀴, '
               '설계 %.0f N, 검토 %.0f N(안전계수 2), 옆 %.0f / %.0f N, 관절 토크 설계 12.5 / 검토 25 N·m.  응력은 검토 하중, 처짐은 '
               '설계 하중.\n'
               % (P.L1, LV.SPAR[0], LV.SPAR[1], LV.SPAR[0] - LV.Y_TO, LV.SPAR[1] - LV.Y_TF, LV.Y_SI, LV.Y_SO, LV.KNEE_GAP, YTC,
                  F_STATIC, F_DES, F_CHK, FL_DES, FL_CHK))
    out.append('재료: PETG — 탄성 %.0f MPa, 층 안 인장·굽힘 %.0f MPa, 지압 %.0f MPa(v2·v3 값), **층 사이(출력 높이 방향)는 0.6배인 '
               '%.0f MPa로 가정**(시험 출력으로 확인), 밀도 %.2f, HDT 약 %.0f °C.  링크는 옆면을 바닥에 놓고(y가 출력 높이) '
               '출력한다: 링크 굽힘·비틀림의 면판 응력은 층 안, 인서트를 뽑는 힘은 층 사이로 간다.  hub 인서트·받침 링·무릎 캡·hip 판·'
               '클램프 링도 출력 부품이다(`BOM.md` 2절).  PLA 값은 이 스크립트에 없어 모두 PETG 값으로 계산했다.  hub 인서트의 보스와 무릎 캡의 관은 '
               '플랜지를 바닥에 놓고 뽑아 축이 출력 높이 방향이므로 굽힘 응력을 층 사이 값과 견줬고, 눕혀 뽑는 hip 판은 층 안 값과 견줬다.  '
               '나사·핀·베어링은 구매품이다.\n'
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
    for comp, nm in ((True, '**PETG, 갈래 함께 굽음**'), (False, 'PETG, 갈래 따로 굽음 (아래 한계)')):
        res, d, tw, sf, fb = legs3('PETG', comp)
        E, G, lim, _r = MAT['PETG']
        vm = max(res['thigh']['vm'], res['shank']['vm'])
        sb = plate_buckling(LV.WALL, fb, E, 0.35)
        fine = max(res['thigh']['d_v2'], res['shank']['d_v2']) <= 1.0 and vm <= lim and sf <= sb
        worst = 'thigh' if res['thigh']['vm'] >= res['shank']['vm'] else 'shank'
        rows.append((nm, '%.2f / %.2f' % (res['thigh']['d_v2'], res['shank']['d_v2']), '%.1f (%.1f)' % (d, tw),
                     '%.0f (%s s %.0f) / %.0f' % (vm, {'thigh': '허벅지', 'shank': '정강이'}[worst], res[worst]['at'], lim),
                     '%.0f / %.0f (폭 %.0f)' % (sf, sb, fb), '통과' if fine else '**초과**'))
        chosen[comp] = (res, d, tw, sf, sb, vm)
    table(('단면', '링크 끝 옆 처짐 허벅지 / 정강이 [mm] (한계 1)', '접지점 옆 변위 [mm] (비틀림)', '검토 응력 / 강도 [MPa]',
           '면판 압축 / 좌굴 [MPa]', '판정'), rows)
    res, d, tw, sf, sb, vm = chosen[True]
    _res2, d2, _tw2, _sf2, _sb2, vm2 = chosen[False]
    rib_now = LV.RIB
    LV.RIB = 2.0
    res_r2, d_r2, _t, _f, _b = legs3('PETG', True)
    LV.RIB = rib_now
    b_rib = 2 * LV.sh_half(sum(LV.RIB_SPAN['tibia']) / 2) - 2 * LV.WALL
    tau_cr = lambda t: 5.35 * math.pi ** 2 * E_PETG / (12 * (1 - NU_PETG ** 2)) * (t / b_rib) ** 2
    out.append('\n**링크 25 cm + 리브 %.1f (11-07)**: 관절 사이 거리를 %.0f mm 늘렸다.  관절 원과 그 안의 모터 자리, 갈래는 175 mm 링크 그대로고, 옆면은 관절 원에서 관절 원까지 한 직선이다.  '
               '리브 간격을 유지해 허벅지 %d개, 정강이 %d개로 늘렸다.  링크 응력은 거의 그대로지만 옆 처짐은 길이만큼 늘고, **리브를 2.0 → %.1f로 두껍게 해도 '
               '옆 처짐은 정강이 끝 %.2f → %.2f mm, 접지점 %.1f → %.1f mm로 거의 줄지 않는다**(옆 처짐은 면·옆 벽이 정한다).  리브가 좋아지는 곳은 리브 자체의 전단 좌굴이다(%.1f → %.1f MPa).\n'
               % (LV.RIB, LV.STRETCH, LV.RIBS['thigh'], LV.RIBS['tibia'], LV.RIB, res_r2['shank']['d_v2'], res['shank']['d_v2'], d_r2, d, tau_cr(2.0), tau_cr(LV.RIB)))
    table(('항목', '175 mm, 리브 2.0 (허브 버전 9)', '%.0f mm, 리브 %.1f' % (P.L1, LV.RIB)),
          (('검토 응력 [MPa]', '%.1f' % BASE_175['vm'], '%.1f' % vm),
           ('링크 끝 옆 처짐 허벅지 / 정강이 [mm]', '%.2f / %.2f' % BASE_175['end'], '%.2f / %.2f' % (res['thigh']['d_v2'], res['shank']['d_v2'])),
           ('접지점 옆 변위, 갈래 함께 / 따로 [mm]', '%.1f / %.1f' % (BASE_175['contact'], BASE_175['contact_sep']), '%.1f / %.1f' % (d, d2)),
           ('로봇 모델 질량 [kg]', '%.3f' % BASE_175['mass'], '%.3f' % ROBOT_MODEL)))
    out.append('\n- **정강이 옆 처짐 %.2f mm가 v2부터 쓴 링크 하나 1 mm 기준을 넘는다**: 정강이는 무릎 패드(r 70) 밖에서 폭이 '
               '%.0f mm뿐이다(v4 정강이는 바깥으로 %.0f까지 넓혔지만, v5는 그쪽에 타이어가 있다).  허벅지는 폭 %.0f 상자라 단단하다(%.2f mm).  '
               '접지점 전체 옆 변위는 %.1f mm로 v4(10.5) · v2(12.8) · 175 mm v5(8.6)보다 크다.  링크 하나 기준을 지킬지, 접지점 변위로 '
               '볼지는 결정할 것 — 줄이려면 정강이 폭을 키워야 하는데 바깥에 타이어, 안쪽에 허벅지가 있다.'
               % (res['shank']['d_v2'], LV.Y_SO - LV.Y_SI, P.TIBIA_WIDE_OUT - P.Y_TIBIA_IN, LV.SPAR[1] - LV.Y_TF,
                  res['thigh']['d_v2'], d))
    out.append('- 갈래를 따로 굽는다고 보면 접지점 옆 변위 %.1f → %.1f mm, 검토 응력 %.0f → %.0f MPa — 실제는 그 사이다.  크로치 '
               '아래 두 상자를 잇는 것은 무릎 끝(모터 + 61807)뿐이라 함께 굽음은 위쪽 값이다.' % (d, d2, vm, vm2))
    out.append('- 비교: v4 SLA 레진 상자(바퀴 안쪽) 링크 끝 처짐 0.87 / 0.94 mm, 접지점 옆 변위 10.5 mm, v2 PETG 0.95 / 1.00 · 12.8 mm.')
    m_petg = {k: v * MAT['PETG'][3] for k, v in VOL.items()}
    out.append('- 질량(Fusion 부피, 한쪽): 허벅지 %.1f cm³ → PETG **%.0f g**, 정강이 %.1f cm³ → **%.0f g**, 받침 링 %.1f cm³ → %.0f g × 2.  '
               '벽 3.5 · 리브 3.0은 노즐 0.4로 둘레선만으로 채워지므로 꽉 찬 밀도로 셌다.  로봇 모델 질량 **%.2f kg**'
               '(`calc/mass_properties.json`; 일부 부품이 아직 금속 밀도라 실제보다 무겁다 — README 5절).  닫힌 칸은 허벅지 %.0f cm³ · '
               '정강이 %.0f cm³ — FDM이라 빈칸 그대로다(11-02 배출 구멍 없앰).'
               % (VOL['thigh'], m_petg['thigh'], VOL['shank'], m_petg['shank'], VOL_RING, VOL_RING * MAT['PETG'][3],
                  ROBOT_MODEL, CELLS['thigh'], CELLS['shank']))

    # ------------------------------------------------------------------ 2. joints
    h('2. 관절부')
    out.append('출력 부품(hub 인서트, 받침 링, hip 판, 클램프 링, 무릎 캡)의 응력은 PETG 값과 견줬다: 지압 %.0f, 층 안 %.0f, 층 사이 %.0f MPa.  '
               '괄호 안이 한계다.\n' % (C_PETG, S_PETG, S_PETG_Z))
    S_H, S_K, S_W = (0.0, P.Z_HIP), (P.KNEE_X, P.KNEE_Z), (0.0, P.R_WHEEL)
    y_out = {j: P.y_output(j) for j in ('hip', 'knee', 'wheel')}
    W_boss = math.pi * P.HUB_BOSS_D ** 3 / 32 - 6 * (math.pi / 4 * 3.4 ** 2) * 10 ** 2 / (P.HUB_BOSS_D / 2) / 2
    WA = math.pi / 4 * (7.0 ** 2 - 3.2 ** 2)                    # DIN 125 M3 washer
    a_head = math.pi / 4 * (4.5 ** 2 - 2.8 ** 2)                # M2.5 head on the hip plate
    rows = []
    M = {}
    V = {}                                                      # verdicts quoted in section 3
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
            V[('② 볼트', joint, tag)] = ok(fb, AS_M3 * PROOF['12.9'])
            V[('보스', joint, tag)] = ok(sb, S_PETG_Z)
            rows.append(('%s 출력면 옆 모멘트 %s %.0f N·m%s' % (joint, tag, m / 1000, ' (모터가 다 받을 때)' if joint == 'knee' else ''),
                         '② M3 12.9 인장 %.0f N (보증 %.0f), 와셔 밑 hub 인서트 면압 %.0f MPa (%.0f), hub 인서트 Ø28 보스 굽힘 %.0f MPa (층 사이 %.0f)'
                         % (fb, AS_M3 * PROOF['12.9'], fb / WA, C_PETG, sb, S_PETG_Z),
                         ok(max(fb / (AS_M3 * PROOF['12.9']), fb / WA / C_PETG, sb / S_PETG_Z), 1.0)))
            fi = bolt_circle_max(m, 6, LV.FLANGE_BOLT_R)
            pw = flange_wall_pressure(m)
            V[('벽', joint, tag)] = ok(pw, C_PETG)
            rows.append(('%s ⑥ 플랜지 → 받침 링 (PETG 벽 %.0f mm를 끼움), %s' % (joint, LV.WALL_J, tag),
                         'M3 × 16 6개 (r %.0f, 받침 링의 손 탭에 %.0f mm 물림) 하나에 외력 %.0f N; 받침 링 밑 PETG 벽 압축 %.0f MPa '
                         '(%.0f, 층 사이로 눌림, 예압 제외)' % (LV.FLANGE_BOLT_R, LV.RING[2], fi, pw, C_PETG),
                         '벽 %s, 탭 나사산은 계산 안 함' % V[('벽', joint, tag)]))
    for tag in ('검토', '설계'):
        m = M[('hip', tag)]
        ft = bolt_circle_max(m, 6, P.AKE_PCD_RING / 2)
        web = 289.0 * m / 133000.0
        V[('① 볼트', tag)] = ok(ft, AS_M25 * PROOF['12.9'])
        V[('웹', tag)] = ok(web, S_PETG)
        rows.append(('hip 고정링 ① M2.5 12.9 · hip 판, %s' % tag,
                     '인장 %.0f N (보증 %.0f), 머리 밑 hip 판 면압 %.0f MPa (%.0f); hip 판 웹 5 mm (v3 식 비례: 한 단면이 다 받으면) %.0f MPa (층 안 %.0f)'
                     % (ft, AS_M25 * PROOF['12.9'], ft / a_head, C_PETG, web, S_PETG),
                     ok(max(ft / (AS_M25 * PROOF['12.9']), ft / a_head / C_PETG, web / S_PETG), 1.0)))
    for joint in ('knee', 'wheel'):
        for tag in ('검토', '설계'):
            m = M[(joint, tag)]
            b, a, t = P.STATOR_RECESS_D / 2, P.CAN_CLEAR_R, P.KNEE_PLATE_T
            sc, ss, sg_ = annular(b, a, t, m), annular(b, a, t, m, outer='simple'), annular(b, a, t - 2.0, m, outer='simple')
            ft = bolt_circle_max(m, 6, P.AKE_PCD_RING / 2)
            pk, spread = ring_peak(ft, (P.RING_STATOR[0] - P.RING_STATOR[1]) / 2, P.RING_T, t - P.RING_T - 1.7)
            V[('판', joint, tag)] = ok(max(sg_ / S_PETG, ft / (AS_M25 * PROOF['8.8']), pk / C_PETG), 1.0)
            rows.append(('%s PETG 모터 판 %.0f mm + 고정링 ① M2.5 8.8, %s: %.0f N·m%s'
                         % (joint, t, tag, m / 1000, ' (모터가 다 받을 때)' if joint == 'knee' else ''),
                         '판 굽힘 바깥 고정 %.0f / 단순 지지 %.0f (리드 홈 자리 %.0f mm: %.0f) MPa (층 안 %.0f); 나사 인장 %.0f N (보증 %.0f), '
                         '클램프 링 밑 PETG 면압 %.0f MPa (%.0f)' % (sc, ss, t - 2.0, sg_, S_PETG, ft, AS_M25 * PROOF['8.8'], pk, C_PETG),
                         V[('판', joint, tag)]))
    f = T_CHK / (6 * LV.FLANGE_BOLT_R)
    rows.append(('⑥ 토크 25 N·m (검토)', '볼트 하나에 옆 힘 %.0f N → PETG 벽 구멍 지압 %.1f MPa (%.0f; Ø3.4 × %.0f, 마찰은 세지 않음)'
                 % (f, f / (3.4 * LV.WALL_J), C_PETG, LV.WALL_J), ok(f / (3.4 * LV.WALL_J), C_PETG)))
    f2 = T_CHK / (6 * P.AKE_PCD_RING / 2)
    web = P.KNEE_PLATE_T - P.POCKET - P.RING_T - 1.7
    rows.append(('고정링 → PETG 모터 판, 토크 25 N·m (검토)', 'M2.5 6 (r 18.25): 개당 %.0f N → 구멍 지압 %.1f MPa (%.0f; 웹 %.1f mm)'
                 % (f2, f2 / (2.5 * web), C_PETG, web), ok(f2 / (2.5 * web), C_PETG)))
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
        s_tube = R * lever / W_tube
        m_cap = R * (LV.SPAR[1] - LV.CAP_T - y_b)
        f_cap = bolt_circle_max(m_cap, 3, LV.CAP_BOLT_R)
        p_hole = 6 * m_cap / (2 * LV.CAP_R[1] * (LV.SPAR[1] - LV.SPAR[0]) ** 2)
        V[('61807', tag)] = ok(max(R / 1000 / BRG_61807[1], p_seat / C_PETG), 1.0)
        V[('캡 관', tag)] = ok(s_tube, S_PETG_Z)
        V[('캡 구멍', tag)] = ok(p_hole, C_PETG) if LV.CAP_FIT == 0 else ok(f_cap, PULL_M25)
        rows.append(('무릎 %s: 모멘트 %.0f N·m를 모터 베어링과 61807이 짝힘으로 (간격 %.0f mm)' % (tag, m / 1000, dd),
                     '61807 반경 하중 %.2f kN (C0 %.2f kN), 정강이 자리 PETG 면압 %.1f MPa (%.0f); 캡 관 굽힘 %.0f MPa (층 사이 %.0f, 팔 %.1f mm); '
                     '캡 관을 막대 구멍에 꼭 맞게 다듬음(Ø35, 11-02) → 구멍 면압 %.0f MPa (%.0f), 캡 M2.5 인서트 3개는 예압만 '
                     '(관이 놀면 인발 %.0f N, 어림 한계 %.0f)'
                     % (R / 1000, BRG_61807[1], p_seat, C_PETG, s_tube, S_PETG_Z, lever, p_hole, C_PETG, f_cap, PULL_M25),
                     '61807·자리 %s, 캡 관 %s, 캡 구멍 %s' % (V[('61807', tag)], V[('캡 관', tag)], V[('캡 구멍', tag)])))
    out.append('\n**무릎 바깥 지지 (61807)** — 모터 출력 베어링이 모멘트를 못 받는다고(핀) 보면 무릎 옆 모멘트는 두 베어링의 짝힘이 '
               '된다.  AKE60 출력 베어링 자리는 비공개라 출력면 안쪽 4 mm로 잡았다.\n')
    table(('경우', '힘·응력', '판정'), rows)

    # ------------------------------------------------------------------ 3. verdict
    mh = M[('hip', '검토')]
    h('3. 정리')
    out.append('사용자 11-09: 강도는 보지 않는다.  넘는 항목은 고치지 않고 그대로 둔 기록이다.\n')
    out.append('- **링크 몸통(PETG 상자)**: 검토 응력 %.0f MPa (%.0f) %s, 면판 좌굴 여유 %.1f배.  처짐은 허벅지 %.2f mm로 '
               '단단하고 **정강이 %.2f mm는 v2의 링크 하나 1 mm 기준을 넘는다**(1절); 접지점 전체 옆 변위는 %.1f mm(갈래 따로 '
               '굽으면 %.1f)로 v4(10.5) · v2(12.8)보다 크다.  정강이는 바깥에 타이어, 안쪽에 허벅지가 있어 폭 30을 못 넘는다 — **기준을 접지점 변위로 볼지 결정할 것**.'
               % (vm, S_PETG, ok(vm, S_PETG), sb / sf, res['thigh']['d_v2'], res['shank']['d_v2'], d, d2))
    out.append('- **⑥ hub 인서트 고정**: M3 × 16 6개가 플랜지 → PETG 벽 %.0f mm → 받침 링(Ø%.0f / Ø%.0f × %.0f, 링크 바깥 면 쪽 Ø%.0f 구멍으로 넣음)의 '
               '손 탭에 물린다.  hip에서 볼트 하나의 외력이 검토 %.1f kN · 설계 %.1f kN이고, 벽 압축은 검토 %.0f MPa (%.0f) %s.  **받침 링의 플라스틱 나사산이 '
               '이 힘을 견디는지는 계산하지 않았다** — 출력 부품이고 이 스크립트에 그 한계 값이 없다(11-09 사용자: 강도는 보지 않는다).  hip이 높아져 옆 힘의 팔이 '
               '길어진 탓이고, 관절부를 바꾸지 않기로 해서(사용자 11-07) 그대로 두었다.  조임은 `BOM.md` 4절대로 손으로 가볍게 하고 혐기성 나사 고정제는 쓰지 않는다.'
               % (LV.WALL_J, 2 * LV.RING[1], 2 * LV.RING[0], LV.RING[2], 2 * LV.BORE_R,
                  bolt_circle_max(mh, 6, LV.FLANGE_BOLT_R) / 1000, bolt_circle_max(M[('hip', '설계')], 6, LV.FLANGE_BOLT_R) / 1000,
                  flange_wall_pressure(mh), C_PETG, V[('벽', 'hip', '검토')]))
    out.append('- **hip 출력부**: hip 옆 모멘트가 검토 %.0f N·m다(175 mm 링크에서 %d, 안쪽 바퀴였다면 %.0f).  검토 하중에서 ② M3 12.9 인장 %.0f N (보증 %.0f) %s, '
               'hub 인서트 Ø28 보스 굽힘 %.0f MPa (층 사이 %.0f) %s, ① M2.5 12.9 %s, hip 판 웹 %.0f MPa (층 안 %.0f, 한 단면이 다 받는다고 볼 때) %s.  '
               '설계 하중에서는 ② %s, 보스 %s, ① %s, 웹 %s.  바깥 바퀴 + 긴 링크 + AKE60 출력부(M3 6개, PCD 20)에서 오는 값이라 다리 설계로는 못 고친다.'
               % (mh / 1000, BASE_175['m_hip'], M4[('hip', '검토')] / 1000, bolt_circle_max(mh, 6, 10), AS_M3 * PROOF['12.9'], V[('② 볼트', 'hip', '검토')],
                  mh / W_boss, S_PETG_Z, V[('보스', 'hip', '검토')], V[('① 볼트', '검토')], 289.0 * mh / 133000, S_PETG, V[('웹', '검토')],
                  V[('② 볼트', 'hip', '설계')], V[('보스', 'hip', '설계')], V[('① 볼트', '설계')], V[('웹', '설계')]))
    out.append('- **PETG 모터 판 + 클램프 링 ①**(모터가 옆 모멘트를 다 받는다고 볼 때): 바퀴는 검토 %s · 설계 %s, 무릎은 검토 %s · 설계 %s.  무릎은 실제로는 '
               '61807이 짝힘으로 나눠 받는다(11-02 캡 관을 막대 구멍에 꼭 맞게 함; 실제 몫은 두 베어링 강성에 달림).  그 짝힘에서 61807·정강이 자리는 검토 %s, '
               '캡 관 굽힘은 검토 %s · 설계 %s, 캡 구멍은 검토 %s.'
               % (V[('판', 'wheel', '검토')], V[('판', 'wheel', '설계')], V[('판', 'knee', '검토')], V[('판', 'knee', '설계')],
                  V[('61807', '검토')], V[('캡 관', '검토')], V[('캡 관', '설계')], V[('캡 구멍', '검토')]))

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
               '채움 100 %로 출력한다.  PETG는 오래 눌리면 줄어든다(크리프) — 출력 부품의 나사산과 인서트에 들어가는 나사는 손으로 가볍게 조이고 '
               '처음 몇 주 다시 조인다.  출력 부품의 나사산에는 혐기성 나사 고정제를 쓰지 않는다(`BOM.md` 6절: PLA·PETG가 갈라질 수 있다).')
    with open(os.path.join(HERE, 'strength_v5.md'), 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(out) + '\n')
    print('\n'.join(out))


if __name__ == '__main__':
    main()
