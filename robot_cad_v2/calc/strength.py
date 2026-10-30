"""Hand calculations of the v2 legs, joint interfaces, wheel hub and torso load paths (brief items 3, 4 and 7).

    python calc/strength.py            # writes calc/strength.md (the README quotes its numbers)

Loads (brief item 3, robot 7 kg + payload 5 kg = 12 kg): static 60 N per wheel, design load 5 x = 300 N,
checked with the safety factor 2 at 600 N; lateral force 0.5 x vertical at the tyre contact (150 / 300 N);
motor torque 12.5 N m with SF 2 = 25 N m at every joint and hub interface.  Stresses are evaluated at the check
level (SF 2 included) against the material strength; deflections at the design level, as the brief asks.

In-plane bending of a link is bounded by the joint torques: a knee holding the 300 N wheel load would need
F x knee lever = 24 N m, twice what the AKE60-8 can give, so the knee yields at 12.5 N m.  The links are therefore
checked with the motor torque at both ends (12.5 N m design, 25 N m check); the locked-joint value is listed
for reference.

Materials: printed PETG (Prusament-class datasheet, in-layer): E = 2.0 GPa, tensile 45 MPa, compressive 50 MPa;
legs print lying on their back edge so beam stresses run along the layers.  Section (params.PRINT): 12 walls
(4.8 mm) on the side faces, 8 layers (1.6 mm) on the edges, 20 % gyroid core counted at 10 % of the modulus and
strength.  The links are widened outboard where no other part sweeps (params.FEMUR_WIDE_* / TIBIA_WIDE_*), so the
section varies along the link: deflections are integrated along it.
6061-T6: E 69 GPa, yield 276 MPa.  Screws: M3 12.9 at the motor outputs, M2.5 12.9 at the hip stator ring,
M2.5 8.8 (DIN 7984) at the knee / wheel stator rings.  Clamp rings: SUS304 params.RING_T (params.RING_*).
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'fusion'))
import params as P            # noqa: E402  (pure Python dimensions, no Fusion import)

SIM = os.path.join(os.path.dirname(os.path.dirname(HERE)), 'wheeled_biped', 'results', 'ake_cad')

E_PETG, S_PETG, C_PETG = 2000.0, 45.0, 50.0
E_AL, Y_AL = 69000.0, 276.0
E_SUS = 193000.0
Y_6063 = {'T5': 108.0, 'T6': 177.0}               # KS minimum yield (11 / 18 kgf/mm2), tube / rail fallback
E_CF = 120000.0                                    # pultruded carbon rod (option only)
AS_M3, AS_M25 = 5.03, 3.39                         # stress areas mm2
PROOF = {'8.8': 600.0, '12.9': 970.0}
INSERT_M3_PULL = 1000.0                            # heat-set M3 in PETG, order of magnitude (supplier tests)

F_STATIC = 60.0
F_DES, F_CHK = 5 * F_STATIC, 10 * F_STATIC
FL_DES, FL_CHK = P.LATERAL * F_DES, P.LATERAL * F_CHK
T_DES, T_CHK = P.MOTOR_PEAK * 1000.0, 2 * P.MOTOR_PEAK * 1000.0      # N mm
PHI = P.PHI
YF = (P.Y_FEMUR_IN + P.Y_FEMUR_OUT) / 2
YT = (P.Y_TIBIA_IN + P.Y_TIBIA_OUT) / 2
YTC = P.tyre_centre_y()
out = []
verdict = []


def h(t):
    out.append('\n## ' + t + '\n')


def row(*cells):
    out.append('| ' + ' | '.join(str(c) for c in cells) + ' |')


def table(head, rows):
    row(*head)
    row(*['---'] * len(head))
    for r in rows:
        row(*r)


def ok(v, lim):
    """Verdict; up to 5 % over the limit is reported as borderline."""
    return '통과' if v <= lim else ('경계' if v <= 1.05 * lim else '**초과**')


# ----------------------------------------------------------------------------- printed beam section
WALL, INFILL = P.PRINT['thigh']
SKIN = 1.6
CORE = INFILL / 2                     # gyroid modulus / strength fraction ~ half its density


def section(ya, yb, hgt, yg, groove=(12.0, 8.0), wall=WALL, core=CORE):
    """Rectangles (y0, y1, n0, n1, factor) of a printed beam spanning robot y ya..yb (lateral) and hgt (in-plane):
    walls on the side faces, skins on the edges, the 12 x 8 cable groove centred at y = yg on the -n edge."""
    w, s = wall, SKIN
    r = [(ya, ya + w, -hgt / 2, hgt / 2, 1.0), (yb - w, yb, -hgt / 2, hgt / 2, 1.0),
         (ya + w, yb - w, -hgt / 2, -hgt / 2 + s, 1.0), (ya + w, yb - w, hgt / 2 - s, hgt / 2, 1.0),
         (ya + w, yb - w, -hgt / 2 + s, hgt / 2 - s, core)]
    if groove:
        gw, gd = groove
        cut = []
        for (y0, y1, n0, n1, f) in r:
            a0, a1 = max(y0, yg - gw / 2), min(y1, yg + gw / 2)
            b0, b1 = max(n0, -hgt / 2), min(n1, -hgt / 2 + gd)
            if a1 > a0 and b1 > b0:
                cut.append((a0, a1, b0, b1, -f))
        r += cut
    return r


def props(rects):
    """EA, Iy (out-of-plane), In (in-plane), outer fibre distances cy, cn, lateral centroid yc."""
    EA = sum((y1 - y0) * (n1 - n0) * f for (y0, y1, n0, n1, f) in rects)
    yc = sum((y1 - y0) * (n1 - n0) * f * (y0 + y1) / 2 for (y0, y1, n0, n1, f) in rects) / EA
    nc = sum((y1 - y0) * (n1 - n0) * f * (n0 + n1) / 2 for (y0, y1, n0, n1, f) in rects) / EA
    Iy = sum(f * ((n1 - n0) * (y1 - y0) ** 3 / 12 + (y1 - y0) * (n1 - n0) * ((y0 + y1) / 2 - yc) ** 2)
             for (y0, y1, n0, n1, f) in rects)          # out-of-plane bending
    In = sum(f * ((y1 - y0) * (n1 - n0) ** 3 / 12 + (y1 - y0) * (n1 - n0) * ((n0 + n1) / 2 - nc) ** 2)
             for (y0, y1, n0, n1, f) in rects)          # in-plane bending
    cy = max(max(abs(y0 - yc), abs(y1 - yc)) for (y0, y1, n0, n1, f) in rects if f > 0.5)
    cn = max(max(abs(n0 - nc), abs(n1 - nc)) for (y0, y1, n0, n1, f) in rects if f > 0.5)
    return EA, Iy, In, cy, cn, yc


def ramp(s, s1, s2, a, b):
    return a if s <= s1 else (b if s >= s2 else a + (b - a) * (s - s1) / (s2 - s1))


def link(name, L, ya, yb, yg, hgt, z_root, z_tip, locked_root, locked_tip, wall=WALL, core=CORE):
    """Link of length L from the upper joint (s = 0, height z_root) to the lower one (z_tip); section y ya..yb(s).
    Out-of-plane moment Mx = lateral force x height + vertical force x (section centroid - tyre plane), split into
    bending (cos phi) and torsion (sin phi) of the inclined link; in-plane moment = the joint torque at both ends.
    Stresses at both ends and where the widened part ends; tip deflections integrated along the link."""
    sec = lambda s: props(section(ya, yb(s), hgt, yg, wall=wall, core=core))
    s_end = next((L * i / 400 for i in range(400) if yb(L * i / 400) <= yb(L) + 1e-9), L)
    rows = []
    vm_max = 0.0
    for tag, F, Fl, Tm in (('검토(SF 2)', F_CHK, FL_CHK, T_CHK), ('설계', F_DES, FL_DES, T_DES)):
        for where, s in (('위 끝', 0.0), ('넓은 부분 끝', s_end), ('아래 끝', L)):
            A, Iy, In, cy, cn, yc = sec(s)
            z = z_root + (z_tip - z_root) * s / L
            Mx = Fl * z + F * (yc - YTC)
            Mb, Tq = Mx * math.cos(PHI), Mx * math.sin(PHI)
            s_out, s_in, s_ax = Mb * cy / Iy, Tm * cn / In, F * math.cos(PHI) / A
            tau = Tq / (2 * (yb(s) - ya - wall) * (hgt - SKIN) * SKIN)
            vm = math.sqrt((s_out + s_in + s_ax) ** 2 + 3 * tau ** 2)
            if tag.startswith('검토'):
                vm_max = max(vm_max, vm)
            rows.append((name, tag, where, '%.0f' % (yb(s) - ya), '%.1f' % (Mx / 1000), '%.1f' % (Mb / 1000),
                         '%.1f' % (Tq / 1000), '%.1f' % (Tm / 1000), '%.1f' % s_out, '%.1f' % s_in, '%.1f' % tau,
                         '**%.1f**' % vm))
    # deflections of the link end at the design level (cantilever from the upper joint)
    n = 400
    d_lat = d_in = d_locked = 0.0
    for i in range(n):
        s = (i + 0.5) * L / n
        A, Iy, In, cy, cn, yc = sec(s)
        z = z_root + (z_tip - z_root) * s / L
        Mb = (FL_DES * z + F_DES * (yc - YTC)) * math.cos(PHI)
        arm = (L - s) * L / n
        d_lat += Mb * arm / (E_PETG * Iy)
        d_in += T_DES * arm / (E_PETG * In)                                   # 12.5 N m at both ends (worst case)
        d_locked += (locked_root + (locked_tip - locked_root) * s / L) * arm / (E_PETG * In)
    A0, Iy0, In0, _cy, _cn, _yc = sec(0.0)
    return dict(name=name, rows=rows, vm=vm_max, EIy=E_PETG * Iy0, EIn=E_PETG * In0, d_lat=d_lat, d_in=d_in,
                d_locked=d_locked, s_end=s_end)


def bolt_circle_max(M, n, r):
    """Largest screw tension of n screws on radius r under moment M (neutral axis through the centre)."""
    return 2 * M / (n * r)


def ring_peak(F, width, t, petg):
    """Peak PETG pressure under a clamp ring next to one screw: ring = beam (thickness t, radial width) on an
    elastic PETG foundation (thickness petg), p = F beta / (2 width).  Also returns the spread length pi / beta."""
    beta = (E_PETG / petg / (4 * E_SUS * t ** 3 / 12)) ** 0.25
    return F * beta / (2 * width), math.pi / beta


def ring_area(od, idia, n, hole):
    return math.pi / 4 * (od ** 2 - idia ** 2) - n * math.pi / 4 * hole ** 2


def main():
    out.append('# v2 강도 손계산 (`python calc/strength.py`)\n')
    out.append('하중(브리프 3번, 로봇 7 kg + 짐 5 kg = 12 kg): 정지 %.0f N/바퀴, 설계 %.0f N, 검토 %.0f N(안전계수 2), '
               '옆 하중 %.0f / %.0f N(접지점), 모터 토크 12.5 N·m → 검토 25 N·m.  기준 자세 L0 = %.0f mm, 허벅지 수직에서 '
               '%.1f°, 무릎은 hip보다 %.1f mm 앞, 타이어 중심면 y = %.1f mm(왼쪽).  응력은 검토 하중(SF 2 포함)으로 '
               '재료 강도와 비교하고, 처짐은 설계 하중으로 본다.'
               % (F_STATIC, F_DES, F_CHK, FL_DES, FL_CHK, P.L0, math.degrees(PHI), P.KNEE_X, YTC))
    out.append('\n재료: PETG(층 방향) E 2.0 GPa, 인장 45 MPa, 압축 50 MPa / 6061-T6 E 69 GPa, 항복 276 MPa / '
               '볼트 8.8 보증응력 600 MPa, 12.9 보증응력 970 MPa.')

    # ------------------------------------------------------------------ 1. links
    k_lock = F_DES * P.KNEE_X
    yg_f, yg_t = (P.Y_FEMUR_IN + P.Y_FEMUR_OUT) / 2, (P.Y_TIBIA_IN + P.Y_TIBIA_OUT) / 2
    sf = P.L1 - P.FEMUR_WIDE_END
    st = P.L2 - P.TIBIA_WIDE_END
    wide_f = lambda s: ramp(s, sf - P.WIDE_TAPER, sf, P.FEMUR_WIDE_OUT, P.Y_FEMUR_OUT)
    wide_t = lambda s: ramp(s, st - P.WIDE_TAPER, st, P.TIBIA_WIDE_OUT, P.Y_TIBIA_OUT)
    nf = '허벅지 %.0f→%.0f×%.0f' % (P.FEMUR_WIDE_OUT - P.Y_FEMUR_IN, P.FEMUR_T, P.FEMUR_H)
    nt = '정강이 %.0f→%.0f×%.0f' % (P.TIBIA_WIDE_OUT - P.Y_TIBIA_IN, P.TIBIA_T, P.TIBIA_H)
    links = [link(nf, P.L1, P.Y_FEMUR_IN, wide_f, yg_f, P.FEMUR_H, P.Z_HIP, P.KNEE_Z, 0.0, k_lock),
             link(nt, P.L2, P.Y_TIBIA_IN, wide_t, yg_t, P.TIBIA_H, P.KNEE_Z, P.R_WHEEL, k_lock, 0.0)]
    before = [link('보강 전 허벅지 30×50', P.L1, P.Y_FEMUR_IN, lambda s: P.Y_FEMUR_OUT, yg_f, 50.0, P.Z_HIP,
                   P.KNEE_Z, 0.0, k_lock, wall=2.4, core=0.2),
              link('보강 전 정강이 30×46', P.L2, P.Y_TIBIA_IN, lambda s: P.Y_TIBIA_OUT, yg_t, 46.0, P.KNEE_Z,
                   P.R_WHEEL, k_lock, 0.0, wall=2.4, core=0.2)]
    h('1. 허벅지·정강이 링크 (PETG, 등 쪽 모서리를 바닥에 두고 출력)')
    out.append('PETG만으로 옆 강성을 올리려고 두 링크를 다른 부품이 지나가지 않는 바깥쪽으로 넓혔다: 허벅지는 hip부터 무릎 '
               '%.0f mm 앞까지 폭 %.0f mm(무릎 쪽은 정강이 머리가 바깥에 있어 30 mm), 정강이는 무릎부터 바퀴 축 %.0f mm 위까지 폭 '
               '%.0f mm(아래는 바퀴 컵·드라이버 커넥터), 넓은 끝 %.0f mm는 경사로 줄어든다.  정강이 면내 높이 %.0f mm.  '
               '출력: 벽 12겹(4.8 mm), 위아래 8층, 자이로이드 20 %%(강성 10 %%로 계산), 12×8 배선 홈 포함.  면내 굽힘은 '
               '양 끝 관절 토크(설계 12.5, 검토 25 N·m) — 무릎을 잠근다고 가정하면 설계 하중에서 %.1f N·m가 필요하지만 '
               'AKE60-8은 12.5 N·m에서 밀리므로 링크가 그 이상을 받지 않는다.  면외 굽힘 = 옆 하중 × 높이 + 수직 하중 × '
               '(단면 도심 − 타이어 면), 기운 링크에서 굽힘(cos)과 비틀림(sin)으로 나눔.\n'
               % (P.FEMUR_WIDE_END, P.FEMUR_WIDE_OUT - P.Y_FEMUR_IN, P.TIBIA_WIDE_END, P.TIBIA_WIDE_OUT - P.Y_TIBIA_IN,
                  P.WIDE_TAPER, P.TIBIA_H, k_lock / 1000))
    table(('링크', '하중', '위치', '폭 [mm]', 'Mx [N·m]', '면외 M [N·m]', 'T [N·m]', '면내 M [N·m]', 'σ 면외 [MPa]',
           'σ 면내 [MPa]', 'τ [MPa]', 'von Mises [MPa]'), [r for L in links for r in L['rows']])
    out.append('\n강도 45 MPa(층 방향) 기준 검토 하중 최대: ' +
               ', '.join('%s %.1f MPa (%s)' % (L['name'], L['vm'], ok(L['vm'], S_PETG)) for L in links) + '.')
    for L in links:
        verdict.append(('%s 응력(검토 600 N + 옆 300 N + 25 N·m)' % L['name'], '%.1f MPa / 45' % L['vm'],
                        ok(L['vm'], S_PETG)))
    out.append('\n링크 끝 처짐(설계 하중 300 N, 옆 150 N, 관절 토크 12.5 N·m; 단면을 따라 적분):\n')
    table(('링크', 'EI 면외, 위 끝 [N·mm²]', 'EI 면내, 위 끝 [N·mm²]', '면내 [mm] (한계 1)', '면내, 관절 잠금 가정 [mm]',
           '옆(면외) [mm] (한계 1)'),
          [(L['name'], '%.3g' % L['EIy'], '%.3g' % L['EIn'], '**%.2f**' % L['d_in'], '%.2f' % L['d_locked'],
            '**%.2f**' % L['d_lat']) for L in links + before])
    for L in links:
        verdict.append(('%s 면내 처짐(설계)' % L['name'], '%.2f mm / 1' % L['d_in'], ok(L['d_in'], 1.0)))
        verdict.append(('%s 옆 처짐(설계, 옆 150 N)' % L['name'], '%.2f mm / 1' % L['d_lat'], ok(L['d_lat'], 1.0)))
    out.append('\n보강 전(폭 30 mm, 벽 6겹, 자이로이드 40 %%) 옆 처짐 %.1f / %.1f mm가 %.2f / %.2f mm로 줄었다.  옆 강성은 단면 '
               '폭의 세제곱에 비례하므로 같은 폭에서 채움만 늘려서는 1 mm에 닿지 않는다(100 %% 채움 30 mm 폭: 약 3.4 / 2.8 mm).  '
               '대가는 질량(링크 한쪽 약 +0.4 kg, README 참조)이다.  옆 150 N은 바퀴당 수직의 0.5배, 로봇 전체로 2.5 g 옆 가속에 '
               '해당하는 충격 하중이다.'
               % (before[0]['d_lat'], before[1]['d_lat'], links[0]['d_lat'], links[1]['d_lat']))

    # ------------------------------------------------------------------ 2. joints
    h('2. 관절 체결부 (AKE60-8: 출력 Ø27 6×M3 PCD 20 + Ø3 다웰 3개, 고정링 6×M2.5 PCD 36.5)')
    out.append(('출력 쪽(hip·knee 구동 보스)과 PETG 고정판(knee·wheel)의 나사 머리는 모두 %g mm SUS304 클램프 링 위에 '
                '앉는다(맨 PETG 위 M3 와셔면 검토 하중에서 지압 130 MPa 이상으로 눌린다).  고정링 링은 나사 간격(%.1f mm)이 '
                '링이 힘을 퍼뜨리는 길이보다 길어 나사 옆 최대 지압(탄성 기초 위 보)으로 보고, 출력 링은 나사 간격 10 mm가 '
                '퍼짐 길이보다 짧아 평균 지압으로 본다.\n') % (P.RING_T, P.AKE_PCD_RING * math.sin(math.pi / 6)))
    rows = []
    a_out = ring_area(P.RING_OUT[0], P.RING_OUT[1], 6, 3.4) / 6
    a_st = ring_area(P.RING_STATOR[0], P.RING_STATOR[1], 6, 2.8) / 6
    web = P.KNEE_PLATE_T - P.POCKET - P.RING_T - P.CBORE_M25[1]
    floor = P.BEARING_FACE - P.RING_T
    f = T_CHK / (9 * 10.0)
    rows.append(('구동 보스 ← 출력, 토크 25 N·m', 'M3 6 + 다웰 3 (r 10): 개당 %.0f N' % f,
                 'PETG 구멍 지압 %.1f MPa (보스 바닥 %.1f mm)' % (f / (3.0 * floor), floor), ok(f / (3.0 * floor), C_PETG)))
    f = T_CHK / (6 * P.AKE_PCD_RING / 2)
    rows.append(('고정링 → PETG 판, 토크 25 N·m', 'M2.5 6 (r 18.25): 개당 %.0f N' % f,
                 'PETG 구멍 지압 %.1f MPa (링 홈 아래 웹 %.1f mm, 체결 마찰 무시)' % (f / (2.5 * web), web),
                 ok(f / (2.5 * web), C_PETG)))
    verdict.append(('관절 토크 25 N·m 전달(출력·고정링)', '지압 ≤ %.0f MPa / 50' % (f / (2.5 * web)), ok(f / (2.5 * web), C_PETG)))
    Mx_hip = lambda F, Fl: Fl * P.Z_HIP + F * (YF - YTC)
    Mx_knee = lambda F, Fl: Fl * P.KNEE_Z + F * (P.Y_FEMUR_OUT - YTC)
    Mx_wheel = lambda F, Fl: Fl * P.R_WHEEL + F * P.wheel_offset()
    W_boss = math.pi * P.HUB_BOSS_D ** 3 / 32 - 6 * (math.pi / 4 * 3.4 ** 2) * 10 ** 2 / (P.HUB_BOSS_D / 2) / 2
    for joint, fn in (('hip', Mx_hip), ('knee', Mx_knee)):
        for tag, F, Fl in (('검토', F_CHK, FL_CHK), ('설계', F_DES, FL_DES)):
            M = fn(F, Fl)
            fb = bolt_circle_max(M, 6, 10.0)
            sb = M / W_boss
            pb = fb / a_out
            res = ok(max(fb / (AS_M3 * PROOF['12.9']), sb / S_PETG, pb / C_PETG), 1.0)
            rows.append(('%s 출력 보스, 옆 하중 %s: Mx %.0f N·m' % (joint, tag, M / 1000),
                         'M3 12.9 인장 %.0f N (보증 %.0f)' % (fb, AS_M3 * PROOF['12.9']),
                         'Ø28 보스 굽힘 %.0f MPa (45), 링 아래 지압 %.0f MPa (50)' % (sb, pb), res))
            verdict.append(('%s 출력 보스 옆 하중(%s)' % (joint, tag), '굽힘 %.0f MPa / 45, 지압 %.0f / 50' % (sb, pb), res))
    for joint, fn, cls, plate in (('hip', Mx_hip, '12.9', '6061 hip 판 (링 없음)'),
                                  ('knee', Mx_knee, '8.8', 'PETG 무릎 판 + 링'),
                                  ('wheel', Mx_wheel, '8.8', 'PETG 바퀴 판 + 링')):
        for tag, F, Fl in (('검토', F_CHK, FL_CHK), ('설계', F_DES, FL_DES)):
            M = fn(F, Fl)
            ft = bolt_circle_max(M, 6, P.AKE_PCD_RING / 2)
            proof = AS_M25 * PROOF[cls]
            if joint == 'hip':
                txt, res = '알루미늄 판, 머리 지압 문제 없음', ok(ft, proof)
            else:
                pb = ft / a_st
                pk, spread = ring_peak(ft, (P.RING_STATOR[0] - P.RING_STATOR[1]) / 2, P.RING_T, web)
                txt = ('SUS 링 아래 PETG 지압 평균 %.1f, 나사 옆 최대 %.0f MPa (50; 링 t %g, 웹 %.1f mm, 퍼짐 %.0f mm)'
                       % (pb, pk, P.RING_T, web, spread))
                res = ok(max(ft / proof, pk / C_PETG), 1.0)
            rows.append(('%s 고정링, 옆 하중 %s: Mx %.0f N·m' % (joint, tag, M / 1000),
                         'M2.5 %s 인장 %.0f N (보증 %.0f), %s' % (cls, ft, proof, plate), txt, res))
            if tag == '검토':
                verdict.append(('%s 고정링 나사 옆 하중(검토)' % joint, '%.0f N / %.0f' % (ft, proof), res))
    table(('부위', '체결', '응력', '판정'), rows)
    out.append('\nhip 출력 보스는 검토 하중(600 N + 옆 300 N)에서 PETG 굽힘이 강도를 넘는다.  보스 지름은 hip 판의 Ø30 구멍과 '
               '고정링 나사 머리(r 18.25)에 묶여 Ø28이 최대이므로, 같은 PETG로는 SF 2를 맞출 수 없다(설계 하중에서는 SF %.1f).  '
               '이 하중에서는 AKE60 출력 베어링(정격 비공개)이 먼저 한계일 가능성이 크다 — CubeMars에 허용 모멘트 확인 필요.  '
               'hip 고정링 M2.5는 8.8이면 검토 하중에서 보증하중을 넘어 12.9(ISO 4762, 머리 높이 2.5 mm)로 지정했다; '
               'AKE 링 나사산(알루미늄, 깊이 5 mm) 전단 한계도 약 2.5~3 kN이라 나사는 끝까지 물려야 한다.'
               % (S_PETG / (Mx_hip(F_DES, FL_DES) / W_boss)))

    # ------------------------------------------------------------------ 3. hub
    h('3. 바퀴 허브 (PETG 100 %, 옆으로 눕혀 출력)와 17 mm 육각')
    d = P.wheel_offset()
    f1 = P.Y_TIBIA_IN - 1.0
    f0 = f1 - 8.0
    arm_joint = f0 - YTC
    rows = []
    rows.append(('출력면 → 타이어 중심면', '%.1f mm (타이어 폭 %.0f)' % (d, P.TYRE_W), '', ''))
    for tag, F, Fl in (('검토', F_CHK, FL_CHK), ('설계', F_DES, FL_DES)):
        M = F * d + Fl * P.R_WHEEL
        fb = bolt_circle_max(M, 6, 10.0)
        rows.append(('플랜지 → 출력, %s: Mx %.1f N·m' % (tag, M / 1000), 'M3 × %d 12.9 인장 %.0f N' % (P.HUB_SCREW_L, fb),
                     'Ø28 보스 %.1f MPa, DIN 125 와셔 지압 %.0f MPa' % (M / W_boss, fb / 30.4),
                     ok(max(M / W_boss / S_PETG, fb / 30.4 / C_PETG), 1.0)))
        if tag == '검토':
            verdict.append(('허브 플랜지 → 바퀴 모터 출력(검토)', '와셔 지압 %.0f MPa / 50' % (fb / 30.4),
                            ok(fb / 30.4, C_PETG)))
            fb_hub = fb
    # 10-30: M3 x 14 with the washer seat 9.5 from the output face (heads clear of the hex hub register spigot):
    # thread shear in the output taps over the shorter engagement (internal thread, ~50 % of the cylinder at the
    # major diameter; output material not published -- 6061-T6 shear 0.6 x yield assumed)
    eng = P.HUB_SCREW_L - P.HUB_SCREW_SEAT - 0.5
    tau = fb_hub / (0.5 * math.pi * 3.0 * eng)
    rows.append(('M3 × %d 출력 탭 물림 %.1f mm, 검토' % (P.HUB_SCREW_L, eng), '나사 인장 %.0f N (12.9 보증 %.0f N)'
                 % (fb_hub, PROOF['12.9'] * AS_M3), '나사산 전단 %.0f MPa (알루미늄 가정 %.0f)' % (tau, 0.6 * Y_AL),
                 ok(max(tau / (0.6 * Y_AL), fb_hub / (PROOF['12.9'] * AS_M3)), 1.0)))
    f = T_CHK / (6 * 18.0)
    rows.append(('육각 허브 → 플랜지, 토크 25 N·m', 'M3 6개 + 열압입 인서트 (r 18), 개당 전단 %.0f N' % f,
                 '인서트(Ø4×6) 지압 %.1f MPa, Ø20 등록 턱이 반경 하중' % (f / (4.0 * 6.0)), ok(f / 24.0, C_PETG)))
    for tag, F, Fl in (('검토', F_CHK, FL_CHK), ('설계', F_DES, FL_DES)):
        M = F * arm_joint + Fl * P.R_WHEEL
        fb = bolt_circle_max(M, 6, 18.0)
        rows.append(('육각 허브 → 플랜지 굽힘, %s: %.1f N·m' % (tag, M / 1000), 'M3 인서트 인발 %.0f N (약 %.0f)'
                     % (fb, INSERT_M3_PULL), 'M5 축 볼트는 바퀴만 육각 허브에 고정 — 이 접합부는 M3 6개가 버팀',
                     ok(fb, INSERT_M3_PULL)))
        if tag == '검토':
            verdict.append(('육각 허브 → 플랜지 인서트 인발(검토)', '%.0f N / 약 1000' % fb, ok(fb, INSERT_M3_PULL)))
    p_hex = T_CHK / (6 * 7.0) / (4.0 * P.HEX_L)
    rows.append(('17 mm 육각 ↔ RC 휠, 25 N·m', '6면 접촉, 깊이 %.0f mm' % P.HEX_L, '%.1f MPa' % p_hex, ok(p_hex, C_PETG)))
    rows.append(('AKE60 출력 베어링', '반경 600 N이 %.0f mm에 → %.1f N·m' % (d, F_CHK * d / 1000),
                 'AKE60-8 베어링 정격 비공개', '확인 필요'))
    table(('항목', '값', '응력·힘', '판정'), rows)
    tbl = []
    for W in (45.0, 60.0, 76.0):
        dd = d - P.TYRE_W / 2 + W / 2
        tbl.append('폭 %.0f mm: %.1f mm → 정지 %.2f N·m, 설계 %.1f N·m' % (W, dd, F_STATIC * dd / 1000, F_DES * dd / 1000))
    out.append('\n타이어 폭에 따른 출력 굽힘 팔(= 바퀴 판 %.0f mm − 포켓 + 간격 %.0f mm + 폭/2): ' % (P.WHEEL_PLATE_T, P.GAP_TYRE)
               + '; '.join(tbl) + '.  60 mm(1/8 트러기, 17 mm 육각 150 mm급에서 가장 좁은 흔한 규격)를 기본값으로 했다.')

    # ------------------------------------------------------------------ 4. torso
    h('4. 몸통 하중 경로')
    rows = []
    span = 2 * P.TUBE_X - P.TUBE_D
    flange = 38.0
    A1, A2 = flange * P.TRAY_T, 2.4 * 8.0
    y1, y2 = -P.TRAY_T / 2, -P.TRAY_T - 4.0
    yc = (A1 * y1 + A2 * y2) / (A1 + A2)
    I = flange * P.TRAY_T ** 3 / 12 + A1 * (y1 - yc) ** 2 + 2.4 * 8 ** 3 / 12 + A2 * (y2 - yc) ** 2
    c = max(abs(0 - yc), abs(-P.TRAY_T - 8 - yc))
    for tag, F in (('설계 120 N', 120.0), ('검토 250 N', 250.0)):
        w = F / 7 / span
        M = w * span ** 2 / 8
        dlt = 5 * w * span ** 4 / (384 * E_PETG * I)
        rows.append(('배터리 트레이 리브 7개(T 38×2.4 + 2.4×8), 봉 사이 %.0f mm, %s' % (span, tag),
                     'σ %.1f MPa' % (M * c / I), '처짐 %.2f mm (한계 0.5)' % dlt, ok(dlt if F < 200 else 0, 0.5)))
        if F < 200:
            verdict.append(('배터리 트레이 처짐(설계 120 N)', '%.2f mm / 0.5' % dlt, ok(dlt, 0.5)))
    It = math.pi / 64 * P.TUBE_D ** 4                                       # solid bars (10-29)
    Lt = 2 * (P.YW - P.HIP_PLATE_T)
    for tag, F in (('설계 120 N', 120.0), ('검토 250 N', 250.0)):
        w = F / 2 / P.BATTERY[1]
        M = w * P.BATTERY[1] * (Lt / 4 - P.BATTERY[1] / 8)
        dlt = 5 * (F / 2 / Lt) * Lt ** 4 / (384 * E_AL * It)
        rows.append(('척추 봉 Ø%.0f 통짜 6061 2개, 경간 %.0f(단순 지지), %s' % (P.TUBE_D, Lt, tag),
                     'σ %.0f MPa (항복 276)' % (M * P.TUBE_D / 2 / It), '처짐 %.2f mm' % dlt,
                     ok(dlt if F < 200 else 0, 0.5)))
        if F < 200:
            verdict.append(('척추 봉 처짐(설계 120 N)', '%.2f mm / 0.5' % dlt, ok(dlt, 0.5)))
    Ir = P.RAIL_S ** 4 / 12
    for tag, F in (('설계 250 N', 250.0), ('검토 500 N', 500.0)):
        M = F / 2 * Lt / 4
        dlt = (F / 2) * Lt ** 3 / (48 * E_AL * Ir)
        rows.append(('위 레일 %.0f×%.0f 통짜 6061 2개, 가운데 집중, %s' % (P.RAIL_S, P.RAIL_S, tag),
                     'σ %.0f MPa' % (M * P.RAIL_S / 2 / Ir), '처짐 %.2f mm (윗판이 더 받쳐 줌)' % dlt,
                     ok(M * P.RAIL_S / 2 / Ir, Y_AL / 2)))
        if F > 300:
            verdict.append(('위 레일 굽힘(검토 500 N)', '%.0f MPa / 276' % (M * P.RAIL_S / 2 / Ir),
                            ok(M * P.RAIL_S / 2 / Ir, Y_AL)))
    # 6063 instead of 6061-T6 for the round / square bar (only if 6061-T6 cannot be bought): KS minimum yield;
    # E is the same, so the deflections above do not change.  Rail checked also at the Ø4.4 bolt holes (net section).
    x_hole = Lt / 2 - 33.3                                                  # inner bolt-hole pair, from the rail end
    I_net = Ir - 4.4 * P.RAIL_S ** 3 / 12                                   # vertical Ø4.4 hole through the bar
    s_rail = max(500.0 / 2 * Lt / 4, 500.0 / 4 * x_hole * Ir / I_net) * P.RAIL_S / 2 / Ir
    w = 250.0 / 2 / P.BATTERY[1]
    s_tube = w * P.BATTERY[1] * (Lt / 4 - P.BATTERY[1] / 8) * P.TUBE_D / 2 / It
    for alloy, ya in Y_6063.items():
        rows.append(('6063-%s 대체: 위 레일(검토 500 N, 구멍 단면 포함) / 척추 봉(검토 250 N)' % alloy,
                     'σ %.0f / %.0f MPa' % (s_rail, s_tube), '항복 ≥ %.0f MPa (KS 최소)' % ya,
                     ok(max(s_rail, s_tube), ya)))
        verdict.append(('6063-%s 대체 시 레일·척추 봉(검토)' % alloy, '%.0f MPa / %.0f' % (max(s_rail, s_tube), ya),
                        ok(max(s_rail, s_tube), ya)))
    for tag, Mm in (('설계 15 N·m', 15000.0), ('검토 30 N·m', 30000.0)):
        f = Mm / (2 * 50.0)
        rows.append(('팔 모멘트, 볼트 4개 받침(팔 50 mm), %s' % tag, 'M4 인서트 인발 개당 %.0f N' % f,
                     'PETG M4 열압입 인서트 약 900 N; 받침은 레일 위 x = ±50 줄에', ok(f, 900.0)))
    verdict.append(('윗판 M4 인서트(검토 30 N·m)', '%.0f N / 약 900' % (30000.0 / 100.0), ok(300.0, 900.0)))
    rows.append(('전장 데크(0.5 kg, hip 판에 M3 4개)', '검토 하중 %.0f N' % (0.5 * 9.81 * 10),
                 'M3 전단 개당 %.0f N' % (0.5 * 9.81 * 10 / 4), '통과'))
    Wp = 110 * 5.0 ** 2 / 6
    Mh = Mx_hip(F_CHK, FL_CHK)
    rows.append(('hip 판 웹 5 mm 6061, 옆 모멘트(검토) %.0f N·m' % (Mh / 1000), '한 단면(폭 110)이 전부 받으면 W %.0f mm³'
                 % Wp, 'σ ~%.0f MPa(검토) / ~%.0f MPa(설계), 봉·레일 두 경로로 나뉘면 절반' % (Mh / Wp, Mh / 2 / Wp),
                 '설계 SF %.1f (FEA 권장)' % (Y_AL / (Mh / 2 / Wp))))
    verdict.append(('hip 판 옆 모멘트(설계)', '~%.0f MPa / 276' % (Mh / 2 / Wp), ok(Mh / 2 / Wp, Y_AL)))
    table(('항목', '하중·응력', '결과', '판정'), rows)

    # ------------------------------------------------------------------ 5. heat
    h('5. 모터 열과 PETG 70 °C')
    out.append('PETG와 맞닿는 모터 면: knee·wheel 고정링(Ø42 파일럿 포켓과 링 면), 드라이버 컵 립(Ø88 하우징 위).  '
               'hip 모터는 알루미늄 hip 판에 붙어 열이 빠진다.  구리 손실 P ≈ 0.75 × R(선간 0.577 Ω) × I², '
               'I = 토크 / (0.13 × 8).  정격 5 N·m에서 약 %.0f W.\n' % (0.75 * 0.577 * (5.0 / 1.04) ** 2))
    rows = []
    knee = {}
    for tag in ('robot', 'payload5'):
        p = os.path.join(SIM, tag + '.json')
        if not os.path.exists(p):
            continue
        sc = json.load(open(p, encoding='utf-8'))['scen']
        knee[tag] = {k: abs(v['steady_mean']['left_knee']) for k, v in sc.items() if k.startswith('stand_L0.')}
        for k in ('stand_L0.240', 'stand_L0.310'):
            t = knee[tag][k]
            i = t / 1.04
            rows.append(('로봇만' if tag == 'robot' else '짐 5 kg', k.replace('stand_L0.', 'L0 0.'), '%.2f N·m (%.0f %%)'
                         % (t, 100 * t / 5.0), '%.1f A' % i, '%.1f W' % (0.75 * 0.577 * i * i)))
    table(('경우', '자세', '무릎 정지 토크 (정격 대비)', '전류', '구리 손실'), rows)
    if len(knee) == 2:
        r, p5 = knee['robot'], knee['payload5']
        pts = sorted((float(k.split('L0.')[1]) / 1000.0, v) for k, v in p5.items())
        l_rated = next((a[0] + (a[1] - 5.0) / (a[1] - b[1]) * (b[0] - a[0]) for a, b in zip(pts, pts[1:])
                        if a[1] > 5.0 >= b[1]), None)
        out.append('\n로봇만일 때 무릎 정지 토크는 정격의 %.0f~%.0f %%로 열 문제가 없다.  짐 5 kg를 싣고 낮은 자세(L0 0.24~0.27 m)로 '
                   '오래 서 있으면 무릎이 정격을 넘어(%.0f~%.0f %%) 모터 외피가 PETG 연화 온도(약 70 °C)에 가까워질 수 있다 → '
                   '짐을 실을 때는 L0 ≥ %.3f m(정격 이하)로 서 있고, moteus 온도를 감시할 것.'
                   % (100 * min(r.values()) / 5.0, 100 * max(r.values()) / 5.0, 100 * p5['stand_L0.270'] / 5.0,
                      100 * p5['stand_L0.240'] / 5.0, l_rated))
    with open(os.path.join(HERE, 'strength.md'), 'w', encoding='utf-8') as fo:
        fo.write('\n'.join(out) + '\n')
    with open(os.path.join(HERE, 'strength_verdict.json'), 'w', encoding='utf-8') as fo:
        json.dump(verdict, fo, ensure_ascii=False, indent=1)
    print('\n'.join(out))
    print('\n=== verdict')
    for v in verdict:
        print(' | '.join(v))


if __name__ == '__main__':
    main()
