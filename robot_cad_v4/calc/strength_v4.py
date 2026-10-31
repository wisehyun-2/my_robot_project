"""v4 hand calculations: SLA resin thigh / shank (closed box + Warren truss ribs) and their resin joint ends.

    python calc/strength_v4.py        # -> calc/strength_v4.md

Loads, load cases and the section / plate formulas are those of robot_cad_v3/calc/strength.py (copied here so the
v4 folder stands on its own): contact 60 N static per wheel, design 300 N, check 600 N (SF 2), lateral 0.5 x, motor
torque 12.5 / 25 N m.  Stresses are compared at the check load with the resin strength (tensile 41 MPa = the low end of
the sheet value for axial / combined stress, flexural 69 MPa for plate and boss bending), deflections at the design
load.  Parts that v4 shares with v3 (hip plate, spine bars, rails, hub flange, rim, tyre) keep their v3 results
(robot_cad_v3/calc/strength.md sections 2-3).
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'fusion'))
import params as P            # noqa: E402  (pure Python dimensions, no Fusion import)

# ----------------------------------------------------------------------------- materials
E_SLA, NU_SLA = 2731.0, 0.35                       # ABS-like black SLA resin (user's sheet, 11-01)
S_SLA_T, S_SLA_F, HDT_SLA = 41.0, 69.0, 60.0       # tensile 41..58, flexural 69..76 MPa (low ends), HDT 0.45 MPa [C]
MAT = {   # E, G, strength (MPa), density (g/cm3)
    'SLA': (E_SLA, E_SLA / (2 * (1 + NU_SLA)), S_SLA_T, P.SLA_RHO),
    '6061-T6': (69000.0, 26000.0, 276.0, 2.70),
    'PETG': (2000.0, 740.0, 45.0, 1.27)}
E_SUS = 193000.0
AS_M3, AS_M25 = 5.03, 3.39
PROOF = {'8.8': 600.0, '12.9': 970.0}

F_STATIC = 60.0
F_DES, F_CHK = 5 * F_STATIC, 10 * F_STATIC
FL_DES, FL_CHK = P.LATERAL * F_DES, P.LATERAL * F_CHK
T_DES, T_CHK = P.MOTOR_PEAK * 1000.0, 2 * P.MOTOR_PEAK * 1000.0      # N mm
PHI = P.PHI
YTC = P.tyre_centre_y()
LINKS3 = (('thigh', (0.0, P.Z_HIP), (P.KNEE_X, P.KNEE_Z)), ('shank', (P.KNEE_X, P.KNEE_Z), (0.0, P.R_WHEEL)))
LEN = {n: math.hypot(b[0] - a[0], b[1] - a[1]) for n, a, b in LINKS3}
H = {'thigh': P.FEMUR_H, 'shank': P.TIBIA_H}
BASE = {'thigh': (P.Y_FEMUR_IN, P.Y_FEMUR_OUT), 'shank': (P.Y_TIBIA_IN, P.Y_TIBIA_OUT)}
WIDE = {'thigh': (P.FEMUR_WIDE_OUT, P.FEMUR_WIDE_END), 'shank': (P.TIBIA_WIDE_OUT, P.TIBIA_WIDE_END)}
# Fusion build 11-01 (one side, the part alone): volume cm3 -> SLA_RHO
LINK_VOL_V4 = (251.1, 220.6)
LINK_MASS_V3, LINK_MASS_V2 = (377.4, 394.3), (424.0, 381.0)
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


# ----------------------------------------------------------------------------- sections (v3 strength.py)
def box(yin, yout, h_, t_in, t_out, t_edge):
    """Closed box (plates t_in at yin, t_out at yout, edge walls t_edge at n = +-h/2): A, yc, Iy (lateral bending),
    In (in-plane), J (Bredt), Am, thinnest wall, outer fibre distances."""
    r = [(yin, yin + t_in, -h_ / 2, h_ / 2), (yout - t_out, yout, -h_ / 2, h_ / 2),
         (yin + t_in, yout - t_out, -h_ / 2, -h_ / 2 + t_edge), (yin + t_in, yout - t_out, h_ / 2 - t_edge, h_ / 2)]
    A = sum((b - a) * (d - c) for a, b, c, d in r)
    yc = sum((b - a) * (d - c) * (a + b) / 2 for a, b, c, d in r) / A
    Iy = sum((d - c) * (b - a) ** 3 / 12 + (b - a) * (d - c) * ((a + b) / 2 - yc) ** 2 for a, b, c, d in r)
    In = sum((b - a) * (d - c) ** 3 / 12 + (b - a) * (d - c) * ((c + d) / 2) ** 2 for a, b, c, d in r)
    wy, wn = (yout - yin) - (t_in + t_out) / 2, h_ - t_edge
    Am = wy * wn
    J = 4 * Am ** 2 / (wn / t_in + wn / t_out + 2 * wy / t_edge)
    return dict(A=A, yc=yc, Iy=Iy, In=In, J=J, Am=Am, t=min(t_in, t_out, t_edge), cy=max(yc - yin, yout - yc), cn=h_ / 2)


def with_rib(d, yin, yout, wall, rib):
    """A Warren rib crossed by the section (one per section, thickness rib / sin 45 across the cavity width) adds a web
    in the middle of the box: area and lateral bending only (it does not close a new torsion cell)."""
    tw, wy = rib * math.sqrt(2.0), yout - yin - 2 * wall
    d = dict(d)
    d['A'] += tw * wy
    d['Iy'] += tw * wy ** 3 / 12 + tw * wy * ((yin + yout) / 2 - d['yc']) ** 2
    return d


def make(wall=None, wide=True, rib=0.0, solid=False):
    """Section function per link: base width (v3) or widened outboard as v2 / v4 (taper over WIDE_TAPER)."""
    def f(n):
        yin, y0 = BASE[n]
        L, h_ = LEN[n], H[n]

        def sec(s):
            yout = y0
            if wide:
                wo, end = WIDE[n]
                se = L - end
                sf = se - P.WIDE_TAPER
                yout = wo if s <= sf else (wo + (y0 - wo) * (s - sf) / P.WIDE_TAPER if s <= se else y0)
            if solid:
                return box(yin, yout, h_, (yout - yin) / 2 - 0.01, (yout - yin) / 2 - 0.01, h_ / 2 - 0.01)
            d = box(yin, yout, h_, wall, wall, wall)
            return with_rib(d, yin, yout, wall, rib) if rib else d
        return sec
    return {n: f(n) for n in ('thigh', 'shank')}


def contact_moment(S, y_s, F, Fl, sgn):
    dx, dy, dz = -S[0], YTC - y_s, -S[1]
    fy, fz = sgn * Fl, F
    return dy * fz - dz * fy, -dx * fz, dx * fy


def legs3(secs, mat, n=200):
    """v3 strength.legs3: max von Mises at the check load, v2-metric link-end lateral deflection, in-plane deflection,
    lateral contact displacement (design load, bending + twist) and its twist share."""
    E, G, _S, _rho = MAT[mat]
    res, delta, twist = {}, {'+': 0.0, '-': 0.0}, {'+': 0.0, '-': 0.0}
    sig_face = 0.0
    for name, a, b in LINKS3:
        L = LEN[name]
        u = ((b[0] - a[0]) / L, (b[1] - a[1]) / L)
        nv = (-u[1], u[0])
        d_v2 = d_in = vm_max = 0.0
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
                    mx, _my, mz = contact_moment(Sm, sc['yc'], 0.0, 1.0, sg)
                    T, Mb = Mx * u[0] + Mz * u[1], Mx * nv[0] + Mz * nv[1]
                    t1, mb1 = mx * u[0] + mz * u[1], mx * nv[0] + mz * nv[1]
                    db, dt = Mb * mb1 / (E * sc['Iy']) * L / n, T * t1 / (G * sc['J']) * L / n
                    delta[k] += db + dt
                    twist[k] += dt
                Mb2 = (FL_DES * Sm[1] + F_DES * (sc['yc'] - YTC)) * math.cos(PHI)
                d_v2 += Mb2 * (L - sm) * L / n / (E * sc['Iy'])
                d_in += T_DES * (L - sm) * L / n / (E * sc['In'])
            for sg in (1, -1):
                Mx, _My, Mz = contact_moment(S, sec['yc'], F_CHK, FL_CHK, sg)
                T, Mb = Mx * u[0] + Mz * u[1], Mx * nv[0] + Mz * nv[1]
                s_out, s_in, s_ax = abs(Mb) * sec['cy'] / sec['Iy'], T_CHK * sec['cn'] / sec['In'], F_CHK / sec['A']
                tau = abs(T) / (2 * sec['Am'] * sec['t'])
                vm_max = max(vm_max, math.sqrt((s_out + s_in + s_ax) ** 2 + 3 * tau ** 2))
                sig_face = max(sig_face, s_out + s_ax)
        res[name] = dict(vm=vm_max, d_v2=d_v2, d_in=d_in)
    k = '+' if abs(delta['+']) >= abs(delta['-']) else '-'
    return res, abs(delta[k]), abs(twist[k]), sig_face


def beam_mass(secs, rho):
    return sum(sum(secs[n](LEN[n] * (i + 0.5) / 100)['A'] for i in range(100)) * LEN[n] / 100
               for n in ('thigh', 'shank')) * rho / 1000


def plate_buckling(t, b, k=4.0):
    """Compressive buckling stress of a plate strip of width b (edges simply supported, k = 4 for a long strip)."""
    return k * math.pi ** 2 * E_SLA / (12 * (1 - NU_SLA ** 2)) * (t / b) ** 2


def annular(b, a, t, M, E=E_SLA, nu=NU_SLA, outer='clamped'):
    """Max radial bending stress of an annular plate (inner edge r = b fixed to a rigid boss tilted by the moment M,
    outer edge r = a clamped or simply supported), n = 1 Fourier mode (v3 strength.annular)."""
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


def ring_peak(F, width, t, base):
    """Peak resin pressure under a SUS clamp ring next to one screw: ring = beam (thickness t, radial width) on an
    elastic resin foundation (thickness base), p = F beta / (2 width); also the spread length pi / beta."""
    beta = (E_SLA / base / (4 * E_SUS * t ** 3 / 12)) ** 0.25
    return F * beta / (2 * width), math.pi / beta


def main():
    out.append('# v4 강도 손계산 — SLA 레진 허벅지·정강이 (`python calc/strength_v4.py`)\n')
    out.append('하중은 v3와 같다(브리프: 로봇 7 kg + 짐 5 kg): 정지 %.0f N/바퀴, 설계 %.0f N, 검토 %.0f N(안전계수 2), 옆 %.0f / %.0f N, '
               '관절 토크 설계 12.5 / 검토 25 N·m.  응력은 검토 하중으로, 처짐은 설계 하중으로 본다.  계산식은 '
               '`robot_cad_v3/calc/strength.py`(1절 상자·접지점 3차원 모멘트, 1b절 원판)와 같다.\n'
               % (F_STATIC, F_DES, F_CHK, FL_DES, FL_CHK))
    out.append('재료(사용자 자료, 11-01): SLA 유사ABS 검정 레진 — 밀도 1.20, 인장 41~58 MPa, 인장 탄성 2731 MPa, 굽힘 69~76 MPa, '
               '연신 11 %%, HDT %.0f °C(0.45 MPa).  전단 탄성은 ν 0.35로 %.0f MPa.  **판정은 낮은 쪽 값**: 링크 보(굽힘 + 비틀림 '
               '+ 축력)는 인장 %.0f, 판·보스 굽힘은 굽힘강도 %.0f MPa와 비교.\n'
               % (HDT_SLA, MAT['SLA'][1], S_SLA_T, S_SLA_F))

    # ------------------------------------------------------------------ 1. link section
    h('1. 링크 단면 — 왜 넓히고, 왜 닫힌 상자 + 트러스인가')
    out.append('SLA는 속을 채워 출력하므로 꽉 찬 링크는 레진(=값)과 무게만 늘고, 옆 강성은 단면의 바깥 치수가 정한다(옆 굽힘 ∝ 폭³, '
               '비틀림 ∝ 둘러싼 넓이²).  그래서 v2 PETG처럼 바깥쪽으로 넓히고(허벅지 %.0f → %.0f, 정강이 %.0f → %.0f mm), 속은 비운 닫힌 '
               '상자에 대각선 리브(와렌 트러스)를 넣었다.  옆이 뚫린 평면 트러스는 비틀림이 거의 0이라 이 다리에 맞지 않는다(v3 1절: '
               '기운 링크에서 옆 하중의 상당 부분이 비틀림).  "보 질량"은 175 mm 보 부분만(관절 자리 제외), 리브는 단면마다 하나가 '
               '대각선으로 걸린다고 보고 옆 굽힘에만 넣었다.\n'
               % (P.Y_FEMUR_OUT - P.Y_FEMUR_IN, P.FEMUR_WIDE_OUT - P.Y_FEMUR_IN, P.Y_TIBIA_OUT - P.Y_TIBIA_IN,
                  P.TIBIA_WIDE_OUT - P.Y_TIBIA_IN))
    v3box = {n: (lambda n: (lambda s: box(BASE[n][0], BASE[n][1], H[n], 2.5, 3.0, 2.5)))(n) for n in ('thigh', 'shank')}
    cases = [('SLA, v3 폭(30 mm), 꽉 채움', make(wide=False, solid=True), 'SLA', None),
             ('SLA, 넓힌 외형, 꽉 채움', make(solid=True), 'SLA', None),
             ('SLA, 넓힌 상자 벽 2.5', make(2.5), 'SLA', 2.5),
             ('SLA, 넓힌 상자 벽 2.5 + 리브 2', make(2.5, rib=2.0), 'SLA', 2.5),
             ('SLA, 넓힌 상자 벽 3.0', make(3.0), 'SLA', 3.0),
             ('**SLA, 넓힌 상자 벽 %.1f + 리브 %.1f (v4)**' % (P.SLA_WALL, P.SLA_RIB), make(P.SLA_WALL, rib=P.SLA_RIB), 'SLA',
              P.SLA_WALL),
             ('SLA, 넓힌 상자 벽 4.0 + 리브 2', make(4.0, rib=2.0), 'SLA', 4.0),
             ('참고: v3 6061 상자 (30 mm, 벽 2.5 / 덮개 3)', v3box, '6061-T6', None)]
    rows, chosen = [], None
    for nm, secs, mat, wall in cases:
        res, d, tw, sf = legs3(secs, mat)
        vm = max(res['thigh']['vm'], res['shank']['vm'])
        lim = MAT[mat][2]
        ribbed = '리브' in nm
        sb = None if wall is None else plate_buckling(wall, max(H.values()), 8.0 if ribbed else 4.0)
        bk = '-' if sb is None else '%.0f / %.0f%s' % (sf, sb, ' (리브 칸)' if ribbed else '')
        fine = max(res['thigh']['d_v2'], res['shank']['d_v2']) <= 1.0 and vm <= lim and (sb is None or sf <= sb)
        verdict = '통과' if fine else '**초과**'
        rows.append((nm, '%.2f / %.2f' % (res['thigh']['d_v2'], res['shank']['d_v2']), '%.1f (%.1f)' % (d, tw),
                     '%.0f / %.0f' % (vm, lim), bk, '%.0f' % beam_mass(secs, MAT[mat][3]), verdict))
        if 'v4' in nm:
            chosen = (res, d, tw, sf, vm)
    table(('단면', '링크 끝 옆 처짐 허벅지 / 정강이 [mm] (한계 1)', '접지점 옆 변위 [mm] (비틀림)', '검토 응력 / 강도 [MPa]',
           '면판 압축 / 좌굴 [MPa] (폭 %.0f)' % max(H.values()), '보 질량 [g]', '판정'), rows)
    res, d, tw, sf, vm = chosen
    bw = max(H.values())
    b25, b25r = plate_buckling(2.5, bw), plate_buckling(2.5, bw, 8.0)
    b30, b30r = plate_buckling(P.SLA_WALL, bw), plate_buckling(P.SLA_WALL, bw, 8.0)
    out.append('\n- v3 폭(30 mm)으로는 레진을 꽉 채워도 링크 끝이 2.6 mm 휜다 — **넓히는 것이 먼저**다.  넓힌 외형은 꽉 채우면 '
               '처짐 0.4 mm지만 보 부분만 1.2 kg(링크 4개 약 2.8 kg)이라 값·무게가 맞지 않는다.')
    out.append('- **면판 좌굴**: 옆 굽힘에서 눌리는 면판(폭 최대 %.0f mm)은 리브가 없으면 긴 판으로 좌굴한다(k = 4) — 벽 2.5는 '
               '좌굴 %.0f MPa로 검토 압축보다 낮아 리브 없이는 안 된다.  리브가 면판을 삼각형 칸(변 약 45~55 mm)으로 나누면 좌굴 응력이 '
               '대략 2배(k ≈ 8: 벽 2.5 %.0f, 벽 %.1f %.0f MPa)가 된다.  v4 벽 %.1f는 리브 없이도 %.0f MPa로 버티지만 리브 칸 기준 여유가 '
               '약 %.1f배다 — **트러스 리브의 실제 몫은 얇은 벽이 좌굴하지 않게 붙잡는 것**이고, 리브가 더하는 옆 강성은 10 %% 안팎이다.'
               % (bw, b25, b25r, P.SLA_WALL, b30r, P.SLA_WALL, b30, b30r / sf))
    out.append('- v4(벽 %.1f + 리브 %.1f): 링크 끝 옆 처짐 %.2f / %.2f mm(v2 PETG 0.95 / 1.00), 접지점 옆 변위 %.1f mm(v2 PETG 12.8, v3 '
               '6061 1.3), 검토 응력 %.0f MPa < %.0f.  비틀림이 접지점 변위의 %.0f %%로 여전히 크다 — 더 줄이려면 면내 높이를 키워야 '
               '한다(+10 mm면 약 8.7 mm, 자세 스윕 다시 필요).'
               % (P.SLA_WALL, P.SLA_RIB, res['thigh']['d_v2'], res['shank']['d_v2'], d, vm, S_SLA_T, 100 * tw / d))
    m_v4 = [v * P.SLA_RHO for v in LINK_VOL_V4]
    out.append('- Fusion 실측(한쪽, 관절부 포함): 허벅지 %.1f cm³ = **%.0f g**, 정강이 %.1f cm³ = **%.0f g** — v3 6061 %.0f / %.0f g, v2 PETG '
               '%.0f / %.0f g.  레진은 링크 4개 %.0f cm³(SLA 견적은 보통 레진 부피·높이로 정해진다).'
               % (LINK_VOL_V4[0], m_v4[0], LINK_VOL_V4[1], m_v4[1], LINK_MASS_V3[0], LINK_MASS_V3[1], LINK_MASS_V2[0],
                  LINK_MASS_V2[1], 2 * sum(LINK_VOL_V4)))

    # ------------------------------------------------------------------ 2. joint ends
    h('2. 레진 관절부 (v2 출력 설계: 모터 판 9 mm, SUS 클램프 링, 접착 인서트)')
    rows = []
    Mop = {}
    for joint, S, ys in (('knee', (P.KNEE_X, P.KNEE_Z), P.Y_RING_KNEE), ('wheel', (0.0, P.R_WHEEL), P.Y_RING_WHEEL)):
        for tag, F, Fl in (('검토', F_CHK, FL_CHK), ('설계', F_DES, FL_DES)):
            Mx, _My, Mz = max((contact_moment(S, ys, F, Fl, sg) for sg in (1, -1)), key=lambda m: math.hypot(m[0], m[2]))
            M = math.hypot(Mx, Mz)
            Mop[(joint, tag)] = M
            b, a, t = P.STATOR_RECESS_D / 2, P.CAN_CLEAR_R, P.KNEE_PLATE_T
            sc = annular(b, a, t, M, outer='clamped')
            ss = annular(b, a, t, M, outer='simple')
            sg_ = annular(b, a, t - 2.0, M, outer='simple')
            rows.append(('%s 모터 판 %.0f mm (r %.1f → %.0f), %s: 관절 모멘트 %.0f N·m' % (joint, t, b, a, tag, M / 1000),
                         '바깥 고정 %.0f / 단순 지지 %.0f MPa (리드 홈 자리 %.0f mm: %.0f)' % (sc, ss, t - 2.0, sg_),
                         ok(sg_, S_SLA_F)))
    YF = (P.Y_FEMUR_IN + P.Y_FEMUR_OUT) / 2
    W_boss = math.pi * P.HUB_BOSS_D ** 3 / 32 - 6 * (math.pi / 4 * 3.4 ** 2) * 10 ** 2 / (P.HUB_BOSS_D / 2) / 2
    boss = {}
    for joint, fn in (('hip', lambda F, Fl: Fl * P.Z_HIP + F * (YF - YTC)),
                      ('knee', lambda F, Fl: Fl * P.KNEE_Z + F * (P.Y_FEMUR_OUT - YTC))):
        for tag, F, Fl in (('검토', F_CHK, FL_CHK), ('설계', F_DES, FL_DES)):
            M = fn(F, Fl)
            sb = M / W_boss
            fb = bolt_circle_max(M, 6, 10.0)
            pk, spread = ring_peak(fb, (P.RING_OUT[0] - P.RING_OUT[1]) / 2, P.RING_T, 6.0)
            boss[(joint, tag)] = (sb, fb, pk)
            rows.append(('%s 구동 보스 Ø%.0f(레진), 옆 하중 %s: Mx %.0f N·m' % (joint, P.HUB_BOSS_D, tag, M / 1000),
                         '굽힘 %.0f MPa; M3 12.9 인장 %.0f N → SUS 출력 링 밑 레진 최대 면압 %.0f MPa(퍼짐 %.0f mm)'
                         % (sb, fb, pk, spread), ok(max(sb, pk), S_SLA_F)))
    for joint, cls, plate in (('knee', '8.8', '허벅지 모터 판'), ('wheel', '8.8', '정강이 모터 판')):
        fn = {'knee': lambda F, Fl: Fl * P.KNEE_Z + F * (P.Y_FEMUR_OUT - YTC),
              'wheel': lambda F, Fl: Fl * P.R_WHEEL + F * P.wheel_offset()}[joint]
        M = fn(F_CHK, FL_CHK)
        ft = bolt_circle_max(M, 6, P.AKE_PCD_RING / 2)
        pk, spread = ring_peak(ft, (P.RING_STATOR[0] - P.RING_STATOR[1]) / 2, P.RING_T, P.KNEE_PLATE_T - P.RING_T - 1.7)
        rows.append(('%s 고정링 M2.5 %s, 옆 하중 검토: Mx %.0f N·m' % (joint, cls, M / 1000),
                     '인장 %.0f N (보증 %.0f) → SUS 고정 링 밑 레진 최대 면압 %.0f MPa(퍼짐 %.0f mm), %s'
                     % (ft, AS_M25 * PROOF[cls], pk, spread, plate), ok(max(ft / (AS_M25 * PROOF[cls]), pk / S_SLA_F), 1.0)))
    f = T_CHK / (9 * 10.0)
    floor = P.BEARING_FACE - P.RING_T
    rows.append(('구동 보스 ← 출력, 토크 25 N·m(검토)', 'M3 6 + 다웰 3 (r 10): 개당 %.0f N' % f,
                 '레진 구멍 지압 %.1f MPa (보스 %.1f mm)' % (f / (3.0 * floor), floor), ok(f / (3.0 * floor), S_SLA_T)))
    f2 = T_CHK / (6 * P.AKE_PCD_RING / 2)
    web = P.KNEE_PLATE_T - P.POCKET - P.RING_T - 1.7
    rows.append(('고정링 → 레진 모터 판, 토크 25 N·m(검토)', 'M2.5 6 (r 18.25): 개당 %.0f N' % f2,
                 '레진 구멍 지압 %.1f MPa (포켓·링 자리 사이 %.1f mm, 마찰 무시)' % (f2 / (2.5 * web), web),
                 ok(f2 / (2.5 * web), S_SLA_T)))
    table(('부위', '응력', '판정'), rows)
    sbh = boss[('hip', '검토')][0]
    out.append('\n- **hip 구동 보스 Ø28이 가장 빠듯하다**: 검토 %.0f MPa로 굽힘강도 %.0f와 거의 같다(설계 하중에서 %.0f MPa, 안전계수 약 2).  '
               'v2 PETG도 같은 자리에서 SF 2 검토를 넘었다(68 / 45).  Ø28은 hip 판 Ø30 구멍이 정해서 키울 수 없다 — 실물에서 걱정되면 '
               '보스만 6061 선반 부품(링크에 M3로 체결)으로 바꾸는 길이 있다(5절).' % (sbh, S_SLA_F, boss[('hip', '설계')][0]))
    out.append('- 모터 판 9 mm는 리드 홈 자리에서도 검토 하중 굽힘강도 안이다(무릎 %.0f MPa).  SUS 클램프 링이 나사 힘을 퍼뜨려 레진 면압을 '
               '낮춘다 — 링이 없으면 M2.5 머리 바로 밑이 100 MPa를 넘는다.' % annular(P.STATOR_RECESS_D / 2, P.CAN_CLEAR_R,
                                                                   P.KNEE_PLATE_T - 2.0, Mop[('knee', '검토')], outer='simple'))
    out.append('- 레진은 오래 눌리면 서서히 줄어든다(크리프).  **조임 토크를 금속보다 낮게**: 출력 M3 12.9는 1.0 N·m(약 1.7 kN), 고정링 '
               'M2.5는 0.5 N·m 정도로 두고 나사 고정제(중강도)를 쓴다.  처음 몇 주는 풀림을 다시 확인.')

    # ------------------------------------------------------------------ 3. heat
    h('3. 열 — HDT 60 °C')
    out.append('v3는 knee·wheel 모터가 6061 링크에 붙어 열이 금속으로 빠졌지만, v4는 v2처럼 수지에 붙는다.  AKE60-8 구리 손실은 '
               '정격 5 N·m에서 약 10 W(v3 strength.md 4절) — 계속 정격을 내면 고정자 링 면이 60 °C를 넘을 수 있고, 그 온도에서 레진 '
               '모터 판은 고정링 나사 체결력에 눌려 변형된다.  실물에서 서 있기·주행 뒤 **모터 링 면 온도를 재서 50 °C 밑**인지 '
               '확인할 것.  넘으면 (a) 무릎·바퀴 모터 자리만 6061 링(선반, 레진 판에 끼워 같은 M2.5로 체결)으로 바꾸거나 (b) 고온용 '
               '레진(HDT 80 °C 이상)으로 출력한다.')

    # ------------------------------------------------------------------ 4. print notes
    h('4. 출력 메모')
    out.append('- 한 덩어리 출력(덮개 없음).  속 빈 상자는 레진이 고이므로 배선 창 2곳(%.0f × %.0f) + 앞 모서리 Ø%.0f 배수 구멍 2곳으로 '
               '빼고 세척한다.  리브마다 배선 채널(%.0f × %.0f)과 앞 배수 채널(8 × 8)이 뚫려 있어 모든 칸이 이어진다(Fusion: 셸 1개 = '
               '갇힌 빈 공간 없음).' % (P.DUCT_WINDOW[0], P.DUCT_WINDOW[1], P.SLA_DRAIN_D, P.SLA_CHANNEL, P.SLA_CHANNEL))
    out.append('- Ø42.05 모터 포켓·Ø3 다웰 구멍은 출력 오차(±0.1~0.2)가 있으니 실물에 맞춰 사포·리머로 맞춘다.  M2.5 인서트는 열압입이 '
               '안 되므로(열경화 수지) 에폭시로 붙인다.')
    out.append('- 배선은 덮개가 없으니 출력 뒤 창으로 밀어 넣는다: 16 AWG 2가닥 + CAN을 커넥터 달기 전에 넣거나, XT30 플러그는 창 '
               '%.0f × %.0f로 통과한다.' % P.DUCT_WINDOW)

    # ------------------------------------------------------------------ 5. options
    h('5. 더 단단하게 / 더 싸게 하려면 (결정할 것)')
    out.append('1. 면내 높이 +10 mm(허벅지 60, 정강이 66): 접지점 변위 10.5 → 약 8.7 mm.  자세 스윕을 다시 봐야 한다.')
    out.append('2. hip 구동 보스만 6061 선반 부품: 보스 굽힘 여유 4배, 가공비는 링크 가공의 몇 분의 1.')
    out.append('3. 열이 문제면 무릎·바퀴 모터 자리 6061 링(3절).')
    with open(os.path.join(HERE, 'strength_v4.md'), 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(out) + '\n')
    print('\n'.join(out))



if __name__ == '__main__':
    main()
