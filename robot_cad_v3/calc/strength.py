"""Hand calculations of the v3 legs (6061 thigh / shank), joint interfaces, wheel hub and torso load paths
(brief items 3, 4 and 7).  Sections 3-4 (hub, torso) are the v2 parts unchanged.

    python calc/strength.py            # writes calc/strength.md (the README quotes its numbers)

v3 links (section 1): closed 6061 boxes (face plate + walls machined in one piece, screwed cover), checked with the
v2 criteria (stress at the check load, link-end deflection at the design load) and additionally with the lateral
displacement of the tyre contact under the design load, which counts both links' bending AND twist (virtual work,
full 3-D moment of the contact forces; v2 counted bending only, from Mx alone).  The same model gives the thickness
table (how thin a 6061 / SUS304 box or a flat plate may be) and the v2 PETG reference.  The joint parts of the links
(motor seat plates, spools, cover joint) follow in section 1b.

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

SIM = os.path.join(os.path.dirname(os.path.dirname(HERE)), 'wheeled_biped', 'results', 'ake_cad_v3')

E_PETG, S_PETG, C_PETG = 2000.0, 45.0, 50.0
E_AL, Y_AL = 69000.0, 276.0
E_SUS = 193000.0
MAT = {   # E, G, yield (MPa), density (g/cm3); PETG G = E / 2.7; SUS304 yield = KS / EN minimum (annealed sheet)
    '6061-T6': (E_AL, 26000.0, Y_AL, 2.70), 'SUS304': (E_SUS, 77000.0, 205.0, 7.93), 'PETG': (E_PETG, 740.0, S_PETG, 1.27)}
P_AL = 260.0                                       # 6061-T6 limiting surface pressure under heads / washers (VDI 2230
                                                   # AlMgSi1 order, conservative)
FAT_AL = 95.0                                      # 6061-T6 fatigue strength, 5e8 cycles, fully reversed
LINK_MASS_V3 = (377.4, 394.3)                      # g, one side, shell + cover (Fusion build 10-31: 322.7 + 54.7,
                                                   # 333.2 + 61.1, 10 cover screws)
Y_6063 = {'T5': 108.0, 'T6': 177.0}               # KS minimum yield (11 / 18 kgf/mm2), tube / rail fallback
E_CF = 120000.0                                    # pultruded carbon rod (option only)
AS_M3, AS_M25 = 5.03, 3.39                         # stress areas mm2
PROOF = {'8.8': 600.0, '12.9': 970.0}
INSERT_M3_PULL = 1000.0                            # heat-set M3 in PETG, order of magnitude (supplier tests)
LG_INSERT_D = 4.0                                  # legs.INSERT_M3 hole: the insert bears on this diameter

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
WALL, INFILL = 4.8, 0.20             # the v2 PETG links (12 walls, 20 % gyroid): kept for the comparison
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
    """Link of length L from the upper joint (s = 0, height z_root) to the lower one (z_tip); section y ya(s)..yb(s)
    (ya a number or a function of s).
    Out-of-plane moment Mx = lateral force x height + vertical force x (section centroid - tyre plane), split into
    bending (cos phi) and torsion (sin phi) of the inclined link; in-plane moment = the joint torque at both ends.
    Stresses at both ends and where the widened part ends; tip deflections integrated along the link."""
    ya_s = ya if callable(ya) else (lambda s: ya)
    sec = lambda s: props(section(ya_s(s), yb(s), hgt, yg, wall=wall, core=core))
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
            tau = Tq / (2 * (yb(s) - ya_s(s) - wall) * (hgt - SKIN) * SKIN)
            vm = math.sqrt((s_out + s_in + s_ax) ** 2 + 3 * tau ** 2)
            if tag.startswith('검토'):
                vm_max = max(vm_max, vm)
            rows.append((name, tag, where, '%.1f' % (yb(s) - ya_s(s)), '%.1f' % (Mx / 1000), '%.1f' % (Mb / 1000),
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


# ----------------------------------------------------------------------------- v3: metal boxes, contact deflection
def box(yin, yout, h, t_in, t_out, t_edge):
    """Closed box (thin plates t_in at yin, t_out at yout, edge walls t_edge at n = +-h/2): A, yc, Iy (lateral
    bending), In (in-plane), J (Bredt), Am, thinnest wall, outer fibre distances."""
    r = [(yin, yin + t_in, -h / 2, h / 2), (yout - t_out, yout, -h / 2, h / 2),
         (yin + t_in, yout - t_out, -h / 2, -h / 2 + t_edge), (yin + t_in, yout - t_out, h / 2 - t_edge, h / 2)]
    A = sum((b - a) * (d - c) for a, b, c, d in r)
    yc = sum((b - a) * (d - c) * (a + b) / 2 for a, b, c, d in r) / A
    Iy = sum((d - c) * (b - a) ** 3 / 12 + (b - a) * (d - c) * ((a + b) / 2 - yc) ** 2 for a, b, c, d in r)
    In = sum((b - a) * (d - c) ** 3 / 12 + (b - a) * (d - c) * ((c + d) / 2) ** 2 for a, b, c, d in r)
    wy, wn = (yout - yin) - (t_in + t_out) / 2, h - t_edge
    Am = wy * wn
    J = 4 * Am ** 2 / (wn / t_in + wn / t_out + 2 * wy / t_edge)
    return dict(A=A, yc=yc, Iy=Iy, In=In, J=J, Am=Am, t=min(t_in, t_out, t_edge), cy=max(yc - yin, yout - yc), cn=h / 2)


def flat_plate(y_face_out, t, h, outward=True):
    """Solid flat plate in the leg plane (no lateral depth), for the comparison."""
    yc = y_face_out - t / 2 if outward else y_face_out + t / 2
    a, b = max(t, h), min(t, h)
    J = a * b ** 3 * (1 / 3 - 0.21 * b / a)
    return dict(A=t * h, yc=yc, Iy=h * t ** 3 / 12, In=t * h ** 3 / 12, J=J, Am=0.0, t=t, cy=t / 2, cn=h / 2,
                tau_open=True)


def petg_v2(yin, yout, h):
    """v2 printed section as built (12 walls 4.8 on the side faces, 1.6 skins, 10 % core) for the comparison."""
    w, s, core = WALL, SKIN, CORE
    r = [(yin, yin + w, -h / 2, h / 2, 1), (yout - w, yout, -h / 2, h / 2, 1),
         (yin + w, yout - w, -h / 2, -h / 2 + s, 1), (yin + w, yout - w, h / 2 - s, h / 2, 1),
         (yin + w, yout - w, -h / 2 + s, h / 2 - s, core)]
    A = sum((b - a) * (d - c) * f for a, b, c, d, f in r)
    yc = sum((b - a) * (d - c) * f * (a + b) / 2 for a, b, c, d, f in r) / A
    Iy = sum(f * ((d - c) * (b - a) ** 3 / 12 + (b - a) * (d - c) * ((a + b) / 2 - yc) ** 2) for a, b, c, d, f in r)
    In = sum(f * ((b - a) * (d - c) ** 3 / 12 + (b - a) * (d - c) * ((c + d) / 2) ** 2) for a, b, c, d, f in r)
    wy, wn = (yout - yin) - w, h - s
    Am = wy * wn
    J = 4 * Am ** 2 / (2 * wn / w + 2 * wy / s)
    bc, hc = yout - yin - 2 * w, h - 2 * s
    a, b = max(bc, hc), min(bc, hc)
    J += core * a * b ** 3 * (1 / 3 - 0.21 * b / a)
    return dict(A=A, yc=yc, Iy=Iy, In=In, J=J, Am=Am, t=s, cy=max(yc - yin, yout - yc), cn=h / 2)


LINKS3 = (('thigh', (0.0, P.Z_HIP), (P.KNEE_X, P.KNEE_Z)), ('shank', (P.KNEE_X, P.KNEE_Z), (0.0, P.R_WHEEL)))


def contact_moment(S, y_s, F, Fl, sgn):
    """Moment (x, y, z) about the section point S = (x, z) at lateral y_s of the contact forces: vertical F and
    lateral sgn * Fl at the contact (0, YTC, 0)."""
    dx, dy, dz = -S[0], YTC - y_s, -S[1]
    fy, fz = sgn * Fl, F
    return dy * fz - dz * fy, -dx * fz, dx * fy


def legs3(secs, mat, n=200):
    """secs[name](s) -> section.  Returns per link: stress rows at three stations, max von Mises (check), v2-metric
    link-end lateral deflection, in-plane deflection; and the lateral contact displacement (design load, both links,
    bending + twist, worse lateral direction) with its twist share."""
    E, G, Sy, _rho = MAT[mat]
    res, delta, twist = {}, {'+': 0.0, '-': 0.0}, {'+': 0.0, '-': 0.0}
    for name, a, b in LINKS3:
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        u = ((b[0] - a[0]) / L, (b[1] - a[1]) / L)
        nv = (-u[1], u[0])
        d_v2 = d_in = 0.0
        vm_max, rows = 0.0, []
        stations = (0.0, L / 2, L)
        for i in range(n + 1):
            s = L * i / n
            S = (a[0] + u[0] * s, a[1] + u[1] * s)
            sec = secs[name](s)
            if i < n:                                   # integrate on the midpoints
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
                Mb2 = (FL_DES * Sm[1] + F_DES * (sc['yc'] - YTC)) * math.cos(PHI)          # v2 metric
                d_v2 += Mb2 * (L - sm) * L / n / (E * sc['Iy'])
                d_in += T_DES * (L - sm) * L / n / (E * sc['In'])
            for tag, F, Fl, Tm in (('검토(SF 2)', F_CHK, FL_CHK, T_CHK), ('설계', F_DES, FL_DES, T_DES)):
                worst = None
                for sg in (1, -1):
                    Mx, _My, Mz = contact_moment(S, sec['yc'], F, Fl, sg)
                    T, Mb = Mx * u[0] + Mz * u[1], Mx * nv[0] + Mz * nv[1]
                    s_out, s_in, s_ax = abs(Mb) * sec['cy'] / sec['Iy'], Tm * sec['cn'] / sec['In'], F / sec['A']
                    tau = (abs(T) * sec['t'] / sec['J'] if sec.get('tau_open') else abs(T) / (2 * sec['Am'] * sec['t']))
                    vm = math.sqrt((s_out + s_in + s_ax) ** 2 + 3 * tau ** 2)
                    if worst is None or vm > worst[-1]:
                        worst = (Mb, T, s_out, s_in, tau, vm)
                if tag.startswith('검토'):
                    vm_max = max(vm_max, worst[-1])
                if any(abs(s - st) < L / n / 2 for st in stations):
                    where = '위 끝' if s < 1 else ('가운데' if s < L - 1 else '아래 끝')
                    rows.append((name, tag, where, '%.1f' % (abs(worst[0]) / 1000), '%.1f' % (abs(worst[1]) / 1000),
                                 '%.1f' % (Tm / 1000), '%.1f' % worst[2], '%.1f' % worst[3], '%.1f' % worst[4],
                                 '**%.1f**' % worst[5]))
        res[name] = dict(rows=rows, vm=vm_max, d_v2=d_v2, d_in=d_in, Sy=Sy)
    k = '+' if abs(delta['+']) >= abs(delta['-']) else '-'
    return res, abs(delta[k]), abs(twist[k])


def annular(b, a, t, M, E=E_AL, nu=0.33, outer='clamped'):
    """Max radial bending stress of an annular plate (inner edge r = b fixed to a rigid boss tilted by the moment M,
    outer edge r = a clamped or simply supported), n = 1 Fourier mode of w = (C1 r + C2 r^3 + C3 / r + C4 r ln r)
    cos(theta); the boss moment = pi b (Mr - b V) with the Kirchhoff shear V (checked: the same at every radius)."""
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
    V = lambda r, h=1e-6: -D * (lap(r + h) - lap(r - h)) / (2 * h) - Mrt(r) / r
    scale = M / (math.pi * b * (Mr(b) - b * V(b)))
    return max(abs(6 * Mr(b + (a - b) * i / 100) * scale / t ** 2) for i in range(101))


def _solve4(A, y):
    """Gauss elimination with partial pivoting (4 x 4, no numpy in calc/)."""
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


def main():
    out.append('# v3 강도 손계산 (`python calc/strength.py`)\n')
    out.append('하중(브리프 3번, 로봇 7 kg + 짐 5 kg = 12 kg): 정지 %.0f N/바퀴, 설계 %.0f N, 검토 %.0f N(안전계수 2), '
               '옆 하중 %.0f / %.0f N(접지점), 모터 토크 12.5 N·m → 검토 25 N·m.  기준 자세 L0 = %.0f mm, 허벅지 수직에서 '
               '%.1f°, 무릎은 hip보다 %.1f mm 앞, 타이어 중심면 y = %.1f mm(왼쪽).  응력은 검토 하중(SF 2 포함)으로 '
               '재료 강도와 비교하고, 처짐은 설계 하중으로 본다.'
               % (F_STATIC, F_DES, F_CHK, FL_DES, FL_CHK, P.L0, math.degrees(PHI), P.KNEE_X, YTC))
    out.append('\n재료: PETG(층 방향) E 2.0 GPa, 인장 45 MPa, 압축 50 MPa / 6061-T6 E 69 GPa, 항복 276 MPa / '
               '볼트 8.8 보증응력 600 MPa, 12.9 보증응력 970 MPa.')

    # ------------------------------------------------------------------ 1. links (v3: 6061 closed boxes)
    k_lock = F_DES * P.KNEE_X
    W_, F_, C_ = P.LINK_WALL, P.LINK_FLOOR, P.COVER_T
    yc_f = P.Y_FEMUR_IN + P.FEMUR_IN_RELIEF[0]                  # thigh cover outer face (1.5 behind Y_FEMUR_IN)
    v3 = {'thigh': lambda s: box(yc_f, P.Y_FEMUR_OUT, P.FEMUR_H, C_, F_, W_),
          'shank': lambda s: box(P.Y_TIBIA_IN, P.Y_TIBIA_OUT, P.TIBIA_H, F_, C_, W_)}
    r3, d3, tw3 = legs3(v3, '6061-T6')
    sf, st = P.L1 - 55.0, P.L2 - 50.0                           # the v2 widened links (params of git 987a914)
    v2 = {'thigh': lambda s: petg_v2(P.Y_FEMUR_IN, ramp(s, sf - 15, sf, 250.0, P.Y_FEMUR_OUT), P.FEMUR_H),
          'shank': lambda s: petg_v2(P.Y_TIBIA_IN, ramp(s, st - 15, st, 260.5, P.Y_TIBIA_OUT), P.TIBIA_H)}
    r2, d2, tw2 = legs3(v2, 'PETG')
    h('1. 허벅지·정강이 링크 (v3: 6061-T6 닫힌 상자)')
    out.append('v3 링크는 6061-T6 판을 양면 가공한 셸(바깥 판 %.1f mm + 둘레 벽 %.1f mm, 관절 자리·구동 보스 한 덩어리)에 '
               '%.1f mm 6061 덮개를 M3 접시 볼트(10.9) 10개로 덮은 **닫힌 상자**다.  폭(로봇 y)은 모터 길이가 정하는 30 mm(v2는 PETG 옆 '
               '강성 때문에 허벅지 76 / 정강이 55 mm까지 넓혔다), 면내 높이 허벅지 %.0f / 정강이 %.0f mm, 허벅지 덮개는 셸 볼트 '
               '머리를 피해 v2 안쪽 면보다 %.1f mm 들어가 있다(상자 깊이 %.1f mm).  면외 굽힘·비틀림은 접지점의 수직 하중과 옆 '
               '하중이 단면에 만드는 3차원 모멘트를 링크 축(비틀림)과 그 수직(굽힘)으로 나눠 계산했다(v2는 Mx만 cos/sin으로 '
               '나눔 — 무릎 쪽 비틀림을 낮게 봄).  면내 굽힘은 v2와 같이 양 끝 관절 토크(설계 12.5, 검토 25 N·m)로 제한된다 '
               '(무릎을 잠그면 설계 하중에서 %.1f N·m가 필요하지만 AKE60-8은 12.5 N·m에서 밀린다).\n'
               % (F_, W_, C_, P.FEMUR_H, P.TIBIA_H, P.FEMUR_IN_RELIEF[0], P.Y_FEMUR_OUT - yc_f, k_lock / 1000))
    table(('링크', '하중', '위치', '면외 M [N·m]', 'T [N·m]', '면내 M [N·m]', 'σ 면외 [MPa]', 'σ 면내 [MPa]', 'τ [MPa]',
           'von Mises [MPa]'), [row_ for k in ('thigh', 'shank') for row_ in r3[k]['rows']])
    out.append('\n6061-T6 항복 276 MPa 기준 검토 하중 최대: 허벅지 %.0f, 정강이 %.0f MPa(안전계수 2를 넣고도 항복의 %.0f %%).  '
               '설계 하중에서는 절반이라 피로 한도(약 %.0f MPa, 5억 회)보다 낮다.'
               % (r3['thigh']['vm'], r3['shank']['vm'], 100 * max(r3['thigh']['vm'], r3['shank']['vm']) / Y_AL, FAT_AL))
    for k, nm in (('thigh', '허벅지'), ('shank', '정강이')):
        verdict.append(('%s 6061 상자 응력(검토 600 N + 옆 300 N + 25 N·m)' % nm, '%.0f MPa / 276' % r3[k]['vm'],
                        ok(r3[k]['vm'], Y_AL)))
    out.append('\n처짐(설계 하중 300 N, 옆 150 N, 관절 토크 12.5 N·m).  "링크 끝 옆 처짐"은 v2 기준 그대로(각 링크를 위 관절에서 '
               '외팔보로, 굽힘만), "접지점 옆 변위"는 두 링크의 굽힘과 비틀림을 모두 넣어 바퀴 접지점이 옆으로 얼마나 밀리는지'
               '(가상일, 모터·베어링은 강체):\n')
    table(('', '링크 끝 옆 처짐 허벅지 / 정강이 [mm] (한계 1)', '링크 끝 면내 처짐 [mm] (한계 1)',
           '접지점 옆 변위 [mm] (그중 비틀림)', '링크 질량 한쪽 [g]'),
          [('v2 PETG(넓힌 링크, 출력 설정 반영)', '%.2f / %.2f' % (r2['thigh']['d_v2'], r2['shank']['d_v2']),
            '%.2f / %.2f' % (r2['thigh']['d_in'], r2['shank']['d_in']), '%.1f (%.1f)' % (d2, tw2), '424 + 381 (10-30 추정식)'),
           ('**v3 6061 상자**', '**%.2f / %.2f**' % (r3['thigh']['d_v2'], r3['shank']['d_v2']),
            '%.2f / %.2f' % (r3['thigh']['d_in'], r3['shank']['d_in']), '**%.1f** (%.1f)' % (d3, tw3),
            '%.0f + %.0f (Fusion, 덮개 포함)' % LINK_MASS_V3)])
    for k, nm in (('thigh', '허벅지'), ('shank', '정강이')):
        verdict.append(('%s 옆 처짐(설계, 옆 150 N)' % nm, '%.2f mm / 1' % r3[k]['d_v2'], ok(r3[k]['d_v2'], 1.0)))
        verdict.append(('%s 면내 처짐(설계)' % nm, '%.2f mm / 1' % r3[k]['d_in'], ok(r3[k]['d_in'], 1.0)))
    verdict.append(('접지점 옆 변위(설계, 두 링크 굽힘 + 비틀림)', '%.1f mm (v2 %.1f)' % (d3, d2), '참고'))
    out.append('\nv2 기준(링크 끝, 굽힘만)으로는 v2도 약 1 mm였지만, 비틀림까지 넣으면 v2 PETG는 접지점이 %.0f mm 밀렸다(그중 비틀림 '
               '%.0f mm) — 기운 링크에서 옆 하중의 상당 부분이 비틀림이고 PETG 전단 탄성계수가 0.74 GPa뿐이어서다.  6061 상자는 '
               '%.1f mm로 %.0f배 단단하다.' % (d2, tw2, d3, d2 / d3))

    # thickness table: how thin can a metal link be (same envelope, same loads)
    h('1a. 두께를 어디까지 줄일 수 있나 (같은 30 mm 폭, 같은 하중)')
    out.append('벽·판 두께 t를 모두 같게 둔 상자, 그리고 상자 대신 링크 평면에 놓인 통판(로봇 y 방향 두께 t) 비교.  '
               '판정: 링크 끝 옆 처짐 ≤ 1 mm(v2 기준)와 검토 응력 ≤ 항복.  질량은 보 부분만(175 mm × 단면, 허벅지 + 정강이 '
               '한쪽, 관절 자리 제외 — 관절 자리는 두께와 거의 무관하게 한쪽 약 0.4 kg).\n')
    rows = []
    for mat, ts in (('6061-T6', (1.0, 1.5, 2.0, 2.5, 3.0)), ('SUS304', (0.5, 0.8, 1.0, 1.2, 1.5))):
        E, G, Sy, rho = MAT[mat]
        for t in ts:
            sec = {'thigh': lambda s, t=t: box(yc_f, P.Y_FEMUR_OUT, P.FEMUR_H, t, t, t),
                   'shank': lambda s, t=t: box(P.Y_TIBIA_IN, P.Y_TIBIA_OUT, P.TIBIA_H, t, t, t)}
            r, d, tw = legs3(sec, mat, n=100)
            m = (sec['thigh'](0)['A'] + sec['shank'](0)['A']) * 175.0 * rho * 1e-3
            vm = max(r['thigh']['vm'], r['shank']['vm'])
            dv = max(r['thigh']['d_v2'], r['shank']['d_v2'])
            rows.append(('%s 상자 t %.1f' % (mat, t), '%.2f / %.2f' % (r['thigh']['d_v2'], r['shank']['d_v2']),
                         '%.1f' % d, '%.0f / %.0f' % (vm, Sy), '%.0f' % m, ok(max(dv, vm / Sy), 1.0)))
    for mat, ts in (('6061-T6', (10.0, 15.0, 18.0)), ('SUS304', (8.0, 10.0, 12.0))):
        E, G, Sy, rho = MAT[mat]
        for t in ts:
            sec = {'thigh': lambda s, t=t: flat_plate(P.Y_FEMUR_OUT, t, P.FEMUR_H),
                   'shank': lambda s, t=t: flat_plate(P.Y_TIBIA_IN, t, P.TIBIA_H, outward=False)}
            r, d, tw = legs3(sec, mat, n=100)
            m = (sec['thigh'](0)['A'] + sec['shank'](0)['A']) * 175.0 * rho * 1e-3
            vm = max(r['thigh']['vm'], r['shank']['vm'])
            dv = max(r['thigh']['d_v2'], r['shank']['d_v2'])
            rows.append(('%s 통판 %.0f mm' % (mat, t), '%.2f / %.2f' % (r['thigh']['d_v2'], r['shank']['d_v2']),
                         '%.1f' % d, '%.0f / %.0f' % (vm, Sy), '%.0f' % m, ok(max(dv, vm / Sy), 1.0)))
    table(('단면', '링크 끝 옆 처짐 허벅지 / 정강이 [mm]', '접지점 옆 변위 [mm]', '검토 응력 / 항복 [MPa]',
           '보 질량 [g]', '판정'), rows)
    out.append('\n- 닫힌 상자는 6061 1 mm, SUS304 0.5 mm로도 계산상 통과한다(접지점 변위도 v2의 1/4 이하) — **두께를 정하는 것은 보가 '
               '아니라 가공과 관절 자리**다.  6061 셸은 27 mm 깊이 포켓을 3축으로 깎으므로 벽 2.0~2.5 mm가 현실적인 하한'
               '(떨림·휨), 덮개는 M3 접시 머리(1.7 mm) 때문에 3 mm, 모터 자리는 아래 1b(판 굽힘)로 정해진다.\n'
               '- SUS304는 같은 강성이면 질량이 6061과 거의 같다(E/ρ가 비슷).  다만 0.5~1.0 mm 판은 깎을 수 없어 판금(절곡·용접) '
               '상자가 되고, 모터 자리(Ø42 포켓·M2.5)는 따로 붙여야 한다.  항복(205 MPa)이 6061보다 낮고 무거워(7.93 g/cm³) '
               '관절 자리까지 SUS로 하면 약 2.9배 무겁다.\n'
               '- 상자 대신 **통판**(링크 평면에 판 하나)은 옆 굽힘이 두께의 세제곱이라 6061 15~18 mm, SUS304 10~12 mm가 필요하고 '
               '비틀림에 약해 접지점 변위가 크다 — 판 한 장으로 얇게 만드는 길은 없다.')

    # ------------------------------------------------------------------ 1b. joint parts of the 6061 links
    h('1b. 6061 링크의 관절 자리 (모터 자리 판, 스풀, 덮개 체결)')
    rows = []
    a_seat = P.HOUSING_R - P.HOUSING_WALL
    t_seat = P.Y_FEMUR_OUT - P.Y_RING_KNEE + P.SEAT_POCKET
    t_groove = t_seat - 2.0
    Mop = {}
    for joint, S, ys in (('knee', (P.KNEE_X, P.KNEE_Z), P.Y_RING_KNEE), ('wheel', (0.0, P.R_WHEEL), P.Y_RING_WHEEL)):
        for tag, F, Fl in (('검토', F_CHK, FL_CHK), ('설계', F_DES, FL_DES)):
            Mx, _My, Mz = max((contact_moment(S, ys, F, Fl, sg) for sg in (1, -1)), key=lambda m: math.hypot(m[0], m[2]))
            M = math.hypot(Mx, Mz)
            Mop[(joint, tag)] = M
            sc = annular(21.0, a_seat, t_seat, M, outer='clamped')
            ss = annular(21.0, a_seat, t_seat, M, outer='simple')
            sg_ = annular(21.0, a_seat, t_groove, M, outer='simple')
            rows.append(('%s 모터 자리 판 %.0f mm (r 21 → %.1f), %s: 관절 모멘트 %.0f N·m' % (joint, t_seat, a_seat, tag, M / 1000),
                         '바깥 고정 %.0f / 단순 지지 %.0f MPa (리드 홈 자리 %.0f mm: %.0f)' % (sc, ss, t_groove, sg_),
                         ok(sg_, Y_AL)))
            if tag == '검토':
                verdict.append(('%s 모터 자리 판 굽힘(검토, 리드 홈 자리, 단순 지지)' % joint, '%.0f MPa / 276' % sg_,
                                ok(sg_, Y_AL)))
    for t_try in (4.0, 5.0):
        rows.append(('참고: 모터 자리 %.0f mm였다면(knee, 검토, 단순 지지)' % t_try,
                     '%.0f MPa' % annular(21.0, a_seat, t_try, Mop[('knee', '검토')], outer='simple'), '참고'))
    Wsp = math.pi * (P.SPOOL_R ** 4 - 14.0 ** 4) / (4 * P.SPOOL_R)
    Mh = max(math.hypot(m[0], m[2]) for m in (contact_moment((0.0, P.Z_HIP), (P.Y_FEMUR_IN + P.Y_FEMUR_OUT) / 2, F_CHK,
                                                             FL_CHK, sg) for sg in (1, -1)))
    Mk = Mop[('knee', '검토')]
    for joint, M in (('hip(허벅지)', Mh), ('knee(정강이)', Mk)):
        d_c = (P.Y_FEMUR_OUT - F_ / 2) - (yc_f + C_ / 2)
        Fc = M / d_c
        rows.append(('%s 스풀 r %.0f / Ø28 구멍, 검토 %.0f N·m' % (joint, P.SPOOL_R, M / 1000),
                     '굽힘 %.0f MPa; 짝힘 %.0f N이 덮개(%.0f mm)·바깥 판으로: 덮개 구멍 지압 %.0f MPa, 덮개 인장 %.0f MPa'
                     % (M / Wsp, Fc, C_, Fc / (C_ * 2 * P.SPOOL_R), Fc / (C_ * P.FEMUR_H)),
                     ok(max(M / Wsp, Fc / (C_ * 2 * P.SPOOL_R)), Y_AL)))
        verdict.append(('%s 스풀·덮개 지압(검토)' % joint, '%.0f / %.0f MPa / 276' % (M / Wsp, Fc / (C_ * 2 * P.SPOOL_R)),
                        ok(max(M / Wsp, Fc / (C_ * 2 * P.SPOOL_R)), Y_AL)))
    # cover screws: shear flow of the box at the cover joint (torsion Bredt + lateral shear V Q / I) times the largest
    # screw spacing along an edge (params.COVER_S; the first gap from the head screws at -(head r - 4) cos 60)
    head_s = -(P.LINK_HEAD_R - 4.0) * math.cos(math.radians(60))
    shear = 0.58 * 940.0 * math.pi / 4 * 2.387 ** 2   # ISO 10642 10.9, shear yield on the thread minor diameter
    for name, link_, a_, b_ in (('허벅지', 'femur', (0.0, P.Z_HIP), (P.KNEE_X, P.KNEE_Z)),
                                ('정강이', 'tibia', (P.KNEE_X, P.KNEE_Z), (0.0, P.R_WHEEL))):
        sec = v3['thigh' if link_ == 'femur' else 'shank'](0)
        L_ = math.hypot(b_[0] - a_[0], b_[1] - a_[1])
        lever = abs(a_[0] * (b_[1] - a_[1]) - a_[1] * (b_[0] - a_[0])) / L_    # contact (0, 0) to the link axis
        T_chk = FL_CHK * lever + F_CHK * abs(sec['yc'] - YTC)
        q_t = T_chk / (2 * sec['Am'])
        y_cov = (yc_f + C_ / 2) if link_ == 'femur' else (P.Y_TIBIA_OUT - C_ / 2)
        q_v = FL_CHK * (P.FEMUR_H if link_ == 'femur' else P.TIBIA_H) * C_ * abs(sec['yc'] - y_cov) / sec['Iy']
        st_ = (head_s,) + tuple(P.COVER_S[link_])
        pitch = max(b2 - a2 for a2, b2 in zip(st_, st_[1:]))
        fs = (q_t + q_v) * pitch
        rows.append(('%s 덮개 M3 접시 볼트(10.9), 간격 ≤ %.0f mm, 검토' % (name, pitch),
                     '비틀림 %.0f N·m(팔 %.0f mm) → 전단 흐름 %.1f + 옆 전단 %.1f N/mm → 볼트 하나 %.0f N (골지름 전단 한계 '
                     '약 %.0f N), 덮개 지압 %.0f MPa' % (T_chk / 1000, lever, q_t, q_v, fs, shear, fs / (3.0 * C_)),
                     ok(max(fs / shear, fs / (3.0 * C_) / Y_AL), 1.0)))
        verdict.append(('%s 덮개 볼트 전단(검토)' % name, '%.0f N / %.0f' % (fs, shear), ok(fs, shear)))
    f_m3 = 0.7 * PROOF['12.9'] * AS_M3
    rows.append(('출력 M3 12.9 와셔(DIN 125) → 6061 보스 바닥', '조임 %.1f kN(보증 70 %%) + 검토 외력' % (f_m3 / 1000),
                 '와셔 면압 %.0f MPa (허용 약 %.0f)' % (4100.0 / 30.4, P_AL), ok(4100.0 / 30.4, P_AL)))
    a_low = math.pi / 4 * (4.5 ** 2 - 2.8 ** 2)
    rows.append(('고정링 M2.5 8.8 저두 머리 → 6061 자리(링 없음)', '검토 볼트 힘 ≤ %.0f N(2절)' % (PROOF['8.8'] * AS_M25),
                 '머리 면압 ≤ %.0f MPa (허용 약 %.0f)' % (PROOF['8.8'] * AS_M25 / a_low, P_AL),
                 ok(PROOF['8.8'] * AS_M25 / a_low, P_AL)))
    verdict.append(('고정링·출력 나사 머리 면압(6061, 링 없음)', '≤ %.0f MPa / %.0f' % (PROOF['8.8'] * AS_M25 / a_low, P_AL),
                    ok(PROOF['8.8'] * AS_M25 / a_low, P_AL)))
    table(('부위', '응력·힘', '판정'), rows)
    out.append('\n- 모터 자리 판은 모터 파일럿 끝(r 21)이 기울려는 모멘트를 하우징 벽(r %.1f)까지 판 굽힘으로 넘긴다(n = 1 원판 해, '
               '바깥 고정과 단순 지지 사이가 실제).  **8 mm는 모터 위치(v2와 같은 링 면)와 가공 한 면에서 정해진 값**이고 강도로는 '
               '4~5 mm면 된다 — 줄이려면 바깥 면에 Ø83 × 3 포켓을 더 깎아야 해(가공비) 두지 않았다(관절 4곳 합계 약 −0.18 kg).\n'
               '- 스풀(구동 보스 관)이 관절 모멘트를 바깥 판과 덮개 사이의 짝힘으로 상자에 넘긴다: 덮개 구멍을 스풀에 Ø36 H8/g6로 '
               '끼워 지압으로 받고, 덮개 볼트는 상자의 전단 흐름만 받는다.\n'
               '- v2의 SUS304 클램프 링(8개)은 맨 PETG 위 나사 머리 지압 때문이었다 — 6061에서는 와셔·저두 머리 면압이 허용치 안이라 뺐다.'
               % a_seat)

    # ------------------------------------------------------------------ 2. joints
    h('2. 관절 체결부 (AKE60-8: 출력 Ø27 6×M3 PCD 20 + Ø3 다웰 3개, 고정링 6×M2.5 PCD 36.5)')
    out.append('v3: hip·knee 구동 보스와 knee·wheel 모터 자리가 모두 6061 링크의 일부라 v2의 SUS304 클램프 링(PETG 지압용)을 '
               '뺐다.  출력 M3 12.9는 DIN 125 와셔로 보스 바닥(출력면에서 %.1f mm)에, 고정링 M2.5 저두 볼트는 모터 자리 바깥 면의 '
               'Ø4.8 × 1.7 카운터보어에 바로 앉는다(면압은 1b절).  바퀴 출력 → PETG 허브 플랜지는 v2 그대로(3절).\n'
               % P.BEARING_FACE)
    rows = []
    web = P.Y_FEMUR_OUT - P.Y_RING_KNEE - P.CBORE_M25[1]          # seat under the pocket, to the counterbore floor
    floor = P.BEARING_FACE
    f = T_CHK / (9 * 10.0)
    rows.append(('구동 보스 ← 출력, 토크 25 N·m', 'M3 6 + 다웰 3 (r 10): 개당 %.0f N' % f,
                 '6061 구멍 지압 %.1f MPa (보스 바닥 %.1f mm)' % (f / (3.0 * floor), floor), ok(f / (3.0 * floor), Y_AL)))
    f = T_CHK / (6 * P.AKE_PCD_RING / 2)
    rows.append(('고정링 → 6061 모터 자리, 토크 25 N·m', 'M2.5 6 (r 18.25): 개당 %.0f N' % f,
                 '6061 구멍 지압 %.1f MPa (포켓 아래 자리 %.1f mm, 체결 마찰 무시)' % (f / (2.5 * web), web),
                 ok(f / (2.5 * web), Y_AL)))
    verdict.append(('관절 토크 25 N·m 전달(출력·고정링, 6061)', '지압 ≤ %.0f MPa / 276' % (f / (2.5 * web)),
                    ok(f / (2.5 * web), Y_AL)))
    Mx_hip = lambda F, Fl: Fl * P.Z_HIP + F * (YF - YTC)
    Mx_knee = lambda F, Fl: Fl * P.KNEE_Z + F * (P.Y_FEMUR_OUT - YTC)
    Mx_wheel = lambda F, Fl: Fl * P.R_WHEEL + F * P.wheel_offset()
    W_boss = math.pi * P.HUB_BOSS_D ** 3 / 32 - 6 * (math.pi / 4 * 3.4 ** 2) * 10 ** 2 / (P.HUB_BOSS_D / 2) / 2
    for joint, fn in (('hip', Mx_hip), ('knee', Mx_knee)):
        for tag, F, Fl in (('검토', F_CHK, FL_CHK), ('설계', F_DES, FL_DES)):
            M = fn(F, Fl)
            fb = bolt_circle_max(M, 6, 10.0)
            sb = M / W_boss
            pb = fb / 30.4                                       # DIN 125 M3 washer on the 6061 boss floor
            res = ok(max(fb / (AS_M3 * PROOF['12.9']), sb / Y_AL, pb / P_AL), 1.0)
            rows.append(('%s 출력 보스(6061), 옆 하중 %s: Mx %.0f N·m' % (joint, tag, M / 1000),
                         'M3 12.9 인장 %.0f N (보증 %.0f)' % (fb, AS_M3 * PROOF['12.9']),
                         'Ø28 보스 굽힘 %.0f MPa (276), 와셔 면압 %.0f MPa (%.0f)' % (sb, pb, P_AL), res))
            verdict.append(('%s 출력 보스 옆 하중(%s)' % (joint, tag), '굽힘 %.0f MPa / 276, 와셔 %.0f / %.0f' % (sb, pb, P_AL),
                            res))
    for joint, fn, cls, plate in (('hip', Mx_hip, '12.9', '6061 hip 판'),
                                  ('knee', Mx_knee, '8.8', '6061 허벅지 모터 자리'),
                                  ('wheel', Mx_wheel, '8.8', '6061 정강이 모터 자리')):
        for tag, F, Fl in (('검토', F_CHK, FL_CHK), ('설계', F_DES, FL_DES)):
            M = fn(F, Fl)
            ft = bolt_circle_max(M, 6, P.AKE_PCD_RING / 2)
            proof = AS_M25 * PROOF[cls]
            a_head = math.pi / 4 * ((4.5 if cls == '8.8' else 4.5) ** 2 - 2.8 ** 2)
            txt = '머리 면압 %.0f MPa (%.0f)' % (ft / a_head, P_AL)
            res = ok(max(ft / proof, ft / a_head / P_AL), 1.0)
            rows.append(('%s 고정링, 옆 하중 %s: Mx %.0f N·m' % (joint, tag, M / 1000),
                         'M2.5 %s 인장 %.0f N (보증 %.0f), %s' % (cls, ft, proof, plate), txt, res))
            if tag == '검토':
                verdict.append(('%s 고정링 나사 옆 하중(검토)' % joint, '%.0f N / %.0f' % (ft, proof), res))
    table(('부위', '체결', '응력', '판정'), rows)
    out.append('\nv2에서 SF 2 검토 하중에 걸렸던 hip·knee 출력 보스(PETG 굽힘 68 / 49 MPa, 한계 45)는 6061이라 항복 276 MPa의 '
               '약 1/4로 풀렸다.  남는 한계는 체결 나사와 AKE60 출력 베어링이다: 검토 하중에서 hip 출력 M3 12.9가 보증하중 '
               '가까이 당겨지고, 이 하중에서는 AKE60 출력 베어링(정격 비공개)이 먼저 한계일 수 있다 — CubeMars에 허용 모멘트 '
               '확인 필요.  hip 고정링 M2.5는 12.9(ISO 4762); AKE 링 나사산(알루미늄, 깊이 5 mm) 전단 한계 약 2.5~3 kN이라 '
               '나사는 끝까지 물려야 한다.')

    # ------------------------------------------------------------------ 3. hub
    h('3. 바퀴 허브 플랜지 (PETG 100 %)와 한 조각 림 (PETG, M365 8.5 × 2 통고무 타이어)')
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
    # 10-31 rev 2 rim: 6 x M3 x 14 + DIN 125 washers clamp RIM_SCREW_SEAT of the rim hub (the first rim: 2.5 mm)
    ts = P.RIM_SCREW_SEAT
    f = T_CHK / (6 * 18.0)
    rows.append(('림 → 플랜지, 토크 25 N·m', 'M3 × 14 6개 + 열압입 인서트 (r 18), 개당 전단 %.0f N' % f,
                 '인서트(Ø4×6) 지압 %.1f, 림 구멍(체결 %.0f mm) 지압 %.1f MPa, Ø11 등록 턱이 반경 하중'
                 % (f / 24.0, ts, f / (3.0 * ts)), ok(max(f / 24.0, f / (3.0 * ts)), C_PETG)))
    for tag, F, Fl in (('검토', F_CHK, FL_CHK), ('설계', F_DES, FL_DES)):
        M = F * arm_joint + Fl * P.R_WHEEL
        fb = bolt_circle_max(M, 6, 18.0)
        pw, tp = fb / 30.4, fb / (math.pi * 7.0 * ts)
        res = ok(max(fb / INSERT_M3_PULL, pw / C_PETG), 1.0)
        rows.append(('림 → 플랜지 굽힘, %s: %.1f N·m' % (tag, M / 1000), 'M3 인서트 인발 %.0f N (약 %.0f)'
                     % (fb, INSERT_M3_PULL), 'DIN 125 와셔 지압 %.0f MPa (50), 림 허브 %.0f mm 뚫림 전단 %.1f MPa'
                     % (pw, ts, tp), res))
        if tag == '검토':
            verdict.append(('림 → 플랜지 인서트 인발·와셔 지압(검토)', '%.0f N / 약 1000, %.0f MPa / 50' % (fb, pw), res))
    # 10-31: shear across the rim / flange face = torque share + the wheel load on half the screws, friction and the
    # register ignored (conservative); the M3 is fully threaded, so the shear plane cuts the thread (minor dia 2.387)
    f_t, f_r = T_CHK / (6 * 18.0), F_CHK / 3
    f_s = f_t + f_r
    tau, tau_y = f_s / (math.pi / 4 * 2.387 ** 2), 0.58 * 640.0
    b_rim, b_ins = f_s / (3.0 * ts), f_s / (LG_INSERT_D * 5.7)
    res = ok(max(tau / tau_y, b_rim / C_PETG, b_ins / C_PETG), 1.0)
    rows.append(('림 → 플랜지 전단, 토크 25 N·m + 바퀴 하중 600 N(검토)',
                 'M3 × 14 8.8 하나에 %.0f + %.0f = %.0f N (마찰·등록 턱 무시, 하중은 나사 3개)' % (f_t, f_r, f_s),
                 '나사 골지름 전단 %.0f MPa (8.8 전단 항복 %.0f), 림 구멍 지압 %.1f, 인서트 바깥 지압 %.1f MPa (50)'
                 % (tau, tau_y, b_rim, b_ins), res))
    verdict.append(('림 → 플랜지 전단(검토, 토크 + 바퀴 하중)', '나사 %.0f / %.0f MPa, 지압 ≤ %.0f / 50' % (tau, tau_y, max(b_rim, b_ins)),
                    res))
    reg = F_CHK / (2 * P.RIM_REG[0] * P.RIM_REG[1])
    rows.append(('Ø%.0f × %.0f 등록 턱 지압, 바퀴 하중 600 N(검토)' % (2 * P.RIM_REG[0], P.RIM_REG[1]),
                 '림 홈 Ø%.1f(반경 틈 %.1f < 나사 구멍 0.2): 등록 턱이 먼저 닿음' % (2 * (P.RIM_REG[0] + P.RIM_REG_GAP[0]),
                                                                      P.RIM_REG_GAP[0]),
                 '%.1f MPa (50)' % reg, ok(reg, C_PETG)))
    f_pre = 500.0 / (0.2 * 3.0)                    # 0.5 N m, T = K F d with K 0.2
    rows.append(('림 체결 마찰, M3 0.5 N·m로 조임', '나사 하나 약 %.0f N, 6개 %.1f kN' % (f_pre, 6 * f_pre / 1000),
                 '마찰(μ 0.3) 토크 %.0f N·m, 옆 힘 %.1f kN — 설계 12.5 N·m·300 N에서 미끄러지지 않음, 와셔 지압 %.0f MPa'
                 % (0.3 * 6 * f_pre * 0.018, 0.3 * 6 * f_pre / 1000, f_pre / 30.4), '참고'))
    # rim hub + web as a plate tilted by M about a diameter: the radial moment per unit length at radius r is at most
    # M / (pi r) (cos distribution, the hub shear's share ignored -> conservative), sigma = 6 m / t^2.  Critical at
    # the hub flange's edge (r 23), where the plate leaves its support.
    rs = P.TYRE_BORE_D / 2
    ri = rs - P.RIM_SEAT_T
    t_at = lambda r: (P.RIM_HUB_T if r <= P.RIM_HUB_R else
                      P.RIM_HUB_T - (P.RIM_HUB_T - P.RIM_WEB_T) * (r - P.RIM_HUB_R) / (ri - P.RIM_HUB_R))
    plate = lambda M, r, t: 6 * M / (math.pi * r * t ** 2)
    stations = (23.0, P.RIM_HUB_R, (P.RIM_HUB_R + ri) / 2, ri)
    for tag, F, Fl in (('검토', F_CHK, FL_CHK), ('설계', F_DES, FL_DES)):
        M = F * arm_joint + Fl * P.R_WHEEL
        sv = [plate(M, r, t_at(r)) for r in stations]
        rows.append(('림 허브·웨브 판 굽힘, %s: %.1f N·m' % (tag, M / 1000),
                     '허브 %g mm (r ≤ %g) → 웨브 %g mm (r %.1f), 구멍 없음' % (P.RIM_HUB_T, P.RIM_HUB_R, P.RIM_WEB_T, ri),
                     ', '.join('r %.0f: %.0f' % (r, s_) for r, s_ in zip(stations, sv)) + ' MPa', ok(max(sv), S_PETG)))
        if tag == '검토':
            verdict.append(('림 허브·웨브 판 굽힘(검토)', '%.1f MPa / 45' % max(sv), ok(max(sv), S_PETG)))
            first = (plate(M, 23.0, 4.75), bolt_circle_max(M, 6, 18.0) / (math.pi * 5.5 * 2.5))
        else:
            first += (plate(M, 23.0, 4.75),)
    # the seat ring (6 mm) carries the tyre load as a cantilever from the web to the tyre centre (fillet and lugs
    # ignored); section = the 60 deg arc under the contact patch.  Printed flange face down, so this stress runs across
    # the layers.
    arm_seat = P.TYRE_BORE_W / 2
    arc = rs * math.pi / 3
    Ws = arc * P.RIM_SEAT_T ** 2 / 6
    for tag, F in (('검토', F_CHK), ('설계', F_DES)):
        sg = F * arm_seat / Ws
        rows.append(('림 안장 링 %.0f mm (웨브에서 외팔), %s %.0f N' % (P.RIM_SEAT_T, tag, F),
                     '타이어 중심까지 %.2f mm, 60° 호 %.0f mm 폭' % (arm_seat, arc), '%.1f MPa (층 사이 방향, 필릿·돌기 무시)' % sg,
                     ok(sg, S_PETG)))
        if tag == '검토':
            verdict.append(('림 안장 링 굽힘(검토)', '%.1f MPa / 45' % sg, ok(sg, S_PETG)))
    rf = P.RIM_FLANGE_D / 2
    sfl = 6 * (FL_CHK * (rf - rs) / 2 / arc) / P.RIM_FLANGE_T ** 2
    rows.append(('열린 쪽 플랜지 뿌리, 옆 %.0f N(검토)' % FL_CHK, '%g mm 판, %.1f mm 턱, 60° 호' % (P.RIM_FLANGE_T, rf - rs),
                 '%.1f MPa (림과 한 조각, 나사 없음)' % sfl, ok(sfl, S_PETG)))
    n, lw, lh, ll = P.RIM_LUGS
    fl = T_CHK / (rs + lh / 2) / (n / 4)
    flat = ll - 2 * lh
    rows.append(('타이어 ↔ 림 토크 25 N·m(검토)', '돌기 %d개 %g × %g × 높이 %g, 1/4이 받는다고 봄: 개당 %.0f N'
                 % (n, lw, ll, lh, fl),
                 '고무 면압 %.2f MPa(45° 끝 뺀 %.1f mm, 2 MPa로 봄), 돌기 뿌리 전단 %.2f MPa'
                 % (fl / (lh * flat), flat, fl / (lw * ll)), ok(fl / (lh * flat), 2.0)))
    rows.append(('AKE60 출력 베어링', '반경 600 N이 %.0f mm에 → %.1f N·m' % (d, F_CHK * d / 1000),
                 'AKE60-8 베어링 정격 비공개', '확인 필요'))
    table(('항목', '값', '응력·힘', '판정'), rows)
    out.append('\n10-31 첫 림(원판 4.75 mm, 나사 머리 밑 2.5 mm, Ø22 구멍 6개)은 같은 식으로 허브 플랜지 끝(r 23)에서 검토 '
               '%.0f / 설계 %.0f MPa, 머리 밑 2.5 mm 뚫림 전단 %.0f MPa(검토)였다 — 머리 자리만이 아니라 원판 전체가 얇았다.  '
               '나사 머리 밑에 SUS 클램프 링을 대면 지압·뚫림은 풀리지만 플랜지 끝 굽힘은 그대로라, 허브를 %g mm로 두껍게 '
               '하고(와셔 자리 %g mm, M3 × 14 + DIN 125) 웨브를 %g mm까지 줄여 가며 구멍을 없앴다.  타이어는 열린 쪽 플랜지와 '
               '돌기 끝(둘 다 Ø%.0f)을 넘어 끼운다: 안지름 Ø%.0f → %.1f %% 늘림(M365 한 조각 림과 같은 방식, 뜨거운 물에 데움).'
               % (first[0], first[2], first[1], P.RIM_HUB_T, P.RIM_SCREW_SEAT, P.RIM_WEB_T, P.RIM_FLANGE_D,
                  P.TYRE_BORE_D, 100 * (P.RIM_FLANGE_D / P.TYRE_BORE_D - 1)))
    tip = P.RIM_SCREW_SEAT + 0.5 - 14.0 + 6.0          # flange face -> insert hole floor (legs.hub_flange: 6.0 deep)
    out.append('\n림 나사 길이(10-31 사용자 질문): M3 × 14가 와셔 자리 %g mm + 와셔 0.5에서 플랜지 면 너머 %.1f mm까지 들어가 인서트'
               '(5.7)에 %.1f mm 물리고, 끝이 인서트 구멍(Ø4 × 6) 바닥에서 **%.1f mm** 남는다(와셔 자리 8 mm일 때 0.5 — 나사 길이 공차 '
               '±0.35, 출력 공차, 인서트를 녹여 넣을 때 바닥에 밀려 쌓이는 PETG를 합치면 나사가 바닥에 먼저 닿아 와셔를 덜 누를 수 '
               '있었다).  물림 %.1f mm는 지름의 1.5배라 황동 나사산이 약 3 kN을 버텨 나사(8.8 보증 3.0 kN)와 비슷하고, 이 연결에서 '
               '먼저 한계에 닿는 것은 인서트가 PETG에서 빠지는 쪽(약 1 kN, 물림과 무관)이다 — 조임 토크 0.5 N·m 정도.'
               % (P.RIM_SCREW_SEAT, 14.0 - P.RIM_SCREW_SEAT - 0.5, 14.0 - P.RIM_SCREW_SEAT - 0.6, tip,
                  14.0 - P.RIM_SCREW_SEAT - 0.6))
    tbl = []
    for W in (45.0, 54.0, 60.0, 76.0):
        dd = d - P.TYRE_W / 2 + W / 2
        tbl.append('폭 %.0f mm: %.1f mm → 정지 %.2f N·m, 설계 %.1f N·m' % (W, dd, F_STATIC * dd / 1000, F_DES * dd / 1000))
    out.append('\n타이어 폭에 따른 출력 굽힘 팔(= 바퀴 판 %.0f mm − 포켓 + 간격 %.0f mm + 폭/2): ' % (P.WHEEL_PLATE_T, P.GAP_TYRE)
               + '; '.join(tbl) + '.  10-31부터 54 mm(M365 8.5 × 2, 사용자 주문).')

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
    h('5. 모터 열 (v3: 모터가 모두 6061에 붙음)')
    out.append('v3: knee·wheel 모터도 6061 링크 자리에 붙어 hip처럼 열이 금속으로 빠진다; PETG와 맞닿는 곳은 드라이버 컵 '
               '립(Ø88 하우징 위)뿐이다.  구리 손실 P ≈ 0.75 × R(선간 0.577 Ω) × I², '
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
