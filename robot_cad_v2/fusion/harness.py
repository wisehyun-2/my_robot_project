"""Wiring harness: 16 AWG power pairs (XT30), 26 AWG twisted CAN pairs (JST PH-3), battery leads, switch and fan
wires, modelled as real bundles along routed paths (brief item 8).

    battery XT60-F -- adapter XT60-M / XT90-F -- mjpower-ss XT90-M
    mjpower-ss P5 -> pi3hat J1                (XT30 M-F, 16 AWG)
    mjpower-ss P3 -> left hip J1, P6 -> right hip J1   (M-F)
    hip J2 -> knee J1, knee J2 -> wheel J1   (F-F), wheel J2 free under a cap
    pi3hat JC1 -> left hip CAN2, JC2 -> right hip CAN2   (PH-3 F-F, 26 AWG twisted, pin 1 H / 2 L, pin 3 empty)
    hip CAN1 -> knee CAN2, knee CAN1 -> wheel CAN2, wheel CAN1 = 120 ohm terminator
    pi3hat JP6 (1 = 5 V, 3 = GND) -> fan;  mjpower-ss PH-4 -> rocker switch in the back panel

Every cable is a list of runs.  A fixed run belongs to one link and is written in nominal-pose robot
coordinates; a loop joins an anchor on one link to an anchor on the next link with a fixed length and is
re-shaped for every pose (cubic Bezier between the anchors and their tangents, scaled to the length):
see loop_points().  Loop lengths = longest anchor distance over the checked poses + slack.
"""
import math

import cadlib as C
import layout as LY
import legs as LG
import params as P
import torso as TO

AWG16, AWG12, CAN_OD, THIN = 2.9, 4.0, 2.2, 1.2
XT30_L, PH_L = 15.5, 6.0
SFX = {1: 'L', -1: 'R'}


# ----------------------------------------------------------------------------- geometry helpers
def smooth(pts, n=3):
    """Chaikin corner cutting that keeps the end points (bends of a few mm radius)."""
    for _ in range(n):
        out = [pts[0]]
        for a, b in zip(pts[:-1], pts[1:]):
            out.append(C.lerp(a, b, 0.25))
            out.append(C.lerp(a, b, 0.75))
        out.append(pts[-1])
        pts = out
    return pts


def bezier(p0, t0, p3, t3, k, n=24):
    """Cubic Bezier from p0 leaving along t0 to p3 arriving along t3 (handle length k)."""
    p1 = C.add(p0, C.mul(C.unit(t0), k))
    p2 = C.sub(p3, C.mul(C.unit(t3), k))
    out = []
    for i in range(n + 1):
        u = i / n
        a, b, c, d = (1 - u) ** 3, 3 * u * (1 - u) ** 2, 3 * u * u * (1 - u), u ** 3
        out.append(tuple(a * p0[j] + b * p1[j] + c * p2[j] + d * p3[j] for j in range(3)))
    return out


def length(pts):
    return sum(C.norm(C.sub(b, a)) for a, b in zip(pts[:-1], pts[1:]))


def loop_points(p0, t0, p3, t3, L, droop=(0.0, 0.0, 0.0)):
    """Curve of length L between two anchors: handle length found by bisection; if the anchors are further
    apart than L the result is the straight (stretched) line and the caller reports it."""
    lo, hi = 0.0, 3.0 * L
    if C.norm(C.sub(p3, p0)) >= L:
        return [p0, p3], False
    for _ in range(40):
        k = 0.5 * (lo + hi)
        if length(bezier(p0, t0, p3, t3, k)) < L:
            lo = k
        else:
            hi = k
    return bezier(p0, t0, p3, t3, 0.5 * (lo + hi)), True


def wires_along(B, pts, wires, pref=(0.0, 1.0, 0.0)):
    """Bodies of parallel wires along pts: wires = [(offset mm, radius mm)]; the wires lie side by side
    along pref projected normal to the path (x then z used where the path runs along pref)."""
    out = []
    n = len(pts)
    sides = []
    for i in range(n):
        a = pts[max(0, i - 1)]
        b = pts[min(n - 1, i + 1)]
        t = C.unit(C.sub(b, a)) if C.norm(C.sub(b, a)) > 1e-6 else (1.0, 0.0, 0.0)
        for cand in (pref, (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)):
            q = C.sub(cand, C.mul(t, C.dot(cand, t)))
            if C.norm(q) > 0.3:
                break
        sd = C.unit(q)
        if sides and C.dot(sd, sides[-1]) < 0:
            sd = C.mul(sd, -1)
        sides.append(sd)
    for (off, rad) in wires:
        q = [C.add(p, C.mul(sd, off)) for p, sd in zip(pts, sides)]
        out.append(B.chain(q, rad))
    return out


POWER = [(-1.55, AWG16 / 2), (1.55, AWG16 / 2)]
POWER_LOOKS = ['wire red', 'wire black']
BUNDLE = [(-3.5, AWG16 / 2), (-0.4, AWG16 / 2), (2.45, CAN_OD / 2)]     # power pair + CAN pair together
BUNDLE_LOOKS = ['wire red', 'wire black', 'wire green']
CAN = [(0.0, CAN_OD / 2)]


# ----------------------------------------------------------------------------- ports (robot coordinates)
def board_port(joint, s, port):
    """(mating frame, rear point, cable direction) of a plug in a moteus side-entry port."""
    import assembly as A
    f = A.port_frame(joint, s, port)
    L = XT30_L if port.startswith('J') else PH_L
    d = C.col(f, 2)
    return f, C.apply(f, (0.0, 0.0, L)), d


def vertical_plug_frame(p):
    return C.frame(p, (1, 0, 0), (0, 1, 0), (0, 0, 1))


def mj_port(name):
    """mjpower-ss vertical XT30-F outputs (P1..P6): XT30U-M plug pushed in from the top."""
    import vendor as V
    m = TO.mjpower_frame()
    x, y = V.MJPOWER_OUT[name]
    base = C.apply(m, (x, y, 1.6 + 10.8 - 4.0))
    return vertical_plug_frame(base), C.add(base, (0, 0, XT30_L)), (0.0, 0.0, 1.0)


HAT_XT30 = {'J1': (7.15, 27.2), 'J2': (12.85, 27.2)}
HAT_CAN = {'JC1': (25.05, -20.0), 'JC2': (25.05, -8.6), 'JC3': (25.05, 2.8), 'JC4': (12.85, -20.0), 'JC5': (12.85, -8.6)}


def hat_port(name):
    m = TO.pi3hat_frame()
    if name in HAT_XT30:
        x, z = HAT_XT30[name]
        base = C.apply(m, (x, 12.3 - 4.0, z))
        return vertical_plug_frame(base), C.add(base, (0, 0, XT30_L)), (0.0, 0.0, 1.0)
    x, z = HAT_CAN[name]
    base = C.apply(m, (x, 8.1 - 4.0, z))
    return vertical_plug_frame(base), C.add(base, (0, 0, PH_L)), (0.0, 0.0, 1.0)


def grommet(s, y):
    g = P.theta_dir(P.GROMMET_DEG)
    return (g[0] * P.GROMMET_R, s * y, P.Z_HIP + g[1] * P.GROMMET_R)


# ----------------------------------------------------------------------------- leg anchors (robot coordinates)
def _groove_point(a, b, half, edge, s_along, depth, y):
    ux, uz = b[0] - a[0], b[1] - a[1]
    L = math.hypot(ux, uz)
    ux, uz = ux / L, uz / L
    nx, nz = LG.edge_normal(a, b, edge)
    off = half - depth
    return (a[0] + ux * s_along + nx * off, y, a[1] + uz * s_along + nz * off), (nx, 0.0, nz), (ux, 0.0, uz)


GROOVE_IN = 8.0      # cables enter / leave a groove this far from its end walls


def femur_groove(s):
    H, K = (0.0, P.Z_HIP), (P.KNEE_X, P.KNEE_Z)
    y = s * (P.Y_FEMUR_IN + P.Y_FEMUR_OUT) / 2
    p0, n, u = _groove_point(H, K, P.FEMUR_H / 2, LG.FEMUR_EDGE, LG.FEMUR_GROOVE[0] + GROOVE_IN, LG.GROOVE_DEPTH / 2, y)
    p1, _, _ = _groove_point(H, K, P.FEMUR_H / 2, LG.FEMUR_EDGE, LG.FEMUR_GROOVE[1] - GROOVE_IN, LG.GROOVE_DEPTH / 2, y)
    return p0, p1, n, u


def tibia_groove(s):
    K, W = (P.KNEE_X, P.KNEE_Z), (0.0, P.R_WHEEL)
    y = s * (P.Y_TIBIA_IN + P.Y_TIBIA_OUT) / 2
    p0, n, u = _groove_point(K, W, P.TIBIA_H / 2, LG.TIBIA_EDGE, LG.TIBIA_GROOVE[0] + GROOVE_IN, LG.GROOVE_DEPTH / 2, y)
    p1, _, _ = _groove_point(K, W, P.TIBIA_H / 2, LG.TIBIA_EDGE, LG.TIBIA_GROOVE[1] - GROOVE_IN, LG.GROOVE_DEPTH / 2, y)
    return p0, p1, n, u


Y_HIP_LOOP = P.FEMUR_WIDE_OUT + 10.0    # plane of the hip loop: outboard of the (widened) thigh, never crossed by a link


def hip_loop_anchors(s):
    """(torso anchor, tangent), (thigh anchor, tangent).  Both ends lie in the plane y = Y_HIP_LOOP: the torso
    end at the grommet's radius / angle (never swept by the thigh), the thigh end 12 mm off the thigh's back
    edge at the groove entry; tangents in the plane, so the loop stays in it for every pose."""
    a = grommet(s, Y_HIP_LOOP)
    g0, _g1, n, _u = femur_groove(s)
    b = C.add(g0, C.mul(n, 12.0))
    b = (b[0], s * Y_HIP_LOOP, b[2])
    return (a, C.unit((0.3, 0.0, -0.95))), (b, (-n[0], 0.0, -n[2]))


KNEE_LOOP_R, KNEE_LOOP_DEG = 86.6, 43.5


def knee_loop_anchors(s):
    """(thigh anchor, tangent), (shank anchor, tangent): the thigh end sits in the outer part of the thigh's
    lateral band (y +198), r 87 from the knee and 43.5 deg off the thigh's axis towards the knee wedge (44 mm
    outside the thigh's edge, still 10.7 deg = 16 mm outside the shank at full flexion), the shank end in the
    shank's band 8 mm off its wedge edge; neither link can sweep over the other's anchor and the loop crosses the
    1.5 mm gap plane inside the open wedge."""
    K = (P.KNEE_X, P.KNEE_Z)
    t_f = math.atan2(P.Z_HIP - K[1], 0.0 - K[0]) + math.radians(KNEE_LOOP_DEG)     # thigh axis (knee -> hip) + 43.5 deg
    a = (K[0] + KNEE_LOOP_R * math.cos(t_f), s * (P.Y_FEMUR_OUT - 6.0), K[1] + KNEE_LOOP_R * math.sin(t_f))
    t0, _t1, n, _u = tibia_groove(s)
    b = C.add(t0, C.mul(n, 13.0))                # the bundle fans out sideways (5 mm) where it runs along y
    b = (b[0], s * (P.Y_TIBIA_IN + 6.5), b[2])
    return (a, (0.0, float(s), 0.0)), (b, (0.0, float(s), 0.0))


# loop lengths (mm) from poses.chord_table over L0 200..335 x hip -80..+60: anchor distance hip 56..135,
# knee 20..105 -> length = longest distance + about 20 mm
HIP_LOOP_L = 155.0
KNEE_LOOP_L = 125.0
Z_UP, Y_SIDE = (0.0, 0.0, 1.0), (0.0, 1.0, 0.0)


def run(link, pts, wires, looks, pref=Z_UP):
    return (link, pts, wires, looks, pref)


# ----------------------------------------------------------------------------- cable definitions
def cables(s):
    """List of cables for side s: dict(name, runs=[(link, pts, wires, looks, pref) | ('loop', ...)], plugs=[...]).

    Port convention on every moteus: incoming J1 + CAN2 at -x_b, outgoing J2 + CAN1 at +x_b."""
    k = SFX[s]
    out = []
    M = lambda p: (p[0], s * p[1], p[2])
    ZH = P.Z_HIP
    F, T = 'femur_' + k, 'tibia_' + k
    # --- torso: mjpower-ss -> hip J1 (power, M-F); approaches J1 from the inboard side, above J2
    pf, pr, pd = mj_port('P3' if s > 0 else 'P6')
    hf, hr, hd = board_port('hip', s, 'J1')
    rise = 12.0 if s > 0 else 8.0           # P6 stays low under the P5 -> pi3hat cable
    pts = [pr, C.add(pr, (0, 0, rise)), M((-24.0, 42.0, ZH - 16.0)), M((-30.0, 75.0, ZH - 10.0)), M((-60.0, 112.0, ZH + 2.0)),
           M((-67.0, 119.0, ZH + 20.0)), C.add(hr, C.mul(hd, 16.0)), hr]
    out.append(dict(name='power_board_to_hip_%s' % k, runs=[run('torso', pts, POWER, POWER_LOOKS)],
                    plugs=[('xt30', pf, 'torso'), ('xt30', hf, 'torso')]))
    # --- torso: pi3hat JC -> hip CAN2 (runs above the power cables)
    cf, cr, cd = hat_port('JC1' if s > 0 else 'JC2')
    hf2, hr2, hd2 = board_port('hip', s, 'CAN2')
    pts = [cr, C.add(cr, (0, 0, 14)), (12.0, s * 2.0, ZH + 6.0), M((-20.0, 60.0, ZH + 10.0)), M((-42.0, 118.0, ZH + 26.0)),
           M((-49.0, 137.2, ZH + 28.0)), C.add(hr2, C.mul(hd2, 10.0)), hr2]
    out.append(dict(name='can_hat_to_hip_%s' % k, runs=[run('torso', pts, CAN, ['wire green'])],
                    plugs=[('ph3', cf, 'torso'), ('ph3', hf2, 'torso')]))
    # --- hip J2 + CAN1 -> shell grommet -> loop -> thigh groove -> knee J1 / CAN2
    jf, jr, jd = board_port('hip', s, 'J2')
    nf, nr, nd = board_port('hip', s, 'CAN1')
    g_in = grommet(s, P.YW - P.HIP_PLATE_T - 2.0)
    join = C.lerp(C.add(jr, C.mul(jd, 22.0)), C.add(nr, C.mul(nd, 22.0)), 0.5)
    (a_h, ta_h), (b_h, tb_h) = hip_loop_anchors(s)
    t_pts = [join, M((-70.0, 150.0, ZH + 32.0)), M((-58.0, 162.0, ZH + 55.0)), g_in, C.add(a_h, (0, -s * 12.0, 0)), a_h]
    g0, g1, n_f, u_f = femur_groove(s)
    kj1f, kj1r, kj1d = board_port('knee', s, 'J1')
    kc2f, kc2r, kc2d = board_port('knee', s, 'CAN2')
    K = (P.KNEE_X, P.KNEE_Z)
    split = (K[0] - 63.0, s * (P.Y_FEMUR_IN - 8.0), K[1] + 18.0)
    f_pts = [b_h, (b_h[0], g0[1], b_h[2]), g0, C.add(g0, C.mul(u_f, 12.0)), C.add(g1, C.mul(u_f, -12.0)), g1,
             C.add(g1, C.mul(n_f, 16.0)), (K[0] - 57.0, s * (P.Y_FEMUR_IN + 4.0), K[1] + 22.0), split]
    out.append(dict(name='hip_to_knee_%s' % k, runs=[
        run('torso', [jr, C.add(jr, C.mul(jd, 12.0)), join], POWER, POWER_LOOKS),
        run('torso', [nr, C.add(nr, C.mul(nd, 12.0)), join], CAN, ['wire green']),
        run('torso', t_pts, BUNDLE, BUNDLE_LOOKS),
        ('loop', 'torso', a_h, ta_h, F, b_h, tb_h, HIP_LOOP_L),
        run(F, f_pts, BUNDLE, BUNDLE_LOOKS, Y_SIDE),
        run(F, [split, C.add(kj1r, C.mul(kj1d, 14.0)), kj1r], POWER, POWER_LOOKS),
        run(F, [split, C.add(kc2r, C.mul(kc2d, 12.0)), kc2r], CAN, ['wire green']),
    ], plugs=[('xt30', jf, 'torso'), ('ph3', nf, 'torso'), ('xt30', kj1f, F), ('ph3', kc2f, F)]))
    # --- knee J2 + CAN1 -> knee loop (behind the knee) -> shank groove -> wheel J1 / CAN2
    kj2f, kj2r, kj2d = board_port('knee', s, 'J2')
    kc1f, kc1r, kc1d = board_port('knee', s, 'CAN1')
    (a_k, ta_k), (b_k, tb_k) = knee_loop_anchors(s)
    kjoin = (K[0] - 69.0, s * (P.Y_FEMUR_IN - 9.0), K[1] - 16.0)
    t0, t1, n_t, u_t = tibia_groove(s)
    wj1f, wj1r, wj1d = board_port('wheel', s, 'J1')
    wc2f, wc2r, wc2d = board_port('wheel', s, 'CAN2')
    # wheel driver plugs point up and back (WHEEL_CONN_DEG): the bundle leaves the groove end 12 mm off the back
    # edge, moves outboard to the plugs' level there and splits 20 mm behind the J1 / CAN2 plug ends
    wsplit = C.lerp(C.add(wj1r, C.mul(wj1d, 20.0)), C.add(wc2r, C.mul(wc2d, 20.0)), 0.5)
    e1 = C.add(t1, C.mul(n_t, 12.0))
    s_pts = [b_k, (b_k[0], t0[1], b_k[2]), t0, C.add(t0, C.mul(u_t, 12.0)), C.add(t1, C.mul(u_t, -12.0)), t1,
             e1, (e1[0], C.lerp(e1, wsplit, 0.5)[1], e1[2]), (e1[0], wsplit[1], e1[2]), wsplit]
    out.append(dict(name='knee_to_wheel_%s' % k, runs=[
        run(F, [kj2r, C.add(kj2r, C.mul(kj2d, 12.0)), kjoin], POWER, POWER_LOOKS),
        run(F, [kc1r, C.add(kc1r, C.mul(kc1d, 12.0)), kjoin], CAN, ['wire green']),
        run(F, [kjoin, C.add(kjoin, (-14.0, 0.0, 0.0)), (K[0] - 83.0, s * (P.Y_FEMUR_OUT - 14.0), K[1] + 10.0),
                C.add(a_k, (0.0, -s * 4.0, 0.0)), a_k], BUNDLE, BUNDLE_LOOKS, Y_SIDE),
        ('loop', F, a_k, ta_k, T, b_k, tb_k, KNEE_LOOP_L),
        run(T, s_pts, BUNDLE, BUNDLE_LOOKS, Y_SIDE),
        run(T, [wsplit, C.add(wj1r, C.mul(wj1d, 16.0)), wj1r], POWER, POWER_LOOKS),
        run(T, [wsplit, C.add(wc2r, C.mul(wc2d, 14.0)), wc2r], CAN, ['wire green']),
    ], plugs=[('xt30', kj2f, F), ('ph3', kc1f, F), ('xt30', wj1f, T), ('ph3', wc2f, T)]))
    return out


def torso_cables():
    """Cables that exist once: battery -> adapter -> mjpower-ss, P5 -> pi3hat, switch, fan."""
    ZH = P.Z_HIP
    out = []
    W12 = [(-2.3, AWG12 / 2), (2.3, AWG12 / 2)]
    bx, by, bz = P.BATTERY
    b0 = (P.BATTERY_X - 6.0, P.BATTERY_Y + by / 2, ZH + P.BATTERY_Z0 + bz - 18.0)
    xt60 = (70.0, 148.0, ZH + 10.0)                       # XT60 pair centre (vertical), in front of the hip driver
    pts = [b0, C.add(b0, (0, 4.5, 0)), (30.0, 159.5, ZH + 95.0), (62.0, 159.5, ZH + 60.0), (70.0, 152.0, ZH + 36.0),
           C.add(xt60, (0, 0, 16.0))]
    out.append(dict(name='battery_lead', runs=[run('torso', pts, W12, ['wire red', 'wire black'])], plugs=[]))
    m = TO.mjpower_frame()
    xt90 = C.apply(m, (-35.0, 13.5, 1.6 + 8.5))
    rear = C.add(xt90, (0.0, 28.0, 0.0))
    pts = [C.add(xt60, (0, 0, -16.0)), (70.0, 140.0, ZH - 17.0), (66.0, 128.0, ZH - 30.0), (40.0, 116.0, ZH - 42.0),
           C.add(rear, (0, 10, 0)), rear]
    out.append(dict(name='xt90_xt60_adapter_leads', runs=[run('torso', pts, W12, ['wire red', 'wire black'])], plugs=[]))
    pf, pr, pd = mj_port('P5')
    hf, hr, hd = hat_port('J1')
    pts = [pr, C.add(pr, (0, 0, 20)), (-6.0, -40.0, ZH - 4.0), C.add(hr, (0, 0, 12)), hr]
    out.append(dict(name='power_board_to_pi3hat', runs=[run('torso', pts, POWER, POWER_LOOKS)],
                    plugs=[('xt30', pf, 'torso'), ('xt30', hf, 'torso')]))
    ph4 = C.apply(m, (-31.0, -19.0, 1.6 + 6.0 + 4.0))
    sw = (-P.TORSO_X + 2.0 + 24.0, P.SWITCH_Y, ZH + P.SWITCH_Z)
    pts = [ph4, C.add(ph4, (0, 0, 10)), (-70.0, 95.0, ZH - 2.0), (-92.0, P.SWITCH_Y, ZH + 40.0), sw]
    out.append(dict(name='switch_harness', runs=[run('torso', pts, [(-1.3, THIN / 2), (0.0, THIN / 2), (1.3, THIN / 2)],
                                                      ['wire red', 'wire black', 'wire white'])], plugs=[]))
    hat = TO.pi3hat_frame()
    jp6 = C.apply(hat, (-12.0, 1.5 + 8.5, 20.0))
    fan = TO.fan_frame()
    fp = C.apply(fan, (-13.0, 13.0, 0.0))
    pts = [jp6, C.add(jp6, (0, 0, 6)), C.add(fp, (0, 0, 8)), fp]
    out.append(dict(name='fan_wires', runs=[run('torso', pts, [(-0.7, 0.55), (0.7, 0.55)], ['wire red', 'wire black'])],
                    plugs=[]))
    return out, xt60, xt90
