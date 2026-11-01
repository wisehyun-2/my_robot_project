"""v5 wiring: the v4 harness (robot_cad_v4/fusion/harness.py: same drivers, plugs, battery, power board, pi3hat,
switch and fan; same daisy chain hip -> knee -> wheel per leg) with the two leg cables re-routed for the v5 links,
which have no cable holes (user 11-01): the cables run outside along the flat back faces, held there by adhesive
cable-tie mounts, and go to the wheel driver that now sits inboard of the shank.

    hip_to_knee:  hip J2 + CAN1 -> shell grommet -> hip loop (plane y Y_HIP_LOOP, outboard of the thigh) -> thigh back
                  face (y Y_TD, the inner tine's band) -> behind the knee cup -> knee J1 / CAN2
    knee_to_wheel: knee J2 + CAN1 -> behind the knee -> knee loop (thigh -> shank, inside the open knee wedge) ->
                  shank back face (y Y_SD) -> inboard of the shank near the wheel -> over the wheel cup -> wheel J1 / CAN2
                  (the wheel driver's ports turned / re-clocked with the wheel group)

The thigh's inner tine and the shank lie in separate y bands (1.5 mm gap), so a cable on one link's back face in its
own band is never reached by the other link.  Loops: their spare length bulges where nothing comes - the hip loop
upwards over the hip (in its plane only the wheel can come, and it stays below), the knee loop backwards into the
open side of the knee wedge.  Everything else (runs, loops re-shaped per pose, plugs) as in the v4 module, whose
helpers are re-exported here so the v4 tools (poses.loops / measure_loops / show_pose) can use this module in its place.
"""
import math

import cadlib as C
import harness as HN
import legs_v5 as LV
import params as P

smooth, bezier, length, loop_points, wires_along = HN.smooth, HN.bezier, HN.length, HN.loop_points, HN.wires_along
POWER, POWER_LOOKS, BUNDLE, BUNDLE_LOOKS, CAN = HN.POWER, HN.POWER_LOOKS, HN.BUNDLE, HN.BUNDLE_LOOKS, HN.CAN
Z_UP, Y_SIDE, SFX, run = HN.Z_UP, HN.Y_SIDE, HN.SFX, HN.run
board_port, grommet, torso_cables = HN.board_port, HN.grommet, HN.torso_cables

Y_HIP_LOOP = LV.SPAR[1] + 10.0              # hip loop plane: 10 outboard of the thigh's outer face (never crossed by it)
Y_TD = (LV.Y_TF + LV.Y_TO) / 2              # thigh cable band: the inner tine (y 178 .. 204)
Y_SD = LV.Y_SI + 6.5                        # shank cable band: inner part of the shank (y 205.5 .. 235.5)
ON_FACE = 4.0                               # bundle axis this far off a back face (bundle 2.9 thick: resting on it)
S_THIGH = (47.0, 120.0)                     # thigh run along the back face (s); the hip loop lands at the first
S_SHANK = (65.0, 125.0)                     # shank run along the back face (s); the knee loop lands at the first
OFF_HIP, OFF_KNEE = 12.0, 13.0              # loop anchors this far off the back face (as v4)
KNEE_LOOP_R, KNEE_LOOP_DEG = 86.6, 43.5     # thigh-side knee loop anchor (v4: r from the knee, angle off the thigh axis)
WHEEL_ARC = (72.0, (90.0, 50.0, 20.0))      # cable over the wheel cup: radius about the axle, angles from +x (deg)
Y_WHEEL_RUN = 196.0                         # ... in this plane (inboard of the shank face 205.5, in the plugs' band)
# loop lengths = longest anchor distance over L0 200..335 x hip -80..+60 + about 20 mm (v5run.chords, 11-01:
# hip 43.7 .. 142.0 (L0 200, hip +30), knee 16.8 .. 92.1 (L0 335, hip +60))
HIP_LOOP_L = 162.0
KNEE_LOOP_L = 115.0


def _lp(g, s_along, t_back, y):
    """Left-side robot point at s along a link, t_back behind its axis (+ = back face side), height y."""
    x, z = LV.at(g, s_along, t_back, g['nb'])
    return (x, y, z)


def _wedge(g, sign):
    """Direction (x, z) of the open side of the knee wedge in a link's frame (nominal: straight back): 0.464 along
    the link away from the knee + 0.886 off its back face (thigh: sign -1, the knee is at its end; shank: +1)."""
    u, nb = g['u'], g['nb']
    return (0.464 * sign * u[0] + 0.886 * nb[0], 0.464 * sign * u[1] + 0.886 * nb[1])


def wheel_port(s, port):
    """board_port of the wheel driver after the outboard turn and the re-clocking (legs_v5.wheel_turn / wheel_clock)."""
    f, r, d = HN.board_port('wheel', s, port)
    T = C.mmul(LV.wheel_clock(s), LV.wheel_turn(s))
    return C.mmul(T, f), C.apply(T, r), C.applyv(T, d)


def hip_loop_anchors(s):
    """(torso anchor, tangent), (thigh anchor, tangent), both in the plane y = Y_HIP_LOOP: the torso end at the shell
    grommet's radius / angle (never swept by the thigh), leaving upwards; the thigh end OFF_HIP behind the back face
    where the thigh run starts, arriving down the thigh (so the spare length bulges up over the hip, away from the
    wheel, which reaches this plane only below and behind the hip in deep crouch)."""
    g = LV.geo('thigh')
    a = HN.grommet(s, Y_HIP_LOOP)
    sw = S_THIGH[0]
    b = _lp(g, sw, LV.th_half(sw) + OFF_HIP, Y_HIP_LOOP)
    u = g['u']
    return (a, C.unit((0.3, 0.0, 0.95))), ((b[0], s * b[1], b[2]), (u[0], 0.0, u[1]))


def knee_loop_anchors(s):
    """Thigh end r 86.6 from the knee, 43.5 deg off the thigh axis towards the knee wedge, in the thigh's inner band
    (y Y_FEMUR_OUT - 6: the shank never reaches that y); shank end OFF_KNEE behind the shank's back face where the
    shank run starts, in the shank's band.  The loop leaves the thigh towards the open side of the wedge and comes back
    into the shank from it (spare length bulges away from the knee), drifting in y across the 1.5 mm gap plane."""
    K = (P.KNEE_X, P.KNEE_Z)
    t_f = math.atan2(P.Z_HIP - K[1], 0.0 - K[0]) + math.radians(KNEE_LOOP_DEG)
    a = (K[0] + KNEE_LOOP_R * math.cos(t_f), s * (P.Y_FEMUR_OUT - 6.0), K[1] + KNEE_LOOP_R * math.sin(t_f))
    gT, gS = LV.geo('thigh'), LV.geo('tibia')
    b = _lp(gS, S_SHANK[0], LV.sh_half(S_SHANK[0]) + OFF_KNEE, Y_SD)
    wt, ws = _wedge(gT, -1.0), _wedge(gS, 1.0)
    ta = C.unit((wt[0], 0.35 * s, wt[1]))
    tb = C.unit((-ws[0], 0.35 * s, -ws[1]))
    return (a, ta), ((b[0], s * b[1], b[2]), tb)


def _along(g, hf, s0, s1, y, n=3):
    """Points resting on a link's back face from s0 to s1 (left side)."""
    return [_lp(g, s0 + (s1 - s0) * i / n, hf(s0 + (s1 - s0) * i / n) + ON_FACE, y) for i in range(n + 1)]


def leg_cables(s):
    """The two re-routed leg cables of side s (dicts as harness.cables)."""
    k = SFX[s]
    M = lambda p: (p[0], s * p[1], p[2])
    ZH = P.Z_HIP
    F, T = 'femur_' + k, 'tibia_' + k
    K = (P.KNEE_X, P.KNEE_Z)
    gT, gS = LV.geo('thigh'), LV.geo('tibia')
    out = []
    # --- hip J2 + CAN1 -> grommet -> hip loop -> thigh back face -> knee J1 / CAN2
    jf, jr, jd = board_port('hip', s, 'J2')
    nf, nr, nd = board_port('hip', s, 'CAN1')
    g_in = grommet(s, P.YW - P.HIP_PLATE_T - 2.0)
    join = C.lerp(C.add(jr, C.mul(jd, 22.0)), C.add(nr, C.mul(nd, 22.0)), 0.5)
    (a_h, ta_h), (b_h, tb_h) = hip_loop_anchors(s)
    t_pts = [join, M((-70.0, 150.0, ZH + 32.0)), M((-58.0, 162.0, ZH + 55.0)), g_in, C.add(a_h, (0, -s * 12.0, 0)), a_h]
    kj1f, kj1r, kj1d = board_port('knee', s, 'J1')
    kc2f, kc2r, kc2d = board_port('knee', s, 'CAN2')
    split = (K[0] - 63.0, s * (P.Y_FEMUR_IN - 8.0), K[1] + 18.0)        # behind the knee cup (v4)
    s0, s1 = S_THIGH
    f_pts = [b_h, M(_lp(gT, s0, LV.th_half(s0) + OFF_HIP, Y_TD))] + \
        [M(p) for p in _along(gT, LV.th_half, s0 + 8.0, s1, Y_TD)] + \
        [(K[0] - 57.0, s * (P.Y_FEMUR_IN + 4.0), K[1] + 22.0), split]
    out.append(dict(name='hip_to_knee_%s' % k, runs=[
        run('torso', [jr, C.add(jr, C.mul(jd, 12.0)), join], POWER, POWER_LOOKS),
        run('torso', [nr, C.add(nr, C.mul(nd, 12.0)), join], CAN, ['wire green']),
        run('torso', t_pts, BUNDLE, BUNDLE_LOOKS),
        ('loop', 'torso', a_h, ta_h, F, b_h, tb_h, HIP_LOOP_L),
        run(F, f_pts, BUNDLE, BUNDLE_LOOKS, Y_SIDE),
        run(F, [split, C.add(kj1r, C.mul(kj1d, 14.0)), kj1r], POWER, POWER_LOOKS),
        run(F, [split, C.add(kc2r, C.mul(kc2d, 12.0)), kc2r], CAN, ['wire green']),
    ], plugs=[('xt30', jf, 'torso'), ('ph3', nf, 'torso'), ('xt30', kj1f, F), ('ph3', kc2f, F)]))
    # --- knee J2 + CAN1 -> knee loop -> shank back face -> inboard of the shank -> over the wheel cup -> wheel J1 / CAN2
    kj2f, kj2r, kj2d = board_port('knee', s, 'J2')
    kc1f, kc1r, kc1d = board_port('knee', s, 'CAN1')
    (a_k, ta_k), (b_k, tb_k) = knee_loop_anchors(s)
    kjoin = (K[0] - 69.0, s * (P.Y_FEMUR_IN - 9.0), K[1] - 16.0)
    wj1f, wj1r, wj1d = wheel_port(s, 'J1')
    wc2f, wc2r, wc2d = wheel_port(s, 'CAN2')
    wsplit = C.lerp(C.add(wj1r, C.mul(wj1d, 20.0)), C.add(wc2r, C.mul(wc2d, 20.0)), 0.5)
    r_arc, angs = WHEEL_ARC
    arc = [M((r_arc * math.cos(math.radians(a)), Y_WHEEL_RUN, P.R_WHEEL + r_arc * math.sin(math.radians(a))))
           for a in angs]
    s0, s1 = S_SHANK
    turn_in = _lp(gS, s1 + 4.0, LV.sh_half(s1 + 4.0) + ON_FACE + 2.0, Y_SD)       # leaves the face, moves inboard
    s_pts = [b_k] + [M(p) for p in _along(gS, LV.sh_half, s0 + 8.0, s1, Y_SD)] + \
        [M(turn_in), M((turn_in[0], Y_WHEEL_RUN, turn_in[2]))] + arc + [wsplit]
    out.append(dict(name='knee_to_wheel_%s' % k, runs=[
        run(F, [kj2r, C.add(kj2r, C.mul(kj2d, 12.0)), kjoin], POWER, POWER_LOOKS),
        run(F, [kc1r, C.add(kc1r, C.mul(kc1d, 12.0)), kjoin], CAN, ['wire green']),
        run(F, [kjoin, C.add(kjoin, (-14.0, 0.0, 0.0)), (K[0] - 83.0, s * (P.Y_FEMUR_OUT - 14.0), K[1] + 10.0),
                C.sub(a_k, C.mul(ta_k, 8.0)), a_k], BUNDLE, BUNDLE_LOOKS, Y_SIDE),     # into the loop's direction
        ('loop', F, a_k, ta_k, T, b_k, tb_k, KNEE_LOOP_L),
        run(T, s_pts, BUNDLE, BUNDLE_LOOKS, Y_SIDE),
        run(T, [wsplit, C.add(wj1r, C.mul(wj1d, 16.0)), wj1r], POWER, POWER_LOOKS),
        run(T, [wsplit, C.add(wc2r, C.mul(wc2d, 14.0)), wc2r], CAN, ['wire green']),
    ], plugs=[('xt30', kj2f, F), ('ph3', kc1f, F), ('xt30', wj1f, T), ('ph3', wc2f, T)]))
    return out


def cables(s):
    """v4 cables of side s with the two leg cables replaced (power board -> hip, pi3hat -> hip unchanged)."""
    v5 = leg_cables(s)
    names = {c['name'] for c in v5}
    return [c for c in HN.cables(s) if c['name'] not in names] + v5
