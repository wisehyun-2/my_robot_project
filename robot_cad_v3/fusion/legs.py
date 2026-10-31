"""Designed leg parts: 6061-T6 thigh and shank (v3) with their covers, PETG wheel hub flange + one-piece rim and the
moteus driver cups.

All parts are written in robot coordinates of the LEFT leg in the nominal pose and mirrored for the
right leg by cadlib.Builder(side=-1).  Hole patterns at motor interfaces come from the real motor
placement of each side (layout.frames), converted back to left-side coordinates with lc().

v3 links (params 'v3 metal links'): a shell machined from 40 mm 6061 plate - face plate + perimeter walls, the motor
housing (r 41.5 .. 44, cup register OD and three tapped cup bosses), the motor seat (Ø42 H7 pilot pocket, 8 mm with the
v2 motor planes), the driven boss with its spool - closed into a box by a 3 mm cover on M3 countersunk screws.  The
spool carries the joint couple between the face plate and the cover (the cover hole fits on it); the leg cables run
inside the box through two windows in the back wall (no groove, clips or inserts).
"""
import math

import cadlib as C
import layout as LY
import params as P

M3_CLEAR, M25_CLEAR = 3.4, 2.8
INSERT_M25, INSERT_M3 = 3.6, 4.0          # heat-set insert holes (printed parts)
TAP_M25, TAP_M3 = 2.05, 2.5               # tapped holes in the 6061 links, modelled at the minor diameter (fasteners.py)
BEARING_FACE = P.BEARING_FACE             # v3: the hip / knee output washers sit on the 6061 boss floor (no clamp ring)
CSK_M3 = 6.72                             # ISO 10642 M3 head (theoretical dk); cover countersink Ø + 0.3
CUP_R_IN, CUP_R_OUT = 41.0, 43.4
CUP_LIP_BRIDGE = 2.0                      # knee / wheel cups: the first 2 mm of the wall run out to the lip OD (10-30;
                                          # before, the lip over the housing OD was a loose ring, r 43.4 vs 44.05)
CUP_COLUMNS = (-30.0, -90.0, 210.0)       # screw columns, deg in the board frame (+x_b towards +z_b), left side


def lc(p, s):
    """robot point of side s -> builder (left) coordinates."""
    return (p[0], s * p[1], p[2])


def ldir(v, s):
    return (v[0], s * v[1], v[2])


def _beam(B, a, b, half, y0, y1):
    ux, uz = b[0] - a[0], b[1] - a[1]
    L = math.hypot(ux, uz)
    ux, uz = ux / L, uz / L
    nx, nz = -uz, ux
    poly = [(a[0] + nx * half, a[1] + nz * half), (a[0] - nx * half, a[1] - nz * half),
            (b[0] - nx * half, b[1] - nz * half), (b[0] + nx * half, b[1] + nz * half)]
    return B.prism(poly, y0, y1)


def _axes(a, b):
    ux, uz = b[0] - a[0], b[1] - a[1]
    L = math.hypot(ux, uz)
    return (ux / L, uz / L), L


def _at(a, u, n, s_along, t):
    """Leg-plane point s_along from a along u and t along the in-plane normal n."""
    return (a[0] + u[0] * s_along + n[0] * t, a[1] + u[1] * s_along + n[1] * t)


def _outline(B, a, b, half, r_head, y0, y1, grow=0.0):
    """Link footprint extruded y0..y1: head disc at a, beam band a -> b, motor housing disc at b."""
    body = B.ycyl(a[0], a[1], y0, y1, r_head + grow)
    C.union(body, B.ycyl(b[0], b[1], y0, y1, P.HOUSING_R + grow))
    C.union(body, _beam(B, a, b, half + grow, y0, y1))
    return body


def _cavity(B, a, b, half, r_head, y0, y1):
    """Beam pocket inside the perimeter walls (the motor housing stays a closed ring)."""
    w = P.LINK_WALL
    cav = B.ycyl(a[0], a[1], y0, y1, r_head - w)
    C.union(cav, _beam(B, a, b, half - w, y0, y1))
    C.cut(cav, B.ycyl(b[0], b[1], y0 - 1, y1 + 1, P.HOUSING_R))
    return cav


def cover_points(link):
    """Cover screw positions (leg-plane x, z; the same on both sides): two on the head arc at +-60 deg from the link
    axis, then pairs along both edges at COVER_S (clear of the cable windows at DUCT_S)."""
    a, b, half = _link_geo(link)
    u, _L = _axes(a, b)
    n = (-u[1], u[0])
    r = P.LINK_HEAD_R - 4.0
    out = [_at(a, u, n, -r * math.cos(math.radians(60)), sg * r * math.sin(math.radians(60))) for sg in (1, -1)]
    for st in COVER_S[link]:
        out += [_at(a, u, n, st, sg * (half - 4.0)) for sg in (1, -1)]
    return out


def _cover_bosses(B, body, link, y_top, y_floor):
    """Ø7 bosses merged into the walls under the cover screws, M3 tapped from the wall top (7.5 deep)."""
    sg = 1 if y_floor > y_top else -1
    for (x, z) in cover_points(link):
        C.union(body, B.ycyl(x, z, y_top, y_floor + sg * 0.5, P.COVER_BOSS_R))
    for (x, z) in cover_points(link):
        C.cut(body, B.ycyl(x, z, y_top - sg * 0.1, y_top + sg * 7.5, TAP_M3 / 2))
    return body


def _duct_windows(B, body, link, y_top, y_duct):
    """Cable windows through the back wall at DUCT_S, from the open side of the shell to DUCT_NOTCH[1] past the
    duct plane (the cover closes them into windows)."""
    a, b, half = _link_geo(link)
    u, _L = _axes(a, b)
    nb = edge_normal(a, b, FEMUR_EDGE if link == 'femur' else TIBIA_EDGE)
    sg = 1 if y_duct > y_top else -1
    y0, y1 = y_top - sg * 1.0, y_duct + sg * P.DUCT_NOTCH[1]
    w = P.LINK_WALL
    for st in DUCT_S[link]:
        x, z = _at(a, u, nb, st, half - w / 2)
        C.cut(body, B.box((x, (y0 + y1) / 2, z), (u[0], 0, u[1]), (nb[0], 0, nb[1]), P.DUCT_NOTCH[0], w + 2.0,
                          abs(y1 - y0)))
    return body


def _driven_boss(B, s, body, joint, c, y_link_in, y_face_out):
    """Boss reaching through a Ø30 hole to a motor output (output faces +y on the left for hip and knee):
    Ø28 boss, Ø11.8 x 1.4 spigot into the output recess, 6 x M3 x 16 (12.9) + DIN 125 washers on the 6061 floor
    BEARING_FACE from the output face, 3 x Ø3 dowels, Ø28 access bore to the outer face.  v3: the boss continues as a
    spool r SPOOL_R through the box (y_link_in .. y_face_out), joined to the face plate and fitted in the cover hole,
    so the joint moment enters the box as a couple between face plate and cover."""
    fr = LY.frames(joint, s)
    y_out = P.y_output(joint)
    C.union(body, B.ycyl(c[0], c[1], y_link_in, y_face_out, P.SPOOL_R))
    C.union(body, B.ycyl(c[0], c[1], y_out, y_link_in + 0.5, P.HUB_BOSS_D / 2))
    C.union(body, B.ycyl(c[0], c[1], y_out - 1.4, y_out, 5.9))
    floor = y_out + BEARING_FACE
    C.cut(body, B.ycyl(c[0], c[1], floor, y_face_out + 1, 14.0))
    for p in LY.motor_points(fr['motor'], P.AKE_PCD_OUT / 2, P.AKE_OUT_M3, P.AKE['out']):
        q = lc(p, s)
        C.cut(body, B.ycyl(q[0], q[2], y_out - 2, floor + 1, M3_CLEAR / 2))
    for p in LY.motor_points(fr['motor'], P.AKE_PCD_OUT / 2, P.AKE_OUT_PIN, P.AKE['out']):
        q = lc(p, s)
        C.cut(body, B.ycyl(q[0], q[2], y_out - 2, y_out + 7.0, 1.5))
    return body


def _motor_seat(B, s, body, joint, c, y_plate_motor_side, y_plate_far, y_cavity_end):
    """Seat holding a motor ring inside the housing (pocket on the motor side), Ø30 hole, 6 x M2.5 low-head screws
    from the far side in Ø4.8 x 1.7 counterbores (on the 6061, no clamp ring), lead groove (params.LEAD_GROOVE_*,
    2 deep) at the motor's lead direction (the leads then run axially in the r 34.5 .. 41.5 gap to the cup), motor
    cavity = housing bore r HOUSING_R - HOUSING_WALL."""
    fr = LY.frames(joint, s)
    m = fr['motor']
    sg = 1 if y_plate_far > y_plate_motor_side else -1          # far side direction along +y (left coords)
    ring = P.Y_RING_HIP if joint == 'hip' else (P.Y_RING_KNEE if joint == 'knee' else P.Y_RING_WHEEL)
    C.cut(body, B.ycyl(c[0], c[1], y_cavity_end, y_plate_motor_side, P.HOUSING_R - P.HOUSING_WALL))
    C.cut(body, B.ycyl(c[0], c[1], y_plate_motor_side - sg * 0.01, ring, P.PILOT_D_HIP / 2))   # Ø42 H7 (machined)
    C.cut(body, B.ycyl(c[0], c[1], y_plate_motor_side - sg, y_plate_far + sg, 15.0))   # Ø30 output hole
    for p in LY.motor_points(m, P.AKE_PCD_RING / 2, P.AKE_RING_M25, P.AKE['ring']):
        q = lc(p, s)
        C.cut(body, B.ycyl(q[0], q[2], ring - sg * 0.5, y_plate_far + sg, M25_CLEAR / 2))
        C.cut(body, B.ycyl(q[0], q[2], y_plate_far - sg * P.CBORE_M25[1], y_plate_far + sg, P.CBORE_M25[0] / 2))
    ex = ldir(C.col(m, 0), s)
    d = C.unit((ex[0], 0.0, ex[2]))
    g0 = y_plate_motor_side - sg * 0.5
    g1 = y_plate_motor_side + sg * 2.0
    r0, r1 = P.LEAD_GROOVE_R
    mid = (r0 + r1) / 2
    C.cut(body, B.box((c[0] + d[0] * mid, (g0 + g1) / 2, c[1] + d[2] * mid), d, (0, 1, 0), r1 - r0, abs(g1 - g0),
                      P.LEAD_GROOVE_W))
    return body


def cup_column_points(joint, s, r=40.5):
    b = LY.frames(joint, s)['board']
    out = []
    for t in CUP_COLUMNS:
        tt = math.radians(t)
        xb = r * math.cos(tt)
        out.append(C.apply(b, (xb if s > 0 else -xb, 0.0, r * math.sin(tt))))
    return out


def _cup_bosses(B, s, body, joint, y_face, y_seat, depth=6.0):
    """Three Ø6.8 bosses inside the housing bore under the driver cup's screw columns (r 40.5, the can stays 2.6 mm
    clear), M2.5 tapped from the cup face (v2: heat-set inserts in the printed housing)."""
    sg = 1 if joint == 'wheel' else -1      # into the part from the cup side (-y_b direction)
    for p in cup_column_points(joint, s):
        q = lc(p, s)
        C.union(body, B.ycyl(q[0], q[2], y_face, y_seat - sg * 0.5, 3.4))
    for p in cup_column_points(joint, s):
        q = lc(p, s)
        C.cut(body, B.ycyl(q[0], q[2], y_face + sg * 0.1, y_face - sg * depth, TAP_M25 / 2))
    return body


def edge_normal(a, b, side_n):
    ux, uz = b[0] - a[0], b[1] - a[1]
    L = math.hypot(ux, uz)
    return (-uz / L * side_n, ux / L * side_n)


# side of the beam edge facing the knee wedge (behind the knee): thigh lower-back edge, shank upper-back edge
FEMUR_EDGE = -1
TIBIA_EDGE = -1
GROOVE_DEPTH = 8.0                    # v2 edge groove: harness anchors outside the links still refer to its line
FEMUR_GROOVE = (42.0, P.L1 - 50.0)
TIBIA_GROOVE = (60.0, P.L2 - 52.0)
DUCT_IN = 8.0                         # cables enter / leave the box this far inside the old groove ends
DUCT_S = {'femur': (FEMUR_GROOVE[0] + DUCT_IN, FEMUR_GROOVE[1] - DUCT_IN),
          'tibia': (TIBIA_GROOVE[0] + DUCT_IN, TIBIA_GROOVE[1] - DUCT_IN)}
COVER_S = P.COVER_S                   # cover screw stations along the edges


def _link_geo(link):
    if link == 'femur':
        return (0.0, P.Z_HIP), (P.KNEE_X, P.KNEE_Z), P.FEMUR_H / 2
    return (P.KNEE_X, P.KNEE_Z), (0.0, P.R_WHEEL), P.TIBIA_H / 2


def cover_levels(link):
    """(cover outer face, wall top = cover inner face, face plate inner face) in left-side robot y."""
    if link == 'femur':
        y_cov = P.Y_FEMUR_IN + P.FEMUR_IN_RELIEF[0]
        return y_cov, y_cov + P.COVER_T, P.Y_FEMUR_OUT - P.LINK_FLOOR
    return P.Y_TIBIA_OUT, P.Y_TIBIA_OUT - P.COVER_T, P.Y_TIBIA_IN + P.LINK_FLOOR


def duct_y(link):
    return (P.Y_FEMUR_IN + P.Y_FEMUR_OUT) / 2 if link == 'femur' else (P.Y_TIBIA_IN + P.Y_TIBIA_OUT) / 2


def femur(s):
    """6061 thigh shell: face plate outboard (knee seat), cover inboard 1.5 mm behind the v2 inner face (shell
    screw heads); the hip spool and the knee housing / cup face keep y = Y_FEMUR_IN."""
    B = C.Builder(side=s)
    H, K, half = _link_geo('femur')
    yi, yo = P.Y_FEMUR_IN, P.Y_FEMUR_OUT
    _y_cov, y_top, y_fl = cover_levels('femur')
    body = _outline(B, H, K, half, P.LINK_HEAD_R, y_top, yo)
    C.union(body, B.ycyl(K[0], K[1], yi, yo, P.HOUSING_R))
    C.cut(body, _cavity(B, H, K, half, P.LINK_HEAD_R, y_top - 1.0, y_fl))
    _driven_boss(B, s, body, 'hip', H, yi, yo)
    _cover_bosses(B, body, 'femur', y_top, y_fl)
    y_seat = P.Y_RING_KNEE - P.SEAT_POCKET
    _motor_seat(B, s, body, 'knee', K, y_seat, yo, yi - 1)
    _cup_bosses(B, s, body, 'knee', yi, y_seat)
    _duct_windows(B, body, 'femur', y_top, duct_y('femur'))
    return body


def tibia(s):
    """6061 shank shell: face plate inboard (knee boss + wheel seat), cover outboard flush with Y_TIBIA_OUT."""
    B = C.Builder(side=s)
    K, W, half = _link_geo('tibia')
    yi, yo = P.Y_TIBIA_IN, P.Y_TIBIA_OUT
    _y_cov, y_top, y_fl = cover_levels('tibia')
    body = _outline(B, K, W, half, P.LINK_HEAD_R, yi, y_top)
    C.union(body, B.ycyl(W[0], W[1], yi, yo, P.HOUSING_R))
    C.cut(body, _cavity(B, K, W, half, P.LINK_HEAD_R, y_fl, y_top + 1.0))
    _driven_boss(B, s, body, 'knee', K, yi, yo)
    _cover_bosses(B, body, 'tibia', y_top, y_fl)
    y_seat = P.Y_RING_WHEEL + P.SEAT_POCKET
    _motor_seat(B, s, body, 'wheel', W, y_seat, yi, yo + 1)
    _cup_bosses(B, s, body, 'wheel', yo, y_seat)
    _duct_windows(B, body, 'tibia', y_top, duct_y('tibia'))
    return body


def cover(link, s):
    """3 mm 6061 cover (laser / waterjet + countersinks): the beam footprint between the head and the motor housing
    (0.3 mm clear of the housing OD and of the cup's register lip), a hole fitting on the spool, countersunk M3 at cover_points()."""
    B = C.Builder(side=s)
    a, b, half = _link_geo(link)
    y0, y1, _fl = cover_levels(link)
    sg = 1 if y1 > y0 else -1                        # from the outer face into the link
    body = B.ycyl(a[0], a[1], y0, y1, P.LINK_HEAD_R)
    C.union(body, _beam(B, a, b, half, y0, y1))
    C.cut(body, B.ycyl(b[0], b[1], y0 - sg, y1 + sg, P.HOUSING_R + 0.3))
    C.cut(body, B.ycyl(a[0], a[1], y0 - sg, y1 + sg, P.SPOOL_R + P.SPOOL_FIT))
    rk = (CSK_M3 + 0.3) / 2
    for (x, z) in cover_points(link):
        C.cut(body, B.ycyl(x, z, y0 - sg, y1 + sg, M3_CLEAR / 2))
        C.cut(body, B.ycyl(x, z, y0 - sg * 0.01, y0 + sg * (rk - M3_CLEAR / 2), rk, M3_CLEAR / 2))
    return body


# ----------------------------------------------------------------------------- wheel hub + rim
HUB_SCREWS = [math.radians(30 + 60 * k) for k in range(6)]     # rim -> flange: 6 x M3 into heat-set inserts, r 18


def hub_levels():
    """Robot y (left) of the hub stations, from the motor output towards the robot centre."""
    y_out = P.y_output('wheel')                    # output face, faces -y
    y_plate = P.Y_TIBIA_IN                        # shank plate inner face
    f1 = y_plate - 1.0                            # flange outer face (1 mm from the plate / screw heads)
    f0 = f1 - 8.0                                 # flange inner face = rim disc face
    y_tc = P.tyre_centre_y()
    b_in, b_out = y_tc + P.TYRE_BORE_W / 2, y_tc - P.TYRE_BORE_W / 2    # tyre bore edges (motor side, centre side)
    return dict(out=y_out, plate=y_plate, f1=f1, f0=f0, tc=y_tc, b_in=b_in, b_out=b_out)


def hub_flange(s):
    B = C.Builder(side=s)
    W = (0.0, P.R_WHEEL)
    lv = hub_levels()
    fr = LY.frames('wheel', s)
    y_out = lv['out']
    body = B.ycyl(W[0], W[1], lv['f1'] - 0.5, y_out, P.HUB_BOSS_D / 2)
    C.union(body, B.ycyl(W[0], W[1], y_out, y_out + 1.4, 5.9))
    C.union(body, B.ycyl(W[0], W[1], lv['f0'], lv['f1'], 23.0))
    bear = y_out - P.HUB_SCREW_SEAT                # washer seat: the M3 x 14 heads stop 2.0 short of the rim hub face
    for p in LY.motor_points(fr['motor'], P.AKE_PCD_OUT / 2, P.AKE_OUT_M3, P.AKE['out']):
        q = lc(p, s)
        C.cut(body, B.ycyl(q[0], q[2], lv['f0'] - 1, y_out + 2, M3_CLEAR / 2))
        C.cut(body, B.ycyl(q[0], q[2], lv['f0'] - 1, bear, 3.75))
    for p in LY.motor_points(fr['motor'], P.AKE_PCD_OUT / 2, P.AKE_OUT_PIN, P.AKE['out']):
        q = lc(p, s)
        C.cut(body, B.ycyl(q[0], q[2], y_out - 7.0, y_out + 2, 1.5))
    C.union(body, B.ycyl(W[0], W[1], lv['f0'] - P.RIM_REG[1], lv['f0'] + 0.5, P.RIM_REG[0]))   # centre register
    for a in HUB_SCREWS:
        C.cut(body, B.ycyl(18 * math.cos(a), W[1] + 18 * math.sin(a), lv['f0'] - 1, lv['f0'] + 6.0, INSERT_M3 / 2))
    return body


def rim(s):
    """One-piece printed rim for the M365 8.5 x 2 solid tyre (10-31 rev 2, rim_out merged): a RIM_HUB_T hub on the
    hub flange (6 x M3 x 14 + DIN 125 washers into the flange inserts, Ø11 centre register in a Ø11.2 recess) tapering to a RIM_WEB_T
    web that is also the motor-side bead flange, the Ø141 seat ring with RIM_LUGS lugs keyed into the tyre's bore
    cavities (torque by the lugs, not by friction), and the open-side bead flange the tyre is stretched over.
    Prints on the flat flange face; only the open-side flange's 4.5 mm ledge needs support."""
    B = C.Builder(side=s)
    W = (0.0, P.R_WHEEL)
    lv = hub_levels()
    f0, tc = lv['f0'], lv['tc']
    rs = P.TYRE_BORE_D / 2
    ri = rs - P.RIM_SEAT_T
    rf = P.RIM_FLANGE_D / 2
    body = B.ycyl(W[0], W[1], lv['b_in'], f0, rf)                                 # motor-side flange
    C.union(body, B.ycyl(W[0], W[1], f0 - P.RIM_WEB_T, f0, ri + 0.5))            # web
    C.union(body, B.ycyl(W[0], W[1], f0 - P.RIM_HUB_T, f0 - P.RIM_WEB_T + 0.01, P.RIM_HUB_R, ri))   # hub + taper
    y_w = f0 - P.RIM_WEB_T
    fil = B.ycyl(W[0], W[1], y_w - 5.0, y_w + 0.5, ri + 0.5)                     # 5 mm 45 deg fillet web -> seat
    C.cut(fil, B.ycyl(W[0], W[1], y_w - 5.5, y_w + 0.5, ri + 0.5, ri - 5.5))
    C.union(body, fil)
    y_o = lv['b_out'] - P.RIM_FLANGE_T
    seat = B.ycyl(W[0], W[1], y_o, lv['b_in'] + 0.5, rs)
    C.union(seat, B.ycyl(W[0], W[1], y_o, lv['b_out'], rf))                      # open-side flange
    C.cut(seat, B.ycyl(W[0], W[1], y_o - 1, lv['b_in'] + 1, ri))
    C.union(body, seat)
    n, lw, lh, ll = P.RIM_LUGS
    lugs = None
    for k in range(n):
        a = math.radians(360.0 / n * k)
        u = (math.cos(a), 0.0, math.sin(a))
        rc = rs + (lh - 0.5) / 2
        lug = B.box((W[0] + u[0] * rc, tc, W[1] + u[2] * rc), u, (0, 1, 0), lh + 0.5, ll, lw)
        if lugs is None:
            lugs = lug
        else:
            C.union(lugs, lug)
    for sg in (1, -1):                                                           # 45 deg ends
        ye = tc + sg * ll / 2
        cut = B.ycyl(W[0], W[1], ye - sg * (lh + 1.0), ye + sg * 0.5, rs + lh + 2.0)
        C.cut(cut, B.ycyl(W[0], W[1], ye - sg * (lh + 1.0), ye + sg * 0.5, rs + lh + 1.0, rs - 0.5))
        C.cut(lugs, cut)
    C.union(body, lugs)
    C.cut(body, B.ycyl(W[0], W[1], f0 - P.RIM_REG[1] - P.RIM_REG_GAP[1], f0 + 1,
                       P.RIM_REG[0] + P.RIM_REG_GAP[0]))                    # register recess
    for a in HUB_SCREWS:
        x, z = 18 * math.cos(a), W[1] + 18 * math.sin(a)
        C.cut(body, B.ycyl(x, z, f0 - P.RIM_HUB_T - 1, f0 + 1, M3_CLEAR / 2))
        C.cut(body, B.ycyl(x, z, f0 - P.RIM_HUB_T - 1, f0 - P.RIM_SCREW_SEAT, 3.8))    # washer + head seat
    return body


# ----------------------------------------------------------------------------- driver cups
SCREW_LEN = (4, 5, 6, 8, 10, 12, 14, 16, 18, 20, 22, 25, 30, 35, 40, 45, 50)


def cup_levels(joint, s):
    """Board-frame y of the cup rim (mount face) and of the cup floor's outer face."""
    b = LY.frames(joint, s)['board']
    o_y = C.origin(b)[1] * s
    y_mount = {'hip': P.YW - P.HIP_PLATE_T, 'knee': P.Y_FEMUR_IN, 'wheel': P.Y_TIBIA_OUT}[joint]
    rim = (y_mount - o_y) * LY.JOINTS[joint]['a']
    f_out = P.MOTEUS['pcb_t'] + P.MOTEUS['top_h'] + P.CUP_CLEAR_TOP + P.CUP_WALL
    return rim, f_out


def cup_screw(joint, engage=4.0):
    """(length, counterbore depth) of the column screws: the shortest standard screw that reaches through the
    column with a counterbore of at most 6 mm, the counterbore set for the given thread engagement."""
    rim, f_out = cup_levels(joint, 1)
    col = f_out - rim
    L = min(v for v in SCREW_LEN if v >= col - 6.0 + engage)
    return L, col - (L - engage)


def cup(joint, s):
    """Cover holding a moteus r4.11 on its motor axis.  Built in the board frame, then placed.

    Board frame: origin = encoder IC on the PCB motor-side face, +y_b away from the motor,
    +z_b to the connector edge.  The rim sits on the mount (hip plate inner face / link housing end):
    a register (Ø80 boss on the hip plate, housing OD Ø88 on the links) centres the cup on the same
    axis as the motor's Ø42 pilot pocket (the link lip joins the wall through a CUP_LIP_BRIDGE ring above
    the housing end); 3 screw columns; a window at the connector edge for the plugs, terminator and cap;
    a slot for the phase leads."""
    Bb = C.Builder(robot=False)
    fr = LY.frames(joint, s)
    b = fr['board']
    rim, f_out = cup_levels(joint, s)          # board y of the mount face (negative) and of the floor
    f_in = f_out - P.CUP_WALL
    yb = lambda v: (0.0, v, 0.0)

    def ycyl(x, z, y0, y1, r):
        return Bb.cyl((x, y0, z), (x, y1, z), r)

    body = ycyl(0, 0, rim, f_out, CUP_R_OUT)
    C.cut(body, ycyl(0, 0, rim - 1, f_in, CUP_R_IN))
    if joint != 'hip':
        C.union(body, ycyl(0, 0, rim - 3.0, rim, P.HOUSING_R + 2.45))
        ring = ycyl(0, 0, rim, rim + CUP_LIP_BRIDGE, P.HOUSING_R + 2.45)
        C.cut(ring, ycyl(0, 0, rim - 1, rim + CUP_LIP_BRIDGE + 1, CUP_R_IN))
        C.union(body, ring)
    # standoffs to the PCB far face
    for (x, z) in P.MOTEUS['holes']:
        C.union(body, ycyl(x, z, P.MOTEUS['pcb_t'], f_in + 0.5, 3.2))
        C.cut(body, ycyl(x, z, P.MOTEUS['pcb_t'] - 1, P.MOTEUS['pcb_t'] + 5.0, INSERT_M25 / 2))
    # screw columns (rim -> floor), counterbored on the floor to fit a standard screw (cup_screw)
    sgn = 1 if s > 0 else -1
    cb = cup_screw(joint)[1]
    for t in CUP_COLUMNS:
        tt = math.radians(t)
        x, z = 40.5 * math.cos(tt) * sgn, 40.5 * math.sin(tt)
        C.union(body, ycyl(x, z, rim, f_out, 3.4))
        hole = 1.7 if joint == 'hip' else M25_CLEAR / 2
        C.cut(body, ycyl(x, z, rim - 1, f_out + 1, hole))
        C.cut(body, ycyl(x, z, f_out - cb, f_out + 1, 3.0))
    # register (after the columns): over the hip plate's Ø80 boss, or a lip over the link housing OD
    if joint == 'hip':
        C.cut(body, ycyl(0, 0, rim - 1, rim + 1.5, 40.02))
    else:
        C.cut(body, ycyl(0, 0, rim - 4.0, rim + 0.01, P.HOUSING_R + 0.05))
    # connector window (plugs, terminator, cap) and phase-lead slot
    C.cut(body, Bb.abox(-18.0, 18.0, -12.0, f_in + 0.01, 27.0, 70.0))
    psi = math.radians(P.LEAD_ANGLE)
    d = (math.cos(psi) * sgn, 0.0, math.sin(psi))
    C.cut(body, Bb.box((d[0] * 40.5, (rim - 1 + 1.0) / 2, d[2] * 40.5), d, (0, 1, 0), 13.0, abs(rim - 1 - 1.0), 9.0))
    # vents over the FETs (board z 11.2 .. 16.8)
    for x in (-12.0, -4.0, 4.0, 12.0):
        C.cut(body, Bb.abox(x - 2.0, x + 2.0, f_in - 1, f_out + 1, 10.5, 17.5))
    C.TBM.transform(body, C.part_matrix(b))          # board frame -> robot placement (Fusion axes)
    if joint != 'hip':                                # the register lip stops where the link beam joins the housing
        B = C.Builder(side=s)
        if joint == 'knee':
            a, c, half, y0, y1 = (0.0, P.Z_HIP), (P.KNEE_X, P.KNEE_Z), P.FEMUR_H / 2 + 0.3, P.Y_FEMUR_IN - 0.1, P.Y_FEMUR_IN + 3.1
        else:
            a, c, half, y0, y1 = (P.KNEE_X, P.KNEE_Z), (0.0, P.R_WHEEL), P.TIBIA_H / 2 + 0.3, P.Y_TIBIA_OUT - 3.1, P.Y_TIBIA_OUT + 0.1
        C.cut(body, _beam(B, a, c, half, y0, y1))
    return body
