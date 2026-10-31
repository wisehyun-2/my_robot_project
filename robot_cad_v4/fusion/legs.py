"""Designed leg parts: SLA resin thigh and shank (v4), PETG wheel hub flange + one-piece rim and the moteus driver cups.

All parts are written in robot coordinates of the LEFT leg in the nominal pose and mirrored for the
right leg by cadlib.Builder(side=-1).  Hole patterns at motor interfaces come from the real motor
placement of each side (layout.frames), converted back to left-side coordinates with lc().

v4 links (params 'v4 SLA resin links'): one SLA print each.  The outline is the v2 PETG link - head disc around the
driven boss, beam band, motor housing disc, widened outboard (_wide) - but hollow: a closed box with SLA_WALL walls,
braced by a Warren truss of SLA_RIB ribs from edge wall to edge wall across the full width (_ribs).  The driven boss
continues as a spool tube through the box to the outer face (v3), so the joint moment enters both face plates.  The
joint ends are the v2 printed design: 9 mm motor plate, Ø42.05 pocket, SUS clamp rings under the stator and output
screw heads, inserts (bonded) for the driver cup screws.  The leg cables run inside the box (v3 duct line): in through
a window in the back wall, along a channel cut through the ribs, out through the second window; the channel also links
every truss cell for draining the resin, with two drain holes in the front edge wall.
"""
import math

import cadlib as C
import layout as LY
import params as P

M3_CLEAR, M25_CLEAR = 3.4, 2.8
INSERT_M25, INSERT_M3 = 3.6, 4.0          # insert holes: heat-set in the PETG parts, bonded (epoxy) in the SLA links
BEARING_FACE = P.BEARING_FACE             # on the hip / knee bosses the heads bear on a P.RING_T clamp ring
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


def _wide(B, a, b, half, r, y0, y1, end_before_b, ramp=None, inset=0.0):
    """Outboard extension of a link (band y0..y1): the head disc r at a and the beam a->b up to end_before_b from b,
    full width until WIDE_TAPER before that point, then the outer face ramps back (in the beam direction).  ramp =
    (y0, y1) of the outer link that defines the ramp plane (default: this band); inset moves the plane inwards (the
    cavity of a hollow link)."""
    (ux, uz), L = _axes(a, b)
    s_end = L - end_before_b
    s_full = s_end - P.WIDE_TAPER
    w = B.ycyl(a[0], a[1], y0, y1, r)
    C.union(w, _beam(B, a, (a[0] + ux * s_end, a[1] + uz * s_end), half, y0, y1))
    r0, r1 = ramp or (y0, y1)
    dy, ds = r1 - r0, s_end - s_full
    k = math.hypot(dy, ds)
    p = (a[0] + ux * (s_full - inset * dy / k), r1 - inset * ds / k, a[1] + uz * (s_full - inset * dy / k))
    n = (ux * dy, ds, uz * dy)                        # {(q - p) . n <= 0}: before the ramp and inside the band
    return C.inter(w, B.halfspace(p, n))


def _driven_boss(B, s, body, joint, c, y_link_in, y_face_out):
    """Boss reaching through a Ø30 hole to a motor output (output faces +y on the left for hip and knee):
    Ø28 boss, Ø11.8 x 1.4 spigot into the output recess, 6 x M3 x 16 (12.9) + DIN 125 washers on a stainless clamp
    ring (vendor.clamp_ring_out) lying on a floor BEARING_FACE - RING_T from the output face, 3 x Ø3 dowels, Ø28
    access bore to the outer face inside a spool tube r SPOOL_R that joins both face plates of the hollow link."""
    fr = LY.frames(joint, s)
    y_out = P.y_output(joint)
    C.union(body, B.ycyl(c[0], c[1], y_link_in, y_face_out, P.SPOOL_R))
    C.union(body, B.ycyl(c[0], c[1], y_out, y_link_in + 0.5, P.HUB_BOSS_D / 2))
    C.union(body, B.ycyl(c[0], c[1], y_out - 1.4, y_out, 5.9))
    floor = y_out + BEARING_FACE - P.RING_T
    C.cut(body, B.ycyl(c[0], c[1], floor, y_face_out + 1, 14.0))
    for p in LY.motor_points(fr['motor'], P.AKE_PCD_OUT / 2, P.AKE_OUT_M3, P.AKE['out']):
        q = lc(p, s)
        C.cut(body, B.ycyl(q[0], q[2], y_out - 2, floor + 1, M3_CLEAR / 2))
    for p in LY.motor_points(fr['motor'], P.AKE_PCD_OUT / 2, P.AKE_OUT_PIN, P.AKE['out']):
        q = lc(p, s)
        C.cut(body, B.ycyl(q[0], q[2], y_out - 2, y_out + P.DOWEL_HOLE, 1.5))
    return body


def _motor_seat(B, s, body, joint, c, y_plate_motor_side, y_plate_far, y_cavity_end):
    """Plate holding a motor ring (pocket on the motor side), Ø30 hole, 6 x M2.5 low-head screws from
    the far side through a stainless clamp ring (vendor.clamp_ring_stator) in a recess, lead groove (params.LEAD_GROOVE_*,
    2 deep) + axial lead slot at the motor's lead direction, can cavity r 37 (housing r 37 .. 44 holds the cup inserts)."""
    fr = LY.frames(joint, s)
    m = fr['motor']
    sg = 1 if y_plate_far > y_plate_motor_side else -1          # far side direction along +y (left coords)
    ring = P.Y_RING_HIP if joint == 'hip' else (P.Y_RING_KNEE if joint == 'knee' else P.Y_RING_WHEEL)
    C.cut(body, B.ycyl(c[0], c[1], y_cavity_end, y_plate_motor_side, P.CAN_CLEAR_R))
    C.cut(body, B.ycyl(c[0], c[1], y_plate_motor_side - sg * 0.01, ring, 21.025))      # Ø42.05 pilot pocket (printed)
    C.cut(body, B.ycyl(c[0], c[1], y_plate_motor_side - sg, y_plate_far + sg, 15.0))   # Ø30 output hole
    for p in LY.motor_points(m, P.AKE_PCD_RING / 2, P.AKE_RING_M25, P.AKE['ring']):
        q = lc(p, s)
        C.cut(body, B.ycyl(q[0], q[2], ring - sg * 0.5, y_plate_far + sg, M25_CLEAR / 2))
    C.cut(body, B.ycyl(c[0], c[1], y_plate_far - sg * (P.RING_T + P.CBORE_M25[1]), y_plate_far + sg,
                       P.STATOR_RECESS_D / 2))
    ex = ldir(C.col(m, 0), s)
    d = C.unit((ex[0], 0.0, ex[2]))
    g0 = y_plate_motor_side - sg * 0.5
    g1 = y_plate_motor_side + sg * 2.0
    r0, r1 = P.LEAD_GROOVE_R
    mid = (r0 + r1) / 2
    C.cut(body, B.box((c[0] + d[0] * mid, (g0 + g1) / 2, c[1] + d[2] * mid), d, (0, 1, 0), r1 - r0, abs(g1 - g0),
                      P.LEAD_GROOVE_W))
    a0, a1 = y_cavity_end - sg * 0.5, g1
    C.cut(body, B.box((c[0] + d[0] * 38.0, (a0 + a1) / 2, c[1] + d[2] * 38.0), d, (0, 1, 0), 6.0, abs(a1 - a0), 7.0))
    return body


def cup_column_points(joint, s, r=40.5):
    b = LY.frames(joint, s)['board']
    out = []
    for t in CUP_COLUMNS:
        tt = math.radians(t)
        xb = r * math.cos(tt)
        out.append(C.apply(b, (xb if s > 0 else -xb, 0.0, r * math.sin(tt))))
    return out


def _cup_inserts(B, s, body, joint, y_face, depth=5.0):
    sg = 1 if joint == 'wheel' else -1      # into the part from the cup side (-y_b direction)
    for p in cup_column_points(joint, s):
        q = lc(p, s)
        C.cut(body, B.ycyl(q[0], q[2], y_face + sg * 0.1, y_face - sg * depth, INSERT_M25 / 2))
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


def _link_geo(link):
    if link == 'femur':
        return (0.0, P.Z_HIP), (P.KNEE_X, P.KNEE_Z), P.FEMUR_H / 2
    return (P.KNEE_X, P.KNEE_Z), (0.0, P.R_WHEEL), P.TIBIA_H / 2


def duct_y(link):
    return (P.Y_FEMUR_IN + P.Y_FEMUR_OUT) / 2 if link == 'femur' else (P.Y_TIBIA_IN + P.Y_TIBIA_OUT) / 2


def link_levels(link):
    """(inner face, base outer face, widened outer face, end of the widening before the lower joint, inner face of the
    box cavity) in left-side robot y."""
    if link == 'femur':
        yi = P.Y_FEMUR_IN
        return yi, P.Y_FEMUR_OUT, P.FEMUR_WIDE_OUT, P.FEMUR_WIDE_END, yi + P.FEMUR_IN_RELIEF[0] + P.SLA_WALL
    yi = P.Y_TIBIA_IN
    return yi, P.Y_TIBIA_OUT, P.TIBIA_WIDE_OUT, P.TIBIA_WIDE_END, yi + P.SLA_WALL


def truss_nodes(link):
    """Rib end points (s along the link, n across it) of the Warren truss: SLA_RIBS diagonals between the cavity's
    head end and the motor housing, alternating edge to edge, the first one leaving the back (cable) edge."""
    _a, _b, half = _link_geo(link)
    L = P.L1 if link == 'femur' else P.L2
    s0, s1 = P.HEAD_R - P.SLA_WALL, L - P.HOUSING_R
    nb = FEMUR_EDGE if link == 'femur' else TIBIA_EDGE
    n_in = half - P.SLA_WALL
    k = P.SLA_RIBS[link]
    return [(s0 + (s1 - s0) * i / k, n_in * (nb if i % 2 == 0 else -nb)) for i in range(k + 1)]


def _cavity(B, link):
    """Inside of the hollow box: head disc + beam band (base width) + the widened band, inset by SLA_WALL, minus the
    motor housing disc (its r 37 .. 44 ring closes the box at the lower joint)."""
    a, b, half = _link_geo(link)
    yi, yo, wo, end, yc = link_levels(link)
    w = P.SLA_WALL
    cav = B.ycyl(a[0], a[1], yc, yo - w, P.HEAD_R - w)
    C.union(cav, _beam(B, a, b, half - w, yc, yo - w))
    C.union(cav, _wide(B, a, b, half - w, P.HEAD_R - w, yo - w - 0.5, wo - w, end, ramp=(yo - 0.5, wo), inset=w))
    C.cut(cav, B.ycyl(b[0], b[1], yi - 1.0, wo + 1.0, P.HOUSING_R))
    return cav


def _ribs(B, link, outer):
    """Warren truss ribs (SLA_RIB thick, full width) clipped to the outer link, with the cable / drain channel cut
    through them along the duct line and a smaller drain channel along the front edge."""
    a, b, half = _link_geo(link)
    u, _L = _axes(a, b)
    n = (-u[1], u[0])
    yi, yo, wo, _end, _yc = link_levels(link)
    nodes = truss_nodes(link)
    ribs = None
    for (s0, n0), (s1, n1) in zip(nodes, nodes[1:]):
        p0, p1 = _at(a, u, n, s0, n0), _at(a, u, n, s1, n1)
        d = (p1[0] - p0[0], p1[1] - p0[1])
        Ld = math.hypot(*d)
        d = (d[0] / Ld, d[1] / Ld)
        mid = ((p0[0] + p1[0]) / 2, (yi + wo) / 2, (p0[1] + p1[1]) / 2)
        r = B.box(mid, (d[0], 0, d[1]), (0, 1, 0), Ld + 4 * P.SLA_WALL, wo - yi + 2.0, P.SLA_RIB)
        ribs = C.union(ribs, r)
    C.inter(ribs, outer)
    nb = edge_normal(a, b, FEMUR_EDGE if link == 'femur' else TIBIA_EDGE)
    sA, sB = nodes[0][0] - 10.0, nodes[-1][0] + 10.0
    ch, w = P.SLA_CHANNEL, P.SLA_WALL
    for depth, size, sign in ((ch, ch, 1), (8.0, 8.0, -1)):     # back channel (cables), front drain channel
        t = half - w - depth / 2
        cx, cz = _at(a, u, (nb[0] * sign, nb[1] * sign), (sA + sB) / 2, t)
        C.cut(ribs, B.box((cx, duct_y(link), cz), (u[0], 0, u[1]), (nb[0], 0, nb[1]), sB - sA, depth + 0.02, size))
    return ribs


def _openings(B, body, link):
    """Cable windows through the back wall at DUCT_S (on the duct line) and Ø SLA_DRAIN_D drain holes through the
    front edge wall near both ends of the cavity."""
    a, b, half = _link_geo(link)
    u, _L = _axes(a, b)
    nb = edge_normal(a, b, FEMUR_EDGE if link == 'femur' else TIBIA_EDGE)
    w = P.SLA_WALL
    yd = duct_y(link)
    for st in DUCT_S[link]:
        x, z = _at(a, u, nb, st, half - w / 2)
        C.cut(body, B.box((x, yd, z), (u[0], 0, u[1]), (nb[0], 0, nb[1]), P.DUCT_WINDOW[0], w + 2.0, P.DUCT_WINDOW[1]))
    nodes = truss_nodes(link)
    for st in (nodes[0][0] + 8.0, nodes[-1][0] - 8.0):
        p0 = _at(a, u, nb, st, -half - 1.0)
        p1 = _at(a, u, nb, st, -half + w + 1.0)
        C.cut(body, B.cyl((p0[0], yd, p0[1]), (p1[0], yd, p1[1]), P.SLA_DRAIN_D / 2))
    return body


def _hollow_link(B, link):
    """Outer v2 link outline (head, beam, housing, widened band) -> closed box with truss ribs and openings."""
    a, b, half = _link_geo(link)
    yi, yo, wo, end, _yc = link_levels(link)
    body = B.ycyl(a[0], a[1], yi, yo, P.HEAD_R)
    C.union(body, B.ycyl(b[0], b[1], yi, yo, P.HOUSING_R))
    C.union(body, _beam(B, a, b, half, yi, yo))
    C.union(body, _wide(B, a, b, half, P.HEAD_R, yo - 0.5, wo, end))
    if link == 'femur':
        dr, r0 = P.FEMUR_IN_RELIEF                   # inner face relief over the shell screw heads (v2 10-31)
        rel = B.abox(-500.0, 500.0, yi - 1.0, yi + dr, 0.0, 1000.0)
        C.cut(rel, B.ycyl(a[0], a[1], yi - 2.0, yi + dr + 1.0, r0))
        C.cut(rel, B.ycyl(b[0], b[1], yi - 2.0, yi + dr + 1.0, P.HOUSING_R))
        C.cut(body, rel)
    ribs = _ribs(B, link, C.TBM.copy(body))
    C.cut(body, _cavity(B, link))
    C.union(body, ribs)
    _openings(B, body, link)
    return body


def femur(s):
    """SLA thigh: hollow truss box, hip boss + spool (head), knee motor plate at the outer face, cup inserts inboard."""
    B = C.Builder(side=s)
    H, K, _half = _link_geo('femur')
    yi, yo, wo, _end, _yc = link_levels('femur')
    body = _hollow_link(B, 'femur')
    _driven_boss(B, s, body, 'hip', H, yi, wo)
    _motor_seat(B, s, body, 'knee', K, yo - P.KNEE_PLATE_T, yo, yi - 1)
    _cup_inserts(B, s, body, 'knee', yi)
    return body


def tibia(s):
    """SLA shank: hollow truss box, knee boss + spool (head), wheel motor plate at the inner face, cup inserts outboard."""
    B = C.Builder(side=s)
    K, W, _half = _link_geo('tibia')
    yi, yo, wo, _end, _yc = link_levels('tibia')
    body = _hollow_link(B, 'tibia')
    _driven_boss(B, s, body, 'knee', K, yi, wo)
    _motor_seat(B, s, body, 'wheel', W, yi + P.WHEEL_PLATE_T, yi, yo + 1)
    _cup_inserts(B, s, body, 'wheel', yo)
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
    C.union(body, B.ycyl(W[0], W[1], y_out, y_out + P.HUB_SPIGOT[1], P.HUB_SPIGOT[0]))   # 0.2 off the recess floor
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
