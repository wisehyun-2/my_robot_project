"""v5 legs: printed thigh / shank for the outboard-wheel robot (concept A "monolith", chosen by the user 11-01; MJF PA12
until 11-02, then PETG FDM - the user prints the links on an own printer).

Skeleton:
  * wheel outboard: the wheel group turned 180 deg about the vertical line through the axle in the shank mid-plane
    (tyre centre y 267.5, driver cup inboard); the driver / cup / plugs re-clocked about the wheel axis so the cup's
    notch lies on the shank again and the plugs keep their v4 angle to it;
  * thigh = fork: one deep box from the hip, then two tines - the inner one with the knee motor (v4 printed seat) and
    an outer spar outboard of the shank head - split by a slot with a half-round crotch; a 61807 bearing in the shank
    head runs on a knee cap bolted to the spar, so the knee is held on both sides and the thigh works as a 108 mm deep
    box (PA12 E 1.7 GPa is 1.6 x softer than the v4 SLA resin and the outboard wheel puts the contact 94 mm further
    from the thigh: widening alone could not do it); the slot is 9 mm wider than the shank head needs (plan A, 11-01)
    so the shank can be fitted with its hub insert on;
  * shank: deep at the knee (90), tapering straight to the wheel housing; the head pad (outboard layer inside the fork,
    clear of the tyre) is the shank outline within r 70 of the knee;
  * closed PA12 boxes (WALL) braced by plain Warren ribs (RIB, no holes - user 11-01) joined to every wall; the cells
    they close off stay closed voids (11-02, user: no small holes - the per-cell Ø6 powder escapes went; the links are
    to be printed on the user's own printer, an MJF print would keep its unsintered powder inside); no cable holes
    (user 11-01; no wiring in the model); 6061 turned hub inserts in the hip / knee bosses, SUS clamp rings under the stator screws (v4);
  * 11-02 (user): the knee gap (knee plate face -> shank inner face) is KNEE_GAP 4.0 (was 1.5): the wheel group turns
    about a plane WHEEL_SHIFT / 2 further out, so the whole turned wheel group sits 2.5 further outboard (tyre centre
    y 270, robot 594 wide) and the shank keeps a flat 30 mm base layer;
  * 11-02 (user, PETG): the hub inserts are no longer held by heat-set inserts in tension - 6 x ISO 10642 M3 x 16 12.9
    go from the insert flange through a 6 mm PETG wall into a 6061 backing ring (hub_ring) dropped in through the
    Ø44 access bore, so the wall is only clamped (compression); the knee cap tube fits the spar hole (Ø35, ream).
Form (user 11-01: printable; faces that can be flat stay flat - no needless shallow steps, recesses or cuts; round only
where it has to be): every link is a straight prism of the tangent hull of two joint circles - flat side and edge
faces, straight flanks, round only around the joints (and the fork crotch, which relieves the stress); the knee cap
sits flush, the hip access bore is left open (11-02, user: the hip cap only covered it).  Geometry in CAD
coordinates of side s (Builder(side=s)); s = +1 is +y, the robot's right, and the -y leg is the mirror image.
"""
import math

import cadlib as C
import layout as LY
import legs as LG
import params as P

# ----------------------------------------------------------------------------- dimensions (+y side)
Y_TI, Y_TO = P.Y_FEMUR_IN, P.Y_FEMUR_OUT        # 174 .. 204 knee housing (knee motor seat at 195 .. 204)
Y_TF = 178.0                                     # thigh inner face: clear of the knee cup's register lip (y 174 ..
                                                 # 177.1), so the thigh stays deep at the knee; the housing stands 4 proud
WHEEL_SHIFT = 2.5                                # the turned wheel group sits this much further outboard (11-02)
Y_SI4, Y_SO4 = P.Y_TIBIA_IN, P.Y_TIBIA_OUT       # 205.5 .. 235.5 v4 shank layer: the wheel group is built there
Y_MSH = (Y_SI4 + Y_SO4) / 2 + WHEEL_SHIFT / 2    # 221.75 wheel group turn plane
Y_SI, Y_SO = 2 * Y_MSH - Y_SO4, 2 * Y_MSH - Y_SI4   # 208 .. 238 shank base layer (the wheel housing flush in it, the
KNEE_GAP = Y_SI - Y_TO                           # driver cup's lip on its inner end); 4.0 gap to the knee plate
PAD_T = 15.0                                     # shank head pad (inside the fork) on the base layer
Y_SH = Y_SO + PAD_T                              # 253 shank head pad outboard face
SPAR = (Y_SH + 10.5, Y_SH + 35.5)                # outer spar 263.5 .. 288.5, 10.5 outboard of the shank head pad (11-01,
                                                 # user: plan A): the shank goes into the fork held 9.5 mm outboard,
                                                 # clear of the knee plate with its insert boss, spigot and dowels
                                                 # (12.5 proud of its inner face), then slides inboard onto the motor
                                                 # output; robot 594 wide (tyre outer face 297)
# Longer links (user 11-07, 175 -> 250 mm): the joint circles, everything inside them and the fork stay as on the 175 mm link, only the
# distance between the joints grows, so the motors are assembled as before.  The flanks stay one straight line from joint circle to joint
# circle (user 11-07: a constant-depth piece between 175 mm ends left visible kinks in the flanks, rejected).
BASE_LINK = 175.0
STRETCH = P.L1 - BASE_LINK
CROTCH_FACE = 108.0 + STRETCH                    # the slot's faces end 67 mm from the knee (as before the widening)
CROTCH = (CROTCH_FACE - (SPAR[0] - Y_TO) / 2, (SPAR[0] - Y_TO) / 2)   # half-round end of the slot y Y_TO .. SPAR[0]:
                                                 # crotch 96.75 from the knee on the slot centre line, r 29.75 (the wider
                                                 # slot contains the old one, so every old shank clearance stays)
THIGH = [(0.0, 38.0), (P.L1, 30.0)]              # thigh outline: hip circle r 38 -> knee circle r 30 (the spar's end)
R_HIP = 38.0
R_BOSS = 28.0                                    # solid spar boss round the knee cap
SHANK = [(0.0, 45.0), (P.L2, 22.5)]              # shank outline: knee r 45 -> r 22.5 at the axle (inside the wheel
                                                 # housing; the flank stays < 28.3 inside the wheel cup's lip)
PAD_R = 70.0                                     # head pad = the shank outline within r 70 of the knee (concentric: the
                                                 # same crotch / tyre clearance in every pose)
WALL, RIB = 3.5, 3.0                             # rib 2.0 -> 3.0 (user 11-07: thicker for strength)
RIBS = {'thigh': 9, 'tibia': 10}                 # Warren ribs per link: about the 175 mm pitch (thigh 5, shank 6) on the longer span;
                                                 # odd / even as before, so the last rib ends on the same side of each joint
BRG = (35.0, 47.0, 7.0)                          # 61807 knee outer bearing d, D, B
CAP_R = (14.5, 17.5, 23.0)                       # knee cap tube bore / tube OD (in the bearing) / flange
CAP_T = 3.0
CAP_BOLT_R = 20.5                                # 3 x M2.5 into inserts (knee cap; the hip cap went 11-02)
CAP_FIT = 0.0                                    # spar hole radial play on the cap tube: a fit (Ø35 H7 / g6, print small
                                                 # and ream; 11-02 - with 0.5 the 3 cap inserts carried the cap moment)
SPOOL_R = 26.0                                   # tube through the box round the joint (Ø28 bore, Ø44 above the ring)
INSERT_FL = (24.0, 6.0)                          # 6061 hub insert flange radius, thickness (in a pocket at the link face)
WALL_J = 6.0                                     # PETG wall clamped between the insert flange and the backing ring
RING = (14.5, 21.5, 6.0)                         # 6061 backing ring r in, r out, thickness (one part for hip and knee)
BORE_R = 22.0                                    # access bore above the ring (ring drops in with 0.5 play; at the knee
                                                 # the 61807 outer ring sits on the r 22 .. 23.5 shoulder, its inner
                                                 # ring - on the knee cap - runs free over the bore)
FLANGE_BOLT_R = 18.0                             # 6 x ISO 10642 M3 x 16 12.9: flange countersink -> PETG wall -> ring tap
INSERT_CSK = 7.0                                 # countersink Ø7.0 x 90 deg in the flange (head flush), 2.5 to its edge


def wheel_turn(s):
    """Robot 4x4: the outboard-wheel turn of side s (180 deg about z through the shank mid-plane)."""
    return C.rot_about((0.0, s * Y_MSH, 0.0), (0, 0, 1), math.pi)


# the turn mirrors the driver / cup clocking in x: re-clock them about the wheel axis so the cup's notch lies on the
# shank again and the plugs keep their v4 angle to the shank (52 deg off it, now towards +x)
WHEEL_CLOCK = 2 * math.atan2(P.KNEE_X, P.KNEE_Z - P.R_WHEEL)


def wheel_clock(s):
    return C.rot_about((0.0, 0.0, P.R_WHEEL), (0, 1, 0), WHEEL_CLOCK)


def geo(link):
    if link == 'thigh':
        a, b = (0.0, P.Z_HIP), (P.KNEE_X, P.KNEE_Z)
        nb = LG.edge_normal(a, b, LG.FEMUR_EDGE)
    else:
        a, b = (P.KNEE_X, P.KNEE_Z), (0.0, P.R_WHEEL)
        nb = LG.edge_normal(a, b, LG.TIBIA_EDGE)
    (ux, uz), L = LG._axes(a, b)
    n = (-uz, ux)
    assert abs(n[0] + nb[0]) < 1e-9 and abs(n[1] + nb[1]) < 1e-9     # +n = the +x side, nb = the -x side
    return dict(a=a, b=b, u=(ux, uz), n=n, nb=nb, L=L)


def tangent(chain):
    """Tangent points (s, h) of the straight flank of the hull of two circles [(s1, r1), (s2, r2)] on the link axis."""
    (s1, r1), (s2, r2) = chain
    a = (r1 - r2) / (s2 - s1)
    b = math.sqrt(1.0 - a * a)
    return (s1 + r1 * a, r1 * b), (s2 + r2 * a, r2 * b)


def flank(chain, s):
    """Half depth of that flank at s (the straight line, also beyond its tangent points)."""
    (sa, ha), (sb, hb) = tangent(chain)
    return ha + (hb - ha) * (s - sa) / (sb - sa)


def th_half(s):
    return flank(THIGH, s)


def sh_half(s):
    return flank(SHANK, s)


def at(g, s_along, t, nvec=None):
    n = nvec or g['n']
    return (g['a'][0] + g['u'][0] * s_along + n[0] * t, g['a'][1] + g['u'][1] * s_along + n[1] * t)


def x3(p2, y):
    return (p2[0], y, p2[1])


def ccw(poly):
    a = sum(poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1] for i in range(len(poly)))
    return poly if a > 0 else poly[::-1]


# ----------------------------------------------------------------------------- straight solids (tangent hulls)
def disc_hull(B, c1, r1, c2, r2, y0, y1):
    """Prism y0..y1 of the tangent hull of two circles (x, z centres): two discs and the exact quad between their
    tangent points - flat flanks meeting the arcs tangentially (one edge each, as an extruded sketch would)."""
    L = math.hypot(c2[0] - c1[0], c2[1] - c1[1])
    u = ((c2[0] - c1[0]) / L, (c2[1] - c1[1]) / L)
    v = (-u[1], u[0])
    a = (r1 - r2) / L
    b = math.sqrt(1.0 - a * a)
    q = []
    for sg in (1, -1):
        m = (a * u[0] + sg * b * v[0], a * u[1] + sg * b * v[1])
        q += [(c1[0] + r1 * m[0], c1[1] + r1 * m[1]), (c2[0] + r2 * m[0], c2[1] + r2 * m[1])]
    body = B.ycyl(c1[0], c1[1], y0, y1, r1)
    C.union(body, B.ycyl(c2[0], c2[1], y0, y1, r2))
    C.union(body, B.prism(ccw([q[0], q[1], q[3], q[2]]), y0, y1))
    return body


def hull(B, g, chain, y0, y1, inset=0.0):
    """Link outline prism y0..y1 of a two-circle chain on the link axis; inset moves every face in (the cavity)."""
    (s1, r1), (s2, r2) = chain
    return disc_hull(B, at(g, s1, 0.0), r1 - inset, at(g, s2, 0.0), r2 - inset, y0 + inset, y1 - inset)


# ----------------------------------------------------------------------------- outer shapes / cavities
def _slot(B, g, w):
    """Fork slot y Y_TO .. SPAR[0] below the crotch (half-round end, axis along n); w > 0 gives the cavity's keep-out
    (the slot grown by the wall)."""
    s_c, r_c = CROTCH
    L = g['L']
    yc = (Y_TO + SPAR[0]) / 2
    slot = B.prism(ccw([at(g, s_c + r_c, -80.0), at(g, L + 60.0, -80.0), at(g, L + 60.0, 80.0),
                        at(g, s_c + r_c, 80.0)]), Y_TO - w, SPAR[0] + w)
    p0, p1 = at(g, s_c + r_c, -80.0), at(g, s_c + r_c, 80.0)
    C.union(slot, B.cyl(x3(p0, yc), x3(p1, yc), r_c + w))
    return slot


def thigh_shape(B, w=0.0):
    """Outer shape (w = 0) or cavity (w = WALL): one straight hull from the hip to the knee over the whole width, the
    knee housing in the inner tine, the fork slot."""
    g = geo('thigh')
    K = g['b']
    body = hull(B, g, THIGH, Y_TF, SPAR[1], w)
    if w:                                         # the cavity stops at the knee housing and the spar's cap boss
        C.cut(body, B.ycyl(K[0], K[1], Y_TI - 1, Y_TO + 1, P.HOUSING_R))
        C.cut(body, B.ycyl(K[0], K[1], SPAR[0] - 5, SPAR[1] + 1, R_BOSS))
    else:
        C.union(body, B.ycyl(K[0], K[1], Y_TI, Y_TO, P.HOUSING_R))          # knee housing (cup lip fits on Ø88)
    C.cut(body, _slot(B, g, w))
    return body


def shank_shape(B, w=0.0):
    """Outer shape / cavity: straight hull from the knee to the axle over the base layer, the head pad layer (to Y_SH)
    trimmed to r PAD_R about the knee."""
    g = geo('tibia')
    K, W = g['a'], g['b']
    body = hull(B, g, SHANK, Y_SI, Y_SH, w)
    trim = B.abox(-400.0, 400.0, Y_SO - w, Y_SH + 1.0, -400.0, 800.0)
    C.cut(trim, B.ycyl(K[0], K[1], Y_SO - w - 1.0, Y_SH + 2.0, PAD_R - w))
    C.cut(body, trim)
    if w:
        C.cut(body, B.ycyl(W[0], W[1], Y_SI - 1, Y_SH + 1, P.HOUSING_R))
    return body


def truss(B, g, outer, n_ribs, s0, s1, hfun, y0, y1):
    """Warren ribs (RIB thick, y0..y1) between the edge walls (local half depth hfun(s)), clipped to the outer shape."""
    nodes = [(s0 + (s1 - s0) * i / n_ribs, (hfun(s0 + (s1 - s0) * i / n_ribs) - WALL) * (1 if i % 2 == 0 else -1))
             for i in range(n_ribs + 1)]
    ribs = None
    for (sa, na), (sb, nbv) in zip(nodes, nodes[1:]):
        pa, pb = at(g, sa, na), at(g, sb, nbv)
        d = (pb[0] - pa[0], pb[1] - pa[1])
        Ld = math.hypot(*d)
        mid = ((pa[0] + pb[0]) / 2, (y0 + y1) / 2, (pa[1] + pb[1]) / 2)
        ribs = C.union(ribs, B.box(mid, (d[0] / Ld, 0, d[1] / Ld), (0, 1, 0), Ld + 4 * WALL, y1 - y0, RIB))
    C.inter(ribs, outer)
    return ribs, nodes


RIB_SPAN = {'thigh': (R_HIP + 2.0, P.L1 - P.HOUSING_R - 3.0), 'tibia': (34.0, P.L2 - P.HOUSING_R - 3.0)}


def rib_nodes(link):
    """(s, t) nodes of the Warren ribs of a link (as truss() places them; for checks)."""
    s0, s1 = RIB_SPAN[link]
    n_ribs = RIBS[link]
    hfun = th_half if link == 'thigh' else sh_half
    return [(s0 + (s1 - s0) * i / n_ribs, (hfun(s0 + (s1 - s0) * i / n_ribs) - WALL) * (1 if i % 2 == 0 else -1))
            for i in range(n_ribs + 1)]


# ----------------------------------------------------------------------------- joint ends
def y_ring(y_link_in):
    """Backing ring seat (the PETG wall's outboard face) of a joint whose link face is y_link_in."""
    return y_link_in + INSERT_FL[1] + WALL_J


def _flange_bolts(c, f):
    for k in range(6):
        a = math.radians(30 + 60 * k)
        f(c[0] + FLANGE_BOLT_R * math.cos(a), c[1] + FLANGE_BOLT_R * math.sin(a))


def hub_pocket(B, s, body, c, y_link_in, y_face_out):
    """Printed side of a driven joint: SPOOL_R tube through the box, the 6061 insert flange's pocket in the link face,
    the WALL_J wall it is clamped against (Ø28 hole, 6 x Ø3.4 for the flange bolts), then the Ø44 access bore to the
    outer face that takes the backing ring (11-02: no heat-set inserts pulled by the joint moment)."""
    C.union(body, B.ycyl(c[0], c[1], y_link_in, y_face_out, SPOOL_R))
    C.cut(body, B.ycyl(c[0], c[1], y_link_in - 1.0, y_link_in + INSERT_FL[1], INSERT_FL[0] + 0.15))
    C.cut(body, B.ycyl(c[0], c[1], y_link_in - 1.0, y_face_out + 1.0, 14.0))
    yr = y_ring(y_link_in)
    C.cut(body, B.ycyl(c[0], c[1], yr, y_face_out + 1.0, BORE_R))
    _flange_bolts(c, lambda x, z: C.cut(body, B.ycyl(x, z, yr - WALL_J - 0.1, yr + 0.1, LG.M3_CLEAR / 2)))
    return body


def hub_ring(B, c, y_link_in):
    """6061 backing ring on the PETG wall (r RING, 6 x M3 tapped through: tap drill Ø2.5 modelled)."""
    yr = y_ring(y_link_in)
    ring = B.ycyl(c[0], c[1], yr, yr + RING[2], RING[1])
    C.cut(ring, B.ycyl(c[0], c[1], yr - 1, yr + RING[2] + 1, RING[0]))
    _flange_bolts(c, lambda x, z: C.cut(ring, B.ycyl(x, z, yr - 1, yr + RING[2] + 1, P.TAP_DRILL['M3'] / 2)))
    return ring


def hub_insert(B, s, joint, c, y_link_in):
    """6061 turned hub insert: Ø28 boss (+ Ø11.8 x 1.4 spigot) from the motor output to the link, flange inside the
    link head, 6 x M3 x 16 12.9 + washers on a floor BEARING_FACE from the output face (no clamp ring on metal),
    3 x Ø3 dowels 7.5 deep, 6 x Ø3.4 flange bolt holes countersunk in its motor-side face (r 18)."""
    fr = LY.frames(joint, s)
    y_out = P.y_output(joint)
    yt = y_link_in + INSERT_FL[1]
    body = B.ycyl(c[0], c[1], y_out, yt, P.HUB_BOSS_D / 2)
    C.union(body, B.ycyl(c[0], c[1], y_out - 1.4, y_out, 5.9))
    C.union(body, B.ycyl(c[0], c[1], y_link_in, yt, INSERT_FL[0]))
    floor = y_out + P.BEARING_FACE
    C.cut(body, B.ycyl(c[0], c[1], floor, yt + 1, 9.0))
    for p in LY.motor_points(fr['motor'], P.AKE_PCD_OUT / 2, P.AKE_OUT_M3, P.AKE['out']):
        q = LG.lc(p, s)
        C.cut(body, B.ycyl(q[0], q[2], y_out - 2, floor + 1, LG.M3_CLEAR / 2))
        C.cut(body, B.ycyl(q[0], q[2], floor, yt + 1, 3.6))
    for p in LY.motor_points(fr['motor'], P.AKE_PCD_OUT / 2, P.AKE_OUT_PIN, P.AKE['out']):
        q = LG.lc(p, s)
        C.cut(body, B.ycyl(q[0], q[2], y_out - 2, y_out + P.DOWEL_HOLE, 1.5))
    csk = (INSERT_CSK - LG.M3_CLEAR) / 2                 # 90 deg countersink depth
    def bolt(x, z):
        C.cut(body, B.ycyl(x, z, y_link_in - 1, yt + 1, LG.M3_CLEAR / 2))
        C.cut(body, B.ycyl(x, z, y_link_in - 0.5, y_link_in + csk, INSERT_CSK / 2 + 0.5, LG.M3_CLEAR / 2))
    _flange_bolts(c, bolt)
    return body


def _cap_bolts(B, c, y0, y1, d):
    out = None
    for k in range(3):
        a = math.radians(90 + 120 * k)
        out = C.union(out, B.ycyl(c[0] + CAP_BOLT_R * math.cos(a), c[1] + CAP_BOLT_R * math.sin(a), y0, y1, d / 2))
    return out


def cap_seat(B, body, c):
    """Flush cap pocket (r 23.2 x CAP_T) in the thigh's outer face + 3 x M2.5 insert holes."""
    C.cut(body, B.ycyl(c[0], c[1], SPAR[1] - CAP_T, SPAR[1] + 1.0, CAP_R[2] + 0.2))
    C.cut(body, _cap_bolts(B, c, SPAR[1] - CAP_T - 6.0, SPAR[1] - CAP_T + 0.1, LG.INSERT_M25))
    return body


def wheel_housing(B, s):
    """v4 printed wheel seat (9 mm plate, Ø42.05 pocket, clamp ring recess, lead groove) built for the inboard wheel
    and turned with the wheel group (plate now on the outer face), cup inserts turned and re-clocked with the cup."""
    W = (0.0, P.R_WHEEL)
    h = B.ycyl(W[0], W[1], Y_SI4, Y_SO4, P.HOUSING_R)
    LG._motor_seat(B, s, h, 'wheel', W, Y_SI4 + P.WHEEL_PLATE_T, Y_SI4, Y_SO4 + 1)
    C.transform(h, wheel_turn(s))
    for p in LG.cup_column_points('wheel', s):
        q = LG.lc(p, s)
        tool = B.ycyl(q[0], q[2], Y_SO4 + 0.1, Y_SO4 - 5.0, LG.INSERT_M25 / 2)
        C.transform(tool, wheel_turn(s))
        C.transform(tool, wheel_clock(s))
        C.cut(h, tool)
    return h


# ----------------------------------------------------------------------------- links
def thigh(s=1):
    """dict of temporary bodies: main (printed), insert (6061 hub), ring (6061 backing ring).  The hip access bore
    (Ø44 above the ring, for the ring and the hip output screws) stays open in the outer face: no hip cap (11-02)."""
    B = C.Builder(side=s)
    g = geo('thigh')
    H, K, L = g['a'], g['b'], g['L']
    body = thigh_shape(B)
    outer = C.TBM.copy(body)
    s_r0, s_r1 = RIB_SPAN['thigh']
    ribs, nodes = truss(B, g, C.TBM.copy(outer), RIBS['thigh'], s_r0, s_r1, th_half, Y_TF, SPAR[1])
    C.cut(body, thigh_shape(B, WALL))
    C.union(body, ribs)
    hub_pocket(B, s, body, H, Y_TF, SPAR[1])
    LG._motor_seat(B, s, body, 'knee', K, Y_TO - P.KNEE_PLATE_T, Y_TO, Y_TI - 1)
    LG._cup_inserts(B, s, body, 'knee', Y_TI)
    C.cut(body, B.ycyl(K[0], K[1], SPAR[0] - 1, SPAR[1] + 1, CAP_R[1] + CAP_FIT))   # knee cap through the spar
    cap_seat(B, body, K)
    return dict(main=body, insert=hub_insert(B, s, 'hip', H, Y_TF), ring=hub_ring(B, H, Y_TF))


def shank(s=1):
    """dict of temporary bodies: main (printed), insert (6061 hub), ring (6061 backing ring).  The 61807 sits on the
    r 22 .. 23.5 shoulder of its seat over the Ø44 access bore (the ring goes in before the bearing)."""
    B = C.Builder(side=s)
    g = geo('tibia')
    K, W, L = g['a'], g['b'], g['L']
    body = shank_shape(B)
    outer = C.TBM.copy(body)
    s_r0, s_r1 = RIB_SPAN['tibia']
    ribs, nodes = truss(B, g, C.TBM.copy(outer), RIBS['tibia'], s_r0, s_r1, sh_half, Y_SI, Y_SH)
    C.cut(body, shank_shape(B, WALL))
    C.union(body, ribs)
    C.cut(body, B.ycyl(W[0], W[1], Y_SI - 1, Y_SH + 1, P.HOUSING_R))             # wheel housing goes in here
    C.union(body, wheel_housing(B, s))
    hub_pocket(B, s, body, K, Y_SI, Y_SH)
    C.cut(body, B.ycyl(K[0], K[1], Y_SH - BRG[2], Y_SH + 1, BRG[1] / 2))         # 61807 seat in the pad's outer face
    return dict(main=body, insert=hub_insert(B, s, 'knee', K, Y_SI), ring=hub_ring(B, K, Y_SI))


def knee_parts(s=1):
    """61807 bearing (in the shank head pad) and the knee cap (tube in the bearing, flange flush in the spar's outer
    face on 3 x M2.5 into inserts; a 2 mm face closes it, removable for the knee output screws)."""
    B = C.Builder(side=s)
    K = (P.KNEE_X, P.KNEE_Z)
    brg = B.ycyl(K[0], K[1], Y_SH - BRG[2], Y_SH, BRG[1] / 2)
    C.cut(brg, B.ycyl(K[0], K[1], Y_SH - BRG[2] - 1, Y_SH + 1, BRG[0] / 2))
    cap = B.ycyl(K[0], K[1], Y_SH - BRG[2] + 0.5, SPAR[1], CAP_R[1])
    C.union(cap, B.ycyl(K[0], K[1], SPAR[1] - CAP_T, SPAR[1], CAP_R[2]))
    C.cut(cap, B.ycyl(K[0], K[1], Y_SH - BRG[2], SPAR[1] - 2.0, CAP_R[0]))
    C.cut(cap, _cap_bolts(B, K, SPAR[1] - CAP_T - 1, SPAR[1] + 1, LG.M25_CLEAR))
    return dict(bearing=brg, cap=cap)
