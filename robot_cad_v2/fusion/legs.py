"""Designed leg parts (PETG): thigh, shank, two-piece wheel hub and the moteus driver cups.

All parts are written in robot coordinates of the LEFT leg in the nominal pose and mirrored for the
right leg by cadlib.Builder(side=-1).  Hole patterns at motor interfaces come from the real motor
placement of each side (layout.frames), converted back to left-side coordinates with lc().

Print orientation (README): thigh and shank lie on their back edge (joint axes horizontal), so in-plane
and lateral bending of the beam and the bending of the driven bosses all run along the layers.
"""
import math

import cadlib as C
import layout as LY
import params as P

M3_CLEAR, M25_CLEAR = 3.4, 2.8
INSERT_M25, INSERT_M3 = 3.6, 4.0          # heat-set insert holes
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


def _wide(B, a, b, half, y0, y1, end_before_b):
    """Outboard extension of a link (band y0..y1): the head disc at a and the beam a->b up to end_before_b from b,
    full width until WIDE_TAPER before that point, then the outer face ramps back to y0 (in the beam direction)."""
    ux, uz = b[0] - a[0], b[1] - a[1]
    L = math.hypot(ux, uz)
    ux, uz = ux / L, uz / L
    s_end = L - end_before_b
    s_full = s_end - P.WIDE_TAPER
    w = B.ycyl(a[0], a[1], y0, y1, P.HEAD_R)
    C.union(w, _beam(B, a, (a[0] + ux * s_end, a[1] + uz * s_end), half, y0, y1))
    p = (a[0] + ux * s_full, y1, a[1] + uz * s_full)
    dy, ds = y1 - y0, s_end - s_full
    n = (ux * dy, ds, uz * dy)                        # {(q - p) . n <= 0}: before the ramp and inside the band
    return C.inter(w, B.halfspace(p, n))


def _driven_boss(B, s, body, joint, c, y_link_in, y_face_out):
    """Boss reaching through a Ø30 hole to a motor output (output faces +y on the left for hip and knee):
    Ø28 boss, Ø11.8 x 1.4 spigot into the output recess, 6 x M3 x 16 (12.9) on a stainless clamp ring
    (vendor.clamp_ring_out) lying on a floor BEARING_FACE - RING_T from the output face, 3 x Ø3 dowels,
    Ø28 access pocket to the outer face."""
    fr = LY.frames(joint, s)
    y_out = P.y_output(joint)
    C.union(body, B.ycyl(c[0], c[1], y_out, y_link_in + 0.5, P.HUB_BOSS_D / 2))
    C.union(body, B.ycyl(c[0], c[1], y_out - 1.4, y_out, 5.9))
    floor = y_out + BEARING_FACE - P.RING_T
    C.cut(body, B.ycyl(c[0], c[1], floor, y_face_out + 1, 14.0))
    for p in LY.motor_points(fr['motor'], P.AKE_PCD_OUT / 2, P.AKE_OUT_M3, P.AKE['out']):
        q = lc(p, s)
        C.cut(body, B.ycyl(q[0], q[2], y_out - 2, floor + 1, M3_CLEAR / 2))
    for p in LY.motor_points(fr['motor'], P.AKE_PCD_OUT / 2, P.AKE_OUT_PIN, P.AKE['out']):
        q = lc(p, s)
        C.cut(body, B.ycyl(q[0], q[2], y_out - 2, y_out + 7.0, 1.5))
    return body


def _motor_seat(B, s, body, joint, c, y_plate_motor_side, y_plate_far, y_cavity_end):
    """Plate holding a motor ring (pocket on the motor side), Ø30 hole, 6 x M2.5 low-head screws from
    the far side through a stainless clamp ring (vendor.clamp_ring_stator) in a recess, lead groove (params.LEAD_GROOVE_*,
    2 deep) + axial lead slot at the motor's lead direction, can cavity r 37, M2.5 inserts for the driver cup columns."""
    fr = LY.frames(joint, s)
    m = fr['motor']
    sg = 1 if y_plate_far > y_plate_motor_side else -1          # far side direction along +y (left coords)
    ring = P.Y_RING_HIP if joint == 'hip' else (P.Y_RING_KNEE if joint == 'knee' else P.Y_RING_WHEEL)
    # can cavity
    C.cut(body, B.ycyl(c[0], c[1], y_cavity_end, y_plate_motor_side, P.CAN_CLEAR_R))
    C.cut(body, B.ycyl(c[0], c[1], y_plate_motor_side - sg * 0.01, ring, 21.025))      # Ø42 H7 pilot pocket
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


def _edge_groove(B, body, a, b, half, y_mid, side_n, s0, s1, width=12.0, depth=8.0):
    """Cable groove along a beam edge (side_n = +-1 picks the edge normal)."""
    ux, uz = b[0] - a[0], b[1] - a[1]
    L = math.hypot(ux, uz)
    ux, uz = ux / L, uz / L
    nx, nz = -uz * side_n, ux * side_n
    sm = (s0 + s1) / 2
    cpt = (a[0] + ux * sm + nx * half, y_mid, a[1] + uz * sm + nz * half)
    C.cut(body, B.box(cpt, (ux, 0, uz), (0, 1, 0), s1 - s0, width, 2 * depth))
    # clip inserts (M2.5) beside the groove at three stations (assembly.CLIP_T)
    for t in (0.3, 0.5, 0.7):
        st = s0 + (s1 - s0) * t
        for dy in (-(width / 2 + 3.6), width / 2 + 3.6):
            p0 = (a[0] + ux * st + nx * (half + 1), y_mid + dy, a[1] + uz * st + nz * (half + 1))
            p1 = (a[0] + ux * st + nx * (half - 6), y_mid + dy, a[1] + uz * st + nz * (half - 6))
            C.cut(body, B.cyl(p0, p1, INSERT_M25 / 2))
    return body


def edge_normal(a, b, side_n):
    ux, uz = b[0] - a[0], b[1] - a[1]
    L = math.hypot(ux, uz)
    return (-uz / L * side_n, ux / L * side_n)


# side of the beam edge facing the knee wedge (behind the knee): thigh lower-back edge, shank upper-back edge
FEMUR_EDGE = -1
TIBIA_EDGE = -1
GROOVE_DEPTH = 8.0
FEMUR_GROOVE = (42.0, P.L1 - 50.0)
TIBIA_GROOVE = (60.0, P.L2 - 52.0)


def femur(s):
    B = C.Builder(side=s)
    H, K = (0.0, P.Z_HIP), (P.KNEE_X, P.KNEE_Z)
    yi, yo = P.Y_FEMUR_IN, P.Y_FEMUR_OUT
    body = B.ycyl(H[0], H[1], yi, yo, P.HEAD_R)
    C.union(body, B.ycyl(K[0], K[1], yi, yo, P.HOUSING_R))
    C.union(body, _beam(B, H, K, P.FEMUR_H / 2, yi, yo))
    C.union(body, _wide(B, H, K, P.FEMUR_H / 2, yo - 0.5, P.FEMUR_WIDE_OUT, P.FEMUR_WIDE_END))
    _driven_boss(B, s, body, 'hip', H, yi, P.FEMUR_WIDE_OUT)
    _motor_seat(B, s, body, 'knee', K, yo - P.KNEE_PLATE_T, yo, yi - 1)
    _cup_inserts(B, s, body, 'knee', yi)
    _edge_groove(B, body, H, K, P.FEMUR_H / 2, (yi + yo) / 2, FEMUR_EDGE, *FEMUR_GROOVE)
    return body


def tibia(s):
    B = C.Builder(side=s)
    K, W = (P.KNEE_X, P.KNEE_Z), (0.0, P.R_WHEEL)
    yi, yo = P.Y_TIBIA_IN, P.Y_TIBIA_OUT
    body = B.ycyl(K[0], K[1], yi, yo, P.HEAD_R)
    C.union(body, B.ycyl(W[0], W[1], yi, yo, P.HOUSING_R))
    C.union(body, _beam(B, K, W, P.TIBIA_H / 2, yi, yo))
    C.union(body, _wide(B, K, W, P.TIBIA_H / 2, yo - 0.5, P.TIBIA_WIDE_OUT, P.TIBIA_WIDE_END))
    _driven_boss(B, s, body, 'knee', K, yi, P.TIBIA_WIDE_OUT)
    _motor_seat(B, s, body, 'wheel', W, yi + P.WHEEL_PLATE_T, yi, yo + 1)
    _cup_inserts(B, s, body, 'wheel', yo)
    _edge_groove(B, body, K, W, P.TIBIA_H / 2, (yi + yo) / 2, TIBIA_EDGE, *TIBIA_GROOVE)
    return body


# ----------------------------------------------------------------------------- wheel hub (two pieces)
HUB_SCREWS = [math.radians(30 + 60 * k) for k in range(6)]     # hex hub -> flange: 6 x M3 into heat-set inserts, r 18


def hub_levels():
    """Robot y (left) of the hub stations, from the motor output towards the robot centre."""
    y_out = P.y_output('wheel')                    # output face, faces -y
    y_plate = P.Y_TIBIA_IN                        # shank plate inner face
    f1 = y_plate - 1.0                            # flange outer face (1 mm from the plate / screw heads)
    f0 = f1 - 8.0                                 # flange inner face = hex hub flange face
    h0 = f0 - 6.0                                 # hex hub flange inner face
    y_tc = P.tyre_centre_y()
    seat = y_tc + P.WHEEL_SEAT                    # wheel frame Z points to -y on the left: y = seat - Z
    hex1 = seat + P.HEX_L + 2.0
    return dict(out=y_out, plate=y_plate, f1=f1, f0=f0, h0=h0, seat=seat, hex1=hex1, tc=y_tc)


def hub_flange(s):
    B = C.Builder(side=s)
    W = (0.0, P.R_WHEEL)
    lv = hub_levels()
    fr = LY.frames('wheel', s)
    y_out = lv['out']
    body = B.ycyl(W[0], W[1], lv['f1'] - 0.5, y_out, P.HUB_BOSS_D / 2)
    C.union(body, B.ycyl(W[0], W[1], y_out, y_out + 1.4, 5.9))
    C.union(body, B.ycyl(W[0], W[1], lv['f0'], lv['f1'], 23.0))
    bear = y_out - BEARING_FACE
    for p in LY.motor_points(fr['motor'], P.AKE_PCD_OUT / 2, P.AKE_OUT_M3, P.AKE['out']):
        q = lc(p, s)
        C.cut(body, B.ycyl(q[0], q[2], lv['f0'] - 1, y_out + 2, M3_CLEAR / 2))
        C.cut(body, B.ycyl(q[0], q[2], lv['f0'] - 1, bear, 3.75))
    for p in LY.motor_points(fr['motor'], P.AKE_PCD_OUT / 2, P.AKE_OUT_PIN, P.AKE['out']):
        q = lc(p, s)
        C.cut(body, B.ycyl(q[0], q[2], y_out - 7.0, y_out + 2, 1.5))
    C.cut(body, B.ycyl(W[0], W[1], lv['f0'] - 1, lv['f0'] + 7.0, 4.75))          # M5 head pocket
    C.cut(body, B.ycyl(W[0], W[1], lv['f0'] - 1, lv['f0'] + 1.6, 10.1))          # register recess
    for a in HUB_SCREWS:
        C.cut(body, B.ycyl(18 * math.cos(a), W[1] + 18 * math.sin(a), lv['f0'] - 1, lv['f0'] + 6.0, INSERT_M3 / 2))
    return body


def hex_hub(s):
    B = C.Builder(side=s)
    W = (0.0, P.R_WHEEL)
    lv = hub_levels()
    body = B.ycyl(W[0], W[1], lv['h0'], lv['f0'], 23.0)
    C.union(body, B.ycyl(W[0], W[1], lv['f0'], lv['f0'] + 1.5, 10.0))            # register spigot
    C.union(body, B.ycyl(W[0], W[1], lv['hex1'], lv['h0'] + 0.5, 15.0))
    r = P.HEX_AF / 2 / math.cos(math.radians(30))
    poly = [(r * math.cos(math.radians(30 + 60 * k)), W[1] + r * math.sin(math.radians(30 + 60 * k))) for k in range(6)]
    C.union(body, B.prism(poly, lv['seat'], lv['hex1'] + 0.5))
    C.cut(body, B.ycyl(W[0], W[1], lv['seat'] - 1, lv['f0'] + 3, 2.65))            # M5 axle bolt
    for a in HUB_SCREWS:
        x, z = 18 * math.cos(a), W[1] + 18 * math.sin(a)
        C.cut(body, B.ycyl(x, z, lv['h0'] - 1, lv['f0'] + 1, M3_CLEAR / 2))
        C.cut(body, B.ycyl(x, z, lv['h0'] - 1, lv['h0'] + 3.5, 3.25))
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
