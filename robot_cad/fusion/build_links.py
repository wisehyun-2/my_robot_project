"""Step 2: adapt the upkie links to the AKE outputs and build the designed leg parts.

All leg parts are modelled in the coordinates of the link that carries them
(femur_link = upkie upper_leg frame, tibia_link = upkie lower_leg frame,
wheel_link = tibia frame moved to the axle). Local +z points inboard.
Hole patterns at every motor interface are generated from the motor placement,
so they line up with the CubeMars STEP geometry.
"""
import math

import adsk.core
import adsk.fusion

import build_setup as S
import cadlib as C
import params as P

B = C.Builder()                    # part-frame builder (mm)

# motor hole patterns (motor local angle a = atan2(z, x), degrees)
AKE50_OUT_M3 = (-150, -90, -30, 30, 90, 150)
AKE50_OUT_PIN = (-120, 0, 120)
AKE50_RING_M25 = (-120, -60, 0, 60, 120, 180)
AKE50_RING_PIN = (-90, 30, 150)
AKE50_LEAD = -90                   # phase leads leave the stator at r 24, 3 mm past the ring face
AKE60_OUT_M3 = (-120, -60, 0, 60, 120, 180)
AKE60_OUT_PIN = (-150, -30, 90)
AKE60_RING_M25 = (-150, -90, -30, 30, 90, 150)
AKE60_RING_PIN = ()                # the AKE60 ring has no dowel holes (Ø2.1 through holes instead)


def motor_points(m_motor, r, angles, y=0.0):
    """Points of a motor-local hole circle mapped through m_motor (to the parent frame)."""
    out = []
    for a in angles:
        t = math.radians(a)
        out.append(C.apply(m_motor, (r * math.cos(t), y, r * math.sin(t))))
    return out


def to_local(m_parent_inv, pts):
    return [C.apply(m_parent_inv, p) for p in pts]


def zhole(body, x, y, z0, z1, d):
    C.cut(body, B.cyl((x, y, z0), (x, y, z1), d / 2))


def zcyl(x, y, z0, z1, r):
    return B.cyl((x, y, z0), (x, y, z1), r)


# ----------------------------------------------------------------------------- upkie link bosses
def replace_boss(comp, body_name, boss_len, holes, pins, cb_depth_from_face):
    """Cut the qdd100 boss at (0, 60), fill its screw holes, add a Ø26 AKE boss and the hole pattern.

    holes/pins: (x, y) in the link frame. Screws go in from the outer face (z 106)."""
    body = comp.bRepBodies.item(0)
    t = C.copy(body)
    C.cut(t, zcyl(0, 60, 131.0, 151.0, 23.0))                 # old boss (Ø44 / Ø40 / Ø39)
    C.union(t, zcyl(0, 60, 106.0, 131.0, 20.0))               # fill the qdd100 screw holes
    zf = 131.0 + boss_len
    C.union(t, zcyl(0, 60, 131.0, zf, P.HUB_BOSS_D / 2))      # new boss reaching the motor output
    for (x, y) in holes:
        zhole(t, x, y, 104.0, zf + 1.0, 3.4)                  # M3 clearance
        zhole(t, x, y, 104.0, zf - cb_depth_from_face, 6.5)   # socket head counterbore
    for (x, y) in pins:
        zhole(t, x, y, zf - 6.0, zf + 1.0, 3.0)               # Ø3 dowel pins
    new = comp.bRepBodies.add(t)
    new.name = body_name
    body.deleteMe()
    new.material = C.material('PA-CF print')
    new.appearance = C.appearance('print black')
    return new


def adapt_upkie_links():
    torso_m = S.torso_frame()
    femur_m, tibia_m = S.femur_frame(), S.tibia_frame()
    # hip: AKE50 output (torso) -> femur frame
    hip_w = C.mmul(torso_m, S.hip_motor_in_torso())
    fin = C.minv(femur_m)
    h = to_local(fin, motor_points(hip_w, 10.0, AKE50_OUT_M3, P.AKE50['out']))
    p = to_local(fin, motor_points(hip_w, 10.0, AKE50_OUT_PIN, P.AKE50['out']))
    boss_f = P.femur_z(P.YW - P.HIP_PLATE_T + P.HIP_POCKET + (P.AKE50['ring'] - P.AKE50['out'])) - 131.0
    upper = C.find_comp('upkie_upper_leg_AKE')
    replace_boss(upper, 'upper_leg (upkie 2.0, AKE50 hip boss)', boss_f, [q[:2] for q in h], [q[:2] for q in p], 13.0)
    # knee: AKE60 output (femur) -> tibia frame
    knee_w = C.mmul(femur_m, S.knee_motor_in_femur())
    tin = C.minv(tibia_m)
    h = to_local(tin, motor_points(knee_w, 10.0, AKE60_OUT_M3, P.AKE60['out']))
    p = to_local(tin, motor_points(knee_w, 10.0, AKE60_OUT_PIN, P.AKE60['out']))
    out_y = P.Y_ADAPTER_IN + P.ADAPTER_POCKET - (P.AKE60['out'] - P.AKE60['ring'])
    boss_t = P.tibia_z(out_y) - 131.0
    lower = C.find_comp('upkie_lower_leg_AKE')
    replace_boss(lower, 'lower_leg (upkie 2.0, AKE60 knee boss)', boss_t, [q[:2] for q in h], [q[:2] for q in p], 13.0)
    print('femur boss %.2f mm, tibia boss %.2f mm' % (boss_f, boss_t))


# ----------------------------------------------------------------------------- moteus-c1 board
def build_c1():
    """moteus-c1 (38 x 38 x 9 mm, 8.9 g). Local frame: encoder chip face at z = 0 facing -z."""
    comp = C.find_comp('moteus_c1') or C.master('moteus_c1')
    C.clear_bodies(comp)
    pcb = B.box((0, 0, 1.8), (1, 0, 0), (0, 1, 0), 38.0, 38.0, 1.6)
    for sx in (-1, 1):
        for sy in (-1, 1):
            C.cut(pcb, B.box((sx * 19, sy * 19, 1.8), (1, 1, 0), (-1, 1, 0), 4.0, 4.0, 3.0))   # corner chamfers
            C.cut(pcb, zcyl(sx * 15, sy * 15, 0, 4, 1.1))
    C.add_body(comp, pcb, 'pcb', 'FR4 PCB', 'pcb black')
    parts = C.copy(B.box((0, 0, 0.5), (1, 0, 0), (0, 1, 0), 4.0, 4.0, 1.0))                     # encoder IC
    C.union(parts, B.box((0, -8, 2.6 + 0.5), (1, 0, 0), (0, 1, 0), 7.0, 7.0, 1.0))           # MCU
    for x in (-8, 0, 8):
        C.union(parts, B.box((x, 8, 2.6 + 1.2), (1, 0, 0), (0, 1, 0), 5.0, 6.0, 2.4))      # MOSFETs
    for x in (-12, 12):
        C.union(parts, zcyl(x, -2, 2.6, 8.4, 3.15))                                          # bulk caps
    C.add_body(comp, parts, 'components', 'FR4 PCB', 'print grey')
    conn = B.box((-10, -15.5, 2.6 + 3.0), (1, 0, 0), (0, 1, 0), 8.0, 4.5, 6.0)               # CAN (JST PH-3)
    C.union(conn, B.box((10, -15.5, 2.6 + 3.0), (1, 0, 0), (0, 1, 0), 8.0, 4.5, 6.0))
    C.add_body(comp, conn, 'connectors', 'PETG print', 'connector white')
    return comp


def c1_matrix_for_motor(m_motor, magnet_y, gap=1.5):
    """c1 frame: chip face 'gap' behind the motor's magnet face, local +z away from the motor."""
    ax = (m_motor[0][1], m_motor[1][1], m_motor[2][1])          # motor local +y (towards the back)
    o = C.apply(m_motor, (0.0, magnet_y + gap, 0.0))
    ex = (m_motor[0][0], m_motor[1][0], m_motor[2][0])
    ez = ax
    ey = C.cross(ez, ex)
    return C.frame(o, ex, ey, ez)


# ----------------------------------------------------------------------------- femur link parts
def knee_adapter(comp):
    kc = (0.0, -115.0)
    z_in = P.femur_z(P.Y_ADAPTER_IN)             # 123.7 (touches the femur knee loop)
    z_out = P.femur_z(P.Y_ADAPTER_OUT)           # 115.7
    km = S.knee_motor_in_femur()
    ring_z = z_in - P.ADAPTER_POCKET
    b = zcyl(kc[0], kc[1], z_out, z_in, 46.0)
    C.cut(b, B.box((0, -71.0 + 50, (z_in + z_out) / 2), (1, 0, 0), (0, 1, 0), 120, 100, 20))   # clear the femur crank
    C.cut(b, zcyl(kc[0], kc[1], ring_z, z_in + 1, 21.025))          # Ø42 H7 pilot pocket
    C.cut(b, zcyl(kc[0], kc[1], z_out - 1, z_in + 1, 15.0))         # Ø30 output clearance
    for (x, y, _z) in motor_points(km, 18.25, AKE60_RING_M25):
        zhole(b, x, y, z_out - 1, ring_z + 0.5, 2.8)                 # M2.5 clearance
        zhole(b, x, y, z_out - 1, z_out + 2.8, 5.0)                  # head counterbore
    for a in (0, -45, -90, -135, 180):                               # upkie femur ring holes (R40.5)
        x, y = kc[0] + 40.5 * math.cos(math.radians(a)), kc[1] + 40.5 * math.sin(math.radians(a))
        zhole(b, x, y, z_in - 7.0, z_in + 1, 2.5)                    # M3 tapped, 7 deep (aluminium)
    for a in (-112.5, -67.5, 157.5, 22.5, 202.5, -22.5):             # lightening holes
        x, y = kc[0] + 30.5 * math.cos(math.radians(a)), kc[1] + 30.5 * math.sin(math.radians(a))
        if y < -78:
            zhole(b, x, y, z_out - 1, z_in + 1, 7.0)
    C.add_body(comp, b, 'knee_adapter_AKE60', 'Al6061', 'alu dark')


def knee_cover(comp):
    kc = (0.0, -115.0)
    z_f = 148.7                                  # femur knee loop inboard face
    km = S.knee_motor_in_femur()
    magnet_z = C.apply(km, (0, P.AKE60['back'], 0))[2]
    chip_z = magnet_z + 1.5
    cup_in = chip_z + 9.0 + 1.0
    b = zcyl(kc[0], kc[1], z_f, z_f + 3.0, 46.0)
    C.cut(b, B.box((0, -71.0 + 50, z_f + 1.5), (1, 0, 0), (0, 1, 0), 120, 100, 10))
    C.union(b, zcyl(kc[0], kc[1], z_f, cup_in + 2.5, 31.0))
    C.cut(b, zcyl(kc[0], kc[1], z_f - 1, cup_in, 28.5))
    C.cut(b, B.box((0, kc[1] + 30, z_f + 4.0), (1, 0, 0), (0, 1, 0), 14, 12, 8))     # cable / lead slot
    for a in (0, -45, -90, -135, 180):
        x, y = kc[0] + 40.5 * math.cos(math.radians(a)), kc[1] + 40.5 * math.sin(math.radians(a))
        zhole(b, x, y, z_f - 1, z_f + 4, 3.4)
        zhole(b, x, y, z_f + 1.5, z_f + 4, 6.5)
    for sx in (-1, 1):
        for sy in (-1, 1):
            C.union(b, zcyl(kc[0] + sx * 15, kc[1] + sy * 15, cup_in - 1.0, cup_in + 0.5, 2.5))
    C.add_body(comp, b, 'knee_cover', 'PA-CF print', 'accent orange')
    return chip_z


# ----------------------------------------------------------------------------- tibia link parts
def ankle_outline(z0, z1, r_circle):
    b = zcyl(0, -115.0, z0, z1, r_circle)
    C.union(b, B.rbox((0, -120.0, (z0 + z1) / 2), (1, 0, 0), (0, 1, 0), 88.0, 32.0, z1 - z0, 6.0))
    return b


def ankle_plate(comp):
    wm = S.wheel_motor_in_tibia()
    z_out = 131.0                                # tibia inboard face (plate outboard face)
    z_in = z_out + P.ANKLE_T
    ring_z = z_out + P.ANKLE_POCKET
    b = ankle_outline(z_out, z_in, 30.0)
    C.cut(b, zcyl(0, -115, z_out - 1, ring_z, 21.025))
    C.cut(b, zcyl(0, -115, z_out - 1, z_in + 1, 15.0))
    for (x, y, _z) in motor_points(wm, 18.25, AKE50_RING_M25):
        zhole(b, x, y, ring_z - 0.5, z_in + 1, 2.8)
        zhole(b, x, y, z_in - 2.8, z_in + 1, 5.0)
    for (x, y, _z) in motor_points(wm, 18.25, AKE50_RING_PIN):
        zhole(b, x, y, ring_z - 0.5, ring_z + 4.0, 2.5)
    for x in (-37.0, 37.0):
        for y in (-111.0, -129.0):
            zhole(b, x, y, z_out - 1, z_out + 7.0, 2.5)              # M3 tapped, 7 deep (prong screws)
    (lx, ly, _z) = motor_points(wm, 24.0, (AKE50_LEAD,))[0]
    d = C.unit((lx, ly + 115.0, 0.0))
    C.cut(b, B.box((lx + d[0] * 10, ly + d[1] * 10, z_out + 2.5), d, (-d[1], d[0], 0), 32, 9, 7))  # lead groove
    C.add_body(comp, b, 'ankle_plate_AKE50', 'Al6061', 'alu dark')


def wheel_cover(comp):
    wm = S.wheel_motor_in_tibia()
    z_t = 106.0                                  # tibia outer face
    magnet_z = C.apply(wm, (0, P.AKE50['back'], 0))[2]
    chip_z = magnet_z - 1.5
    cup_in = chip_z - 9.0 - 1.0
    b = ankle_outline(z_t - 3.0, z_t, 34.0)
    C.union(b, zcyl(0, -115, cup_in - 2.5, z_t, 31.0))
    C.cut(b, zcyl(0, -115, cup_in, z_t + 1, 28.5))
    C.cut(b, B.box((0, -115 + 30, z_t - 4.0), (1, 0, 0), (0, 1, 0), 14, 12, 8))
    for x in (-37.0, 37.0):
        for y in (-111.0, -129.0):
            zhole(b, x, y, z_t - 4, z_t + 1, 3.4)
            zhole(b, x, y, z_t - 4, z_t - 1.5, 6.5)
    for sx in (-1, 1):
        for sy in (-1, 1):
            C.union(b, zcyl(sx * 15, -115 + sy * 15, cup_in - 0.5, cup_in + 1.0, 2.5))
    C.add_body(comp, b, 'wheel_cover', 'PA-CF print', 'accent orange')


# ----------------------------------------------------------------------------- wheel link parts
def hub(comp, wm_wheel):
    z_face = C.apply(wm_wheel, (0, P.AKE50['out'], 0))[2]             # 135.5: motor output face
    z_f0 = z_face + 4.0
    z_f1 = z_f0 + P.HUB_FLANGE_T
    b = zcyl(0, 0, z_face, z_f0, P.HUB_BOSS_D / 2)
    C.union(b, zcyl(0, 0, z_f0, z_f1, P.HUB_FLANGE_D / 2))
    C.cut(b, zcyl(0, 0, z_face - 1, z_f1 + 1, 4.0))
    for (x, y, _z) in motor_points(wm_wheel, 10.0, AKE50_OUT_M3):
        zhole(b, x, y, z_face - 1, z_f1 + 1, 3.4)
        zhole(b, x, y, z_f1 - 4.0, z_f1 + 1, 6.5)
    for (x, y, _z) in motor_points(wm_wheel, 10.0, AKE50_OUT_PIN):
        zhole(b, x, y, z_face - 1, z_face + 5.0, 3.0)
    for k in range(6):
        a = math.radians(30 + 60 * k)
        zhole(b, 25 * math.cos(a), 25 * math.sin(a), z_f0 - 1, z_f1 + 1, 3.3)   # M4 tapped (rim screws)
    C.add_body(comp, b, 'wheel_hub_AKE50', 'Al6061', 'alu')
    return z_f1


def rim(comp, z_disc):
    zc = z_disc                                   # disc against the hub flange
    z_lo = P.tibia_z(P.Y_ANKLE_IN - 1.8)          # tyre outboard edge, 1.8 mm inboard of the ankle plate
    z_hi = z_lo + P.TYRE_W
    r_seat = P.TYRE_ID / 2
    b = zcyl(0, 0, zc, zc + P.RIM_DISC_T, 34.0)
    C.cut(b, zcyl(0, 0, zc - 1, zc + P.RIM_DISC_T + 1, 15.0))
    band = zcyl(0, 0, z_lo, z_hi, r_seat)
    C.cut(band, zcyl(0, 0, z_lo - 1, z_hi + 1, P.RIM_BAND_ID / 2))
    C.union(b, band)
    for z0 in (z_lo, z_hi - 2.5):
        fl = zcyl(0, 0, z0, z0 + 2.5, P.RIM_FLANGE_OD / 2)
        C.cut(fl, zcyl(0, 0, z0 - 1, z0 + 3.5, r_seat - 0.5))
        C.union(b, fl)
    zs = zc + 6.0
    for k in range(6):
        a = math.radians(60 * k)
        u = (math.cos(a), math.sin(a), 0.0)
        mid_r = (32.0 + P.RIM_BAND_ID / 2) / 2
        tilt = math.radians(14)
        ud = (math.cos(a + tilt), math.sin(a + tilt), 0.0)
        c = (u[0] * mid_r, u[1] * mid_r, zs)
        C.union(b, B.box(c, ud, (-ud[1], ud[0], 0.0), 36.0, 8.0, 12.0))
    C.cut(b, zcyl(0, 0, zc - 10, zc + 30, 15.0))
    for k in range(6):
        a = math.radians(30 + 60 * k)
        x, y = 25 * math.cos(a), 25 * math.sin(a)
        zhole(b, x, y, zc - 1, zc + 20, 4.5)
        zhole(b, x, y, zc + 3.0, zc + 20, 8.0)
    rim_copy = C.copy(b)
    C.add_body(comp, b, 'wheel_rim_150', 'PA-CF print', 'print grey')
    return z_lo, z_hi, rim_copy


def tyre(comp, z_lo, z_hi, rim_body=None):
    ri, ro = P.TYRE_ID / 2, P.TYRE_OD / 2
    rr = 4.0
    t = zcyl(0, 0, z_lo, z_hi, ro - rr)
    C.cut(t, zcyl(0, 0, z_lo - 1, z_hi + 1, ri))
    outer = zcyl(0, 0, z_lo + rr, z_hi - rr, ro)
    C.cut(outer, zcyl(0, 0, z_lo, z_hi, ro - rr - 0.5))
    C.union(t, outer)
    for zt in (z_lo + rr, z_hi - rr):
        # createTorus ignored the centre's z here, so build it at the origin and move it
        tor = C.TBM.createTorus(adsk.core.Point3D.create(0, 0, 0), adsk.core.Vector3D.create(0, 0, 1),
                                (ro - rr) * C.CM, rr * C.CM)
        mv = adsk.core.Matrix3D.create()
        mv.translation = adsk.core.Vector3D.create(0, 0, zt * C.CM)
        C.TBM.transform(tor, mv)
        C.union(t, tor)
    if t.lumps.count != 1:
        raise RuntimeError('tyre has %d lumps' % t.lumps.count)
    zc = (z_lo + z_hi) / 2
    for k in range(40):                           # transverse tread sipes
        a = math.radians(9 * k)
        u = (math.cos(a), math.sin(a), 0.0)
        c = (u[0] * ro, u[1] * ro, zc + (5.0 if k % 2 else -5.0))
        C.cut(t, B.box(c, u, (-u[1], u[0], 0.0), 4.0, 3.0, 14.0))
    groove = zcyl(0, 0, zc - 1.2, zc + 1.2, ro + 1)
    C.cut(groove, zcyl(0, 0, zc - 2, zc + 2, ro - 1.8))
    C.cut(t, groove)
    if rim_body is not None:
        C.cut(t, rim_body)                        # bead seats for the rim flanges
    C.add_body(comp, t, 'tyre_150_TPU', 'TPU 95A', 'rubber')


# ----------------------------------------------------------------------------- main
def part_comp(link_occ, name):
    """Component directly inside a link (occurrence at the identity, link frame coordinates)."""
    with C.IdentityParent(link_occ):
        occ = link_occ.component.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ.component


def main():
    adapt_upkie_links()
    c1 = build_c1()

    femur = C.root_occ('femur_link:1')
    tibia = C.root_occ('tibia_link:1')
    wheel = C.root_occ('wheel_link:1')
    torso = C.root_occ('torso:1')

    knee_adapter(part_comp(femur, 'knee_adapter_AKE60'))
    knee_cover(part_comp(femur, 'knee_cover'))
    ankle_plate(part_comp(tibia, 'ankle_plate_AKE50'))
    wheel_cover(part_comp(tibia, 'wheel_cover'))

    wm_wheel = C.mmul(C.minv(S.wheel_frame()), C.mmul(S.tibia_frame(), S.wheel_motor_in_tibia()))
    z_disc = hub(part_comp(wheel, 'wheel_hub_AKE50'), wm_wheel)
    z_lo, z_hi, rim_body = rim(part_comp(wheel, 'wheel_rim_150'), z_disc)
    tyre(part_comp(wheel, 'tyre_150_TPU'), z_lo, z_hi, rim_body)

    # motor drivers on the motor backs
    C.instance(femur, c1, C.to_matrix3d(c1_matrix_for_motor(S.knee_motor_in_femur(), P.AKE60['back'])))
    C.instance(tibia, c1, C.to_matrix3d(c1_matrix_for_motor(S.wheel_motor_in_tibia(), P.AKE50['back'])))
    C.instance(torso, c1, S.torso_child_matrix(c1_matrix_for_motor(S.hip_motor_in_torso(), P.AKE50['back'])))
    print('tyre z %.1f..%.1f (robot y %.1f..%.1f)' % (z_lo, z_hi, P.Y_TIBIA_IN + 131 - z_hi, P.Y_TIBIA_IN + 131 - z_lo))
