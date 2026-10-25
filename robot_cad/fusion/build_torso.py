"""Step 3: torso (shell, hip plates, spine, battery, Raspberry Pi 4B + pi3hat, power board).

Torso component frame: robot axes (x forward, y left, z up) with the origin at the
midpoint of the two hip axes; geometry is written in robot coordinates (mm) and
converted to Fusion's Y-up axes by cadlib.Builder(robot=True).

Layout (DIABLO / Ascento style capsule):
  * the bottom of the shell is a half cylinder (R66) about the hip axis, so the upkie
    femur, which moves inboard of the hip plate from 76 mm below the hip centre on,
    clears the torso at every hip angle up to about +-80 deg;
  * hip motors sit inside 8 mm aluminium hip plates flush with the side faces,
    the two plates are tied by two Ø16 aluminium tubes that also carry the battery;
  * the 24 V 15 Ah battery (75 x 310 x 75) lies across the robot right above the hip
    axis; Raspberry Pi 4B + pi3hat and the power board sit on a sled under it.
"""
import math

import adsk.core
import adsk.fusion

import build_links as L
import build_setup as S
import cadlib as C
import params as P

R = C.Builder(robot=True)
RZ180 = C.frame((0, 0, 0), (-1, 0, 0), (0, -1, 0), (0, 0, 1))      # left -> right (rigid)


def ycyl(x, z, y0, y1, r):
    return R.cyl((x, y0, z), (x, y1, z), r)


def xcyl(y, z, x0, x1, r):
    return R.cyl((x0, y, z), (x1, y, z), r)


def zcyl(x, y, z0, z1, r):
    return R.cyl((x, y, z0), (x, y, z1), r)


def box(c, lx, ly, lz):
    return R.box(c, (1, 0, 0), (0, 1, 0), lx, ly, lz)


def mirror_y(pts):
    return [(p[0], -p[1], p[2]) for p in pts]


def comp_in_torso(torso, name):
    with C.IdentityParent(torso):
        occ = torso.component.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ.component


# ----------------------------------------------------------------------------- hip plate / cage / tube
def hip_plate():
    comp = C.master('hip_plate_AKE50')
    hm = S.hip_motor_in_torso()
    y_in = P.YW - P.HIP_PLATE_T                     # 170
    y_ring = y_in + P.HIP_POCKET                    # 173
    b = ycyl(0, 0, y_in, P.YW - 3.0, 60.0)          # flange (behind the shell side wall)
    # chamfer the flange's outboard edge so it stays clear of the shell's inner fillet (R9)
    keep = ycyl(0, 0, y_in - 1, y_in + 2.0, 61.0)
    C.union(keep, R.cyl((0, y_in + 2.0, 0), (0, P.YW - 3.0, 0), 60.0, 53.8))
    C.inter(b, keep)
    C.union(b, ycyl(0, 0, P.YW - 3.0, P.YW, 49.5))   # boss, flush with the shell side face
    C.cut(b, ycyl(0, 0, y_in - 1, y_ring, 21.025))  # Ø42 H7 pilot pocket
    C.cut(b, ycyl(0, 0, y_in - 1, P.YW + 1, 15.0))  # Ø30 output clearance
    for (x, y, z) in L.motor_points(hm, 18.25, L.AKE50_RING_M25):
        C.cut(b, ycyl(x, z, y_ring - 0.5, P.YW + 1, 1.4))
        C.cut(b, ycyl(x, z, P.YW - 2.8, P.YW + 1, 2.5))
    for (x, y, z) in L.motor_points(hm, 18.25, L.AKE50_RING_PIN):
        C.cut(b, ycyl(x, z, y_ring - 0.5, y_ring + 4.0, 1.25))
    (lx, _ly, lz) = L.motor_points(hm, 24.0, (L.AKE50_LEAD,))[0]
    C.cut(b, R.box((lx, y_in + 2.5, lz + 13.0), (0, 0, 1), (1, 0, 0), 32.0, 9.0, 7.0))   # lead groove (up)
    for a in (0, 180, -60, -120):                    # hip cage screws, M3 tapped
        t = math.radians(a)
        C.cut(b, ycyl(38.5 * math.cos(t), 38.5 * math.sin(t), y_in - 1, y_in + 6, 1.25))
    for k in range(8):                               # shell screws, M3 tapped from the flange face
        t = math.radians(22.5 + 45 * k)
        C.cut(b, ycyl(57 * math.cos(t), 57 * math.sin(t), y_in + 0.5, P.YW - 2.9, 1.25))
    for sx in (-1, 1):                               # spine tube screws (M5 into the tube plugs)
        C.cut(b, ycyl(sx * P.TUBE_X, P.TUBE_Z, y_in - 1, P.YW - 2.9, 2.75))
    for a in (30, 150, -90):                         # lightening pockets on the inner face
        t = math.radians(a)
        C.cut(b, ycyl(33 * math.cos(t), 33 * math.sin(t), y_in - 1, y_in + 4, 6.5))
    C.add_body(comp, b, 'hip_plate_AKE50', 'Al6061', 'alu dark')
    return comp


def hip_cage():
    comp = C.master('hip_cage_c1')
    y_in = P.YW - P.HIP_PLATE_T                     # plate inner face 170
    hm = S.hip_motor_in_torso()
    magnet_y = C.apply(hm, (0, P.AKE50['back'], 0))[1]
    chip_y = magnet_y - 1.5
    bot_in = chip_y - 9.0 - 1.0
    b = ycyl(0, 0, bot_in - 2.5, y_in - 3.0, 36.0)
    C.union(b, ycyl(0, 0, y_in - 3.0, y_in, 41.5))
    C.cut(b, ycyl(0, 0, bot_in, y_in + 1, 33.5))
    for a in (0, 180, -60, -120):
        t = math.radians(a)
        C.cut(b, ycyl(38.5 * math.cos(t), 38.5 * math.sin(t), y_in - 4, y_in + 1, 1.7))
    C.cut(b, R.box((0, bot_in + 9, 34.0), (0, 1, 0), (1, 0, 0), 18.0, 14.0, 10.0))    # lead / cable window
    for sx in (-1, 1):
        for sz in (-1, 1):
            C.union(b, ycyl(sx * 15, sz * 15, bot_in - 0.5, bot_in + 1.0, 2.5))
    C.add_body(comp, b, 'hip_cage_c1', 'PA-CF print', 'print grey')
    return comp


def spine_tube():
    comp = C.master('spine_tube_16x1.5')
    y = P.YW - P.HIP_PLATE_T
    C.add_body(comp, R.tube((0, -y, 0), (0, y, 0), P.TUBE_D / 2, P.TUBE_D / 2 - P.TUBE_WALL), 'tube', 'Al6061', 'alu')
    return comp


# ----------------------------------------------------------------------------- shell
def outer_profile(t, half_w):
    """Side profile (x-z) extruded along y, offset inwards by t."""
    r0 = P.TORSO_R_BOTTOM - t
    x0 = P.TORSO_X - t
    rt = P.TORSO_TOP_R - t
    zt = P.TORSO_TOP - P.TORSO_TOP_R
    xc = P.TORSO_X - P.TORSO_TOP_R
    b = ycyl(0, 0, -half_w, half_w, r0)
    C.union(b, box((0, 0, zt / 2), 2 * x0, 2 * half_w, zt))
    C.union(b, box((0, 0, (P.TORSO_TOP - t) / 2), 2 * xc, 2 * half_w, P.TORSO_TOP - t))
    for sx in (-1, 1):
        C.union(b, ycyl(sx * xc, zt, -half_w, half_w, rt))
    return b


def side_edges(body, y_abs):
    xf = y_abs * C.CM                    # Fusion X = robot y
    out = []
    for e in body.edges:
        bb = e.boundingBox
        if abs(abs(bb.minPoint.x) - xf) < 1e-4 and abs(abs(bb.maxPoint.x) - xf) < 1e-4:
            out.append(e)
    return out


def shell(comp):
    hw = P.YW
    tmp = C.add_body(comp, outer_profile(0.0, hw), 'shell_tmp')
    C.fillet_edges(comp, side_edges(tmp, hw), 12.0)
    tmp = comp.bRepBodies.itemByName('shell_tmp')
    col = adsk.core.ObjectCollection.create()
    col.add(tmp)
    si = comp.features.shellFeatures.createInput(col, False)
    si.insideThickness = adsk.core.ValueInput.createByReal(P.SHELL_T * C.CM)
    comp.features.shellFeatures.add(si)
    tmp = comp.bRepBodies.itemByName('shell_tmp')
    s = C.copy(tmp)
    tmp.deleteMe()

    for sy in (-1, 1):
        y0, y1 = sy * (hw - 5), sy * (hw + 1)
        C.cut(s, ycyl(0, 0, y0, y1, 49.7))                                       # hip opening
        for k in range(8):
            t = math.radians(22.5 + 45 * k)
            C.cut(s, ycyl(57 * math.cos(t), 57 * math.sin(t), y0, y1, 1.7))      # M3 to the hip plate
        for sx in (-1, 1):
            C.cut(s, ycyl(sx * P.TUBE_X, P.TUBE_Z, y0, y1, 2.75))                # M5 to the tubes
        C.cut(s, R.slot((-46, sy * hw, 58), (-34, sy * hw, 58), (0, 1, 0), 5.0, 12.0))   # leg cable port
        inlay = ycyl(0, 0, sy * (hw - 1.0), sy * (hw + 1), 54.0)
        C.cut(inlay, ycyl(0, 0, sy * (hw - 2), sy * (hw + 2), 50.5))
        C.cut(s, inlay)                                                           # groove for the accent ring
    # front: camera / light window with a recess for the dark visor
    C.cut(s, R.rbox((P.TORSO_X, 0, 79), (0, 1, 0), (0, 0, 1), 200.0, 34.0, 12.0, 8.0))
    C.cut(s, R.rbox((P.TORSO_X, 0, 79), (0, 1, 0), (0, 0, 1), 212.0, 46.0, 4.0, 12.0))      # 2 mm visor recess
    # back: vents, power button, charge jack
    for k in range(7):
        y = -90 + 30 * k
        C.cut(s, R.slot((-P.TORSO_X, y, 62), (-P.TORSO_X, y, 90), (1, 0, 0), 2.5, 12.0))
    C.cut(s, xcyl(125, 80, -P.TORSO_X - 2, -P.TORSO_X + 5, 8.0))
    C.cut(s, xcyl(-125, 80, -P.TORSO_X - 2, -P.TORSO_X + 5, 4.0))
    # lid: handle screws
    for sy in (-1, 1):
        C.cut(s, zcyl(0, sy * 66, P.TORSO_TOP - 5, P.TORSO_TOP + 1, 1.7))

    big = 1000.0
    zs = P.LID_SPLIT_Z
    zb = P.BOTTOM_SPLIT_Z
    pieces = {   # name: (keep box, appearance); every piece is printed as a left and a right half
        'bottom_cover': (box((0, 0, zb - big / 2), big, big, big), 'shell grey'),
        'shell_lower': (box((0, 0, (zs + zb) / 2), big, big, zs - zb), 'shell white'),
        'lid': (box((0, 0, (zs + big) / 2), big, big, big - zs), 'shell white'),
    }
    for name, (keep, look) in pieces.items():
        part = C.copy(s)
        C.inter(part, keep)
        for side, sy in (('L', 1), ('R', -1)):
            half = C.copy(part)
            C.inter(half, box((0, sy * big / 2, 0), big, big, big * 2))
            C.add_body(comp, half, '%s_%s' % (name, side), 'PETG print', look)
    # accent rings in the side grooves
    for sy in (-1, 1):
        ring = ycyl(0, 0, sy * (P.YW - 1.0), sy * P.YW, 53.8)
        C.cut(ring, ycyl(0, 0, sy * (P.YW - 2), sy * (P.YW + 1), 50.7))
        C.add_body(comp, ring, 'accent_ring_%s' % ('L' if sy > 0 else 'R'), 'PETG print', 'accent orange')
    # dark visor + light bar behind it
    v = R.rbox((P.TORSO_X - 1.0, 0, 79), (0, 1, 0), (0, 0, 1), 211.6, 45.6, 2.0, 12.0)
    C.add_body(comp, v, 'visor', 'Acrylic', 'visor')
    C.add_body(comp, R.rbox((P.TORSO_X - 6.0, 0, 79), (0, 1, 0), (0, 0, 1), 180.0, 6.0, 2.0, 3.0), 'led_bar',
               'PETG print', 'led')
    # carry handle on the lid
    h = R.rbox((0, 0, P.TORSO_TOP + 17), (0, 1, 0), (0, 0, 1), 150.0, 48.0, 18.0, 14.0)
    C.cut(h, R.rbox((0, 0, P.TORSO_TOP + 20), (0, 1, 0), (0, 0, 1), 110.0, 18.0, 30.0, 8.0))
    C.cut(h, box((0, 0, P.TORSO_TOP - 50), 100, 400, 100))
    for sy in (-1, 1):
        C.cut(h, zcyl(0, sy * 66, P.TORSO_TOP - 1, P.TORSO_TOP + 10, 1.25))
    C.add_body(comp, h, 'handle', 'PA-CF print', 'shell grey')


# ----------------------------------------------------------------------------- battery, tray, sled
def battery(comp):
    bx, by, bz = P.BATTERY
    x0, y0 = P.BATTERY_X, P.BATTERY_Y
    z0 = P.BATTERY_Z0
    cell = R.rbox((x0, y0, z0 + bz / 2), (1, 0, 0), (0, 0, 1), bx, bz, by, 5.0)
    b = C.add_body(comp, cell, 'battery_24V_15Ah', None, 'battery')
    rho = P.BATTERY_MASS / (b.volume * 1e-6)                      # kg/m3 from cm3
    b.material = C.custom_density_material('battery 24V 15Ah (1.8 kg)', rho)
    ye = y0 + by / 2
    lead = R.cyl((x0 - 6, ye, z0 + 60), (x0 - 6, ye + 8, z0 + 60), 2.2)
    C.union(lead, R.cyl((x0 + 6, ye, z0 + 60), (x0 + 6, ye + 8, z0 + 60), 2.2))
    C.add_body(comp, lead, 'leads', 'Steel', 'print black')
    xt = box((x0, ye + 12, z0 + 60), 16.0, 8.0, 8.0)
    C.add_body(comp, xt, 'XT60', 'PETG print', 'connector yellow')


def tray(comp):
    z0 = P.TUBE_Z + P.TUBE_D / 2
    z1 = z0 + P.TRAY_T
    t = box((P.BATTERY_X, 0, (z0 + z1) / 2), 80.0, 320.0, P.TRAY_T)
    for sx in (-1, 1):
        C.union(t, box((P.BATTERY_X + sx * (P.BATTERY[0] / 2 + 2.0), 0, z1 + 6), 3.0, 300.0, 12.0))
    for sy in (-1, 1):
        C.union(t, box((P.BATTERY_X, sy * (P.BATTERY[1] / 2 + 2.5), z1 + 5), 80.0, 3.0, 10.0))
    C.cut(t, box((P.BATTERY_X, P.BATTERY[1] / 2 + 2.5, z1 + 5), 24.0, 6.0, 12.0))      # lead notch
    for sx in (-1, 1):
        for sy in (-1, 1):
            sad = box((sx * P.TUBE_X, sy * 110, z0 - 6), 22.0, 16.0, 12.0)
            C.cut(sad, ycyl(sx * P.TUBE_X, P.TUBE_Z, sy * 110 - 10, sy * 110 + 10, P.TUBE_D / 2 + 0.2))
            C.union(t, sad)
    for sy in (-1, 1):                                                              # strap slots
        C.cut(t, box((P.BATTERY_X, sy * 80, (z0 + z1) / 2), 60.0, 6.0, 10.0))
    C.add_body(comp, t, 'battery_tray', 'PA-CF print', 'print grey')


def sled(comp):
    z1 = P.SLED_TOP
    z0 = z1 - 3.0
    s = box((0, 0, (z0 + z1) / 2), 76.0, 260.0, 3.0)
    r_in = P.TORSO_R_BOTTOM - P.SHELL_T - 0.2
    for sy in (-1, 1):
        foot = box((0, sy * 100, (z0 - 70.0) / 2), 56.0, 12.0, z0 + 70.0)       # z -70 .. z0
        C.inter(foot, ycyl(0, 0, sy * 100 - 10, sy * 100 + 10, r_in))         # follows the shell bottom
        C.union(s, foot)
    C.add_body(comp, s, 'electronics_sled', 'PA-CF print', 'print grey')


# ----------------------------------------------------------------------------- electronics
def pcb_to_robot(X, Y, z, xc, yc, flip=False):
    """Raspberry Pi board coordinates (X along 85, Y along 56) -> robot."""
    return (xc + (Y - 28.0), yc + (42.5 - X), z)


def rpi_box(X0, X1, Y0, Y1, z0, z1, xc, yc):
    a = pcb_to_robot(X0, Y0, z0, xc, yc)
    b = pcb_to_robot(X1, Y1, z1, xc, yc)
    c = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (z0 + z1) / 2)
    return box(c, abs(b[0] - a[0]), abs(b[1] - a[1]), z1 - z0)


def rpi4(comp, xc, yc, z_pcb):
    z1 = z_pcb + 1.4
    pcb = rpi_box(0, 85, 0, 56, z_pcb, z1, xc, yc)
    holes = [(3.5, 3.5), (61.5, 3.5), (3.5, 52.5), (61.5, 52.5)]
    for (X, Y) in holes:
        p = pcb_to_robot(X, Y, 0, xc, yc)
        C.cut(pcb, zcyl(p[0], p[1], z_pcb - 1, z1 + 1, 1.35))
    C.add_body(comp, pcb, 'rpi4b_pcb', 'FR4 PCB', 'pcb green')
    metal = rpi_box(66, 87.5, 37.75, 53.75, z1, z1 + 13.5, xc, yc)                 # Ethernet
    C.union(metal, rpi_box(70, 87, 20.45, 33.55, z1, z1 + 16.0, xc, yc))           # USB 3
    C.union(metal, rpi_box(70, 87, 2.45, 15.55, z1, z1 + 16.0, xc, yc))            # USB 2
    C.union(metal, rpi_box(7.2, 16.2, -1.2, 6.3, z1, z1 + 3.2, xc, yc))            # USB-C
    C.union(metal, rpi_box(21.5, 28.5, -1.0, 6.5, z1, z1 + 3.0, xc, yc))           # micro HDMI
    C.union(metal, rpi_box(35.0, 42.0, -1.0, 6.5, z1, z1 + 3.0, xc, yc))
    C.union(metal, rpi_box(22.0, 37.0, 25.0, 40.0, z1, z1 + 2.4, xc, yc))          # SoC lid
    C.add_body(comp, metal, 'rpi4b_connectors', 'Connector', 'steel')
    hdr = rpi_box(7.1, 57.9, 50.0, 55.0, z1, z1 + 8.5, xc, yc)                     # 40-pin header
    C.union(hdr, rpi_box(50.0, 57.0, -2.0, 12.0, z1, z1 + 6.0, xc, yc))            # audio jack
    C.add_body(comp, hdr, 'rpi4b_header', 'PETG print', 'print black')
    return holes


def pi3hat(comp, xc, yc, z_pcb, holes):
    z1 = z_pcb + 1.6
    pcb = rpi_box(0, 65, 0, 56.5, z_pcb, z1, xc, yc)
    for (X, Y) in holes:
        p = pcb_to_robot(X, Y, 0, xc, yc)
        C.cut(pcb, zcyl(p[0], p[1], z_pcb - 1, z1 + 1, 1.35))
    C.add_body(comp, pcb, 'pi3hat_pcb', 'FR4 PCB', 'pcb blue')
    con = rpi_box(7.1, 57.9, 50.0, 55.0, z_pcb - 3.0, z_pcb, xc, yc)              # stacking socket
    for k in range(5):                                                            # JC1..JC5 CAN-FD (JST PH-3)
        X = 6 + 11 * k
        C.union(con, rpi_box(X, X + 8, 0.5, 5.0, z1, z1 + 6.0, xc, yc))
    C.add_body(comp, con, 'pi3hat_connectors', 'PETG print', 'connector white')
    pw = rpi_box(48, 60, 30, 40, z1, z1 + 8.0, xc, yc)                              # XT30 power in
    C.add_body(comp, pw, 'pi3hat_XT30', 'PETG print', 'connector yellow')
    ic = rpi_box(22, 29, 24, 31, z1, z1 + 1.4, xc, yc)                              # MCU
    C.union(ic, rpi_box(36, 39, 26, 29, z1, z1 + 1.0, xc, yc))                     # IMU
    C.add_body(comp, ic, 'pi3hat_ics', 'FR4 PCB', 'print black')
    st = None
    for (X, Y) in holes:
        p = pcb_to_robot(X, Y, 0, xc, yc)
        c = zcyl(p[0], p[1], z_pcb - 11.5 + 1.6 - 1.6, z_pcb, 2.5)
        C.cut(c, zcyl(p[0], p[1], z_pcb - 20, z_pcb + 1, 1.35))
        st = c if st is None else C.union(st, c)
    C.add_body(comp, st, 'standoffs_M2.5x11', 'Steel', 'steel')


def power_board(comp, yc, z_pcb):
    z1 = z_pcb + 1.6
    C.add_body(comp, box((0, yc, (z_pcb + z1) / 2), 45.0, 60.0, 1.6), 'power_dist_pcb', 'FR4 PCB', 'pcb black')
    xt = box((0, yc - 24, z1 + 4), 16.0, 8.0, 8.0)                                  # XT60 in
    C.add_body(comp, xt, 'power_dist_XT60', 'PETG print', 'connector yellow')
    out = None
    for k in range(4):                                                             # 4 x XT30 out
        c = box((-15 + 10 * k, yc + 24, z1 + 3), 8.0, 10.0, 6.0)
        out = c if out is None else C.union(out, c)
    C.add_body(comp, out, 'power_dist_XT30', 'PETG print', 'connector yellow')
    cap = zcyl(12, yc, z1, z1 + 13, 5.0)
    C.union(cap, box((-8, yc, z1 + 1.5), 12.0, 16.0, 3.0))
    C.add_body(comp, cap, 'power_dist_parts', 'Al6061', 'print grey')


def back_panel_parts(comp):
    xb = -P.TORSO_X
    btn = xcyl(125, 80, xb - 1.5, xb, 10.0)
    C.union(btn, xcyl(125, 80, xb, xb + 20, 8.0))
    C.add_body(comp, btn, 'power_button', 'Connector', 'steel')
    C.add_body(comp, xcyl(125, 80, xb - 2.5, xb - 1.5, 6.5), 'power_button_led', 'PETG print', 'led')
    jack = xcyl(-125, 80, xb - 1.5, xb, 6.0)
    C.union(jack, xcyl(-125, 80, xb, xb + 16, 4.0))
    C.add_body(comp, jack, 'charge_jack_5.5x2.1', 'Connector', 'steel')


# ----------------------------------------------------------------------------- main
def main():
    C.material('PA-CF print').materialProperties.itemById('structural_Density').value = 900.0
    torso = C.root_occ('torso:1')

    plate = hip_plate()
    cage = hip_cage()
    tube = spine_tube()
    C.instance(torso, plate, C.to_matrix3d(C.robot_to_fusion(C.frame((0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)))))
    C.instance(torso, plate, C.to_matrix3d(C.robot_to_fusion(RZ180)))
    C.instance(torso, cage, adsk.core.Matrix3D.create())
    C.instance(torso, cage, C.to_matrix3d(C.robot_to_fusion(RZ180)))
    for sx in (-1, 1):
        C.instance(torso, tube, C.to_matrix3d(C.robot_to_fusion(
            C.frame((sx * P.TUBE_X, 0, P.TUBE_Z), (1, 0, 0), (0, 1, 0), (0, 0, 1)))))

    # right hip motor and its driver: the left ones rotated 180 deg about z (rigid copies)
    ake50 = C.find_comp('AKE50-8_KV110')
    c1 = C.find_comp('moteus_c1')
    C.instance(torso, ake50, S.torso_child_matrix(C.mmul(RZ180, S.hip_motor_in_torso())))
    C.instance(torso, c1, S.torso_child_matrix(C.mmul(RZ180, L.c1_matrix_for_motor(S.hip_motor_in_torso(),
                                                                                   P.AKE50['back']))))

    shell(comp_in_torso(torso, 'torso_shell'))
    battery(comp_in_torso(torso, 'battery_24V_15Ah'))
    tray(comp_in_torso(torso, 'battery_tray'))
    sled(comp_in_torso(torso, 'electronics_sled'))
    z_rpi = P.SLED_TOP + 2.5
    holes = rpi4(comp_in_torso(torso, 'raspberry_pi_4b'), 0.0, -25.0, z_rpi)
    pi3hat(comp_in_torso(torso, 'pi3hat_r4.5'), 0.0, -25.0, z_rpi + 1.4 + 11.5, holes)
    power_board(comp_in_torso(torso, 'power_dist'), 70.0, z_rpi)
    back_panel_parts(comp_in_torso(torso, 'back_panel'))
