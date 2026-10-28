"""Torso frame, battery and electronics (the TITA-like shell is in shell.py).

Load paths (brief item 4): battery tray on the two 6061 spine tubes, electronics deck on the hip plates,
top payload plate on two 6061 rails that bolt to the hip plate columns -- the shell carries nothing.
Coordinates: robot axes, origin on the floor below the hip midpoint (nominal pose); the hip axis is
at z = Z_HIP, so torso features are written as z = Z_HIP + dz.
"""
import math

import cadlib as C
import layout as LY
import legs as LG
import params as P

ZH = P.Z_HIP


def hip_plate(s):
    """6061-T6 hip plate (8 mm stock): hub 8 mm with the Ø42 pilot pocket, Ø30 hole, 6 x M2.5 low-head
    counterbored on the outer face, Ø80 x 1.5 register boss for the driver cup on the inner face;
    5 mm web: disc r 68 + column + top bar for the payload rails, M5 countersunk for the tubes,
    M4 countersunk for the rails, M3 tapped for the cup, deck and shell."""
    B = C.Builder(side=s)
    y_in = P.YW - P.HIP_PLATE_T                 # 164
    y_web = y_in + P.HIP_WEB_T                  # 169
    web = B.ycyl(0, ZH, y_in, y_web, 68.0)
    C.union(web, B.prism([(-22.0, ZH + 30), (22.0, ZH + 30), (22.0, ZH + 128), (-22.0, ZH + 128)], y_in, y_web))
    C.union(web, B.prism([(-64.0, ZH + 125), (64.0, ZH + 125), (64.0, ZH + P.RAIL_Z + P.RAIL_S / 2 + 2),
                          (-64.0, ZH + P.RAIL_Z + P.RAIL_S / 2 + 2)], y_in, y_web))
    for sx in (-1, 1):                          # gussets column -> bar
        C.union(web, B.prism([(sx * 22.0, ZH + 100), (sx * 22.0, ZH + 125), (sx * 47.0, ZH + 125)][::sx], y_in, y_web))
    C.union(web, B.ycyl(0, ZH, y_web - 0.01, P.YW, 26.0))                       # 3 mm hub boss through the shell
    C.union(web, B.ycyl(0, ZH, y_in - 1.5, y_in + 0.01, 40.0))                  # cup register boss Ø80
    fr = LY.frames('hip', s)
    m = fr['motor']
    C.cut(web, B.ycyl(0, ZH, y_in - 2, P.Y_RING_HIP, 21.025))                  # Ø42 pilot pocket
    C.cut(web, B.ycyl(0, ZH, y_in - 2, P.YW + 1, 15.0))
    for p in LY.motor_points(m, P.AKE_PCD_RING / 2, P.AKE_RING_M25, P.AKE['ring']):
        q = LG.lc(p, s)
        C.cut(web, B.ycyl(q[0], q[2], P.Y_RING_HIP - 0.5, P.YW + 1, LG.M25_CLEAR / 2))
        C.cut(web, B.ycyl(q[0], q[2], P.YW - P.CBORE_M25_HIP[1], P.YW + 1, P.CBORE_M25_HIP[0] / 2))
    ex = LG.ldir(C.col(m, 0), s)
    d = C.unit((ex[0], 0.0, ex[2]))
    C.cut(web, B.box((d[0] * 31.25, y_in - 0.25, ZH + d[2] * 31.25), d, (0, 1, 0), 19.5, 4.5, 7.0))  # lead groove
    for p in LG.cup_column_points('hip', s):                                     # M3 tapped for the cup
        q = LG.lc(p, s)
        C.cut(web, B.ycyl(q[0], q[2], y_in - 2, y_web + 0.5, 1.25))
    for sx in (-1, 1):
        C.cut(web, B.ycyl(sx * P.TUBE_X, ZH + P.TUBE_Z, y_in - 1, y_web + 1, 2.7))          # M5 tube screw
        C.cut(web, B.ycyl(sx * P.TUBE_X, ZH + P.TUBE_Z, y_web - 2.6, y_web + 1, 2.7, 5.4))  # 90 deg countersink
        C.cut(web, B.ycyl(sx * P.RAIL_X, ZH + P.RAIL_Z, y_in - 1, y_web + 1, 2.2))          # M4 rail screw
        C.cut(web, B.ycyl(sx * P.RAIL_X, ZH + P.RAIL_Z, y_web - 2.2, y_web + 1, 2.2, 4.4))
        C.cut(web, B.ycyl(sx * 45.0, ZH - 42.0, y_in - 1, y_web + 1, 1.25))                  # deck flange M3
    for (x, z) in ((-50.0, 30.0), (50.0, 30.0), (-60.0, -20.0), (60.0, -20.0), (-40.0, 140.0), (40.0, 140.0)):
        C.cut(web, B.ycyl(x, ZH + z, y_web - 6, y_web + 1, 1.25))                         # shell M3 (tapped)
    # lightening (the hip cables pass outside the disc, through the shell grommet only); holes kept >= 3 mm
    # from the cup's M3 columns (285 / 345 / 45 deg on the left), the tube screws and the lead groove (235 deg)
    C.cut(web, B.prism([(-15.0, ZH + 60), (15.0, ZH + 60), (15.0, ZH + 118), (-15.0, ZH + 118)], y_in - 1, y_web + 1))
    for a in (0.0, 90.0, 180.0, 255.0, 300.0):
        t = math.radians(a)
        x, z = 50.0 * math.cos(t), 50.0 * math.sin(t)
        C.cut(web, B.ycyl(x, ZH + z, y_in - 2, y_web + 1, 11.0))
    for sx in (-1, 1):
        C.cut(web, B.prism([(sx * 30.0, ZH + 129), (sx * 43.0, ZH + 129), (sx * 43.0, ZH + 141), (sx * 30.0, ZH + 141)][::sx],
                           y_in - 1, y_web + 1))
    return web


def spine_tube(sx):
    B = C.Builder()
    y = P.YW - P.HIP_PLATE_T
    return B.tube((sx * P.TUBE_X, -y, ZH + P.TUBE_Z), (sx * P.TUBE_X, y, ZH + P.TUBE_Z), P.TUBE_D / 2, P.TUBE_D / 2 - P.TUBE_WALL)


def rail(sx):
    B = C.Builder()
    y = P.YW - P.HIP_PLATE_T
    s, w = P.RAIL_S, P.RAIL_WALL
    c = (sx * P.RAIL_X, 0.0, ZH + P.RAIL_Z)
    b = B.box(c, (1, 0, 0), (0, 0, 1), s, s, 2 * y)
    C.cut(b, B.box(c, (1, 0, 0), (0, 0, 1), s - 2 * w, s - 2 * w, 2 * y + 2))
    for yy in (-100.0, -33.3, 33.3, 100.0):                   # top plate bolts (M4 through)
        C.cut(b, B.zcyl(sx * P.RAIL_X, yy, ZH + P.RAIL_Z - 10, ZH + P.RAIL_Z + 10, 2.2))
    return b


def rail_plugs(sx):
    """Tapped 6061 end plugs (rails M4, tubes M5)."""
    B = C.Builder()
    y = P.YW - P.HIP_PLATE_T
    out = None
    for sy in (-1, 1):
        s = P.RAIL_S - 2 * P.RAIL_WALL - 0.1
        out = C.union(out, B.box((sx * P.RAIL_X, sy * (y - 5.0), ZH + P.RAIL_Z), (1, 0, 0), (0, 0, 1), s, s, 10.0))
        out = C.union(out, B.ycyl(sx * P.TUBE_X, ZH + P.TUBE_Z, sy * (y - 10.0), sy * y, P.TUBE_D / 2 - P.TUBE_WALL - 0.05))
    return out


def top_plate():
    """PETG payload plate on the rails (params.TOP_PLATE), M4 heat-set inserts on a 25 mm grid,
    8 x M4 x 25 through-bolts through both rail walls, nylock nuts under the rails."""
    B = C.Builder()
    lx, ly, t = P.TOP_PLATE
    z0 = ZH + P.RAIL_Z + P.RAIL_S / 2
    b = B.rbox((0, 0, z0 + t / 2), (1, 0, 0), (0, 1, 0), lx, ly, t, 12.0)
    for sx in (-1, 1):
        for yy in (-100.0, -33.3, 33.3, 100.0):
            C.cut(b, B.zcyl(sx * P.RAIL_X, yy, z0 - 1, z0 + t + 1, 2.2))
            C.cut(b, B.zcyl(sx * P.RAIL_X, yy, z0 + t - 4.0, z0 + t + 1, 4.0))
    for i in range(-2, 3):
        for j in range(-4, 5):
            x, y = 25.0 * i, 25.0 * j
            if abs(x) > lx / 2 - 10 or abs(y) > ly / 2 - 10:
                continue
            if any(abs(x - sx * P.RAIL_X) < 8 and abs(y - yy) < 8 for sx in (-1, 1) for yy in (-100.0, -33.3, 33.3, 100.0)):
                continue
            C.cut(b, B.zcyl(x, y, z0 + t - 6.0, z0 + t + 1, 2.8))        # M4 heat-set insert (Ø5.6 x 5.7)
    return b


def tray():
    """PETG battery tray on the spine tubes: 3 mm floor, front / back and end stops, tube saddles, strap slots."""
    B = C.Builder()
    z0 = ZH + P.TRAY_Z
    z1 = z0 + P.TRAY_T
    bx, by, bz = P.BATTERY
    xc = P.BATTERY_X
    t = B.abox(-P.TUBE_X - 9, P.TUBE_X + 9, -by / 2 - 4, by / 2 + 4, z0, z1)
    for yr in P.TRAY_RIBS:                       # ribs between the tubes, clear of the hip drivers (|y| < 123)
        C.union(t, B.abox(-P.TUBE_X + 8.4, P.TUBE_X - 8.4, yr - 1.2, yr + 1.2, z0 - 8.0, z0 + 0.01))
    for sx in (-1, 1):
        C.union(t, B.abox(xc + sx * (bx / 2 + 1.0), xc + sx * (bx / 2 + 3.5), -by / 2 + 10, by / 2 - 10, z1, z1 + 14.0))
        for yy in (-140.0, -60.0, 60.0, 140.0):
            sad = B.abox(sx * P.TUBE_X - 9, sx * P.TUBE_X + 9, yy - 8, yy + 8, ZH + P.TUBE_Z - 4, z0 + 0.01)
            C.cut(sad, B.ycyl(sx * P.TUBE_X, ZH + P.TUBE_Z, yy - 9, yy + 9, P.TUBE_D / 2 + 0.15))
            C.union(t, sad)
    for sy in (-1, 1):
        C.union(t, B.abox(xc - bx / 2, xc + bx / 2, sy * (by / 2 + 1.0), sy * (by / 2 + 3.5), z1, z1 + 14.0))
    C.cut(t, B.abox(xc - 12, xc + 12, by / 2, by / 2 + 5, z1 - 1, z1 + 20))              # lead notch (+y end)
    for sy in (-1, 1):                                                                   # 25 mm strap slots
        C.cut(t, B.abox(xc - bx / 2 - 6, xc - bx / 2 - 2.5, sy * 80 - 13, sy * 80 + 13, z0 - 1, z1 + 1))
        C.cut(t, B.abox(xc + bx / 2 + 2.5, xc + bx / 2 + 6, sy * 80 - 13, sy * 80 + 13, z0 - 1, z1 + 1))
    return t


def battery_frame():
    bx, by, bz = P.BATTERY
    return C.frame((P.BATTERY_X, P.BATTERY_Y, ZH + P.BATTERY_Z0 + bz / 2), (1, 0, 0), (0, 1, 0), (0, 0, 1))


def deck():
    """PETG electronics deck between the hip plates (3 mm, ribbed), hung on 4 flanges bolted to the
    hip plate webs (M3).  Bosses: Raspberry Pi 4B (5 mm, M2.5 inserts), mjpower-ss (5 mm, M2.5 inserts)."""
    B = C.Builder()
    z1 = ZH + P.DECK_Z
    z0 = z1 - P.DECK_T_FLOOR
    y_in = P.YW - P.HIP_PLATE_T
    d = B.abox(-55, 55, -y_in + 0.5, y_in - 0.5, z0, z1)
    for sy in (-1, 1):
        for sx in (-1, 1):                                          # flanges outside the hip cup (r 43.4)
            C.union(d, B.abox(sx * 45 - 8, sx * 45 + 8, sy * (y_in - 3.5), sy * (y_in - 0.5), z0, ZH - 33.0))
            C.cut(d, B.ycyl(sx * 45, ZH - 42.0, sy * (y_in - 5), sy * (y_in + 1), 1.7))
    for x in (-40.0, 40.0):                                        # ribs underneath
        C.union(d, B.abox(x - 1.5, x + 1.5, -y_in + 4, y_in - 4, z0 - 6.0, z0 + 0.01))
    for (x0, x1, y0, y1) in ((-32, 32, 104, 156), (-32, 32, -156, -104), (-18, 18, -86, -45), (-15, 15, 12, 58)):
        # windows (lightening, cable pass-through) clear of the Pi and mjpower-ss bosses
        C.cut(d, B.abox(x0, x1, y0, y1, z0 - 7, z1 + 1))
    for (x, y) in rpi_holes_robot() + mjpower_holes_robot():
        C.union(d, B.zcyl(x, y, z1 - 0.01, z1 + 5.0, 3.0))
        C.cut(d, B.zcyl(x, y, z1 - 2.0, z1 + 5.1, 1.8))
    return d


# ----------------------------------------------------------------------------- electronics placement
PI_Y0 = -98.0        # Raspberry Pi X = 0 edge (robot y); USB end at +y
PI_X0 = 28.0         # Raspberry Pi Y = 0 edge (robot x): Pi Y runs towards -x
BOSS = 5.0


def rpi_frame():
    """Pi frame (X along 85 towards USB, Y along 56, Z up): X -> robot +y, Y -> robot -x, Z -> +z."""
    z = ZH + P.DECK_Z + BOSS
    return C.frame((PI_X0, PI_Y0, z), (0, 1, 0), (-1, 0, 0), (0, 0, 1))


def rpi_holes_robot():
    m = rpi_frame()
    return [C.apply(m, (x, y, 0))[:2] for (x, y) in ((3.5, 3.5), (61.5, 3.5), (3.5, 52.5), (61.5, 52.5))]


def pi3hat_frame():
    """pi3hat STEP frame: x -> robot +x (CAN side), y -> +z, z -> -y (Pi USB end).  Pi -> hat:
    x_hat = 27.75 - Y_pi, z_hat = 32.5 - X_pi, Pi top face + 11 mm (stack: socket 8.5 + header 2.5)."""
    pm = rpi_frame()
    o = C.apply(pm, (32.5, 27.75, 1.4 + 11.0))
    return C.frame(o, (1, 0, 0), (0, 0, 1), (0, -1, 0))


MJ_C = (0.0, 35.0)   # mjpower-ss centre (robot x, y); board X -> robot -y (XT90 end at +y), Y -> robot +x


def mjpower_frame():
    z = ZH + P.DECK_Z + BOSS
    return C.frame((MJ_C[0], MJ_C[1], z), (0, -1, 0), (1, 0, 0), (0, 0, 1))


def mjpower_holes_robot():
    m = mjpower_frame()
    return [C.apply(m, (sx * 32.0, sy * 22.0, 0))[:2] for sx in (-1, 1) for sy in (-1, 1)]


def fan_frame():
    """Fan blowing along -x into the Pi / hat gap, at the Pi's back edge."""
    pm = rpi_frame()
    c = C.apply(pm, (40.0, 56.0 + 4.0, 1.4 + 10.0))
    return C.frame(c, (0, 1, 0), (0, 0, 1), (1, 0, 0))
