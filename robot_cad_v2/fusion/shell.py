"""TITA-proportioned torso shell: shape and colour layout follow Direct Drive Tech's TITA (no logo or lettering),
with TITA's yellow accents in blue.

Shape: a hip section of full width (|y| <= YW) around the hip plates and narrower front / rear sections
(TORSO_Y_END) that the knee drivers and tyres swing past; length 2 x TORSO_X with the hip axis at mid-length
(TITA: length ≈ 1.4 x width).  Side view: raked front face (a short top-front chamfer, then a long slope back to
the bottom); the rear (user's sketch) is an upper slope from the top down to a short vertical face and a lower slope
from the face back to the bottom, cutting the whole width.  Plan view: chamfered vertical corners at both ends.
Colours: body dark grey; blue = hip saddles, drum arcs, three-bar stripes on the right front top corner and the right
rear upper slope, the rear band with its side dashes; white = drum light rings, the line along the top of the
vertical rear face and the side ticks.

The shell carries no load: the hip section hangs on the hip plates (M3), the front / rear sections are joined to it
by inner splice tabs (M3 + nuts); the payload plate sits on the rails through an opening in the top.
"""
import math

import adsk.core
import adsk.fusion

import cadlib as C
import params as P

ZH = P.Z_HIP
X, YE, YW = P.TORSO_X, P.TORSO_Y_END, P.YW
T, BOT = P.TORSO_TOP, P.TORSO_BOT
XM, XT = P.TORSO_X_MID, P.TORSO_X_TAPER
BIG = 1000.0


# ----------------------------------------------------------------------------- outer form
def end_side_profile():
    pts = [(-X, T), (P.FRONT_TOP_X, T), (X, P.FRONT_Z), (P.FRONT_BOT_X, BOT), (-X, BOT)]
    return [(x, ZH + z) for (x, z) in pts]


def rear_slopes():
    """[(point on the slope, unit vector along it towards the rear end, outward normal)] of the upper and lower rear
    slopes (robot coordinates, y = 0)."""
    out = []
    for (x0, z0), (x1, z1) in (((-P.REAR_TOP_X, T), (-X, P.REAR_Z_TOP)), ((-P.REAR_BOT_X, BOT), (-X, P.REAR_Z_BOT))):
        d = C.unit((x1 - x0, 0.0, z1 - z0))
        n = (-abs(d[2]), 0.0, abs(d[0]) if z0 > 0 else -abs(d[0]))        # backwards and up / down
        out.append(((x0, 0.0, ZH + z0), d, n))
    return out


def slope_point(which, x, y):
    """Point of the upper (0) / lower (1) rear slope surface at robot x, y."""
    p, d, _n = rear_slopes()[which]
    return C.add(C.add(p, C.mul(d, (x - p[0]) / d[0])), (0.0, y, 0.0))


def end_plan():
    c = P.PLAN_CORNER
    return [(-X, -(YE - c)), (-X + c, -YE), (X - c, -YE), (X, -(YE - c)), (X, YE - c), (X - c, YE), (-X + c, YE),
            (-X, YE - c)]


def mid_side_profile():
    ch = P.MID_CH
    pts = [(-XM, T), (XM, T), (XT, T - ch), (XT, BOT + ch), (XM, BOT), (-XM, BOT), (-XT, BOT + ch), (-XT, T - ch)]
    return [(x, ZH + z) for (x, z) in pts]


def mid_plan():
    return [(-XT, -YE), (-XM, -YW), (XM, -YW), (XT, -YE), (XT, YE), (XM, YW), (-XM, YW), (-XT, YE)]


def outer_solid():
    B = C.Builder()
    z0, z1 = ZH + BOT - 1, ZH + T + 1
    end = B.prism(end_side_profile(), -YE, YE, plane='xz')
    C.inter(end, B.prism(end_plan(), z0, z1, plane='xy'))
    mid = B.prism(mid_side_profile(), -YW, YW, plane='xz')
    C.inter(mid, B.prism(mid_plan(), z0, z1, plane='xy'))
    s = C.union(end, mid)
    for (p, _d, n) in rear_slopes():                         # the rear: upper and lower slopes over the full width
        C.inter(s, B.halfspace(p, n))
    return s


def hollow(comp):
    """Outer solid shelled inwards by SHELL_T with Fusion's shell feature, returned as a temporary body."""
    tmp = C.add_body(comp, outer_solid(), 'shell_tmp')
    oc = adsk.core.ObjectCollection.create()
    oc.add(tmp)
    si = comp.features.shellFeatures.createInput(oc, False)
    si.insideThickness = adsk.core.ValueInput.createByReal(P.SHELL_T * C.CM)
    comp.features.shellFeatures.add(si)
    tmp = comp.bRepBodies.itemByName('shell_tmp')
    h = C.copy(tmp)
    tmp.deleteMe()
    return h


# ----------------------------------------------------------------------------- front face frame (on the slope)
def face_frame():
    """Top-front corner p0 (robot, y = 0), unit vector down the slope u, outward normal n, slope length."""
    p0 = (X, 0.0, ZH + P.FRONT_Z)
    d = (P.FRONT_BOT_X - X, 0.0, BOT - P.FRONT_Z)
    L = C.norm(d)
    u = C.mul(d, 1.0 / L)
    n = (-u[2], 0.0, u[0])                       # forward and down
    return p0, u, n, L


def on_face(s, y, h=0.0):
    p0, u, n, _L = face_frame()
    return C.add(C.add(C.add(p0, C.mul(u, s)), (0.0, y, 0.0)), C.mul(n, h))


SPLICE = [(sx, sy, where) for sx in (-1, 1) for sy in (-1, 1) for where in ('top', 'bot', 'side')]


SPLICE_X = XT - 2.0                 # tab centres just inside the seam (the front slope starts at FRONT_BOT_X)


def _seam_surface(sx, sy, where):
    """(outer surface point, direction along the shell across the seam, outward normal) of a splice tab."""
    x = sx * SPLICE_X
    if where == 'side':
        return (x, sy * YE, ZH + 40.0), (1.0, 0.0, 0.0), (0.0, float(sy), 0.0)
    k = 0 if where == 'top' else 1
    p, d, n = rear_slopes()[k]
    if sx < 0 and x < p[0]:                                  # rear seam lies on a rear slope
        return slope_point(k, x, sy * 60.0), d, n
    return (x, sy * 60.0, ZH + (T if where == 'top' else BOT)), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0 if k == 0 else -1.0)


def _splice_box(B, sx, sy, where):
    q, d, n = _seam_surface(sx, sy, where)
    c = C.sub(q, C.mul(n, P.SHELL_T + 1.25))
    v = (0.0, 0.0, 1.0) if where == 'side' else (0.0, 1.0, 0.0)
    return B.box(c, d, v, 20.0, 14.0, 2.5)


def _splice_holes(B, sx, sy, where):
    q, d, n = _seam_surface(sx, sy, where)
    out = []
    for dx in (-6.0, 6.0):
        a = C.add(q, C.mul(d, dx))
        out.append(B.cyl(C.sub(a, C.mul(n, 6.0)), C.add(a, C.mul(n, 2.0)), 1.7))
    return out


# rear vertical face (x = -X, z REAR_Z_BOT .. REAR_Z_TOP above the hip axis), top to bottom: white line, knob panel
# (switch beside it), blue band, two rows of perforation
REAR_LINE_Z = P.REAR_Z_TOP - 4.0
REAR_PANEL_Z, REAR_PANEL_H, REAR_PANEL_W = 50.0, 36.0, 140.0
REAR_BAND_Z = 18.0
REAR_HOLES_Z = (2.0, -12.0)


def openings(h):
    B = C.Builder()
    lx, ly, t = P.TOP_PLATE
    C.cut(h, B.abox(-lx / 2 - 1, lx / 2 + 1, -ly / 2 - 1, ly / 2 + 1, ZH + 120, ZH + T + 5))    # payload plate
    for sy in (-1, 1):
        y0, y1 = sy * (YW - P.SHELL_T - 1), sy * (YW + 1)
        C.cut(h, B.ycyl(0, ZH, y0, y1, 26.6))                                  # hip plate boss
        C.cut(h, B.ycyl(0, ZH, sy * (YW - 1.0), y1, 55.0))                     # drum ring seat (1 mm)
        g = P.theta_dir(P.GROMMET_DEG)
        C.cut(h, B.ycyl(g[0] * P.GROMMET_R, ZH + g[1] * P.GROMMET_R, y0, y1, 8.0))   # cable grommet Ø16
        for (x, z) in ((-50.0, 30.0), (50.0, 30.0), (-60.0, -20.0), (60.0, -20.0), (-40.0, 140.0), (40.0, 140.0)):
            C.cut(h, B.ycyl(x, ZH + z, y0, y1, 1.7))                           # M3 to the hip plate
        for x in (-205.0, -198.0, -191.0):                                     # side vents (three slots)
            C.cut(h, B.slot((x, sy * YE, ZH + 30.0), (x, sy * YE, ZH + 48.0), (0, 1, 0), 1.25, 8.0))
    p0, u, n, L = face_frame()
    C.cut(h, B.cyl(on_face(32.0, 0.0, -4.0), on_face(32.0, 0.0, 2.0), 5.0))   # camera window
    for k in range(9):                                                          # chin vents
        y = -64.0 + 16.0 * k
        C.cut(h, B.slot(on_face(L - 34.0, y), on_face(L - 20.0, y), n, 1.5, 8.0))
    for row, z in enumerate(REAR_HOLES_Z):                                      # rear perforation
        for k in range(9):
            y = -80.0 + 20.0 * k + (10.0 if row else 0.0)
            if abs(y) <= 85.0:
                C.cut(h, B.xcyl(y, ZH + z, -X - 2, -X + 6, 2.0))
    C.cut(h, B.abox(-X - 2, -X + 4, P.SWITCH_Y - 9.5, P.SWITCH_Y + 9.5, ZH + P.SWITCH_Z - 6.5, ZH + P.SWITCH_Z + 6.5))
    for (sx, sy, where) in SPLICE:
        for hb in _splice_holes(B, sx, sy, where):
            C.cut(h, hb)
    return h


def pieces(h):
    """Printable pieces (each fits a 256 mm bed): front / hip / rear sections x left / right, and the blue saddles
    cut from the hip sections (outside the end width, above SADDLE_Z)."""
    B = C.Builder()
    out = {}
    for side, sy in (('L', 1), ('R', -1)):
        ylo, yhi = (0.0, BIG) if sy > 0 else (-BIG, 0.0)
        for name, x0, x1 in (('front', XT, BIG), ('mid', -XT, XT), ('rear', -BIG, -XT)):
            part = C.copy(h)
            C.inter(part, B.abox(x0, x1, ylo, yhi, -BIG, BIG))
            if name == 'mid':
                y0, y1 = (YE, BIG) if sy > 0 else (-BIG, -YE)
                sad = C.copy(part)
                C.inter(sad, B.abox(-BIG, BIG, y0, y1, ZH + P.SADDLE_Z, BIG))
                C.cut(part, B.abox(-BIG, BIG, y0, y1, ZH + P.SADDLE_Z, BIG))
                out['saddle_' + side] = sad
            out['shell_%s_%s' % (name, side)] = part
    return out


def splice_tabs():
    B = C.Builder()
    out = None
    for (sx, sy, where) in SPLICE:
        tab = _splice_box(B, sx, sy, where)
        for hb in _splice_holes(B, sx, sy, where):
            C.cut(tab, hb)
        out = C.union(out, tab)
    return out


# ----------------------------------------------------------------------------- hip drums
ARC_HALF = 60.0                                   # blue arc: +-60 deg around straight down


def _arc_wedge(B, s):
    """Region within ARC_HALF of straight down around the hip axis (left-side builder coordinates)."""
    a = math.radians(ARC_HALF)
    w = B.halfspace((0.0, 0.0, ZH), (math.cos(a), 0.0, math.sin(a)))
    C.inter(w, B.halfspace((0.0, 0.0, ZH), (-math.cos(a), 0.0, math.sin(a))))
    return w


def drum_ring(s):
    """Black drum disc around the hip boss (flush in the 1 mm seat) with grooves for the white light ring and the
    blue arc inlays."""
    B = C.Builder(side=s)
    r = B.ycyl(0, ZH, YW - 1.0, YW, 54.9)
    C.cut(r, B.ycyl(0, ZH, YW - 2, YW + 1, 27.2))
    C.cut(r, light_ring(s))
    C.cut(r, drum_arc(s))
    return r


def light_ring(s):
    B = C.Builder(side=s)
    r = B.ycyl(0, ZH, YW - 1.0, YW, 50.0)
    C.cut(r, B.ycyl(0, ZH, YW - 2, YW + 1, 47.0))
    return r


def drum_arc(s):
    B = C.Builder(side=s)
    r = B.ycyl(0, ZH, YW - 1.0, YW, 43.0)
    C.cut(r, B.ycyl(0, ZH, YW - 2, YW + 1, 35.0))
    C.inter(r, _arc_wedge(B, s))
    return r


# ----------------------------------------------------------------------------- glued-on trim (0.6 mm)
TRIM = 0.6


def _bar(B, a, b, n, width):
    """Flat bar from a to b lying on a surface with outward normal n."""
    d = C.sub(b, a)
    c = C.add(C.mul(C.add(a, b), 0.5), C.mul(C.unit(n), TRIM / 2))
    return B.box(c, d, C.cross(n, d), C.norm(d), width, TRIM)


def _wrap_band(B, z, hgt, x_side_end, dashes):
    """Band at height z around the rear: rear face, both corner chamfers, both sides to x_side_end, then dashes."""
    c = P.PLAN_CORNER
    out = _bar(B, (-X, -(YE - c), z), (-X, YE - c, z), (-1, 0, 0), hgt)
    for sy in (-1, 1):
        out = C.union(out, _bar(B, (-X, sy * (YE - c), z), (-X + c, sy * YE, z), (-1, sy, 0), hgt))
        out = C.union(out, _bar(B, (-X + c, sy * YE, z), (x_side_end, sy * YE, z), (0, sy, 0), hgt))
        x = x_side_end + 5.0
        for _k in range(dashes):
            out = C.union(out, _bar(B, (x, sy * YE, z), (x + 7.0, sy * YE, z), (0, sy, 0), hgt))
            x += 11.0
    return out


def stripes_blue():
    """Three-bar stripes on the right (-y) front top corner and the right rear upper slope, the rear band with its
    side dashes."""
    B = C.Builder()
    out = None
    for x0 in (125.0, 143.0, 161.0):                   # front set on the flat top
        a = (x0, -YE + 3.0, ZH + T)
        b = (x0 + 39.0, -YE + 67.0, ZH + T)
        out = C.union(out, _bar(B, a, b, (0, 0, 1), 8.0))
    _p, _d, nu = rear_slopes()[0]
    for x0 in (98.0, 118.0, 138.0):                    # rear set on the upper rear slope
        a, b = slope_point(0, -x0, -YE + 3.0), slope_point(0, -(x0 + 30.0), -YE + 67.0)
        out = C.union(out, _bar(B, a, b, nu, 8.0))
    return C.union(out, _wrap_band(B, ZH + REAR_BAND_Z, 6.0, -140.0, 3))


def lines_white():
    """White line along the top of the vertical rear face (wrapping onto the sides) and two slanted ticks per side."""
    B = C.Builder()
    out = _wrap_band(B, ZH + REAR_LINE_Z, 3.0, -150.0, 0)
    for sy in (-1, 1):
        for x in (-115.0, 115.0):
            a = (x - 4.0, sy * YE, ZH + 116.0)
            b = (x + 4.0, sy * YE, ZH + 128.0)
            out = C.union(out, _bar(B, a, b, (0, sy, 0), 3.0))
    return out


def face_panel():
    """Dark textured face plate on the raked front face (TITA's 'face'), clear of the chin vents."""
    B = C.Builder()
    p0, u, n, L = face_frame()
    s0, s1 = 18.0, L - 42.0
    c = on_face((s0 + s1) / 2, 0.0, 0.5)
    p = B.rbox(c, (0, 1, 0), u, 190.0, s1 - s0, 1.0, 8.0)
    C.cut(p, B.cyl(on_face(32.0, 0.0, -2.0), on_face(32.0, 0.0, 3.0), 11.5))
    return p


def camera():
    """Camera bezel at the top centre of the face; the lens is a second body."""
    B = C.Builder()
    bez = B.cyl(on_face(32.0, 0.0, 0.0), on_face(32.0, 0.0, 4.5), 11.0)
    C.cut(bez, B.cyl(on_face(32.0, 0.0, 1.5), on_face(32.0, 0.0, 5.0), 6.5))
    lens = B.cyl(on_face(32.0, 0.0, 1.5), on_face(32.0, 0.0, 4.0), 6.4)
    return bez, lens


def rear_panel():
    """Raised dark panel on the vertical rear face with two round knobs (TITA's rear)."""
    B = C.Builder()
    zc = ZH + REAR_PANEL_Z
    p = B.rbox((-X - 0.6, 0.0, zc), (0, 1, 0), (0, 0, 1), REAR_PANEL_W, REAR_PANEL_H, 1.2, 6.0)
    for y in (-52.0, 52.0):
        C.union(p, B.xcyl(y, zc, -X - 5.0, -X - 1.0, 7.0))
    return p
