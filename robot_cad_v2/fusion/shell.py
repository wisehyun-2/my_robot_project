"""TITA-proportioned torso shell: shape and colour layout follow Direct Drive Tech's TITA (no logo or lettering),
with TITA's yellow accents in blue.

Shape: a hip section of full width (|y| <= YW) around the hip plates and narrower front / rear sections
(TORSO_Y_END) that the knee drivers and tyres swing past; length 2 x TORSO_X with the hip axis at mid-length
(TITA: length ≈ 1.4 x width).  Side view: raked front face (a short top-front chamfer, then a long slope back to
the bottom); the rear (user's sketch) is an upper slope from the top down to a short vertical face and a lower slope
from the face back to the bottom, cutting the whole width.  Plan view: chamfered vertical corners at both ends; the
hip section's tapered ends run the full height (10-31: no top / bottom edge chamfers, so the saddle's front and back
faces go straight up to the top and to the rear upper slope).
Colours: body dark grey, blue hip saddles.  10-31 (user): no glued-on decor any more (stripes, rear band, white
lines, face panel, camera, rear knob panel) and no hip drums (ring, light ring, arc); the shell side wall fills the
old drum seat.  Openings: payload plate, hip bosses, cable grommets, side / chin vents, rear perforation, switch.

The shell carries no load: the hip section hangs on the hip plates (M3), the front / rear sections are joined to it
by inner splice tabs (M3 + nuts); the payload plate sits on the rails through an opening in the top.
"""
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
    pts = [(-XT, T), (XT, T), (XT, BOT), (-XT, BOT)]                     # 10-31: no edge chamfers (MID_CH 20 before)
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


# rear vertical face (x = -X, z REAR_Z_BOT .. REAR_Z_TOP above the hip axis): two rows of perforation, rocker switch
REAR_HOLES_Z = (2.0, -12.0)


def openings(h):
    B = C.Builder()
    lx, ly, t = P.TOP_PLATE
    C.cut(h, B.abox(-lx / 2 - 1, lx / 2 + 1, -ly / 2 - 1, ly / 2 + 1, ZH + 120, ZH + T + 5))    # payload plate
    for sy in (-1, 1):
        y0, y1 = sy * (YW - P.SHELL_T - 1), sy * (YW + 1)
        C.cut(h, B.ycyl(0, ZH, y0, y1, 26.6))                                  # hip plate boss
        g = P.theta_dir(P.GROMMET_DEG)
        C.cut(h, B.ycyl(g[0] * P.GROMMET_R, ZH + g[1] * P.GROMMET_R, y0, y1, 8.0))   # cable grommet Ø16
        for (x, z) in ((-50.0, 30.0), (50.0, 30.0), (-60.0, -20.0), (60.0, -20.0), (-40.0, 140.0), (40.0, 140.0)):
            C.cut(h, B.ycyl(x, ZH + z, y0, y1, 1.7))                           # M3 to the hip plate
        for x in (-205.0, -198.0, -191.0):                                     # side vents (three slots)
            C.cut(h, B.slot((x, sy * YE, ZH + 30.0), (x, sy * YE, ZH + 48.0), (0, 1, 0), 1.25, 8.0))
    p0, u, n, L = face_frame()
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
