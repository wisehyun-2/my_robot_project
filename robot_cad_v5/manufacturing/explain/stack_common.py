"""Shared part of the v5 joint section figures (hip_joint_stack.py, knee_joint_stack.py, wheel_output_stack.py): the
v4 fusion modules (params, legs, layout) and legs_v5 loaded outside Fusion, the font, the annotation helpers of the v2 /
v3 explain figures and the parts every joint draws the same way (AKE60-8, moteus board stack, 6061 hub insert, the
Warren-rib box of the PA12 links).  Robot y of the LEFT leg on the horizontal axis, radius from the joint axis
vertically; every drawing function takes the half s (+1 upper, -1 lower).
"""
import math
import pathlib
import sys
from unittest import mock
HERE = pathlib.Path(__file__).resolve().parent
V5 = HERE.parents[1]
sys.path.insert(0, str(V5.parent / 'robot_cad_v4' / 'fusion'))
sys.path.insert(0, str(V5 / 'fusion'))
# legs.py / layout.py / legs_v5.py import cadlib, which loads the Fusion API at import time; only their pure-Python
# levels, hole sizes and frames are used here, so outside Fusion a stub stands in for adsk (no geometry is built)
for _m in ('adsk', 'adsk.core', 'adsk.fusion'):
    sys.modules.setdefault(_m, mock.MagicMock())
import params as P                               # noqa: E402
import legs as LG                                # noqa: E402
import layout as LY                              # noqa: E402
import legs_v5 as LV                             # noqa: E402
import matplotlib                                # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt                  # noqa: E402
from matplotlib import font_manager              # noqa: E402
from matplotlib.patches import Circle, Polygon, Rectangle   # noqa: E402,F401
for F in ('C:/Windows/Fonts/malgun.ttf', '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc'):
    if pathlib.Path(F).exists():
        font_manager.fontManager.addfont(F)
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=F).get_name()
        break
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['hatch.linewidth'] = 0.6
A = P.AKE

ROBOT_KG = 10.93                                  # robot_mass() 11-02 (hub version 6, PETG legs, PETG parts at print density, tyre 721 g)
W_STATIC = ROBOT_KG * P.G / 2                    # 51 N per wheel
F_DES, F_CHK = 300.0, 600.0                      # 5 / 10 x 60 N static (brief, v4 strength_v4.py)
FL_DES, FL_CHK = P.LATERAL * F_DES, P.LATERAL * F_CHK         # 150 / 300 N
YTC_V4 = P.tyre_centre_y()                       # 173.5 inboard tyre (v4)
YTC = 2 * LV.Y_MSH - YTC_V4                      # 267.5 outboard tyre (v5, turned about the shank mid-plane)


def mx(z, y_ref, F, Fl, ytc=YTC):
    """Out-of-plane moment at a joint face y_ref (N m): lateral force x height + vertical force x lateral arm
    (calc/strength.py Mx_hip / Mx_knee / Mx_wheel, signs taken adding)."""
    return (Fl * z + F * abs(ytc - y_ref)) / 1000


# DIN 125 M3 washer, ISO 4762 M3 head, DIN 7984 M2.5 low head, ISO 4762 M2.5 head (fasteners.py)
W_OD, W_ID, W_T = 7.0, 3.2, 0.5
M3_DK, M3_K = 5.5, 3.0
M25L_DK, M25L_K = 4.5, 1.5
M25_DK, M25_K = 4.5, 2.5
CSK3_DK, CSK3_K = 6.72, 1.86                     # ISO 10642 M3 (k = dk / 2 x tan 45 - flat, ~1.86)
INS3_L, INS25_L = 5.7, 4.0                       # brass heat-set inserts M3 x 5.7 (hole Ø4.0 x 6), M2.5 x 4 (Ø3.6)

g = dict(plate='#c9d4de', al='#a9c6e4', thigh='#c7b6e6', shank='#9fd29f', cap='#9c8cc9', cup='#f5d0d8',
         stator='#b8b8b8', rotor='#7f9fbf', out='#d9b38c', pcb='#2f7d62', spacer='#dcdcdc', magnet='#8e44ad',
         screw='#2e2e2e', washer='#e3c034', pin='#c0392b', ring='#6f86a6', insert='#b8860b', bearing='#8f8f8f',
         ball='#5e5e5e', flange='#e07b39', rim='#f2a65a', tyre='#4a4a4a', bar='#c9d4de')
HATCH = dict(hatch='////', hatchcolor='#8796a5')
HATCH_AL = dict(hatch='\\\\\\\\', hatchcolor='#6d8db0')
NEW = '#7b3fa0'                                  # tags of fasteners that are not in the model yet (README 5절)
CLASH = '#d0021b'


# ----------------------------------------------------------------------------- drawing primitives
def poly(ax, pts, c, s=1, z=1, **kw):
    p = Polygon([(x, s * r) for x, r in pts], fc=c, ec=kw.pop('ec', 'k'), lw=kw.pop('lw', 0.9), zorder=z, **kw)
    ax.add_patch(p)
    return p


def rect(ax, x0, x1, r0, r1, c, s=1, z=1, **kw):
    return poly(ax, [(x0, r0), (x1, r0), (x1, r1), (x0, r1)], c, s, z, **kw)


def hole(ax, x0, x1, r0, r1, s=1, z=1.2):
    """Hole / pocket in a section: white, thin outline."""
    return rect(ax, x0, x1, r0, r1, 'white', s, z, lw=0.6)


def zig(p0, p1, n=4, a=0.8):
    """Break line points strictly between p0 and p1 (zigzag of amplitude a)."""
    (x0, y0), (x1, y1) = p0, p1
    L = math.hypot(x1 - x0, y1 - y0)
    nx, ny = -(y1 - y0) / L, (x1 - x0) / L
    return [(x0 + (x1 - x0) * k / (2 * n) + nx * a * (1 if k % 2 else -1),
             y0 + (y1 - y0) * k / (2 * n) + ny * a * (1 if k % 2 else -1)) for k in range(1, 2 * n)]


def arc(c, r, a0, a1, n=24):
    """Points of an arc in (y, radius) about c, angles in deg from +y towards +radius."""
    return [(c[0] + r * math.cos(math.radians(a0 + (a1 - a0) * k / n)),
             c[1] + r * math.sin(math.radians(a0 + (a1 - a0) * k / n))) for k in range(n + 1)]


def load_path(ax, lw, path, c='red', alpha=0.7, ls='-'):
    ax.plot(*zip(*path[:-1]), color=c, lw=lw, alpha=alpha, zorder=6, ls=ls, solid_capstyle='round',
            solid_joinstyle='round')
    ax.annotate('', xy=path[-1], xytext=path[-2], zorder=6,
                arrowprops=dict(arrowstyle='-|>', color=c, alpha=alpha, lw=lw, mutation_scale=8 * lw))


def tag(ax, n, xy, c='#1f4e9c', to=None, fs=13):
    if to is not None:
        ax.plot([xy[0], to[0]], [xy[1], to[1]], 'k-', lw=0.9, zorder=9)
    ax.text(*xy, n, fontsize=fs, weight='bold', color='white', ha='center', va='center', zorder=10,
            bbox=dict(boxstyle='circle,pad=0.2', fc=c, ec='white', lw=1.0))


def dim_h(ax, x0, x1, r, txt, ext=(), fs=11, c='k', above=False, dx=0.0):
    ax.annotate('', xy=(x0, r), xytext=(x1, r), zorder=8,
                arrowprops=dict(arrowstyle='<->', color=c, lw=0.9, shrinkA=0, shrinkB=0))
    for x, re in zip((x0, x1), ext):
        if re is not None:
            ax.plot([x, x], [re, r], color=c, lw=0.6, ls=':', zorder=8)
    ax.text((x0 + x1) / 2 + dx, r + (0.6 if above else -0.6), txt, fontsize=fs, ha='center',
            va='bottom' if above else 'top', color=c, zorder=8, bbox=dict(fc='white', ec='none', alpha=0.8, pad=0.3))


def dim_v(ax, x, r0, r1, txt, fs=11, c='k', left=True, ext=()):
    ax.annotate('', xy=(x, r0), xytext=(x, r1), zorder=8,
                arrowprops=dict(arrowstyle='<->', color=c, lw=0.9, shrinkA=0, shrinkB=0))
    for r, xe in zip((r0, r1), ext):
        if xe is not None:
            ax.plot([xe, x], [r, r], color=c, lw=0.6, ls=':', zorder=8)
    ax.text(x + (-0.8 if left else 0.8), (r0 + r1) / 2, txt, fontsize=fs, rotation=90, ha='right' if left else 'left',
            va='center', color=c, zorder=8, bbox=dict(fc='white', ec='none', alpha=0.8, pad=0.3))


def label(ax, txt, xy, xt, fs=11, c='k', ha='left', rp=None):
    if rp is None:                               # leader leaves the text end that faces its target
        rp = (0, 0.5) if xy[0] < xt[0] + 3 else (1, 0.5)
    ax.annotate(txt, xy=xy, xytext=xt, fontsize=fs, color=c, ha=ha, va='center', zorder=11,
                arrowprops=dict(arrowstyle='->', lw=0.8, color=c, shrinkA=2, shrinkB=0, relpos=rp),
                bbox=dict(fc='white', ec='none', alpha=0.85, pad=0.6))


def screw(ax, hb, tip, r, dk, k, d, s=1, z=3, c=None, **kw):
    """Socket / low head screw: head bearing face hb, tip, axis at radius r (head on the side away from the tip)."""
    sg = 1 if tip > hb else -1
    rect(ax, hb, tip, r - d / 2, r + d / 2, c or g['screw'], s, z, **kw)
    rect(ax, hb - sg * k, hb, r - dk / 2, r + dk / 2, c or g['screw'], s, z, **kw)


def csk_screw(ax, y_face, tip, r, s=1, z=3, dk=CSK3_DK, d=3.0):
    """ISO 10642 countersunk screw, head flush with y_face (90 deg cone), shank to tip."""
    sg = 1 if tip > y_face else -1
    k = (dk - d) / 2
    poly(ax, [(y_face, r - dk / 2), (y_face, r + dk / 2), (y_face + sg * k, r + d / 2), (tip, r + d / 2),
              (tip, r - d / 2), (y_face + sg * k, r - d / 2)], g['screw'], s, z)


def legend(ax, items, y, cols=4, dy=0.022, w=0.25):
    ax.text(0.0, y + 0.02, '색', fontsize=14, weight='bold', va='bottom')
    for k, (c, txt, *hatch) in enumerate(items):
        cx, cy = (k % cols) * w, y - (k // cols) * dy
        ax.add_patch(Rectangle((cx, cy - 0.008), 0.05, 0.016, fc=c, ec='k', lw=0.6, **(hatch[0] if hatch else {})))
        ax.text(cx + 0.065, cy, txt, fontsize=11.5, va='center')


def rows_block(ax, rows, y0, dy, head_fs=14.5, body_fs=13, gap=0.03):
    """Numbered fastener rows (number, tag colour, bold head, body) like the v2 / v3 figures."""
    y = y0
    for n, c, head, body in rows:
        ax.text(0.012, y, n, fontsize=15, weight='bold', color='white', ha='center', va='center',
                bbox=dict(boxstyle='circle,pad=0.22', fc=c, ec='none'))
        ax.text(0.04, y, head, fontsize=head_fs, weight='bold', va='center')
        ax.text(0.04, y - gap, body, fontsize=body_fs, va='top', linespacing=1.4)
        y -= dy * (1 + body.count('\n') * 0.25)
    return y


def fits_block(ax, items, y, step=0.042, fs=12.5, x=0.07):
    for n, txt in items:
        ax.text(x - 0.045, y, n, fontsize=14, weight='bold', color='white', ha='center', va='center',
                bbox=dict(boxstyle='circle,pad=0.22', fc='#2e7d32', ec='none'))
        ax.text(x, y + 0.013, txt, fontsize=fs, va='top', linespacing=1.4)
        y -= step + 0.019 * txt.count('\n')
    return y


# ----------------------------------------------------------------------------- AKE60-8 and the moteus board stack
def motor(ax, s, ym, ring, out, magnet=True):
    """AKE60-8: stator (Ø42 pilot x 3, front face r 20.3 .. 30.75), rotor can + cap, output carrier (Ø12 x 1.5
    recess); with magnet: the rotor boss up to the empty magnet seat."""
    sf, cb, pb = ym(A['stator_front']), ym(A['can_back']), ym(A['ring'] + A['pilot_len'])
    poly(ax, [(ring, 13.6), (ring, 21.0), (pb, 21.0), (pb, 20.3), (sf, 20.3), (sf, 30.75), (cb, 30.75), (cb, 13.6)],
         g['stator'], s)
    poly(ax, [(ym(A['can_front']), 31.2), (ym(A['can_front']), A['can_r']), (ym(A['cap_outer']), A['can_r']),
              (ym(A['cap_outer']), 31.2)], g['rotor'], s)
    poly(ax, [(cb, 7.9), (ym(A['cap_face']), 7.9), (ym(A['cap_face']), 34.2), (cb, 34.2)], g['rotor'], s)
    if magnet:
        rect(ax, cb, ym(A['magnet']), 0, A['boss_r'], g['rotor'], s)
    poly(ax, [(ym(A['out'] + 1.5), 0), (ym(A['out'] + 1.5), 6.0), (out, 6.0), (out, 13.5), (cb, 13.5), (cb, 0)],
         g['out'], s)


class Board:
    """moteus r4.11 on its cup standoffs, magnet spacer, magnet: robot y of the board frame levels for a joint whose
    motor back points AX along y."""

    def __init__(self, joint, ym, ax_sign):
        self.ym, self.AX = ym, ax_sign
        self.OB = ym(P.board_offset())               # PCB motor-side face
        self.PCB = P.MOTEUS['pcb_t']
        self.RIM_B, self.FOUT_B = LG.cup_levels(joint, 1)
        self.RIM, self.FOUT, self.FIN = self.yb(self.RIM_B), self.yb(self.FOUT_B), self.yb(self.FOUT_B - P.CUP_WALL)
        self.CUP_L, self.CUP_CB = LG.cup_screw(joint)
        self.CS_HB = self.yb(self.FOUT_B - self.CUP_CB)           # column screw head bearing face
        self.CS_TIP = self.CS_HB - ax_sign * self.CUP_L
        self.RB = sorted({round(math.hypot(x, z), 2) for x, z in P.MOTEUS['holes']})   # board screws r 25.0, 30.48
        self.MAG = ym(A['magnet'])                  # rotor boss end (empty magnet seat)
        self.SP1 = ym(A['magnet'] + P.SPACER_H)     # spacer top = magnet top
        _ang = sorted(((t + 180.0) % 360.0) - 180.0 for t in LG.CUP_COLUMNS)
        self.COL_STEP = _ang[1] - _ang[0]           # 60: the 3 columns sit 60 deg apart, away from the connector edge
        self.COL_R = 40.5

    def yb(self, v):                                 # board y -> robot y
        return self.OB + self.AX * v

    def draw(self, ax, s):
        yb, PCB, AX = self.yb, self.PCB, self.AX
        # magnet spacer (6061) on the rotor boss, Ø6 x 2.5 magnet in its pocket
        rect(ax, self.MAG, self.SP1, 0, 4.5, g['spacer'], s, z=1.5, lw=0.7)
        rect(ax, self.ym(A['magnet'] - P.SPACER_SPIGOT[1]), self.MAG, 0, P.SPACER_SPIGOT[0] / 2, g['spacer'], s, z=1.5,
             lw=0.7)
        rect(ax, self.SP1 - AX * P.SPACER_POCKET[1], self.SP1, 0, 3.0, g['magnet'], s, z=1.6, lw=0.6)
        # PCB + AS5047P on the motor side
        rect(ax, yb(0), yb(PCB), 0, 33.0, g['pcb'], s, z=1.5, lw=0.7)
        rect(ax, yb(-P.MOTEUS['ic_h']), yb(0), 0, 2.5, '#111111', s, z=1.6, lw=0.5)
        # cup standoff + brass insert + DIN 7984 M2.5 x 6 board screw (from the motor side of the PCB)
        rs = self.RB[1] if s > 0 else self.RB[0]
        rect(ax, yb(self.FOUT_B - P.CUP_WALL + 0.5), yb(PCB), rs - 3.2, rs + 3.2, g['cup'], s, z=1.1)
        rect(ax, yb(PCB + 5.0), yb(PCB), rs - LG.INSERT_M25 / 2, rs + LG.INSERT_M25 / 2, 'white', s, z=1.2, lw=0.5)
        rect(ax, yb(PCB + 0.1 + INS25_L), yb(PCB + 0.1), rs - LG.INSERT_M25 / 2, rs + LG.INSERT_M25 / 2, g['insert'], s,
             z=2.5, lw=0.7)
        rect(ax, yb(6), yb(0), rs - 1.25, rs + 1.25, g['screw'], s, z=3)
        rect(ax, yb(0), yb(-1.5), rs - 2.25, rs + 2.25, g['screw'], s, z=3)


# ----------------------------------------------------------------------------- 6061 hub insert (legs_v5.hub_insert)
class Insert:
    """6061 turned hub insert of a driven joint (hip / knee): Ø28 boss from the motor output face to the link, Ø11.8 x
    1.4 spigot, flange r 24 x 6 flush in the link's inner face, Ø18 bore + Ø7.2 counterbores from the washer floor
    (BEARING_FACE from the output face) outwards, Ø3 x 7.5 dowel holes, 6 x Ø3.4 flange bolt holes at r 20."""

    def __init__(self, joint, y_link_in):
        self.OUT = P.y_output(joint)
        self.YI = y_link_in
        self.YT = y_link_in + LV.INSERT_FL[1]
        self.FL_R = LV.INSERT_FL[0]
        self.FLOOR = self.OUT + P.BEARING_FACE      # washer seat (6061: no SUS ring, v4 had one 2 mm lower)
        self.HB = self.FLOOR + W_T                  # M3 x 16 head bearing face
        self.M3_TIP = self.HB - 16
        self.ENG3 = self.OUT - self.M3_TIP          # 4.3 in the 5 mm output taps
        self.PIN1 = self.OUT + P.DOWEL_HOLE         # dowel hole bottom = pin end (pressed to the bottom)
        self.PIN0 = self.PIN1 - 10.0                # 2.5 proud into the output's 3 mm holes
        self.RQ = P.AKE_PCD_OUT / 2
        self.RB = LV.INSERT_BOLT_R
        self.SPIG = (5.9, 1.4)
        self.BORE_R, self.CB_R = 9.0, 3.6

    def draw(self, ax, s, screws=True):
        OUT, YI, YT, FL = self.OUT, self.YI, self.YT, self.FLOOR
        sr, sl = self.SPIG
        if s > 0:                                    # upper half through an output M3: its counterbore joins the bore
            pts = [(OUT - sl, 0), (OUT - sl, sr), (OUT, sr), (OUT, 14.0), (YI, 14.0), (YI, self.FL_R), (YT, self.FL_R),
                   (YT, self.RQ + self.CB_R), (FL, self.RQ + self.CB_R), (FL, 0)]
        else:
            pts = [(OUT - sl, 0), (OUT - sl, sr), (OUT, sr), (OUT, 14.0), (YI, 14.0), (YI, self.FL_R), (YT, self.FL_R),
                   (YT, self.BORE_R), (FL, self.BORE_R), (FL, 0)]
        poly(ax, pts, g['al'], s, z=1.5, **HATCH_AL)
        rect(ax, YI, YT, self.RB - LG.M3_CLEAR / 2, self.RB + LG.M3_CLEAR / 2, 'white', s, z=1.6, lw=0.6)  # flange bolts
        k = (LV.INSERT_CSK - LG.M3_CLEAR) / 2                   # Ø7.0 x 90 deg countersink on the inboard face
        poly(ax, [(YI, self.RB - LV.INSERT_CSK / 2), (YI, self.RB + LV.INSERT_CSK / 2), (YI + k, self.RB + LG.M3_CLEAR / 2),
                  (YI + k, self.RB - LG.M3_CLEAR / 2)], 'white', s, z=1.6, lw=0.6)
        if s > 0:
            hole(ax, OUT, FL, self.RQ - LG.M3_CLEAR / 2, self.RQ + LG.M3_CLEAR / 2, s, z=1.6)
            if screws:                               # output M3 x 16 12.9 + DIN 125 washer on the floor
                rect(ax, FL, self.HB, self.RQ - W_OD / 2, self.RQ + W_OD / 2, g['washer'], s, z=3, lw=0.8)
                screw(ax, self.HB, self.M3_TIP, self.RQ, M3_DK, M3_K, 3.0, s)
        else:
            hole(ax, OUT, self.PIN1, self.RQ - 1.5, self.RQ + 1.5, s, z=1.6)
            if screws:                               # dowel Ø3 x 10
                rect(ax, self.PIN0, self.PIN1, self.RQ - 1.5, self.RQ + 1.5, g['pin'], s, z=3, lw=1.0)


def link_insert_holes(ax, s, y_face, depth, r, d, ins_l, sg=1):
    """Heat-set insert hole (Ø d x depth from y_face into the part, direction sg) + the brass insert flush with it."""
    hole(ax, y_face, y_face + sg * depth, r - d / 2, r + d / 2, s)
    rect(ax, y_face, y_face + sg * ins_l, r - d / 2, r + d / 2, g['insert'], s, z=2.5, lw=0.7)


# ----------------------------------------------------------------------------- PA12 box links (legs_v5)
def rib_crossings(link, L_from_joint=None):
    """Radii (from a joint) where the Warren ribs cross the link axis, and their width there: [(r, w)].  The ribs run
    edge to edge (legs_v5.rib_nodes); on the axis each is a band RIB / sin(angle) wide.  r = s from the link start, or
    L - s when L_from_joint is the link length (the far joint)."""
    out = []
    nodes = LV.rib_nodes(link)
    for (sa, ta), (sb, tb) in zip(nodes, nodes[1:]):
        s0 = sa + (sb - sa) * (0 - ta) / (tb - ta)
        w = LV.RIB / math.sin(math.atan2(abs(tb - ta), sb - sa))
        out.append(((L_from_joint - s0) if L_from_joint else s0, w))
    return out


def ribs(ax, s, cavity_patch, crossings, y0, y1, c, r_max=1e9):
    """Rib bands across the cavity (clipped to it) at the given axis crossings."""
    for r, w in crossings:
        if r + w / 2 > r_max:
            continue
        p = rect(ax, y0 - 1, y1 + 1, r - w / 2, r + w / 2, c, s, z=1.15, lw=0.8)
        p.set_clip_path(cavity_patch)
