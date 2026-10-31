"""2D drawings (PDF) and part-frame STEP of the v3 6061 thigh / shank shells and their covers (WBv3-M11..M14, L / R),
with the fits to every mating part (AKE60-8 pilot / output, driver cup, covers, dowels, neighbouring links).

    python link_drawings.py                  # -> step/*.step, drawings/*.pdf
    python link_drawings.py --png DIR        # also PNG previews
    python link_drawings.py --only thigh_L   # one part

Source: the Fusion export export/parts_step/<name>.step (robot frame, Fusion world axes).  Each part is moved to its
own frame: origin = upper joint axis on the face plate's outer face (covers: on the face that sits on the shell),
X = towards the lower joint, Z = towards the cover side, Y = Z x X; the geometry is the Fusion part unchanged (make_step
checks the dowel holes are params.DOWEL_HOLE = 7.5 deep, so the 10 mm pins stand 2.5 proud of face A and cannot bottom
in the AKE output's 3 mm holes).  Views are hidden-line projections (hlr_gmsh.py); dimensions, tables and notes are
written here.
"""
import argparse
import math
import os
import pathlib
import sys
import unicodedata

import numpy as np

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402
from matplotlib.patches import Circle, PathPatch, Polygon  # noqa: E402
from matplotlib.path import Path  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
V3 = HERE.parent
sys.path.insert(0, str(V3 / 'fusion'))
sys.path.insert(0, str(HERE))
import params as P  # noqa: E402
import hlr_gmsh as G  # noqa: E402
import gmsh  # noqa: E402

SRC = V3 / 'export' / 'parts_step'
STEP_DIR = HERE / 'step'
PDF_DIR = HERE / 'drawings'
DATE = '2026-10-31'
PROJECT = 'WheelBiped v3'
DOWEL_DEPTH = P.DOWEL_HOLE   # from face A: pin 10 -> 2.5 proud, AKE output hole 3 deep
RHO = 2.70

for f in (r'C:\Windows\Fonts\malgun.ttf', '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc'):
    if os.path.exists(f):
        font_manager.fontManager.addfont(f)
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=f).get_name()
        break
plt.rcParams.update({'hatch.linewidth': 0.3, 'hatch.color': 'black', 'pdf.fonttype': 42})

LW_VIS, LW_HID, LW_THIN = 0.8, 0.3, 0.3
FS, FS_S = 7.5, 6.0
SHEETS = {'A3': (420.0, 297.0)}
CENTRE = (0, (8, 1.5, 1.5, 1.5))


# ----------------------------------------------------------------------------- parts and frames
def frame(link, s, cover):
    """3 x 4 [R | t]: Fusion world point (robot y, z, x) -> part frame."""
    if link == 'thigh':
        a, b = np.array([0.0, P.Z_HIP]), np.array([P.KNEE_X, P.KNEE_Z])
        y0 = P.Y_FEMUR_IN + P.FEMUR_IN_RELIEF[0] + P.COVER_T if cover else P.Y_FEMUR_OUT
        zs = -s
    else:
        a, b = np.array([P.KNEE_X, P.KNEE_Z]), np.array([0.0, P.R_WHEEL])
        y0 = P.Y_TIBIA_OUT - P.COVER_T if cover else P.Y_TIBIA_IN
        zs = s
    u = (b - a) / np.linalg.norm(b - a)
    X = np.array([u[0], 0.0, u[1]])
    Z = np.array([0.0, zs, 0.0])
    Y = np.cross(Z, X)
    R = np.vstack([X, Y, Z])
    O = np.array([a[0], s * y0, a[1]])
    Pm = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float)      # Fusion world (y, z, x) -> robot (x, y, z)
    A = R @ Pm
    t = -R @ O
    return [list(A[i]) + [float(t[i])] for i in range(3)]


def parts():
    """Shells L / R are different parts (motor-side hole angles); each cover is one part for both sides (its holes are
    symmetric about the link axis; make_step checks that the right-hand export is the same solid)."""
    out = {}
    for link in ('thigh', 'shank'):
        for side, s in (('L', 1), ('R', -1)):
            out['%s_%s' % (link, side)] = dict(link=link, side=side, s=s, cover=False, src=link + '_' + side)
        out['%s_cover' % link] = dict(link=link, side='LR', s=1, cover=True, src=link + '_cover_L')
    return out


PARTS = parts()
NUMBER = {('thigh', False): 'WBv3-M11', ('thigh', True): 'WBv3-M12', ('shank', False): 'WBv3-M13', ('shank', True): 'WBv3-M14'}
TITLE = {('thigh', False): '허벅지 셸', ('thigh', True): '허벅지 덮개', ('shank', False): '정강이 셸', ('shank', True): '정강이 덮개'}
JOINT = {'thigh': ('hip', 'knee'), 'shank': ('knee', 'wheel')}


def levels(link):
    """Shell Z levels (part frame) from params."""
    if link == 'thigh':
        top = P.Y_FEMUR_OUT - (P.Y_FEMUR_IN + P.FEMUR_IN_RELIEF[0] + P.COVER_T)      # wall top = cover seat
        a = P.Y_FEMUR_OUT - P.y_output('hip')                                           # face A (hip output face)
        spool = P.Y_FEMUR_OUT - P.Y_FEMUR_IN
        sg = 1.0                    # spigot / dowels go towards +Z
    else:
        top = P.TIBIA_T - P.COVER_T
        a = P.y_output('knee') - P.Y_TIBIA_IN
        spool = P.TIBIA_T
        sg = -1.0
    seat_floor = P.SEAT_POCKET + 0.0
    ring = P.Y_FEMUR_OUT - P.Y_RING_KNEE if link == 'thigh' else P.Y_RING_WHEEL - P.Y_TIBIA_IN
    return dict(top=top, A=a, spigot=a + sg * 1.4, washer=a - sg * P.BEARING_FACE, spool=spool, sg=sg,
                C=ring, seat=ring + seat_floor, cup=P.FEMUR_T if link == 'thigh' else P.TIBIA_T,
                dowel=a - sg * DOWEL_DEPTH)


# ----------------------------------------------------------------------------- STEP in the part frame
def make_step(name):
    """Part-frame STEP of the Fusion export (shells: the dowel holes checked DOWEL_DEPTH deep).  Returns the volume in mm3."""
    p = PARTS[name]
    if p['cover']:                                   # the right-hand cover must be the same solid in its frame
        sig = []
        for side, s in (('R', -1), ('L', 1)):
            G.load(SRC / ('%s_cover_%s.step' % (p['link'], side)), frame(p['link'], s, True))
            G.mesh(curv=24, hmax=6.0)
            sig.append((round(G.volume(), 2), sorted((round(h['cx'], 2), round(h['cy'], 2), h['d'])
                                                     for h in G.group_holes(G.z_cylinders()))))
        assert sig[0] == sig[1], 'covers L / R differ'
    G.load(SRC / (p['src'] + '.step'), frame(p['link'], p['s'], p['cover']))
    if not p['cover']:
        G.mesh(curv=24, hmax=6.0)
        lv = levels(p['link'])
        holes = [h for h in G.group_holes(G.z_cylinders()) if abs(h['d'] - 3.0) < 1e-3 and not h['convex']]
        assert len(holes) == 3, holes
        for h in holes:
            assert abs(h['z1'] - h['z0'] - DOWEL_DEPTH) < 1e-3 and min(abs(h['z0'] - lv['dowel']), abs(h['z1'] - lv['dowel'])) < 1e-3, h
        gmsh.model.mesh.clear()
    assert len(gmsh.model.getEntities(3)) == 1
    STEP_DIR.mkdir(exist_ok=True)
    G.write_step(STEP_DIR / (name + '.step'))
    return G.volume()


def holes_of(name):
    G.load(STEP_DIR / (name + '.step'))
    G.mesh(curv=24, hmax=6.0)
    return G.group_holes(G.z_cylinders()), [c for c in G.z_cylinders() if c['kind'] == 'cone']


def windows():
    """Cable windows of the loaded shell: [(x centre, y centre)] of their floors (Z 8.5)."""
    out = []
    for _d, f in gmsh.model.getEntities(2):
        if gmsh.model.getType(2, f) != 'Plane':
            continue
        b = gmsh.model.getBoundingBox(2, f)
        if abs(b[2] - 8.5) < 1e-3 and abs(b[5] - 8.5) < 1e-3:
            out.append(((b[0] + b[3]) / 2, (b[1] + b[4]) / 2))
    assert len(out) == 2, out
    return sorted(out)


# ----------------------------------------------------------------------------- sheet
def wrap(s, width):
    out, cur, n = [], '', 0
    for word in s.split(' '):
        w = sum(2 if unicodedata.east_asian_width(c) in 'WF' else 1 for c in word)
        if cur and n + 1 + w > width:
            out.append(cur)
            cur, n = word, w
        else:
            cur, n = (cur + ' ' + word if cur else word), (n + 1 + w if cur else w)
    out.append(cur)
    return out


class Sheet:
    def __init__(self, size='A3'):
        self.w, self.h = SHEETS[size]
        self.fig = plt.figure(figsize=(self.w / 25.4, self.h / 25.4))
        self.ax = self.fig.add_axes([0, 0, 1, 1])
        self.ax.set_xlim(0, self.w)
        self.ax.set_ylim(0, self.h)
        self.ax.set_aspect('equal')
        self.ax.axis('off')
        self.rect(10, 10, self.w - 10, self.h - 10, lw=1.2)

    def rect(self, x0, y0, x1, y1, lw=0.5, fill=False, color='k'):
        self.ax.add_patch(Polygon([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], closed=True, fill=fill, lw=lw,
                                  edgecolor=color, facecolor=color if fill else 'none'))

    def line(self, pts, lw=LW_THIN, ls='-', color='k'):
        xs, ys = zip(*pts)
        self.ax.plot(xs, ys, lw=lw, ls=ls, color=color, solid_capstyle='round')

    def text(self, p, s, fs=FS, ha='center', va='center', rot=0, weight='normal', color='k'):
        self.ax.text(p[0], p[1], s, fontsize=fs, ha=ha, va=va, rotation=rot, weight=weight, color=color)

    def arrow(self, tip, frm, L=2.4, W=0.7):
        dx, dy = tip[0] - frm[0], tip[1] - frm[1]
        n = math.hypot(dx, dy) or 1.0
        ux, uy = dx / n, dy / n
        b = (tip[0] - ux * L, tip[1] - uy * L)
        self.ax.add_patch(Polygon([tip, (b[0] - uy * W, b[1] + ux * W), (b[0] + uy * W, b[1] - ux * W)],
                                  closed=True, color='k', lw=0))

    def dim(self, p1, p2, at, text, horizontal=True, fs=FS):
        if horizontal:
            a, b = (p1[0], at), (p2[0], at)
            for p in (p1, p2):
                sg = 1 if at > p[1] else -1
                self.line([(p[0], p[1] + sg * 1.0), (p[0], at + sg * 1.5)])
        else:
            a, b = (at, p1[1]), (at, p2[1])
            for p in (p1, p2):
                sg = 1 if at > p[0] else -1
                self.line([(p[0] + sg * 1.0, p[1]), (at + sg * 1.5, p[1])])
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        if L > 7:
            self.line([a, b])
            self.arrow(a, b)
            self.arrow(b, a)
        else:
            ux, uy = (b[0] - a[0]) / (L or 1), (b[1] - a[1]) / (L or 1)
            self.line([(a[0] - ux * 5, a[1] - uy * 5), (b[0] + ux * 5, b[1] + uy * 5)])
            self.arrow(a, (a[0] - ux * 3, a[1] - uy * 3))
            self.arrow(b, (b[0] + ux * 3, b[1] + uy * 3))
        m = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
        if horizontal:
            self.text((m[0], m[1] + 0.8), text, fs=fs, va='bottom')
        elif L > 7:
            self.text((m[0] - 0.8, m[1]), text, fs=fs, ha='right', va='center', rot=90)
        else:
            self.text((m[0] - 4.5, m[1]), text, fs=fs, ha='right', va='center')

    def leader(self, tip, knee, text, fs=FS, right=True):
        end = (knee[0] + (4 if right else -4), knee[1])
        self.line([tip, knee, end])
        self.arrow(tip, knee)
        self.text((end[0] + (0.8 if right else -0.8), end[1]), text, fs=fs, ha='left' if right else 'right')

    def table(self, x, y, cols, rows, fs=FS_S, row_h=4.4, title=None):
        """cols: [(header, width)], rows: lists of strings, wrapped to the column width (rows grow).
        Returns the bottom y."""
        if title:
            self.text((x, y + 3.2), title, fs=FS, ha='left', weight='bold')
        unit = fs * 0.3528 * 0.55                       # mm per wrap column (Hangul counts 2)
        line_h = fs * 0.3528 * 1.3
        cells = [[wrap(c, max(4, int((w - 2.0) / unit))) for c, (_, w) in zip(r, cols)] for r in rows]
        hs = [max(row_h, max(len(c) for c in cr) * line_h + 1.3) for cr in cells]
        wsum = sum(w for _, w in cols)
        ys = [y, y - row_h]
        for hh in hs:
            ys.append(ys[-1] - hh)
        for i, yy in enumerate(ys):
            self.line([(x, yy), (x + wsum, yy)], lw=0.5 if i in (0, 1, len(ys) - 1) else 0.25)
        cx = x
        for k, (hdr, w) in enumerate(cols):
            self.line([(cx, y), (cx, ys[-1])], lw=0.5 if k == 0 else 0.25)
            self.text((cx + w / 2, y - row_h / 2), hdr, fs=fs, weight='bold')
            for i, cr in enumerate(cells):
                n = len(cr[k])
                y_mid = (ys[i + 1] + ys[i + 2]) / 2
                for j, part in enumerate(cr[k]):
                    self.text((cx + 1.0, y_mid + ((n - 1) / 2 - j) * line_h), part, fs=fs, ha='left')
            cx += w
        self.line([(cx, y), (cx, ys[-1])], lw=0.5)
        return ys[-1]

    def notes(self, x, y, lines, width, title='주기', fs=FS_S, step=3.5):
        self.text((x, y), title, fs=FS, ha='left', weight='bold')
        yy = y - 1.0
        for i, s in enumerate(lines):
            for j, part in enumerate(wrap(s, width)):
                yy -= step
                self.text((x + (0 if j == 0 else 3.6), yy), ('%d. ' % (i + 1) if j == 0 else '') + part, fs=fs, ha='left')
        return yy

    # --- GD&T
    def symbol(self, kind, cx, cy, s=3.4):
        h = s / 2
        if kind == 'flat':
            self.line([(cx - h, cy - h * 0.5), (cx + h * 0.4, cy - h * 0.5), (cx + h, cy + h * 0.5),
                       (cx - h * 0.4, cy + h * 0.5), (cx - h, cy - h * 0.5)], lw=0.5)
        elif kind == 'par':
            for dx in (-0.6, 0.6):
                self.line([(cx + dx - h * 0.4, cy - h * 0.7), (cx + dx + h * 0.4, cy + h * 0.7)], lw=0.5)
        elif kind == 'perp':
            self.line([(cx - h * 0.8, cy - h * 0.7), (cx + h * 0.8, cy - h * 0.7)], lw=0.5)
            self.line([(cx, cy - h * 0.7), (cx, cy + h * 0.8)], lw=0.5)
        elif kind == 'pos':
            self.ax.add_patch(Circle((cx, cy), h * 0.6, fill=False, lw=0.5))
            self.line([(cx - h * 0.95, cy), (cx + h * 0.95, cy)], lw=0.5)
            self.line([(cx, cy - h * 0.95), (cx, cy + h * 0.95)], lw=0.5)
        elif kind == 'conc':
            self.ax.add_patch(Circle((cx, cy), h * 0.75, fill=False, lw=0.5))
            self.ax.add_patch(Circle((cx, cy), h * 0.35, fill=False, lw=0.5))

    def fcf(self, x, y, items, h=4.4):
        """Feature control frame at (x, y) = bottom left: [symbol kind, tolerance, datum...].  Returns the right x."""
        cx = x
        widths = [5.5] + [max(5.0, 1.45 * len(t) + 2.2) for t in items[1:]]
        for k, w in enumerate(widths):
            self.rect(cx, y, cx + w, y + h, lw=0.5)
            if k == 0:
                self.symbol(items[0], cx + w / 2, y + h / 2)
            else:
                self.text((cx + w / 2, y + h / 2), items[k], fs=FS_S)
            cx += w
        return cx

    def datum(self, tip, toward, letter):
        """Datum feature symbol: filled triangle on the feature at tip, box with the letter at toward."""
        dx, dy = toward[0] - tip[0], toward[1] - tip[1]
        n = math.hypot(dx, dy) or 1.0
        ux, uy = dx / n, dy / n
        w = 1.4
        self.ax.add_patch(Polygon([(tip[0] - uy * w, tip[1] + ux * w), (tip[0] + uy * w, tip[1] - ux * w),
                                   (tip[0] + ux * 2.2, tip[1] + uy * 2.2)], closed=True, color='k', lw=0))
        self.line([(tip[0] + ux * 2.2, tip[1] + uy * 2.2), toward], lw=0.5)
        self.rect(toward[0] - 2.4, toward[1] - 2.4, toward[0] + 2.4, toward[1] + 2.4, lw=0.6)
        self.text(toward, letter, fs=FS, weight='bold')

    def title_block(self, name, number, info):
        w, rh = 190.0, 5.6
        x0, y0 = self.w - 10 - w, 10.0
        rows = [('품명', name, '도번', number),
                ('재질', info['재질'], '수량', info['수량']),
                ('공정', info['공정'], '척도', info['척도']),
                ('후처리', info['후처리'], '투상', '3각법'),
                ('STEP', 'step/%s.step' % info['step'], '일반 공차', 'KS B ISO 2768-mK'),
                ('프로젝트', PROJECT + ' (robot_cad_v3)', '작성일', DATE)]
        cw = (16.0, 106.0, 18.0, 50.0)
        hh = rh * len(rows)
        self.rect(x0, y0, x0 + w, y0 + hh, lw=1.0)
        for i in range(1, len(rows)):
            self.line([(x0, y0 + i * rh), (x0 + w, y0 + i * rh)], lw=0.3)
        cx = x0
        for c in cw[:-1]:
            cx += c
            self.line([(cx, y0), (cx, y0 + hh)], lw=0.3)
        for i, r in enumerate(rows):
            yy = y0 + hh - (i + 0.5) * rh
            cx = x0
            for k, (c, s) in enumerate(zip(cw, r)):
                self.text((cx + 1.0, yy), s, fs=FS_S if k % 2 == 0 else FS - 0.5, ha='left')
                cx += c
        return y0 + hh


class View:
    """Hidden-line view of the part-frame STEP.  toward = from the part to the viewer, xdir = screen right;
    paper = origin + scale * (p . xdir, p . (toward x xdir)).  cut = (axis, value, keep_box) for a section
    (the faces on the plane are hatched), crop = box for a detail."""

    def __init__(self, S, name, toward, xdir, origin, scale, hidden=True, cut=None, crop=None, label=None, label_dy=8.0):
        self.S, self.o, self.k = S, origin, scale
        G.load(STEP_DIR / (name + '.step'))
        if cut:
            G.keep_box(*cut[2])
        if crop:
            G.keep_box(*crop)
        vis, hid, pr = G.project(toward, xdir)
        self.pr = pr
        if cut:
            for loops in G.section_loops(pr, cut[0], cut[1]):
                verts, codes = [], []
                for i, loop in enumerate(loops):
                    pts = [self.p2(q) for q in loop]
                    a = sum(pts[j][0] * pts[j - 1][1] - pts[j - 1][0] * pts[j][1] for j in range(len(pts)))
                    if (a < 0) != (i == 0):
                        pts.reverse()
                    verts += pts + [pts[0]]
                    codes += [Path.MOVETO] + [Path.LINETO] * (len(pts) - 1) + [Path.CLOSEPOLY]
                S.ax.add_patch(PathPatch(Path(verts, codes), facecolor='none', edgecolor='k', lw=0, hatch='////'))
        if hidden:
            for pl in hid:
                S.line([self.p2(q) for q in pl], lw=LW_HID, ls=(0, (2.5, 1.2)), color='0.45')
        pts = []
        for pl in vis:
            pp = [self.p2(q) for q in pl]
            pts += pp
            S.line(pp, lw=LW_VIS)
        xs, ys = zip(*pts)
        self.box = (min(xs), max(xs), min(ys), max(ys))
        if label:
            S.text(((self.box[0] + self.box[1]) / 2, self.box[2] - label_dy), label, fs=FS + 1, weight='bold')

    def p2(self, q):
        return (self.o[0] + self.k * q[0], self.o[1] + self.k * q[1])

    def pt(self, p):
        q = self.pr.p2(np.array(p, float))[0]
        return self.p2(q)

    def axis_line(self, p0, p1, ext=3.0):
        a, b = self.pt(p0), self.pt(p1)
        L = math.hypot(b[0] - a[0], b[1] - a[1]) or 1
        u = ((b[0] - a[0]) / L, (b[1] - a[1]) / L)
        self.S.line([(a[0] - u[0] * ext, a[1] - u[1] * ext), (b[0] + u[0] * ext, b[1] + u[1] * ext)], ls=CENTRE, lw=0.25)


def finish(S, pdf, png_dir, tag):
    pdf.savefig(S.fig)
    if png_dir:
        S.fig.savefig(pathlib.Path(png_dir) / (tag + '.png'), dpi=120)
    plt.close(S.fig)


def info(name, scale, mass):
    p = PARTS[name]
    if p['cover']:
        return {'재질': 'AL6061-T6 판재 3t', '수량': '2개 (왼쪽·오른쪽 공용)',
                '공정': '2D CNC 한 면 (외곽·구멍·접시), Ø36 H8 리머', '후처리': '흑색 아노다이징 권장 (셸과 같이)',
                '척도': scale, 'step': name, 'mass': mass}
    return {'재질': 'AL6061-T6 판재 40t', '수량': '1개 (%s)' % ('왼쪽' if p['side'] == 'L' else '오른쪽'),
            '공정': '3축 MCT 양면 2셋업', '후처리': '흑색 아노다이징 권장 (★ 끼워맞춤 면은 처리 후 치수)',
            '척도': scale, 'step': name, 'mass': mass}


# ----------------------------------------------------------------------------- tolerances / fits
FITS_COMMON = {
    'pilot': ('Ø42 H7 (+0.025/0)', 'AKE60-8 고정자 파일럿 Ø42 (공차 비공개, h7 0/-0.025 가정)', (42.0, 0.025, 0.0), (42.0, 0.0, -0.025)),
    'spool': ('Ø36 g6 (-0.009/-0.025)', '덮개 구멍 Ø36 H8 (+0.039/0)', (36.0, 0.039, 0.0), (36.0, -0.009, -0.025)),
    'dowel': ('Ø3 N7 (-0.004/-0.014)', '평행 핀 ISO 8734 Ø3 m6 (+0.008/+0.002)', (3.0, -0.004, -0.014), (3.0, 0.008, 0.002)),
    'house': ('Ø88 h8 (0/-0.054)', '드라이버 컵 등록 립 Ø88.1 (PETG 출력 ±0.1)', (88.1, 0.1, -0.1), (88.0, 0.0, -0.054)),
    'boss': ('Ø28 (0/-0.1)', 'Ø30 (+0.1/0) 구멍', (30.0, 0.1, 0.0), (28.0, 0.0, -0.1)),
    'spigot': ('Ø11.8 h9 (0/-0.043)', 'AKE60-8 출력 홈 Ø12 × 1.5 (공차 미표기, ±0.05 가정)', (12.0, 0.05, -0.05), (11.8, 0.0, -0.043)),
}


def clr(hole, shaft):
    lo = (hole[0] + hole[2]) - (shaft[0] + shaft[1])
    hi = (hole[0] + hole[1]) - (shaft[0] + shaft[2])
    if hi < 0:
        return '억지 %.3f ~ %.3f' % (-hi, -lo)
    if lo < 0:
        return '%.3f ~ +%.3f (중간)' % (lo, hi)
    return '틈 %.3f ~ %.3f' % (lo, hi)


def fit_rows(link):
    j_up, j_lo = JOINT[link]
    lv = levels(link)
    F = FITS_COMMON
    up_motor = 'hip' if link == 'thigh' else 'knee'
    rows = [
        ['B 스피곳 ↔ %s 출력 홈' % up_motor, F['spigot'][0], F['spigot'][1], '지름 ' + clr(F['spigot'][2], F['spigot'][3]) + ' (안내만, 위치는 핀)'],
        ['스피곳 높이 ↔ 홈 깊이', '1.4 (0/-0.05)', '홈 깊이 1.5 (미표기)', '끝 틈 0.10 ~ 0.15 (A면이 먼저 닿음)'],
        ['P 핀 구멍 ↔ 평행 핀', F['dowel'][0], F['dowel'][1], clr(F['dowel'][2], F['dowel'][3]) + ' (압입)'],
        ['핀 돌출 ↔ %s 출력 핀 구멍' % up_motor, '구멍 깊이 %.1f (+0.1/0)' % DOWEL_DEPTH,
         '핀 길이 10 ±0.25, AKE 구멍 깊이 3', '돌출 2.15 ~ 2.75, 끝 틈 0.25 ~ 0.85'],
        ['M 출력 나사 ↔ %s 출력 M3 탭' % up_motor, 'Ø3.4 (+0.1/0), 위치 Ø0.2', 'M3 × 16 12.9 + DIN 125 와셔', '나사 틈 0.4 ≥ 위치 공차 합'],
        ['와셔 자리 ↔ 출력 M3 탭 (깊이 5)', 'A에서 11.2 ±0.05', 'M3 × 16 (±0.35), 와셔 0.5', '물림 3.9 ~ 4.7, 탭 바닥 틈 0.3 이상'],
        ['스풀 ↔ 덮개 (%s)' % NUMBER[(link, True)], F['spool'][0], F['spool'][1], clr(F['spool'][2], F['spool'][3])],
        ['D 포켓 ↔ %s 모터 파일럿' % j_lo, F['pilot'][0], F['pilot'][1], clr(F['pilot'][2], F['pilot'][3])],
        ['S 고정링 구멍 ↔ AKE 고정링 M2.5 탭', 'Ø2.8 (+0.1/0), 위치 Ø0.1', 'M2.5 × 8 DIN 7984, PCD 36.5', '나사 틈 0.3 ≥ 위치 공차 합'],
        ['하우징 바깥 ↔ %s 드라이버 컵' % j_lo, F['house'][0], F['house'][1], clr(F['house'][2], F['house'][3]) + ' (꽉 끼면 립을 사포로)'],
        ['T 덮개 탭 ↔ 덮개 접시 구멍', 'M3 탭 위치 Ø0.1 (A·B·D)', '덮개 접시 구멍 위치 Ø0.1 (B)', '접시 머리 어긋남 ≤ 0.1'],
    ]
    if link == 'thigh':
        rows.insert(5, ['보스 Ø28 ↔ hip 판 Ø30 (WBv2-M01)', F['boss'][0], 'hip 판 Ø30 관통 (일반 공차)', '반경 틈 1.0 이상 (위치 오차 포함 0.9)'])
        rows.append(['Ø30 구멍 ↔ 정강이 knee 보스 Ø28', 'Ø30 (+0.1/0), 위치 Ø0.2 (D)', '정강이 보스 Ø28 (0/-0.1)', '반경 틈 1.0 ~ 1.1 (위치 0.1 빼면 0.9)'])
        rows.append(['축간 175 ↔ 다리 길이', '175 ±0.03', '정강이 175 ±0.03', '다리 길이 오차 ≤ 0.06'])
        rows.append(['무릎 축 방향 틈 (허벅지 Z0 면–정강이 안쪽 판)', '포켓 바닥 C: Z %.1f ±0.05' % lv['C'],
                     '정강이 A: Z -7.5 ±0.05, AKE 링→출력 0.5', '틈 1.5 ±0.1 (+모터 공차)'])
    else:
        rows.insert(5, ['보스 Ø28 ↔ 허벅지 Ø30 (WBv3-M11)', F['boss'][0], '허벅지 Ø30 (+0.1/0)', '반경 틈 1.0 ~ 1.1 (위치 0.1 빼면 0.9)'])
        rows.append(['Ø30 구멍 ↔ 바퀴 허브 플랜지 Ø28', 'Ø30 (+0.1/0), 위치 Ø0.2 (D)', '허브 플랜지 보스 Ø28 (PETG ±0.1)', '반경 틈 0.9 ~ 1.1'])
        rows.append(['축간 175 ↔ 다리 길이', '175 ±0.03', '허벅지 175 ±0.03', '다리 길이 오차 ≤ 0.06'])
        rows.append(['무릎 축 방향 틈 (허벅지 바깥 판–Z0 면)', 'A: Z -7.5 ±0.05', '허벅지 포켓 바닥 Z 6.5 ±0.05, AKE 0.5', '틈 1.5 ±0.1 (+모터 공차)'])
    return rows


def gdt_rows(link):
    up = 'hip' if link == 'thigh' else 'knee'
    lo = JOINT[link][1]
    return [
        (['flat', '0.02'], 'A: 구동 보스 끝면 (%s 출력 면에 닿음)' % up),
        (['perp', 'Ø0.02', 'A'], 'B: 스피곳 Ø11.8 h9 축'),
        (['pos', 'Ø0.02', 'A', 'B'], 'P 핀 구멍 3곳 (각도는 구멍 표 좌표)'),
        (['pos', 'Ø0.2', 'A', 'B'], 'M 출력 나사 구멍 6곳'),
        (['conc', 'Ø0.05', 'B'], '스풀 Ø36 g6'),
        (['flat', '0.02'], 'C: Ø42 포켓 바닥 = %s 고정링 면' % lo),
        (['par', '0.03', 'A'], 'C면 (두 관절 축 평행)'),
        (['perp', 'Ø0.02', 'C'], 'D: Ø42 H7 포켓 축 (B에서 175 ±0.03)'),
        (['conc', 'Ø0.05', 'D'], '하우징 바깥 Ø88 h8 (컵·엔코더 중심)'),
        (['pos', 'Ø0.1', 'C', 'D'], 'S 고정링 구멍 6곳'),
        (['pos', 'Ø0.2', 'C', 'D'], 'Ø30 구멍, K 컵 탭 3곳'),
        (['par', '0.05', 'C'], '컵 면 (하우징 끝면 Z 30)'),
        (['flat', '0.05'], '덮개 자리 (벽 윗면)'),
        (['pos', 'Ø0.1', 'A', 'B'], 'T 덮개 탭 10곳'),
    ]


def hole_rows(link, holes):
    """Classify the shell's Z-axis holes into the hole table (ID, X, Y, spec, use) and the label list."""
    lv = levels(link)
    spec = {
        'T': ('M3 탭 깊이 6 (기초 Ø2.5 깊이 7.5), 벽 윗면 Z %.1f에서' % lv['top'], '덮개 ISO 10642 M3 × 8 10.9'),
        'M': ('Ø3.4 (+0.1/0) 보스 바닥 관통 (Z %.1f ~ %.1f)' % tuple(sorted((lv['A'], lv['washer']))),
              '출력 M3 × 16 12.9 + DIN 125 와셔'),
        'P': ('★ Ø3 N7 (-0.004/-0.014) 깊이 %.1f (+0.1/0), A면에서' % DOWEL_DEPTH, '평행 핀 Ø3 × 10 m6 압입 (돌출 2.5)'),
        'S': ('Ø2.8 (+0.1/0) 관통 + 카운터보어 Ø4.8 (+0.1/0) 깊이 1.7 (+0.1/0), Z0 면', 'AKE 고정링 M2.5 × 8 DIN 7984'),
        'K': ('M2.5 탭 깊이 5 (기초 Ø2.05 깊이 6), 컵 면 Z %.0f에서' % lv['cup'], '드라이버 컵 M2.5'),
    }
    found = {k: [] for k in spec}
    for h in holes:
        if h['convex']:
            continue
        d = h['d']
        if abs(d - 2.5) < 0.01:
            found['T'].append(h)
        elif abs(d - 3.4) < 0.01:
            found['M'].append(h)
        elif abs(d - 3.0) < 0.01:
            found['P'].append(h)
        elif abs(d - 2.8) < 0.01:
            found['S'].append(h)
        elif abs(d - 2.05) < 0.01:
            found['K'].append(h)
    expect = {'T': 10, 'M': 6, 'P': 3, 'S': 6, 'K': 3}
    for k, n in expect.items():
        assert len(found[k]) == n, (k, len(found[k]))
    rows, labels = [], []
    for k in ('P', 'M', 'T', 'S', 'K'):
        hs = found[k]
        if k == 'T':
            hs.sort(key=lambda h: (h['cx'], -h['cy']))
        else:
            c = (0.0, 0.0) if k in ('P', 'M') else (175.0, 0.0)
            hs.sort(key=lambda h: (math.atan2(h['cy'] - c[1], h['cx'] - c[0]) + 2 * math.pi) % (2 * math.pi))
        for i, h in enumerate(hs):
            lab = '%s%d' % (k, i + 1)
            rows.append([lab, '%.2f' % (h['cx'] if abs(h['cx']) > 0.004 else 0.0), '%.2f' % (h['cy'] if abs(h['cy']) > 0.004 else 0.0),
                         spec[k][0] if i == 0 else '〃', spec[k][1] if i == 0 else '〃'])
            labels.append((lab, h))
    return rows, labels


def z_rows(link):
    lv = levels(link)
    j_up, j_lo = JOINT[link]
    rows = [(0.0, '', '%s 판 바깥 면 = 기준 Z0 (S 카운터보어 쪽)' % ('바깥' if link == 'thigh' else '안쪽')),
            (1.7, '+0.1/0', 'S 카운터보어 바닥'), (2.5, '±0.1', '판 안쪽 면 (상자 포켓 바닥)'),
            (6.0, '+0.2/0', '리드선 홈 바닥 (폭 12, 반경 23 ~ 41)'),
            (lv['C'], '±0.05', 'C: Ø42 포켓 바닥 = %s 고정링 면 ★' % j_lo),
            (8.0, '±0.1', '모터 자리 판의 모터 쪽 면 (하우징 안 바닥)'), (8.5, '', '배선 창 2곳 바닥 (폭 10, 뒤 벽)'),
            (lv['top'] - 7.5, '', 'T 탭 기초 구멍 바닥 (벽 윗면에서 7.5)'), (24.0, '', 'K 탭 기초 구멍 바닥 (컵 면에서 6)'),
            (lv['top'], '±0.05', '벽 윗면 = 덮개 자리 ★ (평면도 0.05)'),
            (lv['washer'], '±0.05', '출력 나사 와셔 자리 (접근 구멍 Ø28 바닥, A에서 11.2)'),
            (lv['dowel'], '+0.1/0', 'P 핀 구멍 바닥 (A에서 %.1f)' % DOWEL_DEPTH),
            (30.0, '±0.05', '하우징 끝면 = %s 드라이버 컵 면, 스풀 끝' % j_lo),
            (lv['A'], '±0.05', 'A: 구동 보스 끝면 = %s 출력 면 ★' % j_up),
            (lv['spigot'], 'A에서 0/-0.05', '스피곳 끝 (A에서 1.4)')]
    rows.sort(key=lambda r: r[0])
    return [['%.1f' % z, tol, s] for z, tol, s in rows]


# ----------------------------------------------------------------------------- shell drawing
def label_holes(S, V, labels, z):
    for lab, h in labels:
        c = (175.0, 0.0) if lab[0] in 'SK' else (0.0, 0.0)
        if lab[0] == 'T':
            u = (0.0, 1.0 if h['cy'] > 0 else -1.0)
        else:
            dx, dy = h['cx'] - c[0], h['cy'] - c[1]
            n = math.hypot(dx, dy) or 1.0
            u = (dx / n, dy / n)
        r = h['d'] / 2 + (2.6 if lab[0] != 'T' else 5.2)
        p = V.pt((h['cx'] + u[0] * r, h['cy'] + u[1] * r, z))
        S.text(p, lab, fs=4.8, color='#1f4e9c')


def section_marks(S, V, z):
    for x in (-36.0, 225.0):
        a, b = V.pt((x, 0, z)), V.pt((x + 7.0, 0, z))
        S.line([a, b], lw=1.4)
        m = V.pt((x + 3.5, 0, z))
        tip = V.pt((x + 3.5, 6.0, z))
        S.line([m, tip], lw=0.6)
        S.arrow(tip, m)
        S.text((tip[0], tip[1] + 3.0), 'A', fs=FS + 1, weight='bold')


def draw_shell(name, pdf, png_dir, mass):
    p = PARTS[name]
    link, side = p['link'], p['side']
    lv = levels(link)
    holes, _cones = holes_of(name)
    win = windows()
    wy = 1.0 if win[0][1] > 0 else -1.0                 # the back wall's side in this part's Y
    hrows, labels = hole_rows(link, holes)
    half = 25.0 if link == 'thigh' else 28.0
    num = '%s-%s' % (NUMBER[(link, False)], side)
    sname = '왼쪽' if side == 'L' else '오른쪽'
    title = '%s %s (%s)' % (TITLE[(link, False)], sname, name)
    j_up, j_lo = JOINT[link]
    cz = lv['cup']

    # ---- sheet 1: plan views
    S = Sheet()
    V = View(S, name, (0, 0, 1), (1, 0, 0), (50.0, 206.0), 1.0,
             label='덮개 쪽(+Z)에서 본 그림  1:1   (%s, 원점 = B축)' % ('로봇 안쪽 면' if link == 'thigh' else '로봇 바깥 면'),
             label_dy=12)
    V.axis_line((-34, 0, cz), (225, 0, cz))
    V.axis_line((0, -34, cz), (0, 34, cz))
    V.axis_line((175, -50, cz), (175, 50, cz))
    label_holes(S, V, [l for l in labels if l[0][0] in ('TKS' + ('PM' if link == 'thigh' else ''))], cz)
    section_marks(S, V, cz)
    top = V.box[3]
    S.dim(V.pt((0, 0, cz)), V.pt((175, 0, cz)), top + 7, '175 ±0.03  (B축 ~ D축)')
    S.dim(V.pt((-28, 0, cz)), V.pt((219, 0, cz)), top + 16, '247')
    S.dim(V.pt((175, -44, cz)), V.pt((175, 44, cz)), V.box[1] + 9, 'Ø88 h8 (0/-0.054)', horizontal=False)
    S.dim(V.pt((100, -half, cz)), V.pt((100, half, cz)), V.pt((100, 0, cz))[0], '보 폭 %.0f' % (2 * half), horizontal=False)
    S.leader(V.pt((-28 * 0.707, 28 * 0.707, cz)), V.pt((-14, 44, cz)), 'R28')
    S.leader(V.pt((175 + 21.0 * 0.6, 21.0 * 0.8, cz)), V.pt((200, 52, cz)), 'Ø42 H7 깊이 1.5 ±0.05 (D)')
    if link == 'thigh':
        S.datum(V.pt((5.9 * 0.7, -5.9 * 0.7, cz)), V.pt((22, -38, cz)), 'B')
    S.datum(V.pt((175 - 21 * 0.7, -21 * 0.7, cz)), V.pt((126, -38, cz)), 'D')
    S.leader(V.pt((win[0][0], wy * (half - 1.2), cz)), V.pt((win[0][0] + 10, wy * (half + 12), cz)),
             '배선 창 2곳 (뒤 벽, 폭 10, 바닥 Z 8.5)')

    V2 = View(S, name, (0, 0, -1), (1, 0, 0), (50.0, 100.0), 1.0,
              label='바깥 판 쪽(-Z)에서 본 그림  1:1   (X축으로 뒤집어 봄: 위·아래가 바뀜)', label_dy=10)
    V2.axis_line((-34, 0, 0), (225, 0, 0))
    V2.axis_line((175, -50, 0), (175, 50, 0))
    V2.axis_line((0, -34, 0), (0, 34, 0))
    label_holes(S, V2, [l for l in labels if l[0][0] in ('SM' + ('P' if link == 'shank' else ''))], 0.0)
    S.leader(V2.pt((14 * 0.7, 14 * 0.7, 0)), V2.pt((30, 40, 0)), 'Ø28 (+0.2/0) 접근 구멍, 바닥 = 와셔 자리')
    S.leader(V2.pt((175 + 15 * 0.7, 15 * 0.7, 0)), V2.pt((205, 52, 0)), 'Ø30 (+0.1/0) 관통')
    if link == 'shank':
        S.datum(V2.pt((-5.9 * 0.7, 5.9 * 0.7, 0)), V2.pt((-22, 40, 0)), 'B')

    # GD&T table (right column)
    x0, y0 = 304.0, 278.0
    S.text((x0, y0 + 3.2), '기하 공차 (데이텀 A·B·C·D)', fs=FS, ha='left', weight='bold')
    yy = y0
    for items, desc in gdt_rows(link):
        yy -= 6.4
        S.fcf(x0, yy, items)
        S.text((x0 + 33.0, yy + 2.2), desc, fs=FS_S - 0.4, ha='left')
    yy -= 5.0
    yy = S.notes(x0, yy - 2, [
        'A: 구동 보스 끝면 (%s 모터 출력 면과 맞닿음). B: 스피곳 축. C: 아래 관절 모터 자리 바닥. D: Ø42 포켓 축.' % j_up,
        'A·B·C·D와 Ø88 하우징은 가능한 한 같은 셋업에서 (두 관절 축의 평행·거리).',
        '구멍 ID(파란 글씨)는 3쪽 구멍 표.',
    ], width=58, title='데이텀')
    S.title_block(title, num + ' (1/3)', info(name, '1:1', mass))
    finish(S, pdf, png_dir, name + '_1')

    # ---- sheet 2: section A-A and two details (2:1)
    S = Sheet()
    cut = ('y', 0.0, (-60, 260, 0.0, 60, -20, 50))
    zlo = min(lv['spigot'], 0.0)
    V = View(S, name, (0, -1, 0), (1, 0, 0), (62.0, 236.0 - zlo), 1.0, hidden=False, cut=cut,
             label='단면 A-A  1:1  (Y = 0, 두 관절 축을 지나는 면)', label_dy=9)
    S.text((V.box[0] - 4, V.pt((0, 0, 0))[1]), 'Z0', fs=FS_S, ha='right')
    for x, lab in ((0.0, '상세 E'), (175.0, '상세 F')):
        q = V.pt((x, 0, lv['cup'] + 2))
        S.text((q[0], V.box[3] + 4), lab + ' ↓', fs=FS_S)
    k = 2.0
    # detail E: upper joint
    ox, oy = 84.0, 120.0 - k * zlo
    E = View(S, name, (0, -1, 0), (1, 0, 0), (ox, oy), k, hidden=False, cut=cut, crop=(-32, 32, -1, 61, -20, 50),
             label='상세 E (위 관절, %s)  2:1' % j_up, label_dy=13 if link == 'thigh' else 26)
    P2 = lambda x, z: E.pt((x, 0, z))  # noqa: E731
    xl = E.box[0] - 6
    S.dim(P2(-28, 0), P2(-28, lv['A']), xl - 14, '%.1f ±0.05' % abs(lv['A']), horizontal=False)
    S.dim(P2(-14, lv['A']), P2(-14, lv['washer']), xl - 2 if link == 'thigh' else xl - 2, '11.2 ±0.05', horizontal=False)
    S.dim(P2(5.9, lv['A']), P2(5.9, lv['spigot']), E.box[1] + 6, '1.4 (0/-0.05)', horizontal=False)
    S.dim(P2(28, 0), P2(28, lv['top']), E.box[1] + 20, '%.1f ±0.05' % lv['top'], horizontal=False)
    zt = max(lv['spigot'], lv['spool'])
    zb = min(lv['spigot'], 0.0)
    S.dim(P2(-5.9, lv['spigot']), P2(5.9, lv['spigot']), (E.box[3] + 5) if link == 'thigh' else (E.box[2] - 6),
          'Ø11.8 h9 (B)')
    S.dim(P2(-14, lv['A']), P2(14, lv['A']), (E.box[3] + 14) if link == 'thigh' else (E.box[2] - 15), 'Ø28 (0/-0.1)')
    S.dim(P2(-18, lv['spool']), P2(18, lv['spool']), (E.box[3] + 23) if link == 'thigh' else (E.box[3] + 6), 'Ø36 g6')
    zb_ = 0.0 if link == 'thigh' else lv['spool']                  # open end of the Ø28 access bore
    S.dim(P2(-14, zb_), P2(14, zb_), E.box[2] - 6 if link == 'thigh' else E.box[3] + 15, 'Ø28 (+0.2/0) 접근 구멍')
    S.datum(P2(-10, lv['A']), (P2(-10, lv['A'])[0] - 14, P2(-10, lv['A'])[1] + (8 if link == 'thigh' else -8)), 'A')
    del zt, zb
    # detail F: lower joint
    ox, oy = 200.0 - k * 130.0, 120.0
    F = View(S, name, (0, -1, 0), (1, 0, 0), (ox, oy), k, hidden=False, cut=cut, crop=(129, 221, -1, 61, -1, 31),
             label='상세 F (아래 관절, %s 모터 자리)  2:1' % j_lo, label_dy=13)
    Q = lambda x, z: F.pt((x, 0, z))  # noqa: E731
    xr = F.box[1] + 6
    S.dim(Q(219, 0), Q(219, lv['cup']), xr + 16, '30 ±0.05', horizontal=False)
    S.dim(Q(196, 0), Q(196, lv['C']), xr + 2, '%.1f ±0.05' % lv['C'], horizontal=False)
    S.dim(Q(154, lv['C']), Q(154, 8.0), F.box[0] - 4, '1.5 ±0.05', horizontal=False)
    S.dim(Q(133.5, 0), Q(133.5, 8.0), F.box[0] - 30, '8 ±0.1', horizontal=False)
    S.dim(Q(175 - 21.006, 8.0), Q(175 + 21.006, 8.0), F.box[3] + 5, 'Ø42 H7 (+0.025/0) (D)')
    S.dim(Q(160, 0), Q(190, 0), F.box[2] - 6, 'Ø30 (+0.1/0)')
    S.dim(Q(133.5, lv['cup']), Q(216.5, lv['cup']), F.box[3] + 14, 'Ø83 ±0.1')
    S.dim(Q(131, lv['cup']), Q(219, lv['cup']), F.box[3] + 23, 'Ø88 h8 (0/-0.054)')
    S.datum(Q(175 - 18.0, lv['C']), (Q(157, lv['C'])[0] - 6, Q(157, lv['C'])[1] + 22), 'C')
    S.notes(22.0, 58.0, [
        '단면 해칭 = Y 0 면의 재료.  상세 E·F의 둘레 직선은 잘라 낸 경계(실제 모서리 아님).',
        'A면·스피곳·P 핀 구멍·M 구멍은 같은 셋업.  %s' % (
            'A면과 Ø42 포켓·C면이 모두 +Z 쪽이라 한 셋업으로 평행 0.03을 맞추기 쉬움.' if link == 'thigh' else
            'A면(-Z 쪽)과 Ø42 포켓·C면(+Z 쪽)이 반대 면: 두 번째 셋업은 A면을 바닥 기준으로 잡아 C ∥ A 0.03.'),
        '와셔 자리 11.2 ±0.05: 출력 M3 × 16이 AKE 탭(깊이 5)에 3.9 ~ 4.7 물리고 바닥에 닿지 않음.',
        '스피곳 1.4 (0/-0.05): AKE 홈 깊이 1.5(공차 미표기)보다 낮아 A면이 먼저 닿음 — 핀 3개가 위치, 스피곳은 안내만.',
    ], width=104, title='단면 주기')
    S.title_block(title, num + ' (2/3)', info(name, '1:1, 2:1', mass))
    finish(S, pdf, png_dir, name + '_2')

    # ---- sheet 3: tables and notes
    S = Sheet()
    yb = S.table(14.0, 280.0, [('ID', 9), ('X', 15), ('Y', 15), ('가공', 104), ('용도', 57)], hrows, row_h=4.15,
                 title='구멍 표 (원점 = B축, X·Y는 1쪽 +Z 그림 기준, 위치 공차는 1쪽 기하 공차)')
    yb = S.table(14.0, yb - 8.0, [('Z', 12), ('공차', 22), ('면', 166)], z_rows(link), row_h=4.0,
                 title='두께 방향 면 높이 (Z, %s 판 바깥 면 Z0 기준, +Z = 덮개 쪽)' % ('바깥' if link == 'thigh' else '안쪽'))
    feat = [['B', 'Ø11.8 h9 스피곳, A에서 1.4 (0/-0.05) 돌출 (AKE 출력 홈 Ø12 × 1.5)'],
            ['보스', 'Ø28 (0/-0.1) 구동 보스 A ~ Z %.1f, 이어서 Ø36 g6 스풀이 상자를 꿰뚫음' % (30.0 if link == 'thigh' else 0.0)],
            ['머리', 'R28 외곽, 안쪽 벽 R25.5 (벽 2.5), 상자 보 폭 %.0f, 벽·판 2.5' % (2 * half)],
            ['D', 'Ø42 H7 포켓 깊이 1.5 ±0.05, Ø30 관통, Ø83 하우징 안, Ø88 h8 하우징 바깥'],
            ['리드 홈', '모터 자리 판 모터 쪽 면에 폭 12 (+0.2/0) × 깊이 2, 반경 23 ~ 41 (1쪽 그림 방향)'],
            ['창', '배선 창 2곳: 뒤 벽(Y %s 쪽, 중심 X %.0f · %.0f) 폭 10, 벽 윗면 ~ Z 8.5, 모서리 C0.5 (선 피복 보호)'
             % ('+' if wy > 0 else '-', win[0][0], win[1][0])]]
    S.table(14.0, yb - 8.0, [('형상', 14), ('내용', 186)], feat, row_h=4.0, title='형상')
    yr = S.table(222.0, 280.0, [('부위', 46), ('이 부품', 40), ('상대 부품', 54), ('끼워맞춤 / 틈', 48)],
                 fit_rows(link), row_h=4.15, title='맞물림 공차 (상대 부품과의 끼워맞춤)', fs=FS_S - 0.6)
    yy = S.notes(222.0, yr - 7.0, [
        '단위 mm, 3각법. 지시 없는 공차 KS B ISO 2768-mK. 지시 없는 형상은 같은 이름의 STEP(step/%s.step, 위 원점·축).' % name,
        '소재 AL6061-T6 판 40t (완성 두께 %.1f). 3축 MCT 양면 2셋업, 안쪽 세로 모서리 R ≤ 3 허용(공구 반경).' % (
            lv['spigot'] - 0.0 if link == 'thigh' else lv['cup'] - lv['spigot']),
        '★ 표시 치수는 표면 처리 뒤 값: Ø42 H7·Ø36 g6·Ø3 N7·Ø88 h8·Ø11.8 h9와 A·C면은 마스킹하거나 피막 두께만큼 보정.',
        '표면: 흑색 아노다이징(황산 10 ~ 15 µm) 권장 — 생략 견적도 함께. 탭 M3·M2.5 6H, 처리 뒤 게이지 통과.',
        '버 제거, 바깥 모서리 C0.2 ~ 0.5.  배선 창·Ø28 접근 구멍 입구 C0.5.',
        'P 핀: ISO 8734 Ø3 × 10 m6 경화 3개를 구멍 바닥까지 압입(돌출 2.5, AKE 핀 구멍 깊이 3에 닿지 않음).',
        '%s: %s 부품과 거울상이 아님(모터 출력·고정링·컵 구멍 각도가 다름). 좌우를 바꿔 쓰지 말 것.' % (
            sname, '오른쪽' if side == 'L' else '왼쪽'),
        '완성 질량 약 %.0f g (6061, 덮개·나사 제외).' % (mass * RHO / 1000),
    ], width=104)
    S.title_block(title, num + ' (3/3)', info(name, '-', mass))
    finish(S, pdf, png_dir, name + '_3')


# ----------------------------------------------------------------------------- cover drawing
def draw_cover(name, pdf, png_dir, mass):
    p = PARTS[name]
    link = p['link']
    holes, cones = holes_of(name)
    num = NUMBER[(link, True)]
    title = '%s, 좌우 공용 (%s)' % (TITLE[(link, True)], name)
    half = 25.0 if link == 'thigh' else 28.0
    csk = sorted([h for h in holes if abs(h['d'] - 3.4) < 0.01 and not h['convex']], key=lambda h: (h['cx'], -h['cy']))
    assert len(csk) == 10 and len(cones) == 10, (len(csk), len(cones))
    xmax = G.bbox()[3]

    S = Sheet()
    V = View(S, name, (0, 0, 1), (1, 0, 0), (50.0, 215.0), 1.0, label='바깥 면(+Z, 접시 쪽)에서 본 그림  1:1', label_dy=30)
    V.axis_line((-34, 0, 3), (xmax + 6, 0, 3))
    V.axis_line((0, -34, 3), (0, 34, 3))
    for i, h in enumerate(csk):
        q = V.pt((h['cx'], h['cy'] + (6.0 if h['cy'] > 0 else -6.0), 3))
        S.text(q, 'H%d' % (i + 1), fs=5.0, color='#1f4e9c')
    top = V.box[3]
    S.dim(V.pt((-28, 0, 3)), V.pt((xmax, 0, 3)), top + 8, '%.1f' % (xmax + 28))
    S.dim(V.pt((60, -half, 3)), V.pt((60, half, 3)), V.box[1] + 10, '%.0f' % (2 * half), horizontal=False)
    S.leader(V.pt((-18.02 * 0.707, 18.02 * 0.707, 3)), V.pt((-20, 42, 3)), 'Ø36 H8 (+0.039/0) 리머 (B)')
    S.datum(V.pt((-18.02 * 0.707, -18.02 * 0.707, 3)), V.pt((-34, -14, 3)), 'B')
    S.leader(V.pt((-28 * 0.707, -28 * 0.707, 3)), V.pt((-36, -40, 3)), 'R28')
    S.leader(V.pt((csk[5]['cx'] + 1.0, csk[5]['cy'] - 1.0, 3)), V.pt((csk[5]['cx'] + 22, csk[5]['cy'] - 17, 3)),
             '10 × Ø3.4 관통 + 90° 접시 Ø7.0 (+Z 면)')
    # detail: section through H-hole
    h = csk[2]
    cut = ('y', h['cy'], (h['cx'] - 9, h['cx'] + 9, h['cy'], h['cy'] + 10, -1, 4))
    k = 6.0
    D = View(S, name, (0, -1, 0), (1, 0, 0), (110.0 - k * h['cx'], 112.0), k, hidden=False, cut=cut,
             label='접시 구멍 단면 (H%d 중심, Y = %.2f)  6:1' % (3, h['cy']), label_dy=12)
    R = lambda x, z: D.pt((x, h['cy'], z))  # noqa: E731
    S.dim(R(h['cx'] - 3.5, 3), R(h['cx'] + 3.5, 3), D.box[3] + 6, 'Ø7.0 ±0.1 (90°)')
    S.dim(R(h['cx'] - 1.7, 0), R(h['cx'] + 1.7, 0), D.box[2] - 6, 'Ø3.4 (+0.1/0)')
    S.dim(R(h['cx'] + 9, 0), R(h['cx'] + 9, 3), D.box[1] + 8, '3 (소재)', horizontal=False)
    S.datum(R(h['cx'] - 7, 0), (R(h['cx'] - 7, 0)[0] - 8, R(h['cx'] - 7, 0)[1] - 10), 'A')

    rows = [['H%d' % (i + 1), '%.2f' % hh['cx'], '%.2f' % (hh['cy'] if abs(hh['cy']) > 0.004 else 0.0),
             'Ø3.4 (+0.1/0) 관통 + 90° 접시 Ø7.0 ±0.1' if i == 0 else '〃', 'ISO 10642 M3 × 8 10.9' if i == 0 else '〃']
            for i, hh in enumerate(csk)]
    rows.append(['B', '0.00', '0.00', 'Ø36 H8 (+0.039/0) 관통, 리머 ★', '셸 스풀 Ø36 g6에 끼움'])
    rows.append(['R', '175.00', '0.00', '끝 원호 R44.3 (+0.2/0), 중심 = 셸 D축', '셸 하우징 Ø88·컵 립과 반경 틈 0.3'])
    yb = S.table(222.0, 280.0, [('ID', 9), ('X', 15), ('Y', 15), ('가공', 74), ('용도', 75)], rows, row_h=4.3,
                 title='구멍·원호 표 (원점 = B축, X는 아래 관절 쪽)')
    yy = yb - 8.0
    S.text((222.0, yy + 3.2), '기하 공차', fs=FS, ha='left', weight='bold')
    for items, desc in ((['flat', '0.1'], 'A: 안쪽 면 (셸 벽 윗면에 닿는 면)'),
                        (['perp', 'Ø0.03', 'A'], 'B: Ø36 H8 구멍 축'),
                        (['pos', 'Ø0.1', 'A', 'B'], 'H1 ~ H10 (셸 T 탭과 짝, 접시 머리 중심 맞춤)'),
                        (['pos', 'Ø0.2', 'A', 'B'], 'R44.3 원호 중심 (X 175, Y 0)')):
        yy -= 6.2
        S.fcf(222.0, yy, items)
        S.text((262.0, yy + 2.2), desc, fs=FS_S, ha='left')
    F = FITS_COMMON
    yy = S.table(222.0, yy - 9.0, [('부위', 46), ('이 부품', 40), ('상대 부품', 52), ('끼워맞춤 / 틈', 50)], [
        ['B 구멍 ↔ 셸 스풀', 'Ø36 H8 (+0.039/0)', '셸 스풀 Ø36 g6', clr(F['spool'][2], F['spool'][3])],
        ['H 접시 ↔ 셸 T 탭', '위치 Ø0.1 (A·B)', '셸 M3 탭 위치 Ø0.1', '접시 머리 어긋남 ≤ 0.1'],
        ['H 접시 ↔ M3 접시 머리', 'Ø7.0 ±0.1 90°', 'ISO 10642 M3 dk 6.72', '머리가 면보다 0.1 ~ 0.2 아래'],
        ['R44.3 ↔ 셸 하우징·컵 립', 'R44.3 (+0.2/0)', '하우징 Ø88 h8, 컵 립 Ø88.1', '반경 틈 0.25 ~ 0.55'],
        ['외곽 ↔ 셸 외곽', '일반 공차', '셸 외곽 (같은 선)', '어긋남 ≤ 0.3 (외관)'],
    ], row_h=4.3, title='맞물림 공차', fs=FS_S - 0.4)
    S.notes(222.0, yy - 7.0, [
        '단위 mm. 지시 없는 공차 KS B ISO 2768-mK. 지시 없는 형상은 STEP(step/%s.step).' % name,
        '소재 AL6061-T6 판 3t (두께는 소재 공차), 가공 뒤 평면도 0.1. 2D CNC 한 면, 레이저·워터젯이면 Ø36 H8은 리머 마무리.',
        'A면(Z0)이 셸 벽 윗면에 닿고, 접시는 +Z 면. 버 제거, 모서리 C0.2.',
        '표면: 셸과 같은 흑색 아노다이징 권장. Ø36 H8은 처리 뒤 치수(★).',
        '왼쪽·오른쪽 다리에 같은 부품 (구멍 배치가 링크 축에 대칭): 2개. 접시 면(+Z)이 바깥을 보게 조립.',
        '완성 질량 약 %.0f g.' % (mass * RHO / 1000),
    ], width=96)
    S.title_block(title, num, info(name, '1:1, 6:1', mass))
    finish(S, pdf, png_dir, name)


# ----------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--png', default=None)
    ap.add_argument('--only', default=None, help='comma separated part names')
    a = ap.parse_args()
    names = a.only.split(',') if a.only else list(PARTS)
    PDF_DIR.mkdir(exist_ok=True)
    if a.png:
        pathlib.Path(a.png).mkdir(parents=True, exist_ok=True)
    for name in names:
        vol = make_step(name)
        with PdfPages(PDF_DIR / (name + '.pdf'), metadata={'Title': name, 'Author': PROJECT}) as pdf:
            if PARTS[name]['cover']:
                draw_cover(name, pdf, a.png, vol)
            else:
                draw_shell(name, pdf, a.png, vol)
        print('%s: step/%s.step (%.3f cm3, %.1f g), drawings/%s.pdf' % (name, name, vol / 1000, vol * RHO / 1000, name))


if __name__ == '__main__':
    main()
