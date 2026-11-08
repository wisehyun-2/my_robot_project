"""2D drawings (PDF) and part-frame STEP of the WheelBiped v5 parts.

    python part_drawings.py                  # -> step/*.step, drawings/*.pdf (every part)
    python part_drawings.py --only thigh shank
    python part_drawings.py --png DIR        # also PNG previews

Source: the Fusion export ../export/parts_step/<name>.step (robot frame, Fusion world axes (robot y, z, x); v5run.exports).
  * v5 parts (new in v5, detailed sheets): thigh, shank (PETG FDM print since 11-02; L / R separate parts, the sheets draw
    the right one = CAD +y, the left one has its own STEP), the 6061 hub inserts (hip, knee), the 6061 backing ring (11-02,
    one part for hip / knee, L / R) and the 6061 knee cap (no hip cap since 11-02).  Each is moved to its own
    frame (links: origin = upper joint axis on the inner face, X towards the lower joint, Z outboard; round parts:
    origin on the axis at the face named in the sheet, Z outboard, X = CAD +x, the knee side) and dimensioned from params /
    legs_v5; the hole tables are read back from the geometry.  Part names follow the robot: front = rocker switch = CAD -x,
    _L = CAD -y (user 11-08).
  * the other designed parts (v4 parts used unchanged: cups, hub flange, rim, shells, top plate, tray, deck, saddles,
    splice tabs, hip plate, spine bars, rails, stator clamp ring, magnet spacer): one sheet each, three hidden-line views
    along the robot axes with the overall sizes, material / process from the v4 notes.  The machined ones also have the
    fully dimensioned v2 drawings (robot_cad_v2/manufacturing/drawings), copied when the solid is unchanged.
Views are hidden-line projections (robot_cad_v3/manufacturing/hlr_gmsh.py); dimensions, tables and notes are written
here.  Explanatory production drawings for quotes; the STEP is the master geometry.
"""
import argparse
import math
import os
import pathlib
import shutil
import sys
import unicodedata
from unittest import mock

import numpy as np

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402
from matplotlib.patches import Circle, PathPatch, Polygon  # noqa: E402
from matplotlib.path import Path  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
V5 = HERE.parent
ROOT = V5.parent
sys.path.insert(0, str(ROOT / 'robot_cad_v3' / 'manufacturing'))     # hlr_gmsh
sys.path.insert(0, str(ROOT / 'robot_cad_v4' / 'fusion'))
sys.path.insert(0, str(V5 / 'fusion'))
for _m in ('adsk', 'adsk.core', 'adsk.fusion'):
    sys.modules.setdefault(_m, mock.MagicMock())
import params as P  # noqa: E402
import link_length as LL  # noqa: E402
LL.lengthen(P)                # v5: links 250 mm (user 11-07), before anything reads them at import
import v5_params as VP  # noqa: E402
VP.apply(P)                   # v5-only values (11-08)
import legs as LG  # noqa: E402
import legs_v5 as LV  # noqa: E402
import hlr_gmsh as G  # noqa: E402
import gmsh  # noqa: E402

SRC = V5 / 'export' / 'parts_step'
STEP_DIR = HERE / 'step'
PDF_DIR = HERE / 'drawings'
V2_DRAW = ROOT / 'robot_cad_v2' / 'manufacturing'
DATE = '2026-11-08'
PROJECT = 'WheelBiped v5'

for f in (r'C:\Windows\Fonts\malgun.ttf', '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc'):
    if os.path.exists(f):
        font_manager.fontManager.addfont(f)
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=f).get_name()
        break
plt.rcParams.update({'hatch.linewidth': 0.3, 'hatch.color': 'black', 'pdf.fonttype': 42})

LW_VIS, LW_HID, LW_THIN = 0.7, 0.3, 0.3
FS, FS_S = 7.5, 6.0
CENTRE = (0, (8, 1.5, 1.5, 1.5))
PM = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float)      # Fusion world (y, z, x) -> robot (x, y, z)


# ----------------------------------------------------------------------------- frames
def rows(X, Z, O):
    """3 x 4 [R | t]: Fusion world point -> part frame with axes X, Z (robot vectors), Y = Z x X, origin O (robot)."""
    X, Z = np.array(X, float), np.array(Z, float)
    X, Z = X / np.linalg.norm(X), Z / np.linalg.norm(Z)
    R = np.vstack([X, np.cross(Z, X), Z])
    A, t = R @ PM, -R @ np.array(O, float)
    return [list(A[i]) + [float(t[i])] for i in range(3)]


def link_frame(link, s):
    if link == 'thigh':
        a, b, y0 = (0.0, P.Z_HIP), (P.KNEE_X, P.KNEE_Z), LV.Y_TF
    else:
        a, b, y0 = (P.KNEE_X, P.KNEE_Z), (0.0, P.R_WHEEL), LV.Y_SI
    return rows((b[0] - a[0], 0.0, b[1] - a[1]), (0.0, s, 0.0), (a[0], s * y0, a[1]))


def axis_frame(c, y0, s):
    return rows((1.0, 0.0, 0.0), (0.0, s, 0.0), (c[0], s * y0, c[1]))


HIP, KNEE = (0.0, P.Z_HIP), (P.KNEE_X, P.KNEE_Z)
Y_OUT = {j: P.y_output(j) for j in ('hip', 'knee')}
CAP0 = LV.SPAR[1] - LV.CAP_T
DETAIL = {   # name: (source stem, frame(s), title, number)
    'thigh': ('thigh', lambda s: link_frame('thigh', s), '허벅지', 'WBv5-P01'),
    'shank': ('shank', lambda s: link_frame('shank', s), '정강이', 'WBv5-P02'),
    'hub_insert_hip': ('hub_insert_hip', lambda s: axis_frame(HIP, Y_OUT['hip'], s), 'hip hub 인서트', 'WBv5-M01'),
    'hub_insert_knee': ('hub_insert_knee', lambda s: axis_frame(KNEE, Y_OUT['knee'], s), '무릎 hub 인서트', 'WBv5-M02'),
    'knee_cap': ('knee_cap', lambda s: axis_frame(KNEE, LV.Y_SH - LV.BRG[2] + 0.5, s), '무릎 캡', 'WBv5-M03'),
    'hub_ring': ('hub_ring_knee', lambda s: axis_frame(KNEE, LV.y_ring(LV.Y_SI), s), '받침 링 (hip · 무릎)', 'WBv5-M04'),
}


# ----------------------------------------------------------------------------- part-frame STEP
def signature():
    G.mesh(curv=24, hmax=6.0)
    holes = G.group_holes(G.z_cylinders())
    gmsh.model.mesh.clear()
    return round(G.volume(), 1), holes


def same_rotated(hl, hr):
    """An angle (deg) that turns the Z holes of one side's part (hl) onto the other side's (hr), or None."""
    key = lambda h: (round(h['d'], 2), h['convex'], round(h['z0'], 2), round(h['z1'], 2))
    ref = [h for h in hl if math.hypot(h['cx'], h['cy']) > 0.5]
    if not ref:
        return 0.0
    h0 = ref[0]
    a0 = math.atan2(h0['cy'], h0['cx'])
    for h in hr:
        if key(h) != key(h0) or abs(math.hypot(h['cx'], h['cy']) - math.hypot(h0['cx'], h0['cy'])) > 0.02:
            continue
        th = math.atan2(h['cy'], h['cx']) - a0
        c, s_ = math.cos(th), math.sin(th)
        left = [(key(q), c * q['cx'] - s_ * q['cy'], s_ * q['cx'] + c * q['cy']) for q in hl]
        if len(hl) == len(hr) and all(any(key(r) == kq and math.hypot(r['cx'] - x, r['cy'] - y) < 0.05 for r in hr)
                                      for kq, x, y in left):
            return math.degrees(th) % 360
    return None


def make_steps(name):
    """Part-frame STEP of L and R; returns dict(vol, holes, shared (None = L / R different), angle)."""
    stem, fr, _t, _n = DETAIL[name]
    sig = {}
    STEP_DIR.mkdir(exist_ok=True)
    for side, s in (('R', 1), ('L', -1)):         # s = sign of CAD y; the sheets draw the +y part, the robot's right
        G.load(SRC / ('%s_%s.step' % (stem, side)), fr(s))
        assert len(gmsh.model.getEntities(3)) == 1
        G.write_step(STEP_DIR / ('%s_%s.step' % (name, side)))
        sig[side] = signature()
    ang = None
    if name not in ('thigh', 'shank') and abs(sig['R'][0] - sig['L'][0]) < 0.5:
        ang = same_rotated(sig['R'][1], sig['L'][1])
    return dict(vol=sig['R'][0], holes=sig['R'][1], shared=ang is not None, angle=ang)


# ----------------------------------------------------------------------------- sheet (v3 link_drawings.Sheet)
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
    def __init__(self, w=420.0, h=297.0):
        self.w, self.h = w, h
        self.fig = plt.figure(figsize=(w / 25.4, h / 25.4))
        self.ax = self.fig.add_axes([0, 0, 1, 1])
        self.ax.set_xlim(0, w)
        self.ax.set_ylim(0, h)
        self.ax.set_aspect('equal')
        self.ax.axis('off')
        self.rect(10, 10, w - 10, h - 10, lw=1.2)

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

    def table(self, x, y, cols, rows_, fs=FS_S, row_h=4.4, title=None):
        """cols: [(header, width)], rows_: lists of strings, wrapped to the column width.  Returns the bottom y."""
        if title:
            self.text((x, y + 3.2), title, fs=FS, ha='left', weight='bold')
        unit = fs * 0.3528 * 0.55
        line_h = fs * 0.3528 * 1.3
        cells = [[wrap(c, max(4, int((w - 2.0) / unit))) for c, (_, w) in zip(r, cols)] for r in rows_]
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

    def title_block(self, name, number, info):
        w, rh = 190.0, 5.6
        x0, y0 = self.w - 10 - w, 10.0
        rows_ = [('품명', name, '도번', number),
                 ('재질', info['재질'], '수량', info['수량']),
                 ('공정', info['공정'], '척도', info['척도']),
                 ('후처리', info.get('후처리', '-'), '투상', '3각법'),
                 ('STEP', info['step'], '일반 공차', info.get('공차', 'KS B ISO 2768-mK')),
                 ('프로젝트', PROJECT + ' (robot_cad_v5)', '작성일', DATE)]
        cw = (16.0, 106.0, 18.0, 50.0)
        hh = rh * len(rows_)
        self.rect(x0, y0, x0 + w, y0 + hh, lw=1.0)
        self.rect(x0, y0, x0 + w, y0 + hh, lw=0, fill=True, color='white')
        self.rect(x0, y0, x0 + w, y0 + hh, lw=1.0)
        for i in range(1, len(rows_)):
            self.line([(x0, y0 + i * rh), (x0 + w, y0 + i * rh)], lw=0.3)
        cx = x0
        for c in cw[:-1]:
            cx += c
            self.line([(cx, y0), (cx, y0 + hh)], lw=0.3)
        for i, r in enumerate(rows_):
            yy = y0 + hh - (i + 0.5) * rh
            cx = x0
            for k, (c, s) in enumerate(zip(cw, r)):
                self.text((cx + 1.0, yy), s, fs=FS_S if k % 2 == 0 else FS - 0.5, ha='left')
                cx += c
        return y0 + hh


class View:
    """Hidden-line view of a STEP (part frame).  toward = from the part to the viewer, xdir = screen right;
    paper = origin + scale * (p . xdir, p . (toward x xdir)).  cut = (axis, value, keep_box): section, the faces on the
    plane hatched; crop = box for a detail."""

    def __init__(self, S, path, toward, xdir, origin, scale, hidden=True, cut=None, crop=None, label=None,
                 label_dy=8.0, mesh=(72, 2.5)):
        self.S, self.o, self.k = S, origin, scale
        G.load(path)
        if cut:
            G.keep_box(*cut[2])
        if crop:
            G.keep_box(*crop)
        G_mesh = G.mesh
        G.mesh = lambda curv=mesh[0], hmax=mesh[1]: G_mesh(curv, hmax)
        try:
            vis, hid, pr = G.project(toward, xdir)
        finally:
            G.mesh = G_mesh
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
        S.fig.savefig(pathlib.Path(png_dir) / (tag + '.png'), dpi=110)
    plt.close(S.fig)


# ----------------------------------------------------------------------------- hole tables
HOLE_NAMES = [   # (diameter, convex, name) for the Z holes of the v5 parts
    (LG.M3_CLEAR, False, 'M3 통과 Ø3.4'), (LG.M25_CLEAR, False, 'M2.5 통과 Ø2.8'),
    (LG.INSERT_M25, False, 'M2.5 열압입 인서트 구멍 Ø3.6'), (P.TAP_DRILL['M3'], False, 'M3 탭 (드릴 Ø2.5)'),
    (3.0, False, '평행핀 Ø3 구멍'), (7.2, False, 'M3 머리 카운터보어 Ø7.2'),
    (28.0, False, 'Ø28 접근 구멍'), (30.0, False, 'Ø30 출력 구멍'), (42.05, False, 'Ø42.05 파일럿 포켓'),
    (P.STATOR_RECESS_D, False, 'Ø45.6 고정 링 홈'), (2 * LV.INSERT_FL[0] + 0.3, False, 'Ø48.3 인서트 플랜지 자리'),
    (2 * (LV.CAP_R[2] + 0.2), False, 'Ø46.4 캡 자리'), (2 * (LV.CAP_R[1] + LV.CAP_FIT), False, 'Ø35 캡 관 구멍 (H7)'),
    (LV.BRG[1], False, 'Ø47 61807 자리'), (2 * LV.BORE_R, False, 'Ø44 접근 구멍 (받침 링)'),
    (2 * LV.RING[1], True, 'Ø43 받침 링'), (2 * LV.RING[0], False, 'Ø29 받침 링 안'),
    (2 * P.CAN_CLEAR_R, False, 'Ø74 캔 공간'), (18.0, False, 'Ø18 구멍'), (2 * LV.CAP_R[0], False, 'Ø29 캡 관 안'),
    (2 * P.HOUSING_R, True, 'Ø88 하우징 (컵 립 끼움)'), (28.0, True, 'Ø28 보스'), (11.8, True, 'Ø11.8 스피곳'),
    (2 * LV.INSERT_FL[0], True, 'Ø48 플랜지'), (2 * LV.CAP_R[1], True, 'Ø35 관 (61807 안륜)'),
    (2 * LV.CAP_R[2], True, 'Ø46 플랜지'),
    (2 * (LV.R_HIP - LV.WALL), False, 'hip 머리 속 (벽 안쪽)'), (2 * LV.SPOOL_R, True, '스풀 관 Ø52 (Ø44 구멍 둘레)'),
    (2 * LV.R_BOSS, True, '바깥 막대 보스 Ø56 (캡 둘레)'),
    (2 * LV.THIGH[1][1], True, '바깥 막대 끝 R30 (외형)'), (2 * LV.R_HIP, True, 'hip 머리 R38 (외형)'),
    (2 * LV.SHANK[0][1], True, '무릎 머리 R45 (외형)'), (2 * (LV.SHANK[0][1] - LV.WALL), False, '무릎 머리 속 (벽 안쪽)'),
    (2 * LV.PAD_R, True, '머리 패드 R70 (외형)'), (2 * (LV.PAD_R - LV.WALL), False, '머리 패드 속 (벽 안쪽)'),
    (2 * LV.SHANK[1][1], True, '바퀴 쪽 R22.5 (외형)'),
]


def hole_name(h):
    best = min(HOLE_NAMES, key=lambda r: (r[1] != h['convex'], abs(r[0] - h['d'])))
    return best[2] if abs(best[0] - h['d']) < 0.06 and best[1] == h['convex'] else (
        'Ø%.2f %s' % (h['d'], '바깥 원통' if h['convex'] else '구멍'))


def hole_rows(holes, z_name='Z', skip=lambda h: False):
    """Group the Z cylinders by (diameter, convex, z range): one row per kind with the count and positions."""
    g = {}
    for h in holes:
        if skip(h):
            continue
        k = (round(h['d'], 2), h['convex'], round(h['z0'], 1), round(h['z1'], 1))
        g.setdefault(k, []).append(h)
    out = []
    for (d, cv, z0, z1), hs in sorted(g.items(), key=lambda kv: (kv[0][1], kv[0][0])):
        pos = ', '.join('(%.1f, %.1f)' % (h['cx'], h['cy']) for h in hs[:6]) + (' …' if len(hs) > 6 else '')
        rr = sorted({round(math.hypot(h['cx'], h['cy']), 2) for h in hs})
        where = pos if len(hs) <= 3 or len(rr) != 1 else 'r %.2f 원 위 %d개' % (rr[0], len(hs))
        out.append([hole_name(hs[0]), 'Ø%.2f' % d, str(len(hs)), where, '%s %.1f ~ %.1f' % (z_name, z0, z1)])
    return out


# ----------------------------------------------------------------------------- detailed v5 sheets
def info_v5(name, scale, mass_g, shared, extra):
    base = {'step': 'step/%s_R.step, %s_L.step' % (name, name), '척도': scale}
    base.update(extra)
    if '수량' not in base:
        base['수량'] = '2개 (왼쪽·오른쪽 같은 부품)' if shared else '왼쪽 1 + 오른쪽 1 (서로 다른 부품)'
    return base


def link_sheets(name, pdf, png_dir, res):
    """Thigh / shank: (1) outer and inner face views, (2) section through both joint axes + tables."""
    stem, _fr, title, number = DETAIL[name]
    thigh = name == 'thigh'
    path_drawn = STEP_DIR / ('%s_R.step' % name)
    L = P.L1 if thigh else P.L2
    r_a, r_b = (LV.THIGH if thigh else LV.SHANK)[0][1], (LV.THIGH if thigh else LV.SHANK)[1][1]
    y0 = LV.Y_TF if thigh else LV.Y_SI
    zf = (LV.SPAR[1] if thigh else LV.Y_SH) - y0                  # outer face Z
    mass = res['vol'] / 1000
    extra = {'재질': 'PETG (FDM, 11-02) — calc/strength_v5.md', '공정': 'FDM 한 덩어리: 벽 3.5 둘레선, 채움 100 %, 닫힌 칸',
             '후처리': 'M2.5 열압입 인서트, 끼움 자리 다듬기', '공차': '출력 ±0.2 (끼워맞춤은 주기)'}
    j_up, j_lo = ('hip', '무릎') if thigh else ('무릎', '바퀴')

    # ---- sheet 1: face views
    S = Sheet()
    ox = 72.0
    VA = View(S, path_drawn, (0, 0, 1), (1, 0, 0), (ox, 216.0), 1.0, label='바깥 면 쪽(+Z)에서 본 그림  1:1  (원점 = %s 축)' % j_up,
              label_dy=11)
    for V, z in ((VA, zf),):
        V.axis_line((-r_a - 6, 0, z), (L + 50, 0, z))
        V.axis_line((0, -r_a - 6, z), (0, r_a + 6, z))
        V.axis_line((L, -50, z), (L, 50, z))
    top = VA.box[3]
    S.dim(VA.pt((0, 0, zf)), VA.pt((L, 0, zf)), top + 6, '%.0f ±0.2  (%s 축 ~ %s 축)' % (L, j_up, j_lo))
    x_end = L + P.HOUSING_R
    S.dim(VA.pt((-r_a, 0, zf)), VA.pt((x_end, 0, zf)), top + 14, '%.0f' % (r_a + x_end))
    S.leader(VA.pt((-r_a * 0.707, r_a * 0.707, zf)), (VA.pt((-r_a, r_a, zf))[0] + 2, top + 3), 'R%.0f' % r_a, right=False)
    VB = View(S, path_drawn, (0, 0, -1), (1, 0, 0), (ox, 106.0), 1.0,
              label='안쪽 면 쪽(-Z)에서 본 그림  1:1  (X축으로 뒤집어 봄: 위·아래가 바뀜)', label_dy=10)
    VB.axis_line((-r_a - 6, 0, 0), (L + 50, 0, 0))
    VB.axis_line((0, -r_a - 6, 0), (0, r_a + 6, 0))
    VB.axis_line((L, -50, 0), (L, 50, 0))
    if thigh:
        S.dim(VB.pt((L, -P.HOUSING_R, 0)), VB.pt((L, P.HOUSING_R, 0)), VB.box[1] + 8, 'Ø%.0f (컵 립 끼움)'
              % (2 * P.HOUSING_R), horizontal=False)
    S.title_block('%s 오른쪽 (%s_R)' % (title, name), number + ' (1/3)', info_v5(name, '1:1', mass, False, extra))
    finish(S, pdf, png_dir, name + '_1')

    # ---- sheet 2: section through both joint axes (Y = 0), Z levels, notes
    S = Sheet()
    cut = ('y', 0.0, (-60, L + 60, 0.0, 60, -10, zf + 10))
    VS = View(S, path_drawn, (0, -1, 0), (1, 0, 0), (ox, 168.0), 1.0, hidden=False, cut=cut,
              label='단면 A-A  1:1  (Y = 0: 두 관절 축을 지나는 면, 아래 = 안쪽 면)', label_dy=9)
    zl = levels(name)
    xr = VS.box[1] + 6
    zs = sorted(set(round(v, 2) for v in zl.values()))
    for i, (a_, b_) in enumerate(zip(zs, zs[1:])):
        S.dim(VS.pt((L + 52, 0, a_)), VS.pt((L + 52, 0, b_)), xr + 4 + 9 * (i % 2), '%.1f' % (b_ - a_), horizontal=False)
    S.dim(VS.pt((-r_a, 0, 0)), VS.pt((-r_a, 0, zf)), VS.box[0] - 8, '%.0f' % zf, horizontal=False)
    zrows = [['%.1f' % v, k] for k, v in sorted(zl.items(), key=lambda kv: kv[1])]
    S.table(20.0, 128.0, [('Z', 14), ('면 (y = 로봇 오른쪽 다리의 CAD y)', 96)], zrows, title='두께 방향 면 높이 (Z, 안쪽 면 Z0, +Z = 바깥)',
            row_h=4.0)
    S.notes(150.0, 128.0, link_notes(name), width=118)
    S.title_block('%s 오른쪽 (%s_R)' % (title, name), number + ' (2/3)', info_v5(name, '1:1', mass, False, extra))
    finish(S, pdf, png_dir, name + '_2')

    # ---- sheet 3: hole table (its own sheet since the links are 250 mm: on sheet 1 it covered the lower joint)
    S = Sheet()
    hr = hole_rows(res['holes'], skip=lambda h: h['d'] > 80 and not h['convex'])
    S.table(20.0, 278.0, [('구멍·원통', 90), ('Ø', 22), ('개', 12), ('위치 (X, Y) 또는 원', 180), ('Z', 50)], hr,
            title='Z 방향 구멍·원통 표 (원점 %s 축 · 안쪽 면, 오른쪽 부품)' % j_up, row_h=5.0, fs=FS)
    S.title_block('%s 오른쪽 (%s_R)' % (title, name), number + ' (3/3)', info_v5(name, '1:1', mass, False, extra))
    finish(S, pdf, png_dir, name + '_3')


def levels(name):
    """{face: Z} of a link in its part frame (inner face Z0)."""
    if name == 'thigh':
        y0 = LV.Y_TF
        d = {'안쪽 면 (y %.0f)' % LV.Y_TF: LV.Y_TF, '무릎 하우징 끝 (컵 립, y %.0f)' % LV.Y_TI: LV.Y_TI,
             '무릎 판 모터 쪽 (y %.0f)' % (LV.Y_TO - P.KNEE_PLATE_T): LV.Y_TO - P.KNEE_PLATE_T,
             'Ø42.05 포켓 바닥 (y %.1f)' % P.Y_RING_KNEE: P.Y_RING_KNEE,
             '안쪽 갈래 바깥 면 = 무릎 판 바깥 (y %.0f)' % LV.Y_TO: LV.Y_TO,
             '바깥 막대 안쪽 면 (y %.0f)' % LV.SPAR[0]: LV.SPAR[0], '바깥 면 (y %.0f)' % LV.SPAR[1]: LV.SPAR[1],
             '무릎 캡 자리 바닥 (y %.1f)' % CAP0: CAP0,
             'hub 인서트 플랜지 자리 바닥 (y %.0f)' % (LV.Y_TF + LV.INSERT_FL[1]): LV.Y_TF + LV.INSERT_FL[1],
             '받침 링 자리 = Ø44 구멍 시작 (y %.0f)' % LV.y_ring(LV.Y_TF): LV.y_ring(LV.Y_TF)}
    else:
        y0 = LV.Y_SI
        d = {'안쪽 면 = 바퀴 하우징 끝 (컵 립, y %.0f)' % LV.Y_SI: LV.Y_SI, '기본층 바깥 면 (y %.0f)' % LV.Y_SO: LV.Y_SO,
             '머리 패드 바깥 면 (y %.0f)' % LV.Y_SH: LV.Y_SH, '61807 자리 바닥 = 턱 (y %.0f)' % (LV.Y_SH - LV.BRG[2]): LV.Y_SH - LV.BRG[2],
             'hub 인서트 플랜지 자리 바닥 (y %.0f)' % (LV.Y_SI + LV.INSERT_FL[1]): LV.Y_SI + LV.INSERT_FL[1],
             '받침 링 자리 = Ø44 구멍 시작 (y %.0f)' % LV.y_ring(LV.Y_SI): LV.y_ring(LV.Y_SI),
             '바퀴 판 모터 쪽 (y %.1f)' % (LV.Y_SO - P.WHEEL_PLATE_T): LV.Y_SO - P.WHEEL_PLATE_T}
    return {k: v - y0 for k, v in d.items()}


def link_notes(name):
    thigh = name == 'thigh'
    common = [
        '단위 mm, 3각법.  형상의 원본은 STEP(step/%s_R.step · _L.step, 위 원점·축).  왼쪽 부품은 거울상에 가깝지만 모터 판의 '
        '리드 홈·컵 인서트 각도가 다르므로 _L STEP으로 따로 출력한다.' % name,
        '닫힌 상자 벽 %.1f, 와렌 리브 %.1f (구멍 없음).  리브가 나눈 칸은 닫힌 빈칸 — 배출 구멍 없음(11-02), FDM 출력 기준 '
        '(MJF로 뽑으면 칸 속 가루가 빠지지 않는다).' % (LV.WALL, LV.RIB),
        '열압입 인서트는 M2.5 × 4 (구멍 Ø3.6)만 — 컵 기둥 · 무릎 캡.  hub 인서트는 열압입 인서트 대신 PETG 벽 %.0f mm를 '
        '6061 받침 링(Ø44 구멍으로 넣음)과 끼워 ISO 10642 M3 × 16 12.9 6개 (r %.0f)로 고정(11-02).' % (LV.WALL_J, LV.FLANGE_BOLT_R),
        'Ø42.05 파일럿 포켓·Ø47 61807 자리·Ø48.3 플랜지 자리·Ø35 캡 관 구멍은 출력 오차만큼 실물에 맞춰 다듬는다(리머·사포).',
    ]
    if thigh:
        return common + [
            '안 A: 바깥 막대 y %.1f ~ %.1f, 갈래 틈 %.1f — 정강이를 9.5 mm 바깥에 띄워 넣고 안쪽으로 밀어 조립 (11-02 바퀴 묶음을 '
            '2.5 mm 바깥으로 옮겨 막대도 2.5 바깥).' % (LV.SPAR[0], LV.SPAR[1], LV.SPAR[0] - LV.Y_TO),
            'Ø35 캡 관 구멍: 무릎 캡 관 Ø35 g6에 꼭 맞게 H7 (11-02) — 0.2 작게 뽑고 리머.',
            'Ø88 무릎 하우징은 안쪽 면보다 4 mm 나와 있다(컵 립 y 174 ~ 177 자리) — 출력은 바깥 면(평평)을 바닥에.',
            'hip 쪽 Ø44 접근 구멍(받침 링 넣기, hip 출력 나사 조임)은 바깥 면에 열린 채 둔다 — hip 캡 없음(11-02).',
        ]
    return common + [
        '61807 자리 Ø47 × 7, 그 아래 Ø44 구멍 — 턱 r 22 ~ 23.5에 바깥륜만 앉는다(안륜은 자유).  바깥륜 압입.',
        '바퀴 판(9 mm, 바퀴 쪽 면에 Ø45.6 × 3.7 고정 링 홈)과 컵 인서트는 v4 바퀴 자리를 바퀴와 함께 180° 돌린 것.',
        '안쪽 면(y %.0f)은 바퀴 하우징 끝과 같은 높이로 평평 — 출력 바닥면.  허벅지 무릎 판과 틈 %.1f (11-02, 전 1.5).'
        % (LV.Y_SI, LV.KNEE_GAP),
    ]


def round_sheet(name, pdf, png_dir, res):
    """Hub inserts, knee cap: end views (both faces) and the axial section, 2:1, hole table, notes."""
    stem, _fr, title, number = DETAIL[name]
    path_drawn = STEP_DIR / ('%s_R.step' % name)
    k = 2.0
    S = Sheet()
    G.load(path_drawn)
    bb = G.bbox()
    zmin, zmax, rmax = bb[2], bb[5], max(abs(bb[0]), abs(bb[3]), abs(bb[1]), abs(bb[4]))
    V1 = View(S, path_drawn, (0, 0, -1), (1, 0, 0), (25.0 + k * rmax, 200.0), k, label='-Z 쪽에서 본 그림  2:1', label_dy=9)
    V2 = View(S, path_drawn, (0, 0, 1), (1, 0, 0), (25.0 + k * rmax, 85.0), k, label='+Z 쪽에서 본 그림  2:1', label_dy=9)
    cut = ('y', 0.0, (-rmax - 5, rmax + 5, 0.0, rmax + 5, zmin - 5, zmax + 5))
    xs = 25.0 + 2 * k * rmax + 40.0
    V3 = View(S, path_drawn, (0, -1, 0), (0, 0, 1), (xs - k * zmin, 200.0), k, hidden=True, cut=cut,
              label='단면 A-A  2:1  (Y = 0, 축을 지나는 면; 가로 = Z)', label_dy=12 + 7 * len(round_z_dims(name)))
    for V, z in ((V1, zmin), (V2, zmax)):
        V.axis_line((-rmax - 3, 0, z), (rmax + 3, 0, z))
        V.axis_line((0, -rmax - 3, z), (0, rmax + 3, z))
    V3.axis_line((0, 0, zmin - 3), (0, 0, zmax + 3))
    S.dim(V3.pt((zmin * 0, 0, zmin)), V3.pt((0, 0, zmax)), V3.box[3] + 6, '%.1f' % (zmax - zmin))
    for z_a, z_b in round_z_dims(name):
        S.dim(V3.pt((0, 0, z_a)), V3.pt((0, 0, z_b)), V3.box[2] - 6 - 7 * round_z_dims(name).index((z_a, z_b)),
              '%.1f' % abs(z_b - z_a))
    for i, (r_, z_) in enumerate(round_r_dims(name)):
        S.dim(V3.pt((r_, 0, z_)), V3.pt((-r_, 0, z_)), V3.box[1] + 8 + 9 * i, 'Ø%.2f' % (2 * r_), horizontal=False)
    S.notes(290.0, 282.0, round_notes(name, res), width=66)
    S.table(150.0, 128.0, [('구멍·원통', 44), ('Ø', 13), ('개', 6), ('위치 (X, Y) 또는 원', 34), ('Z', 25)],
            hole_rows(res['holes']), title='Z 방향 구멍·원통 표 (원점 = 주기 1의 면, 축 위)', row_h=4.0, fs=FS_S - 0.6)
    mass = res['vol'] / 1000 * (2.70 if name.startswith(('hub', 'knee')) else 1.01)
    S.title_block('%s (%s)' % (title, name), number, info_v5(name, '2:1', mass, res['shared'], round_info(name, res, mass)))
    finish(S, pdf, png_dir, name)


def round_z_dims(name):
    if name.startswith('hub_insert'):
        y_li = LV.Y_TF if name.endswith('hip') else LV.Y_SI
        z_fl = y_li - Y_OUT['hip' if name.endswith('hip') else 'knee']
        return [(0.0, z_fl), (z_fl, z_fl + LV.INSERT_FL[1]), (0.0, P.BEARING_FACE), (-1.4, 0.0)]
    if name == 'hub_ring':
        return [(0.0, LV.RING[2])]
    Lt = LV.SPAR[1] - (LV.Y_SH - LV.BRG[2] + 0.5)                  # knee cap
    return [(0.0, Lt - LV.CAP_T), (Lt - LV.CAP_T, Lt), (Lt - 2.0, Lt)]


def round_r_dims(name):
    if name.startswith('hub_insert'):
        y_li = LV.Y_TF if name.endswith('hip') else LV.Y_SI
        z_fl = y_li - Y_OUT['hip' if name.endswith('hip') else 'knee']
        return [(14.0, z_fl * 0.5), (LV.INSERT_FL[0], z_fl + 3.0), (5.9, -0.7), (9.0, z_fl + 5.0)]
    if name == 'hub_ring':
        return [(LV.RING[1], 4.0), (LV.RING[0], 2.0)]
    Lt = LV.SPAR[1] - (LV.Y_SH - LV.BRG[2] + 0.5)                  # knee cap
    return [(LV.CAP_R[1], 6.0), (LV.CAP_R[2], Lt - 1.5), (LV.CAP_R[0], 10.0)]


def round_info(name, res, mass):
    if name.startswith('hub_insert'):
        return {'재질': 'AL6061-T6 환봉 Ø50', '공정': '선반 + MCT (구멍·카운터보어·접시)', '후처리': '흑색 아노다이징 권장 (끼움면 마스킹)'}
    if name == 'hub_ring':
        return {'재질': 'AL6061-T6 환봉 Ø45', '공정': '선반 + 드릴 · M3 탭 6', '후처리': '버 제거', '수량': '4개 (hip · 무릎 × 왼 · 오른 같은 부품)'}
    return {'재질': 'AL6061-T6 환봉 Ø50', '공정': '선반 + 드릴 (M2.5 구멍 3)', '후처리': '흑색 아노다이징 권장'}


def round_notes(name, res):
    rot = '' if not res['shared'] else (
        '  왼쪽 다리도 같은 부품 — 왼쪽 STEP과 비교해 같은 모양(축 둘레로 %.0f° 돌려 끼움).' % res['angle'])
    if name.startswith('hub_insert'):
        j = 'hip' if name.endswith('hip') else '무릎'
        return [
            '단위 mm, 3각법, 원점 = 축 위 모터 출력 면 쪽 끝면(Z0, 스피곳 시작), +Z = 링크 쪽.%s' % rot,
            '%s 모터(AKE60-8) 출력부에 Ø11.8 h9 스피곳 × %.1f (0/-0.05) — AKE 홈 Ø12 × 1.5보다 %.1f 낮아 Z0 면이 먼저 닿음 (11-08).' % (
                j, LV.INSERT_SPIGOT[1], 1.5 - LV.INSERT_SPIGOT[1]),
            '평행핀 구멍 Ø3 N7 × %.1f (+0.2/0) (출력부 핀과 같은 PCD 20, M3와 30° 어긋남) — 핀 Ø3 × 10을 바닥까지 압입, %.1f 돌출 '
            '(출력부 핀 구멍 깊이 3: 얕게 가공되면 핀이 먼저 바닥에 닿음).' % (P.DOWEL_HOLE, 10 - P.DOWEL_HOLE),
            '출력 M3 × 16 12.9: Ø3.4 통과 + Ø7.2 카운터보어, 와셔 자리(바닥) Z %.1f ±0.05 — 탭 물림 4.3.' % P.BEARING_FACE,
            'Ø28 보스 (0/-0.1): 링크 판 Ø30 구멍과 반경 틈 1.0.  플랜지 Ø48 (-0.05/-0.15) → 링크 자리 Ø48.3.',
            '플랜지 나사 6개: Ø3.4 통과 + Ø%.1f × 90° 접시 (r %.0f, 30° + 60° 간격) — ISO 10642 M3 × 16 12.9 머리가 면과 같게, '
            'PETG 벽 %.0f를 지나 6061 받침 링 탭에 (11-02).' % (LV.INSERT_CSK, LV.FLANGE_BOLT_R, LV.WALL_J),
            '버 제거, 모서리 C0.2.  질량 약 %.0f g.' % (res['vol'] / 1000 * 2.70),
        ]
    if name == 'hub_ring':
        return [
            '단위 mm, 3각법, 원점 = 축 위 PETG 벽 쪽 면(Z0), +Z = 링크 바깥 쪽.  hip · 무릎 · 왼쪽 · 오른쪽 같은 부품 4개.',
            'M3 탭 6개 관통 (r %.0f, 30° + 60° 간격 — hub 인서트 플랜지 구멍과 같은 각도), 탭 길이 %.0f 전체.'
            % (LV.FLANGE_BOLT_R, LV.RING[2]),
            'Ø%.0f (-0.05/-0.15) → 링크의 Ø%.0f 접근 구멍에 반경 0.5로 떨어뜨려 넣고, 벽의 Ø3.4 구멍에 맞춰 ISO 10642 M3 × 16 '
            '12.9 6개로 hub 인서트 플랜지와 PETG 벽 %.0f를 끼움 (조임 약 0.7 N·m + 나사 고정제).'
            % (2 * LV.RING[1], 2 * LV.BORE_R, LV.WALL_J),
            '안지름 Ø%.0f: 출력 M3 나사 렌치가 지나감.  버 제거, 모서리 C0.3.  질량 약 %.0f g.' % (2 * LV.RING[0], res['vol'] / 1000 * 2.70),
        ]
    return [                                                        # knee cap
        '단위 mm, 3각법, 원점 = 축 위 관 끝(61807 안, Z0), +Z = 바깥 막대 쪽(플랜지).%s' % rot,
        '관 Ø35 g6 → 61807 안륜(Ø35), 바깥 막대 구멍 Ø35 H7 (꼭 맞음, 11-02).',
        '플랜지 Ø46 (-0.05/-0.15) × 3 → 막대 바깥 면 자리 Ø46.4 × 3 (면과 같은 높이).  끝은 2 mm 면으로 막힘.',
        'M2.5 구멍 Ø2.8 × 3 (r %.1f, 90° + 120° 간격) — DIN 7984 M2.5 × 8 → 막대 인서트.' % LV.CAP_BOLT_R,
        '버 제거, 모서리 C0.2.  질량 약 %.0f g.' % (res['vol'] / 1000 * 2.70),
    ]


# ----------------------------------------------------------------------------- generic sheets (other parts)
OTHER = {   # stem: (title, material, process, quantity, sides; the first side is the one drawn: CAD +y, +x)
    'cup_hip': ('hip 드라이버 컵', 'PETG', '3D 프린트 30 %, 바닥(덮개 면)을 바닥에, 서포트는 커넥터 창만', '왼 1 + 오른 1', 'RL'),
    'cup_knee': ('무릎 드라이버 컵', 'PETG', '3D 프린트 30 %, 바닥을 바닥에, 서포트는 커넥터 창만', '왼 1 + 오른 1', 'RL'),
    'cup_wheel': ('바퀴 드라이버 컵', 'PETG', '3D 프린트 30 %, 바닥을 바닥에, 서포트는 커넥터 창만', '왼 1 + 오른 1', 'RL'),
    'hub_flange': ('바퀴 허브 플랜지', 'PETG', '3D 프린트 100 %, 모터 쪽 면을 바닥에, M3 × 5.7 인서트 6', '왼 1 + 오른 1', 'RL'),
    'rim': ('바퀴 림 (한 조각)', 'PETG', '3D 프린트 벽 6겹 · 채움 40 %, 허브 쪽 평면을 바닥에', '왼 1 + 오른 1', 'RL'),
    'shell_front': ('몸통 셸 앞', 'PETG 진회색', '3D 프린트 25 %, 바깥 면을 위로', '왼 1 + 오른 1', 'RL'),
    'shell_mid': ('몸통 셸 가운데 (hip)', 'PETG 진회색', '3D 프린트 25 %, 바깥 면을 위로', '왼 1 + 오른 1', 'RL'),
    'shell_rear': ('몸통 셸 뒤', 'PETG 진회색', '3D 프린트 25 %, 바깥 면을 위로', '왼 1 + 오른 1', 'RL'),
    'saddle': ('hip 위 안장', 'PETG 파랑', '3D 프린트 25 %', '왼 1 + 오른 1', 'RL'),
    'top_plate': ('위판 (짐 판)', 'PETG', '3D 프린트 25 %, M4 열압입 인서트', '1', ''),
    'battery_tray': ('배터리 받침', 'PETG', '3D 프린트 30 %, 리브를 위로', '1', ''),
    'electronics_deck': ('전장 데크', 'PETG', '3D 프린트 30 %', '1', ''),
    'splice_tabs': ('셸 이음판', 'PETG', '3D 프린트 100 %, M3 + 너트', '1 세트', ''),
    'hip_plate': ('hip 판', 'AL6061-T6 판 10t 이상', '3축 MCT 양면 (v2 도면 WBv2-M01 참고)', '왼 1 + 오른 1', 'RL'),
    'spine_tube': ('척추 봉', 'AL6061-T6 환봉 Ø16', '선반: 양 끝 면삭 + M5 탭', '2 (앞 · 뒤)', 'BF'),
    'rail': ('위 레일', 'AL6061-T6 각봉 15 × 15', '양 끝 M4 탭, Ø4.4 관통 4', '2 (앞 · 뒤)', 'BF'),
    'clamp_ring_stator_D45_t2.0_SUS304': ('고정 링 (고정자)', 'SUS304 2t', '레이저 가공', '4 (무릎·바퀴 × 2)', ''),
    'magnet_spacer_D9x5.35_(6061)': ('자석 스페이서', 'AL6061 (비자성)', '선반', '6', ''),
}
V2_COPY = {'hip_plate': ('hip_plate_L', 'hip_plate_R'), 'spine_tube': ('spine_tube',), 'rail': ('rail',),
           'clamp_ring_stator_D45_t2.0_SUS304': ('clamp_ring_stator',), 'magnet_spacer_D9x5.35_(6061)': ('magnet_spacer',)}
V4_CHANGED = {'shell_front': 'v4 부품에서 스위치 쪽 양옆 세로 모서리만 R30 라운드(v4는 45° 모따기, 사용자 11-02)',
              'hub_flange': 'v4 부품에서 평행핀 구멍 3개(Ø3, 출력 쪽 면에서) 깊이 7.0 → %.1f — 핀 Ø3 × 10을 바닥까지 넣으면 %.1f 나와 '
                            '출력부 핀 구멍(깊이 3)에 여유 %.1f, 스피곳 Ø11.8 높이 1.3 → %.1f(출력부 홈 1.5 바닥에서 %.1f 뜸; 사용자 11-08)'
                            % (P.HUB_DOWEL_HOLE, 10 - P.HUB_DOWEL_HOLE, P.HUB_DOWEL_HOLE - 7.0, P.HUB_SPIGOT[1], 1.5 - P.HUB_SPIGOT[1]),
              'rim': 'v4 부품에서 등록 홈만 Ø11.2 × 3.2 → × %.1f — 허브 쪽 면을 바닥에 두고 출력해 홈 바닥이 브리지로 처져도 허브 플랜지의 '
                     'Ø11 × 3 턱이 바닥에 먼저 닿지 않게(사용자 11-08)' % (P.RIM_REG[1] + P.RIM_REG_GAP[1]),
              'shell_mid': 'v4 부품에 hip 판 나사 4곳 안쪽 받침 Ø8 × %.1f(셸 벽에서 hip 판 웹 면까지, 조이면 벽이 휘던 1.6 틈을 메움) — 나사 '
                           'ISO 7380 M3 × %d(사용자 11-08)' % (P.SHELL_PAD, P.SHELL_SCREW_L),
              'saddle': 'v4 부품에 hip 판 나사 2곳 안쪽 받침 Ø8 × %.1f(셸 hip 조각과 같은 이유, 사용자 11-08)' % P.SHELL_PAD,
              'top_plate': 'v4 부품에서 M4 × 25 머리 카운터보어 깊이만 4.0 → %.1f(머리 높이 4.0이 판 위로 나오지 않게, 사용자 11-08) — '
                           'x ±52 레일 위 열의 M4 인서트에 넣는 짐 나사는 판 윗면에서 6 이하로(그 밑이 레일)' % P.TOP_CBORE}


def other_sheet(stem, pdf, png_dir):
    """Three hidden-line views along the CAD axes (Fusion world: X = CAD y, Y = CAD z, Z = CAD x; the robot's front is -x), overall
    sizes, title block."""
    title, mat, proc, qty, sides = OTHER[stem]
    files = [stem + '_' + c for c in sides] if sides else [stem]
    src = SRC / (files[0] + '.step')
    G.load(src)
    x0, y0, z0, x1, y1, z1 = G.bbox()
    vol = G.volume()
    sx, sy, sz = x1 - x0, y1 - y0, z1 - z0                       # robot y, z, x extents
    k = min(1.0, 330.0 / (sz + sx), 185.0 / (sy + sx))
    for v in (1.0, 0.5, 0.4, 0.25, 0.2, 0.1):
        if k >= v:
            k = v
            break
    S = Sheet()
    # first view: from CAD +y (world +X), screen x = CAD -x (world -Z), up = CAD z (world Y)
    fx, fy = 30.0 + k * z1, 72.0 - k * y0
    VF = View(S, src, (1, 0, 0), (0, 0, -1), (fx, fy), k, label='+y(바깥)에서 본 그림', label_dy=7, mesh=(36, 6.0))
    VT = View(S, src, (0, 1, 0), (0, 0, -1), (fx, VF.box[3] + 18 + k * x1), k, label='위(+z)에서 본 그림', label_dy=7,
              mesh=(36, 6.0))
    VS = View(S, src, (0, 0, -1), (-1, 0, 0), (VF.box[1] + 25 + k * x1, fy), k, label='앞(-x, 스위치 쪽)에서 본 그림', label_dy=7,
              mesh=(36, 6.0))
    S.dim((VF.box[0], VF.box[2]), (VF.box[1], VF.box[2]), VF.box[2] - 12, '%.1f (x)' % sz)
    S.dim((VF.box[0], VF.box[2]), (VF.box[0], VF.box[3]), VF.box[0] - 6, '%.1f (z)' % sy, horizontal=False)
    S.dim((VS.box[0], VS.box[2]), (VS.box[1], VS.box[2]), VS.box[2] - 12, '%.1f (y)' % sx)
    notes = ['단위 mm, 3각법이 아닌 CAD 축 방향 그림(그림 이름 참고, 로봇 앞 = -x).  전체 크기만 적었다 — 형상의 원본은 STEP (export/parts_step/%s.step, '
             'CAD 좌표).' % ', '.join(files),
             '부피 %.1f cm³.  %s(robot_cad_v4 코드로 만듦).' % (vol / 1000, V4_CHANGED.get(stem, 'v4와 같은 부품'))]
    if sides == 'RL':
        notes.append('왼쪽 부품은 거울상 — export/parts_step/%s_L.step으로 따로 만든다.' % stem)
    if stem in V2_COPY:
        notes.append('치수·공차가 다 들어간 가공 도면: drawings/v2_%s.pdf (v2 도면, 같은 형상 확인).' % V2_COPY[stem][0])
    if stem == 'hip_plate':
        notes.append('v2 도면의 이름은 v2 기준이다: v2의 hip_plate_L이 이 로봇의 hip_plate_R(CAD +y), v2의 hip_plate_R이 hip_plate_L이다.')
    S.notes(25.0, 52.0, notes, width=110)
    S.title_block('%s (%s)' % (title, stem), 'WBv5-S%02d' % (list(OTHER).index(stem) + 1),
                  {'재질': mat, '수량': qty, '공정': proc, '척도': '1:%g' % (1 / k) if k < 1 else '1:1',
                   'step': '../export/parts_step/%s.step' % files[0], '공차': 'KS B ISO 2768-mK / 출력 ±0.2'})
    finish(S, pdf, png_dir, stem)


def copy_v2(stem):
    """Copy the v2 machined drawing when the v2 part-frame solid has the same volume as the v5 export."""
    out = []
    for src in V2_COPY[stem]:
        pdf = V2_DRAW / 'drawings' / (src + '.pdf')
        stp = V2_DRAW / 'step' / (src + '.step')
        if not (pdf.exists() and stp.exists()):
            continue
        G.load(stp)
        v2 = G.volume()
        # v2's _L is CAD +y, which this robot calls _R
        name5 = stem + ({'L': '_R', 'R': '_L'}[src[-1]] if src.endswith(('_L', '_R')) else ('_F' if stem in ('spine_tube', 'rail') else ''))
        G.load(SRC / (name5 + '.step'))          # (the magnet spacer's export also holds the magnet: compare per solid)
        vols = [gmsh.model.occ.getMass(3, t) for _d, t in gmsh.model.getEntities(3)]
        v5 = min(vols, key=lambda v: abs(v - v2))
        same = abs(v2 - v5) < 0.002 * v5
        if same:
            shutil.copy(pdf, PDF_DIR / ('v2_%s.pdf' % src))
        out.append((src, round(v2 / 1000, 2), round(v5 / 1000, 2), same))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--only', nargs='*')
    ap.add_argument('--png')
    a = ap.parse_args()
    PDF_DIR.mkdir(exist_ok=True)
    if a.png:
        os.makedirs(a.png, exist_ok=True)
    G.init()
    names = a.only or (list(DETAIL) + list(OTHER))
    for name in names:
        if name in DETAIL:
            res = make_steps(name)
            with PdfPages(PDF_DIR / (name + '.pdf')) as pdf:
                (link_sheets if name in ('thigh', 'shank') else round_sheet)(name, pdf, a.png, res)
            print(name, 'vol %.1f cm3' % (res['vol'] / 1000), 'L/R same' if res['shared'] else 'L/R differ', res['angle'])
        else:
            with PdfPages(PDF_DIR / (name + '.pdf')) as pdf:
                other_sheet(name, pdf, a.png)
            print(name, 'ok', copy_v2(name) if name in V2_COPY else '')


if __name__ == '__main__':
    main()
