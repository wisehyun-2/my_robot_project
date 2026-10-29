"""2D drawings (PDF) for the quote parts: views are hidden-line projections of step/*.step, the dimensions
and notes are written here.  Run make_parts.py first.

    python drawings.py              # -> drawings/*.pdf
    python drawings.py --png DIR    # also PNG previews
"""
import argparse
import math
import pathlib
import unicodedata

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402
from matplotlib.patches import PathPatch, Polygon  # noqa: E402
from matplotlib.path import Path  # noqa: E402

import make_parts as M  # noqa: E402
import occ as O  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
OUT = HERE / 'drawings'
DATE = '2026-10-29'
PROJECT = 'WheelBiped v2'

FONT = '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc'     # any font with Hangul (no U+2212: write '-')
try:
    font_manager.fontManager.addfont(FONT)
    plt.rcParams['font.family'] = font_manager.FontProperties(fname=FONT).get_name()
except (FileNotFoundError, RuntimeError):
    pass
plt.rcParams.update({'hatch.linewidth': 0.3, 'hatch.color': 'black', 'pdf.fonttype': 42})

LW_VIS, LW_HID, LW_THIN = 0.9, 0.4, 0.35
FS, FS_S = 8, 6.5
SHEETS = {'A3': (420.0, 297.0), 'A4': (297.0, 210.0)}
CENTRE = (0, (8, 1.5, 1.5, 1.5))

GENERAL_NOTES = [
    '단위 mm, 3각법.',
    '지시 없는 치수 공차: KS B ISO 2768-m.',
    '모서리 버 제거, 지시 없는 바깥 모서리 C0.2~0.5.',
    '지시 없는 형상·치수는 같은 이름의 STEP 파일 기준.',
]


def wrap(s, width):
    """Wrap counting East Asian wide characters as 2 columns."""
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


# ----------------------------------------------------------------------------- sheet and views
class Sheet:
    def __init__(self, size='A3'):
        self.w, self.h = SHEETS[size]
        self.fig = plt.figure(figsize=(self.w / 25.4, self.h / 25.4))
        self.ax = self.fig.add_axes([0, 0, 1, 1])
        self.ax.set_xlim(0, self.w)
        self.ax.set_ylim(0, self.h)
        self.ax.set_aspect('equal')
        self.ax.axis('off')
        self.ax.add_patch(Polygon([(10, 10), (self.w - 10, 10), (self.w - 10, self.h - 10), (10, self.h - 10)],
                                  closed=True, fill=False, lw=1.2))

    def line(self, pts, lw=LW_THIN, ls='-', color='k'):
        xs, ys = zip(*pts)
        self.ax.plot(xs, ys, lw=lw, ls=ls, color=color, solid_capstyle='round')

    def text(self, p, s, fs=FS, ha='center', va='center', rot=0, weight='normal'):
        self.ax.text(p[0], p[1], s, fontsize=fs, ha=ha, va=va, rotation=rot, weight=weight)

    def arrow(self, tip, frm):
        dx, dy = tip[0] - frm[0], tip[1] - frm[1]
        n = math.hypot(dx, dy) or 1.0
        ux, uy = dx / n, dy / n
        L, W = 2.6, 0.75
        b = (tip[0] - ux * L, tip[1] - uy * L)
        self.ax.add_patch(Polygon([tip, (b[0] - uy * W, b[1] + ux * W), (b[0] + uy * W, b[1] - ux * W)],
                                  closed=True, color='k', lw=0))

    def dim(self, p1, p2, at, text, horizontal=True, fs=FS):
        """Linear dimension between p1 and p2; horizontal measures x with the line at height `at`,
        vertical measures y with the line at x = `at`."""
        if horizontal:
            a, b = (p1[0], at), (p2[0], at)
            for p in (p1, p2):
                s = 1 if at > p[1] else -1
                self.line([(p[0], p[1] + s * 1.0), (p[0], at + s * 1.5)])
        else:
            a, b = (at, p1[1]), (at, p2[1])
            for p in (p1, p2):
                s = 1 if at > p[0] else -1
                self.line([(p[0] + s * 1.0, p[1]), (at + s * 1.5, p[1])])
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        if L > 7:
            self.line([a, b])
            self.arrow(a, b)
            self.arrow(b, a)
        else:                                                   # arrows outside
            ux, uy = (b[0] - a[0]) / (L or 1), (b[1] - a[1]) / (L or 1)
            self.line([(a[0] - ux * 5, a[1] - uy * 5), (b[0] + ux * 5, b[1] + uy * 5)])
            self.arrow(a, (a[0] - ux * 3, a[1] - uy * 3))
            self.arrow(b, (b[0] + ux * 3, b[1] + uy * 3))
        m = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
        if horizontal:
            self.text((m[0], m[1] + 1.0), text, fs=fs, va='bottom')
        else:
            self.text((m[0] - 1.0, m[1]), text, fs=fs, ha='right', rot=90)

    def leader(self, tip, knee, text, fs=FS, right=True):
        """Arrow at tip, line to knee, short shelf, then the text."""
        end = (knee[0] + (5 if right else -5), knee[1])
        self.line([tip, knee, end])
        self.arrow(tip, knee)
        self.text((end[0] + (1 if right else -1), end[1]), text, fs=fs, ha='left' if right else 'right')

    def table(self, x, y, cols, rows, fs=FS_S, row_h=5.0, title=None):
        """cols: [(header, width)], rows: lists of strings; (x, y) = top left.  Returns the bottom y."""
        if title:
            self.text((x, y + 3.5), title, fs=FS, ha='left')
        wsum = sum(w for _, w in cols)
        n = len(rows) + 1
        for i in range(n + 1):
            self.line([(x, y - i * row_h), (x + wsum, y - i * row_h)], lw=0.5 if i in (0, 1, n) else 0.3)
        cx = x
        for k, (hdr, w) in enumerate(cols):
            self.line([(cx, y), (cx, y - n * row_h)], lw=0.5 if k == 0 else 0.3)
            self.text((cx + w / 2, y - row_h / 2), hdr, fs=fs)
            for i, r in enumerate(rows):
                self.text((cx + 1.2, y - (i + 1.5) * row_h), r[k], fs=fs, ha='left')
            cx += w
        self.line([(cx, y), (cx, y - n * row_h)], lw=0.5)
        return y - n * row_h

    def notes(self, x, y, lines, width, title='주기', fs=FS_S, step=3.9):
        """Numbered notes wrapped at `width` columns (Hangul = 2).  Returns the bottom y."""
        self.text((x, y), title, fs=FS, ha='left')
        yy = y
        for i, s in enumerate(lines):
            for j, part in enumerate(wrap(s, width)):
                yy -= step
                self.text((x + (0 if j == 0 else 4.0), yy), ('%d. ' % (i + 1) if j == 0 else '') + part, fs=fs, ha='left')
        return yy

    def title_block(self, name, number, info):
        w, rh = 190.0, 6.0
        x0, y0 = self.w - 10 - w, 10.0
        rows = [('품명', name, '도번', number),
                ('재질', info['재질'], '수량', info['수량']),
                ('공정', info['공정'], '척도', info['척도']),
                ('후처리', info['후처리'], '투상', '3각법'),
                ('STEP', 'step/%s.step' % info['step'], '일반 공차', 'KS B ISO 2768-m'),
                ('프로젝트', PROJECT + ' (robot_cad_v2)', '작성일', DATE)]
        cw = (18.0, 102.0, 18.0, 52.0)
        h = rh * len(rows)
        self.ax.add_patch(Polygon([(x0, y0), (x0 + w, y0), (x0 + w, y0 + h), (x0, y0 + h)], closed=True, fill=False, lw=1.0))
        for i in range(1, len(rows)):
            self.line([(x0, y0 + i * rh), (x0 + w, y0 + i * rh)], lw=0.3)
        cx = x0
        for c in cw[:-1]:
            cx += c
            self.line([(cx, y0), (cx, y0 + h)], lw=0.3)
        for i, r in enumerate(rows):
            yy = y0 + h - (i + 0.5) * rh
            cx = x0
            for k, (c, s) in enumerate(zip(cw, r)):
                self.text((cx + 1.2, yy), s, fs=FS_S if k % 2 == 0 else FS - 0.5, ha='left')
                cx += c


class View:
    """Orthographic view: paper = origin + scale * (p . xdir, p . up), up = toward x xdir."""

    def __init__(self, sheet, shape, toward, xdir, origin, scale, hidden=True, label=None, label_dy=7.0):
        self.sh, self.o, self.k = sheet, origin, scale
        self.xd = xdir
        t = toward
        self.up = (t[1] * xdir[2] - t[2] * xdir[1], t[2] * xdir[0] - t[0] * xdir[2], t[0] * xdir[1] - t[1] * xdir[0])
        vis, hid = O.project(shape, toward, xdir)
        if hidden:
            for pl in hid:
                sheet.line([self.p2(q) for q in pl], lw=LW_HID, ls=(0, (3, 1.5)), color='0.35')
        pts = []
        for pl in vis:
            pp = [self.p2(q) for q in pl]
            pts += pp
            sheet.line(pp, lw=LW_VIS)
        xs, ys = zip(*pts)
        self.box = (min(xs), max(xs), min(ys), max(ys))
        if label:
            sheet.text(((self.box[0] + self.box[1]) / 2, self.box[2] - label_dy), label, fs=FS + 1)

    def p2(self, q):
        return (self.o[0] + self.k * q[0], self.o[1] + self.k * q[1])

    def pt(self, p):
        return self.p2((sum(a * b for a, b in zip(p, self.xd)), sum(a * b for a, b in zip(p, self.up))))

    def hatch(self, shape, axis, value):
        """Hatch the faces of `shape` on the plane axis = value (the cut faces of a section)."""
        for loops in O.section_faces(shape, axis, value):
            verts, codes = [], []
            for i, loop in enumerate(loops):
                pts = [self.pt(p) for p in loop]
                a = sum(pts[j][0] * pts[j - 1][1] - pts[j - 1][0] * pts[j][1] for j in range(len(pts)))
                if (a < 0) != (i == 0):                 # outer loop one way, holes the other
                    pts.reverse()
                verts += pts + [pts[0]]
                codes += [Path.MOVETO] + [Path.LINETO] * (len(pts) - 1) + [Path.CLOSEPOLY]
            self.sh.ax.add_patch(PathPatch(Path(verts, codes), facecolor='none', edgecolor='k', lw=0, hatch='////'))

    def axis_line(self, p0, p1, ext=3.0):
        a, b = self.pt(p0), self.pt(p1)
        L = math.hypot(b[0] - a[0], b[1] - a[1]) or 1
        u = ((b[0] - a[0]) / L, (b[1] - a[1]) / L)
        self.sh.line([(a[0] - u[0] * ext, a[1] - u[1] * ext), (b[0] + u[0] * ext, b[1] + u[1] * ext)], ls=CENTRE)


def half(shape, axis, keep_negative, value=0.0):
    """Section half: the part of `shape` on one side of the plane axis = value."""
    big = 1000.0
    lo, hi = [-big] * 3, [big] * 3
    i = 'xyz'.index(axis)
    if keep_negative:
        hi[i] = value
    else:
        lo[i] = value
    return O.common(shape, O.box(lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))


def load(name):
    return O.solids(O.read_step(M.STEP_DIR / (name + '.step')))[0]


def info(name, scale):
    p = M.PARTS[name]
    return {'재질': p['material'], '수량': '%d개' % p['qty'], '후처리': p['finish'], '공정': p['process'],
            '척도': scale, 'step': name}


def finish(S, pdf, png_dir, tag):
    pdf.savefig(S.fig)
    if png_dir:
        S.fig.savefig(pathlib.Path(png_dir) / (tag + '.png'), dpi=110)
    plt.close(S.fig)


# ----------------------------------------------------------------------------- hip plate
def hip_holes(shape):
    """Off-axis holes of the quote STEP: sorted list of (kind, x, y)."""
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Cylinder
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS
    seen = {}
    fe = TopExp_Explorer(shape, TopAbs_FACE)
    while fe.More():
        f = TopoDS.Face(fe.Current())
        a = BRepAdaptor_Surface(f)
        if a.GetType() == GeomAbs_Cylinder:
            o = a.Cylinder().Location()
            d = round(2 * a.Cylinder().Radius(), 2)
            if o.X() ** 2 + o.Y() ** 2 > 1.0:
                b = O.bbox(f)
                key = (d, round(o.X(), 2), round(o.Y(), 2))
                z0, z1 = seen.get(key, (99, -99))
                seen[key] = (min(z0, b[4]), max(z1, b[5]))
        fe.Next()
    holes = []
    for (d, x, y), (z0, z1) in seen.items():
        kind = {2.8: 'A', 2.5: 'B', 5.4: 'E', 4.4: 'F', 22.0: 'G'}.get(d)
        if kind == 'B' and z1 < 9.0:                   # B: driver cup holes run through the Ø80 boss edge
            kind = 'C' if (y < -30 and abs(abs(x) - 45) < 0.5) else 'D'
        if kind:
            holes.append((kind, x, y))
    holes.sort(key=lambda h: (h[0], -h[2], h[1]))
    return holes


HOLE_SPEC = {
    'A': ('Ø2.8 관통 + Ø4.8 깊이 2.7 카운터보어(Z=0 면) ★', 'AKE 고정링 M2.5×6 ISO 4762'),
    'B': ('M3 탭 관통 (기초 Ø2.5)', '드라이버 컵 M3×40'),
    'C': ('M3 탭 관통 (기초 Ø2.5)', '전장 데크 M3×8'),
    'D': ('M3 탭 관통 (기초 Ø2.5)', '셸 M3×6'),
    'E': ('Ø5.4 관통 + 90° 접시 Ø11.2 (Z=3 면)', '척추 관 마개 M5×12 ISO 10642'),
    'F': ('Ø4.4 관통 + 90° 접시 Ø9.0 (Z=3 면)', '레일 마개 M4×12 ISO 10642'),
    'G': ('Ø22 관통 (경량 구멍)', ''),
}


def draw_hip_plate(name, pdf, png_dir):
    s = 1 if name.endswith('L') else -1
    tag = 'L' if s > 0 else 'R'
    sh = load(name)
    side, other = ('왼쪽', '오른쪽') if s > 0 else ('오른쪽', '왼쪽')
    front = '로봇 앞(+x) →' if s > 0 else '← 로봇 앞(+x)'
    groove_deg = 235.0 if s > 0 else 305.0
    title = 'hip 판 %s (hip_plate_%s)' % (side, tag)

    # ---- sheet 1: plan view from the robot centre side (+Z) and the hole table
    S = Sheet('A3')
    V = View(S, sh, (0, 0, 1), (1, 0, 0), (118.0, 110.0), 1.0,
             label='안쪽 면(로봇 중심 쪽, +Z)에서 본 그림  1:1      %s' % front, label_dy=24)
    V.axis_line((-68, 0, 0), (68, 0, 0))
    V.axis_line((0, -68, 0), (0, 145, 0))
    for yy, va in ((-79.0, 'top'), (164.0, 'bottom')):            # section B-B cut line
        p = V.pt((0, yy, 0))
        S.line([(p[0] - 4, p[1]), (p[0] + 4, p[1])], lw=1.2)
        S.arrow((p[0] - 4, p[1] + (3 if va == 'bottom' else -3)), (p[0] - 4 + 6, p[1] + (3 if va == 'bottom' else -3)))
        S.text((p[0] + 6, p[1]), 'B', fs=FS + 1)
    holes = hip_holes(sh)
    count = {}
    rows = []
    for (k, x, y) in holes:
        count[k] = count.get(k, 0) + 1
        lab = '%s%d' % (k, count[k])
        p = V.pt((x, y, 9.5))
        off = 7.5 if k == 'G' else 2.0
        S.text((p[0] + off, p[1] + off), lab, fs=5.5)
        rows.append([lab, '%.2f' % (x if abs(x) >= 0.005 else 0.0), '%.2f' % (y if abs(y) >= 0.005 else 0.0),
                     HOLE_SPEC[k][0], HOLE_SPEC[k][1]])
    # outline dimensions
    bx = V.box
    S.dim(V.pt((-64, 145, 0)), V.pt((64, 145, 0)), bx[3] + 13, '128')
    S.dim(V.pt((-22, 128, 0)), V.pt((22, 128, 0)), bx[3] + 5, '44')
    S.dim(V.pt((-68, 0, 0)), V.pt((68, 0, 0)), bx[2] - 6, 'Ø136')
    S.dim(V.pt((68, -68, 0)), V.pt((64, 145, 0)), bx[1] + 16, '213', horizontal=False)
    S.dim(V.pt((64, 0, 0)), V.pt((64, 125, 0)), bx[1] + 8, '125', horizontal=False)
    S.dim(V.pt((-64, 0, 0)), V.pt((-64, 60, 0)), bx[0] - 6, '60', horizontal=False)
    S.dim(V.pt((-64, 60, 0)), V.pt((-64, 118, 0)), bx[0] - 6, '58', horizontal=False)
    S.dim(V.pt((-64, 129, 0)), V.pt((-64, 135.5, 0)), bx[0] - 6, '6.5', horizontal=False)
    S.dim(V.pt((-64, 125, 0)), V.pt((-64, 129, 0)), bx[0] - 14, '4', horizontal=False)
    S.dim(V.pt((-15, 95, 0)), V.pt((15, 95, 0)), V.pt((0, 95, 0))[1], '30')
    g = math.radians(groove_deg)
    tags = [('P1', (35.0 * math.cos(g), 35.0 * math.sin(g)), (-52.0 * s, -28.0)),
            ('P2', (-15.0, 112.0), (-38.0, 108.0)),
            ('P3', (-36.5, 133.0), (-50.0, 118.0)),
            ('P4', (34.5, 112.5), (52.0, 106.0))]
    for lab, tip, knee in tags:
        right = knee[0] > tip[0]
        S.leader(V.pt((tip[0], tip[1], 9.5)), V.pt((knee[0], knee[1], 9.5)), lab, fs=6.5, right=right)
    yb = S.table(228.0, 274.0, [('구멍', 10), ('X', 14), ('Y', 14), ('가공', 80), ('용도', 52)], rows, row_h=4.4,
                 title='구멍 표 (원점 = hip 축 = Ø42 포켓 중심, X·Y는 위 그림 기준)')
    yb = S.table(228.0, yb - 9, [('형상', 10), ('내용', 118), ('비고', 42)], [
        ['P1', '리드선 홈: 폭 7, 반경 21.5~41 (중심 31.25), 바닥 Z=6.0, 방향 X축에서 %.0f°' % groove_deg, 'AKE 상 리드선'],
        ['P2', '창 30 × 58 (X ±15, Y 60~118), 아래는 G1과 이어짐', '경량'],
        ['P3', '창 13 × 6.5 (X ±30~43, Y 129~135.5) 2곳', '경량 (D1·D2 탭 살 확보)'],
        ['P4', '보강 삼각 25 × 25 (45°, X ±22~47, Y 100~125) 2곳', ''],
        ['중심', 'Ø42 H7 포켓 깊이 4.0, Ø30 관통, Ø80 보스, Ø52 허브: 2쪽 상세 C', ''],
    ], row_h=4.4, title='형상 표')
    S.notes(228.0, yb - 9, [
        '★ A 구멍 위치 공차 ±0.05 (PCD 36.5, 60° 등간격), 다른 구멍 ±0.1.',
        '탭 M3 6H 관통 11개(B·C·D), 아노다이징 뒤 탭 게이지 통과.',
        '내부 세로 모서리(창, 보강 삼각, 리드선 홈)는 공구 반경 R3 이하 허용.',
        '%s hip 판: %s 판과 거울상이며 같은 부품이 아님. 두께 방향은 2쪽.' % (side, other),
    ], width=118, title='이 쪽 주기')
    S.title_block(title, 'WBv2-M01-%s (1/2)' % tag, info(name, '1:1'))
    finish(S, pdf, png_dir, name + '_1')

    # ---- sheet 2: section B-B (X = 0) and detail C (hub, 3.5:1)
    S = Sheet('A3')
    hb = half(sh, 'x', True)
    V = View(S, hb, (1, 0, 0), (0, 0, -1), (48.0, 118.0), 1.0, hidden=False, label='단면 B-B  1:1')
    V.hatch(hb, 'x', 0.0)
    p = V.pt((0, 20, 0))
    S.leader((p[0] + 2, p[1]), (p[0] + 14, p[1] + 14), '상세 C')
    det = O.common(hb, O.box(-1000, 1000, -2.0, 48.0, -10, 20))
    k = 3.5
    V = View(S, det, (1, 0, 0), (0, 0, -1), (150.0, 78.0), k, hidden=False, label='상세 C (hub 부분, Y ≥ 0)  3.5:1', label_dy=20)
    V.hatch(det, 'x', 0.0)
    P = lambda y, z: V.pt((0, y, z))  # noqa: E731
    top = V.box[3]
    S.dim(P(24, 0), P(30, 3.0), top + 7, '3.0')
    S.dim(P(30, 3.0), P(30, 8.0), top + 7, '5.0')
    S.dim(P(30, 8.0), P(30, 9.5), top + 7, '1.5')
    S.dim(P(30, 0), P(30, 9.5), top + 17, '9.5')
    S.dim(P(18, 5.5), P(18, 9.5), V.box[2] - 8, '4.0 ±0.05')
    S.leader(P(21, 7.5), (P(21, 7.5)[0] - 30, P(21, 7.5)[1] - 12), 'Ø42 H7 (+0.025/0)', right=False)
    S.leader(P(15, 2.5), (P(15, 2.5)[0] + 30, P(15, 2.5)[1] - 18), 'Ø30 관통')
    S.leader(P(26, 1.5), (P(26, 1.5)[0] + 30, P(26, 1.5)[1] + 4), 'Ø52 허브 보스 (Z 0~3)')
    S.leader(P(36, 9.0), (P(36, 9.0)[0] - 30, P(36, 9.0)[1] + 14), 'Ø80 등록 보스 (Z 8~9.5)', right=False)
    S.leader(P(44, 5.5), (P(44, 5.5)[0] + 30, P(44, 5.5)[1] + 14), 'Ø22 경량 구멍 G1')
    S.text(((V.box[0] + V.box[1]) / 2, V.box[2] - 28), '← Z=9.5 (로봇 중심 쪽)        Z=0 (바깥 쪽) →', fs=FS_S)
    yb = S.table(250.0, 274.0, [('Z', 12), ('면', 146)], [
        ['0', '허브 바깥 면 Ø52 (허벅지 쪽, 셸 밖으로 3 mm)'],
        ['2.7', 'A 카운터보어 바닥'],
        ['3.0', '웹 바깥 면 (E·F 접시 자리가 있는 면)'],
        ['5.5', 'Ø42 포켓 바닥 = AKE 고정링 면: 평면도 0.02, Z=8 면과 평행 0.03'],
        ['6.0', '리드선 홈(P1) 바닥'],
        ['8.0', '웹 안쪽 면 = 기준 면 (척추 관·레일 끝이 닿음)'],
        ['9.5', 'Ø80 등록 보스 면 (드라이버 컵)'],
    ], row_h=5.0, title='두께 방향 면 높이 (Z)')
    S.notes(250.0, yb - 9, GENERAL_NOTES + [
        '소재 AL6061-T6 판재 10t 이상(가능하면 12t). 완성 두께 9.5 mm (8 mm 허브 + 1.5 mm 등록 보스).',
        '양면 2공정. Ø42 포켓, Z=5.5 바닥, Z=8 면은 같은 공정에서 가공 권장(모터 축 정렬).',
        '검정 아노다이징(황산 10~15 µm). Ø42 H7 포켓은 마스킹하거나 피막 뒤 H7 만족.',
        '나사: A는 M2.5×6 (AKE 링 물림 3.2 mm). E·F 접시 자리 지름은 KS B ISO 10642 이론 머리 지름이라 머리가 Z=3 면보다 0.5~0.9 아래.',
    ], width=96)
    S.title_block(title, 'WBv2-M01-%s (2/2)' % tag, info(name, '1:1, 3.5:1'))
    finish(S, pdf, png_dir, name + '_2')


# ----------------------------------------------------------------------------- small parts (A4)
def concentricity(S, x0, y0, tip, text):
    for i, (w, s) in enumerate(((7, '◎'), (12, 'Ø0.03'), (6, 'A'))):
        xx = x0 + sum((7, 12, 6)[:i])
        S.ax.add_patch(Polygon([(xx, y0), (xx + w, y0), (xx + w, y0 + 6), (xx, y0 + 6)], closed=True, fill=False, lw=0.5))
        S.text((xx + w / 2, y0 + 3), s, fs=FS)
    S.line([(x0 + 3.5, y0), tip])
    S.arrow(tip, (x0 + 3.5, y0))
    S.text((x0 + 28, y0 + 3), text, fs=FS_S, ha='left')


def draw_magnet_spacer(pdf, png_dir):
    name = 'magnet_spacer'
    sh = load(name)
    S = Sheet('A4')
    hb = half(sh, 'y', False)
    V = View(S, hb, (0, -1, 0), (1, 0, 0), (70.0, 105.0), 6.0, hidden=False, label='단면 (축 포함)  6:1', label_dy=18)
    V.hatch(hb, 'y', 0.0)
    V.axis_line((0, 0, 0), (0, 0, 6.8))
    P = lambda x, z: V.pt((x, 0, z))  # noqa: E731
    S.dim(P(-4.5, 6.8), P(4.5, 6.8), V.box[3] + 15, 'Ø9 ±0.05')
    S.dim(P(-3.02, 6.8), P(3.02, 6.8), V.box[3] + 6, 'Ø6.04 +0.03/0')
    S.dim(P(-2.95, 0), P(2.95, 0), V.box[2] - 8, 'Ø5.90 ±0.02  (기준 A)')
    S.dim(P(4.5, 2.3), P(4.5, 6.8), V.box[1] + 9, '4.5 ±0.03', horizontal=False)
    S.dim(P(2.95, 0), P(2.95, 2.3), V.box[1] + 9, '2.3 ±0.1', horizontal=False)
    S.dim(P(-4.5, 4.3), P(-4.5, 6.8), V.box[0] - 8, '2.5 +0.05/0', horizontal=False)
    concentricity(S, V.box[0] - 6, V.box[3] + 24, P(-3.02, 6.2), '포켓 Ø6.04, 스피곳 A 기준')
    S.notes(150.0, 185.0, GENERAL_NOTES + [
        '재질 비자성 필수(AL6061-T6). 강·스테인리스 금지.',
        'Ø6.04 포켓: AKE 자석 Ø6 × 2.5를 에폭시로 붙임(자석은 AKE에서 떼어 재사용).',
        'Ø5.90 스피곳(A): AKE 로터의 빈 자석 포켓에 접착. 로터 포켓 실측 뒤 그 값 -0.10으로 조정 가능.',
        '높이 4.5가 엔코더 칩–자석 간격(2.0 ±0.5)을 정함.',
        '자석 쪽 끝면은 축에 직각, 버·턱 없이.',
    ], width=78)
    S.title_block('자석 스페이서 (magnet spacer Ø9 × 4.5)', 'WBv2-M02', info(name, '6:1'))
    finish(S, pdf, png_dir, name)


def draw_end_plug_round(pdf, png_dir):
    name = 'end_plug_round'
    sh = load(name)
    S = Sheet('A4')
    hb = half(sh, 'y', False)
    V = View(S, hb, (0, -1, 0), (1, 0, 0), (75.0, 100.0), 5.0, hidden=False, label='단면 (축 포함)  5:1', label_dy=10)
    V.hatch(hb, 'y', 0.0)
    V.axis_line((0, 0, 0), (0, 0, 10))
    P = lambda x, z: V.pt((x, 0, z))  # noqa: E731
    S.dim(P(-6.95, 10), P(6.95, 10), V.box[3] + 8, 'Ø13.90 ±0.02 (관 실측 내경 -0.10)')
    S.dim(P(6.95, 0), P(6.95, 10), V.box[1] + 9, '10 ±0.1', horizontal=False)
    S.leader(P(2.1, 6.0), (V.box[1] + 22, V.box[2] + 8), 'M5 탭 관통 (기초 Ø4.2)')
    S.notes(150.0, 185.0, GENERAL_NOTES + [
        '척추 관(Ø16 × 1.0, 안지름 Ø14) 양 끝에 끼우고 록타이트 638 등 고정제로 접착.',
        '바깥지름은 실제 관에 맞춤: 관 실측 안지름 -0.10 (지름 틈 0.05~0.15).',
        '양 끝 바깥 모서리 C0.3, 탭 입구 C0.3.',
        'hip 판 쪽에서 ISO 10642 M5×12가 약 7.5 mm 물림.',
    ], width=78)
    S.title_block('척추 관 마개 (end plug round, M5)', 'WBv2-M03', info(name, '5:1'))
    finish(S, pdf, png_dir, name)


def draw_end_plug_square(pdf, png_dir):
    name = 'end_plug_square'
    sh = load(name)
    S = Sheet('A4')
    k, a = 4.0, 6.45
    V = View(S, sh, (0, 0, 1), (1, 0, 0), (62.0, 157.0), k, hidden=True, label='위에서 본 그림  4:1', label_dy=6)
    P = lambda x, y: V.pt((x, y, 10))  # noqa: E731
    S.dim(P(-a, a), P(a, a), V.box[3] + 7, '12.90 ±0.02 (각관 실측 안치수 -0.10)')
    S.dim(P(a, -a), P(a, a), V.box[1] + 9, '12.90 ±0.02', horizontal=False)
    S.leader(P(a - 0.5, -a + 0.5), (V.box[1] + 22, V.box[2] - 2), 'C1.0 (긴 모서리 4곳)')
    S.leader(P(1.17, 1.17), (V.box[1] + 22, V.box[3] - 6), 'M4 탭 관통 (기초 Ø3.3)')
    V2 = View(S, sh, (0, -1, 0), (1, 0, 0), (62.0, 72.0), k, hidden=True, label='앞에서 본 그림  4:1', label_dy=7)
    S.dim(V2.pt((a, 0, 0)), V2.pt((a, 0, 10)), V2.box[1] + 9, '10 ±0.1', horizontal=False)
    S.notes(150.0, 185.0, GENERAL_NOTES + [
        '레일(각관 15 × 15 × 1.0, 안치수 13 × 13) 양 끝에 끼우고 록타이트 638 등으로 접착.',
        '바깥 치수는 실제 각관에 맞춤: 실측 안치수 -0.10.',
        'C1.0: 각관 안쪽 모서리 R(소재 R0.5~1.0)에 걸리지 않게 함.',
        '양 끝면 모서리 C0.3, 탭 입구 C0.3.',
        'hip 판 쪽에서 ISO 10642 M4×12가 약 7.5 mm 물림.',
    ], width=78)
    S.title_block('레일 마개 (end plug square, M4)', 'WBv2-M04', info(name, '4:1'))
    finish(S, pdf, png_dir, name)


def draw_spine_tube(pdf, png_dir):
    name = 'spine_tube'
    sh = load(name)
    S = Sheet('A4')
    V = View(S, sh, (0, -1, 0), (1, 0, 0), (30.0, 165.0), 0.6, hidden=True, label='앞에서 본 그림  0.6:1')
    V.axis_line((0, 0, 0), (328, 0, 0))
    S.dim(V.pt((0, 0, 8)), V.pt((328, 0, 8)), V.box[3] + 8, '328.00 ±0.05')
    V2 = View(S, sh, (-1, 0, 0), (0, -1, 0), (55.0, 95.0), 3.0, hidden=False, label='끝면  3:1')
    V2.axis_line((0, 8, 0), (0, -8, 0))
    V2.axis_line((0, 0, 8), (0, 0, -8))
    S.dim(V2.pt((0, 8, 0)), V2.pt((0, -8, 0)), V2.box[3] + 6, 'Ø16 (소재)')
    S.leader(V2.pt((0, -4.95, -4.95)), (V2.box[1] + 12, V2.box[2] + 4), 'Ø14, 두께 1.0 (소재)')
    S.notes(120.0, 126.0, GENERAL_NOTES + [
        '시판 6061-T6 원관 Ø16 × 1.0(인발관)을 잘라 양 끝 면삭.',
        '길이 328.00 ±0.05: 척추 관 2개와 레일 2개(WBv2-M06), 4개를 같은 값으로 같은 세팅에서 맞춤.',
        '양 끝면 축에 직각 0.05, 안·밖 버 제거.',
        '4개 길이가 다르면 hip 판이 비틀려 좌우 hip 모터 축이 어긋남.',
        '양 끝에 마개 WBv2-M03을 접착(조립 때).',
        '6061-T6을 구할 수 없으면 6063-T5/T6 사용 가능(강도 재계산 통과, calc/strength.md 4절).',
    ], width=100)
    S.title_block('척추 관 (spine tube Ø16 × 1.0)', 'WBv2-M05', info(name, '0.6:1, 3:1'))
    finish(S, pdf, png_dir, name)


def draw_rail(pdf, png_dir):
    name = 'rail'
    sh = load(name)
    S = Sheet('A4')
    V = View(S, sh, (0, 0, 1), (1, 0, 0), (30.0, 160.0), 0.6, hidden=True, label='위에서 본 그림  0.6:1', label_dy=14)
    V.axis_line((0, 0, 7.5), (328, 0, 7.5))
    top = V.box[3]
    for i, x in enumerate(M.RAIL_HOLES):
        S.dim(V.pt((0, 7.5, 7.5)), V.pt((x, 0, 7.5)), top + 5 + 5 * i, '%.1f ±0.1' % x)
    S.dim(V.pt((0, 7.5, 7.5)), V.pt((328, 7.5, 7.5)), top + 27, '328.00 ±0.05')
    S.leader(V.pt((M.RAIL_HOLES[0] + 1.6, -1.6, 7.5)), (V.pt((M.RAIL_HOLES[0], 0, 0))[0] + 15, V.box[2] - 7),
             '4 × Ø4.4 관통 (위·아래 벽, 폭 중앙)')
    V2 = View(S, sh, (-1, 0, 0), (0, -1, 0), (55.0, 92.0), 3.0, hidden=False, label='끝면  3:1')
    S.dim(V2.pt((0, 7.5, 7.5)), V2.pt((0, -7.5, 7.5)), V2.box[3] + 6, '15 (소재)')
    S.dim(V2.pt((0, -7.5, -7.5)), V2.pt((0, -7.5, 7.5)), V2.box[1] + 8, '15', horizontal=False)
    S.leader(V2.pt((0, -7.0, 2.0)), (V2.box[1] + 18, V2.box[2] + 2), '두께 1.0 (소재)')
    S.notes(120.0, 126.0, GENERAL_NOTES + [
        '시판 6061-T6 각관 15 × 15 × 1.0을 잘라 양 끝 면삭.',
        '길이 328.00 ±0.05: 척추 관(WBv2-M05) 2개와 함께 4개를 같은 값으로 같은 세팅에서 맞춤.',
        'Ø4.4 관통 4곳: 위·아래 벽을 한 번에(폭 중앙, 위치 ±0.1). 윗판 M4×25 관통 볼트 자리.',
        '양 끝면 직각 0.05, 버 제거. 양 끝에 마개 WBv2-M04 접착(조립 때).',
        '6061-T6을 구할 수 없으면 6063-T5/T6 사용 가능(강도 재계산 통과, calc/strength.md 4절). 관통 볼트는 각관이 눌리지 않을 만큼만 조임.',
    ], width=100)
    S.title_block('위 레일 (rail 15 × 15 × 1.0)', 'WBv2-M06', info(name, '0.6:1, 3:1'))
    finish(S, pdf, png_dir, name)


def draw_ring(name, number, title, extra, pdf, png_dir):
    sh = load(name)
    od, idia, pcd, hole, a0 = M.RING_SPECS[name]
    S = Sheet('A4')
    k = 3.0 if od < 30 else 2.0
    V = View(S, sh, (0, 0, 1), (1, 0, 0), (75.0, 118.0), k, hidden=False, label='위에서 본 그림  %g:1' % k, label_dy=17)
    V.axis_line((-od / 2, 0, 1.5), (od / 2, 0, 1.5))
    V.axis_line((0, -od / 2, 1.5), (0, od / 2, 1.5))
    P = lambda x, y: V.pt((x, y, 1.5))  # noqa: E731
    S.dim(P(-od / 2, 0), P(od / 2, 0), V.box[3] + 8, 'Ø%g' % od)
    S.dim(P(-idia / 2, 0), P(idia / 2, 0), V.box[2] - 8, 'Ø%g' % idia)
    t = math.radians(a0)
    hp = (pcd / 2 * math.cos(t) + hole / 2 * 0.71, pcd / 2 * math.sin(t) + hole / 2 * 0.71)
    S.leader(P(*hp), (V.box[1] + 8, V.box[3] - 4), '6 × Ø%g 관통, PCD %g, 60° 등간격' % (hole, pcd))
    S.notes(150.0, 140.0, ['단위 mm. 두께 %g (소재 SUS304 2B %gt, 평판).' % (M.P.RING_T, M.P.RING_T),
                           '지시 없는 공차 ±0.1 (레이저), 평탄도 0.1.',
                           '양면 버·드로스 제거, 질소 절단 권장(산화 변색 최소).',
                           '같은 이름의 DXF(dxf/) 또는 STEP(step/) 사용.'] + extra, width=78)
    S.title_block(title, number, info(name, '%g:1' % k))
    finish(S, pdf, png_dir, name)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--png', default=None, help='directory for PNG previews')
    a = ap.parse_args()
    OUT.mkdir(exist_ok=True)
    jobs = [
        ('hip_plate_L', lambda pdf, d: draw_hip_plate('hip_plate_L', pdf, d)),
        ('hip_plate_R', lambda pdf, d: draw_hip_plate('hip_plate_R', pdf, d)),
        ('magnet_spacer', draw_magnet_spacer),
        ('end_plug_round', draw_end_plug_round),
        ('end_plug_square', draw_end_plug_square),
        ('spine_tube', draw_spine_tube),
        ('rail', draw_rail),
        ('clamp_ring_output', lambda pdf, d: draw_ring(
            'clamp_ring_output', 'WBv2-M07', '출력 클램프 링 (clamp ring output Ø27)',
            ['hip·knee 구동 보스 바닥, 출력 M3×16 머리 밑.', '구멍–테두리 살 안 2.05, 바깥 1.8 mm: 2t 레이저가 어려우면 와이어커팅·워터젯.'], pdf, d)),
        ('clamp_ring_stator', lambda pdf, d: draw_ring(
            'clamp_ring_stator', 'WBv2-M08', '고정링 클램프 링 (clamp ring stator Ø45)',
            ['knee·wheel PETG 판 바깥 홈, 고정링 M2.5×8 머리 밑.',
             '안쪽 살(구멍–안지름) 1.85 mm, 바깥 살 2.85 mm: 2t 레이저가 어려우면 와이어커팅·워터젯.'], pdf, d)),
    ]
    for name, fn in jobs:
        with PdfPages(OUT / (name + '.pdf'), metadata={'Title': name, 'Author': PROJECT}) as pdf:
            fn(pdf, a.png)
        print('drawings/%s.pdf' % name)


if __name__ == '__main__':
    main()
