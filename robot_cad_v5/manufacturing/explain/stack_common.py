"""Shared part of the v5 joint section figures (real_sections.py): the v4 params and legs_v5 loaded outside Fusion, the
font, the loads, the colours and the annotation helpers (dimension, label, tag, the countersunk screw ⑥ that is not in
the model, the text blocks).  Robot y on the horizontal axis, radius from the joint axis vertically; csk_screw takes
the half s (+1 upper, -1 lower).
"""
import pathlib
import sys
from unittest import mock
HERE = pathlib.Path(__file__).resolve().parent
V5 = HERE.parents[1]
sys.path.insert(0, str(V5 / 'v4' / 'fusion'))
sys.path.insert(0, str(V5 / 'fusion'))
# legs_v5.py imports cadlib, which loads the Fusion API at import time; only its pure-Python dimensions are used
# here, so outside Fusion a stub stands in for adsk (no geometry is built)
for _m in ('adsk', 'adsk.core', 'adsk.fusion'):
    sys.modules.setdefault(_m, mock.MagicMock())
import params as P                               # noqa: E402
import link_length as LL                         # noqa: E402
LL.lengthen(P)                                   # v5: links 250 mm (user 11-07), before anything reads them at import
import v5_params as VP  # noqa: E402
VP.apply(P)                   # v5-only values (11-08)
import legs_v5 as LV                             # noqa: E402
import matplotlib                                # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt                  # noqa: E402
from matplotlib import font_manager              # noqa: E402
from matplotlib.patches import Polygon           # noqa: E402
for F in ('C:/Windows/Fonts/malgun.ttf', '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc',
          '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'):      # Windows PC, then the Ubuntu laptop's Korean font
    if pathlib.Path(F).exists():
        font_manager.fontManager.addfont(F)
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=F).get_name()
        break
plt.rcParams['axes.unicode_minus'] = False

F_DES, F_CHK = 300.0, 600.0                      # 5 / 10 x 60 N static (brief, v4 strength_v4.py)
FL_DES, FL_CHK = P.LATERAL * F_DES, P.LATERAL * F_CHK         # 150 / 300 N
CSK3_DK = 6.72                                   # ISO 10642 M3 head diameter

g = dict(plate='#c9d4de', insert_hub='#a9c6e4', thigh='#c7b6e6', shank='#9fd29f', cap='#9c8cc9', cup='#f5d0d8',
         stator='#b8b8b8', out='#d9b38c', pcb='#2f7d62', spacer='#dcdcdc',
         screw='#2e2e2e', washer='#e3c034', pin='#c0392b', ring='#6f86a6', insert='#b8860b', bearing='#8f8f8f',
         flange='#e07b39', rim='#f2a65a', tyre='#4a4a4a')
NEW = '#7b3fa0'                                  # tags of fasteners that are not in the model yet (README 5절)
__all__ = ['F_CHK', 'F_DES', 'FL_CHK', 'FL_DES', 'HERE', 'LV', 'NEW', 'P', 'csk_screw', 'dim_h', 'fits_block', 'g', 'label',
           'plt', 'rows_block', 'tag']        # what real_sections.py takes from here


# ----------------------------------------------------------------------------- drawing primitives
def poly(ax, pts, c, s=1, z=1, **kw):
    p = Polygon([(x, s * r) for x, r in pts], fc=c, ec=kw.pop('ec', 'k'), lw=kw.pop('lw', 0.9), zorder=z, **kw)
    ax.add_patch(p)
    return p


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


def label(ax, txt, xy, xt, fs=11, c='k', ha='left', rp=None):
    if rp is None:                               # leader leaves the text end that faces its target
        rp = (0, 0.5) if xy[0] < xt[0] + 3 else (1, 0.5)
    ax.annotate(txt, xy=xy, xytext=xt, fontsize=fs, color=c, ha=ha, va='center', zorder=11,
                arrowprops=dict(arrowstyle='->', lw=0.8, color=c, shrinkA=2, shrinkB=0, relpos=rp),
                bbox=dict(fc='white', ec='none', alpha=0.85, pad=0.6))


def csk_screw(ax, y_face, tip, r, s=1, z=3, dk=CSK3_DK, d=3.0):
    """ISO 10642 countersunk screw, head flush with y_face (90 deg cone), shank to tip."""
    sg = 1 if tip > y_face else -1
    k = (dk - d) / 2
    poly(ax, [(y_face, r - dk / 2), (y_face, r + dk / 2), (y_face + sg * k, r + d / 2), (tip, r + d / 2),
              (tip, r - d / 2), (y_face + sg * k, r - d / 2)], g['screw'], s, z)


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
