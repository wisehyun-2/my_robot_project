"""Encoder gap figure from the main-branch params (magnet spacer 5.35, MAGNET_PCB 2.55): AKE rotor boss + spacer +
moteus r4.11 AS5147(P) section along the motor axis with mm axes, and Bz vs magnet-to-die distance.
Explanatory figure, not a manufacturing drawing.

    python encoder_gap.py            # -> encoder_gap.png next to this script
"""
import pathlib
import sys
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / 'fusion'))
import params as P
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Rectangle, Polygon
from matplotlib.ticker import MultipleLocator
F = '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc'
font_manager.fontManager.addfont(F)
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['font.family'] = font_manager.FontProperties(fname=F).get_name()

BOSS = P.AKE['magnet']                       # 16.5 rotor Ø9 boss face
SP = P.SPACER_H                              # 5.35
(SD, SL), (PD, PDEP) = P.SPACER_SPIGOT, P.SPACER_POCKET
MAG_TOP = BOSS + SP                          # 21.85 magnet top = pocket rim
B = P.board_offset()                         # 24.4 PCB motor-side face (cup standoffs)
CAD_TOP = B - P.MOTEUS['ic_h']               # 23.0 r4.5 STEP chip block (1.40)
A1, A2 = 0.10, 1.00                          # TSSOP-14 standoff 0.05..0.15, body 0.80..1.05
TOP = B - A1 - A2                            # 23.30 real package top (faces the magnet)
DIE_H = 0.85                                 # die surface above the PCB (review notes section 10: 0.85 +-0.15)
DIE = B - DIE_H                              # 23.55
OLD_SP = 4.5
C = dict(rotor='#bdbdbd', al='#9fb4c8', pcb='#2e7d32', ic='#333333', die='#f2c14e', n='#d9534f', s='#428bca')
DIE_C = '#c77d00'

fig = plt.figure(figsize=(20, 12.5))
# ------------------------------------------------------------------ 1. section with mm axes
VX = 2.5
ax = fig.add_axes([0.05, 0.05, 0.55, 0.88]); ax.set_aspect(VX)
for side in ('top',):
    ax.spines[side].set_visible(False)
ax.set_xlim(-17.5, 23.6); ax.set_ylim(12.6, 26.6)
ax.yaxis.set_major_locator(MultipleLocator(1.0)); ax.yaxis.set_minor_locator(MultipleLocator(0.1))
ax.set_xticks(range(-6, 7, 2)); ax.xaxis.set_minor_locator(MultipleLocator(0.5))
ax.tick_params(axis='y', which='major', length=7, labelsize=10); ax.tick_params(axis='y', which='minor', length=3)
ax.tick_params(axis='x', labelsize=9)
ax.set_ylabel('모터 축 방향 위치 y [mm]  (params.py 모터 프레임, +y = 모터 뒤쪽 = 보드 쪽)', fontsize=11)
ax.set_xlabel('반지름 방향 [mm] (축 중심 0)', fontsize=10)
ax.grid(axis='y', which='major', color='0.85', lw=0.6, zorder=0)
ax.grid(axis='y', which='minor', color='0.94', lw=0.4, zorder=0)
sec = ax.secondary_yaxis('right', functions=(lambda y: y - MAG_TOP, lambda d: d + MAG_TOP))
sec.yaxis.set_major_locator(MultipleLocator(0.5)); sec.yaxis.set_minor_locator(MultipleLocator(0.1))
sec.set_ylabel('자석 윗면에서 거리 [mm]', fontsize=11, color='#7a3e00')
sec.tick_params(colors='#7a3e00', labelsize=10)

ax.add_patch(Polygon([(-7, 12.6), (7, 12.6), (7, 15.5), (4.5, 15.5), (4.5, BOSS), (3.0, BOSS), (3.0, BOSS - 2.5),
                      (-3.0, BOSS - 2.5), (-3.0, BOSS), (-4.5, BOSS), (-4.5, 15.5), (-7, 15.5)], fc=C['rotor'], ec='k',
                     lw=0.8, zorder=2))
ax.text(-6.6, 13.3, 'AKE60 로터 (회전)', fontsize=10, va='center', zorder=3)
ax.add_patch(Polygon([(-SD / 2, BOSS - SL), (SD / 2, BOSS - SL), (SD / 2, BOSS), (4.5, BOSS), (4.5, MAG_TOP), (PD / 2, MAG_TOP),
                      (PD / 2, MAG_TOP - PDEP), (-PD / 2, MAG_TOP - PDEP), (-PD / 2, MAG_TOP), (-4.5, MAG_TOP), (-4.5, BOSS),
                      (-SD / 2, BOSS)], fc=C['al'], ec='k', lw=0.9, zorder=2))
ax.add_patch(Rectangle((-3.0, MAG_TOP - 2.5), 3.0, 2.5, fc=C['n'], ec='k', lw=0.8, zorder=3))
ax.add_patch(Rectangle((0.0, MAG_TOP - 2.5), 3.0, 2.5, fc=C['s'], ec='k', lw=0.8, zorder=3))
ax.text(-1.5, MAG_TOP - 1.25, 'N', color='w', ha='center', va='center', fontsize=12, weight='bold', zorder=4)
ax.text(1.5, MAG_TOP - 1.25, 'S', color='w', ha='center', va='center', fontsize=12, weight='bold', zorder=4)
ax.add_patch(Rectangle((-7, B), 14, P.MOTEUS['pcb_t'], fc=C['pcb'], ec='k', lw=0.8, zorder=2))
ax.text(0.3, B + 0.82, 'moteus r4.11 PCB 1.65', color='w', fontsize=9.5, va='center', zorder=3)
ax.add_patch(Rectangle((-2.2, TOP), 4.4, A2, fc=C['ic'], ec='k', lw=0.8, zorder=3))
for sx in (-1, 1):
    ax.plot([sx * 2.2, sx * 2.55, sx * 2.75, sx * 3.2], [B - A1 - 0.5, B - A1 - 0.5, B - 0.02, B - 0.02], color='0.55',
            lw=2.2, zorder=3)
ax.add_patch(Rectangle((-1.3, DIE), 2.6, 0.203, fc=C['die'], ec='k', lw=0.6, zorder=4))
ax.plot([-1.1, 1.1], [DIE, DIE], 'o', color='r', ms=4, zorder=5)
ax.add_patch(Rectangle((-2.6, CAD_TOP), 5.2, B - CAD_TOP, fc='none', ec='m', lw=1.2, ls='--', zorder=4))

kw = dict(fontsize=9.5, va='center', zorder=6, arrowprops=dict(arrowstyle='->', lw=0.8))
ax.annotate('moteus r4.11 엔코더 AS5147(P), TSSOP-14\n실물 높이 약 1.1 (최대 1.2, 데이터시트)', xy=(-2.2, TOP + 0.35),
            xytext=(-17.2, 25.6), **kw)
ax.annotate('다이(노랑) 표면: 기판 위 약 0.85 ± 0.15\n빨간 점 = 홀 센서 (반경 1.1)', xy=(-1.1, DIE), xytext=(-17.2, 22.9), **kw)
ax.annotate('AKE 부속 자석 Ø6 × 2.5 (지름 방향 자화)\n포켓 Ø%.2f ± 0.05 × 깊이 %.1f' % (PD, PDEP), xy=(-2.2, MAG_TOP - 0.8),
            xytext=(-17.2, 20.4), **kw)
ax.annotate('자석 스페이서 (main) 몸통 Ø9 × %.2f ± 0.05\n스피곳 Ø%.2f ± 0.05 × %.1f → 로터 자석 자리' % (SP, SD, SL),
            xy=(-4.5, 18.0), xytext=(-17.2, 17.6), **kw)
ax.annotate('CAD 칩 (보라 점선) = r4.5 STEP 블록 1.40', xy=(2.6, CAD_TOP + 0.6), xytext=(6.5, 26.25), color='m',
            fontsize=9.5, va='center', zorder=6, arrowprops=dict(arrowstyle='->', color='m', lw=0.8))


def vdim(x, y0, y1, text, col):
    ax.annotate('', xy=(x, y1), xytext=(x, y0), zorder=6,
                arrowprops=dict(arrowstyle='<->', color=col, lw=1.4, shrinkA=0, shrinkB=0))
    ax.text(x + 0.25, (y0 + y1) / 2, text, color=col, fontsize=10, va='center', zorder=6)


for y, col, x0 in ((MAG_TOP, '0.35', 3.0), (CAD_TOP, 'm', 2.6), (TOP, 'k', 2.2), (DIE, DIE_C, 1.3), (B, '0.35', 7.0)):
    ax.plot([x0, 14.4], [y, y], color=col, lw=0.7, ls=':', zorder=5)
vdim(5.6, MAG_TOP, CAD_TOP, 'CAD\n%.2f' % (CAD_TOP - MAG_TOP), 'm')
vdim(7.9, MAG_TOP, TOP, '칩 겉면\n≈%.2f' % (TOP - MAG_TOP), 'k')
vdim(10.2, MAG_TOP, DIE, '다이\n≈%.2f' % (DIE - MAG_TOP), DIE_C)
vdim(12.3, MAG_TOP, B, 'PCB\n%.2f' % (B - MAG_TOP), '0.25')
for y, t, col, dy in ((B, '%.2f 보드 모터 쪽 면 (컵 고정)' % B, 'k', 0.12), (DIE, '≈%.2f 다이 표면' % DIE, DIE_C, 0.1),
                      (TOP, '≈%.2f 실제 칩 겉면' % TOP, 'k', -0.02), (CAD_TOP, '%.2f CAD 칩 면' % CAD_TOP, 'm', -0.12),
                      (MAG_TOP, '%.2f 자석 윗면 = 스페이서 윗면' % MAG_TOP, 'k', 0.0),
                      (BOSS, '%.2f 로터 보스 끝면' % BOSS, 'k', 0.0), (BOSS - SL, '%.2f 스피곳 끝' % (BOSS - SL), '0.3', 0.15),
                      (BOSS - 2.5, '%.2f 로터 자석 자리 바닥 (STEP)' % (BOSS - 2.5), '0.3', -0.15)):
    ax.text(14.6, y + dy, t, fontsize=9, va='center', color=col, zorder=6)
ax.set_title('① 모터 축 단면 — main 치수 (세로 2.5배 확대, 눈금은 실제 mm)', fontsize=13, loc='left')

# ------------------------------------------------------------------ 2. Bz vs die distance
MU0 = 4e-7 * np.pi


def bz(R, H, Br, d, r=1.1e-3, n_phi=360, n_z=100):
    M = Br / MU0
    phi = (np.arange(n_phi) + 0.5) * 2 * np.pi / n_phi; zs = (np.arange(n_z) + 0.5) * H / n_z
    Pp, Z = np.meshgrid(phi, zs)
    dx, dy, dz = r - R * np.cos(Pp), -R * np.sin(Pp), -d - Z
    Hz = np.sum(M * np.cos(Pp) * dz / (dx * dx + dy * dy + dz * dz) ** 1.5) * R * (2 * np.pi / n_phi) * (H / n_z) / (4 * np.pi)
    return abs(MU0 * Hz * 1e3)


bx = fig.add_axes([0.70, 0.56, 0.28, 0.37])
ds = np.linspace(1.0, 3.2, 45)
bx.axhspan(35, 70, color='#c8e6c9', alpha=0.7, label='AS5147P 요구 35~70 mT')
for name, Br, ls in (('Ø6×2.5 N38H', 1.23, '-'), ('Ø6×2.5 N35SH', 1.19, '--')):
    bx.plot(ds, [bz(3e-3, 2.5e-3, Br, d * 1e-3) for d in ds], 'k' + ls, lw=1.6, label=name)
now = DIE - MAG_TOP
win = (2.34 - DIE_H, 2.76 - DIE_H)
bx.axvspan(*win, color='g', alpha=0.12)
bx.axvline(now, color='g', lw=1.8)
b = bz(3e-3, 2.5e-3, 1.23, now * 1e-3)
bx.plot(now, b, 'o', color='g')
bx.text(now + 0.04, b + 3, 'main 스페이서 %.2f\n다이 %.2f mm ≈%.0f mT\n허용 창 %.2f~%.2f' % (SP, now, b, win[0], win[1]),
        color='g', fontsize=9.5)
old = DIE - (BOSS + OLD_SP)
bo = bz(3e-3, 2.5e-3, 1.23, old * 1e-3)
bx.axvline(old, color='r', lw=1.2, ls=':')
bx.plot(old, bo, 'o', color='r')
bx.text(old + 0.04, bo + 3, '이전 4.5\n%.2f mm ≈%.0f mT' % (old, bo), color='r', fontsize=9.5)
bx.set_xlabel('자석 윗면 → 다이 표면 [mm]', fontsize=10)
bx.set_ylabel('Bz [mT] (다이, 반경 1.1 mm)', fontsize=10)
bx.set_ylim(15, 90); bx.set_xlim(1.0, 3.2); bx.grid(alpha=0.3); bx.legend(fontsize=9, loc='upper right')
bx.set_title('② 다이 자기장 (원통 자석 계산, ±20 % 정도)', fontsize=13, loc='left')

# ------------------------------------------------------------------ 3. numbers
cx = fig.add_axes([0.68, 0.04, 0.31, 0.45]); cx.axis('off')
rows = [('③ main 치수 (params.py, af87032)', '', 'k', 13),
        ("로터 보스 끝면  AKE['magnet']", '%.2f' % BOSS, 'k', 10),
        ('+ 스페이서 몸통  SPACER_H', '%.2f' % SP, 'k', 10),
        ('= 자석 윗면', '%.2f' % MAG_TOP, 'k', 10),
        ('+ 자석 윗면 → PCB 면  MAGNET_PCB', '%.2f' % P.MAGNET_PCB, 'k', 10),
        ('= PCB 모터 쪽 면  board_offset()', '%.2f' % B, 'k', 10),
        ('', '', 'k', 6),
        ('자석 → CAD 칩 (STEP 블록 1.40)', '%.2f' % (CAD_TOP - MAG_TOP), 'm', 10),
        ('자석 → 실제 칩 겉면 (칩 높이 0.85~1.20)', '%.2f~%.2f' % (P.MAGNET_PCB - 1.20, P.MAGNET_PCB - 0.85), 'k', 10),
        ('자석 → 다이 표면 (기판 위 0.85 ± 0.15)', '%.2f (%.2f~%.2f)' % (now, now - 0.15, now + 0.15), DIE_C, 10),
        ('', '', 'k', 6),
        ('허용: 자석 윗면–PCB 2.34~2.76 (검토사항 10절)', '', 'g', 10),
        ('mjbots 가이드 air gap 약 2 (1.5~2.5)는 칩 겉면 기준.', '', '0.3', 10),
        ('main은 겉면 약 1.45로 가이드보다 가깝지만', '', '0.3', 10),
        ('다이 자기장은 35~70 mT 창의 가운데(약 52 mT).', '', '0.3', 10)]
y = 0.98
for t, v, col, fs in rows:
    cx.text(0.0, y, t, fontsize=fs, color=col, va='top', transform=cx.transAxes)
    if v:
        cx.text(0.98, y, v, fontsize=fs, color=col, va='top', ha='right', transform=cx.transAxes)
    y -= 0.062 if fs > 8 else 0.025
fig.savefig(sys.argv[1] if len(sys.argv) > 1 else HERE / 'encoder_gap.png', dpi=95, bbox_inches='tight')
