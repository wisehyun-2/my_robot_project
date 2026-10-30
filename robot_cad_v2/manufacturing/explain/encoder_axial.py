"""Axial view of the wheel driver cup: moteus r4.11 outline, mount holes / standoffs, AS5147(P) package and Hall circle,
AKE magnet, magnet spacer and the cup, projected along the motor axis (board frame, from main-branch params).  Explanatory figure, not a
manufacturing drawing.

    python encoder_axial.py          # -> encoder_axial.png next to this script
"""
import math
import pathlib
import sys
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / 'fusion'))
import params as P
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Circle, Rectangle, Polygon, FancyBboxPatch, Wedge
from matplotlib.ticker import MultipleLocator
F = '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc'
font_manager.fontManager.addfont(F)
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['font.family'] = font_manager.FontProperties(fname=F).get_name()

MO = P.MOTEUS
HOLES = MO['holes']
CUP_R_IN, CUP_R_OUT = 41.0, 43.4
LIP_IN, LIP_OUT = P.HOUSING_R + 0.05, P.HOUSING_R + 2.45      # register lip over the Ø88 link housing
CUP_COLUMNS = (-30.0, -90.0, 210.0)
SP_D = 9.0
(PD, _), MAG_D, HALL_R = P.SPACER_POCKET, 6.0, 1.1
D_BODY, E1_BODY, E_LEADS = 5.0, 4.4, 6.4                # TSSOP-14: body length, body width, lead tip span
C = dict(pcb='#2e7d32', so='#f0c987', cup='#cfd8dc', ic='#333333', n='#d9534f', s='#428bca', al='#9fb4c8')

fig = plt.figure(figsize=(20, 11.5))


def board_outline(ax, z_sign=-1):
    """r4.11 outline in the board frame; screen y = z_sign * z (connector edge down)."""
    hw, tab, z0, z1, zt = MO['half_w'], MO['tab_half_w'], MO['edge_far'], MO['edge_conn'], 26.3
    pts = [(-hw, z0), (hw, z0), (hw, zt), (tab, zt), (tab, z1), (-tab, z1), (-tab, zt), (-hw, zt)]
    ax.add_patch(Polygon([(x, z_sign * z) for x, z in pts], closed=True, fc=C['pcb'], ec='k', lw=1.0, alpha=0.30, zorder=2))


def chip(ax, lw=0.8):
    ax.add_patch(Rectangle((-D_BODY / 2, -E1_BODY / 2), D_BODY, E1_BODY, fc=C['ic'], ec='k', lw=lw, alpha=0.85, zorder=6))
    for i in range(7):
        x = -1.95 + 0.65 * i
        for sy in (-1, 1):
            ax.add_patch(Rectangle((x - 0.15, sy * E1_BODY / 2 if sy > 0 else -E_LEADS / 2), 0.30,
                                   (E_LEADS - E1_BODY) / 2, fc='0.7', ec='0.4', lw=0.4, zorder=6))


# ------------------------------------------------------------------ 1. whole cup, 1:1-ish
ax = fig.add_axes([0.02, 0.05, 0.50, 0.88]); ax.set_aspect('equal')
ax.add_patch(Wedge((0, 0), LIP_OUT, 0, 360, width=LIP_OUT - LIP_IN, fc='#eceff1', ec='0.4', lw=0.8, zorder=1))
ax.add_patch(Circle((0, 0), P.HOUSING_R, fc='none', ec='0.5', lw=0.8, ls='--', zorder=1))
ax.add_patch(Wedge((0, 0), CUP_R_OUT, 0, 360, width=CUP_R_OUT - CUP_R_IN, fc=C['cup'], ec='k', lw=0.8, zorder=1))
for t in CUP_COLUMNS:
    tt = math.radians(t)
    x, z = 40.5 * math.cos(tt), 40.5 * math.sin(tt)
    ax.add_patch(Circle((x, -z), 3.4, fc=C['cup'], ec='k', lw=0.8, zorder=2))
    ax.add_patch(Circle((x, -z), 1.4, fc='w', ec='k', lw=0.6, zorder=3))
board_outline(ax)
for (x, z) in HOLES:
    ax.add_patch(Circle((x, -z), 3.2, fc=C['so'], ec='k', lw=0.8, zorder=4))
    ax.add_patch(Circle((x, -z), 1.8, fc='w', ec='k', lw=0.6, zorder=5))
    ax.add_patch(Circle((x, -z), 1.3, fc='none', ec='b', lw=0.6, ls=':', zorder=5))
ax.add_patch(Circle((0, 0), SP_D / 2, fc=C['al'], ec='k', lw=0.8, zorder=5))
ax.add_patch(Wedge((0, 0), MAG_D / 2, 90, 270, fc=C['n'], ec='k', lw=0.6, zorder=6))
ax.add_patch(Wedge((0, 0), MAG_D / 2, -90, 90, fc=C['s'], ec='k', lw=0.6, zorder=6))
chip(ax, lw=0.6)
ax.plot([-46, 46], [0, 0], color='0.4', lw=0.5, ls='-.', zorder=7); ax.plot([0, 0], [-46, 46], color='0.4', lw=0.5, ls='-.', zorder=7)


def hdim(a, x0, x1, y, text, col='k', off=1.2):
    a.annotate('', xy=(x1, y), xytext=(x0, y), arrowprops=dict(arrowstyle='<->', color=col, lw=1.0, shrinkA=0, shrinkB=0), zorder=8)
    a.text((x0 + x1) / 2, y + off, text, ha='center', va='center', color=col, fontsize=10, zorder=8,
           bbox=dict(fc='w', ec='none', alpha=0.8, pad=0.5))


def vdim(a, x, y0, y1, text, col='k', off=1.0, ha='left'):
    a.annotate('', xy=(x, y1), xytext=(x, y0), arrowprops=dict(arrowstyle='<->', color=col, lw=1.0, shrinkA=0, shrinkB=0), zorder=8)
    a.text(x + (off if ha == 'left' else -off), (y0 + y1) / 2, text, ha=ha, va='center', color=col, fontsize=10, zorder=8,
           bbox=dict(fc='w', ec='none', alpha=0.8, pad=0.5))


hdim(ax, -20, 20, 21.5, '40 (볼트 간격)')
hdim(ax, -20, 20, -27.5, '40')
vdim(ax, 24.5, 0, 15, '15', off=0.8)
vdim(ax, 24.5, 0, -23, '23', off=0.8)
vdim(ax, -26.5, 15, -23, '38', off=0.8, ha='right')
for (x, z), r in ((HOLES[1], 25.0), (HOLES[3], math.hypot(20, 23))):
    ax.plot([0, x], [0, -z], color='m', lw=0.9, zorder=7)
    ax.text(x * 0.55 + 1.0, -z * 0.55 + (1.2 if z < 0 else -1.2), 'R%.2f' % r, color='m', fontsize=10, zorder=8,
            bbox=dict(fc='w', ec='none', alpha=0.8, pad=0.4))
hdim(ax, -CUP_R_IN, CUP_R_IN, 36.0, '컵 안지름 Ø%.0f' % (2 * CUP_R_IN), '0.3')
ax.text(0, -LIP_OUT - 1.8, '컵 립 안지름 Ø%.1f → 정강이 하우징 Ø%.0f(점선)에 끼움 = 중심 기준' % (2 * LIP_IN, 2 * P.HOUSING_R), ha='center',
        fontsize=9.5, color='0.35')
ax.text(-21.5, -(MO['edge_conn'] - 2.0), 'XT30 커넥터 쪽', fontsize=9, color='k')
ax.text(-15.0, 12.5, 'moteus r4.11 (46 × 53)', fontsize=10, color=C['pcb'], weight='bold')
ax.annotate('스탠드오프 Ø6.4\n인서트 구멍 Ø3.6\nPCB 구멍 Ø2.6 (파랑 점선)\nM2.5 볼트', xy=(23.0, 16.0), xytext=(27.5, 6.0), fontsize=8.5,
            va='center', arrowprops=dict(arrowstyle='->', lw=0.7), zorder=8)
ax.text(30.0, -30.0, '나사 기둥 3개\n(컵 → 하우징)', fontsize=8.5, va='center', color='0.3')
ax.set_xlim(-49, 49); ax.set_ylim(-49, 49)
ax.xaxis.set_major_locator(MultipleLocator(10)); ax.yaxis.set_major_locator(MultipleLocator(10))
ax.xaxis.set_minor_locator(MultipleLocator(1)); ax.yaxis.set_minor_locator(MultipleLocator(1))
ax.grid(which='major', color='0.88', lw=0.5, zorder=0)
ax.set_xlabel('보드 x [mm] (모터 축 = 0)', fontsize=10); ax.set_ylabel('보드 z [mm] (아래 = 커넥터 쪽)', fontsize=10)
ax.set_yticks(range(-40, 50, 10)); ax.set_yticklabels([str(-v) for v in range(-40, 50, 10)])
ax.set_title('① cup_wheel 축 방향 투영 (모터 축과 평행하게 본 모습, 부품을 겹쳐 그림, 왼쪽 다리)', fontsize=13, loc='left')

# ------------------------------------------------------------------ 2. centre zoom
bx = fig.add_axes([0.55, 0.05, 0.44, 0.88]); bx.set_aspect('equal')
bx.add_patch(Circle((0, 0), SP_D / 2, fc=C['al'], ec='k', lw=1.0, zorder=1))
bx.add_patch(Circle((0, 0), PD / 2, fc='#e8eef4', ec='k', lw=0.8, ls='--', zorder=2))
bx.add_patch(Wedge((0, 0), MAG_D / 2, 90, 270, fc=C['n'], ec='k', lw=0.8, alpha=0.9, zorder=3))
bx.add_patch(Wedge((0, 0), MAG_D / 2, -90, 90, fc=C['s'], ec='k', lw=0.8, alpha=0.9, zorder=3))
bx.text(-1.7, 2.25, 'N', color='w', fontsize=13, weight='bold', ha='center', zorder=4)
bx.text(1.7, 2.25, 'S', color='w', fontsize=13, weight='bold', ha='center', zorder=4)
chip(bx, lw=1.0)
bx.add_patch(Rectangle((-1.3, -1.3), 2.6, 2.6, fc='#f2c14e', ec='k', lw=0.6, alpha=0.9, zorder=7))
bx.add_patch(Circle((0, 0), HALL_R, fc='none', ec='r', lw=1.4, zorder=8))
for k in range(4):
    a = math.radians(90 * k)
    bx.plot(HALL_R * math.cos(a), HALL_R * math.sin(a), 's', color='r', ms=6, zorder=9)
bx.add_patch(Circle((0, 0), 0.2, fc='none', ec='lime', lw=1.6, zorder=9))
bx.add_patch(Circle((0, 0), 0.3, fc='none', ec='orange', lw=1.2, ls='--', zorder=9))
bx.plot([-6, 6], [0, 0], color='0.4', lw=0.5, ls='-.', zorder=10); bx.plot([0, 0], [-6, 6], color='0.4', lw=0.5, ls='-.', zorder=10)
hdim(bx, -SP_D / 2, SP_D / 2, 4.85, '스페이서 몸통 Ø9', off=0.3)
hdim(bx, -PD / 2, PD / 2, -4.0, '포켓 Ø%.2f ± 0.05' % PD, '0.25', off=-0.3)
hdim(bx, -MAG_D / 2, MAG_D / 2, -4.9, '자석 Ø6 (±0.1)', C['n'], off=-0.3)
hdim(bx, -D_BODY / 2, D_BODY / 2, 3.6, '칩 몸체 5.0', 'k', off=0.3)
vdim(bx, 3.3, -E1_BODY / 2, E1_BODY / 2, '칩 몸체 4.4', 'k', off=0.15)
vdim(bx, 5.0, -E_LEADS / 2, E_LEADS / 2, '리드\n끝\n6.4', '0.35', off=0.12)
hdim(bx, -HALL_R, HALL_R, -1.75, '홀 원 Ø2.2', 'r', off=-0.3)
bx.annotate('홀 소자 4개 (반경 1.1, 다이 표면)', xy=(0, HALL_R), xytext=(-5.8, 4.4), color='r', fontsize=10,
            arrowprops=dict(arrowstyle='->', color='r'), zorder=11)
bx.annotate('다이 (노랑, 패키지 안, 크기는 그림용)', xy=(-1.3, -0.9), xytext=(-5.8, -2.6), fontsize=10,
            arrowprops=dict(arrowstyle='->'), zorder=11)
bx.annotate('칩 중심 허용 원\n반경 0.2: 모든 등급·온도에서 35~70 mT\n반경 0.3 (주황 점선): 상온이면 만족', xy=(0.14, -0.14),
            xytext=(1.2, -5.7), fontsize=10, color='green', arrowprops=dict(arrowstyle='->', color='green'), zorder=11)
bx.text(-5.9, 5.75, '자석은 로터와 함께 돎 → N/S 방향은 계속 바뀜 (그림은 한 순간)', fontsize=9.5, color='0.3')
bx.set_xlim(-6.2, 6.2); bx.set_ylim(-6.2, 6.2)
bx.xaxis.set_major_locator(MultipleLocator(1)); bx.yaxis.set_major_locator(MultipleLocator(1))
bx.xaxis.set_minor_locator(MultipleLocator(0.1)); bx.yaxis.set_minor_locator(MultipleLocator(0.1))
bx.grid(which='major', color='0.85', lw=0.5, zorder=0)
bx.set_xlabel('[mm] (모터 축 = 0)', fontsize=10)
bx.set_title('② 가운데 확대: 칩·홀 원·자석·포켓·스페이서', fontsize=13, loc='left')
fig.savefig(sys.argv[1] if len(sys.argv) > 1 else HERE / 'encoder_axial.png', dpi=95, bbox_inches='tight')
