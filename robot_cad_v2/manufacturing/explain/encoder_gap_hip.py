"""Hip encoder stack: section through the hip plate, AKE motor (simplified), magnet spacer, driver cup and moteus r4.11
along the motor axis (plane of the board's connector axis), with the dimension chain from the plate pocket floor to the
die, a zoom of the magnet - chip gap and the tolerance stack.  From fusion/params.py; explanatory figure, not a
manufacturing drawing.

    python encoder_gap_hip.py        # -> encoder_gap_hip.png next to this script
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
from matplotlib.patches import Rectangle, Polygon
from matplotlib.ticker import MultipleLocator
F = '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc'
font_manager.fontManager.addfont(F)
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['font.family'] = font_manager.FontProperties(fname=F).get_name()

A, MO = P.AKE, P.MOTEUS
RING = A['ring']                                    # -8.0 pilot end = plate pocket floor (hip plate Z 5.5)
ZOFF = RING - 5.5                                   # motor y = plate Z + ZOFF (plate Z 0 = hub outer face)
MOUNT = 8.0 + ZOFF                                  # -5.5 plate web inner face = cup rim
BOSS80 = 9.5 + ZOFF                                 # -4.0 Ø80 register boss face
BOSS = A['magnet']                                  # 16.5 rotor Ø9 boss face
SP = P.SPACER_H
(SD, SL), (PD, PDEP) = P.SPACER_SPIGOT, P.SPACER_POCKET
MAG_TOP = BOSS + SP                                 # 21.85
B = P.board_offset()                                # 24.40 PCB motor-side face
PCB_T = MO['pcb_t']
F_IN = B + PCB_T + MO['top_h'] + P.CUP_CLEAR_TOP    # 32.55 cup floor inside
F_OUT = F_IN + P.CUP_WALL                           # 34.95
A1, A2, DIE_H = 0.10, 1.00, 0.85
TOP, DIE = B - A1 - A2, B - DIE_H                   # 23.30 chip top, 23.55 die surface
CAD_TOP = B - MO['ic_h']
C = dict(plate='#9aa3ad', motor='#c8c8c8', rotor='#9fb0c4', al='#9fb4c8', cup='#cfe3cf', pcb='#2e7d32', so='#f0c987',
         ic='#333333', die='#f2c14e', n='#d9534f', s='#428bca')
DIE_C = '#c77d00'

fig = plt.figure(figsize=(21, 14.5))


def mirror(pts):
    return pts + [(-x, y) for x, y in reversed(pts)]


def hdim(a, x0, x1, y, text, col='k', off=0.8, fs=9.5):
    a.annotate('', xy=(x1, y), xytext=(x0, y), arrowprops=dict(arrowstyle='<->', color=col, lw=1.0, shrinkA=0, shrinkB=0), zorder=9)
    a.text((x0 + x1) / 2, y + off, text, ha='center', va='center', color=col, fontsize=fs, zorder=9,
           bbox=dict(fc='w', ec='none', alpha=0.8, pad=0.3))


def vdim(a, x, y0, y1, text, col='k', dx=0.6, fs=9.5, ha='left'):
    a.annotate('', xy=(x, y1), xytext=(x, y0), arrowprops=dict(arrowstyle='<->', color=col, lw=1.1, shrinkA=0, shrinkB=0), zorder=9)
    a.plot([x - 0.7, x + 0.7], [y0, y0], color=col, lw=0.6, zorder=9); a.plot([x - 0.7, x + 0.7], [y1, y1], color=col, lw=0.6, zorder=9)
    a.text(x + (dx if ha == 'left' else -dx), (y0 + y1) / 2, text, ha=ha, va='center', color=col, fontsize=fs, zorder=9,
           bbox=dict(fc='w', ec='none', alpha=0.85, pad=0.3))


# ------------------------------------------------------------------ 1. whole stack (plane through the motor axis and the board z axis)
ax = fig.add_axes([0.04, 0.49, 0.94, 0.48]); ax.set_aspect('equal')
# hip plate (machined 6061): hub Ø52 Z0..3, web Z3..8, Ø42 pocket Z5.5..9.5, Ø80 boss Z8..9.5 with the Ø63 x 0.5 relief, Ø30 hole
RD, RH = P.HIP_STATOR_RELIEF
plate = [(15.0, 0.0), (26.0, 0.0), (26.0, 3.0), (60.0, 3.0), (60.0, 8.0), (40.0, 8.0), (40.0, 9.5), (RD / 2, 9.5), (RD / 2, 9.5 - RH),
         (P.PILOT_D_HIP / 2, 9.5 - RH), (P.PILOT_D_HIP / 2, 5.5), (15.0, 5.5)]
for sx in (1, -1):
    ax.add_patch(Polygon([(sx * r, z + ZOFF) for r, z in plate], fc=C['plate'], ec='k', lw=0.9, zorder=2))
ax.text(47.0, 3.2 + ZOFF, 'hip 판 (6061 가공)', fontsize=9.5, zorder=8)
# AKE60 (simplified): output disc, pilot, stator, spinning can + cap, Ø9 boss
ax.add_patch(Polygon(mirror([(13.5, A['out']), (13.5, RING + 0.5)]), fc=C['rotor'], ec='k', lw=0.6, zorder=3))
ax.add_patch(Polygon(mirror([(13.6, RING), (21.0, RING), (21.0, RING + A['pilot_len']), (20.3, RING + A['pilot_len']),
                             (20.3, A['stator_front']), (30.75, A['stator_front']), (30.75, A['can_back']), (0.0, A['can_back'])]),
                     fc=C['motor'], ec='k', lw=0.8, zorder=3))
for sx in (1, -1):
    ax.add_patch(Rectangle((sx * 31.2 if sx > 0 else -A['can_r'], A['can_front']), A['can_r'] - 31.2, A['can_back'] - A['can_front'],
                           fc=C['rotor'], ec='k', lw=0.6, zorder=3))
ax.add_patch(Polygon(mirror([(0.0, A['can_back']), (34.5, A['can_back']), (34.5, A['cap_outer']), (31.0, A['cap_outer']),
                             (31.0, A['cap_face']), (4.5, A['cap_face']), (4.5, BOSS), (0.0, BOSS)]), fc=C['rotor'], ec='k', lw=0.8, zorder=3))
ax.text(-12.0, 6.0, 'AKE60-8\n(단순화)', fontsize=11, ha='center', zorder=8)
ax.text(-27.0, 12.0, '회전 캔', fontsize=8.5, zorder=8, rotation=90)
# spacer + magnet
ax.add_patch(Polygon(mirror([(0.0, BOSS - SL), (SD / 2, BOSS - SL), (SD / 2, BOSS), (4.5, BOSS), (4.5, MAG_TOP), (PD / 2, MAG_TOP),
                             (PD / 2, MAG_TOP - PDEP), (0.0, MAG_TOP - PDEP)]), fc=C['al'], ec='k', lw=0.8, zorder=4))
ax.add_patch(Rectangle((-3.0, MAG_TOP - 2.5), 3.0, 2.5, fc=C['n'], ec='k', lw=0.5, zorder=5))
ax.add_patch(Rectangle((0.0, MAG_TOP - 2.5), 3.0, 2.5, fc=C['s'], ec='k', lw=0.5, zorder=5))
# driver cup (PETG print): wall r 41..43.4 from the rim on the plate web, floor, one screw column in this plane (z = -40.5)
cup = [(41.0, MOUNT), (43.4, MOUNT), (43.4, F_OUT), (0.0, F_OUT), (0.0, F_IN), (41.0, F_IN)]
for sx in (1, -1):
    ax.add_patch(Polygon([(sx * r, y) for r, y in cup], fc=C['cup'], ec='k', lw=0.9, zorder=4))
col = [(-40.5 + 3.4, BOSS80 + 1.0), (-40.02, BOSS80 + 1.0), (-40.02, MOUNT), (-43.9, MOUNT), (-43.9, F_IN), (-40.5 + 3.4, F_IN)]
ax.add_patch(Polygon(col, fc=C['cup'], ec='k', lw=0.8, zorder=4))
ax.plot([-40.5, -40.5], [F_OUT - 4.45, MOUNT - 4.0], color='0.15', lw=2.2, zorder=6)
ax.text(-38.5, 9.0, 'M3 × 40\n→ hip 판 탭 B', fontsize=8.5, zorder=8)
# standoffs (x = ±20 in the board, projected) + PCB + XT30 + chip
for z in (-15.0, 23.0):
    ax.add_patch(Rectangle((z - 3.2, B + PCB_T), 6.4, F_IN - B - PCB_T, fc=C['so'], ec='k', lw=0.7, zorder=5))
    ax.plot([z, z], [B - 0.5, B + PCB_T + 5.0], color='0.2', lw=1.6, zorder=6)
ax.add_patch(Rectangle((MO['edge_far'], B), MO['edge_conn'] - MO['edge_far'], PCB_T, fc=C['pcb'], ec='k', lw=0.8, zorder=5))
ax.add_patch(Rectangle((27.0, B + PCB_T), 7.5, MO['top_h'], fc='#e0b030', ec='k', lw=0.6, zorder=5))
ax.text(30.75, B + PCB_T + 2.7, 'XT30', fontsize=8, ha='center', rotation=90, zorder=8)
ax.add_patch(Rectangle((-2.2, TOP), 4.4, A2, fc=C['ic'], ec='k', lw=0.6, zorder=6))
ax.text(-17.5, B + PCB_T + 0.3, 'moteus r4.11', fontsize=9, color='w', zorder=8, va='bottom')
ax.text(-15.0, F_IN - 1.8, '스탠드오프 (볼트 x = ±20을 투영)', fontsize=8.5, ha='center', zorder=8)
ax.text(30.0, F_OUT + 0.6, '드라이버 컵 (PETG 출력)', fontsize=9.5, zorder=8)
# chain dimensions (right side)
for y, col_, x0 in ((RING, 'k', 21.0), (MOUNT, 'k', 43.4), (B, 'k', 35.0), (B + PCB_T, 'k', 35.0), (BOSS, 'k', 4.5), (MAG_TOP, 'k', 4.5),
                    (DIE, DIE_C, 2.2)):
    ax.plot([x0, 61.0], [y, y], color=col_, lw=0.5, ls=':', zorder=7)
vdim(ax, 48.0, RING, MOUNT, '2.50 (가공)')
vdim(ax, 48.0, MOUNT, B + PCB_T, '%.2f (컵 출력: 테두리 → 스탠드오프 끝)' % (B + PCB_T - MOUNT))
vdim(ax, 56.5, B, B + PCB_T, 'PCB %.2f' % PCB_T, fs=8.5)
vdim(ax, -48.0, RING, BOSS, '24.50 (AKE 내부)', ha='right')
vdim(ax, -48.0, BOSS, MAG_TOP, '%.2f 스페이서' % SP, ha='right')
vdim(ax, -56.0, MAG_TOP, DIE, '%.2f 자석 → 다이' % (DIE - MAG_TOP), DIE_C, ha='right')
for y, t, dy in ((RING, '%.2f 포켓 바닥 = 모터 고정면' % RING, -0.9), (MOUNT, '%.2f 웹 안쪽 면 = 컵 테두리' % MOUNT, 0.6),
                 (B + PCB_T, '%.2f PCB 윗면 = 스탠드오프 끝' % (B + PCB_T), 0.9), (B, '%.2f PCB 모터 쪽 면' % B, -0.6),
                 (BOSS, '%.2f 로터 보스 끝면' % BOSS, 0.0), (MAG_TOP, '%.2f 자석 윗면 (다이 %.2f는 ② 확대)' % (MAG_TOP, DIE), -1.0)):
    ax.text(61.5, y + dy, t, fontsize=9, va='center', zorder=8)
ax.annotate('고정자 앞면 %.1f, Ø63 × %.1f 홈 바닥 %.1f (틈 %.1f)' % (A['stator_front'], RH, BOSS80 - RH, A['stator_front'] - BOSS80 + RH),
            xy=(26.0, BOSS80 - RH), xytext=(52.0, -12.5), fontsize=9, color='b', arrowprops=dict(arrowstyle='->', color='b', lw=0.8), zorder=9)
ax.set_xlim(-76, 92); ax.set_ylim(ZOFF - 2.5, F_OUT + 3.0)
ax.xaxis.set_major_locator(MultipleLocator(10)); ax.yaxis.set_major_locator(MultipleLocator(5))
ax.xaxis.set_minor_locator(MultipleLocator(1)); ax.yaxis.set_minor_locator(MultipleLocator(1))
ax.grid(which='major', color='0.9', lw=0.5, zorder=0)
ax.set_xlabel('보드 z 방향 [mm] (모터 축 = 0, + = 커넥터 쪽)', fontsize=10)
ax.set_ylabel('모터 축 방향 위치 y [mm] (params.py 모터 프레임)', fontsize=10)
ax.set_title('① hip 엔코더 쌓임 단면 (실제 비율, 왼쪽 hip, 모터는 단순화)', fontsize=13, loc='left')

# ------------------------------------------------------------------ 2. zoom of the gap
bx = fig.add_axes([0.05, 0.03, 0.42, 0.40]); bx.set_aspect(1.6)
bx.add_patch(Polygon(mirror([(0.0, BOSS - 0.5), (4.5, BOSS - 0.5), (4.5, MAG_TOP), (PD / 2, MAG_TOP), (PD / 2, MAG_TOP - PDEP),
                             (0.0, MAG_TOP - PDEP)]), fc=C['al'], ec='k', lw=0.8, zorder=2))
bx.add_patch(Rectangle((-3.0, MAG_TOP - 2.5), 3.0, 2.5, fc=C['n'], ec='k', lw=0.6, zorder=3))
bx.add_patch(Rectangle((0.0, MAG_TOP - 2.5), 3.0, 2.5, fc=C['s'], ec='k', lw=0.6, zorder=3))
bx.add_patch(Rectangle((-6.5, B), 13.0, PCB_T, fc=C['pcb'], ec='k', lw=0.8, zorder=2))
bx.add_patch(Rectangle((-2.2, TOP), 4.4, A2, fc=C['ic'], ec='k', lw=0.8, zorder=3))
for sx in (-1, 1):
    bx.plot([sx * 2.2, sx * 2.55, sx * 2.75, sx * 3.2], [B - A1 - 0.5, B - A1 - 0.5, B - 0.02, B - 0.02], color='0.55', lw=2.0, zorder=3)
bx.add_patch(Rectangle((-1.3, DIE), 2.6, 0.203, fc=C['die'], ec='k', lw=0.5, zorder=4))
bx.plot([-1.1, 1.1], [DIE, DIE], 'o', color='r', ms=4, zorder=5)
bx.add_patch(Rectangle((-2.6, CAD_TOP), 5.2, B - CAD_TOP, fc='none', ec='m', lw=1.0, ls='--', zorder=4))
vdim(bx, 3.6, MAG_TOP, TOP, '칩 겉면 ≈%.2f' % (TOP - MAG_TOP), 'k', dx=0.15, fs=9)
vdim(bx, 5.4, MAG_TOP, DIE, '다이 ≈%.2f' % (DIE - MAG_TOP), DIE_C, dx=0.15, fs=9)
vdim(bx, -3.7, MAG_TOP, B, 'PCB %.2f' % (B - MAG_TOP), '0.3', dx=0.15, fs=9, ha='right')
bx.text(-6.3, CAD_TOP - 0.25, 'CAD 칩(1.40) %.2f' % (CAD_TOP - MAG_TOP), color='m', fontsize=8.5)
bx.set_xlim(-6.5, 7.5); bx.set_ylim(BOSS + 3.0, B + PCB_T + 0.6)
bx.yaxis.set_major_locator(MultipleLocator(0.5)); bx.yaxis.set_minor_locator(MultipleLocator(0.1))
bx.xaxis.set_major_locator(MultipleLocator(1))
bx.grid(axis='y', which='major', color='0.88', lw=0.5, zorder=0)
bx.set_ylabel('y [mm]', fontsize=10)
bx.set_title('② 자석 – 칩 확대 (세로 1.6배, 6관절 같음)', fontsize=13, loc='left')

# ------------------------------------------------------------------ 3. tolerance stack, hip vs wheel
cx = fig.add_axes([0.53, 0.03, 0.46, 0.40]); cx.axis('off')
rows = [('③ 자석 → 다이 1.70 mm의 오차 (± mm)', '', '', 'k', 13),
        ('항목', 'hip', 'wheel·knee', '0.3', 10),
        ('모터 고정면 → 컵이 앉는 면', '2.50 가공 ±0.10', '23.50 출력 ±0.15', 'k', 10),
        ('AKE: 고정면 → 로터 보스 (미공개)', '±0.10', '±0.10', 'k', 10),
        ('스페이서 몸통 5.35', '±0.05', '±0.05', 'k', 10),
        ('자석 두께 - 포켓 깊이', '±0.10', '±0.10', 'k', 10),
        ('컵: 테두리 → 스탠드오프 끝', '31.55 출력 ±0.20', '10.55 출력 ±0.15', 'k', 10),
        ('PCB 두께 1.65', '±0.10', '±0.10', 'k', 10),
        ('다이 높이 0.85', '±0.15', '±0.15', 'k', 10),
        ('합계 (RSS)', '약 ±0.32', '약 ±0.32', 'r', 10.5),
        ('', '', '', 'k', 6),
        ('허용 (모든 등급·온도에서 35~70 mT): 다이 1.24~2.26', '', '', 'g', 10),
        ('hip은 높이 기준이 가공면이지만 컵이 31.55 mm로 깊어서', '', '', '0.3', 10),
        ('출력 오차 몫이 커짐 → 조립할 때 한 번 재서 맞춤:', '', '', '0.3', 10),
        ('웹 안쪽 면 → 자석 윗면 27.35 (목표), 테두리 → 스탠드오프 끝 31.55,', '', '', '0.3', 10),
        ('자석 → PCB = 31.55 - PCB 두께 - 27.35 = 2.55 (2.34~2.76)', '', '', '0.3', 10)]
y = 0.98
for t, h, w, col_, fs in rows:
    cx.text(0.0, y, t, fontsize=fs, color=col_, va='top', transform=cx.transAxes)
    if h:
        cx.text(0.62, y, h, fontsize=fs, color=col_, va='top', transform=cx.transAxes)
        cx.text(0.83, y, w, fontsize=fs, color=col_, va='top', transform=cx.transAxes)
    y -= 0.058 if fs > 8 else 0.02
fig.savefig(sys.argv[1] if len(sys.argv) > 1 else HERE / 'encoder_gap_hip.png', dpi=95, bbox_inches='tight')
