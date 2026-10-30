"""Left hip joint: 6061 hip plate -> AKE60-8 -> thigh driven boss, plus the moteus driver cup on the plate's inner face --
the parts and screws that hang the leg on the torso, half sections to scale from fusion/params.py and the cup / hole
levels of fusion/legs.py (robot y of the LEFT leg on the horizontal axis, radius from the hip axis vertically; upper
half through an output M3 screw, lower half through a dowel pin, which shares its angle with a stator screw).
Overview (cup to thigh outer face) + enlarged detail, same style as wheel_output_stack.py.  Explanatory figure, not a
manufacturing drawing.

    python hip_joint_stack.py         # -> hip_joint_stack.png next to this script
"""
import math
import pathlib
import sys
from unittest import mock
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / 'fusion'))
# legs.py / layout.py import cadlib, which loads the Fusion API at import time; only their pure-Python frame and cup
# level helpers are used here, so outside Fusion a stub stands in for adsk (no geometry is built)
for _m in ('adsk', 'adsk.core', 'adsk.fusion'):
    sys.modules.setdefault(_m, mock.MagicMock())
import params as P
import legs as LG
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Polygon, Rectangle
for F in ('C:/Windows/Fonts/malgun.ttf', '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc'):
    if pathlib.Path(F).exists():
        font_manager.fontManager.addfont(F)
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=F).get_name()
        break
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['hatch.linewidth'] = 0.6
A = P.AKE

# ----------------------------------------------------------------------------- stations (robot y, left leg)
AX = P.AXIS_HIP                                  # -1: the hip motor's back points inboard (-y)


def ym(local):                                   # AKE local y -> robot y
    return P.Y_RING_HIP + AX * (local - A['ring'])


OUT = P.y_output('hip')                          # 167.0 output face (faces the thigh, +y)
RING = P.Y_RING_HIP                              # 166.5 pilot end face = pocket floor
YW = P.YW                                        # 172.0 hip plate hub face (outer)
Y_IN = YW - P.HIP_PLATE_T                        # 164.0 web inner face = cup rim
Y_WEB = Y_IN + P.HIP_WEB_T                       # 169.0 web outer face
BOSS_H, BOSS_R, HUB_R, WEB_R = 1.5, 40.0, 26.0, 68.0   # torso.hip_plate: Ø80 x 1.5 register boss, Ø52 hub, r 68 disc
BOSS_F = Y_IN - BOSS_H                           # 162.5 boss face
REL_R = P.HIP_STATOR_RELIEF[0] / 2               # 31.5
REL = BOSS_F + P.HIP_STATOR_RELIEF[1]            # 163.0 relief floor
POCK_R = P.PILOT_D_HIP / 2                       # Ø42 H7 pocket (mid tolerance)
HOLE_R = P.RING_STATOR[1] / 2                    # Ø30 hole (= stator ring ID, params)
CB_D, CB_T = P.CBORE_M25_HIP
CBF = YW - CB_T                                  # 169.3 counterbore floor = M2.5 head bearing face
S25_L, S25_K, S25_DK = 6, 2.5, 4.5               # ISO 4762 M2.5 x 6 (head k 2.5, dk 4.5)
S25_TIP = CBF - S25_L                            # 163.3
ENG25 = RING - S25_TIP                           # 3.2 in the 5 mm stator ring taps
FI, FO, FW = P.Y_FEMUR_IN, P.Y_FEMUR_OUT, P.FEMUR_WIDE_OUT     # thigh 174 .. 204, widened outboard to 250
BOSS28 = P.HUB_BOSS_D / 2                        # Ø28 driven boss
SPIG_R, SPIG_L = 5.9, 1.4                        # legs._driven_boss: Ø11.8 x 1.4 spigot
FLOOR = OUT + P.BEARING_FACE - P.RING_T          # 176.2 boss floor (output ring)
SEAT = OUT + P.BEARING_FACE                      # 178.2 washer seat
W_T, W_OD = 0.5, 7.0                             # DIN 125 M3
HB = SEAT + W_T                                  # 178.7 M3 head bearing face
M3_L, M3_K, M3_DK = 16, 3.0, 5.5                 # ISO 4762 M3 x 16
M3_TIP = HB - M3_L                               # 162.7
ENG3 = OUT - M3_TIP                              # 4.3 in the 5 mm output taps
REACH = FW - (HB + M3_K)                         # 68.3 thigh outer face -> M3 head top (wrench reach)
PIN0, PIN1 = OUT - 3.0, OUT + 7.0                # dowel 3 in the output + 7 in the boss
RP, RQ = P.AKE_PCD_RING / 2, P.AKE_PCD_OUT / 2
RO_OD, RO_ID = P.RING_OUT
# driver cup + moteus (board frame y: + away from the motor = robot -y here)
OB = ym(P.board_offset())                        # 134.1 PCB motor-side face


def yb(v):                                       # board y -> robot y
    return OB + AX * v


RIM_B, FOUT_B = LG.cup_levels('hip', 1)
RIM, FOUT, FIN = yb(RIM_B), yb(FOUT_B), yb(FOUT_B - P.CUP_WALL)   # 164.0, 123.55, 125.95
CUP_L, CUP_CB = LG.cup_screw('hip')              # M3 x 40, counterbore 4.45
CS_HB = yb(FOUT_B - CUP_CB)                      # 128.0 column screw head bearing face
CS_TIP = CS_HB - AX * CUP_L                      # 168.0
CS_ENG = CS_TIP - Y_IN                           # 4.0 in the 5 mm web
COL_R = math.hypot(*(lambda p: (p[0], p[2] - P.Z_HIP))(LG.cup_column_points('hip', 1)[0]))  # 40.5
_ang = sorted(((t + 180.0) % 360.0) - 180.0 for t in LG.CUP_COLUMNS)
COL_STEP = _ang[1] - _ang[0]                     # 60: the 3 columns sit 60 deg apart, away from the connector edge
REG_R = 40.02                                    # legs.cup: hip register cut over the Ø80 boss
COL_W, COL_HOLE, COL_CB = 3.4, 1.7, 3.0          # legs.cup: column r, M3 hole r, counterbore r
RB = sorted({round(math.hypot(x, z), 2) for x, z in P.MOTEUS['holes']})   # board screw radii 25.0, 30.48
PCB = P.MOTEUS['pcb_t']
INS0, INS_L = PCB + 0.1, 4.0                     # standoff insert M2.5 x 4 (hole Ø3.6 x 5)
BS_L = 6                                         # DIN 7984 M2.5 x 6 board screws (head 1.5)
MAG = ym(A['magnet'])                            # 142.0 rotor boss end (empty magnet seat)
SP1 = ym(A['magnet'] + P.SPACER_H)               # 136.65 spacer top = magnet top
BAR_R = math.hypot(P.TUBE_X, P.TUBE_Z)           # 61.7 spine bar axis from the hip axis

g = dict(plate='#c9d4de', thigh='#c7b6e6', cup='#f5d0d8', stator='#b8b8b8', rotor='#7f9fbf', out='#d9b38c',
         pcb='#2f7d62', spacer='#dcdcdc', magnet='#8e44ad', screw='#2e2e2e', washer='#e3c034', pin='#c0392b',
         ring='#6f86a6', insert='#b8860b', bar='#c9d4de')
HATCH = dict(hatch='////', hatchcolor='#8796a5')


def zig(p0, p1, n=4, a=0.8):
    """Break line points strictly between p0 and p1 (zigzag of amplitude a)."""
    (x0, y0), (x1, y1) = p0, p1
    L = math.hypot(x1 - x0, y1 - y0)
    nx, ny = -(y1 - y0) / L, (x1 - x0) / L
    return [(x0 + (x1 - x0) * k / (2 * n) + nx * a * (1 if k % 2 else -1),
             y0 + (y1 - y0) * k / (2 * n) + ny * a * (1 if k % 2 else -1)) for k in range(1, 2 * n)]


def draw_parts(ax, detail):
    a = 0.7 if detail else 1.4
    rb = 50.0 if detail else 82.0                # thigh beam break radius (lower half, towards the knee)
    yend = 196.0 if detail else FW               # thigh drawn up to here

    def poly(pts, c, s=1, z=1, **kw):
        ax.add_patch(Polygon([(x, s * r) for x, r in pts], fc=c, ec=kw.pop('ec', 'k'), lw=kw.pop('lw', 0.9),
                             zorder=z, **kw))

    def rect(x0, x1, r0, r1, c, s=1, z=1, **kw):
        poly([(x0, r0), (x1, r0), (x1, r1), (x0, r1)], c, s, z, **kw)

    for s in (1, -1):
        # 6061 hip plate: hub 8 (pocket, Ø30 hole, M2.5 counterbores run out into the hole), web 5, Ø80 x 1.5 boss
        rw = 50.0 if detail else (WEB_R if s < 0 else BAR_R + P.HIP_CSK_EAR_R)
        pts = [(RING, HOLE_R), (CBF, HOLE_R), (CBF, RP + CB_D / 2), (YW, RP + CB_D / 2), (YW, HUB_R), (Y_WEB, HUB_R),
               (Y_WEB, rw)]
        if detail:
            pts += zig((Y_WEB, rw), (Y_IN, rw), 2, a)
        pts += [(Y_IN, rw), (Y_IN, BOSS_R), (BOSS_F, BOSS_R), (BOSS_F, REL_R), (REL, REL_R), (REL, POCK_R),
                (RING, POCK_R)]
        poly(pts, g['plate'], s, **HATCH)
        # thigh (PETG): Ø28 boss + Ø11.8 spigot, head r 30 (upper), beam to the knee (lower), Ø28 access pocket
        top = P.HEAD_R if s > 0 else rb
        pts = [(OUT - SPIG_L, 0), (OUT - SPIG_L, SPIG_R), (OUT, SPIG_R), (OUT, BOSS28), (FI, BOSS28), (FI, top)]
        if s < 0:
            pts += zig((FI, top), (yend, top), 6 if detail else 9, a)
        pts += [(yend, top)]
        if yend < FW:
            pts += zig((yend, top), (yend, BOSS28), 3, a)
        pts += [(yend, BOSS28), (FLOOR, BOSS28), (FLOOR, 0)]
        poly(pts, g['thigh'], s)
        # AKE60-8: stator (Ø42 pilot x 3, front face r 20.3 .. 30.75), rotor can + cap, output carrier (Ø12 x 1.5 recess)
        sf, cb, pb = ym(A['stator_front']), ym(A['can_back']), ym(A['ring'] + A['pilot_len'])
        poly([(RING, 13.6), (RING, 21.0), (pb, 21.0), (pb, 20.3), (sf, 20.3), (sf, 30.75), (cb, 30.75), (cb, 13.6)],
             g['stator'], s)
        poly([(ym(A['can_front']), 31.2), (ym(A['can_front']), A['can_r']), (ym(A['cap_outer']), A['can_r']),
              (ym(A['cap_outer']), 31.2)], g['rotor'], s)
        poly([(cb, 7.9), (ym(A['cap_face']), 7.9), (ym(A['cap_face']), 34.2), (cb, 34.2)], g['rotor'], s)
        rect(cb, MAG, 0, A['boss_r'], g['rotor'], s)
        poly([(ym(A['out'] + 1.5), 0), (ym(A['out'] + 1.5), 6.0), (OUT, 6.0), (OUT, 13.5), (cb, 13.5), (cb, 0)],
             g['out'], s)
        # magnet spacer (6061) on the rotor boss, Ø6 x 2.5 magnet in its pocket
        rect(MAG, SP1, 0, 4.5, g['spacer'], s, z=1.5, lw=0.7)
        rect(ym(A['magnet'] - P.SPACER_SPIGOT[1]), MAG, 0, P.SPACER_SPIGOT[0] / 2, g['spacer'], s, z=1.5, lw=0.7)
        rect(SP1 - AX * P.SPACER_POCKET[1], SP1, 0, 3.0, g['magnet'], s, z=1.6, lw=0.6)
        # moteus r4.11 PCB + AS5047P on the motor side
        rect(yb(0), yb(PCB), 0, 33.0, g['pcb'], s, z=1.5, lw=0.7)
        rect(yb(-P.MOTEUS['ic_h']), yb(0), 0, 2.5, '#111111', s, z=1.6, lw=0.5)
        # driver cup (PETG): floor + wall r 41 .. 43.4; upper half through a screw column (register over the boss)
        if s > 0:
            poly([(FOUT, 0), (FOUT, COL_R + COL_W), (RIM, COL_R + COL_W), (RIM, REG_R), (BOSS_F, REG_R),
                  (BOSS_F, COL_R - COL_W), (FIN, COL_R - COL_W), (FIN, 0)], g['cup'], s)
            rect(FOUT, CS_HB, COL_R - COL_CB, COL_R + COL_CB, 'white', s, z=1.2, lw=0.6)
            rect(CS_HB, BOSS_F, COL_R - COL_HOLE, COL_R + COL_HOLE, 'white', s, z=1.2, lw=0.6)
            rect(BOSS_F, RIM, REG_R, COL_R + COL_HOLE, 'white', s, z=1.2, lw=0.6)
            rect(CS_TIP, Y_WEB, COL_R - 1.25, COL_R + 1.25, 'white', s, z=1.2, lw=0.6)    # M3 tap drill beyond the tip
        else:
            poly([(FOUT, 0), (FOUT, LG.CUP_R_OUT), (RIM, LG.CUP_R_OUT), (RIM, LG.CUP_R_IN), (FIN, LG.CUP_R_IN),
                  (FIN, 0)], g['cup'], s)
        # cup standoff + brass insert + board screw (from the motor side of the PCB)
        rs = RB[1] if s > 0 else RB[0]
        rect(yb(FOUT_B - P.CUP_WALL + 0.5), yb(PCB), rs - 3.2, rs + 3.2, g['cup'], s, z=1.1)
        rect(yb(PCB + 5.0), yb(PCB), rs - LG.INSERT_M25 / 2, rs + LG.INSERT_M25 / 2, 'white', s, z=1.2, lw=0.5)
        rect(yb(INS0 + INS_L), yb(INS0), rs - LG.INSERT_M25 / 2, rs + LG.INSERT_M25 / 2, g['insert'], s, z=2.5, lw=0.7)
        rect(yb(BS_L), yb(0), rs - 1.25, rs + 1.25, g['screw'], s, z=3)
        rect(yb(0), yb(-1.5), rs - 2.25, rs + 2.25, g['screw'], s, z=3)
        # stator screws: ISO 4762 M2.5 x 6 12.9 from the hub face counterbore (6 on PCD 36.5: both halves)
        rect(S25_TIP, CBF, RP - 1.25, RP + 1.25, g['screw'], s, z=3)
        rect(CBF, CBF + S25_K, RP - S25_DK / 2, RP + S25_DK / 2, g['screw'], s, z=3)
        # SUS304 output ring on the boss floor
        rect(FLOOR, FLOOR + P.RING_T, RO_ID / 2, RO_OD / 2, g['ring'], s, z=2, lw=1.0)
        if not detail and s > 0:
            # spine bar Ø16 (6061) end on the web inner face + ISO 10642 M5 x 12 from the web countersink
            pts = [(Y_IN, BAR_R - 8), (Y_IN, BAR_R + 8), (113.0, BAR_R + 8)] + zig((113.0, BAR_R + 8), (113.0, BAR_R - 8),
                                                                                  3, 1.4) + [(113.0, BAR_R - 8)]
            poly(pts, g['bar'], s, **HATCH)
            hk = (P.CSK_M5[1] - 5.0) / 2
            rect(Y_WEB - 12.0, Y_WEB - hk, BAR_R - 2.5, BAR_R + 2.5, g['screw'], s, z=3)
            poly([(Y_WEB - hk, BAR_R - 2.5), (Y_WEB, BAR_R - P.CSK_M5[1] / 2), (Y_WEB, BAR_R + P.CSK_M5[1] / 2),
                  (Y_WEB - hk, BAR_R + 2.5)], g['screw'], s, z=3)
    # upper half: output M3 x 16 at r 10 (washer under the head), cup column M3 x 40 at r 40.5
    rect(SEAT, HB, RQ - W_OD / 2, RQ + W_OD / 2, g['washer'], z=3, lw=0.8)
    rect(HB, HB + M3_K, RQ - M3_DK / 2, RQ + M3_DK / 2, g['screw'], z=3)
    rect(M3_TIP, HB, RQ - 1.5, RQ + 1.5, g['screw'], z=3)
    rect(CS_HB - M3_K, CS_HB, COL_R - M3_DK / 2, COL_R + M3_DK / 2, g['screw'], z=3)
    rect(CS_HB, CS_TIP, COL_R - 1.5, COL_R + 1.5, g['screw'], z=3)
    # lower half: dowel Ø3 x 10 (3 mm in the output, 7 mm in the boss)
    rect(PIN0, PIN1, -RQ - 1.5, -RQ + 1.5, g['pin'], z=3, lw=1.0)


def load_path(ax, lw, path):
    ax.plot(*zip(*path[:-1]), color='red', lw=lw, alpha=0.7, zorder=6, solid_capstyle='round', solid_joinstyle='round')
    ax.annotate('', xy=path[-1], xytext=path[-2], zorder=6,
                arrowprops=dict(arrowstyle='-|>', color='red', alpha=0.7, lw=lw, mutation_scale=8 * lw))


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
            va='bottom' if above else 'top', color=c, zorder=8)


def dim_v(ax, x, r0, r1, txt, fs=11, c='k', left=True):
    ax.annotate('', xy=(x, r0), xytext=(x, r1), arrowprops=dict(arrowstyle='<->', color=c, lw=0.9, shrinkA=0, shrinkB=0))
    ax.text(x + (-1.0 if left else 1.0), (r0 + r1) / 2, txt, fontsize=fs, rotation=90, ha='right' if left else 'left',
            va='center', color=c)


def label(ax, txt, xy, xt, fs=11, c='k', ha='left', rp=None):
    if rp is None:                               # leader leaves the text end that faces its target
        rp = (0, 0.5) if xy[0] < xt[0] + 3 else (1, 0.5)
    ax.annotate(txt, xy=xy, xytext=xt, fontsize=fs, color=c, ha=ha, va='center', zorder=11,
                arrowprops=dict(arrowstyle='->', lw=0.8, color=c, shrinkA=2, shrinkB=0, relpos=rp),
                bbox=dict(fc='white', ec='none', alpha=0.85, pad=0.6))


fig = plt.figure(figsize=(24, 28))
fig.suptitle('hip 관절 — 허벅지를 몸통(hip 판)에 매다는 연결  (왼쪽 다리 단면, 모델 치수 mm, 실제 비율)',
             fontsize=22, x=0.01, ha='left', y=0.995)

# ============================================================================= (a) overview
ax = fig.add_axes([0.01, 0.585, 0.37, 0.375]); ax.set_aspect('equal'); ax.axis('off')
draw_parts(ax, detail=False)
# thigh beam (lower) -> boss -> output -> through the motor -> stator (upper) -> plate -> web -> spine bar
load_path(ax, 3.0, [(FW - 12, -80), (205, -42), (186, -21), (FLOOR - 1.5, -10.5), (OUT + 1.2, -12.2), (OUT - 3, -9.5),
                    (157, -5), (157, 9), (ym(4), 17.5), (RING - 1.5, 19.2), (CBF - 1.2, 20.3), (CBF - 0.5, 24.5),
                    (Y_WEB - 2.5, 30), (Y_WEB - 2.5, BAR_R - 3), (Y_IN - 3, BAR_R - 1), (124, BAR_R - 1)])
ax.plot([FOUT - 4, FW + 4], [0, 0], 'k-.', lw=0.7, zorder=8)
ax.set_title('(a) 전체 — 드라이버 컵부터 허벅지 바깥 면까지', fontsize=17, loc='left')
T = dict(fontsize=12, ha='center', va='center', zorder=9)
ax.text((FI + FW) / 2 + 8, -66, 'PETG 허벅지 보\n↓ 무릎까지 (175)', **T)
ax.text((FI + FW) / 2 + 4, 22, '허벅지 머리 (PETG)', **T)
label(ax, '6061 hip 판 (웹 5 + 허브 3)', (Y_WEB - 1.0, 66.0), (178.0, 77.0), fs=12)
ax.text(136, BAR_R + 3.5, '척추 봉 Ø16 (6061)', fontsize=11, ha='center', va='center', zorder=9)
ax.text(140, BAR_R - 4.5, '← 힘이 몸통으로', color='red', fontsize=11, ha='center', va='center', zorder=9)
ax.text(Y_WEB + 2, BAR_R + 1, 'M5 × 12 접시 (몸통)', fontsize=10.5, ha='left', va='center', zorder=9)
ax.text(152, 23, 'AKE60-8', **T)
ax.text(FOUT + 2, -62, '드라이버 컵 + moteus', fontsize=11, ha='left', va='center', zorder=9)
ax.text(109, 79, '← 몸통 안쪽 (로봇 가운데)', fontsize=11, ha='left', va='center', zorder=9)
ax.text(FW + 5, 79, '바깥 (+y) →', fontsize=11, ha='right', va='center', zorder=9)
ax.text(FW - 12, -89, '바퀴·정강이·무릎에서 ↑', color='red', fontsize=11, ha='center', va='center', zorder=9)
R1, R2 = -100.0, -110.0
dim_h(ax, FOUT, RIM, R1, '컵 %.2f' % (RIM - FOUT), (-44, -70), fs=12)
dim_h(ax, BOSS_F, YW, R2, 'hip 판 %.1f' % (YW - BOSS_F), (-70, -26), fs=12)
dim_h(ax, FI, FO, R1, '30', (-82, -82), fs=12)
dim_h(ax, FO, FW, R1, '46 (보강)', (None, -82), fs=12)
dim_h(ax, FI, FW, R2, '허벅지 폭 %.0f' % (FW - FI), (-82, -82), fs=12)
dim_h(ax, HB + M3_K, FW, 40.0, '② 렌치 깊이 %.1f' % REACH, (8, 31), fs=12, above=True)
ax.add_patch(Rectangle((121, -51), 76, 102, fill=False, ec='#1f4e9c', lw=1.6, ls='--', zorder=12))
ax.text(121, -52.5, '(b) 확대 범위', color='#1f4e9c', fontsize=12, va='top', zorder=12)
ax.set_xlim(108, FW + 6); ax.set_ylim(-118, 84)

# ============================================================================= (b) detail
ax = fig.add_axes([0.01, 0.01, 0.62, 0.555]); ax.set_aspect('equal'); ax.axis('off')
draw_parts(ax, detail=True)
load_path(ax, 4.0, [(194, -48), (192, -30), (181, -20.5), (FLOOR - 1.5, -10.5), (OUT + 1.2, -12.2), (OUT - 3, -9.5),
                    (ym(4), -10.5), (ym(4), -17.5), (RING - 1.5, -19.2), (CBF - 1.2, -20.3), (CBF - 0.5, -24.5),
                    (Y_WEB - 2.0, -30), (Y_WEB - 2.5, -45.5)])
ax.text(196, -52.5, '↑ 허벅지 보에서', color='red', fontsize=11, ha='center', va='center', zorder=9)
ax.text(Y_WEB - 2.5, -52.5, '웹 → 척추 봉·레일·데크 (몸통)', color='red', fontsize=11, ha='center', va='center', zorder=9,
        bbox=dict(fc='white', ec='none', alpha=0.85, pad=0.4))
ax.plot([FOUT - 3, 199], [0, 0], 'k-.', lw=0.8, zorder=8)
ax.text(199, 0.6, '회전축', fontsize=10, ha='right', va='bottom', zorder=8)
ax.set_title('(b) 확대 — 위쪽 절반: 출력 M3 나사 자리 단면,  아래쪽 절반: 평행핀 자리 단면,  빨간 선: 힘이 지나가는 길\n'
             '      (고정링 M2.5 · 컵 기둥 M3 · 보드 M2.5는 각도가 달라도 제 반지름에 겹쳐 그림)', fontsize=16, loc='left')
T = dict(fontsize=12, ha='center', va='center', zorder=9)
ax.text(152, 24.5, 'AKE60-8 고정자', **T)
ax.text(152, 8.2, 'AKE 출력부 (회전)', **T)
ax.text(151.5, 32.85, '로터 캔 (회전)', fontsize=9.5, ha='center', va='center', zorder=9, color='white')
ax.text(186.5, 23.5, '허벅지 머리\n(PETG)', **T)
ax.text(186.5, -40, '허벅지 보 (PETG)\n↓ 무릎', **T)
ax.text(Y_WEB - 2.5, 47.0, '6061\nhip 판', fontsize=11, ha='center', va='center', zorder=9,
        bbox=dict(fc='white', ec='none', alpha=0.85, pad=0.4))
L = 97.0                                          # left label column (inboard of the cup floor)
label(ax, 'AS5047P (엔코더)', (yb(-0.7), 1.0), (L, 24.0), fs=11)
label(ax, 'moteus r4.11 (1.65 t)', (yb(PCB / 2), 14.0), (L, 17.5), fs=11)
label(ax, '자석 Ø6 × 2.5\n→ 기판 %.2f' % P.MAGNET_PCB, (SP1 + 1.0, -2.0), (L, -8.0), fs=11)
label(ax, '자석 스페이서\nØ9 × %.2f (6061)' % P.SPACER_H, (139.5, -4.2), (L, -19.0), fs=11)
label(ax, '기둥 사이 컵 벽 Ø%.0f\n↔ Ø80 보스: 틈 %.1f' % (2 * LG.CUP_R_IN, LG.CUP_R_IN - BOSS_R), (RIM - 0.6, -41.2),
      (L, -31.0), fs=11)
label(ax, '드라이버 컵 (PETG 30 %)', (FOUT + 1.0, -36.0), (L, -45.0), fs=11)
R = 176.5                                         # upper right label column (above the thigh head)
label(ax, 'Ø80 × 1.5 등록 보스 (D)', (BOSS_F + 0.7, 38.3), (R, 62.5), fs=11)
label(ax, 'Ø%.0f × %.1f 홈: 고정자 앞면 틈 %.1f' % (P.HIP_STATOR_RELIEF[0], P.HIP_STATOR_RELIEF[1], REL - ym(A['stator_front'])),
      (REL - 0.25, 29.5), (R, 58.5), fs=11)
label(ax, 'Ø42 H7 포켓 (모델 Ø%.4f)' % P.PILOT_D_HIP, (RING - 1.2, 21.0), (R, 54.5), fs=11)
label(ax, '카운터보어 Ø%.1f × %.1f (Ø30 쪽으로 터짐)' % (CB_D, CB_T), (YW - 0.5, HOLE_R + 0.45), (R, 50.5), fs=11)
label(ax, 'SUS304 출력 링 %.0f t (Ø%.0f / Ø%.1f)' % (P.RING_T, RO_OD, RO_ID), (FLOOR + 1.0, 12.8), (R, 46.5), fs=11)
label(ax, 'Ø%.0f 보스 / Ø30 판 구멍: 반경 틈 %.1f' % (P.HUB_BOSS_D, HOLE_R - BOSS28), (170.6, 14.5), (R, 42.5), fs=11)
label(ax, 'Ø28 접근 구멍 → 허벅지 바깥 면 (y %.0f)' % FW, (194.5, 13.8), (R, 38.5), fs=11, rp=(0.48, 0))
label(ax, '핀 · M3 × %d: PCD %.0f' % (M3_L, P.AKE_PCD_OUT), (PIN0 + 0.5, -RQ - 1.5), (199.5, -21.0), fs=11)
label(ax, 'M2.5 × %d: PCD %.1f' % (S25_L, P.AKE_PCD_RING), (CBF + 1.0, -RP - 2.3), (199.5, -26.0), fs=11)
label(ax, 'M3 × %d: r %.1f (%.0f° 간격 3곳)' % (CUP_L, COL_R, COL_STEP), (138.0, COL_R + 1.4), (124.5, 51.5), fs=11, rp=(0.6, 0))

# screws / pins
tag(ax, '1', (178.5, -15.0), to=(CBF + 1.2, -RP - 1.8))
tag(ax, '2', (185.0, 8.0), to=(HB + 1.5, RQ - 1.0))
tag(ax, '3', (177.5, -5.0), c='#c0392b', to=(PIN1 - 1.0, -RQ + 0.8))
tag(ax, '4', (128.5, 21.5), to=(yb(3.5), RB[1] - 0.6))
tag(ax, '5', (146.0, 47.5), to=(146.0, COL_R + 1.4))
tag(ax, 'A', (162.0, -3.0), c='#2e7d32', to=(OUT - 0.7, -5.0))
tag(ax, 'B', (163.6, 9.2), c='#2e7d32', to=(168.6, 14.5))
tag(ax, 'C', (158.5, -26.5), c='#2e7d32', to=(RING - 1.0, -20.8))
tag(ax, 'D', (157.5, 47.5), c='#2e7d32', to=(BOSS_F + 0.5, 39.6))
ax.add_patch(Rectangle((157.0, 35.2), 8.0, 10.3, fill=False, ec='#2e7d32', lw=1.2, ls='--', zorder=12))

# axial dimensions: below the parts, and the plate stack on top
dim_h(ax, FOUT, RIM, -61.0, '컵 기둥 %.2f' % (RIM - FOUT), (-44, -44), fs=11)
dim_h(ax, FOUT, CS_HB, -56.0, '%.2f' % CUP_CB, (-44, None), fs=11)
dim_h(ax, BOSS_F, YW, -61.0, 'hip 판 %.1f' % (YW - BOSS_F), (-50, -26), fs=11, dx=1.2)
dim_h(ax, BOSS_F, Y_IN, -56.0, '%.1f' % BOSS_H, (-41, -50), fs=10.5)
dim_h(ax, Y_IN, Y_WEB, -56.0, '웹 %.0f' % P.HIP_WEB_T, (None, -50), fs=10.5)
dim_h(ax, YW, FI, -61.0, '%.0f' % P.GAP_FEMUR, (None, -50), fs=11)
dim_h(ax, OUT, FLOOR, -66.0, '보스 바닥 %.1f' % (FLOOR - OUT), (-14, -14), fs=11)
dim_h(ax, FLOOR, SEAT, -61.0, '링 %.0f' % P.RING_T, (None, -14), fs=10.5)
dim_h(ax, OUT, SEAT, -71.0, '와셔 자리 %.1f' % P.BEARING_FACE, (-14, -14), fs=11)
dim_h(ax, M3_TIP, OUT, -76.0, '%.1f' % ENG3, (-11.5, -14), fs=11, c='#b03020')
ax.text(M3_TIP - 0.8, -76.6, '② M3 × %d 물림 →' % M3_L, fontsize=11, ha='right', va='top', color='#b03020')
dim_h(ax, S25_TIP, RING, -71.0, '%.1f' % ENG25, (-RP - 1.25, -RP - 1.25), fs=11, c='#b03020')
ax.text(S25_TIP - 0.8, -71.6, '① M2.5 × %d 물림 →' % S25_L, fontsize=11, ha='right', va='top', color='#b03020')
dim_h(ax, BOSS_F, RING, 59.5, '포켓 %.1f' % (RING - BOSS_F), (50.5, 50.5), fs=10.5, above=True)
dim_h(ax, RING, CBF, 54.0, '%.1f' % (CBF - RING), (50.5, 50.5), fs=10.5, above=True)
dim_h(ax, CBF, YW, 54.0, '%.1f' % CB_T, (None, 50.5), fs=10.5, above=True)
dim_h(ax, Y_IN, CS_TIP, 64.5, '⑤ 물림 %.1f' % CS_ENG, (50.5, 50.5), fs=10.5, above=True, c='#b03020')
ax.set_xlim(96, 219); ax.set_ylim(-81, 69)

# D: register detail, x 3 (a radial plane 2.6 mm beside a column's screw hole: the column, not the hole, is cut)
ia = ax.inset_axes([97.0, 28.0, 24.0, 31.0], transform=ax.transData, zorder=13)
ia.set_aspect('equal'); ia.set_xticks([]); ia.set_yticks([])
for sp in ia.spines.values():
    sp.set_edgecolor('#2e7d32'); sp.set_linewidth(1.4)
e = COL_HOLE + 0.9
dth = math.asin(e / COL_R)
CH0 = COL_R * math.cos(dth) - math.sqrt(COL_W ** 2 - (COL_R * math.sin(dth)) ** 2)     # 38.22: column edge in that plane
ia.add_patch(Polygon([(BOSS_F, 30), (BOSS_F, BOSS_R), (Y_IN, BOSS_R), (Y_IN, 48), (172, 48), (172, 30)], fc=g['plate'],
                     ec='k', lw=1.0, **HATCH))
ia.add_patch(Polygon([(150, CH0), (BOSS_F, CH0), (BOSS_F, REG_R), (RIM, REG_R), (RIM, LG.CUP_R_OUT), (150, LG.CUP_R_OUT)],
                     fc=g['cup'], ec='k', lw=1.0))
ia.plot([BOSS_F, Y_IN], [BOSS_R, BOSS_R], color='#2e7d32', lw=4, solid_capstyle='butt', zorder=5)
ia.plot([Y_IN, Y_IN], [REG_R, LG.CUP_R_OUT], color='#1f4e9c', lw=4, solid_capstyle='butt', zorder=5)
ia.annotate('기둥 안쪽 Ø%.2f\n↔ 보스 Ø80 (틈 %.2f)' % (2 * REG_R, REG_R - BOSS_R), xy=(BOSS_F + 0.6, BOSS_R - 0.05),
            xytext=(158.5, 36.9), fontsize=10, color='#2e7d32', va='center', ha='left', zorder=8,
            bbox=dict(fc='white', ec='none', alpha=0.85, pad=0.4),
            arrowprops=dict(arrowstyle='->', color='#2e7d32', lw=0.9))
ia.annotate('컵 테두리가 웹 면에 앉음', xy=(Y_IN - 0.1, 43.0), xytext=(158.5, 44.5), fontsize=10, color='#1f4e9c',
            va='center', ha='left', zorder=8, arrowprops=dict(arrowstyle='->', color='#1f4e9c', lw=0.9))
ia.text(160.0, 41.8, 'PETG 컵 기둥', fontsize=10, ha='center', va='center')
ia.text(165.1, 41.0, 'hip 판 웹', fontsize=10, ha='center', va='center', rotation=90,
        bbox=dict(fc='white', ec='none', alpha=0.85, pad=0.3))
ia.text(163.25, 37.6, '보스\n1.5', fontsize=9.5, ha='center', va='center', bbox=dict(fc='white', ec='none', alpha=0.85, pad=0.3))
ia.set_xlim(158.26, 166.0); ia.set_ylim(35.5, 45.5)
ax.text(97.0, 60.0, 'D 확대 ×3 — 컵 기둥 옆면 단면\n(나사 구멍에서 %.1f 옆, 등록면이 보이는 자리)' % e, fontsize=11,
        va='bottom', color='#2e7d32', weight='bold')

# ============================================================================= text columns
tx = fig.add_axes([0.40, 0.585, 0.59, 0.375]); tx.axis('off'); tx.set_xlim(0, 1); tx.set_ylim(0, 1)
rows = [
    ('1', '#1f4e9c', 'M2.5 × %d  ISO 4762 12.9   × 6' % S25_L,
     'hip 판(6061) → AKE 고정자 링 (PCD %.1f).  허브 면의 카운터보어 Ø%.1f × %.1f(Ø30 구멍 쪽으로 터짐) 안, 머리 %.1f t는\n'
     '면보다 %.1f 들어감.  금속 판이라 클램프 링 없음.  고정자 탭(깊이 5)에 %.1f mm 물림 (8.8은 검토 하중에서 보증 초과)'
     % (P.AKE_PCD_RING, CB_D, CB_T, S25_K, CB_T - S25_K, ENG25)),
    ('2', '#1f4e9c', 'M3 × %d  ISO 4762 12.9   × 6   + 평와셔 DIN 125 M3 (Ø7 × 0.5)   × 6   + SUS304 출력 링   × 1' % M3_L,
     '허벅지 구동 보스 → 모터 출력부 (PCD %.0f).  보스 바닥(출력면에서 %.1f)에 링 Ø%.0f / Ø%.1f × %.0f t, 와셔 자리 %.1f\n'
     '→ 출력부 탭(깊이 5)에 %.1f mm 물림.  머리는 Ø28 접근 구멍 안, 허벅지 바깥 면에서 %.1f → 볼 엔드 2.5 mm 렌치(90 mm 이상)'
     % (P.AKE_PCD_OUT, FLOOR - OUT, RO_OD, RO_ID, P.RING_T, P.BEARING_FACE, ENG3, REACH)),
    ('3', '#c0392b', '평행핀 ISO 8734 Ø3 × 10  m6 경화   × 3',
     '출력부 구멍 3 mm + 구동 보스 7 mm (PCD %.0f, M3와 30° 어긋남 = ①과 같은 각도).\n'
     '허벅지와 출력부의 각도를 맞추고 관절 토크를 나사와 함께 받음' % P.AKE_PCD_OUT),
    ('4', '#1f4e9c', 'M2.5 × %d  저두 볼트 DIN 7984   × 4   + 황동 열압입 인서트 M2.5 × %.0f   × 4' % (BS_L, INS_L),
     'moteus → 컵 스탠드오프 (보드 구멍 (±20, -15) · (±20, 23) = r %.1f / %.1f).  보드의 모터 쪽 면에서 조임,\n'
     '인서트 구멍 Ø%.1f × 5, 인서트 %.0f mm에 다 물림 (끝은 구멍 바닥에서 %.2f)'
     % (RB[0], RB[1], LG.INSERT_M25, INS_L, PCB + 5.0 - BS_L)),
    ('5', '#1f4e9c', 'M3 × %d  ISO 4762   × 3' % CUP_L,
     '드라이버 컵 → hip 판 M3 탭 (r %.1f, 보드 커넥터 반대쪽 %.0f° 간격).  컵 바닥 카운터보어 Ø6 × %.2f,\n'
     '머리는 바닥 면보다 %.2f 들어감.  웹 %.0f mm에 %.1f mm 물림, 끝은 웹 바깥 면에서 %.1f'
     % (COL_R, COL_STEP, CUP_CB, CUP_CB - M3_K, P.HIP_WEB_T, CS_ENG, Y_WEB - CS_TIP)),
]
tx.text(0.0, 0.985, '나사·핀  (hip 한쪽 기준 — 로봇 전체는 × 2)', fontsize=18, weight='bold', va='top')
y = 0.90
for n, c, head, body in rows:
    tx.text(0.012, y, n, fontsize=15, weight='bold', color='white', ha='center', va='center',
            bbox=dict(boxstyle='circle,pad=0.22', fc=c, ec='none'))
    tx.text(0.04, y, head, fontsize=14.5, weight='bold', va='center')
    tx.text(0.04, y - 0.03, body, fontsize=13, va='top', linespacing=1.4)
    y -= 0.14
tx.text(0.0, y + 0.02, '조립 순서  (manufacturing/explain/assembly/hip)', fontsize=16, weight='bold', va='top')
tx.text(0.0, y - 0.025,
        '1) 몸통 안쪽에서 모터를 hip 판 Ø42 포켓에 넣고(C), 바깥 면에서 ① M2.5 × %d 6개를 조임\n' % S25_L +
        '2) 작업대에서 컵 스탠드오프에 인서트 4개 → moteus를 얹고 ④ M2.5 × %d 4개\n' % BS_L +
        '3) 컵을 Ø80 보스(D)에 씌우고 ⑤ M3 × %d 3개를 컵 바닥에서 조임\n' % CUP_L +
        '4) ③ 핀 3개를 출력부에 꽂고, 허벅지 Ø28 보스를 Ø30 구멍으로 넣어 스피곳(A)을 Ø12 홈에 맞춤\n'
        '5) 허벅지 바깥 Ø28 구멍 안에 SUS 출력 링 + 와셔, ② M3 × %d 6개 조임 (실제로는 몸통·셸 조립 뒤 다리를 닮)' % M3_L,
        fontsize=13, va='top', linespacing=1.5)

tb = fig.add_axes([0.64, 0.01, 0.35, 0.555]); tb.axis('off'); tb.set_xlim(0, 1); tb.set_ylim(0, 1)
tb.text(0.0, 0.97, '중심 맞춤 (끼워맞춤) · 통과 틈', fontsize=16, weight='bold', va='top')
y = 0.912
for n, txt in (('A', 'Ø11.8 스피곳 (허벅지 보스, 길이 1.4) → 출력부 Ø12 × 1.5 홈'),
               ('B', 'Ø28 보스는 hip 판 Ø30 구멍을 반경 틈 %.1f mm로 지나감 (닿지 않음)' % (HOLE_R - BOSS28)),
               ('C', 'AKE Ø42 파일럿 → Ø42 H7 포켓, 보스 면에서 깊이 %.1f (홈 면에서 %.1f).\n'
                     '고정자 앞면은 Ø63 홈 바닥에서 %.1f 띄움 → 파일럿 끝면만 포켓 바닥에 앉음'
                % (RING - BOSS_F, RING - REL, REL - ym(A['stator_front']))),
               ('D', 'Ø80 × 1.5 등록 보스 → 컵 기둥 3개 안쪽 Ø%.2f (엔코더를 모터 축에)' % (2 * REG_R))):
    tb.text(0.025, y, n, fontsize=14, weight='bold', color='white', ha='center', va='center',
            bbox=dict(boxstyle='circle,pad=0.22', fc='#2e7d32', ec='none'))
    tb.text(0.07, y + 0.013, txt, fontsize=12.5, va='top', linespacing=1.4)
    y -= 0.06 if '\n' in txt else 0.042
y -= 0.01
tb.text(0.0, y, '힘이 지나가는 길 (빨간 선, 다리 → 몸통)', fontsize=16, weight='bold', va='top')
steps = ['바퀴 → 정강이 → 무릎 → 허벅지 보', '허벅지 머리 → Ø28 구동 보스 (바닥 %.1f mm)' % (FLOOR - OUT),
         'A 스피곳 + ③ 핀 × 3 + ② M3 × %d × 6' % M3_L, 'AKE 출력부 → 모터 안 출력 베어링 → 고정자',
         'C Ø42 파일럿 + ① M2.5 × %d × 6' % S25_L, '6061 hip 판 허브 → 웹 %.0f mm' % P.HIP_WEB_T,
         '척추 봉 Ø16 (M5) · 위 레일 (M4) · 데크 → 몸통']
y -= 0.045
for k, st in enumerate(steps):
    tb.text(0.03, y, ('↓ ' if k else '   ') + st, fontsize=12.5, va='top')
    y -= 0.026
y -= 0.012
tb.text(0.0, y, '하중  (calc/strength.md 2절 · 4절, 검토 = SF 2)', fontsize=16, weight='bold', va='top')
tb.text(0.0, y - 0.04,
        '하중: 바퀴당 정지 60 N, 설계 300 N, 검토 600 N, 옆 150 / 300 N, 모터 12.5 → 검토 25 N·m\n'
        '• 출력 → 구동 보스, 토크 25 N·m: ②③ 개당 278 N, PETG 구멍 지압 10.1 MPa → 통과\n'
        '• 옆 하중 설계 Mx 63 N·m: ② 인장 2110 N (보증 4879), Ø28 보스 굽힘 32 MPa (45),\n'
        '   링 아래 지압 32 MPa (50) → 통과\n'
        '• 옆 하중 검토 Mx 127 N·m: ② 4220 N, 굽힘 65 MPa (45), 지압 64 MPa (50) → 초과\n'
        '   (보스는 Ø30 구멍과 ① 머리에 묶여 Ø28이 최대 — 설계 하중에서 SF 1.4)\n'
        '• ① 검토 Mx 127 N·m: M2.5 12.9 인장 2312 N (보증 3288) → 통과\n'
        '   (AKE 링 나사산 전단 한계 약 2.5~3 kN → 끝까지 물릴 것)\n'
        '• hip 판 웹 5 mm: σ 약 138 MPa(설계) / 276(검토) → 설계 SF 2.0, FEA 권장\n'
        '• AKE60 출력 베어링 허용 모멘트 비공개 → CubeMars 확인',
        fontsize=12.5, va='top', linespacing=1.45)
y = 0.07
tb.text(0.0, y + 0.02, '색', fontsize=14, weight='bold', va='bottom')
for k, (c, txt) in enumerate(((g['plate'], '6061 hip 판'), (g['thigh'], 'PETG 허벅지'), (g['cup'], 'PETG 컵'),
                              (g['stator'], 'AKE 고정자'), (g['out'], 'AKE 출력부'), (g['rotor'], 'AKE 로터'),
                              (g['ring'], 'SUS 링'), (g['washer'], '와셔'), (g['pin'], '평행핀'),
                              (g['insert'], '황동 인서트'), (g['pcb'], 'moteus'), (g['spacer'], '자석 스페이서'))):
    cx, cy = (k % 4) * 0.25, y - (k // 4) * 0.022
    kw = HATCH if c == g['plate'] else {}
    tb.add_patch(Rectangle((cx, cy - 0.008), 0.05, 0.016, fc=c, ec='k', lw=0.6, **kw))
    tb.text(cx + 0.065, cy, txt, fontsize=11.5, va='center')

fig.savefig(sys.argv[1] if len(sys.argv) > 1 else HERE / 'hip_joint_stack.png', dpi=80)
