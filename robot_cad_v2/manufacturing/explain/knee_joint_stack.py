"""Left knee joint: thigh knee plate (PETG) -> AKE60-8 -> shank driven boss, plus the moteus driver cup on the thigh's
housing end -- the parts and screws that carry the shank on the thigh, half sections to scale from fusion/params.py and
the cup / hole levels of fusion/legs.py (robot y of the LEFT leg on the horizontal axis, radius from the knee axis
vertically; upper half through an output M3 screw, towards the hip along the thigh beam, lower half through a dowel pin,
which shares its angle with a stator screw, towards the wheel along the shank beam).  Overview + enlarged detail, same
style as wheel_output_stack.py.  Explanatory figure, not a manufacturing drawing.

    python knee_joint_stack.py        # -> knee_joint_stack.png next to this script
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
A = P.AKE

# ----------------------------------------------------------------------------- stations (robot y, left leg)
AX = P.AXIS_KNEE                                 # -1: the knee motor's back points inboard (-y)


def ym(local):                                   # AKE local y -> robot y
    return P.Y_RING_KNEE + AX * (local - A['ring'])


OUT = P.y_output('knee')                         # 198.0 output face (faces the shank, +y)
RING = P.Y_RING_KNEE                             # 197.5 pilot end face = pocket floor
FI, FO, FW = P.Y_FEMUR_IN, P.Y_FEMUR_OUT, P.FEMUR_WIDE_OUT     # thigh 174 .. 204 (widened to 250 away from the knee)
KP0, KP1 = FO - P.KNEE_PLATE_T, FO               # knee plate 195 .. 204 (motor side .. far face)
POCK_R = 21.025                                  # legs._motor_seat: printed Ø42.05 pilot pocket (not in params)
HOLE_R = P.RING_STATOR[1] / 2                    # Ø30 plate hole (= stator ring ID, params)
REC0 = KP1 - (P.RING_T + P.CBORE_M25[1])         # 200.3 recess floor (ring on it)
REC_R = P.STATOR_RECESS_D / 2
RS_OD, RS_ID = P.RING_STATOR
S25_L, S25_K, S25_DK = 8, 1.5, 4.5               # DIN 7984 M2.5 x 8 low head (k 1.5, dk 4.5)
S25_HB = REC0 + P.RING_T                         # 202.3 head bearing face (on the ring)
S25_TIP = S25_HB - S25_L                         # 194.3
ENG25 = RING - S25_TIP                           # 3.2 in the 5 mm stator ring taps
WEB = REC0 - RING                                # 2.8 PETG web between pocket floor and recess
TI, TO, TW = P.Y_TIBIA_IN, P.Y_TIBIA_OUT, P.TIBIA_WIDE_OUT     # shank 205.5 .. 235.5, widened outboard to 260.5
BOSS28 = P.HUB_BOSS_D / 2
SPIG_R, SPIG_L = 5.9, 1.4                        # legs._driven_boss: Ø11.8 x 1.4 spigot
FLOOR = OUT + P.BEARING_FACE - P.RING_T          # 207.2 boss floor (output ring)
SEAT = OUT + P.BEARING_FACE                      # 209.2 washer seat
W_T, W_OD = 0.5, 7.0                             # DIN 125 M3
HB = SEAT + W_T                                  # 209.7 M3 head bearing face
M3_L, M3_K, M3_DK = 16, 3.0, 5.5                 # ISO 4762 M3 x 16
M3_TIP = HB - M3_L                               # 193.7
ENG3 = OUT - M3_TIP                              # 4.3 in the 5 mm output taps
REACH = TW - (HB + M3_K)                         # 47.8 shank outer face -> M3 head top (wrench reach)
PIN0, PIN1 = OUT - 3.0, OUT + 7.0
RP, RQ = P.AKE_PCD_RING / 2, P.AKE_PCD_OUT / 2
RO_OD, RO_ID = P.RING_OUT
# driver cup + moteus (board frame y: + away from the motor = robot -y here)
OB = ym(P.board_offset())                        # 165.1 PCB motor-side face


def yb(v):                                       # board y -> robot y
    return OB + AX * v


RIM_B, FOUT_B = LG.cup_levels('knee', 1)
RIM, FOUT, FIN = yb(RIM_B), yb(FOUT_B), yb(FOUT_B - P.CUP_WALL)   # 174.0, 154.55, 156.95
CUP_L, CUP_CB = LG.cup_screw('knee')             # M2.5 x 18, counterbore 5.45
CS_HB = yb(FOUT_B - CUP_CB)                      # 160.0 column screw head bearing face
CS_TIP = CS_HB - AX * CUP_L                      # 178.0
CS_K = 2.5                                       # ISO 4762 M2.5 head
COL_R = math.hypot(*(lambda p: (p[0] - P.KNEE_X, p[2] - P.KNEE_Z))(LG.cup_column_points('knee', 1)[0]))   # 40.5
_ang = sorted(((t + 180.0) % 360.0) - 180.0 for t in LG.CUP_COLUMNS)
COL_STEP = _ang[1] - _ang[0]                     # 60: the 3 columns sit 60 deg apart, away from the connector edge
COL_W, COL_HOLE, COL_CB = 3.4, LG.M25_CLEAR / 2, 3.0      # legs.cup: column r, M2.5 hole r, counterbore r
INS_HOLE = 5.0                                   # legs._cup_inserts depth
LI0, LI1 = FI + 0.1, FI + 0.1 + 4.0              # link insert M2.5 x 4 (fasteners.py: 0.1 below the housing face)
CS_ENG = CS_TIP - LI0                            # 3.9 in the insert (4.0 from the housing face)
LIP_R = P.HOUSING_R + 2.45                       # legs.cup: lip OD r 46.45
REG_R = P.HOUSING_R + 0.05                       # lip ID r 44.05 over the housing OD r 44
LIP_L = 3.0                                      # lip length over the housing
BRIDGE = RIM + AX * LG.CUP_LIP_BRIDGE            # 172.0 bridge ring start (lip joined to the wall)
RB = sorted({round(math.hypot(x, z), 2) for x, z in P.MOTEUS['holes']})   # board screw radii 25.0, 30.48
PCB = P.MOTEUS['pcb_t']
INS0, INS_L = PCB + 0.1, 4.0                     # standoff insert M2.5 x 4
BS_L = 6                                         # DIN 7984 M2.5 x 6 board screws
MAG = ym(A['magnet'])                            # 173.0 rotor boss end (empty magnet seat)
SP1 = ym(A['magnet'] + P.SPACER_H)               # 167.65 spacer top = magnet top
# thigh widening (params / legs._wide): full to FW from WIDE_END + WIDE_TAPER before the knee, ramping back over WIDE_TAPER
W_FULL, W_END = P.FEMUR_WIDE_END + P.WIDE_TAPER, P.FEMUR_WIDE_END      # 70, 55 from the knee

g = dict(thigh='#c7b6e6', shank='#9fd29f', cup='#f5d0d8', stator='#b8b8b8', rotor='#7f9fbf', out='#d9b38c',
         pcb='#2f7d62', spacer='#dcdcdc', magnet='#8e44ad', screw='#2e2e2e', washer='#e3c034', pin='#c0392b',
         ring='#6f86a6', insert='#b8860b')


def zig(p0, p1, n=4, a=0.8):
    """Break line points strictly between p0 and p1 (zigzag of amplitude a)."""
    (x0, y0), (x1, y1) = p0, p1
    L = math.hypot(x1 - x0, y1 - y0)
    nx, ny = -(y1 - y0) / L, (x1 - x0) / L
    return [(x0 + (x1 - x0) * k / (2 * n) + nx * a * (1 if k % 2 else -1),
             y0 + (y1 - y0) * k / (2 * n) + ny * a * (1 if k % 2 else -1)) for k in range(1, 2 * n)]


def draw_parts(ax, detail):
    a = 0.7 if detail else 1.4
    rc = 50.0 if detail else 88.0                # beam break radius: thigh (upper, to the hip), shank (lower, to the wheel)
    yend = 226.0 if detail else TW               # shank drawn up to here

    def poly(pts, c, s=1, z=1, **kw):
        ax.add_patch(Polygon([(x, s * r) for x, r in pts], fc=c, ec=kw.pop('ec', 'k'), lw=kw.pop('lw', 0.9),
                             zorder=z, **kw))

    def rect(x0, x1, r0, r1, c, s=1, z=1, **kw):
        poly([(x0, r0), (x1, r0), (x1, r1), (x0, r1)], c, s, z, **kw)

    for s in (1, -1):
        # thigh (PETG): housing r 44 around the can cavity r 37 (open at the cup end), knee plate 9 with the Ø42.05
        # pocket 2.5, Ø30 hole, Ø45.6 x 3.7 recess; the beam goes on up to the hip (upper half)
        top = rc if s > 0 else P.HOUSING_R
        pts = [(RING, HOLE_R), (REC0, HOLE_R), (REC0, REC_R), (KP1, REC_R), (KP1, top)]
        if s > 0:
            pts += zig((KP1, top), (FI, top), 5 if detail else 8, a)
        pts += [(FI, top), (FI, P.CAN_CLEAR_R), (KP0, P.CAN_CLEAR_R), (KP0, POCK_R), (RING, POCK_R)]
        poly(pts, g['thigh'], s)
        if s > 0 and not detail:                 # widened part of the thigh beam (outboard, from 70 / 55 before the knee)
            pts = [(FO - 0.5, W_END), (FW, W_FULL), (FW, rc)] + zig((FW, rc), (FO - 0.5, rc), 7, a) + [(FO - 0.5, rc)]
            poly(pts, g['thigh'], s)
        # shank (PETG): Ø28 boss + Ø11.8 spigot, head r 30 (upper), beam to the wheel (lower), Ø28 access pocket
        top = P.HEAD_R if s > 0 else rc
        pts = [(OUT - SPIG_L, 0), (OUT - SPIG_L, SPIG_R), (OUT, SPIG_R), (OUT, BOSS28), (TI, BOSS28), (TI, top)]
        if s < 0:
            pts += zig((TI, top), (yend, top), 4 if detail else 9, a)
        pts += [(yend, top)]
        if yend < TW:
            pts += zig((yend, top), (yend, BOSS28), 3, a)
        pts += [(yend, BOSS28), (FLOOR, BOSS28), (FLOOR, 0)]
        poly(pts, g['shank'], s)
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
        # driver cup (PETG): floor + wall r 41 .. 43.4, 2 mm bridge ring to the lip; upper half through a screw column
        # (beam side: the lip is cut away where the thigh beam joins the housing), lower half: lip over the housing
        if s > 0:
            poly([(FOUT, 0), (FOUT, COL_R + COL_W), (BRIDGE, COL_R + COL_W), (BRIDGE, LIP_R), (RIM, LIP_R),
                  (RIM, COL_R - COL_W), (FIN, COL_R - COL_W), (FIN, 0)], g['cup'], s)
            rect(FOUT, CS_HB, COL_R - COL_CB, COL_R + COL_CB, 'white', s, z=1.2, lw=0.6)
            rect(CS_HB, RIM, COL_R - COL_HOLE, COL_R + COL_HOLE, 'white', s, z=1.2, lw=0.6)
            # brass insert in the thigh housing end (hole Ø3.6 x 5)
            rect(FI, FI + INS_HOLE, COL_R - LG.INSERT_M25 / 2, COL_R + LG.INSERT_M25 / 2, 'white', s, z=1.2, lw=0.5)
            rect(LI0, LI1, COL_R - LG.INSERT_M25 / 2, COL_R + LG.INSERT_M25 / 2, g['insert'], s, z=2.5, lw=0.7)
        else:
            poly([(FOUT, 0), (FOUT, LG.CUP_R_OUT), (BRIDGE, LG.CUP_R_OUT), (BRIDGE, LIP_R), (RIM - AX * LIP_L, LIP_R),
                  (RIM - AX * LIP_L, REG_R), (RIM, REG_R), (RIM, LG.CUP_R_IN), (FIN, LG.CUP_R_IN), (FIN, 0)], g['cup'], s)
        # cup standoff + brass insert + board screw (from the motor side of the PCB)
        rs = RB[1] if s > 0 else RB[0]
        rect(yb(FOUT_B - P.CUP_WALL + 0.5), yb(PCB), rs - 3.2, rs + 3.2, g['cup'], s, z=1.1)
        rect(yb(PCB + 5.0), yb(PCB), rs - LG.INSERT_M25 / 2, rs + LG.INSERT_M25 / 2, 'white', s, z=1.2, lw=0.5)
        rect(yb(INS0 + INS_L), yb(INS0), rs - LG.INSERT_M25 / 2, rs + LG.INSERT_M25 / 2, g['insert'], s, z=2.5, lw=0.7)
        rect(yb(BS_L), yb(0), rs - 1.25, rs + 1.25, g['screw'], s, z=3)
        rect(yb(0), yb(-1.5), rs - 2.25, rs + 2.25, g['screw'], s, z=3)
        # SUS304 stator ring in the recess + DIN 7984 M2.5 x 8 low heads (6 on PCD 36.5: both halves)
        rect(REC0, REC0 + P.RING_T, RS_ID / 2, RS_OD / 2, g['ring'], s, z=2, lw=1.0)
        rect(S25_TIP, S25_HB, RP - 1.25, RP + 1.25, g['screw'], s, z=3)
        rect(S25_HB, S25_HB + S25_K, RP - S25_DK / 2, RP + S25_DK / 2, g['screw'], s, z=3)
        # SUS304 output ring on the boss floor
        rect(FLOOR, FLOOR + P.RING_T, RO_ID / 2, RO_OD / 2, g['ring'], s, z=2, lw=1.0)
    # upper half: output M3 x 16 at r 10 (washer under the head), cup column M2.5 x 18 at r 40.5
    rect(SEAT, HB, RQ - W_OD / 2, RQ + W_OD / 2, g['washer'], z=3, lw=0.8)
    rect(HB, HB + M3_K, RQ - M3_DK / 2, RQ + M3_DK / 2, g['screw'], z=3)
    rect(M3_TIP, HB, RQ - 1.5, RQ + 1.5, g['screw'], z=3)
    rect(CS_HB - CS_K, CS_HB, COL_R - 2.25, COL_R + 2.25, g['screw'], z=3)
    rect(CS_HB, CS_TIP, COL_R - 1.25, COL_R + 1.25, g['screw'], z=3)
    # lower half: dowel Ø3 x 10 (3 mm in the output, 7 mm in the boss)
    rect(PIN0, PIN1, -RQ - 1.5, -RQ + 1.5, g['pin'], z=3, lw=1.0)


def load_path(ax, lw, path):
    ax.plot(*zip(*path[:-1]), color='red', lw=lw, alpha=0.7, zorder=6, solid_capstyle='round', solid_joinstyle='round')
    ax.annotate('', xy=path[-1], xytext=path[-2], zorder=6,
                arrowprops=dict(arrowstyle='-|>', color='red', alpha=0.7, lw=lw, mutation_scale=8 * lw))


# shank beam (lower) -> boss -> output -> through the motor -> stator (upper) -> pilot + web -> knee plate -> thigh beam
PATH = [(FLOOR - 1.5, -10.5), (OUT + 1.2, -12.2), (OUT - 3, -9.5), (188.5, -5), (188.5, 9), (ym(4), 17.5),
        (RING - 1.5, 19.2), (199.0, 21.6), (199.6, 27.0), (193.5, 36.0), (188.5, 43.0)]


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


def label(ax, txt, xy, xt, fs=11, c='k', ha='left', rp=None):
    if rp is None:                               # leader leaves the text end that faces its target
        rp = (0, 0.5) if xy[0] < xt[0] + 3 else (1, 0.5)
    ax.annotate(txt, xy=xy, xytext=xt, fontsize=fs, color=c, ha=ha, va='center', zorder=11,
                arrowprops=dict(arrowstyle='->', lw=0.8, color=c, shrinkA=2, shrinkB=0, relpos=rp),
                bbox=dict(fc='white', ec='none', alpha=0.85, pad=0.6))


fig = plt.figure(figsize=(24, 28))
fig.suptitle('무릎 관절 — 정강이를 허벅지에 매다는 연결  (왼쪽 다리 단면, 모델 치수 mm, 실제 비율)',
             fontsize=22, x=0.01, ha='left', y=0.995)

# ============================================================================= (a) overview
ax = fig.add_axes([0.01, 0.585, 0.37, 0.375]); ax.set_aspect('equal'); ax.axis('off')
draw_parts(ax, detail=False)
load_path(ax, 3.0, [(TW - 12, -84), (238, -42), (217, -21)] + PATH[:-1] + [(200, 46), (200, 88)])
ax.plot([FOUT - 4, TW + 4], [0, 0], 'k-.', lw=0.7, zorder=8)
ax.set_title('(a) 전체 — 드라이버 컵부터 정강이 바깥 면까지', fontsize=17, loc='left')
T = dict(fontsize=12, ha='center', va='center', zorder=9)
ax.text(FI + 11.5, 66, 'PETG 허벅지 보\n↑ hip까지\n(175)', **T)
ax.text((FO + FW) / 2 + 4, 80, '허벅지 보강부 (y %.0f → %.0f)' % (FO, FW), fontsize=11, ha='center', va='center', zorder=9)
ax.text((TI + TW) / 2, 22, '정강이 머리 (PETG)', **T)
ax.text((TI + TW) / 2 + 4, -66, 'PETG 정강이 보\n↓ 바퀴까지 (175)', **T)
ax.text(183, 23, 'AKE60-8', **T)
ax.text(FOUT + 1, -62, '드라이버 컵 + moteus', fontsize=11, ha='left', va='center', zorder=9)
ax.text(FOUT - 3, 93, '← 몸통 쪽 (안쪽)', fontsize=11, ha='left', va='center', zorder=9)
ax.text(TW + 5, 93, '바깥 (+y) →', fontsize=11, ha='right', va='center', zorder=9)
ax.text(200, 96, '↑ hip으로', color='red', fontsize=11, ha='center', va='center', zorder=9)
ax.text(TW - 12, -93, '바퀴에서 ↑', color='red', fontsize=11, ha='center', va='center', zorder=9)
label(ax, '보강부는 무릎에서 %.0f부터 경사, %.0f부터 전폭\n(정강이 머리 r %.0f과 겹치지 않음)' % (W_END, W_FULL, P.HEAD_R),
      (226.0, 62.0), (207.5, 47.5), fs=10.5, rp=(0.45, 1))
R1, R2 = -100.0, -110.0
dim_h(ax, FOUT, RIM, R1, '컵 %.2f' % (RIM - FOUT), (-44, -70), fs=12)
dim_h(ax, FI, FO, R2, '허벅지 %.0f' % (FO - FI), (-44, -44), fs=12)
dim_h(ax, TI, TO, R1, '30', (-88, -88), fs=12)
dim_h(ax, TO, TW, R1, '25 (보강)', (None, -88), fs=12)
dim_h(ax, TI, TW, R2, '정강이 폭 %.0f' % (TW - TI), (-88, -88), fs=12)
dim_h(ax, HB + M3_K, TW, 36.0, '② 렌치 깊이 %.1f' % REACH, (8, 31), fs=12, above=True)
ax.add_patch(Rectangle((152, -52), 76, 104, fill=False, ec='#1f4e9c', lw=1.6, ls='--', zorder=12))
ax.text(152, -53.5, '(b) 확대 범위', color='#1f4e9c', fontsize=12, va='top', zorder=12)
ax.set_xlim(144, TW + 6); ax.set_ylim(-118, 100)

# ============================================================================= (b) detail
ax = fig.add_axes([0.01, 0.01, 0.62, 0.555]); ax.set_aspect('equal'); ax.axis('off')
draw_parts(ax, detail=True)
load_path(ax, 4.0, [(224, -48), (221, -30), (212, -20.5)] + PATH + [(186.0, 48.5)])
ax.text(224, -52.5, '↑ 정강이 보에서', color='red', fontsize=11, ha='center', va='center', zorder=9)
ax.text(184.0, 54.0, '↑ 허벅지 보 → hip', color='red', fontsize=11, ha='center', va='center', zorder=9,
        bbox=dict(fc='white', ec='none', alpha=0.85, pad=0.4))
ax.plot([FOUT - 3, 229], [0, 0], 'k-.', lw=0.8, zorder=8)
ax.text(229, 0.6, '회전축', fontsize=10, ha='right', va='bottom', zorder=8)
ax.set_title('(b) 확대 — 위쪽 절반: 출력 M3 나사 자리 단면 (허벅지 보 쪽),  아래쪽 절반: 평행핀 자리 단면 (정강이 보 쪽),\n'
             '      빨간 선: 힘이 지나가는 길  (고정링 M2.5 · 컵 기둥 M2.5 · 보드 M2.5는 각도가 달라도 제 반지름에 겹쳐 그림)',
             fontsize=16, loc='left')
T = dict(fontsize=12, ha='center', va='center', zorder=9)
ax.text(183.5, 24.5, 'AKE60-8 고정자', **T)
ax.text(183.5, 8.2, 'AKE 출력부 (회전)', **T)
ax.text(182.5, 32.85, '로터 캔 (회전)', fontsize=9.5, ha='center', va='center', zorder=9, color='white')
ax.text(216.5, 23.5, '정강이 머리\n(PETG)', **T)
ax.text(216.5, -40, '정강이 보 (PETG)\n↓ 바퀴', **T)
ax.text(188.0, -40.5, '허벅지 하우징 (PETG)', **T)
ax.text(199.5, 44.5, '허벅지\n무릎 판', fontsize=11, ha='center', va='center', zorder=9,
        bbox=dict(fc='white', ec='none', alpha=0.85, pad=0.4))
L = 127.0                                         # left label column (inboard of the cup floor)
label(ax, 'AS5047P (엔코더)', (yb(-0.7), 1.0), (L, 24.0), fs=11)
label(ax, 'moteus r4.11 (1.65 t)', (yb(PCB / 2), 14.0), (L, 17.5), fs=11)
label(ax, '자석 Ø6 × 2.5\n→ 기판 %.2f' % P.MAGNET_PCB, (SP1 + 1.0, -2.0), (L, -8.0), fs=11)
label(ax, '자석 스페이서\nØ9 × %.2f (6061)' % P.SPACER_H, (SP1 + 3.0, -4.2), (L, -19.0), fs=11)
label(ax, '드라이버 컵 (PETG 30 %)', (FOUT + 1.0, -36.0), (L, -32.0), fs=11)
label(ax, '컵 립 Ø%.1f → 하우징 Ø%.0f (D)\n입구 %.0f mm 링으로 컵 벽과 한 덩어리' % (2 * REG_R, 2 * P.HOUSING_R, LG.CUP_LIP_BRIDGE),
      (RIM + 1.5, -REG_R - 1.2), (L, -48.0), fs=11)
label(ax, 'M2.5 × %d: r %.1f (%.0f° 간격 3곳)' % (CUP_L, COL_R, COL_STEP), (163.0, COL_R + 1.25), (L, 51.5), fs=11)
label(ax, '황동 인서트 M2.5 × 4 (구멍 Ø%.1f × %.0f)' % (LG.INSERT_M25, INS_HOLE), (LI0 + 2.0, COL_R + 1.4), (L, 57.5), fs=11)
R = 207.5                                         # upper right label column (above the shank head)
label(ax, 'SUS304 고정 링 %.0f t (Ø%.0f / Ø%.0f)' % (P.RING_T, RS_OD, RS_ID), (REC0 + 1.0, 21.6), (R, 60.0), fs=11)
label(ax, '홈 Ø%.1f × %.1f (링 + 저두 머리)' % (P.STATOR_RECESS_D, KP1 - REC0), (KP1 - 0.6, 21.9), (R, 56.0), fs=11)
label(ax, 'Ø42.05 포켓 깊이 %.1f (파일럿 %.1f 나옴)' % (RING - KP0, A['pilot_len'] - (RING - KP0)), (KP0 + 1.2, POCK_R),
      (R, 52.0), fs=11)
label(ax, '고정자 앞면 ↔ 판: 틈 %.1f' % (KP0 - ym(A['stator_front'])), ((KP0 + ym(A['stator_front'])) / 2, 26.0),
      (R, 48.0), fs=11)
label(ax, 'SUS304 출력 링 %.0f t (Ø%.0f / Ø%.1f)' % (P.RING_T, RO_OD, RO_ID), (FLOOR + 1.0, 12.8), (R, 44.0), fs=11)
label(ax, 'Ø%.0f 보스 / Ø30 판 구멍·링: 반경 틈 %.1f' % (P.HUB_BOSS_D, HOLE_R - BOSS28), (201.3, 14.5), (R, 40.0), fs=11)
label(ax, 'Ø28 접근 구멍 → 정강이 바깥 면 (y %.1f)' % TW, (224.5, 13.8), (R, 36.0), fs=11, rp=(0.42, 0))
label(ax, '캔 r %.1f ↔ 하우징 r %.0f: 틈 %.1f' % (A['can_r'], P.CAN_CLEAR_R, P.CAN_CLEAR_R - A['can_r']), (182.0, 35.8),
      (R, 32.0), fs=11)
label(ax, '핀 · M3 × %d: PCD %.0f' % (M3_L, P.AKE_PCD_OUT), (PIN0 + 0.5, -RQ - 1.5), (229.5, -21.0), fs=11)
label(ax, 'M2.5 × %d: PCD %.1f' % (S25_L, P.AKE_PCD_RING), (S25_HB + 0.8, -RP - 2.3), (229.5, -26.0), fs=11)

# screws / pins
tag(ax, '1', (209.5, -15.0), to=(S25_HB + 0.8, -RP - 1.8))
tag(ax, '2', (216.0, 8.0), to=(HB + 1.5, RQ - 1.0))
tag(ax, '3', (208.5, -5.0), c='#c0392b', to=(PIN1 - 1.0, -RQ + 0.8))
tag(ax, '4', (159.5, 21.5), to=(yb(3.5), RB[1] - 0.6))
tag(ax, '5', (170.0, 47.5), to=(168.0, COL_R + 1.2))
tag(ax, 'A', (193.0, -3.0), c='#2e7d32', to=(OUT - 0.7, -5.0))
tag(ax, 'B', (194.6, 9.2), c='#2e7d32', to=(199.6, 14.5))
tag(ax, 'C', (189.5, -26.5), c='#2e7d32', to=(RING - 1.0, -20.8))
tag(ax, 'D', (181.0, -52.0), c='#2e7d32', to=(RIM - AX * 1.5, -REG_R - 0.3))

# axial dimensions: below the parts, and the knee plate stack on top
dim_h(ax, FOUT, CS_HB, -56.0, '%.2f' % CUP_CB, (-44, None), fs=11)
dim_h(ax, FOUT, RIM, -61.0, '컵 기둥 %.2f' % (RIM - FOUT), (-44, -47), fs=11)
dim_h(ax, FI, KP0, -61.0, '하우징 %.0f' % (KP0 - FI), (-47, -44), fs=11)
dim_h(ax, ym(A['stator_front']), KP0, -56.0, '%.1f' % (KP0 - ym(A['stator_front'])), (-30.75, -44), fs=10.5)
dim_h(ax, KP0, KP1, -61.0, '판 %.0f' % P.KNEE_PLATE_T, (None, -44), fs=11)
dim_h(ax, KP1, TI, -61.0, '%.1f' % P.GAP_TIBIA, (None, -50), fs=11)
dim_h(ax, OUT, FLOOR, -66.0, '보스 바닥 %.1f' % (FLOOR - OUT), (-14, -14), fs=11)
dim_h(ax, FLOOR, SEAT, -56.0, '링 %.0f' % P.RING_T, (-14, -14), fs=10.5)
dim_h(ax, OUT, SEAT, -71.0, '와셔 자리 %.1f' % P.BEARING_FACE, (-14, -14), fs=11)
dim_h(ax, M3_TIP, OUT, -76.0, '%.1f' % ENG3, (-11.5, -14), fs=11, c='#b03020')
ax.text(M3_TIP - 0.8, -76.6, '② M3 × %d 물림 →' % M3_L, fontsize=11, ha='right', va='top', color='#b03020')
dim_h(ax, S25_TIP, RING, -71.0, '%.1f' % ENG25, (-RP - 1.25, -RP - 1.25), fs=11, c='#b03020')
ax.text(S25_TIP - 0.8, -71.6, '① M2.5 × %d 물림 →' % S25_L, fontsize=11, ha='right', va='top', color='#b03020')
dim_h(ax, KP0, RING, 64.0, '포켓 %.1f' % (RING - KP0), (50.5, 50.5), fs=10.5, above=True, dx=-3.6)
dim_h(ax, RING, REC0, 64.0, '웹 %.1f' % WEB, (None, 50.5), fs=10.5, above=True)
dim_h(ax, REC0, KP1, 64.0, '홈 %.1f' % (KP1 - REC0), (None, 50.5), fs=10.5, above=True, dx=3.8)
dim_h(ax, FI, CS_TIP, 64.0, '⑤ 물림 %.1f' % (CS_TIP - FI), (50.5, COL_R + 1.25), fs=10.5, above=True, c='#b03020')
ax.set_xlim(126, 249); ax.set_ylim(-81, 69)

# ============================================================================= text columns
tx = fig.add_axes([0.40, 0.585, 0.59, 0.375]); tx.axis('off'); tx.set_xlim(0, 1); tx.set_ylim(0, 1)
rows = [
    ('1', '#1f4e9c', 'M2.5 × %d  저두 볼트 DIN 7984 8.8   × 6   + SUS304 고정 링 Ø%.0f / Ø%.0f × %.0f t   × 1'
     % (S25_L, RS_OD, RS_ID, P.RING_T),
     '허벅지 무릎 판 → AKE 고정자 링 (PCD %.1f).  판 바깥 면 홈 Ø%.1f × %.1f 안의 링 위에서 조임, 머리 %.1f t는 면보다 %.1f 들어감.\n'
     '판 웹 %.1f mm를 링과 고정자 사이에 끼움 → 고정자 탭(깊이 5)에 %.1f mm 물림'
     % (P.AKE_PCD_RING, P.STATOR_RECESS_D, KP1 - REC0, S25_K, KP1 - S25_HB - S25_K, WEB, ENG25)),
    ('2', '#1f4e9c', 'M3 × %d  ISO 4762 12.9   × 6   + 평와셔 DIN 125 M3 (Ø7 × 0.5)   × 6   + SUS304 출력 링   × 1' % M3_L,
     '정강이 구동 보스 → 모터 출력부 (PCD %.0f).  보스 바닥(출력면에서 %.1f)에 링 Ø%.0f / Ø%.1f × %.0f t, 와셔 자리 %.1f\n'
     '→ 출력부 탭(깊이 5)에 %.1f mm 물림.  머리는 Ø28 접근 구멍 안, 정강이 바깥 면에서 %.1f'
     % (P.AKE_PCD_OUT, FLOOR - OUT, RO_OD, RO_ID, P.RING_T, P.BEARING_FACE, ENG3, REACH)),
    ('3', '#c0392b', '평행핀 ISO 8734 Ø3 × 10  m6 경화   × 3',
     '출력부 구멍 3 mm + 정강이 보스 7 mm (PCD %.0f, M3와 30° 어긋남 = ①과 같은 각도).\n'
     '정강이와 출력부의 각도를 맞추고 관절 토크를 나사와 함께 받음' % P.AKE_PCD_OUT),
    ('4', '#1f4e9c', 'M2.5 × %d  저두 볼트 DIN 7984   × 4   + 황동 열압입 인서트 M2.5 × %.0f   × 4' % (BS_L, INS_L),
     'moteus → 컵 스탠드오프 (보드 구멍 (±20, -15) · (±20, 23) = r %.1f / %.1f).  보드의 모터 쪽 면에서 조임,\n'
     '인서트 구멍 Ø%.1f × 5, 인서트 %.0f mm에 다 물림 (끝은 구멍 바닥에서 %.2f)'
     % (RB[0], RB[1], LG.INSERT_M25, INS_L, PCB + 5.0 - BS_L)),
    ('5', '#1f4e9c', 'M2.5 × %d  ISO 4762   × 3   + 황동 열압입 인서트 M2.5 × 4   × 3 (허벅지)' % CUP_L,
     '드라이버 컵 → 허벅지 하우징 끝면 인서트 (r %.1f, 보드 커넥터 반대쪽 %.0f° 간격, 구멍 Ø%.1f × %.0f).\n'
     '컵 바닥 카운터보어 Ø6 × %.2f (머리는 바닥 면보다 %.2f 들어감), 인서트에 %.1f mm 물림 (하우징 끝면부터 %.1f, 끝은 구멍 바닥에서 %.1f)'
     % (COL_R, COL_STEP, LG.INSERT_M25, INS_HOLE, CUP_CB, CUP_CB - CS_K, CS_ENG, CS_TIP - FI, FI + INS_HOLE - CS_TIP)),
]
tx.text(0.0, 0.985, '나사·핀  (무릎 한쪽 기준 — 로봇 전체는 × 2)', fontsize=18, weight='bold', va='top')
y = 0.90
for n, c, head, body in rows:
    tx.text(0.012, y, n, fontsize=15, weight='bold', color='white', ha='center', va='center',
            bbox=dict(boxstyle='circle,pad=0.22', fc=c, ec='none'))
    tx.text(0.04, y, head, fontsize=14.5, weight='bold', va='center')
    tx.text(0.04, y - 0.03, body, fontsize=13, va='top', linespacing=1.4)
    y -= 0.14
tx.text(0.0, y + 0.02, '조립 순서  (manufacturing/explain/assembly/knee)', fontsize=16, weight='bold', va='top')
tx.text(0.0, y - 0.025,
        '1) 허벅지 하우징 끝면에 인서트 3개 (열압입) → 허벅지 안쪽에서 모터를 넣어 Ø42 파일럿을 포켓(C)에 앉힘\n'
        '2) 무릎 판 바깥 면 홈에 SUS 고정 링을 놓고 ① M2.5 × %d 6개를 조임\n' % S25_L +
        '3) ③ 핀 3개를 출력부에 꽂고, 정강이 Ø28 보스를 Ø30 구멍으로 넣어 스피곳(A)을 Ø12 홈에 맞춤\n'
        '4) 정강이 바깥 Ø28 구멍 안에 SUS 출력 링 + 와셔, ② M3 × %d 6개 조임\n' % M3_L +
        '5) 작업대에서 컵 인서트 4개 → moteus → ④ M2.5 × %d 4개, 컵 립을 하우징 Ø88에 씌우고(D) ⑤ M2.5 × %d 3개'
        % (BS_L, CUP_L),
        fontsize=13, va='top', linespacing=1.5)

tb = fig.add_axes([0.64, 0.01, 0.35, 0.555]); tb.axis('off'); tb.set_xlim(0, 1); tb.set_ylim(0, 1)
tb.text(0.0, 0.97, '중심 맞춤 (끼워맞춤) · 통과 틈', fontsize=16, weight='bold', va='top')
y = 0.912
for n, txt in (('A', 'Ø11.8 스피곳 (정강이 보스, 길이 1.4) → 출력부 Ø12 × 1.5 홈'),
               ('B', 'Ø28 보스는 무릎 판 Ø30 구멍과 SUS 고정 링 안지름 Ø30을\n반경 틈 %.1f mm로 지나감 (닿지 않음)' % (HOLE_R - BOSS28)),
               ('C', 'AKE Ø42 파일럿 → 출력한 Ø42.05 포켓, 깊이 %.1f (파일럿 %.0f 중 %.1f는 밖).\n'
                     '고정자 앞면은 판에서 %.1f 띄움 → 파일럿 끝면만 포켓 바닥에 앉음'
                % (RING - KP0, A['pilot_len'], A['pilot_len'] - (RING - KP0), KP0 - ym(A['stator_front']))),
               ('D', '컵 립 안지름 Ø%.1f × %.0f → 허벅지 하우징 Ø%.0f 바깥 (엔코더를 모터 축에).\n'
                     '허벅지 보가 붙는 쪽은 립이 잘려 없음 (위쪽 절반)' % (2 * REG_R, LIP_L, 2 * P.HOUSING_R))):
    tb.text(0.025, y, n, fontsize=14, weight='bold', color='white', ha='center', va='center',
            bbox=dict(boxstyle='circle,pad=0.22', fc='#2e7d32', ec='none'))
    tb.text(0.07, y + 0.013, txt, fontsize=12.5, va='top', linespacing=1.4)
    y -= 0.06 if '\n' in txt else 0.042
y -= 0.01
tb.text(0.0, y, '힘이 지나가는 길 (빨간 선, 정강이 → 허벅지)', fontsize=16, weight='bold', va='top')
steps = ['바퀴 → 정강이 보', '정강이 머리 → Ø28 구동 보스 (바닥 %.1f mm)' % (FLOOR - OUT),
         'A 스피곳 + ③ 핀 × 3 + ② M3 × %d × 6' % M3_L, 'AKE 출력부 → 모터 안 출력 베어링 → 고정자',
         'C Ø42 파일럿 + ① M2.5 × %d × 6 (SUS 링 + PETG 웹 %.1f)' % (S25_L, WEB),
         '허벅지 무릎 판 %.0f → 하우징 Ø%.0f' % (P.KNEE_PLATE_T, 2 * P.HOUSING_R), '허벅지 보 → hip']
y -= 0.045
for k, st in enumerate(steps):
    tb.text(0.03, y, ('↓ ' if k else '   ') + st, fontsize=12.5, va='top')
    y -= 0.026
y -= 0.012
tb.text(0.0, y, '하중  (calc/strength.md 2절 · 5절, 검토 = SF 2)', fontsize=16, weight='bold', va='top')
tb.text(0.0, y - 0.04,
        '하중: 바퀴당 정지 60 N, 설계 300 N, 검토 600 N, 옆 150 / 300 N, 모터 12.5 → 검토 25 N·m\n'
        '• 출력 → 구동 보스, 토크 25 N·m: ②③ 개당 278 N, PETG 구멍 지압 10.1 MPa → 통과\n'
        '• 고정링 → PETG 판, 토크 25 N·m: ① 개당 228 N, PETG 구멍 지압 32.6 MPa\n'
        '   (링 홈 아래 웹 2.8 mm, 체결 마찰 무시) → 통과\n'
        '• 옆 하중 설계 Mx 45 N·m: ② 인장 1485 N (보증 4879), Ø28 보스 굽힘 23 MPa (45),\n'
        '   링 아래 지압 23 MPa (50) → 통과\n'
        '• 옆 하중 검토 Mx 89 N·m: ② 2970 N, 굽힘 45 MPa (45), 지압 45 MPa (50) → 경계\n'
        '• ① 설계 Mx 45 N·m: M2.5 8.8 인장 814 N (보증 2034), SUS 링 아래 PETG 지압\n'
        '   평균 5.8 / 나사 옆 최대 10 MPa (50) → 통과;  검토 Mx 89: 1627 N, 11.5 / 21 MPa → 통과\n'
        '• 열 (5절): 무릎 정지 토크 로봇만 L0 0.310 m 2.59 N·m (정격 52 %),\n'
        '   짐 5 kg L0 0.240 m 7.20 N·m (144 %) → 모터 외피가 PETG 연화 온도(약 70 °C)에\n'
        '   가까워질 수 있음 — 짐을 실으면 L0 ≥ 0.300 m, moteus 온도 감시\n'
        '• AKE60 출력 베어링 정격 비공개 → CubeMars 확인',
        fontsize=12.5, va='top', linespacing=1.45)
y = 0.05
tb.text(0.0, y + 0.02, '색', fontsize=14, weight='bold', va='bottom')
for k, (c, txt) in enumerate(((g['thigh'], 'PETG 허벅지'), (g['shank'], 'PETG 정강이'), (g['cup'], 'PETG 컵'),
                              (g['stator'], 'AKE 고정자'), (g['out'], 'AKE 출력부'), (g['rotor'], 'AKE 로터'),
                              (g['ring'], 'SUS 링'), (g['washer'], '와셔'), (g['pin'], '평행핀'),
                              (g['insert'], '황동 인서트'), (g['pcb'], 'moteus'), (g['spacer'], '자석 스페이서'))):
    cx, cy = (k % 4) * 0.25, y - (k // 4) * 0.02
    tb.add_patch(Rectangle((cx, cy - 0.008), 0.05, 0.016, fc=c, ec='k', lw=0.6))
    tb.text(cx + 0.065, cy, txt, fontsize=11.5, va='center')

fig.savefig(sys.argv[1] if len(sys.argv) > 1 else HERE / 'knee_joint_stack.png', dpi=80)
