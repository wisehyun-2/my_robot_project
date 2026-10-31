"""Wheel output stack (10-31 rev 2 wheel): shank plate -> AKE60 -> hub flange -> one-piece printed rim -> Xiaomi M365
8.5 x 2 solid tyre, the parts and screws that carry the robot's weight into the wheel, half sections to scale from
fusion/params.py and the hub / rim levels of fusion/legs.py (robot y of the LEFT leg on the horizontal axis, radius from
the wheel axis vertically; upper half through an output M3 screw, lower half through a dowel pin, which shares its angle
with a stator screw and lies 5 deg from a rim screw).  Overview (whole wheel) + enlarged detail with two insets: the rim
seat ring / lugs in the tyre bore (c) and a lug in its tyre cavity seen along the axis (d).  Explanatory figure, not a
manufacturing drawing; wheel_joint_section.py and wheel_joint_contact.py zoom further into the motor joint.

    python wheel_output_stack.py      # -> wheel_output_stack.png next to this script
"""
import json
import math
import pathlib
import sys
from unittest import mock
HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / 'fusion'))
sys.path.insert(0, str(ROOT / 'calc'))
# legs.py / layout.py / fasteners.py import cadlib, which loads the Fusion API at import time; only their pure-Python
# levels, hole sizes and frames are used here, so outside Fusion a stub stands in for adsk (no geometry is built)
for _m in ('adsk', 'adsk.core', 'adsk.fusion'):
    sys.modules.setdefault(_m, mock.MagicMock())
import params as P
import legs as LG
import layout as LY
import fasteners as FX
import strength as S                             # load cases + bolt-circle formula of calc/strength.md section 3
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
def ym(local):                                   # AKE local y -> robot y (the wheel motor's back points to +y)
    return P.Y_RING_WHEEL + (local - A['ring'])


LV = LG.hub_levels()
OUT = LV['out']                                  # 211.5 output face (faces the robot centre)
RING = P.Y_RING_WHEEL                            # 212.0 stator ring face = pilot pocket floor
PL0, PL1 = LV['plate'], LV['plate'] + P.WHEEL_PLATE_T          # shank wheel plate 205.5 .. 214.5
SH1 = P.Y_TIBIA_OUT                              # 235.5
REC = PL0 + P.RING_T + P.CBORE_M25[1]            # stator ring recess floor 209.2
F1, F0 = LV['f1'], LV['f0']                      # hub flange 196.5 .. 204.5 (f0 = rim hub face on the flange)
TC, B_IN, B_OUT = LV['tc'], LV['b_in'], LV['b_out']           # tyre centre 173.5, bore band 155.25 .. 191.75
T0, T1 = TC - P.TYRE_W / 2, TC + P.TYRE_W / 2    # 146.5 .. 200.5
RO = P.R_WHEEL                                   # 101
# hub flange (legs.hub_flange)
FL_R, BOSS_R, SPIG_R, SPIG_L = 23.0, P.HUB_BOSS_D / 2, 5.9, 1.4   # Ø46 disc, Ø28 boss, Ø11.8 x 1.4 spigot
REG_R, REG_L = P.RIM_REG                         # Ø11 x 3 register spigot on the flange face
REC_R, REC_L = REG_R + P.RIM_REG_GAP[0], REG_L + P.RIM_REG_GAP[1]   # legs.rim: Ø11.2 x 3.2 recess in the rim hub
W_OD, W_ID, W_T = FX.WASHER[3]                   # DIN 125 M3: Ø7 / Ø3.2 x 0.5
DK, K = FX.SHCS[3][:2]                           # ISO 4762 M3 head Ø5.5 x 3
RQ, RP = P.AKE_PCD_OUT / 2, P.AKE_PCD_RING / 2   # output M3 r 10, stator M2.5 r 18.25
OUT_CB_R = 3.75                                  # legs.hub_flange: output screw counterbore Ø7.5
BEAR = OUT - P.HUB_SCREW_SEAT                    # 202.0 washer seat of the output M3 x 14 (counterbore floor)
LS = P.HUB_SCREW_L                               # 14
HB_O = BEAR - W_T                                # 201.5 output screw head bearing face
ENG = HB_O + LS - OUT                            # 4.0 thread engagement in the 5 mm output taps
HEAD_GAP = (HB_O - K) - F0                       # 2.0 output screw head end -> rim hub face
# one-piece rim (legs.rim)
RH = 18.0                                        # legs.HUB_SCREWS radius (PCD 36), angles 30 + 60 k
RIM_L = 14                                       # 'FX ISO 4762 M3x14 rim'
RIM_CB_R = 3.8                                   # legs.rim: washer + head counterbore Ø7.6
INS_L = 5.7                                      # 'FX insert M3x5.7 hub' (hole Ø4.0 x 6)
RB = P.TYRE_BORE_D / 2                           # 70.5 seat ring OD = tyre bore
RI = RB - P.RIM_SEAT_T                           # 64.5 seat ring ID
RF = P.RIM_FLANGE_D / 2                          # 75 bead flanges (and lug tips)
HUB0 = F0 - P.RIM_HUB_T                          # 186.5 rim hub inner face
YWEB = F0 - P.RIM_WEB_T                          # 190.5 web inner face at the seat ring
Y_O = B_OUT - P.RIM_FLANGE_T                     # 152.25 open-side flange outer face
FIL = 5.0                                        # legs.rim: 5 mm 45 deg fillet web -> seat ring
SEAT_R = F0 - P.RIM_SCREW_SEAT                   # 188.5 rim washer seat (counterbore floor)
HB_R = SEAT_R - W_T                              # 188.0 rim screw head bearing face
HEAD_R = HB_R - K                                # 185.0 rim screw head end
TIP_R = HB_R + RIM_L                             # 202.0
ENG_R = TIP_R - F0                               # 4.5 past the flange face (4.4 in the inserts)
PROUD = HUB0 - HEAD_R                            # 2.5 rim screw heads stand proud of the rim hub face
NL, LUG_W, LUG_H, LUG_L = P.RIM_LUGS             # 24 lugs 8 wide x 4.5 high x 26.5 long, 45 deg ends
NC, CAV_W, CAV_D, CAV_L = P.TYRE_CAVITY          # 24 tyre bore cavities 9 x 10 deep x 27.5
R_BEAD, R_FULL = RB + P.TYRE_BEAD_H, P.TYRE_FLARE_R            # 75.5, 85
_k = (RI - P.RIM_HUB_R) / (YWEB - HUB0)          # hub taper slope dr / dy (r 32 at HUB0 -> r 64.5 at YWEB)
Y_FC = (RI - FIL + YWEB - P.RIM_HUB_R + _k * HUB0) / (1 + _k)  # fillet chamfer meets the taper (189.95, r 60.05)
R_FC = RI - FIL + YWEB - Y_FC
# angles of the hole circles in the robot frame (deg from +x towards +z, left wheel)
_m = LY.frames('wheel', 1)['motor']
ANG = {k: sorted(round(math.degrees(math.atan2(p[2] - P.R_WHEEL, p[0])) % 360) for p in
                 LY.motor_points(_m, r, a, A['out']))
       for k, r, a in (('out', RQ, P.AKE_OUT_M3), ('pin', RQ, P.AKE_OUT_PIN), ('ring', RP, P.AKE_RING_M25))}
ANG['rim'] = sorted(round(math.degrees(a)) for a in LG.HUB_SCREWS)
PIN_RIM = min(abs(p - r) for p in ANG['pin'] for r in ANG['rim'])     # 5 deg
OUT_RIM = min(abs(p - r) for p in ANG['out'] for r in ANG['rim'])     # 25 deg
# loads (calc/strength.py section 3 formulas)
ROBOT_KG = json.loads((ROOT / 'calc' / 'mass_properties.json').read_text(encoding='utf-8'))['links']['robot']['mass']
W_STATIC = ROBOT_KG * P.G / 2
ARM = P.wheel_offset()                           # 38.0 output face -> tyre centre plane
ARM_J = F0 - TC                                  # 23.0 rim / flange face -> tyre centre plane
WA = math.pi / 4 * (W_OD ** 2 - W_ID ** 2)       # 30.4 mm2 washer bearing area
M_OUT = {t: S.F_CHK * ARM + S.FL_CHK * RO if t == 'chk' else S.F_DES * ARM + S.FL_DES * RO for t in ('chk', 'des')}
M_RIM = S.F_CHK * ARM_J + S.FL_CHK * RO
P_OUT = {t: S.bolt_circle_max(M_OUT[t], 6, RQ) / WA for t in M_OUT}  # 58 / 29 MPa
F_INS = S.bolt_circle_max(M_RIM, 6, RH)          # 817 N insert pull (check)
F_LUG = S.T_CHK / (RB + LUG_H / 2) / (NL / 4)    # 57 N per lug (a quarter of the lugs carry 25 N m)


def _t_at(r):                                    # rim hub / web thickness at radius r (strength.py)
    if r <= P.RIM_HUB_R:
        return P.RIM_HUB_T
    return P.RIM_HUB_T - (P.RIM_HUB_T - P.RIM_WEB_T) * (r - P.RIM_HUB_R) / (RI - P.RIM_HUB_R)


S_PLATE = max(6 * M_RIM / (math.pi * r * _t_at(r) ** 2) for r in (FL_R, P.RIM_HUB_R, (P.RIM_HUB_R + RI) / 2, RI))
# view ranges (robot y, r)
R_CLIP = 58.0                                    # (b): rim hub / web drawn up to this radius (break line)
B_X, B_R = (146.0, 243.0), (-62.0, 64.0)         # (b) axes range (labels inside the rim on the left)
B_BOX = (182.0, 243.0)                           # (b) parts range marked in (a)
C_X, C_R = (141.0, 205.0), (56.0, 106.0)         # (c) range (axes from C_R[0] - 26: labels inside the rim)

g = dict(petg='#9fd29f', flange='#e07b39', rim='#f2a65a', stator='#b8b8b8', rotor='#7f9fbf', out='#d9b38c',
         tyre='#4a4a4a', screw='#2e2e2e', washer='#e3c034', pin='#c0392b', ring='#6f86a6', insert='#b8860b')


def tyre_pts(rr=12.0, n=10):
    """Tyre half section through a bore cavity (vendor.m365_tyre): rounded crown, side face to TYRE_FLARE_R, straight
    flare to the bead band (TYRE_BEAD_H high, bore width), bore Ø141 with the cavity open to it."""
    pts = [(T0, R_FULL)]
    for cx, a0, a1 in ((T0 + rr, 180, 90), (T1 - rr, 90, 0)):
        for k in range(n + 1):
            t = math.radians(a0 + (a1 - a0) * k / n)
            pts.append((cx + rr * math.cos(t), RO - rr + rr * math.sin(t)))
    return pts + [(T1, R_FULL), (B_IN, R_BEAD), (B_IN, RB), (TC + CAV_L / 2, RB), (TC + CAV_L / 2, RB + CAV_D),
                  (TC - CAV_L / 2, RB + CAV_D), (TC - CAV_L / 2, RB), (B_OUT, RB), (B_OUT, R_BEAD)]




def rim_pts(r_clip=None):
    """Rim half section through a lug: hub face, taper, fillet, seat ring, open-side flange, lug, motor-side flange.
    With r_clip: only the hub and the web below that radius, closed by a break line."""
    if r_clip is not None:
        y = HUB0 + (r_clip - P.RIM_HUB_R) / _k
        return [(HUB0, 0), (HUB0, P.RIM_HUB_R), (y, r_clip)] + zig((y, r_clip), (F0, r_clip), 2, 0.6) + \
               [(F0, r_clip), (F0, 0)]
    lug = [(TC - LUG_L / 2, RB), (TC - LUG_L / 2 + LUG_H, RB + LUG_H), (TC + LUG_L / 2 - LUG_H, RB + LUG_H),
           (TC + LUG_L / 2, RB)]
    return ([(HUB0, 0), (HUB0, P.RIM_HUB_R), (Y_FC, R_FC), (YWEB - FIL, RI), (Y_O, RI), (Y_O, RF), (B_OUT, RF),
             (B_OUT, RB)] + lug + [(B_IN, RB), (B_IN, RF), (F0, RF), (F0, 0)])


def zig(p0, p1, n=4, a=0.8):
    """Break line points strictly between p0 and p1 (zigzag of amplitude a)."""
    (x0, y0), (x1, y1) = p0, p1
    L = math.hypot(x1 - x0, y1 - y0)
    nx, ny = -(y1 - y0) / L, (x1 - x0) / L
    return [(x0 + (x1 - x0) * k / (2 * n) + nx * a * (1 if k % 2 else -1),
             y0 + (y1 - y0) * k / (2 * n) + ny * a * (1 if k % 2 else -1)) for k in range(1, 2 * n)]


def draw_parts(ax, detail=False):
    def poly(pts, c, s=1, z=1, **kw):
        ax.add_patch(Polygon([(x, s * r) for x, r in pts], fc=c, ec=kw.pop('ec', 'k'), lw=kw.pop('lw', 0.9),
                             zorder=z, **kw))

    def rect(x0, x1, r0, r1, c, s=1, z=1, **kw):
        poly([(x0, r0), (x1, r0), (x1, r1), (x0, r1)], c, s, z, **kw)

    def pocket(x_open, x_floor, r0, r1, s, z=1.2):
        """Hole opening at x_open (the part's edge line there is covered), floor at x_floor."""
        d = 0.15 if x_floor > x_open else -0.15
        rect(x_open - d, x_floor, r0, r1, 'white', s, z, ec='none')
        ax.plot([x_open - d, x_floor, x_floor, x_open - d], [s * r0, s * r0, s * r1, s * r1], 'k-', lw=0.6, zorder=z)

    for s in (1, -1):
        if not detail:                           # M365 tyre (block tread not drawn), section through a bore cavity
            poly(tyre_pts(), g['tyre'], s)
        # one-piece printed rim: hub 10 (r <= 32) tapering to the 6 mm web, fillet, seat ring, lug, bead flanges
        poly(rim_pts(R_CLIP if detail else None), g['rim'], s)
        rect(F0 - REC_L, F0, 0, REC_R, 'white', s, z=1.2, lw=0.6)                 # Ø11.2 x 3.2 register recess
        # rim screws (r 18; 6 x, both halves): counterbore Ø7.6 to the washer seat, Ø3.4 hole, flange insert
        pocket(HUB0, SEAT_R, RH - RIM_CB_R, RH + RIM_CB_R, s)
        rect(SEAT_R, F0, RH - LG.M3_CLEAR / 2, RH + LG.M3_CLEAR / 2, 'white', s, z=1.2, lw=0.6)
        # shank (PETG): wheel plate + housing around the motor (can cavity r 37); the beam goes on up to the knee
        top = 52.0 if s > 0 else P.HOUSING_R
        poly([(PL0, P.STATOR_RECESS_D / 2), (PL0, top), (SH1, top), (SH1, P.CAN_CLEAR_R), (PL1, P.CAN_CLEAR_R),
              (PL1, 21.025), (RING, 21.025), (RING, 15.0), (REC, 15.0), (REC, P.STATOR_RECESS_D / 2)], g['petg'], s)
        rect(REC - P.RING_T, REC, P.RING_STATOR[1] / 2, P.RING_STATOR[0] / 2, g['ring'], s, z=2, lw=1.0)
        # AKE60-8: stator (pilot Ø42 x 3, front face r 20.3 .. 30.75), rotor can + cap, output carrier
        sf, cb = ym(A['stator_front']), ym(A['can_back'])
        poly([(RING, 13.6), (RING, 21.0), (RING + 3, 21.0), (RING + 3, 20.3), (sf, 20.3), (sf, 30.75), (cb, 30.75),
              (cb, 13.6)], g['stator'], s)
        poly([(ym(A['can_front']), 31.2), (ym(A['can_front']), A['can_r']), (ym(A['cap_outer']), A['can_r']),
              (ym(A['cap_outer']), 31.2)], g['rotor'], s)
        poly([(cb, 7.9), (ym(A['cap_face']), 7.9), (ym(A['cap_face']), 34.2), (cb, 34.2)], g['rotor'], s)
        poly([(OUT + 1.5, 0), (OUT + 1.5, 6.0), (OUT, 6.0), (OUT, 13.5), (cb, 13.5), (cb, 0)], g['out'], s)
        # hub flange (PETG 100 %): Ø11 x 3 register spigot, Ø46 disc, Ø28 boss through the Ø30 plate hole, Ø11.8
        # spigot into the output's Ø12 recess
        poly([(F0 - REG_L, 0), (F0 - REG_L, REG_R), (F0, REG_R), (F0, FL_R), (F1, FL_R), (F1, BOSS_R), (OUT, BOSS_R),
              (OUT, SPIG_R), (OUT + SPIG_L, SPIG_R), (OUT + SPIG_L, 0)], g['flange'], s, z=1.5)
        pocket(F0, F0 + 6.0, RH - LG.INSERT_M3 / 2, RH + LG.INSERT_M3 / 2, s, z=1.6)  # insert hole Ø4.0 x 6
        rect(F0 + 0.1, F0 + 0.1 + INS_L, RH - LG.INSERT_M3 / 2, RH + LG.INSERT_M3 / 2, g['insert'], s, z=2.5, lw=0.8)
        # rim screw ISO 4762 M3 x 14 8.8 + DIN 125 washer
        rect(HB_R, SEAT_R, RH - W_OD / 2, RH + W_OD / 2, g['washer'], s, z=3, lw=0.8)
        rect(HEAD_R, HB_R, RH - DK / 2, RH + DK / 2, g['screw'], s, z=3)
        rect(HB_R, TIP_R, RH - 1.25, RH + 1.25, g['screw'], s, z=3)
        # stator ring screws: DIN 7984 M2.5 x 8 low head on the SUS ring (6 on PCD 36.5: both halves)
        rect(REC - P.RING_T, REC - P.RING_T + 8.0, RP - 1.25, RP + 1.25, g['screw'], s, z=3)
        rect(REC - P.RING_T - 1.5, REC - P.RING_T, RP - 2.25, RP + 2.25, g['screw'], s, z=3)
    # upper half: output M3 x 14 12.9 at r 10 in the flange counterbore Ø7.5 (washer under the head)
    rect(F0, BEAR, RQ - OUT_CB_R, RQ + OUT_CB_R, 'white', z=1.6, lw=0.6)
    rect(BEAR, OUT, RQ - LG.M3_CLEAR / 2, RQ + LG.M3_CLEAR / 2, 'white', z=1.6, lw=0.6)
    rect(HB_O, BEAR, RQ - W_OD / 2, RQ + W_OD / 2, g['washer'], z=3, lw=0.8)
    rect(HB_O - K, HB_O, RQ - DK / 2, RQ + DK / 2, g['screw'], z=3)
    rect(HB_O, HB_O + LS, RQ - 1.25, RQ + 1.25, g['screw'], z=3)
    # lower half: dowel Ø3 x 10 (3 mm in the output, 7 mm in the flange)
    rect(OUT - 7.0, OUT + 3.0, -RQ - 1.5, -RQ + 1.5, g['pin'], z=3, lw=1.0)


def load_path(ax, lw, path):
    ax.plot(*zip(*path[:-1]), color='red', lw=lw, alpha=0.7, zorder=6, solid_capstyle='round', solid_joinstyle='round')
    ax.annotate('', xy=path[-1], xytext=path[-2], zorder=6,
                arrowprops=dict(arrowstyle='-|>', color='red', alpha=0.7, lw=lw, mutation_scale=8 * lw))


# ground -> rubber beside the cavity -> seat ring -> web -> hub -> register / clamp face -> flange -> dowel -> output
# -> bearing -> stator -> pilot / ring screws -> shank
PATH_RIM = [(TC + 16.0, -84.0), (TC + 16.0, -68.0), (YWEB - 1.5, -63.0), (193.5, -52.0)]
PATH_JOINT = [(192.5, -30.0), (190.0, -15.0), (194.8, -4.6), (200.0, -8.0), (206.0, -10.0), (OUT + 4, -9.5),
              (ym(4), -11), (ym(4), -17.5), (RING + 2, -18), (REC + 1, -26), (PL0 + 4, -37), (226.0, -40.5)]


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


fig = plt.figure(figsize=(24, 32))
fig.suptitle('바퀴 출력부 — 로봇 무게를 바퀴로 넘기는 연결  (왼쪽 다리 단면, 모델 치수 mm, 실제 비율)',
             fontsize=22, x=0.01, ha='left', y=0.995)
T = dict(fontsize=12, ha='center', va='center', zorder=9)

# ============================================================================= (a) overview
ax = fig.add_axes([0.01, 0.565, 0.25, 0.395]); ax.set_aspect('equal'); ax.axis('off')
draw_parts(ax)
load_path(ax, 3.0, [(TC, -RO + 1)] + PATH_RIM + PATH_JOINT)
ax.plot([T0 - 3, ym(A['cap_face']) + 3], [0, 0], 'k-.', lw=0.7, zorder=8)
ax.set_title('(a) 전체 — 바퀴 하나', fontsize=17, loc='left')
ax.text(TC, 93, '타이어 Ø%.0f × %.0f\nM365 8.5 × 2 통고무' % (P.TYRE_OD, P.TYRE_W), color='white', **T)
ax.text(TC, -93, '바닥 반력\n정지 약 %.0f N' % W_STATIC, color='white', weight='bold', **T)
ax.annotate('', xy=(TC, -RO + 1), xytext=(TC, -RO - 10), zorder=7,
            arrowprops=dict(arrowstyle='-|>', color='red', lw=3, mutation_scale=24))
ax.text(PL0 + 15, 44, 'PETG 정강이\n↑ 무릎까지', fontsize=11, ha='center', va='center', zorder=9)
ax.text(226, 25, 'AKE60-8', fontsize=11, ha='center', va='center', zorder=9)
ax.text(SH1 + 1.5, 71, 'moteus\n컵(생략)', fontsize=10, ha='left', va='center', color='#555')
label(ax, '한 조각 림\n(PETG)', (189.5, 40.0), (207.0, 90.0), fs=11)
label(ax, '돌기 24개\n→ 타이어 구멍', (TC - 6, RB + LUG_H - 0.5), (152.0, 42.0), fs=11, rp=(0.5, 1))
label(ax, '허브 플랜지', (200.5, -20.0), (207.0, -78.0), fs=11)
dim_v(ax, 138, -RO, RO, 'Ø%.0f' % P.TYRE_OD, ext=(TC, TC))
R1, R2 = -110.0, -119.0
dim_h(ax, T0, T1, R1, '타이어 폭 %.0f' % P.TYRE_W, (-89, -89), fs=12)
dim_h(ax, T1, PL0, R1, '%.0f' % P.GAP_TYRE, (-89, -P.HOUSING_R), fs=12)
dim_h(ax, PL0, PL1, R1, '%.0f' % P.WHEEL_PLATE_T, (None, -P.HOUSING_R), fs=12)
dim_h(ax, TC, OUT, R2, '굽힘 팔 %.1f' % ARM, (-RO, -14), fs=12, c='#b03020')
ax.add_patch(Rectangle((B_BOX[0], B_R[0]), B_BOX[1] - B_BOX[0], B_R[1] - B_R[0], fill=False, ec='#1f4e9c', lw=1.6,
                       ls='--', zorder=12))
ax.text(B_BOX[1], B_R[0] - 1.0, '(b) 확대 범위', color='#1f4e9c', fontsize=11, ha='right', va='top', zorder=12)
ax.add_patch(Rectangle((C_X[0], C_R[0]), C_X[1] - C_X[0], C_R[1] - C_R[0], fill=False, ec='#7a4b00', lw=1.4, ls='--',
                       zorder=12))
ax.text(C_X[0], C_R[1] + 1.0, '(c) 확대 범위', color='#7a4b00', fontsize=11, va='bottom', zorder=12)
ax.set_xlim(128, 250); ax.set_ylim(-124, 114)

# ============================================================================= (c) rim seat ring, lugs, tyre bore
ax = fig.add_axes([0.28, 0.565, 0.71, 0.19]); ax.set_aspect('equal'); ax.axis('off')
draw_parts(ax)
ax.set_title('(c) 확대 — 림 안장 링 · 돌기 · 타이어 안쪽  (위쪽 절반, 돌기 하나와 그 구멍을 지나는 단면)', fontsize=16,
             loc='left')
ax.text(TC, 92.5, '타이어 (통고무, 블록 트레드 생략)', color='white', **T)
ax.text(TC, RB + 2.0, '돌기', fontsize=11, ha='center', va='center', zorder=9)
ax.text(TC, (RI + RB) / 2, '안장 링 %.0f t' % P.RIM_SEAT_T, fontsize=11, ha='center', va='center', zorder=9)
XL, XR = C_X[0] - 1.0, C_X[1] + 2.0               # label columns left (open side) and right (motor side) of the wheel
label(ax, '타이어 Ø%.0f × %.0f (M365 8.5 × 2 통고무)' % (P.TYRE_OD, P.TYRE_W), (T1 - 2.0, 97.0), (XR, 99.0), fs=11.5)
label(ax, '비드 띠 %.0f → Ø%.0f까지 경사 (모델 추정, 실물 확인)' % (P.TYRE_BEAD_H, 2 * R_FULL),
      (197.0, 79.5), (XR, 88.0), fs=11.5)
label(ax, '웨브 쪽 플랜지 Ø%.0f (웨브 바깥 끝, %.2f t)' % (P.RIM_FLANGE_D, F0 - B_IN), (195.0, RF - 1.0), (XR, 77.0),
      fs=11.5)
label(ax, '%.0f × 45° 필릿 (웨브 → 안장 링)' % FIL, (YWEB - FIL / 2 - 0.4, RI - FIL / 2 + 0.4), (XR, 66.0), fs=11.5)
label(ax, '웨브 %.0f → 림 허브 %.0f (아래로 이어짐, (b))' % (P.RIM_WEB_T, P.RIM_HUB_T), (193.6, 58.5), (XR, 59.0),
      fs=11.5)
label(ax, '타이어 구멍 %d개: 폭 %.0f × 길이 %.1f × 깊이 %.0f\n(돌기 위로 %.1f 빈 공간)'
      % (NC, CAV_W, CAV_L, CAV_D, CAV_D - LUG_H), (TC - 11.0, RB + CAV_D - 1.5), (XL, 92.0), fs=11.5, ha='right')
label(ax, '돌기 %d개: 폭 %.0f × 길이 %.1f × 높이 %.1f\n(양 끝 45°, 윗면 %.1f, 끝 = Ø%.0f) — 바퀴 토크를 받음'
      % (NL, LUG_W, LUG_L, LUG_H, LUG_L - 2 * LUG_H, 2 * (RB + LUG_H)), (TC - 9.5, RB + 3.0), (XL, 79.0), fs=11.5,
      ha='right')
label(ax, '열린 쪽 플랜지 Ø%.0f × %.0f (타이어를 이쪽에서 끼움)' % (P.RIM_FLANGE_D, P.RIM_FLANGE_T), (Y_O + 0.6, RF - 1.5),
      (XL, 70.0), fs=11.5, ha='right')
label(ax, '안장 링 Ø%.0f = 타이어 안지름 (모델은 틈 0), 안쪽 Ø%.0f' % (P.TYRE_BORE_D, 2 * RI), (Y_O + 4.5, RI + 0.3),
      (XL, 61.0), fs=11.5, ha='right')
dim_h(ax, B_OUT, B_IN, 62.0, '타이어 안쪽 폭 %.1f = 두 플랜지 사이' % P.TYRE_BORE_W, (RB, RB), fs=11)
dim_h(ax, Y_O, B_OUT, 59.5, '%.0f' % P.RIM_FLANGE_T, (RI, RI), fs=11)
dim_v(ax, TC - 5.0, RB, RB + LUG_H, '%.1f' % LUG_H, fs=10.5, left=True)
dim_v(ax, TC + 11.0, RB, RB + CAV_D, '%.0f' % CAV_D, fs=10.5, left=False)
ax.set_xlim(C_X[0] - 46, C_X[1] + 46); ax.set_ylim(C_R[0], C_R[1])

# ============================================================================= (b) detail
ax = fig.add_axes([0.01, 0.01, 0.52, 0.525]); ax.set_aspect('equal'); ax.axis('off')
draw_parts(ax, detail=True)
load_path(ax, 4.0, [(193.5, -R_CLIP + 0.8)] + PATH_JOINT)
ax.text(193.0, -R_CLIP - 1.5, '↑ 타이어 → 안장 링 → 웨브에서', color='red', fontsize=11, ha='right', va='center', zorder=9,
        bbox=dict(fc='white', ec='none', alpha=0.85, pad=0.4))
ax.text(226.0, -P.HOUSING_R - 1.2, '정강이 → 무릎', color='red', fontsize=11, ha='center', va='top', zorder=9)
ax.plot([HEAD_R - 3, 241], [0, 0], 'k-.', lw=0.8, zorder=8)
ax.text(241, 0.6, '회전축', fontsize=10, ha='right', va='bottom', zorder=8)
ax.set_title('(b) 확대 — 위쪽 절반: 출력 M3 나사 자리 단면,  아래쪽 절반: 평행핀 자리 단면,  빨간 선: 무게가 지나가는 길\n'
             '      (고정자 M2.5는 핀과 같은 각도, 림 M3는 핀에서 %d° 옆 — 둘 다 제 반지름에 양쪽 절반 모두 그림)' % PIN_RIM,
             fontsize=16, loc='left')
ax.text(PL0 + 1.8, 38, '정강이 바퀴 판 (PETG)', fontsize=11, rotation=90, ha='center', va='center', zorder=9)
ax.text(226.5, 25.2, 'AKE60-8 고정자', **T)
ax.text(226.5, 7.0, 'AKE 출력부 (회전)', **T)
ax.text(227, 32.85, '로터 캔 (회전)', fontsize=9.5, ha='center', va='center', zorder=9, color='white')
ax.text(225, 46, '정강이 하우징 (PETG)', **T)
ax.text(191.5, 26.0, '림 허브\n%.0f t' % P.RIM_HUB_T, fontsize=11, ha='center', va='center', zorder=9)
ax.text(192.6, 49.0, '림 웨브', fontsize=11, ha='center', va='center', zorder=9)
X = 180.0                                         # label column inside the rim (left of the rim hub)
label(ax, '허브 플랜지 (PETG 100 %)', (201.0, 21.5), (X, 61.0), fs=12, c='#9c4a17', ha='right')
label(ax, 'SUS304 클램프 링 2 t (Ø45 / Ø30)', (REC - 1.0, 21.8), (X, 56.5), fs=11, ha='right')
label(ax, 'Ø28 보스 / Ø30 판 구멍 (반경 틈 1.0)', (205.0, 14.0), (X, 52.0), fs=11, ha='right')
label(ax, 'Ø46 허브 플랜지 ↔ 림 허브 면 (맞대어 조임)', (F0 + 0.2, FL_R - 1.0), (X, 47.5), fs=11, ha='right')
label(ax, '림 허브 Ø%.0f × %.0f → 웨브 %.0f → %.0f (Ø%.0f까지, 구멍 없음)'
      % (2 * P.RIM_HUB_R, P.RIM_HUB_T, P.RIM_HUB_T, P.RIM_WEB_T, 2 * RI), (188.0, 40.0), (X, 43.0), fs=11, ha='right')
label(ax, '림 카운터보어 Ø%.1f, 와셔 자리는 플랜지 면에서 %.0f' % (2 * RIM_CB_R, P.RIM_SCREW_SEAT), (187.5, RH + RIM_CB_R - 0.3),
      (X, 30.0), fs=11, ha='right')
label(ax, '머리 끝이 림 허브 면보다 %.1f 나옴' % PROUD, (HEAD_R + 0.2, RH - 2.0), (X, 12.5), fs=11, ha='right')
label(ax, 'Ø%.1f 카운터보어 (플랜지): 머리 끝 → 림 면 %.1f' % (2 * OUT_CB_R, HEAD_GAP), (F0 + 1.0, RQ - 2.4),
      (X, 6.0), fs=11, ha='right')
label(ax, 'Ø42 파일럿 / Ø42.05 포켓', (214.2, 21.0), (219.5, 41.0), fs=11)
label(ax, 'Ø%.0f × %.0f 등록 턱 (플랜지) → 림 Ø%.1f × %.1f 홈' % (2 * REG_R, REG_L, 2 * REC_R, REC_L), (F0 - 1.5, -REG_R + 0.4),
      (X, -4.5), fs=11, ha='right')
label(ax, '핀 · M3 × %d: PCD %.0f' % (LS, P.AKE_PCD_OUT), (206.0, -RQ - 1.5), (X, -12.0), fs=11, ha='right')
label(ax, '림 M3 × %d: PCD %.0f (황동 인서트)' % (RIM_L, 2 * RH), (199.0, -RH - 2.0), (X, -19.0), fs=11, ha='right')
label(ax, 'M2.5 × 8: PCD %.1f' % P.AKE_PCD_RING, (205.8, -RP - 2.2), (X, -26.0), fs=11, ha='right')

# screws / pins
tag(ax, '1', (208.0, 30.0), to=(206.2, RP + 2.2))
tag(ax, '2', (204.5, 5.0), to=(HB_O - 1.5, RQ - 1.5))
tag(ax, '3', (209.5, -3.0), c='#c0392b', to=(208.0, -RQ + 1.4))
tag(ax, '4', (183.0, 24.5), to=(HEAD_R + 1.0, RH + 2.4))
tag(ax, 'A', (215.6, -3.2), c='#2e7d32', to=(212.6, -5.0))
tag(ax, 'B', (190.0, -8.5), c='#2e7d32', to=(F0 - 1.0, -REG_R - 0.2))
tag(ax, 'C', (218.5, -25.5), c='#2e7d32', to=(213.6, -21.0))

# axial dimensions: rim side (over the web) and joint side (below the shank housing)
dim_h(ax, HUB0, F0, -48.0, '림 허브 %.0f' % P.RIM_HUB_T, (-34, -34), fs=11)
dim_h(ax, SEAT_R, F0, -36.0, '%.0f' % P.RIM_SCREW_SEAT, (-RH - RIM_CB_R, -FL_R), fs=11)
dim_h(ax, F0, F1, -48.0, '플랜지 %.0f' % (F1 - F0), (-FL_R, -FL_R), fs=11)
dim_h(ax, F0, TIP_R, -30.0, '%.1f' % ENG_R, (-FL_R, -RH - 1.25), fs=11, c='#b03020')
ax.text(TIP_R + 0.6, -30.6, '← ④ 인서트 물림', fontsize=10.5, va='top', color='#b03020', zorder=8,
        bbox=dict(fc='white', ec='none', alpha=0.85, pad=0.3))
dim_h(ax, F1, PL0, -53.0, '1', (-FL_R, -48), fs=11)
dim_h(ax, PL0, PL1, -53.0, '판 %.0f' % P.WHEEL_PLATE_T, (None, -48), fs=11)
dim_h(ax, BEAR, OUT, -58.0, '%.1f' % P.HUB_SCREW_SEAT, (-RQ - 3.5, -14), fs=11)
dim_h(ax, OUT, OUT + ENG, -63.0, '%.1f' % ENG, (-14, None), fs=11, c='#b03020')
ax.text(OUT + ENG + 0.8, -63.6, '← ② M3 × %d 물림' % LS, fontsize=10.5, va='top', color='#b03020')
dim_h(ax, RING, PL1, 55.0, '포켓 2.5', (52, 52), fs=10.5, above=True)
dim_h(ax, PL0, REC, 55.0, '링 자리 3.7', (52, 52), fs=10.5, above=True)
ax.set_xlim(*B_X); ax.set_ylim(B_R[0] - 6, B_R[1])

# ============================================================================= (d) lug in its tyre cavity, along the axis
DA = 10.0                                         # half angle shown around the top lug [deg]
R_TOP = 86.0


def P2(r, th):
    t = math.radians(th)
    return (r * math.cos(t), r * math.sin(t))


def notch_ring(r_base, w, h, th0, th1, centres):
    """Arc at r_base from th1 down to th0 with straight-sided boxes (width w, out to radius h) on the given centre
    angles, like the lug / cavity boxes of legs.rim and vendor.m365_tyre (the same outline for a lug or a cavity)."""
    arc = []
    n = 40
    half = math.degrees(math.atan2(w / 2, math.sqrt(r_base ** 2 - (w / 2) ** 2)))
    rho = math.sqrt(r_base ** 2 - (w / 2) ** 2)
    a = th1
    for c in sorted(centres, reverse=True):
        arc += [P2(r_base, a + (c + half - a) * k / n) for k in range(n + 1)]
        u, v = P2(1, c), P2(1, c + 90)
        arc += [(u[0] * h + v[0] * w / 2, u[1] * h + v[1] * w / 2), (u[0] * h - v[0] * w / 2, u[1] * h - v[1] * w / 2),
                (u[0] * rho - v[0] * w / 2, u[1] * rho - v[1] * w / 2)]
        a = c - half
    return arc + [P2(r_base, a + (th0 - a) * k / n) for k in range(n + 1)]


dd = fig.add_axes([0.555, 0.299, 0.22, 0.142]); dd.set_aspect('equal'); dd.axis('off')
lo, hi = 90.0 - DA, 90.0 + DA
rub = [P2(R_TOP, lo + (hi - lo) * k / 80) for k in range(81)] + notch_ring(RB, CAV_W, RB + CAV_D, lo, hi, [90.0])
dd.add_patch(Polygon(rub, fc=g['tyre'], ec='k', lw=0.9))
rim_d = notch_ring(RB, LUG_W, RB + LUG_H, lo, hi, [90.0]) + [P2(RI, lo + (hi - lo) * k / 80) for k in range(81)]
dd.add_patch(Polygon(rim_d, fc=g['rim'], ec='k', lw=0.9))
dd.text(*P2(R_TOP - 3.5, 90 - 6.5), '타이어', color='white', fontsize=11, ha='center', va='center')
dd.text(0, (RI + RB) / 2, '안장 링 %.0f t' % P.RIM_SEAT_T, fontsize=11, ha='center', va='center')
dd.text(0, RB + LUG_H / 2, '돌기', fontsize=11, ha='center', va='center')
dd.annotate('', xy=(-CAV_W / 2, RB + CAV_D + 1.2), xytext=(CAV_W / 2, RB + CAV_D + 1.2), zorder=8,
            arrowprops=dict(arrowstyle='<->', lw=0.9, color='white', shrinkA=0, shrinkB=0))
dd.text(0, RB + CAV_D + 1.8, '구멍 %.0f' % CAV_W, color='white', fontsize=10.5, ha='center', va='bottom')
dd.annotate('', xy=(-LUG_W / 2, RB + LUG_H + 1.2), xytext=(LUG_W / 2, RB + LUG_H + 1.2), zorder=8,
            arrowprops=dict(arrowstyle='<->', lw=0.9, shrinkA=0, shrinkB=0))
dd.text(0, RB + LUG_H + 1.7, '돌기 %.0f' % LUG_W, fontsize=10.5, ha='center', va='bottom')
for sg in (1, -1):                                # torque: the lug side pushes the rubber (either direction)
    dd.annotate('', xy=(sg * (CAV_W / 2 + 2.2), RB + 2.3), xytext=(sg * (LUG_W / 2 - 0.6), RB + 2.3), zorder=9,
                arrowprops=dict(arrowstyle='-|>', color='red', lw=2.0, mutation_scale=14))
dd.set_xlim(-14.5, 14.5); dd.set_ylim(RI - 0.5, R_TOP + 0.5)

# ============================================================================= text columns
tx = fig.add_axes([0.28, 0.775, 0.71, 0.185]); tx.axis('off'); tx.set_xlim(0, 1); tx.set_ylim(0, 1)
rows = [
    ('1', '#1f4e9c', 'M2.5 × 8  저두 볼트 DIN 7984 8.8   × 6',
     '정강이 판 → AKE 고정자 링 (PCD 36.5).  SUS304 클램프 링 2 t (Ø45 / Ø30) 위에서 조임,\n'
     '머리 1.5 t는 판 면보다 0.2 들어감.  고정자 탭(깊이 5)에 3.2 mm 물림'),
    ('2', '#1f4e9c', 'M3 × %d  ISO 4762 12.9   × 6   + 평와셔 DIN 125 M3 (Ø7 × 0.5)   × 6' % LS,
     '허브 플랜지 → 모터 출력부 (PCD 20).  머리는 플랜지 카운터보어 Ø%.1f 안, 림 허브가 덮음 (머리 끝 → 림 면 %.1f).\n'
     '와셔 자리가 출력면에서 %.1f → 출력부 탭(깊이 5)에 %.1f mm 물림' % (2 * OUT_CB_R, HEAD_GAP, P.HUB_SCREW_SEAT, ENG)),
    ('3', '#c0392b', '평행핀 ISO 8734 Ø3 × 10  m6 경화   × 3',
     '출력부 구멍 3 mm + 허브 플랜지 7 mm (PCD 20, M3와 30° 어긋남 = ①과 같은 각도).\n'
     '두 부품 위치를 맞추고 바퀴 토크를 나사와 함께 받음'),
    ('4', '#1f4e9c', 'M3 × %d  ISO 4762 8.8   × 6   + 평와셔 DIN 125 M3   × 6   + 황동 열압입 인서트 M3 × %.1f   × 6'
     % (RIM_L, INS_L),
     '림 허브 → 허브 플랜지 (r %.0f = PCD %.0f, 핀에서 %d° · 출력 M3에서 %d° 옆).  림 허브 %.0f t 안 카운터보어 Ø%.1f,\n'
     '와셔 자리가 플랜지 면에서 %.0f → 인서트(구멍 Ø%.1f × 6)에 %.1f mm 물림.  머리는 림 허브 면보다 %.1f 나옴 (림 안쪽)'
     % (RH, 2 * RH, PIN_RIM, OUT_RIM, P.RIM_HUB_T, 2 * RIM_CB_R, P.RIM_SCREW_SEAT, LG.INSERT_M3, ENG_R, PROUD)),
]
tx.text(0.0, 0.99, '나사·핀  (바퀴 한쪽 기준 — 로봇 전체는 × 2)', fontsize=18, weight='bold', va='top')
y = 0.84
for n, c, head, body in rows:
    tx.text(0.012, y, n, fontsize=15, weight='bold', color='white', ha='center', va='center',
            bbox=dict(boxstyle='circle,pad=0.22', fc=c, ec='none'))
    tx.text(0.03, y, head, fontsize=14, weight='bold', va='center')
    tx.text(0.03, y - 0.055, body, fontsize=12.5, va='top', linespacing=1.4)
    y -= 0.215
X2 = 0.70
tx.text(X2, 0.99, '조립 순서', fontsize=16, weight='bold', va='top')
tx.text(X2, 0.87,
        '1) 허브 플랜지를 모터 출력부에: ③ 핀 3개 → ② M3 × %d 6개\n'
        '    (머리는 나중에 림 허브가 덮음)\n' % LS +
        '2) 타이어를 뜨거운 물에 데워 림의 열린 쪽 플랜지 ·\n'
        '    돌기 끝(Ø%.0f)을 넘겨 끼우고, 돌기 %d개를 타이어\n'
        '    안쪽 구멍에 맞춤 (안지름이 %.1f %% 늘어나야 함)\n'
        % (P.RIM_FLANGE_D, NL, 100 * (P.RIM_FLANGE_D / P.TYRE_BORE_D - 1)) +
        '3) 림을 허브 플랜지 Ø%.0f 등록 턱(B)에 맞추고,\n'
        '    림 열린 쪽에서 ④ M3 × %d 6개 + 와셔를 인서트에 조임' % (2 * REG_R, RIM_L),
        fontsize=12.5, va='top', linespacing=1.5)

tb = fig.add_axes([0.55, 0.01, 0.44, 0.525]); tb.axis('off'); tb.set_xlim(0, 1); tb.set_ylim(0, 1)
tb.text(0.0, 0.99, '중심 맞춤 (반경 방향 하중을 받는 끼워맞춤)', fontsize=16, weight='bold', va='top')
y = 0.945
for n, txt in (('A', 'Ø11.8 스피곳 (허브 플랜지) → 출력부 Ø12 × 1.5 홈'),
               ('B', 'Ø%.0f × %.0f 등록 턱 (허브 플랜지) → 림 허브 Ø%.1f × %.1f 홈 (반경 틈 %.1f)'
                % (2 * REG_R, REG_L, 2 * REC_R, REC_L, REC_R - REG_R)),
               ('C', 'AKE Ø42 파일럿 → 정강이 Ø42.05 포켓, 깊이 2.5 (고정자 앞면은 판에서 1.5 띄움)')):
    tb.text(0.02, y, n, fontsize=14, weight='bold', color='white', ha='center', va='center',
            bbox=dict(boxstyle='circle,pad=0.22', fc='#2e7d32', ec='none'))
    tb.text(0.055, y, txt, fontsize=12.5, va='center')
    y -= 0.03
tb.text(0.0, 0.845, '(d) 돌기 — 축 방향에서 본 타이어 중심면 단면 (%d개, %d° 간격)' % (NL, 360 // NL), fontsize=16,
        weight='bold', va='top')
tb.text(0.53, 0.79,
        '• 돌기와 타이어 구멍이 같은 각도에 %d개씩\n'
        '• 폭: 돌기 %.0f / 구멍 %.0f → 원주 방향 틈 %.1f씩\n'
        '• 높이: 돌기 %.1f / 구멍 깊이 %.0f → 위로 %.1f 빈 공간\n'
        '• 길이: 돌기 %.1f / 구멍 %.1f → 축 방향 틈 %.1f씩\n'
        '• 바퀴 토크는 돌기 옆면이 고무를 밀어 전함\n'
        '   (빨간 화살표, 마찰이 아님).  검토 25 N·m를\n'
        '   돌기 1/4이 받는다고 보면 개당 %.0f N\n'
        '• 구멍 치수는 판매 사진 추정 → 실물을 재서\n'
        '   params.TYRE_CAVITY · RIM_LUGS를 맞출 것'
        % (NL, LUG_W, CAV_W, (CAV_W - LUG_W) / 2, LUG_H, CAV_D, CAV_D - LUG_H, LUG_L, CAV_L, (CAV_L - LUG_L) / 2,
           F_LUG),
        fontsize=12.5, va='top', linespacing=1.45)
y = 0.52
tb.text(0.0, y, '무게가 지나가는 길 (빨간 선)', fontsize=16, weight='bold', va='top')
steps = ['바닥', '타이어 (통고무) → 안지름 Ø%.0f가 림 안장 링에 앉음 (토크는 돌기 %d개, (d))' % (P.TYRE_BORE_D, NL),
         '안장 링 %.0f t → 웨브 %.0f → %.0f mm → 림 허브 %.0f mm' % (P.RIM_SEAT_T, P.RIM_WEB_T, P.RIM_HUB_T, P.RIM_HUB_T),
         'B 등록 턱 + 맞댄 면 (④ M3 × %d × 6이 조임)' % RIM_L, '허브 플랜지',
         'A 스피곳 + ③ 핀 × 3 + ② M3 × %d × 6' % LS, 'AKE 출력부 → 모터 안 출력 베어링 → 고정자',
         'C Ø42 파일럿 + ① M2.5 × 8 × 6', 'PETG 정강이 → 무릎']
y -= 0.04
for k, st in enumerate(steps):
    tb.text(0.02, y, ('↓ ' if k else '   ') + st, fontsize=12.5, va='top')
    y -= 0.021
y -= 0.015
tb.text(0.0, y, '하중  (calc/strength.md 3절, 검토 = SF 2)', fontsize=16, weight='bold', va='top')
tb.text(0.0, y - 0.035,
        '정지: 로봇 %.2f kg (CAD) → 바퀴당 약 %.0f N, 굽힘 %.0f N × %.0f mm = %.1f N·m\n'
        '계산 하중 (브리프: 로봇 %.0f kg + 짐 %.0f kg): 정지 %.0f N × 5 = 설계 %.0f N, 검토 %.0f N, 옆 %.0f / %.0f N\n'
        '• ② 와셔 지압: 설계 %.0f MPa → 통과, 검토 %.0f MPa (PETG 50) → 초과\n'
        '• ④ 인서트 인발 (검토) %.0f N (약 1000), 와셔 지압 %.0f MPa → 통과\n'
        '• 림 허브·웨브 판 굽힘 (검토) 최대 %.0f MPa (45) → 통과\n'
        '• AKE60 출력 베어링 정격은 비공개 → CubeMars 확인'
        % (ROBOT_KG, W_STATIC, W_STATIC, ARM, W_STATIC * ARM / 1000, P.ROBOT_KG, P.PAYLOAD_KG, S.F_STATIC, S.F_DES,
           S.F_CHK, S.FL_DES, S.FL_CHK, P_OUT['des'], P_OUT['chk'], F_INS, F_INS / WA, S_PLATE),
        fontsize=12.5, va='top', linespacing=1.45)
y = 0.04
tb.text(0.0, y + 0.018, '색', fontsize=14, weight='bold', va='bottom')
for k, (c, txt) in enumerate(((g['petg'], 'PETG 정강이'), (g['flange'], '허브 플랜지'), (g['rim'], 'PETG 림'),
                              (g['tyre'], '타이어'), (g['stator'], 'AKE 고정자'), (g['out'], 'AKE 출력부'),
                              (g['ring'], 'SUS 링'), (g['washer'], '와셔'), (g['insert'], '황동 인서트'),
                              (g['pin'], '평행핀'))):
    cx, cy = (k % 5) * 0.2, y - (k // 5) * 0.02
    tb.add_patch(Rectangle((cx, cy - 0.007), 0.04, 0.014, fc=c, ec='k', lw=0.6))
    tb.text(cx + 0.05, cy, txt, fontsize=11.5, va='center')

fig.savefig(sys.argv[1] if len(sys.argv) > 1 else HERE / 'wheel_output_stack.png', dpi=80)
