"""Wheel joint section with the M3 output screws + washers (upper half) and the Ø3 dowel pins (lower half), and how the
one-piece printed rim (10-31 rev 2) sits on the hub flange: Ø11 x 3 register spigot on the flange into the rim's
Ø11.2 x 3.2 recess, 6 x M3 x 14 8.8 + DIN 125 washers into the flange's heat-set inserts.  Drawn to scale from
fusion/params.py and the hub / rim levels of fusion/legs.py (explanatory figure, not a manufacturing drawing).

    python wheel_joint_section.py            # -> wheel_joint_pins_washers.png next to this script
"""
import math
import pathlib
import sys
from unittest import mock
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / 'fusion'))
# legs.py / layout.py / fasteners.py import cadlib, which loads the Fusion API at import time; only their pure-Python
# levels, hole sizes and frames are used here, so outside Fusion a stub stands in for adsk (no geometry is built)
for _m in ('adsk', 'adsk.core', 'adsk.fusion'):
    sys.modules.setdefault(_m, mock.MagicMock())
import params as P
import legs as LG
import layout as LY
import fasteners as FX
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Polygon
for F in ('/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc', 'C:/Windows/Fonts/malgun.ttf'):
    if pathlib.Path(F).exists():
        font_manager.fontManager.addfont(F); plt.rcParams['font.family'] = font_manager.FontProperties(fname=F).get_name()
        break
plt.rcParams['axes.unicode_minus'] = False
A = P.AKE
def a(y): return A['ring'] - y                       # motor local y -> axial a (0 = stator ring face, + toward the tyre)
def ar(y): return P.Y_RING_WHEEL - y                 # robot y (left wheel) -> the same a
LV = LG.hub_levels()
OUT = a(A['out']); SF = a(A['stator_front']); CF = a(A['can_front'])
FAR = P.WHEEL_PLATE_T - P.POCKET; MF = -P.POCKET; REC = FAR - P.RING_T - P.CBORE_M25[1]
F1, F0 = ar(LV['f1']), ar(LV['f0'])                  # hub flange outer face 7.5, rim face 15.5
HUB0 = ar(LV['f0'] - P.RIM_HUB_T)                    # rim hub inner face 25.5
W_OD, W_ID, W_T = FX.WASHER[3]; DK, K = FX.SHCS[3][:2]
BEAR = ar(LV['out'] - P.HUB_SCREW_SEAT)              # 10.0 washer seat of the output M3 x 14 (counterbore floor)
LS = P.HUB_SCREW_L; ENG = LS - P.HUB_SCREW_SEAT - W_T
HEAD_GAP = F0 - (BEAR + W_T + K)                     # 2.0 output screw head end -> rim hub face
OD, ID = P.RING_STATOR; RP = P.AKE_PCD_RING / 2; RO = P.AKE_PCD_OUT / 2
REG_R, REG_L = P.RIM_REG; REC_R, REC_L = REG_R + P.RIM_REG_GAP[0], REG_L + P.RIM_REG_GAP[1]   # legs.hub_flange spigot / legs.rim recess
RH, RIM_L, INS_L = 18.0, 14, 5.7                     # legs.HUB_SCREWS radius, 'FX ISO 4762 M3x14 rim', insert length
RSEAT = ar(LV['f0'] - P.RIM_SCREW_SEAT)              # 23.5 rim washer seat
RTIP = RSEAT + W_T - RIM_L                           # 10.0
RCUT = 27.0                                          # rim hub drawn up to this radius (break line)
# angle of the rim screw next to the lower-half dowel, in the motor frame (the frames of layout.py)
_m = LY.frames('wheel', 1)['motor']
_rob = lambda p: math.degrees(math.atan2(p[2] - P.R_WHEEL, p[0])) % 360
PIN_A = 90                                           # P.AKE_OUT_PIN entry drawn in the lower half
_pin_r = _rob(LY.motor_points(_m, RO, [PIN_A], A['out'])[0])
_d = min(((math.degrees(t) - _pin_r + 180) % 360 - 180 for t in LG.HUB_SCREWS), key=abs)
RIM_A = PIN_A + _d                                   # 95: 5 deg beside the dowel
g = dict(stator='#b8b8b8', rotor='#7f9fbf', out='#d9b38c', petg='#9fd29f', ring='#8a9bb0', hub='#e07b39',
         rim='#f2a65a', screw='#3a3a3a', washer='#d4af37', pin='#c0392b', insert='#b8860b')
fig, ax = plt.subplots(figsize=(17, 12.5)); ax.set_aspect('equal'); ax.axis('off')
def poly(pts, c, s=1, **kw): ax.add_patch(Polygon([(x, s * y) for x, y in pts], fc=c, ec=kw.pop('ec', 'k'), lw=kw.pop('lw', 0.9), **kw))
def rect(x0, x1, r0, r1, c, s=1, **kw): poly([(x0, r0), (x1, r0), (x1, r1), (x0, r1)], c, s, **kw)
def zig(p0, p1, n=3, amp=0.5):
    (x0, y0), (x1, y1) = p0, p1
    return [(x0 + (x1 - x0) * k / (2 * n) + amp * (1 if k % 2 else -1), y0 + (y1 - y0) * k / (2 * n)) for k in range(1, 2 * n)]
for s in (1, -1):
    poly([(0, 13.6), (0, 21), (-3, 21), (-3, 20.3), (SF, 20.3), (SF, 30.75), (-14, 30.75), (-14, 13.6)], g['stator'], s)
    rect(-14, CF, 31.2, A['can_r'], g['rotor'], s)
    poly([(FAR, 15), (REC, 15), (REC, P.STATOR_RECESS_D / 2), (FAR, P.STATOR_RECESS_D / 2), (FAR, 46), (-14, 46),
          (-14, P.CAN_CLEAR_R), (MF, P.CAN_CLEAR_R), (MF, 21), (0, 21), (0, 15)], g['petg'], s)
    rect(REC, REC + P.RING_T, ID / 2, OD / 2, g['ring'], s, lw=1.2)
def sym(pts):                                        # upper-half outline from the axis back to the axis -> both halves
    return pts + [(x, -r) for x, r in reversed(pts)]
# output carrier (spins), Ø12 x 1.5 recess
poly(sym([(-14, 13.5), (OUT, 13.5), (OUT, 6), (OUT - 1.5, 6), (OUT - 1.5, 0)]) + [(-14, -13.5)], g['out'])
# one-piece rim (partly): 10 mm hub face to face on the flange, Ø11.2 x 3.2 register recess
poly(sym([(F0, 0), (F0, RCUT)] + zig((F0, RCUT), (HUB0, RCUT)) + [(HUB0, RCUT), (HUB0, 0)]), g['rim'])
poly(sym([(F0, 0), (F0, REC_R), (F0 + REC_L, REC_R), (F0 + REC_L, 0)]), 'white', lw=0.6)
# hub flange: Ø11.8 x 1.4 spigot, Ø28 boss, Ø46 disc, Ø11 x 3 register spigot into the rim
poly(sym([(OUT - 1.4, 0), (OUT - 1.4, 5.9), (OUT, 5.9), (OUT, 14), (F1, 14), (F1, 23), (F0, 23), (F0, REG_R),
          (F0 + REG_L, REG_R), (F0 + REG_L, 0)]), g['hub'])
# upper half: M3 x 14 at r 10 with washer, counterbore Ø7.5 from the washer seat to the rim face (closed by the rim)
rect(BEAR, F0, RO - 3.75, RO + 3.75, 'white', lw=0.6)
rect(OUT, BEAR, RO - LG.M3_CLEAR / 2, RO + LG.M3_CLEAR / 2, 'white', lw=0.6)
rect(BEAR, BEAR + W_T, RO - W_OD / 2, RO + W_OD / 2, g['washer'], lw=0.8)
rect(BEAR + W_T, BEAR + W_T + K, RO - DK / 2, RO + DK / 2, g['screw'])
rect(BEAR + W_T - LS, BEAR + W_T, RO - 1.25, RO + 1.25, g['screw'])
# lower half: dowel Ø3 x 10 at r 10 (3 mm in the output, 7 mm in the flange); stator M2.5 x 8 at r 18.25 (same angle)
rect(OUT - 3, OUT + 7, -RO - 1.5, -RO + 1.5, g['pin'], lw=1.0)
rect(REC, REC + P.RING_T, -RP - 1.4, -RP + 1.4, 'white', lw=0)
rect(-3.2, REC + P.RING_T, -RP - 1.25, -RP + 1.25, g['screw'], lw=0.6)
rect(REC + P.RING_T, REC + P.RING_T + 1.5, -RP - 2.25, -RP + 2.25, g['screw'], lw=0.6)
# lower half: rim M3 x 14 8.8 + washer at r 18 (5 deg beside the dowel), brass insert in the flange
rect(RSEAT, HUB0 + 0.15, -RH - 3.8, -RH + 3.8, 'white', ec='none')
ax.plot([HUB0 + 0.15, RSEAT, RSEAT, HUB0 + 0.15], [-RH - 3.8, -RH - 3.8, -RH + 3.8, -RH + 3.8], 'k-', lw=0.6)
rect(F0, RSEAT, -RH - LG.M3_CLEAR / 2, -RH + LG.M3_CLEAR / 2, 'white', lw=0.6)
rect(F0 - 6.0, F0, -RH - LG.INSERT_M3 / 2, -RH + LG.INSERT_M3 / 2, 'white', lw=0.6)
rect(F0 - 0.1 - INS_L, F0 - 0.1, -RH - LG.INSERT_M3 / 2, -RH + LG.INSERT_M3 / 2, g['insert'], lw=0.8)
rect(RSEAT, RSEAT + W_T, -RH - W_OD / 2, -RH + W_OD / 2, g['washer'], lw=0.8)
rect(RSEAT + W_T, RSEAT + W_T + K, -RH - DK / 2, -RH + DK / 2, g['screw'])
rect(RTIP, RSEAT + W_T, -RH - 1.25, -RH + 1.25, g['screw'])
# axis and half labels
ax.plot([-15, HUB0 + 4], [0, 0], 'k-.', lw=0.7)
ax.text(-15, 1.0, '관절(바퀴) 회전축', fontsize=9)
HB = dict(fc='white', ec='none', alpha=0.85, pad=0.4)
ax.text(-15, 47.0, '위쪽 절반: 출력 M3 나사 자리(0°)를 지나는 단면 (고정자 M2.5는 ±30°라 이 면에 없음)', fontsize=11,
        weight='bold', va='bottom', bbox=HB)
ax.text(-15, -47.0, '아래쪽 절반: 평행핀 자리(%d°)를 지나는 단면 — 고정자 M2.5도 같은 각도, 림 M3는 %d° 옆(%d°).\n'
        '핀 3개와 출력 나사 6개는 같은 원(PCD 20)에서 각도만 다름' % (PIN_A, abs(RIM_A - PIN_A), RIM_A),
        fontsize=11, weight='bold', va='top', bbox=HB)
ax.text(-9, 25, 'AKE 고정자', ha='center', fontsize=10); ax.text(-9, -25, 'AKE 고정자', ha='center', fontsize=10)
ax.text(-6, 3.5, '모터 출력부\n(회전)', ha='center', fontsize=10)
ax.text(-5, 41.5, 'PETG 정강이 바퀴 판', ha='center', fontsize=10)
ax.annotate('허브 플랜지 (PETG 100 %, 회전)', xy=((F1 + F0) / 2, 21.5), xytext=(F1 - 3.5, 34.0), fontsize=10, va='center',
            arrowprops=dict(arrowstyle='->', lw=0.9, relpos=(0.3, 0)), bbox=dict(fc='white', ec='none', alpha=0.85, pad=0.3))
ax.text((F0 + HUB0) / 2 + 1.2, 23.0, '림 허브\n(일부)', ha='center', va='center', fontsize=10)
def call(xy, xt, txt, c='k'):
    ax.annotate(txt, xy=xy, xytext=xt, fontsize=11, color=c, va='center',
                arrowprops=dict(arrowstyle='->', color=c, lw=1.0, relpos=(0, 0.5)),
                bbox=dict(fc='white', ec='none', alpha=0.85, pad=0.4))
X = HUB0 + 7
call((BEAR + 0.25, RO + 3.2), (X, 36), '② 평와셔 DIN 125 M3 (Ø7 × 0.5, 노란색) — 6개/관절\n   나사 머리 힘을 넓게 펴서 PETG가 눌리지 않게,\n'
     '   두께 0.5만큼 나사가 덜 들어가 탭 바닥(5 mm)에 안 닿게', '#9a7b00')
call((BEAR + 2.0, RO - 1.0), (X, 24), '③ M3 × %d (12.9) — 6개/관절\n   허브 플랜지를 모터 출력부에 조임, 출력부 탭에 %.1f mm 물림.\n'
     '   카운터보어 Ø7.5는 림 허브가 덮음 (머리 끝 → 림 면 %.1f mm)' % (LS, ENG, HEAD_GAP))
call((F0 + 1.5, REG_R - 0.4), (X, 12), '⑥ Ø%.0f × %.0f 등록 턱 (허브 플랜지) → 림 허브 Ø%.1f × %.1f 홈 (반경 틈 %.1f)\n'
     '   림 중심 맞춤 + 바퀴 반경 하중.  림 쪽 면이 평면이라 그 면을 바닥에 두고 출력'
     % (2 * REG_R, REG_L, 2 * REC_R, REC_L, REC_R - REG_R), '#2e7d32')
call((OUT + 2, -RO), (X, -5), '④ 평행핀 ISO 8734 Ø3 × 10 (m6, 경화) — 3개/관절\n   출력부 핀 구멍에 3 mm, 허브 플랜지에 7 mm 끼움.\n'
     '   두 부품 위치를 정확히 맞추고(나사 구멍은 0.2 mm 여유),\n   모터 토크를 나사와 함께 받아 플랜지가 돌며 밀리지 않게', '#c0392b')
call((OUT - 0.7, -3.0), (X, -16.5), '⑤ Ø11.8 스피곳: 플랜지가 출력부 홈(Ø12)에 들어가 중심 맞춤')
call((RSEAT + 2.0, -RH + 1.0), (X, -25), '⑦ 림 M3 × %d (8.8) + 평와셔 DIN 125 — 6개/바퀴, r %.0f\n'
     '   림 허브를 허브 플랜지 황동 인서트(M3 × %.1f)에 조임, 인서트 %.1f mm 물림.\n'
     '   와셔 자리는 플랜지 면에서 %.0f, 머리 끝이 림 허브 면보다 %.1f 나옴'
     % (RIM_L, RH, INS_L, F0 - RTIP, P.RIM_SCREW_SEAT, RSEAT + W_T + K - HUB0))
call((REC + P.RING_T + 0.75, -RP - 1.5), (X, -35.5), '① M2.5 × 8 저두 + 클램프 링 — 정강이 판을 모터 고정자에 조임 (6개/관절)')
ax.text(X, 43.5, 'hip·knee 구동 보스도 같은 방식: 평행핀 3 + M3 × 16 6 + 평와셔 6 (와셔 자리는 출력면에서 %.1f).\n'
        '단, 거기는 와셔 밑에 출력 클램프 링(Ø%.0f, %.0f mm)이 한 장 더 깔린다.'
        % (P.BEARING_FACE, P.RING_OUT[0], P.RING_T), fontsize=10, va='center')
ax.set_xlim(-16, X + 52); ax.set_ylim(-53, 50)
ax.set_title('바퀴 관절 단면 — 평행핀·평와셔·나사 배치, 림 허브 체결 (모델 치수, 실제 비율)', fontsize=15, loc='left')
fig.savefig(sys.argv[1] if len(sys.argv) > 1 else HERE / 'wheel_joint_pins_washers.png', dpi=100, bbox_inches='tight')
