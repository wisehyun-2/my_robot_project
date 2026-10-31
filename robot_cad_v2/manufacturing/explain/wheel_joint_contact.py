"""Wheel joint half section: what touches and what does not, to scale from fusion/params.py and the hub / rim levels of
fusion/legs.py.  Static side: the AKE Ø42 pilot in the shank pocket (stator front and motor can stand clear).  Turning
side (10-31 rev 2 wheel): the hub flange boss on the motor output face, the one-piece rim's 10 mm hub face to face on
the flange (Ø11 x 3 register spigot in a Ø11.2 x 3.2 recess, 6 x M3 x 14 into heat-set inserts).  Explanatory figure,
not a manufacturing drawing; wheel_joint_section.py adds the output screws, washers and dowel pins.

    python wheel_joint_contact.py    # -> wheel_joint_contact.png next to this script
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
from matplotlib.patches import Polygon, Rectangle
for F in ('C:/Windows/Fonts/malgun.ttf', '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc'):
    if pathlib.Path(F).exists():
        font_manager.fontManager.addfont(F)
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=F).get_name()
        break
plt.rcParams['axes.unicode_minus'] = False
A = P.AKE
# axial coordinate a [mm]: 0 = AKE stator ring face = pocket floor, + toward the plate's far side (tyre / shank)
def a(y): return A['ring'] - y          # motor local y (+ to the back) -> a
def ar(y): return P.Y_RING_WHEEL - y    # robot y (left wheel) -> a
LV = LG.hub_levels()
far = P.WHEEL_PLATE_T - P.POCKET; mface = -P.POCKET; rec = far - P.RING_T - P.CBORE_M25[1]
OD, ID = P.RING_STATOR; RP = P.AKE_PCD_RING / 2
OUT = a(A['out']); F1, F0 = ar(LV['f1']), ar(LV['f0']); HUB0 = ar(LV['f0'] - P.RIM_HUB_T)
FL_R, BOSS_R, SPIG_R, SPIG_L = 23.0, P.HUB_BOSS_D / 2, 5.9, 1.4      # legs.hub_flange
REG_R, REG_L = P.RIM_REG; REC_R, REC_L = REG_R + P.RIM_REG_GAP[0], REG_L + P.RIM_REG_GAP[1]  # flange spigot / rim recess
W_OD, W_ID, W_T = FX.WASHER[3]; DK, K = FX.SHCS[3][:2]
RH, RIM_L, INS_L = 18.0, 14, 5.7
RSEAT = ar(LV['f0'] - P.RIM_SCREW_SEAT); RTIP = RSEAT + W_T - RIM_L
RCUT = 30.0                             # rim hub drawn up to this radius (break line)
# the section: through a stator screw; the nearest rim screw lies a few degrees beside it (layout.py frames)
_m = LY.frames('wheel', 1)['motor']
_rob = lambda p: math.degrees(math.atan2(p[2] - P.R_WHEEL, p[0])) % 360
SEC_A = 30                              # an entry of P.AKE_RING_M25 (no dowel or output screw at this angle)
_s = _rob(LY.motor_points(_m, RP, [SEC_A], A['ring'])[0])
DRIM = min(((math.degrees(t) - _s + 180) % 360 - 180 for t in LG.HUB_SCREWS), key=abs)   # 5 deg
fig, ax = plt.subplots(figsize=(17, 9.5)); ax.set_aspect('equal'); ax.axis('off')
g = dict(stator='#b8b8b8', rotor='#7f9fbf', petg='#9fd29f', ring='#8a9bb0', hub='#e07b39', out='#d9b38c', rim='#f2a65a',
         screw='#444444', washer='#d4af37', insert='#b8860b')
def poly(pts, c, **kw): ax.add_patch(Polygon(pts, fc=c, ec=kw.pop('ec', 'k'), lw=kw.pop('lw', 1.0), **kw))
def rect(x0, x1, r0, r1, c, **kw): poly([(x0, r0), (x1, r0), (x1, r1), (x0, r1)], c, **kw)
# stator (fixed): pilot r 13.6..21 from a 0 to -3, body behind it, front face at a(stator_front) r 20.3..30.75
sf = a(A['stator_front']); cf = a(A['can_front'])
poly([(0, 13.6), (0, 21), (-A['pilot_len'], 21), (-A['pilot_len'], 20.3), (sf, 20.3), (sf, 30.75), (-14, 30.75),
      (-14, 13.6)], g['stator'])
# rotor bell (spinning): r 30.75..34.5 from can_front back
poly([(cf, 31.2), (cf, A['can_r']), (-14, A['can_r']), (-14, 31.2)], g['rotor'])
# output carrier (spins with the wheel): face a = +0.5, Ø12 x 1.5 recess
poly([(-14, 0), (OUT - 1.5, 0), (OUT - 1.5, 6.0), (OUT, 6.0), (OUT, 13.5), (-14, 13.5)], g['out'], lw=0.8)
# PETG plate + housing around the motor (can cavity r 37)
poly([(far, 15), (rec, 15), (rec, P.STATOR_RECESS_D / 2), (far, P.STATOR_RECESS_D / 2), (far, 46), (-14, 46),
      (-14, P.CAN_CLEAR_R), (mface, P.CAN_CLEAR_R), (mface, 21), (0, 21), (0, 15)], g['petg'])
# clamp ring, stator screw (DIN 7984 M2.5 x 8)
ax.add_patch(Rectangle((rec, ID / 2), P.RING_T, (OD - ID) / 2, fc=g['ring'], ec='k', lw=1.2))
ax.add_patch(Rectangle((rec, RP - 1.4), P.RING_T, 2.8, fc='white', ec='none'))
ax.add_patch(Rectangle((-3.2, RP - 1.25), rec + P.RING_T + 3.2, 2.5, fc=g['screw'], ec='k', lw=0.6))
ax.add_patch(Rectangle((rec + P.RING_T, RP - 2.25), 1.5, 4.5, fc=g['screw'], ec='k', lw=0.6))
# one-piece rim (partly): hub face on the flange, Ø11.2 x 3.2 recess, rim screw counterbore / hole
poly([(F0, 0), (F0, RCUT)] + [(F0 + (HUB0 - F0) * k / 6, RCUT + (0.6 if k % 2 else -0.6)) for k in range(1, 6)]
     + [(HUB0, RCUT), (HUB0, 0)], g['rim'])                                     # break line at RCUT
rect(F0, F0 + REC_L, 0, REC_R, 'white', lw=0.6)
rect(RSEAT, HUB0 + 0.15, RH - 3.8, RH + 3.8, 'white', ec='none')
ax.plot([HUB0 + 0.15, RSEAT, RSEAT, HUB0 + 0.15], [RH - 3.8, RH - 3.8, RH + 3.8, RH + 3.8], 'k-', lw=0.6)
rect(F0, RSEAT, RH - LG.M3_CLEAR / 2, RH + LG.M3_CLEAR / 2, 'white', lw=0.6)
# hub flange (PETG 100 %, spins): Ø11.8 x 1.4 spigot, Ø28 boss through the Ø30 hole, Ø46 disc, Ø11 x 3 register
poly([(OUT - SPIG_L, 0), (OUT - SPIG_L, SPIG_R), (OUT, SPIG_R), (OUT, BOSS_R), (F1, BOSS_R), (F1, FL_R), (F0, FL_R),
      (F0, REG_R), (F0 + REG_L, REG_R), (F0 + REG_L, 0)], g['hub'])
rect(F0 - 6.0, F0, RH - LG.INSERT_M3 / 2, RH + LG.INSERT_M3 / 2, 'white', lw=0.6)
rect(F0 - 0.1 - INS_L, F0 - 0.1, RH - LG.INSERT_M3 / 2, RH + LG.INSERT_M3 / 2, g['insert'], lw=0.8)
# rim screw ISO 4762 M3 x 14 8.8 + DIN 125 washer
rect(RSEAT, RSEAT + W_T, RH - W_OD / 2, RH + W_OD / 2, g['washer'], lw=0.8)
rect(RSEAT + W_T, RSEAT + W_T + K, RH - DK / 2, RH + DK / 2, g['screw'])
rect(RTIP, RSEAT + W_T, RH - 1.25, RH + 1.25, g['screw'])
# contact highlights (red): pilot end face + Ø42 rim, flange boss on the output face, rim hub face on the flange
RED = dict(color='red', lw=4, solid_capstyle='butt', zorder=6)
ax.plot([0, 0], [15, 21], **RED)
ax.plot([mface, 0], [21, 21], **RED)
ax.plot([OUT, OUT], [6.0, 13.5], **RED)
ax.plot([F0, F0], [REC_R, RH - LG.INSERT_M3 / 2], **RED)
ax.plot([F0, F0], [RH + LG.INSERT_M3 / 2, FL_R], **RED)
T = dict(fontsize=11)
ax.text(-9, 25, 'AKE 고정자\n(움직이지 않음)', ha='center', **T)
ax.text(-11.2, 32.8, '모터 캔(회전)', ha='center', va='center', fontsize=8, color='white')
ax.text(-5, 42, 'PETG 정강이 (바퀴 판 + 모터를 감싸는 하우징)', ha='center', **T)
ax.text(-7, 7.0, 'AKE 출력부\n(바퀴와 회전)', ha='center', va='center', **T)
ax.text(8.6, 9.2, '허브 플랜지\n(PETG 100 %, 회전)', ha='center', va='center', fontsize=10, linespacing=1.15)
ax.text((F0 + HUB0) / 2 + 1.0, 26.5, '림 허브 (일부, 회전)', ha='center', va='center', fontsize=10)
X = HUB0 + 8.0
def call(txt, xy, y, c='k'):
    ax.annotate(txt, xy=xy, xytext=(X, y), color=c, fontsize=11, va='center',
                arrowprops=dict(arrowstyle='->', color=c, lw=1.0, relpos=(0, 0.5)),
                bbox=dict(fc='white', ec='none', alpha=0.85, pad=0.4))
call('① 닿는 곳(빨강): Ø42 파일럿 끝면이 포켓 바닥에 앉고\n    Ø42 둘레가 포켓 벽(Ø42.05)에 끼워짐 → 축 맞춤 + 조임 힘을 받는 곳',
     (0, 18.5), 34.0, 'red')
call('② 고정자 앞면과 PETG 사이 %.1f mm 틈 (닿지 않음)' % (mface - sf), ((sf + mface) / 2, 25), 40.5)
call('③ 돌아가는 모터 캔과 PETG 하우징 사이 %.1f mm 틈' % (P.CAN_CLEAR_R - A['can_r']),
     (-11, (A['can_r'] + P.CAN_CLEAR_R) / 2), 46.0)
call('④ 나사(M2.5 × 8)가 클램프 링 %.0f mm → PETG 웹 → 고정자를 함께 조임' % P.RING_T, (rec + P.RING_T + 0.7, RP + 1.0),
     28.0)
call('⑤ 돌아가는 허브 플랜지와 정강이 판 사이: 면 %.1f mm, Ø28 보스 ↔ Ø30 구멍 반경 %.1f mm 틈 (닿지 않음)'
     % (F1 - far, ID / 2 - BOSS_R), ((far + F1) / 2, FL_R - 0.4), 22.6)
call('⑥ 닿는 곳(빨강): 림 허브 면이 허브 플랜지 면(Ø%.0f)에 맞닿음 —\n'
     '    림 M3 × %d 6개 + 와셔가 플랜지의 황동 인서트에 조임 (이 단면에서 %d° 옆)' % (2 * FL_R, RIM_L, abs(DRIM)),
     (F0 + 0.1, 11.0), 12.5, 'red')
call('⑦ Ø%.0f × %.0f 등록 턱 ↔ 림 Ø%.1f × %.1f 홈: 반경·바닥 틈 %.1f — 끼울 때 림 중심을 잡고,\n'
     '    반경 하중이 걸리면 한쪽 벽이 닿아 받음' % (2 * REG_R, REG_L, 2 * REC_R, REC_L, REC_R - REG_R),
     (F0 + REG_L - 0.3, REG_R + 0.1), 4.5, '#2e7d32')
call('⑧ 닿는 곳(빨강): 허브 플랜지 보스 면이 모터 출력면에 맞닿음 (M3 × %d 12.9 6개가 조임)' % P.HUB_SCREW_L,
     (OUT, 12.0), -2.5, 'red')
ax.annotate('클램프 링 (%.0f mm)' % P.RING_T, xy=(rec + 1, 22.0), xytext=(-2.0, 50.0), fontsize=11, va='center',
            ha='center', arrowprops=dict(arrowstyle='->'), bbox=dict(fc='white', ec='none', alpha=0.85, pad=0.4))
ax.plot([-14, HUB0 + 3], [0, 0], 'k-.', lw=0.6)
ax.text(-14, -1.0, '관절(바퀴) 회전축', fontsize=9, va='top')
ax.set_xlim(-15, X + 72); ax.set_ylim(-5.5, 52)
ax.set_title('바퀴 관절 단면 (모델 치수, 실제 비율) — 축을 지나 자른 위쪽 절반: 고정자 M2.5 자리, 림 M3는 %d° 옆을 겹쳐 그림'
             % abs(DRIM), fontsize=14, loc='left')
fig.savefig(sys.argv[1] if len(sys.argv) > 1 else HERE / 'wheel_joint_contact.png', dpi=110, bbox_inches='tight')
