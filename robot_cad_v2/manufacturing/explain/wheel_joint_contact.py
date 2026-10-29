"""Wheel joint half section: what touches (Ø42 pilot in the H7 pocket) and what does not (stator front, motor can),
to scale from fusion/params.py.  Explanatory figure, not a manufacturing drawing; wheel_joint_section.py adds the
screws, washers and dowel pins.

    python wheel_joint_contact.py    # -> wheel_joint_contact.png next to this script
"""
import pathlib
import sys
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / 'fusion'))
import params as P
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Polygon, Rectangle
F = '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc'
font_manager.fontManager.addfont(F); plt.rcParams['axes.unicode_minus'] = False; plt.rcParams['font.family'] = font_manager.FontProperties(fname=F).get_name()
A = P.AKE
# axial coordinate a [mm]: 0 = AKE stator ring face = pocket floor, + toward the plate's far side (tyre / shank)
def a(y): return A['ring'] - y          # motor local y (+ to the back) -> a
far = P.WHEEL_PLATE_T - P.POCKET; mface = -P.POCKET; rec = far - P.RING_T - P.CBORE_M25[1]
OD, ID = P.RING_STATOR; RP = P.AKE_PCD_RING / 2
fig, ax = plt.subplots(figsize=(15, 9)); ax.set_aspect('equal'); ax.axis('off')
g = dict(stator='#b8b8b8', rotor='#7f9fbf', petg='#9fd29f', ring='#8a9bb0', boss='#e07b39', screw='#444444')
# stator (fixed): pilot r 13.6..21 from a 0 to -3, body behind it, front face at a(stator_front) r 20.3..30.75
sf = a(A['stator_front']); cf = a(A['can_front'])
ax.add_patch(Polygon([(0, 13.6), (0, 21), (-A['pilot_len'], 21), (-A['pilot_len'], 20.3), (sf, 20.3), (sf, 30.75),
                      (-14, 30.75), (-14, 13.6)], fc=g['stator'], ec='k', lw=1.0))
# rotor bell (spinning): r 30.75..34.5 from can_front back
ax.add_patch(Polygon([(cf, 31.2), (cf, A['can_r']), (-14, A['can_r']), (-14, 31.2)], fc=g['rotor'], ec='k', lw=1.0))
# output carrier (spins with the hub): r < 13.5, face a = +0.5
ax.add_patch(Polygon([(a(A['out']), 10), (a(A['out']), 13.5), (-14, 13.5), (-14, 10)], fc=g['boss'], ec='k', lw=0.8, alpha=0.6))
# PETG plate + housing around the motor (can cavity r 37)
ax.add_patch(Polygon([(far, 15), (rec, 15), (rec, P.STATOR_RECESS_D / 2), (far, P.STATOR_RECESS_D / 2), (far, 46),
                      (-14, 46), (-14, P.CAN_CLEAR_R), (mface, P.CAN_CLEAR_R), (mface, 21), (0, 21), (0, 15)],
                     fc=g['petg'], ec='k', lw=1.0))
# clamp ring, screw
ax.add_patch(Rectangle((rec, ID / 2), P.RING_T, (OD - ID) / 2, fc=g['ring'], ec='k', lw=1.2))
ax.add_patch(Rectangle((rec, RP - 1.4), P.RING_T, 2.8, fc='white', ec='none'))
ax.add_patch(Rectangle((-3.2, RP - 1.25), rec + P.RING_T + 3.2, 2.5, fc=g['screw'], ec='k', lw=0.6))
ax.add_patch(Rectangle((rec + P.RING_T, RP - 2.25), 1.5, 4.5, fc=g['screw'], ec='k', lw=0.6))
# hub flange boss Ø28 through the Ø30 hole (spins)
ax.add_patch(Polygon([(a(A['out']), 10), (a(A['out']), 14), (far + 1.0, 14), (far + 1.0, 26), (far + 7, 26), (far + 7, 10)],
                     fc=g['boss'], ec='k', lw=0.8))
# contact highlight: pilot end face (r 15..21 at a 0) and pilot rim (r 21, a -2.5..0)
ax.plot([0, 0], [15, 21], color='red', lw=4, solid_capstyle='butt')
ax.plot([mface, 0], [21, 21], color='red', lw=4, solid_capstyle='butt')
T = dict(fontsize=11)
ax.text(-9, 25, 'AKE 고정자\n(움직이지 않음)', ha='center', **T)
ax.text(-11.2, 32.8, '모터 캔(회전)', ha='center', va='center', fontsize=8, color='white')
ax.text(-5, 42, 'PETG 정강이 (바퀴 판 + 모터를 감싸는 하우징)', ha='center', **T)
ax.text(far + 4, 18, '허브 플랜지\nØ28 보스\n(바퀴와 회전)', ha='center', fontsize=10)
ax.annotate('① 닿는 곳(빨강): Ø42 파일럿 끝면이 포켓 바닥에 앉고\n    Ø42 둘레가 포켓 벽(H7)에 끼워짐 → 축 맞춤 + 조임 힘을 받는 곳',
            xy=(0, 18.5), xytext=(far + 10, 29.5), color='red', fontsize=11, va='center',
            arrowprops=dict(arrowstyle='->', color='red'))
ax.annotate('② 고정자 앞면과 PETG 사이 1.5 mm 틈 (닿지 않음)', xy=((sf + mface) / 2, 25), xytext=(far + 10, 37),
            fontsize=11, va='center', arrowprops=dict(arrowstyle='->'))
ax.annotate('③ 돌아가는 모터 캔과 PETG 하우징 사이 2.5 mm 틈', xy=(-11, (A['can_r'] + P.CAN_CLEAR_R) / 2), xytext=(far + 10, 44),
            fontsize=11, va='center', arrowprops=dict(arrowstyle='->'))
ax.annotate('④ 나사가 링 → PETG 웹 → 고정자를 함께 조임', xy=(rec + P.RING_T + 0.7, RP), xytext=(far + 10, RP - 2),
            fontsize=11, va='center', arrowprops=dict(arrowstyle='->'))
ax.annotate('클램프 링 (2 mm)', xy=(rec + 1, 21.5), xytext=(far + 10, 23.5), fontsize=11, va='center',
            arrowprops=dict(arrowstyle='->'))
ax.plot([-14, far + 7], [10, 10], 'k-.', lw=0.5)
ax.text(-14, 9.0, '↓ 이 아래로 r = 10 mm 더 가면 관절 회전축', fontsize=9)
ax.set_xlim(-15, far + 45); ax.set_ylim(8, 47.5)
ax.set_title('바퀴 관절 단면 (모델 치수, 실제 비율) — 축을 지나 자른 위쪽 일부', fontsize=14, loc='left')
fig.savefig(sys.argv[1] if len(sys.argv) > 1 else HERE / 'wheel_joint_contact.png', dpi=110, bbox_inches='tight')
