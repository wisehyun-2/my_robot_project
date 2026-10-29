"""Wheel joint section with the M3 output screws + washers (upper half) and the Ø3 dowel pins (lower half),
drawn to scale from fusion/params.py (explanatory figure, not a manufacturing drawing).

    python wheel_joint_section.py            # -> wheel_joint_pins_washers.png next to this script
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
font_manager.fontManager.addfont(F); plt.rcParams['font.family'] = font_manager.FontProperties(fname=F).get_name()
A = P.AKE
def a(y): return A['ring'] - y                       # motor local y -> axial a (0 = stator ring face, + toward the tyre)
OUT = a(A['out']); SF = a(A['stator_front']); CF = a(A['can_front'])
FAR = P.WHEEL_PLATE_T - P.POCKET; MF = -P.POCKET; REC = FAR - P.RING_T - P.CBORE_M25[1]
F1 = FAR + 1.0; F0 = F1 + 8.0; BEAR = OUT + P.BEARING_FACE; H0 = F0 + 6.0
OD, ID = P.RING_STATOR; RP = P.AKE_PCD_RING / 2; RO = P.AKE_PCD_OUT / 2
g = dict(stator='#b8b8b8', rotor='#7f9fbf', out='#d9b38c', petg='#9fd29f', ring='#8a9bb0', hub='#e07b39',
         hex='#f2a65a', screw='#3a3a3a', washer='#d4af37', pin='#c0392b')
fig, ax = plt.subplots(figsize=(17, 12.5)); ax.set_aspect('equal'); ax.axis('off')
def poly(pts, c, s=1, **kw): ax.add_patch(Polygon([(x, s * y) for x, y in pts], fc=c, ec='k', lw=kw.pop('lw', 0.9), **kw))
def rect(x0, x1, r0, r1, c, s=1, **kw): poly([(x0, r0), (x1, r0), (x1, r1), (x0, r1)], c, s, **kw)
for s in (1, -1):
    poly([(0, 13.6), (0, 21), (-3, 21), (-3, 20.3), (SF, 20.3), (SF, 30.75), (-14, 30.75), (-14, 13.6)], g['stator'], s)
    rect(-14, CF, 31.2, A['can_r'], g['rotor'], s)
    poly([(FAR, 15), (REC, 15), (REC, P.STATOR_RECESS_D / 2), (FAR, P.STATOR_RECESS_D / 2), (FAR, 46), (-14, 46),
          (-14, P.CAN_CLEAR_R), (MF, P.CAN_CLEAR_R), (MF, 21), (0, 21), (0, 15)], g['petg'], s)
    rect(REC, REC + P.RING_T, ID / 2, OD / 2, g['ring'], s, lw=1.2)
# output carrier (spins), Ø12 x 1.5 recess
poly([(-14, -13.5), (OUT, -13.5), (OUT, -6), (OUT - 1.5, -6), (OUT - 1.5, 6), (OUT, 6), (OUT, 13.5), (-14, 13.5)], g['out'])
# hub flange: boss Ø28 + spigot Ø11.8 + disc Ø46, M5 head pocket Ø9.5, register recess Ø20.2
poly([(OUT - 1.4, -5.9), (OUT, -5.9), (OUT, -14), (F1, -14), (F1, -23), (F0, -23), (F0, -10.1), (F0 - 1.6, -10.1), (F0 - 1.6, -4.75),
      (F0 - 7, -4.75), (F0 - 7, 4.75), (F0 - 1.6, 4.75), (F0 - 1.6, 10.1), (F0, 10.1), (F0, 23), (F1, 23), (F1, 14), (OUT, 14),
      (OUT, 5.9), (OUT - 1.4, 5.9)], g['hub'], 1)
# hex hub (partly): register spigot Ø20 in the recess
poly([(F0 - 1.5, -10), (F0, -10), (F0, -23), (H0, -23), (H0, 23), (F0, 23), (F0, 10), (F0 - 1.5, 10)], g['hex'])
# M5 axle bolt (head in the flange pocket)
rect(F0 - 6.5, F0 - 1.5, -4.25, 4.25, g['screw']); rect(F0 - 1.5, H0 + 2, -2.5, 2.5, g['screw'])
# upper half: M3 x 16 at r 10 with washer, counterbore Ø7.5 down to BEAR; stator M2.5 x 8 at r 18.25
rect(BEAR, F0, RO - 3.75, RO + 3.75, 'white', lw=0.6)
rect(BEAR, BEAR + 0.5, RO - 3.5, RO + 3.5, g['washer'], lw=0.8)
rect(BEAR + 0.5, BEAR + 3.5, RO - 2.75, RO + 2.75, g['screw'])
rect(BEAR + 0.5 - 16, BEAR + 0.5, RO - 1.25, RO + 1.25, g['screw'])
rect(REC, REC + P.RING_T, RP - 1.4, RP + 1.4, 'white', lw=0)
rect(-3.2, REC + P.RING_T, RP - 1.25, RP + 1.25, g['screw'], lw=0.6)
rect(REC + P.RING_T, REC + P.RING_T + 1.5, RP - 2.25, RP + 2.25, g['screw'], lw=0.6)
# lower half: dowel Ø3 x 10 at r 10, 3 mm in the output, 7 mm in the flange
rect(OUT - 3, OUT + 7, -RO - 1.5, -RO + 1.5, g['pin'], lw=1.0)
# axis and half labels
ax.plot([-15, H0 + 3], [0, 0], 'k-.', lw=0.7)
ax.text(-15, 1.0, '관절(바퀴) 회전축', fontsize=9)
ax.text(-15, 44, '위쪽 절반: M3 나사 자리(0°)를 지나는 단면', fontsize=11, weight='bold')
ax.text(-15, -44.5, '아래쪽 절반: 평행핀 자리(90°)를 지나는 단면 — 핀 3개와 나사 6개는 같은 원(PCD 20)에서 각도만 다름', fontsize=11, weight='bold')
ax.text(-9, 25, 'AKE 고정자', ha='center', fontsize=10); ax.text(-9, -25, 'AKE 고정자', ha='center', fontsize=10)
ax.text(-6, 3.5, '모터 출력부\n(회전)', ha='center', fontsize=10)
ax.text(-5, 41.5, 'PETG 정강이 바퀴 판', ha='center', fontsize=10)
ax.text(F1 + 4.5, 19.5, '허브 플랜지\n(PETG, 회전)', ha='center', fontsize=10)
ax.text(H0 - 3, -18, '육각 허브', ha='center', fontsize=10)
ax.text(H0 + 2.3, 0.8, '← M5 바퀴 축 볼트', fontsize=9)
def call(xy, xt, txt, c='k'):
    ax.annotate(txt, xy=xy, xytext=xt, fontsize=11, color=c, va='center', arrowprops=dict(arrowstyle='->', color=c, lw=1.0))
X = H0 + 6
call((REC + P.RING_T + 0.75, RP + 1.5), (X, 38), '① M2.5 × 8 저두 + 클램프 링 — 정강이 판을 모터 고정자에 조임 (6개/관절)')
call((BEAR + 0.25, RO + 3.2), (X, 28), '② 평와셔 DIN 125 M3 (Ø7 × 0.5, 노란색) — 6개/관절\n   나사 머리 힘을 넓게 펴서 PETG가 눌리지 않게,\n   두께 0.5만큼 나사가 덜 들어가 탭 바닥(5 mm)에 안 닿게', '#9a7b00')
call((BEAR + 2.0, RO - 1.0), (X, 16), '③ M3 × 16 (12.9) — 6개/관절\n   허브 플랜지를 모터 출력부에 조임, 출력부 탭에 4.3 mm 물림')
call((OUT + 2, -RO), (X, -12), '④ 평행핀 ISO 8734 Ø3 × 10 (m6, 경화) — 3개/관절\n   출력부 핀 구멍에 3 mm, 허브 플랜지에 7 mm 끼움.\n   두 부품 위치를 정확히 맞추고(나사 구멍은 0.2 mm 여유),\n   모터 토크를 나사와 함께 받아 플랜지가 돌며 밀리지 않게', '#c0392b')
call((OUT - 0.7, -3.0), (X, -27), '⑤ Ø11.8 스피곳: 플랜지가 출력부 홈(Ø12)에 들어가 중심 맞춤')
ax.text(X, -36, 'hip·knee 구동 보스도 같은 방식: 평행핀 3 + M3 × 16 6 + 평와셔 6.\n단, 거기는 와셔 밑에 출력 클램프 링(Ø27, 2 mm)이 한 장 더 깔린다.', fontsize=10, va='center')
ax.set_xlim(-16, X + 48); ax.set_ylim(-47, 48)
ax.set_title('바퀴 관절 단면 — 평행핀·평와셔·나사 배치 (모델 치수, 실제 비율)', fontsize=15, loc='left')
fig.savefig(sys.argv[1] if len(sys.argv) > 1 else HERE / 'wheel_joint_pins_washers.png', dpi=100, bbox_inches='tight')
