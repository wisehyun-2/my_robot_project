"""Stator clamp ring figure: top view, a radial strip through one hole (why the inner web is 1.85 and the outer 2.85 mm)
and a knee / wheel joint half section, to scale from fusion/params.py.  Explanatory figure, not a manufacturing drawing.

    python stator_ring.py            # -> stator_ring_explain.png next to this script
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
from matplotlib.patches import Circle, Rectangle, Polygon
F = '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc'
font_manager.fontManager.addfont(F); plt.rcParams['axes.unicode_minus'] = False; plt.rcParams['font.family'] = font_manager.FontProperties(fname=F).get_name()
OD, ID = P.RING_STATOR; R_O, R_I = OD/2, ID/2; PCD = P.AKE_PCD_RING; RP = PCD/2; RH = 1.4
R_BOSS = P.HUB_BOSS_D/2; R_HOLE_PETG = 15.0; R_RECESS = P.STATOR_RECESS_D/2; HEAD = 4.5/2
C = dict(boss='#e07b39', ring='#8a9bb0', petg='#9fd29f', motor='#c9c9c9', screw='#555555', hole='white')
fig = plt.figure(figsize=(16, 9.2))
# ---------------- top view
ax = fig.add_axes([0.02, 0.06, 0.40, 0.84]); ax.set_aspect('equal'); ax.axis('off')
ax.add_patch(Circle((0,0), R_RECESS, fc=C['petg'], ec='k', lw=0.6, alpha=0.35))
ax.add_patch(Circle((0,0), R_O, fc=C['ring'], ec='k', lw=1.2))
ax.add_patch(Circle((0,0), R_I, fc='white', ec='k', lw=1.2))
ax.add_patch(Circle((0,0), R_BOSS, fc=C['boss'], ec='k', lw=1.0, alpha=0.85))
for k in range(6):
    t = math.radians(30 + 60*k); x, y = RP*math.cos(t), RP*math.sin(t)
    ax.add_patch(Circle((x,y), HEAD, fc='none', ec=C['screw'], lw=0.8, ls='--'))
    ax.add_patch(Circle((x,y), RH, fc='white', ec='k', lw=1.0))
ax.add_patch(Circle((0,0), RP, fc='none', ec='b', lw=0.8, ls='-.'))
ax.text(0, 0, '정강이(또는 바퀴 허브)\nØ28 구동 보스\n(모터 출력과 함께 회전)', ha='center', va='center', fontsize=10)
ax.text(0, -R_RECESS-2.2, 'PETG 판 홈 Ø45.6 (초록)', ha='center', fontsize=9)
ax.text(0, R_O+1.0, '고정링 클램프 링 Ø45 / Ø30, 두께 2 (회색)', ha='center', fontsize=11)
ax.text(RP*0.72, -RP*0.86, '구멍 원 PCD 36.5\n(모터 탭 위치)', color='b', fontsize=9, ha='center')
ax.text(-25, -25, '점선 원 = M2.5 나사 머리 자리', fontsize=9, color=C['screw'])
t = math.radians(30)
ax.plot([R_BOSS*math.cos(t), R_RECESS*math.cos(t)], [R_BOSS*math.sin(t), R_RECESS*math.sin(t)], 'r-', lw=2)
ax.text(R_RECESS*math.cos(t)+0.5, R_RECESS*math.sin(t)+0.5, '② 띠', color='r', fontsize=10)
ax.set_xlim(-26, 26); ax.set_ylim(-26, 26)
ax.set_title('① 위에서 본 모습 (실제 비율, 반지름 mm)', fontsize=13, loc='left')
# ---------------- radial strip (zoom)
bx = fig.add_axes([0.45, 0.60, 0.53, 0.30]); bx.axis('off')
def seg(x0, x1, y0, y1, c, lab=None, **kw):
    bx.add_patch(Rectangle((x0, y0), x1-x0, y1-y0, fc=c, ec='k', lw=0.8, **kw))
    if lab: bx.text((x0+x1)/2, (y0+y1)/2, lab, ha='center', va='center', fontsize=9)
seg(12.0, R_BOSS, 0, 1, C['boss'], '보스\n(회전)')
seg(R_BOSS, R_I, 0, 1, 'white', '틈 1.0')
seg(R_I, RP-RH, 0, 1, C['ring'], '안쪽 살\n1.85')
seg(RP-RH, RP+RH, 0, 1, 'white', '구멍\nØ2.8')
seg(RP+RH, R_O, 0, 1, C['ring'], '바깥 살\n2.85')
seg(R_O, R_RECESS, 0, 1, C['petg'], '')
for x in (R_BOSS, R_I, RP-RH, RP, RP+RH, R_O):
    bx.plot([x, x], [-0.15, 0], 'k', lw=0.8); bx.text(x, -0.3, '%.2f' % x, ha='center', va='top', fontsize=9)
bx.annotate('', xy=(RP, 1.35), xytext=(RP, 1.05), arrowprops=dict(arrowstyle='<-', color='b'))
bx.text(RP, 1.4, '구멍 중심 18.25 = 모터가 정함 (못 옮김)', color='b', ha='center', fontsize=10)
bx.annotate('', xy=((R_I+R_O)/2, -0.75), xytext=((R_I+R_O)/2, -0.55), arrowprops=dict(arrowstyle='<-', color='r'))
bx.text((R_I+R_O)/2, -0.95, '링 폭의 한가운데 18.75 → 구멍이 0.5 mm 안쪽에 있음', color='r', ha='center', fontsize=10)
bx.text(13.0, 1.4, '← 축 쪽', fontsize=9); bx.text(22.0, 1.4, '바깥 쪽 →', fontsize=9)
bx.set_xlim(11.5, 23.5); bx.set_ylim(-1.2, 1.7)
bx.set_title('② 반지름 방향으로 잘라 늘린 띠 (①의 30° 선)', fontsize=13, loc='left')
# ---------------- axial half section of the knee joint
cx = fig.add_axes([0.47, 0.03, 0.51, 0.52]); cx.set_aspect('equal'); cx.axis('off')
far = P.KNEE_PLATE_T - P.POCKET; rec = far - P.RING_T - P.CBORE_M25[1]
A = P.AKE; SF = A['ring'] - A['stator_front']; PL = -A['pilot_len']; MF = -P.POCKET; RS = 30.75
cx.add_patch(Polygon([(-10,10),(-10,RS),(SF,RS),(SF,20.3),(PL,20.3),(PL,21),(0,21),(0,13.5),(0.5,13.5),(0.5,10)], fc=C['motor'], ec='k', lw=0.8))
cx.add_patch(Rectangle((-10, 31.2), A['ring'] - A['can_front'] + 10, A['can_r'] - 31.2, fc='#7f9fbf', ec='k', lw=0.8))
cx.text(-8.9, 32.85, '모터 캔(회전)', fontsize=8, va='center')
cx.add_patch(Polygon([(MF, 21), (0, 21), (0, R_HOLE_PETG), (rec, R_HOLE_PETG), (rec, R_RECESS), (far, R_RECESS),
                      (far, 40), (-10, 40), (-10, P.CAN_CLEAR_R), (MF, P.CAN_CLEAR_R)], fc=C['petg'], ec='k', lw=0.8))
cx.plot([0, 0, MF], [13.5 + 1.5, 21, 21], color='r', lw=2.5)
cx.add_patch(Rectangle((rec, R_I), P.RING_T, R_O - R_I, fc=C['ring'], ec='k', lw=1.2))
cx.add_patch(Rectangle((rec, RP-RH), P.RING_T, 2*RH, fc='white', ec='none'))
cx.add_patch(Rectangle((-3.2, RP-1.25), rec + P.RING_T + 3.2, 2.5, fc=C['screw'], ec='k', lw=0.6))
cx.add_patch(Rectangle((rec + P.RING_T, RP-HEAD), 1.5, 2*HEAD, fc=C['screw'], ec='k', lw=0.6))
cx.add_patch(Polygon([(0.5,10),(0.5,14),(far+1.5,14),(far+1.5,40),(far+8,40),(far+8,10)], fc=C['boss'], ec='k', lw=0.8, alpha=0.85))
cx.text(-6.6, 25, 'AKE 모터\n고정자\n(움직이지\n않음)', ha='center', va='center', fontsize=10)
cx.text(2.0, 38.0, 'PETG 판', ha='center', fontsize=10)
cx.text(far+4.7, 30, '정강이\n몸체\n(회전)', ha='center', fontsize=10)
cx.text(5.0, 11.6, 'Ø28 구동 보스 (회전) →', ha='center', fontsize=9)
cx.annotate('클램프 링 (두께 2)', xy=(rec + 1.0, 21.5), xytext=(far + 9, 21.5), fontsize=10, va='center',
            arrowprops=dict(arrowstyle='->'))
cx.annotate('M2.5 나사 → 모터 탭에만 물림', xy=(-2.0, RP), xytext=(far + 9, RP - 1.2), fontsize=10, va='center',
            arrowprops=dict(arrowstyle='->'))
cx.annotate('바깥 살 2.85: 바깥은 PETG 판뿐 → 여유', xy=(rec + 1.0, 21.0), xytext=(far + 9, 25.5), fontsize=10, color='g', va='center',
            arrowprops=dict(arrowstyle='->', color='g'))
cx.annotate('안쪽 살 1.85: 바로 아래에서 보스가 돌아서\n1 mm 틈을 남겨야 함 → 더 못 늘림', xy=(rec + 1.0, 15.9), xytext=(far + 9, 14.8),
            fontsize=10, color='r', va='center', arrowprops=dict(arrowstyle='->', color='r'))
cx.annotate('고정자 앞면과 PETG 사이 1.5 mm 틈 (닿지 않음)', xy=((SF + MF) / 2, 27.0), xytext=(far + 9, 39.5), fontsize=10, va='center',
            arrowprops=dict(arrowstyle='->', relpos=(0, 0.5)))
cx.annotate('닿는 곳(빨강): Ø42 파일럿 → 포켓 바닥·벽(H7)', xy=(0, 18.5), xytext=(far + 9, 36.5), fontsize=10, color='r',
            va='center', arrowprops=dict(arrowstyle='->', color='r', relpos=(0, 0.5)))
cx.plot([rec - 0.5, far + 1], [R_BOSS, R_BOSS], 'r:', lw=0.8)
cx.text(-9.5, 10.6, '아래쪽(보이지 않는 곳)에 관절 회전축', fontsize=8)
cx.set_xlim(-10.5, far + 30); cx.set_ylim(9.5, 41)
cx.set_title('③ 관절 단면 (회전축을 지나 자른 모습의 일부, 실제 비율) — knee·wheel 같은 구조', fontsize=13, loc='left')
fig.savefig(sys.argv[1] if len(sys.argv) > 1 else HERE / 'stator_ring_explain.png', dpi=110)
