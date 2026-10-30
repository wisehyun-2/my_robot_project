"""What a boss and a pilot are, and the Ø63 x 0.5 relief in the hip plate's Ø80 boss face (before / after, added 10-30).
Explanatory figure, not a manufacturing drawing.  Run make_parts.py first.

    python hip_relief.py             # -> hip_relief.png next to this script
"""
import math
import pathlib
import sys
HERE = pathlib.Path(__file__).resolve().parent
ROOT = str(HERE.parents[1])
sys.path.insert(0, ROOT + '/fusion'); sys.path.insert(0, ROOT + '/manufacturing')
import params as P
import occ as O
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Polygon, Rectangle, Circle, Wedge
from matplotlib.ticker import MultipleLocator
F = '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc'
font_manager.fontManager.addfont(F)
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['font.family'] = font_manager.FontProperties(fname=F).get_name()

R_REL, D_REL = P.HIP_STATOR_RELIEF[0] / 2, P.HIP_STATOR_RELIEF[1]   # relief Ø63 x 0.5 deep in the boss face (Z 9.5 -> 9.0)
RP = P.PILOT_D_HIP / 2
C = dict(plate='#9aa3ad', motor='#c8c8c8', pilot='#5b8fd6', boss='#f0a040', rel='#ffd54f', screw='#3a3a3a',
         bad='#d9534f', ok='#2e7d32')
fig = plt.figure(figsize=(22, 17))


def lab(ax, xy, xyt, t, col='k', fs=10):
    ax.annotate(t, xy=xy, xytext=xyt, color=col, fontsize=fs, zorder=10,
                arrowprops=dict(arrowstyle='->', color=col, lw=0.9))


# ------------------------------------------------------------------ 1. terms: pilot and boss (exploded side view, schematic)
a1 = fig.add_axes([0.02, 0.64, 0.45, 0.32]); a1.set_aspect('equal'); a1.axis('off')
a1.add_patch(Polygon([(-60, 3), (-26, 3), (-26, 0), (-15, 0), (-15, 5.5), (-RP, 5.5), (-RP, 8), (-60, 8)], fc=C['plate'], ec='k', lw=0.8))
a1.add_patch(Polygon([(60, 3), (26, 3), (26, 0), (15, 0), (15, 5.5), (RP, 5.5), (RP, 8), (60, 8)], fc=C['plate'], ec='k', lw=0.8))
for sx in (-1, 1):
    a1.add_patch(Polygon([(sx * RP, 8), (sx * 40, 8), (sx * 40, 9.5), (sx * RP, 9.5)], fc=C['boss'], ec='k', lw=0.9))
lift = 9.0                                               # motor drawn lifted (exploded)
a1.add_patch(Polygon([(-30.75, 9.5 + lift), (30.75, 9.5 + lift), (30.75, 22 + lift), (-30.75, 22 + lift)], fc=C['motor'], ec='k', lw=0.8))
a1.add_patch(Polygon([(-RP, 5.5 + lift), (RP, 5.5 + lift), (RP, 8.5 + lift), (20.3, 8.5 + lift), (20.3, 9.5 + lift), (-20.3, 9.5 + lift),
                      (-20.3, 8.5 + lift), (-RP, 8.5 + lift)], fc=C['pilot'], ec='k', lw=0.9))
a1.add_patch(Rectangle((-13.6, 5.5 + lift), 27.2, 3.0, fc='w', ec='k', lw=0.6))
a1.text(0, 26.0, 'AKE60 모터 (들어 올려 그림)', ha='center', fontsize=11)
a1.text(0, 6.6 + lift, '모터 출력부', ha='center', fontsize=8.5)
a1.annotate('', xy=(0, 10.2), xytext=(0, 13.6), arrowprops=dict(arrowstyle='->', lw=2.0, color='0.3'))
lab(a1, (-RP + 1, 7.0 + lift), (-60, 20.5), '파일럿 (pilot): 모터 앞에 튀어나온 정밀한 원통\nØ42, 길이 3 → 판의 같은 지름 구멍(포켓)에 꼭 끼워져\n모터 중심을 판에 맞춤', C['pilot'], 10.5)
lab(a1, (-RP + 0.5, 7.0), (-60, -6.0), '포켓 (pocket): 파일럿이 들어가는 막힌 구멍\nØ42 H7, 깊이 4.0 → 바닥이 모터 높이 기준면', '0.25', 10.5)
lab(a1, (32, 9.0), (36, 15.5), '보스 (boss): 판 면에서 솟은 원통형 턱\nØ80, 높이 1.5 → 드라이버 컵이\n이 바깥 둘레를 잡아 중심을 맞춤', '#c06000', 10.5)
a1.text(46, 4.0, 'hip 판', fontsize=11)
a1.set_xlim(-62, 62); a1.set_ylim(-9, 33)
a1.set_title('① 용어: 파일럿·포켓·보스 (모터 축을 지나는 단면, 실제 비율)', fontsize=13, loc='left')

# ------------------------------------------------------------------ 2. isometric of the real hip plate with the relief
sh = O.solids(O.read_step(ROOT + '/manufacturing/step/hip_plate_L.step'))[0]
after = sh                                               # make_parts.py already cuts the relief
tw = np.array([1.0, -1.0, 1.25]); tw /= np.linalg.norm(tw)
xd = np.array([1.0, 1.0, 0.0]); xd /= np.linalg.norm(xd)
yd = np.cross(tw, xd)
vis, hid = O.project(after, tuple(tw), tuple(xd))
a2 = fig.add_axes([0.50, 0.60, 0.48, 0.38]); a2.set_aspect('equal'); a2.axis('off')
for pl in hid:
    xs, ys = zip(*pl); a2.plot(xs, ys, color='0.8', lw=0.4, ls='--')
for pl in vis:
    xs, ys = zip(*pl); a2.plot(xs, ys, color='0.2', lw=0.7)


def ring3d(r, z, col, lw=2.2, ls='-'):
    t = np.linspace(0, 2 * np.pi, 200)
    pts = np.c_[r * np.cos(t), r * np.sin(t), np.full_like(t, z)]
    a2.plot(pts @ xd, pts @ yd, color=col, lw=lw, ls=ls, zorder=5)


ring3d(40.0, 9.5, C['boss'], 2.6)
ring3d(R_REL, 9.0, '#e6a800', 2.6)
ring3d(RP, 9.0, C['pilot'], 2.2)
ring3d(30.75, 9.5, C['bad'], 1.4, '--')
P2 = lambda x, y, z: (np.dot((x, y, z), xd), np.dot((x, y, z), yd))  # noqa: E731
lab(a2, P2(-28, -28.3, 9.5), (-118, 10), 'Ø80 보스 바깥 둘레 (주황): 컵 기둥 3개가 잡는 곳', '#c06000')
lab(a2, P2(22, -22.3, 9.0), (-118, -10), '새 홈 Ø63 × 0.5 (노랑): 보스 면 안쪽을 0.5 깎음', '#b08000')
lab(a2, P2(14.9, -14.9, 9.0), (-118, -30), 'Ø42 H7 포켓 (파랑): 파일럿 자리', C['pilot'])
lab(a2, P2(-21.7, 21.7, 9.5), (-118, 30), '빨간 점선 = 고정자 앞면 바깥 지름 Ø61.5\n(이 안쪽이 보스 면과 닿던 곳)', C['bad'])
a2.set_title('② 실제 hip 판(업로드용 STEP, 홈 포함), 로봇 안쪽에서 비스듬히 봄', fontsize=13, loc='left')


# ------------------------------------------------------------------ 3/4. section before / after (one side)
def section(ax, relief, title):
    ax.set_aspect('equal'); ax.set_xlim(12, 46); ax.set_ylim(4.0, 14.5)
    pts = [(15.0, 0.0), (26.0, 0.0), (26.0, 3.0), (46.0, 3.0), (46.0, 8.0), (40.0, 8.0), (40.0, 9.5)]
    if relief:
        pts += [(R_REL, 9.5), (R_REL, 9.5 - D_REL), (RP, 9.5 - D_REL)]
    else:
        pts += [(RP, 9.5)]
    pts += [(RP, 5.5), (15.0, 5.5)]
    ax.add_patch(Polygon(pts, fc=C['plate'], ec='k', lw=1.0, zorder=2))
    ax.add_patch(Polygon([(RP, 8.0), (40.0, 8.0), (40.0, 9.5)] + ([(R_REL, 9.5), (R_REL, 9.0), (RP, 9.0)] if relief else [(RP, 9.5)]),
                         fc=C['boss'], ec='k', lw=0.8, alpha=0.8, zorder=3))
    m = [(13.6, 5.5), (RP, 5.5), (RP, 8.5), (20.3, 8.5), (20.3, 9.5), (30.75, 9.5), (30.75, 14.5), (13.6, 14.5)]
    ax.add_patch(Polygon(m, fc=C['motor'], ec='k', lw=1.0, zorder=4))
    ax.add_patch(Polygon([(13.6, 5.5), (RP, 5.5), (RP, 8.5), (13.6, 8.5)], fc=C['pilot'], ec='k', lw=0.8, zorder=5))
    ax.plot([15.0, RP], [5.5, 5.5], color=C['ok'], lw=3, zorder=6)
    if relief:
        ax.annotate('', xy=(27.0, 9.5), xytext=(27.0, 9.0), arrowprops=dict(arrowstyle='<->', color='b', lw=1.2, shrinkA=0, shrinkB=0), zorder=8)
        lab(ax, (27.3, 9.25), (33.0, 12.8), '틈 0.5: 고정자 앞면이\n보스에 절대 안 닿음', 'b')
        lab(ax, (R_REL - 0.3, 9.1), (33.0, 6.0), '홈 끝 Ø63 (반지름 31.5)\n보스 바깥 Ø63~Ø80은 그대로', '#b08000')
    else:
        ax.plot([20.3, 30.75], [9.5, 9.5], color=C['bad'], lw=3, zorder=6)
        lab(ax, (28.0, 9.5), (33.0, 12.8), '고정자 앞면 = 보스 면\n(틈 0, 동시에 닿음)', C['bad'])
    lab(ax, (16.0, 5.5), (12.4, 4.4), '파일럿 끝 = 포켓 바닥 (기준면)', C['ok'])
    ax.text(25.0, 12.5, 'AKE60 고정자', ha='center', fontsize=10, zorder=9)
    ax.text(16.6, 7.0, '파일럿', fontsize=9.5, color='w', zorder=9)
    ax.text(41.0, 5.0, 'hip 판', fontsize=10, zorder=9)
    ax.xaxis.set_major_locator(MultipleLocator(5)); ax.yaxis.set_major_locator(MultipleLocator(1))
    ax.grid(color='0.9', lw=0.5, zorder=0)
    ax.set_xlabel('모터 축에서 반지름 [mm]', fontsize=9); ax.set_ylabel('hip 판 두께 방향 Z [mm]\n(0 = 바깥 허브 면, 9.5 = 보스 면)', fontsize=9)
    ax.set_title(title, fontsize=13, loc='left')


section(fig.add_axes([0.05, 0.33, 0.60, 0.25]), False, '③ 홈 넣기 전 (10-30 이전 설계, 단면 한쪽)')
section(fig.add_axes([0.05, 0.03, 0.60, 0.25]), True, '④ 홈을 넣은 뒤 (현재 설계)')

# ------------------------------------------------------------------ 5. top view (inside face) of the centre
a5 = fig.add_axes([0.69, 0.03, 0.30, 0.52]); a5.set_aspect('equal')
a5.add_patch(Circle((0, 0), 46, fc=C['plate'], ec='k', lw=0.6, alpha=0.5))
a5.add_patch(Circle((0, 0), 40, fc=C['boss'], ec='k', lw=1.0))
a5.add_patch(Circle((0, 0), R_REL, fc=C['rel'], ec='k', lw=1.0, hatch='//'))
a5.add_patch(Circle((0, 0), RP, fc='w', ec=C['pilot'], lw=1.6))
a5.add_patch(Circle((0, 0), 15, fc='0.85', ec='k', lw=0.6))
a5.add_patch(Circle((0, 0), 30.75, fc='none', ec=C['bad'], lw=1.4, ls='--'))
for t in (-30.0, -90.0, 210.0):
    a5.add_patch(Wedge((0, 0), 43.9, t + 90 - 4.8, t + 90 + 4.8, width=3.9, fc='#cfe3cf', ec='k', lw=0.6))
for r, txt, ang in ((40, 'Ø80 보스 (높이 1.5)', 62), (R_REL, '새 홈 Ø63 × 0.5', 35), (RP, 'Ø42 H7 포켓', 10), (15, 'Ø30', -20)):
    a5.text(r * math.cos(math.radians(ang)) + 0.8, r * math.sin(math.radians(ang)), txt, fontsize=9.5,
            bbox=dict(fc='w', ec='none', alpha=0.8, pad=0.3), zorder=8)
a5.text(-44, -50, '빨간 점선: 고정자 앞면 바깥 Ø61.5  초록: 컵 기둥이 보스를 잡는 곳', fontsize=9)
a5.set_xlim(-48, 48); a5.set_ylim(-52, 48)
a5.xaxis.set_major_locator(MultipleLocator(10)); a5.yaxis.set_major_locator(MultipleLocator(10))
a5.grid(color='0.9', lw=0.5)
a5.set_title('⑤ 안쪽 면에서 본 가운데 (홈 = 노란 빗금)', fontsize=13, loc='left')
fig.savefig(sys.argv[1] if len(sys.argv) > 1 else HERE / 'hip_relief.png', dpi=95, bbox_inches='tight')
