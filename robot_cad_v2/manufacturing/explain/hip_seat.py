"""Hip motor seating: the AKE pilot end vs the stator front face against the hip plate pocket floor / Ø80 boss face.
Four cases (CAD, pilot seats, stator seats, printed leg plate for comparison); the 0.1 mm differences are drawn x5.  From fusion/params.py; explanatory figure, not a manufacturing drawing.

    python hip_seat.py               # -> hip_seat.png next to this script
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
from matplotlib.ticker import MultipleLocator
F = '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc'
font_manager.fontManager.addfont(F)
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['font.family'] = font_manager.FontProperties(fname=F).get_name()

A = P.AKE
RING = A['ring']                       # -8.0
ZOFF = RING - 5.5                      # hip plate Z -> motor y
EX = 5.0                               # exaggeration of the 0.1 mm offsets
C = dict(plate='#9aa3ad', petg='#9fd29f', motor='#c8c8c8', screw='#3a3a3a', bad='#d9534f', ok='#2e7d32')


def hip_plate(ax, floor_shift=0.0):
    pts = [(15.0, 0.0), (26.0, 0.0), (26.0, 3.0), (46.0, 3.0), (46.0, 8.0), (40.0, 8.0), (40.0, 9.5), (21.0, 9.5),
           (21.0, 5.5 + floor_shift), (15.0, 5.5 + floor_shift)]
    ax.add_patch(Polygon([(r, z + ZOFF) for r, z in pts], fc=C['plate'], ec='k', lw=1.0, zorder=2))


def petg_plate(ax):
    # knee / wheel plate: 9 thick, pocket 2.5 from the motor-side face (motor y -8.0 .. -5.5), plate face at -5.5
    pts = [(15.0, RING - 6.5), (46.0, RING - 6.5), (46.0, RING + P.POCKET), (21.0, RING + P.POCKET), (21.0, RING), (15.0, RING)]
    ax.add_patch(Polygon(pts, fc=C['petg'], ec='k', lw=1.0, zorder=2))


def motor(ax, dy=0.0, p=4.0):
    """AKE pilot (r 13.6..21, 3 long), step to r 20.3, stator front at ring + p, stator up to y 0 (cut)."""
    r0 = RING + dy
    pts = [(13.6, r0), (21.0, r0), (21.0, r0 + 3.0), (20.3, r0 + 3.0), (20.3, r0 + p), (30.75, r0 + p), (30.75, 0.5),
           (13.6, 0.5)]
    ax.add_patch(Polygon(pts, fc=C['motor'], ec='k', lw=1.0, zorder=3))
    ax.text(25.0, -1.3, 'AKE60 고정자', ha='center', fontsize=10, zorder=8)
    return r0, r0 + p


def screw(ax, ring_y):
    # ISO 4762 M2.5 x 6 at PCD 36.5 (r 18.25): head in the Ø4.8 x 2.7 counterbore on the plate outer face (Z 0)
    z0 = ZOFF
    ax.add_patch(Rectangle((18.25 - 2.25, z0), 4.5, 2.5, fc=C['screw'], ec='k', lw=0.6, zorder=4))
    ax.add_patch(Rectangle((18.25 - 1.25, z0 + 2.5), 2.5, 6.0 - 0.0, fc=C['screw'], ec='k', lw=0.6, zorder=4))
    ax.annotate('', xy=(17.0, ring_y + 1.4), xytext=(17.0, ring_y + 3.3),
                arrowprops=dict(arrowstyle='->', color='orange', lw=2.0), zorder=9)


def mark(ax, y, x0, x1, col, lw=3.0):
    ax.plot([x0, x1], [y, y], color=col, lw=lw, solid_capstyle='butt', zorder=7)


fig, axg = plt.subplots(2, 2, figsize=(22, 15.5))
plt.subplots_adjust(left=0.05, right=0.99, top=0.93, bottom=0.08, hspace=0.42, wspace=0.10)
axs = axg.ravel()
titles = ['① 설계(CAD): 파일럿 끝과 고정자 앞면이 동시에 닿음', '② 실물 A: p가 s보다 조금 김 → 파일럿 끝이 바닥에 앉음 (정상)',
          '③ 실물 B: p가 s보다 조금 짧음 → 고정자 앞면이 보스에 먼저 앉음', '④ 비교: knee·wheel (출력 PETG 판)']
notes = [('포켓 깊이 s = 4.0 (hip 판 가공, ±0.05)\nAKE 파일럿 끝 → 고정자 앞면 p = 4.0 (모델 값, CubeMars 공차 미공개)\n'
          '→ 실물에서는 p와 s가 조금씩 달라서 ② 또는 ③ 중 하나가 됨', 'k'),
         ('M2.5 나사(주황 화살표)가 고정링을 바닥으로 당겨 고정 → 기준면이 하나\n'
          '모터 위치·엔코더 높이 = 설계값 (자석 → 다이 1.70)\n보스 면과는 p - s 만큼 틈 (예: 0.1)', C['ok']),
         ('파일럿 끝이 바닥에서 s - p 만큼 뜸 (예: 0.1) → 모터 전체가 그만큼 보드 쪽으로 올라감\n'
          '→ 자석이 칩 쪽으로 0.1 다가감: 자석 → 다이 1.70 → 1.60\n나사를 조이면 고정링을 바닥 쪽으로 당겨 모터 몸체에 무리한 힘이 걸림', C['bad']),
         ('포켓 깊이 2.5, 판 면이 고정자 앞면보다 1.5 낮음 → 고정자 앞면은 절대 안 닿음\n'
          'AKE 치수가 조금 달라도 항상 파일럿 끝(포켓 바닥) 하나만 기준', C['ok'])]
for ax, t, (n, col) in zip(axs, titles, notes):
    ax.set_aspect('equal'); ax.set_xlim(12.0, 47.0); ax.set_ylim(-14.3, 1.2)
    ax.set_title(t, fontsize=13, loc='left')
    ax.xaxis.set_major_locator(MultipleLocator(5)); ax.yaxis.set_major_locator(MultipleLocator(2))
    ax.xaxis.set_minor_locator(MultipleLocator(1)); ax.yaxis.set_minor_locator(MultipleLocator(1))
    ax.grid(color='0.9', lw=0.5, zorder=0)
    ax.set_xlabel('모터 축에서 반지름 [mm]', fontsize=9)
    ax.set_ylabel('모터 축 방향 y [mm]', fontsize=9)
    ax.text(0.0, -0.17, n, transform=ax.transAxes, fontsize=11, color=col, va='top')
kw = dict(fontsize=10, zorder=9)


def lab(ax, xy, xyt, t, col='k'):
    ax.annotate(t, xy=xy, xytext=xyt, color=col, arrowprops=dict(arrowstyle='->', color=col, lw=0.9), **kw)


# 1. CAD
ax = axs[0]
hip_plate(ax); ring, front = motor(ax); screw(ax, ring)
mark(ax, ring, 15.0, 21.0, C['ok']); mark(ax, front, 21.0, 30.75, C['ok'])
lab(ax, (14.0, ring), (12.4, -4.0), '파일럿 끝 =\n포켓 바닥 (기준면)', C['ok'])
lab(ax, (30.0, front), (33.0, -1.5), 'Ø80 보스 면 = 고정자 앞면\n(틈 0)', C['ok'])
ax.text(34.0, -8.3, 'hip 판 (6061)', fontsize=10, zorder=8)

# 2. p > s
ax = axs[1]
hip_plate(ax); ring, front = motor(ax, 0.0, 4.0 + 0.1 * EX); screw(ax, ring)
mark(ax, ring, 15.0, 21.0, C['ok'])
ax.annotate('', xy=(27.0, front), xytext=(27.0, RING + 4.0), arrowprops=dict(arrowstyle='<->', color='b', lw=1.2, shrinkA=0, shrinkB=0), zorder=9)
lab(ax, (27.3, (front + RING + 4.0) / 2), (33.0, -1.5), '고정자 앞면과 보스 면 사이 틈 p - s\n(0.1을 5배로 그림)', 'b')
lab(ax, (14.0, ring), (12.4, -4.0), '파일럿 끝이\n바닥에 닿음', C['ok'])

# 3. p < s
ax = axs[2]
hip_plate(ax); d = 0.1 * EX
ring, front = motor(ax, d, 4.0 - 0.1 * EX); screw(ax, ring)
mark(ax, front, 21.0, 30.75, C['bad'])
ax.annotate('', xy=(14.2, ring), xytext=(14.2, RING), arrowprops=dict(arrowstyle='<->', color=C['bad'], lw=1.2), zorder=9)
lab(ax, (14.6, (ring + RING) / 2), (12.4, -4.0), '바닥에서 뜸 s - p\n(0.1을 5배로 그림)', C['bad'])
lab(ax, (30.0, front), (33.0, -1.5), '고정자 앞면이\n보스 면에 먼저 닿음', C['bad'])

# 4. PETG
ax = axs[3]
petg_plate(ax); ring, front = motor(ax)
mark(ax, ring, 15.0, 21.0, C['ok'])
ax.annotate('', xy=(33.0, front), xytext=(33.0, RING + P.POCKET), arrowprops=dict(arrowstyle='<->', color='b', lw=1.2), zorder=9)
ax.text(33.6, (front + RING + P.POCKET) / 2, '틈 1.5', fontsize=11, color='b', va='center', zorder=8)
lab(ax, (14.0, ring), (12.4, -4.0), '파일럿 끝만 닿음', C['ok'])
ax.text(34.0, -11.0, 'PETG 판 (포켓 2.5)', fontsize=10, zorder=8)

fig.suptitle('hip 모터 자리: 모터가 어느 면에 앉는가 (단면 한쪽, 실제 비율, 0.1 mm 차이만 5배 과장)', fontsize=14, x=0.01, ha='left')
fig.text(0.01, 0.005, 's = 포켓 바닥 → Ø80 보스 면 (hip 판, 가공 4.0 ± 0.05)     p = AKE 파일럿 끝 → 고정자 앞면 (모델 4.0, CubeMars 공차 미공개)     '
         '주황 화살표 = M2.5 나사가 고정링을 당기는 방향', fontsize=10)
fig.savefig(sys.argv[1] if len(sys.argv) > 1 else HERE / 'hip_seat.png', dpi=95, bbox_inches='tight')
