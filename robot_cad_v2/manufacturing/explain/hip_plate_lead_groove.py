"""Hip plate lead groove 7 -> 12 mm wide (10-29): the AKE60's three phase pins (Ø1.5 x 4, r 25 on the lead direction,
3 mm apart, tips on the Ø80 boss face Z 9.5) sat on the edges of the 7 mm groove, leaving no room to solder and
lead the wires off.  Renders from the motor side (lead_groove_renders/, Fusion), a section across the groove through
the pins and one along it.  Explanatory figure, not a manufacturing drawing.  Dimensions in the quote part frame
(Z = 0 outer hub face, 9.5 inner boss face); pin data measured on the AKE60 STEP in the assembly.

    python hip_plate_lead_groove.py [OUT.png]
"""
import pathlib
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Polygon, Rectangle

HERE = pathlib.Path(__file__).resolve().parent
plt.rcParams['font.family'] = 'Malgun Gothic'
plt.rcParams['axes.unicode_minus'] = False
RED, GREEN, GREY, PIN = '#d62728', '#2a9d3c', '#c9ced6', '#d9d4c4'
PINS = (-3.0, 0.0, 3.0)
PIN_R, PIN_L, R_PIN = 0.75, 4.0, 25.0
FLOOR, BOSS, WEB_IN = 6.0, 9.5, 8.0
G0, G1 = 23.0, 41.0


def render(ax, path, title, colour, notes):
    ax.imshow(plt.imread(path)[250:1000, 250:1300])
    ax.set_axis_off()
    ax.set_title(title, fontsize=14, weight='bold', color=colour)
    for txt, xy, xyt, c in notes:
        ax.annotate(txt, (xy[0] - 250, xy[1] - 250), (xyt[0] - 250, xyt[1] - 250), fontsize=12, color=c, weight='bold',
                    ha='center', va='center', bbox=dict(fc='white', ec=c, alpha=0.92, boxstyle='round,pad=0.35'),
                    arrowprops=dict(arrowstyle='->', color=c, lw=2.0) if xy != xyt else None)


def across(ax, width, colour, title):
    """Section across the groove at r 25, looking outward along the groove (pins in section)."""
    h = width / 2
    plate = [(-11, 3.0), (11, 3.0), (11, BOSS), (h, BOSS), (h, FLOOR), (-h, FLOOR), (-h, BOSS), (-11, BOSS)]
    ax.add_patch(Polygon(plate, closed=True, fc=GREY, ec='k', lw=1.2))
    ax.text(-8.5, 4.5, '판 (hip plate)', fontsize=10, ha='center')
    ax.add_patch(Rectangle((-11, BOSS + PIN_L), 22, 1.5, fc='#e8e2d0', ec='0.4', lw=0.8, hatch='///'))
    ax.text(0, BOSS + PIN_L + 0.75, '모터 고정자 앞면 (핀이 나오는 면)', fontsize=10, ha='center', va='center')
    for a in PINS:
        hit = abs(a) + PIN_R > h
        ax.add_patch(Rectangle((a - PIN_R, BOSS), 2 * PIN_R, PIN_L, fc=RED if hit else PIN, ec='k', lw=1.0))
    ax.text(0, BOSS + 2.0, '3상 핀\nØ1.5', fontsize=9, ha='center', va='center')
    ax.annotate('', (-h, FLOOR - 0.6), (h, FLOOR - 0.6), arrowprops=dict(arrowstyle='<->', color=colour, lw=1.4,
                                                                         shrinkA=0, shrinkB=0))
    ax.text(0, FLOOR - 1.3, '홈 폭 %.0f' % width, color=colour, fontsize=12, weight='bold', ha='center', va='center')
    if h < 3 + PIN_R:
        ax.annotate('바깥 핀이 홈 가장자리\n판 면에 걸침 (0.25 mm씩)', (3.6, BOSS), (7.0, 12.2), fontsize=11, color=RED,
                    weight='bold', ha='center', arrowprops=dict(arrowstyle='->', color=RED, lw=1.6))
    else:
        ax.annotate('', (3 + PIN_R, BOSS - 0.6), (h, BOSS - 0.6),
                    arrowprops=dict(arrowstyle='<->', color=colour, lw=1.4, shrinkA=0, shrinkB=0))
        ax.text((3 + PIN_R + h) / 2, BOSS - 1.3, '%.2f' % (h - 3 - PIN_R), color=colour, fontsize=11.5, weight='bold',
                ha='center')
        ax.text(0, 7.3, '핀 아래 3.5 mm, 양옆 %.2f mm 빈 공간\n→ 납땜·선 굽힘 자리' % (h - 3 - PIN_R), color=colour,
                fontsize=11, weight='bold', ha='center', va='center')
    ax.set_xlim(-11.5, 11.5); ax.set_ylim(2.0, 16.0); ax.set_aspect('equal')
    ax.set_xlabel('홈을 가로지르는 방향 [mm] (0 = 홈 중심)', fontsize=10)
    ax.set_ylabel('Z [mm] (판 바깥 0 → 안쪽 9.5)', fontsize=10)
    ax.set_title(title, fontsize=13, weight='bold', color=colour)
    ax.grid(alpha=0.25); ax.tick_params(labelsize=9)


def along(ax):
    """Section along the groove axis (after): pocket, 2 mm wall, groove, pin, stator front, bell rim, wire path."""
    plate = [(15, 0.0), (26, 0.0), (26, 3.0), (46, 3.0), (46, WEB_IN), (G1, WEB_IN), (G1, FLOOR), (G0, FLOOR),
             (G0, BOSS), (21.006, BOSS), (21.006, 5.5), (15, 5.5)]
    ax.add_patch(Polygon(plate, closed=True, fc=GREY, ec='k', lw=1.2))
    ax.add_patch(Rectangle((20.3, BOSS + PIN_L), 10.45, 1.5, fc='#e8e2d0', ec='0.4', lw=0.8, hatch='///'))
    ax.text(25.5, BOSS + PIN_L + 0.75, '고정자 앞면', fontsize=9.5, ha='center', va='center')
    ax.add_patch(Rectangle((R_PIN - PIN_R, BOSS), 2 * PIN_R, PIN_L, fc=PIN, ec='k', lw=1.0))
    ax.add_patch(Rectangle((33.0, BOSS + 3.5), 1.5, 3.0, fc='#9aa0aa', ec='0.3', lw=0.8))
    ax.text(35.0, BOSS + 5.4, '모터 로터 캔\n(돌아감)', fontsize=9, ha='left', va='center')
    ax.add_patch(Rectangle((40.02, WEB_IN), 2.4, 7.0, fc='none', ec='#3a78c2', lw=1.2, ls='--'))
    ax.text(41.2, WEB_IN + 7.3, '드라이버 컵 벽(점선)\n이 방향엔 폭 9 슬롯', fontsize=9, ha='center', va='bottom',
            color='#3a78c2')
    path = [(R_PIN, BOSS + 0.1), (R_PIN + 0.3, 7.6), (27.5, 6.9), (37.0, 6.9), (38.3, 7.8), (38.6, BOSS + 2.0),
            (38.6, BOSS + 5.4)]
    xs, ys = zip(*path)
    ax.plot(xs, ys, color='#e07b00', lw=3.2, solid_capstyle='round')
    ax.annotate('납땜한 상 선: 홈 바닥을 따라 바깥으로 →\n캔 바깥(r 35~40)에서 위로 → 드라이버', (32.0, 6.9), (31.0, 1.4),
                fontsize=10.5, color='#e07b00', weight='bold', ha='center', arrowprops=dict(arrowstyle='->', color='#e07b00'))
    ax.annotate('', (21.006, BOSS + 0.6), (G0, BOSS + 0.6), arrowprops=dict(arrowstyle='<->', lw=1.2, shrinkA=0, shrinkB=0))
    ax.text(22.0, BOSS + 1.2, '2.0', fontsize=10, ha='center')
    ax.annotate('', (G0, FLOOR + 0.5), (R_PIN - PIN_R, FLOOR + 0.5), arrowprops=dict(arrowstyle='<->', lw=1.2,
                                                                                  shrinkA=0, shrinkB=0))
    ax.text(23.6, FLOOR + 0.9, '1.25', fontsize=10, ha='center')
    ax.plot([G0, 40.0], [BOSS, BOSS], color='0.35', lw=1.0, ls=':')
    ax.text(31.5, BOSS + 0.35, 'Ø80 보스 면 높이 (홈 양옆, 핀 끝과 같은 높이)', fontsize=9, ha='center', color='0.3')
    ax.annotate('', (29.0, FLOOR), (29.0, BOSS), arrowprops=dict(arrowstyle='<->', lw=1.2, shrinkA=0, shrinkB=0))
    ax.text(29.3, 8.3, '깊이 3.5', fontsize=10, ha='left', va='center')
    ax.text(17.5, 7.4, 'Ø42 포켓\n(모터 고정링)', fontsize=9.5, ha='center')
    ax.text(32.0, 4.4, '판', fontsize=10, ha='center')
    ax.set_xlim(14, 47); ax.set_ylim(0, 19.5); ax.set_aspect('equal')
    ax.set_xlabel('hip 축에서 반경 [mm] (홈 방향)', fontsize=10)
    ax.set_ylabel('Z [mm]', fontsize=10)
    ax.set_title('③ 고친 뒤 — 홈 방향 단면: 선이 빠지는 길 (홈 반경 23~41, 바닥 Z 6)', fontsize=13, weight='bold',
                 color=GREEN)
    ax.grid(alpha=0.25); ax.tick_params(labelsize=9)


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else HERE / 'hip_plate_lead_groove.png'
    rd = HERE / 'lead_groove_renders'
    fig = plt.figure(figsize=(18, 21))
    fig.text(0.5, 0.995, 'hip 판 리드선 홈 폭 7 → 12 mm: 모터 3상 핀이 판에 닿지 않고 선 뺄 자리가 생김', ha='center',
             va='top', fontsize=20, weight='bold')
    fig.text(0.5, 0.977, '왼쪽 판(오른쪽 판은 거울상). 핀 3개 Ø1.5 × 4 mm, 간격 3 mm, 반경 25 — 핀 끝이 판 안쪽 면(Ø80 보스 면)'
             '에 닿는 높이까지 나와 있음', ha='center', va='top', fontsize=13)
    a = fig.add_axes([0.02, 0.62, 0.47, 0.33])
    render(a, rd / 'pins_before.png', '① 고치기 전 — 모터 쪽에서 본 모습 (홈 폭 7)', RED,
           [('바깥 핀 2개가\n홈 가장자리 판 면에 닿음', (630, 690), (420, 430), RED),
            ('', (950, 700), (420, 430), RED),
            ('리드선 홈 (폭 7)', (750, 400), (750, 400), 'k')])
    a = fig.add_axes([0.51, 0.62, 0.47, 0.33])
    render(a, rd / 'pins_after.png', '① 고친 뒤 — 같은 시점 (홈 폭 12)', GREEN,
           [('세 핀 모두 홈 안에 떠 있음\n(판과 최소 1.25 mm)', (790, 690), (720, 430), GREEN),
            ('리드선 홈 (폭 12)', (560, 300), (560, 300), 'k')])
    across(fig.add_axes([0.05, 0.37, 0.42, 0.23]), 7.0, RED, '② 고치기 전 — 핀을 지나며 홈을 가로지른 단면')
    across(fig.add_axes([0.54, 0.37, 0.42, 0.23]), 12.0, GREEN, '② 고친 뒤 — 같은 단면')
    along(fig.add_axes([0.06, 0.04, 0.6, 0.27]))
    fig.text(0.70, 0.31, '바뀐 것 / 그대로인 것', fontsize=15, weight='bold')
    fig.text(0.70, 0.30,
             '· 홈 폭만 7 → 12 mm (반경 23~41, 바닥 Z 6 그대로)\n'
             '· 핀–판 최소 간격 0 → 1.25 mm\n'
             '  (옆벽까지 2.25, 홈 안쪽 끝까지 1.25)\n'
             '· 판 1장 -0.31 cm³ (약 -0.8 g)\n'
             '· Ø42 포켓 옆 살 2.0 mm, 다른 살도\n'
             '  1.5 mm 미만 없음 (wall_check)\n'
             '· 구멍 21개 위치·지름 그대로\n'
             '· 드라이버 컵의 리드선 슬롯(폭 9)과\n'
             '  같은 방향이라 선이 컵 안으로 이어짐\n'
             '· Ø80 등록 보스가 끊기는 길이만 7 → 12 mm\n'
             '  (둘레의 5 %, 컵 맞춤에 영향 없음)',
             fontsize=12, va='top', linespacing=1.6)
    fig.savefig(out, dpi=90, bbox_inches='tight')
    print('wrote', out)


if __name__ == '__main__':
    main()
