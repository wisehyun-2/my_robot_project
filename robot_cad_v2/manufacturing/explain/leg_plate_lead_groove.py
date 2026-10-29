"""Knee / wheel PETG plate lead groove 7 -> 12 mm wide, start r 21.5 -> 23 (10-29, same groove as the hip plate):
the AKE60's three phase pins (Ø1.5 x 4, r 25 on the lead direction, 3 mm apart) stop 1.5 mm short of the plate face
(pocket 2.5 deep here, 4 on the hip), so they never touched, but the outer two hung over the edges of the 7 mm groove
and the soldered wires had only the 1.5 mm gap there; the groove end also left a 0.5 mm PETG fin beside the Ø42 pocket.
Renders from the motor side (leg_groove_renders/, Fusion, left thigh knee plate; the shank wheel plate is the same
section), a section across the groove through the pins and one along it.  Explanatory figure, not a drawing.
Heights Z from the plate's far face (0) to its motor-side face (9).

    python leg_plate_lead_groove.py [OUT.png]
"""
import pathlib
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle

HERE = pathlib.Path(__file__).resolve().parent
plt.rcParams['font.family'] = 'Malgun Gothic'
plt.rcParams['axes.unicode_minus'] = False
RED, GREEN, GREY, PIN = '#d62728', '#2a9d3c', '#c9ced6', '#d9d4c4'
PINS = (-3.0, 0.0, 3.0)
PIN_R, PIN_L, R_PIN = 0.75, 4.0, 25.0
T, FLOOR, TIP = 9.0, 7.0, 10.5             # plate face, groove floor (2 deep), pin tips (1.5 off the face)
POCKET_R, POCKET_Z = 21.025, 6.5           # Ø42.05 pilot pocket, 2.5 deep
RECESS_R, RECESS_Z = 22.8, 3.7             # far-face stator ring recess Ø45.6
G1 = 41.0


def render(ax, path, title, colour, notes):
    ax.imshow(plt.imread(path)[:, 150:1350])
    ax.set_axis_off()
    ax.set_title(title, fontsize=14, weight='bold', color=colour)
    for txt, xy, xyt, c in notes:
        ax.annotate(txt, (xy[0] - 150, xy[1]), (xyt[0] - 150, xyt[1]), fontsize=12, color=c, weight='bold',
                    ha='center', va='center', bbox=dict(fc='white', ec=c, alpha=0.92, boxstyle='round,pad=0.35'),
                    arrowprops=dict(arrowstyle='->', color=c, lw=2.0) if xy != xyt else None)


def across(ax, width, colour, title):
    """Section across the groove at r 25, looking outward along the groove (pins in section)."""
    h = width / 2
    plate = [(-11, 0.0), (11, 0.0), (11, T), (h, T), (h, FLOOR), (-h, FLOOR), (-h, T), (-11, T)]
    ax.add_patch(Polygon(plate, closed=True, fc=GREY, ec='k', lw=1.2))
    ax.text(-8.0, 2.0, 'PETG 판', fontsize=10, ha='center')
    ax.add_patch(Rectangle((-11, TIP + PIN_L), 22, 1.5, fc='#e8e2d0', ec='0.4', lw=0.8, hatch='///'))
    ax.text(0, TIP + PIN_L + 0.75, '모터 고정자 (핀이 나오는 면)', fontsize=10, ha='center', va='center')
    for a in PINS:
        ax.add_patch(Rectangle((a - PIN_R, TIP), 2 * PIN_R, PIN_L, fc=PIN, ec='k', lw=1.0))
    ax.text(0, TIP + 2.0, '3상 핀\nØ1.5', fontsize=9, ha='center', va='center')
    ax.annotate('', (-h, FLOOR - 0.6), (h, FLOOR - 0.6), arrowprops=dict(arrowstyle='<->', color=colour, lw=1.4,
                                                                         shrinkA=0, shrinkB=0))
    ax.text(0, FLOOR - 1.3, '홈 폭 %.0f' % width, color=colour, fontsize=12, weight='bold', ha='center', va='center')
    ax.annotate('', (8.5, T), (8.5, TIP), arrowprops=dict(arrowstyle='<->', lw=1.2, shrinkA=0, shrinkB=0))
    ax.text(8.9, (T + TIP) / 2, '1.5', fontsize=10, ha='left', va='center')
    if h < 3 + PIN_R:
        ax.annotate('바깥 핀이 홈 가장자리 위에 걸림 (0.25 mm씩)\n→ 납땜한 선이 판 면과 1.5 mm 틈에 끼임',
                    (3.6, T + 0.2), (0.0, 17.5), fontsize=10.5, color=RED, weight='bold', ha='center', va='center',
                    arrowprops=dict(arrowstyle='->', color=RED, lw=1.6))
    else:
        ax.annotate('', (3 + PIN_R, T - 0.6), (h, T - 0.6),
                    arrowprops=dict(arrowstyle='<->', color=colour, lw=1.4, shrinkA=0, shrinkB=0))
        ax.text((3 + PIN_R + h) / 2, T - 1.3, '%.2f' % (h - 3 - PIN_R), color=colour, fontsize=11.5, weight='bold',
                ha='center')
        ax.text(0, 17.6, '핀 아래 3.5 mm, 양옆 %.2f mm 빈 공간 → 납땜·선 굽힘 자리' % (h - 3 - PIN_R), color=colour,
                fontsize=11, weight='bold', ha='center', va='center')
    ax.set_xlim(-11.5, 11.5); ax.set_ylim(-0.5, 19.0); ax.set_aspect('equal')
    ax.set_xlabel('홈을 가로지르는 방향 [mm] (0 = 홈 중심)', fontsize=10)
    ax.set_ylabel('Z [mm] (판 바깥 0 → 모터 쪽 면 9)', fontsize=10)
    ax.set_title(title, fontsize=13, weight='bold', color=colour)
    ax.grid(alpha=0.25); ax.tick_params(labelsize=9)


def along(ax, g0, colour, title):
    """Section along the groove axis: Ø30 hole, far-face recess, Ø42 pocket, wall to the groove, groove, pin, can,
    axial lead slot in the housing wall (r 35 .. 41, past the can r 34.5), wire path to the driver cup."""
    plate = [(15.0, RECESS_Z), (RECESS_R, RECESS_Z), (RECESS_R, 0.0), (44.0, 0.0), (44.0, 31.0), (G1, 31.0),
             (G1, FLOOR), (g0, FLOOR), (g0, T), (POCKET_R, T), (POCKET_R, POCKET_Z), (15.0, POCKET_Z)]
    ax.add_patch(Polygon(plate, closed=True, fc=GREY, ec='k', lw=1.2))
    ax.text(30.0, 3.2, 'PETG 판 (두께 9)', fontsize=10, ha='center')
    ax.text(42.5, 20.0, '하우징\n벽\nr 41\n~44', fontsize=8.5, ha='center', va='center')
    ax.add_patch(Rectangle((R_PIN - PIN_R, TIP), 2 * PIN_R, PIN_L, fc=PIN, ec='k', lw=1.0))
    ax.add_patch(Rectangle((20.3, TIP + PIN_L), 10.45, 1.5, fc='#e8e2d0', ec='0.4', lw=0.8, hatch='///'))
    ax.text(25.5, TIP + PIN_L + 0.75, '고정자', fontsize=9.5, ha='center', va='center')
    ax.add_patch(Rectangle((33.0, 14.0), 1.5, 14.7, fc='#9aa0aa', ec='0.3', lw=0.8))
    ax.text(32.6, 24.0, '로터 캔\n(돌아감)', fontsize=9, ha='right', va='center')
    ax.plot([37.0, 37.0], [T, 31.0], color='0.35', lw=1.0, ls=':')
    ax.text(36.6, 29.6, '캔 공간 r 37', fontsize=8.5, ha='right', color='0.3')
    ax.add_patch(Rectangle((35.0, FLOOR), 6.0, 31.0 - FLOOR, fc='none', ec='#3a78c2', lw=1.2, ls='--'))
    ax.text(38.0, 32.0, '세로 슬롯 r 35~41, 폭 7\n→ 드라이버 컵 슬롯(폭 9)', fontsize=9, ha='center', va='bottom',
            color='#3a78c2')
    path = [(R_PIN, TIP + 0.1), (R_PIN + 0.4, 8.0), (27.5, 7.8), (38.0, 7.8), (39.2, 9.5), (39.2, 30.5)]
    xs, ys = zip(*path)
    ax.plot(xs, ys, color='#e07b00', lw=3.2, solid_capstyle='round')
    ax.annotate('납땜한 상 선: 홈 바닥을 따라 바깥으로 →\n캔 바깥 슬롯을 따라 위로 → 드라이버', (32.0, 7.8), (29.0, -3.6),
                fontsize=10.5, color='#e07b00', weight='bold', ha='center', annotation_clip=False,
                arrowprops=dict(arrowstyle='->', color='#e07b00'))
    wall = g0 - POCKET_R
    ax.annotate('', (POCKET_R, T + 0.6), (g0, T + 0.6), arrowprops=dict(arrowstyle='<->', lw=1.2, color=colour,
                                                                       shrinkA=0, shrinkB=0))
    ax.text((POCKET_R + g0) / 2, T + 1.9 if wall < 1 else T + 1.2, '%.1f' % wall, fontsize=11, ha='center',
            color=colour, weight='bold')
    ax.annotate('', (29.0, FLOOR), (29.0, T), arrowprops=dict(arrowstyle='<->', lw=1.2, shrinkA=0, shrinkB=0))
    ax.text(29.3, 8.0, '2', fontsize=10, ha='left', va='center')
    ax.text(18.0, 8.3, 'Ø42 포켓\n(모터 링)', fontsize=9.5, ha='center')
    ax.set_xlim(14, 47); ax.set_ylim(-5, 36); ax.set_aspect('equal')
    ax.set_xlabel('모터 축에서 반경 [mm] (홈 방향)', fontsize=10)
    ax.set_ylabel('Z [mm]', fontsize=10)
    ax.set_title(title, fontsize=13, weight='bold', color=colour)
    ax.grid(alpha=0.25); ax.tick_params(labelsize=9)


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else HERE / 'leg_plate_lead_groove.png'
    rd = HERE / 'leg_groove_renders'
    fig = plt.figure(figsize=(18, 24))
    fig.text(0.5, 0.995, 'knee·wheel PETG 판 리드선 홈: 폭 7 → 12 mm, 시작 반경 21.5 → 23 mm (hip 판과 같은 홈)',
             ha='center', va='top', fontsize=20, weight='bold')
    fig.text(0.5, 0.980, '허벅지 knee 판(왼쪽)으로 그림. 정강이 wheel 판도 같은 단면이고 오른쪽은 거울상. '
             '핀 끝은 판 면에서 1.5 mm 떨어져 있어 판에 닿지는 않았음', ha='center', va='top', fontsize=13)
    a = fig.add_axes([0.02, 0.655, 0.47, 0.295])
    render(a, rd / 'knee_before.png', '① 고치기 전 — 모터 쪽에서 본 모습 (홈 폭 7)', RED,
           [('바깥 핀 2개가 홈\n가장자리 위에 걸림', (575, 800), (330, 560), RED),
            ('', (925, 800), (330, 560), RED),
            ('포켓 옆 살 0.5 mm', (750, 1022), (1130, 940), RED),
            ('세로 슬롯(폭 7) → 컵', (750, 150), (750, 150), 'k')])
    a = fig.add_axes([0.51, 0.655, 0.47, 0.295])
    render(a, rd / 'knee_after.png', '① 고친 뒤 — 같은 시점 (홈 폭 12)', GREEN,
           [('세 핀 모두 홈 안\n(옆벽까지 2.25 mm)', (900, 800), (1230, 600), GREEN),
            ('포켓 옆 살 2.0 mm', (750, 985), (1130, 1040), GREEN),
            ('세로 슬롯(폭 7) → 컵', (750, 150), (750, 150), 'k')])
    across(fig.add_axes([0.05, 0.45, 0.42, 0.19]), 7.0, RED, '② 고치기 전 — 핀을 지나며 홈을 가로지른 단면')
    across(fig.add_axes([0.54, 0.45, 0.42, 0.19]), 12.0, GREEN, '② 고친 뒤 — 같은 단면')
    along(fig.add_axes([0.03, 0.06, 0.46, 0.36]), 21.5, RED, '③ 고치기 전 — 홈 방향 단면 (포켓 옆 살 0.5)')
    along(fig.add_axes([0.52, 0.06, 0.46, 0.36]), 23.0, GREEN, '③ 고친 뒤 — 홈 방향 단면 (포켓 옆 살 2.0)')
    fig.text(0.05, 0.035,
             '바뀐 것: 홈 폭 7 → 12, 시작 반경 21.5 → 23 (깊이 2, 끝 반경 41, 세로 슬롯 폭 7 그대로).  '
             '핀–판 최소 1.5 → 1.95 mm, 홈 옆벽까지 2.25 mm.\n'
             '          Ø42 포켓 옆 PETG 살 0.5 → 2.0 mm (0.5 mm는 0.4 노즐로 제대로 나오지 않는 두께).\n'
             '그대로인 것: 구멍·포켓·나사 위치, 컵 인서트, 링크 바깥 형상 — 깎아 내기만 해서 짝 부품 조립 위치는 그대로.',
             fontsize=12.5, va='top', linespacing=1.7)
    fig.savefig(out, dpi=90, bbox_inches='tight')
    print('wrote', out)


if __name__ == '__main__':
    main()
