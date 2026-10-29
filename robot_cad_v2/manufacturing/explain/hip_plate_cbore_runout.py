"""Hip plate M2.5 counterbores before / after the 10-29 run-out into the Ø30 hole (left plate, A1 shown; all six are
the same): Fusion renders of the outer hub face and a cutaway, a close-up seen from the outer face and a radial
section, plus why the 0.86 mm wall could not be made thicker.  Explanatory figure, not a manufacturing drawing.

    python hip_plate_cbore_runout.py --before OLD_hip_plate_L.step [OUT.png]
        OLD = `git show 18c41cd:robot_cad_v2/manufacturing/step/hip_plate_L.step`; the renders in cbore_renders/
        were made in Fusion from the same two STEPs (removed material red, new side walls green, M2.5 x 6 screw).
"""
import argparse
import math
import pathlib

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Rectangle

import hip_plate_thin_walls as T

HERE = pathlib.Path(__file__).resolve().parent
RED, GREEN, BLUE = T.RED, T.GREEN, '#3a78c2'
C1 = (1.59, 18.18)                                  # A1 centre (part frame)
RC = math.hypot(*C1)                                # 18.25 = PCD 36.5 / 2
U = (C1[0] / RC, C1[1] / RC)                        # radial direction of A1
N = (-U[1], U[0])
R_BORE, R_CB, R_HEAD, R_HOLE, R_BOSS, R_OPEN = 15.0, 2.4, 2.25, 1.4, 14.0, 14.0


def view(x, y):
    """Part frame -> drawing seen from the outer face (Z = 0 towards the viewer): x mirrored."""
    return -x, y


def render(ax, path, title, colour, notes, crop=(0, 1100, 100, 1450)):
    """Fusion render (1500 x 1100 px) with labels; note positions are in the uncropped image's pixels."""
    y0, y1, x0, x1 = crop
    ax.imshow(plt.imread(path)[y0:y1, x0:x1])
    ax.set_axis_off()
    ax.set_title(title, fontsize=13, weight='bold', color=colour, pad=6)
    for (txt, xy, xyt, c) in notes:
        ax.annotate(txt, (xy[0] - x0, xy[1] - y0), (xyt[0] - x0, xyt[1] - y0), fontsize=11.5, color=c, weight='bold',
                    ha='center', va='center', bbox=dict(fc='white', ec=c, alpha=0.92, boxstyle='round,pad=0.35'),
                    arrowprops=dict(arrowstyle='->', color=c, lw=2.0) if xy != xyt else None)


def detail(ax, edges, after):
    """Close-up of A1 seen from the outer face: STEP edges, removed material, screw head seat, boss."""
    g = np.arange(-9.0, 9.0, 0.025)
    X, Y = np.meshgrid(g + C1[0] - 2.2 * U[0], g + C1[1] - 2.2 * U[1])
    R = np.hypot(X, Y)
    along, across = X * U[0] + Y * U[1], X * N[0] + Y * N[1]
    cb = np.hypot(X - C1[0], Y - C1[1]) <= R_CB
    removed = (along >= R_OPEN) & (along <= RC) & (np.abs(across) <= R_CB) & (R >= R_BORE) & ~cb
    seat = (np.hypot(X - C1[0], Y - C1[1]) <= R_HEAD) & (np.hypot(X - C1[0], Y - C1[1]) >= R_HOLE)
    VX, VY = view(X, Y)
    ax.contourf(VX, VY, seat.astype(float), levels=[0.5, 1.5], colors=[BLUE], alpha=0.28)
    if after:
        ax.contourf(VX, VY, removed.astype(float), levels=[0.5, 1.5], colors=[GREEN], alpha=0.22)
    else:
        ax.contourf(VX, VY, removed.astype(float), levels=[0.5, 1.5], colors=[RED], alpha=0.75)
    x0, x1 = VX.min(), VX.max(); y0, y1 = VY.min(), VY.max()
    for p in edges:
        z = p[:, 2].mean()
        if z > 5.6:
            continue
        vx, vy = view(p[:, 0], p[:, 1])
        if vx.max() < x0 or vx.min() > x1 or vy.max() < y0 or vy.min() > y1:
            continue
        if z < 0.05:
            ax.plot(vx, vy, color='k', lw=1.6)                          # outer hub face edges
        elif z < 2.75:
            ax.plot(vx, vy, color='0.25', lw=1.0)                       # counterbore floor / walls
        else:
            ax.plot(vx, vy, color='0.6', lw=0.8, ls='--')               # Ø2.8 hole, pocket floor (below)
    hc = view(*C1)
    ax.add_patch(Circle(hc, R_HEAD, fc='none', ec=BLUE, lw=1.6, ls=(0, (4, 2))))
    ax.add_patch(Circle((0, 0), R_BOSS, fc='none', ec='#7a4fb0', lw=1.6, ls=(0, (6, 3))))
    ax.set_xlim(x0, x1); ax.set_ylim(y0, y1); ax.set_aspect('equal'); ax.set_xticks([]); ax.set_yticks([])
    lbl = dict(fontsize=10, ha='center', va='center')
    ax.text(*view(*(c * 11.2 for c in U)), 'Ø30 구멍 (빈 공간)', **lbl)
    ax.text(*view(*(np.array(C1) + 5.2 * np.array(N) + 1.0 * np.array(U))), '판 바깥 면\n(Z = 0)', **lbl)
    bx, by = view(*(R_BOSS * np.array(U) - 3.3 * np.array(N)))
    ax.annotate('허벅지 출력 보스 Ø28 (보라 점선)\n이 구멍 안에서 돌아감, 틈 1.0', (bx, by), (bx + 2.5, by - 4.6), fontsize=10,
                color='#7a4fb0', arrowprops=dict(arrowstyle='->', color='#7a4fb0'))
    sx, sy = view(*(np.array(C1) - 1.9 * np.array(N) + 1.0 * np.array(U)))
    ax.annotate('볼트 머리 Ø4.5가 앉는 면\n(파랑, 전후 같음)', (sx, sy), (sx - 4.2, sy + 3.4), fontsize=10, color=BLUE,
                arrowprops=dict(arrowstyle='->', color=BLUE))
    if after:
        p0, p1 = view(*(R_BORE * np.array(U) - R_CB * np.array(N))), view(*(R_BORE * np.array(U) + R_CB * np.array(N)))
        ax.annotate('', p0, p1, arrowprops=dict(arrowstyle='<->', color=GREEN, lw=1.6, shrinkA=0, shrinkB=0))
        m = view(*((R_BORE - 1.6) * np.array(U)))
        ax.text(m[0], m[1], '폭 4.8 그대로 Ø30 구멍까지 터짐\n(초록 = 깎아 낸 자리)', color=GREEN, fontsize=11.5,
                weight='bold', ha='center', va='center', bbox=dict(fc='white', ec='none', alpha=0.85))
    else:
        p0, p1 = view(*(R_BORE * np.array(U))), view(*((RC - R_CB) * np.array(U)))
        ax.annotate('', p0, p1, arrowprops=dict(arrowstyle='<->', color=RED, lw=1.6, shrinkA=0, shrinkB=0))
        m = view(*((R_BORE - 1.6) * np.array(U)))
        ax.text(m[0], m[1], '살 0.86 mm (화살표)\n(빨강 = 깎아 낼 자리)', color=RED, fontsize=11.5, weight='bold',
                ha='center', va='center', bbox=dict(fc='white', ec='none', alpha=0.85))


def section(fig, rect, tri, after):
    a = T.sec_plot(fig, rect, tri,
                   (lambda S, Z: np.zeros_like(S, bool)) if after else
                   (lambda S, Z: (S > 14.95) & (S < 15.9) & (Z < 2.75)),
                   GREEN if after else RED, 'A1 볼트를 지나는 단면 (반경 방향)', (11, 24), (-2.6, 10.2))
    a.add_patch(Rectangle((RC - R_HEAD, 0.2), 2 * R_HEAD, 2.5, fc='none', ec='0.15', lw=1.4, ls='--'))
    a.add_patch(Rectangle((RC - 1.25, 2.7), 2.5, 6.0, fc='none', ec='0.15', lw=1.4, ls='--'))
    a.add_patch(Rectangle((10.0, -2.0), R_BOSS - 10.0, 7.0, fc='none', ec='#7a4fb0', lw=1.4, ls=(0, (6, 3))))
    a.text(12.0, 1.0, '허벅지\n보스', color='#7a4fb0', fontsize=9, ha='center')
    a.text(RC, 1.45, 'M2.5\n머리', fontsize=9, ha='center', va='center')
    a.text(RC, 6.0, '나사\n(점선)', fontsize=9, ha='center', va='center')
    a.text(16.0, 9.25, 'Ø42 포켓 (모터 고정링 자리)', fontsize=9, ha='center')
    a.annotate('머리 아래 판 2.8 mm\n(Z 2.7~5.5, 그대로)', (RC + 2.0, 4.1), (11.2, 6.9), fontsize=9.5, ha='left',
               arrowprops=dict(arrowstyle='->', color='0.3'))
    if after:
        a.add_patch(Rectangle((R_BORE, 0.0), 0.86, 2.7, fc='none', ec=GREEN, lw=1.6, ls=':'))
        a.annotate('여기 있던 0.86 mm 살을 깎아 냄 (초록 점선)', (R_BORE + 0.4, 1.2), (16.6, -2.0), fontsize=10,
                   color=GREEN, weight='bold', arrowprops=dict(arrowstyle='->', color=GREEN))
    else:
        T.dim(a, R_BORE, RC - 2.4, -0.4, '0.86 mm', 16.6, -2.0, RED)
    return a


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--before', required=True)
    ap.add_argument('--renders', default=str(HERE / 'cbore_renders'))
    ap.add_argument('out', nargs='?', default=str(HERE / 'hip_plate_cbore_runout.png'))
    args = ap.parse_args()
    eo, so = T.load(args.before)
    en, sn = T.load(T.NEW)
    rd = pathlib.Path(args.renders)

    fig = plt.figure(figsize=(19, 31))
    fig.text(0.5, 0.997, 'hip 판 ② — M2.5 볼트 머리 자리(카운터보어)를 Ø30 구멍 쪽으로 터 준 변경', ha='center', va='top',
             fontsize=21, weight='bold')
    fig.text(0.5, 0.986, '왼쪽 = 고치기 전,  오른쪽 = 고친 뒤.  6곳 모두 같고 확대·단면은 A1 기준.  '
             '판을 바깥 면(허벅지 쪽, Z = 0)에서 본 모습', ha='center', va='top', fontsize=13.5)
    hw, x0, x1 = 0.47, 0.02, 0.51
    # row 1: hub renders
    hub = (0, 1100, 0, 1500)
    a = fig.add_axes([x0, 0.76, hw, 0.205])
    render(a, rd / 'before_hub.png', '① 고치기 전 — 허브 바깥 면 (빨강 = 깎아 낼 살)', RED,
           [('볼트 머리 자리 Ø4.8 × 깊이 2.7', (240, 330), (330, 170), 'k'),
            ('Ø30 구멍과 머리 자리 사이\n얇은 살 0.86 mm (6곳)', (330, 400), (720, 560), RED),
            ('Ø30 구멍', (980, 700), (980, 700), 'k')], hub)
    a = fig.add_axes([x1, 0.76, hw, 0.205])
    render(a, rd / 'after_hub.png', '① 고친 뒤 — 같은 자리 (초록 = 새로 생긴 옆벽)', GREEN,
           [('볼트 구멍 Ø2.8·위치는 그대로', (240, 330), (330, 170), 'k'),
            ('머리 자리가 Ø30 구멍까지\n이어진 U자 홈 (6곳)', (330, 400), (720, 560), GREEN),
            ('Ø30 구멍', (980, 700), (980, 700), 'k')], hub)
    # row 2: cutaway renders
    a = fig.add_axes([x0, 0.53, hw, 0.205])
    render(a, rd / 'before_cut.png', '② 고치기 전 — A1에서 잘라 본 모습 (볼트 끼운 상태)', RED,
           [('M2.5 × 6 볼트 (머리 Ø4.5)', (600, 650), (420, 330), 'k'),
            ('얇은 살 0.86 mm\n(머리 옆, Ø30 구멍 쪽)', (1000, 640), (1180, 230), RED),
            ('Ø30 구멍 벽', (1300, 650), (1300, 650), 'k'),
            ('판 바깥 면 (Z = 0)', (520, 1010), (520, 1010), 'k')])
    a = fig.add_axes([x1, 0.53, hw, 0.205])
    render(a, rd / 'after_cut.png', '② 고친 뒤 — 같은 단면 (빨간 살이 없어짐)', GREEN,
           [('볼트·머리 자리 바닥은 그대로', (600, 650), (420, 330), 'k'),
            ('살이 있던 자리가 비고\n옆벽(초록)만 남음', (1000, 760), (1180, 230), GREEN),
            ('Ø30 구멍 벽', (1300, 650), (1300, 650), 'k'),
            ('판 바깥 면 (Z = 0)', (520, 1010), (520, 1010), 'k')])
    # row 3: close-ups from the outer face
    for j, (edges, after) in enumerate(((eo, False), (en, True))):
        a = fig.add_axes([x0 + j * (x1 - x0), 0.3, hw, 0.205])
        detail(a, edges, after)
        a.set_title('③ %s — 바깥 면에서 본 확대도 (A1 한 곳)' % ('고치기 전' if not after else '고친 뒤'),
                    fontsize=13, weight='bold', color=GREEN if after else RED)
    # row 4: radial sections
    section(fig, [x0 + 0.03, 0.12, hw - 0.04, 0.155], so[1], False).set_title(
        '④ 고치기 전 — A1 볼트를 지나는 단면 (점선 = 볼트, 보라 = 허벅지 보스)', fontsize=13, weight='bold', color=RED)
    section(fig, [x1 + 0.03, 0.12, hw - 0.04, 0.155], sn[1], True).set_title(
        '④ 고친 뒤 — 같은 단면', fontsize=13, weight='bold', color=GREEN)
    # why / what stays
    fig.text(0.03, 0.093, '왜 살을 두껍게 하지 않고 없앴나', fontsize=15, weight='bold')
    fig.text(0.03, 0.086,
             '살 두께 = 볼트 중심 반경 18.25 - 머리 자리 반경 2.4 - Ø30 구멍 반경 15 = 0.85 mm\n'
             '  · 볼트 중심 반경 18.25 (PCD 36.5): AKE 모터 고정링의 나사 위치라 바꿀 수 없음\n'
             '  · 머리 자리 Ø4.8: M2.5 볼트 머리가 Ø4.5라 더 줄일 수 없음 (M2.5 머리는 종류를 바꿔도 Ø4.5 이상)\n'
             '  · Ø30 구멍: 허벅지 출력 보스 Ø28이 이 안에서 돌아감 (틈 1.0 mm). 줄이면 보스에 닿을 위험\n'
             '→ 두껍게 할 수 있는 값이 없어서, 부러질 수 있는 얇은 살을 아예 깎아 냄',
             fontsize=12.5, va='top', linespacing=1.7)
    fig.text(0.53, 0.093, '바뀌지 않는 것', fontsize=15, weight='bold')
    fig.text(0.53, 0.086,
             '  · 볼트 6개의 위치(±0.05)·구멍 Ø2.8·머리 자리 지름 Ø4.8·깊이 2.7\n'
             '  · 머리가 앉는 면: 머리 바깥 반경 16.0이 구멍 반경 15보다 커서 머리 전체가 바닥에 닿음\n'
             '    (면적 9.8 mm² 그대로, 파랑)\n'
             '  · 머리 아래 판 두께 2.8 mm (Z 2.7~5.5) — 볼트가 모터 고정링을 당기는 힘을 받는 곳\n'
             '  · 허벅지 보스와 Ø30 구멍 사이 틈 1.0 mm (홈은 공간만 넓힘)\n'
             '  · 가공: 머리 자리 공구를 안쪽으로 4.25 mm 직선으로 밀면 끝. 얇은 살이 없어 떨림·버 걱정 없음',
             fontsize=12.5, va='top', linespacing=1.7)
    fig.savefig(args.out, dpi=90, bbox_inches='tight')
    print('wrote', args.out)


if __name__ == '__main__':
    main()
