"""Hip plate screw map: the left plate (quote STEP, seen from inside) with every hole coloured by the screw that goes in
it.  Explanatory figure, not a manufacturing drawing.  Run make_parts.py first.

    python hip_plate_screws.py       # -> hip_plate_screws.png next to this script
"""
import pathlib
import sys
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / 'fusion'))
sys.path.insert(0, str(HERE.parent))
import occ as O
import drawings as D
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
sh = O.solids(O.read_step(HERE.parent / 'step' / 'hip_plate_L.step'))[0]
holes = D.hip_holes(sh)
SPEC = {   # kind: (colour, screw, qty per plate, where it threads, head side)
    'A': ('#d62728', 'ISO 4762 M2.5 × 6 (12.9급)', 'hip 모터 고정자 → 모터 고정링 탭', '바깥 면(허벅지 쪽)에서'),
    'B': ('#1f77b4', 'ISO 4762 M3 × 40', '드라이버 컵 → hip 판 M3 탭', '안쪽(컵 바닥)에서'),
    'C': ('#2ca02c', 'ISO 4762 M3 × 8', '전장 데크 플랜지 → hip 판 M3 탭', '안쪽(데크 플랜지)에서'),
    'D': ('#9467bd', 'ISO 7380 M3 × 6 (버튼)', '셸 → hip 판 M3 탭', '바깥(셸 위)에서'),
    'E': ('#ff7f0e', 'ISO 10642 M5 × 12 (접시)', 'hip 판 → 척추 관 마개 M5 탭', '바깥 면(접시 자리)에서'),
    'F': ('#8c564b', 'ISO 10642 M4 × 12 (접시)', 'hip 판 → 레일 마개 M4 탭', '바깥 면(접시 자리)에서'),
}
fig = plt.figure(figsize=(17, 11))
ax = fig.add_axes([0.0, 0.02, 0.5, 0.92]); ax.set_aspect('equal'); ax.axis('off')
vis, hid = O.project(sh, (0, 0, 1), (1, 0, 0))
for pl in hid:
    xs, ys = zip(*pl); ax.plot(xs, ys, color='0.75', lw=0.4, ls='--')
for pl in vis:
    xs, ys = zip(*pl); ax.plot(xs, ys, color='0.25', lw=0.8)
count = {}
for (k, x, y) in holes:
    c = SPEC[k][0]; count[k] = count.get(k, 0) + 1
    r = {'A': 3.2, 'B': 2.6, 'C': 2.6, 'D': 2.6, 'E': 5.6, 'F': 4.5}[k]
    ax.add_patch(Circle((x, y), r, fc=c, ec='k', lw=0.6, alpha=0.9, zorder=5))
    ax.text(x, y, k, ha='center', va='center', fontsize=7 if k in 'ABCD' else 9, color='white', weight='bold', zorder=6)
ax.text(0, 160, '왼쪽 hip 판 — 로봇 안쪽(가운데)에서 본 모습, 로봇 앞(+x) →', ha='center', fontsize=12)
ax.text(0, -78, '가운데 Ø42 포켓 = hip 모터 자리 (모터는 이쪽, 로봇 안쪽에 붙음)', ha='center', fontsize=10)
ax.set_xlim(-80, 80); ax.set_ylim(-85, 170)
# legend table
bx = fig.add_axes([0.5, 0.05, 0.49, 0.88]); bx.axis('off'); bx.set_xlim(0, 100); bx.set_ylim(0, 100)
bx.text(0, 97, 'hip 판 1장에 들어가는 나사 (좌우 2장이면 ×2)', fontsize=14, weight='bold')
y = 89
for k, (c, screw, what, head) in SPEC.items():
    n = sum(1 for h in holes if h[0] == k)
    bx.add_patch(Circle((3, y), 1.8, fc=c, ec='k', lw=0.6))
    bx.text(3, y, k, ha='center', va='center', color='white', fontsize=10, weight='bold')
    bx.text(7, y + 1.6, '%s  × %d개' % (screw, n), fontsize=12.5, weight='bold', va='center')
    bx.text(7, y - 2.2, '%s  ·  머리: %s' % (what, head), fontsize=10.5, va='center')
    y -= 11
bx.text(0, y + 2, 'hip 판에 나사산(탭)이 있는 곳: B·C·D (M3 11개)\nA·E·F는 판을 지나가기만 하고, 나사산은 모터·마개 쪽에 있음\nA는 반드시 12.9급 강철 볼트 (스테인리스 A2 금지)\n경량 구멍 없음 (가공비 절감으로 제거)',
        fontsize=11, va='top', linespacing=1.6)
fig.savefig(sys.argv[1] if len(sys.argv) > 1 else HERE / 'hip_plate_screws.png', dpi=100, bbox_inches='tight')
