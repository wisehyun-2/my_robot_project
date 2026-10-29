"""Hip plate thin walls, before / after the 10-29 fix: the three spots under 1 mm on the left plate (lead groove
next to the Ø42 pocket, M2.5 counterbores next to the Ø30 hole, M5 countersinks next to the rim), each as a radial
section through the hip axis, old plate left, new plate right.  Explanatory figure, not a manufacturing drawing.
gmsh (OCC) only, so it runs in the repo .venv.

    python hip_plate_thin_walls.py --before OLD_hip_plate_L.step [OUT.png]
        OLD = the quote STEP before the fix, e.g. `git show 18c41cd:robot_cad_v2/manufacturing/step/hip_plate_L.step`
"""
import argparse
import math
import pathlib

import gmsh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection, PolyCollection
from matplotlib.patches import Circle

HERE = pathlib.Path(__file__).resolve().parent
NEW = HERE.parent / 'step' / 'hip_plate_L.step'
plt.rcParams['font.family'] = 'Malgun Gothic'
plt.rcParams['axes.unicode_minus'] = False
RED, GREEN = '#d62728', '#2a9d3c'

G = math.radians(235.0)                 # lead groove P1 direction (left plate)
UG = (math.cos(G), math.sin(G))
A1 = (1.59, 18.18)                      # M2.5 counterbore A1 (one of six)
UA = (A1[0] / math.hypot(*A1), A1[1] / math.hypot(*A1))
E1 = (-47.0, 40.0)                      # M5 countersink E1 (one of two)
UE = (E1[0] / math.hypot(*E1), E1[1] / math.hypot(*E1))
CUTS = [(UG, 14.0, 44.0), (UA, 12.0, 24.0), (UE, 52.0, 72.0)]


def load(path):
    """Top-view edge polylines and the three radial sections (triangles in (r, Z)) of one plate STEP."""
    gmsh.initialize(); gmsh.option.setNumber('General.Terminal', 0)
    gmsh.model.occ.importShapes(str(path)); gmsh.model.occ.synchronize()
    vol = gmsh.model.getEntities(3)[0]
    edges = []
    for (d, t) in gmsh.model.getEntities(1):
        lo, hi = gmsh.model.getParametrizationBounds(d, t)
        p = np.array(gmsh.model.getValue(d, t, list(np.linspace(lo[0], hi[0], 60)))).reshape(-1, 3)
        edges.append(p)
    tags = []
    for u, s0, s1 in CUTS:
        rect = gmsh.model.occ.addRectangle(s0, -1.0, 0.0, s1 - s0, 12.0)              # x = r, y = Z
        gmsh.model.occ.rotate([(2, rect)], 0, 0, 0, 1, 0, 0, math.pi / 2)            # (r, Z, 0) -> (r, 0, Z)
        gmsh.model.occ.rotate([(2, rect)], 0, 0, 0, 0, 0, 1, math.atan2(u[1], u[0]))
        out, _ = gmsh.model.occ.intersect([vol], [(2, rect)], removeObject=False, removeTool=True)
        tags.append([t for (d, t) in out if d == 2])
    gmsh.model.occ.synchronize()
    gmsh.option.setNumber('Mesh.MeshSizeMax', 0.4)
    gmsh.option.setNumber('Mesh.MeshSizeFromCurvature', 20)
    gmsh.model.mesh.generate(2)
    nt, xyz, _ = gmsh.model.mesh.getNodes()
    P = np.zeros((int(nt.max()) + 1, 3)); P[nt.astype(int)] = xyz.reshape(-1, 3)
    secs = []
    for ts, (u, _, _) in zip(tags, CUTS):
        tri = np.concatenate([P[gmsh.model.mesh.getElementsByType(2, t)[1].astype(int).reshape(-1, 3)] for t in ts])
        secs.append(np.stack([tri[..., 0] * u[0] + tri[..., 1] * u[1], tri[..., 2]], axis=-1))
    gmsh.finalize()
    return edges, secs


def sec_plot(fig, rect, tri, mark, colour, title, xlim, ylim):
    a = fig.add_axes(rect)
    c = tri.mean(axis=1)
    hot = mark(c[:, 0], c[:, 1])
    col = np.where(hot[:, None], np.array(matplotlib.colors.to_rgba(colour)), np.array([0.79, 0.81, 0.84, 1.0]))
    a.add_collection(PolyCollection(tri, facecolors=col, edgecolors=col, linewidths=0.3))
    e = np.concatenate([tri[:, [0, 1]], tri[:, [1, 2]], tri[:, [2, 0]]])          # outline = edges used once
    er = np.round(e, 4)
    swap = (er[:, 0, 0] > er[:, 1, 0]) | ((er[:, 0, 0] == er[:, 1, 0]) & (er[:, 0, 1] > er[:, 1, 1]))
    er[swap] = er[swap][:, ::-1]
    _, inv, cnt = np.unique(er.reshape(-1, 4), axis=0, return_inverse=True, return_counts=True)
    a.add_collection(LineCollection(e[cnt[inv.ravel()] == 1], colors='k', linewidths=0.9))
    a.set_xlim(*xlim); a.set_ylim(*ylim); a.set_aspect('equal')
    a.set_title(title, fontsize=11.5, weight='bold', color=colour)
    a.set_xlabel('hip 축에서 반경 r [mm]', fontsize=9)
    a.set_ylabel('Z [mm] (0 바깥 → 9.5 안쪽)', fontsize=9)
    a.tick_params(labelsize=8); a.grid(alpha=0.25)
    return a


def dim(a, x0, x1, y, txt, tx, ty, colour):
    a.annotate('', (x0, y), (x1, y), arrowprops=dict(arrowstyle='<->', color=colour, lw=1.4, shrinkA=0, shrinkB=0))
    a.annotate(txt, ((x0 + x1) / 2, y), (tx, ty), color=colour, fontsize=12, weight='bold',
               arrowprops=dict(arrowstyle='-', color=colour, lw=0.8))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--before', required=True)
    ap.add_argument('out', nargs='?', default=str(HERE / 'hip_plate_thin_walls.png'))
    args = ap.parse_args()
    _, old = load(args.before)
    edges, new = load(NEW)

    fig = plt.figure(figsize=(20, 14))
    fig.text(0.5, 0.995, '왼쪽 hip 판의 얇은 살 3곳: 고치기 전(빨강) → 고친 뒤(초록)   (오른쪽 판은 거울상, 같은 자리)',
             ha='center', va='top', fontsize=18, weight='bold')
    fig.text(0.5, 0.965, '고친 판에는 1.5 mm 미만 살이 없음 (manufacturing/wall_check.py). 가장 얇은 곳은 A 구멍 Ø2.8 ↔ Ø30 구멍 1.85 mm '
             '(볼트 머리 아래, 원래 그대로).  구멍 위치·지름은 하나도 바뀌지 않음.', ha='center', va='top', fontsize=11.5)

    # top view of the new plate
    ax = fig.add_axes([0.0, 0.03, 0.33, 0.88]); ax.set_aspect('equal'); ax.axis('off')
    for p in edges:
        top = p[:, 2].mean() > 7.9
        ax.plot(p[:, 0], p[:, 1], color='0.2' if top else '0.7', lw=0.9 if top else 0.45, zorder=2 if top else 1)
    spots = [(23.0 * UG[0], 23.0 * UG[1]), (15.4 * UA[0], 15.4 * UA[1]), (65.5 * UE[0], 65.5 * UE[1])]
    for c in spots:
        ax.add_patch(Circle(c, 6.5, fc='none', ec=GREEN, lw=2.5, zorder=5))
    kw = dict(fontsize=12, color=GREEN, weight='bold', zorder=6, bbox=dict(fc='white', ec='none', alpha=0.85, pad=1.5))
    ax.annotate('①  리드선 홈 시작 r 21.5 → 23\n     포켓 옆 살 0.49 → 2.0 mm', spots[0], (-100, -90),
                arrowprops=dict(arrowstyle='->', color=GREEN, lw=1.6), **kw)
    ax.annotate('②  카운터보어를 Ø30 쪽으로 터서\n     0.86 mm 살을 없앰 (6곳)', spots[1], (36, 92),
                arrowprops=dict(arrowstyle='->', color=GREEN, lw=1.6), **kw)
    ax.annotate('③  E 구멍에 귀 R8 추가\n     테두리 살 0.74 → 2.4 mm (2곳)', spots[2], (-100, 92),
                arrowprops=dict(arrowstyle='->', color=GREEN, lw=1.6), **kw)
    ax.text(0, 160, '고친 판, 안쪽 면(로봇 중심 쪽)에서 본 모습, 로봇 앞(+x) →', ha='center', fontsize=11)
    ax.set_xlim(-100, 100); ax.set_ylim(-98, 168)

    rows = [  # mark old, mark new, titles, limits, dims old / new, labels
        (lambda S, Z: (S > 20.9) & (S < 21.7) & (Z > 5.9), lambda S, Z: (S > 20.9) & (S < 23.1) & (Z > 5.9),
         '① 리드선 홈 방향(235°) 단면', (14, 44), (-0.5, 12.5),
         (21.006, 21.515, 10.2, '0.49 mm', 25.0, 11.2), (21.006, 23.0, 10.2, '2.0 mm', 26.0, 11.2),
         [(18.0, 7.5, 'Ø42 포켓'), (32.0, 7.3, '리드선 홈 (바닥 Z=6)')]),
        (lambda S, Z: (S > 14.95) & (S < 15.9) & (Z < 2.75), lambda S, Z: np.zeros_like(S, bool),
         '② A1 구멍 쪽 단면', (12, 24), (-2.8, 10.2),
         (15.0, 15.86, -0.4, '0.86 mm', 12.4, -2.3), None,
         [(13.5, 5.0, 'Ø30\n관통'), (18.0, 7.8, 'Ø42 포켓'), (18.25, 1.0, '카운터\n보어')]),
        (lambda S, Z: (S > 66.0) & (S < 68.1) & (Z < 4.5), lambda S, Z: (S > 66.0) & (Z < 4.5),
         '③ E1 구멍 쪽 단면', (52, 72), (-0.5, 10.0),
         (67.27, 68.01, 2.5, '0.74 mm', 63.0, 0.5), (67.32, 69.72, 2.5, '2.4 mm', 63.5, 0.5),
         [(61.7, 6.8, 'M5 볼트\n구멍'), (56.0, 6.5, '웹')]),
    ]
    for k, (mo, mn, title, xl, yl, do, dn, labels) in enumerate(rows):
        y0 = 0.655 - 0.315 * k
        for j, (tri, mark, colour, tag, dd) in enumerate(((old[k], mo, RED, '고치기 전', do),
                                                           (new[k], mn, GREEN, '고친 뒤', dn))):
            a = sec_plot(fig, [0.36 + 0.325 * j, y0, 0.30, 0.25], tri, mark, colour, '%s — %s' % (title, tag), xl, yl)
            if dd:
                dim(a, *dd, colour)
            for (x, y, t) in labels:
                a.text(x, y, t, fontsize=9, ha='center')
        if k == 1:
            fig.text(0.36 + 0.325 + 0.15, y0 + 0.02, '카운터보어가 Ø30 구멍과 이어져 얇은 벽이 없음\n(볼트 머리 자리 바닥 Z=2.7은 그대로)',
                     fontsize=10.5, color=GREEN, ha='center')
    fig.savefig(args.out, dpi=100, bbox_inches='tight')
    print('wrote', args.out)


if __name__ == '__main__':
    main()
