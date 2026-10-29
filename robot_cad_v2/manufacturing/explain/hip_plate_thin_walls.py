"""Hip plate thin walls: where the left plate (quote STEP) has walls under ~1 mm, as a top view, 3D close-ups and
sections.  Explanatory figure, not a manufacturing drawing.  Uses gmsh (OCC) instead of OCP, so it runs in the repo
.venv.  Run make_parts.py first.

    python hip_plate_thin_walls.py       # -> hip_plate_thin_walls.png next to this script
"""
import math
import pathlib
import sys

import gmsh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection, PolyCollection
from matplotlib.patches import Circle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE = pathlib.Path(__file__).resolve().parent
STEP = HERE.parent / 'step' / 'hip_plate_L.step'
plt.rcParams['font.family'] = 'Malgun Gothic'
plt.rcParams['axes.unicode_minus'] = False
RED = '#d62728'

gmsh.initialize(); gmsh.option.setNumber('General.Terminal', 0)
gmsh.model.occ.importShapes(str(STEP)); gmsh.model.occ.synchronize()
VOL = gmsh.model.getEntities(3)[0]


def section_cut(u, s0, s1):
    """section faces of the plate on the vertical half plane through the hip axis along unit u, r in [s0, s1]"""
    rect = gmsh.model.occ.addRectangle(s0, -1.0, 0.0, s1 - s0, 12.0)          # x = r, y = Z
    gmsh.model.occ.rotate([(2, rect)], 0, 0, 0, 1, 0, 0, math.pi / 2)         # (r, Z, 0) -> (r, 0, Z)
    gmsh.model.occ.rotate([(2, rect)], 0, 0, 0, 0, 0, 1, math.atan2(u[1], u[0]))
    out, _ = gmsh.model.occ.intersect([VOL], [(2, rect)], removeObject=False, removeTool=True)
    return [t for (d, t) in out if d == 2]


def section_tris(tags, u):
    """mesh triangles of the section faces in (r, Z)"""
    tri = []
    for t in tags:
        _, nt = gmsh.model.mesh.getElementsByType(2, t)
        tri.append(P[nt.astype(int).reshape(-1, 3)])
    tri = np.concatenate(tri)
    return np.stack([tri[..., 0] * u[0] + tri[..., 1] * u[1], tri[..., 2]], axis=-1)


# --- thin-wall spots (left plate, hole table frame: origin = hip axis, Z = 0 outside, 9.5 inside) ---
G = math.radians(235.0)                 # lead groove P1 direction
UG = (math.cos(G), math.sin(G))
A1 = (1.59, 18.18)                      # AKE ring M2.5 counterbore (one of six)
UA = (A1[0] / math.hypot(*A1), A1[1] / math.hypot(*A1))
E1 = (-47.0, 40.0)                      # spine tube plug M5 countersink (one of two)
UE = (E1[0] / math.hypot(*E1), E1[1] / math.hypot(*E1))
SPOTS = [  # centre xy, close-up half size, view (elev, azim), seen from outside?
    ((21.25 * UG[0], 21.25 * UG[1]), 7.5, (42, 275), False),
    ((15.4 * UA[0], 15.4 * UA[1]), 6.0, (-52, -100), True),
    ((64.5 * UE[0], 64.5 * UE[1]), 8.0, (-45, 170), True),
]

fig = plt.figure(figsize=(20, 14.5))
fig.text(0.5, 0.995, '왼쪽 hip 판(hip_plate_L)에서 살이 1 mm 안팎으로 얇은 곳 3군데  (오른쪽 판은 거울상, 같은 자리)',
         ha='center', va='top', fontsize=18, weight='bold')

# --- 1. top view (wireframe of every edge, seen from inside like drawing sheet 1) ---
ax = fig.add_axes([0.0, 0.03, 0.36, 0.9]); ax.set_aspect('equal'); ax.axis('off')
for (d, t) in gmsh.model.getEntities(1):
    lo, hi = gmsh.model.getParametrizationBounds(d, t)
    p = np.array(gmsh.model.getValue(d, t, list(np.linspace(lo[0], hi[0], 60)))).reshape(-1, 3)
    top = p[:, 2].mean() > 7.9
    ax.plot(p[:, 0], p[:, 1], color='0.2' if top else '0.7', lw=0.9 if top else 0.45, zorder=2 if top else 1)
for c, _, _, _ in SPOTS:
    ax.add_patch(Circle(c, 6.5, fc='none', ec=RED, lw=2.5, zorder=5))
kw = dict(fontsize=12.5, color=RED, weight='bold', zorder=6,
          bbox=dict(fc='white', ec='none', alpha=0.85, pad=1.5))
ax.annotate('①  Ø42 포켓 ↔ 리드선 홈\n     사이 살 0.51 mm', SPOTS[0][0], (-98, -88),
            arrowprops=dict(arrowstyle='->', color=RED, lw=1.6), **kw)
ax.annotate('②  카운터보어 ↔ Ø30 구멍\n     사이 살 0.86 mm (6곳)', SPOTS[1][0], (40, 92),
            arrowprops=dict(arrowstyle='->', color=RED, lw=1.6), **kw)
ax.annotate('③  접시 자리 ↔ 바깥 테두리\n     최소 0.74 mm (2곳)', SPOTS[2][0], (-98, 92),
            arrowprops=dict(arrowstyle='->', color=RED, lw=1.6), **kw)
ax.text(0, 157, '안쪽 면(로봇 중심 쪽)에서 본 모습, 로봇 앞(+x) →\n진한 선 = 안쪽 면 모서리, 연한 선 = 그 아래 모서리',
        ha='center', fontsize=11)
ax.set_xlim(-100, 100); ax.set_ylim(-95, 168)

# --- 2. surface mesh for the 3D close-ups ---
gmsh.option.setNumber('Mesh.MeshSizeMax', 0.35)
gmsh.option.setNumber('Mesh.MeshSizeMin', 0.1)
gmsh.option.setNumber('Mesh.MeshSizeFromCurvature', 20)
gmsh.model.mesh.generate(2)
tags, xyz, _ = gmsh.model.mesh.getNodes()
P = np.zeros((int(tags.max()) + 1, 3)); P[tags.astype(int)] = xyz.reshape(-1, 3)
_, ntri = gmsh.model.mesh.getElementsByType(2)
TRI = P[ntri.astype(int).reshape(-1, 3)]
# sections: cut, re-mesh (the plate faces too, cheap) and keep the section triangles
gmsh.model.mesh.clear()
SEC_TAGS = [section_cut(UG, 14, 44), section_cut(UA, 12, 24), section_cut(UE, 52, 69)]
gmsh.model.occ.synchronize()
gmsh.option.setNumber('Mesh.MeshSizeMax', 0.4)
gmsh.model.mesh.generate(2)
tags, xyz, _ = gmsh.model.mesh.getNodes()
P = np.zeros((int(tags.max()) + 1, 3)); P[tags.astype(int)] = xyz.reshape(-1, 3)
SEC = [section_tris(t, u) for t, u in zip(SEC_TAGS, (UG, UA, UE))]
CEN = TRI.mean(axis=1)
NRM = np.cross(TRI[:, 1] - TRI[:, 0], TRI[:, 2] - TRI[:, 0])
NRM /= np.linalg.norm(NRM, axis=1, keepdims=True) + 1e-12
R = np.hypot(CEN[:, 0], CEN[:, 1]); ANG = np.arctan2(CEN[:, 1], CEN[:, 0])


def ang_d(a, b):
    return np.abs((a - b + np.pi) % (2 * np.pi) - np.pi)


def thin_mask(k):
    """triangles on the faces that bound thin wall k"""
    if k == 0:   # sliver between the Ø42 pocket (r 21.006) and the groove start (r 21.5), Z 6 ~ 9.5
        return (R > 20.9) & (R < 21.6) & (ang_d(ANG, G) < 3.6 / 21.25) & (CEN[:, 2] > 5.95)
    if k == 1:   # Ø30 bore / Ø4.8 counterbore walls, Z 0 ~ 2.7, next to each of the six A holes
        aa = np.array([math.atan2(y, x) for x, y in [(1.59, 18.18), (-14.95, 10.47), (16.54, 7.71),
                                                     (-16.54, -7.71), (14.95, -10.47), (-1.59, -18.18)]])
        near = np.min(np.stack([ang_d(ANG, a) for a in aa]), axis=0) < 2.6 / 15.5
        return near & (R > 14.9) & (R < 16.2) & (CEN[:, 2] < 2.75)
    # countersink cone / outer rim beside E1, E2 (Z 3 ~ 4.5, where the wall is under ~2 mm)
    m = np.zeros(len(CEN), bool)
    for ex, ey in [(-47.0, 40.0), (47.0, 40.0)]:
        a = math.atan2(ey, ex)
        m |= (ang_d(ANG, a) < 4.0 / 64.0) & (R > 63.5) & (CEN[:, 2] < 4.5) & (CEN[:, 2] > 2.9)
    return m


def closeup(rect, k, c, half, view, outside):
    a3 = fig.add_axes(rect, projection='3d')
    sel = (np.abs(CEN[:, 0] - c[0]) < half) & (np.abs(CEN[:, 1] - c[1]) < half)
    # keep the faces on the viewed side only (the painter's sort in mplot3d mixes far faces in)
    sel &= (CEN[:, 2] < 5.6) if outside else (CEN[:, 2] > 5.4)
    tri = TRI[sel]; n = NRM[sel]
    light = np.array([0.35, -0.45, -0.82 if outside else 0.82]); light /= np.linalg.norm(light)
    shade = 0.35 + 0.6 * np.abs(n @ light)
    col = np.stack([shade * 0.78, shade * 0.80, shade * 0.84, np.ones_like(shade)], axis=1)
    col[thin_mask(k)[sel]] = [0.86, 0.12, 0.12, 1.0]
    a3.add_collection3d(Poly3DCollection(tri, facecolors=col, edgecolors='none'))
    zl = (-0.5, 6.0) if outside else (5.0, 10.0)
    a3.set_xlim(c[0] - half, c[0] + half); a3.set_ylim(c[1] - half, c[1] + half); a3.set_zlim(*zl)
    a3.set_box_aspect((1, 1, (zl[1] - zl[0]) / (2 * half)), zoom=1.15)
    a3.view_init(*view); a3.axis('off')
    a3.text2D(0.5, 0.97, '3D 확대 (%s에서 봄), 빨간 면 = 얇은 살' % ('바깥 면' if outside else '안쪽 면'),
              transform=a3.transAxes, ha='center', fontsize=11)
    return a3


def sec_plot(rect, tri, red, title, xlim, ylim):
    a = fig.add_axes(rect)
    c = tri.mean(axis=1)
    isred = red(c[:, 0], c[:, 1])
    col = np.where(isred[:, None], np.array(matplotlib.colors.to_rgba(RED)), np.array([0.79, 0.81, 0.84, 1.0]))
    a.add_collection(PolyCollection(tri, facecolors=col, edgecolors=col, linewidths=0.3))
    # outline = triangle edges used once
    e = np.concatenate([tri[:, [0, 1]], tri[:, [1, 2]], tri[:, [2, 0]]])
    er = np.round(e, 4)
    swap = (er[:, 0, 0] > er[:, 1, 0]) | ((er[:, 0, 0] == er[:, 1, 0]) & (er[:, 0, 1] > er[:, 1, 1]))
    er[swap] = er[swap][:, ::-1]
    _, inv, cnt = np.unique(er.reshape(-1, 4), axis=0, return_inverse=True, return_counts=True)
    a.add_collection(LineCollection(e[cnt[inv.ravel()] == 1], colors='k', linewidths=0.9))
    a.set_xlim(*xlim); a.set_ylim(*ylim)
    a.set_aspect('equal'); a.set_title(title, fontsize=11)
    a.set_xlabel('hip 축에서 반경 r [mm]', fontsize=9.5)
    a.set_ylabel('Z [mm]  (0 바깥 면 → 9.5 안쪽 면)', fontsize=9.5)
    a.tick_params(labelsize=8.5); a.grid(alpha=0.25)
    return a


def dim(a, x0, x1, y, txt, tx, ty):
    a.annotate('', (x0, y), (x1, y), arrowprops=dict(arrowstyle='<->', color=RED, lw=1.4, shrinkA=0, shrinkB=0))
    a.annotate(txt, ((x0 + x1) / 2, y), (tx, ty), color=RED, fontsize=12.5, weight='bold',
               arrowprops=dict(arrowstyle='-', color=RED, lw=0.8))


def row(top, k, head, note):
    fig.text(0.375, top, head, fontsize=14, weight='bold', color=RED, va='top')
    fig.text(0.375, top - 0.024, note, fontsize=10.5, va='top', linespacing=1.4)
    closeup([0.37, top - 0.315, 0.29, 0.255], k, *SPOTS[k])


# row 1: pocket / groove sliver
row(0.955, 0, '①  Ø42 모터 포켓 ↔ 리드선 홈 P1 사이 살: 0.51 mm  ← 가장 얇음',
    'Ø42 포켓(모터 고정링 자리, 깊이 4)의 벽 바로 바깥에서 리드선 홈(폭 7, 깊이 3.5)이 반경 21.5부터 시작해서, 둘 사이에\n'
    '두께 0.51 × 높이 3.5 × 폭 7 mm의 얇은 지느러미 같은 벽이 혼자 서 있음. 가공 중 떨리거나 휘고, 조립 때 쉽게 찌그러짐.')
a = sec_plot([0.685, 0.64, 0.30, 0.25], SEC[0], lambda S, Z: (S > 20.9) & (S < 21.7) & (Z > 5.9),
             '① 단면: hip 축에서 리드선 홈 방향(235°)으로 자른 면', (14, 44), (-0.5, 12.5))
dim(a, 21.006, 21.515, 10.0, '0.51 mm', 24.0, 11.0)
a.text(18.0, 7.5, 'Ø42 포켓\n(빈 공간)', fontsize=9.5, ha='center')
a.text(31.5, 7.3, '리드선 홈 P1 (바닥 Z=6)', fontsize=9.5, ha='center')
a.text(20.5, 2.0, '판 몸체', fontsize=9.5, ha='center')

# row 2: counterbore / Ø30 bore
row(0.64, 1, '②  M2.5 카운터보어 ↔ 가운데 Ø30 구멍 사이 살: 0.86 mm (A1~A6 6곳)',
    'AKE 고정링 볼트 머리 자리(Ø4.8 × 깊이 2.7, 바깥 면)가 PCD 36.5에 있어서 Ø30 구멍 벽과 0.86 mm만 남음.\n'
    '볼트 머리 아래(Z 2.7~5.5)는 1.86 mm로 괜찮음. 얇은 건 머리 자리 높이 2.7 mm 구간뿐.')
a = sec_plot([0.685, 0.325, 0.30, 0.25], SEC[1], lambda S, Z: (S > 14.95) & (S < 15.9) & (Z < 2.75),
             '② 단면: hip 축에서 A1 구멍 쪽으로 자른 면', (12, 24), (-2.6, 10.2))
dim(a, 15.0, 15.86, -0.4, '0.86 mm', 12.3, -2.2)
a.text(13.5, 5.0, 'Ø30\n관통', fontsize=9.5, ha='center')
a.text(18.25, 1.0, 'Ø4.8\n카운터보어', fontsize=9, ha='center')
a.text(18.25, 4.0, 'Ø2.8', fontsize=9, ha='center')
a.text(18.0, 7.8, 'Ø42 포켓', fontsize=9.5, ha='center')

# row 3: countersink / outer rim
row(0.325, 2, '③  M5 접시 자리 ↔ 판 바깥 테두리(Ø136) 사이 살: 최소 0.74 mm (E1·E2 2곳)',
    '척추 관 마개 M5 접시머리 자리(Ø11.2, 바깥 쪽 웹 면 Z=3)의 가장자리가 판 테두리에 바짝 붙음.\n'
    '면에서 0.74 mm, 깊어질수록 두꺼워져 Z≥5.9에서 3.6 mm. 칼날처럼 얇은 가장자리가 생김.')
a = sec_plot([0.685, 0.01, 0.30, 0.25], SEC[2], lambda S, Z: (S > 66.0) & (Z < 4.5),
             '③ 단면: hip 축에서 E1 구멍 쪽으로 자른 면', (52, 70), (-0.5, 10.0))
dim(a, 67.27, 68.01, 2.6, '0.74 mm', 63.0, 0.6)
a.text(61.7, 6.8, 'M5 볼트\n구멍', fontsize=9.5, ha='center')
a.text(55.5, 6.5, '웹 (Z 3~8)', fontsize=9.5, ha='center')

gmsh.finalize()
out = sys.argv[1] if len(sys.argv) > 1 else HERE / 'hip_plate_thin_walls.png'
fig.savefig(out, dpi=100, bbox_inches='tight')
print('wrote', out)
