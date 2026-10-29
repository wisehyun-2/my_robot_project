"""Wall thickness check of a machined part (STEP): from every surface triangle a ray goes into the material along
the inward normal; the distance to the next surface is the local wall.  Spots under the limit are grouped and
listed.  Runs on gmsh (OCC kernel), so it works where OCP is blocked.

    python wall_check.py step/hip_plate_L.step [--min 1.5] [--size 0.5]

Convex edges never show (the ray leaves the edge region) and square-ish corners are filtered out (thickness());
slivers and wedges sharper than 60 deg do show.  Rays that graze a face at the edge of a hole can report a short
distance on one triangle only, so groups smaller than --min-area (default 0.1 mm²) are listed separately as
'specks'.
"""
import argparse
import math
import sys

import gmsh
import numpy as np
from scipy.spatial import cKDTree


def mesh(path, size):
    gmsh.initialize(); gmsh.option.setNumber('General.Terminal', 0)
    gmsh.model.occ.importShapes(str(path)); gmsh.model.occ.synchronize()
    vol = gmsh.model.getEntities(3)
    assert len(vol) == 1, 'one solid expected'
    for k, v in (('Mesh.MeshSizeMax', size), ('Mesh.MeshSizeMin', size / 20), ('Mesh.MeshSizeFromCurvature', 24)):
        gmsh.option.setNumber(k, v)
    gmsh.model.mesh.generate(2)
    tags, xyz, _ = gmsh.model.mesh.getNodes()
    P = np.zeros((int(tags.max()) + 1, 3)); P[tags.astype(int)] = np.asarray(xyz).reshape(-1, 3)
    tris, face = [], []
    for (_, f) in gmsh.model.getEntities(2):
        _, nt = gmsh.model.mesh.getElementsByType(2, f)
        t = P[np.asarray(nt).astype(int).reshape(-1, 3)]
        if not len(t):
            continue
        # outward orientation of this face's triangles: probe just behind the largest one
        n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
        k = int(np.argmax(np.linalg.norm(n, axis=1)))
        c = t[k].mean(axis=0); nk = n[k] / np.linalg.norm(n[k])
        h = 0.2 * math.sqrt(np.linalg.norm(n[k]) / 2)
        if gmsh.model.isInside(*vol[0], list(c + h * nk)) > 0:
            t = t[:, ::-1]                                  # winding points inward: flip
        tris.append(t); face += [f] * len(t)
    gmsh.finalize()
    return np.concatenate(tris), np.array(face)


def thickness(T, tmax, facing=0.5):
    """Distance from each triangle centroid along the inward normal to the first other triangle (inf if > tmax).
    Hits on a face that is not turned back towards the start (outward normals within acos(facing) of
    anti-parallel, i.e. a wall or a wedge sharper than 60 deg) are ignored: that is the far face of a
    square-ish corner, not a wall."""
    C = T.mean(axis=1)
    N = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    A = 0.5 * np.linalg.norm(N, axis=1)
    N /= (2 * A)[:, None] + 1e-15
    reach = tmax + np.max(np.linalg.norm(T - C[:, None, :], axis=2))
    tree = cKDTree(C)
    out = np.full(len(T), np.inf)
    e1, e2 = T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]
    for i, cand in enumerate(tree.query_ball_point(C, reach)):
        cand = np.asarray(cand)
        cand = cand[cand != i]
        if not len(cand):
            continue
        o, d = C[i] - 1e-4 * N[i], -N[i]                    # Moller-Trumbore, ray o + t d
        p = np.cross(d, e2[cand])
        det = np.einsum('ij,ij->i', e1[cand], p)
        ok = np.abs(det) > 1e-12
        inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
        s = o - T[cand, 0]
        u = np.einsum('ij,ij->i', s, p) * inv
        q = np.cross(s, e1[cand])
        v = (q @ d) * inv
        t = np.einsum('ij,ij->i', e2[cand], q) * inv
        hit = ok & (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9) & (t > 1e-6) & (t <= tmax)
        if hit.any():
            j = np.flatnonzero(hit)[np.argmin(t[hit])]
            if N[cand[j]] @ d > facing:
                out[i] = t[j]
    return C, A, out


def groups(C, mask, link=0.8):
    """Connected groups of the flagged centroids (single linkage within `link` mm)."""
    idx = np.flatnonzero(mask)
    if not len(idx):
        return []
    tree = cKDTree(C[idx])
    parent = list(range(len(idx)))

    def root(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]; a = parent[a]
        return a
    for a, b in tree.query_pairs(link):
        ra, rb = root(a), root(b)
        if ra != rb:
            parent[ra] = rb
    g = {}
    for k in range(len(idx)):
        g.setdefault(root(k), []).append(idx[k])
    return [np.array(v) for v in g.values()]


def check(path, tmin=1.5, size=0.5, min_area=0.1, quiet=False):
    T, face = mesh(path, size)
    C, A, t = thickness(T, tmin)
    out = []
    for g in groups(C, t < tmin):
        k = g[np.argmin(t[g])]
        out.append(dict(t=float(t[k]), at=tuple(float(v) for v in C[k]), area=float(A[g].sum()), n=len(g),
                        faces=sorted(set(int(f) for f in face[g]))))
    out.sort(key=lambda r: r['t'])
    walls = [r for r in out if r['area'] >= min_area]
    specks = [r for r in out if r['area'] < min_area]
    if not quiet:
        print('%s: %d triangles, walls under %.2f mm: %d (+ %d specks < %.2f mm2)'
              % (path, len(T), tmin, len(walls), len(specks), min_area))
        for r in walls:
            x, y, z = r['at']
            print('  wall %.2f mm  at x %7.2f y %7.2f z %5.2f (r %6.2f, %6.1f deg)  area %.2f mm2'
                  % (r['t'], x, y, z, math.hypot(x, y), math.degrees(math.atan2(y, x)), r['area']))
        for r in specks:
            x, y, z = r['at']
            print('  speck %.2f mm at x %7.2f y %7.2f z %5.2f  area %.3f mm2' % (r['t'], x, y, z, r['area']))
    return walls, specks


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('step', nargs='+')
    ap.add_argument('--min', type=float, default=1.5)
    ap.add_argument('--size', type=float, default=0.5)
    ap.add_argument('--min-area', type=float, default=0.1)
    a = ap.parse_args()
    bad = 0
    for p in a.step:
        w, _ = check(p, a.min, a.size, a.min_area)
        bad += len(w)
    sys.exit(1 if bad else 0)
