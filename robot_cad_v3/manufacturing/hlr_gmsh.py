"""gmsh (OpenCascade kernel) helpers for the v3 link drawings: STEP in / out in a part frame, hidden-line projection,
section faces and hole recognition.  (The v2 quote drawings used OCP's HLR; OCP does not run on this PC, gmsh does.)

Hidden lines: every B-rep edge and every mesh silhouette of a curved face is cut into short pieces, and a piece is
hidden when a triangle of the part's surface mesh covers its projection strictly inside and nearer to the viewer.
Edges are sampled on the mesh nodes of the curves, so they are the same chords the triangles end on (no false
occlusion at tessellated circle boundaries).
"""
import math

import gmsh
import numpy as np

PIECE = 0.35            # mm, hidden / visible resolution along an edge
SMOOTH_DEG = 2.0        # edges between tangent faces (cylinder halves, seams) are not drawn


def init():
    if not gmsh.isInitialized():
        gmsh.initialize()
    gmsh.option.setNumber('General.Terminal', 0)


def _quat(R):
    """Unit quaternion (w, x, y, z) of a proper rotation matrix (Shepperd)."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = 2.0 * math.sqrt(t + 1.0)
        return (0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s)
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
    q = [0.0, 0.0, 0.0, 0.0]
    q[0] = (R[k, j] - R[j, k]) / s
    q[1 + i] = 0.25 * s
    q[1 + j] = (R[j, i] + R[i, j]) / s
    q[1 + k] = (R[k, i] + R[i, k]) / s
    return tuple(q)


def load(step_path, rows=None):
    """Fresh model with the STEP solid; rows = 3 x 4 [R | t] applied to it (proper rotation; done as one rotation about
    the origin + a translation so the faces stay analytic - a general affine transform turns them into B-splines).
    Returns the volume tags."""
    init()
    gmsh.clear()
    gmsh.model.add('part')
    ents = gmsh.model.occ.importShapes(str(step_path))
    if rows is not None:
        R = np.array([r[:3] for r in rows], float)
        t = [r[3] for r in rows]
        assert abs(np.linalg.det(R) - 1.0) < 1e-6 and np.allclose(R @ R.T, np.eye(3), atol=1e-6)
        w, x, y, z = _quat(R)
        ang = 2.0 * math.atan2(math.sqrt(x * x + y * y + z * z), w)
        if ang > 1e-12:
            n = math.sqrt(x * x + y * y + z * z)
            gmsh.model.occ.rotate(ents, 0, 0, 0, x / n, y / n, z / n, ang)
        gmsh.model.occ.translate(ents, *t)
    gmsh.model.occ.synchronize()
    return [t for _d, t in gmsh.model.getEntities(3)]


def keep_box(x0, x1, y0, y1, z0, z1):
    """Intersect the model's solids with a box (section halves, detail crops)."""
    vols = gmsh.model.getEntities(3)
    box = gmsh.model.occ.addBox(x0, y0, z0, x1 - x0, y1 - y0, z1 - z0)
    gmsh.model.occ.intersect(vols, [(3, box)], removeObject=True, removeTool=True)
    gmsh.model.occ.synchronize()
    return [t for _d, t in gmsh.model.getEntities(3)]


def write_step(path):
    gmsh.write(str(path))


def bbox():
    return gmsh.model.getBoundingBox(-1, -1)


def volume():
    return sum(gmsh.model.occ.getMass(3, t) for _d, t in gmsh.model.getEntities(3))


# ----------------------------------------------------------------------------- mesh
def mesh(curv=72, hmax=2.5):
    gmsh.model.mesh.clear()
    gmsh.option.setNumber('Mesh.MeshSizeFromCurvature', curv)
    gmsh.option.setNumber('Mesh.MeshSizeMax', hmax)
    gmsh.option.setNumber('Mesh.MeshSizeMin', 0.02)
    gmsh.option.setNumber('Mesh.Algorithm', 6)
    gmsh.model.mesh.generate(2)
    tags, xyz, _ = gmsh.model.mesh.getNodes()
    pos = np.zeros((int(tags.max()) + 1, 3))
    pos[tags.astype(int)] = xyz.reshape(-1, 3)
    tris, owner = [], []
    for _d, s in gmsh.model.getEntities(2):
        types, _et, nt = gmsh.model.mesh.getElements(2, s)
        for ty, n in zip(types, nt):
            if ty == 2:
                t = n.astype(int).reshape(-1, 3)
                tris.append(t)
                owner.append(np.full(len(t), s))
    return pos, np.vstack(tris), np.concatenate(owner)


def curve_polyline(pos, c):
    """Ordered mesh nodes of curve c (closed curves repeat the first point)."""
    nt, _xyz, par = gmsh.model.mesh.getNodes(1, c, includeBoundary=True, returnParametricCoord=True)
    if len(nt) == 0:
        return None
    order = np.argsort(par)
    pts = pos[nt.astype(int)[order]]
    b = gmsh.model.getBoundary([(1, c)], combined=False, oriented=False)
    if len(b) < 2 or b[0][1] == b[1][1]:            # closed curve
        pts = np.vstack([pts, pts[:1]])
    # drop duplicate consecutive nodes
    keep = np.ones(len(pts), bool)
    keep[1:] = np.linalg.norm(np.diff(pts, axis=0), axis=1) > 1e-9
    return pts[keep]


def _face_normal(f, p):
    xyz, uv = gmsh.model.getClosestPoint(2, f, list(p))
    n = np.array(gmsh.model.getNormal(f, list(uv)))
    return n / (np.linalg.norm(n) or 1.0)


def sharp_curves(pos):
    """{curve: polyline} for the edges to draw: two different adjacent faces meeting at an angle."""
    out = {}
    for _d, c in gmsh.model.getEntities(1):
        up, _down = gmsh.model.getAdjacencies(1, c)
        faces = sorted(set(int(f) for f in up))
        if len(faces) < 2:
            continue                                     # seam of a periodic face
        pl = curve_polyline(pos, c)
        if pl is None or len(pl) < 2:
            continue
        mid = pl[len(pl) // 2] if len(pl) > 2 else (pl[0] + pl[1]) / 2
        n1, n2 = _face_normal(faces[0], mid), _face_normal(faces[1], mid)
        if abs(float(n1 @ n2)) > math.cos(math.radians(SMOOTH_DEG)):
            continue                                     # tangent faces (split cylinder, blend)
        out[c] = pl
    return out


# ----------------------------------------------------------------------------- projection
class Projector:
    """View along -toward (toward = unit vector from the part to the viewer); screen x = xdir, y = toward x xdir."""

    def __init__(self, pos, tris, owner, toward, xdir, cell=2.0):
        self.v = np.array(toward, float)
        self.xd = np.array(xdir, float)
        self.up = np.cross(self.v, self.xd)
        self.pos, self.tris, self.owner = pos, tris, owner
        P = pos[tris]                                    # (M, 3, 3)
        self.t2 = np.stack([P @ self.xd, P @ self.up], axis=-1)    # (M, 3, 2)
        self.td = P @ self.v                             # (M, 3)
        a = self.t2
        area = (a[:, 1, 0] - a[:, 0, 0]) * (a[:, 2, 1] - a[:, 0, 1]) - (a[:, 2, 0] - a[:, 0, 0]) * (a[:, 1, 1] - a[:, 0, 1])
        ok = np.abs(area) > 1e-7                         # edge-on triangles cannot hide anything
        self.t2, self.td, self.area = self.t2[ok], self.td[ok], area[ok]
        self.cell = cell
        lo = self.t2.min(axis=1)
        hi = self.t2.max(axis=1)
        self.o = lo.min(axis=0) - 1.0
        i0 = np.floor((lo - self.o) / cell).astype(int)
        i1 = np.floor((hi - self.o) / cell).astype(int)
        self.grid = {}
        for k in range(len(self.t2)):
            for i in range(i0[k, 0], i1[k, 0] + 1):
                for j in range(i0[k, 1], i1[k, 1] + 1):
                    self.grid.setdefault((i, j), []).append(k)
        self.grid = {key: np.array(v) for key, v in self.grid.items()}

    def p2(self, P):
        P = np.atleast_2d(P)
        return np.stack([P @ self.xd, P @ self.up], axis=-1)

    def hidden(self, P):
        """Boolean per 3D point: covered by a nearer triangle."""
        q = self.p2(P)
        d = P @ self.v
        out = np.zeros(len(P), bool)
        ij = np.floor((q - self.o) / self.cell).astype(int)
        keys = {}
        for n, (i, j) in enumerate(map(tuple, ij)):
            keys.setdefault((i, j), []).append(n)
        for key, idx in keys.items():
            cand = self.grid.get(key)
            if cand is None:
                continue
            idx = np.array(idx)
            a = self.t2[cand]                            # (C, 3, 2)
            x, y = q[idx, 0][:, None], q[idx, 1][:, None]
            x0, y0 = a[None, :, 0, 0], a[None, :, 0, 1]
            x1, y1 = a[None, :, 1, 0], a[None, :, 1, 1]
            x2, y2 = a[None, :, 2, 0], a[None, :, 2, 1]
            ar = self.area[cand][None, :]
            l1 = ((x - x0) * (y2 - y0) - (x2 - x0) * (y - y0)) / ar
            l2 = ((x1 - x0) * (y - y0) - (x - x0) * (y1 - y0)) / ar
            l0 = 1.0 - l1 - l2
            eps = 1e-6
            inside = (l0 > eps) & (l1 > eps) & (l2 > eps)
            dz = self.td[cand]
            depth = l0 * dz[None, :, 0] + l1 * dz[None, :, 1] + l2 * dz[None, :, 2]
            cover = inside & (depth > d[idx][:, None] + 2e-3)
            out[idx] = cover.any(axis=1)
        return out

    def split(self, pl):
        """Polyline (3D) -> (visible, hidden) lists of 2D polylines."""
        pieces, mids = [], []
        for a, b in zip(pl[:-1], pl[1:]):
            n = max(1, int(math.ceil(np.linalg.norm(b - a) / PIECE)))
            for k in range(n):
                p0, p1 = a + (b - a) * k / n, a + (b - a) * (k + 1) / n
                pieces.append((p0, p1))
                mids.append((p0 + p1) / 2)
        if not pieces:
            return [], []
        hid = self.hidden(np.array(mids))
        vis_out, hid_out = [], []
        cur, cur_h = None, None
        for (p0, p1), h in zip(pieces, hid):
            if cur is not None and h == cur_h:
                cur.append(p1)
            else:
                if cur is not None:
                    (hid_out if cur_h else vis_out).append(cur)
                cur, cur_h = [p0, p1], h
        (hid_out if cur_h else vis_out).append(cur)
        f = lambda L: [self.p2(np.array(s)).tolist() for s in L]   # noqa: E731
        return f(vis_out), f(hid_out)

    def silhouettes(self, planar):
        """Silhouette curves of the curved faces: zero crossings of (exact surface normal . view) interpolated along
        the triangle edges (marching triangles).  Faces seen edge-on everywhere (cylinders along the view) have none."""
        segs = []
        for _d, s in gmsh.model.getEntities(2):
            if s in planar:
                continue
            nt, _xyz, uv = gmsh.model.mesh.getNodes(2, s, includeBoundary=True, returnParametricCoord=True)
            if len(nt) == 0:
                continue
            nrm = np.array(gmsh.model.getNormal(s, list(uv))).reshape(-1, 3)
            dot = dict(zip(nt.astype(int), nrm @ self.v))
            vals = np.fromiter(dot.values(), float)
            if np.abs(vals).max() < 1e-3 or (vals.min() > 0) or (vals.max() < 0):
                continue                                 # edge-on everywhere, or one-sided: no silhouette
            for t in self.tris[self.owner == s]:
                d = [dot.get(int(k), 0.0) for k in t]
                cr = []
                for i, j in ((0, 1), (1, 2), (2, 0)):
                    if (d[i] > 0) != (d[j] > 0) and abs(d[i] - d[j]) > 1e-12:
                        f = d[i] / (d[i] - d[j])
                        cr.append(self.pos[t[i]] + (self.pos[t[j]] - self.pos[t[i]]) * f)
                if len(cr) == 2:
                    segs.append(np.array(cr))
        return chain(segs)


def chain(segs, tol=1e-5):
    """Join 2-point segments sharing end points into polylines."""
    key = lambda p: tuple(np.round(p / tol).astype(np.int64))   # noqa: E731
    ends = {}
    for i, s in enumerate(segs):
        for e in (0, 1):
            ends.setdefault(key(s[e]), []).append((i, e))
    used = np.zeros(len(segs), bool)
    out = []
    for i in range(len(segs)):
        if used[i]:
            continue
        used[i] = True
        line = [segs[i][0], segs[i][1]]
        for direction in (1, 0):                    # grow at the end, then at the start
            while True:
                tip = line[-1] if direction else line[0]
                nxt = None
                for j, e in ends.get(key(tip), []):
                    if not used[j]:
                        nxt = (j, e)
                        break
                if nxt is None:
                    break
                j, e = nxt
                used[j] = True
                other = segs[j][1 - e]
                if direction:
                    line.append(other)
                else:
                    line.insert(0, other)
        out.append(np.array(line))
    return out


def project(toward, xdir, silhouettes=True):
    """Hidden-line projection of the current model.  Returns (visible, hidden) 2D polylines and the projector."""
    pos, tris, owner = mesh()
    pr = Projector(pos, tris, owner, toward, xdir)
    vis, hid = [], []
    for _c, pl in sharp_curves(pos).items():
        v, h = pr.split(pl)
        vis += v
        hid += h
    if silhouettes:
        planar = [s for _d, s in gmsh.model.getEntities(2) if gmsh.model.getType(2, s) == 'Plane']
        for seg in pr.silhouettes(planar):
            v, h = pr.split(seg)
            vis += v
            hid += h
    return vis, hid, pr


def section_loops(pr, axis, value, tol=1e-4):
    """Planar faces of the current model lying on axis = value -> list of faces, each a list of 2D loops."""
    i = 'xyz'.index(axis)
    pos = pr.pos
    faces = []
    for _d, s in gmsh.model.getEntities(2):
        if gmsh.model.getType(2, s) != 'Plane':
            continue
        b = gmsh.model.getBoundingBox(2, s)
        if abs(b[i] - value) > tol or abs(b[i + 3] - value) > tol:
            continue
        _lt, curves_per_loop = gmsh.model.occ.getCurveLoops(s)
        loops = []
        for cl in curves_per_loop:
            chain = None
            pls = [curve_polyline(pos, int(abs(c))) for c in cl]
            pls = [p for p in pls if p is not None]
            for pl in pls:
                if chain is None:
                    chain = list(pl)
                    continue
                e = chain[-1]
                if np.linalg.norm(pl[0] - e) <= np.linalg.norm(pl[-1] - e):
                    chain += list(pl[1:])
                else:
                    chain += list(pl[::-1][1:])
            if chain is not None:
                if np.linalg.norm(chain[0] - chain[-1]) > 0.05 and len(pls) > 1:
                    # first curve may have been reversed: retry starting reversed
                    alt = list(pls[0][::-1])
                    for pl in pls[1:]:
                        e = alt[-1]
                        alt += list(pl[1:]) if np.linalg.norm(pl[0] - e) <= np.linalg.norm(pl[-1] - e) else list(pl[::-1][1:])
                    if np.linalg.norm(alt[0] - alt[-1]) < np.linalg.norm(chain[0] - chain[-1]):
                        chain = alt
                loops.append(pr.p2(np.array(chain)))
        faces.append(loops)
    return faces


# ----------------------------------------------------------------------------- hole / boss recognition (Z-axis cylinders)
def z_cylinders(pos=None):
    """Cylindrical (and conical) faces with their axis along Z: list of dict(face, kind, cx, cy, d (or d0/d1 for
    cones), z0, z1, convex)."""
    vols = [t for _d, t in gmsh.model.getEntities(3)]
    out = []
    for _d, s in gmsh.model.getEntities(2):
        ty = gmsh.model.getType(2, s)
        if ty not in ('Cylinder', 'Cone'):
            continue
        _nt, xyz, _p = gmsh.model.mesh.getNodes(2, s, includeBoundary=True)
        P = xyz.reshape(-1, 3)
        if len(P) < 5:
            continue
        b = gmsh.model.getBoundingBox(2, s)
        if ty == 'Cylinder':
            # algebraic circle fit in XY
            A = np.column_stack([P[:, 0], P[:, 1], np.ones(len(P))])
            rhs = -(P[:, 0] ** 2 + P[:, 1] ** 2)
            sol, *_ = np.linalg.lstsq(A, rhs, rcond=None)
            cx, cy = -sol[0] / 2, -sol[1] / 2
            r = math.sqrt(max(cx * cx + cy * cy - sol[2], 0.0))
            res = np.abs(np.hypot(P[:, 0] - cx, P[:, 1] - cy) - r).max()
            if res > 1e-3 or b[5] - b[2] < 1e-4:
                continue
            q = P[0]
            u = np.array([cx - q[0], cy - q[1], 0.0])
            u /= np.linalg.norm(u) or 1.0
            probe = q + u * 0.05
            probe[2] = (b[2] + b[5]) / 2
            inside = any(gmsh.model.isInside(3, v, list(probe)) for v in vols)
            out.append(dict(face=s, kind='cyl', cx=cx, cy=cy, d=2 * r, z0=b[2], z1=b[5], convex=bool(inside)))
        else:
            zs = np.unique(np.round(P[:, 2], 4))
            if len(zs) < 2:
                continue
            rings = []
            for z in (zs.min(), zs.max()):
                Q = P[np.abs(P[:, 2] - z) < 1e-3]
                if len(Q) < 3:
                    break
                A = np.column_stack([Q[:, 0], Q[:, 1], np.ones(len(Q))])
                sol, *_ = np.linalg.lstsq(A, -(Q[:, 0] ** 2 + Q[:, 1] ** 2), rcond=None)
                cx, cy = -sol[0] / 2, -sol[1] / 2
                rings.append((cx, cy, 2 * math.sqrt(max(cx * cx + cy * cy - sol[2], 0.0)), z))
            if len(rings) == 2:
                out.append(dict(face=s, kind='cone', cx=rings[0][0], cy=rings[0][1], d0=rings[0][2], d1=rings[1][2],
                                z0=rings[0][3], z1=rings[1][3], convex=False))
    return out


def group_holes(cyls, digits=2):
    """Merge faces of the same (centre, diameter, convex) and return sorted dicts with the merged z range."""
    g = {}
    for c in cyls:
        if c['kind'] != 'cyl':
            continue
        key = (round(c['cx'], digits), round(c['cy'], digits), round(c['d'], 3), c['convex'])
        z0, z1 = g.get(key, (1e9, -1e9))
        g[key] = (min(z0, c['z0']), max(z1, c['z1']))
    return [dict(cx=k[0], cy=k[1], d=k[2], convex=k[3], z0=v[0], z1=v[1]) for k, v in sorted(g.items())]
