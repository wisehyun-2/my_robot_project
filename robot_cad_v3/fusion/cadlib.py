"""Helper layer over the Fusion API used by the v2 / v3 build scripts.

Geometry is created with the TemporaryBRepManager (fast, exact booleans) in millimetres and handed to
Fusion in centimetres.  Designed parts are written in robot coordinates (x forward, y left, z up) and
converted to Fusion's Y-up world with ``rf`` (Fusion X = robot y, Y = robot z, Z = robot x).
``Builder(side=s)`` also mirrors the lateral coordinate (y -> s * y), so one function builds the left
(s = +1) and the right (s = -1) part.
"""
import math

import adsk.core
import adsk.fusion

CM = 0.1  # cm per mm
BIG = 2000.0


# ----------------------------------------------------------------------------- vectors
def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def mul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def norm(a):
    return math.sqrt(dot(a, a))


def unit(a):
    n = norm(a)
    return (a[0] / n, a[1] / n, a[2] / n)


def lerp(a, b, t):
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t)


def mirror(p, s):
    return (p[0], s * p[1], p[2])


def rf(p):
    """robot (x, y, z) -> Fusion axes (X, Y, Z) = (y, z, x)."""
    return (p[1], p[2], p[0])


# ----------------------------------------------------------------------------- 4x4 rigid frames (row major, mm)
def frame(origin, ex, ey, ez):
    return [[ex[0], ey[0], ez[0], origin[0]],
            [ex[1], ey[1], ez[1], origin[1]],
            [ex[2], ey[2], ez[2], origin[2]],
            [0.0, 0.0, 0.0, 1.0]]


def mmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def minv(m):
    r = [[m[j][i] for j in range(3)] for i in range(3)]
    t = [m[0][3], m[1][3], m[2][3]]
    ti = [-(r[i][0] * t[0] + r[i][1] * t[1] + r[i][2] * t[2]) for i in range(3)]
    return [r[0] + [ti[0]], r[1] + [ti[1]], r[2] + [ti[2]], [0.0, 0.0, 0.0, 1.0]]


def apply(m, p):
    return tuple(m[i][0] * p[0] + m[i][1] * p[1] + m[i][2] * p[2] + m[i][3] for i in range(3))


def applyv(m, v):
    return tuple(m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2] for i in range(3))


def col(m, j):
    return (m[0][j], m[1][j], m[2][j])


def origin(m):
    return (m[0][3], m[1][3], m[2][3])


def translate(t):
    return frame(t, (1, 0, 0), (0, 1, 0), (0, 0, 1))


def rot_about(axis_point, axis_dir, angle):
    """4x4 rotation by angle (rad) about a line."""
    k = unit(axis_dir)
    c, s = math.cos(angle), math.sin(angle)
    v = 1 - c
    r = [[c + k[0] * k[0] * v, k[0] * k[1] * v - k[2] * s, k[0] * k[2] * v + k[1] * s],
         [k[1] * k[0] * v + k[2] * s, c + k[1] * k[1] * v, k[1] * k[2] * v - k[0] * s],
         [k[2] * k[0] * v - k[1] * s, k[2] * k[1] * v + k[0] * s, c + k[2] * k[2] * v]]
    p = axis_point
    t = [p[i] - sum(r[i][j] * p[j] for j in range(3)) for i in range(3)]
    return [r[0] + [t[0]], r[1] + [t[1]], r[2] + [t[2]], [0.0, 0.0, 0.0, 1.0]]


PERM = [[0, 1, 0, 0], [0, 0, 1, 0], [1, 0, 0, 0], [0, 0, 0, 1]]   # robot -> Fusion axes


def robot_to_fusion(m):
    """A transform written in robot axes (both sides) -> the same transform in Fusion axes."""
    return mmul(mmul(PERM, m), minv(PERM))


def to_matrix3d(m):
    """4x4 (mm) -> Fusion Matrix3D (cm)."""
    mx = adsk.core.Matrix3D.create()
    mx.setWithArray([m[0][0], m[0][1], m[0][2], m[0][3] * CM,
                     m[1][0], m[1][1], m[1][2], m[1][3] * CM,
                     m[2][0], m[2][1], m[2][2], m[2][3] * CM,
                     0, 0, 0, 1])
    return mx


def part_matrix(m_robot):
    """Matrix3D placing a part-frame model (STEP) with a frame written in robot axes."""
    return to_matrix3d(mmul(PERM, m_robot))


def from_matrix3d(mx):
    a = mx.asArray()
    return [[a[0], a[1], a[2], a[3] / CM], [a[4], a[5], a[6], a[7] / CM], [a[8], a[9], a[10], a[11] / CM],
            [0.0, 0.0, 0.0, 1.0]]


# ----------------------------------------------------------------------------- temporary B-rep
TBM = adsk.fusion.TemporaryBRepManager.get()
UNION = adsk.fusion.BooleanTypes.UnionBooleanType
DIFF = adsk.fusion.BooleanTypes.DifferenceBooleanType
INTER = adsk.fusion.BooleanTypes.IntersectionBooleanType


def union(a, b):
    if b is None:
        return a
    if a is None:
        return b
    TBM.booleanOperation(a, b, UNION)
    return a


def cut(a, b):
    if b is not None:
        TBM.booleanOperation(a, b, DIFF)
    return a


def inter(a, b):
    TBM.booleanOperation(a, b, INTER)
    return a


def copy(b):
    return TBM.copy(b)


def transform(b, m_robot_mm):
    """Transform a temporary body by a robot-axes 4x4 (mm)."""
    TBM.transform(b, to_matrix3d(robot_to_fusion(m_robot_mm)))
    return b


class Builder:
    """Primitive factory in robot coordinates (mm in, cm out); side s mirrors y."""

    def __init__(self, side=1, robot=True):
        self.s = side
        self.robot = robot

    def P(self, p):
        q = (p[0], self.s * p[1], p[2]) if self.robot else p
        q = rf(q) if self.robot else q
        return adsk.core.Point3D.create(q[0] * CM, q[1] * CM, q[2] * CM)

    def V(self, v):
        q = (v[0], self.s * v[1], v[2]) if self.robot else v
        q = rf(q) if self.robot else q
        return adsk.core.Vector3D.create(q[0], q[1], q[2])

    def cyl(self, p0, p1, r, r1=None):
        return TBM.createCylinderOrCone(self.P(p0), r * CM, self.P(p1), (r if r1 is None else r1) * CM)

    def ycyl(self, x, z, y0, y1, r, r1=None):
        return self.cyl((x, y0, z), (x, y1, z), r, r1)

    def xcyl(self, y, z, x0, x1, r):
        return self.cyl((x0, y, z), (x1, y, z), r)

    def zcyl(self, x, y, z0, z1, r):
        return self.cyl((x, y, z0), (x, y, z1), r)

    def box(self, center, u, v, lu, lv, lw):
        """Box centred at center; edges lu along u, lv along v, lw along u x v."""
        obb = adsk.core.OrientedBoundingBox3D.create(self.P(center), self.V(unit(u)), self.V(unit(v)),
                                                     lu * CM, lv * CM, lw * CM)
        return TBM.createBox(obb)

    def abox(self, x0, x1, y0, y1, z0, z1):
        """Axis-aligned box between two corners."""
        c = ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)
        return self.box(c, (1, 0, 0), (0, 1, 0), abs(x1 - x0), abs(y1 - y0), abs(z1 - z0))

    def sphere(self, c, r):
        return TBM.createSphere(self.P(c), r * CM)

    def tube(self, p0, p1, ro, ri):
        b = self.cyl(p0, p1, ro)
        if ri > 0:
            ext = mul(unit(sub(p1, p0)), 1.0)
            cut(b, self.cyl(sub(p0, ext), add(p1, ext), ri))
        return b

    def rbox(self, center, u, v, lu, lv, lw, r):
        """Box with its four edges parallel to u x v rounded by radius r."""
        u, v = unit(u), unit(v)
        w = cross(u, v)
        r = min(r, lu / 2 - 1e-3, lv / 2 - 1e-3)
        b = self.box(center, u, v, lu - 2 * r, lv, lw)
        union(b, self.box(center, u, v, lu, lv - 2 * r, lw))
        for su in (-1, 1):
            for sv in (-1, 1):
                c = add(add(center, mul(u, su * (lu / 2 - r))), mul(v, sv * (lv / 2 - r)))
                union(b, self.cyl(sub(c, mul(w, lw / 2)), add(c, mul(w, lw / 2)), r))
        return b

    def slot(self, a, b, axis, r, h):
        """Stadium prism: centres a, b in a plane normal to axis, radius r, height h (centred)."""
        axis = unit(axis)
        d = sub(b, a)
        L = norm(d)
        s = self.cyl(sub(a, mul(axis, h / 2)), add(a, mul(axis, h / 2)), r)
        if L > 1e-6:
            union(s, self.cyl(sub(b, mul(axis, h / 2)), add(b, mul(axis, h / 2)), r))
            mid = mul(add(a, b), 0.5)
            union(s, self.box(mid, d, cross(axis, d), L, 2 * r, h))
        return s

    def halfspace(self, p, n):
        """Big box filling {q : (q - p) . n <= 0}."""
        n = unit(n)
        a = (1, 0, 0) if abs(n[0]) < 0.9 else (0, 1, 0)
        u = unit(cross(n, a))
        c = sub(p, mul(n, BIG / 2))
        return self.box(c, u, cross(n, u), BIG, BIG, BIG)

    def prism(self, poly, v0, v1, plane='xz'):
        """Convex polygon prism.  poly: CCW list of 2-D points in the plane; plane 'xz' extrudes along y
        (v0..v1), 'xy' along z, 'yz' along x."""
        def P3(a, b, v):
            if plane == 'xz':
                return (a, v, b)
            if plane == 'xy':
                return (a, b, v)
            return (v, a, b)
        us = [p[0] for p in poly]
        ws = [p[1] for p in poly]
        lo, hi = min(v0, v1), max(v0, v1)
        cc = P3((min(us) + max(us)) / 2, (min(ws) + max(ws)) / 2, (lo + hi) / 2)
        L = max(max(us) - min(us), max(ws) - min(ws)) + 10
        # slab spanning lo..hi, then cut down by one half-space per polygon edge
        if plane == 'xz':
            TBM_body = self.box(cc, (1, 0, 0), (0, 0, 1), L, L, hi - lo)   # third axis = x cross z = -y
        elif plane == 'xy':
            TBM_body = self.box(cc, (1, 0, 0), (0, 1, 0), L, L, hi - lo)
        else:
            TBM_body = self.box(cc, (0, 1, 0), (0, 0, 1), L, L, hi - lo)
        n = len(poly)
        area2 = sum(poly[i][0] * poly[(i + 1) % n][1] - poly[(i + 1) % n][0] * poly[i][1] for i in range(n))
        sgn = 1.0 if area2 > 0 else -1.0
        for i in range(n):
            a, b = poly[i], poly[(i + 1) % n]
            d = (b[0] - a[0], b[1] - a[1])
            out2 = (d[1] * sgn, -d[0] * sgn)               # outward normal of a CCW polygon edge
            p3 = P3(a[0], a[1], 0.0)
            n3 = P3(out2[0], out2[1], 0.0)
            inter(TBM_body, self.halfspace(p3, n3))
        return TBM_body

    def hull2(self, c1, r1, c2, r2, v0, v1, plane='xz'):
        """Convex hull of two circles (2-D centres in the plane) extruded v0..v1."""
        def P3(p, v):
            return (p[0], v, p[1]) if plane == 'xz' else (p[0], p[1], v)
        b = self.cyl(P3(c1, v0), P3(c1, v1), r1)
        union(b, self.cyl(P3(c2, v0), P3(c2, v1), r2))
        dx, dz = c2[0] - c1[0], c2[1] - c1[1]
        L = math.hypot(dx, dz)
        if L <= abs(r1 - r2):
            return b
        ux, uz = dx / L, dz / L
        a = math.acos((r1 - r2) / L)
        pts = []
        for sg in (1, -1):
            nx = ux * math.cos(a) - sg * uz * math.sin(a)
            nz = sg * ux * math.sin(a) + uz * math.cos(a)
            pts.append(((c1[0] + r1 * nx, c1[1] + r1 * nz), (c2[0] + r2 * nx, c2[1] + r2 * nz)))
        poly = [pts[0][0], pts[0][1], pts[1][1], pts[1][0]]
        union(b, self.prism(poly, v0, v1, plane))
        return b

    def chain(self, pts, r):
        """Cable: cylinders between the nodes, spheres at the inner nodes, flat ends."""
        q = [pts[0]]
        for p in pts[1:-1]:
            if norm(sub(p, q[-1])) > max(0.8, r):
                q.append(p)
        if norm(sub(pts[-1], q[-1])) < max(0.8, r) and len(q) > 1:
            q.pop()
        q.append(pts[-1])
        body = None
        for i in range(1, len(q)):
            seg = self.cyl(q[i - 1], q[i], r)
            if i > 1:
                try:
                    union(seg, self.sphere(q[i - 1], r * 0.999))
                except RuntimeError:
                    pass
            try:
                body = union(body, seg)
            except RuntimeError:
                body = union(body, self.cyl(q[i - 1], q[i], r * 0.99))
        return body


def torus(center, axis, R, r):
    """Torus about a robot-axes axis (TBM ignores the centre, so build at the origin and move it)."""
    t = TBM.createTorus(adsk.core.Point3D.create(0, 0, 0), adsk.core.Vector3D.create(*rf(axis)), R * CM, r * CM)
    mv = adsk.core.Matrix3D.create()
    c = rf(center)
    mv.translation = adsk.core.Vector3D.create(c[0] * CM, c[1] * CM, c[2] * CM)
    TBM.transform(t, mv)
    return t


# ----------------------------------------------------------------------------- design helpers
def app():
    return adsk.core.Application.get()


def design():
    return adsk.fusion.Design.cast(app().activeProduct)


def find_comp(name):
    for c in design().allComponents:
        if c.name == name:
            return c
    return None


class IdentityParent:
    """Temporarily put a root-level occurrence at the identity transform (see library())."""

    def __init__(self, occ):
        self.occ = occ

    def __enter__(self):
        self.saved = self.occ.transform2.copy()
        self.occ.transform2 = adsk.core.Matrix3D.create()
        return self.occ

    def __exit__(self, *exc):
        self.occ.transform2 = self.saved
        return False


def root_occ(name):
    for o in design().rootComponent.occurrences:
        if o.name == name or o.component.name == name:
            return o
    raise KeyError(name)


def library():
    """Hidden root-level component holding the master occurrence (identity) of every reused part.

    Occurrences.addExistingComponent(comp, m) places the new occurrence at m * (world transform of
    comp's first occurrence) and composes with a moved parent; keeping the first occurrence at the
    identity in an identity parent and adding instances while their parent sits at the identity gives
    'local = m'.  _library owns those components: never delete it."""
    root = design().rootComponent
    for o in root.occurrences:
        if o.component.name == '_library':
            return o
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = '_library'
    occ.isLightBulbOn = False
    return occ


def master(name):
    """New (or emptied) master component in _library."""
    c = find_comp(name)
    if c is not None:
        clear_bodies(c)
        return c
    lib = library().component
    occ = lib.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ.component


def instance(parent_occ, comp, m, name=None):
    """Place comp in the root-level occurrence parent_occ with local Matrix3D m."""
    with IdentityParent(parent_occ):
        o = parent_occ.component.occurrences.addExistingComponent(comp, m)
    return o


def new_part(parent_occ, name):
    """New empty component directly inside a root-level occurrence (identity transform)."""
    with IdentityParent(parent_occ):
        occ = parent_occ.component.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ.component


def clear_bodies(comp):
    while comp.bRepBodies.count:
        comp.bRepBodies.item(0).deleteMe()


MAT_LIB = 'Fusion 재료 라이브러리'
MATERIALS = {
    # name: (library id, density kg/m3)
    'Al6061': ('PrismMaterial-231', 2700.0),
    'PETG print': ('PrismMaterial-022', 1270.0),          # solid; printed parts get an effective density below
    'PETG 60%': ('PrismMaterial-022', 1270.0 * 0.80),     # 4 perimeters + 40 % gyroid ≈ 80 % of solid in these sections
    'PETG shell': ('PrismMaterial-022', 1270.0),          # thin walls print solid
    'Rubber': ('PrismMaterial-053', 1150.0),
    'Nylon': ('PrismMaterial-023', 1140.0),
    'Steel': ('PrismMaterial-231', 7850.0),               # Al asset with steel density (the steel asset copy fails)
    'Brass': ('PrismMaterial-231', 8500.0),
    'FR4 PCB': ('PrismMaterial-401', 1850.0),
    'Copper wire': ('PrismMaterial-231', 3000.0),          # copper + silicone insulation, solid tube
    'Connector': ('PrismMaterial-231', 1600.0),
    'Foam': ('PrismMaterial-022', 60.0),
}

APPEARANCES = {
    # name: (library id, rgb or None)
    'white': ('Prism-127', (238, 238, 234)),
    'black': ('Prism-127', (28, 28, 30)),
    'black satin': ('Prism-127', (45, 46, 50)),
    'dark grey': ('Prism-127', (72, 75, 80)),
    'alu': ('Prism-028', None),
    'alu black': ('Prism-127', (38, 38, 42)),
    'rubber': ('Prism-131', None),
    'pcb green': ('Prism-127', (20, 110, 50)),
    'pcb blue': ('Prism-127', (25, 60, 150)),
    'pcb black': ('Prism-127', (25, 25, 28)),
    'pcb red': ('Prism-127', (160, 30, 30)),
    'battery': ('Prism-127', (35, 90, 160)),
    'connector yellow': ('Prism-127', (235, 190, 30)),
    'connector white': ('Prism-127', (240, 240, 235)),
    'wire red': ('Prism-127', (200, 30, 30)),
    'wire black': ('Prism-127', (20, 20, 20)),
    'wire yellow': ('Prism-127', (230, 200, 40)),
    'wire green': ('Prism-127', (40, 150, 60)),
    'wire white': ('Prism-127', (230, 230, 230)),
    'steel': ('Prism-256', None),
    'brass': ('Prism-127', (190, 150, 60)),
    'magnet': ('Prism-127', (120, 120, 130)),
    'heat shrink': ('Prism-127', (40, 40, 40)),
    'led': ('Prism-127', (120, 220, 255)),
    'red': ('Prism-127', (200, 30, 30)),
    'blue': ('Prism-127', (20, 105, 225)),            # replaces TITA's yellow accents
    'shell grey': ('Prism-127', (52, 54, 58)),        # TITA's dark textured body
    'lens': ('Prism-127', (10, 12, 20)),
}


def material(name):
    d = design()
    m = d.materials.itemByName(name)
    if m:
        return m
    lib_id, rho = MATERIALS[name]
    base = app().materialLibraries.itemByName(MAT_LIB).materials.itemById(lib_id)
    m = d.materials.addByCopy(base, name)
    try:
        m.materialProperties.itemById('structural_Density').value = rho
    except RuntimeError:
        pass
    return m


def custom_density_material(name, rho, base='PETG print'):
    d = design()
    m = d.materials.itemByName(name)
    if m is None:
        lib_id = MATERIALS[base][0]
        m = d.materials.addByCopy(app().materialLibraries.itemByName(MAT_LIB).materials.itemById(lib_id), name)
    m.materialProperties.itemById('structural_Density').value = rho
    return m


def appearance(name):
    d = design()
    a = d.appearances.itemByName(name)
    if a:
        return a
    lib_id, rgb = APPEARANCES[name]
    lib = app().materialLibraries.item(0)
    base = lib.appearances.itemById(lib_id)
    a = d.appearances.addByCopy(base, name)
    if rgb:
        p = a.appearanceProperties.itemById('opaque_albedo')
        if p is None:
            p = a.appearanceProperties.itemById('surface_albedo')
        if p is not None:
            p.value = adsk.core.Color.create(rgb[0], rgb[1], rgb[2], 255)
    return a


def add_body(comp, temp, name, mat=None, look=None):
    b = comp.bRepBodies.add(temp)
    b.name = name
    if mat:
        b.material = material(mat) if isinstance(mat, str) else mat
    if look:
        b.appearance = appearance(look)
    return b


def fillet_edges(comp, edges, r_mm):
    if not edges:
        return None
    oc = adsk.core.ObjectCollection.create()
    for e in edges:
        oc.add(e)
    fi = comp.features.filletFeatures.createInput()
    fi.addConstantRadiusEdgeSet(oc, adsk.core.ValueInput.createByReal(r_mm * CM), True)
    return comp.features.filletFeatures.add(fi)


def polar(n, r, a0_deg=0.0):
    return [(r * math.cos(math.radians(a0_deg + 360.0 * k / n)), r * math.sin(math.radians(a0_deg + 360.0 * k / n)))
            for k in range(n)]
