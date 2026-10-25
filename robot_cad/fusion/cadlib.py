"""Small helper layer over the Fusion API used by the build scripts.

Geometry is created with the TemporaryBRepManager (fast, exact booleans) in
millimetres and handed to Fusion in centimetres. Two kinds of local frames exist:

* part frames (STEP files, designed parts): coordinates are used as they are;
* robot frames (torso component): x forward, y left, z up, converted to Fusion's
  Y-up world with ``rf`` (Fusion X = robot y, Y = robot z, Z = robot x).
"""
import math

import adsk.core
import adsk.fusion

CM = 0.1  # cm per mm


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


def unit(a):
    n = math.sqrt(dot(a, a))
    return (a[0] / n, a[1] / n, a[2] / n)


def rf(p):
    """robot (x, y, z) -> Fusion axes (X, Y, Z) = (y, z, x)."""
    return (p[1], p[2], p[0])


# ----------------------------------------------------------------------------- 4x4 rigid frames
def frame(origin, ex, ey, ez):
    """4x4 matrix (row major) mapping local coordinates to the parent frame."""
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


# ----------------------------------------------------------------------------- temporary B-rep
TBM = adsk.fusion.TemporaryBRepManager.get()
UNION = adsk.fusion.BooleanTypes.UnionBooleanType
DIFF = adsk.fusion.BooleanTypes.DifferenceBooleanType
INTER = adsk.fusion.BooleanTypes.IntersectionBooleanType


class Builder:
    """Primitive factory in a local frame given by a point/vector mapping (mm in, cm out)."""

    def __init__(self, robot=False):
        self.robot = robot

    def _p(self, p):
        q = rf(p) if self.robot else p
        return adsk.core.Point3D.create(q[0] * CM, q[1] * CM, q[2] * CM)

    def _v(self, v):
        q = rf(v) if self.robot else v
        return adsk.core.Vector3D.create(q[0], q[1], q[2])

    def cyl(self, p0, p1, r, r1=None):
        return TBM.createCylinderOrCone(self._p(p0), r * CM, self._p(p1), (r if r1 is None else r1) * CM)

    def box(self, center, u, v, lu, lv, lw):
        """Box centred at center; edges lu along u, lv along v, lw along u x v."""
        obb = adsk.core.OrientedBoundingBox3D.create(self._p(center), self._v(unit(u)), self._v(unit(v)),
                                                     lu * CM, lv * CM, lw * CM)
        return TBM.createBox(obb)

    def sphere(self, c, r):
        return TBM.createSphere(self._p(c), r * CM)

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
        L = math.sqrt(dot(d, d))
        s = self.cyl(sub(a, mul(axis, h / 2)), add(a, mul(axis, h / 2)), r)
        union(s, self.cyl(sub(b, mul(axis, h / 2)), add(b, mul(axis, h / 2)), r))
        if L > 1e-6:
            mid = mul(add(a, b), 0.5)
            union(s, self.box(mid, d, cross(axis, d), L, 2 * r, h))
        return s


def union(a, b):
    TBM.booleanOperation(a, b, UNION)
    return a


def cut(a, b):
    TBM.booleanOperation(a, b, DIFF)
    return a


def inter(a, b):
    TBM.booleanOperation(a, b, INTER)
    return a


def copy(b):
    return TBM.copy(b)


# ----------------------------------------------------------------------------- design helpers
def app():
    return adsk.core.Application.get()


def design():
    return adsk.fusion.Design.cast(app().activeProduct)


def find_comp(name):
    d = design()
    for c in d.allComponents:
        if c.name == name:
            return c
    return None


class IdentityParent:
    """Temporarily put a root-level occurrence at the identity transform.

    Occurrences added inside a component whose root occurrence is moved get odd
    local transforms through the API; adding them while the parent sits at the
    identity makes 'local = given matrix'."""

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
        if o.name == name:
            return o
    raise KeyError(name)


def library():
    """Root-level component holding the master occurrence (identity) of every reused part.

    Occurrences.addExistingComponent(comp, m) places the new occurrence at
    m * (world transform of comp's first occurrence) and also composes with a moved
    parent; keeping the first occurrence at the identity in an identity parent and
    adding instances while their parent sits at the identity gives 'local = m'."""
    root = design().rootComponent
    for o in root.occurrences:
        if o.component.name == '_library':
            return o
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = '_library'
    occ.isLightBulbOn = False
    return occ


def master(name):
    """New empty master component in _library."""
    lib = library().component
    occ = lib.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ.component


def instance(parent_occ, comp, m, label=None):
    """Place comp in the root-level occurrence parent_occ with local Matrix3D m."""
    with IdentityParent(parent_occ):
        o = parent_occ.component.occurrences.addExistingComponent(comp, m)
    return o


def new_comp(parent, name, m=None):
    """New component under parent (an occurrence transform m, Matrix3D or None)."""
    occ = parent.occurrences.addNewComponent(m or adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ


def clear_bodies(comp):
    while comp.bRepBodies.count:
        comp.bRepBodies.item(0).deleteMe()


MAT_LIB = 'Fusion 재료 라이브러리'
MATERIALS = {
    # name: (library id, density kg/m3)
    'Al6061': ('PrismMaterial-231', 2700.0),
    'PA-CF print': ('PrismMaterial-023', 1150.0),     # printed nylon-CF, solid
    'PETG print': ('PrismMaterial-022', 1270.0),
    'TPU 95A': ('PrismMaterial-053', 1210.0),
    'Steel': ('PrismMaterial-231', 7850.0),        # Al asset with steel density (the steel asset copy fails)
    'FR4 PCB': ('PrismMaterial-401', 1850.0),
    'Acrylic': ('PrismMaterial-022', 1190.0),
    'Connector': ('PrismMaterial-231', 1500.0),    # hollow metal connector shells, as solid blocks
}

APPEARANCES = {
    # name: (library id, rgb or None)
    'shell white': ('Prism-127', (236, 236, 232)),
    'shell grey': ('Prism-127', (70, 74, 80)),
    'accent orange': ('Prism-127', (240, 110, 20)),
    'print black': ('Prism-113', None),
    'print grey': ('Prism-116', None),
    'alu': ('Prism-028', None),
    'alu dark': ('Prism-032', None),
    'rubber': ('Prism-131', None),
    'pcb green': ('Prism-127', (20, 110, 50)),
    'pcb blue': ('Prism-127', (25, 60, 150)),
    'pcb black': ('Prism-127', (25, 25, 28)),
    'battery': ('Prism-127', (35, 90, 160)),
    'connector yellow': ('Prism-127', (235, 190, 30)),
    'connector white': ('Prism-127', (240, 240, 235)),
    'visor': ('Prism-165', None),
    'led': ('Prism-127', (120, 220, 255)),
    'red': ('Prism-127', (200, 30, 30)),
    'steel': ('Prism-256', None),
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
    except RuntimeError:          # some library assets (steel) refuse edits; their own density is used
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
    col = adsk.core.ObjectCollection.create()
    for e in edges:
        col.add(e)
    fi = comp.features.filletFeatures.createInput()
    fi.addConstantRadiusEdgeSet(col, adsk.core.ValueInput.createByReal(r_mm * CM), True)
    return comp.features.filletFeatures.add(fi)


def polar(n, r, a0_deg=0.0):
    """n points on a circle (angle in the local x-y plane)."""
    return [(r * math.cos(math.radians(a0_deg + 360.0 * k / n)), r * math.sin(math.radians(a0_deg + 360.0 * k / n)))
            for k in range(n)]
