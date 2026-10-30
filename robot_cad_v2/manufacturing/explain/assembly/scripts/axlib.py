"""Exploded-assembly renderer for the v2 robot (runs inside Fusion through fusion_rpc.py).

A job module (job_<name>.py) lists the parts to copy from WheelBiped_v2, the BOM-only fasteners to model, the
exploded (home) offsets, the assembly steps, the cameras and the label texts.  build() copies the parts into a NEW
unsaved direct-design document (the robot document is never changed); every group / fastener is its own
occurrence, moved with transform2.

Coordinates: jobs are written in robot axes (mm, x forward, y left, z up); Fusion world = (robot y, z, x) in cm.
Offsets are 4-tuples (dx, dy, dz, a): a robot vector plus an axial back-off a along each fastener's own insertion
direction (a fastener's body is built in a local frame whose +z is the direction it travels when fitted).
"""
import importlib
import json
import math
import os
import sys
import tempfile

import adsk.core
import adsk.fusion

FUS = r'C:\Users\User\Desktop\myrobot\my_robot_project\robot_cad_v2\fusion'
HERE = r'C:\Users\User\Desktop\myrobot\my_robot_project\robot_cad_v2\manufacturing\explain\assembly\scripts'
WORK = os.path.join(tempfile.gettempdir(), 'wbv2_assembly')     # raw captures / frames (not in git)
SRC_DOC = 'WheelBiped_v2'


def modules():
    if FUS not in sys.path:
        sys.path.insert(0, FUS)
    import v2run
    return v2run.load()


def load_job(job):
    if HERE not in sys.path:
        sys.path.insert(0, HERE)
    M = modules()
    name = 'job_' + job
    sys.modules.pop(name, None)
    return importlib.import_module(name).config(M), M


def work(job, sub):
    d = os.path.join(WORK, job, sub)
    os.makedirs(d, exist_ok=True)
    return d


def rf(p):
    return (p[1], p[2], p[0])


def unit(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v)


# ----------------------------------------------------------------------------- fastener bodies (local frame)
def _hexp(B, af, z0, z1):
    r = af / 2 / math.cos(math.radians(30))
    poly = [(r * math.cos(math.radians(30 + 60 * k)), r * math.sin(math.radians(30 + 60 * k))) for k in range(6)]
    return B.prism(poly, z0, z1, plane='xy')


def _ring(B, z0, z1, ro, ri):
    b = B.zcyl(0, 0, z0, z1, ro)
    C = sys.modules['cadlib']
    if ri > 0:
        C.cut(b, B.zcyl(0, 0, z0 - 1, z1 + 1, ri))
    return b


def _thread(C, B, body, d, z0, z1, pitch):
    n = int(abs(z1 - z0) / pitch)
    for i in range(1, n):
        zc = z0 + i * pitch
        C.cut(body, _ring(B, zc - 0.18, zc + 0.18, d / 2 + 1.0, d / 2 - 0.16))


def fastener_body(M, f):
    """Body of a fastener dict (kind, p, origin, ez) in robot coordinates.  Local frame: +z = travel direction,
    origin = head bearing face (screws), top face (countersunk), contact face (washer / nut / insert / pin)."""
    C = M['cadlib']
    B = C.Builder()
    k, p = f['kind'], f['p']
    if k in ('shcs', 'lowhead', 'button', 'csk'):
        d, L = p['d'], p['L']
        if k == 'csk':
            h = (p['dk'] - d) / 2
            body = B.cyl((0, 0, 0), (0, 0, h), p['dk'] / 2, d / 2)
            C.union(body, B.zcyl(0, 0, h - 0.01, L, d / 2))
            top = 0.0
        else:
            hk = p['k']
            if k == 'button':
                body = B.zcyl(0, 0, -0.45 * hk, 0.0, p['dk'] / 2)
                C.union(body, B.cyl((0, 0, -hk), (0, 0, -0.45 * hk + 0.01), p['dk'] * 0.3, p['dk'] / 2))
            else:
                body = B.zcyl(0, 0, -hk, 0.0, p['dk'] / 2)
            C.union(body, B.zcyl(0, 0, -0.01, L, d / 2))
            top = -hk
        C.cut(body, _hexp(B, p['s'], top - 0.2, top + p['t']))
        tl = p.get('thread', L)
        _thread(C, B, body, d, L - tl, L, p.get('pitch', 0.8))
    elif k == 'washer':
        body = _ring(B, 0.0, p['t'], p['od'] / 2, p['id'] / 2)
    elif k in ('nut', 'nyloc'):
        if k == 'nyloc':                                    # nylon dome first (the side away from the part)
            body = _hexp(B, p['s'], 0.3 * p['m'], p['m'])
            C.union(body, B.zcyl(0, 0, 0.0, 0.3 * p['m'] + 0.01, p['s'] * 0.46))
        else:
            body = _hexp(B, p['s'], 0.0, p['m'])
        C.cut(body, B.zcyl(0, 0, -1, p['m'] + 1, p['d'] / 2))
    elif k == 'insert':
        body = _ring(B, 0.0, p['L'], p['od'] / 2, p['id'] / 2)
        for zc in (p['L'] * 0.28, p['L'] * 0.72):
            C.cut(body, _ring(B, zc - 0.35, zc + 0.35, p['od'], p['od'] / 2 - 0.25))
    elif k == 'pin':
        d, L = p['d'], p['L']
        body = B.zcyl(0, 0, 0.3, L - 0.3, d / 2)
        C.union(body, B.cyl((0, 0, 0), (0, 0, 0.31), d / 2 - 0.3, d / 2))
        C.union(body, B.cyl((0, 0, L - 0.31), (0, 0, L), d / 2, d / 2 - 0.3))
    else:
        raise ValueError(k)
    ez = unit(f['ez'])
    a = (1.0, 0.0, 0.0) if abs(ez[0]) < 0.9 else (0.0, 1.0, 0.0)
    ex = unit(C.cross(a, ez))
    ey = C.cross(ez, ex)
    C.transform(body, C.frame(f['origin'], ex, ey, ez))
    return body


# standard sizes (mm): ISO 4762 / DIN 7984 / ISO 7380 / ISO 10642 heads, DIN 125 washers, nuts
def shcs(d, L, **kw):
    t = {2.5: (4.5, 2.5, 2.0, 1.1), 3: (5.5, 3.0, 2.5, 1.3), 4: (7.0, 4.0, 3.0, 2.0), 5: (8.5, 5.0, 4.0, 2.5)}[d]
    return dict(kind='shcs', p=dict(d=d, L=L, dk=t[0], k=t[1], s=t[2], t=t[3], pitch=0.45 + 0.1 * d, **kw))


def lowhead(d, L):
    t = {2.5: (4.5, 1.5, 1.5, 0.9), 3: (5.5, 2.0, 2.0, 1.1)}[d]
    return dict(kind='lowhead', p=dict(d=d, L=L, dk=t[0], k=t[1], s=t[2], t=t[3], pitch=0.7))


def button(d, L):
    t = {3: (5.7, 1.65, 2.0, 1.0)}[d]
    return dict(kind='button', p=dict(d=d, L=L, dk=t[0], k=t[1], s=t[2], t=t[3], pitch=0.8))


def csk(d, L):
    t = {4: (8.96, 2.5, 1.6), 5: (11.2, 3.0, 2.0)}[d]
    return dict(kind='csk', p=dict(d=d, L=L, dk=t[0], s=t[1], t=t[2], pitch=0.45 + 0.1 * d))


def washer(d):
    t = {3: (7.0, 3.2, 0.5), 4: (9.0, 4.3, 0.8)}[d]
    return dict(kind='washer', p=dict(od=t[0], id=t[1], t=t[2]))


def nut(d, nyloc=False):
    t = {3: (5.5, 2.4), 4: (7.0, 5.0)}[d]
    return dict(kind='nyloc' if nyloc else 'nut', p=dict(s=t[0], m=t[1], d=d))


def insert(d, L, od):
    return dict(kind='insert', p=dict(L=L, od=od, id=d * 0.85))


def pin(d, L):
    return dict(kind='pin', p=dict(d=d, L=L))


def fast(group, name, spec, origin, ez, look='steel'):
    f = dict(spec)
    f.update(group=group, name=name, origin=tuple(origin), ez=tuple(ez), look=look)
    return f


def sector(c_xz, a_deg, b_deg):
    """Removal region (list of half-spaces, robot) of the sector a..b (deg, atan2(dz, dx)) about the robot-y axis
    through (x, z) = c_xz (b - a < 180)."""
    ta, tb = math.radians(a_deg), math.radians(b_deg)
    c = (c_xz[0], 0.0, c_xz[1])
    return [(c, (math.sin(ta), 0.0, -math.cos(ta))), (c, (-math.sin(tb), 0.0, math.cos(tb)))]


def on_sector_face(c_xz, t_deg, r, y):
    t = math.radians(t_deg)
    return (c_xz[0] + r * math.cos(t), y, c_xz[1] + r * math.sin(t))


# ----------------------------------------------------------------------------- scratch document
def _scratch_docs():
    app = adsk.core.Application.get()
    out = []
    for d in app.documents:
        dd = adsk.fusion.Design.cast(d.products.itemByProductType('DesignProductType'))
        if dd and (dd.attributes.itemByName('ax', 'job') or dd.attributes.itemByName('wx', 'tag')):
            out.append((d, dd))
    return out


def doc(job):
    for d, dd in _scratch_docs():
        a = dd.attributes.itemByName('ax', 'job')
        if a and a.value == job:
            if adsk.core.Application.get().activeDocument != d:
                d.activate()
            return d, dd
    raise RuntimeError('scratch doc for %s missing: run f_ax_build.py' % job)


def build(job):
    cfg, M = load_job(job)
    C = M['cadlib']
    app = C.app()
    for d, _dd in _scratch_docs():
        d.close(False)
    src = None
    for d in app.documents:
        if d.name.startswith(SRC_DOC):
            src = d
    if src is None:
        raise RuntimeError(SRC_DOC + ' is not open')
    src.activate()
    des = C.design()
    B = C.Builder()
    items = {}
    for g, pre, side, trims in cfg['picks']:
        for o in des.rootComponent.allOccurrences:
            if not o.fullPathName.startswith(pre):
                continue
            if side:
                bb = o.boundingBox
                if (bb.minPoint.x + bb.maxPoint.x) * side < 0:
                    continue
            for b in o.bRepBodies:
                if not b.isVisible:
                    continue
                t = C.TBM.copy(b)
                for (p, n) in trims or ():
                    C.inter(t, B.halfspace(p, n))
                if t.faces.count == 0:
                    continue
                items.setdefault(g, []).append((t, cfg['recolor'].get(g, b.appearance), b.name))
    fasts = [(f, fastener_body(M, f)) for f in cfg['fasteners']]
    newd = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    nd = adsk.fusion.Design.cast(app.activeProduct)
    nd.designType = adsk.fusion.DesignTypes.DirectDesignType
    nd.attributes.add('ax', 'job', job)
    root = nd.rootComponent
    root.isOriginFolderLightBulbOn = False
    cache = {}

    def look_of(a):
        if isinstance(a, str):
            return C.appearance(a)
        if isinstance(a, tuple):
            na = nd.appearances.itemByName(a[0])
            if na is None:
                base = app.materialLibraries.item(0).appearances.itemById('Prism-127')
                na = nd.appearances.addByCopy(base, a[0])
                pr = na.appearanceProperties.itemById('opaque_albedo') or na.appearanceProperties.itemById('surface_albedo')
                pr.value = adsk.core.Color.create(a[1][0], a[1][1], a[1][2], 255)
            return na
        if a.name not in cache:
            try:
                cache[a.name] = nd.appearances.itemByName(a.name) or nd.appearances.addByCopy(a, a.name)
            except Exception:
                cache[a.name] = C.appearance('dark grey')
        return cache[a.name]

    def add_occ(name, bodies, group, axis=None):
        occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        occ.component.name = name
        for t, a, bn in bodies:
            b = occ.component.bRepBodies.add(t)
            b.name = bn
            b.appearance = look_of(a)
        occ.component.attributes.add('ax', 'group', group)
        if axis is not None:
            occ.component.attributes.add('ax', 'axis', ','.join('%.4f' % v for v in axis))
        return occ

    for g, v in items.items():
        add_occ(g, v, g)
    missing = [g for g, _p, _s, _t in cfg['picks'] if g not in items]
    for f, t in fasts:
        o, e = rf(f['origin']), rf(unit(f['ez']))
        add_occ(f['name'], [(t, f['look'], f['name'])], f['group'], o + e)
    return {g: len(v) for g, v in items.items()}, len(fasts), sorted(set(missing))


# ----------------------------------------------------------------------------- state, cameras, captures
def groups(dd):
    out = {}
    for o in dd.rootComponent.occurrences:
        a = o.component.attributes.itemByName('ax', 'group')
        if a is None:
            continue
        ax = o.component.attributes.itemByName('ax', 'axis')
        out.setdefault(a.value, []).append([o, tuple(float(v) for v in ax.value.split(',')) if ax else None, None])
    return out


def place(G, off, spin=None, visible=None):
    """off: group -> (dx, dy, dz, a) robot mm; spin: group -> angle (rad) about each fastener's own axis."""
    spin = spin or {}
    for g, lst in G.items():
        o4 = tuple(off.get(g, (0.0, 0.0, 0.0, 0.0)))
        ang = spin.get(g, 0.0)
        v = rf(o4[:3])
        for rec in lst:
            o, ax, last = rec
            key = (o4, ang)
            if key != last:
                m = adsk.core.Matrix3D.create()
                t = list(v)
                if ax is not None:
                    if ang:
                        m.setToRotation(ang, adsk.core.Vector3D.create(*ax[3:]),
                                        adsk.core.Point3D.create(ax[0] / 10, ax[1] / 10, ax[2] / 10))
                    t = [t[i] - o4[3] * ax[3 + i] for i in range(3)]
                tr = m.translation
                tr.x += t[0] / 10
                tr.y += t[1] / 10
                tr.z += t[2] / 10
                m.translation = tr
                o.transform2 = m
                rec[2] = key
            if visible is not None:
                on = g in visible
                if o.isLightBulbOn != on:
                    o.isLightBulbOn = on


Z4 = (0.0, 0.0, 0.0, 0.0)


def states(cfg):
    """Per step: offsets before / after, sub-moves {group: (from, to)}, groups a still shows."""
    off = {g: tuple(v) for g, v in cfg['home'].items()}
    placed = set(cfg['fixed'])
    out = []
    for st in cfg['steps']:
        before = dict(off)
        seq = []
        for sub in st['moves']:
            mv = {}
            for g, tgt in sub:
                to = tuple(off[tgt][:3]) + (0.0,) if isinstance(tgt, str) else Z4
                mv[g] = (off[g], to)
            for g, (_a, b) in mv.items():
                off[g] = b
            seq.append(mv)
        gs = [g for sub in st['moves'] for g, _t in sub]
        vis = set(st['bench']) if st.get('bench') else placed | set(gs)
        placed |= {g for g in gs if off[g] == Z4}
        out.append(dict(st, before=before, after=dict(off), seq=seq, vis=sorted(vis), groups=gs))
    return out


def lerp4(a, b, t):
    return tuple(a[i] + (b[i] - a[i]) * t for i in range(4))


def grid_off():
    ui = adsk.core.Application.get().userInterface
    lc = adsk.core.ListControlDefinition.cast(ui.commandDefinitions.itemById('ViewLayoutGridOnCommand').controlDefinition)
    lc.listItems.item(lc.listItems.count - 1).isSelected = True       # radio list: [grid on, grid off]


def setup_view():
    vp = adsk.core.Application.get().activeViewport
    vp.visualStyle = adsk.core.VisualStyles.ShadedWithVisibleEdgesOnlyVisualStyle
    grid_off()
    return vp


def set_view(vdir, target):
    """Perspective view from robot direction vdir, fitted to the visible bodies."""
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    d = rf(unit(vdir))
    t = rf(target)
    cam.target = adsk.core.Point3D.create(t[0] / 10, t[1] / 10, t[2] / 10)
    cam.eye = adsk.core.Point3D.create((t[0] + d[0] * 800) / 10, (t[1] + d[1] * 800) / 10, (t[2] + d[2] * 800) / 10)
    cam.upVector = adsk.core.Vector3D.create(0, 1, 0)
    cam.cameraType = adsk.core.CameraTypes.PerspectiveCameraType
    cam.isSmoothTransition = False
    cam.isFitView = True
    vp.camera = cam
    vp.fit()
    vp.refresh()


def recenter(target_robot=None, factor=1.0):
    """Keep the view direction; look at target (robot mm, default: current target); eye at factor x the distance."""
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    e, t = cam.eye, cam.target
    v = (e.x - t.x, e.y - t.y, e.z - t.z)
    if target_robot is None:
        tx, ty, tz = t.x, t.y, t.z
    else:
        w = rf(target_robot)
        tx, ty, tz = w[0] / 10, w[1] / 10, w[2] / 10
    cam.isSmoothTransition = False
    cam.isFitView = False
    cam.target = adsk.core.Point3D.create(tx, ty, tz)
    cam.eye = adsk.core.Point3D.create(tx + v[0] * factor, ty + v[1] * factor, tz + v[2] * factor)
    cam.viewExtents = cam.viewExtents * factor
    vp.camera = cam
    vp.refresh()


def cam_dict():
    c = adsk.core.Application.get().activeViewport.camera
    return dict(eye=[c.eye.x, c.eye.y, c.eye.z], target=[c.target.x, c.target.y, c.target.z],
                up=[c.upVector.x, c.upVector.y, c.upVector.z], ext=c.viewExtents)


def set_cam(cd):
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    cam.cameraType = adsk.core.CameraTypes.PerspectiveCameraType
    cam.isSmoothTransition = False
    cam.isFitView = False
    cam.eye = adsk.core.Point3D.create(*cd['eye'])
    cam.target = adsk.core.Point3D.create(*cd['target'])
    cam.upVector = adsk.core.Vector3D.create(*cd['up'])
    cam.viewExtents = cd['ext']
    vp.camera = cam
    vp.refresh()


def project(G, groups_):
    """View pixel of each visible group: bbox centre of a part, of the topmost member of a fastener set."""
    vp = adsk.core.Application.get().activeViewport
    out = {}
    for g in groups_:
        occs = [r[0] for r in G.get(g, []) if r[0].isLightBulbOn]
        if not occs:
            continue
        if len(occs) > 1 and all(r[1] is not None for r in G[g]):
            occs = [max(occs, key=lambda o: o.boundingBox.maxPoint.y)]
        bb = None
        for o in occs:
            b = o.boundingBox
            if bb is None:
                bb = b.copy()
            else:
                bb.combine(b)
        c = adsk.core.Point3D.create((bb.minPoint.x + bb.maxPoint.x) / 2, (bb.minPoint.y + bb.maxPoint.y) / 2,
                                     (bb.minPoint.z + bb.maxPoint.z) / 2)
        p = vp.modelToViewSpace(c)
        out[g] = [p.x, p.y]
    return out


def centre(G, g):
    bb = None
    for r in G[g]:
        b = r[0].boundingBox
        if bb is None:
            bb = b.copy()
        else:
            bb.combine(b)
    return [(bb.minPoint.x + bb.maxPoint.x) * 5, (bb.minPoint.y + bb.maxPoint.y) * 5, (bb.minPoint.z + bb.maxPoint.z) * 5]


def project_world(w_mm):
    vp = adsk.core.Application.get().activeViewport
    p = vp.modelToViewSpace(adsk.core.Point3D.create(w_mm[0] / 10, w_mm[1] / 10, w_mm[2] / 10))
    return [p.x, p.y]


def snap(path):
    vp = adsk.core.Application.get().activeViewport
    vp.saveAsImageFile(path, vp.width, vp.height)
    return path


def dump(path, data):
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=1)
