"""Exploded-assembly renderer for the v5 robot (runs inside Fusion through fusion_rpc.py).

The v2 engine (robot_cad_v2/manufacturing/explain/assembly/scripts/axlib.py) adapted to v5:
- the parts come from export/WheelBiped_v5.f3d (hub version 11, nominal pose, robot part names), imported into an
  unsaved document; the hub document (the user's pose) is never opened or changed;
- the v5 model already holds almost every fastener ('FX ...' occurrences): a pick with each=<travel vector> copies
  every matching occurrence on its own, with its own axis, so the screws turn in; only the fasteners the model lacks
  (hub insert flange bolts, knee cap screws and their inserts) are built here from their BOM sizes;
- a pick can take one body of an occurrence (body=...) and filter by position (side, where);
- steps may show some groups see-through (ghost) and move groups to any offset (a 4-tuple target), not only onto
  their seat or onto another group (the shank goes into the thigh's fork sideways, then onto the knee output).

Module names are axlib5 / job5_* so a v2 script in the same Fusion session (axlib / job_*) never loads these.
Coordinates: jobs are written in CAD axes (mm; v5 robot: +y = right side _R, -x = front); Fusion world = (y, z, x)
in cm.  Offsets are 4-tuples (dx, dy, dz, a): a vector plus an axial back-off a along each fastener's own travel
direction (a built fastener's body is made in a local frame whose +z is the direction it travels when fitted).
"""
import importlib
import json
import math
import os
import sys
import tempfile

import adsk.core
import adsk.fusion

ROOT = r'C:\Users\User\Desktop\myrobot\my_robot_project\robot_cad_v5'
FUS = os.path.join(ROOT, 'fusion')
HERE = os.path.join(ROOT, r'manufacturing\explain\assembly\scripts')
F3D = os.path.join(ROOT, r'export\WheelBiped_v5.f3d')
WORK = os.path.join(tempfile.gettempdir(), 'wbv5_assembly')     # raw captures / frames (not in git)
NS = 'ax5'
GHOST = 0.22                                                     # opacity of see-through groups


def modules():
    if FUS not in sys.path:
        sys.path.insert(0, FUS)
    sys.modules.pop('v5run', None)
    import v5run
    assert v5run.__file__.lower().startswith(FUS.lower()), v5run.__file__
    return v5run.load()


def load_job(job):
    if HERE not in sys.path:
        sys.path.insert(0, HERE)
    M = modules()
    name = 'job5_' + job
    sys.modules.pop(name, None)
    mod = importlib.import_module(name)
    assert mod.__file__.lower().startswith(HERE.lower()), mod.__file__
    return mod.config(M), M


def work(job, sub):
    d = os.path.join(WORK, job, sub)
    os.makedirs(d, exist_ok=True)
    return d


def rf(p):
    return (p[1], p[2], p[0])


def f2r(p):
    """Fusion world point (cm) -> CAD mm."""
    return (p.z * 10.0, p.x * 10.0, p.y * 10.0)


def unit(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v)


def along(a, b, d):
    """Half-space (CAD) keeping the part of a link within d of joint a (x, z), towards joint b."""
    u = unit((b[0] - a[0], 0.0, b[1] - a[1]))
    return [((a[0] + u[0] * d, 0.0, a[1] + u[2] * d), u)]


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
    """Body of a built fastener dict (kind, p, origin, ez) in CAD coordinates.  Local frame: +z = travel direction,
    origin = head bearing face (screws), top face (countersunk), contact face (insert)."""
    C = M['cadlib']
    B = C.Builder()
    k, p = f['kind'], f['p']
    if k in ('shcs', 'lowhead', 'csk'):
        d, L = p['d'], p['L']
        if k == 'csk':
            h = (p['dk'] - d) / 2
            body = B.cyl((0, 0, 0), (0, 0, h), p['dk'] / 2, d / 2)
            C.union(body, B.zcyl(0, 0, h - 0.01, L, d / 2))
            top = 0.0
        else:
            hk = p['k']
            body = B.zcyl(0, 0, -hk, 0.0, p['dk'] / 2)
            C.union(body, B.zcyl(0, 0, -0.01, L, d / 2))
            top = -hk
        C.cut(body, _hexp(B, p['s'], top - 0.2, top + p['t']))
        _thread(C, B, body, d, 0.0 if k != 'csk' else 2.0, L, p.get('pitch', 0.5))
    elif k == 'insert':
        body = _ring(B, 0.0, p['L'], p['od'] / 2, p['id'] / 2)
        for zc in (p['L'] * 0.28, p['L'] * 0.72):
            C.cut(body, _ring(B, zc - 0.35, zc + 0.35, p['od'], p['od'] / 2 - 0.25))
    else:
        raise ValueError(k)
    ez = unit(f['ez'])
    a = (1.0, 0.0, 0.0) if abs(ez[0]) < 0.9 else (0.0, 1.0, 0.0)
    ex = unit(C.cross(a, ez))
    ey = C.cross(ez, ex)
    C.transform(body, C.frame(f['origin'], ex, ey, ez))
    return body


# standard sizes (mm): DIN 7984 / ISO 10642 heads, heat-set inserts
def lowhead(d, L):
    t = {2.5: (4.5, 1.5, 1.5, 0.9), 3: (5.5, 2.0, 2.0, 1.1)}[d]
    return dict(kind='lowhead', p=dict(d=d, L=L, dk=t[0], k=t[1], s=t[2], t=t[3], pitch=0.45))


def csk(d, L):
    t = {3: (6.72, 2.0, 1.2), 4: (8.96, 2.5, 1.6)}[d]
    return dict(kind='csk', p=dict(d=d, L=L, dk=t[0], s=t[1], t=t[2], pitch=0.5))


def insert(d, L, od):
    return dict(kind='insert', p=dict(L=L, od=od, id=d * 0.85))


def fast(group, name, spec, origin, ez, look='steel'):
    f = dict(spec)
    f.update(group=group, name=name, origin=tuple(origin), ez=tuple(ez), look=look)
    return f


def pick(g, path, body=None, trims=None, side=None, where=None, each=None):
    """Parts of group g: occurrences whose full path starts with path; body = one body name only; side = sign of
    the CAD y of the occurrence centre; where = (axis 0/1/2, lo, hi) on that centre (CAD mm); each = travel vector
    (CAD): every occurrence becomes its own fastener occurrence turning about its own axis."""
    return dict(g=g, path=path, body=body, trims=trims, side=side, where=where, each=each)


def sector(c_xz, a_deg, b_deg):
    """Removal region (list of half-spaces, CAD) of the sector a..b (deg, atan2(dz, dx)) about the y axis
    through (x, z) = c_xz (b - a < 180)."""
    ta, tb = math.radians(a_deg), math.radians(b_deg)
    c = (c_xz[0], 0.0, c_xz[1])
    return [(c, (math.sin(ta), 0.0, -math.cos(ta))), (c, (-math.sin(tb), 0.0, math.cos(tb)))]


def on_sector_face(c_xz, t_deg, r, y):
    t = math.radians(t_deg)
    return (c_xz[0] + r * math.cos(t), y, c_xz[1] + r * math.sin(t))


def polar(c_xz, r, t_deg):
    t = math.radians(t_deg)
    return (c_xz[0] + r * math.cos(t), c_xz[1] + r * math.sin(t))


# ----------------------------------------------------------------------------- documents
def _docs(key):
    app = adsk.core.Application.get()
    out = []
    for d in app.documents:
        dd = adsk.fusion.Design.cast(d.products.itemByProductType('DesignProductType'))
        if dd and dd.attributes.itemByName(NS, key):
            out.append((d, dd))
    return out


def src_doc():
    """The imported f3d (unsaved, tagged NS/src); imported once and kept open between jobs."""
    found = _docs('src')
    if found:
        d, dd = found[0]
        d.activate()
    else:
        app = adsk.core.Application.get()
        im = app.importManager
        d = im.importToNewDocument(im.createFusionArchiveImportOptions(F3D))
        dd = adsk.fusion.Design.cast(d.products.itemByProductType('DesignProductType'))
        dd.attributes.add(NS, 'src', F3D)
    tag = sys.modules['v5run'].TAG                    # v5run.doc() must never take this copy for the hub document
    a = dd.attributes.itemByName(tag[0], tag[1])
    if a:
        a.deleteMe()
    return d, dd


def close_src():
    for d, _dd in _docs('src'):
        d.close(False)


def doc(job):
    for d, dd in _docs('job'):
        a = dd.attributes.itemByName(NS, 'job')
        if a.value == job:
            if adsk.core.Application.get().activeDocument != d:
                d.activate()
            return d, dd
    raise RuntimeError('scratch doc for %s missing: run f5_build.py' % job)


def _bbox(bodies):
    bb = None
    for b in bodies:
        x = b.boundingBox
        if bb is None:
            bb = x.copy()
        else:
            bb.combine(x)
    return bb


def build(job):
    cfg, M = load_job(job)
    C = M['cadlib']
    app = C.app()
    for d, _dd in _docs('job'):
        d.close(False)
    _sd, sdes = src_doc()
    nt = sys.modules['v5run'].NAMES_TAG
    if not sdes.attributes.itemByName(nt[0], nt[1]):
        raise RuntimeError('the f3d has the old part names')
    B = C.Builder()
    items, eachs, counts = {}, [], {}
    occs = list(sdes.rootComponent.allOccurrences)
    for pk in cfg['picks']:
        g = pk['g']
        for o in occs:
            if not o.fullPathName.startswith(pk['path']):
                continue
            bodies = [b for b in o.bRepBodies if b.isVisible and (pk['body'] is None or b.name == pk['body'])]
            if not bodies:
                continue
            bb = _bbox(bodies)
            cw = adsk.core.Point3D.create((bb.minPoint.x + bb.maxPoint.x) / 2, (bb.minPoint.y + bb.maxPoint.y) / 2,
                                          (bb.minPoint.z + bb.maxPoint.z) / 2)
            c = f2r(cw)
            if pk['side'] and c[1] * pk['side'] < 0:
                continue
            if pk['where'] and not (pk['where'][1] <= c[pk['where'][0]] <= pk['where'][2]):
                continue
            copies = []
            for b in bodies:
                t = C.TBM.copy(b)
                for (p, n) in pk['trims'] or ():
                    C.inter(t, B.halfspace(p, n))
                if t.faces.count == 0:
                    continue
                copies.append((t, cfg['recolor'].get(g, b.appearance), b.name))
            if not copies:
                continue
            counts[g] = counts.get(g, 0) + 1
            if pk['each']:
                eachs.append((g, o.component.name, copies, (cw.x * 10, cw.y * 10, cw.z * 10) + rf(unit(pk['each']))))
            else:
                items.setdefault(g, []).extend(copies)
    fasts = [(f, fastener_body(M, f)) for f in cfg['fasteners']]
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    nd = adsk.fusion.Design.cast(app.activeProduct)
    nd.designType = adsk.fusion.DesignTypes.DirectDesignType
    nd.attributes.add(NS, 'job', job)
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
        if a is None:
            return C.appearance('dark grey')
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
        occ.component.attributes.add(NS, 'group', group)
        if axis is not None:
            occ.component.attributes.add(NS, 'axis', ','.join('%.4f' % v for v in axis))
        return occ

    for g, v in items.items():
        add_occ(g, v, g)
    for g, name, v, ax in eachs:
        add_occ(name, v, g, ax)
    for f, t in fasts:
        o, e = rf(f['origin']), rf(unit(f['ez']))
        add_occ(f['name'], [(t, f['look'], f['name'])], f['group'], o + e)
        counts[f['group']] = counts.get(f['group'], 0) + 1
    missing = sorted({pk['g'] for pk in cfg['picks']} - set(counts))
    return counts, missing


# ----------------------------------------------------------------------------- state, cameras, captures
def groups(dd):
    out = {}
    for o in dd.rootComponent.occurrences:
        a = o.component.attributes.itemByName(NS, 'group')
        if a is None:
            continue
        ax = o.component.attributes.itemByName(NS, 'axis')
        out.setdefault(a.value, []).append([o, tuple(float(v) for v in ax.value.split(',')) if ax else None, None])
    return out


def place(G, off, spin=None, visible=None, ghost=()):
    """off: group -> (dx, dy, dz, a) CAD mm; spin: group -> angle (rad) about each fastener's own axis; ghost: groups
    drawn see-through."""
    spin = spin or {}
    for g, lst in G.items():
        o4 = tuple(off.get(g, (0.0, 0.0, 0.0, 0.0)))
        ang = spin.get(g, 0.0)
        v = rf(o4[:3])
        op = GHOST if g in ghost else 1.0
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
            if abs(o.component.opacity - op) > 1e-3:
                o.component.opacity = op
            if visible is not None:
                on = g in visible
                if o.isLightBulbOn != on:
                    o.isLightBulbOn = on


Z4 = (0.0, 0.0, 0.0, 0.0)


def states(cfg):
    """Per step: offsets before / after, sub-moves {group: (from, to)}, groups a still shows.  A move target is 0
    (its seat), a group name (onto that group's current offset) or a 4-tuple offset.  st['pre'] = offsets set at the
    step's start without a move (fasteners that wait far out in the overview start just outside the part)."""
    off = {g: tuple(v) for g, v in cfg['home'].items()}
    placed = set(cfg['fixed'])
    out = []
    for st in cfg['steps']:
        off.update({g: tuple(float(x) for x in v) for g, v in st.get('pre', {}).items()})
        before = dict(off)
        seq = []
        for sub in st['moves']:
            mv = {}
            for g, tgt in sub:
                if isinstance(tgt, str):
                    to = tuple(off[tgt][:3]) + (0.0,)
                elif isinstance(tgt, tuple):
                    to = tuple(float(v) for v in tgt)
                else:
                    to = Z4
                mv[g] = (off[g], to)
            for g, (_a, b) in mv.items():
                off[g] = b
            seq.append(mv)
        gs = []
        for sub in st['moves']:
            for g, _t in sub:
                if g not in gs:
                    gs.append(g)
        vis = set(st['bench']) if st.get('bench') else placed | set(gs)
        placed |= {g for g in gs if off[g] == Z4}
        out.append(dict(st, before=before, after=dict(off), seq=seq, vis=sorted(vis), groups=gs,
                        ghost=list(st.get('ghost', []))))
    return out


def lerp4(a, b, t):
    return tuple(a[i] + (b[i] - a[i]) * t for i in range(4))


def grid_off():
    ui = adsk.core.Application.get().userInterface
    lc = adsk.core.ListControlDefinition.cast(ui.commandDefinitions.itemById('ViewLayoutGridOnCommand').controlDefinition)
    lc.listItems.item(lc.listItems.count - 1).isSelected = True       # radio list: [grid on, grid off]


def setup_view():
    app = adsk.core.Application.get()
    app.userInterface.activeSelections.clear()
    vp = app.activeViewport
    vp.visualStyle = adsk.core.VisualStyles.ShadedWithVisibleEdgesOnlyVisualStyle
    grid_off()
    return vp


def set_view(vdir, target):
    """Perspective view from CAD direction vdir, fitted to the visible bodies."""
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


def recenter(target_cad=None, factor=1.0):
    """Keep the view direction; look at target (CAD mm, default: current target); eye at factor x the distance."""
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    e, t = cam.eye, cam.target
    v = (e.x - t.x, e.y - t.y, e.z - t.z)
    if target_cad is None:
        tx, ty, tz = t.x, t.y, t.z
    else:
        w = rf(target_cad)
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
