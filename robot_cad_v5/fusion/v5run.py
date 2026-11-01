"""robot_cad_v5 = the v4 robot (robot_cad_v4/fusion, git 681ad00) with the wheels turned outboard and MJF PA12 legs
(legs_v5.py, concept A "monolith", chosen by the user 11-01).  The torso, hip plates, motors, cups, wheels and
fasteners are built by the v4 code; this module only swaps the links.

Run each step in its own MCP script (put this folder on sys.path, import v5run):
    v5run.build()            # makes a NEW document (attribute wbv5 / tag): v4 robot without harness, v4 links and
                             # output clamp rings removed, v5 legs added
    v5run.harness()          # v4 torso cables + the v5 leg cables (harness_v5), plugs, XT60 / XT90 / switch
    v5run.turn()             # wheel groups outboard - last and in its own script: in the build script the new parts'
                             # bounding boxes are stale and the per-part decision goes wrong, and adding parts to a
                             # link (cadlib.IdentityParent) resets turned children (idempotent: a second run turns 0)
    v5run.report()           # v5 leg parts: volume, mass, lumps / shells
    v5run.rib_check()        # every rib joined to its walls (point sampling), voids
    v5run.chords()           # loop anchor distances over the pose grid (sizes harness_v5.*_LOOP_L)
    v5run.interference()     # 6 poses: interference without the nominal loops + each pose's loops measured
    v5run.images()           # ../images (legs, robot, truss sections)
    v5run.robot_mass()       # v4 print densities (PETG parts) + tyre 721 g, robot mass / COM
    v5run.export()           # ../export/WheelBiped_v5.f3d
"""
import importlib
import math
import os
import sys

import adsk.core
import adsk.fusion

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.dirname(HERE)
V4 = os.path.join(os.path.dirname(BASE), 'robot_cad_v4', 'fusion')
IMG = os.path.join(BASE, 'images')
EXP = os.path.join(BASE, 'export')
TAG = ('wbv5', 'tag', 'wbv5_robot')
SIDES = ((1, 'L'), (-1, 'R'))
LOOK = {'main': 'black satin', 'insert': 'alu', 'bearing': 'steel', 'cap': 'dark grey', 'hipcap': 'dark grey'}
MAT = {'main': 'PA12 MJF', 'insert': 'Al6061', 'bearing': 'Steel', 'cap': 'Al6061', 'hipcap': 'PA12 MJF'}
V5_PARTS = ('thigh', 'shank', 'kneecap', 'kneebearing')
POSES = ((310, 0), (200, 0), (240, -60), (240, 60), (335, 0), (200, -80))       # (L0 mm, hip deg)


def load():
    """v4 modules (v4run.load) + legs_v5; this folder and the v4 one first on sys.path, files checked."""
    for p in (HERE, V4):
        while p in sys.path:
            sys.path.remove(p)
    sys.path.insert(0, V4)
    sys.path.insert(0, HERE)
    sys.modules.pop('v4run', None)
    import v4run
    M = v4run.load()
    for n in ('legs_v5', 'harness_v5'):
        sys.modules.pop(n, None)
    import legs_v5
    import harness_v5
    M['legs_v5'], M['harness_v5'], M['v4run'] = legs_v5, harness_v5, v4run
    M['poses'].HN = harness_v5                    # the v4 pose tools (loops, measure_loops, show_pose) use v5 cables
    for k in ('params', 'cadlib', 'legs'):
        assert M[k].__file__.lower().startswith(V4.lower()), (k, M[k].__file__)
    for m in (legs_v5, harness_v5):
        assert m.__file__.lower().startswith(HERE.lower()), m.__file__
    _pa12(M['cadlib'])
    return M


def _pa12(C):
    C.MATERIALS.setdefault('PA12 MJF', ('PrismMaterial-023', 1010.0))       # HP MJF PA12 1.01 g/cm3


def doc():
    app = adsk.core.Application.get()
    for d in app.documents:
        dd = adsk.fusion.Design.cast(d.products.itemByProductType('DesignProductType'))
        if dd and dd.attributes.itemByName(TAG[0], TAG[1]):
            if app.activeDocument != d:
                d.activate()
            return d
    raise RuntimeError('v5 document missing (run build() first)')


def _new_doc():
    app = adsk.core.Application.get()
    for d in list(app.documents):                      # one v5 document at a time
        dd = adsk.fusion.Design.cast(d.products.itemByProductType('DesignProductType'))
        if dd and dd.attributes.itemByName(TAG[0], TAG[1]):
            d.close(False)
    d = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    adsk.fusion.Design.cast(app.activeProduct).attributes.add(*TAG)
    return d


def f2r(p):
    return (p.z * 10.0, p.x * 10.0, p.y * 10.0)


def _centre(o):
    bb = o.boundingBox
    return f2r(adsk.core.Point3D.create((bb.minPoint.x + bb.maxPoint.x) / 2, (bb.minPoint.y + bb.maxPoint.y) / 2,
                                        (bb.minPoint.z + bb.maxPoint.z) / 2))


ORIG_INBOARD = ('clamp ring stator', 'FX DIN 7984 M2.5x8 stator')   # tibia parts that sat inboard of the shank
NOT_TURNED = ('shank_', 'kneebearing_', 'harness_', 'XT30U plug', 'JST PHR-3')   # v5 parts: built in place
REPLACE = ('AKE60-8_KV80_stator',)
STATOR = ('AKE60-8_KV80_stator', 'clamp ring stator', 'FX DIN 7984 M2.5x8 stator', 'magnet spacer')


def turn_wheels(M):
    """Turn the wheel group outboard (legs_v5.wheel_turn), the driver / cup / plugs / their screws also re-clocked
    (wheel_clock).  Decided per occurrence, so it is idempotent even when cadlib.IdentityParent has reset some child
    transforms: a part is turned when it sits on the other side of the shank mid-plane than in the v4 (inboard-wheel)
    build - every wheel_k child and the stator ring / its screws sat inboard, the other tibia parts near the axle
    outboard."""
    C, P, LV = M['cadlib'], M['params'], M['legs_v5']
    n = 0
    for s, k in SIDES:
        R = C.to_matrix3d(C.robot_to_fusion(LV.wheel_turn(s)))
        Rc = C.to_matrix3d(C.robot_to_fusion(LV.wheel_clock(s)))
        tib, whl = C.root_occ('tibia_' + k), C.root_occ('wheel_' + k)
        cand = [(o, False) for o in tib.childOccurrences] + [(o, True) for o in whl.childOccurrences]
        for o, in_wheel in cand:
            name = o.component.name
            if name.startswith(NOT_TURNED):
                continue
            c = _centre(o)
            if not in_wheel and math.hypot(c[0], c[2] - P.R_WHEEL) >= 70.0:
                continue
            orig_in = in_wheel or name.startswith(ORIG_INBOARD)
            turned = abs(c[1]) > LV.Y_MSH if orig_in else abs(c[1]) < LV.Y_MSH
            if turned:
                continue
            m = o.transform2.copy()
            m.transformBy(R)
            if not in_wheel and not name.startswith(STATOR):
                m.transformBy(Rc)
            if name.startswith(REPLACE):
                # the AKE60 STEP component loses a transform2 change at the next recompute (e.g. any occurrence
                # deleted, 11-01): a new instance at the turned place instead (links at the nominal pose = identity)
                C.instance(whl if in_wheel else tib, o.component, m)
                o.deleteMe()
            else:
                o.transform2 = m
            n += 1
    return n


def build():
    _new_doc()
    M = load()
    M['v4run'].build(with_harness=False)          # the harness follows the v4 links: re-route it for v5 later
    C, LV = M['cadlib'], M['legs_v5']             # (v4run.build reloaded the v4 modules in place)
    _pa12(C)
    M['finalize'].show_all()
    dropped = []
    for s, k in SIDES:                             # v4 links out
        for link, name in (('femur_', 'thigh_'), ('tibia_', 'shank_')):
            for o in list(C.root_occ(link + k).childOccurrences):
                if o.component.name == name + k:
                    dropped.append(o.component.name)
                    o.deleteMe()
    for o in list(C.design().rootComponent.allOccurrences):      # the 6061 hub inserts replace the output rings
        if o.component.name.startswith('clamp ring output') and o.assemblyContext is not None \
                and o.assemblyContext.component.name != '_library':
            dropped.append(o.component.name)
            o.deleteMe()
    for s, k in SIDES:
        th, sh, kn = LV.thigh(s), LV.shank(s), LV.knee_parts(s)
        for link, name, parts in (('femur_', 'thigh_' + k, th), ('tibia_', 'shank_' + k, sh),
                                  ('femur_', 'kneecap_' + k, {'cap': kn['cap']}),
                                  ('tibia_', 'kneebearing_' + k, {'bearing': kn['bearing']})):
            comp = C.new_part(C.root_occ(link + k), name)
            for role, body in parts.items():
                C.add_body(comp, body, '%s %s' % (name, role), MAT[role], LOOK[role])
    C.design().rootComponent.partNumber = 'WheelBiped_v5'
    return dict(dropped=dropped)


def turn():
    """Wheel groups outboard (at the nominal pose: the decisions use world bounding boxes)."""
    doc()
    M = load()
    M['poses'].reset()
    return turn_wheels(M)


HARNESS_PARTS = ('xt30_plug', 'jst_ph3_plug', 'xt60_pair', 'xt90_xt60_adapter', 'rocker_switch')


def harness():
    """v4 assembly.build_harness with harness_v5's cables (re-runnable: the previous harness and its plugs go first).
    Returns the cables whose loops came out stretched."""
    doc()
    M = load()
    C, A, V, HV = M['cadlib'], M['assembly'], M['vendor'], M['harness_v5']
    masters = {k: getattr(V, k)() for k in HARNESS_PARTS}
    names = {c.name for c in masters.values()}
    for o in list(C.design().rootComponent.allOccurrences):
        ctx = o.assemblyContext
        if ctx is None or ctx.component.name == '_library':
            continue
        if o.component.name.startswith('harness_') or o.component.name in names:
            o.deleteMe()
    parts = {'xt30': masters['xt30_plug'], 'ph3': masters['jst_ph3_plug'], 'xt60': masters['xt60_pair'],
             'xt90': masters['xt90_xt60_adapter'], 'switch': masters['rocker_switch']}
    L = A.links()
    B = C.Builder()
    stretched = []
    tc, xt60, xt90 = HV.torso_cables()
    for cab in HV.cables(1) + HV.cables(-1) + tc:
        for i, rn in enumerate(cab['runs']):
            if rn[0] == 'loop':
                _, la, a, ta, lb, b, tb, length = rn
                pts, ok = HV.loop_points(a, ta, b, tb, length)
                if not ok:
                    stretched.append(cab['name'])
                A._add_wires(L[la], 'harness_%s_loop' % cab['name'], HV.wires_along(B, pts, HV.BUNDLE, HV.Y_SIDE),
                             HV.BUNDLE_LOOKS)
            else:
                link, pts, wires, looks, pref = rn
                A._add_wires(L[link], 'harness_%s_%d' % (cab['name'], i), HV.wires_along(B, HV.smooth(pts), wires, pref),
                             looks)
        for (kind, f, link) in cab['plugs']:
            A.place(L[link], parts[kind], f)
    P = M['params']
    A.place(L['torso'], parts['xt60'], C.frame(C.add(xt60, (0, 0, -16.0)), (1, 0, 0), (0, 1, 0), (0, 0, 1)))
    A.place(L['torso'], parts['xt90'], C.frame(xt90, (1, 0, 0), (0, 0, -1), (0, 1, 0)))
    A.place(L['torso'], parts['switch'], C.frame((-P.TORSO_X, P.SWITCH_Y, P.Z_HIP + P.SWITCH_Z), (0, 1, 0), (0, 0, -1),
                                                 (-1, 0, 0)))
    return dict(stretched=stretched)


GRID = dict(L0s=(200.0, 240.0, 280.0, 310.0, 335.0), hips=(-80.0, -60.0, -30.0, 0.0, 30.0, 60.0))


def chords():
    """(min, max) anchor distance of every loop over GRID ((distance, L0, hip))."""
    doc()
    M = load()
    return M['poses'].chord_table(GRID['L0s'], GRID['hips'])


def v5_occs(C):
    out = []
    for o in C.design().rootComponent.allOccurrences:
        n = o.component.name
        if n.startswith(V5_PARTS) and n.endswith(('_L', '_R')) and o.assemblyContext is not None:
            out.append(o)
    return out


def report():
    """(component, role, volume cm3, mass g, lumps, shells) of the v5 leg parts."""
    doc()
    M = load()
    rows = []
    for o in v5_occs(M['cadlib']):
        for b in o.component.bRepBodies:
            rows.append((o.component.name, b.name.split(' ')[-1], round(b.volume, 1),
                         round(b.physicalProperties.mass * 1000, 1), b.lumps.count, b.shells.count))
    return rows


def rib_check(step_y=3.0):
    """Sample every Warren rib's centre line (1 mm along it, every step_y in y across the cavity) in freshly built
    left links: a point inside the outer shape but outside the part is a gap (the ribs have no holes, so any gap is a
    fault).  Returns per link: lumps, shells (1 = every cell reaches an escape hole), voids, rib ends not joined to a
    wall, gaps."""
    M = load()
    C, LV = M['cadlib'], M['legs_v5']
    B = C.Builder(side=1)
    IN = (adsk.fusion.PointContainment.PointInsidePointContainment,
          adsk.fusion.PointContainment.PointOnPointContainment)
    res = {}
    for link in ('thigh', 'tibia'):
        g = LV.geo(link)
        if link == 'thigh':
            body, outer = LV.thigh(1)['main'], LV.thigh_shape(B)
            y0, y1 = LV.Y_TF + LV.WALL + 0.5, LV.SPAR[1] - LV.WALL - 0.5
        else:
            body, outer = LV.shank(1)['main'], LV.shank_shape(B)
            y0, y1 = LV.Y_SI + LV.WALL + 0.5, LV.Y_SH - LV.WALL - 0.5
        nys = int((y1 - y0) / step_y)
        ys = [round(y0 + (y1 - y0) * j / nys, 2) for j in range(nys + 1)]
        voids = [sh for sh in body.shells if sh.isVoid]
        loose, other = [], []
        nodes = LV.rib_nodes(link)
        for i, ((sa, ta), (sb, tb)) in enumerate(zip(nodes, nodes[1:])):
            L = math.hypot(sb - sa, tb - ta)
            m = int(L) + 1
            for y in ys:
                pts = []
                for kk in range(m + 1):
                    tau = kk / m
                    s_, t_ = sa + (sb - sa) * tau, ta + (tb - ta) * tau
                    p = LV.at(g, s_, t_)
                    P3 = B.P((p[0], y, p[1]))
                    pts.append((s_, t_, outer.pointContainment(P3) in IN, body.pointContainment(P3) in IN))
                for e in (pts[0], pts[-1]):
                    if e[2] and not e[3]:
                        loose.append((i, y, round(e[0], 1)))
                for s_, t_, ino, sol in pts:
                    if ino and not sol:
                        other.append((i, y, round(s_, 1), round(t_, 1)))
        res[link] = dict(lumps=body.lumps.count, shells=body.shells.count, voids=len(voids), y_levels=len(ys),
                         loose_rib_ends=loose, gaps=other[:20], n_gaps=len(other),
                         escapes=sum(len(f) for _s, _d, f in LV.cell_escapes(link)))
    return res


def interference(poses=POSES, loops=True):
    """Per pose: (L0, hip, mating, real, real pairs, loops {cable: (min distance mm, nearest part, not stretched,
    anchor distance)}).  The nominal loop bodies ride on one link, so they are left out of the interference and each
    pose's loops are built and measured separately (poses.measure_loops)."""
    doc()
    M = load()
    K, PO, C = M['check'], M['poses'], M['cadlib']
    short = lambda n: n.split('/')[-1][:38]
    root = C.design().rootComponent
    nominal = [o for o in root.allOccurrences if o.component.name.endswith('_loop')]
    out = []
    try:
        for o in nominal:
            o.isLightBulbOn = False
        for L0, hip in poses:
            T = PO.transforms(L0, hip)
            PO.apply(T)
            leaves = [o for o in K.leaves() if not o.component.name.endswith('_loop')]
            mating, real = K.classify(K.interference(leaves))
            lps = {}
            if loops:
                lps = {k: (round(v[0], 1), v[1].split('/')[-1][:30], v[2], round(v[3], 1))
                       for k, v in PO.measure_loops(T, leaves).items()}
            out.append((L0, hip, len(mating), len(real), sorted({(short(a), short(b)) for a, b, v in real})[:12], lps))
    finally:
        PO.reset()
        for o in nominal:
            o.isLightBulbOn = True
    return out


def show(torso=True, right=True):
    import cadlib as C
    root = C.design().rootComponent
    for o in root.occurrences:
        n = o.component.name
        o.isLightBulbOn = n != '_library' and (torso or n != 'torso') and (right or not n.endswith('_R'))
    for o in root.allOccurrences:
        if o.assemblyContext is not None and o.component.name != '_library' \
                and o.assemblyContext.component.name != '_library':
            o.isLightBulbOn = True
    root.isJointsFolderLightBulbOn = False


def snap(name, eye, target, fit=True):
    import view
    os.makedirs(IMG, exist_ok=True)
    adsk.core.Application.get().userInterface.activeSelections.clear()   # a face picked in the window shows up
    return view.snap(os.path.join(IMG, name + '.png'), eye, target, fit=fit)


def images():
    """Leg views (left leg alone), robot views and truss sections (left links cut above the rib holes: the ribs read
    as continuous bands, the holes show in the rib walls of the oblique views)."""
    doc()
    M = load()
    C, LV = M['cadlib'], M['legs_v5']
    ui = adsk.core.Application.get().userInterface
    lc = adsk.core.ListControlDefinition.cast(ui.commandDefinitions.itemById('ViewLayoutGridOnCommand').controlDefinition)
    lc.listItems.item(lc.listItems.count - 1).isSelected = True          # grid off
    out = []
    show(torso=False, right=False)
    snap('_warm', (760, 1050, 560), (40, 235, 240))
    adsk.doEvents()
    for name, eye, tgt in (('01_leg_iso_outboard', (760, 1050, 560), (40, 235, 240)),
                           ('02_leg_side_outboard', (40, 1600, 250), (40, 235, 250)),
                           ('03_leg_iso_inboard_front', (760, -320, 470), (40, 215, 240)),
                           ('04_leg_back', (-760, 640, 470), (20, 225, 240))):
        out.append(snap(name, eye, tgt))
    show()
    out.append(snap('05_robot_iso', (1500, 1150, 900), (0, 0, 290)))
    out.append(snap('06_robot_front', (1700, 0, 330), (0, 0, 290)))
    out.append(snap('07_robot_side', (0, 1900, 330), (0, 0, 290)))
    out.append(snap('11_robot_back_cables', (-1500, 900, 700), (0, 60, 300)))
    show(torso=False, right=False)
    out.append(snap('12_leg_cables_back_inboard', (-620, -260, 480), (10, 215, 230)))
    show()
    root = C.design().rootComponent
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = '_section'
    try:
        for o in root.occurrences:
            o.isLightBulbOn = o.component.name == '_section'
        B = C.Builder(side=1)
        for body, yc in ((LV.thigh(1)['main'], LV.Y_TO - LV.WALL - 1.5), (LV.shank(1)['main'], LV.Y_SO - LV.WALL - 2.5)):
            C.cut(body, B.abox(-400, 400, yc, 400, -400, 800))
            C.add_body(occ.component, body, 'section', None, 'white')
        snap('_warm', (40, 1600, 250), (40, 235, 250))
        adsk.doEvents()
        out.append(snap('08_section', (40, 1600, 250), (40, 235, 250)))
        out.append(snap('09_section_iso', (620, 1050, 780), (40, 205, 270)))
        out.append(snap('10_section_thigh_zoom', (330, 640, 520), (25, 195, 330), fit=False))
    finally:
        occ.deleteMe()
        show()
    w = os.path.join(IMG, '_warm.png')
    if os.path.exists(w):
        os.remove(w)
    return out


def robot_mass():
    """v4 workflow: PETG parts at their print density, tyre 721 g (the PA12 legs are modelled hollow: material
    density as is), then the mass report (run in a separate script from the build: stale physical properties)."""
    doc()
    M = load()
    show()
    M['check'].apply_print_mass()
    txt, data, _g = M['check'].report()
    return txt


def export():
    doc()
    M = load()
    show()
    for o in M['cadlib'].design().rootComponent.occurrences:
        if o.component.name == '_library':
            o.isLightBulbOn = False
    d = M['cadlib'].design()
    os.makedirs(EXP, exist_ok=True)
    f3d = os.path.join(EXP, 'WheelBiped_v5.f3d')
    em = d.exportManager
    em.execute(em.createFusionArchiveExportOptions(f3d))
    return f3d
