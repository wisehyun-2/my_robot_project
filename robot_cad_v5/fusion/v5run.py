"""robot_cad_v5 = the v4 robot (robot_cad_v4/fusion, git 681ad00) with the wheels turned outboard and MJF PA12 legs
(legs_v5.py, concept A "monolith", chosen by the user 11-01).  The torso, hip plates, motors, cups, wheels and
fasteners are built by the v4 code; this module only swaps the links.  No wiring in the model (user 11-01: all cables
removed; the external leg route that was tried is harness_v5.py in git 5a01c18); the back-panel rocker switch stays.

Run each step in its own MCP script (put this folder on sys.path, import v5run):
    v5run.build()            # makes a NEW document (attribute wbv5 / tag): v4 robot without harness, v4 links and
                             # output clamp rings removed, v5 legs added, back-panel switch
    v5run.turn()             # wheel groups outboard - last and in its own script: in the build script the new parts'
                             # bounding boxes are stale and the per-part decision goes wrong, and adding parts to a
                             # link (cadlib.IdentityParent) resets turned children (idempotent: a second run turns 0)
    v5run.relegs()           # instead of build(): in the opened hub file, only the leg parts rebuilt (then turn())
    v5run.shift()            # hub file only, once (11-02): the turned wheel groups 2.5 mm further out (legs_v5
                             # WHEEL_SHIFT; build() + turn() place them there directly) - before relegs()
    v5run.reshell()          # hub file: only the shell pieces rebuilt (rear plan corners rounded, user 11-02; then
                             # turn(): the deletion reverts the AKE60 stators)
    v5run.report()          # v5 leg parts: volume, mass, lumps / shells
    v5run.rib_check()        # every rib joined to its walls (point sampling), closed cells (voids)
    v5run.interference()     # 6 poses
    v5run.images()           # ../images (legs, robot, truss sections)
    v5run.robot_mass()       # v4 print densities (PETG parts) + tyre 721 g, robot mass / COM
    v5run.export()           # ../export/WheelBiped_v5.f3d
    v5run.exports()          # ../export/WheelBiped_v5_assembly.step, parts_step/*.step, parts_stl/*.stl (then
                             # ../manufacturing/part_drawings.py for the 2D drawings)
    v5run.sections()         # real joint sections -> ../manufacturing/explain/sections_v5.json.gz (then
                             # ../manufacturing/explain/real_sections.py for the section figures)
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
LOOK = {'main': 'black satin', 'insert': 'alu', 'ring': 'alu', 'bearing': 'steel', 'cap': 'dark grey'}
MAT = {'main': 'PETG print', 'insert': 'Al6061', 'ring': 'Al6061', 'bearing': 'Steel', 'cap': 'Al6061'}   # links PETG
                                                 # FDM since 11-02 (user; MJF PA12 before), printed solid (walls 3.5)
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
    sys.modules.pop('legs_v5', None)
    import legs_v5
    M['legs_v5'], M['v4run'] = legs_v5, v4run
    for k in ('params', 'cadlib', 'legs'):
        assert M[k].__file__.lower().startswith(V4.lower()), (k, M[k].__file__)
    assert legs_v5.__file__.lower().startswith(HERE.lower()), legs_v5.__file__
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
NOT_TURNED = ('shank_', 'kneebearing_')          # v5 parts: built in place
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
    M['v4run'].build(with_harness=False, rear_round=True)   # no wiring (user 11-01), round rear shell corners (11-02)
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
    _add_legs(C, LV)                               # (the back-panel rocker switch: v4run.build places it, 11-02)
    C.design().rootComponent.partNumber = 'WheelBiped_v5'
    return dict(dropped=dropped)


def _add_legs(C, LV):
    for s, k in SIDES:
        th, sh, kn = LV.thigh(s), LV.shank(s), LV.knee_parts(s)
        for link, name, parts in (('femur_', 'thigh_' + k, th), ('tibia_', 'shank_' + k, sh),
                                  ('femur_', 'kneecap_' + k, {'cap': kn['cap']}),
                                  ('tibia_', 'kneebearing_' + k, {'bearing': kn['bearing']})):
            comp = C.new_part(C.root_occ(link + k), name)
            for role, body in parts.items():
                C.add_body(comp, body, '%s %s' % (name, role), MAT[role], LOOK[role])


def relegs():
    """Rebuild only the v5 leg parts (thigh, shank, knee cap, 61807 of both sides) from legs_v5 in the ACTIVE document
    - e.g. the hub file WheelBiped_v5, so a leg change becomes a new version of it: the old components are deleted and
    new ones added at the nominal pose.  Run turn() afterwards in its own script (the deletion makes the AKE60 stators
    revert, adding parts to a link resets turned children)."""
    d = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    if not d.attributes.itemByName(TAG[0], TAG[1]):
        d.attributes.add(*TAG)
    M = load()
    C, LV = M['cadlib'], M['legs_v5']
    M['poses'].reset()
    M['finalize'].show_all()
    names = tuple('%s_%s' % (p, k) for p in V5_PARTS for _s, k in SIDES)
    dropped = []
    for _s, k in SIDES:
        for link in ('femur_', 'tibia_'):
            for o in list(C.root_occ(link + k).childOccurrences):
                if o.component.name in names:
                    dropped.append(o.component.name)
                    o.deleteMe()
    _add_legs(C, LV)
    return dict(dropped=dropped)


SHELL_PARTS = ('shell_', 'saddle_', 'splice_tabs')


def reshell():
    """Rebuild only the torso shell pieces (front / mid / rear sections, saddles, splice tabs) in the v5 document - the
    hub file - with the rear plan corners rounded (shell.REAR_ROUND, user 11-02: v4 keeps its 45 deg chamfers).  The
    shell screws, grommets and nuts stay (nothing they sit on moves).  Run turn() afterwards in its own script."""
    doc()
    M = load()
    C, A = M['cadlib'], M['assembly']
    M['poses'].reset()
    M['finalize'].show_all()
    M['shell'].REAR_ROUND = True
    dropped = []
    for o in list(C.root_occ('torso').childOccurrences):
        if o.component.name.startswith(SHELL_PARTS):
            dropped.append(o.component.name)
            o.deleteMe()
    A.build_shell()
    return dict(dropped=sorted(dropped))


def turn():
    """Wheel groups outboard (at the nominal pose: the decisions use world bounding boxes)."""
    doc()
    M = load()
    M['poses'].reset()
    return turn_wheels(M)


def wheel_group(C, P, k):
    """(occurrence, in_wheel) of the turned wheel group of side k: every wheel_k child and the tibia parts near the
    axle except the v5 links (the set turn_wheels works on)."""
    tib, whl = C.root_occ('tibia_' + k), C.root_occ('wheel_' + k)
    out = [(o, True) for o in whl.childOccurrences]
    for o in tib.childOccurrences:
        c = _centre(o)
        if not o.component.name.startswith(NOT_TURNED) and math.hypot(c[0], c[2] - P.R_WHEEL) < 70.0:
            out.append((o, False))
    return out


def shift(dy=None, stator_from=216.7):
    """Hub file migration (11-02): move the turned wheel groups of an older version dy further outboard (default
    legs_v5.WHEEL_SHIFT) - the turn plane moved from y 220.5 to Y_MSH.  Refuses unless the wheel stators still sit
    at +-stator_from (so it runs once); the AKE60 stator is re-instanced (REPLACE).  Then relegs(), then turn() (0)."""
    doc()
    M = load()
    C, P, LV = M['cadlib'], M['params'], M['legs_v5']
    M['poses'].reset()
    dy = LV.WHEEL_SHIFT if dy is None else dy
    moved = []
    for s, k in SIDES:
        tib = C.root_occ('tibia_' + k)
        st = [o for o in tib.childOccurrences if o.component.name.startswith('AKE60-8_KV80_stator')
              and math.hypot(_centre(o)[0], _centre(o)[2] - P.R_WHEEL) < 70.0]
        assert len(st) == 1 and abs(abs(_centre(st[0])[1]) - stator_from) < 0.3,             ('wheel stator not at the old place', k, [round(_centre(o)[1], 2) for o in st])
        T = C.to_matrix3d(C.robot_to_fusion(C.translate((0.0, s * dy, 0.0))))
        whl = C.root_occ('wheel_' + k)
        for o, in_wheel in wheel_group(C, P, k):
            m = o.transform2.copy()
            m.transformBy(T)
            if o.component.name.startswith(REPLACE):
                C.instance(whl if in_wheel else tib, o.component, m)
                o.deleteMe()
            else:
                o.transform2 = m
            moved.append(o.component.name if o.isValid else 'stator')
    return len(moved)


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
    fault).  Returns per link: lumps, shells (1 + the closed truss cells, 11-02: no escape holes), voids, rib ends
    not joined to a wall, gaps."""
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
                         loose_rib_ends=loose, gaps=other[:20], n_gaps=len(other))
    return res


def interference(poses=POSES):
    """Per pose: (L0, hip, mating, real, real pairs)."""
    doc()
    M = load()
    K, PO = M['check'], M['poses']
    short = lambda n: n.split('/')[-1][:38]
    out = []
    try:
        for L0, hip in poses:
            PO.apply(PO.transforms(L0, hip))
            mating, real = K.classify(K.interference())
            out.append((L0, hip, len(mating), len(real), sorted({(short(a), short(b)) for a, b, v in real})[:12]))
    finally:
        PO.reset()
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
    """Leg views (left leg alone), robot views and truss sections (left links cut at y 199 / 229.5, looking from
    outboard: the plain ribs and the closed cells)."""
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


LEG_FILE = {'main': None, 'insert': 'hub_insert_', 'ring': 'hub_ring_', 'cap': 'knee_cap'}


def exports():
    """STEP / STL of the v5 model: export/WheelBiped_v5_assembly.step (whole robot) and one file per designed part in
    export/parts_step + parts_stl - the v4 designed parts (finalize.design_parts, component STEP as v4) and every body
    of the v5 leg components on its own (thigh_L -> thigh_L + hub_insert_hip_L, shank_L -> shank_L +
    hub_insert_knee_L, kneecap_L -> knee_cap_L; the 61807 is bought), in robot coordinates (Fusion world, nominal
    pose).  The single bodies go through a scratch document closed unsaved, so the robot document is not changed."""
    doc()
    M = load()
    C, F = M['cadlib'], M['finalize']
    M['poses'].reset()
    show()
    for o in C.design().rootComponent.occurrences:
        if o.component.name == '_library':
            o.isLightBulbOn = False
    d = C.design()
    em = d.exportManager
    ps, pl = os.path.join(EXP, 'parts_step'), os.path.join(EXP, 'parts_stl')
    os.makedirs(ps, exist_ok=True)
    os.makedirs(pl, exist_ok=True)

    def stl(body, fn):
        o = em.createSTLExportOptions(body, os.path.join(pl, fn + '.stl'))
        o.isBinaryFormat = True
        o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
        em.execute(o)
        done.append(fn + '.stl')

    done = []
    em.execute(em.createSTEPExportOptions(os.path.join(EXP, 'WheelBiped_v5_assembly.step'), d.rootComponent))
    done.append('WheelBiped_v5_assembly.step')
    for name in sorted(F.design_parts()):
        if name.startswith(V5_PARTS):
            continue
        comp = C.find_comp(name)
        safe = F._file_name(name)
        em.execute(em.createSTEPExportOptions(os.path.join(ps, safe + '.step'), comp))
        done.append(safe + '.step')
        solids = [b for b in comp.bRepBodies if b.isSolid]
        for b in solids:
            stl(b, safe if len(solids) == 1 else safe + '__' + F._file_name(b.name))
    singles = []
    for o in v5_occs(C):
        link, k = o.component.name.rsplit('_', 1)
        if link == 'kneebearing':
            continue
        for b in o.component.bRepBodies:
            role = b.name.split(' ')[-1]
            base = LEG_FILE[role] or link
            if role in ('insert', 'ring'):
                base += 'hip' if link == 'thigh' else 'knee'
            fn = '%s_%s' % (base, k)
            stl(b, fn)
            singles.append((fn, C.TBM.copy(b)))
    app = adsk.core.Application.get()
    main = app.activeDocument
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    tmp = app.activeDocument
    try:
        td = adsk.fusion.Design.cast(app.activeProduct)
        td.designType = adsk.fusion.DesignTypes.DirectDesignType
        root = td.rootComponent
        for fn, tb in singles:
            b = root.bRepBodies.add(tb)
            b.name = fn
            td.exportManager.execute(td.exportManager.createSTEPExportOptions(os.path.join(ps, fn + '.step'), root))
            b.deleteMe()
            done.append(fn + '.step')
    finally:
        tmp.close(False)
        main.activate()
    return done


SECTION_JOINTS = {'hip': (0.0, 'Z_HIP', (110.0, 300.0)), 'knee': ('KNEE_X', 'KNEE_Z', (130.0, 300.0)),
                  'wheel': (0.0, 'R_WHEEL', (140.0, 300.0))}


def sections(path=None, angles=None, r_max=95.0):
    """Real 2D sections of the left leg (nominal pose): every visible body near the hip / knee / wheel axis cut by
    planes through that axis (angle a in deg from robot +x towards +z; the plane holds the axis and that direction).
    Writes JSON records (joint, angle, component, occurrence path, body, edge polylines in (robot y, t) with t the
    distance from the axis along the plane, solid) for ../manufacturing/explain/real_sections.py (default path:
    ../manufacturing/explain/sections_v5.json.gz).  The plane intersections are wires: real_sections chains them."""
    import gzip
    import json
    doc()
    M = load()
    P = M['params']
    M['poses'].reset()
    path = path or os.path.join(BASE, 'manufacturing', 'explain', 'sections_v5.json.gz')
    angles = list(range(0, 180, 5)) if angles is None else angles
    design = M['cadlib'].design()
    tbm = adsk.fusion.TemporaryBRepManager.get()
    bodies = []
    for o in design.rootComponent.allOccurrences:
        if '_library' in o.fullPathName or not o.isVisible:
            continue
        for b in o.component.bRepBodies:
            p = b.createForAssemblyContext(o)
            if p.isVisible:
                lo, hi = p.boundingBox.minPoint, p.boundingBox.maxPoint
                bodies.append((o, b, p, (lo.z * 10, hi.z * 10), (lo.x * 10, hi.x * 10), (lo.y * 10, hi.y * 10)))
    out = []
    for jn, (xk, zk, (y0, y1)) in SECTION_JOINTS.items():
        xc = getattr(P, xk) if isinstance(xk, str) else xk
        zc = getattr(P, zk)
        cand = [(o, b, tbm.copy(p)) for o, b, p, X, Y, Z in bodies
                if Y[1] >= y0 and Y[0] <= y1
                and math.hypot(max(X[0] - xc, 0.0, xc - X[1]), max(Z[0] - zc, 0.0, zc - Z[1])) <= r_max]
        for a in angles:
            ca, sa = math.cos(math.radians(a)), math.sin(math.radians(a))
            plane = adsk.core.Plane.create(adsk.core.Point3D.create((y0 + y1) / 20.0, zc / 10.0, xc / 10.0),
                                           adsk.core.Vector3D.create(0.0, ca, -sa))
            for o, b, tb in cand:
                try:
                    res = tbm.planeIntersection(tb, plane)
                except Exception:
                    res = None
                if res is None or res.edges.count == 0:
                    continue
                lines = []
                for e in res.edges:
                    ev = e.evaluator or (e.geometry.evaluator if e.geometry is not None else None)
                    if ev is None:
                        continue
                    ok, p0, p1 = ev.getParameterExtents()
                    ok, st = ev.getStrokes(p0, p1, 0.0015)
                    lines.append([[round(q.x * 10, 4), round((q.z * 10 - xc) * ca + (q.y * 10 - zc) * sa, 4)] for q in st])
                out.append(dict(joint=jn, angle=a, comp=o.component.name, path=o.fullPathName, body=b.name,
                                lines=lines, solid=b.isSolid))
    with (gzip.open(path, 'wt', encoding='utf-8') if path.endswith('.gz') else open(path, 'w', encoding='utf-8')) as fh:
        json.dump(out, fh)
    return path, len(out)
