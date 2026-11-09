"""robot_cad_v5 = the v4 robot (robot_cad_v4/fusion, git 681ad00) with the wheels turned outboard and MJF PA12 legs
(legs_v5.py, concept A "monolith", chosen by the user 11-01).  The torso, hip plates, motors, cups, wheels and
fasteners are built by the v4 code; this module only swaps the links.  No wiring in the model (user 11-01: all cables
removed; the external leg route that was tried is harness_v5.py in git 5a01c18); the rocker switch stays.
The robot's front is the rocker switch side = CAD -x, so its left is CAD -y: see "part names" below (v4 names it the other way).

Run each step in its own MCP script (put this folder on sys.path, import v5run):
    v5run.build()            # makes a NEW document (attribute wbv5 / tag): v4 robot without harness, v4 links and
                             # output clamp rings removed, parts named from the switch side, v5 legs added
    v5run.rename_to_robot()  # hub file, once (11-08): an older version's part and joint names turned to the switch side
    v5run.turn()             # wheel groups outboard - last and in its own script: in the build script the new parts'
                             # bounding boxes are stale and the per-part decision goes wrong, and adding parts to a
                             # link (cadlib.IdentityParent) resets turned children (idempotent: a second run turns 0)
    v5run.relegs()           # instead of build(): in the opened hub file, only the leg parts rebuilt (then turn())
    v5run.shift()            # hub file only, once (11-02): the turned wheel groups 2.5 mm further out (legs_v5
                             # WHEEL_SHIFT; build() + turn() place them there directly) - before relegs()
    v5run.reshell()          # hub file: only the shell pieces rebuilt (plan corners on the switch side rounded, user 11-02; then
                             # turn(): the deletion reverts the AKE60 stators)
    v5run.relayout()         # hub file: deck rebuilt, Pi / pi3hat / power board / fan placed with the IMU on the centre line (11-07; then turn());
                             # 11-08: the power board is the power_dist r4.5b (pdist_v5.py), an mjpower-ss is swapped for it
    v5run.resplit()          # hub file (11-09): battery tray / electronics deck in two pieces each, bolted flanges (split_v5.py;
                             # unjoint() first, then turn(), joints(); build() and relayout() make the pieces directly)
    v5run.relink()           # hub file, once (11-07): links 175 -> 250 mm, every part moves with its joint (then relegs(), turn())
    v5run.deepen_hub_dowels()  # hub file, once (11-08): wheel hub flange dowel holes 7.0 -> 7.5 and the pins with them
                             # (unjoint() first, then turn(), joints(); build() makes them so from HUB_DOWEL_HOLE)
    v5run.joints()           # last step of a model change: 6 as-built revolute joints (the design becomes parametric;
                             # torso grounded, every nested part grounded to its parent). Rebuild steps above and
                             # images() refuse a jointed design: v5run.unjoint() first (drops joints and timeline)
    v5run.joint_check()      # every joint turned and back: the parts below it move as one rigid set about its axis
    v5run.pose(L0, hip)      # a pose (with joints: joint values; Fusion rounds them to 0.1 deg), pose() = nominal
    v5run.report()          # v5 leg parts: volume, mass, lumps / shells
    v5run.rib_check()        # every rib joined to its walls (point sampling), closed cells (voids)
    v5run.interference()     # 6 poses
    v5run.images()           # ../images (legs, robot, truss sections)
    v5run.robot_mass()       # v4 print densities (PETG parts) + tyre 721 g, robot mass / COM
    v5run.urdf_export()      # ../calc/urdf_data.json + ../export/links_stl (one STL per link) for wbr.models.build_v5 (URDF)
    v5run.export()           # ../export/WheelBiped_v5.f3d
    v5run.exports()          # ../export/WheelBiped_v5_assembly.step, parts_step/*.step, parts_stl/*.stl (then
                             # ../manufacturing/part_drawings.py for the 2D drawings)
    v5run.sections()         # real joint sections -> ../manufacturing/explain/sections_v5.json.gz (then
                             # ../manufacturing/explain/real_sections.py for the section figures)
"""
import importlib
import math
import os
import re
import sys

import adsk.core
import adsk.fusion

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import link_length as LL  # noqa: E402
sys.modules.pop('v5_params', None)               # Fusion keeps modules between scripts: always this folder's current file
import v5_params as VP  # noqa: E402
assert VP.__file__.lower().startswith(HERE.lower()), VP.__file__

BASE = os.path.dirname(HERE)
V4 = os.path.join(os.path.dirname(BASE), 'robot_cad_v4', 'fusion')
IMG = os.path.join(BASE, 'images')
EXP = os.path.join(BASE, 'export')
TAG = ('wbv5', 'tag', 'wbv5_robot')
NAMES_TAG = ('wbv5', 'names', 'switch_side_front')   # on a design whose parts are named from the switch side
SIDES = ((1, 'R'), (-1, 'L'))                        # (sign of CAD y, part suffix): the front is CAD -x, so +y is the robot's right
LOOK ={'main': 'black satin', 'insert': 'alu', 'ring': 'alu', 'bearing': 'steel', 'cap': 'dark grey'}
MAT = {'main': 'PETG print', 'insert': 'Al6061', 'ring': 'Al6061', 'bearing': 'Steel', 'cap': 'Al6061'}   # links PETG
                                                 # FDM since 11-02 (user; MJF PA12 before), printed solid (walls 3.5)
V5_PARTS = ('thigh', 'shank', 'kneecap', 'kneebearing')
HUB_DOWEL_HOLE = VP.HUB_DOWEL_HOLE                # wheel hub flange dowel holes (v5_params, 11-08 user)
POSES = tuple((L0 * LL.SCALE, hip) for L0, hip in ((310, 0), (200, 0), (240, -60), (240, 60), (335, 0), (200, -80)))    # (L0 mm, hip deg):
                                                 # the poses of the 175 mm link at the same knee angles


def _v4_loader(v4run):
    """v4run.load with the v5 link length: same modules and order, params lengthened before the modules that read them at import.  The
    v4 files stay as they are, and v4run is dropped from sys.modules afterwards, so a v4 script in this Fusion session gets a plain one."""
    def loader(names=None):
        if v4run.PATH not in sys.path:
            sys.path.insert(0, v4run.PATH)
        out = {}
        for n in v4run.MODS:
            m = sys.modules.get(n)
            if m is not None and v4run.PATH.lower() not in (getattr(m, '__file__', '') or '').lower():
                del sys.modules[n]
        for n in v4run.MODS:
            if os.path.exists(os.path.join(v4run.PATH, n + '.py')):
                out[n] = importlib.reload(importlib.import_module(n))
                if n == 'params':
                    LL.lengthen(out[n])
                    VP.apply(out[n])
        return out
    return loader


def load():
    """v4 modules (v4run.load, v5 link length) + legs_v5; this folder and the v4 one first on sys.path, files checked."""
    for p in (HERE, V4):
        while p in sys.path:
            sys.path.remove(p)
    sys.path.insert(0, V4)
    sys.path.insert(0, HERE)
    sys.modules.pop('v4run', None)
    import v4run
    sys.modules.pop('v4run', None)
    v4run.load = _v4_loader(v4run)
    M = v4run.load()
    sys.modules.pop('legs_v5', None)
    import legs_v5
    M['legs_v5'], M['v4run'] = legs_v5, v4run
    for k in ('params', 'cadlib', 'legs'):
        assert M[k].__file__.lower().startswith(V4.lower()), (k, M[k].__file__)
    assert legs_v5.__file__.lower().startswith(HERE.lower()), legs_v5.__file__
    _pa12(M['cadlib'])
    M['torso'].centre_imu()                        # v5's electronics layout (11-07); v4 keeps the Pi on the -y side
    sys.modules.pop('pdist_v5', None)
    import pdist_v5
    assert pdist_v5.__file__.lower().startswith(HERE.lower()), pdist_v5.__file__
    pdist_v5.patch(M)                              # power_dist r4.5b instead of the mjpower-ss (11-08 user)
    M['pdist_v5'] = pdist_v5
    sys.modules.pop('split_v5', None)
    import split_v5
    assert split_v5.__file__.lower().startswith(HERE.lower()), split_v5.__file__
    M['split_v5'] = split_v5                       # battery tray / deck in two pieces each (11-09 user)
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


# ----------------------------------------------------------------------------- part names
# The v4 code names parts as if +x were the front: +y = _L, +x = shell_front, rail_F, spine_tube_F.  This robot's front is the rocker switch
# side = -x (user 11-07), so its left is -y, and every name that says a side or an end says the opposite one in the v4 code (user 11-08).  The
# v4 files stay as they are: what they build is renamed here.  CAD coordinates do not change, only names.
_OTHER = {'L': 'R', 'R': 'L', 'F': 'B', 'B': 'F', 'front': 'rear', 'rear': 'front'}


def robot_name(v4_name):
    """The name in this robot of a part, body or joint the v4 code calls v4_name: shell_front_L -> shell_rear_R, femur_L -> femur_R,
    'thigh_L main' -> 'thigh_R main', rail_F -> rail_B.  Names that say neither stay."""
    n = re.sub(r'_([LR])(?=$|[ _])', lambda m: '_' + _OTHER[m.group(1)], v4_name)
    n = re.sub(r'^shell_(front|rear)(?=_)', lambda m: 'shell_' + _OTHER[m.group(1)], n)
    return re.sub(r'^(rail|spine_tube)_([FB])(?=$| )', lambda m: m.group(1) + '_' + _OTHER[m.group(2)], n)


def _rename(pairs):
    """[(entity, new name)] through temporary names: the new name of one is usually the old name of another."""
    pairs = [(e, n) for e, n in pairs if e.name != n]
    for i, (e, _n) in enumerate(pairs):
        e.name = '~rename%d' % i
    for e, n in pairs:
        e.name = n
    return len(pairs)


def _name_parts(comps):
    """Components just built by the v4 code, and their bodies, get this robot's names."""
    n = _rename([(c, robot_name(c.name)) for c in comps])
    for c in comps:
        _rename([(b, robot_name(b.name)) for b in c.bRepBodies])
    return n


def rename_to_robot():
    """The whole document, once: components, bodies and joints named from the switch side.  Nothing moves.  Does nothing on a design that has
    it (NAMES_TAG), so a second run cannot swap them back.  Returns the number of components renamed."""
    d = adsk.fusion.Design.cast(doc().products.itemByProductType('DesignProductType'))
    if d.attributes.itemByName(NAMES_TAG[0], NAMES_TAG[1]):
        return 0
    n = _name_parts(list(d.allComponents))
    _rename([(j, robot_name(j.name)) for j in d.rootComponent.asBuiltJoints])
    d.attributes.add(*NAMES_TAG)
    return n


def _need_names(C):
    """SIDES pairs a suffix with a CAD side, so an older version (hub 10 or earlier, its f3d) would get the geometry of the other leg."""
    if not C.design().attributes.itemByName(NAMES_TAG[0], NAMES_TAG[1]):
        raise RuntimeError('parts still have the v4 names: run v5run.rename_to_robot() first')


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
    (wheel_clock), and since 11-09 the motor stator, clamp ring and its screws too (legs_v5.CLOCK_MOTOR).  Decided per occurrence, so it is idempotent even when cadlib.IdentityParent has reset some child
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
            if not in_wheel and (LV.CLOCK_MOTOR or not name.startswith(STATOR)):   # 11-09: the motor with the cup
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
    M['v4run'].build(with_harness=False, rear_round=True, centre_imu=True)   # no wiring (user 11-01), round shell corners on the switch side
                                                   # (11-02), the IMU on the centre line (11-07)
    C, LV = M['cadlib'], M['legs_v5']             # (v4run.build reloaded the v4 modules in place)
    _pa12(C)
    M['finalize'].show_all()
    rename_to_robot()                              # before SIDES is used on the v4 parts
    Ms = load()                                    # fresh modules (v4run.build reloaded them without the v5 patches)
    Ms['split_v5'].apply(Ms)                       # tray / deck in two pieces (11-09 user)
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
    _add_legs(C, LV)                               # (the rocker switch: v4run.build places it, 11-02)
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
    _need_direct(C, 'relegs')
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


def relink():
    """Hub file migration, once (user 11-07): the links 175 -> 250 mm.  Every part keeps its place on its joint and only moves in a straight
    line with that joint: the torso and the hip-end parts of the thigh go up by 2 dL cos PHI, the knee-end parts of the thigh and the shank
    move dL along the shank, the wheel end and the wheels stay where they are.  No part near a joint is rebuilt or turned here.  Every moved
    part is re-instanced at its new place: a transform2 set on a child is lost as soon as its link's own transform is set (IdentityParent,
    poses.reset; measured 11-07: nearly all moved parts went back).  Refuses unless the knee motors are at the 175 mm place.
    Route, each in its own script: unjoint(), relink(), relegs() (the printed links), turn(), joints()."""
    doc()
    M = load()
    C, P, LV = M['cadlib'], M['params'], M['legs_v5']
    _need_direct(C, 'relink')
    M['poses'].reset()
    M['finalize'].show_all()                       # (new instances copy their master's bulb)
    dL = LV.STRETCH
    sn, cs = math.sin(P.PHI), math.cos(P.PHI)
    d_hip, d_knee = (0.0, 0.0, 2.0 * dL * cs), (dL * sn, 0.0, dL * cs)
    H0 = (0.0, P.Z_HIP - d_hip[2])
    K0 = (P.KNEE_X - d_knee[0], P.KNEE_Z - d_knee[2])
    W0 = (0.0, P.R_WHEEL)

    def dist(c, q):
        return math.hypot(c[0] - q[0], c[2] - q[1])

    def move(o, parent, d):
        T = C.to_matrix3d(C.robot_to_fusion(C.translate(d)))
        m = o.transform2.copy()
        m.transformBy(T)
        C.instance(parent, o.component, m)
        o.deleteMe()

    for s, k in SIDES:
        st = [o for o in C.root_occ('femur_' + k).childOccurrences if o.component.name.startswith(REPLACE[0])]
        assert len(st) == 1 and dist(_centre(st[0]), K0) < 30.0, ('knee motor not at the 175 mm place', k, [_centre(o) for o in st])
    count = dict(torso=0, hip_end=0, knee_end=0, wheel_end=0)
    far = []
    t = C.root_occ('torso')
    for o in list(t.childOccurrences):
        move(o, t, d_hip)
        count['torso'] += 1
    for s, k in SIDES:
        fem, tib = C.root_occ('femur_' + k), C.root_occ('tibia_' + k)
        for o in list(fem.childOccurrences):
            if o.component.name.startswith(V5_PARTS):
                continue
            c = _centre(o)
            dh, dk = dist(c, H0), dist(c, K0)
            far.append((round(min(dh, dk), 1), o.component.name))
            move(o, fem, d_hip if dh < dk else d_knee)
            count['hip_end' if dh < dk else 'knee_end'] += 1
        for o in list(tib.childOccurrences):
            if o.component.name.startswith(V5_PARTS):
                continue
            c = _centre(o)
            dk, dw = dist(c, K0), dist(c, W0)
            far.append((round(min(dk, dw), 1), o.component.name))
            if dk < dw:
                move(o, tib, d_knee)
                count['knee_end'] += 1
            else:
                count['wheel_end'] += 1
    return dict(count=count, d_hip=d_hip, d_knee=d_knee, farthest_from_a_joint=sorted(far, reverse=True)[:6])


SHELL_PARTS = ('shell_', 'saddle_', 'splice_tabs')


def reshell():
    """Rebuild only the torso shell pieces (front / mid / rear sections, saddles, splice tabs) in the v5 document - the
    hub file - with the plan corners on the switch side rounded (shell.REAR_ROUND, the v4 code's rear, user 11-02: v4 keeps its 45 deg
    chamfers).  The shell screws, grommets and nuts stay (nothing they sit on moves).  Run turn() afterwards in its own script."""
    doc()
    M = load()
    C, A = M['cadlib'], M['assembly']
    _need_direct(C, 'reshell')
    M['poses'].reset()
    M['finalize'].show_all()
    M['shell'].REAR_ROUND = True
    dropped = []
    for o in list(C.root_occ('torso').childOccurrences):
        if o.component.name.startswith(SHELL_PARTS):
            dropped.append(o.component.name)
            o.deleteMe()
    A.build_shell()
    _name_parts([o.component for o in C.root_occ('torso').childOccurrences if o.component.name.startswith(SHELL_PARTS)])
    return dict(dropped=sorted(dropped))


BOARD = 'power_dist_r4.5b'                     # pdist_v5.NAME (11-08 user; mjpower-ss until hub version 14)
ELECTRONICS = ('electronics_deck', 'raspberry_pi_4b', 'pi3hat_r4.5', 'standoff M2.5x11 M-F (pi3hat)', BOARD, 'fan 30x30x7 5V',
               'FX insert M2.5x4 deck', 'FX DIN 7984 M2.5x6 mjpower', 'FX pi3hat M2.5x5')


def relayout():
    """Hub file: the deck rebuilt and the Pi, hat, power board, fan and their fasteners placed for torso.centre_imu() (11-07, user: the
    IMU on the robot's centre line).  11-08 (user): the power board is the power_dist r4.5b (pdist_v5); an mjpower-ss found in the torso
    is dropped and its _library master removed.  Direct design only: unjoint() first, then this, turn() in its own script, joints()."""
    doc()
    M = load()
    C, A, F, TO, PD = M['cadlib'], M['assembly'], M['fasteners'], M['torso'], M['pdist_v5']
    assert PD.NAME == BOARD
    _need_direct(C, 'relayout')
    M['poses'].reset()
    M['finalize'].show_all()
    t = C.root_occ('torso')
    SP = M['split_v5']
    deck = 'electronics_deck'
    dropped = SP.drop(t, (deck,))                          # the deck, one piece or two (11-09)
    SP.drop_fasteners(M, t, (deck,))
    masters = {deck: None}
    for o in list(t.childOccurrences):
        name = o.component.name
        if name in ELECTRONICS or name == PD.OLD_NAME:
            masters[name] = o.component
            dropped.append(name)
            o.deleteMe()
    old_board = masters.pop(PD.OLD_NAME, None) is not None
    if BOARD not in masters:
        masters[BOARD] = M['vendor'].mjpower_ss()          # patched: the power_dist master
    assert sorted(masters) == sorted(ELECTRONICS), sorted(set(ELECTRONICS) - set(masters))
    SP.add(M, t, (deck,))                                  # the deck in two pieces + its joint screws (11-09 user)
    A.place(t, masters['raspberry_pi_4b'], TO.rpi_frame())
    A.place(t, masters['pi3hat_r4.5'], TO.pi3hat_frame())
    pm = TO.rpi_frame()
    for (x, y) in ((3.5, 3.5), (61.5, 3.5), (3.5, 52.5), (61.5, 52.5)):
        A.place(t, masters['standoff M2.5x11 M-F (pi3hat)'], C.frame(C.apply(pm, (x, y, 1.4)), (1, 0, 0), (0, 1, 0), (0, 0, 1)))
    A.place(t, masters[BOARD], TO.mjpower_frame())
    A.place(t, masters['fan 30x30x7 5V'], TO.fan_frame())
    n = 0
    for rec in F.positions():
        link, name, o, ez = rec[:4]
        if name in masters:
            assert link == 'torso', rec
            C.instance(t, masters[name], C.part_matrix(F._frame(o, ez, rec[4] if len(rec) > 4 else None)))
            n += 1
    lib_dropped = PD.drop_old_master(C) if old_board else 0
    return dict(dropped=len(dropped), old_board=old_board, old_master_dropped=lib_dropped, fasteners=n, imu=imu_pose())


def resplit():
    """Hub file (11-09, user: the battery tray and the electronics deck printed in two pieces each and joined): the one-piece parts
    replaced by the split_v5 pieces and their joint screws / nuts (a second run rebuilds them).  Direct design only: unjoint() first,
    then this, turn() in its own script, joints()."""
    doc()
    M = load()
    _need_direct(M['cadlib'], 'resplit')
    M['poses'].reset()
    M['finalize'].show_all()
    return M['split_v5'].apply(M)


def turn():
    """Wheel groups outboard (at the nominal pose: the decisions use world bounding boxes)."""
    doc()
    M = load()
    _need_direct(M['cadlib'], 'turn')
    M['poses'].reset()
    return turn_wheels(M)


# ----------------------------------------------------------------------------- joints
AXIS_SKETCHES = ('hip_axis', 'knee_axis', 'wheel_axis')       # finalize._axis_circle, in the femur / tibia components


def _need_direct(C, what):
    """Bodies can only be added to a direct design here (cadlib.add_body has no base feature) and a transform2 change
    of a jointed link does not survive the next recompute."""
    _need_names(C)
    if C.design().designType != adsk.fusion.DesignTypes.DirectDesignType:
        raise RuntimeError('%s: the design has joints (parametric) - run v5run.unjoint() first, joints() again last' % what)


def _joint_map(C):
    return {j.name: j for j in C.design().rootComponent.asBuiltJoints}


def _drop_axis_sketches(C):
    for link in ('femur_', 'tibia_'):
        for _s, k in SIDES:
            sks = C.root_occ(link + k).component.sketches
            for sk in [sks.item(i) for i in range(sks.count) if sks.item(i).name in AXIS_SKETCHES]:
                sk.deleteMe()


def joints():
    """Six as-built revolute joints hip / knee / wheel _L / _R (finalize.joints: the design becomes parametric, torso
    grounded, every nested occurrence grounded to its parent).  Why the model needs this at all: femur_k, tibia_k and
    wheel_k are unconnected siblings at the root, so without joints moving one leaves the others behind.  Run it last,
    in its own script; does nothing when the joints exist.  Joint value 0 = the nominal pose."""
    doc()
    M = load()
    C = M['cadlib']
    _need_names(C)
    root = C.design().rootComponent
    if root.asBuiltJoints.count:
        return [j.name for j in root.asBuiltJoints]
    _drop_axis_sketches(C)
    names = M['finalize'].joints()
    root.isJointsFolderLightBulbOn = True          # (finalize hides them for its renders; here they are to be picked)
    return names


def unjoint():
    """Back to a direct design for a rebuild step: the joints and the timeline are dropped.  The switch also makes the
    turned wheel groups revert (126 parts, 11-07), so turn() must follow even with no rebuild.  Route: unjoint(), the
    rebuild, turn() in its own script, joints() (checked 11-07: back on every saved part position)."""
    doc()
    M = load()
    C = M['cadlib']
    _nominal(M)
    d = C.design()
    if d.designType != adsk.fusion.DesignTypes.DirectDesignType:
        d.designType = adsk.fusion.DesignTypes.DirectDesignType
    _drop_axis_sketches(C)
    return d.designType


def _pose(M, L0, hip, spin=0.0):
    """poses.transforms(L0, hip, spin) through the joints when the design has them: poses.apply on a jointed link does
    not touch the joint values, and a recompute puts the link back.  Joint value = minus the turn about robot +y
    (measured 11-07; same on both sides).  Fusion rounds rotationValue to 0.1 deg, so a joint pose can differ from
    transforms() by up to 0.05 deg per joint (0.4 mm at the wheel): still a rigid leg, just a slightly different pose.
    A joint pose also returns to 0 at design.computeAll() (not captured); analyzeInterference does not recompute (a
    170 deg hip gave 20 real interferences through this path)."""
    C, PO, P = M['cadlib'], M['poses'], M['params']
    J = _joint_map(C)
    if not J:
        PO.apply(PO.transforms(L0, hip, spin))
        return
    phi0, phi = P.PHI, math.acos(L0 / (2 * P.L1))
    h = math.radians(hip)
    df, dt = phi + h - phi0, h - phi + phi0            # poses.transforms: femur turned by -df, tibia by -dt
    for _s, k in SIDES:
        J['hip_' + k].jointMotion.rotationValue = df
        J['knee_' + k].jointMotion.rotationValue = dt - df
        J['wheel_' + k].jointMotion.rotationValue = -math.radians(spin)


def _nominal(M):
    """The nominal pose; writes nothing when the joints are already at 0 (so a read-only script can still call it)."""
    J = _joint_map(M['cadlib'])
    if not J:
        M['poses'].reset()
        return
    for j in J.values():
        if abs(j.jointMotion.rotationValue) > 1e-9:
            j.jointMotion.rotationValue = 0.0


def pose(L0=None, hip=0.0, spin=0.0):
    """Show a pose (leg length L0 mm, hip deg, wheel spin deg); pose() = the nominal pose."""
    doc()
    M = load()
    if L0 is None:
        _nominal(M)
    else:
        _pose(M, L0, hip, spin)


def joint_check(angle_deg=20.0):
    """Every joint turned by angle_deg and back.  Per joint: links that moved, the largest difference of rigid motion
    between the occurrences below one link (0 = nothing left behind), how far a point on the axis moved (mm) and the
    largest transform difference after turning back.  Hidden masters in _library must not move."""
    doc()
    M = load()
    C, P = M['cadlib'], M['params']
    root = C.design().rootComponent
    _nominal(M)

    def state():
        return {o.fullPathName: o.transform2.copy() for o in root.allOccurrences}

    def relative(after, before):
        m = before.copy()
        m.invert()
        m.transformBy(after)
        return m

    def diff(a, b):
        return max(abs(x - y) for x, y in zip(a.asArray(), b.asArray()))

    axes = dict(hip=(0.0, P.Z_HIP), knee=(P.KNEE_X, P.KNEE_Z), wheel=(0.0, P.R_WHEEL))
    res = {}
    for j in root.asBuiltJoints:
        a0 = state()
        j.jointMotion.rotationValue = math.radians(angle_deg)
        a1 = state()
        x, z = axes[j.name.split('_')[0]]
        p = C.rf((x, 0.0, z))
        p = adsk.core.Point3D.create(p[0] * C.CM, p[1] * C.CM, p[2] * C.CM)
        links, spread, shift = [], 0.0, 0.0
        for g in {n.split('+')[0] for n in a0}:
            names = [n for n in a0 if n.split('+')[0] == g]
            R0 = relative(a1[names[0]], a0[names[0]])
            spread = max(spread, max(diff(relative(a1[n], a0[n]), R0) for n in names))
            if diff(R0, adsk.core.Matrix3D.create()) > 1e-6:
                links.append(g)
                q = p.copy()
                q.transformBy(R0)
                shift = max(shift, p.distanceTo(q) * 10.0)
        j.jointMotion.rotationValue = 0.0
        a2 = state()
        res[j.name] = dict(moved=sorted(links), spread=round(spread, 6), axis_shift_mm=round(shift, 4),
                           back=round(max(diff(a2[n], a0[n]) for n in a0), 6))
    return res


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
    _need_direct(C, 'shift')
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


def _dowel_holes(body):
    """The Ø3 dowel holes of a wheel hub flange body: [(bottom centre, open centre, depth mm)] in the body's coordinates
    (the bottom is the circle on a small flat disc, the open end the circle on the output face)."""
    out = []
    for f in body.faces:
        g = f.geometry
        if not (isinstance(g, adsk.core.Cylinder) and abs(g.radius - 0.15) < 1e-4):
            continue
        ends = []
        for e in f.edges:
            if isinstance(e.geometry, adsk.core.Circle3D):
                other = [x for x in e.faces if x != f][0]
                ends.append((isinstance(other.geometry, adsk.core.Plane) and other.area < 0.08, e.geometry.center))
        bot = [c for small, c in ends if small]
        top = [c for small, c in ends if not small]
        assert len(bot) == 1 and len(top) == 1, ('dowel hole ends', len(bot), len(top))
        out.append((bot[0], top[0], bot[0].distanceTo(top[0]) * 10.0))
    return out


def deepen_hub_dowels(depth=None):
    """Hub file migration (11-08, user): the wheel hub flange dowel holes 7.0 -> HUB_DOWEL_HOLE deep, and each wheel's
    3 pins moved to the new hole bottoms (pressed to the bottom: 2.5 instead of 3.0 proud into the AKE output's 3 mm
    holes, as the hip / knee inserts).  build() makes them so from params.HUB_DOWEL_HOLE; this changes an existing
    document.  Measured on the bodies (any turn state); a second run changes nothing.  Route: unjoint(), this, turn(),
    joints()."""
    doc()
    M = load()
    C = M['cadlib']
    _need_direct(C, 'deepen_hub_dowels')
    depth = HUB_DOWEL_HOLE if depth is None else depth
    root = C.design().rootComponent
    tbm = adsk.fusion.TemporaryBRepManager.get()
    rep = []
    for _s, k in SIDES:
        fo = [o for o in root.allOccurrences if o.component.name == 'hub_flange_' + k and '_library' not in o.fullPathName]
        assert len(fo) == 1, ('hub_flange_' + k, len(fo))
        fo = fo[0]
        wheel = fo.assemblyContext                      # wheel_k: the pins are its children as well
        assert wheel is not None and wheel.component.name == 'wheel_' + k, fo.fullPathName
        comp = fo.component
        bodies = [b for b in comp.bRepBodies if b.isSolid]
        assert len(bodies) == 1, ('hub_flange_' + k, len(bodies))
        body = bodies[0]
        bname = body.name
        holes = _dowel_holes(body)                      # component coordinates
        assert len(holes) == 3, ('hub_flange_' + k, len(holes))
        pins = [o for o in wheel.childOccurrences if o.component.name.startswith('FX ISO 8734')]   # proxies from the root
        assert len(pins) == 3, ('pins of wheel_' + k, len(pins))
        for b, t, dep in holes:
            add = depth - dep
            if add < -1e-3:
                raise RuntimeError('hub_flange_%s: hole already %.3f deep (> %.3f)' % (k, dep, depth))
            if add <= 1e-3:
                continue
            u = t.vectorTo(b)                           # open end -> bottom: into the material
            u.normalize()
            p1, p2 = b.copy(), b.copy()
            v = u.copy()
            v.scaleBy(-0.001)                           # 0.01 mm back into the hole: a clean cut
            p1.translateBy(v)
            v = u.copy()
            v.scaleBy(add / 10.0)
            p2.translateBy(v)
            tool = comp.bRepBodies.add(tbm.createCylinderOrCone(p1, 0.15, p2, 0.15))
            tool.name = 'dowel_cut_tmp'
            coll = adsk.core.ObjectCollection.create()
            coll.add(tool)
            body = comp.bRepBodies.itemByName(bname)
            ci = comp.features.combineFeatures.createInput(body, coll)
            ci.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
            ci.isKeepToolBodies = False
            comp.features.combineFeatures.add(ci)
        assert comp.bRepBodies.count == 1, ('hub_flange_' + k, [b.name for b in comp.bRepBodies])
        # each pin (origin = its end in the hole, on the hole axis) to its hole bottom, in world coordinates (a root
        # proxy's transform2): re-instanced, because a proxy override goes back when the parent link is set later
        body = comp.bRepBodies.itemByName(bname)
        moved = 0
        for b, t, dep in _dowel_holes(body.createForAssemblyContext(fo)):
            u = t.vectorTo(b)
            u.normalize()
            hit = []
            for o in [o for o in wheel.childOccurrences if o.component.name.startswith('FX ISO 8734')]:
                w = b.vectorTo(o.transform2.translation.asPoint())
                along = w.dotProduct(u)
                if math.sqrt(max(w.length ** 2 - along ** 2, 0.0)) < 1e-4:
                    hit.append((along, o))
            assert len(hit) == 1, ('pin on the hole axis', k, len(hit))
            along, o = hit[0]
            if abs(along) > 1e-5:
                o = _reinstance(C, o, o.transform2.translation.asPoint().vectorTo(b))
                moved += 1
            got = o.transform2.translation.asPoint().distanceTo(b) * 10.0
            assert got < 1e-3, ('pin not at the hole bottom', k, got)
        rep.append((k, [round(h[2], 3) for h in _dowel_holes(body)], moved, round(body.volume * 1000.0, 3)))
    return rep


def _reinstance(C, o, d=None, comp=None):
    """Occurrence proxy o (a child of a root-level link) replaced by a fresh instance of comp (default its own
    component) moved by the world vector d (cm).  An inserted placement survives later moves of the parent link
    (IdentityParent, poses.reset); a transform set on the proxy does not.  Returns the new proxy."""
    parent = o.assemblyContext
    assert parent is not None and parent.assemblyContext is None, o.fullPathName
    m = o.nativeObject.transform2.copy()
    if d is not None:
        inv = parent.transform2.copy()
        inv.invert()
        dl = d.copy()
        dl.transformBy(inv)                           # a vector: rotation only
        t = m.translation
        t.add(dl)
        m.translation = t
    want = o.transform2.copy()
    if d is not None:
        t = want.translation
        t.add(d)
        want.translation = t
    n = C.instance(parent, comp or o.component, m).createForAssemblyContext(parent)
    got = n.transform2.asArray()
    assert max(abs(a - b) for a, b in zip(want.asArray(), got)) < 1e-7, (o.fullPathName, want.asArray(), got)
    o.deleteMe()
    return n


def _end_disc(face, small=0.08):
    """(bottom centre, open centre, depth mm) of a cylindrical pocket face: the bottom is the circle on the smaller
    adjacent flat face (area < small cm2), the open end the circle on the other one."""
    ends = []
    for e in face.edges:
        if isinstance(e.geometry, (adsk.core.Circle3D, adsk.core.Arc3D)):   # an arc where a pocket breaks a rounded edge
            other = [x for x in e.faces if x != face][0]
            ends.append((isinstance(other.geometry, adsk.core.Plane) and other.area < small, e.geometry.center))
    bot = [c for sm, c in ends if sm]
    top = [c for sm, c in ends if not sm]
    if len(bot) != 1 or len(top) != 1:
        return None
    return bot[0], top[0], bot[0].distanceTo(top[0]) * 10.0


def _cut_cyl(comp, bname, a, b, r_mm):
    """Cut a cylinder (component coordinates a -> b, cm) from body bname of comp (direct design)."""
    tbm = adsk.fusion.TemporaryBRepManager.get()
    tool = comp.bRepBodies.add(tbm.createCylinderOrCone(a, r_mm / 10.0, b, r_mm / 10.0))
    tool.name = 'stackup_cut_tmp'
    coll = adsk.core.ObjectCollection.create()
    coll.add(tool)
    ci = comp.features.combineFeatures.createInput(comp.bRepBodies.itemByName(bname), coll)
    ci.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
    ci.isKeepToolBodies = False
    comp.features.combineFeatures.add(ci)


def _deepen(comp, bname, r_mm, depth_mm, count, small):
    """Every cylindrical pocket of radius r_mm in body bname to depth_mm (the extra cut along its own axis, same
    radius).  Returns [(old depth, bottom before, unit axis into the material)] in component coordinates."""
    body = comp.bRepBodies.itemByName(bname)
    pockets = []
    for f in body.faces:
        g = f.geometry
        if isinstance(g, adsk.core.Cylinder) and abs(g.radius * 10.0 - r_mm) < 1e-3:
            e = _end_disc(f, small)
            if e:
                pockets.append(e)
    assert len(pockets) == count, (bname, r_mm, len(pockets), count)
    out = []
    for b, t, dep in pockets:
        add = depth_mm - dep
        assert add > -1e-3, (bname, r_mm, dep, depth_mm)
        u = t.vectorTo(b)
        u.normalize()
        if add > 1e-3:
            p1, p2 = b.copy(), b.copy()
            v = u.copy()
            v.scaleBy(-0.001)
            p1.translateBy(v)
            v = u.copy()
            v.scaleBy(add / 10.0)
            p2.translateBy(v)
            _cut_cyl(comp, bname, p1, p2, r_mm)
        out.append((dep, b, u, add))
    return out


def _swap_master(C, F, old, new):
    """Every instance of fastener master old replaced by master new (same placement: origin = head bearing face, +z =
    travel), the old master dropped from _library.  New masters come from fasteners.specs() (v5 params)."""
    sp = F.specs()[new]
    comp = C.find_comp(new)
    if comp is None:
        comp = C.master(new)
        C.add_body(comp, sp[0](), new, sp[1], sp[2])
    root = C.design().rootComponent
    olds = [o for o in root.allOccurrences if o.component.name == old and '_library' not in o.fullPathName]
    for o in olds:
        _reinstance(C, o, None, comp)
    lib = C.library().component
    for o in list(lib.occurrences):
        if o.component.name == old:
            o.deleteMe()
    return len(olds)


def fix_stackups():
    """Hub file migration (11-08, user: "문제 2건 + 쉬운 것" after the axial stack-up review): on the parts the v5 leg / shell
    rebuilds do not touch - wheel hub flange dowel holes to HUB_DOWEL_HOLE (pins with them) and spigot to HUB_SPIGOT
    height, rim register recess to RIM_REG + RIM_REG_GAP deep, top plate counterbores to TOP_CBORE (M4 x 25 heads with
    them), shell M3 screws M3 x SHELL_SCREW_L and moteus board screws M2.5 x BOARD_SCREW_L.  build() makes all of it
    from v5_params.  Route: unjoint(), this, relegs() (insert holes, insert spigot), reshell() (shell pads), turn(),
    robot_mass(), joints().  A second run changes nothing."""
    deep = deepen_hub_dowels()
    doc()
    M = load()
    C, P, F = M['cadlib'], M['params'], M['fasteners']
    _need_direct(C, 'fix_stackups')
    root = C.design().rootComponent
    rep = dict(dowels=deep)

    def one(name):
        o = [x for x in root.allOccurrences if x.component.name == name and '_library' not in x.fullPathName]
        assert len(o) == 1, (name, len(o))
        return o[0]

    for _s, k in SIDES:
        # hub flange spigot: its tip face (a flat disc of r 5.9) down to HUB_SPIGOT[1] above the output face
        fo = one('hub_flange_' + k)
        comp, bname = fo.component, 'hub_flange_' + k
        body = comp.bRepBodies.itemByName(bname)
        tips = [f for f in body.faces if isinstance(f.geometry, adsk.core.Plane) and f.area < 1.2   # not the output face
                and any(isinstance(e.geometry, adsk.core.Circle3D) and abs(e.geometry.radius * 10.0 - P.HUB_SPIGOT[0]) < 1e-3
                        for e in f.edges)]
        assert len(tips) == 1, ('spigot tip', k, len(tips))
        tip = tips[0]
        n = tip.evaluator.getNormalAtPoint(tip.pointOnFace)[1]          # out of the material = towards the motor
        out_face = _dowel_holes(body)[0][1]                             # an open dowel end lies on the output face
        h = out_face.vectorTo(tip.pointOnFace).dotProduct(n) * 10.0
        trim = h - P.HUB_SPIGOT[1]
        assert trim > -1e-3, ('spigot', k, h)
        if trim > 1e-3:
            c = tip.centroid
            a, b = c.copy(), c.copy()
            v = n.copy()
            v.scaleBy(0.01)
            a.translateBy(v)
            v = n.copy()
            v.scaleBy(-trim / 10.0)
            b.translateBy(v)
            _cut_cyl(comp, bname, a, b, P.HUB_SPIGOT[0] + 0.3)
        rep['spigot_' + k] = (round(h, 3), round(trim, 3))
        # rim register recess (Ø11.2, its floor the smaller flat face)
        ro = one('rim_' + k)
        rr = P.RIM_REG[0] + P.RIM_REG_GAP[0]
        rep['rim_' + k] = [(round(d, 3), round(add, 3)) for d, _b, _u, add in
                           _deepen(ro.component, 'rim_' + k, rr, P.RIM_REG[1] + P.RIM_REG_GAP[1], 1, 1.2)]
    # top plate: Ø8 counterbores to TOP_CBORE, the 8 M4 x 25 heads down onto the new floors (world = torso local)
    tp = one('top_plate')
    torso = tp.assemblyContext
    to_world = tp.transform2
    cb = _deepen(tp.component, 'top_plate', 4.0, P.TOP_CBORE, 8, 0.4)
    assert len([o for o in torso.childOccurrences if o.component.name == 'FX ISO 4762 M4x25 top']) == 8
    moved = 0
    for dep, b, u, add in cb:
        screws = [o for o in torso.childOccurrences if o.component.name == 'FX ISO 4762 M4x25 top']
        old, new = b.copy(), b.copy()
        v = u.copy()
        v.scaleBy(add / 10.0)
        new.translateBy(v)
        old.transformBy(to_world)
        new.transformBy(to_world)
        hit = [o for o in screws if o.transform2.translation.asPoint().distanceTo(old) < 1e-4
               or o.transform2.translation.asPoint().distanceTo(new) < 1e-4]
        assert len(hit) == 1, ('M4 head on the counterbore floor', len(hit))
        o = hit[0]
        if o.transform2.translation.asPoint().distanceTo(new) > 1e-5:
            o = _reinstance(C, o, o.transform2.translation.asPoint().vectorTo(new))
            moved += 1
        assert o.transform2.translation.asPoint().distanceTo(new) < 1e-4
    rep['top_cbore'] = (sorted(set(round(d, 3) for d, *_r in cb)), moved)
    # fastener masters
    rep['shell_screws'] = _swap_master(C, F, 'FX ISO 7380 M3x6 shell', 'FX ISO 7380 M3x%d shell' % P.SHELL_SCREW_L) \
        if P.SHELL_SCREW_L != 6 else 0
    rep['board_screws'] = _swap_master(C, F, 'FX DIN 7984 M2.5x6 board', 'FX DIN 7984 M2.5x%d board' % P.BOARD_SCREW_L) \
        if P.BOARD_SCREW_L != 6 else 0
    return rep


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
    +y links: a point inside the outer shape but outside the part is a gap (the ribs have no holes, so any gap is a
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
    K = M['check']
    short = lambda n: n.split('/')[-1][:38]
    out = []
    try:
        for L0, hip in poses:
            _pose(M, L0, hip)
            mating, real = K.classify(K.interference())
            out.append((L0, hip, len(mating), len(real), sorted({(short(a), short(b)) for a, b, v in real})[:12]))
    finally:
        _nominal(M)
    return out


def show(torso=True, both_legs=True):
    """Everything on; both_legs=False keeps only the +y leg (the robot's right, the one the leg views look at)."""
    import cadlib as C
    root = C.design().rootComponent
    for o in root.occurrences:
        n = o.component.name
        o.isLightBulbOn = n != '_library' and (torso or n != 'torso') and (both_legs or not n.endswith('_L'))
    for o in root.allOccurrences:
        if o.assemblyContext is not None and o.component.name != '_library' \
                and o.assemblyContext.component.name != '_library':
            o.isLightBulbOn = True
    root.isJointsFolderLightBulbOn = root.asBuiltJoints.count > 0


def snap(name, eye, target, fit=True):
    import view
    os.makedirs(IMG, exist_ok=True)
    app = adsk.core.Application.get()
    app.userInterface.activeSelections.clear()   # a face picked in the window shows up
    view.look(eye, target, fit=fit)
    vp = app.activeViewport
    if fit:                                      # (fit alone cut into the tyre of the taller robot)
        cam = vp.camera
        cam.isFitView = False
        e, t = cam.eye, cam.target
        cam.eye = adsk.core.Point3D.create(t.x + (e.x - t.x) * 1.15, t.y + (e.y - t.y) * 1.15, t.z + (e.z - t.z) * 1.15)
        vp.camera = cam
        vp.refresh()
        adsk.doEvents()
    path = os.path.join(IMG, name + '.png')
    vp.saveAsImageFile(path, 1600, 1200)
    return path


def images():
    """Leg views (+y leg alone), robot views and truss sections (+y links cut at y 199 / 229.5, looking from
    outboard: the plain ribs and the closed cells)."""
    doc()
    M = load()
    C, LV, P = M['cadlib'], M['legs_v5'], M['params']
    _need_direct(C, 'images')                      # (adds a temporary section component with bodies)
    ui = adsk.core.Application.get().userInterface
    lc = adsk.core.ListControlDefinition.cast(ui.commandDefinitions.itemById('ViewLayoutGridOnCommand').controlDefinition)
    lc.listItems.item(lc.listItems.count - 1).isSelected = True          # grid off
    out = []
    up = (P.Z_HIP - 411.0) / 2.0                   # the views were set on the 175 mm robot (hip at z 411)

    def lift(p):
        return (p[0], p[1], p[2] + up)

    show(torso=False, both_legs=False)
    snap('_warm', lift((760, 1050, 560)), lift((40, 235, 240)))
    adsk.doEvents()
    for name, eye, tgt in (('01_leg_iso_outboard', (760, 1050, 560), (40, 235, 240)),
                           ('02_leg_side_outboard', (40, 1600, 250), (40, 235, 250)),
                           ('03_leg_iso_inboard_rear', (760, -320, 470), (40, 215, 240)),
                           ('04_leg_front', (-760, 640, 470), (20, 225, 240))):
        out.append(snap(name, lift(eye), lift(tgt)))
    show()
    out.append(snap('05_robot_iso', lift((1500, 1150, 900)), lift((0, 0, 290))))
    out.append(snap('06_robot_rear', lift((1700, 0, 330)), lift((0, 0, 290))))
    out.append(snap('07_robot_side', lift((0, 1900, 330)), lift((0, 0, 290))))
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
        snap('_warm', lift((40, 1600, 250)), lift((40, 235, 250)))
        adsk.doEvents()
        out.append(snap('08_section', lift((40, 1600, 250)), lift((40, 235, 250))))
        out.append(snap('09_section_iso', lift((620, 1050, 780)), lift((40, 205, 270))))
        out.append(snap('10_section_thigh_zoom', (350, 640, 620), (45, 195, 430), fit=False))    # the thigh's middle
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
    _nominal(M)
    show()
    M['check'].apply_print_mass()
    txt, data, _g = M['check'].report()
    return txt


def save_mass_json(path=None, apply_print=False):
    """Per-link mass properties of the v5 robot (nominal pose, robot axes, mm / kg / kg mm^2) for
    wbr/bench/ake_cad.py -> ../calc/mass_properties.json.  v4's check.save_mass_json writes the INBOARD tyre centre
    (params.tyre_centre_y, 173.5); here the wheel group is turned about Y_MSH, so the tyre centre is the mirrored
    value (270.0), checked against the tyre bodies of the model.  The hub file keeps the print densities and the tyre
    mass of robot_mass(), so by default nothing is changed (a read-only script works); apply_print=True re-applies
    them first (modifies the document, e.g. after relegs() / reshell())."""
    import json
    doc()
    M = load()
    C, P, K, LV = M['cadlib'], M['params'], M['check'], M['legs_v5']
    _need_names(C)
    _nominal(M)                                    # (robot axes at the nominal pose; no write when already there)
    if apply_print:
        show()
        K.apply_print_mass()
    root = C.design().rootComponent
    hidden = [o.fullPathName for o in root.allOccurrences if not o.isLightBulbOn and '_library' not in o.fullPathName]
    assert not hidden, ('hidden occurrences are left out of the report: show() first', hidden[:5])
    off = [o.fullPathName + ' | ' + b.name for o in K.leaves() for b in o.bRepBodies if not b.isLightBulbOn]
    assert not off, ('hidden bodies are left out of the report too (11-07: shell_front_L, 132 g): turn them on', off[:5])
    raw = sorted({o.component.name for o in K.leaves() for b in o.bRepBodies if b.isSolid
                  and b.material.name == 'PETG print' and not o.component.name.startswith(('thigh_', 'shank_'))})
    assert not raw, ('printed parts without their print density: apply_print=True', raw)
    tyres = [o for o in K.leaves() if o.component.name.startswith('tyre M365')]
    assert tyres and all(b.material.name.startswith('tyre ') for o in tyres for b in o.bRepBodies), 'tyre mass not applied'
    txt, data, groups = K.report()
    y_tyre = 2 * LV.Y_MSH - P.tyre_centre_y()
    ty = [(m, c) for path, m, c, _i in groups['wheel_' + SIDES[0][1]] if 'tyre M365' in path]     # the +y wheel
    y_meas = sum(m * c[1] for m, c in ty) / sum(m for m, c in ty)
    assert abs(y_meas - y_tyre) < 0.5, ('tyre centre', y_meas, y_tyre)
    out = dict(units="mm, kg, kg mm^2 about the COM, CAD axes (+x knee side, +y the _R parts, z up, floor origin; the robot's front is the rocker switch side = -x, wbr.models.build_v5 turns them)",
               pose=dict(L0=P.L0, L1=P.L1, L2=P.L2, R_wheel=P.R_WHEEL, Z_hip=P.Z_HIP, phi_deg=math.degrees(P.PHI),
                         knee=(P.KNEE_X, P.KNEE_Z), top_plate_z=P.Z_HIP + P.RAIL_Z + P.RAIL_S / 2 + P.TOP_PLATE[2],
                         tyre_centre_y=y_tyre, tyre_centre_y_model=y_meas, wheel_offset=P.wheel_offset(),
                         wheels='outboard'),
               links={k: dict(mass=v['mass'], com=list(v['com']), inertia=v['inertia']) for k, v in data.items()})
    path = path or os.path.join(BASE, 'calc', 'mass_properties.json')
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(out, f, indent=1)
    return txt + '\ntyre centre y %.2f (model %.2f) -> %s' % (y_tyre, y_meas, path)


# pi3hat r4.5 IMU footprints (references/pi3hat/hw/r4.5/pi3_hat.brd, mm from the board corner: x from the 40-pin header side
# towards the CAN connectors, y from the XT30 side towards the Pi's USB end).  Board size 56.5 x 65, 1.6 thick.  The firmware
# tries the BMI088 first, then the ICM-42688-P.
IMU_BRD = {'BMI088': (36.0, 29.7), 'ICM42688': (35.0, 24.8)}
IMU_ABOVE_PCB = 0.5         # chip centre above the PCB top face


def imu_pose():
    """The IMU in robot axes, measured from the pi3hat occurrence: position of both footprints and the raw IMU axes of
    docs/reference.md (+x towards the CAN connectors / the Pi's HDMI, +y towards the Pi's USB, +z from the Pi to the hat)."""
    doc()
    M = load()
    C = M['cadlib']
    occ = [o for o in C.design().rootComponent.allOccurrences
           if o.component.name.startswith('pi3hat') and '_library' not in o.fullPathName]
    assert len(occ) == 1, occ
    T = occ[0].transform2

    def world(x_s, y_s, z_s):                       # STEP frame of the hat: x = board x - 28.25, y up from the PCB bottom, z = 32.5 - board y
        p = adsk.core.Point3D.create(x_s / 10.0, y_s / 10.0, z_s / 10.0)
        p.transformBy(T)
        return f2r(p)

    def vec(x_s, y_s, z_s):
        a, b = world(0, 0, 0), world(x_s, y_s, z_s)
        return [b[i] - a[i] for i in range(3)]

    chips = {k: list(world(x - 28.25, 1.6 + IMU_ABOVE_PCB, 32.5 - y)) for k, (x, y) in IMU_BRD.items()}
    return dict(chips=chips, axes=dict(x=vec(1, 0, 0), y=vec(0, 0, -1), z=vec(0, 1, 0)), board='pi3hat r4.5',
                note='position mm from the floor origin, axes = raw IMU axes in CAD coordinates')


def urdf_export(out=None):
    """What the URDF of the v5 robot needs from the model: ../calc/urdf_data.json (joint axes, IMU pose) and one STL per link
    (../export/links_stl, Fusion axes in mm, nominal pose).  Writes the document nothing; a body the user hid is
    shown for the export and hidden again.  Then: wheeled_biped: python -m wbr.models.build_v5."""
    import json
    doc()
    M = load()
    C, P, LV = M['cadlib'], M['params'], M['legs_v5']
    _need_names(C)
    _nominal(M)
    root = C.design().rootComponent
    out = out or os.path.join(EXP, 'links_stl')
    os.makedirs(out, exist_ok=True)
    em = C.design().exportManager
    hidden = []
    for o in root.allOccurrences:
        if '_library' in o.fullPathName:
            continue
        for b in o.bRepBodies:
            if not b.isLightBulbOn:
                b.isLightBulbOn = True
                hidden.append(b)
    done = []
    try:
        for occ in root.occurrences:
            if occ.component.name == '_library':
                continue
            fn = os.path.join(out, occ.component.name + '.stl')
            o = em.createSTLExportOptions(occ, fn)
            o.isBinaryFormat = True
            o.isOneFilePerBody = False
            o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
            em.execute(o)
            done.append(occ.component.name)
    finally:
        for b in hidden:
            b.isLightBulbOn = False
    J = {}
    for j in root.asBuiltJoints:
        v = j.geometry.primaryAxisVector
        J[j.name] = dict(origin=list(f2r(j.geometry.origin)), axis=[v.z, v.x, v.y])
    data = dict(units="mm, CAD axes (+x knee side, +y the _R parts, z up, floor origin; the robot's front is the rocker switch side = -x, wbr.models.build_v5 turns them), nominal pose",
                imu=imu_pose(), joints=J,
                joint_y=dict(hip=P.YW, knee=LV.Y_MSH - 13.75, wheel=2 * LV.Y_MSH - P.tyre_centre_y()),
                tyre=dict(radius=P.R_WHEEL, width=P.TYRE_W, centre_y=2 * LV.Y_MSH - P.tyre_centre_y()),
                links=done)
    path = os.path.join(BASE, 'calc', 'urdf_data.json')
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=1)
    return path, done, data['joint_y'], data['imu']


def export():
    doc()
    M = load()
    _need_names(M['cadlib'])
    _nominal(M)
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
    _need_names(C)
    _nominal(M)
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
    """Real 2D sections of the +y leg (nominal pose): every visible body near the hip / knee / wheel axis cut by
    planes through that axis (angle a in deg from robot +x towards +z; the plane holds the axis and that direction).
    Writes JSON records (joint, angle, component, occurrence path, body, edge polylines in (robot y, t) with t the
    distance from the axis along the plane, solid) for ../manufacturing/explain/real_sections.py (default path:
    ../manufacturing/explain/sections_v5.json.gz).  The plane intersections are wires: real_sections chains them."""
    import gzip
    import json
    doc()
    M = load()
    P = M['params']
    _need_names(M['cadlib'])
    _nominal(M)
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
