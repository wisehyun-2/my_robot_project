"""Pose sweep: leg kinematics, cable loops re-shaped per pose, interference and minimum clearances (brief item 10).

Leg length L0 (hip axis -> wheel axle) with the wheel kept under the hip for hip = 0; hip swings the whole
leg (+ forward).  L1 = L2, so the thigh turns by d(phi) and the shank by -d(phi), both plus the hip swing.
The links are moved by setting their occurrence transforms (direct design, no joint solver).

Pairs that cannot change with the pose are reported once (joint_gaps): thigh / shank / torso lie in separate
lateral bands (2.0 and 1.5 mm face gaps), driven bosses turn inside Ø30 holes (1.0 mm), hub and tyre turn
about the axle.  The sweep measures the pairs that can: knee driver / cup / thigh cables vs torso, tyre /
rim / hub vs torso and thigh, both cable loops vs everything, and the full interference analysis.
"""
import math
import time

import adsk.core
import adsk.fusion

import cadlib as C
import check as K
import harness as HN
import params as P

LINKS = ('femur_L', 'tibia_L', 'wheel_L', 'femur_R', 'tibia_R', 'wheel_R')


def transforms(L0, hip_deg, spin_deg=0.0):
    phi0, phi = P.PHI, math.acos(L0 / (2 * P.L1))
    h = math.radians(hip_deg)
    df = (phi + h) - phi0
    Tf = C.rot_about((0.0, 0.0, P.Z_HIP), (0, 1, 0), -df)
    K0 = (P.KNEE_X, 0.0, P.KNEE_Z)
    tf = phi + h
    Kn = (P.L1 * math.sin(tf), 0.0, P.Z_HIP - P.L1 * math.cos(tf))
    dt = (-phi + h) - (-phi0)
    Tt = C.mmul(C.translate(C.sub(Kn, K0)), C.rot_about(K0, (0, 1, 0), -dt))
    Tw = C.mmul(Tt, C.rot_about((0.0, 0.0, P.R_WHEEL), (0, 1, 0), math.radians(spin_deg)))
    out = {'torso': C.translate((0, 0, 0))}
    for k in ('L', 'R'):
        out['femur_' + k], out['tibia_' + k], out['wheel_' + k] = Tf, Tt, Tw
    return out


def apply(T):
    for name in LINKS:
        C.root_occ(name).transform2 = C.to_matrix3d(C.robot_to_fusion(T[name]))


def reset():
    apply(transforms(P.L0, 0.0))


def show_all():
    root = C.design().rootComponent
    for o in root.occurrences:
        o.isLightBulbOn = o.component.name != '_library'
        if o.component.name != '_library':
            for c in o.childOccurrences:
                c.isLightBulbOn = True


def loops():
    """[(cable name, link_a, a, ta, link_b, b, tb, length)] of both sides."""
    out = []
    for s in (1, -1):
        for cab in HN.cables(s):
            for rn in cab['runs']:
                if rn[0] == 'loop':
                    out.append((cab['name'],) + tuple(rn[1:]))
    return out


def loop_curve(T, lp):
    name, la, a, ta, lb, b, tb, L = lp
    a2, b2 = C.apply(T[la], a), C.apply(T[lb], b)
    ta2, tb2 = C.applyv(T[la], ta), C.applyv(T[lb], tb)
    pts, ok = HN.loop_points(a2, ta2, b2, tb2, L)
    return pts, ok, C.norm(C.sub(b2, a2))


def chord_table(L0s, hips):
    """Anchor distance of every loop over the pose grid (to size the loop lengths)."""
    out = {}
    for lp in loops():
        ds = []
        for L0 in L0s:
            for h in hips:
                T = transforms(L0, h)
                ds.append((loop_curve(T, lp)[2], L0, h))
        out[lp[0]] = (min(ds), max(ds))
    return out


# ----------------------------------------------------------------------------- measuring
def _bbox(o):
    bb = o.boundingBox
    return (bb.minPoint.x * 10, bb.minPoint.y * 10, bb.minPoint.z * 10, bb.maxPoint.x * 10, bb.maxPoint.y * 10,
            bb.maxPoint.z * 10)


def _bb_gap(a, b):
    d = 0.0
    for i in range(3):
        g = max(a[i] - b[i + 3], b[i] - a[i + 3], 0.0)
        d += g * g
    return math.sqrt(d)


def link_of(o):
    p = o.fullPathName.split('+')[0]
    return p.split(':')[0]


def _hip_joint_part(o):
    """A fastener of the hip joint riding on the thigh (output screws, washers, dowels within 25 mm of the hip axis):
    it turns with the hip output inside the motor, so it belongs to the joint gaps, not to the thigh-side sweep."""
    if not o.component.name.startswith('FX '):
        return False
    bb = o.boundingBox
    x = (bb.minPoint.z + bb.maxPoint.z) * 5.0             # Fusion (X, Y, Z) = robot (y, z, x), cm -> mm
    z = (bb.minPoint.y + bb.maxPoint.y) * 5.0
    return math.hypot(x, z - P.Z_HIP) < 25.0


GROUPS = {
    # name: (selector of moving parts, selector of the parts they are measured against)
    'knee driver / cup / thigh cables vs torso': (
        lambda o, lk: lk.startswith('femur') and not o.component.name.startswith(('thigh', 'AKE60'))
        and not _hip_joint_part(o),
        lambda o, lk: lk == 'torso'),
    'tyre / rim / hub vs torso': (
        lambda o, lk: lk.startswith('wheel') and not o.component.name.startswith('AKE60'),
        lambda o, lk: lk == 'torso'),
    'tyre / rim / hub vs thigh, knee cup, thigh cables': (
        lambda o, lk: lk.startswith('wheel') and not o.component.name.startswith('AKE60'),
        lambda o, lk: lk.startswith('femur')),
    'thigh vs torso shell / shell screws': (                    # 10-31: the thigh body itself (ISO 7380 heads)
        lambda o, lk: lk.startswith('femur') and o.component.name.startswith('thigh'),
        lambda o, lk: lk == 'torso' and o.component.name.startswith(('shell_', 'saddle_', 'deco_', 'FX ISO 7380',
                                                                     'FX grommet'))),
    'shank parts vs torso': (
        lambda o, lk: lk.startswith('tibia'),
        lambda o, lk: lk == 'torso'),
}


def measure_groups(leaves, thresh=30.0):
    info = [(o, link_of(o), _bbox(o)) for o in leaves]
    res = {}
    for g, (sel_a, sel_b) in GROUPS.items():
        best = (1e9, '', '')
        A = [x for x in info if sel_a(x[0], x[1])]
        Bs = [x for x in info if sel_b(x[0], x[1])]
        for (oa, la, ba) in A:
            side = la[-1]
            for (ob, lb, bb) in Bs:
                if lb != 'torso' and lb[-1] != side:
                    continue
                if _bb_gap(ba, bb) > thresh or _bb_gap(ba, bb) > best[0]:
                    continue
                d = K.min_distance(oa, ob)
                if d < best[0]:
                    best = (d, oa.fullPathName, ob.fullPathName)
        res[g] = best
    return res


def measure_loops(T, leaves, scratch=None, thresh=30.0):
    """Build each loop for pose T in a temporary root component, measure it against every leaf except its
    own cable."""
    B = C.Builder()
    out = {}
    info = [(o, _bbox(o)) for o in leaves]
    root = C.design().rootComponent
    for lp in loops():
        pts, ok, chord = loop_curve(T, lp)
        bodies = HN.wires_along(B, pts, HN.BUNDLE, HN.Y_SIDE)
        tmp = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        tmp.component.name = 'pose_loop_' + lp[0]
        for b in bodies:
            tmp.component.bRepBodies.add(b)
        r = 6.0
        f = [C.rf(p) for p in pts]                     # bounding box from the curve (Fusion axes, mm)
        bbl = (min(q[0] for q in f) - r, min(q[1] for q in f) - r, min(q[2] for q in f) - r,
               max(q[0] for q in f) + r, max(q[1] for q in f) + r, max(q[2] for q in f) + r)
        best = (1e9, '')
        for (o, bb) in info:
            if o.component.name.startswith('harness_'):        # cables may rest on cables; links and parts may not
                continue
            if _bb_gap(bbl, bb) > thresh or _bb_gap(bbl, bb) > best[0]:
                continue
            d = K.min_distance(tmp, o)
            if d < best[0]:
                best = (d, o.fullPathName)
        out[lp[0]] = (best[0], best[1], ok, chord)
        tmp.deleteMe()
    return out


def show_pose(L0, hip, spin=0.0):
    """Move the links to a pose and draw the pose's cable loops in a root component 'pose_loops' (the nominal
    loops are hidden).  clear_pose() undoes it."""
    root = C.design().rootComponent
    clear_pose()
    T = transforms(L0, hip, spin)
    apply(T)
    for o in root.allOccurrences:
        if o.component.name.endswith('_loop'):
            o.isLightBulbOn = False
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = 'pose_loops'
    B = C.Builder()
    for lp in loops():
        pts, ok, chord = loop_curve(T, lp)
        for b, look in zip(HN.wires_along(B, pts, HN.BUNDLE, HN.Y_SIDE), HN.BUNDLE_LOOKS):
            C.add_body(occ.component, b, lp[0], 'Copper wire', look)
    return T


def clear_pose():
    root = C.design().rootComponent
    for o in [o for o in root.occurrences if o.component.name in ('pose_loops', 'pose_scratch')
              or o.component.name.startswith('pose_loop_')]:
        o.deleteMe()
    reset()
    for o in root.allOccurrences:
        if o.component.name.endswith('_loop'):
            o.isLightBulbOn = True


def sweep(L0s=(200.0, 240.0, 280.0, 310.0, 335.0), hips=(-80.0, -60.0, -30.0, 0.0, 30.0, 60.0), loops_too=True):
    root = C.design().rootComponent
    show_all()
    nominal_loops = [o for o in root.allOccurrences if o.component.name.endswith('_loop')]
    for o in nominal_loops:
        o.isLightBulbOn = False
    scratch = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    scratch.component.name = 'pose_scratch'
    rows = []
    t0 = time.time()
    try:
        for L0 in L0s:
            for h in hips:
                T = transforms(L0, h)
                apply(T)
                leaves = [o for o in K.leaves() if not o.component.name.endswith('_loop')
                          and o.component.name != 'pose_scratch']
                inter = K.classify(K.interference(leaves))[1]
                grp = measure_groups(leaves)
                lps = measure_loops(T, leaves, scratch.component) if loops_too else {}
                rows.append(dict(L0=L0, hip=h, real=inter, groups=grp, loops=lps))
    finally:
        reset()
        scratch.deleteMe()
        for o in nominal_loops:
            o.isLightBulbOn = True
    return rows, time.time() - t0
