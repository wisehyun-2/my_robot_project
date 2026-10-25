"""Step 4: right leg, as-built revolute joints (hip, knee, wheel on both sides).

The right leg reuses the left link components: each right link is the left link
turned 180 deg about the vertical axis (a rigid copy, so every interface stays
exact), then the hip and knee are re-posed so the right knee points forward too.
On the right side the motor STEP models are not re-posed with the links, so the
output-disc bolt pattern of the hip and knee motor models is rotated against the
link holes there (cosmetic only: a real output turns with its link).
"""
import math

import adsk.core
import adsk.fusion

import build_setup as S
import cadlib as C
import params as P

RZ180_W = C.frame((0, 0, 0), (-1, 0, 0), (0, -1, 0), (0, 0, 1))


def right_frames():
    fem, tib, whl = S.femur_frame(), S.tibia_frame(), S.wheel_frame()
    fem, tib, whl = (C.mmul(RZ180_W, m) for m in (fem, tib, whl))
    yr = -P.Y_FEMUR_IN
    hip_pt = (0.0, yr, P.Z_HIP)
    rh = C.rot_about(hip_pt, (0, 1, 0), -2.0 * P.PHI)
    fem, tib, whl = (C.mmul(rh, m) for m in (fem, tib, whl))
    knee = C.apply(fem, (0.0, -115.0, 0.0))                 # femur-local knee centre (x, z valid)
    wheel = C.apply(whl, (0.0, 0.0, 0.0))
    cur = math.atan2(wheel[0] - knee[0], -(wheel[2] - knee[2]))
    want = math.atan2(0.0 - knee[0], -(P.R_WHEEL - knee[2]))
    beta = -(want - cur)
    rk = C.rot_about(knee, (0, 1, 0), beta)
    tib, whl = (C.mmul(rk, m) for m in (tib, whl))
    w = C.apply(whl, (0, 0, 0))
    k = C.apply(fem, (0.0, -115.0, 0.0))
    assert abs(w[0]) < 1e-6 and abs(w[2] - P.R_WHEEL) < 1e-6, w
    assert abs(k[0] - P.KNEE_X) < 1e-6 and abs(k[2] - P.KNEE_Z) < 1e-6, k
    return fem, tib, whl


def add_right_links():
    root = C.design().rootComponent
    fem, tib, whl = right_frames()
    out = {}
    for name, m in (('femur_link', fem), ('tibia_link', tib), ('wheel_link', whl)):
        comp = C.find_comp(name)
        occ = root.occurrences.addExistingComponent(comp, adsk.core.Matrix3D.create())
        occ.transform2 = S.link_matrix(m)
        out[name] = occ
    return out


# ----------------------------------------------------------------------------- joints
def circle_edge(link_occ, part_name, radius_mm):
    """A circular edge of the given radius on a part inside a root-level link occurrence (proxy)."""
    for sub in link_occ.childOccurrences:
        if sub.component.name != part_name:
            continue
        for body in sub.bRepBodies:
            for e in body.edges:
                g = e.geometry
                if g.curveType == adsk.core.Curve3DTypes.Circle3DCurveType and abs(g.radius - radius_mm * C.CM) < 1e-4:
                    return e
    raise KeyError('%s r=%s in %s' % (part_name, radius_mm, link_occ.name))


def ground_children_to_parents():
    """Fix every part to its link. Most nested occurrences came out with 'Ground to Parent'
    off; the joint solver then left them in place while their link moved."""
    # setting the flag reorders the collection, so gather first and verify afterwards
    comps = {o.component.name: o.component for o in C.design().rootComponent.occurrences}
    kids = [c.occurrences.item(i) for c in comps.values() for i in range(c.occurrences.count)]
    for c in kids:
        c.isGroundToParent = True
    left = [c.name for c in kids if not c.isGroundToParent]
    if left:
        raise RuntimeError('not grounded to parent: %s' % left)


def axis_circle(comp, x_mm, y_mm):
    """Sketch circle in the link's own x-y plane around a joint axis (link local z)."""
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'joint_axis_%d_%d' % (round(x_mm), round(y_mm))
    return sk.sketchCurves.sketchCircles.addByCenterRadius(
        adsk.core.Point3D.create(x_mm * C.CM, y_mm * C.CM, 0), 8.0 * C.CM)


def revolute(name, occ1, occ2, edge):
    root = C.design().rootComponent
    geo = adsk.fusion.JointGeometry.createByCurve(edge, adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
    ji = root.asBuiltJoints.createInput(occ1, occ2, geo)
    ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    j = root.asBuiltJoints.add(ji)
    j.name = name
    return j


def main():
    right = add_right_links()
    # joints exist only in parametric designs: everything built so far becomes base features
    C.design().designType = adsk.fusion.DesignTypes.ParametricDesignType
    torso = C.root_occ('torso:1')
    torso.isGrounded = True                    # driving a joint then moves the leg, not the torso
    ground_children_to_parents()
    left = {n: C.root_occ(n + ':1') for n in ('femur_link', 'tibia_link', 'wheel_link')}
    # A joint drives the occurrence that owns its geometry: an edge of a part inside a link
    # would move that part alone. The joint axes are therefore sketch circles drawn in the
    # link components themselves (femur: hip and knee, tibia: wheel axle).
    fem_c, tib_c = C.find_comp('femur_link'), C.find_comp('tibia_link')
    fem_hip = axis_circle(fem_c, 0.0, 60.0)
    fem_knee = axis_circle(fem_c, 0.0, -115.0)
    tib_axle = axis_circle(tib_c, 0.0, -115.0)
    for side, links in (('L', left), ('R', right)):
        f, t = links['femur_link'], links['tibia_link']
        revolute('hip_' + side, torso, f, fem_hip.createForAssemblyContext(f))
        revolute('knee_' + side, f, t, fem_knee.createForAssemblyContext(f))
        revolute('wheel_' + side, t, links['wheel_link'], tib_axle.createForAssemblyContext(t))
    for c in (fem_c, tib_c):
        for sk in c.sketches:
            sk.isVisible = False
    print('joints', C.design().rootComponent.asBuiltJoints.count)


def hip_edge(torso, side):
    """Ø30 hole edge of the left (+y) or right hip plate inside the torso."""
    want = 1.0 if side == 'L' else -1.0
    for sub in torso.childOccurrences:
        if sub.component.name != 'hip_plate_AKE50':
            continue
        for body in sub.bRepBodies:
            for e in body.edges:
                g = e.geometry
                if g.curveType == adsk.core.Curve3DTypes.Circle3DCurveType and abs(g.radius - 1.5) < 1e-4:
                    if g.center.x * want > 0:                 # Fusion X = robot y
                        return e
    raise KeyError('hip plate ' + side)
