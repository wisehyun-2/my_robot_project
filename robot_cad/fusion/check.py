"""Mass / centre-of-mass report and interference check of the assembled robot.

Purchased motors come as STEP models with many surface bodies, so their mass is
taken from the datasheet (AKE50-8 190 g, AKE60-8 260 g) and placed at the centre of
the motor body; every other body uses its Fusion material. Results in robot axes
(x forward, y left, z up, mm; origin on the floor below the hip midpoint).
"""
import adsk.core
import adsk.fusion

import cadlib as C
import params as P

MOTORS = {'AKE50-8_KV110': P.AKE50['mass'], 'AKE60-8_KV80': P.AKE60['mass']}


def f2r(p):
    """Fusion point (cm) -> robot (mm)."""
    return (p.z * 10.0, p.x * 10.0, p.y * 10.0)


def leaf_items(occ, out, path):
    """Collect (path, mass kg, com robot mm) for every body / motor below occ (proxies)."""
    name = occ.component.name
    if name in MOTORS:
        bb = occ.boundingBox
        c = adsk.core.Point3D.create((bb.minPoint.x + bb.maxPoint.x) / 2, (bb.minPoint.y + bb.maxPoint.y) / 2,
                                     (bb.minPoint.z + bb.maxPoint.z) / 2)
        out.append((path + '/' + occ.name, MOTORS[name], f2r(c)))
        return
    for b in occ.bRepBodies:
        if not b.isSolid:
            continue
        pp = b.physicalProperties
        out.append((path + '/' + occ.name + '/' + b.name, pp.mass, f2r(pp.centerOfMass)))
    for c in occ.childOccurrences:
        leaf_items(c, out, path + '/' + occ.name)


def com(items):
    m = sum(i[1] for i in items)
    c = tuple(sum(i[1] * i[2][k] for i in items) / m for k in range(3))
    return m, c


def report(verbose=False):
    root = C.design().rootComponent
    groups = {}
    allitems = []
    for occ in root.occurrences:
        if occ.component.name == '_library':
            continue
        items = []
        leaf_items(occ, items, '')
        groups[occ.name] = items
        allitems += items
    lines = []
    for g, items in groups.items():
        m, c = com(items)
        lines.append('%-14s %6.3f kg  COM (%7.1f, %7.1f, %7.1f)' % (g, m, c[0], c[1], c[2]))
    m, c = com(allitems)
    lines.append('%-14s %6.3f kg  COM (%7.1f, %7.1f, %7.1f)' % ('robot', m, c[0], c[1], c[2]))
    t = groups['torso:1']
    mt, ct = com(t)
    lines.append('torso COM relative to the hip axis midpoint: dx %.2f mm, dy %.2f mm, height %.1f mm'
                 % (ct[0], ct[1], ct[2] - P.Z_HIP))
    if verbose:
        for it in sorted(t, key=lambda i: -i[1]):
            lines.append('   %-70s %7.1f g (%6.1f, %6.1f, %6.1f)' % (it[0][-70:], it[1] * 1000, *it[2]))
    return '\n'.join(lines), ct


def interference(skip_motor_internal=True):
    d = C.design()
    root = d.rootComponent
    col = adsk.core.ObjectCollection.create()

    def walk(occ):
        if occ.component.name == '_library':
            return
        if occ.childOccurrences.count == 0 or occ.component.name in MOTORS:
            col.add(occ)
        else:
            for c in occ.childOccurrences:
                walk(c)
    for o in root.occurrences:
        walk(o)
    inp = d.createInterferenceInput(col)
    inp.areCoincidentFacesIncluded = False
    res = d.analyzeInterference(inp)
    lines = []
    for r in res:
        a, b = r.entityOne, r.entityTwo
        na = a.assemblyContext.name + '/' + a.name if hasattr(a, 'assemblyContext') and a.assemblyContext else a.name
        nb = b.assemblyContext.name + '/' + b.name if hasattr(b, 'assemblyContext') and b.assemblyContext else b.name
        v = r.interferenceBody.volume * 1000.0 if r.interferenceBody else -1
        lines.append('%-60s x %-60s %9.2f mm3' % (na[-60:], nb[-60:], v))
    return lines
