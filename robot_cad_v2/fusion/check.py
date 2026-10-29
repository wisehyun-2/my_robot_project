"""Mass / centre of mass / inertia report, interference and minimum-clearance checks.

Purchased STEP models (AKE60 = many surface bodies, moteus, pi3hat) get their datasheet mass at the
centre of their bounding box; every other body uses its Fusion material.  Results are in robot axes
(x forward, y left, z up, mm; origin on the floor below the hip midpoint in the nominal pose).
Physical properties read in the same script that created the bodies came out stale in v1: run
the report in a separate script.
"""
import math

import adsk.core
import adsk.fusion

import cadlib as C
import params as P

LUMPED = {   # component name: (mass kg (datasheet), cylinder radius / length about robot y for the inertia, mm)
    'AKE60-8_KV80_stator': (P.AKE['mass'] - P.AKE['out_mass'], 34.5, 25.0),
    'AKE60-8_KV80_output': (P.AKE['out_mass'], 13.5, 6.0),
    'moteus_r4.11': (P.MOTEUS['mass'], 0.0, 0.0),
    'pi3hat_r4.5': (0.031, 0.0, 0.0),
    'raspberry_pi_4b': (0.046, 0.0, 0.0),
    'mjpower_ss': (0.034, 0.0, 0.0),
}


def lumped_inertia(m, r, h):
    """Solid cylinder about robot y (kg mm^2, robot axes)."""
    ia = 0.5 * m * r * r
    idm = m * (3 * r * r + h * h) / 12.0
    return [[idm, 0.0, 0.0], [0.0, ia, 0.0], [0.0, 0.0, idm]]


SKIN = 1.6                  # 8 top / bottom layers of 0.2 mm
PRINT_NORMAL = {            # build direction of parts printed lying on an edge (robot x, z): faces facing it are skins
    'thigh': lambda: _beam_normal((0.0, P.Z_HIP), (P.KNEE_X, P.KNEE_Z)),
    'shank': lambda: _beam_normal((P.KNEE_X, P.KNEE_Z), (0.0, P.R_WHEEL)),
}


def _beam_normal(a, b):
    ux, uz = b[0] - a[0], b[1] - a[1]
    L = math.hypot(ux, uz)
    return C.rf((-uz / L, 0.0, ux / L))           # Fusion axes


def skin_area(body, normal):
    """Surface (cm2) facing +-normal (|cos| > 0.7): each face's display mesh is split by triangle normal and the face
    area shared out by mesh area, so a curved face counts the part of it that faces the build direction (until 10-30
    one normal per face decided whole cylinders: thigh 480 g instead of 424, and the r 37 can cavity flipped +15.6 g
    when the lead groove changed)."""
    tot = 0.0
    for f in body.faces:
        mc = f.meshManager.createMeshCalculator()
        mc.setQuality(adsk.fusion.TriangleMeshQualityOptions.NormalQualityTriangleMesh)
        m = mc.calculate()
        xyz, idx = m.nodeCoordinatesAsDouble, m.nodeIndices
        on = all_ = 0.0
        for k in range(0, len(idx), 3):
            a, b, c = 3 * idx[k], 3 * idx[k + 1], 3 * idx[k + 2]
            n = C.cross((xyz[b] - xyz[a], xyz[b + 1] - xyz[a + 1], xyz[b + 2] - xyz[a + 2]),
                        (xyz[c] - xyz[a], xyz[c + 1] - xyz[a + 1], xyz[c + 2] - xyz[a + 2]))
            n2 = math.sqrt(n[0] ** 2 + n[1] ** 2 + n[2] ** 2)
            all_ += n2
            if abs(n[0] * normal[0] + n[1] * normal[1] + n[2] * normal[2]) > 0.7 * n2:
                on += n2
        if all_ > 0:
            tot += f.area * on / all_
    return tot


def print_fraction(body, t_wall, infill, normal=None):
    """Printed / solid mass of a PETG body: walls of thickness t_wall solid, the core at 'infill'.  With the build
    direction given (Fusion axes), surface facing it gets SKIN instead of t_wall (top / bottom layers, skin_area)."""
    V = body.volume                    # cm3
    A = body.area                      # cm2
    if normal is None:
        shell = A * t_wall * 0.1
    else:
        skin = skin_area(body, normal)
        shell = (A - skin) * t_wall * 0.1 + skin * SKIN * 0.1
    shell = min(V, shell)
    return (shell + infill * (V - shell)) / V


def apply_print_mass(wheel_kg=P.WHEEL_MASS):
    """Give every printed body a PETG density scaled to its print settings (params.PRINT) and the bought RC
    wheel its real mass (the model's solid rubber tyre would weigh 0.7 kg).  Run after a build."""
    out = []
    for o in leaves():
        n = o.component.name
        prof = next((v for k, v in P.PRINT.items() if n.startswith(k)), None)
        if prof is not None:
            nrm = next((fn() for k, fn in PRINT_NORMAL.items() if n.startswith(k)), None)
            for b in o.bRepBodies:
                if b.isSolid:
                    f = print_fraction(b, *prof, normal=nrm)
                    b.material = C.custom_density_material('PETG printed %d%%' % round(100 * f), P.PETG_RHO * f)
                    out.append((n, round(f, 2)))
        elif n.startswith('RC wheel'):
            vol = sum(b.volume for b in o.bRepBodies if b.isSolid)
            for b in o.bRepBodies:
                b.material = C.custom_density_material('RC wheel %.0f g' % (wheel_kg * 1000), wheel_kg / (vol * 1e-6))
    return out


def f2r(p):
    """Fusion point (cm) -> robot (mm)."""
    return (p.z * 10.0, p.x * 10.0, p.y * 10.0)


def leaves(occ=None, out=None):
    """Leaf occurrences (and lumped purchased parts) below occ, _library excluded."""
    if out is None:
        out = []
    kids = C.design().rootComponent.occurrences if occ is None else occ.childOccurrences
    for o in kids:
        if o.component.name == '_library' or not o.isLightBulbOn:
            continue
        if o.component.name in LUMPED or o.childOccurrences.count == 0:
            out.append(o)
        else:
            leaves(o, out)
    return out


def items(occ):
    """(path, mass kg, com robot mm, inertia about com (kg mm^2, robot axes 3x3)) of the bodies below occ."""
    out = []
    for o in leaves(occ) if occ.childOccurrences.count else [occ]:
        name = o.component.name
        if name in LUMPED:
            bb = o.boundingBox
            c = adsk.core.Point3D.create((bb.minPoint.x + bb.maxPoint.x) / 2, (bb.minPoint.y + bb.maxPoint.y) / 2,
                                         (bb.minPoint.z + bb.maxPoint.z) / 2)
            m, r, h = LUMPED[name]
            out.append((o.fullPathName, m, f2r(c), lumped_inertia(m, r, h)))
            continue
        for b in o.bRepBodies:
            if not b.isSolid or not b.isLightBulbOn:
                continue
            pp = b.physicalProperties
            out.append((o.fullPathName + '/' + b.name, pp.mass, f2r(pp.centerOfMass), _inertia(pp)))
    return out


def _inertia(pp):
    """Inertia about the body's COM in robot axes, kg mm^2."""
    ok, xx, yy, zz, xy, yz, xz = pp.getXYZMomentsOfInertia()        # kg cm^2 about the world origin, Fusion axes
    m = pp.mass
    c = pp.centerOfMass
    cx, cy, cz = c.x, c.y, c.z
    # parallel axis back to the COM (Fusion axes, cm)
    Ixx = xx - m * (cy * cy + cz * cz)
    Iyy = yy - m * (cx * cx + cz * cz)
    Izz = zz - m * (cx * cx + cy * cy)
    Ixy = xy + m * cx * cy
    Iyz = yz + m * cy * cz
    Ixz = xz + m * cx * cz
    F = [[Ixx, -Ixy, -Ixz], [-Ixy, Iyy, -Iyz], [-Ixz, -Iyz, Izz]]
    # Fusion (X, Y, Z) = robot (y, z, x): robot index i -> Fusion index perm[i]
    perm = (2, 0, 1)
    return [[F[perm[i]][perm[j]] * 100.0 for j in range(3)] for i in range(3)]


def combine(its):
    m = sum(i[1] for i in its)
    c = tuple(sum(i[1] * i[2][k] for i in its) / m for k in range(3))
    I = [[0.0] * 3 for _ in range(3)]
    for (_n, mi, ci, Ii) in its:
        d = (ci[0] - c[0], ci[1] - c[1], ci[2] - c[2])
        dd = d[0] ** 2 + d[1] ** 2 + d[2] ** 2
        for a in range(3):
            for b in range(3):
                I[a][b] += (Ii[a][b] if Ii else 0.0) + mi * ((dd if a == b else 0.0) - d[a] * d[b])
    return m, c, I


def report():
    """Mass, COM and inertia per link and for the robot (nominal pose).  Returns (text, dict)."""
    root = C.design().rootComponent
    groups, allitems = {}, []
    for occ in root.occurrences:
        if occ.component.name in ('_library',) or not occ.isLightBulbOn:
            continue
        its = items(occ)
        groups[occ.component.name] = its
        allitems += its
    data = {}
    lines = []
    for g, its in groups.items():
        m, c, I = combine(its)
        data[g] = dict(mass=m, com=c, inertia=I)
        lines.append('%-9s %6.3f kg  COM (%7.1f, %7.1f, %7.1f) mm  Ixx %6.0f Iyy %6.0f Izz %6.0f kg mm2'
                     % (g, m, c[0], c[1], c[2], I[0][0], I[1][1], I[2][2]))
    m, c, I = combine(allitems)
    data['robot'] = dict(mass=m, com=c, inertia=I)
    lines.append('%-9s %6.3f kg  COM (%7.1f, %7.1f, %7.1f) mm' % ('robot', m, c[0], c[1], c[2]))
    t = data['torso']
    lines.append('torso COM relative to the hip axis midpoint: dx %.2f mm, dy %.2f mm, height %.1f mm'
                 % (t['com'][0], t['com'][1], t['com'][2] - P.Z_HIP))
    return '\n'.join(lines), data, groups


def save_mass_json(path):
    """Per-link mass properties (nominal pose, robot axes, mm / kg / kg mm^2) for wbr/bench/ake_cad.py."""
    import json
    txt, data, _g = report()
    out = dict(units='mm, kg, kg mm^2 about the COM, robot axes (x fwd, y left, z up, floor origin)',
               pose=dict(L0=P.L0, L1=P.L1, L2=P.L2, R_wheel=P.R_WHEEL, Z_hip=P.Z_HIP, phi_deg=math.degrees(P.PHI),
                         knee=(P.KNEE_X, P.KNEE_Z), top_plate_z=P.Z_HIP + P.RAIL_Z + P.RAIL_S / 2 + P.TOP_PLATE[2],
                         tyre_centre_y=P.tyre_centre_y(), wheel_offset=P.wheel_offset()),
               links={k: dict(mass=v['mass'], com=list(v['com']), inertia=v['inertia']) for k, v in data.items()})
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(out, f, indent=1)
    return txt


def suggest_battery_offset():
    """Battery position that puts the torso COM over the hip-axis midpoint (the tray follows in x)."""
    _t, data, groups = report()
    mt, ct = data['torso']['mass'], data['torso']['com']
    tray = sum(i[1] for i in groups['torso'] if 'battery_tray' in i[0])
    bx = P.BATTERY_X - ct[0] * mt / (P.BATTERY_MASS + tray)
    by = P.BATTERY_Y - ct[1] * mt / P.BATTERY_MASS
    return ct, bx, by


def interference(occs=None, coincident=False):
    d = C.design()
    col = adsk.core.ObjectCollection.create()
    for o in (occs or leaves()):
        col.add(o)
    inp = d.createInterferenceInput(col)
    inp.areCoincidentFacesIncluded = coincident
    res = d.analyzeInterference(inp)
    out = []
    for r in res:
        a, b = r.entityOne, r.entityTwo
        na = (a.assemblyContext.fullPathName + '/' if a.assemblyContext else '') + a.name
        nb = (b.assemblyContext.fullPathName + '/' if b.assemblyContext else '') + b.name
        v = r.interferenceBody.volume * 1000.0 if r.interferenceBody else -1
        out.append((na, nb, v))
    return out


CONNECTOR_BODIES = ('housing', 'XT60', 'XT90-F', 'fan', 'pack', 'switch', 'cap', 'body')
RECEPTACLES = ('XT30-F x6', 'XT90-M', 'PH-4 switch')      # mjpower-ss connectors a plug is pushed into
BOARD_BODY = '본체1'          # moteus / pi3hat STEP body name (their connectors are solid blocks)


def classify(res):
    """Split interference results into (mating, real).  Mating: a wire ending inside its plug / the pack /
    the fan / the switch, wires of one bundle touching, plugs / terminator / cap inside a moteus header."""
    mating, real = [], []
    cable = lambda n: '_'.join(n.split('_')[:-2])          # harness_<cable>_<run|loop>_<wire>
    for (a, b, v) in res:
        na, nb = a.split('/')[-1], b.split('/')[-1]
        ha, hb = na.startswith('harness'), nb.startswith('harness')
        if ha and hb and cable(na) == cable(nb):
            mating.append((a, b, v))
        elif (ha and nb in CONNECTOR_BODIES) or (hb and na in CONNECTOR_BODIES):
            mating.append((a, b, v))
        elif BOARD_BODY in (na, nb) and ({na, nb} & {'housing', 'body', 'cap'}):
            mating.append((a, b, v))
        elif ({na, nb} & set(RECEPTACLES)) and ({na, nb} & {'housing', 'XT90-F'}):
            mating.append((a, b, v))
        else:
            real.append((a, b, v))
    return mating, real


def min_distance(a, b):
    """Minimum distance (mm) between two entities (occurrences or body proxies)."""
    r = C.app().measureManager.measureMinimumDistance(a, b)
    return r.value * 10.0
