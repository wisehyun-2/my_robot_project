"""Drive the as-built joints to test postures for collisions.

Leg length L0 (hip axis -> wheel axle) with the wheel kept under the hip (knee forward):
for L1 = L2 the femur turns by d(phi) and the knee by 2 d(phi). A positive hip_extra
swings the whole leg forward. Joint signs are found from the motion itself.
"""
import math

import adsk.core
import adsk.fusion

import cadlib as C
import check
import params as P


def joints():
    return {j.name: j for j in C.design().rootComponent.asBuiltJoints}


def wheel_center(side):
    occ = C.root_occ('wheel_link:%d' % (1 if side == 'L' else 2))
    m = occ.transform2
    t = m.translation
    return (t.z * 10, t.x * 10, t.y * 10)          # robot mm


def set_pose(L0, hip_extra_deg=0.0, signs=None):
    phi = math.acos(L0 / (2 * P.L1))
    d = phi - P.PHI
    js = joints()
    for side in ('L', 'R'):
        sh, sk = signs[side]
        js['hip_' + side].jointMotion.rotationValue = sh * (d + math.radians(hip_extra_deg))
        js['knee_' + side].jointMotion.rotationValue = sk * 2 * d


def find_signs():
    """Pick the joint signs that shorten the leg with the wheel staying under the hip."""
    out = {}
    for side in ('L', 'R'):
        best = None
        for sh in (1, -1):
            for sk in (1, -1):
                set_pose(260.0, 0.0, {side: (sh, sk), ('R' if side == 'L' else 'L'): (0, 0)})
                w = wheel_center(side)
                err = abs(w[0]) + abs(w[2] - (P.Z_HIP - 260.0))
                if best is None or err < best[0]:
                    best = (err, sh, sk)
        out[side] = best[1:]
    reset()
    return out


def reset():
    for j in joints().values():
        j.jointMotion.rotationValue = 0.0


def sweep(poses):
    signs = find_signs()
    report = ['signs %s' % signs]
    for (L0, hip) in poses:
        set_pose(L0, hip, signs)
        wl = wheel_center('L')
        hits = check.interference()
        report.append('L0 %.0f hip %+.0f deg: wheel L at x %.1f z %.1f -> %d interference(s)'
                      % (L0, hip, wl[0], wl[2], len(hits)))
        report += ['    ' + h for h in hits]
    reset()
    return '\n'.join(report)
