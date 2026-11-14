"""Joint frames of the nominal pose: motor (CubeMars STEP frame), moteus board (IC frame) per joint and side.

Left side (s = +1) frames are written directly; right side frames mirror the left vectors (y -> -y) and
then restore right-handedness, keeping the motor axis and lead direction (motor x is negated in z)
and the board normal and connector direction (board x is negated).  The motor hole patterns are
symmetric (+-30/90/150 and 0/60/120/180 deg) except the output dowels, which are always taken from the
real placement.
"""
import math

import cadlib as C
import params as P


def wheel_conn_deg():
    return P.WHEEL_CONN_DEG


JOINTS = {
    # joint: centre (x, z), ring face y (left), motor back direction along y (left), connector edge angle
    'hip': dict(c=(0.0, P.Z_HIP), ring=P.Y_RING_HIP, a=P.AXIS_HIP, conn=P.HIP_CONN_DEG),
    'knee': dict(c=(P.KNEE_X, P.KNEE_Z), ring=P.Y_RING_KNEE, a=P.AXIS_KNEE, conn=P.KNEE_CONN_DEG),
    'wheel': dict(c=(0.0, P.R_WHEEL), ring=P.Y_RING_WHEEL, a=P.AXIS_WHEEL, conn=wheel_conn_deg()),
}


def _left(joint):
    j = JOINTS[joint]
    ey = (0.0, float(j['a']), 0.0)
    dx, dz = P.theta_dir(j['conn'])
    bz = (dx, 0.0, dz)
    by = ey
    bx = C.cross(by, bz)
    psi = math.radians(P.LEAD_ANGLE)
    ex = C.add(C.mul(bx, math.cos(psi)), C.mul(bz, math.sin(psi)))
    ez = C.cross(ex, ey)
    o = (j['c'][0], j['ring'] - j['a'] * P.AKE['ring'], j['c'][1])
    ob = C.add(o, C.mul(ey, P.board_offset()))
    return dict(motor=C.frame(o, ex, ey, ez), board=C.frame(ob, bx, by, bz))


def frames(joint, s):
    """dict(motor=4x4, board=4x4, step_board=4x4) for side s (+1 left, -1 right)."""
    f = _left(joint)
    if s < 0:
        m, b = f['motor'], f['board']
        M = lambda v: (v[0], -v[1], v[2])
        f = dict(motor=C.frame(M(C.origin(m)), M(C.col(m, 0)), M(C.col(m, 1)), C.mul(M(C.col(m, 2)), -1)),
                 board=C.frame(M(C.origin(b)), C.mul(M(C.col(b, 0)), -1), M(C.col(b, 1)), M(C.col(b, 2))))
    f['step_board'] = C.mmul(f['board'], C.translate((0.0, 0.0, -P.MOTEUS['step_dz'])))
    return f


def motor_points(m, r, angles, y):
    """Points of a motor-local hole circle (radius r, angles in deg, at motor y) in robot coordinates."""
    out = []
    for a in angles:
        t = math.radians(a)
        out.append(C.apply(m, (r * math.cos(t), y, r * math.sin(t))))
    return out


def board_point(b, x, y, z):
    return C.apply(b, (x, y, z))


def axis_point(joint, s, y):
    j = JOINTS[joint]
    return (j['c'][0], s * y, j['c'][1])
