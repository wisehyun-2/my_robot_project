"""Step 1: new direct-modelling design, link components, imported motors and upkie links.

Component tree (left side; the right side is added by build_assembly.py):
    torso            frame = robot axes at the hip-axis midpoint (Fusion Y up)
    femur_link_L     frame = upkie upper_leg part frame
    tibia_link_L     frame = upkie lower_leg part frame
    wheel_link_L     frame = tibia frame moved to the wheel axle
"""
import math
import os

import adsk.core
import adsk.fusion

import cadlib as C
import params as P

HERE = os.path.dirname(os.path.abspath(__file__))
STEP_DIR = os.path.join(os.path.dirname(HERE), 'upkie_step')
AKE50_STEP = r'C:\Users\User\Desktop\ake50\AKE50-8 KV110.STEP'
AKE60_STEP = r'C:\Users\User\Desktop\ake60\AKE60-8-KV80.STEP'


# ----------------------------------------------------------------------------- frames (robot axes, mm)
def femur_frame():
    s, c = math.sin(P.PHI), math.cos(P.PHI)
    ey = (-s, 0.0, c)
    ez = (0.0, -1.0, 0.0)
    ex = C.cross(ey, ez)
    hip = (0.0, P.Y_FEMUR_IN, P.Z_HIP)                    # = local (0, 60, 131)
    o = C.sub(hip, C.add(C.mul(ey, 60.0), C.mul(ez, 131.0)))
    return C.frame(o, ex, ey, ez)


def tibia_frame():
    ey = C.unit((P.KNEE_X, 0.0, P.KNEE_Z - P.R_WHEEL))
    ez = (0.0, -1.0, 0.0)
    ex = C.cross(ey, ez)
    knee = (P.KNEE_X, P.Y_TIBIA_IN, P.KNEE_Z)              # = local (0, 60, 131)
    o = C.sub(knee, C.add(C.mul(ey, 60.0), C.mul(ez, 131.0)))
    return C.frame(o, ex, ey, ez)


def wheel_frame():
    t = tibia_frame()
    o = C.apply(t, (0.0, -115.0, 0.0))
    return [[t[0][0], t[0][1], t[0][2], o[0]],
            [t[1][0], t[1][1], t[1][2], o[1]],
            [t[2][0], t[2][1], t[2][2], o[2]],
            [0.0, 0.0, 0.0, 1.0]]


def torso_frame():
    return C.frame((0.0, 0.0, P.Z_HIP), (1, 0, 0), (0, 1, 0), (0, 0, 1))


# motor frames inside their parent link (parent coordinates, mm)
def hip_motor_in_torso():
    """robot axes relative to the hip midpoint; front (output) faces +y (outboard)."""
    oy = P.YW - P.HIP_PLATE_T + P.HIP_POCKET + P.AKE50['ring']      # ring face at YW-5
    return C.frame((0.0, oy, 0.0), (1, 0, 0), (0, -1, 0), (0, 0, -1))


def knee_motor_in_femur():
    ring_z = P.femur_z(P.Y_ADAPTER_IN + P.ADAPTER_POCKET)
    return C.frame((0.0, -115.0, ring_z - P.AKE60['ring']), (0, 1, 0), (0, 0, 1), (1, 0, 0))


def wheel_motor_in_tibia():
    ring_z = P.tibia_z(P.Y_TIBIA_IN - P.ANKLE_POCKET)
    return C.frame((0.0, -115.0, ring_z + P.AKE50['ring']), (-1, 0, 0), (0, 0, -1), (0, -1, 0))


def link_matrix(m_robot):
    """part-frame link placed with a robot-axes frame -> Fusion world Matrix3D."""
    return C.to_matrix3d(C.mmul(C.PERM, m_robot))


def torso_child_matrix(m_robot):
    """part placed inside the torso with a frame written in robot axes."""
    return C.to_matrix3d(C.mmul(C.PERM, m_robot))


# ----------------------------------------------------------------------------- build
def import_step(path, name):
    """Import a STEP file as a master component in _library (at the identity)."""
    lib = C.library().component
    im = C.app().importManager
    n0 = lib.occurrences.count
    im.importToTarget(im.createSTEPImportOptions(path), lib)
    occ = lib.occurrences.item(n0)
    occ.component.name = name
    return occ.component


LINK_FRAMES = {
    'torso:1': lambda: C.to_matrix3d(C.robot_to_fusion(torso_frame())),
    'femur_link:1': lambda: link_matrix(femur_frame()),
    'tibia_link:1': lambda: link_matrix(tibia_frame()),
    'wheel_link:1': lambda: link_matrix(wheel_frame()),
}


def place_links():
    for name, f in LINK_FRAMES.items():
        C.root_occ(name).transform2 = f()


def main(new_doc=True):
    app = C.app()
    if new_doc:
        app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    doc = app.activeDocument
    doc.name = 'WheelBiped_AKE'
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.DirectDesignType
    root = design.rootComponent
    if root.occurrences.count or root.bRepBodies.count:
        raise RuntimeError('active design is not empty')

    # masters live in _library at the identity; links are populated at the identity
    # and moved to the standing pose at the end (see cadlib.instance)
    C.library()
    torso = C.new_comp(root, 'torso')
    femur = C.new_comp(root, 'femur_link')
    tibia = C.new_comp(root, 'tibia_link')
    wheel = C.new_comp(root, 'wheel_link')

    ake50 = import_step(AKE50_STEP, 'AKE50-8_KV110')
    ake60 = import_step(AKE60_STEP, 'AKE60-8_KV80')
    upper = import_step(os.path.join(STEP_DIR, 'upper_leg.step'), 'upkie_upper_leg_AKE')
    lower = import_step(os.path.join(STEP_DIR, 'lower_leg.step'), 'upkie_lower_leg_AKE')

    C.instance(torso, ake50, torso_child_matrix(hip_motor_in_torso()), 'hip_motor')
    C.instance(femur, upper, adsk.core.Matrix3D.create())
    C.instance(femur, ake60, C.to_matrix3d(knee_motor_in_femur()), 'knee_motor')
    C.instance(tibia, lower, adsk.core.Matrix3D.create())
    C.instance(tibia, ake50, C.to_matrix3d(wheel_motor_in_tibia()), 'wheel_motor')

    place_links()
    app.activeViewport.fit()
    for occ in root.occurrences:
        print(occ.name, [round(v, 2) for v in occ.transform2.translation.asArray()])
