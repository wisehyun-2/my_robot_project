"""Step 5: part numbers, images and exports of the finished design.

    set_part_numbers()  STEP product names = component names (Fusion defaults to time stamps)
    images()            PNG views into robot_cad/images
    exports()           Fusion archive, assembly STEP and per-part STEP / STL into robot_cad/export

The hidden component _library holds the first (master) occurrence of every reused part
and must stay: it owns those component definitions, so deleting it deletes every
instance (also when the masters are moved elsewhere first, in a parametric design).
Hidden occurrences are not written to STEP, so it does not show up in the exports.
"""
import os

import adsk.core
import adsk.fusion

import cadlib as C
import pose_test
import view

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.dirname(HERE)
IMG = os.path.join(BASE, 'images')
EXP = os.path.join(BASE, 'export')

# designed / modified parts that get their own files (component name -> how it is made)
PARTS = {
    'upkie_upper_leg_AKE': 'print PA-CF (upkie_parts 2.0 upper_leg, hip boss for AKE50)',
    'upkie_lower_leg_AKE': 'print PA-CF (upkie_parts 2.0 lower_leg, knee boss for AKE60)',
    'hip_plate_AKE50': 'CNC 6061-T6, 8 mm',
    'knee_adapter_AKE60': 'CNC 6061-T6, 8 mm',
    'ankle_plate_AKE50': 'CNC 6061-T6, 8 mm',
    'wheel_hub_AKE50': 'CNC 6061-T6 (lathe)',
    'wheel_rim_150': 'print PA-CF',
    'tyre_150_TPU': 'print TPU 95A (or cast PU 85A)',
    'knee_cover': 'print PA-CF',
    'wheel_cover': 'print PA-CF',
    'hip_cage_c1': 'print PA-CF',
    'spine_tube_16x1.5': '6061 tube 16 x 1.5, 340 mm',
    'torso_shell': 'print PETG/ASA (6 halves), handle PA-CF, visor 2 mm dark acrylic',
    'battery_tray': 'print PA-CF',
    'electronics_sled': 'print PA-CF',
}


def set_part_numbers():
    d = C.design()
    d.rootComponent.partNumber = 'WheelBiped_AKE'
    for c in d.allComponents:
        if c != d.rootComponent:
            c.partNumber = c.name
            if c.name in PARTS:
                c.description = PARTS[c.name]
    for o in d.rootComponent.occurrences:
        if o.component.name == '_library':
            o.isLightBulbOn = False


def show_all(show_joints=False):
    root = C.design().rootComponent
    for o in root.occurrences:
        o.isLightBulbOn = o.component.name != '_library'
    root.isJointsFolderLightBulbOn = show_joints
    for c in C.design().allComponents:
        for sk in c.sketches:
            sk.isVisible = False


def snap(name, eye, target, w=1600, h=1200, ortho=False, up=(0, 0, 1)):
    os.makedirs(IMG, exist_ok=True)
    view.look(eye, target, up=up, ortho=ortho)
    path = os.path.join(IMG, name + '.png')
    C.app().activeViewport.saveAsImageFile(path, w, h)
    return path


def set_visible(names, on):
    """Show or hide parts by component name anywhere in the tree (bodies of torso_shell by name)."""
    root = C.design().rootComponent
    for o in root.allOccurrences:
        if o.component.name in names:
            o.isLightBulbOn = on
    shell = C.find_comp('torso_shell')
    for b in shell.bRepBodies:
        if b.name in names:
            b.isLightBulbOn = on


def images():
    show_all()
    out = []
    c = (0, 0, 280)
    out.append(snap('01_iso_front_left', (1100, 800, 750), c))
    out.append(snap('02_iso_back_right', (-1000, -850, 700), c))
    out.append(snap('03_front', (2000, 0, 280), c, ortho=True))
    out.append(snap('04_side_left', (0, 2000, 280), c, ortho=True))
    out.append(snap('05_top', (0, 0, 2500), c, ortho=True, up=(1, 0, 0)))
    out.append(snap('06_leg_outboard', (250, 900, 260), (40, 190, 220)))
    out.append(snap('07_leg_inboard', (380, -300, 300), (40, 180, 210)))
    hide = ['lid_L', 'lid_R', 'shell_lower_L', 'handle', 'visor', 'led_bar', 'bottom_cover_L', 'accent_ring_L']
    set_visible(hide, False)
    out.append(snap('08_torso_inside', (650, 750, 750), (0, 0, 400)))
    set_visible(hide, True)
    signs = pose_test.find_signs()
    pose_test.set_pose(240.0, 0.0, signs)
    out.append(snap('09_crouch_L0_240', (1100, 800, 700), (0, 0, 230)))
    pose_test.set_pose(335.0, 0.0, signs)
    out.append(snap('10_stretch_L0_335', (1100, 800, 750), (0, 0, 280)))
    pose_test.reset()
    view.look((1100, 800, 750), c)
    return out


def exports():
    d = C.design()
    em = d.exportManager
    os.makedirs(os.path.join(EXP, 'parts_step'), exist_ok=True)
    os.makedirs(os.path.join(EXP, 'parts_stl'), exist_ok=True)
    done = []
    f3d = os.path.join(EXP, 'WheelBiped_AKE.f3d')
    em.execute(em.createFusionArchiveExportOptions(f3d))
    done.append(f3d)
    step = os.path.join(EXP, 'WheelBiped_AKE_assembly.step')
    em.execute(em.createSTEPExportOptions(step, d.rootComponent))
    done.append(step)
    for name in PARTS:
        comp = C.find_comp(name)
        p = os.path.join(EXP, 'parts_step', name + '.step')
        em.execute(em.createSTEPExportOptions(p, comp))
        done.append(p)
        for b in comp.bRepBodies:
            if not b.isSolid:
                continue
            fn = name if comp.bRepBodies.count == 1 else name + '__' + b.name
            s = os.path.join(EXP, 'parts_stl', fn + '.stl')
            o = em.createSTLExportOptions(b, s)
            o.isBinaryFormat = True
            o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
            em.execute(o)
            done.append(s)
    return done
