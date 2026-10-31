"""Shared state / camera / snapshot helpers for the wheel-output explode renders (scratch doc built by f_wx_build.py).
World axes: X = robot y (wheel axis), Y = robot z (up), Z = robot x (forward); Fusion units cm."""
import json
import math
import os
import tempfile

import adsk.core
import adsk.fusion

TAG = 'wheel_output_explode'
WORK = os.path.join(tempfile.gettempdir(), 'wbv2_wheel_assembly')   # raw captures / video frames (not in git)
OUT = os.path.join(WORK, 'raw')
FRAMES = os.path.join(WORK, 'frames')
AX_Z = 101.0                                  # wheel axis height (robot z, mm) = params.R_WHEEL (M365 tyre, 10-31)

# exploded (home) offsets along the wheel axis, robot mm (- = towards the robot centre)
HOME = dict(SHANK=0, MOTOR=55, RING=-30, S25=-45, PINS=-60, FLANGE=-88, INS=-108, WASH=-112, S316=-135,
            RIM=-215, TYRE=-310, RWASH=-380, S314=-420)
ORDER = list(HOME)
NAMES = dict(SHANK='정강이 (PETG, 아래쪽만)', MOTOR='AKE60-8 모터', RING='SUS304 클램프 링 2 t',
             S25='① M2.5 × 8 저두 × 6', INS='황동 인서트 M3 × 6', FLANGE='허브 플랜지 (PETG)',
             PINS='③ 평행핀 Ø3 × 10 × 3', WASH='평와셔 M3 × 6', S316='② M3 × 14 (12.9) × 6',
             RIM='한 조각 림 (PETG)', TYRE='M365 8.5 × 2 통고무 타이어', RWASH='평와셔 M3 × 6 (림)',
             S314='④ M3 × 14 (8.8) × 6')
# moves: (group, target) with target 0 (final place) or another group's name (ride on that group's current offset)
STEPS = [
    dict(key='s1', title='1단계  모터 → 정강이 포켓',
         text='AKE60-8을 정강이 바깥쪽에서 하우징 안으로 넣어 Ø42 파일럿을 Ø42.05 포켓(깊이 2.5)에 앉힌다.\n'
              '출력부는 Ø30 판 구멍 안쪽을 향한다.',
         moves=[[('MOTOR', 0)]], view=(0.85, 0.8, 0.45)),
    dict(key='s1b', title='2단계  SUS 클램프 링 + ① M2.5 × 8 × 6 → 모터 고정자',
         text='정강이 안쪽(바퀴 쪽) 면의 홈에 SUS 클램프 링을 넣고,\n① M2.5 × 8 저두 볼트 6개를 판을 지나 고정자 링 탭에 조인다 (물림 3.2 mm).',
         moves=[[('RING', 0)], [('S25', 0)]], spin=['S25'], view=(1.0, -0.6, 0.42)),
    dict(key='s2', title='3단계  허브 플랜지에 인서트 열압입 (작업대)',
         text='인두로 황동 인서트 M3 6개를 허브 플랜지 안쪽 면(Ø4.0 × 6 구멍)에 녹여 넣는다.',
         moves=[[('INS', 'FLANGE')]], bench=['FLANGE', 'INS'], view=(0.55, -0.8, 0.45)),
    dict(key='s3', title='4단계  허브 플랜지 → 모터 출력부',
         text='③ 평행핀 3개를 출력부 구멍에 꽂고, 플랜지를 Ø30 판 구멍으로 넣어 Ø11.8 스피곳을 맞춘 뒤\n'
              '평와셔 + ② M3 × 14 (12.9) 6개로 출력부에 조인다 (머리는 플랜지 안쪽 카운터보어 안, 림 허브 면에서 2.0 mm).',
         moves=[[('PINS', 0)], [('FLANGE', 0), ('INS', 0)], [('WASH', 0)], [('S316', 0)]], spin=['S316'],
         view=(0.8, -0.65, 0.4)),
    dict(key='s4', title='5단계  타이어 → 한 조각 림 (작업대)',
         text='타이어를 뜨거운 물(60~80 °C)에 데워 열린 쪽 플랜지(Ø150) 위로 굴려 넣고, 안쪽 구멍 24개를 림 돌기 24개에\n'
              '맞춘다 (안지름 Ø141 → 6.4 % 늘림, M365 순정 한 조각 림과 같은 방식). 토크는 돌기가 전한다.',
         moves=[[('TYRE', 'RIM')]], bench=['RIM', 'TYRE'], view=(0.7, -0.65, 0.4)),
    dict(key='s5', title='6단계  림 + 타이어 → 허브 플랜지, ④ M3 × 14 × 6',
         text='림 허브의 Ø11.2 홈을 허브 플랜지의 Ø11 등록 턱에 맞춰 밀어 넣고, 열린 쪽에서 평와셔 + ④ M3 × 14 (8.8)\n'
              '6개를 플랜지 인서트에 0.5 N·m로 조인다 (림 허브 9 mm 체결, 인서트 물림 4.4 mm, 나사 끝–구멍 바닥 1.5 mm).',
         moves=[[('RIM', 0), ('TYRE', 0)], [('RWASH', 0)], [('S314', 0)]], spin=['S314'],
         view=(0.55, -0.8, 0.35)),
]


VIEW0 = (1.0, -0.3, 0.38)                     # overview
VIEW_DONE = (0.85, -0.55, 0.42)
CUTS = [dict(key='cut1', a=-5.0, b=90.0, view=(0.62, -0.72, 0.6),
             title='단면 ①  ② M3 × 14 (-5°) · ④ M3 × 14 (90°) 자리를 지나는 단면'),
        dict(key='cut2', a=25.0, b=145.0, view=(0.12, -0.7, 0.9),
             title='단면 ②  평행핀 · M2.5 × 8 (25°, 145°) 자리를 지나는 단면')]


def states():
    """Per step: offsets before / after, the sub-moves {group: (from, to)}, and the groups a still shows."""
    off = dict(HOME)
    placed = {'SHANK'}
    out = []
    for st in STEPS:
        before = dict(off)
        seq = []
        for sub in st['moves']:
            mv = {}
            for g, tgt in sub:
                mv[g] = (off[g], off[tgt] if isinstance(tgt, str) else float(tgt))
            for g, (_a, b) in mv.items():
                off[g] = b
            seq.append(mv)
        gs = [g for sub in st['moves'] for g, _t in sub]
        vis = set(st['bench']) if st.get('bench') else placed | set(gs)
        placed |= {g for g in gs if off[g] == 0}
        out.append(dict(st, before=before, after=dict(off), seq=seq, vis=sorted(vis), groups=gs))
    return out


def grid_off():
    ui = adsk.core.Application.get().userInterface
    lc = adsk.core.ListControlDefinition.cast(ui.commandDefinitions.itemById('ViewLayoutGridOnCommand').controlDefinition)
    lc.listItems.item(lc.listItems.count - 1).isSelected = True      # radio list: [grid on, grid off]
    adsk.core.Application.get().activeViewport.refresh()
    return [(it.name, it.isSelected) for it in lc.listItems]


def doc():
    app = adsk.core.Application.get()
    for d in app.documents:
        dd = adsk.fusion.Design.cast(d.products.itemByProductType('DesignProductType'))
        if dd and dd.attributes.itemByName('wx', 'tag'):
            if app.activeDocument != d:
                d.activate()
            return d, dd
    raise RuntimeError('scratch doc missing: run f_wx_build.py')


def groups(dd):
    out = {}
    for o in dd.rootComponent.occurrences:
        g = o.component.attributes.itemByName('wx', 'group').value
        a = o.component.attributes.itemByName('wx', 'axis')
        ax = tuple(float(v) for v in a.value.split(',')) if a else None
        out.setdefault(g, []).append((o, ax))
    return out


def place(G, off, spin=None, visible=None):
    """off: group -> axial offset (robot mm), spin: group -> angle (rad) about each fastener's own axis."""
    spin = spin or {}
    for g, lst in G.items():
        d = off.get(g, 0.0)
        for o, ax in lst:
            m = adsk.core.Matrix3D.create()
            a = spin.get(g, 0.0)
            if a and ax is not None:
                m.setToRotation(a, adsk.core.Vector3D.create(1, 0, 0),
                                adsk.core.Point3D.create(0, ax[1] / 10, ax[0] / 10))
            t = m.translation
            t.x += d / 10
            m.translation = t
            o.transform2 = m
            if visible is not None:
                on = g in visible
                if o.isLightBulbOn != on:
                    o.isLightBulbOn = on


def robot_dir(v):
    n = math.sqrt(sum(c * c for c in v))
    x, y, z = (c / n for c in v)
    return (y, z, x)                           # robot -> world


def set_view(vdir, target=None, fit=True, dist=600.0):
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    d = robot_dir(vdir)
    tgt = target or (200.0, AX_Z, 0.0)          # world mm
    cam.target = adsk.core.Point3D.create(tgt[0] / 10, tgt[1] / 10, tgt[2] / 10)
    cam.eye = adsk.core.Point3D.create((tgt[0] + d[0] * dist) / 10, (tgt[1] + d[1] * dist) / 10,
                                       (tgt[2] + d[2] * dist) / 10)
    cam.upVector = adsk.core.Vector3D.create(0, 1, 0)
    cam.cameraType = adsk.core.CameraTypes.PerspectiveCameraType
    cam.isSmoothTransition = False
    cam.isFitView = fit
    vp.camera = cam
    if fit:
        vp.fit()
    vp.refresh()
    return vp.camera


def cam_dict():
    c = adsk.core.Application.get().activeViewport.camera
    return dict(eye=[c.eye.x, c.eye.y, c.eye.z], target=[c.target.x, c.target.y, c.target.z],
                up=[c.upVector.x, c.upVector.y, c.upVector.z], ext=c.viewExtents, fov=c.perspectiveAngle)


def set_cam(cd):
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    cam.cameraType = adsk.core.CameraTypes.PerspectiveCameraType
    cam.isSmoothTransition = False
    cam.isFitView = False
    cam.eye = adsk.core.Point3D.create(*cd['eye'])
    cam.target = adsk.core.Point3D.create(*cd['target'])
    cam.upVector = adsk.core.Vector3D.create(*cd['up'])
    cam.viewExtents = cd['ext']
    vp.camera = cam
    vp.refresh()


def recenter(target_mm, factor=1.0):
    """Keep the view direction, look at target (world mm), move the eye to factor x the distance."""
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    e, t = cam.eye, cam.target
    v = (e.x - t.x, e.y - t.y, e.z - t.z)
    cam.isSmoothTransition = False
    cam.isFitView = False
    tx, ty, tz = target_mm[0] / 10, target_mm[1] / 10, target_mm[2] / 10
    cam.target = adsk.core.Point3D.create(tx, ty, tz)
    cam.eye = adsk.core.Point3D.create(tx + v[0] * factor, ty + v[1] * factor, tz + v[2] * factor)
    cam.viewExtents = cam.viewExtents * factor
    vp.camera = cam
    vp.refresh()


def zoom(factor):
    vp = adsk.core.Application.get().activeViewport
    t = vp.camera.target
    recenter((t.x * 10, t.y * 10, t.z * 10), factor)


def project(G, groups_):
    """View-space pixel position of each visible group: bbox centre of a single part, of the topmost member for a
    set of fasteners (the centre of a bolt circle is empty)."""
    vp = adsk.core.Application.get().activeViewport
    out = {}
    for g in groups_:
        occs = [o for o, _ax in G[g] if o.isLightBulbOn]
        if not occs:
            continue
        if len(occs) > 1:
            occs = [max(occs, key=lambda o: o.boundingBox.maxPoint.y)]
        bb = occs[0].boundingBox
        c = adsk.core.Point3D.create((bb.minPoint.x + bb.maxPoint.x) / 2, (bb.minPoint.y + bb.maxPoint.y) / 2,
                                     (bb.minPoint.z + bb.maxPoint.z) / 2)
        p = vp.modelToViewSpace(c)
        out[g] = [p.x, p.y]
    return out


def project_pt(world_mm):
    vp = adsk.core.Application.get().activeViewport
    p = vp.modelToViewSpace(adsk.core.Point3D.create(world_mm[0] / 10, world_mm[1] / 10, world_mm[2] / 10))
    return [p.x, p.y]


def snap(name, scale=1):
    vp = adsk.core.Application.get().activeViewport
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, name)
    vp.saveAsImageFile(path, vp.width * scale, vp.height * scale)
    return path


def dump(name, data):
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, name), 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=1)
