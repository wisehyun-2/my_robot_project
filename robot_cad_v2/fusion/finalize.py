"""Finishing steps of the v2 model: part numbers, images, BOM, as-built joints and exports.

Run each step in its own MCP script after v2run.load() (a failed script rolls the whole step back):
    finalize.set_part_numbers()
    finalize.images()        # direct design, poses set with poses.apply
    finalize.bom()           # ../BOM.csv, ../BOM.md (purchased parts counted in the model, fasteners by rule)
    finalize.joints()        # switches the design to parametric (assembly.reset() switches back for a rebuild)
    finalize.exports()       # ../export: f3d (with the joints), assembly STEP, designed parts STEP + STL

_library holds the master of every reused part and must stay (it owns those components); it is hidden, and
hidden occurrences are not written to STEP.
"""
import csv
import math
import os

import adsk.core
import adsk.fusion

import cadlib as C
import layout as LY
import legs as LG
import params as P
import poses as PO
import view

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.dirname(HERE)
IMG = os.path.join(BASE, 'images')
EXP = os.path.join(BASE, 'export')

# designed parts: component name prefix -> (material / process, how to make it)
PRINT_NOTE = {
    'thigh': 'PETG, 무릎 바깥쪽 모서리(+x, 배선 홈 반대)를 바닥에(하우징 아래 서포트), 벽 12겹(4.8 mm) · 위아래 8층 · '
             '자이로이드 20 %, 보스 주변 100 % 모디파이어',
    'shank': 'PETG, 무릎 바깥쪽 모서리(+x, 배선 홈 반대)를 바닥에(하우징 아래 서포트), 벽 12겹(4.8 mm) · 위아래 8층 · '
             '자이로이드 20 %, '
             '보스 주변 100 % 모디파이어',
    'hub_flange': 'PETG 100 %, 모터 쪽 면을 바닥에',
    'hex_hub': 'PETG 100 %, 플랜지 면을 바닥에(육각 위)',
    'cup_': 'PETG 30 %, 바닥(덮개 면)을 바닥에, 서포트는 커넥터 창만',
    'cable_clips': 'PETG 40 %',
    'top_plate': 'PETG 25 %, M4 열압입 인서트',
    'battery_tray': 'PETG 30 %, 리브를 위로',
    'electronics_deck': 'PETG 30 %',
    'shell_': 'PETG 진회색 25 %, 바깥 면을 위로(서포트 필요), 앞·hip·뒤 × 좌우 6조각',
    'saddle_': 'PETG 파랑 25 % (hip 위 안장)',
    'drum_ring': 'PETG 검정, 흰 빛 링·파란 호 홈',
    'deco_': 'PETG 얇은 장식판(색: 이름·그림 참조), 순간접착제로 붙임',
    'splice_tabs': 'PETG 100 %, 셸 조각 이음판(M3 + 너트)',
}
MACHINED = {
    'hip_plate': '6061-T6 판 10t 이상(완성 9.5 mm) 3축 MCT 양면, Ø42 H7, 보스 면 Ø63 × 0.5 홈, M5·M4 90° 접시, M3 탭 11개, 검정 아노다이징 '
                 '(manufacturing/ 도면 WBv2-M01)',
    'spine_tube': '6061-T6 환봉 Ø16(통짜), 길이 %.0f mm, 양 끝 면삭 + M5 탭 깊이 %.0f'
                  % (2 * (P.YW - P.HIP_PLATE_T), P.END_TAP['M5'][1]),
    'rail': '6061-T6 각봉 15 × 15(통짜), 길이 %.0f mm, 양 끝 M4 탭 깊이 %.0f, Ø4.4 관통 4곳'
            % (2 * (P.YW - P.HIP_PLATE_T), P.END_TAP['M4'][1]),
    'clamp ring output': 'SUS304 %g mm 레이저 가공, Ø%g/Ø%g, Ø3.4 × 6 (PCD %g)' % ((P.RING_T,) + P.RING_OUT + (P.AKE_PCD_OUT,)),
    'clamp ring stator': 'SUS304 %g mm 레이저 가공, Ø%g/Ø%g, Ø2.8 × 6 (PCD %g)' % ((P.RING_T,) + P.RING_STATOR + (P.AKE_PCD_RING,)),
    'magnet spacer': '6061(비자성) 선반 가공, Ø9 × %g, 위 Ø%.2f × %g 자석 포켓, 아래 Ø%.2f × %g 스피곳'
                     % ((P.SPACER_H,) + P.SPACER_POCKET + P.SPACER_SPIGOT),
}
PURCHASED = {   # component name prefix -> (BOM name, spec / source)
    'AKE60-8_KV80_stator': ('CubeMars AKE60-8 KV80', '8:1, 12.5 / 5 N·m, 0.26 kg, Ø69 × 25'),
    'moteus_r4.11': ('mjbots moteus r4.11', 'XT30PW ×2, JST PH-3 CAN ×2'),
    'pi3hat_r4.5': ('mjbots pi3hat r4.5', 'CAN-FD ×5, IMU, 24 V → 5 V'),
    'raspberry_pi_4b': ('Raspberry Pi 4 Model B', '4 GB 이상'),
    'mjpower_ss': ('mjbots power dist (mjpower-ss)', 'XT90 입력, XT30 출력 ×6, 프리차지·스위치'),
    'battery_24V_15Ah': ('배터리 24 V 15 Ah', '75 × 310 × 75 mm, XT60(F) 리드'),
    'RC wheel': ('1/8 트러기 RC 휠·타이어(접착 완제품)', 'Ø150 × 60, 17 mm 육각, 약 250 g'),
    'M5 serrated wheel nut': ('M5 세레이션 플랜지 휠 너트', '1/8 RC용'),
    'XT30U plug': ('XT30U 플러그(16 AWG)', '암·수는 README 배선표'),
    'JST PHR-3': ('JST PHR-3 하우징 + SPH-002T-P0.5S 단자 3개', 'CAN 케이블 끝'),
    'CAN terminator': ('CAN-FD 종단 저항 120 Ω (PH-3)', '20 × 15 × 10 이하, 바퀴 드라이버 CAN1'),
    'XT30 cap': ('XT30 보호 캡', '바퀴 드라이버 J2(24 V 살아 있음)'),
    'XT90-F to XT60-M': ('XT90(F) → XT60(M) 변환 케이블', '배터리 → 전원보드'),
    'XT60 pair': ('XT60 커넥터 쌍', '배터리 리드 ↔ 변환기'),
    'fan 30x30x7': ('팬 30 × 30 × 7 mm 5 V', 'pi3hat JP6(1번 5 V, 3번 GND), 데크에 1.5 mm 양면 폼테이프'),
    'rocker switch': ('로커 스위치 19 × 13 (mjbots PH4 스위치 하네스)', '몸통 뒷면 수직면 로봇 왼쪽(+y), 바깥에서 조작'),
    'standoff M2.5x11': ('M2.5 × 11 육각 스탠드오프(수-암)', 'pi3hat 동봉'),
}


def design_parts():
    """{component name: (kind, note)} of the designed parts present in the model."""
    out = {}
    for c in C.design().allComponents:
        n = c.name
        for k, v in PRINT_NOTE.items():
            if n.startswith(k):
                out[n] = ('3D 프린트', v)
        for k, v in MACHINED.items():
            if n.startswith(k):
                out[n] = ('가공', v)
    return out


def set_part_numbers():
    d = C.design()
    d.rootComponent.partNumber = 'WheelBiped_v2'
    dp = design_parts()
    for c in d.allComponents:
        if c != d.rootComponent:
            c.partNumber = c.name
            if c.name in dp:
                c.description = dp[c.name][1]
    for o in d.rootComponent.occurrences:
        if o.component.name == '_library':
            o.isLightBulbOn = False


# ----------------------------------------------------------------------------- images
def show_all():
    root = C.design().rootComponent
    for o in root.occurrences:
        o.isLightBulbOn = o.component.name != '_library'
    for o in root.allOccurrences:              # the masters too (hidden by _library): new instances copy their bulb
        if o.component.name != '_library':
            o.isLightBulbOn = True
    root.isJointsFolderLightBulbOn = False


def set_visible(prefixes, on):
    for o in C.design().rootComponent.allOccurrences:
        if o.component.name.startswith(tuple(prefixes)):
            o.isLightBulbOn = on


SHELL = ('shell_', 'saddle_', 'deco_', 'drum_ring', 'splice_tabs')


def snap(name, eye, target, **kw):
    return view.snap(os.path.join(IMG, name + '.png'), eye, target, **kw)


def _lift(dz):
    """Move the whole robot (every root occurrence) by dz mm along robot z, e.g. to put a crouched pose's
    wheels back on the floor for a picture; _lift(-dz) moves it back."""
    p = C.rf((0.0, 0.0, dz))
    for o in C.design().rootComponent.occurrences:
        if o.component.name == '_library':
            continue
        m = o.transform2
        v = m.translation
        m.translation = adsk.core.Vector3D.create(v.x + p[0] * C.CM, v.y + p[1] * C.CM, v.z + p[2] * C.CM)
        o.transform2 = m


def images():
    show_all()
    PO.reset()
    c = (0, 0, 290)
    # close-ups (fit=False) keep the zoom of the previous perspective view, so they follow a fitted perspective one
    out = [snap('01_iso_front_left', (1100, 800, 750), c),
           snap('07_leg_outboard_left', (330, 760, 300), (40, 200, 210), fit=False),
           snap('02_iso_back_right', (-1000, -850, 700), c),
           snap('08_leg_inboard_front_left', (520, -120, 120), (40, 200, 200), fit=False),
           snap('03_front', (2000, 0, 290), c, ortho=True),
           snap('04_side_left', (0, 2000, 290), c, ortho=True),
           snap('05_back', (-2000, 0, 290), c, ortho=True),
           snap('06_top', (0, 0, 2500), c, ortho=True, up=(1, 0, 0))]
    set_visible(SHELL + ('top_plate',), False)
    out.append(snap('09_torso_inside', (650, 750, 760), (0, 0, 400)))
    out.append(snap('10_torso_inside_back', (-650, -700, 700), (0, 0, 380)))
    set_visible(SHELL + ('top_plate',), True)
    set_visible(('cup_knee_L',), False)
    out.append(snap('11_knee_driver_left_inboard', (170, -40, 150), (85, 175, 230), fit=False))
    set_visible(('cup_knee_L',), True)
    set_visible(('cup_wheel_L',), False)
    out.append(snap('12_wheel_driver_left_outboard', (-110, 500, 120), (-10, 245, 95), fit=False))
    set_visible(('cup_wheel_L',), True)
    for (L0, hip), nm, eye, ortho in (((200.0, 0.0), '13_crouch_L0_200', (1100, 800, 700), False),
                                      ((335.0, 0.0), '14_stretch_L0_335', (1100, 800, 700), False),
                                      ((P.L0, -80.0), '15_hip_minus80', (0, 2000, 290), True),
                                      ((P.L0, 60.0), '16_hip_plus60', (0, 2000, 290), True)):
        PO.show_pose(L0, hip)                     # cable loops redrawn for the pose
        dz = L0 - P.L0 if hip == 0.0 else 0.0     # wheels back on the floor
        _lift(dz)
        out.append(snap(nm, eye, (0, 0, 250), ortho=ortho))
        _lift(-dz)
    PO.clear_pose()
    view.look((1100, 800, 750), c)
    return out


def inside_images():
    """Views of the inside: shell and payload plate hidden, then layers peeled off (rails, battery, tray)."""
    show_all()
    c = (0, 0, 420)
    out = []
    set_visible(SHELL + ('top_plate',), False)
    out.append(snap('17_inside_front_top', (700, 500, 900), c))
    out.append(snap('18_inside_hip_left_from_front', (250, 30, 330), (0, 150, 390), fit=False))
    out.append(snap('19_inside_front', (2000, 0, 420), c, ortho=True))
    out.append(snap('20_inside_top', (0, 0, 2500), c, ortho=True, up=(1, 0, 0)))
    set_visible(('femur_L', 'tibia_L', 'wheel_L'), False)
    out.append(snap('21_inside_side_left_leg_off', (0, 2000, 420), c, ortho=True))
    set_visible(('femur_L', 'tibia_L', 'wheel_L'), True)
    set_visible(('rail_', 'battery_24V'), False)
    out.append(snap('22_battery_tray_and_tubes', (700, -500, 900), c))
    set_visible(('battery_tray', 'spine_tube_'), False)
    out.append(snap('23_electronics_deck', (330, 230, 560), (0, 0, 335), fit=False))
    set_visible(SHELL + ('top_plate', 'rail_', 'battery_24V', 'battery_tray', 'spine_tube_'), True)
    set_visible(('shell_front_L', 'shell_mid_L', 'shell_rear_L', 'saddle_L', 'drum_ring_L', 'deco_light_ring_L',
                 'deco_drum_arc_L'), False)
    out.append(snap('24_cutaway_left_shell_off', (900, 950, 700), (0, 0, 330)))
    show_all()
    view.look((1100, 800, 750), (0, 0, 290))
    return out


# ----------------------------------------------------------------------------- BOM
def cup_screws():
    """{joint: (counterbore depth, length)} of the cup column screws (legs.cup_screw, 4 mm engagement)."""
    return {joint: LG.cup_screw(joint)[::-1] for joint in ('hip', 'knee', 'wheel')}


WIRE_CLASS = ((3.5, '12 AWG 배터리·변환기 리드', '배터리·변환기에 포함'),
              (2.5, '16 AWG 실리콘선', '전원 데이지 체인, P5 → pi3hat'),
              (1.8, 'CAN 26 AWG 꼬은선(PH-3, 1번 H·2번 L)', 'JC1·JC2 → hip → knee → 바퀴'),
              (0.0, '가는 선(팬·스위치 하네스)', '팬·mjbots 스위치 하네스에 포함'))


def wires():
    """{(class, colour): (m, note)} of the modelled wires; a wire body's centre-line length is
    L = A^2 / (4 pi V) and its diameter 4 V / A (long round body)."""
    out = {}
    for o in C.design().rootComponent.allOccurrences:
        if not o.component.name.startswith('harness_'):
            continue
        for b in o.component.bRepBodies:
            A, V = b.area * 100.0, b.volume * 1000.0          # mm2, mm3
            od = 4 * V / A
            cls, note = next((c, n) for (lim, c, n) in WIRE_CLASS if od >= lim)
            colour = (b.appearance.name if b.appearance else '?').replace('wire ', '')
            m, _n = out.get((cls, colour), (0.0, note))
            out[(cls, colour)] = (m + A * A / (4 * math.pi * V) / 1000.0, note)
    return out


def counts():
    """Instances of every component in the visible tree (the _library masters excluded)."""
    n = {}
    for o in C.design().rootComponent.allOccurrences:
        if o.fullPathName.startswith('_library'):
            continue
        n[o.component.name] = n.get(o.component.name, 0) + 1
    return n


def top_plate_inserts():
    lx, ly, _t = P.TOP_PLATE
    k = 0
    for i in range(-2, 3):
        for j in range(-4, 5):
            x, y = 25.0 * i, 25.0 * j
            if abs(x) > lx / 2 - 10 or abs(y) > ly / 2 - 10:
                continue
            if any(abs(x - sx * P.RAIL_X) < 8 and abs(y - yy) < 8 for sx in (-1, 1) for yy in (-100.0, -33.3, 33.3, 100.0)):
                continue
            k += 1
    return k


def fasteners():
    cs = cup_screws()
    rows = [
        # (group, item, spec, qty, where)
        ('관절', '육각 홈붙이 볼트 ISO 4762 12.9', 'M2.5 × 6', 12, 'hip 고정링 → 6061 hip 판(나사산 3.2 mm, AKE 링 탭 깊이 5)'),
        ('관절', '저두 볼트 DIN 7984 8.8', 'M2.5 × 8', 24, 'knee·wheel 고정링 → PETG 판, SUS 클램프 링 위'),
        ('관절', '육각 홈붙이 볼트 ISO 4762 12.9', 'M3 × 16', 36, '모터 출력 → hip·knee 구동 보스(클램프 링 위)와 허브 플랜지'),
        ('관절', '평와셔 DIN 125 A2', 'M3', 36, '위 M3 × 16 머리 밑(나사산 4.3 mm)'),
        ('관절', '평행 핀 ISO 8734 m6 경화', 'Ø3 × 10', 18, '모터 출력 다웰(출력 3 mm + PETG 7 mm)'),
        ('바퀴', '육각 홈붙이 볼트 ISO 4762 8.8', 'M3 × 8', 12, '육각 허브 → 허브 플랜지'),
        ('바퀴', '열압입 인서트 황동', 'M3 × 5.7 (구멍 Ø4.0 × 6)', 12, '허브 플랜지'),
        ('바퀴', '육각 홈붙이 볼트 ISO 4762 8.8', 'M5 × 50', 2, '바퀴 축 볼트(머리는 플랜지 포켓 안, 육각 허브를 누름)'),
        ('드라이버', '저두 볼트 DIN 7984', 'M2.5 × 6', 24, 'moteus → 컵 스탠드오프(보드 모터 쪽에서)'),
        ('드라이버', '열압입 인서트 황동', 'M2.5 × 4 (구멍 Ø3.6 × 5)', 24, '컵 스탠드오프'),
        ('드라이버', '육각 홈붙이 볼트 ISO 4762', 'M3 × %d' % cs['hip'][1], 6,
         'hip 컵 → hip 판 M3 탭(컵 바닥 카운터보어 %.1f mm, 나사산 4 mm)' % cs['hip'][0]),
        ('드라이버', '육각 홈붙이 볼트 ISO 4762', 'M2.5 × %d' % cs['knee'][1], 6,
         'knee 컵 → 허벅지 인서트(카운터보어 %.1f mm)' % cs['knee'][0]),
        ('드라이버', '육각 홈붙이 볼트 ISO 4762', 'M2.5 × %d' % cs['wheel'][1], 6,
         'wheel 컵 → 정강이 인서트(카운터보어 %.1f mm)' % cs['wheel'][0]),
        ('드라이버', '열압입 인서트 황동', 'M2.5 × 4 (구멍 Ø3.6 × 5)', 12, '링크의 컵 기둥 자리'),
        ('배선', '저두 볼트 DIN 7984', 'M2.5 × 6', 24, '케이블 클립 → 링크 홈 옆 인서트'),
        ('배선', '열압입 인서트 황동', 'M2.5 × 4 (구멍 Ø3.6 × 5)', 24, '링크 배선 홈 옆'),
        ('배선', '고무 그로밋', 'Ø16 구멍, 판 1.4 mm', 2, '셸 hip 케이블 출구'),
        ('몸통', '접시 볼트 ISO 10642', 'M5 × 12', 4, 'hip 판 → 척추 봉 끝 M5 탭(물림 약 7.5 mm)'),
        ('몸통', '접시 볼트 ISO 10642', 'M4 × 12', 4, 'hip 판 → 위 레일 끝 M4 탭(물림 약 7.5 mm)'),
        ('몸통', '육각 홈붙이 볼트 ISO 4762', 'M4 × 25', 8, '윗판 → 위 레일(관통)'),
        ('몸통', '나일론 너트 DIN 985 + 와셔 DIN 125', 'M4', 8, '위 레일 아래'),
        ('몸통', '열압입 인서트 황동', 'M4 × 5.7 (구멍 Ø5.6 × 6)', top_plate_inserts(), '윗판 25 mm 격자(짐·로봇팔)'),
        ('몸통', '육각 홈붙이 볼트 ISO 4762', 'M3 × 8', 4, '전장 데크 플랜지 → hip 판 M3 탭'),
        ('몸통', '버튼 볼트 ISO 7380', 'M3 × 6', 12, '셸(hip 부분) → hip 판 M3 탭'),
        ('몸통', '버튼 볼트 ISO 7380 + 육각 너트 DIN 934', 'M3 × 8', 24, '셸 앞·뒤 조각 ↔ hip 조각 이음판 12개'),
        ('몸통', '저두 볼트 DIN 7984', 'M2.5 × 6', 4, 'mjpower-ss → 데크 보스'),
        ('몸통', '열압입 인서트 황동', 'M2.5 × 4 (구멍 Ø3.6 × 5)', 8, '데크 보스(라즈베리파이 4, mjpower-ss 4)'),
        ('몸통', '육각 너트 / 볼트 (pi3hat 동봉)', 'M2.5', 4, 'pi3hat 위 스탠드오프'),
        ('몸통', '벨크로 스트랩', '폭 25 mm × 400 mm', 2, '배터리 + 트레이를 척추 봉에 묶음(트레이 슬롯)'),
        ('몸통', '폼 패드', '75 × 310 × 1 mm', 1, '배터리 밑'),
    ]
    return rows


def bom():
    n = counts()
    rows = []
    for name, (kind, note) in sorted(design_parts().items()):
        if n.get(name):
            rows.append((kind, name, note, n[name], ''))
    seen = {}
    for name, k in n.items():
        for pre, (item, spec) in PURCHASED.items():
            if name.startswith(pre):
                seen[item] = (spec, seen.get(item, ('', 0))[1] + k)
    for item, (spec, k) in seen.items():
        rows.append(('구매품', item, spec, k, ''))
    for (grp, item, spec, k, where) in fasteners():
        rows.append(('체결·' + grp, item, spec, k, where))
    for (cls, colour), (m, note) in sorted(wires().items()):
        buy = '%.1f m (모델 %.2f m + 20 %%)' % (math.ceil(m * 1.2 * 10) / 10, m)
        rows.append(('배선', '%s, %s' % (cls, colour), buy, 1, note))
    path = os.path.join(BASE, 'BOM.csv')
    with open(path, 'w', encoding='utf-8-sig', newline='') as f:
        w = csv.writer(f)
        w.writerow(('구분', '품목', '규격·설명', '수량', '위치·비고'))
        w.writerows(rows)
    md = ['# WheelBiped v2 BOM (`finalize.bom()`이 모델에서 생성)\n',
          '| 구분 | 품목 | 규격·설명 | 수량 | 위치·비고 |', '|---|---|---|---|---|']
    md += ['| %s | %s | %s | %s | %s |' % r for r in rows]
    with open(os.path.join(BASE, 'BOM.md'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(md) + '\n')
    return rows


# ----------------------------------------------------------------------------- joints
def _axis_circle(comp, x, z, name):
    """Sketch circle about a joint axis drawn in a link component (robot x, z; axis along robot y)."""
    sk = comp.sketches.add(comp.yZConstructionPlane)
    sk.name = name
    p = C.rf((x, 0.0, z))
    c = sk.modelToSketchSpace(adsk.core.Point3D.create(p[0] * C.CM, p[1] * C.CM, p[2] * C.CM))
    circ = sk.sketchCurves.sketchCircles.addByCenterRadius(c, 0.8)
    sk.isVisible = False
    return circ


def _revolute(name, occ1, occ2, curve):
    root = C.design().rootComponent
    geo = adsk.fusion.JointGeometry.createByCurve(curve, adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
    ji = root.asBuiltJoints.createInput(occ1, occ2, geo)
    ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    j = root.asBuiltJoints.add(ji)
    j.name = name
    return j


def joints():
    """Six as-built revolute joints (torso grounded).  Joints exist only in parametric designs: everything
    built so far becomes base features.  Every nested occurrence is grounded to its parent first, otherwise
    the joint solver leaves parts behind when a link moves."""
    PO.reset()
    d = C.design()
    d.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = d.rootComponent
    torso = C.root_occ('torso')
    torso.isGrounded = True
    kids = [c.occurrences.item(i) for c in d.allComponents for i in range(c.occurrences.count) if c != root]
    for o in kids:                       # setting the flag reorders the collections: gather first
        o.isGroundToParent = True
    left = [o.name for o in kids if not o.isGroundToParent]
    if left:
        raise RuntimeError('not grounded to parent: %s' % left[:5])
    out = []
    for k in ('L', 'R'):
        f, t, w = C.root_occ('femur_' + k), C.root_occ('tibia_' + k), C.root_occ('wheel_' + k)
        hip = _axis_circle(f.component, 0.0, P.Z_HIP, 'hip_axis')
        knee = _axis_circle(f.component, P.KNEE_X, P.KNEE_Z, 'knee_axis')
        axle = _axis_circle(t.component, 0.0, P.R_WHEEL, 'wheel_axis')
        out.append(_revolute('hip_' + k, torso, f, hip.createForAssemblyContext(f)))
        out.append(_revolute('knee_' + k, f, t, knee.createForAssemblyContext(f)))
        out.append(_revolute('wheel_' + k, t, w, axle.createForAssemblyContext(t)))
    root.isJointsFolderLightBulbOn = False
    return [j.name for j in out]


def drive_check(angle_deg=20.0):
    """Turn every joint by angle_deg and back; returns how far a child part moved with each link (mm)."""
    root = C.design().rootComponent
    res = {}
    for j in root.asBuiltJoints:
        link = j.occurrenceTwo
        kid = link.childOccurrences.item(0)
        p0 = kid.boundingBox.minPoint.copy()
        j.jointMotion.rotationValue = math.radians(angle_deg)
        p1 = kid.boundingBox.minPoint.copy()
        j.jointMotion.rotationValue = 0.0
        p2 = kid.boundingBox.minPoint
        res[j.name] = (round(p0.distanceTo(p1) * 10, 1), round(p0.distanceTo(p2) * 10, 3))
    return res


# ----------------------------------------------------------------------------- exports
def _file_name(name):
    return name.replace(' ', '_').replace('Ø', 'D')


def exports():
    d = C.design()
    em = d.exportManager
    os.makedirs(os.path.join(EXP, 'parts_step'), exist_ok=True)
    os.makedirs(os.path.join(EXP, 'parts_stl'), exist_ok=True)
    done = []
    f3d = os.path.join(EXP, 'WheelBiped_v2.f3d')
    em.execute(em.createFusionArchiveExportOptions(f3d))
    done.append(f3d)
    step = os.path.join(EXP, 'WheelBiped_v2_assembly.step')
    em.execute(em.createSTEPExportOptions(step, d.rootComponent))
    done.append(step)
    for name in sorted(design_parts()):
        comp = C.find_comp(name)
        safe = _file_name(name)
        p = os.path.join(EXP, 'parts_step', safe + '.step')
        em.execute(em.createSTEPExportOptions(p, comp))
        done.append(p)
        solids = [b for b in comp.bRepBodies if b.isSolid]
        for b in solids:
            fn = safe if len(solids) == 1 else safe + '__' + _file_name(b.name)
            s = os.path.join(EXP, 'parts_stl', fn + '.stl')
            o = em.createSTLExportOptions(b, s)
            o.isBinaryFormat = True
            o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
            em.execute(o)
            done.append(s)
    return done
