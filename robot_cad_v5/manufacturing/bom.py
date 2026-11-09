"""WheelBiped v5 BOM for ordering (11-08, user: "v5 모델의 bom 파일").

Quantities of every modelled part are the instance counts in the exported assembly STEP (export/WheelBiped_v5_assembly.step =
hub version 16).  Every STEP component must be classified below, so a part added to the model and
not to this file stops the script.  Printed masses and bed sizes come from export/parts_stl.  Parts that are not in the model
(the v5 joint screws 6 and 7 and their inserts, wiring, tools) are added by hand and marked as such.

11-09 user: every part is printed on the user's 300 x 300 mm printer in PLA or PETG, strength not considered - there is no
machining order any more.  The former machined parts (6061 / SUS304) are printed solid from the same STL; their tapped holes are
in the STL as tap-drill holes (hand-tapped after printing) and their fits are worked to the real part.  The script stops when a
printed part does not fit the bed in the orientation given here.

    python manufacturing/bom.py        # -> ../BOM.md, ../BOM.csv (run from robot_cad_v5)

Specs of the parts the v4 code builds: robot_cad_v4/fusion/finalize.py (PRINT_NOTE, MACHINED, PURCHASED) and fasteners.py (specs);
v5 parts and joint screws: fusion/legs_v5.py, manufacturing/part_drawings.py, manufacturing/explain/real_sections.py.
"""
import csv
import math
import os
import re
import struct
from collections import defaultdict

import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(BASE, 'fusion'))
import split_v5 as SP  # noqa: E402  (Fusion-free constants of the two-piece tray / deck)
STEP = os.path.join(BASE, 'export', 'WheelBiped_v5_assembly.step')
STL = os.path.join(BASE, 'export', 'parts_stl')
RHO = 1.27e-3                      # PETG g/mm3 (v4 params.PETG_RHO); PLA 1.24e-3
RHO_AL, RHO_SUS = 2.70e-3, 7.93e-3  # the former machined parts as modelled (6061-T6, SUS304)
WASTE = 1.15                       # filament: + 15 % for supports, brims and failed prints
BED = 300.0                        # user 11-09: the printer prints up to 300 x 300 mm
BRIM = 5.0                         # brim per side, for the margin note
FLANGE_BOLTS, CAP_BOLTS = 6, 3     # legs_v5: _flange_bolts (6 per hub insert, r 18), _cap_bolts (3 per knee cap, r 20.5)
HIP_PLATE_M3, END_TAPS = 11, 2     # v4 torso.hip_plate: M3 tapped (cup 3, deck 2, shell 6); spine / rail: both ends tapped
MODEL = '허브 버전 17(11-09, 받침·데크 두 조각) = `export/WheelBiped_v5.f3d`, 링크 25 cm'
MODEL_KG = 11.611                  # v5run.robot_mass() of that model (metal parts as modelled)
MAT = 'PLA 또는 PETG'
HOT = 'PETG 권장(모터 열)'          # parts on the motors / drivers: PLA softens at about 55-60 C


# ----------------------------------------------------------------------------- model counts (assembly STEP)
def _decode(s):
    s = re.sub(r'\\X2\\([0-9A-F]+)\\X0\\', lambda m: ''.join(chr(int(m.group(1)[i:i + 4], 16))
                                                            for i in range(0, len(m.group(1)), 4)), s)
    return re.sub(r'\\X\\([0-9A-F]{2})', lambda m: chr(int(m.group(1), 16)), s).replace("''", "'")


def step_counts(path=STEP):
    """{component name: instances in the whole tree}, number of leaf instances."""
    txt = open(path, 'r', encoding='latin-1').read()
    prod, pdf, pd, nauo = {}, {}, {}, []
    for m in re.finditer(r'#(\d+)\s*=\s*(.*?);\s*(?=#\d+\s*=|ENDSEC)', txt, re.S):
        k, v = int(m.group(1)), ' '.join(m.group(2).split())
        if v.startswith('PRODUCT('):
            strs = re.findall(r"'((?:[^']|'')*)'", v)
            prod[k] = _decode(strs[1] or strs[0])
        elif v.startswith('PRODUCT_DEFINITION_FORMATION'):
            pdf[k] = int(re.findall(r'#(\d+)', v)[-1])
        elif v.startswith('PRODUCT_DEFINITION('):
            pd[k] = int(re.findall(r'#(\d+)', v)[0])
        elif v.startswith('NEXT_ASSEMBLY_USAGE_OCCURRENCE'):
            r = re.findall(r'#(\d+)', v)
            nauo.append((int(r[0]), int(r[1])))
    name = {d: prod.get(pdf.get(f), '?') for d, f in pd.items()}
    kids = defaultdict(list)
    for p, c in nauo:
        kids[p].append(c)
    child = {c for _p, c in nauo}
    roots = [d for d in pd if d not in child and kids.get(d)]
    assert len(roots) == 1, roots
    n, leaves = defaultdict(int), [0]

    def walk(d):
        for c in kids.get(d, []):
            n[name[c]] += 1
            if kids.get(c):
                walk(c)
            else:
                leaves[0] += 1
    walk(roots[0])
    return dict(n), leaves[0]


# ----------------------------------------------------------------------------- printed parts (STL)
def stl(path):
    b = open(path, 'rb').read()
    k = struct.unpack('<I', b[80:84])[0]
    a = np.frombuffer(b[84:84 + 50 * k], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return a['v'].reshape(-1, 3, 3).astype(float)


def _hull(p):
    p = np.unique(np.round(p, 3), axis=0)
    p = p[np.lexsort((p[:, 1], p[:, 0]))]

    def half(pts):
        h = []
        for q in pts:
            while len(h) >= 2 and ((h[-1][0] - h[-2][0]) * (q[1] - h[-2][1]) - (h[-1][1] - h[-2][1]) * (q[0] - h[-2][0])) <= 0:
                h.pop()
            h.append(q)
        return h
    lo, up = half(p), half(p[::-1])
    return np.array(lo[:-1] + up[:-1])


def square_bed(p2):
    """Smallest square bed side (mm) that takes the 2D outline p2 at any angle (0.25 deg steps)."""
    h = _hull(p2)
    best = 1e9
    for deg in np.arange(0.0, 90.0, 0.25):
        t = math.radians(deg)
        q = h @ np.array([[math.cos(t), math.sin(t)], [-math.sin(t), math.cos(t)]])
        best = min(best, (q.max(0) - q.min(0)).max())
    return best


def stl_info(name, axis=None):
    """Volume mm3, area mm2, sorted box, smallest square bed, print height - lying on the face normal to axis (STL x, y, z), or
    lying flat (on the axis of the smallest extent) when axis is None."""
    t = stl(os.path.join(STL, name + '.stl'))
    a, b, c = t[:, 0], t[:, 1], t[:, 2]
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1).sum()
    vol = abs(np.einsum('ij,ij->i', a, np.cross(b, c)).sum()) / 6.0
    p = t.reshape(-1, 3)
    ext = p.max(0) - p.min(0)
    ax = int(np.argmin(ext)) if axis is None else axis
    return vol, area, sorted(ext.round(0), reverse=True), square_bed(p[:, [i for i in range(3) if i != ax]]), ext[ax]


# ----------------------------------------------------------------------------- what every model part is
# (STEP component name or prefix, as in the v4 / v5 code)
CONTAINERS = ('Untitled', 'torso', 'femur_', 'tibia_', 'wheel_')          # assembly components: no part of their own
IN_MOTOR = ('AKE60-8_KV80_output',)                                        # the rotor half of the bought AKE60 STEP

PURCHASED = [   # (prefix, item, spec, source, note)
    ('AKE60-8_KV80_stator', 'CubeMars AKE60-8 KV80', '8:1, 피크 12.5 / 정격 5 N·m, 0.26 kg, Ø69 × 25', 'CubeMars',
     'hip·무릎·바퀴 × 좌우. 부속 자석 Ø6 × 2.5 N38H(엔코더용)를 출력한 자석 스페이서(2절)에 붙여 쓴다'),
    ('moteus_r4.11', 'mjbots moteus r4.11', 'XT30PW-M ×2, JST PH-3 CAN ×2, 10~44 V', 'mjbots', '관절마다 1개, 드라이버 컵 안'),
    ('raspberry_pi_4b', 'Raspberry Pi 4 Model B', '4 GB 이상', '라즈베리파이 판매점', 'microSD는 이 표 맨 아래'),
    ('pi3hat_r4.5', 'mjbots pi3hat r4.5', 'CAN-FD ×5, IMU, 24 V → 5 V, M2.5 × 11 스탠드오프·나사 동봉', 'mjbots',
     '파이에는 USB-C 전원을 넣지 않는다(pi3hat이 5 V를 만든다)'),
    ('standoff M2.5x11', 'M2.5 × 11 육각 스탠드오프(수-암)', 'pi3hat 동봉 — 따로 사지 않는다', '(pi3hat 동봉)', '파이 ↔ pi3hat'),
    ('power_dist_r4.5b', 'mjbots power dist r4.5b',
     'XT90 입력, XT30 출력 ×6, 프리차지, CAN-FD(전압·전류·에너지, 소프트 종료), 10~44 V, 80 × 50, 구멍 74 × 44', 'mjbots',
     '배터리를 파이·pi3hat에 바로 물리지 않는다(11-08 사용자: mjpower-ss 대신, 자동 종료용 CAN). 동봉: XT30U 수 6, XT90S 암 1, '
     'PH-4·PH-3 짝 커넥터, 로커 스위치 하네스. CAN1 → pi3hat JC3, 기본 CAN ID 32, 보드에 종단 저항 없음(선이 0.5 m 미만이라 pi3hat 쪽만으로 됨)'),
    ('battery_24V_15Ah', '배터리 24 V 15 Ah', '75 × 310 × 75 mm, XT60(F) 리드', '(가지고 있는 배터리)', '새로 살 때는 크기·커넥터가 같은 것'),
    ('fan 30x30x7', '팬 30 × 30 × 7 mm 5 V', '2핀', '전자 부품상', 'pi3hat JP6(1번 5 V, 3번 GND), 데크에 1.5 mm 양면 폼테이프'),
    ('rocker switch', '로커 스위치 19 × 13 + mjbots PH4 스위치 하네스', 'power dist J2(PH-4)용, E-Switch R1966 + 30 cm 하네스',
     '(power dist r4.5b 동봉)', '몸통 앞면(CAD −x) 수직면, 로봇 오른쪽(CAD +y)의 19 × 13 구멍'),
    ('CAN terminator', 'CAN-FD 종단 저항 120 Ω (JST PH-3)', '20 × 15 × 10 이하', 'mjbots (또는 120 Ω 저항 + PHR-3로 직접)',
     '바퀴 드라이버 CAN1(다리 끝). moteus에는 종단 저항이 없고 pi3hat 쪽에는 있다'),
    ('XT30 cap', 'XT30 보호 캡', 'XT30 수 커넥터용', '전자 부품상', '바퀴 드라이버 J2(24 V가 살아 있는 빈 포트)'),
    ('tyre M365', 'Xiaomi M365 8.5 × 2 (8 1/2 × 2) 통고무 허니콤 타이어', '외경 202 × 폭 54, 안지름 141 × 안쪽 폭 36.5, 721 g',
     'AliExpress', '사용자가 이미 주문함(v2 때) — 받았는지 확인'),
]


def cnt(prefix):
    """Model count of the STEP components starting with prefix."""
    return lambda n: sum(v for k, v in n.items() if k.startswith(prefix))


# 11-09 user: the former machined parts are printed too (no machining order).  STEP components they are counted from (the hub
# inserts and backing rings are bodies of thigh_* / shank_*, the knee caps are kneecap_*):
EX_STEP = ('hip_plate_', 'spine_tube_', 'rail_', 'clamp ring stator', 'magnet spacer', 'kneecap_')
SOLID = '채움 100 %'
EX_MACHINED = [  # (drawing, item, was, print settings / face on the bed, STL axis normal to the bed or None, after printing,
    #             [(STL name, count from the STEP counts n, metal density as modelled)])
    ('WBv5-S14', 'hip 판', '6061-T6 판, 3축 MCT 양면, 검정 아노다이징',
     SOLID + ', 안쪽 면(Ø80 컵 받침 턱)을 바닥에 — 턱 밖 웹 밑 1.5 mm만 바닥 서포트', 0,
     'M3 탭 %d(Ø2.5 구멍: 컵 3, 데크 2, 셸 6), Ø42 모터 파일럿 자리를 모터에 맞춰 다듬기. 왼·오른쪽 거울상 = 다른 파일' % HIP_PLATE_M3,
     [('hip_plate_L', cnt('hip_plate_L'), RHO_AL), ('hip_plate_R', cnt('hip_plate_R'), RHO_AL)]),
    ('WBv5-S15', '척추 봉 Ø16 × 328', '6061-T6 환봉, 길이 328.00 ±0.05',
     SOLID + ', 눕혀 대각선으로, 둥근 아래쪽은 바닥 서포트', None,
     '양 끝 M5 탭 깊이 11(Ø4.2 구멍, 눕혀 뽑은 구멍이라 드릴로 정리 후 탭). 길이 328을 레일과 같게(길면 끝면을 사포로)',
     [('spine_tube_F', cnt('spine_tube_F'), RHO_AL), ('spine_tube_B', cnt('spine_tube_B'), RHO_AL)]),
    ('WBv5-S16', '위 레일 15 × 15 × 328', '6061-T6 각봉, 길이 328.00 ±0.05',
     SOLID + ', Ø4.4 관통 구멍이 위아래로 서게 옆면을 바닥에, 대각선으로 — 서포트 없음', 1,
     '양 끝 M4 탭 깊이 9(Ø3.3 구멍). 길이 328을 척추 봉과 같게',
     [('rail_F', cnt('rail_F'), RHO_AL), ('rail_B', cnt('rail_B'), RHO_AL)]),
    ('WBv5-S17', '고정자 클램프 링 Ø45 × 2', 'SUS304 2t 레이저',
     SOLID + ', 평평하게', None, '없음. 작고 얇아 여분 2개를 더 뽑는다',
     [('clamp_ring_stator_D45_t2.0_SUS304', cnt('clamp ring stator'), RHO_SUS)]),
    ('WBv5-S18', '자석 스페이서 Ø9 × 5.35', '6061(비자성) 선반',
     SOLID + ', 자석 구멍 쪽을 바닥에 스피곳을 위로, 층 0.1 mm 권장', 1,
     'AKE 부속 자석 Ø6 × 2.5를 구멍에 순간접착, Ø5.9 스피곳을 끼울 자리에 맞춰 다듬기. 작아서 여분 2개를 더 뽑는다(출력이라 비자성 조건은 저절로 맞음)',
     [('magnet_spacer_D9x5.35_(6061)__spacer', cnt('magnet spacer'), RHO_AL)]),
    ('WBv5-M01', 'hip hub 인서트', '6061-T6 환봉 선반 + MCT, 아노다이징 권장',
     SOLID + ', 플랜지 바깥 면(접시 구멍 쪽, 평평)을 바닥에, 보스·스피곳을 위로 — 서포트 없음', 0,
     '평행핀 구멍 Ø3 × 7.5에 Ø3 드릴 → 핀 압입, Ø11.8 스피곳을 모터 출력부 Ø12 홈에, Ø48 플랜지를 링크 자리 Ø48.3에 맞춰 다듬기. 왼·오른쪽 거울상 = 다른 파일',
     [('hub_insert_hip_L', cnt('thigh_L'), RHO_AL), ('hub_insert_hip_R', cnt('thigh_R'), RHO_AL)]),
    ('WBv5-M02', '무릎 hub 인서트', '6061-T6 환봉 선반 + MCT, 아노다이징 권장',
     SOLID + ', hip hub 인서트와 같게 플랜지 바깥 면을 바닥에 — 서포트 없음', 0,
     'hip hub 인서트와 같음(보스 길이만 다름). 왼·오른쪽 거울상 = 다른 파일',
     [('hub_insert_knee_L', cnt('shank_L'), RHO_AL), ('hub_insert_knee_R', cnt('shank_R'), RHO_AL)]),
    ('WBv5-M03', '무릎 캡', '6061-T6 환봉 선반 + 드릴, 아노다이징 권장',
     SOLID + ', 플랜지를 바닥에 관을 위로 — 서포트 없음', 0,
     'Ø35 관을 61807 안륜·허벅지 바깥 막대 구멍에 꼭 맞게 사포로. 왼·오른쪽 같은 부품',
     [('knee_cap_L', cnt('kneecap_L'), RHO_AL), ('knee_cap_R', cnt('kneecap_R'), RHO_AL)]),
    ('WBv5-M04', '받침 링 Ø43 / Ø29 × 6', '6061-T6 환봉 선반, M3 탭 관통 6',
     SOLID + ', 평평하게', 0,
     'M3 탭 관통 %d(Ø2.5 구멍). hip·무릎 × 왼·오른쪽 모두 같은 부품 — 한 파일로 4개를 뽑아도 된다' % FLANGE_BOLTS,
     [('hub_ring_hip_L', cnt('thigh_L'), RHO_AL), ('hub_ring_hip_R', cnt('thigh_R'), RHO_AL),
      ('hub_ring_knee_L', cnt('shank_L'), RHO_AL), ('hub_ring_knee_R', cnt('shank_R'), RHO_AL)]),
]
EX_HOT = ('WBv5-S14', 'WBv5-S17', 'WBv5-S18', 'WBv5-M01', 'WBv5-M02', 'WBv5-M04')   # on the motors: PETG recommended
NOT_ORDERED = ('clamp ring output',)   # v4 part removed from v5 (the hub inserts replace it); its STEP / STL files are left in export/

LEG_PRINT = ('FDM 한 덩어리: 벽 3.5 둘레선, 채움 100 %, 닫힌 칸(배출 구멍 없음)')
PRINTED = [     # (prefix, drawing, item, settings / orientation, colour, STL axis of the face on the bed or None, note, material)
    ('thigh_', 'WBv5-P01', '허벅지', LEG_PRINT + ', 바깥 면(평평, y 288.5)을 바닥에 — 갈래 틈 안 서포트', '검정', 0,
     '열압입 M2.5 인서트 6(컵 3 + 무릎 캡 3), 캡 관 구멍 Ø35 H7은 0.2 작게 뽑고 리머', HOT),
    ('shank_', 'WBv5-P02', '정강이', LEG_PRINT + ', 안쪽 면(평평, y 208)을 바닥에 — 바퀴 모터 칸 Ø74 안 서포트', '검정', 0,
     '열압입 M2.5 인서트 3(바퀴 컵), 61807 자리 Ø47 압입', HOT),
    ('cup_hip', 'WBv5-S01', 'hip 드라이버 컵', '채움 30 %, 바닥(덮개 면)을 바닥에, 서포트는 커넥터 창만', '검정', None, '열압입 M2.5 인서트 4', HOT),
    ('cup_knee', 'WBv5-S02', '무릎 드라이버 컵', '채움 30 %, 바닥을 바닥에, 서포트는 커넥터 창만', '검정', None, '열압입 M2.5 인서트 4', HOT),
    ('cup_wheel', 'WBv5-S03', '바퀴 드라이버 컵', '채움 30 %, 바닥을 바닥에, 서포트는 커넥터 창만', '검정', None, '열압입 M2.5 인서트 4', HOT),
    ('hub_flange', 'WBv5-S04', '바퀴 허브 플랜지', '채움 100 %, 모터 쪽 면을 바닥에', '검정', None, '열압입 M3 인서트 6', HOT),
    ('rim_', 'WBv5-S05', '바퀴 림(한 조각)', '벽 6겹(2.4 mm) · 채움 40 %, 허브 쪽 평면을 바닥에, 열린 쪽 플랜지 밑 4.5 mm 턱만 서포트',
     '검정', None, '', MAT),
    ('shell_front', 'WBv5-S06', '몸통 셸 앞(스위치 쪽)', '채움 25 %, 바깥 면을 위로(서포트 필요)', '진회색', None, 'v4와 달리 세로 모서리 R30', MAT),
    ('shell_mid', 'WBv5-S07', '몸통 셸 가운데(hip)', '채움 25 %, 바깥 면을 위로(서포트 필요)', '진회색', None, '', MAT),
    ('shell_rear', 'WBv5-S08', '몸통 셸 뒤(무릎 쪽)', '채움 25 %, 바깥 면을 위로(서포트 필요)', '진회색', None, '', MAT),
    ('saddle_', 'WBv5-S09', 'hip 위 안장', '채움 25 %', '파랑', None, '', MAT),
    ('top_plate', 'WBv5-S10', '위판(짐 판)', '채움 25 %', '검정', None, '열압입 M4 인서트 35', MAT),
    ('battery_tray', 'WBv5-S11', '배터리 받침(두 조각)',
     '채움 30 %, 긴 옆면(159 × 28)을 바닥에 세워서 — 배터리 옆 턱 2줄(14 mm) 밑만 서포트(눕히면 바닥 판 밑 전체에 서포트)', '진회색', 2,
     '11-09 두 조각: 가운데(리브 자리)의 이음 턱끼리 M3 × 10 버튼 볼트 + 너트 2쌍(3절 끝). _L·_R은 다른 파일(리드선 홈은 _R 끝)', MAT),
    ('electronics_deck', 'WBv5-S12', '전장 데크(두 조각)',
     '채움 30 %, 놓인 그대로(보스·끝 플랜지를 위로) — 바닥 판·리브 밑 서포트', '진회색', 1,
     '11-09 두 조각: y 80(라즈베리파이와 끝 창 사이) 이음 턱끼리 M3 × 10 + 너트 2쌍(3절 끝). 열압입 M2.5 인서트 8은 모두 _L(파이·전원보드 쪽)', MAT),
    ('splice_tabs', 'WBv5-S13', '셸 이음판 12개(한 파일)', '채움 100 %', '진회색', None,
     '12개가 로봇 위치 그대로 한 파일에 있다 — 슬라이서에서 나눠 놓는다(베드 값은 무시)', MAT),
]
PRINT_PROFILE = {   # wall mm, infill (v4 params.PRINT; v5 links and the former machined parts solid)
    'thigh_': (99.0, 1.0), 'shank_': (99.0, 1.0), 'hub_flange': (99.0, 1.0), 'rim_': (2.4, 0.40), 'cup_': (1.6, 0.30),
    'top_plate': (1.6, 0.25), 'battery_tray': (1.6, 0.30), 'electronics_deck': (1.6, 0.30), 'shell_': (1.6, 0.25),
    'saddle_': (1.6, 0.25), 'splice_tabs': (1.6, 1.0),
}
EX_COLOUR = '검정'

# fasteners modelled by the v4 code ('FX ...' masters): name -> (group, item, spec, where, expected count from v4 fasteners.specs)
FX = {
    'FX ISO 4762 M2.5x6 12.9 hip': ('관절', '육각 홈붙이 볼트 ISO 4762 12.9', 'M2.5 × 6', 'hip 고정자 링 → hip 판 ①', 12),
    'FX DIN 7984 M2.5x8 stator': ('관절', '저두 볼트 DIN 7984 8.8', 'M2.5 × 8', '무릎·바퀴 고정자 링 → 링크 모터 판, 클램프 링 위 ①', 24),
    'FX ISO 4762 M3x16 12.9': ('관절', '육각 홈붙이 볼트 ISO 4762 12.9', 'M3 × 16', 'hip·무릎 모터 출력 → hub 인서트 ② (와셔 위)', 24),
    'FX ISO 8734 3x10': ('관절', '평행 핀 ISO 8734 m6 경화', 'Ø3 × 10', '출력 다웰 ③: hub 인서트 12 (Ø3 압입) + 바퀴 허브 플랜지 6', 18),
    'FX DIN 125 M3': ('관절', '평와셔 DIN 125 A2', 'M3', '② M3 × 16, 허브 M3 × 14, 림 M3 × 14 머리 밑', 48),
    'FX ISO 4762 M3x14 12.9 hub': ('바퀴', '육각 홈붙이 볼트 ISO 4762 12.9', 'M3 × 14', '바퀴 모터 출력 → 허브 플랜지 (와셔 위)', 12),
    'FX ISO 4762 M3x14 rim': ('바퀴', '육각 홈붙이 볼트 ISO 4762 8.8', 'M3 × 14', '림 → 허브 플랜지 인서트 (와셔 위)', 12),
    'FX insert M3x5.7 hub': ('바퀴', '열압입 인서트 황동', 'M3 × 5.7 (구멍 Ø4.0)', '허브 플랜지', 12),
    'FX DIN 7984 M2.5x5 board': ('드라이버', '저두 볼트 DIN 7984', 'M2.5 × 5', 'moteus → 컵 스탠드오프 (끝이 4 mm 인서트 안, 11-08)', 24),
    'FX insert M2.5x4 cup': ('드라이버', '열압입 인서트 황동', 'M2.5 × 4 (구멍 Ø3.6)', '드라이버 컵 스탠드오프', 24),
    'FX ISO 4762 M3x40 hip cup': ('드라이버', '육각 홈붙이 볼트 ISO 4762', 'M3 × 40', 'hip 컵 → hip 판 M3 탭', 6),
    'FX ISO 4762 M2.5x18 knee cup': ('드라이버', '육각 홈붙이 볼트 ISO 4762', 'M2.5 × 18', '무릎 컵 → 허벅지 인서트', 6),
    'FX ISO 4762 M2.5x18 wheel cup': ('드라이버', '육각 홈붙이 볼트 ISO 4762', 'M2.5 × 18', '바퀴 컵 → 정강이 인서트', 6),
    'FX insert M2.5x4 link cup': ('드라이버', '열압입 인서트 황동', 'M2.5 × 4 (구멍 Ø3.6 × 6)', '허벅지·정강이의 컵 기둥 자리(열압입)', 12),
    'FX grommet D16': ('몸통', '고무 그로밋', 'Ø16 구멍, 판 1.4 mm (안지름 14)', '셸 hip 케이블 출구', 2),
    'FX ISO 10642 M5x12': ('몸통', '접시 볼트 ISO 10642', 'M5 × 12', 'hip 판 → 척추 봉 끝 M5 탭', 4),
    'FX ISO 10642 M4x12': ('몸통', '접시 볼트 ISO 10642', 'M4 × 12', 'hip 판 → 위 레일 끝 M4 탭', 4),
    'FX ISO 4762 M4x25 top': ('몸통', '육각 홈붙이 볼트 ISO 4762', 'M4 × 25', '위판 → 위 레일(관통)', 8),
    'FX DIN 985 M4': ('몸통', '나일론 너트 DIN 985', 'M4', '위 레일 아래', 8),
    'FX DIN 125 M4': ('몸통', '평와셔 DIN 125 A2', 'M4', '위 레일 아래 나일론 너트 밑', 8),
    'FX insert M4x5.7 top': ('몸통', '열압입 인서트 황동', 'M4 × 5.7 (구멍 Ø5.6)', '위판 25 mm 격자(짐·로봇팔)', 35),
    'FX ISO 4762 M3x8 deck': ('몸통', '육각 홈붙이 볼트 ISO 4762', 'M3 × 8', '전장 데크 플랜지 → hip 판 M3 탭', 4),
    'FX ISO 7380 M3x8 shell': ('몸통', '버튼 볼트 ISO 7380', 'M3 × 8', '셸(hip 조각·안장) 안쪽 받침 → hip 판 M3 탭 (11-08)', 12),
    'FX ISO 7380 M3x8 splice': ('몸통', '버튼 볼트 ISO 7380', 'M3 × 8', '셸 앞·뒤 조각 ↔ hip 조각 이음판 12개', 24),
    'FX DIN 934 M3 splice': ('몸통', '육각 너트 DIN 934', 'M3', '이음판 안쪽', 24),
    'FX DIN 7984 M2.5x6 mjpower': ('몸통', '저두 볼트 DIN 7984', 'M2.5 × 6', 'power dist r4.5b → 데크 보스', 4),
    'FX insert M2.5x4 deck': ('몸통', '열압입 인서트 황동', 'M2.5 × 4 (구멍 Ø3.6)', '데크 보스(라즈베리파이 4, power dist 4)', 8),
    'FX pi3hat M2.5x5': ('몸통', 'M2.5 × 5 나사 (pi3hat 동봉)', 'M2.5 × 5', 'pi3hat → 스탠드오프 위 — 따로 사지 않는다', 4),
    'FX foam pad': ('몸통', '폼 패드', '75 × 310 × 1 mm', '배터리 밑', 1),
    # v5 (11-09, fusion/split_v5.py): the two-piece battery tray and deck
    'FX ISO 7380 M3x10 split': ('몸통', '버튼 볼트 ISO 7380', 'M3 × 10', '배터리 받침·전장 데크 두 조각의 이음 턱 (조각 이음마다 2)', 4),
    'FX DIN 934 M3 split': ('몸통', '육각 너트 DIN 934', 'M3', '배터리 받침·전장 데크 이음 턱 (+y 조각 쪽)', 4),
}
INCLUDED = ('FX pi3hat M2.5x5',)          # comes with the pi3hat: no order line


def not_modelled(n):
    """Joint parts of the v5 legs that are not modelled, from the model counts of the parts that carry them."""
    hubs = n['thigh_L'] + n['thigh_R'] + n['shank_L'] + n['shank_R']
    caps = n['kneecap_L'] + n['kneecap_R']
    return [
        ('관절', '접시 볼트 ISO 10642 12.9', 'M3 × 16', '⑥ hub 인서트 플랜지(접시, 면과 같게) → 링크 벽 6 → 받침 링 탭(출력 부품, 손으로 가볍게, '
         '혐기성 고정제 쓰지 않음) (hub 인서트마다 %d)' % FLANGE_BOLTS, FLANGE_BOLTS * hubs),
        ('관절', '저두 볼트 DIN 7984 8.8', 'M2.5 × 8', '⑦ 무릎 캡 플랜지 → 허벅지 바깥 막대 인서트 (캡마다 %d)' % CAP_BOLTS, CAP_BOLTS * caps),
        ('관절', '열압입 인서트 황동', 'M2.5 × 4 (구멍 Ø3.6)', '⑦ 무릎 캡 자리(허벅지 바깥 막대, 구멍 깊이 7)', CAP_BOLTS * caps),
        ('몸통', '벨크로 스트랩', '폭 25 × 400 mm', '배터리 + 받침을 척추 봉에 묶음(받침 슬롯)', 2),
    ]


def spare(item, k):
    """Order quantity: small parts get some spare (lost / spoilt in assembly)."""
    if '인서트' in item:
        return k + max(4, math.ceil(0.2 * k))
    if '핀' in item:
        return k + 3
    if any(w in item for w in ('볼트', '너트', '와셔', '나사')):
        return k + max(2, math.ceil(0.1 * k))
    return k


# wiring: not in the model (user 11-01); topology of robot_cad_v2 README 8 (= v4 / the v5 route of git 5a01c18)
WIRING = [  # (item, spec, need, order, note)
    ('XT30U 플러그 수(M), 16 AWG 납땜형', '전원 케이블 끝', 3, '— (동봉)', '전원보드 P5 → pi3hat, P3 → 왼 hip, P6 → 오른 hip: 수-암 케이블 3개의 보드 쪽 끝 '
     '(power dist r4.5b에 6개 동봉)'),
    ('XT30U 플러그 암(F), 16 AWG 납땜형', '전원 케이블 끝', 11, 13, '위 3개의 드라이버 쪽 끝 + hip → 무릎 → 바퀴 암-암 4개(양 끝)'),
    ('16 AWG 실리콘선 빨강', '전원', '약 2.7 m', '5 m', '어림: v2 모델 길이 2.4 m + 링크 25 cm로 다리마다 +0.15 m, 실물에서 재서 자름'),
    ('16 AWG 실리콘선 검정', '전원', '약 2.7 m', '5 m', '같음'),
    ('mjbots JST PH3 Cable 30 cm', 'CAN-FD용 꼬은 3선, 1:1', 2, 2, 'pi3hat JC1 → 왼 hip, JC2 → 오른 hip (몸통 안)'),
    ('mjbots JST PH3 Cable 50 cm', '같음', 4, 4, 'hip → 무릎, 무릎 → 바퀴 × 좌우. 25 cm 링크의 다리 경로는 어림 45~55 cm(모델 없음): '
     '실물에 대 보고 모자라면 PHR-3 하우징 + SPH-002T 단자 + 26 AWG 꼬은선으로 직접 만든다'),
    ('mjbots JST PH3 Cable 10 cm', '같음', 1, 1, 'pi3hat JC3 → power dist CAN1 (자동 종료·전력 측정)'),
    ('XT90(F) → XT60(M) 변환 케이블', '12 AWG', 1, 1, '배터리 XT60(F) 리드 → power dist XT90(M). 동봉 XT90S 암에 XT60 수를 납땜해 만들어도 된다'),
    ('접착식 케이블타이 마운트', '20 × 20 mm 안팎', '약 16', 20, '링크에 구멍이 없어 다리 배선은 허벅지·정강이 뒷면을 따라 붙인다(git 5a01c18 경로)'),
    ('케이블타이', '100 × 2.5 mm', '약 30', '1봉(100개)', '마운트·다발 묶음'),
    ('수축 튜브', 'Ø3·Ø6 안팎', '-', '1세트', 'XT30 납땜부, 선 끝'),
]
TOOLS = [   # (item, where); {inserts} = heat-set inserts in the BOM, {m3} {m4} {m5} = tapped holes in the former machined parts
    ('핸드 탭 M3 · M4 · M5 + 탭 핸들', '예전 가공 부품의 탭 구멍(출력 뒤): M3 {m3}(hip 판 · 받침 링), M4 {m4}(레일 끝), M5 {m5}(척추 봉 끝)'),
    ('드릴 날 Ø2.5 · 3.0 · 3.3 · 4.2', '탭 전에 출력 구멍 정리(M3 · M4 · M5), hub 인서트 평행핀 구멍 Ø3'),
    ('사포 #240~400 · 작은 줄', '끼움 자리 다듬기: hip 판 Ø42 파일럿, hub 인서트 스피곳·플랜지, 무릎 캡 관, 자석 스페이서'),
    ('나사 고정제 (선택)', '모터 금속 탭에 들어가는 ①·② 나사에만 소량. 출력 부품의 나사산(⑥, hip 판·봉·레일 탭)에는 혐기성 고정제(243 등)를 '
     '쓰지 않는다 — PLA·PETG가 갈라질 수 있다'),
    ('순간접착제', 'AKE 부속 자석 → 출력 자석 스페이서 구멍'),
    ('열압입 인서트 인두 팁 M2.5 · M3 · M4', '출력 부품 인서트 {inserts}개'),
    ('JST PH 압착 공구', 'mjbots 케이블이 다리 경로에 모자랄 때만: CAN 케이블 단자(SPH-002T)'),
    ('리머 Ø35 H7', '허벅지 바깥 막대의 무릎 캡 구멍 2곳(0.2 작게 출력한 뒤)'),
    ('2.5 mm 육각 볼드라이버, 축 120 mm 이상', 'hip·무릎 출력 볼트 ②를 Ø44 접근 구멍 안으로'),
    ('육각 렌치 1.5 · 2 · 2.5 · 3 mm', 'DIN 7984 M2.5(1.5), ISO 4762 M2.5 · 10642 M3 · 7380 M3(2), ISO 4762 M3 · 10642 M4(2.5), '
     'ISO 4762 M4 · 10642 M5(3)'),
    ('스패너 5.5 · 7 mm', 'M3 너트(셸 이음판, 받침·데크 이음 턱), M4 나일론 너트(위 레일)'),
]


# ----------------------------------------------------------------------------- build the BOM
def classify(n):
    """Every STEP component goes to exactly one group, or the script stops."""
    left = dict(n)
    for k in list(left):
        if k.startswith(CONTAINERS) or k.startswith(IN_MOTOR):
            left.pop(k)
    out = {}
    for prefix, *_r in PURCHASED:
        out[prefix] = sum(left.pop(k) for k in [k for k in left if k.startswith(prefix)])
    for prefix in EX_STEP:                            # counted in EX_MACHINED (kneecap_: knee cap)
        out[prefix] = sum(left.pop(k) for k in [k for k in left if k.startswith(prefix)])
        assert out[prefix], prefix
    for prefix, *_r in PRINTED:
        out[prefix] = {k: left.pop(k) for k in sorted(k for k in left if k.startswith(prefix))}
    for k in FX:
        out[k] = left.pop(k, 0)
    out['61807'] = sum(left.pop(k) for k in [k for k in left if k.startswith('kneebearing_')])
    assert not left, 'model parts with no BOM line: %s' % left
    return out


def print_row(dwg, name, item, spec, colour, axis, note, mat, k, profile):
    vol, area, box, bed, h = stl_info(name, axis)
    wall, infill = profile
    sh = min(vol, area * wall)
    g = (sh + infill * (vol - sh)) * RHO
    return (dwg, name, item, spec, colour, k, g * k, vol, box, bed, note, mat, h)


def rows_all(n):
    c = classify(n)
    bad = {k: (c[k], v[4]) for k, v in FX.items() if c[k] != v[4]}
    assert not bad, 'fastener counts differ from the v4 fasteners.specs: %s' % bad
    R = {'buy': [], 'ex': [], 'print': [], 'fx': [], 'wire': WIRING, 'tools': []}
    for prefix, item, spec, src, note in PURCHASED:
        assert c[prefix], prefix
        R['buy'].append((item, spec, c[prefix], 0 if '동봉' in src else c[prefix], src, note, '있음'))
    R['buy'].append(('깊은 홈 볼 베어링 61807', 'Ø35 × Ø47 × 7, 2RS 또는 ZZ', c['61807'], c['61807'], '베어링 판매점',
                     '정강이 머리 패드 바깥 면 자리(Ø47 압입), 안륜은 무릎 캡 관', '있음'))
    R['buy'].append(('microSD 카드', '32 GB 이상(A1/A2)', 1, 1, '전자 부품상', '라즈베리파이 OS', '없음'))
    metal = 0.0
    for dwg, item, was, spec, axis, after, parts in EX_MACHINED:
        for name, fn, rho in parts:
            k = fn(n)
            assert k, name
            R['ex'].append(print_row(dwg, name, item, spec, EX_COLOUR, axis, '출력 뒤: %s. (예전: %s)' % (after, was),
                                     HOT if dwg in EX_HOT else MAT, k, (99.0, 1.0)))
            metal += R['ex'][-1][7] * rho * k
    # model counts of the former machined STEP components = the printed rows that come from them
    for prefix in EX_STEP:
        rows = [r for r in R['ex'] if r[1].startswith(prefix.replace(' ', '_').replace('kneecap_', 'knee_cap_'))]
        assert sum(r[5] for r in rows) == c[prefix], (prefix, c[prefix])
    for prefix, dwg, item, spec, colour, axis, note, mat in PRINTED:
        for name, k in c[prefix].items():
            prof = PRINT_PROFILE[next(p for p in PRINT_PROFILE if name.startswith(p))]
            R['print'].append(print_row(dwg, name, item, spec, colour, axis, note, mat, k, prof))
    over = [(r[1], round(r[9])) for r in R['ex'] + R['print'] if r[9] > BED and r[1] != 'splice_tabs']
    assert not over, 'parts that do not fit the %.0f mm bed: %s' % (BED, over)
    for name, (grp, item, spec, where, _e) in FX.items():
        if name not in INCLUDED:
            R['fx'].append((grp, item, spec, c[name], where, '있음'))
    for grp, item, spec, where, k in not_modelled(n):
        R['fx'].append((grp, item, spec, k, where, '없음 ★'))
    ins = sum(r[3] for r in R['fx'] if '인서트' in r[1])
    hp = c['hip_plate_']
    taps = dict(m3=HIP_PLATE_M3 * hp + FLANGE_BOLTS * sum(r[5] for r in R['ex'] if r[1].startswith('hub_ring')),
                m4=END_TAPS * c['rail_'], m5=END_TAPS * c['spine_tube_'])
    R['tools'] = [(a, b.format(inserts=ins, **taps)) for a, b in TOOLS]
    R['metal'] = metal
    return R, c


def fx_totals(fx):
    """Order lines: the same item and size summed over every place it is used."""
    t = {}
    for grp, item, spec, k, where, modelled in fx:
        key = (item, spec.split(' (')[0])
        a = t.setdefault(key, [0, [], spec])
        a[0] += k
        a[1].append('%s × %d' % (where.split(' — ')[0].strip(), k))
    return [(item, spec, k, spare(item, k), '; '.join(w)) for (item, _s), (k, w, spec) in sorted(t.items(), key=lambda x: x[0])]


def fmt_q(k):
    return '%d' % k if isinstance(k, int) else str(k)


def print_table(rows):
    L = ['| 도번 | 파일 | 부품 | 재료 | 설정·놓는 면 | 색 | 수량 | 출력 질량(어림) | 외곽(mm) | 놓은 면 기준 베드 / 높이 | 비고 |',
         '|---|---|---|---|---|---|---|---|---|---|---|']
    L += ['| %s | `parts_stl/%s.stl` | %s | %s | %s | %s | %d | %.0f g | %s | %.0f mm / %.0f mm | %s |'
          % (r[0], r[1], r[2], r[11], r[3], r[4], r[5], r[6], ' × '.join('%.0f' % x for x in r[8]), r[9], r[12], r[10]) for r in rows]
    return L


def write(R, c, leaves):
    ex, pr = R['ex'], R['print']
    allp = ex + pr
    g_ex, g_pr = sum(r[6] for r in ex), sum(r[6] for r in pr)
    g_tot = g_ex + g_pr
    colours = defaultdict(float)
    for r in allp:
        colours[r[4]] += r[6]
    fit = [r for r in allp if r[1] != 'splice_tabs']
    widest = max(fit, key=lambda r: r[9])
    tallest = max(fit, key=lambda r: r[12])
    tot = fx_totals(R['fx'])
    n_fx = sum(r[3] for r in R['fx'])
    solid_ex = sum(r[7] * r[5] for r in ex) * RHO
    L = ['# WheelBiped v5 BOM — 주문용', '',
         '`python manufacturing/bom.py`가 만든다(손으로 고치지 말 것). 모델에 있는 부품의 수량은 **조립 STEP**(`export/WheelBiped_v5_assembly.step`, '
         '%s, %.3f kg — 금속 부품 기준)의 인스턴스 수이고, 모델 부품이 하나라도 분류되지 않으면 스크립트가 멈춘다. ' % (MODEL, MODEL_KG) +
         '모델에 없는 것(v5 관절 볼트 ⑥·⑦과 그 인서트, 배선, 공구)은 손으로 넣었고 **★ 또는 "모델 없음"** 으로 표시했다. '
         '출력 질량과 베드 크기는 `export/parts_stl`에서 계산했다. 가격은 넣지 않았다.', '',
         '**11-09 사용자: 모든 부품을 3D 프린터(최대 %.0f × %.0f mm)로 PLA 또는 PETG로 뽑는다. 강도는 보지 않는다.** 그래서 가공 주문은 없고, '
         '예전 가공 부품(6061·SUS304)은 2절에서 출력 부품으로 바뀌었다. 놓는 면을 정한 출력 부품이 베드에 안 들어가면 스크립트가 멈춘다.' % (BED, BED), '',
         '## 0. 한눈에', '',
         '| 묶음 | 종류 | 개수 | 어디에 |', '|---|---|---|---|',
         '| 1. 구매품(모터·전장·바퀴) | %d | %d | mjbots, CubeMars, 전자 부품상, 베어링 |' % (len(R['buy']), sum(r[2] for r in R['buy'])),
         '| 2. 3D 출력 — 예전 가공 부품(가공 주문 없음) | %d | %d | 직접 출력 + 탭·다듬기. 필라멘트 약 %.2f kg |'
         % (len(ex), sum(r[5] for r in ex), g_ex / 1000),
         '| 3. 3D 출력 — 그 밖의 출력 부품 | %d | %d | 직접 출력. 필라멘트 약 %.1f kg |' % (len(pr), sum(r[5] for r in pr), g_pr / 1000),
         '| 4. 체결 부품 | %d | %d (주문 권장 %d) | 나사 쇼핑몰 |' % (len(tot), n_fx, sum(r[3] for r in tot)),
         '| 5. 배선·커넥터(모델 없음) | %d | — | 전자 부품상 |' % len(R['wire']),
         '| 6. 공구·소모품(있으면 생략) | %d | — | |' % len(R['tools']), '',
         '출력 필라멘트 합계(2·3절) 약 **%.1f kg**, 서포트·브림·실패 여유 15 %%를 더하면 **%.1f kg → 1 kg 스풀 %d개**. '
         '조립 STEP의 부품 인스턴스 %d개(링크 묶음 torso·femur·tibia·wheel 7개를 더하면 %d개)를 모두 아래 줄로 나눴다.'
         % (g_tot / 1000, g_tot * WASTE / 1000, math.ceil(g_tot * WASTE / 1000), leaves, leaves + 7), '']
    L += ['## 1. 구매품', '',
          '| 품목 | 규격 | 필요 | 주문 | 구입처 | 비고 |', '|---|---|---|---|---|---|']
    L += ['| %s | %s | %d | %s | %s | %s%s |' % (r[0], r[1], r[2], r[3] if r[3] else '— (동봉)', r[4], r[5],
                                               '' if r[6] == '있음' else ' — 모델 없음') for r in R['buy']]
    L += ['', '## 2. 3D 출력 — 예전 가공 부품 (11-09: 가공 주문 없음)', '',
          '가공으로 주문하려던 부품 %d종 %d개를 **같은 STL로 출력**한다. 모양은 그대로다(모델을 바꾸지 않음). 공통:'
          % (len({r[0] + r[2] for r in ex}), sum(r[5] for r in ex)), '',
          '- **채움 100 %%**(속까지 꽉 차게): 탭 나사산과 끼워맞춤 자리가 속 빈 채움에 걸리지 않게 한다. 모두 작아서 필라멘트는 %.2f kg뿐이다.' % (g_ex / 1000),
          '- **탭**: 금속 가공 때의 탭 자리가 STL에 기초 구멍(M3 Ø2.5, M4 Ø3.3, M5 Ø4.2)으로 들어 있다. 그대로 뽑고, 같은 지름 드릴로 한 번 정리한 뒤 '
          '핸드 탭으로 나사산을 낸다. 플라스틱 나사산이라 세게 조이지 않는다.',
          '- **끼워맞춤**(H7·g6·N7·h9 같은 가공 공차)은 출력 오차(±0.2)보다 작다. 맞닿는 실물(모터, 61807, 핀)에 대 보며 드릴·사포로 다듬는다. '
          '아노다이징·버 제거 같은 금속 후처리는 없다.',
          '- 도면(`manufacturing/drawings/`의 hip_plate·spine_tube·rail·clamp_ring_stator·magnet_spacer·hub_insert_hip·hub_insert_knee·'
          'knee_cap·hub_ring, `v2_*.pdf`)은 **치수 참고용**이다. 재질·공정 칸(6061·SUS304, 선반·MCT·아노다이징)은 예전 가공 기준이다.',
          '- 모델(질량 %.3f kg)에서 이 부품들은 금속으로 %.2f kg이고, 꽉 찬 PETG로 뽑으면 %.2f kg이다. **로봇이 약 %.2f kg 가벼워진다**. '
          'CAD 재질·`calc/mass_properties.json`·URDF는 아직 금속 기준이다.'
          % (MODEL_KG, R['metal'] / 1000, solid_ex / 1000, (R['metal'] - solid_ex) / 1000), '']
    L += print_table(ex)
    L += ['', '**뽑지 않는 것**: `clamp_ring_output_D27_t2.0_SUS304`(v4 출력 클램프 링). v5에서는 hub 인서트가 대신하고 모델에도 없지만 '
          '`export/parts_step`·`parts_stl`에 파일이 남아 있다. 같은 폴더의 `magnet_spacer_…__magnet_D6x2.5_N38H_(AKE).stl`은 모터 부속 자석이라 뽑지 않는다.', '',
          '## 3. 3D 출력 — 그 밖의 출력 부품 (PLA 또는 PETG)', '']
    L += print_table(pr)
    L += ['', '- **재료**: PLA 또는 PETG(사용자 11-09). "PETG 권장(모터 열)"은 모터·드라이버에 닿는 부품이다 — PLA는 55~60 °C 안팎에서 물러지는데, '
          '모터·드라이버는 오래 돌리면 그 근처까지 데워질 수 있다. 강도는 보지 않는다(사용자).',
          '- 출력 질량은 v4 `check.print_fraction`의 식(벽 두께만큼 꽉 차고 속은 채움 비율)으로 STL에서 어림했고 PETG 밀도(1.27) 기준이다. '
          'PLA(1.24)는 약 2 % 가볍다. 허벅지·정강이는 꽉 찬 출력이라 모델 값(827.5 / 420.7 g)과 같다.',
          '- 색별 합계(2·3절): %s. 여유 15 %%를 더하면 **약 %.1f kg → 1 kg 스풀 %d개**(색을 하나로 하면).'
          % (', '.join('%s %.2f kg' % (k, v / 1000) for k, v in sorted(colours.items(), key=lambda x: -x[1])), g_tot * WASTE / 1000,
             math.ceil(g_tot * WASTE / 1000)),
          '- **베드 %.0f × %.0f**(사용자): "놓은 면 기준 베드"는 표의 놓는 면으로 두고 대각선으로 돌리는 것까지 따진 가장 작은 정사각 베드 한 변이고, '
          '"높이"는 그때의 출력 높이다. **모든 부품이 들어간다** — 가장 큰 것은 %s %.0f mm(브림 %.0f mm를 둘러도 %.0f mm), 가장 높은 것은 %s %.0f mm. '
          '척추 봉·위 레일은 길이 328 mm로 베드 한 변(%.0f)보다 길지만 베드 대각선은 %.0f mm라, 45° 돌려 대각선 방향으로 놓으면 들어간다. '
          '부품 길이는 그대로이고, 243 mm는 그렇게 놓았을 때 필요한 정사각 베드 한 변이다(봉 끝이 베드 가장자리에서 약 %.0f mm 안쪽).'
          % (BED, BED, widest[2], widest[9], BRIM, widest[9] + 2 * BRIM, tallest[2], tallest[12], BED, BED * math.sqrt(2),
             (BED - (328.0 + 16.0) / math.sqrt(2)) / 2),
          '- **배터리 받침·전장 데크는 두 조각으로 뽑아 잇는다**(사용자 11-09, `fusion/split_v5.py`): 한 덩어리로는 대각선으로도 304 / 309 mm가 '
          '필요했다. 자른 면마다 바닥 밑으로 이음 턱(두께 %.0f, 깊이 %.0f mm)이 있어 두 턱을 맞대고 ISO 7380 M3 × %d 버튼 볼트 + DIN 934 너트 2쌍으로 '
          '조인다(머리는 -y 조각 쪽, 육각 렌치 2 mm·스패너 5.5 mm). 원하면 턱 면을 접착해도 된다. 조일 때 두 조각의 윗면을 평평한 판에 함께 대고 맞춘다. '
          '배터리 받침은 가운데 리브 자리(y %.0f)에서 나뉘고 조각마다 척추 봉 받침과 벨크로 슬롯이 있어 따로도 얹힌다. 전장 데크는 y %.0f(라즈베리파이 '
          'USB 끝과 +y 끝 창 사이)에서 나뉘고, 이음 턱은 리브 바깥 면까지(셸 바닥 경사와 1.5 mm)다. 6자세 간섭 0건.'
          % (SP.FLANGE_T, SP.FLANGE_D, SP.SCREW_L, SP.TRAY_CUT_Y, SP.DECK_CUT_Y),
          '- 인서트는 모두 열압입이다.', '']
    L += ['## 4. 체결 부품·베어링', '',
          '### 4-1. 주문용 합계 (같은 품목·크기를 모음)', '',
          '주문 권장 = 필요 + 여유(나사·와셔·너트 +10 %, 최소 2 / 인서트 +20 %, 최소 4 / 핀 +3). ★는 모델에 없는 v5 관절 부품이 들어간 줄.', '',
          '| 품목 | 규격 | 필요 | 주문 권장 | 쓰는 곳(수량) |', '|---|---|---|---|---|']
    star = {(f[1], f[2].split(' (')[0]) for f in R['fx'] if f[5] != '있음'}
    L += ['| %s%s | %s | %d | %d | %s |' % (r[0], ' ★' if (r[0], r[1].split(' (')[0]) in star else '', r[1], r[2], r[3], r[4]) for r in tot]
    L += ['', '- pi3hat의 M2.5 × 5 나사 4개와 스탠드오프 4개는 pi3hat에 들어 있어 뺐다.',
          '- 12.9 볼트(①·②·⑥·허브)는 6061 부품을 조이던 때의 등급이다. 이제는 모두 출력 부품을 조이거나 출력 부품의 탭에 들어가고 강도는 보지 않으므로 '
          '(사용자 11-09) **더 싼 등급(8.8·A2)으로 사도 된다**. ⑥은 출력 받침 링의 플라스틱 나사산이라 0.7 N·m·나사 고정제 없이 손으로 가볍게 조인다.',
          '- ISO 4762 M3 × 14는 12.9(허브) 12 + 8.8(림) 12다. 같은 길이이므로 한 등급으로 24개를 사도 된다.', '',
          '### 4-2. 쓰는 곳별 상세', '',
          '| 묶음 | 품목 | 규격 | 수량 | 위치 | 모델 |', '|---|---|---|---|---|---|']
    L += ['| %s | %s | %s | %d | %s | %s |' % (r[0], r[1], r[2], r[3], r[4], r[5]) for r in R['fx']]
    L += ['', '①~⑦은 관절 단면도(`manufacturing/explain/hip_section.png`, `knee_section.png`, `wheel_section.png`)의 번호다.', '',
          '## 5. 배선·커넥터 (모델 없음)', '',
          '사용자 요청(11-01)으로 배선은 모델에 없다. 연결은 v2 README 8절과 같다: 전원보드 P5 → pi3hat, P3·P6 → 왼·오른 hip → 무릎 → 바퀴(XT30 데이지 체인, 16 AWG), '
          'pi3hat JC1·JC2 → 각 다리 hip → 무릎 → 바퀴(CAN, 바퀴 CAN1에 종단 저항), pi3hat JC3 → power dist CAN1(11-08). '
          'CAN은 mjbots 기성 케이블(11-08 사용자 주문), 전원선 길이는 어림이므로 실물에서 재서 자른다. '
          '한 번 깔아 본 v5 다리 경로는 git `5a01c18`(`fusion/harness_v5.py`, 17.5 cm 링크).', '',
          '| 품목 | 규격 | 필요 | 주문 권장 | 비고 |', '|---|---|---|---|---|']
    L += ['| %s | %s | %s | %s | %s |' % (r[0], r[1], fmt_q(r[2]), fmt_q(r[3]), r[4]) for r in R['wire']]
    L += ['', '커넥터 암·수는 실물 보드를 보고 한 번 더 확인한다(moteus·pi3hat은 XT30 수, 전원보드 출력은 암 기준으로 셈).', '',
          '## 6. 공구·소모품 (이미 있으면 생략)', '', '| 품목 | 쓰는 곳 |', '|---|---|']
    L += ['| %s | %s |' % r for r in R['tools']]
    L += ['', '## 7. 주문 전 확인', '',
          '- **가공 주문은 없다**(11-09 사용자: 모두 3D 출력, PLA 또는 PETG, 강도는 보지 않음). 예전 가공 부품은 출력한 뒤 탭·다듬기가 필요하다(2절).',
          '- **타이어**는 v2 때 이미 주문했다(AliExpress). 받았는지만 확인한다. **배터리**는 가지고 있는 24 V 15 Ah(XT60) 기준이다.',
          '- **hip 판·hub 인서트는 왼·오른쪽이 거울상**이라 STL이 따로 있다(각 1개씩). 받침 링 4개, 무릎 캡 2개는 같은 부품이다.',
          '- v2 도면(S14~S18)은 금속 가공용이고 이름이 v2 기준이다: `v2_hip_plate_L.pdf` = 이 로봇의 `hip_plate_R`(CAD +y). 출력은 STL로 한다.',
          '- 61807은 2RS(고무 실)나 ZZ(금속 실) 어느 쪽이든 폭 7 mm면 맞는다.',
          '- 프린터 베드 %.0f × %.0f에 모두 들어간다. 배터리 받침·전장 데크는 두 조각씩 뽑아 M3로 잇는다(3절).' % (BED, BED),
          '- 금속 → 출력으로 로봇이 약 %.2f kg 가벼워진다(2절). 강도 계산(`calc/strength_v5.md`)과 질량·URDF는 아직 금속 기준이다.'
          % ((R['metal'] - solid_ex) / 1000),
          '- 남은 설계 결정(README 5절: 정강이 처짐 기준, hip ②·⑥ 검토 하중 초과, 무릎 완전 신장 멈춤)은 부품 목록을 바꾸지 않는다. '
          '기구 멈춤을 넣기로 하면 그때 부품이 늘어난다.']
    md = '\n'.join(L) + '\n'
    with open(os.path.join(BASE, 'BOM.md'), 'w', encoding='utf-8') as f:
        f.write(md)
    with open(os.path.join(BASE, 'BOM.csv'), 'w', encoding='utf-8-sig', newline='') as f:
        w = csv.writer(f)
        w.writerow(('묶음', '품목', '규격·설명', '필요 수량', '주문 권장', '도번·파일', '구입처·공정', '위치·비고', '모델에 있음'))
        for r in R['buy']:
            w.writerow(('1 구매품', r[0], r[1], r[2], r[3], '', r[4], r[5], r[6]))
        for grp, rows in (('2 출력(예전 가공)', ex), ('3 출력', pr)):
            for r in rows:
                w.writerow((grp, r[1], '%s; %s; %s; 색 %s' % (r[2], r[11], r[3], r[4]), r[5], r[5],
                            '%s; export/parts_stl/%s.stl' % (r[0], r[1]),
                            '출력 약 %.0f g, 정사각 베드 %.0f mm, 높이 %.0f mm' % (r[6], r[9], r[12]), r[10], '있음'))
        for r in tot:
            w.writerow(('4 체결(주문 합계)', r[0], r[1], r[2], r[3], '', '', r[4], '★ 일부 없음' if (r[0], r[1].split(' (')[0]) in star else '있음'))
        for r in R['wire']:
            w.writerow(('5 배선', r[0], r[1], r[2], r[3], '', '', r[4], '없음'))
        for r in R['tools']:
            w.writerow(('6 공구·소모품', r[0], '', '', '', '', '', r[1], '없음'))
    return md


def main():
    n, leaves = step_counts()
    R, c = rows_all(n)
    md = write(R, c, leaves)
    th = stl_info('thigh_R', 0)
    sh = stl_info('shank_R', 0)
    print('thigh %.1f cm3 %.1f g, shank %.1f cm3 %.1f g (README 651.4 / 331.1 cm3, 827.5 / 420.7 g)'
          % (th[0] / 1000, th[0] * RHO, sh[0] / 1000, sh[0] * RHO))
    print('former machined: metal %.0f g -> printed %.0f g' % (R['metal'], sum(r[7] * r[5] for r in R['ex']) * RHO))
    print('leaf instances %d, BOM.md %d lines' % (leaves, md.count('\n')))


if __name__ == '__main__':
    main()
