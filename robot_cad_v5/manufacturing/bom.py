"""WheelBiped v5 BOM for ordering (11-08, user: "v5 모델의 bom 파일").

Quantities of every modelled part are the instance counts in the exported assembly STEP (export/WheelBiped_v5_assembly.step =
hub version 14).  Every STEP component must be classified below, so a part added to the model and
not to this file stops the script.  Printed masses and bed sizes come from export/parts_stl.  Parts that are not in the model
(the v5 joint screws 6 and 7 and their inserts, wiring, tools) are added by hand and marked as such.

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

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.dirname(HERE)
STEP = os.path.join(BASE, 'export', 'WheelBiped_v5_assembly.step')
STL = os.path.join(BASE, 'export', 'parts_stl')
RHO = 1.27e-3                      # PETG g/mm3 (v4 params.PETG_RHO)
WASTE = 1.15                       # filament: + 15 % for supports, brims and failed prints
FLANGE_BOLTS, CAP_BOLTS = 6, 3     # legs_v5: _flange_bolts (6 per hub insert, r 18), _cap_bolts (3 per knee cap, r 20.5)


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
    """Volume mm3, area mm2, sorted box, smallest square bed - lying on the face normal to axis (STL x, y, z), or lying flat
    (on the axis of the smallest extent) when axis is None."""
    t = stl(os.path.join(STL, name + '.stl'))
    a, b, c = t[:, 0], t[:, 1], t[:, 2]
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1).sum()
    vol = abs(np.einsum('ij,ij->i', a, np.cross(b, c)).sum()) / 6.0
    p = t.reshape(-1, 3)
    ext = p.max(0) - p.min(0)
    ax = int(np.argmin(ext)) if axis is None else axis
    return vol, area, sorted(ext.round(0), reverse=True), square_bed(p[:, [i for i in range(3) if i != ax]])


# ----------------------------------------------------------------------------- what every model part is
# (STEP component name or prefix, as in the v4 / v5 code)
CONTAINERS = ('Untitled', 'torso', 'femur_', 'tibia_', 'wheel_')          # assembly components: no part of their own
IN_MOTOR = ('AKE60-8_KV80_output',)                                        # the rotor half of the bought AKE60 STEP

PURCHASED = [   # (prefix, item, spec, source, note)
    ('AKE60-8_KV80_stator', 'CubeMars AKE60-8 KV80', '8:1, 피크 12.5 / 정격 5 N·m, 0.26 kg, Ø69 × 25', 'CubeMars',
     'hip·무릎·바퀴 × 좌우. 부속 자석 Ø6 × 2.5 N38H(엔코더용)를 6061 자석 스페이서에 붙여 쓴다'),
    ('moteus_r4.11', 'mjbots moteus r4.11', 'XT30PW-M ×2, JST PH-3 CAN ×2, 10~44 V', 'mjbots', '관절마다 1개, 드라이버 컵 안'),
    ('raspberry_pi_4b', 'Raspberry Pi 4 Model B', '4 GB 이상', '라즈베리파이 판매점', 'microSD는 이 표 맨 아래'),
    ('pi3hat_r4.5', 'mjbots pi3hat r4.5', 'CAN-FD ×5, IMU, 24 V → 5 V, M2.5 × 11 스탠드오프·나사 동봉', 'mjbots',
     '파이에는 USB-C 전원을 넣지 않는다(pi3hat이 5 V를 만든다)'),
    ('standoff M2.5x11', 'M2.5 × 11 육각 스탠드오프(수-암)', 'pi3hat 동봉 — 따로 사지 않는다', '(pi3hat 동봉)', '파이 ↔ pi3hat'),
    ('mjpower_ss', 'mjbots power dist (mjpower-ss)', 'XT90 입력, XT30 출력 ×6, 프리차지, 10~54 V', 'mjbots', '배터리를 파이·pi3hat에 바로 물리지 않는다'),
    ('battery_24V_15Ah', '배터리 24 V 15 Ah', '75 × 310 × 75 mm, XT60(F) 리드', '(가지고 있는 배터리)', '새로 살 때는 크기·커넥터가 같은 것'),
    ('fan 30x30x7', '팬 30 × 30 × 7 mm 5 V', '2핀', '전자 부품상', 'pi3hat JP6(1번 5 V, 3번 GND), 데크에 1.5 mm 양면 폼테이프'),
    ('rocker switch', '로커 스위치 19 × 13 + mjbots PH4 스위치 하네스', 'mjpower-ss 스위치 헤더용', 'mjbots / 전자 부품상',
     '몸통 앞면(CAD −x) 수직면, 로봇 오른쪽(CAD +y)의 19 × 13 구멍'),
    ('CAN terminator', 'CAN-FD 종단 저항 120 Ω (JST PH-3)', '20 × 15 × 10 이하', 'mjbots (또는 120 Ω 저항 + PHR-3로 직접)',
     '바퀴 드라이버 CAN1(다리 끝). moteus에는 종단 저항이 없고 pi3hat 쪽에는 있다'),
    ('XT30 cap', 'XT30 보호 캡', 'XT30 수 커넥터용', '전자 부품상', '바퀴 드라이버 J2(24 V가 살아 있는 빈 포트)'),
    ('tyre M365', 'Xiaomi M365 8.5 × 2 (8 1/2 × 2) 통고무 허니콤 타이어', '외경 202 × 폭 54, 안지름 141 × 안쪽 폭 36.5, 721 g',
     'AliExpress', '사용자가 이미 주문함(v2 때) — 받았는지 확인'),
]

MACHINED = [    # (prefix, drawing, item, material / process, files, note)
    ('hip_plate_', 'WBv5-S14 (치수: v2 WBv2-M01)', 'hip 판', '6061-T6 판 10t 이상(완성 9.5), 3축 MCT 양면, Ø42 H7, M3 탭 11, 검정 아노다이징',
     'export/parts_step/hip_plate_L·R.step, manufacturing/drawings/hip_plate.pdf, v2_hip_plate_L·R.pdf',
     '좌우 거울상. v2 도면 이름은 반대: v2_hip_plate_L = 이 로봇의 hip_plate_R'),
    ('spine_tube_', 'WBv5-S15 (v2 WBv2-M05)', '척추 봉', '6061-T6 환봉 Ø16 통짜, 길이 328.00 ±0.05, 양 끝 면삭 + M5 탭 깊이 11',
     'export/parts_step/spine_tube_F.step, drawings/spine_tube.pdf, v2_spine_tube.pdf', '앞·뒤 같은 부품. 레일 2개와 같은 세팅으로 길이 맞춤'),
    ('rail_', 'WBv5-S16 (v2 WBv2-M06)', '위 레일', '6061-T6 각봉 15 × 15 통짜, 길이 328.00 ±0.05, 양 끝 M4 탭 깊이 9, Ø4.4 관통 4',
     'export/parts_step/rail_F.step, drawings/rail.pdf, v2_rail.pdf', '앞·뒤 같은 부품'),
    ('clamp ring stator', 'WBv5-S17 (v2 WBv2-M08)', '고정자 클램프 링 Ø45', 'SUS304 2B 2t 레이저, Ø45 / Ø30, Ø2.8 × 6 (PCD 36.5)',
     'export/parts_step/clamp_ring_stator_D45_t2.0_SUS304.step, drawings/clamp_ring_stator…pdf, robot_cad_v2/manufacturing/dxf/clamp_ring_stator.dxf',
     '무릎·바퀴 고정자 나사 머리 밑(PETG 판 위). 작아서 +2 여유 권장'),
    ('magnet spacer', 'WBv5-S18 (v2 WBv2-M02)', '자석 스페이서 Ø9 × 5.35', '6061(비자성 필수) 선반, Ø6.20 × 2.5 자석 포켓, Ø5.90 × 2.3 스피곳',
     'export/parts_step/magnet_spacer_D9x5.35_(6061).step, drawings/magnet_spacer…pdf', '모터마다 1개. 작아서 +2 여유 권장'),
]
V5_MACHINED = [  # parts that are bodies of the v5 leg components: (drawing, item, spec, files, note, count from model counts)
    ('WBv5-M01', 'hip hub 인서트 왼쪽 (hub_insert_hip_L)', '6061-T6 환봉 Ø50, 선반 + MCT: Ø28 보스, Ø11.8 × 1.2 스피곳, 플랜지 Ø48 × 6, '
     'M3 Ø3.4 × 6 (PCD 20, 와셔 자리), 핀 Ø3 N7 × 7.5 (+0.2/0) × 3, 접시 Ø7 × 6 (r 18), 검정 아노다이징 권장(끼움면 마스킹)',
     'manufacturing/step/hub_insert_hip_L.step, drawings/hub_insert_hip.pdf', '왼·오른쪽은 거울상 = 다른 부품', lambda n: n['thigh_L']),
    ('WBv5-M01', 'hip hub 인서트 오른쪽 (hub_insert_hip_R)', '같음(거울상)', 'manufacturing/step/hub_insert_hip_R.step, drawings/hub_insert_hip.pdf',
     '', lambda n: n['thigh_R']),
    ('WBv5-M02', '무릎 hub 인서트 왼쪽 (hub_insert_knee_L)', 'hip hub 인서트와 같은 공정(보스 길이만 다름)',
     'manufacturing/step/hub_insert_knee_L.step, drawings/hub_insert_knee.pdf', '왼·오른쪽은 거울상 = 다른 부품', lambda n: n['shank_L']),
    ('WBv5-M02', '무릎 hub 인서트 오른쪽 (hub_insert_knee_R)', '같음(거울상)', 'manufacturing/step/hub_insert_knee_R.step, drawings/hub_insert_knee.pdf',
     '', lambda n: n['shank_R']),
    ('WBv5-M03', '무릎 캡 (knee_cap)', '6061-T6 환봉 Ø50, 선반 + 드릴: 관 Ø35 g6 (61807 안륜·바깥 막대 구멍), 플랜지 Ø46 × 3, '
     'Ø2.8 × 3 (r 20.5), 검정 아노다이징 권장', 'manufacturing/step/knee_cap_R.step, drawings/knee_cap.pdf', '왼·오른쪽 같은 부품',
     lambda n: n['kneecap_L'] + n['kneecap_R']),
    ('WBv5-M04', '받침 링 (hub_ring)', '6061-T6 환봉 Ø45, 선반: Ø43 / Ø29 × 6, M3 탭 관통 6 (r 18), 버 제거',
     'manufacturing/step/hub_ring_R.step, drawings/hub_ring.pdf', 'hip·무릎 × 왼·오른쪽 같은 부품',
     lambda n: n['thigh_L'] + n['thigh_R'] + n['shank_L'] + n['shank_R']),
]
NOT_ORDERED = ('clamp ring output',)   # v4 part removed from v5 (the 6061 hub inserts replace it); its STEP / STL files are left in export/

LEG_PRINT = ('PETG FDM 한 덩어리: 벽 3.5 둘레선, 채움 100 %, 닫힌 칸(배출 구멍 없음)')
PRINTED = [     # (prefix, drawing, item, settings / orientation, colour, STL axis of the face on the bed or None, note)
    ('thigh_', 'WBv5-P01', '허벅지', LEG_PRINT + ', 바깥 면(평평, y 288.5)을 바닥에 — 갈래 틈 안 서포트', '검정', 0,
     '열압입 M2.5 인서트 6(컵 3 + 무릎 캡 3), 캡 관 구멍 Ø35 H7은 0.2 작게 뽑고 리머'),
    ('shank_', 'WBv5-P02', '정강이', LEG_PRINT + ', 안쪽 면(평평, y 208)을 바닥에 — 바퀴 모터 칸 Ø74 안 서포트', '검정', 0,
     '열압입 M2.5 인서트 3(바퀴 컵), 61807 자리 Ø47 압입'),
    ('cup_hip', 'WBv5-S01', 'hip 드라이버 컵', 'PETG 30 %, 바닥(덮개 면)을 바닥에, 서포트는 커넥터 창만', '검정', None, '열압입 M2.5 인서트 4'),
    ('cup_knee', 'WBv5-S02', '무릎 드라이버 컵', 'PETG 30 %, 바닥을 바닥에, 서포트는 커넥터 창만', '검정', None, '열압입 M2.5 인서트 4'),
    ('cup_wheel', 'WBv5-S03', '바퀴 드라이버 컵', 'PETG 30 %, 바닥을 바닥에, 서포트는 커넥터 창만', '검정', None, '열압입 M2.5 인서트 4'),
    ('hub_flange', 'WBv5-S04', '바퀴 허브 플랜지', 'PETG 100 %, 모터 쪽 면을 바닥에', '검정', None, '열압입 M3 인서트 6'),
    ('rim_', 'WBv5-S05', '바퀴 림(한 조각)', 'PETG 벽 6겹(2.4 mm) · 채움 40 %, 허브 쪽 평면을 바닥에, 열린 쪽 플랜지 밑 4.5 mm 턱만 서포트',
     '검정', None, ''),
    ('shell_front', 'WBv5-S06', '몸통 셸 앞(스위치 쪽)', 'PETG 25 %, 바깥 면을 위로(서포트 필요)', '진회색', None, 'v4와 달리 세로 모서리 R30'),
    ('shell_mid', 'WBv5-S07', '몸통 셸 가운데(hip)', 'PETG 25 %, 바깥 면을 위로(서포트 필요)', '진회색', None, ''),
    ('shell_rear', 'WBv5-S08', '몸통 셸 뒤(무릎 쪽)', 'PETG 25 %, 바깥 면을 위로(서포트 필요)', '진회색', None, ''),
    ('saddle_', 'WBv5-S09', 'hip 위 안장', 'PETG 25 %', '파랑', None, ''),
    ('top_plate', 'WBv5-S10', '위판(짐 판)', 'PETG 25 %', '검정', None, '열압입 M4 인서트 35'),
    ('battery_tray', 'WBv5-S11', '배터리 받침', 'PETG 30 %, 리브를 위로', '진회색', None, ''),
    ('electronics_deck', 'WBv5-S12', '전장 데크', 'PETG 30 %', '진회색', None, '열압입 M2.5 인서트 8'),
    ('splice_tabs', 'WBv5-S13', '셸 이음판 12개(한 파일)', 'PETG 100 %', '진회색', None,
     '12개가 로봇 위치 그대로 한 파일에 있다 — 슬라이서에서 나눠 놓는다(베드 값은 무시)'),
]
PRINT_PROFILE = {   # wall mm, infill (v4 params.PRINT; v5 links solid)
    'thigh_': (99.0, 1.0), 'shank_': (99.0, 1.0), 'hub_flange': (99.0, 1.0), 'rim_': (2.4, 0.40), 'cup_': (1.6, 0.30),
    'top_plate': (1.6, 0.25), 'battery_tray': (1.6, 0.30), 'electronics_deck': (1.6, 0.30), 'shell_': (1.6, 0.25),
    'saddle_': (1.6, 0.25), 'splice_tabs': (1.6, 1.0),
}

# fasteners modelled by the v4 code ('FX ...' masters): name -> (group, item, spec, where, expected count from v4 fasteners.specs)
FX = {
    'FX ISO 4762 M2.5x6 12.9 hip': ('관절', '육각 홈붙이 볼트 ISO 4762 12.9', 'M2.5 × 6', 'hip 고정자 링 → 6061 hip 판 ①', 12),
    'FX DIN 7984 M2.5x8 stator': ('관절', '저두 볼트 DIN 7984 8.8', 'M2.5 × 8', '무릎·바퀴 고정자 링 → PETG 모터 판, SUS 클램프 링 위 ①', 24),
    'FX ISO 4762 M3x16 12.9': ('관절', '육각 홈붙이 볼트 ISO 4762 12.9', 'M3 × 16', 'hip·무릎 모터 출력 → 6061 hub 인서트 ② (와셔 위)', 24),
    'FX ISO 8734 3x10': ('관절', '평행 핀 ISO 8734 m6 경화', 'Ø3 × 10', '출력 다웰 ③: hub 인서트 12 (Ø3 N7 압입) + 바퀴 허브 플랜지 6', 18),
    'FX DIN 125 M3': ('관절', '평와셔 DIN 125 A2', 'M3', '② M3 × 16, 허브 M3 × 14, 림 M3 × 14 머리 밑', 48),
    'FX ISO 4762 M3x14 12.9 hub': ('바퀴', '육각 홈붙이 볼트 ISO 4762 12.9', 'M3 × 14', '바퀴 모터 출력 → PETG 허브 플랜지 (와셔 위)', 12),
    'FX ISO 4762 M3x14 rim': ('바퀴', '육각 홈붙이 볼트 ISO 4762 8.8', 'M3 × 14', '림 → 허브 플랜지 인서트 (와셔 위)', 12),
    'FX insert M3x5.7 hub': ('바퀴', '열압입 인서트 황동', 'M3 × 5.7 (구멍 Ø4.0)', '허브 플랜지', 12),
    'FX DIN 7984 M2.5x5 board': ('드라이버', '저두 볼트 DIN 7984', 'M2.5 × 5', 'moteus → 컵 스탠드오프 (끝이 4 mm 인서트 안, 11-08)', 24),
    'FX insert M2.5x4 cup': ('드라이버', '열압입 인서트 황동', 'M2.5 × 4 (구멍 Ø3.6)', '드라이버 컵 스탠드오프', 24),
    'FX ISO 4762 M3x40 hip cup': ('드라이버', '육각 홈붙이 볼트 ISO 4762', 'M3 × 40', 'hip 컵 → hip 판 M3 탭', 6),
    'FX ISO 4762 M2.5x18 knee cup': ('드라이버', '육각 홈붙이 볼트 ISO 4762', 'M2.5 × 18', '무릎 컵 → 허벅지 인서트', 6),
    'FX ISO 4762 M2.5x18 wheel cup': ('드라이버', '육각 홈붙이 볼트 ISO 4762', 'M2.5 × 18', '바퀴 컵 → 정강이 인서트', 6),
    'FX insert M2.5x4 link cup': ('드라이버', '열압입 인서트 황동', 'M2.5 × 4 (구멍 Ø3.6 × 6)',
                                  'PETG 허벅지·정강이의 컵 기둥 자리(v4 SLA는 에폭시 접착, v5 PETG는 열압입)', 12),
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
    'FX DIN 7984 M2.5x6 mjpower': ('몸통', '저두 볼트 DIN 7984', 'M2.5 × 6', 'mjpower-ss → 데크 보스', 4),
    'FX insert M2.5x4 deck': ('몸통', '열압입 인서트 황동', 'M2.5 × 4 (구멍 Ø3.6)', '데크 보스(라즈베리파이 4, mjpower-ss 4)', 8),
    'FX pi3hat M2.5x5': ('몸통', 'M2.5 × 5 나사 (pi3hat 동봉)', 'M2.5 × 5', 'pi3hat → 스탠드오프 위 — 따로 사지 않는다', 4),
    'FX foam pad': ('몸통', '폼 패드', '75 × 310 × 1 mm', '배터리 밑', 1),
}
INCLUDED = ('FX pi3hat M2.5x5',)          # comes with the pi3hat: no order line


def not_modelled(n):
    """Joint parts of the v5 legs that are not modelled, from the model counts of the parts that carry them."""
    hubs = n['thigh_L'] + n['thigh_R'] + n['shank_L'] + n['shank_R']
    caps = n['kneecap_L'] + n['kneecap_R']
    return [
        ('관절', '접시 볼트 ISO 10642 12.9', 'M3 × 16', '⑥ hub 인서트 플랜지(접시, 면과 같게) → PETG 벽 6 → 6061 받침 링 탭, '
         '0.7 N·m + 나사 고정제 (hub 인서트마다 %d)' % FLANGE_BOLTS, FLANGE_BOLTS * hubs),
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
    ('XT30U 플러그 수(M), 16 AWG 납땜형', '전원 케이블 끝', 3, 4, '전원보드 P5 → pi3hat, P3 → 왼 hip, P6 → 오른 hip: 수-암 케이블 3개의 보드 쪽 끝'),
    ('XT30U 플러그 암(F), 16 AWG 납땜형', '전원 케이블 끝', 11, 13, '위 3개의 드라이버 쪽 끝 + hip → 무릎 → 바퀴 암-암 4개(양 끝)'),
    ('16 AWG 실리콘선 빨강', '전원', '약 2.7 m', '5 m', '어림: v2 모델 길이 2.4 m + 링크 25 cm로 다리마다 +0.15 m, 실물에서 재서 자름'),
    ('16 AWG 실리콘선 검정', '전원', '약 2.7 m', '5 m', '같음'),
    ('CAN 꼬은선 26 AWG (2가닥)', 'CAN_H·CAN_L', '약 2.7 m', '5 m', 'pi3hat JC1 → 왼 다리, JC2 → 오른 다리, hip → 무릎 → 바퀴 (케이블 6개)'),
    ('JST PHR-3 하우징', 'PH 2.0 mm 3핀', 12, 15, 'CAN 케이블 6개 × 양 끝, 1번 H · 2번 L · 3번(GND) 비움, 곧은 배선'),
    ('JST SPH-002T-P0.5S 단자', '24~30 AWG 압착', 24, 50, '하우징마다 2개(3번 비움), 압착 실패 여유'),
    ('XT90(F) → XT60(M) 변환 케이블', '12 AWG', 1, 1, '배터리 XT60(F) 리드 → mjpower-ss XT90(M)'),
    ('접착식 케이블타이 마운트', '20 × 20 mm 안팎', '약 16', 20, '링크에 구멍이 없어 다리 배선은 허벅지·정강이 뒷면을 따라 붙인다(git 5a01c18 경로)'),
    ('케이블타이', '100 × 2.5 mm', '약 30', '1봉(100개)', '마운트·다발 묶음'),
    ('수축 튜브', 'Ø3·Ø6 안팎', '-', '1세트', 'XT30 납땜부, 선 끝'),
]
TOOLS = [   # (item, where); {inserts} = heat-set inserts in the BOM
    ('나사 고정제(중강도, 예: 243급)', '⑥ hub 인서트 볼트(관절 단면도의 조립 순서), 금속 탭에 들어가는 다른 볼트는 선택'),
    ('에폭시 또는 순간접착제', 'AKE 부속 자석 → 6061 자석 스페이서 포켓'),
    ('열압입 인서트 인두 팁 M2.5 · M3 · M4', 'PETG 부품 인서트 {inserts}개'),
    ('JST PH 압착 공구', 'CAN 케이블 단자(SPH-002T)'),
    ('리머 Ø35 H7', '허벅지 바깥 막대의 무릎 캡 구멍 2곳(0.2 작게 출력한 뒤)'),
    ('2.5 mm 육각 볼드라이버, 축 120 mm 이상', 'hip·무릎 출력 볼트 ②를 Ø44 접근 구멍 안으로'),
    ('육각 렌치 1.5 · 2 · 2.5 · 3 mm', 'DIN 7984 M2.5(1.5), ISO 4762 M2.5 · 10642 M3 · 7380 M3(2), ISO 4762 M3 · 10642 M4(2.5), '
     'ISO 4762 M4 · 10642 M5(3)'),
    ('스패너 5.5 · 7 mm', 'M3 너트(이음판), M4 나일론 너트(위 레일)'),
    ('토크 드라이버 0.5~1 N·m (있으면)', '⑥ 0.7 N·m'),
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
    for prefix, *_r in MACHINED:
        out[prefix] = sum(left.pop(k) for k in [k for k in left if k.startswith(prefix)])
    for prefix, *_r in PRINTED:
        out[prefix] = {k: left.pop(k) for k in sorted(k for k in left if k.startswith(prefix))}
    for k in FX:
        out[k] = left.pop(k, 0)
    out['61807'] = sum(left.pop(k) for k in [k for k in left if k.startswith('kneebearing_')])
    for k in [k for k in left if k.startswith('kneecap_')]:
        left.pop(k)                                   # counted in V5_MACHINED (knee cap)
    assert not left, 'model parts with no BOM line: %s' % left
    return out


def rows_all(n):
    c = classify(n)
    bad = {k: (c[k], v[4]) for k, v in FX.items() if c[k] != v[4]}
    assert not bad, 'fastener counts differ from the v4 fasteners.specs: %s' % bad
    R = {'buy': [], 'mach': [], 'print': [], 'fx': [], 'wire': WIRING, 'tools': []}
    for prefix, item, spec, src, note in PURCHASED:
        assert c[prefix], prefix
        R['buy'].append((item, spec, c[prefix], 0 if '동봉' in src else c[prefix], src, note, '있음'))
    R['buy'].append(('깊은 홈 볼 베어링 61807', 'Ø35 × Ø47 × 7, 2RS 또는 ZZ', c['61807'], c['61807'], '베어링 판매점',
                     '정강이 머리 패드 바깥 면 자리(Ø47 압입), 안륜은 무릎 캡 관', '있음'))
    R['buy'].append(('microSD 카드', '32 GB 이상(A1/A2)', 1, 1, '전자 부품상', '라즈베리파이 OS', '없음'))
    for prefix, dwg, item, spec, files, note in MACHINED:
        k = c[prefix]
        R['mach'].append((dwg, item, spec, k, k, files, note))
    for dwg, item, spec, files, note, fn in V5_MACHINED:
        k = fn(n)
        R['mach'].append((dwg, item, spec, k, k, files, note))
    for prefix, dwg, item, spec, colour, axis, note in PRINTED:
        for name, k in c[prefix].items():
            vol, area, box, bed = stl_info(name, axis)
            wall, infill = PRINT_PROFILE[next(p for p in PRINT_PROFILE if name.startswith(p))]
            sh = min(vol, area * wall)
            g = (sh + infill * (vol - sh)) * RHO
            R['print'].append((dwg, name, item, spec, colour, k, g * k, vol, box, bed, note))
    for name, (grp, item, spec, where, _e) in FX.items():
        if name not in INCLUDED:
            R['fx'].append((grp, item, spec, c[name], where, '있음'))
    for grp, item, spec, where, k in not_modelled(n):
        R['fx'].append((grp, item, spec, k, where, '없음 ★'))
    ins = sum(r[3] for r in R['fx'] if '인서트' in r[1])
    R['tools'] = [(a, b.format(inserts=ins)) for a, b in TOOLS]
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


def write(R, c, leaves):
    pr = R['print']
    g_tot = sum(r[6] for r in pr)
    colours = defaultdict(float)
    for r in pr:
        colours[r[4]] += r[6]
    big = [r for r in pr if r[9] > 250.0]
    tot = fx_totals(R['fx'])
    n_fx = sum(r[3] for r in R['fx'])
    L = ['# WheelBiped v5 BOM — 주문용', '',
         '`python manufacturing/bom.py`가 만든다(손으로 고치지 말 것). 모델에 있는 부품의 수량은 **조립 STEP**(`export/WheelBiped_v5_assembly.step`, '
         '허브 버전 14, 링크 25 cm, 11.598 kg)의 인스턴스 수이고, 모델 부품이 하나라도 분류되지 않으면 스크립트가 멈춘다. '
         '모델에 없는 것(v5 관절 볼트 ⑥·⑦과 그 인서트, 배선, 공구)은 손으로 넣었고 **★ 또는 "모델 없음"** 으로 표시했다. '
         '출력 질량과 베드 크기는 `export/parts_stl`에서 계산했다. 가격은 넣지 않았다.', '',
         '## 0. 한눈에', '',
         '| 묶음 | 종류 | 개수 | 어디에 |', '|---|---|---|---|',
         '| 1. 구매품(모터·전장·바퀴) | %d | %d | mjbots, CubeMars, 전자 부품상, 베어링 |' % (len(R['buy']), sum(r[2] for r in R['buy'])),
         '| 2. 가공 주문(CNC·레이저) | %d | %d | CNC + 판금 업체 한 곳 (`robot_cad_v2/manufacturing/주문_요청서.md` 양식) |'
         % (len(R['mach']), sum(r[3] for r in R['mach'])),
         '| 3. 3D 출력(PETG) | %d | %d | 직접 출력 또는 출력 대행. 필라멘트 약 %.1f kg(+15 %% 여유 %.1f kg) |'
         % (len(pr), sum(r[5] for r in pr), g_tot / 1000, g_tot * WASTE / 1000),
         '| 4. 체결 부품 | %d | %d (주문 권장 %d) | 나사 쇼핑몰 |' % (len(tot), n_fx, sum(r[3] for r in tot)),
         '| 5. 배선·커넥터(모델 없음) | %d | — | 전자 부품상 |' % len(R['wire']),
         '| 6. 공구·소모품(있으면 생략) | %d | — | |' % len(R['tools']), '',
         '조립 STEP의 부품 인스턴스 %d개(링크 묶음 torso·femur·tibia·wheel 7개를 더하면 README의 "부품 477개"와 같다)를 모두 아래 줄로 나눴다.'
         % leaves, '']
    L += ['## 1. 구매품', '',
          '| 품목 | 규격 | 필요 | 주문 | 구입처 | 비고 |', '|---|---|---|---|---|---|']
    L += ['| %s | %s | %d | %s | %s | %s%s |' % (r[0], r[1], r[2], r[3] if r[3] else '— (동봉)', r[4], r[5],
                                               '' if r[6] == '있음' else ' — 모델 없음') for r in R['buy']]
    L += ['', '## 2. 가공 주문', '',
          '도면은 `manufacturing/drawings/`, 부품 좌표 STEP은 `manufacturing/step/`(v5 새 부품)·`export/parts_step/`(v4·v2 부품). '
          'v2 부품(S14~S18)의 공차·탭·후처리는 `v2_*.pdf`와 `robot_cad_v2/manufacturing/주문_요청서.md`에 다 있다. '
          '탭은 STEP에 기초 구멍으로만 들어 있으니 요청서·도면으로 전한다.', '',
          '| 도번 | 부품 | 재질·공정 | 수량 | 파일 | 비고 |', '|---|---|---|---|---|---|']
    L += ['| %s | %s | %s | %d | %s | %s |' % (r[0], r[1], r[2], r[3], r[5], r[6]) for r in R['mach']]
    L += ['', '**주문하지 않는 것**: `clamp_ring_output_D27_t2.0_SUS304`(v4 출력 클램프 링). v5에서는 6061 hub 인서트가 대신하고 모델에도 없지만 '
          '`export/parts_step`·`parts_stl`에 파일이 남아 있다.', '',
          '## 3. 3D 출력 (PETG)', '',
          '| 도번 | 파일 | 부품 | 설정·놓는 면 | 색 | 수량 | 출력 질량(어림) | 외곽(mm) | 필요한 정사각 베드 | 비고 |',
          '|---|---|---|---|---|---|---|---|---|---|']
    L += ['| %s | `parts_stl/%s.stl` | %s | %s | %s | %d | %.0f g | %s | %.0f mm | %s |'
          % (r[0], r[1], r[2], r[3], r[4], r[5], r[6], ' × '.join('%.0f' % x for x in r[8]), r[9], r[10]) for r in pr]
    L += ['', '- 출력 질량은 v4 `check.print_fraction`의 식(벽 두께만큼 꽉 차고 속은 채움 비율)으로 STL에서 어림했다. 허벅지·정강이는 꽉 찬 PETG라 '
          '모델 값(827.5 / 420.7 g)과 같다.',
          '- 색별 합계: %s. 서포트·브림·실패 여유 15 %%를 더하면 **약 %.1f kg → 1 kg 스풀 %d개**(색을 하나로 하면).'
          % (', '.join('%s %.2f kg' % (k, v / 1000) for k, v in sorted(colours.items(), key=lambda x: -x[1])), g_tot * WASTE / 1000,
             math.ceil(g_tot * WASTE / 1000)),
          '- **베드**: "필요한 정사각 베드"는 대각선으로 돌려 놓는 것까지 따진 가장 작은 정사각 베드 한 변이다(허벅지·정강이는 정해진 평평한 면, '
          '나머지는 눕혀 놓은 면 기준, 브림 여유 없음). **%s는 250 mm 넘게 필요해서 256 × 256 베드(Bambu X1·P1·A1)에도 안 들어간다** — '
          '300 × 300급 프린터나 출력 대행을 쓴다.' % ', '.join(dict.fromkeys(r[2] for r in big if r[1] != 'splice_tabs')),
          '- 인서트는 모두 PETG 열압입이다(v4 SLA 링크의 에폭시 접착 인서트 대신).', '']
    L += ['## 4. 체결 부품·베어링', '',
          '### 4-1. 주문용 합계 (같은 품목·크기를 모음)', '',
          '주문 권장 = 필요 + 여유(나사·와셔·너트 +10 %, 최소 2 / 인서트 +20 %, 최소 4 / 핀 +3). ★는 모델에 없는 v5 관절 부품이 들어간 줄.', '',
          '| 품목 | 규격 | 필요 | 주문 권장 | 쓰는 곳(수량) |', '|---|---|---|---|---|']
    star = {(f[1], f[2].split(' (')[0]) for f in R['fx'] if f[5] != '있음'}
    L += ['| %s%s | %s | %d | %d | %s |' % (r[0], ' ★' if (r[0], r[1].split(' (')[0]) in star else '', r[1], r[2], r[3], r[4]) for r in tot]
    L += ['', '- pi3hat의 M2.5 × 5 나사 4개와 스탠드오프 4개는 pi3hat에 들어 있어 뺐다.',
          '- ⑥ ISO 10642 M3 × 16을 **12.9로 구하기 어려우면 10.9**(보증 하중 약 4.2 kN)도 설계 하중(hip 3.1 kN)은 통과한다. 검토 하중(안전계수 2, 5.2 kN)은 '
          '12.9(4.9 kN)도 넘는다 — README 0절 표, `calc/strength_v5.md`.',
          '- ISO 4762 M3 × 14는 12.9(허브) 12 + 8.8(림) 12다. 같은 길이이므로 12.9로 24개를 사도 된다.', '',
          '### 4-2. 쓰는 곳별 상세', '',
          '| 묶음 | 품목 | 규격 | 수량 | 위치 | 모델 |', '|---|---|---|---|---|---|']
    L += ['| %s | %s | %s | %d | %s | %s |' % (r[0], r[1], r[2], r[3], r[4], r[5]) for r in R['fx']]
    L += ['', '①~⑦은 관절 단면도(`manufacturing/explain/hip_section.png`, `knee_section.png`, `wheel_section.png`)의 번호다.', '',
          '## 5. 배선·커넥터 (모델 없음)', '',
          '사용자 요청(11-01)으로 배선은 모델에 없다. 연결은 v2 README 8절과 같다: 전원보드 P5 → pi3hat, P3·P6 → 왼·오른 hip → 무릎 → 바퀴(XT30 데이지 체인, 16 AWG), '
          'pi3hat JC1·JC2 → 각 다리 hip → 무릎 → 바퀴(CAN, 바퀴 CAN1에 종단 저항). 선 길이는 어림이므로 실물에서 재서 자른다. '
          '한 번 깔아 본 v5 다리 경로는 git `5a01c18`(`fusion/harness_v5.py`, 17.5 cm 링크).', '',
          '| 품목 | 규격 | 필요 | 주문 권장 | 비고 |', '|---|---|---|---|---|']
    L += ['| %s | %s | %s | %s | %s |' % (r[0], r[1], fmt_q(r[2]), fmt_q(r[3]), r[4]) for r in R['wire']]
    L += ['', '커넥터 암·수는 실물 보드를 보고 한 번 더 확인한다(moteus·pi3hat은 XT30 수, 전원보드 출력은 암 기준으로 셈).', '',
          '## 6. 공구·소모품 (이미 있으면 생략)', '', '| 품목 | 쓰는 곳 |', '|---|---|']
    L += ['| %s | %s |' % r for r in R['tools']]
    L += ['', '## 7. 주문 전 확인', '',
          '- **타이어**는 v2 때 이미 주문했다(AliExpress). 받았는지만 확인한다. **배터리**는 가지고 있는 24 V 15 Ah(XT60) 기준이다.',
          '- **hip 판·hub 인서트는 왼·오른쪽이 거울상**이라 서로 다른 부품이다(각 1개씩). 받침 링 4개, 무릎 캡 2개는 같은 부품이다.',
          '- v2 도면을 그대로 쓰는 부품(S14~S18)은 이름이 v2 기준이다: `v2_hip_plate_L.pdf` = 이 로봇의 `hip_plate_R`(CAD +y). 두 장을 다 주문하면 된다.',
          '- 61807은 2RS(고무 실)나 ZZ(금속 실) 어느 쪽이든 폭 7 mm면 맞는다.',
          '- 허벅지·정강이·배터리 받침·전장 데크는 큰 베드가 필요하다(3절).',
          '- 비용: 아노다이징은 hip 판만 도면 지시이고, hub 인서트·무릎 캡은 "권장"이다. 빼면 가공비가 준다(끼움면 마스킹도 필요 없어짐).',
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
        for r in R['mach']:
            w.writerow(('2 가공', r[1], r[2], r[3], r[4], '%s; %s' % (r[0], r[5]), '', r[6], '있음'))
        for r in pr:
            w.writerow(('3 출력', r[1], '%s; %s; 색 %s' % (r[2], r[3], r[4]), r[5], r[5], '%s; export/parts_stl/%s.stl' % (r[0], r[1]),
                        '출력 약 %.0f g, 정사각 베드 %.0f mm' % (r[6], r[9]), r[10], '있음'))
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
    print('leaf instances %d, BOM.md %d lines' % (leaves, md.count('\n')))


if __name__ == '__main__':
    main()
