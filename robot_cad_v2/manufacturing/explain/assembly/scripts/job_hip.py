"""Left hip: 6061 hip plate + AKE60-8 + moteus driver cup + thigh driven boss."""
import math

import axlib as X


def config(M):
    C, P, LY, LG = M['cadlib'], M['params'], M['layout'], M['legs']
    A = P.AKE
    H = (0.0, P.Z_HIP)
    Hp = (0.0, 0.0, P.Z_HIP)
    u = X.unit((P.KNEE_X, 0.0, P.KNEE_Z - P.Z_HIP))
    thigh_trim = [((u[0] * 95.0, 0.0, P.Z_HIP + u[2] * 95.0), u)]
    fr = LY.frames('hip', 1)
    m, b = fr['motor'], fr['board']
    yb = C.col(b, 1)
    nb = tuple(-v for v in yb)
    ny = (0.0, -1.0, 0.0)

    picks = [('PLATE', 'torso:1+hip_plate_L', None, None),
             ('MOTOR', 'torso:1+AKE60-8_KV80_stator', 1, None),
             ('MOTOR', 'torso:1+magnet spacer', 1, None),
             ('MOTOR', 'femur_L:1+AKE60-8_KV80_output', None, None),
             ('CUP', 'torso:1+cup_hip_L', None, None),
             ('BOARD', 'torso:1+moteus_r4.11', 1, None),
             ('THIGH', 'femur_L:1+thigh_L', None, thigh_trim),
             ('RING_O', 'femur_L:1+clamp ring output', None, None)]
    F = []
    bear_s = P.YW - P.CBORE_M25_HIP[1]                              # M2.5 counterbore floor on the outer face
    for i, q in enumerate(LY.motor_points(m, P.AKE_PCD_RING / 2, P.AKE_RING_M25, A['ring'])):
        F.append(X.fast('S25', 'M2.5x6 12.9 #%d' % i, X.shcs(2.5, 6), (q[0], bear_s, q[2]), ny, 'black satin'))
    y_out = P.y_output('hip')
    bear = y_out + P.BEARING_FACE + 0.5
    for i, q in enumerate(LY.motor_points(m, P.AKE_PCD_OUT / 2, P.AKE_OUT_M3, A['out'])):
        F.append(X.fast('S316', 'M3x16 12.9 #%d' % i, X.shcs(3, 16), (q[0], bear, q[2]), ny, 'black satin'))
        F.append(X.fast('WASH', 'washer M3 #%d' % i, X.washer(3), (q[0], bear, q[2]), ny))
    for i, q in enumerate(LY.motor_points(m, P.AKE_PCD_OUT / 2, P.AKE_OUT_PIN, A['out'])):
        F.append(X.fast('PINS', 'dowel 3x10 #%d' % i, X.pin(3, 10), (q[0], y_out + 7.0, q[2]), ny))
    rim, f_out = LG.cup_levels('hip', 1)
    L, cb = LG.cup_screw('hip')
    for i, t in enumerate(LG.CUP_COLUMNS):
        xb, zb = 40.5 * math.cos(math.radians(t)), 40.5 * math.sin(math.radians(t))
        F.append(X.fast('CS', 'M3x%d cup #%d' % (L, i), X.shcs(3, L, thread=18.0), C.apply(b, (xb, f_out - cb, zb)), nb))
    for i, (x, z) in enumerate(P.MOTEUS['holes']):
        F.append(X.fast('CINS', 'insert M2.5 cup #%d' % i, X.insert(2.5, 4.0, 3.6),
                        C.apply(b, (x, P.MOTEUS['pcb_t'] + 0.1, z)), yb, 'brass'))
        F.append(X.fast('BS', 'M2.5x6 DIN7984 board #%d' % i, X.lowhead(2.5, 6), C.apply(b, (x, 0.0, z)), yb))

    home = dict(PLATE=(0, 0, 0, 0), MOTOR=(0, -90, 0, 0), S25=(0, 0, 0, 45), CUP=(0, -210, 0, 0),
                CINS=(0, -210, 0, 60), BOARD=(0, -130, 0, 0), BS=(0, -130, 0, 25), CS=(0, -210, 0, 80),
                PINS=(0, 0, 0, 60), THIGH=(0, 150, 0, 0), RING_O=(0, 260, 0, 0), WASH=(0, 0, 0, 280), S316=(0, 0, 0, 300))
    steps = [
        dict(key='s1', title='1단계  모터 → hip 판 포켓',
             text='AKE60-8을 몸통 안쪽에서 hip 판(6061)의 Ø42 H7 포켓에 넣는다.\n고정자 앞면은 보스 면의 Ø63 × 0.5 홈 덕분에 닿지 않고, 파일럿 끝면만 포켓 바닥에 앉는다.',
             moves=[[('MOTOR', 0)]], view=(0.9, -0.75, 0.45)),
        dict(key='s2', title='2단계  ① M2.5 × 6 (12.9) × 6 → 모터 고정자',
             text='hip 판 바깥 면의 카운터보어(Ø4.8 × 2.7)에 ① M2.5 × 6 (12.9) 6개를 넣어\n고정자 링 탭에 조인다 (물림 3.2 mm, 금속 판이라 클램프 링 없음).',
             moves=[[('S25', 0)]], spin=['S25'], view=(0.9, 0.7, 0.45)),
        dict(key='s3', title='3단계  moteus → 드라이버 컵 (작업대)',
             text='컵 스탠드오프 4곳에 인서트 M2.5를 넣고, moteus r4.11을 얹어\n④ M2.5 × 6 저두 4개로 모터 쪽 면에서 조인다.',
             moves=[[('CINS', 'CUP')], [('BOARD', 'CUP')], [('BS', 'CUP')]], spin=['BS'],
             bench=['CUP', 'CINS', 'BOARD', 'BS'], view=(0.55, 0.85, 0.45)),
        dict(key='s4', title='4단계  드라이버 컵 → hip 판 안쪽 + ⑤ M3 × 40 × 3',
             text='컵 테두리를 hip 판 안쪽의 Ø80 등록 보스에 씌워 엔코더를 모터 축에 맞추고,\n⑤ M3 × 40 3개를 컵 바닥에서 hip 판 M3 탭에 조인다 (물림 4 mm).',
             moves=[[('CUP', 0), ('CINS', 0), ('BOARD', 0), ('BS', 0)], [('CS', 0)]], spin=['CS'],
             view=(0.9, -0.7, 0.45)),
        dict(key='s5', title='5단계  ③ 평행핀 + 허벅지 → hip 출력부',
             text='③ 평행핀 3개를 출력부 구멍에 꽂고, 허벅지의 Ø28 보스를 hip 판 Ø30 구멍으로 넣어\nØ11.8 스피곳을 출력부 Ø12 홈에 맞춘다 (실제로는 몸통·셸을 조립한 뒤 다리를 단다).',
             moves=[[('PINS', 0)], [('THIGH', 0)]], view=(0.9, 0.75, 0.42)),
        dict(key='s6', title='6단계  SUS 출력 링 + 와셔 + ② M3 × 16 × 6',
             text='허벅지 바깥의 Ø28 구멍 안 바닥에 SUS 출력 링(Ø27, 2 t)과 평와셔를 놓고,\n② M3 × 16 (12.9) 6개를 출력부 탭에 조인다 (물림 4.3 mm).',
             moves=[[('RING_O', 0)], [('WASH', 0)], [('S316', 0)]], spin=['S316'], view=(0.8, 0.85, 0.45)),
    ]
    c = H
    cuts = [dict(key='cut1', remove=X.sector(c, -5.0, 45.0), view=(0.75, 0.45, 0.45), target=(0.0, 160.0, P.Z_HIP),
                 zoom=0.7, title='단면 ①  ② M3 × 16 (-5°) · ⑤ M3 × 40 (45°) 자리를 지나는 단면',
                 feats={'② M3 × 16 + 와셔': X.on_sector_face(c, -5, 10.0, 180.0),
                        'SUS 출력 링': X.on_sector_face(c, -5, 13.0, 177.2),
                        '⑤ M3 × 40 (컵 → hip 판)': X.on_sector_face(c, 45, 40.5, 148.0),
                        'A Ø11.8 스피곳': X.on_sector_face(c, -5, 5.5, 166.3),
                        'C Ø42 파일럿 ↔ H7 포켓': X.on_sector_face(c, -5, 21.0, 165.2),
                        'hip 판 (6061)': X.on_sector_face(c, -5, 50.0, 167.0),
                        '허벅지 (PETG)': X.on_sector_face(c, -5, 22.0, 190.0),
                        'AKE60-8': X.on_sector_face(c, -5, 26.0, 152.0),
                        'moteus': X.on_sector_face(c, -5, 10.0, 133.0),
                        '드라이버 컵': X.on_sector_face(c, 45, 42.0, 130.0)}),
            dict(key='cut2', remove=X.sector(c, 25.0, 145.0), view=(0.07, 0.55, 0.85), target=(0.0, 165.0, P.Z_HIP),
                 zoom=0.7, title='단면 ②  ③ 평행핀 · ① M2.5 × 6 (25°, 145°) 자리를 지나는 단면',
                 feats={'③ 평행핀 Ø3 × 10': X.on_sector_face(c, 25, 10.0, 169.0),
                        '③ 평행핀 (145°)': X.on_sector_face(c, 145, 10.0, 169.0),
                        '① M2.5 × 6 (25°)': X.on_sector_face(c, 25, 18.25, 168.0),
                        '① M2.5 × 6 (145°)': X.on_sector_face(c, 145, 18.25, 168.0),
                        'hip 판 (6061)': X.on_sector_face(c, 145, 50.0, 167.0),
                        '허벅지 (PETG)': X.on_sector_face(c, 25, 24.0, 190.0)})]
    return dict(
        job='hip', title='hip 관절 조립', subtitle='왼쪽 · Fusion 모델',
        picks=picks, fasteners=F, fixed=['PLATE'],
        recolor={'THIGH': ('ax thigh blue', (110, 150, 205)), 'CUP': ('ax cup grey', (150, 150, 158))},
        home=home, steps=steps, cuts=cuts, target=Hp, view0=(1.0, 0.3, 0.4), view_done=(0.85, 0.6, 0.45),
        move=36, long_move=56,
        names=dict(PLATE='hip 판 (6061, 검정 아노다이징)', MOTOR='AKE60-8 모터 (hip)', S25='① M2.5 × 6 (12.9) × 6',
                   CUP='드라이버 컵 (PETG)', CINS='황동 인서트 M2.5 × 4 (컵)', BOARD='moteus r4.11',
                   BS='④ M2.5 × 6 저두 × 4', CS='⑤ M3 × 40 × 3', PINS='③ 평행핀 Ø3 × 10 × 3',
                   THIGH='허벅지 (PETG, hip 쪽만)', RING_O='SUS304 출력 링 Ø27', WASH='평와셔 M3 × 6',
                   S316='② M3 × 16 (12.9) × 6'),
        captions=dict(MOTOR='AKE60-8 → hip 판 포켓 (Ø42 H7)', S25='① M2.5 × 6 (12.9) × 6 조임',
                      CINS='황동 인서트 M2.5 × 4 → 컵 스탠드오프', BOARD='moteus r4.11 → 컵', BS='④ M2.5 × 6 저두 × 4 조임',
                      CUP='드라이버 컵 → hip 판 Ø80 보스', CS='⑤ M3 × 40 × 3 조임', PINS='③ 평행핀 Ø3 × 10 × 3 → 출력부',
                      THIGH='허벅지 → 출력부 (Ø11.8 스피곳)', RING_O='SUS304 출력 링 → 허벅지 Ø28 구멍 바닥',
                      WASH='평와셔 M3 × 6', S316='② M3 × 16 (12.9) × 6 조임'),
        short=['모터 → hip 판 포켓', '① M2.5 × 6 (12.9)', 'moteus → 컵', '컵 → hip 판 (⑤)', '③ 핀 + 허벅지',
               '출력 링 + ② M3 × 16'],
        legend=['① M2.5 × 6 (12.9) × 6', '② M3 × 16 (12.9) × 6 + 와셔', '③ 평행핀 Ø3 × 10 × 3', '④ M2.5 × 6 저두 × 4',
                '⑤ M3 × 40 × 3', '인서트 M2.5 × 4, SUS 출력 링 × 1'],
        major=['PLATE', 'MOTOR', 'THIGH', 'CUP'], done_labels=['PLATE', 'THIGH', 'CUP', 'MOTOR'],
        overview=('hip 관절 분해도  (왼쪽)',
                  '모든 부품이 떨어진 상태 — 오른쪽이 바깥(허벅지 쪽), 왼쪽이 몸통 안쪽(드라이버 컵).  모두 hip 축 방향으로 끼워진다.\n'
                  '체결 부품: ① M2.5 × 6 × 6  ② M3 × 16 × 6 + 와셔 + SUS 출력 링  ③ 평행핀 × 3  ④ M2.5 × 6 × 4  ⑤ M3 × 40 × 3'),
        done=('조립 완료 — hip 관절',
              '허벅지 → (② M3 × 16 + ③ 핀 + Ø11.8 스피곳) → AKE 출력부 → 모터 베어링 → 고정자 → (① + Ø42 H7 파일럿) → hip 판 → 몸통.\n'
              '드라이버 컵은 hip 판 Ø80 보스로 중심을 맞추고 ⑤ 3개로 고정.'),
        note='색은 부품 구분용 (실제 허벅지·컵은 검정 PETG)  ·  허벅지는 hip 쪽만, hip 판 외 몸통은 생략  ·  배선·커넥터는 생략  ·  '
             '나사·와셔·핀·인서트는 BOM 규격으로 모델링')
