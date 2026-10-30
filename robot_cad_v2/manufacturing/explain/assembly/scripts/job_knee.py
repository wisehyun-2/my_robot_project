"""Left knee: thigh knee plate + AKE60-8 + shank driven boss + moteus driver cup."""
import math

import axlib as X


def config(M):
    C, P, LY, LG = M['cadlib'], M['params'], M['layout'], M['legs']
    A = P.AKE
    K = (P.KNEE_X, P.KNEE_Z)
    Kp = (K[0], 0.0, K[1])

    def along(a, b, d):                       # half-space keeping the part of a link within d of joint a
        u = X.unit((b[0] - a[0], 0.0, b[1] - a[1]))
        return [((a[0] + u[0] * d, 0.0, a[1] + u[2] * d), u)]
    fr = LY.frames('knee', 1)
    m, b = fr['motor'], fr['board']
    yb = C.col(b, 1)                          # board +y (away from the motor) in robot axes
    ny = (0.0, -1.0, 0.0)
    py = (0.0, 1.0, 0.0)
    nb = tuple(-v for v in yb)

    picks = [('THIGH', 'femur_L:1+thigh_L', None, along(K, (0.0, P.Z_HIP), 88.0)),
             ('MOTOR', 'femur_L:1+AKE60-8_KV80_stator', None, None),
             ('MOTOR', 'femur_L:1+magnet spacer', None, None),
             ('MOTOR', 'tibia_L:1+AKE60-8_KV80_output', None, None),
             ('RING_S', 'femur_L:1+clamp ring stator', None, None),
             ('CUP', 'femur_L:1+cup_knee_L', None, None),
             ('BOARD', 'femur_L:1+moteus_r4.11', None, None),
             ('SHANK', 'tibia_L:1+shank_L', None, along(K, (0.0, P.R_WHEEL), 88.0)),
             ('RING_O', 'tibia_L:1+clamp ring output', None, None)]
    F = []
    ring_top = P.Y_FEMUR_OUT - P.CBORE_M25[1]                       # stator clamp ring on the recess floor
    for i, q in enumerate(LY.motor_points(m, P.AKE_PCD_RING / 2, P.AKE_RING_M25, A['ring'])):
        F.append(X.fast('S25', 'M2.5x8 DIN7984 #%d' % i, X.lowhead(2.5, 8), (q[0], ring_top, q[2]), ny))
    y_out = P.y_output('knee')
    bear = y_out + P.BEARING_FACE + 0.5                             # output ring + washer under the head
    for i, q in enumerate(LY.motor_points(m, P.AKE_PCD_OUT / 2, P.AKE_OUT_M3, A['out'])):
        F.append(X.fast('S316', 'M3x16 12.9 #%d' % i, X.shcs(3, 16), (q[0], bear, q[2]), ny, 'black satin'))
        F.append(X.fast('WASH', 'washer M3 #%d' % i, X.washer(3), (q[0], bear, q[2]), ny))
    for i, q in enumerate(LY.motor_points(m, P.AKE_PCD_OUT / 2, P.AKE_OUT_PIN, A['out'])):
        F.append(X.fast('PINS', 'dowel 3x10 #%d' % i, X.pin(3, 10), (q[0], y_out + 7.0, q[2]), ny))
    rim, f_out = LG.cup_levels('knee', 1)
    L, cb = LG.cup_screw('knee')
    for i, t in enumerate(LG.CUP_COLUMNS):
        xb, zb = 40.5 * math.cos(math.radians(t)), 40.5 * math.sin(math.radians(t))
        q = C.apply(b, (xb, 0.0, zb))
        F.append(X.fast('TINS', 'insert M2.5 thigh #%d' % i, X.insert(2.5, 4.0, 3.6), (q[0], P.Y_FEMUR_IN + 0.1, q[2]), py,
                        'brass'))
        F.append(X.fast('CS', 'M2.5x%d cup #%d' % (L, i), X.shcs(2.5, L), C.apply(b, (xb, f_out - cb, zb)), nb))
    for i, (x, z) in enumerate(P.MOTEUS['holes']):
        F.append(X.fast('CINS', 'insert M2.5 cup #%d' % i, X.insert(2.5, 4.0, 3.6),
                        C.apply(b, (x, P.MOTEUS['pcb_t'] + 0.1, z)), yb, 'brass'))
        F.append(X.fast('BS', 'M2.5x6 DIN7984 board #%d' % i, X.lowhead(2.5, 6), C.apply(b, (x, 0.0, z)), yb))

    home = dict(THIGH=(0, 0, 0, 0), TINS=(0, 0, 0, 30), MOTOR=(0, -90, 0, 0), RING_S=(0, 40, 0, 0), S25=(0, 0, 0, 60),
                PINS=(0, 0, 0, 90), SHANK=(0, 150, 0, 0), RING_O=(0, 215, 0, 0), WASH=(0, 0, 0, 235),
                S316=(0, 0, 0, 260), CUP=(0, -200, 0, 0), CINS=(0, -200, 0, 30), BOARD=(0, -150, 0, 0),
                BS=(0, -150, 0, 25), CS=(0, -200, 0, 40))
    steps = [
        dict(key='s1', title='1단계  인서트 M2.5 × 3 → 허벅지 (열압입)',
             text='인두로 황동 인서트 3개를 허벅지 안쪽 면(드라이버 컵 기둥 자리)에 녹여 넣는다.',
             moves=[[('TINS', 0)]], view=(0.8, -0.8, 0.45)),
        dict(key='s2', title='2단계  모터 → 허벅지 무릎 포켓',
             text='AKE60-8을 허벅지 안쪽에서 하우징 안으로 넣어 Ø42 파일럿을 Ø42.05 포켓(깊이 2.5)에 앉힌다.\n'
                  '출력부는 허벅지 판의 Ø30 구멍 쪽(바깥, 정강이 쪽)을 향한다.',
             moves=[[('MOTOR', 0)]], view=(0.9, -0.75, 0.45)),
        dict(key='s3', title='3단계  SUS 링 + ① M2.5 × 8 × 6 → 모터 고정자',
             text='허벅지 바깥 면의 홈에 SUS 클램프 링(Ø45 / Ø30, 2 t)을 넣고,\n① M2.5 × 8 저두 볼트 6개를 판을 지나 고정자 링 탭에 조인다 (물림 3.2 mm).',
             moves=[[('RING_S', 0)], [('S25', 0)]], spin=['S25'], view=(0.9, 0.7, 0.45)),
        dict(key='s4', title='4단계  ③ 평행핀 + 정강이 → 무릎 출력부',
             text='③ 평행핀 3개를 출력부 구멍(깊이 3)에 꽂고, 정강이의 Ø28 보스를 허벅지 Ø30 구멍으로 넣어\nØ11.8 스피곳을 출력부 Ø12 홈에 맞춘다.',
             moves=[[('PINS', 0)], [('SHANK', 0)]], view=(0.9, 0.75, 0.42)),
        dict(key='s5', title='5단계  SUS 출력 링 + 와셔 + ② M3 × 16 × 6',
             text='정강이 바깥의 Ø28 구멍 안 바닥에 SUS 출력 링(Ø27, 2 t)과 평와셔를 놓고,\n② M3 × 16 (12.9) 6개를 출력부 탭에 조인다 (물림 4.3 mm).',
             moves=[[('RING_O', 0)], [('WASH', 0)], [('S316', 0)]], spin=['S316'], view=(0.8, 0.85, 0.45)),
        dict(key='s6', title='6단계  moteus → 드라이버 컵 (작업대)',
             text='컵 스탠드오프 4곳에 인서트 M2.5를 넣고, moteus r4.11을 얹어\n④ M2.5 × 6 저두 4개로 모터 쪽 면에서 조인다.',
             moves=[[('CINS', 'CUP')], [('BOARD', 'CUP')], [('BS', 'CUP')]], spin=['BS'],
             bench=['CUP', 'CINS', 'BOARD', 'BS'], view=(0.55, 0.85, 0.45)),
        dict(key='s7', title='7단계  드라이버 컵 → 허벅지 하우징 + ⑤ M2.5 × 18 × 3',
             text='컵 립을 허벅지 하우징(Ø88)에 씌워 엔코더를 모터 축에 맞추고,\n⑤ M2.5 × 18 3개를 컵 바닥에서 허벅지 인서트에 조인다.',
             moves=[[('CUP', 0), ('CINS', 0), ('BOARD', 0), ('BS', 0)], [('CS', 0)]], spin=['CS'],
             view=(0.9, -0.7, 0.45)),
    ]
    c = K
    cuts = [dict(key='cut1', remove=X.sector(c, 10.0, 100.0), view=(0.46, 0.65, 0.66), target=(K[0], 196.0, K[1]),
                 zoom=0.7, title='단면 ①  ② M3 × 16 (10°) · ① M2.5 × 8 (100°) 자리를 지나는 단면',
                 feats={'② M3 × 16 + 와셔': X.on_sector_face(c, 10, 10.0, 211.0),
                        'SUS 출력 링': X.on_sector_face(c, 10, 13.0, 208.2),
                        '① M2.5 × 8 + SUS 링': X.on_sector_face(c, 100, 18.25, 201.3),
                        'A Ø11.8 스피곳': X.on_sector_face(c, 10, 5.5, 197.3),
                        'C Ø42 파일럿 ↔ 포켓': X.on_sector_face(c, 10, 21.0, 196.0),
                        '정강이 (PETG)': X.on_sector_face(c, 10, 22.0, 220.0),
                        '허벅지 (PETG)': X.on_sector_face(c, 10, 30.0, 192.0),
                        'AKE60-8': X.on_sector_face(c, 10, 26.0, 182.0),
                        'moteus + 컵': X.on_sector_face(c, 10, 12.0, 160.0)}),
            dict(key='cut2', remove=X.sector(c, 40.0, 160.0), view=(-0.14, 0.62, 0.8), target=(K[0], 196.0, K[1]),
                 zoom=0.7, title='단면 ②  ③ 평행핀 · ① M2.5 × 8 (40°, 160°) 자리를 지나는 단면',
                 feats={'③ 평행핀 Ø3 × 10': X.on_sector_face(c, 40, 10.0, 200.0),
                        '③ 평행핀 (160°)': X.on_sector_face(c, 160, 10.0, 200.0),
                        '① M2.5 × 8 (40°)': X.on_sector_face(c, 40, 18.25, 199.0),
                        '① M2.5 × 8 (160°)': X.on_sector_face(c, 160, 18.25, 199.0),
                        '정강이 (PETG)': X.on_sector_face(c, 40, 24.0, 222.0),
                        '허벅지 (PETG)': X.on_sector_face(c, 160, 30.0, 190.0)})]
    return dict(
        job='knee', title='무릎 관절 조립', subtitle='왼쪽 다리 · Fusion 모델',
        picks=picks, fasteners=F, fixed=['THIGH'],
        recolor={'THIGH': ('ax thigh blue', (110, 150, 205)), 'SHANK': ('ax petg green', (120, 185, 120)),
                 'CUP': ('ax cup grey', (150, 150, 158))},
        home=home, steps=steps, cuts=cuts, target=Kp, view0=(1.0, 0.3, 0.4), view_done=(0.85, 0.6, 0.45),
        move=36, long_move=56,
        names=dict(THIGH='허벅지 (PETG, 무릎 쪽만)', MOTOR='AKE60-8 모터 (무릎)', RING_S='SUS304 클램프 링 Ø45',
                   S25='① M2.5 × 8 저두 × 6', TINS='황동 인서트 M2.5 × 3 (허벅지)', SHANK='정강이 (PETG, 무릎 쪽만)',
                   PINS='③ 평행핀 Ø3 × 10 × 3', RING_O='SUS304 출력 링 Ø27', WASH='평와셔 M3 × 6',
                   S316='② M3 × 16 (12.9) × 6', CUP='드라이버 컵 (PETG)', CINS='황동 인서트 M2.5 × 4 (컵)',
                   BOARD='moteus r4.11', BS='④ M2.5 × 6 저두 × 4', CS='⑤ M2.5 × 18 × 3'),
        captions=dict(TINS='황동 인서트 M2.5 × 3 → 허벅지', MOTOR='AKE60-8 → 허벅지 포켓 (Ø42 파일럿)',
                      RING_S='SUS304 클램프 링 → 허벅지 바깥 홈', S25='① M2.5 × 8 저두 × 6 조임',
                      PINS='③ 평행핀 Ø3 × 10 × 3 → 출력부', SHANK='정강이 → 출력부 (Ø11.8 스피곳)',
                      RING_O='SUS304 출력 링 → 정강이 Ø28 구멍 바닥', WASH='평와셔 M3 × 6', S316='② M3 × 16 (12.9) × 6 조임',
                      CINS='황동 인서트 M2.5 × 4 → 컵 스탠드오프', BOARD='moteus r4.11 → 컵', BS='④ M2.5 × 6 저두 × 4 조임',
                      CUP='드라이버 컵 → 허벅지 하우징 (Ø88 립)', CS='⑤ M2.5 × 18 × 3 조임'),
        short=['인서트 → 허벅지', '모터 → 무릎 포켓', 'SUS 링 + ① M2.5 × 8', '③ 핀 + 정강이', '출력 링 + ② M3 × 16',
               'moteus → 컵', '컵 → 허벅지 (⑤)'],
        legend=['① M2.5 × 8 저두 × 6', '② M3 × 16 (12.9) × 6 + 와셔', '③ 평행핀 Ø3 × 10 × 3', '④ M2.5 × 6 저두 × 4',
                '⑤ M2.5 × 18 × 3', '인서트 M2.5 × 7, SUS 링 × 2'],
        major=['THIGH', 'MOTOR', 'SHANK', 'CUP'], done_labels=['THIGH', 'SHANK', 'CUP', 'MOTOR'],
        overview=('무릎 관절 분해도  (왼쪽 다리)',
                  '모든 부품이 떨어진 상태 — 오른쪽이 바깥(정강이 쪽), 왼쪽이 로봇 중심 쪽(드라이버 컵).  모두 무릎 축 방향으로 끼워진다.\n'
                  '체결 부품: ① M2.5 × 8 × 6  ② M3 × 16 × 6 + 와셔 + SUS 출력 링  ③ 평행핀 × 3  ④ M2.5 × 6 × 4  ⑤ M2.5 × 18 × 3'),
        done=('조립 완료 — 무릎 관절',
              '정강이 → (② M3 × 16 + ③ 핀 + Ø11.8 스피곳) → AKE 출력부 → 모터 베어링 → 고정자 → (① + Ø42 파일럿) → 허벅지.\n'
              '드라이버 컵은 하우징 Ø88 립으로 중심을 맞추고 ⑤ 3개로 고정 (하중 없음).'),
        note='색은 부품 구분용 (실제 허벅지·정강이·컵은 검정 PETG)  ·  허벅지·정강이는 무릎 쪽만  ·  배선·커넥터는 생략  ·  '
             '나사·와셔·핀·인서트는 BOM 규격으로 모델링')
