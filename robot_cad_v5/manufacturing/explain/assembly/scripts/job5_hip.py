"""Right hip (_R, CAD +y): 6061 hip plate + AKE60-8 + moteus driver cup, PETG thigh with the 6061 hub insert,
the 6061 backing ring and the 6 x M3 x 16 countersunk flange bolts (11-02)."""
import axlib5 as X


def config(M):
    P, LV = M['params'], M['legs_v5']
    H = (0.0, P.Z_HIP)
    K = (P.KNEE_X, P.KNEE_Z)
    py, ny = (0.0, 1.0, 0.0), (0.0, -1.0, 0.0)
    y_out = P.y_output('hip')

    picks = [X.pick('PLATE', 'torso:1+hip_plate_R'),
             X.pick('MOTOR', 'torso:1+AKE60-8_KV80_stator', side=1),
             X.pick('MOTOR', 'torso:1+magnet spacer', side=1),
             X.pick('MOTOR', 'femur_R:1+AKE60-8_KV80_output'),
             X.pick('S25', 'torso:1+FX ISO 4762 M2.5x6 12.9 hip', side=1, each=ny),
             X.pick('CUP', 'torso:1+cup_hip_R'),
             X.pick('BOARD', 'torso:1+moteus_r4.11', side=1),
             X.pick('CINS', 'torso:1+FX insert M2.5x4 cup', side=1, each=ny),
             X.pick('BS', 'torso:1+FX DIN 7984 M2.5x5 board', side=1, each=ny),
             X.pick('CS', 'torso:1+FX ISO 4762 M3x40 hip cup', side=1, each=py),
             X.pick('THIGH', 'femur_R:1+thigh_R:', body='thigh_R main', trims=X.along(H, K, 105.0)),
             X.pick('INSERT', 'femur_R:1+thigh_R:', body='thigh_R insert'),
             X.pick('RING', 'femur_R:1+thigh_R:', body='thigh_R ring'),
             X.pick('PINS', 'femur_R:1+FX ISO 8734 3x10', each=py),
             X.pick('WASH', 'femur_R:1+FX DIN 125 M3', each=ny),
             X.pick('S316', 'femur_R:1+FX ISO 4762 M3x16 12.9', each=ny)]
    F = []
    for k in range(6):                         # 6 at r 18, 30 + 60 k deg: insert flange (flush) -> PETG wall -> ring tap
        q = X.polar(H, LV.FLANGE_BOLT_R, 30 + 60 * k)
        F.append(X.fast('F6', 'ISO 10642 M3x16 12.9 hip #%d' % k, X.csk(3, 16), (q[0], LV.Y_TF, q[1]), py, 'black satin'))

    home = dict(PLATE=(0, 0, 0, 0), MOTOR=(0, -90, 0, 0), S25=(0, 0, 0, 35), CUP=(0, -210, 0, 0),
                CINS=(0, -210, 0, 60), BOARD=(0, -130, 0, 0), BS=(0, -130, 0, 25), CS=(0, -210, 0, 80),
                INSERT=(0, 65, 0, 0), PINS=(0, 65, 0, 15), F6=(0, 65, 0, 28), THIGH=(0, 130, 0, 0),
                RING=(0, 250, 0, 0), WASH=(0, 0, 0, 280), S316=(0, 0, 0, 300))
    thigh_set = ['THIGH', 'RING', 'INSERT', 'F6', 'PINS']
    steps = [
        dict(key='s1', title='1단계  모터 → hip 판 포켓',
             text='AKE60-8을 몸통 안쪽에서 hip 판(6061)의 Ø42 H7 포켓에 넣는다.\n'
                  '고정자 앞면은 보스 면의 Ø63 × 0.5 홈 덕분에 닿지 않고, 파일럿 끝면만 포켓 바닥에 앉는다.',
             moves=[[('MOTOR', 0)]], view=(0.9, -0.75, 0.45)),
        dict(key='s2', title='2단계  ① M2.5 × 6 (12.9) × 6 → 모터 고정자',
             text='hip 판 바깥 면의 카운터보어(Ø4.8 × 2.7)에 ① M2.5 × 6 (12.9) 6개를 넣어\n'
                  '고정자 링 탭에 조인다 (물림 3.2 mm, 금속 판이라 클램프 링 없음).',
             moves=[[('S25', 0)]], spin=['S25'], view=(0.9, 0.7, 0.45)),
        dict(key='s3', title='3단계  moteus → 드라이버 컵 (작업대)',
             text='컵 스탠드오프 4곳에 인서트 M2.5를 넣고, moteus r4.11을 얹어\n④ M2.5 × 5 저두 4개로 모터 쪽 면에서 조인다.',
             moves=[[('CINS', 'CUP')], [('BOARD', 'CUP')], [('BS', 'CUP')]], spin=['BS'],
             bench=['CUP', 'CINS', 'BOARD', 'BS'], view=(0.55, 0.85, 0.45)),
        dict(key='s4', title='4단계  드라이버 컵 → hip 판 + ⑤ M3 × 40 × 3',
             text='컵 테두리를 hip 판 안쪽의 Ø80 등록 보스에 씌워 엔코더를 모터 축에 맞추고,\n'
                  '⑤ M3 × 40 3개를 컵 바닥에서 hip 판 M3 탭에 조인다 (물림 4 mm).',
             moves=[[('CUP', 0), ('CINS', 0), ('BOARD', 0), ('BS', 0)], [('CS', 0)]], spin=['CS'],
             view=(0.9, -0.7, 0.45)),
        dict(key='s5', title='5단계  받침 링 → 허벅지 Ø44 구멍 (작업대)',
             text='허벅지 바깥 면의 Ø44 접근 구멍으로 6061 받침 링(Ø43 / Ø29 × 6, M3 탭 6개)을 떨어뜨려\n'
                  'PETG 벽(6 mm) 위에 앉히고, Ø2.5 막대로 링의 탭 6개를 벽 구멍에 맞춘다.',
             moves=[[('RING', 'THIGH')]], bench=['THIGH', 'RING'], ghost=['THIGH'], view=(0.55, 0.8, 0.5)),
        dict(key='s6', title='6단계  hub 인서트 + ⑥ M3 × 16 접시 × 6 + ③ 핀 (작업대)',
             text='6061 hub 인서트를 허벅지 안쪽 면 자리(Ø48.3 × 6)에 넣고, ⑥ M3 × 16 접시 (12.9) 6개를\n'
                  'PETG 벽을 지나 받침 링 탭에 조인다 (약 0.7 N·m). ③ 평행핀 3개를 인서트 구멍 바닥까지 압입한다 (2.5 mm 나옴).',
             moves=[[('INSERT', 'THIGH')], [('F6', 'THIGH')], [('PINS', 'THIGH')]], spin=['F6'],
             bench=thigh_set, view=(0.6, -0.75, 0.45)),
        dict(key='s7', title='7단계  허벅지 → hip 출력부',
             text='인서트의 Ø28 보스를 hip 판 Ø30 구멍으로 넣어 Ø11.8 스피곳을 출력부 Ø12 홈에, ③ 핀을 출력부 구멍에\n'
                  '맞춘다 (실제로는 몸통·셸을 조립한 뒤 다리를 단다).',
             moves=[[(g, 0) for g in thigh_set]], show=['THIGH'], view=(0.9, 0.75, 0.42)),
        dict(key='s8', title='8단계  와셔 + ② M3 × 16 (12.9) × 6 → 출력부',
             text='허벅지 바깥 면의 Ø44 구멍으로 평와셔(먼저)와 ② M3 × 16 6개를 넣어 출력부 탭에 조인다\n'
                  '(물림 4.3 mm, 바깥 면 → 머리 약 107 mm: 2.5 mm 육각 볼드라이버 120 mm 이상, 자석 비트).',
             pre=dict(WASH=(0, 0, 0, 125), S316=(0, 0, 0, 140)),
             moves=[[('WASH', 0)], [('S316', 0)]], spin=['S316'], view=(0.45, 0.85, 0.3), zoom=0.8),
    ]
    c = H
    cuts = [dict(key='cut1', remove=X.sector(c, -5.0, 90.0), view=(0.62, 0.45, 0.6), target=(0.0, 175.0, P.Z_HIP),
                 zoom=0.48, title='단면 ①  ② M3 × 16 (-5°) · ⑥ 플랜지 볼트 (90°) 자리를 지나는 단면',
                 feats={'② M3 × 16 + 와셔': X.on_sector_face(c, -5, 10.0, 180.0),
                        'hub 인서트 (6061)': X.on_sector_face(c, -5, 21.0, 181.0),
                        'PETG 벽 6': X.on_sector_face(c, -5, 17.0, 187.0),
                        '6061 받침 링': X.on_sector_face(c, -5, 18.0, 193.0),
                        '⑥ M3 × 16 접시 → 받침 링': X.on_sector_face(c, 90, 18.0, 186.0),
                        'Ø44 접근 구멍 (바깥 면까지)': X.on_sector_face(c, -5, 23.5, 240.0),
                        'A Ø11.8 스피곳': X.on_sector_face(c, -5, 5.0, y_out - 0.7),
                        'hip 판 (6061)': X.on_sector_face(c, -5, 50.0, 167.0),
                        'AKE60-8': X.on_sector_face(c, -5, 26.0, 152.0),
                        'moteus': X.on_sector_face(c, -5, 10.0, 133.0)}),
            dict(key='cut2', remove=X.sector(c, 25.0, 150.0), view=(0.3, 0.55, 0.78), target=(0.0, 175.0, P.Z_HIP),
                 zoom=0.48, title='단면 ②  ③ 평행핀 · ① M2.5 × 6 (25°), ⑥ 플랜지 볼트 (150°) 자리를 지나는 단면',
                 feats={'③ 평행핀 Ø3 × 10': X.on_sector_face(c, 25, 10.0, 169.0),
                        '① M2.5 × 6 (12.9)': X.on_sector_face(c, 25, 18.25, 168.0),
                        '⑥ M3 × 16 접시': X.on_sector_face(c, 150, 18.0, 186.0),
                        '6061 받침 링': X.on_sector_face(c, 150, 20.6, 193.0),
                        'hub 인서트 플랜지': X.on_sector_face(c, 150, 22.5, 181.0),
                        'hip 판 (6061)': X.on_sector_face(c, 150, 50.0, 167.0),
                        '허벅지 (PETG)': X.on_sector_face(c, 25, 24.0, 250.0)})]
    return dict(
        job='hip', title='hip 관절 조립 (v5)', subtitle='오른쪽 다리 (_R) · Fusion 모델 버전 16',
        picks=picks, fasteners=F, fixed=['PLATE'],
        recolor={'THIGH': ('ax thigh blue', (110, 150, 205)), 'CUP': ('ax cup grey', (150, 150, 158))},
        home=home, steps=steps, cuts=cuts, target=(0.0, 0.0, P.Z_HIP), view0=(1.0, 0.3, 0.4),
        view_done=(0.85, 0.6, 0.45), move=36, long_move=56,
        names=dict(PLATE='hip 판 (6061)', MOTOR='AKE60-8 모터 (hip)', S25='① M2.5 × 6 (12.9) × 6',
                   CUP='드라이버 컵 (PETG)', CINS='황동 인서트 M2.5 × 4 (컵)', BOARD='moteus r4.11',
                   BS='④ M2.5 × 5 저두 × 4', CS='⑤ M3 × 40 × 3', THIGH='허벅지 (PETG, hip 쪽만)',
                   INSERT='hub 인서트 (6061)', RING='받침 링 (6061)', F6='⑥ M3 × 16 접시 (12.9) × 6',
                   PINS='③ 평행핀 Ø3 × 10 × 3', WASH='평와셔 M3 × 6', S316='② M3 × 16 (12.9) × 6'),
        captions=dict(MOTOR='AKE60-8 → hip 판 포켓 (Ø42 H7)', S25='① M2.5 × 6 (12.9) × 6 조임',
                      CINS='황동 인서트 M2.5 × 4 → 컵 스탠드오프', BOARD='moteus r4.11 → 컵', BS='④ M2.5 × 5 저두 × 4 조임',
                      CUP='드라이버 컵 → hip 판 Ø80 보스', CS='⑤ M3 × 40 × 3 조임',
                      RING='받침 링 → Ø44 구멍 → PETG 벽 위', INSERT='hub 인서트 → 허벅지 안쪽 면 자리',
                      F6='⑥ M3 × 16 접시 × 6 → 받침 링 탭', PINS='③ 평행핀 × 3 → 인서트 압입',
                      THIGH='허벅지 → hip 출력부 (Ø11.8 스피곳 · 핀)', WASH='평와셔 M3 × 6 (Ø44 구멍으로)',
                      S316='② M3 × 16 (12.9) × 6 조임'),
        short=['모터 → hip 판 포켓', '① M2.5 × 6 (12.9)', 'moteus → 컵', '컵 → hip 판 (⑤)', '받침 링 → 허벅지',
               '인서트 + ⑥ + ③ 핀', '허벅지 → 출력부', '와셔 + ② M3 × 16'],
        legend=['① M2.5 × 6 (12.9) × 6', '② M3 × 16 (12.9) × 6 + 와셔', '③ 평행핀 Ø3 × 10 × 3',
                '④ M2.5 × 5 저두 × 4  ⑤ M3 × 40 × 3', '⑥ M3 × 16 접시 (12.9) × 6', '6061 인서트 · 받침 링, 인서트 M2.5 × 4'],
        major=['PLATE', 'MOTOR', 'THIGH', 'CUP', 'INSERT', 'RING'], done_labels=['PLATE', 'THIGH', 'CUP', 'MOTOR'],
        overview=('hip 관절 분해도  (오른쪽 다리)',
                  '모든 부품이 떨어진 상태 — 오른쪽이 바깥(허벅지 쪽), 왼쪽이 몸통 안쪽(드라이버 컵).  모두 hip 축 방향으로 끼워진다.\n'
                  '체결 부품: ① M2.5 × 6 × 6  ② M3 × 16 × 6 + 와셔  ③ 평행핀 × 3  ④ M2.5 × 5 × 4  ⑤ M3 × 40 × 3  ⑥ M3 × 16 접시 × 6'),
        done=('조립 완료 — hip 관절',
              '허벅지 → (PETG 벽을 인서트 플랜지와 받침 링이 ⑥으로 물음) → hub 인서트 → (② + ③ 핀 + Ø11.8 스피곳) → AKE 출력부\n'
              '→ 모터 베어링 → 고정자 → (① + Ø42 H7 파일럿) → hip 판 → 몸통.  드라이버 컵은 Ø80 보스로 중심을 맞추고 ⑤ 3개로 고정.'),
        note='색은 부품 구분용 (실제 허벅지·컵은 검정 PETG)  ·  허벅지는 hip 쪽만, hip 판 외 몸통은 생략  ·  배선·커넥터는 생략  ·  '
             '⑥은 모델에 없어 BOM 규격으로 그림')
