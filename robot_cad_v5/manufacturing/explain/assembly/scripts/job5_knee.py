"""Right knee (_R, CAD +y): PETG fork thigh (knee plate + outer spar) + AKE60-8 + moteus cup, PETG shank with the
printed hub insert / backing ring / 61807, the shank held 9.5 mm outboard into the fork then slid onto the output
(plan A, 11-01), output screws through the spar hole, then the printed knee cap (every part printed since 11-09)."""
import axlib5 as X


def config(M):
    P, LV = M['params'], M['legs_v5']
    K = (P.KNEE_X, P.KNEE_Z)
    H = (0.0, P.Z_HIP)
    W = (0.0, P.R_WHEEL)
    py, ny = (0.0, 1.0, 0.0), (0.0, -1.0, 0.0)
    y_out = P.y_output('knee')
    held = LV.SPAR[0] - LV.Y_SH - 1.0                 # 9.5: the shank's offset while it goes into the fork

    picks = [X.pick('THIGH', 'femur_R:1+thigh_R:', body='thigh_R main', trims=X.along(K, H, 140.0)),
             X.pick('TINS', 'femur_R:1+FX insert M2.5x4 link cup', each=py),
             X.pick('MOTOR', 'femur_R:1+AKE60-8_KV80_stator'),
             X.pick('MOTOR', 'femur_R:1+magnet spacer'),
             X.pick('MOTOR', 'tibia_R:1+AKE60-8_KV80_output'),
             X.pick('RING_S', 'femur_R:1+clamp ring stator'),
             X.pick('S25', 'femur_R:1+FX DIN 7984 M2.5x8 stator', each=ny),
             X.pick('CUP', 'femur_R:1+cup_knee_R'),
             X.pick('BOARD', 'femur_R:1+moteus_r4.11'),
             X.pick('CINS', 'femur_R:1+FX insert M2.5x4 cup', each=ny),
             X.pick('BS', 'femur_R:1+FX DIN 7984 M2.5x5 board', each=ny),
             X.pick('CS', 'femur_R:1+FX ISO 4762 M2.5x18 knee cup', each=py),
             X.pick('SHANK', 'tibia_R:1+shank_R:', body='shank_R main', trims=X.along(K, W, 105.0)),
             X.pick('INSERT', 'tibia_R:1+shank_R:', body='shank_R insert'),
             X.pick('RING', 'tibia_R:1+shank_R:', body='shank_R ring'),
             X.pick('PINS', 'tibia_R:1+FX ISO 8734 3x10', each=py),
             X.pick('BRG', 'tibia_R:1+kneebearing_R'),
             X.pick('WASH', 'tibia_R:1+FX DIN 125 M3', each=ny),
             X.pick('S316', 'tibia_R:1+FX ISO 4762 M3x16 12.9', each=ny),
             X.pick('CAP', 'femur_R:1+kneecap_R')]
    F = []
    for k in range(6):                         # 6 at r 18, 30 + 60 k deg: insert flange (flush) -> PETG wall -> ring tap
        q = X.polar(K, LV.FLANGE_BOLT_R, 30 + 60 * k)
        F.append(X.fast('F6', 'ISO 10642 M3x16 12.9 knee #%d' % k, X.csk(3, 16), (q[0], LV.Y_SI, q[1]), py, 'black satin'))
    for k in range(3):                         # knee cap: 3 x M2.5 x 8 DIN 7984 into inserts in the spar (r 20.5)
        q = X.polar(K, LV.CAP_BOLT_R, 90 + 120 * k)
        F.append(X.fast('CAPINS', 'insert M2.5 cap #%d' % k, X.insert(2.5, 4.0, 3.6),
                        (q[0], LV.SPAR[1] - LV.CAP_T, q[1]), ny, 'brass'))
        F.append(X.fast('F7', 'DIN 7984 M2.5x8 cap #%d' % k, X.lowhead(2.5, 8), (q[0], LV.SPAR[1], q[1]), ny))

    u = X.unit((W[0] - K[0], 0.0, W[1] - K[1]))
    R = 190.0                                  # the shank waits this far out along its own length (towards the wheel)
    SH = (u[0] * R, held, u[2] * R)
    home = dict(THIGH=(0, 0, 0, 0), TINS=(0, 0, 0, 30), MOTOR=(0, -90, 0, 0), RING_S=(0, 25, 0, 0), S25=(0, 0, 0, 45),
                CUP=(0, -200, 0, 0), CINS=(0, -200, 0, 30), BOARD=(0, -150, 0, 0), BS=(0, -150, 0, 25),
                CS=(0, -200, 0, 40), CAPINS=(0, 0, 0, 45), CAP=(0, 110, 0, 0), F7=(0, 0, 0, 130),
                WASH=(0, 0, 0, 230), S316=(0, 0, 0, 250),
                SHANK=SH + (0,), RING=(SH[0], SH[1] + 110, SH[2], 0), INSERT=(SH[0], SH[1] - 70, SH[2], 0),
                F6=(SH[0], SH[1] - 70, SH[2], 25), PINS=(SH[0], SH[1] - 70, SH[2], 12),
                BRG=(SH[0], SH[1] + 55, SH[2], 0))
    sset = ['SHANK', 'RING', 'INSERT', 'F6', 'PINS', 'BRG']
    steps = [
        dict(key='s1', title='1단계  인서트 M2.5 × 3 → 무릎 하우징 끝 (열압입)',
             text='인두로 황동 인서트 3개를 허벅지 무릎 하우징(Ø88)의 안쪽 끝면(드라이버 컵 기둥 자리)에 녹여 넣는다.',
             moves=[[('TINS', 0)]], view=(0.8, -0.8, 0.45)),
        dict(key='s2', title='2단계  인서트 M2.5 × 3 → 바깥 막대 캡 자리 (열압입)',
             text='바깥 막대 바깥 면의 캡 자리(Ø46.4 × 3 홈) 바닥의 구멍 3개(r 20.5)에 황동 인서트를 녹여 넣는다.',
             moves=[[('CAPINS', 0)]], view=(0.75, 0.8, 0.45)),
        dict(key='s3', title='3단계  모터 → 허벅지 무릎 포켓',
             text='AKE60-8을 허벅지 안쪽에서 하우징 안으로 넣어 Ø42 파일럿을 무릎 판(9 mm) 포켓에 앉힌다.\n'
                  '출력부는 판의 Ø30 구멍 쪽(갈래 틈 쪽)을 향한다.',
             moves=[[('MOTOR', 0)]], view=(0.9, -0.75, 0.45)),
        dict(key='s4', title='4단계  클램프 링 + ① M2.5 × 8 × 6 → 모터 고정자',
             text='갈래 틈 쪽 무릎 판 면의 홈에 클램프 링(PETG 출력, Ø45 / Ø30, 2 t)을 넣고, ① M2.5 × 8 저두 6개를\n'
                  '판을 지나 고정자 링 탭에 조인다 (짧은 육각 렌치로 갈래 틈 59.5 mm 안에서, 물림 3.2 mm).',
             moves=[[('RING_S', 0)], [('S25', 0)]], spin=['S25'], ghost=['THIGH'], view=(0.85, 0.75, 0.4)),
        dict(key='s5', title='5단계  moteus → 드라이버 컵 (작업대)',
             text='컵 스탠드오프 4곳에 인서트 M2.5를 넣고, moteus r4.11을 얹어\n④ M2.5 × 5 저두 4개로 모터 쪽 면에서 조인다.',
             moves=[[('CINS', 'CUP')], [('BOARD', 'CUP')], [('BS', 'CUP')]], spin=['BS'],
             bench=['CUP', 'CINS', 'BOARD', 'BS'], view=(0.55, 0.85, 0.45)),
        dict(key='s6', title='6단계  드라이버 컵 → 무릎 하우징 + ⑤ M2.5 × 18 × 3',
             text='컵 립을 하우징(Ø88) 끝에 씌워 엔코더를 모터 축에 맞추고,\n⑤ M2.5 × 18 3개를 컵 바닥에서 하우징 끝 인서트에 조인다.',
             moves=[[('CUP', 0), ('CINS', 0), ('BOARD', 0), ('BS', 0)], [('CS', 0)]], spin=['CS'],
             view=(0.9, -0.7, 0.45)),
        dict(key='s7', title='7단계  정강이 준비 (작업대)',
             text='Ø44 구멍으로 받침 링을 떨어뜨려 PETG 벽 위에 앉히고, 안쪽 면 자리에 hub 인서트를 넣어 ⑥ M3 × 16 접시 6개를\n'
                  '받침 링에 가볍게 조인다 (플라스틱 나사산). ③ 평행핀 3개를 인서트에 압입하고, 61807을 패드 바깥 면 자리(Ø47)에 턱까지 압입한다.',
             moves=[[('RING', 'SHANK')], [('INSERT', 'SHANK')], [('F6', 'SHANK')], [('PINS', 'SHANK')],
                    [('BRG', 'SHANK')]], spin=['F6'], ghost=['SHANK'], bench=sset, view=(0.55, -0.75, 0.5)),
        dict(key='s8', title='8단계  정강이 → 갈래 틈 → 무릎 출력부',
             text='정강이를 9.5 mm 바깥에 띄운 채 갈래 틈으로 밀어 넣고 (핀 끝 ↔ 무릎 판 1.0, 패드 ↔ 바깥 막대 1.0),\n'
                  '안쪽으로 밀어 Ø28 보스를 Ø30 구멍에, Ø11.8 스피곳을 출력부 홈에, ③ 핀을 출력부 구멍에 넣는다.',
             moves=[[(g, (0.0, held, 0.0, 0.0)) for g in sset], [(g, 0) for g in sset]], show=['SHANK'],
             view=(0.9, 0.5, 0.2)),
        dict(key='s9', title='9단계  와셔 + ② M3 × 16 (12.9) × 6 → 출력부',
             text='바깥 막대의 캡 구멍(Ø35) → 61807 → Ø44 구멍으로 평와셔(먼저)와 ② M3 × 16 6개를 넣어\n'
                  '출력부 탭에 조인다 (물림 4.3 mm, 렌치 길이 약 76 mm).',
             pre=dict(WASH=(0, 0, 0, 100), S316=(0, 0, 0, 120)),
             moves=[[('WASH', 0)], [('S316', 0)]], spin=['S316'], view=(0.45, 0.85, 0.3), zoom=0.75),
        dict(key='s10', title='10단계  무릎 캡 + ⑦ M2.5 × 8 × 3',
             text='무릎 캡(PETG 출력)의 관(Ø35)을 바깥 막대 구멍(Ø35)과 61807 안륜에 밀어 넣고 (사포로 꼭 맞게 다듬음, 고무망치),\n'
                  '면과 같은 높이의 플랜지를 ⑦ M2.5 × 8 저두 3개로 인서트에 조인다.',
             moves=[[('CAP', 0)], [('F7', 0)]], spin=['F7'], view=(0.75, 0.8, 0.45)),
    ]
    c = K
    tgt = (K[0], 225.0, K[1])
    cuts = [dict(key='cut1', remove=X.sector(c, 10.0, 90.0), view=(0.5, 0.5, 0.62), target=tgt, zoom=0.5,
                 title='단면 ①  ② M3 × 16 (10°) · ⑥ 플랜지 볼트 · ⑦ 캡 나사 (90°) 자리를 지나는 단면',
                 feats={'② M3 × 16 + 와셔': X.on_sector_face(c, 10, 10.0, 211.0),
                        'hub 인서트 (PETG)': X.on_sector_face(c, 10, 21.0, 211.0),
                        '받침 링 (PETG)': X.on_sector_face(c, 10, 18.0, 223.0),
                        '⑥ M3 × 16 접시': X.on_sector_face(c, 90, 18.0, 214.0),
                        '61807': X.on_sector_face(c, 10, 20.5, 249.5),
                        '무릎 캡 (PETG)': X.on_sector_face(c, 10, 16.0, 270.0),
                        '⑦ M2.5 × 8 + 인서트': X.on_sector_face(c, 90, 20.5, 284.0),
                        'A Ø11.8 스피곳': X.on_sector_face(c, 10, 5.0, y_out - 0.7),
                        '정강이 (PETG)': X.on_sector_face(c, 10, 40.0, 210.0),
                        '무릎 판 (허벅지)': X.on_sector_face(c, 10, 40.0, 199.0),
                        '바깥 막대 (허벅지)': X.on_sector_face(c, 10, 24.0, 276.0),
                        'AKE60-8': X.on_sector_face(c, 10, 26.0, 182.0)}),
            dict(key='cut2', remove=X.sector(c, 40.0, 160.0), view=(-0.14, 0.62, 0.8), target=tgt, zoom=0.5,
                 title='단면 ②  ③ 평행핀 · ① M2.5 × 8 (40°, 160°) 자리를 지나는 단면',
                 feats={'③ 평행핀 Ø3 × 10': X.on_sector_face(c, 40, 10.0, 200.5),
                        '③ 평행핀 (160°)': X.on_sector_face(c, 160, 10.0, 200.5),
                        '① M2.5 × 8 + 클램프 링': X.on_sector_face(c, 40, 18.25, 199.0),
                        '① M2.5 × 8 (160°)': X.on_sector_face(c, 160, 18.25, 199.0),
                        '61807': X.on_sector_face(c, 40, 20.5, 249.5),
                        '정강이 (PETG)': X.on_sector_face(c, 40, 40.0, 210.0),
                        '바깥 막대 (허벅지)': X.on_sector_face(c, 160, 24.0, 276.0)})]
    return dict(
        job='knee', title='무릎 관절 조립 (v5)', subtitle='오른쪽 다리 (_R) · Fusion 모델 버전 18',
        picks=picks, fasteners=F, fixed=['THIGH'],
        recolor={'THIGH': ('ax thigh blue', (110, 150, 205)), 'SHANK': ('ax petg green', (120, 185, 120)),
                 'CUP': ('ax cup grey', (150, 150, 158))},
        home=home, steps=steps, cuts=cuts, target=(K[0], 0.0, K[1]), view0=(0.95, 0.4, 0.3),
        view_done=(0.85, 0.6, 0.45), move=36, long_move=56,
        names=dict(THIGH='허벅지 (PETG 갈래, 무릎 쪽만)', TINS='황동 인서트 M2.5 × 3 (하우징)', MOTOR='AKE60-8 모터 (무릎)',
                   RING_S='클램프 링 Ø45 (PETG)', S25='① M2.5 × 8 저두 × 6', CUP='드라이버 컵 (PETG)',
                   CINS='황동 인서트 M2.5 × 4 (컵)', BOARD='moteus r4.11', BS='④ M2.5 × 5 저두 × 4',
                   CS='⑤ M2.5 × 18 × 3', SHANK='정강이 (PETG, 무릎 쪽만)', INSERT='hub 인서트 (PETG)',
                   RING='받침 링 (PETG)', F6='⑥ M3 × 16 접시 (12.9) × 6', PINS='③ 평행핀 Ø3 × 10 × 3',
                   BRG='61807 베어링', WASH='평와셔 M3 × 6', S316='② M3 × 16 (12.9) × 6', CAP='무릎 캡 (PETG)',
                   CAPINS='황동 인서트 M2.5 × 3 (캡 자리)', F7='⑦ M2.5 × 8 저두 × 3'),
        captions=dict(TINS='황동 인서트 M2.5 × 3 → 하우징 끝', CAPINS='황동 인서트 M2.5 × 3 → 바깥 막대 캡 자리',
                      MOTOR='AKE60-8 → 무릎 포켓 (Ø42 파일럿)', RING_S='클램프 링 → 무릎 판 홈',
                      S25='① M2.5 × 8 저두 × 6 조임', CINS='황동 인서트 M2.5 × 4 → 컵 스탠드오프',
                      BOARD='moteus r4.11 → 컵', BS='④ M2.5 × 5 저두 × 4 조임', CUP='드라이버 컵 → 하우징 (Ø88 립)',
                      CS='⑤ M2.5 × 18 × 3 조임', RING='받침 링 → Ø44 구멍 → PETG 벽 위',
                      INSERT='hub 인서트 → 정강이 안쪽 면 자리', F6='⑥ M3 × 16 접시 × 6 → 받침 링 탭',
                      PINS='③ 평행핀 × 3 → 인서트 압입', BRG='61807 → 패드 바깥 면 자리',
                      SHANK='정강이를 9.5 mm 띄워 갈래 틈으로 → 안쪽으로 밀어 출력부에', WASH='평와셔 M3 × 6 (캡 구멍으로)',
                      S316='② M3 × 16 (12.9) × 6 조임', CAP='무릎 캡 → 바깥 막대 구멍 + 61807 안륜',
                      F7='⑦ M2.5 × 8 저두 × 3 조임'),
        short=['인서트 → 하우징 끝', '인서트 → 캡 자리', '모터 → 무릎 포켓', '클램프 링 + ① M2.5 × 8', 'moteus → 컵',
               '컵 → 하우징 (⑤)', '정강이 준비', '정강이 → 갈래 → 출력부', '와셔 + ② M3 × 16', '무릎 캡 + ⑦'],
        legend=['① M2.5 × 8 저두 × 6 + 클램프 링', '② M3 × 16 (12.9) × 6 + 와셔', '③ 평행핀 Ø3 × 10 × 3',
                '④ M2.5 × 5 × 4  ⑤ M2.5 × 18 × 3', '⑥ M3 × 16 접시 × 6  ⑦ M2.5 × 8 × 3', '61807, 인서트 M2.5 × 10'],
        major=['THIGH', 'MOTOR', 'SHANK', 'CUP', 'CAP', 'INSERT', 'RING', 'BRG'],
        done_labels=['THIGH', 'SHANK', 'CUP', 'CAP'],
        overview=('무릎 관절 분해도  (오른쪽 다리)',
                  '오른쪽이 바깥(바깥 막대·무릎 캡 쪽), 왼쪽이 로봇 중심 쪽(드라이버 컵). 정강이는 갈래 틈으로 옆에서 들어간 뒤 축 방향으로 끼워진다.\n'
                  '체결 부품: ① M2.5 × 8 × 6  ② M3 × 16 × 6 + 와셔  ③ 핀 × 3  ④ M2.5 × 5 × 4  ⑤ M2.5 × 18 × 3  ⑥ M3 × 16 접시 × 6  ⑦ M2.5 × 8 × 3'),
        done=('조립 완료 — 무릎 관절',
              '정강이 → (⑥ + 받침 링) → hub 인서트 → (② + ③ 핀 + Ø11.8 스피곳) → AKE 출력부 → 고정자 → (① + Ø42 파일럿) → 무릎 판.\n'
              '바깥에서는 61807(정강이 패드) → 무릎 캡(⑦) → 바깥 막대가 무릎을 한 번 더 잡는다.'),
        note='색은 부품 구분용 (실제 출력 부품은 모두 검정 PETG)  ·  허벅지·정강이는 무릎 쪽만  ·  배선·커넥터는 생략  ·  '
             '⑥·⑦·캡 인서트는 모델에 없어 BOM 규격으로 그림')
