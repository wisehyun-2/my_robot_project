"""Right wheel (_R, CAD +y): outboard wheel group (11-01) - AKE60-8 in the PETG shank's wheel housing, moteus cup on
the housing's inboard end, PETG hub flange on the output, M365 tyre on the one-piece PETG rim."""
import axlib5 as X


def config(M):
    P = M['params']
    W = (0.0, P.R_WHEEL)
    K = (P.KNEE_X, P.KNEE_Z)
    py, ny = (0.0, 1.0, 0.0), (0.0, -1.0, 0.0)

    picks = [X.pick('SHANK', 'tibia_R:1+shank_R:', body='shank_R main', trims=X.along(W, K, 95.0)),
             X.pick('WINS', 'tibia_R:1+FX insert M2.5x4 link cup', each=py),
             X.pick('MOTOR', 'tibia_R:1+AKE60-8_KV80_stator'),
             X.pick('MOTOR', 'tibia_R:1+magnet spacer'),
             X.pick('MOTOR', 'wheel_R:1+AKE60-8_KV80_output'),
             X.pick('RING_S', 'tibia_R:1+clamp ring stator'),
             X.pick('S25', 'tibia_R:1+FX DIN 7984 M2.5x8 stator', each=ny),
             X.pick('CUP', 'tibia_R:1+cup_wheel_R'),
             X.pick('BOARD', 'tibia_R:1+moteus_r4.11'),
             X.pick('BOARD', 'tibia_R:1+CAN terminator'),
             X.pick('BOARD', 'tibia_R:1+XT30 cap'),
             X.pick('CINS', 'tibia_R:1+FX insert M2.5x4 cup', each=ny),
             X.pick('BS', 'tibia_R:1+FX DIN 7984 M2.5x6 board', each=ny),
             X.pick('CS', 'tibia_R:1+FX ISO 4762 M2.5x18 wheel cup', each=py),
             X.pick('PINS', 'wheel_R:1+FX ISO 8734 3x10', each=ny),
             X.pick('HUB', 'wheel_R:1+hub_flange_R'),
             X.pick('HINS', 'wheel_R:1+FX insert M3x5.7 hub', each=ny),
             X.pick('WASH', 'wheel_R:1+FX DIN 125 M3', each=ny, where=(1, 230.0, 250.0)),
             X.pick('S314', 'wheel_R:1+FX ISO 4762 M3x14 12.9 hub', each=ny),
             X.pick('RIM', 'wheel_R:1+rim_R'),
             X.pick('TYRE', 'wheel_R:1+tyre'),
             X.pick('RWASH', 'wheel_R:1+FX DIN 125 M3', each=ny, where=(1, 250.0, 270.0)),
             X.pick('RS', 'wheel_R:1+FX ISO 4762 M3x14 rim', each=ny)]

    home = dict(SHANK=(0, 0, 0, 0), WINS=(0, 0, 0, 30), MOTOR=(0, -90, 0, 0), CUP=(0, -185, 0, 0),
                CINS=(0, -185, 0, 60), BOARD=(0, -115, 0, 0), BS=(0, -115, 0, 22), CS=(0, -185, 0, 40),
                RING_S=(0, 60, 0, 0), S25=(0, 0, 0, 80), PINS=(0, 0, 0, 110), HUB=(0, 140, 0, 0),
                HINS=(0, 140, 0, 25), WASH=(0, 0, 0, 190), S314=(0, 0, 0, 205), RIM=(0, 230, 0, 0),
                TYRE=(0, 310, 0, 0), RWASH=(0, 0, 0, 380), RS=(0, 0, 0, 400))
    steps = [
        dict(key='s1', title='1단계  인서트 M2.5 × 3 → 바퀴 하우징 끝 (열압입)',
             text='인두로 황동 인서트 3개를 정강이 바퀴 하우징(Ø88)의 안쪽 끝면(정강이 안쪽 면과 같은 높이,\n'
                  '드라이버 컵 기둥 자리)에 녹여 넣는다.',
             moves=[[('WINS', 0)]], view=(0.8, -0.8, 0.45)),
        dict(key='s2', title='2단계  모터 → 정강이 바퀴 하우징',
             text='AKE60-8을 정강이 안쪽에서 하우징 안으로 넣어 Ø42 파일럿을 바깥쪽 바퀴 판(9 mm) 포켓에 앉힌다.\n'
                  '출력부는 판의 Ø30 구멍 쪽(바깥, 바퀴 쪽)을 향한다.',
             moves=[[('MOTOR', 0)]], view=(0.85, -0.75, 0.45)),
        dict(key='s3', title='3단계  SUS 링 + ① M2.5 × 8 × 6 → 모터 고정자',
             text='정강이 바깥 면의 홈에 SUS 클램프 링을 넣고, ① M2.5 × 8 저두 6개를 판을 지나 고정자 링 탭에\n'
                  '조인다 (허브 플랜지보다 먼저, 물림 3.2 mm).',
             moves=[[('RING_S', 0)], [('S25', 0)]], spin=['S25'], view=(0.85, 0.75, 0.45)),
        dict(key='s4', title='4단계  moteus → 드라이버 컵 (작업대)',
             text='컵 스탠드오프 4곳에 인서트 M2.5를 넣고, moteus r4.11(CAN 종단 저항·XT30 캡 포함)을 얹어\n'
                  '④ M2.5 × 6 저두 4개로 모터 쪽 면에서 조인다.',
             moves=[[('CINS', 'CUP')], [('BOARD', 'CUP')], [('BS', 'CUP')]], spin=['BS'],
             bench=['CUP', 'CINS', 'BOARD', 'BS'], view=(0.55, 0.85, 0.45)),
        dict(key='s5', title='5단계  드라이버 컵 → 바퀴 하우징 + ⑤ M2.5 × 18 × 3',
             text='컵 립(Ø88.1)을 하우징 안쪽 끝(Ø88)에 씌워 엔코더를 모터 축에 맞추고,\n'
                  '⑤ M2.5 × 18 3개를 컵 바닥에서 하우징 끝 인서트에 조인다.',
             moves=[[('CUP', 0), ('CINS', 0), ('BOARD', 0), ('BS', 0)], [('CS', 0)]], spin=['CS'],
             view=(0.85, -0.75, 0.45)),
        dict(key='s6', title='6단계  인서트 M3 × 6 → 허브 플랜지 (작업대)',
             text='인두로 황동 인서트 M3 × 5.7 6개를 허브 플랜지 바깥 면(림 쪽)의 구멍에 녹여 넣는다.',
             moves=[[('HINS', 'HUB')]], bench=['HUB', 'HINS'], view=(0.6, 0.8, 0.45)),
        dict(key='s7', title='7단계  ③ 핀 + 허브 플랜지 + 와셔 + ② M3 × 14 × 6',
             text='③ 평행핀 3개를 출력부 구멍에 꽂고, 허브 플랜지의 Ø28 보스를 바퀴 판 Ø30 구멍으로 넣어 스피곳을 맞춘 뒤\n'
                  '평와셔 + ② M3 × 14 (12.9) 6개로 출력부에 조인다 (머리는 플랜지 카운터보어 안, 물림 4.0 mm).',
             pre=dict(PINS=(0, 0, 0, 30), HUB=(0, 70, 0, 0), HINS=(0, 70, 0, 0), WASH=(0, 0, 0, 100),
                      S314=(0, 0, 0, 115)),
             moves=[[('PINS', 0)], [('HUB', 0), ('HINS', 0)], [('WASH', 0)], [('S314', 0)]], spin=['S314'],
             view=(0.8, 0.85, 0.45)),
        dict(key='s8', title='8단계  타이어 → 한 조각 림 (작업대)',
             text='M365 타이어를 뜨거운 물(60~80 °C)에 데워 림의 열린 쪽(바깥) 플랜지 위로 굴려 넣고,\n'
                  '안쪽 구멍을 림 돌기에 맞춘다 (토크는 돌기가 전한다).',
             moves=[[('TYRE', 'RIM')]], bench=['RIM', 'TYRE'], view=(0.7, 0.65, 0.4)),
        dict(key='s9', title='9단계  림 + 타이어 → 허브 플랜지 + ④ M3 × 14 × 6',
             text='림 허브의 홈을 허브 플랜지의 등록 턱에 맞춰 밀어 넣고,\n'
                  '바깥에서 평와셔 + ④ M3 × 14 6개를 허브 플랜지 인서트에 조인다.',
             pre=dict(RIM=(0, 120, 0, 0), TYRE=(0, 120, 0, 0), RWASH=(0, 0, 0, 170), RS=(0, 0, 0, 185)),
             moves=[[('RIM', 0), ('TYRE', 0)], [('RWASH', 0)], [('RS', 0)]], spin=['RS'],
             view=(0.6, 0.8, 0.35)),
    ]
    c = W
    tgt = (0.0, 245.0, P.R_WHEEL)
    cuts = [dict(key='cut1', remove=X.sector(c, 5.0, 90.0), view=(0.5, 0.55, 0.6), target=tgt, zoom=0.6,
                 title='단면 ①  ② M3 × 14 (5°) · ④ M3 × 14 (90°) 자리를 지나는 단면',
                 feats={'② M3 × 14 (12.9) + 와셔': X.on_sector_face(c, 5, 10.0, 243.5),
                        '④ M3 × 14 + 인서트': X.on_sector_face(c, 90, 18.0, 250.0),
                        '허브 플랜지 (PETG)': X.on_sector_face(c, 5, 22.0, 240.0),
                        '림 (PETG)': X.on_sector_face(c, 5, 45.0, 252.0),
                        '타이어 M365': X.on_sector_face(c, 5, 90.0, 270.0),
                        'AKE60-8': X.on_sector_face(c, 5, 26.0, 215.0),
                        '정강이 바퀴 하우징': X.on_sector_face(c, 5, 41.0, 225.0),
                        'moteus + 컵': X.on_sector_face(c, 5, 12.0, 197.0)}),
            dict(key='cut2', remove=X.sector(c, 35.0, 155.0), view=(-0.08, 0.6, 0.8), target=tgt, zoom=0.6,
                 title='단면 ②  ③ 평행핀 · ① M2.5 × 8 (35°, 155°) 자리를 지나는 단면',
                 feats={'③ 평행핀 Ø3 × 10': X.on_sector_face(c, 35, 10.0, 234.0),
                        '③ 평행핀 (155°)': X.on_sector_face(c, 155, 10.0, 234.0),
                        '① M2.5 × 8 + SUS 링': X.on_sector_face(c, 35, 18.25, 233.0),
                        '① M2.5 × 8 (155°)': X.on_sector_face(c, 155, 18.25, 233.0),
                        '타이어 M365': X.on_sector_face(c, 35, 90.0, 270.0),
                        '림 (PETG)': X.on_sector_face(c, 155, 45.0, 252.0)})]
    return dict(
        job='wheel', title='바퀴 관절 조립 (v5)', subtitle='오른쪽 다리 (_R) · Fusion 모델 버전 11',
        picks=picks, fasteners=[], fixed=['SHANK'],
        recolor={'SHANK': ('ax petg green', (120, 185, 120)), 'CUP': ('ax cup grey', (150, 150, 158)),
                 'HUB': ('ax hub orange', (215, 110, 55)), 'RIM': ('ax rim orange', (240, 165, 85))},
        home=home, steps=steps, cuts=cuts, target=(0.0, 0.0, P.R_WHEEL), view0=(1.0, 0.3, 0.4),
        view_done=(0.85, 0.6, 0.45), move=36, long_move=56,
        names=dict(SHANK='정강이 (PETG, 바퀴 쪽만)', WINS='황동 인서트 M2.5 × 3 (하우징)', MOTOR='AKE60-8 모터 (바퀴)',
                   RING_S='SUS304 클램프 링 Ø45', S25='① M2.5 × 8 저두 × 6', CUP='드라이버 컵 (PETG)',
                   CINS='황동 인서트 M2.5 × 4 (컵)', BOARD='moteus r4.11', BS='④ M2.5 × 6 저두 × 4',
                   CS='⑤ M2.5 × 18 × 3', PINS='③ 평행핀 Ø3 × 10 × 3', HUB='허브 플랜지 (PETG)',
                   HINS='황동 인서트 M3 × 6 (허브)', WASH='평와셔 M3 × 6', S314='② M3 × 14 (12.9) × 6',
                   RIM='한 조각 림 (PETG)', TYRE='M365 8.5 × 2 통고무 타이어', RWASH='평와셔 M3 × 6 (림)',
                   RS='④ M3 × 14 × 6'),
        captions=dict(WINS='황동 인서트 M2.5 × 3 → 하우징 끝', MOTOR='AKE60-8 → 바퀴 판 포켓 (Ø42 파일럿)',
                      RING_S='SUS304 클램프 링 → 정강이 바깥 면 홈', S25='① M2.5 × 8 저두 × 6 조임',
                      CINS='황동 인서트 M2.5 × 4 → 컵 스탠드오프', BOARD='moteus r4.11 → 컵',
                      BS='④ M2.5 × 6 저두 × 4 조임', CUP='드라이버 컵 → 하우징 끝 (Ø88 립)', CS='⑤ M2.5 × 18 × 3 조임',
                      HINS='황동 인서트 M3 × 6 → 허브 플랜지', PINS='③ 평행핀 × 3 → 출력부',
                      HUB='허브 플랜지 → 출력부 (Ø28 보스 · 스피곳)', WASH='평와셔 M3 × 6',
                      S314='② M3 × 14 (12.9) × 6 조임', TYRE='타이어 → 림 (데워서, 구멍 ↔ 돌기)',
                      RIM='림 + 타이어 → 허브 플랜지', RWASH='평와셔 M3 × 6 (림)', RS='④ M3 × 14 × 6 조임'),
        short=['인서트 → 하우징 끝', '모터 → 바퀴 하우징', 'SUS 링 + ① M2.5 × 8', 'moteus → 컵', '컵 → 하우징 (⑤)',
               '인서트 → 허브 플랜지', '③ 핀 + 허브 + ②', '타이어 → 림', '림 → 허브 + ④'],
        legend=['① M2.5 × 8 저두 × 6 + SUS 링', '② M3 × 14 (12.9) × 6 + 와셔', '③ 평행핀 Ø3 × 10 × 3',
                '④ M3 × 14 × 6 + 와셔 (림)', '⑤ M2.5 × 18 × 3, moteus M2.5 × 6 × 4', '인서트 M2.5 × 7, M3 × 6'],
        major=['SHANK', 'MOTOR', 'CUP', 'HUB', 'RIM', 'TYRE'], done_labels=['SHANK', 'CUP', 'RIM', 'TYRE'],
        overview=('바퀴 관절 분해도  (오른쪽 다리)',
                  '모든 부품이 떨어진 상태 — 오른쪽이 바깥(타이어 쪽), 왼쪽이 로봇 중심 쪽(드라이버 컵).  모두 바퀴 축 방향으로 끼워진다.\n'
                  '체결 부품: ① M2.5 × 8 × 6  ② M3 × 14 (12.9) × 6 + 와셔  ③ 평행핀 × 3  ④ M3 × 14 × 6 + 와셔 + 인서트  ⑤ M2.5 × 18 × 3'),
        done=('조립 완료 — 바퀴 관절',
              '타이어 → (돌기) → 림 → (④ + 등록 턱) → 허브 플랜지 → (② + ③ 핀 + 스피곳) → AKE 출력부 → 고정자 → (① + Ø42 파일럿) → 정강이.\n'
              '드라이버 컵은 하우징 Ø88 끝에 립으로 중심을 맞추고 ⑤ 3개로 고정 (하중 없음).'),
        note='색은 부품 구분용 (실제 정강이·컵·허브 플랜지·림은 검정 PETG)  ·  정강이는 바퀴 쪽만  ·  배선은 생략  ·  나사·와셔·핀·인서트는 모델에 있는 그대로')
