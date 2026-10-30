"""Torso: hip units (hip plate + motor + driver, from job_hip), spine bars, rails, electronics deck, battery tray,
battery, payload plate, TITA-like shell (hip sections, front / rear pieces on splice tabs) and the glued decor."""
import axlib as X


def config(M):
    C, P, TO, SH = M['cadlib'], M['params'], M['torso'], M['shell']
    ZH = P.Z_HIP
    y_in = P.YW - P.HIP_PLATE_T                       # hip plate inner face (164)
    y_web = y_in + P.HIP_WEB_T                        # web outer face (169): countersinks, shell taps
    z1 = ZH + P.DECK_Z                                # deck top
    z0 = ZH + P.RAIL_Z + P.RAIL_S / 2                 # rail top = top plate underside

    def unit_side(k, s):
        return [('HIP' + k, 'torso:1+hip_plate_' + k, None, None),
                ('HIP' + k, 'torso:1+AKE60-8_KV80_stator', s, None), ('HIP' + k, 'torso:1+magnet spacer', s, None),
                ('HIP' + k, 'torso:1+cup_hip_' + k, None, None), ('HIP' + k, 'torso:1+moteus_r4.11', s, None),
                ('HIP' + k, 'femur_%s:1+AKE60-8_KV80_output' % k, None, None)]
    picks = unit_side('L', 1) + unit_side('R', -1) + [
        ('TUBES', 'torso:1+spine_tube_', None, None), ('RAILS', 'torso:1+rail_', None, None),
        ('DECK', 'torso:1+electronics_deck', None, None), ('RPI', 'torso:1+raspberry_pi_4b', None, None),
        ('STAND', 'torso:1+standoff M2.5x11', None, None), ('HAT', 'torso:1+pi3hat', None, None),
        ('MJ', 'torso:1+mjpower_ss', None, None), ('MJ', 'torso:1+XT90-F to XT60-M adapter', None, None),
        ('FAN', 'torso:1+fan 30x30', None, None), ('TRAY', 'torso:1+battery_tray', None, None),
        ('BATT', 'torso:1+battery_24V', None, None), ('TOP', 'torso:1+top_plate', None, None),
        ('SHMID_L', 'torso:1+shell_mid_L', None, None), ('SHMID_L', 'torso:1+saddle_L', None, None),
        ('SHMID_L', 'torso:1+drum_ring_L', None, None), ('SHMID_L', 'torso:1+deco_light_ring_L', None, None),
        ('SHMID_L', 'torso:1+deco_drum_arc_L', None, None),
        ('SHMID_R', 'torso:1+shell_mid_R', None, None), ('SHMID_R', 'torso:1+saddle_R', None, None),
        ('SHMID_R', 'torso:1+drum_ring_R', None, None), ('SHMID_R', 'torso:1+deco_light_ring_R', None, None),
        ('SHMID_R', 'torso:1+deco_drum_arc_R', None, None),
        ('SHFR', 'torso:1+shell_front_', None, None), ('SHFR', 'torso:1+deco_face_panel', None, None),
        ('SHFR', 'torso:1+deco_camera', None, None),
        ('SHRE', 'torso:1+shell_rear_', None, None), ('SHRE', 'torso:1+deco_rear_panel', None, None),
        ('TABS', 'torso:1+splice_tabs', None, None),
        ('DECO', 'torso:1+deco_stripes_blue', None, None), ('DECO', 'torso:1+deco_lines_white', None, None)]
    F = []
    for s, k in ((1, 'L'), (-1, 'R')):
        e_in = (0.0, -s, 0.0)                         # from the outer face towards the robot centre
        for sx in (-1, 1):
            F.append(X.fast('S_M5' + k, 'M5x12 ISO10642 %s%d' % (k, sx), X.csk(5, 12), (sx * P.TUBE_X, s * y_web, ZH + P.TUBE_Z), e_in,
                            'black satin'))
            F.append(X.fast('S_M4' + k, 'M4x12 ISO10642 %s%d' % (k, sx), X.csk(4, 12), (sx * P.RAIL_X, s * y_web, ZH + P.RAIL_Z), e_in,
                            'black satin'))
            F.append(X.fast('S_DK' + k, 'M3x8 deck %s%d' % (k, sx), X.shcs(3, 8), (sx * 45.0, s * (y_in - 3.5), ZH - 42.0),
                            (0.0, s, 0.0)))
        for i, (x, z) in enumerate(((-50.0, 30.0), (50.0, 30.0), (-60.0, -20.0), (60.0, -20.0), (-40.0, 140.0), (40.0, 140.0))):
            F.append(X.fast('S_SH' + k, 'M3x6 button shell %s%d' % (k, i), X.button(3, 6), (x, s * P.YW, ZH + z), e_in))
    for i, (x, y) in enumerate(TO.rpi_holes_robot() + TO.mjpower_holes_robot()):
        F.append(X.fast('DINS', 'insert M2.5 deck #%d' % i, X.insert(2.5, 4.0, 3.6), (x, y, z1 + 4.9), (0, 0, -1), 'brass'))
    for i, (x, y) in enumerate(TO.mjpower_holes_robot()):
        F.append(X.fast('MJS', 'M2.5x6 DIN7984 mjpower #%d' % i, X.lowhead(2.5, 6), (x, y, z1 + 5.0 + 1.6), (0, 0, -1)))
    for sx in (-1, 1):
        for yy in (-100.0, -33.3, 33.3, 100.0):
            F.append(X.fast('TBOLT', 'M4x25 top %d %.0f' % (sx, yy), X.shcs(4, 25), (sx * P.RAIL_X, yy, z0 + 2.0), (0, 0, -1),
                            'black satin'))
            F.append(X.fast('TNUT', 'washer M4 %d %.0f' % (sx, yy), X.washer(4), (sx * P.RAIL_X, yy, z0 - P.RAIL_S - 0.8),
                            (0, 0, 1)))
            F.append(X.fast('TNUT', 'nyloc M4 %d %.0f' % (sx, yy), X.nut(4, nyloc=True), (sx * P.RAIL_X, yy, z0 - P.RAIL_S - 5.8),
                            (0, 0, 1)))
    for (sx, sy, where) in SH.SPLICE:
        q, d, n = SH._seam_surface(sx, sy, where)
        for dx in (-6.0, 6.0):
            a = C.add(q, C.mul(d, dx))
            F.append(X.fast('S_TAB', 'M3x8 button tab %d%d%s%+.0f' % (sx, sy, where, dx), X.button(3, 8), a, C.mul(n, -1)))
            F.append(X.fast('TABNUT', 'M3 nut tab %d%d%s%+.0f' % (sx, sy, where, dx), X.nut(3),
                            C.sub(a, C.mul(n, P.SHELL_T + 2.5 + 2.4)), n))

    home = dict(HIPL=(0, 0, 0, 0), TUBES=(0, -120, 0, 0), RAILS=(0, -120, 0, 0), S_M5L=(0, 0, 0, 50), S_M4L=(0, 0, 0, 50),
                DECK=(0, 0, -220, 0), DINS=(0, 0, -220, 40), RPI=(0, 0, -150, 0), STAND=(0, 0, -120, 0),
                HAT=(0, 0, -95, 0), MJ=(0, 0, -150, 0), MJS=(0, 0, -150, 30), FAN=(0, 0, -150, 0),
                S_DKL=(0, 0, -220, 25), S_DKR=(0, 0, -220, 25), HIPR=(0, -260, 0, 0), S_M5R=(0, -260, 0, 60),
                S_M4R=(0, -260, 0, 60), TRAY=(0, 0, 200, 0), BATT=(0, 0, 300, 0), TOP=(0, 0, 400, 0),
                TBOLT=(0, 0, 400, 40), TNUT=(0, 0, 0, 70), SHMID_L=(0, 260, 300, 0), SHMID_R=(0, -260, 300, 0),
                S_SHL=(0, 0, 0, 110), S_SHR=(0, 0, 0, 110), TABS=(0, 0, 520, 0), SHFR=(320, 0, 60, 0),
                SHRE=(-320, 0, 60, 0), S_TAB=(0, 0, 0, 70), TABNUT=(0, 0, 0, 40), DECO=(0, 0, 650, 0))
    deck_all = ['DECK', 'DINS', 'RPI', 'STAND', 'HAT', 'MJ', 'MJS', 'FAN']
    steps = [
        dict(key='s1', title='1단계  전장 데크 준비 (작업대)',
             text='데크 보스 8곳에 인서트 M2.5를 넣고, Raspberry Pi 4B를 얹어 M2.5 × 11 스탠드오프 4개로 고정,\n'
                  'pi3hat을 스탠드오프 위에, mjpower-ss를 M2.5 × 6 저두 4개로, 팬을 폼테이프로 붙인다.',
             moves=[[('DINS', 'DECK')], [('RPI', 'DECK')], [('STAND', 'DECK')], [('HAT', 'DECK')], [('MJ', 'DECK')],
                    [('MJS', 'DECK')], [('FAN', 'DECK')]], spin=['MJS'], bench=deck_all, view=(0.75, 0.35, 0.9)),
        dict(key='s2', title='2단계  척추 봉 + 위 레일 → 왼쪽 hip 유닛',
             text='hip 유닛(hip 판 + 모터 + 드라이버 컵, hip 영상 참고)에 Ø16 척추 봉 2개와 15 × 15 레일 2개를 대고,\n'
                  '바깥 면 접시 구멍에서 ① M5 × 12 × 2, ② M4 × 12 × 2를 봉·레일 끝 탭에 조인다.',
             moves=[[('TUBES', 0), ('RAILS', 0)], [('S_M5L', 0), ('S_M4L', 0)]], spin=['S_M5L', 'S_M4L'],
             view=(0.9, 0.75, 0.5)),
        dict(key='s3', title='3단계  전장 데크 → 왼쪽 hip 판',
             text='준비한 전장 데크를 hip 판 사이 아래쪽에 넣고,\n데크 플랜지 안쪽에서 ③ M3 × 8 2개를 왼쪽 hip 판 M3 탭에 조인다.',
             moves=[[(g, 0) for g in deck_all], [('S_DKL', 0)]], spin=['S_DKL'], view=(0.95, -0.45, 0.25)),
        dict(key='s4', title='4단계  오른쪽 hip 유닛 → 봉·레일·데크',
             text='오른쪽 hip 유닛을 봉·레일 끝에 대고 ① M5 × 12 × 2, ② M4 × 12 × 2를 조인 뒤,\n데크 오른쪽 플랜지를 ③ M3 × 8 2개로 조인다.',
             moves=[[('HIPR', 0)], [('S_M5R', 0), ('S_M4R', 0)], [('S_DKR', 0)]], spin=['S_M5R', 'S_M4R', 'S_DKR'],
             view=(0.85, -0.8, 0.5)),
        dict(key='s5', title='5단계  배터리 트레이 + 배터리 → 척추 봉',
             text='트레이 밑 안장 8곳을 척추 봉에 얹고, 폼 패드 위에 배터리(24 V 15 Ah)를 놓는다.\n'
                  '벨크로 스트랩 2개(25 × 400)를 트레이 슬롯으로 돌려 배터리·트레이를 봉에 묶는다 (스트랩은 모델 없음).',
             moves=[[('TRAY', 0)], [('BATT', 0)]], view=(0.8, 0.55, 0.85)),
        dict(key='s6', title='6단계  윗판 → 위 레일 + ④ M4 × 25 × 8',
             text='윗판(PETG, M4 인서트 격자)을 레일에 얹고 ④ M4 × 25 8개를 윗판 카운터보어에서 레일을 관통해 넣어\n'
                  '레일 밑에서 평와셔 + 나일론 너트로 조인다.',
             moves=[[('TOP', 0)], [('TBOLT', 0)], [('TNUT', 0)]], spin=['TBOLT', 'TNUT'], view=(0.8, 0.55, 0.8)),
        dict(key='s7', title='7단계  셸 hip 부분 → hip 판 + ⑤ M3 × 6 × 12',
             text='셸 hip 부분(좌·우, 파란 안장·드럼 링 포함)을 hip 판 보스에 끼워 옆에서 씌우고,\n'
                  '⑤ M3 × 6 버튼 볼트 6개씩을 hip 판 웹의 M3 탭에 조인다.',
             moves=[[('SHMID_L', 0), ('SHMID_R', 0)], [('S_SHL', 0), ('S_SHR', 0)]], spin=['S_SHL', 'S_SHR'],
             view=(0.85, 0.75, 0.55)),
        dict(key='s8', title='8단계  셸 앞·뒤 조각 + 이음판 + ⑥ M3 × 8 × 24',
             text='이음판 12장을 이음매 안쪽에 대고 셸 앞·뒤 조각을 끼운 뒤,\n⑥ M3 × 8 버튼 볼트 24개를 바깥에서 넣어 안쪽 M3 너트로 조인다.',
             moves=[[('TABS', 0)], [('SHFR', 0), ('SHRE', 0)], [('S_TAB', 0)], [('TABNUT', 0)]],
             spin=['S_TAB', 'TABNUT'], view=(0.75, 0.8, 0.6)),
        dict(key='s9', title='9단계  장식판 (순간접착)',
             text='파란 줄무늬와 흰 선 장식판을 셸 위에 순간접착제로 붙인다 (하중 없음).',
             moves=[[('DECO', 0)]], view=(0.85, 0.7, 0.55)),
    ]
    tgt = (0.0, 0.0, ZH + 30.0)
    cuts = [dict(key='cut1', remove=[((0, 0, 0), (-1, 0, 0)), ((0, 0, 0), (0, -1, 0))], view=(1.0, 1.0, 0.65),
                 target=(0.0, 60.0, ZH + 40.0), zoom=0.85, title='단면 ①  앞쪽 왼쪽 1/4을 잘라 낸 몸통',
                 feats={'배터리 24 V 15 Ah': (0.0, 80.0, ZH + 89.0), '배터리 트레이': (0.0, 80.0, ZH + 49.5),
                        '척추 봉 Ø16': (P.TUBE_X, 0.0, ZH + P.TUBE_Z), '위 레일 15 × 15': (P.RAIL_X, 0.0, ZH + P.RAIL_Z),
                        '윗판': (0.0, 60.0, ZH + 146.0), '전장 데크': (0.0, 75.0, ZH - 56.0),
                        'mjpower-ss': (0.0, 35.0, ZH - 45.0), 'hip 판 (6061)': (0.0, 166.5, ZH + 60.0),
                        'hip 모터 AKE60-8': (0.0, 150.0, ZH + 25.0), 'moteus + 컵': (0.0, 133.0, ZH + 15.0),
                        '셸 (PETG)': (0.0, 120.0, ZH + 147.0)}),
            dict(key='cut2', remove=[((0, 0, 0), (-1, 0, 0))], view=(1.0, 0.25, 0.3), target=(0.0, 0.0, ZH + 40.0),
                 zoom=0.85, title='단면 ②  앞쪽 절반을 잘라 낸 몸통',
                 feats={'왼쪽 hip 모터': (0.0, 150.0, ZH + 25.0), '오른쪽 hip 모터': (0.0, -150.0, ZH + 25.0),
                        '배터리': (0.0, 0.0, ZH + 89.0), '트레이': (0.0, -60.0, ZH + 49.5),
                        'Raspberry Pi + pi3hat': (0.0, -55.0, ZH - 45.0), 'mjpower-ss': (0.0, 35.0, ZH - 45.0),
                        '윗판': (0.0, -60.0, ZH + 146.0), '전장 데크': (0.0, 90.0, ZH - 56.0),
                        'hip 판 (오른쪽)': (0.0, -166.5, ZH + 60.0)})]
    return dict(
        job='torso', title='몸통 조립', subtitle='Fusion 모델 · 다리 제외',
        picks=picks, fasteners=F, fixed=['HIPL'],
        recolor={'DECK': ('ax deck blue', (110, 150, 205)), 'TRAY': ('ax tray amber', (235, 175, 90))},
        home=home, steps=steps, cuts=cuts, target=tgt, view0=(0.9, 0.75, 0.6), view_done=(0.85, 0.65, 0.5),
        move=28, long_move=44,
        names=dict(HIPL='왼쪽 hip 유닛', HIPR='오른쪽 hip 유닛', TUBES='척추 봉 Ø16 × 2 (6061)', RAILS='위 레일 15 × 15 × 2',
                   S_M5L='① M5 × 12 접시 × 2', S_M4L='② M4 × 12 접시 × 2', S_M5R='① M5 × 12 접시 × 2',
                   S_M4R='② M4 × 12 접시 × 2', DECK='전장 데크 (PETG)', DINS='황동 인서트 M2.5 × 8',
                   RPI='Raspberry Pi 4B', STAND='M2.5 × 11 스탠드오프 × 4', HAT='pi3hat r4.5', MJ='mjpower-ss + XT90 변환',
                   MJS='M2.5 × 6 저두 × 4', FAN='팬 30 × 30', S_DKL='③ M3 × 8 × 2', S_DKR='③ M3 × 8 × 2',
                   TRAY='배터리 트레이 (PETG)', BATT='배터리 24 V 15 Ah', TOP='윗판 (PETG)', TBOLT='④ M4 × 25 × 8',
                   TNUT='M4 와셔 + 나일론 너트 × 8', SHMID_L='셸 hip 부분 (왼쪽)', SHMID_R='셸 hip 부분 (오른쪽)',
                   S_SHL='⑤ M3 × 6 버튼 × 6', S_SHR='⑤ M3 × 6 버튼 × 6', TABS='셸 이음판 × 12', SHFR='셸 앞 조각',
                   SHRE='셸 뒤 조각', S_TAB='⑥ M3 × 8 버튼 × 24', TABNUT='M3 너트 × 24', DECO='장식판 (줄무늬·선)'),
        captions=dict(DINS='황동 인서트 M2.5 × 8 → 데크 보스', RPI='Raspberry Pi 4B → 데크', STAND='M2.5 × 11 스탠드오프 × 4',
                      HAT='pi3hat r4.5 → 스탠드오프 위', MJ='mjpower-ss + XT90 변환 → 데크', MJS='M2.5 × 6 저두 × 4 (mjpower-ss)',
                      FAN='팬 30 × 30 (양면 폼테이프)', TUBES='척추 봉 × 2 + 위 레일 × 2 → 왼쪽 hip 판',
                      S_M5L='① M5 × 12 + ② M4 × 12 접시 (왼쪽)', DECK='전장 데크 → hip 판 사이',
                      S_DKL='③ M3 × 8 × 2 (왼쪽 데크 플랜지)', HIPR='오른쪽 hip 유닛 → 봉·레일 끝',
                      S_M5R='① M5 × 12 + ② M4 × 12 접시 (오른쪽)', S_DKR='③ M3 × 8 × 2 (오른쪽 데크 플랜지)',
                      TRAY='배터리 트레이 → 척추 봉', BATT='배터리 → 트레이 (벨크로 스트랩 2개)', TOP='윗판 → 위 레일',
                      TBOLT='④ M4 × 25 × 8 (윗판 → 레일 관통)', TNUT='평와셔 + 나일론 너트 M4 × 8 (레일 밑)',
                      SHMID_L='셸 hip 부분 (좌·우) → hip 판', S_SHL='⑤ M3 × 6 버튼 × 12 → hip 판 탭', TABS='셸 이음판 × 12',
                      SHFR='셸 앞·뒤 조각', S_TAB='⑥ M3 × 8 버튼 × 24', TABNUT='M3 너트 × 24 (이음판 안쪽)',
                      DECO='장식판 (순간접착)'),
        short=['전장 데크 (작업대)', '봉·레일 → 왼쪽 hip', '데크 → hip 판', '오른쪽 hip 유닛', '트레이 + 배터리',
               '윗판 + ④ M4 × 25', '셸 hip 부분 + ⑤', '셸 앞·뒤 + ⑥', '장식판'],
        legend=['① M5 × 12 접시 × 4', '② M4 × 12 접시 × 4', '③ M3 × 8 × 4 (데크)', '④ M4 × 25 × 8 + 와셔·나일론 너트',
                '⑤ M3 × 6 버튼 × 12 (셸)', '⑥ M3 × 8 버튼 + 너트 × 24'],
        major=['HIPL', 'HIPR', 'TUBES', 'RAILS', 'DECK', 'TRAY', 'BATT', 'TOP', 'SHMID_L', 'SHMID_R', 'SHFR', 'SHRE'],
        done_labels=['SHMID_L', 'SHFR', 'SHRE', 'TOP', 'DECO'],
        overview_labels=['HIPL', 'HIPR', 'TUBES', 'RAILS', 'DECK', 'RPI', 'MJ', 'TRAY', 'BATT', 'TOP', 'SHMID_L', 'SHMID_R',
                         'SHFR', 'SHRE', 'TABS', 'DECO'],
        overview=('몸통 분해도',
                  '모든 부품이 떨어진 상태 — 왼쪽 hip 유닛을 기준으로 봉·레일은 오른쪽에서, 데크는 아래에서, 트레이·배터리·윗판은 위에서,\n'
                  '셸은 옆·앞·뒤에서 들어온다.  다리는 hip 영상, 배선은 생략.'),
        done=('조립 완료 — 몸통',
              '하중 경로: 윗판 → 레일 → hip 판, 배터리 → 트레이 → 척추 봉 → hip 판, hip 판 → hip 모터 → 다리.\n'
              '셸은 하중을 받지 않는다 (hip 판에 ⑤, 조각끼리는 이음판 ⑥).'),
        note='색은 부품 구분용 (데크·트레이)  ·  배선·커넥터·벨크로 스트랩·pi3hat 고정 나사는 생략  ·  '
             '나사·와셔·너트·인서트는 BOM 규격으로 모델링')
