"""Fasteners and small parts of the v2 robot as geometry (10-30, user: every screw / insert goes into the model and
into the interference check -- the wheel hub's M3 heads had clashed with the hex hub while they were BOM lines only).

Screws, washers, nuts, dowels, heat-set inserts, the pi3hat screws, the battery foam pad and the shell grommets are
masters 'FX ...' in _library (one per BOM row), placed on the link that carries them so they move with the joints.
finalize.bom() counts them (EXPECTED holds the hand-counted quantities they replaced, as a cross-check).

Thread convention (so the interference check shows real clashes only): a screw is drawn at its minor diameter
(= tap drill, MINOR) over its whole length, inserts and nut bores at that diameter, inserts at their hole diameter.
A screw in a tapped hole, insert or nut and an insert in its hole therefore only touch, and touching faces are left
out of check.interference(); any interference volume is a clash.  Heads, washers and nuts have their real size.
Not modelled: the 2 velcro battery straps (soft, wrapped).

Each master is built in a local frame: +z = the direction the part travels when fitted, origin = head bearing face
(socket / low / button heads), top face (countersunk), contact face (washer, nut, insert, pin, pad, grommet).
"""
import math

import cadlib as C
import layout as LY
import legs as LG
import params as P
import shell as SH
import torso as TO

MINOR = {2.5: 2.05, 3: 2.5, 4: 3.3, 5: 4.2}
SHCS = {2.5: (4.5, 2.5, 2.0, 1.1), 3: (5.5, 3.0, 2.5, 1.3), 4: (7.0, 4.0, 3.0, 2.0), 5: (8.5, 5.0, 4.0, 2.5)}  # ISO 4762
LOW = {2.5: (4.5, 1.5, 1.5, 0.9)}                                    # DIN 7984 dk, k, s, t
BUTTON = {3: (5.7, 1.65, 2.0, 1.0)}                                  # ISO 7380
CSK = {4: (8.96, 2.5, 1.6), 5: (11.2, 3.0, 2.0)}                     # ISO 10642 dk, s, t
WASHER = {3: (7.0, 3.2, 0.5), 4: (9.0, 4.3, 0.8)}                    # DIN 125 od, id, t
NUT = {3: (5.5, 2.4), 4: (7.0, 5.0)}                                 # DIN 934 / DIN 985 s, m
PI3HAT_T = 1.5                                                       # pi3hat PCB (vendor: STEP y 0..1.5)
B = C.Builder(robot=False)                                           # local frames, mm


# ----------------------------------------------------------------------------- master geometry (local frame)
def _hexp(af, z0, z1):
    r = af / 2 / math.cos(math.radians(30))
    poly = [(r * math.cos(math.radians(30 + 60 * k)), r * math.sin(math.radians(30 + 60 * k))) for k in range(6)]
    return B.prism(poly, z0, z1, plane='xy')


def _ring(z0, z1, ro, ri):
    b = B.zcyl(0, 0, z0, z1, ro)
    if ri > 0:
        C.cut(b, B.zcyl(0, 0, z0 - 1, z1 + 1, ri))
    return b


def socket_screw(d, L, head='shcs'):
    dk, k, s, t = {'shcs': SHCS, 'low': LOW, 'button': BUTTON}[head][d]
    if head == 'button':
        body = B.zcyl(0, 0, -0.45 * k, 0.0, dk / 2)
        C.union(body, B.cyl((0, 0, -k), (0, 0, -0.45 * k + 0.01), dk * 0.3, dk / 2))
    else:
        body = B.zcyl(0, 0, -k, 0.0, dk / 2)
    C.union(body, B.zcyl(0, 0, -0.01, L, MINOR[d] / 2))
    C.cut(body, _hexp(s, -k - 0.2, -k + t))
    return body


def csk_screw(d, L):
    dk, s, t = CSK[d]
    h = (dk - d) / 2
    body = B.cyl((0, 0, 0), (0, 0, h), dk / 2, d / 2)
    C.union(body, B.zcyl(0, 0, h - 0.01, L, MINOR[d] / 2))
    C.cut(body, _hexp(s, -0.2, t))
    return body


def washer(d):
    od, idd, t = WASHER[d]
    return _ring(0.0, t, od / 2, idd / 2)


def nut(d, nyloc=False):
    s, m = NUT[d]
    if nyloc:                                                  # nylon dome first: the nut travels dome-first onto the bolt
        body = _hexp(s, 0.3 * m, m)
        C.union(body, B.zcyl(0, 0, 0.0, 0.3 * m + 0.01, s * 0.46))
    else:
        body = _hexp(s, 0.0, m)
    C.cut(body, B.zcyl(0, 0, -1, m + 1, MINOR[d] / 2))
    return body


def insert(d, L, od):
    body = _ring(0.0, L, od / 2, MINOR[d] / 2)
    for zc in (L * 0.3, L * 0.7):                              # knurl bands
        C.cut(body, _ring(zc - 0.3, zc + 0.3, od, od / 2 - 0.2))
    return body


def dowel(d, L):
    body = B.zcyl(0, 0, 0.3, L - 0.3, d / 2)
    C.union(body, B.cyl((0, 0, 0), (0, 0, 0.31), d / 2 - 0.3, d / 2))
    C.union(body, B.cyl((0, 0, L - 0.31), (0, 0, L), d / 2, d / 2 - 0.3))
    return body


def foam_pad():
    bx, by, _bz = P.BATTERY
    return B.abox(-bx / 2, bx / 2, -by / 2, by / 2, 0.0, 1.0)


def grommet():
    """Rubber grommet in the Ø16 shell hole (1.4 mm wall): flanges Ø20, Ø14 bore (the hip cable bundle passes
    1.5 - 2 mm below the hole centre and needs r 6.5).  z = 0 at the shell outer face, +z outwards."""
    t = P.SHELL_T
    body = B.zcyl(0, 0, 0.0, 1.2, 10.0)
    C.union(body, B.zcyl(0, 0, -t - 0.01, 0.01, 8.0))
    C.union(body, B.zcyl(0, 0, -t - 1.2, -t, 10.0))
    C.cut(body, B.zcyl(0, 0, -t - 2, 2, 7.0))
    return body


# ----------------------------------------------------------------------------- masters and BOM rows
def _cup_len(joint):
    return LG.cup_screw(joint)[0]


def specs():
    """name -> (geometry builder, material, look, (BOM group, item, spec, where), expected count)."""
    Lh, Lk, Lw = _cup_len('hip'), _cup_len('knee'), _cup_len('wheel')
    cb = {j: LG.cup_screw(j)[1] for j in ('hip', 'knee', 'wheel')}
    sc = lambda d, L, head='shcs': (lambda: socket_screw(d, L, head))
    return {
        'FX ISO 4762 M2.5x6 12.9 hip': (sc(2.5, 6), 'Steel', 'black satin', (
            '관절', '육각 홈붙이 볼트 ISO 4762 12.9', 'M2.5 × 6', 'hip 고정링 → 6061 hip 판(나사산 3.2 mm, AKE 링 탭 깊이 5)'), 12),
        'FX DIN 7984 M2.5x8 stator': (sc(2.5, 8, 'low'), 'Steel', 'steel', (
            '관절', '저두 볼트 DIN 7984 8.8', 'M2.5 × 8', 'knee·wheel 고정링 → PETG 판, SUS 클램프 링 위'), 24),
        'FX ISO 4762 M3x16 12.9': (sc(3, 16), 'Steel', 'black satin', (
            '관절', '육각 홈붙이 볼트 ISO 4762 12.9', 'M3 × 16', '모터 출력 → hip·knee 구동 보스(SUS 출력 링 + 와셔 위, 나사산 4.3 mm)'), 24),
        'FX ISO 4762 M3x%d 12.9 hub' % P.HUB_SCREW_L: (sc(3, P.HUB_SCREW_L), 'Steel', 'black satin', (
            '바퀴', '육각 홈붙이 볼트 ISO 4762 12.9', 'M3 × %d' % P.HUB_SCREW_L,
            '바퀴 모터 출력 → 허브 플랜지(와셔 위, 나사산 %.1f mm, 머리는 육각 허브 Ø20 턱에서 0.5 mm)'
            % (P.HUB_SCREW_L - P.HUB_SCREW_SEAT - WASHER[3][2])), 12),
        'FX DIN 125 M3': (lambda: washer(3), 'Steel', 'steel', (
            '관절', '평와셔 DIN 125 A2', 'M3', '출력 M3 × 16·M3 × %d 머리 밑' % P.HUB_SCREW_L), 36),
        'FX ISO 8734 3x10': (lambda: dowel(3.0, 10.0), 'Steel', 'steel', (
            '관절', '평행 핀 ISO 8734 m6 경화', 'Ø3 × 10', '모터 출력 다웰(출력 3 mm + PETG 7 mm)'), 18),
        'FX ISO 4762 M3x8 hex hub': (sc(3, 8), 'Steel', 'steel', (
            '바퀴', '육각 홈붙이 볼트 ISO 4762 8.8', 'M3 × 8', '육각 허브 → 허브 플랜지'), 12),
        'FX insert M3x5.7 hub': (lambda: insert(3, 5.7, LG.INSERT_M3), 'Brass', 'brass', (
            '바퀴', '열압입 인서트 황동', 'M3 × 5.7 (구멍 Ø4.0 × 6)', '허브 플랜지'), 12),
        'FX ISO 4762 M5x55 axle': (sc(5, 55), 'Steel', 'steel', (                 # 10-31: 50 -> 55, full nut
            '바퀴', '육각 홈붙이 볼트 ISO 4762 8.8', 'M5 × 55', '바퀴 축 볼트(머리는 플랜지 포켓 안, 육각 허브를 누름)'), 2),
        'FX DIN 7984 M2.5x6 board': (sc(2.5, 6, 'low'), 'Steel', 'steel', (
            '드라이버', '저두 볼트 DIN 7984', 'M2.5 × 6', 'moteus → 컵 스탠드오프(보드 모터 쪽에서)'), 24),
        'FX insert M2.5x4 cup': (lambda: insert(2.5, 4.0, LG.INSERT_M25), 'Brass', 'brass', (
            '드라이버', '열압입 인서트 황동', 'M2.5 × 4 (구멍 Ø3.6 × 5)', '컵 스탠드오프'), 24),
        'FX ISO 4762 M3x%d hip cup' % Lh: (sc(3, Lh), 'Steel', 'steel', (
            '드라이버', '육각 홈붙이 볼트 ISO 4762', 'M3 × %d' % Lh,
            'hip 컵 → hip 판 M3 탭(컵 바닥 카운터보어 %.1f mm, 나사산 4 mm)' % cb['hip']), 6),
        'FX ISO 4762 M2.5x%d knee cup' % Lk: (sc(2.5, Lk), 'Steel', 'steel', (
            '드라이버', '육각 홈붙이 볼트 ISO 4762', 'M2.5 × %d' % Lk, 'knee 컵 → 허벅지 인서트(카운터보어 %.1f mm)' % cb['knee']), 6),
        'FX ISO 4762 M2.5x%d wheel cup' % Lw: (sc(2.5, Lw), 'Steel', 'steel', (
            '드라이버', '육각 홈붙이 볼트 ISO 4762', 'M2.5 × %d' % Lw, 'wheel 컵 → 정강이 인서트(카운터보어 %.1f mm)' % cb['wheel']), 6),
        'FX insert M2.5x4 link cup': (lambda: insert(2.5, 4.0, LG.INSERT_M25), 'Brass', 'brass', (
            '드라이버', '열압입 인서트 황동', 'M2.5 × 4 (구멍 Ø3.6 × 5)', '링크의 컵 기둥 자리'), 12),
        'FX DIN 7984 M2.5x6 clip': (sc(2.5, 6, 'low'), 'Steel', 'steel', (
            '배선', '저두 볼트 DIN 7984', 'M2.5 × 6', '케이블 클립 → 링크 홈 옆 인서트'), 24),
        'FX insert M2.5x4 clip': (lambda: insert(2.5, 4.0, LG.INSERT_M25), 'Brass', 'brass', (
            '배선', '열압입 인서트 황동', 'M2.5 × 4 (구멍 Ø3.6 × 5)', '링크 배선 홈 옆'), 24),
        'FX grommet D16': (grommet, 'Rubber', 'rubber', ('배선', '고무 그로밋', 'Ø16 구멍, 판 1.4 mm (안지름 14)',
                                                       '셸 hip 케이블 출구'), 2),
        'FX ISO 10642 M5x12': (lambda: csk_screw(5, 12), 'Steel', 'black satin', (
            '몸통', '접시 볼트 ISO 10642', 'M5 × 12', 'hip 판 → 척추 봉 끝 M5 탭(물림 약 7.5 mm)'), 4),
        'FX ISO 10642 M4x12': (lambda: csk_screw(4, 12), 'Steel', 'black satin', (
            '몸통', '접시 볼트 ISO 10642', 'M4 × 12', 'hip 판 → 위 레일 끝 M4 탭(물림 약 7.5 mm)'), 4),
        'FX ISO 4762 M4x25 top': (sc(4, 25), 'Steel', 'black satin', (
            '몸통', '육각 홈붙이 볼트 ISO 4762', 'M4 × 25', '윗판 → 위 레일(관통)'), 8),
        'FX DIN 985 M4': (lambda: nut(4, nyloc=True), 'Steel', 'steel', ('몸통', '나일론 너트 DIN 985', 'M4', '위 레일 아래'), 8),
        'FX DIN 125 M4': (lambda: washer(4), 'Steel', 'steel', ('몸통', '평와셔 DIN 125 A2', 'M4', '위 레일 아래 나일론 너트 밑'), 8),
        'FX insert M4x5.7 top': (lambda: insert(4, 5.7, 5.6), 'Brass', 'brass', (
            '몸통', '열압입 인서트 황동', 'M4 × 5.7 (구멍 Ø5.6 × 6)', '윗판 25 mm 격자(짐·로봇팔)'), _top_grid_count()),
        'FX ISO 4762 M3x8 deck': (sc(3, 8), 'Steel', 'steel', (
            '몸통', '육각 홈붙이 볼트 ISO 4762', 'M3 × 8', '전장 데크 플랜지 → hip 판 M3 탭'), 4),
        'FX ISO 7380 M3x6 shell': (sc(3, 6, 'button'), 'Steel', 'black satin', (
            '몸통', '버튼 볼트 ISO 7380', 'M3 × 6', '셸(hip 부분) → hip 판 M3 탭'), 12),
        'FX ISO 7380 M3x8 splice': (sc(3, 8, 'button'), 'Steel', 'black satin', (
            '몸통', '버튼 볼트 ISO 7380', 'M3 × 8', '셸 앞·뒤 조각 ↔ hip 조각 이음판 12개'), 24),
        'FX DIN 934 M3 splice': (lambda: nut(3), 'Steel', 'steel', ('몸통', '육각 너트 DIN 934', 'M3', '이음판 안쪽'), 24),
        'FX DIN 7984 M2.5x6 mjpower': (sc(2.5, 6, 'low'), 'Steel', 'steel', (
            '몸통', '저두 볼트 DIN 7984', 'M2.5 × 6', 'mjpower-ss → 데크 보스'), 4),
        'FX insert M2.5x4 deck': (lambda: insert(2.5, 4.0, 3.6), 'Brass', 'brass', (
            '몸통', '열압입 인서트 황동', 'M2.5 × 4 (구멍 Ø3.6 × 5)', '데크 보스(라즈베리파이 4, mjpower-ss 4)'), 8),
        'FX pi3hat M2.5x5': (sc(2.5, 5, 'low'), 'Steel', 'steel', (
            '몸통', 'M2.5 나사 (pi3hat 동봉)', 'M2.5 × 5', 'pi3hat → 스탠드오프 위'), 4),
        'FX foam pad': (foam_pad, 'Foam', 'dark grey', ('몸통', '폼 패드', '75 × 310 × 1 mm', '배터리 밑'), 1),
    }


NOT_MODELLED = [('몸통', '벨크로 스트랩', '폭 25 mm × 400 mm', 2, '배터리 + 트레이를 척추 봉에 묶음(트레이 슬롯) — 모델 없음')]


def _top_grid():
    lx, ly, _t = P.TOP_PLATE
    out = []
    for i in range(-2, 3):
        for j in range(-4, 5):
            x, y = 25.0 * i, 25.0 * j
            if abs(x) > lx / 2 - 10 or abs(y) > ly / 2 - 10:
                continue
            if any(abs(x - sx * P.RAIL_X) < 8 and abs(y - yy) < 8 for sx in (-1, 1) for yy in (-100.0, -33.3, 33.3, 100.0)):
                continue
            out.append((x, y))
    return out


def _top_grid_count():
    return len(_top_grid())


# ----------------------------------------------------------------------------- positions
def positions():
    """[(link, master, origin, ez[, ex])] in robot coordinates."""
    import assembly as AS
    A = P.AKE
    out = []
    for s, k in ((1, 'L'), (-1, 'R')):
        Y = lambda y: s * y
        D = lambda sg: (0.0, s * sg, 0.0)
        # wheel joint: stator side on the shank, output side on the wheel
        m = LY.frames('wheel', s)['motor']
        y_out = P.y_output('wheel')
        rec = P.Y_TIBIA_IN + P.RING_T + P.CBORE_M25[1]
        for q in LY.motor_points(m, P.AKE_PCD_RING / 2, P.AKE_RING_M25, A['ring']):
            out.append(('tibia_' + k, 'FX DIN 7984 M2.5x8 stator', (q[0], Y(rec - P.RING_T), q[2]), D(+1)))
        bear = y_out - P.HUB_SCREW_SEAT - WASHER[3][2]
        for q in LY.motor_points(m, P.AKE_PCD_OUT / 2, P.AKE_OUT_M3, A['out']):
            out.append(('wheel_' + k, 'FX ISO 4762 M3x%d 12.9 hub' % P.HUB_SCREW_L, (q[0], Y(bear), q[2]), D(+1)))
            out.append(('wheel_' + k, 'FX DIN 125 M3', (q[0], Y(bear), q[2]), D(+1)))
        for q in LY.motor_points(m, P.AKE_PCD_OUT / 2, P.AKE_OUT_PIN, A['out']):
            out.append(('wheel_' + k, 'FX ISO 8734 3x10', (q[0], Y(y_out - 7.0), q[2]), D(+1)))
        lv = LG.hub_levels()
        for a in LG.HUB_SCREWS:
            x, z = 18 * math.cos(a), P.R_WHEEL + 18 * math.sin(a)
            out.append(('wheel_' + k, 'FX insert M3x5.7 hub', (x, Y(lv['f0'] + 0.1), z), D(+1)))
            out.append(('wheel_' + k, 'FX ISO 4762 M3x8 hex hub', (x, Y(lv['h0'] + 3.5), z), D(+1)))
        out.append(('wheel_' + k, 'FX ISO 4762 M5x55 axle', (0.0, Y(lv['f0'] + 1.5), P.R_WHEEL), D(-1)))
        # knee: stator on the thigh plate (screws from its outer face), output on the shank
        m = LY.frames('knee', s)['motor']
        for q in LY.motor_points(m, P.AKE_PCD_RING / 2, P.AKE_RING_M25, A['ring']):
            out.append(('femur_' + k, 'FX DIN 7984 M2.5x8 stator', (q[0], Y(P.Y_FEMUR_OUT - P.CBORE_M25[1]), q[2]), D(-1)))
        # hip: stator on the hip plate (12.9 from its outer face), output on the thigh
        mh = LY.frames('hip', s)['motor']
        for q in LY.motor_points(mh, P.AKE_PCD_RING / 2, P.AKE_RING_M25, A['ring']):
            out.append(('torso', 'FX ISO 4762 M2.5x6 12.9 hip', (q[0], Y(P.YW - P.CBORE_M25_HIP[1]), q[2]), D(-1)))
        for joint, link, mm in (('knee', 'tibia_' + k, m), ('hip', 'femur_' + k, mh)):
            yo = P.y_output(joint)
            bear = yo + P.BEARING_FACE + WASHER[3][2]
            for q in LY.motor_points(mm, P.AKE_PCD_OUT / 2, P.AKE_OUT_M3, A['out']):
                out.append((link, 'FX ISO 4762 M3x16 12.9', (q[0], Y(bear), q[2]), D(-1)))
                out.append((link, 'FX DIN 125 M3', (q[0], Y(bear), q[2]), D(-1)))
            for q in LY.motor_points(mm, P.AKE_PCD_OUT / 2, P.AKE_OUT_PIN, A['out']):
                out.append((link, 'FX ISO 8734 3x10', (q[0], Y(yo + 7.0), q[2]), D(-1)))
        # driver cups: board screws + standoff inserts, column screws, the link inserts under the columns
        for joint, link in (('hip', 'torso'), ('knee', 'femur_' + k), ('wheel', 'tibia_' + k)):
            b = LY.frames(joint, s)['board']
            yb = C.col(b, 1)
            nb = C.mul(yb, -1)
            for (x, z) in P.MOTEUS['holes']:
                out.append((link, 'FX insert M2.5x4 cup', C.apply(b, (x, P.MOTEUS['pcb_t'] + 0.1, z)), yb))
                out.append((link, 'FX DIN 7984 M2.5x6 board', C.apply(b, (x, 0.0, z)), yb))
            rim, f_out = LG.cup_levels(joint, s)
            L, cb = LG.cup_screw(joint)
            name = {'hip': 'FX ISO 4762 M3x%d hip cup', 'knee': 'FX ISO 4762 M2.5x%d knee cup',
                    'wheel': 'FX ISO 4762 M2.5x%d wheel cup'}[joint] % L
            for t in LG.CUP_COLUMNS:
                tt = math.radians(t)
                xb, zb = 40.5 * math.cos(tt) * s, 40.5 * math.sin(tt)
                out.append((link, name, C.apply(b, (xb, f_out - cb, zb)), nb))
            if joint != 'hip':
                yf, sg = (P.Y_FEMUR_IN + 0.1, +1) if joint == 'knee' else (P.Y_TIBIA_OUT - 0.1, -1)
                for p in LG.cup_column_points(joint, s):
                    out.append((link, 'FX insert M2.5x4 link cup', (p[0], Y(yf), p[2]), D(sg)))
        # cable clips: 3 per link edge groove, 2 screws each into inserts beside the groove
        for (a, bb, half, edge, s01, ym), link in zip(AS._grooves(s), ('femur_' + k, 'tibia_' + k)):
            L = math.hypot(bb[0] - a[0], bb[1] - a[1])
            u = ((bb[0] - a[0]) / L, (bb[1] - a[1]) / L)
            n = LG.edge_normal(a, bb, edge)
            e = (-n[0], 0.0, -n[1])
            for t in AS.CLIP_T:
                st = s01[0] + (s01[1] - s01[0]) * t
                for dy in (-9.6, 9.6):
                    px, pz = a[0] + u[0] * st + n[0] * half, a[1] + u[1] * st + n[1] * half
                    out.append((link, 'FX insert M2.5x4 clip', (px - n[0] * 0.1, Y(ym + dy), pz - n[1] * 0.1), e))
                    out.append((link, 'FX DIN 7984 M2.5x6 clip', (px + n[0] * 2.5, Y(ym + dy), pz + n[1] * 2.5), e))
        # torso frame, deck, shell (hip sections)
        ZH = P.Z_HIP
        y_in = P.YW - P.HIP_PLATE_T
        y_web = y_in + P.HIP_WEB_T
        for sx in (-1, 1):
            out.append(('torso', 'FX ISO 10642 M5x12', (sx * P.TUBE_X, Y(y_web), ZH + P.TUBE_Z), D(-1)))
            out.append(('torso', 'FX ISO 10642 M4x12', (sx * P.RAIL_X, Y(y_web), ZH + P.RAIL_Z), D(-1)))
            out.append(('torso', 'FX ISO 4762 M3x8 deck', (sx * 45.0, Y(y_in - 3.5), ZH - 42.0), D(+1)))
        for (x, z) in ((-50.0, 30.0), (50.0, 30.0), (-60.0, -20.0), (60.0, -20.0), (-40.0, 140.0), (40.0, 140.0)):
            out.append(('torso', 'FX ISO 7380 M3x6 shell', (x, Y(P.YW), ZH + z), D(-1)))
        g = P.theta_dir(P.GROMMET_DEG)
        out.append(('torso', 'FX grommet D16', (g[0] * P.GROMMET_R, Y(P.YW), ZH + g[1] * P.GROMMET_R), D(+1)))
    ZH = P.Z_HIP
    z1 = ZH + P.DECK_Z
    for (x, y) in TO.rpi_holes_robot() + TO.mjpower_holes_robot():
        out.append(('torso', 'FX insert M2.5x4 deck', (x, y, z1 + 4.9), (0, 0, -1)))
    for (x, y) in TO.mjpower_holes_robot():
        out.append(('torso', 'FX DIN 7984 M2.5x6 mjpower', (x, y, z1 + 5.0 + 1.6), (0, 0, -1)))
    pm = TO.rpi_frame()
    for (x, y) in ((3.5, 3.5), (61.5, 3.5), (3.5, 52.5), (61.5, 52.5)):
        out.append(('torso', 'FX pi3hat M2.5x5', C.apply(pm, (x, y, 1.4 + 11.0 + PI3HAT_T)), (0, 0, -1)))
    z0 = ZH + P.RAIL_Z + P.RAIL_S / 2
    t_top = P.TOP_PLATE[2]
    for sx in (-1, 1):
        for yy in (-100.0, -33.3, 33.3, 100.0):
            out.append(('torso', 'FX ISO 4762 M4x25 top', (sx * P.RAIL_X, yy, z0 + t_top - 4.0), (0, 0, -1)))
            out.append(('torso', 'FX DIN 125 M4', (sx * P.RAIL_X, yy, z0 - P.RAIL_S - WASHER[4][2]), (0, 0, 1)))
            out.append(('torso', 'FX DIN 985 M4', (sx * P.RAIL_X, yy, z0 - P.RAIL_S - WASHER[4][2] - NUT[4][1]), (0, 0, 1)))
    for (x, y) in _top_grid():
        out.append(('torso', 'FX insert M4x5.7 top', (x, y, z0 + t_top - 0.1), (0, 0, -1)))
    for (sx, sy, where) in SH.SPLICE:
        q, d, n = SH._seam_surface(sx, sy, where)
        for dx in (-6.0, 6.0):
            a = C.add(q, C.mul(d, dx))
            out.append(('torso', 'FX ISO 7380 M3x8 splice', a, C.mul(n, -1)))
            out.append(('torso', 'FX DIN 934 M3 splice', C.sub(a, C.mul(n, P.SHELL_T + 2.5 + NUT[3][1])), n))
    out.append(('torso', 'FX foam pad', (P.BATTERY_X, P.BATTERY_Y, ZH + P.BATTERY_Z0 - 1.0), (0, 0, 1), (1, 0, 0)))
    return out


def _frame(origin, ez, ex=None):
    ez = C.unit(ez)
    if ex is None:
        a = (1.0, 0.0, 0.0) if abs(ez[0]) < 0.9 else (0.0, 1.0, 0.0)
        ex = C.cross(a, ez)
    ex = C.unit(ex)
    return C.frame(origin, ex, C.cross(ez, ex), ez)


def build_all():
    """Masters + instances.  Run after the links, shell and harness exist (v2run.build)."""
    import assembly as AS
    sp = specs()
    masters = {}
    for name, (geo, mat, look, _bom, _n) in sp.items():
        comp = C.master(name)
        C.add_body(comp, geo(), name, mat, look)
        masters[name] = comp
    L = AS.links()
    n = {}
    for rec in positions():
        link, name, o, ez = rec[:4]
        ex = rec[4] if len(rec) > 4 else None
        C.instance(L[link], masters[name], C.part_matrix(_frame(o, ez, ex)))
        n[name] = n.get(name, 0) + 1
    bad = {k: (n.get(k, 0), v[4]) for k, v in sp.items() if n.get(k, 0) != v[4]}
    return n, bad
