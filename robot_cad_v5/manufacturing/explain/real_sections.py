"""Real 2D joint sections of WheelBiped v5 (left leg, nominal pose): hip, knee and wheel, cut from the Fusion model
(hub WheelBiped_v5) by fusion/v5run.sections() -> sections_v5.json.gz.  Every visible body near a joint axis is cut by
planes through that axis; each figure shows two half planes (upper half at BETA_UP, lower half at BETA_DN, angles from
robot +x towards +z), so the colours are the real modelled parts - nothing here is redrawn from parameters except the
fasteners that are not in the model yet (purple, drawn with their standard length in the modelled holes).
Same layout and colours as the earlier explain figures (overview, enlarged detail, fastener rows, fits, assembly order,
loads), plus the result of a clearance check over every 5 deg half plane: overlaps between any two bodies and the
smallest gap between parts that move against each other (torso / thigh / shank / wheel).

    python real_sections.py            # -> hip_section.png, knee_section.png, wheel_section.png
"""
import gzip
import itertools
import json
import math
import sys
from collections import defaultdict
from functools import reduce

from matplotlib.patches import PathPatch
from matplotlib.path import Path
from shapely import set_precision
from shapely.affinity import scale
from shapely.geometry import GeometryCollection, LineString, MultiLineString, MultiPolygon, Polygon, box
from shapely.ops import linemerge

from stack_common import (CSK3_DK, F_CHK, F_DES, FL_CHK, FL_DES, HATCH, HATCH_AL, HERE, LG, LV, NEW, P, ROBOT_KG,
                          Rectangle, csk_screw, dim_h, fits_block, g, label, plt, rows_block, tag)

sys.path.insert(0, str(HERE.parents[1] / 'calc'))
import strength_v5 as SV                         # noqa: E402  (loads, moments and bolt circle of the hand calculation)

SRC = HERE / 'sections_v5.json.gz'
GROUPS = {'torso': '몸통', 'femur_L': '허벅지 쪽', 'tibia_L': '정강이 쪽', 'wheel_L': '바퀴 쪽'}


# ----------------------------------------------------------------------------- section geometry
def material(lines, tol=2e-3):
    """Chain the plane-intersection edges into rings; material = XOR of the closed rings (nesting gives holes), or the
    open lines of a surface body that does not close."""
    ls = [LineString(p) for p in lines if len(p) >= 2]
    if not ls:
        return None
    m = linemerge(set_precision(MultiLineString(ls), tol))
    rings, open_ = [], []
    for gm in (list(m.geoms) if hasattr(m, 'geoms') else [m]):
        if gm.is_closed and len(gm.coords) >= 4:
            pg = Polygon(gm.coords)
            rings.append(pg if pg.is_valid else pg.buffer(0))
        elif gm.length > 1e-3:
            open_.append(gm)
    if rings:
        mat = reduce(lambda a, b: a.symmetric_difference(b), rings)
        if not mat.is_empty and mat.area > 1e-3:
            return mat
    return MultiLineString([list(x.coords) for x in open_]) if open_ else None


def load():
    with gzip.open(SRC, 'rt', encoding='utf-8') as fh:
        recs = json.load(fh)
    out = []
    for r in recs:
        m = material(r['lines'])
        if m is not None:
            out.append(dict(joint=r['joint'], angle=r['angle'], comp=r['comp'], body=r['body'], path=r['path'],
                            group=r['path'].split('+')[0].split(':')[0], solid=r['solid'], geom=m))
    return out


def half(recs, joint, beta, flip=False, rmax=250.0):
    """Section in the half plane at direction beta (deg) from the joint axis: (robot y, distance r >= 0), r negated
    when flip (the lower half of a figure)."""
    beta %= 360
    alpha, sign = (beta, 1.0) if beta < 180 else (beta - 180, -1.0)
    clip = box(-1e3, 0.0, 1e3, rmax) if sign > 0 else box(-1e3, -rmax, 1e3, 0.0)
    out = []
    for r in recs:
        if r['joint'] != joint or r['angle'] != alpha:
            continue
        gm = r['geom'].intersection(clip)
        if gm.is_empty or (gm.area < 1e-4 and gm.length < 1e-3):
            continue
        if sign < 0:
            gm = scale(gm, 1.0, -1.0, origin=(0, 0))
        if flip:
            gm = scale(gm, 1.0, -1.0, origin=(0, 0))
        out.append(dict(r, geom=gm))
    return out


def check(recs, joint, gap=1.6):
    """Over every 5 deg half plane: overlapping bodies (area > 0.02 mm2; the AKE60's own bodies left out) and the
    smallest gap below `gap` between bodies of different moving groups."""
    gaps, laps = {}, {}
    for beta in range(0, 360, 5):
        H = half(recs, joint, beta)
        for a, b in itertools.combinations(H, 2):
            if (a['path'], a['body']) == (b['path'], b['body']) or ('AKE60' in a['comp'] and 'AKE60' in b['comp']):
                continue
            d = a['geom'].distance(b['geom'])
            if d > gap:
                continue
            ka, kb = sorted([(a['group'], name(a)), (b['group'], name(b))])
            ar = a['geom'].intersection(b['geom']).area
            if ar > 0.02:
                laps[(ka, kb)] = max(laps.get((ka, kb), 0.0), ar)
            elif a['group'] != b['group'] and d < gaps.get((ka, kb), (9e9,))[0]:
                gaps[(ka, kb)] = (d, beta)
    return gaps, laps


# ----------------------------------------------------------------------------- colours / names
STYLE = [   # (key in 'component body', colour, hatch, name)
    ('thigh_L main', g['thigh'], None, 'PETG 허벅지'), ('shank_L main', g['shank'], None, 'PETG 정강이'),
    ('_L insert', g['al'], HATCH_AL, '6061 hub 인서트'), ('_L ring', '#7fa7d6', HATCH_AL, '6061 받침 링'),
    ('kneecap', g['cap'], HATCH_AL, '6061 무릎 캡'), ('kneebearing', g['bearing'], None, '61807'),
    ('AKE60-8_KV80_stator', g['stator'], None, 'AKE60-8 고정자'), ('AKE60-8_KV80_output', g['out'], None, 'AKE 출력부'),
    ('FX DIN 125', g['washer'], None, '와셔'), ('FX ISO 8734', g['pin'], None, '평행핀'),
    ('FX insert', g['insert'], None, '황동 열압입 인서트'), ('FX ', g['screw'], None, '나사'),
    ('clamp ring', g['ring'], None, 'SUS 고정 링'), ('cup_', g['cup'], None, '드라이버 컵 (PETG)'),
    ('moteus', g['pcb'], None, 'moteus r4.11'), ('magnet', g['spacer'], None, '자석 스페이서'),
    ('hub_flange', g['flange'], None, 'PETG 허브 플랜지'), ('rim_', g['rim'], None, 'PETG 림'),
    ('tyre', g['tyre'], None, '타이어 M365'), ('hip_plate', g['plate'], HATCH, '6061 hip 판'),
    ('XT30', '#c8a000', None, 'XT30 캡'), ('CAN', '#c8a000', None, 'CAN 종단'),
]
OTHER = ('#ececec', None, '몸통 부품')


def style(r):
    key = r['comp'] + ' ' + r['body']
    for k, c, h, n in STYLE:
        if k in key:
            return c, h, n
    return OTHER


def name(r):
    return style(r)[2] if not r['comp'].startswith('FX ') else r['comp'][3:]


def polys(gm):
    if isinstance(gm, Polygon):
        return [gm]
    if isinstance(gm, (MultiPolygon, GeometryCollection)):
        return [x for x in gm.geoms if isinstance(x, Polygon)]
    return []


def lines(gm):
    if isinstance(gm, LineString):
        return [gm]
    if isinstance(gm, (MultiLineString, GeometryCollection)):
        return [x for x in gm.geoms if isinstance(x, LineString)]
    return []


def draw(ax, H, lw=0.6):
    for r in H:
        c, h, _n = style(r)
        z = 1.0 if c in (g['thigh'], g['shank'], OTHER[0], g['plate'], g['tyre']) else 2.0
        for pg in polys(r['geom']):
            verts, codes = [], []
            for ring in [pg.exterior] + list(pg.interiors):
                xy = list(ring.coords)
                verts += xy
                codes += [Path.MOVETO] + [Path.LINETO] * (len(xy) - 2) + [Path.CLOSEPOLY]
            ax.add_patch(PathPatch(Path(verts, codes), fc=c, ec='k', lw=lw, zorder=z, **(h or {})))
        for ln in lines(r['geom']):
            x, y = ln.xy
            ax.plot(x, y, color='#6b6b6b', lw=0.8, zorder=2.5)


def where(H, key, pick=None):
    """A point inside the first section whose 'component body' contains key (pick: 'max' / 'min' radius, or the
    representative point)."""
    for r in H:
        if key in r['comp'] + ' ' + r['body']:
            ps = polys(r['geom'])
            if ps:
                pg = max(ps, key=lambda q: q.area)
                if pick is None:
                    p = pg.representative_point()
                    return p.x, p.y
                b = pg.bounds
                return ((b[0] + b[2]) / 2, b[3] if pick == 'max' else b[1])
    return None


def panel(fig, rect_, H, view, title, ylabel=True):
    ax = fig.add_axes(rect_)
    ax.set_aspect('equal')
    draw(ax, H)
    (x0, x1), (r0, r1) = view
    ax.set_xlim(x0, x1)
    ax.set_ylim(r0, r1)
    ax.axhline(0, color='k', lw=0.7, ls='-.', zorder=5)
    ax.set_title(title, fontsize=15, loc='left')
    ax.tick_params(labelsize=10)
    ax.set_xlabel('로봇 y (왼쪽 다리, 바깥 +) [mm]', fontsize=11)
    if ylabel:
        ax.set_ylabel('축에서 거리 [mm]  (위 절반 +, 아래 절반 -)', fontsize=11)
    ax.grid(True, lw=0.25, alpha=0.5)
    return ax


def flange_bolt(ax, y_link_in, s=-1):
    """⑥ ISO 10642 M3 x 16 12.9 (not modelled): head flush in the insert flange's motor-side face, through the PETG
    wall into the 6061 backing ring's tap, r FLANGE_BOLT_R."""
    csk_screw(ax, y_link_in, LV.y_ring(y_link_in) + LV.RING[2], LV.FLANGE_BOLT_R, s=s, z=4)


def text_column(fig, rect_, rows, fits, steps, notes):
    ax = fig.add_axes(rect_)
    ax.axis('off')
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.text(0.0, 0.995, '나사·핀·베어링 (관절 한쪽 기준)', fontsize=16, weight='bold', va='top')
    y = rows_block(ax, rows, 0.955, 0.052, head_fs=13, body_fs=11.5, gap=0.02)
    ax.text(0.0, y + 0.01, '끼워맞춤 · 틈', fontsize=16, weight='bold', va='top')
    y = fits_block(ax, fits, y - 0.035, step=0.026, fs=11.5)
    ax.text(0.0, y + 0.005, '조립 순서', fontsize=16, weight='bold', va='top')
    ax.text(0.0, y - 0.025, '\n'.join(steps), fontsize=11.5, va='top', linespacing=1.45)
    y -= 0.04 + 0.0225 * sum(1 + s.count('\n') for s in steps)
    ax.text(0.0, y, '하중 · 실제 단면 검사', fontsize=16, weight='bold', va='top')
    ax.text(0.0, y - 0.03, '\n'.join(notes), fontsize=11.5, va='top', linespacing=1.45)
    y -= 0.045 + 0.0225 * sum(1 + s.count('\n') for s in notes)
    return ax


def legend_box(fig, rect_, keys):
    """Colour legend under panel (c)."""
    ax = fig.add_axes(rect_)
    ax.axis('off')
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.text(0.0, 0.97, '색  (빗금 = 금속, 회색 선 = 구매품 STEP의 면 몸체)', fontsize=14, weight='bold', va='top')
    for k, key in enumerate(keys):
        c, h, n = next((st[1], st[2], st[3]) for st in STYLE if st[3] == key) if key != OTHER[2] else OTHER
        cx, cy = (k % 5) * 0.2, 0.68 - (k // 5) * 0.27
        ax.add_patch(Rectangle((cx, cy - 0.09), 0.04, 0.18, fc=c, ec='k', lw=0.6, **(h or {})))
        ax.text(cx + 0.05, cy, n, fontsize=12, va='center')


WHY = [   # (both names contain, note): why a listed gap / overlap is as designed
    (('61807', '무릎 캡'), '61807 안륜이 캡 관에 끼워짐 — 정상'),
    (('hub 인서트', 'hip 판'), 'Ø28 보스 ↔ hip 판 Ø30 구멍 — 설계값'),
    (('hub 인서트', 'PETG 허벅지'), 'Ø28 보스 ↔ 무릎 판 Ø30 구멍 — 설계값'),
    (('hub 인서트', 'SUS 고정 링'), 'Ø28 보스 ↔ 고정 링 Ø30 — 설계값'),
    (('허브 플랜지', 'PETG 정강이'), '허브 Ø28 보스 ↔ 바퀴 판 Ø30 구멍 — 설계값'),
    (('허브 플랜지', 'SUS 고정 링'), '허브 보스 ↔ 고정 링 Ø30 — 설계값'),
    (('허브 플랜지', 'M2.5x8 stator'), '고정자 나사 머리 ↔ 허브 플랜지 — 설계값'),
    (('AKE60-8 고정자', '3x10'), '핀 끝 ↔ 모터 몸체 (AKE 형상)'),
    (('AKE60-8 고정자', 'M3x1'), '출력 나사 끝 ↔ 모터 몸체 (AKE 형상)'),
    (('AKE 출력부', ''), '출력부 ↔ 판 포켓 — 설계값'),
    (('무릎 캡', 'PETG 정강이'), '캡 관 끝 ↔ 정강이 구멍 — 설계값'),
    (('XT30', 'moteus'), '커넥터가 보드에 꽂힘 (구매품, 정상)'), (('CAN', 'moteus'), '커넥터가 보드에 꽂힘 (구매품, 정상)'),
]


def why(a, b):
    for (k1, k2), note in WHY:
        if (k1 in a and k2 in b) or (k1 in b and k2 in a):
            return note
    return '확인 필요'


def check_lines(recs, joint):
    gaps, laps = check(recs, joint)
    out = ['• 겹침(면적 > 0.02 mm², 5° 간격 단면 72장): ' + ('없음' if not laps else '')]
    for (a, b), v in sorted(laps.items(), key=lambda kv: -kv[1]):
        out.append('   %.1f mm²  %s ↔ %s — %s' % (v, a[1], b[1], why(a[1], b[1])))
    rows = sorted(gaps.items(), key=lambda kv: kv[1][0])
    out.append('• 서로 움직이는 부품 사이 가장 좁은 틈:')
    for (a, b), (d, beta) in rows[:6]:
        out.append('   %.2f mm  %s(%s) ↔ %s(%s) — %s' % (d, a[1], GROUPS[a[0]], b[1], GROUPS[b[0]], why(a[1], b[1])))
    return out, gaps, laps


def mx_out(joint, F, Fl):
    S = {'hip': (0.0, P.Z_HIP), 'knee': (P.KNEE_X, P.KNEE_Z), 'wheel': (0.0, P.R_WHEEL)}[joint]
    return SV.joint_moment(S, P.y_output(joint), F, Fl)


# ----------------------------------------------------------------------------- figures
def fig_hip(recs):
    BU, BD = 115, 270
    H = half(recs, 'hip', BU) + half(recs, 'hip', BD, flip=True)
    TF, TOF = LV.Y_TF, LV.SPAR[1]
    YR = LV.y_ring(TF)
    OUT = P.y_output('hip')
    fig = plt.figure(figsize=(24, 30))
    fig.suptitle('v5 hip 관절 — 실제 모델 단면 (Fusion WheelBiped_v5 버전 6, 11-02 PETG 설계, 왼쪽 다리, mm)  '
                 '위 절반: 출력 M3 자리 %d°, 아래 절반: ⑥ 플랜지 볼트 · 평행핀 자리 %d°' % (BU, BD),
                 fontsize=20, x=0.01, ha='left', y=0.995)
    ax = panel(fig, [0.04, 0.52, 0.55, 0.45], H, ((118, 296), (-100, 100)), '(a) 전체 — 몸통 hip 판부터 허벅지 바깥 면까지')
    flange_bolt(ax, TF)
    for txt, key, xt, pick in (('PETG 허벅지 (닫힌 상자 벽 3.5, 리브 2.0)', 'thigh_L main', (215, 88), 'max'),
                               ('6061 hip 판 (몸통)', 'hip_plate', (130, 92), 'max'),
                               ('AKE60-8 (hip)', 'AKE60-8_KV80_stator', (125, 60), None),
                               ('드라이버 컵 + moteus', 'cup_', (122, -80), None)):
        p = where(H, key, pick)
        if p:
            label(ax, txt, p, xt, fs=12)
    dim_h(ax, TF, TOF, -96, '허벅지 폭 %.1f (y %.0f ~ %.1f)' % (TOF - TF, TF, TOF), (None, None), fs=12)
    dim_h(ax, P.YW, TF, 70, '%.0f' % (TF - P.YW), (None, None), fs=11)
    label(ax, 'Ø%.0f 접근 구멍 (열림, 캡 없음 11-02):\n받침 링이 이리로 들어가고 ② 렌치가 지나감' % (2 * LV.BORE_R),
          (TOF - 3, LV.BORE_R - 6), (230, 52), fs=12)
    # (b) detail
    bx = panel(fig, [0.62, 0.62, 0.36, 0.35], H, ((158, 202), (-30, 30)), '(b) 확대 — 출력부 · hub 인서트 · 받침 링', ylabel=False)
    flange_bolt(bx, TF)
    dim_h(bx, TF, TF + LV.INSERT_FL[1], -27, '플랜지 %.0f' % LV.INSERT_FL[1], (-24, -24), fs=11)
    dim_h(bx, TF + LV.INSERT_FL[1], YR, -27, 'PETG 벽 %.0f' % LV.WALL_J, (-24, -24), fs=11)
    dim_h(bx, YR, YR + LV.RING[2], -27, '링 %.0f' % LV.RING[2], (-24, -24), fs=11)
    dim_h(bx, OUT, OUT + P.BEARING_FACE, 27, '와셔 자리 %.1f' % P.BEARING_FACE, (24, 24), fs=11, above=True)
    for n, key, xy in (('2', 'FX ISO 4762 M3x16', (190, 14)), ('3', 'FX ISO 8734', (172, -14)),
                       ('1', 'FX ISO 4762 M2.5x6 12.9 hip', (165, -25))):
        p = where(H, key)
        if p:
            tag(bx, n, xy, to=p)
    tag(bx, '6', (195, -24), c=NEW, to=(TF + 9, -LV.FLANGE_BOLT_R))
    label(bx, '6061 받침 링 Ø%.0f/Ø%.0f × %.0f\n(M3 탭 6개, r %.0f)' % (2 * LV.RING[1], 2 * LV.RING[0], LV.RING[2],
          LV.FLANGE_BOLT_R), (YR + 3, -LV.RING[1] + 1), (189, -8), fs=11)
    label(bx, 'hub 인서트 (6061)', (TF - 4, 10), (160, 26), fs=11)
    mh = {k: mx_out('hip', F, Fl) for k, F, Fl in (('설계', F_DES, FL_DES), ('검토', F_CHK, FL_CHK))}
    fb = {k: SV.bolt_circle_max(v, 6, LV.FLANGE_BOLT_R) for k, v in mh.items()}
    f2 = {k: SV.bolt_circle_max(v, 6, P.AKE_PCD_OUT / 2) for k, v in mh.items()}
    rows = [
        ('1', '#1f4e9c', 'ISO 4762 M2.5 × 6 12.9   × 6  (몸통 쪽, v4 그대로)',
         'hip 판 → AKE 고정자 링 (PCD 36.5).  hip 판 카운터보어 Ø4.8 × 2.7'),
        ('2', '#1f4e9c', 'ISO 4762 M3 × 16 12.9 + 와셔 DIN 125   × 6',
         'hub 인서트 → 모터 출력부 (PCD 20), 와셔 자리 = 출력면 + 11.2.  렌치는 허벅지 바깥 면 Ø44 구멍으로\n'
         '(바깥 면 → 머리 약 %.0f mm: 2.5 mm 육각 볼드라이버 120 mm 이상)' % (TOF - (OUT + P.BEARING_FACE + 0.5 + 3.0))),
        ('3', '#c0392b', '평행핀 ISO 8734 Ø3 × 10 m6   × 3', '인서트 Ø3 N7 × 7.5 바닥까지 압입 → 2.5 나와 출력부 구멍으로'),
        ('6', NEW, 'ISO 10642 M3 × 16 12.9 접시   × 6  — 11-02 새로, 모델에 없음',
         '인서트 플랜지(접시 Ø7, 면과 같게) → PETG 벽 %.0f mm → 6061 받침 링 탭 6 mm (r %.0f).  열압입 인서트 없앰'
         % (LV.WALL_J, LV.FLANGE_BOLT_R)),
    ]
    fits = [('A', 'Ø11.8 × 1.4 스피곳 (인서트) → 출력부 Ø12 × 1.5 홈'),
            ('B', 'Ø28 보스 → 몸통 hip 판 Ø30 구멍: 반경 틈 1.0'),
            ('C', '인서트 플랜지 Ø48 → 허벅지 자리 Ø48.3 × 6 (반경 0.15)'),
            ('D', '받침 링 Ø43 → 접근 구멍 Ø44 (반경 0.5, 위에서 떨어뜨려 넣음)')]
    steps = ['1) (작업대) 허벅지 바깥 면 Ø44 구멍으로 받침 링을 넣어 벽(y %.0f) 위에 얹고 구멍 6개를 맞춘다(Ø2.5 막대로)' % YR,
             '2) hub 인서트를 안쪽 면 자리(C)에 넣고 ⑥ 6개를 링 탭에 조인다 — 약 0.7 N·m + 나사 고정제, 핀 ③ 압입',
             '3) 허벅지를 hip 출력부에 끼운다(A, 핀) → ② 6개를 Ø44 구멍으로 넣어 조인다(와셔 먼저, 자석 드라이버)',
             '4) PETG 크리프: 처음 몇 주 ⑥ · ②를 다시 조인다']
    chk, gaps, laps = check_lines(recs, 'hip')
    notes = ['• hip 출력면 옆 모멘트 설계 %.0f / 검토 %.0f N·m (타이어 중심 y %.0f, 팔 %.0f mm)'
             % (mh['설계'] / 1000, mh['검토'] / 1000, SV.YTC, SV.YTC - OUT),
             '• ⑥ 볼트 하나 외력 설계 %.0f / 검토 %.0f N (+ 예압 1 kN, 보증 4879) — 통과; PETG 벽 압축 검토 약 38 MPa (50)'
             % (fb['설계'], fb['검토']),
             '• ② 출력 M3 하나 설계 %.0f / 검토 %.0f N (보증 4879) — 검토 하중에서 초과: AKE60 출력부 한계(다리 설계 무관)'
             % (f2['설계'], f2['검토'])] + chk
    text_column(fig, [0.62, 0.02, 0.37, 0.58], rows, fits, steps, notes)
    legend_box(fig, [0.04, 0.005, 0.55, 0.075], ['PETG 허벅지', '6061 hub 인서트', '6061 받침 링', '6061 hip 판', 'AKE60-8 고정자', 'AKE 출력부',
                    '드라이버 컵 (PETG)', 'moteus r4.11', '나사', '와셔', '평행핀', '몸통 부품'])
    # (c) the thigh along the beam (lower half) in more detail: wall, ring, bore
    cx = panel(fig, [0.04, 0.10, 0.55, 0.38], H, ((170, 292), (-60, 25)),
               '(c) 확대 — 허벅지 hip 머리: PETG 벽을 인서트 플랜지와 받침 링이 끼움, 그 위 Ø44 구멍이 바깥 면까지')
    flange_bolt(cx, TF)
    dim_h(cx, YR, TOF, 18, 'Ø%.0f 구멍 (y %.0f ~ %.1f)' % (2 * LV.BORE_R, YR, TOF), (None, None), fs=12, above=True)
    label(cx, '스풀 관 r %.0f (벽 %.0f)' % (LV.SPOOL_R, LV.SPOOL_R - LV.BORE_R), (240, -LV.SPOOL_R + 1), (250, -45), fs=12)
    tag(cx, '6', (176, -40), c=NEW, to=(TF + 9, -LV.FLANGE_BOLT_R))
    fig.savefig(HERE / 'hip_section.png', dpi=80)
    plt.close(fig)
    return gaps, laps


def fig_knee(recs):
    BU, BD = 130, 270
    H = half(recs, 'knee', BU) + half(recs, 'knee', BD, flip=True)
    SI, SO, SH = LV.Y_SI, LV.Y_SO, LV.Y_SH
    TO = LV.Y_TO
    YR = LV.y_ring(SI)
    OUT = P.y_output('knee')
    fig = plt.figure(figsize=(24, 30))
    fig.suptitle('v5 무릎 관절 — 실제 모델 단면 (Fusion WheelBiped_v5 버전 6, 11-02 PETG 설계, 왼쪽 다리, mm)  '
                 '위 절반: 출력 M3 자리 %d° (허벅지 쪽), 아래 절반: ⑥ 플랜지 볼트 자리 %d° (정강이 쪽)' % (BU, BD),
                 fontsize=20, x=0.01, ha='left', y=0.995)
    ax = panel(fig, [0.04, 0.52, 0.55, 0.45], H, ((140, 296), (-100, 100)), '(a) 전체 — 드라이버 컵부터 바깥 막대까지')
    flange_bolt(ax, SI)
    for txt, key, xt, pick in (('PETG 허벅지 (안쪽 갈래 · 바깥 막대)', 'thigh_L main', (150, 92), 'max'),
                               ('PETG 정강이 (기본층 30 + 머리 패드 15)', 'shank_L main', (255, -88), 'min'),
                               ('AKE60-8 (무릎)', 'AKE60-8_KV80_stator', (146, 60), None),
                               ('드라이버 컵 + moteus', 'cup_', (143, -75), None),
                               ('6061 무릎 캡 (관 Ø35이 61807 안륜에)', 'kneecap', (262, 60), None),
                               ('61807', 'kneebearing', (262, 40), None)):
        p = where(H, key, pick)
        if p:
            label(ax, txt, p, xt, fs=12)
    dim_h(ax, TO, SI, -47, '%.1f' % (SI - TO), (None, None), fs=12, c='#b03020', above=True)
    dim_h(ax, SI, SO, -66, '기본층 %.0f' % (SO - SI), (None, None), fs=11)
    dim_h(ax, SO, SH, -66, '패드 %.0f' % (SH - SO), (None, None), fs=11)
    dim_h(ax, SH, LV.SPAR[0], -72, '%.1f' % (LV.SPAR[0] - SH), (None, None), fs=11)
    dim_h(ax, LV.SPAR[0], LV.SPAR[1], -72, '바깥 막대 %.0f' % (LV.SPAR[1] - LV.SPAR[0]), (None, None), fs=11)
    dim_h(ax, TO, LV.SPAR[0], 96, '갈래 틈 %.1f' % (LV.SPAR[0] - TO), (None, None), fs=12)
    # (b) detail
    bx = panel(fig, [0.62, 0.62, 0.36, 0.35], H, ((180, 236), (-32, 32)), '(b) 확대 — 무릎 판 · hub 인서트 · 받침 링', ylabel=False)
    flange_bolt(bx, SI)
    dim_h(bx, TO, SI, 29, '틈 %.1f (11-02, 전 1.5)' % (SI - TO), (26, 26), fs=11, c='#b03020', above=True)
    dim_h(bx, SI, SI + LV.INSERT_FL[1], -29, '플랜지 %.0f' % LV.INSERT_FL[1], (-26, -26), fs=11)
    dim_h(bx, SI + LV.INSERT_FL[1], YR, -29, '벽 %.0f' % LV.WALL_J, (-26, -26), fs=11)
    dim_h(bx, YR, YR + LV.RING[2], -29, '링 %.0f' % LV.RING[2], (-26, -26), fs=11)
    for n, key, xy in (('1', 'FX DIN 7984 M2.5x8 stator', (196, 29)), ('2', 'FX ISO 4762 M3x16', (214, 17)),
                       ('3', 'FX ISO 8734', (190, -10)), ('5', 'FX ISO 4762 M2.5x18 knee cup', (184, 29))):
        p = where(H, key)
        if p:
            tag(bx, n, xy, to=p)
    tag(bx, '6', (228, -12), c=NEW, to=(SI + 9, -LV.FLANGE_BOLT_R))
    label(bx, '6061 받침 링 (r %.1f ~ %.1f)' % (LV.RING[0], LV.RING[1]), (YR + 3, -LV.RING[1] + 1), (222, -22), fs=11)
    # (c) outer support
    cx = panel(fig, [0.04, 0.10, 0.55, 0.38], H, ((236, 292), (-34, 34)),
               '(c) 확대 — 바깥 지지: 61807 (정강이 패드) + 무릎 캡 (바깥 막대에 꼭 맞음)')
    dim_h(cx, SH - LV.BRG[2], SH, 30, '61807 × %.0f' % LV.BRG[2], (None, None), fs=11, above=True)
    dim_h(cx, SH, LV.SPAR[0], 30, '%.1f' % (LV.SPAR[0] - SH), (None, None), fs=11, above=True)
    label(cx, '턱 r %.0f ~ %.1f: 61807 바깥륜만 앉음\n(안륜은 Ø%.0f 구멍 위에서 자유)' % (LV.BORE_R, LV.BRG[1] / 2, 2 * LV.BORE_R),
          (SH - LV.BRG[2] - 0.3, -LV.BORE_R - 0.7), (238, -30), fs=11)
    label(cx, '캡 관 Ø35 g6 ↔ 막대 구멍 Ø35 H7 (꼭 맞음, 11-02)', (LV.SPAR[0] + 6, -LV.CAP_R[1]), (255, -12), fs=11)
    tag(cx, '7', (285, 31), c=NEW, to=(LV.SPAR[1] - 1.5, LV.CAP_R[2] - 2))
    mk = {k: mx_out('knee', F, Fl) for k, F, Fl in (('설계', F_DES, FL_DES), ('검토', F_CHK, FL_CHK))}
    fb = {k: SV.bolt_circle_max(v, 6, LV.FLANGE_BOLT_R) for k, v in mk.items()}
    reach = LV.SPAR[1] - (OUT + P.BEARING_FACE + 0.5 + 3.0)
    rows = [
        ('1', '#1f4e9c', 'M2.5 × 8 저두 볼트 DIN 7984 8.8   × 6  + SUS304 고정 링 Ø45 / Ø30 × 2t',
         '허벅지 무릎 판(PETG 9) → AKE 고정자 링 (PCD 36.5), 바깥 면 홈 Ø45.6 × 3.7 안 — v4와 같음'),
        ('2', '#1f4e9c', 'M3 × 16 ISO 4762 12.9 + 와셔   × 6',
         '6061 hub 인서트 → 출력부 (PCD 20).  캡을 떼고 바깥 막대 구멍 · 61807 · Ø44 → 렌치 깊이 약 %.0f' % reach),
        ('3', '#c0392b', '평행핀 ISO 8734 Ø3 × 10   × 3', '인서트 구멍 바닥까지 압입 → 2.5 나옴'),
        ('5', '#1f4e9c', 'M2.5 × 18 ISO 4762   × 3 + 황동 인서트', '드라이버 컵 → 허벅지 하우징 끝면(y 174) — v4와 같음'),
        ('6', NEW, 'M3 × 16 접시 ISO 10642 12.9   × 6  — 11-02 새로, 모델에 없음',
         '인서트 플랜지 → 정강이 PETG 벽 %.0f → 6061 받침 링 탭 (r %.0f).  열압입 인서트 없앰' % (LV.WALL_J, LV.FLANGE_BOLT_R)),
        ('7', NEW, 'M2.5 × 8 저두 DIN 7984   × 3 + 황동 인서트 — 모델에 없음',
         '무릎 캡 플랜지 → 바깥 막대 (r %.1f).  캡 관이 구멍에 꼭 맞아 예압만 받음' % LV.CAP_BOLT_R),
        ('8', '#8f8f8f', '깊은 홈 볼 베어링 61807 (Ø35 × Ø47 × 7)   × 1', '정강이 패드 바깥 면 자리, 무릎 캡 관에 안륜'),
    ]
    fits = [('A', 'Ø11.8 스피곳 → 출력부 Ø12 홈, Ø28 보스 → 무릎 판 Ø30 구멍 (반경 1.0)'),
            ('B', '무릎 판 바깥 면 ↔ 정강이 안쪽 면 %.1f (11-02, 전 1.5)' % (SI - TO)),
            ('C', '61807 바깥륜 → 패드 자리 Ø47 압입, 턱 r 22 ~ 23.5'),
            ('D', '캡 관 Ø35 g6 → 61807 안륜 · 바깥 막대 구멍 Ø35 H7 (출력은 작게 뽑고 리머)'),
            ('E', '받침 링 Ø43 → Ø44 구멍 (반경 0.5)')]
    steps = ['1) 무릎 모터를 허벅지 하우징에 넣고 ① 6개(바깥 면 홈의 SUS 링 위), 컵 · moteus ⑤',
             '2) (작업대) 정강이: 받침 링을 61807 자리 쪽 Ø44 구멍으로 넣어 벽(y %.0f)에 얹음 → hub 인서트 + ⑥ 6개(0.7 N·m)'
             % YR,
             '   → 핀 ③ → 61807을 패드 자리에 압입(턱에 닿게)',
             '3) 정강이를 9.5 mm 바깥으로 띄워 갈래에 넣고(핀 끝 ↔ 무릎 판 1.0, 패드 ↔ 바깥 막대 1.0) 안쪽으로 밀어',
             '   보스를 Ø30 구멍 · 스피곳을 출력부 홈에, 핀을 출력부 구멍에',
             '4) ② 6개: 바깥 막대 구멍 → 61807 → Ø44 → 인서트 (와셔 먼저)',
             '5) 무릎 캡을 막대 구멍 · 61807 안륜에 밀어 넣고(꼭 맞음: 고무망치로 곧게) ⑦ 3개']
    chk, gaps, laps = check_lines(recs, 'knee')
    notes = ['• 무릎 출력면 옆 모멘트 설계 %.0f / 검토 %.0f N·m (모터가 다 받을 때); 61807이 짝힘으로 나눠 받음'
             % (mk['설계'] / 1000, mk['검토'] / 1000),
             '• ⑥ 볼트 하나 외력 설계 %.0f / 검토 %.0f N (+ 예압 1 kN, 보증 4879) — 통과' % (fb['설계'], fb['검토']),
             '• 캡 구멍 면압 검토 약 22 MPa (50) — 통과; 61807 반경 하중 검토 2.25 kN (C0 3.35)'] + chk
    text_column(fig, [0.62, 0.02, 0.37, 0.58], rows, fits, steps, notes)
    legend_box(fig, [0.04, 0.005, 0.55, 0.075], ['PETG 허벅지', 'PETG 정강이', '6061 hub 인서트', '6061 받침 링', '6061 무릎 캡', '61807',
                    'AKE60-8 고정자', 'AKE 출력부', 'SUS 고정 링', '드라이버 컵 (PETG)', 'moteus r4.11', '나사', '와셔',
                    '평행핀', '황동 열압입 인서트'])
    fig.savefig(HERE / 'knee_section.png', dpi=80)
    plt.close(fig)
    return gaps, laps


def fig_wheel(recs):
    BU, BD = 125, 270
    H = half(recs, 'wheel', BU) + half(recs, 'wheel', BD, flip=True)
    SI, SO = LV.Y_SI, LV.Y_SO
    OUT = 2 * LV.Y_MSH - P.y_output('wheel')
    fig = plt.figure(figsize=(24, 30))
    fig.suptitle('v5 바퀴 관절 — 실제 모델 단면 (Fusion WheelBiped_v5 버전 6, 11-02 바퀴 묶음 2.5 mm 바깥으로, 왼쪽 다리, mm)  '
                 '위 절반 %d°: 허브 M3 자리, 아래 절반 %d°: 림 M3 · 평행핀 자리' % (BU, BD),
                 fontsize=20, x=0.01, ha='left', y=0.995)
    ax = panel(fig, [0.04, 0.52, 0.55, 0.45], H, ((176, 302), (-108, 108)), '(a) 전체 — 드라이버 컵부터 타이어까지')
    for txt, key, xt, pick in (('PETG 정강이 (바퀴 하우징, 안쪽 면 평평 y %.0f)' % SI, 'shank_L main', (180, 100), 'max'),
                               ('AKE60-8 (바퀴)', 'AKE60-8_KV80_stator', (182, 62), None),
                               ('드라이버 컵 + moteus', 'cup_', (180, -85), None),
                               ('PETG 허브 플랜지', 'hub_flange', (262, 30), None),
                               ('PETG 림', 'rim_', (300, -40), None),
                               ('타이어 M365 8.5 × 2', 'tyre', (300, 104), None)):
        p = where(H, key, pick)
        if p:
            label(ax, txt, p, xt, fs=12, ha='right' if xt[0] > 280 else 'left')
    tc = 2 * LV.Y_MSH - P.tyre_centre_y()
    dim_h(ax, SI, SO, -104, '정강이 %.0f (y %.0f ~ %.0f)' % (SO - SI, SI, SO), (None, None), fs=12)
    dim_h(ax, SO, SO + P.GAP_TYRE, 92, '%.0f' % P.GAP_TYRE, (None, None), fs=11)
    dim_h(ax, OUT, tc, -80, '출력면 → 타이어 중심 %.1f' % (tc - OUT), (None, None), fs=11)
    ax.axvline(tc, color='#b03020', lw=0.8, ls='--', zorder=5)
    ax.text(tc + 1, -60, '타이어 중심\ny %.1f' % tc, color='#b03020', fontsize=11,
            bbox=dict(fc='white', ec='none', alpha=0.85, pad=0.3))
    bx = panel(fig, [0.62, 0.62, 0.36, 0.35], H, ((200, 256), (-32, 32)), '(b) 확대 — 바퀴 판 · 출력부 · 허브 플랜지', ylabel=False)
    for n, key, xy in (('1', 'FX DIN 7984 M2.5x8 stator', (243, 29)), ('2', 'FX ISO 4762 M3x14 12.9 hub', (252, 16)),
                       ('3', 'FX ISO 8734', (222, -10)), ('4', 'FX ISO 4762 M3x14 rim', (252, -26)),
                       ('5', 'FX ISO 4762 M2.5x18 wheel cup', (204, 29))):
        p = where(H, key)
        if p:
            tag(bx, n, xy, to=p)
    rows = [
        ('1', '#1f4e9c', 'M2.5 × 8 저두 DIN 7984 8.8   × 6  + SUS 고정 링',
         '정강이 바퀴 판(PETG 9, 바깥 면 홈) → AKE 고정자 링 — 허브 플랜지보다 먼저'),
        ('2', '#1f4e9c', 'M3 × 14 ISO 4762 12.9 + 와셔   × 6', 'PETG 허브 플랜지 → 바퀴 출력부 (PCD 20)'),
        ('3', '#c0392b', '평행핀 Ø3 × 10   × 3', '허브 플랜지 → 출력부'),
        ('4', '#1f4e9c', 'M3 × 14 ISO 4762   × 6 + 황동 인서트 M3 × 5.7', '림 → 허브 플랜지'),
        ('5', '#1f4e9c', 'M2.5 × 18 ISO 4762   × 3 + 황동 인서트', '드라이버 컵 → 바퀴 하우징 안쪽 끝면 (y %.0f)' % SI),
    ]
    fits = [('A', '허브 Ø28 보스 → 바퀴 판 Ø30 구멍 (반경 1.0)'),
            ('B', '컵 립 Ø88.1 → 바퀴 하우징 Ø88 (정강이 안쪽 면과 같은 높이, 11-02)'),
            ('C', '정강이 바깥 면 ↔ 타이어 %.0f' % P.GAP_TYRE)]
    steps = ['1) 바퀴 모터를 정강이 하우징에 안쪽에서 넣고 ① 6개를 바깥 면에서(SUS 링 위)',
             '2) 컵 · moteus를 하우징 안쪽 끝(y %.0f)에 ⑤' % SI,
             '3) 허브 플랜지를 출력부에(보스 A, 핀 ③) ② 6개',
             '4) 타이어를 림에 끼우고 림을 허브 플랜지에 ④ 6개']
    chk, gaps, laps = check_lines(recs, 'wheel')
    mw = {k: mx_out('wheel', F, Fl) for k, F, Fl in (('설계', F_DES, FL_DES), ('검토', F_CHK, FL_CHK))}
    notes = ['• 바퀴 출력면 옆 모멘트 설계 %.0f / 검토 %.0f N·m; PETG 바퀴 판 · 고정 링 통과 (calc/strength_v5.md 2절)'
             % (mw['설계'] / 1000, mw['검토'] / 1000),
             '• 11-02: 바퀴 묶음을 2.5 mm 바깥으로 → 정강이 안쪽 면 평평(출력 바닥면), 폭 30 유지, 로봇 폭 594'] + chk
    text_column(fig, [0.62, 0.02, 0.37, 0.58], rows, fits, steps, notes)
    legend_box(fig, [0.04, 0.005, 0.55, 0.075], ['PETG 정강이', 'AKE60-8 고정자', 'AKE 출력부', 'SUS 고정 링', '드라이버 컵 (PETG)', 'moteus r4.11',
                    'PETG 허브 플랜지', 'PETG 림', '타이어 M365', '나사', '와셔', '평행핀', '황동 열압입 인서트'])
    cx = panel(fig, [0.04, 0.10, 0.55, 0.38], H, ((196, 226), (30, 52)),
               '(c) 확대 — 컵 립이 바퀴 하우징 끝(y %.0f)에 걸림: 정강이 안쪽 면과 같은 높이 (11-02)' % SI)
    dim_h(cx, SI, SI + 3.1, 49.5, '컵 립 3.1', (None, None), fs=11, above=True)
    label(cx, '정강이 안쪽 면 = 하우징 끝 y %.0f (평평한 출력 바닥면)' % SI, (SI + 0.2, 47.5), (211, 50.8), fs=11)
    fig.savefig(HERE / 'wheel_section.png', dpi=80)
    plt.close(fig)
    return gaps, laps


if __name__ == '__main__':
    R = load()
    for f in (fig_hip, fig_knee, fig_wheel):
        gp, lp = f(R)
        print(f.__name__, 'overlaps', len(lp), 'closest', min((v[0] for v in gp.values()), default=None))
