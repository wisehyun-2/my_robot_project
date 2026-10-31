"""Wheel output stack: shank plate -> AKE60 -> hub flange -> hex hub -> RC wheel, the parts and screws that carry the
robot's weight into the wheel, half sections to scale from fusion/params.py (robot y of the LEFT leg on the horizontal
axis, radius from the wheel axis vertically; upper half through the screws, lower half through a dowel pin).
Overview (whole wheel) + enlarged detail.  Explanatory figure, not a manufacturing drawing; wheel_joint_section.py and
wheel_joint_contact.py zoom further into the motor joint.

    python wheel_output_stack.py      # -> wheel_output_stack.png next to this script
"""
import math
import pathlib
import sys
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / 'fusion'))
import params as P
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Polygon, Rectangle
for F in ('C:/Windows/Fonts/malgun.ttf', '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc'):
    if pathlib.Path(F).exists():
        font_manager.fontManager.addfont(F)
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=F).get_name()
        break
plt.rcParams['axes.unicode_minus'] = False
A = P.AKE

# ----------------------------------------------------------------------------- stations (robot y, left leg)
def ym(local):                                   # AKE local y -> robot y (the wheel motor's back points to +y)
    return P.Y_RING_WHEEL + (local - A['ring'])
OUT = P.y_output('wheel')                        # 211.5 output face (faces the robot centre)
RING = P.Y_RING_WHEEL                            # 212.0 stator ring face = pilot pocket floor
PL0, PL1 = P.Y_TIBIA_IN, P.Y_TIBIA_IN + P.WHEEL_PLATE_T        # shank wheel plate 205.5 .. 214.5
SH1 = P.Y_TIBIA_OUT                              # 235.5
REC = PL0 + P.RING_T + P.CBORE_M25[1]            # stator ring recess floor 209.2
F1 = PL0 - 1.0; F0 = F1 - 8.0; H0 = F0 - 6.0     # hub flange 196.5 .. 204.5, hex hub flange 190.5 .. 196.5
TC = P.tyre_centre_y()                           # 170.5
SEAT = TC + P.WHEEL_SEAT                         # 155.5 hex end face = wheel spoke face
HEX1 = SEAT + P.HEX_L + 2.0                      # 169.5 hex hub Ø30 body start
SOCK = SEAT + P.HEX_L + 1.0                      # 168.5 end of the wheel's hex socket hub (vendor.rc_wheel)
WEB = SEAT - 4.0                                 # 151.5 wheel web outer face (the nut flange bears here)
NUT1 = WEB - 6.0                                 # 145.5 nut far face (modelled flange nut: flange 1 + hex 5)
BOLT_HEAD = F0 + 1.5                             # 198.0 M5 head bears on the hex hub register spigot
BOLT_L = 55.0                                    # 10-31: 50 -> 55 (the M5 x 50 engaged only 3.5 of the 6 mm nut)
BOLT_TIP = BOLT_HEAD - BOLT_L                    # 143.0, 2.5 past the nut, 2.5 inside the tyre side face
BEAR = OUT - P.HUB_SCREW_SEAT                    # 202.0 washer seat of the M3 x 14 in the flange (10-30)
LS = P.HUB_SCREW_L                               # 14
ENG = BEAR - 0.5 + LS - OUT                      # 4.0 thread engagement in the output taps
T0, T1 = TC - P.TYRE_W / 2, TC + P.TYRE_W / 2    # 140.5 .. 200.5
RO, RB = P.R_WHEEL, P.TYRE_BEAD_D / 2
RP, RQ, RH = P.AKE_PCD_RING / 2, P.AKE_PCD_OUT / 2, 18.0        # M2.5 r 18.25, output M3 r 10, hub M3 r 18
HEXF = P.HEX_AF / 2
ROBOT_KG = 9.18                                  # CAD with every fastener modelled (10-30)
W_STATIC = ROBOT_KG * P.G / 2

g = dict(petg='#9fd29f', flange='#e07b39', hexhub='#f2a65a', stator='#b8b8b8', rotor='#7f9fbf', out='#d9b38c',
         tyre='#4a4a4a', rim='#9a9a9a', screw='#2e2e2e', washer='#e3c034', pin='#c0392b', ring='#6f86a6',
         insert='#b8860b', nut='#5a5a5a')


def tyre_pts(rr=14.0, n=10):
    pts = [(T0, RB)]
    for cx, a0, a1 in ((T0 + rr, 180, 90), (T1 - rr, 90, 0)):
        for k in range(n + 1):
            t = math.radians(a0 + (a1 - a0) * k / n)
            pts.append((cx + rr * math.cos(t), RO - rr + rr * math.sin(t)))
    pts.append((T1, RB))
    return pts


def draw_parts(ax, tyre=True):
    def poly(pts, c, s=1, z=1, **kw):
        ax.add_patch(Polygon([(x, s * r) for x, r in pts], fc=c, ec=kw.pop('ec', 'k'), lw=kw.pop('lw', 0.9),
                             zorder=z, **kw))

    def rect(x0, x1, r0, r1, c, s=1, z=1, **kw):
        poly([(x0, r0), (x1, r0), (x1, r1), (x0, r1)], c, s, z, **kw)

    for s in (1, -1):
        # RC wheel: tyre, rim barrel, hex socket hub + web (spokes lie outside this section plane: dashed)
        if tyre:
            poly(tyre_pts(), g['tyre'], s)
            rect(T0 + 2, T1 - 2, RB - 3, RB, g['rim'], s)
        poly([(WEB, 2.8), (SEAT, 2.8), (SEAT, HEXF + 0.1), (SOCK, HEXF + 0.1), (SOCK, 15.5), (WEB, 15.5)], g['rim'], s)
        poly([(WEB + 0.5, 15.5), (WEB + 6, 15.5), (WEB + 6, RB - 3), (WEB + 0.5, RB - 3)], g['rim'], s, z=0.5,
             alpha=0.3, ls='--')
        # shank (PETG): wheel plate + housing around the motor (can cavity r 37); the beam goes on up to the knee
        top = 52.0 if s > 0 else P.HOUSING_R
        poly([(PL0, P.STATOR_RECESS_D / 2), (PL0, top), (SH1, top), (SH1, P.CAN_CLEAR_R), (PL1, P.CAN_CLEAR_R),
              (PL1, 21.025), (RING, 21.025), (RING, 15.0), (REC, 15.0), (REC, P.STATOR_RECESS_D / 2)], g['petg'], s)
        rect(REC - P.RING_T, REC, P.RING_STATOR[1] / 2, P.RING_STATOR[0] / 2, g['ring'], s, z=2, lw=1.0)
        # AKE60-8: stator (pilot Ø42 x 3, front face r 20.3 .. 30.75), rotor can + cap, output carrier
        sf, cb = ym(A['stator_front']), ym(A['can_back'])
        poly([(RING, 13.6), (RING, 21.0), (RING + 3, 21.0), (RING + 3, 20.3), (sf, 20.3), (sf, 30.75), (cb, 30.75),
              (cb, 13.6)], g['stator'], s)
        poly([(ym(A['can_front']), 31.2), (ym(A['can_front']), A['can_r']), (ym(A['cap_outer']), A['can_r']),
              (ym(A['cap_outer']), 31.2)], g['rotor'], s)
        poly([(cb, 7.9), (ym(A['cap_face']), 7.9), (ym(A['cap_face']), 34.2), (cb, 34.2)], g['rotor'], s)
        poly([(OUT + 1.5, 0), (OUT + 1.5, 6.0), (OUT, 6.0), (OUT, 13.5), (cb, 13.5), (cb, 0)], g['out'], s)
        # hub flange (PETG 100 %): Ø46 disc, Ø28 boss through the Ø30 plate hole, Ø11.8 spigot into the Ø12 recess
        poly([(F0 + 7, 0), (F0 + 7, 4.75), (F0 + 1.6, 4.75), (F0 + 1.6, 10.1), (F0, 10.1), (F0, 23.0), (F1, 23.0),
              (F1, 14.0), (OUT, 14.0), (OUT, 5.9), (OUT + 1.4, 5.9), (OUT + 1.4, 0)], g['flange'], s)
        # hex hub (PETG 100 %): 17 mm hex in the wheel, Ø30 body, Ø46 flange, Ø20 register spigot
        poly([(SEAT, 2.65), (SEAT, HEXF), (HEX1 + 0.5, HEXF), (HEX1 + 0.5, 15.0), (H0, 15.0), (H0, 23.0),
              (F0, 23.0), (F0, 10.0), (F0 + 1.5, 10.0), (F0 + 1.5, 2.65)], g['hexhub'], s)
        # M5 x 55 axle bolt (head in the flange pocket, on the hex hub spigot) + serrated flange nut on the web
        rect(BOLT_HEAD, BOLT_HEAD + 5.0, 0, 4.25, g['screw'], s, z=3)
        rect(BOLT_TIP, BOLT_HEAD, 0, 2.5, g['screw'], s, z=3)
        rect(WEB - 1.0, WEB, 2.5, 6.0, g['nut'], s, z=3)
        rect(NUT1, WEB - 1.0, 2.5, 4.0, g['nut'], s, z=3)
        # stator ring screws: DIN 7984 M2.5 x 8 low head on the SUS ring (6 on PCD 36.5: both halves)
        rect(REC - P.RING_T, REC - P.RING_T + 8.0, RP - 1.25, RP + 1.25, g['screw'], s, z=3)
        rect(REC - P.RING_T - 1.5, REC - P.RING_T, RP - 2.25, RP + 2.25, g['screw'], s, z=3)
    # upper half: output M3 x 14 at r 10 (washer under the head), hub M3 x 8 at r 18 into a brass insert
    rect(BEAR - 0.5, BEAR, RQ - 3.5, RQ + 3.5, g['washer'], z=3, lw=0.8)
    rect(BEAR - 3.5, BEAR - 0.5, RQ - 2.75, RQ + 2.75, g['screw'], z=3)
    rect(BEAR - 0.5, BEAR - 0.5 + LS, RQ - 1.25, RQ + 1.25, g['screw'], z=3)
    rect(F0 + 0.1, F0 + 5.8, RH - 2.0, RH + 2.0, g['insert'], z=2.5, lw=0.8)
    rect(H0 + 3.5 - 3.0, H0 + 3.5, RH - 2.75, RH + 2.75, g['screw'], z=3)
    rect(H0 + 3.5, H0 + 3.5 + 8.0, RH - 1.25, RH + 1.25, g['screw'], z=3)
    # lower half: dowel Ø3 x 10 (3 mm in the output, 7 mm in the flange)
    rect(OUT - 7.0, OUT + 3.0, -RQ - 1.5, -RQ + 1.5, g['pin'], z=3, lw=1.0)


def load_path(ax, lw, r0=-RO + 1):
    path = [(TC, r0), (TC, -RB + 1.5), (TC - 6, -16), (SEAT + 5, -HEXF + 2), (180, -12.5), (H0 + 3, -19),
            (F0 + 4, -19), (OUT - 3, -12.5), (OUT + 4, -9.5), (ym(4), -11), (ym(4), -17.5), (RING + 2, -18),
            (REC + 1, -26), (PL0 + 4, -40), (PL0 + 4, -50)]
    ax.plot(*zip(*path[:-1]), color='red', lw=lw, alpha=0.7, zorder=6, solid_capstyle='round', solid_joinstyle='round')
    ax.annotate('', xy=path[-1], xytext=path[-2], zorder=6,
                arrowprops=dict(arrowstyle='-|>', color='red', alpha=0.7, lw=lw, mutation_scale=8 * lw))


def tag(ax, n, xy, c='#1f4e9c', to=None, fs=13):
    if to is not None:
        ax.plot([xy[0], to[0]], [xy[1], to[1]], 'k-', lw=0.9, zorder=9)
    ax.text(*xy, n, fontsize=fs, weight='bold', color='white', ha='center', va='center', zorder=10,
            bbox=dict(boxstyle='circle,pad=0.2', fc=c, ec='white', lw=1.0))


def dim_h(ax, x0, x1, r, txt, ext=(), fs=11, c='k', above=False):
    ax.annotate('', xy=(x0, r), xytext=(x1, r), zorder=8,
                arrowprops=dict(arrowstyle='<->', color=c, lw=0.9, shrinkA=0, shrinkB=0))
    for x, re in zip((x0, x1), ext):
        if re is not None:
            ax.plot([x, x], [re, r], color=c, lw=0.6, ls=':', zorder=8)
    ax.text((x0 + x1) / 2, r + (0.6 if above else -0.6), txt, fontsize=fs, ha='center', va='bottom' if above else 'top',
            color=c, zorder=8)


def dim_v(ax, x, r0, r1, txt, fs=11, c='k'):
    ax.annotate('', xy=(x, r0), xytext=(x, r1), arrowprops=dict(arrowstyle='<->', color=c, lw=0.9, shrinkA=0, shrinkB=0))
    ax.text(x - 1.0, (r0 + r1) / 2, txt, fontsize=fs, rotation=90, ha='right', va='center', color=c)


def label(ax, txt, xy, xt, fs=11, c='k', ha='left'):
    ax.annotate(txt, xy=xy, xytext=xt, fontsize=fs, color=c, ha=ha, va='center', zorder=11,
                arrowprops=dict(arrowstyle='->', lw=0.8, color=c, shrinkA=2, shrinkB=0),
                bbox=dict(fc='white', ec='none', alpha=0.85, pad=0.6))


fig = plt.figure(figsize=(24, 28))
fig.suptitle('바퀴 출력부 — 로봇 무게를 바퀴로 넘기는 연결  (왼쪽 다리 단면, 모델 치수 mm, 실제 비율)',
             fontsize=22, x=0.01, ha='left', y=0.995)

# ============================================================================= (a) overview
ax = fig.add_axes([0.01, 0.585, 0.37, 0.375]); ax.set_aspect('equal'); ax.axis('off')
draw_parts(ax)
load_path(ax, 3.0)
ax.plot([NUT1 - 3, ym(A['cap_face']) + 3], [0, 0], 'k-.', lw=0.7, zorder=8)
ax.set_title('(a) 전체 — 바퀴 하나', fontsize=17, loc='left')
T = dict(fontsize=12, ha='center', va='center', zorder=9)
ax.text(TC, 62, '타이어 Ø150 × 60\n(1/8 트러기, 접착 완제품)', color='white', **T)
ax.text(TC, -62, '바닥 반력\n정지 약 %.0f N' % W_STATIC, color='white', weight='bold', **T)
ax.annotate('', xy=(TC, -RO + 1), xytext=(TC, -RO - 10), zorder=7,
            arrowprops=dict(arrowstyle='-|>', color='red', lw=3, mutation_scale=24))
ax.text(PL0 + 15, 47, 'PETG 정강이\n↑ 무릎까지', **T)
ax.text(226, 25, 'AKE60-8', **T)
ax.text(SH1 + 1, 47, 'moteus\n컵(생략)', fontsize=10, ha='left', va='center', color='#555')
dim_v(ax, 131, -RO, RO, 'Ø150')
R1, R2 = -88.0, -97.0
dim_h(ax, T0, T1, R1, '타이어 폭 60', (-RO + 4, -RO + 4), fs=12)
dim_h(ax, T1, PL0, R1, '5', (-RO + 4, -P.HOUSING_R), fs=12)
dim_h(ax, PL0, PL1, R1, '9', (None, -P.HOUSING_R), fs=12)
dim_h(ax, TC, OUT, R2, '굽힘 팔 41.0', (-RO + 4, -14), fs=12, c='#b03020')
ax.add_patch(Rectangle((142, -36), 99, 74, fill=False, ec='#1f4e9c', lw=1.6, ls='--', zorder=12))
ax.text(142, 39.5, '(b) 확대 범위', color='#1f4e9c', fontsize=12, va='bottom', zorder=12)
ax.set_xlim(124, 246); ax.set_ylim(-106, 82)

# ============================================================================= (b) detail
ax = fig.add_axes([0.01, 0.01, 0.70, 0.555]); ax.set_aspect('equal'); ax.axis('off')
draw_parts(ax, tyre=False)
load_path(ax, 4.0, -46.0)
ax.text(TC + 1.2, -46, '↑ 타이어·림에서', color='red', fontsize=11, va='center', zorder=9)
ax.plot([NUT1 - 3, 241], [0, 0], 'k-.', lw=0.8, zorder=8)
ax.text(241, 0.6, '회전축', fontsize=10, ha='right', va='bottom', zorder=8)
ax.set_title('(b) 확대 — 위쪽 절반: 나사 자리 단면,  아래쪽 절반: 평행핀 자리 단면,  빨간 선: 무게가 지나가는 길',
             fontsize=17, loc='left')
T = dict(fontsize=12, ha='center', va='center', zorder=9)
ax.text(181, 8.8, '육각 허브\n(PETG 100 %)', **T)
ax.text(162, 12.0, '림 육각 소켓', fontsize=10.5, ha='center', va='center', zorder=9)
ax.text(PL0 + 1.8, 38, '정강이 바퀴 판 (PETG)', fontsize=11, rotation=90, ha='center', va='center', zorder=9)
ax.text(226.5, 25.2, 'AKE60-8 고정자', **T)
ax.text(226.5, 7.0, 'AKE 출력부 (회전)', **T)
ax.text(227, 32.85, '로터 캔 (회전)', fontsize=9.5, ha='center', va='center', zorder=9, color='white')
ax.text(225, 46, '정강이 하우징 (PETG)', **T)
label(ax, '허브 플랜지 (PETG 100 %)', (200.5, 21.5), (195, 44.0), fs=12, c='#9c4a17', ha='right')
label(ax, 'SUS304 클램프 링 2 t (Ø45 / Ø30)', (REC - 1.0, 21.8), (195, 39.5), fs=11, ha='right')
label(ax, 'Ø28 보스 / Ø30 판 구멍 (반경 틈 1.0)', (205.0, 14.0), (195, 35.0), fs=11, ha='right')
label(ax, 'Ø46 (허브 플랜지 · 육각 허브 플랜지)', (193.5, 23.0), (195, 30.5), fs=11, ha='right')
label(ax, '림 웹', (153.5, 13), (143, 24), fs=11)

# screws / pins
tag(ax, '1', (202.8, 27.0), to=(206.2, RP + 2.2))
tag(ax, '2', (193.6, 11.8), to=(197.0, RQ + 1.5))
tag(ax, '3', (199.5, -16.5), c='#c0392b', to=(205.5, -RQ - 1.4))
tag(ax, '4', (186.5, 23.0), to=(192.3, RH + 2.8))
tag(ax, '5', (172, 5.4), to=(172, 2.5))
tag(ax, '6', (143.6, 9.5), to=(147.5, 4.0))
tag(ax, 'A', (215.6, -3.2), c='#2e7d32', to=(212.6, -5.0))
tag(ax, 'B', (193.0, -12.5), c='#2e7d32', to=(197.2, -9.6))
tag(ax, 'C', (218.5, -25.5), c='#2e7d32', to=(213.6, -21.0))

# diameters (leaders)
for txt, xy, xt in (('Ø30 몸통', (175, 15.0), (162, 21.0)),
                    ('17 육각(맞변)', (158, HEXF), (143, 16.5)),
                    ('Ø42 파일럿 / Ø42.05 포켓', (214.2, 21.0), (219.5, 41.0))):
    label(ax, txt, xy, xt, fs=11)
for txt, xy, xt in (('핀 · M3 × %d: PCD 20' % LS, (206.0, -RQ - 1.5), (195, -29.0)),
                    ('M2.5 × 8: PCD 36.5 (M3 × 8은 PCD 36)', (205.8, -RP - 2.2), (195, -34.5))):
    label(ax, txt, xy, xt, fs=11, ha='right')

# axial dimensions: hub side (inside the rim, lower left) and joint side (below the shank housing)
dim_h(ax, SEAT, SOCK, -44.0, '육각 물림 13', (-HEXF, -15.5), fs=11)
dim_h(ax, H0, F0, -44.0, '6', (-23, -23), fs=11)
dim_h(ax, F0, F1, -44.0, '8', (None, -23), fs=11)
dim_h(ax, NUT1, WEB, -50.0, '너트 6', (-4, -6), fs=11)
dim_h(ax, BOLT_TIP, BOLT_HEAD, -55.0, 'M5 × 55 (머리 받침면 → 끝)', (-2.5, -4.25), fs=11)
dim_h(ax, NUT1, WEB, -61.0, '너트 물림 6 (전체)', (-2.5, -6), fs=11)
dim_h(ax, BOLT_TIP, NUT1, -50.0, '2.5', (-2.5, -6), fs=11)
dim_h(ax, F1, PL0, -50.0, '1', (-23, -44), fs=11)
dim_h(ax, PL0, PL1, -50.0, '판 9', (None, -44), fs=11)
dim_h(ax, BEAR, OUT, -55.0, '%.1f' % P.HUB_SCREW_SEAT, (-RQ - 3.5, -14), fs=11)
dim_h(ax, OUT, OUT + ENG, -61.0, '%.1f' % ENG, (-14, None), fs=11)
ax.text(OUT + 5.0, -61.6, '← M3 × %d 물림' % LS, fontsize=11, va='top')
dim_h(ax, RING, PL1, 55.0, '포켓 2.5', (52, 52), fs=10.5, above=True)
dim_h(ax, PL0, REC, 55.0, '링 자리 3.7', (52, 52), fs=10.5, above=True)
ax.set_xlim(140.5, 242); ax.set_ylim(-66, 60)

# ============================================================================= text columns
tx = fig.add_axes([0.40, 0.585, 0.59, 0.375]); tx.axis('off'); tx.set_xlim(0, 1); tx.set_ylim(0, 1)
rows = [
    ('1', '#1f4e9c', 'M2.5 × 8  저두 볼트 DIN 7984 8.8   × 6',
     '정강이 판 → AKE 고정자 링 (PCD 36.5).  SUS304 클램프 링 2 t (Ø45 / Ø30) 위에서 조임,\n'
     '머리 1.5 t는 판 면보다 0.2 들어감.  고정자 탭(깊이 5)에 3.2 mm 물림'),
    ('2', '#1f4e9c', 'M3 × %d  ISO 4762 12.9   × 6   + 평와셔 DIN 125 M3 (Ø7 × 0.5)   × 6' % LS,
     '허브 플랜지 → 모터 출력부 (PCD 20).  머리는 플랜지 카운터보어 Ø7.5 안, 육각 허브 Ø20 턱에서 0.5 mm,\n'
     '와셔 자리가 출력면에서 %.1f → 출력부 탭(깊이 5)에 %.1f mm 물림' % (P.HUB_SCREW_SEAT, ENG)),
    ('3', '#c0392b', '평행핀 ISO 8734 Ø3 × 10  m6 경화   × 3',
     '출력부 구멍 3 mm + 허브 플랜지 7 mm (PCD 20, 나사와 다른 각도).\n'
     '두 부품 위치를 맞추고 바퀴 토크를 나사와 함께 받음'),
    ('4', '#1f4e9c', 'M3 × 8  ISO 4762 8.8   × 6   + 황동 열압입 인서트 M3 × 5.7   × 6',
     '육각 허브 → 허브 플랜지 (r 18 = PCD 36).  인서트 구멍 Ø4.0 × 6, 물림 5.5 mm.\n'
     '나사는 바퀴 스포크 사이(30° 어긋남)에 있어 바퀴를 끼운 채 렌치가 들어감'),
    ('5', '#1f4e9c', 'M5 × 55  ISO 4762 8.8   × 1   (바퀴 축 볼트)',
     '머리가 플랜지 포켓(Ø9.5 × 7) 안에서 육각 허브의 Ø20 턱을 누름 → 허브 플랜지를 붙이기 전에 끼움'),
    ('6', '#1f4e9c', 'M5 세레이션 플랜지 휠 너트 (1/8 RC용)   × 1',
     '림 웹을 육각 끝면에 눌러 바퀴를 고정.  너트 높이 6 전체 물림, 볼트 끝이 너트 밖으로 2.5 mm\n'
     '(타이어 바깥면보다 2.5 mm 안쪽).  10-31 M5 × 50 → 55 (전에는 3.5 mm만 물림)'),
]
tx.text(0.0, 0.985, '나사·핀  (바퀴 한쪽 기준 — 로봇 전체는 × 2)', fontsize=18, weight='bold', va='top')
y = 0.90
for n, c, head, body in rows:
    tx.text(0.012, y, n, fontsize=15, weight='bold', color='white', ha='center', va='center',
            bbox=dict(boxstyle='circle,pad=0.22', fc=c, ec='none'))
    tx.text(0.04, y, head, fontsize=14.5, weight='bold', va='center')
    tx.text(0.04, y - 0.03, body, fontsize=13, va='top', linespacing=1.4)
    y -= 0.133

tx.text(0.0, y + 0.02, '조립 순서', fontsize=16, weight='bold', va='top')
tx.text(0.0, y - 0.025,
        '1) 허브 플랜지를 모터 출력부에: ③ 핀 3개 → ② M3 × %d 6개 (머리는 나중에 육각 허브 플랜지에 덮임)\n' % LS +
        '2) ⑤ M5 볼트를 육각 허브에 끼우고, 바퀴를 육각에 끼운 뒤 ⑥ 너트 조임 (볼트 머리를 육각 렌치로 잡고)\n'
        '3) 육각 허브 + 바퀴를 허브 플랜지 Ø20 홈(B)에 맞추고, 스포크 사이로 ④ M3 × 8 6개 조임',
        fontsize=13, va='top', linespacing=1.5)

tb = fig.add_axes([0.72, 0.01, 0.27, 0.555]); tb.axis('off'); tb.set_xlim(0, 1); tb.set_ylim(0, 1)
tb.text(0.0, 0.97, '중심 맞춤 (반경 방향 하중을 받는 끼워맞춤)', fontsize=16, weight='bold', va='top')
y = 0.905
for n, txt in (('A', 'Ø11.8 스피곳 (허브 플랜지)\n→ 출력부 Ø12 × 1.5 홈'),
               ('B', 'Ø20 등록 턱 (육각 허브)\n→ 허브 플랜지 Ø20.2 × 1.6 홈'),
               ('C', 'AKE Ø42 파일럿 → 정강이 Ø42.05 포켓,\n깊이 2.5 (고정자 앞면은 판에서 1.5 띄움)')):
    tb.text(0.03, y, n, fontsize=14, weight='bold', color='white', ha='center', va='center',
            bbox=dict(boxstyle='circle,pad=0.22', fc='#2e7d32', ec='none'))
    tb.text(0.09, y + 0.018, txt, fontsize=13, va='top', linespacing=1.4)
    y -= 0.075
y -= 0.02
tb.text(0.0, y, '무게가 지나가는 길 (빨간 선)', fontsize=16, weight='bold', va='top')
steps = ['바닥', '타이어 → 림', '17 mm 육각 (물림 13)', '육각 허브', 'B 등록 턱 + ④ M3 × 8 × 6',
         '허브 플랜지', 'A 스피곳 + ③ 핀 × 3 + ② M3 × %d × 6' % LS, 'AKE 출력부 → 모터 안 출력 베어링 → 고정자',
         'C Ø42 파일럿 + ① M2.5 × 8 × 6', 'PETG 정강이 → 무릎']
y -= 0.05
for k, st in enumerate(steps):
    tb.text(0.03, y, ('↓ ' if k else '   ') + st, fontsize=13, va='top')
    y -= 0.028
y -= 0.02
tb.text(0.0, y, '하중', fontsize=16, weight='bold', va='top')
tb.text(0.0, y - 0.045,
        '정지: 로봇 %.2f kg → 바퀴당 약 %.0f N,\n'
        '     굽힘 %.0f N × 41 mm = %.1f N·m\n'
        '설계: 12 kg(짐 5 kg) 60 N × 5 = 300 N → 12.3 N·m\n'
        'calc/strength.md 3절: 허브 접합부 모두 통과\n'
        '(검토 하중에서 ② 와셔 지압만 "경계").\n'
        'AKE60 출력 베어링 정격은 비공개 → CubeMars 확인'
        % (ROBOT_KG, W_STATIC, W_STATIC, W_STATIC * 0.041),
        fontsize=13, va='top', linespacing=1.45)
y = 0.085
tb.text(0.0, y + 0.02, '색', fontsize=14, weight='bold', va='bottom')
for k, (c, txt) in enumerate(((g['petg'], 'PETG 정강이'), (g['flange'], '허브 플랜지'), (g['hexhub'], '육각 허브'),
                              (g['stator'], 'AKE 고정자'), (g['out'], 'AKE 출력부'), (g['rim'], 'RC 림'),
                              (g['ring'], 'SUS 링'), (g['washer'], '와셔'), (g['insert'], '황동 인서트'),
                              (g['pin'], '평행핀'))):
    cx, cy = 0.0 + (k % 3) * 0.34, y - (k // 3) * 0.022
    tb.add_patch(Rectangle((cx, cy - 0.008), 0.05, 0.016, fc=c, ec='k', lw=0.6))
    tb.text(cx + 0.07, cy, txt, fontsize=12, va='center')

fig.savefig(sys.argv[1] if len(sys.argv) > 1 else HERE / 'wheel_output_stack.png', dpi=80)
