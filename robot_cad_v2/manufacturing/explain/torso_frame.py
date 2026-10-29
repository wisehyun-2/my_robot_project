"""Torso frame figure: side view of the left hip plate (quote STEP outline) with the spine bar / rail sections,
and a front section through the bars and rails (tapped ends, countersunk screws, top plate bolts), to scale from
fusion/params.py.  Explanatory figure, not a manufacturing drawing.  Run make_parts.py first.

    python torso_frame.py            # -> torso_tubes_rails.png next to this script
"""
import pathlib
import sys
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / 'fusion'))
sys.path.insert(0, str(HERE.parent))
import params as P
import occ as O
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Polygon, Rectangle, Circle
F = '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc'
font_manager.fontManager.addfont(F); plt.rcParams['font.family'] = font_manager.FontProperties(fname=F).get_name()
C = dict(al='#9aa7b3', plate='#3b3f45', tap='#d98c2b', screw='#222222', bat='#3e5c8a', petg='#9fd29f', motor='#c8c8c8', top='#7bbf7b')
YI, YWB = P.YW - P.HIP_PLATE_T, P.YW - P.HIP_PLATE_T + P.HIP_WEB_T    # 164, 169
TX, TZ, TD = P.TUBE_X, P.TUBE_Z, P.TUBE_D; RX, RZ, RS = P.RAIL_X, P.RAIL_Z, P.RAIL_S
bx, by, bz = P.BATTERY; Z0 = P.TRAY_Z; ZB = P.BATTERY_Z0
lx, ly, tt = P.TOP_PLATE; ZT = RZ + RS / 2
fig = plt.figure(figsize=(19, 10.5))
# ---------------- (1) side view: hip plate outline (quote STEP) + member sections
ax = fig.add_axes([0.01, 0.04, 0.33, 0.86]); ax.set_aspect('equal'); ax.axis('off')
hp = O.solids(O.read_step(HERE.parent / 'step' / 'hip_plate_L.step'))[0]
vis, _ = O.project(hp, (0, 0, 1), (1, 0, 0))        # from the robot centre side: X = robot front, Y = up
for pl in vis:
    xs, ys = zip(*pl); ax.plot(xs, ys, color='0.35', lw=0.7)
ax.add_patch(Circle((0, 0), 34.5, fc=C['motor'], ec='k', lw=0.8, alpha=0.6, zorder=0))
ax.text(0, -8, 'hip 모터\n(판 안쪽)', ha='center', fontsize=9)
ax.add_patch(Rectangle((P.BATTERY_X - bx / 2, ZB), bx, bz, fc=C['bat'], ec='k', lw=0.8, alpha=0.9))
ax.text(P.BATTERY_X, ZB + bz / 2, '배터리\n(310 mm가\n좌우 방향)', ha='center', va='center', color='white', fontsize=9)
ax.add_patch(Rectangle((-TX - 9, Z0), 2 * TX + 18, P.TRAY_T, fc=C['petg'], ec='k', lw=0.8))
ax.text(TX + 12, Z0 + 1, '배터리 트레이(PETG)', fontsize=9, va='center')
ax.add_patch(Rectangle((-lx / 2, ZT), lx, tt, fc=C['top'], ec='k', lw=0.8))
ax.text(0, ZT + tt + 3, '윗판 (PETG, 짐·로봇팔)', ha='center', fontsize=9)
for sx in (-1, 1):                                  # solid bars, tapped end (M5 / M4 tap drill) seen end-on
    ax.add_patch(Circle((sx * TX, TZ), TD / 2, fc=C['al'], ec='k', lw=1.2))
    ax.add_patch(Circle((sx * TX, TZ), P.TAP_DRILL['M5'] / 2, fc='white', ec='k', lw=0.6))
    ax.add_patch(Rectangle((sx * RX - RS / 2, RZ - RS / 2), RS, RS, fc=C['al'], ec='k', lw=1.2))
    ax.add_patch(Circle((sx * RX, RZ), P.TAP_DRILL['M4'] / 2, fc='white', ec='k', lw=0.6))
ax.annotate('척추 봉 Ø16 통짜 (앞·뒤 2개)\n배터리 + 트레이를 받침', xy=(TX + 6, TZ - 4), xytext=(40, -58), fontsize=10,
            arrowprops=dict(arrowstyle='->'))
ax.annotate('', xy=(-TX - 6, TZ - 4), xytext=(30, -50), arrowprops=dict(arrowstyle='->'))
ax.annotate('위 레일 15 × 15 통짜 (앞·뒤 2개)\n윗판을 받침', xy=(RX + 8, RZ), xytext=(78, 165), fontsize=10,
            arrowprops=dict(arrowstyle='->'))
ax.annotate('', xy=(-RX - 8, RZ + 4), xytext=(70, 168), arrowprops=dict(arrowstyle='->'))
ax.text(-75, 172, '← 로봇 뒤 (-x)', fontsize=10); ax.text(55, 182, '로봇 앞 (+x) →', fontsize=10)
ax.set_xlim(-80, 135); ax.set_ylim(-75, 190)
ax.set_title('① 옆에서 본 몸통 (왼쪽 hip 판 실제 외곽 + 부재 단면, 1:1 비율)', fontsize=12, loc='left')
# ---------------- (2) front section through the bar (x = +47) and the rail (x = +52)
bx2 = fig.add_axes([0.36, 0.04, 0.63, 0.86]); bx2.set_aspect('equal'); bx2.axis('off')
def R(y0, y1, z0, z1, c, **kw): bx2.add_patch(Rectangle((y0, z0), y1 - y0, z1 - z0, fc=c, ec=kw.pop('ec', 'k'), lw=kw.pop('lw', 0.8), **kw))
for sy in (-1, 1):
    # hip plate web (dark) incl. top bar, register boss and hub boss at the motor
    R(sy * YI, sy * YWB, -50, 145, C['plate']) if sy > 0 else R(-YWB, -YI, -50, 145, C['plate'])
    y0, y1 = sorted((sy * P.YW, sy * YWB)); R(y0, y1, -26, 26, C['plate'])
    # hip motor inside
    y0, y1 = sorted((sy * (YI - 1.5), sy * (YI - 1.5 - 33))); R(y0, y1, -34.5, 34.5, C['motor'], alpha=0.7)
# solid bar, both ends tapped M5 (params.END_TAP) + M5 countersunk screws
R(-YI, YI, TZ - TD / 2, TZ + TD / 2, C['al'], lw=1.2)
for sy in (-1, 1):
    y0, y1 = sorted((sy * (YI - P.END_TAP['M5'][0]), sy * YI)); R(y0, y1, TZ - 2.5, TZ + 2.5, C['tap'], lw=0.5)
    # M5 x 12 countersunk: head cone in the web outer face, shank into the tapped bar end
    tip = YWB - 12 + 0.7
    y0, y1 = sorted((sy * tip, sy * YWB)); R(y0, y1, TZ - 2.5, TZ + 2.5, C['screw'], lw=0.5)
    bx2.add_patch(Polygon([(sy * YWB, TZ - 5.6), (sy * (YWB - 2.9), TZ - 2.7), (sy * (YWB - 2.9), TZ + 2.7), (sy * YWB, TZ + 5.6)],
                          fc=C['screw'], ec='k', lw=0.5))
# solid rail, both ends tapped M4 + M4 countersunk screws
R(-YI, YI, RZ - RS / 2, RZ + RS / 2, C['al'], lw=1.2)
for sy in (-1, 1):
    y0, y1 = sorted((sy * (YI - P.END_TAP['M4'][0]), sy * YI)); R(y0, y1, RZ - 2, RZ + 2, C['tap'], lw=0.5)
    y0, y1 = sorted((sy * (YWB - 12 + 0.6), sy * YWB)); R(y0, y1, RZ - 2, RZ + 2, C['screw'], lw=0.5)
    bx2.add_patch(Polygon([(sy * YWB, RZ - 4.5), (sy * (YWB - 2.3), RZ - 2.2), (sy * (YWB - 2.3), RZ + 2.2), (sy * YWB, RZ + 4.5)],
                          fc=C['screw'], ec='k', lw=0.5))
# top plate + M4 x 25 through bolts with nylock nuts under the rail
R(-ly / 2, ly / 2, ZT, ZT + tt, C['top'])
for y in (-100.0, -33.3, 33.3, 100.0):
    R(y - 2, y + 2, ZT + tt - 25 + 4 - 2, ZT + tt - 4 + 0.01, C['screw'], lw=0.4)
    R(y - 3.5, y + 3.5, ZT + tt - 4, ZT + tt, C['screw'], lw=0.4)                     # head in counterbore
    R(y - 3.5, y + 3.5, RZ - RS / 2 - 5, RZ - RS / 2, '#555555', lw=0.4)               # nylock nut
# tray + battery on the bars
R(-by / 2 - 4, by / 2 + 4, Z0, Z0 + P.TRAY_T, C['petg'])
R(-by / 2, by / 2, ZB, ZB + bz, C['bat'], alpha=0.9)
bx2.text(0, ZB + bz / 2, '배터리 24 V 15 Ah (75 × 310 × 75)', ha='center', va='center', color='white', fontsize=11)
bx2.text(0, Z0 - 3.5, '배터리 트레이 (봉 2개 위에 얹힘, 벨크로로 봉에 묶음)', ha='center', va='top', fontsize=9)
bx2.text(0, ZT + tt + 2.5, '윗판 (PETG 120 × 210 × 6, M4 인서트 격자)', ha='center', fontsize=10)
for sy, lab in ((-1, '오른쪽 hip 판'), (1, '왼쪽 hip 판')):
    bx2.text(sy * 166.5, -58, lab + '\n(6061, 웹 5 mm)', ha='center', va='top', fontsize=10)
    bx2.text(sy * 145, 0, 'hip\n모터', ha='center', va='center', fontsize=9)
def call(xy, xt, txt, c='k'):
    bx2.annotate(txt, xy=xy, xytext=xt, fontsize=10.5, color=c, va='center', arrowprops=dict(arrowstyle='->', color=c, lw=0.9))
call((-120, TZ), (-325, -20), '척추 봉 Ø16 통짜, 길이 328 (앞·뒤 2개)\n→ 배터리 무게를 양쪽 hip 판에 나눠 줌')
call((-120, RZ), (-325, 185), '위 레일 15 × 15 통짜, 길이 328 (앞·뒤 2개)\n→ 윗판(짐·로봇팔) 무게를 hip 판에 전달')
call((153, TZ + 2), (185, 75), '봉 끝에 바로 낸 M5 탭 (깊이 11, 마개·접착 없음)', '#b36b00')
call((155, RZ + 1.5), (185, 160), '레일 끝에 바로 낸 M4 탭 (깊이 9, 마개·접착 없음)', '#b36b00')
call((168, TZ - 4.5), (185, 25), 'M5 × 12 접시머리 볼트\n(hip 판 바깥에서 → 봉 끝 탭, 물림 약 7.5)')
call((168, RZ - 4), (185, 118), 'M4 × 12 접시머리 볼트\n(hip 판 바깥에서 → 레일 끝 탭, 물림 약 7.5)')
call((100, ZT + 2), (185, 200), 'M4 × 25 관통 볼트 8개 + 나일론 너트\n(윗판 → 레일 관통)')
bx2.annotate('', xy=(-YI, -42), xytext=(YI, -42), arrowprops=dict(arrowstyle='<->'))
bx2.text(0, -46, '봉·레일 길이 328 = 두 hip 판 안쪽 면 사이 간격 → 4개가 같은 길이여야 두 판이 평행', ha='center', va='top', fontsize=10.5, color='#b00')
bx2.set_xlim(-330, 300); bx2.set_ylim(-75, 210)
bx2.set_title('② 앞에서 자른 몸통 (봉·레일 중심을 지나는 단면, 실제 비율) — 두 hip 판을 통짜 봉 2개 + 레일 2개가 잇는 사다리 구조', fontsize=12, loc='left')
fig.savefig(sys.argv[1] if len(sys.argv) > 1 else HERE / 'torso_tubes_rails.png', dpi=95, bbox_inches='tight')
