"""Single-part STEP files for the machining quote (MakeIT etc.), plus DXF for the laser-cut rings.

    python -m pip install cadquery-ocp ezdxf matplotlib
    python make_parts.py        # -> step/*.step, dxf/*.dxf
    python drawings.py          # -> drawings/*.pdf

Each part is in its own frame (origin and axes in PARTS[...]['frame']), one body per file, so the quote
site prices it as a single part.  Holes to be tapped are modelled at the tap-drill diameter; threads,
tolerances and finishes are on the PDF drawings.

The hip plates are read from the Fusion export (`../export/parts_step`) and brought to the current
`fusion/params.py` / `torso.py` (2026-10-29 fixes, `../로봇_부품_검토사항.md` 3·7절), so the files are right
whether or not Fusion has been rebuilt since; the re-cuts are idempotent:
  * Ø42 pilot pocket -> params.PILOT_D_HIP (Ø42 H7 at mid tolerance; the 10-28 export has Ø42.05);
  * M5 / M4 countersinks -> 90 deg, params.CSK_M5 / CSK_M4 (ISO 10642 theoretical dk; export ~74 / 69 deg);
  * no lightening holes (removed 10-29 to cut machining cost): the five Ø22 holes, the 30 × 58 window and
    the two small top windows are filled, and the shell M3 tapped holes at x ±40, z +140 re-drilled (in the
    10-28 export their centres fall inside the top windows).
The other parts are built here from params: magnet spacer without the AKE magnet body, spine bar and rail
(solid, tap-drill holes params.END_TAP in both ends; the 10-28 tubes with glued end plugs are gone) and the
clamp rings (stator ring ID from params.RING_STATOR).
"""
import math
import pathlib
import sys

import ezdxf

import occ as O

HERE = pathlib.Path(__file__).resolve().parent
EXPORT = HERE.parent / 'export' / 'parts_step'
sys.path.insert(0, str(HERE.parent / 'fusion'))
import params as P  # noqa: E402  (plain dimensions, no Fusion API)

STEP_DIR = HERE / 'step'
DXF_DIR = HERE / 'dxf'

ZH = P.Z_HIP                                   # hip axis height in the export (robot z)
Y_IN = P.YW - P.HIP_PLATE_T                    # 164: web inner face (left plate, robot y)
Y_WEB = Y_IN + P.HIP_WEB_T                     # 169: web outer face
TORSO_LEN = 2 * Y_IN                           # 328: tube / rail length between the hip plates
RAIL_HOLES = tuple(Y_IN + y for y in (-100.0, -33.3, 33.3, 100.0))   # from one rail end
CSK = {'M5': (P.TUBE_X, P.TUBE_Z) + P.CSK_M5, 'M4': (P.RAIL_X, P.RAIL_Z) + P.CSK_M4}   # x, z, hole Ø, csk Ø

# name -> file, quantity and what goes on the drawing / order
PARTS = {
    'hip_plate_L': dict(qty=1, material='AL6061-T6 판재 10t 이상(가능하면 12t)', process='3축 MCT (양면 2공정)',
                        finish='검정 아노다이징(황산, 10~15 µm)',
                        frame='원점 hip 축, X 로봇 앞(+x), Y 위(+z), Z 로봇 중심 쪽(Z=0 허브 바깥 면, Z=9.5 등록 보스 면)'),
    'hip_plate_R': dict(qty=1, material='AL6061-T6 판재 10t 이상(가능하면 12t)', process='3축 MCT (양면 2공정)',
                        finish='검정 아노다이징(황산, 10~15 µm)',
                        frame='원점 hip 축, X 로봇 뒤(−x), Y 위(+z), Z 로봇 중심 쪽(Z=0 허브 바깥 면, Z=9.5 등록 보스 면)'),
    'magnet_spacer': dict(qty=6, material='AL6061-T6 환봉 Ø10 (비자성)', process='선반', finish='없음(탈지)',
                          frame='Z 축, Z=0 스피곳 끝'),
    'spine_tube': dict(qty=2, material='AL6061-T6 환봉 Ø16 (6063-T5/T6 허용)', process='선반 (절단 + 끝면 면삭 + 양 끝 M5 탭)',
                       finish='없음', frame='X 축, X=0 한쪽 끝면'),
    'rail': dict(qty=2, material='AL6061-T6 각봉 15 × 15 (6063-T5/T6 허용)', process='3축 MCT (끝면 면삭 + 양 끝 M4 탭 + 구멍)',
                 finish='없음', frame='X 길이 방향(X=0 한쪽 끝면), Z 구멍 방향'),
    'clamp_ring_output': dict(qty=4, material='SUS304 2B %gt' % P.RING_T, process='판금 레이저 절단 (대안: 와이어커팅)',
                              finish='없음(버 제거)', frame='Z 두께 방향, 원점 링 중심'),
    'clamp_ring_stator': dict(qty=4, material='SUS304 2B %gt' % P.RING_T, process='판금 레이저 절단 (대안: 와이어커팅)',
                              finish='없음(버 제거)', frame='Z 두께 방향, 원점 링 중심'),
}


# ----------------------------------------------------------------------------- hip plate (export + fixes)
def hip_plate(s):
    """s = +1 left, -1 right.  Export frame: x = robot y (thickness), y = robot z, z = robot x."""
    sh = O.solids(O.read_step(EXPORT / ('hip_plate_%s.step' % ('L' if s > 0 else 'R'))))[0]

    def xr(a, b):                                   # robot-y interval of the left plate -> this side, ordered
        return (min(s * a, s * b), max(s * a, s * b))

    def ycyl(z, x, ya, yb, r):                      # cylinder across the plate at robot (x, z above the hip)
        t0, t1 = xr(ya, yb)
        return O.cyl('x', ZH + z, x, t0, t1, r)

    def ycone(z, x, y_small, y_big, r_small, r_big):
        if s * y_small < s * y_big:                 # cone grows towards +x in the export
            return O.cone('x', ZH + z, x, s * y_small, s * y_big, r_small, r_big)
        return O.cone('x', ZH + z, x, s * y_big, s * y_small, r_big, r_small)

    # Ø42 H7 pilot pocket: fill (covers the 10-28 export's Ø42.05), re-cut (the floor stays at the stator ring face)
    sh = O.fuse(sh, ycyl(0, 0, Y_IN - 1.5, P.Y_RING_HIP, 21.03))
    sh = O.cut(sh, ycyl(0, 0, Y_IN - 2.0, P.Y_RING_HIP, P.PILOT_D_HIP / 2))
    # 90 deg countersinks on the web outer face (ISO 10642 heads): fill the old seat (r <= 4.65), re-drill, re-sink
    for (x, z, dh, dk) in CSK.values():
        for sx in (-1, 1):
            sh = O.fuse(sh, ycyl(z, sx * x, Y_IN, Y_WEB, dh / 2 + 2.3))
            sh = O.cut(sh, ycyl(z, sx * x, Y_IN - 1, Y_WEB + 1, dh / 2))
            depth = (dk - dh) / 2                    # 45 deg half angle
            sh = O.cut(sh, ycone(z, sx * x, Y_WEB - depth, Y_WEB + 1, dh / 2, dk / 2 + 1))
    # lightening holes filled: five Ø22 at r 50 (web, plus the 1 mm they notch from the Ø80 boss rim), the
    # 30 x 58 window, the two top windows (z 129-141 covers the 10-28 and 10-29 exports); shell M3 (tap drill)
    # re-drilled at x ±40, z 140
    t0, t1 = xr(Y_IN, Y_WEB)
    boss = ycyl(0, 0, Y_IN - 1.5, Y_IN, 40.0)
    for a in (0.0, 90.0, 180.0, 255.0, 300.0):
        x, z = 50.0 * math.cos(math.radians(a)), 50.0 * math.sin(math.radians(a))
        sh = O.fuse(sh, ycyl(z, x, Y_IN, Y_WEB, 11.0))
        sh = O.fuse(sh, O.common(ycyl(z, x, Y_IN - 1.5, Y_IN, 11.0), boss))
    sh = O.fuse(sh, O.box(t0, t1, ZH + 60.0, ZH + 118.0, -15.0, 15.0))
    for sx in (-1, 1):
        x0, x1 = sorted((sx * 30.0, sx * 43.0))
        sh = O.fuse(sh, O.box(t0, t1, ZH + 129.0, ZH + 141.0, x0, x1))
        sh = O.cut(sh, ycyl(140.0, sx * 40.0, Y_IN - 1, Y_WEB + 1, 1.25))
    # Ø63 x 0.5 relief in the Ø80 boss face (10-30, params.HIP_STATOR_RELIEF): the AKE stator front clears the boss
    rd, rh = P.HIP_STATOR_RELIEF
    sh = O.cut(sh, ycyl(0, 0, Y_IN - 2.5, Y_IN - 1.5 + rh, rd / 2))
    sh = O.clean(sh)
    # local frame: X = robot x (left) / -x (right), Y = robot z - ZH, Z = inward, 0 at the hub outer face
    if s > 0:
        rows = [(0, 0, 1, 0), (0, 1, 0, -ZH), (-1, 0, 0, P.YW)]
    else:
        rows = [(0, 0, -1, 0), (0, 1, 0, -ZH), (1, 0, 0, P.YW)]
    return O.transform(sh, rows)


# ----------------------------------------------------------------------------- turned / bar parts
def magnet_spacer():
    (sd, sp_l), (pd, pk_d) = P.SPACER_SPIGOT, P.SPACER_POCKET   # same as fusion/vendor.py magnet_spacer()
    sp_r, pk_r = sd / 2, pd / 2
    s = O.cyl('z', 0, 0, 0, sp_l + 0.01, sp_r)
    s = O.fuse(s, O.cyl('z', 0, 0, sp_l, sp_l + P.SPACER_H, 4.5))
    s = O.cut(s, O.cyl('z', 0, 0, sp_l + P.SPACER_H - pk_d, sp_l + P.SPACER_H + 1, pk_r))
    return O.clean(s)


def end_taps(b, size):
    """Tap-drill holes (params.END_TAP drill depth) on the X axis in both ends of a TORSO_LEN bar."""
    depth, r = P.END_TAP[size][0], P.TAP_DRILL[size] / 2
    b = O.cut(b, O.cyl('x', 0, 0, -1, depth, r))
    return O.cut(b, O.cyl('x', 0, 0, TORSO_LEN - depth, TORSO_LEN + 1, r))


def spine_tube():
    return O.clean(end_taps(O.cyl('x', 0, 0, 0, TORSO_LEN, P.TUBE_D / 2), 'M5'))


def rail():
    a = P.RAIL_S / 2
    b = O.box(0, TORSO_LEN, -a, a, -a, a)
    for x in RAIL_HOLES:
        b = O.cut(b, O.cyl('z', x, 0, -a - 1, a + 1, 2.2))
    return O.clean(end_taps(b, 'M4'))


def ring(od, idia, pcd, hole, a0):
    import math
    r = O.cut(O.cyl('z', 0, 0, 0, P.RING_T, od / 2), O.cyl('z', 0, 0, -1, P.RING_T + 1, idia / 2))
    for k in range(6):
        t = math.radians(a0 + 60 * k)
        r = O.cut(r, O.cyl('z', pcd / 2 * math.cos(t), pcd / 2 * math.sin(t), -1, P.RING_T + 1, hole / 2))
    return O.clean(r)


RING_SPECS = {   # od, id, pcd, hole, first hole angle (deg)
    'clamp_ring_output': (P.RING_OUT[0], P.RING_OUT[1], P.AKE_PCD_OUT, 3.4, 0.0),
    'clamp_ring_stator': (P.RING_STATOR[0], P.RING_STATOR[1], P.AKE_PCD_RING, 2.8, 30.0),
}


def ring_dxf(name, path):
    import math
    od, idia, pcd, hole, a0 = RING_SPECS[name]
    doc = ezdxf.new('R2010', setup=True)
    doc.header['$INSUNITS'] = 4                     # mm
    msp = doc.modelspace()
    for r in (od / 2, idia / 2):
        msp.add_circle((0, 0), r)
    for k in range(6):
        t = math.radians(a0 + 60 * k)
        msp.add_circle((pcd / 2 * math.cos(t), pcd / 2 * math.sin(t)), hole / 2)
    doc.saveas(path)


def build(name):
    if name.startswith('hip_plate'):
        return hip_plate(1 if name.endswith('L') else -1)
    if name in RING_SPECS:
        return ring(*RING_SPECS[name])
    return globals()[name]()


def main():
    STEP_DIR.mkdir(exist_ok=True)
    DXF_DIR.mkdir(exist_ok=True)
    for name in PARTS:
        sh = build(name)
        assert len(O.solids(sh)) == 1, name
        O.write_step(sh, STEP_DIR / (name + '.step'), name)
        b = O.bbox(sh)
        print('%-18s %7.2f x %7.2f x %6.2f mm  %9.1f mm3' % (name, b[1] - b[0], b[3] - b[2], b[5] - b[4], O.volume(sh)))
        if name in RING_SPECS:
            ring_dxf(name, DXF_DIR / (name + '.dxf'))


if __name__ == '__main__':
    main()
