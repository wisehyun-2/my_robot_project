"""v5 power board: mjbots power_dist r4.5b instead of the v4 mjpower-ss (11-08, user: the CAN-FD port for the soft power-down,
so the Pi can shut down before the switch cuts the power).  The v4 files stay as they are; v5run.load() calls patch(M), which
swaps the board in the loaded v4 modules (vendor master, deck bosses, board screws, lumped mass).

Board (mjbots power_dist r4.3b mechanical drawing + STEP, which mjbots lists for the r4.5b; 2021-04-07): 80 x 50, PCB 1.5,
4 x M2.5 holes on 74 x 44 (mjpower-ss: 70 x 50, PCB 1.6, 64 x 44), XT90-M input at -X, 6 x XT30-F output at +X, 2 x JST PH-3
CAN next to the XT90, PH-4 switch on the +Y edge, ZH-6 SWD and PH-5 GPIO on the -Y edge, 36 g (shop page).  The connector
boxes below were read off the STEP in Fusion (11-08).  Board frame as for the mjpower-ss: PCB centre at the origin, bottom
face Z = 0, X along 80 with the XT90 at -X, Y along 50 (= -Z of the Y-up STEP) - the same view as the mjbots drawing.
The deck keeps the mjpower-ss place and turn (torso.MJ_C, MJ_ALONG_X): the board is 5 mm longer at each end.

No Fusion import at module level, so the numbers can be read outside Fusion too.
"""
import math

STEP = r'C:\Users\User\Desktop\power_dist_r45b\20210407-power_dist-r43b-mechanical.step'
NAME = 'power_dist_r4.5b'
OLD_NAME = 'mjpower_ss'
MASS = 0.036                 # kg (mjbots shop page)
T = 1.5                      # PCB
HOLES = (37.0, 22.0)         # +- X, +- Y of the 4 M2.5 holes (74 x 44)
SIZE = (80.0, 50.0)

# connectors in the board frame (mm, from the STEP): XT30-F output centres (silkscreen P-numbers not checked: no wiring in
# v5), housing top; XT90-M face centre at -X; PH-4 switch, PH-3 CAN, ZH-6, PH-5 header top centres
OUT = {'P1': (28.0, 12.0), 'P2': (36.0, 12.0), 'P3': (28.0, 0.0), 'P4': (36.0, 0.0), 'P5': (28.0, -12.0), 'P6': (36.0, -12.0)}
XT30_TOP = 11.0
XT90 = (-40.0, 6.0, 7.0)
PH4 = (-5.4, 22.0, 8.5)
CAN = ((-36.3, -10.0, 8.5), (-27.7, -10.0, 8.5))
SWD = (-17.6, -22.4, 8.5)
GPIO = (-3.4, -22.4, 8.65)

# the STEP is one solid; it is split into the PCB and the connector groups (board frame boxes, disjoint)
GROUPS = (('pcb (STEP)', (-41.0, 41.0, -26.0, 26.0, -5.0, T), 'FR4 PCB', 'pcb black'),
          ('XT90-M', (-41.0, -12.5, -4.7, 17.0, T, 14.0), 'Nylon', 'connector yellow'),
          ('XT30-F x6', (24.5, 39.5, -18.0, 18.0, T, 12.0), 'Nylon', 'connector yellow'),
          ('PH-3 CAN x2', (-39.5, -24.5, -16.0, -4.7, T, 10.0), 'Nylon', 'connector white'),
          ('PH-4 switch', (-12.0, 1.0, 19.0, 25.5, T, 10.0), 'Nylon', 'connector white'),
          ('ZH-6 SWD', (-24.0, -11.5, -25.5, -19.5, T, 10.0), 'Nylon', 'connector white'),
          ('PH-5 GPIO', (-11.0, 4.0, -25.5, -19.5, T, 10.0), 'Nylon', 'connector white'))


def master(V, C):
    """The power_dist master in the hidden _library (as vendor.mjpower_ss: the STEP turned to the board frame and split)."""
    import adsk.core
    comp = C.find_comp(NAME)
    if comp is not None:
        return comp
    comp = V.import_step(STEP, NAME)
    rot = adsk.core.Matrix3D.create()
    rot.setToRotation(math.pi / 2, adsk.core.Vector3D.create(1, 0, 0), adsk.core.Point3D.create(0, 0, 0))
    src = [b for b in comp.bRepBodies]
    whole = None
    for b in src:
        t = C.TBM.copy(b)
        C.TBM.transform(t, rot)
        whole = C.union(whole, t)
    for b in src:
        b.deleteMe()
    vol = 0.0
    for name, (x0, x1, y0, y1, z0, z1), mat, look in GROUPS:
        part = C.TBM.copy(whole)
        C.inter(part, V.B.abox(x0, x1, y0, y1, z0, z1))
        if part.faces.count:
            vol += part.volume
            C.add_body(comp, part, name, mat, look)
    if abs(vol - whole.volume) > 1e-3 * whole.volume:
        raise RuntimeError('power_dist split lost volume: %.1f of %.1f mm3' % (vol * 1000, whole.volume * 1000))
    return comp


def drop_old_master(C):
    """The mjpower-ss master left in _library after relayout(): removed once nothing uses it."""
    comp = C.find_comp(OLD_NAME)
    if comp is None:
        return 0
    n = 0
    for o in list(C.library().component.occurrences):
        if o.component == comp:
            o.deleteMe()
            n += 1
    return n


def patch(M):
    """Swap the board in the loaded v4 modules (they are reloaded by every v5run.load(), so this runs once per load)."""
    V, C, TO, F = M['vendor'], M['cadlib'], M['torso'], M['fasteners']
    V.mjpower_ss = lambda: master(V, C)                      # vendor.build_all() makes the board master through this name
    V.MJPOWER_T, V.MJPOWER_OUT, V.MJPOWER_XT30_TOP, V.MJPOWER_XT90, V.MJPOWER_PH4 = T, OUT, XT30_TOP, XT90, PH4

    def holes():
        m = TO.mjpower_frame()
        return [C.apply(m, (sx * HOLES[0], sy * HOLES[1], 0))[:2] for sx in (-1, 1) for sy in (-1, 1)]
    TO.mjpower_holes_robot = holes                          # deck bosses + inserts + board screws (module globals, read at call)

    positions = F.positions

    def positions_v5():
        """The board screws sit on a 1.5 PCB (fasteners.positions puts them on the mjpower-ss 1.6)."""
        out = []
        for rec in positions():
            if rec[1] == 'FX DIN 7984 M2.5x6 mjpower':
                o = rec[2]
                rec = (rec[0], rec[1], (o[0], o[1], o[2] - 1.6 + T)) + tuple(rec[3:])
            out.append(rec)
        return out
    F.positions = positions_v5
    M['check'].LUMPED[NAME] = (MASS, 0.0, 0.0)
    return M
