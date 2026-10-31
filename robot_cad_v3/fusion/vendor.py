"""Purchased parts: imported STEP models (AKE60-8, moteus r4.5, pi3hat r4.4, mjpower-ss r2) and modelled
stand-ins (Raspberry Pi 4B, battery, connectors, M365 tyre, fan, switch, magnet spacer, standoffs).

Every part is a master component in the hidden _library at the identity; instances are placed with
cadlib.instance().  Part frames:
  * AKE60 (CubeMars STEP): +y = motor axis to the back, output face y = -8.5, lead exit at +x.
  * moteus (mjbots r4.5 STEP): PCB motor-side face y = 0 (components to -y = motor side), encoder IC at
    (0, 0, 8), connector edge at z = 43.
  * pi3hat (mjbots r4.4 STEP): PCB y 0..1.5, holes (-24.75 / 24.25, +-29), CAN headers towards +x,
    XT30 at +z, the Raspberry Pi USB end towards -z.
  * modelled parts: documented in each function.
"""
import math
import os

import adsk.core
import adsk.fusion

import cadlib as C
import params as P

AKE60_STEP = r'C:\Users\User\Desktop\ake60\AKE60-8-KV80.STEP'
MOTEUS_STEP = r'C:\Users\User\Desktop\moteus_r45\20210124-moteus-controller-r45-mechanical.step'
PI3HAT_STEP = r'C:\Users\User\Desktop\pihat\20210424-pi3hat-r4-4-mechanical.step'
MJPOWER_STEP = r'C:\Users\User\Desktop\mjpower_ss\20240827-mjpower-ss-r2.step'

# bodies of the CubeMars STEP that turn with the output carrier (the rest is stator + spinning bell)
AKE_OUTPUT_BODIES = ('본체40', '본체36', '본체37', '본체43', '본체44', '본체52')

B = C.Builder(robot=False)          # part-frame primitives (mm)


def import_step(path, name):
    lib = C.library().component
    old = C.find_comp(name)
    if old is not None:
        return old
    im = C.app().importManager
    n0 = lib.occurrences.count
    im.importToTarget(im.createSTEPImportOptions(path), lib)
    occ = lib.occurrences.item(n0)
    occ.component.name = name
    return occ.component


def ake60():
    """Two masters from the CubeMars STEP: stator + spinning bell (fixed to the mount) and the output
    carrier (fixed to the driven link)."""
    st = import_step(AKE60_STEP, 'AKE60-8_KV80_stator')
    out = import_step(AKE60_STEP, 'AKE60-8_KV80_output')
    for b in [b for b in st.bRepBodies if b.name in AKE_OUTPUT_BODIES]:
        b.deleteMe()
    for b in [b for b in out.bRepBodies if b.name not in AKE_OUTPUT_BODIES]:
        b.deleteMe()
    return st, out


def moteus():
    return import_step(MOTEUS_STEP, 'moteus_r4.11')


def pi3hat():
    return import_step(PI3HAT_STEP, 'pi3hat_r4.5')


# ----------------------------------------------------------------------------- modelled boards
def rpi4b():
    """Raspberry Pi 4B from the official mechanical drawing.  Frame: X along the 85 mm edge from the
    end opposite the USB / Ethernet ports, Y along the 56 mm edge from the port side of USB-C / HDMI,
    Z up, PCB bottom at Z = 0."""
    comp = C.master('raspberry_pi_4b')
    t = 1.4
    pcb = B.rbox((42.5, 28.0, t / 2), (1, 0, 0), (0, 1, 0), 85.0, 56.0, t, 3.0)
    for (x, y) in ((3.5, 3.5), (61.5, 3.5), (3.5, 52.5), (61.5, 52.5)):
        C.cut(pcb, B.zcyl(x, y, -1, t + 1, 1.35))
    C.add_body(comp, pcb, 'pcb', 'FR4 PCB', 'pcb green')
    met = B.abox(65.5, 87.5, 37.6, 53.6, t, t + 13.5)                       # Ethernet
    C.union(met, B.abox(69.5, 87.0, 20.3, 33.4, t, t + 16.0))              # USB 3
    C.union(met, B.abox(69.5, 87.0, 2.3, 15.4, t, t + 16.0))               # USB 2
    C.union(met, B.abox(6.8, 15.8, -1.2, 6.3, t, t + 3.2))                 # USB-C
    C.union(met, B.abox(22.5, 29.5, -1.0, 6.5, t, t + 3.0))                # micro HDMI 0
    C.union(met, B.abox(36.0, 43.0, -1.0, 6.5, t, t + 3.0))                # micro HDMI 1
    C.union(met, B.abox(22.0, 37.0, 25.0, 40.0, t, t + 2.4))               # SoC lid
    C.add_body(comp, met, 'connectors', 'Connector', 'steel')
    hdr = B.abox(7.1, 57.9, 49.95, 55.05, t, t + 8.5)                      # 40-pin header
    C.union(hdr, B.abox(50.0, 57.0, -2.0, 12.0, t, t + 6.0))               # audio jack
    C.union(hdr, B.abox(2.0, 15.0, 22.0, 34.0, -1.4, 0.0))                 # micro SD (under)
    C.add_body(comp, hdr, 'header', 'Nylon', 'black')
    return comp


def mjpower_ss():
    """mjbots mjpower-ss r2 from its STEP (20240827-mjpower-ss-r2.step, one solid; 10-31, replaces the hand-made
    stand-in whose XT90 / PH-4 blocks covered two mounting holes).  70 x 50, 4 x Ø2.5 holes on 64 x 44 (M2.5).
    The STEP is Y-up (PCB Y 0..1.6, X along 70, Z along 50); it is turned to the board frame used here (PCB
    centre at the origin, bottom face Z = 0, X along 70 with the XT90 at -X, Y along 50: Y = -Z_step) and split
    into the PCB and the connector groups so plugs can be told apart (check.RECEPTACLES)."""
    comp = C.find_comp('mjpower_ss')
    if comp is not None and not any(b.name == 'pcb (STEP)' for b in comp.bRepBodies):
        for o in list(C.library().component.occurrences):        # the old hand-made master: replace it
            if o.component == comp:
                o.deleteMe()
        comp = None
    if comp is not None:
        return comp
    comp = import_step(MJPOWER_STEP, 'mjpower_ss')
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
    t = MJPOWER_T
    groups = [('pcb (STEP)', (-36.0, 36.0, -26.0, 26.0, -5.0, t), 'FR4 PCB', 'pcb black'),
              ('XT90-M', (-36.0, -7.0, -6.5, 16.5, t, 14.0), 'Nylon', 'connector yellow'),
              ('XT30-F x6', (20.5, 34.5, -18.0, 18.0, t, 12.0), 'Nylon', 'connector yellow'),
              ('slide switch', (-30.5, -15.5, -16.0, -6.0, t, 13.0), 'Nylon', 'black'),
              ('PH-4 switch', (-26.5, -12.5, -24.5, -18.0, t, 10.0), 'Nylon', 'connector white'),
              ('MOSFETs', (4.5, 15.5, -21.0, 21.0, t, 8.0), 'FR4 PCB', 'black')]
    vol = 0.0
    for name, (x0, x1, y0, y1, z0, z1), mat, look in groups:
        part = C.TBM.copy(whole)
        C.inter(part, B.abox(x0, x1, y0, y1, z0, z1))
        if part.faces.count:
            vol += part.volume
            C.add_body(comp, part, name, mat, look)
    if abs(vol - whole.volume) > 1e-3 * whole.volume:
        raise RuntimeError('mjpower-ss split lost volume: %.1f of %.1f mm3' % (vol * 1000, whole.volume * 1000))
    return comp


# mjpower-ss r2 connectors in the board frame (from the STEP): XT30-F outputs (centre; P1 .. P6 as on the silkscreen),
# XT30-F housing top, XT90-M face centre at the -X edge, PH-4 switch header top centre
MJPOWER_T = 1.6
MJPOWER_OUT = {'P1': (24.0, 12.0), 'P2': (31.0, 12.0), 'P3': (24.0, 0.0), 'P4': (31.0, 0.0),
               'P5': (24.0, -12.0), 'P6': (31.0, -12.0)}
MJPOWER_XT30_TOP = 10.9
MJPOWER_XT90 = (-35.0, 4.9, 6.85)
MJPOWER_PH4 = (-19.4, -21.4, 8.3)


def battery():
    """24 V 15 Ah pack 75 x 310 x 75, 1.8 kg.  Frame: centre of the pack, X 75, Y 310, Z 75;
    lead (2 x 12 AWG, 120 mm) and XT60-M leave the +Y end face."""
    comp = C.master('battery_24V_15Ah')
    bx, by, bz = P.BATTERY
    cell = B.rbox((0, 0, 0), (1, 0, 0), (0, 0, 1), bx, bz, by, 6.0)
    b = C.add_body(comp, cell, 'pack', None, 'battery')
    b.material = C.custom_density_material('battery 24V 15Ah', P.BATTERY_MASS / (b.volume * 1e-6))
    return comp


# ----------------------------------------------------------------------------- connectors
def xt30_plug(name='XT30U plug'):
    """XT30U plug body (mates with an XT30PW / vertical XT30).  Frame: mating face at Z = 0, plug
    body along +Z (cable side), 10.2 x 5.2 section."""
    comp = C.master(name)
    b = B.abox(-5.1, 5.1, -2.6, 2.6, 0.0, 15.5)
    C.add_body(comp, b, 'housing', 'Nylon', 'connector yellow')
    return comp


def jst_ph3_plug():
    """JST PHR-3 housing with crimped wires.  Frame: mating face Z = 0, body along +Z, 8.0 x 4.5."""
    comp = C.master('JST PHR-3')
    C.add_body(comp, B.abox(-4.0, 4.0, -2.25, 2.25, 0.0, 6.0), 'housing', 'Nylon', 'connector white')
    return comp


def terminator():
    """CAN-FD 120 ohm terminator envelope from the brief: 20 long x 15 wide x 10 thick, the PH-3
    plug part (6 mm) inside the header.  Frame as jst_ph3_plug."""
    comp = C.master('CAN terminator 120R (PH-3)')
    b = B.abox(-4.0, 4.0, -2.25, 2.25, -6.0, 0.0)
    # 10 mm thick body kept below the neighbouring XT30 plug (which starts 1.8 mm above the PH axis... +y side)
    C.union(b, B.rbox((0, -0.3, 10.0), (1, 0, 0), (0, 1, 0), 15.0, 10.0, 20.0, 2.5))
    C.add_body(comp, b, 'body', 'Nylon', 'heat shrink')
    return comp


def xt30_cap():
    """Protective cap over a live XT30PW-M (rubber boot), 13 x 8 x 14 envelope beyond the edge."""
    comp = C.master('XT30 cap')
    # boot over the XT30PW body (PCB top 1.65 .. 7.15 above the motor-side face): Y = -2.75 .. +3.25 about the plug axis
    C.add_body(comp, B.rbox((0, 0.25, 4.0), (1, 0, 0), (0, 1, 0), 13.0, 6.0, 14.0, 2.0), 'cap', 'Rubber', 'red')
    return comp


def xt90_xt60_adapter():
    """XT90-F -> XT60-M adapter pigtail (housing parts only, the wires are modelled by the harness)."""
    comp = C.master('XT90-F to XT60-M adapter')
    b = B.rbox((0, 0, 14.0), (1, 0, 0), (0, 1, 0), 21.0, 10.5, 28.0, 2.0)        # XT90 section (mjpower-ss STEP)
    C.add_body(comp, b, 'XT90-F', 'Nylon', 'connector yellow')
    return comp


def xt60_pair():
    comp = C.master('XT60 pair (battery + adapter)')
    b = B.rbox((0, 0, 16.0), (1, 0, 0), (0, 1, 0), 16.0, 8.5, 32.0, 2.0)
    C.add_body(comp, b, 'XT60', 'Nylon', 'connector yellow')
    return comp


# ----------------------------------------------------------------------------- misc
def fan30():
    """30 x 30 x 7 mm 5 V fan.  Frame: centre, axis along Z."""
    comp = C.master('fan 30x30x7 5V')
    b = B.rbox((0, 0, 0), (1, 0, 0), (0, 1, 0), 30.0, 30.0, 7.0, 3.0)
    C.cut(b, B.zcyl(0, 0, -4, 4, 14.0))
    C.union(b, B.zcyl(0, 0, -3.5, 3.5, 8.0))
    for k in range(7):
        a = math.radians(360.0 * k / 7)
        C.union(b, B.box((11.0 * math.cos(a), 11.0 * math.sin(a), 0), (math.cos(a + 0.5), math.sin(a + 0.5), 0),
                         (0, 0, 1), 7.0, 5.0, 1.2))
    C.add_body(comp, b, 'fan', 'Nylon', 'black')
    return comp


def rocker_switch():
    """Illuminated rocker switch (mjbots PH-4 harness), panel cut-out 19 x 13, Ø? body 30 deep.
    Frame: panel front face Z = 0, body towards -Z."""
    comp = C.master('rocker switch (mjbots PH4 harness)')
    b = B.rbox((0, 0, 1.0), (1, 0, 0), (0, 1, 0), 21.0, 15.0, 2.0, 1.5)
    C.union(b, B.abox(-9.5, 9.5, -6.5, 6.5, -24.0, 0.0))
    C.add_body(comp, b, 'switch', 'Nylon', 'black')
    C.add_body(comp, B.abox(-8.0, 8.0, -5.0, 5.0, 2.0, 4.5), 'rocker', 'Nylon', 'red')
    return comp


def m365_tyre():
    """Xiaomi M365 8.5 x 2 solid honeycomb tyre (the user's order, 10-31): OD 202, tread TYRE_W wide, bore Ø141 x
    TYRE_BORE_W with 24 rectangular cavities open to the bore, bead band TYRE_BEAD_H then a flare to the full width at
    TYRE_FLARE_R (modelled; listing photo dims only), rounded crown with block tread.  Frame: Z = wheel axis towards
    the robot centre, Z = 0 at the tyre centre plane.  Mass set by check.apply_print_mass (TYRE_MASS)."""
    comp = C.master('tyre M365 8.5x2 solid (%.0f x %.0f)' % (P.TYRE_OD, P.TYRE_W))
    ro, rb = P.TYRE_OD / 2, P.TYRE_BORE_D / 2
    hw, hb = P.TYRE_W / 2, P.TYRE_BORE_W / 2
    r_bead, r_full = rb + P.TYRE_BEAD_H, P.TYRE_FLARE_R
    rr = 12.0
    t = B.zcyl(0, 0, -hw + rr, hw - rr, ro)
    C.union(t, B.zcyl(0, 0, -hw, hw, ro - rr))
    for z in (-hw + rr, hw - rr):
        C.union(t, _ztorus_part(z, ro - rr, rr))
    C.cut(t, B.zcyl(0, 0, -hw - 1, hw + 1, rb))
    for sg in (1, -1):                                           # bead band, flare to the full width
        C.cut(t, B.cyl((0, 0, sg * hb), (0, 0, sg * hw), r_bead, r_full))
        C.cut(t, B.cyl((0, 0, sg * hw), (0, 0, sg * (hw + 2)), r_full + 2))
    n, cw, cd, cl = P.TYRE_CAVITY                                # bore cavities (the rim's lugs sit in them)
    for k in range(n):
        a = math.radians(360.0 / n * k)
        u = (math.cos(a), math.sin(a), 0.0)
        rc = rb + (cd - 1.0) / 2
        C.cut(t, B.box((u[0] * rc, u[1] * rc, 0.0), u, (-u[1], u[0], 0), cd + 1.0, cw, cl))
    for k in range(40):                                          # block tread
        a = math.radians(9 * k)
        u = (math.cos(a), math.sin(a), 0.0)
        for zz in (-hw / 3, hw / 3):
            C.cut(t, B.box((u[0] * (ro + 1.0), u[1] * (ro + 1.0), zz + (4 if k % 2 else -4)), u, (-u[1], u[0], 0),
                           6.0, 4.0, hw / 2))
    C.add_body(comp, t, 'tyre', 'Rubber', 'rubber')
    return comp


def _ztorus_part(z, R, r):
    t = C.TBM.createTorus(adsk.core.Point3D.create(0, 0, 0), adsk.core.Vector3D.create(0, 0, 1), R * C.CM, r * C.CM)
    mv = adsk.core.Matrix3D.create()
    mv.translation = adsk.core.Vector3D.create(0, 0, z * C.CM)
    C.TBM.transform(t, mv)
    return t


def _hex_prism(af, z0, z1):
    """Hexagon prism (across flats af) along Z in the part frame, flats normal to +-X."""
    r = af / 2 / math.cos(math.radians(30))
    poly = [(r * math.cos(math.radians(30 + 60 * k)), r * math.sin(math.radians(30 + 60 * k))) for k in range(6)]
    return B.prism(poly, z0, z1, plane='xy')


def magnet_spacer():
    """6061 spacer glued on the AKE Ø9 rotor boss, the AKE Ø6 x 2.5 magnet glued in its top pocket.
    Frame = motor frame (+y to the back), sits on the boss face y = AKE['magnet']."""
    comp = C.master('magnet spacer Ø9x%g (6061)' % P.SPACER_H)
    y0 = P.AKE['magnet']
    y1 = y0 + P.SPACER_H
    (sd, sl), (pd, pdep) = P.SPACER_SPIGOT, P.SPACER_POCKET
    s = B.cyl((0, y0, 0), (0, y1, 0), 4.5)
    C.union(s, B.cyl((0, y0 - sl, 0), (0, y0, 0), sd / 2))      # spigot into the empty magnet pocket
    C.cut(s, B.cyl((0, y1 - pdep, 0), (0, y1 + 1, 0), pd / 2))  # magnet pocket
    C.add_body(comp, s, 'spacer', 'Al6061', 'alu')
    m = B.cyl((0, y1 - 2.5, 0), (0, y1, 0), 3.0)
    C.add_body(comp, m, 'magnet Ø6x2.5 N38H (AKE)', 'Steel', 'magnet')
    return comp


def clamp_ring(name, od, idia, pcd, angles, hole, y0):
    """Laser-cut stainless ring (P.RING_T) under the screw heads of a motor joint on a printed part.
    Frame = motor frame (+y to the back); the ring spans motor y0 .. y0 - RING_T (away from the motor),
    holes on the motor's own hole circle so one master fits every joint and both sides."""
    comp = C.master(name)
    y1 = y0 - P.RING_T
    r = B.cyl((0, y1, 0), (0, y0, 0), od / 2)
    C.cut(r, B.cyl((0, y1 - 1, 0), (0, y0 + 1, 0), idia / 2))
    for a in angles:
        t = math.radians(a)
        x, z = pcd / 2 * math.cos(t), pcd / 2 * math.sin(t)
        C.cut(r, B.cyl((x, y1 - 1, z), (x, y0 + 1, z), hole / 2))
    C.add_body(comp, r, 'ring', 'Steel', 'steel')
    return comp


def clamp_ring_out():
    """On the floor of a driven boss (hip, knee), under the 6 x M3 heads."""
    return clamp_ring('clamp ring output Ø%.0f t%.1f SUS304' % (P.RING_OUT[0], P.RING_T), P.RING_OUT[0], P.RING_OUT[1],
                      P.AKE_PCD_OUT, P.AKE_OUT_M3, 3.4, P.AKE['out'] - P.BEARING_FACE + P.RING_T)


def clamp_ring_stator():
    """In the far-face recess of the PETG knee / wheel plates, under the 6 x M2.5 low heads."""
    assert P.KNEE_PLATE_T == P.WHEEL_PLATE_T
    far = P.AKE['ring'] - (P.KNEE_PLATE_T - P.POCKET)
    return clamp_ring('clamp ring stator Ø%.0f t%.1f SUS304' % (P.RING_STATOR[0], P.RING_T), P.RING_STATOR[0],
                      P.RING_STATOR[1], P.AKE_PCD_RING, P.AKE_RING_M25, 2.8, far + P.CBORE_M25[1] + P.RING_T)


def standoff(length, name):
    """M2.5 hex standoff, 5 mm AF, male thread 6 mm.  Frame: base (female end) at Z = 0, body to +Z...
    the male stud points to -Z."""
    comp = C.master(name)
    s = _hex_prism(5.0, 0.0, length)
    C.cut(s, B.zcyl(0, 0, length - 5.0, length + 1, 1.025))      # threads at the M2.5 minor Ø (fasteners.MINOR)
    C.union(s, B.zcyl(0, 0, -6.0, 0.0, 1.025))
    C.add_body(comp, s, 'standoff', 'Brass', 'brass')
    return comp


def build_all():
    out = {}
    out['ake_stator'], out['ake_output'] = ake60()
    out['moteus'] = moteus()
    out['pi3hat'] = pi3hat()
    for k, f in (('rpi', rpi4b), ('mjpower', mjpower_ss), ('battery', battery), ('xt30', xt30_plug),
                 ('ph3', jst_ph3_plug), ('term', terminator), ('xt30cap', xt30_cap), ('xt90', xt90_xt60_adapter),
                 ('xt60', xt60_pair), ('fan', fan30), ('switch', rocker_switch), ('wheel', m365_tyre),
                 ('spacer', magnet_spacer)):                 # v3: no clamp rings (6061 links)
        out[k] = f()
    out['standoff11'] = standoff(11.0, 'standoff M2.5x11 M-F (pi3hat)')
    return out
