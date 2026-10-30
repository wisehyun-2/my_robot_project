"""Build an unsaved scratch document with the LEFT wheel output stack for exploded renders:
parts copied (world placement kept) from WheelBiped_v2 + fasteners modelled from the BOM (the robot model has them
as BOM lines only).  Each group / fastener is its own occurrence; attributes 'wx' record the group and spin axis."""
import math
import sys
FUS = r'C:\Users\User\Desktop\myrobot\my_robot_project\robot_cad_v2\fusion'
TAG = 'wheel_output_explode'


def run(_context: str):
    if FUS not in sys.path:
        sys.path.insert(0, FUS)
    import adsk.core
    import adsk.fusion
    import v2run
    M = v2run.load()
    C, P, LY, LG = M['cadlib'], M['params'], M['layout'], M['legs']
    app = C.app()
    for d in app.documents:                       # rebuild: close an old scratch doc without saving
        dd = adsk.fusion.Design.cast(d.products.itemByProductType('DesignProductType'))
        if dd and dd.attributes.itemByName('wx', 'tag'):
            d.close(False)
    src = app.activeDocument
    assert src.name.startswith('WheelBiped_v2'), src.name
    des = C.design()
    B = C.Builder(1)
    picks = [('SHANK', 'tibia_L:1+shank_L'), ('MOTOR', 'tibia_L:1+AKE60-8_KV80_stator'),
             ('MOTOR', 'tibia_L:1+magnet spacer'), ('MOTOR', 'wheel_L:1+AKE60-8_KV80_output'),
             ('RING', 'tibia_L:1+clamp ring stator'), ('FLANGE', 'wheel_L:1+hub_flange_L'),
             ('HEX', 'wheel_L:1+hex_hub_L'), ('WHEEL', 'wheel_L:1+RC wheel'), ('NUT', 'wheel_L:1+M5 serrated')]
    RECOLOR = {'SHANK': ('wx petg green', (120, 185, 120)), 'FLANGE': ('wx flange orange', (215, 105, 40)),
               'HEX': ('wx hex amber', (240, 165, 70))}
    items = {}
    for o in des.rootComponent.allOccurrences:
        for g, pre in picks:
            if o.fullPathName.startswith(pre):
                for b in o.bRepBodies:
                    if not b.isVisible:
                        continue
                    t = C.TBM.copy(b)
                    if g == 'SHANK':
                        C.inter(t, B.halfspace((0, 0, 132.0), (0, 0, 1)))
                    look = RECOLOR.get(g, b.appearance)          # black PETG parts get telling colours
                    items.setdefault(g, []).append((t, look, b.name))
    print({g: len(v) for g, v in items.items()})

    # ---------------------------------------------------------------- fasteners (robot mm, left leg)
    A = P.AKE
    fr = LY.frames('wheel', 1)['motor']
    lv = LG.hub_levels()
    y_out = P.y_output('wheel')
    rec = P.Y_TIBIA_IN + P.RING_T + P.CBORE_M25[1]
    bear = y_out - P.HUB_SCREW_SEAT                # washer seat of the M3 x 14 (10-30)

    def hexprism(x, z, af, y0, y1):
        r = af / 2 / math.cos(math.radians(30))
        poly = [(x + r * math.cos(math.radians(30 + 60 * k)), z + r * math.sin(math.radians(30 + 60 * k)))
                for k in range(6)]
        return B.prism(poly, y0, y1)

    def thread(body, x, z, d, y0, y1, pitch):
        n = int(abs(y1 - y0) / pitch)
        sg = 1 if y1 > y0 else -1
        for i in range(1, n):
            yc = y0 + sg * i * pitch
            ring = B.ycyl(x, z, yc - 0.18, yc + 0.18, d / 2 + 1.0)
            C.cut(ring, B.ycyl(x, z, yc - 1, yc + 1, d / 2 - 0.16))
            C.cut(body, ring)
        return body

    def cap_screw(x, z, y_face, sg, dk, k, d, L, s, t, thread_len=None, pitch=0.8):
        y_b = y_face + sg * k
        body = B.ycyl(x, z, y_face, y_b, dk / 2)
        C.union(body, B.ycyl(x, z, y_b - sg * 0.01, y_b + sg * L, d / 2))
        C.cut(body, hexprism(x, z, s, y_face - sg * 0.2, y_face + sg * t))
        tl = L if thread_len is None else thread_len
        thread(body, x, z, d, y_b + sg * (L - tl), y_b + sg * L, pitch)
        return body

    fast = []                                    # (group, name, temp, look, (x, z))
    for i, p in enumerate(LY.motor_points(fr, P.AKE_PCD_RING / 2, P.AKE_RING_M25, A['ring'])):
        fast.append(('S25', 'M2.5x8 DIN7984 #%d' % i,
                     cap_screw(p[0], p[2], rec - P.RING_T - 1.5, 1, 4.5, 1.5, 2.5, 8.0, 1.5, 0.9, pitch=0.7),
                     'steel', (p[0], p[2])))
    for i, p in enumerate(LY.motor_points(fr, P.AKE_PCD_OUT / 2, P.AKE_OUT_M3, A['out'])):
        x, z = p[0], p[2]
        fast.append(('S316', 'M3x%d ISO4762 12.9 #%d' % (P.HUB_SCREW_L, i),
                     cap_screw(x, z, bear - 3.5, 1, 5.5, 3.0, 3.0, float(P.HUB_SCREW_L), 2.5, 1.3, pitch=0.8), 'black satin',
                     (x, z)))
        w = B.ycyl(x, z, bear - 0.5, bear, 3.5)
        C.cut(w, B.ycyl(x, z, bear - 1, bear + 1, 1.6))
        fast.append(('WASH', 'washer DIN125 M3 #%d' % i, w, 'steel', (x, z)))
    for i, p in enumerate(LY.motor_points(fr, P.AKE_PCD_OUT / 2, P.AKE_OUT_PIN, A['out'])):
        x, z = p[0], p[2]
        pin = B.ycyl(x, z, y_out - 6.7, y_out + 2.7, 1.5)
        C.union(pin, B.cyl((x, y_out - 7.0, z), (x, y_out - 6.69, z), 1.2, 1.5))
        C.union(pin, B.cyl((x, y_out + 2.69, z), (x, y_out + 3.0, z), 1.5, 1.2))
        fast.append(('PINS', 'dowel ISO8734 3x10 #%d' % i, pin, 'steel', (x, z)))
    for i, a in enumerate(LG.HUB_SCREWS):
        x, z = 18 * math.cos(a), P.R_WHEEL + 18 * math.sin(a)
        ins = B.ycyl(x, z, lv['f0'] + 0.1, lv['f0'] + 5.8, 2.15)
        C.cut(ins, B.ycyl(x, z, lv['f0'], lv['f0'] + 6, 1.3))
        for yc in (lv['f0'] + 1.6, lv['f0'] + 4.1):
            g = B.ycyl(x, z, yc - 0.4, yc + 0.4, 3.0)
            C.cut(g, B.ycyl(x, z, yc - 1, yc + 1, 1.9))
            C.cut(ins, g)
        fast.append(('INS', 'insert M3x5.7 #%d' % i, ins, 'brass', (x, z)))
        fast.append(('S38', 'M3x8 ISO4762 8.8 #%d' % i,
                     cap_screw(x, z, lv['h0'] + 0.5, 1, 5.5, 3.0, 3.0, 8.0, 2.5, 1.3, pitch=0.8), 'steel', (x, z)))
    fast.append(('BOLT', 'M5x50 ISO4762 8.8', cap_screw(0.0, P.R_WHEEL, lv['f0'] + 6.5, -1, 8.5, 5.0, 5.0, 50.0, 4.0,
                                                         2.5, thread_len=22.0, pitch=1.0), 'steel', (0.0, P.R_WHEEL)))

    # ---------------------------------------------------------------- new document
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    nd = adsk.fusion.Design.cast(app.activeProduct)
    nd.designType = adsk.fusion.DesignTypes.DirectDesignType
    nd.attributes.add('wx', 'tag', TAG)
    nroot = nd.rootComponent
    nroot.isOriginFolderLightBulbOn = False
    cache = {}

    def look_of(a):
        if isinstance(a, str):
            return C.appearance(a)
        if isinstance(a, tuple):                   # (name, rgb): plastic copy with that colour
            na = nd.appearances.itemByName(a[0])
            if na is None:
                base = app.materialLibraries.item(0).appearances.itemById('Prism-127')
                na = nd.appearances.addByCopy(base, a[0])
                pr = na.appearanceProperties.itemById('opaque_albedo') or na.appearanceProperties.itemById('surface_albedo')
                pr.value = adsk.core.Color.create(a[1][0], a[1][1], a[1][2], 255)
            return na
        if a.name in cache:
            return cache[a.name]
        try:
            na = nd.appearances.itemByName(a.name) or nd.appearances.addByCopy(a, a.name)
        except Exception as e:
            print('appearance copy failed', a.name, e)
            na = C.appearance('dark grey')
        cache[a.name] = na
        return na

    def add_occ(name, bodies, group, axis=None):
        occ = nroot.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        occ.component.name = name
        for t, a, bn in bodies:
            b = occ.component.bRepBodies.add(t)
            b.name = bn
            b.appearance = look_of(a)
        occ.component.attributes.add('wx', 'group', group)
        if axis is not None:
            occ.component.attributes.add('wx', 'axis', '%.4f,%.4f' % axis)
        return occ

    for g, v in items.items():
        add_occ(g, v, g)
    for g, n, t, lk, ax in fast:
        add_occ(n, [(t, lk, n)], g, ax)
    print('occurrences', nroot.occurrences.count)
