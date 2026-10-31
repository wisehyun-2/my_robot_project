"""Component tree and placement of the v3 robot (nominal pose; joints added by finalize.joints).

root
  _library          hidden masters (purchased parts)
  torso             grounded: frame, shell, battery, electronics, hip motors / drivers / cups, harness
  femur_L, tibia_L, wheel_L, femur_R, tibia_R, wheel_R
                    links at the identity in the nominal pose; each holds its designed parts as
                    sub-components and the purchased parts riding on it (motor stators, output
                    carriers of the motors driving it, drivers, cups, connectors).
"""
import adsk.core
import adsk.fusion

import cadlib as C
import layout as LY
import legs as LG
import params as P
import shell as SH
import torso as TO
import vendor as V

LINKS = ('torso', 'femur_L', 'tibia_L', 'wheel_L', 'femur_R', 'tibia_R', 'wheel_R')
SFX = {1: 'L', -1: 'R'}


def reset():
    """Delete everything but the _library masters (direct design; a finalized parametric design with joints is
    switched back first, which drops its timeline and joints)."""
    d = C.design()
    if d.designType != adsk.fusion.DesignTypes.DirectDesignType:
        d.designType = adsk.fusion.DesignTypes.DirectDesignType
    root = d.rootComponent
    for o in [o for o in root.occurrences if o.component.name != '_library']:
        o.deleteMe()
    while root.bRepBodies.count:
        root.bRepBodies.item(0).deleteMe()


def links():
    root = C.design().rootComponent
    out = {}
    for name in LINKS:
        try:
            out[name] = C.root_occ(name)
        except KeyError:
            occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            occ.component.name = name
            out[name] = occ
    return out


def place(parent, comp, m_robot):
    return C.instance(parent, comp, C.part_matrix(m_robot))


def add_part(parent, name, temp, mat='PETG print', look='black'):
    comp = C.new_part(parent, name)
    C.add_body(comp, temp, name, mat, look)
    return comp


def tyre_frame(s):
    return C.frame((0.0, s * P.tyre_centre_y(), P.R_WHEEL), (1, 0, 0), (0, 0, s), (0, -s, 0))


def conn_plug_frame(board, x_b, y_b, z_edge):
    """Frame of a plug mating a side-entry header at the board edge: plug Z along +z_b, centred at
    (x_b, y_b) of the board frame, mating face at z_b = z_edge."""
    o = C.apply(board, (x_b, y_b, z_edge))
    ex = C.col(board, 0)
    ey = C.col(board, 1)
    ez = C.col(board, 2)
    return C.frame(o, ex, ey, ez)


# moteus r4.5 STEP: XT30PW (far side, x -13..-3 / 3..13, y 1.65..7.15), PH-3 CAN (motor side, y -6.2..0).
# The two XT30 (and the two PH-3) are the same net in parallel, so the J1/J2 and CAN1/CAN2 labels only follow
# the brief's chain convention.  The model puts the incoming pair (J1, CAN2) at -x_b and the outgoing pair
# (J2, CAN1) at +x_b so the in and out cables never cross; label the boards accordingly.
MOTEUS_PORTS = {'J1': (-8.0, 4.4), 'J2': (8.0, 4.4), 'CAN2': (-8.0, -3.1), 'CAN1': (8.0, -3.1)}


def port_frame(joint, s, port):
    b = LY.frames(joint, s)['board']
    x, y = MOTEUS_PORTS[port]
    if s < 0:
        x = -x
    return conn_plug_frame(b, x, y, P.MOTEUS['edge_conn'])


def build_torso(parts):
    t = links()['torso']
    for s in (1, -1):
        add_part(t, 'hip_plate_' + SFX[s], TO.hip_plate(s), 'Al6061', 'alu black')
    add_part(t, 'spine_tube_F', TO.spine_tube(1), 'Al6061', 'alu')
    add_part(t, 'spine_tube_B', TO.spine_tube(-1), 'Al6061', 'alu')
    add_part(t, 'rail_F', TO.rail(1), 'Al6061', 'alu')
    add_part(t, 'rail_B', TO.rail(-1), 'Al6061', 'alu')
    add_part(t, 'top_plate', TO.top_plate(), 'PETG print', 'black')
    add_part(t, 'battery_tray', TO.tray(), 'PETG print', 'dark grey')
    place(t, parts['battery'], TO.battery_frame())
    add_part(t, 'electronics_deck', TO.deck(), 'PETG print', 'dark grey')
    place(t, parts['rpi'], TO.rpi_frame())
    place(t, parts['pi3hat'], TO.pi3hat_frame())
    pm = TO.rpi_frame()
    for (x, y) in ((3.5, 3.5), (61.5, 3.5), (3.5, 52.5), (61.5, 52.5)):
        o = C.apply(pm, (x, y, 1.4))
        place(t, parts['standoff11'], C.frame(o, (1, 0, 0), (0, 1, 0), (0, 0, 1)))
    place(t, parts['mjpower'], TO.mjpower_frame())
    place(t, parts['fan'], TO.fan_frame())


def _add_wires(link_occ, name, bodies, looks):
    comp = C.new_part(link_occ, name)
    for i, (b, look) in enumerate(zip(bodies, looks)):
        C.add_body(comp, b, '%s_%d' % (name, i), 'Copper wire', look)
    return comp


def build_harness(parts):
    import harness as HN
    L = links()
    B = C.Builder()
    stretched = []
    tc, xt60, xt90 = HN.torso_cables()
    for cab in HN.cables(1) + HN.cables(-1) + tc:
        for i, rn in enumerate(cab['runs']):
            if rn[0] == 'loop':
                _, la, a, ta, lb, b, tb, length = rn
                pts, ok = HN.loop_points(a, ta, b, tb, length)
                if not ok:
                    stretched.append(cab['name'])
                _add_wires(L[la], 'harness_%s_loop' % cab['name'], HN.wires_along(B, pts, HN.BUNDLE, HN.Y_SIDE),
                           HN.BUNDLE_LOOKS)
            else:
                link, pts, wires, looks, pref = rn
                _add_wires(L[link], 'harness_%s_%d' % (cab['name'], i), HN.wires_along(B, HN.smooth(pts), wires, pref),
                           looks)
        for (kind, f, link) in cab['plugs']:
            place(L[link], parts[kind], f)
    # v3: no cable clips -- the leg cables run inside the 6061 link boxes (legs.DUCT_S windows)
    place(L['torso'], parts['xt60'], C.frame(C.add(xt60, (0, 0, -16.0)), (1, 0, 0), (0, 1, 0), (0, 0, 1)))
    place(L['torso'], parts['xt90'], C.frame(xt90, (1, 0, 0), (0, 0, -1), (0, 1, 0)))
    place(L['torso'], parts['switch'], C.frame((-P.TORSO_X, P.SWITCH_Y, P.Z_HIP + P.SWITCH_Z), (0, 1, 0), (0, 0, -1),
                                               (-1, 0, 0)))
    return stretched


def _plug_link(f):
    """Link carrying a plug frame (by the joint whose driver it plugs into)."""
    o = C.origin(f)
    dk = (o[0] - P.KNEE_X) ** 2 + (o[2] - P.KNEE_Z) ** 2
    dw = (o[0]) ** 2 + (o[2] - P.R_WHEEL) ** 2
    dh = (o[0]) ** 2 + (o[2] - P.Z_HIP) ** 2
    s = 'L' if o[1] > 0 else 'R'
    best = min((dh, 'torso'), (dk, 'femur_' + s), (dw, 'tibia_' + s))
    return best[1]


def build_shell():
    t = links()['torso']
    with C.IdentityParent(t):
        tmp = t.component.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    tmp.component.name = 'shell_tmp'
    h = SH.hollow(tmp.component)
    tmp.deleteMe()
    SH.corner_posts(h)
    SH.openings(h)
    for name, body in SH.pieces(h).items():
        add_part(t, name, body, 'PETG print', 'blue' if name.startswith('saddle') else 'shell grey')
    add_part(t, 'splice_tabs', SH.splice_tabs(), 'PETG print', 'dark grey')


def build_leg(s, parts):
    L = links()
    k = SFX[s]
    torso, femur, tibia, wheel = L['torso'], L['femur_' + k], L['tibia_' + k], L['wheel_' + k]
    fh, fk, fw = LY.frames('hip', s), LY.frames('knee', s), LY.frames('wheel', s)
    st, out, brd, spc = parts['ake_stator'], parts['ake_output'], parts['moteus'], parts['spacer']
    # hip: stator, spacer, driver and cup in the torso; output carrier with the thigh
    place(torso, st, fh['motor'])
    place(torso, spc, fh['motor'])
    place(torso, brd, fh['step_board'])
    add_part(torso, 'cup_hip_' + k, LG.cup('hip', s), 'PETG print', 'black')
    place(femur, out, fh['motor'])
    add_part(femur, 'thigh_' + k, LG.femur(s), 'Al6061', 'alu')                  # v3: 6061-T6 shell + cover
    add_part(femur, 'thigh_cover_' + k, LG.cover('femur', s), 'Al6061', 'alu')
    place(femur, st, fk['motor'])
    place(femur, spc, fk['motor'])
    place(femur, brd, fk['step_board'])
    add_part(femur, 'cup_knee_' + k, LG.cup('knee', s), 'PETG print', 'black')
    place(tibia, out, fk['motor'])
    add_part(tibia, 'shank_' + k, LG.tibia(s), 'Al6061', 'alu')
    add_part(tibia, 'shank_cover_' + k, LG.cover('tibia', s), 'Al6061', 'alu')
    place(tibia, st, fw['motor'])
    place(tibia, spc, fw['motor'])
    place(tibia, brd, fw['step_board'])
    add_part(tibia, 'cup_wheel_' + k, LG.cup('wheel', s), 'PETG print', 'black')
    place(tibia, parts['term'], port_frame('wheel', s, 'CAN1'))
    place(tibia, parts['xt30cap'], port_frame('wheel', s, 'J2'))
    place(wheel, out, fw['motor'])
    add_part(wheel, 'hub_flange_' + k, LG.hub_flange(s), 'PETG print', 'black')
    add_part(wheel, 'rim_' + k, LG.rim(s), 'PETG print', 'black')
    place(wheel, parts['wheel'], tyre_frame(s))
