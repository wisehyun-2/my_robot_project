"""v5 battery tray and electronics deck printed in two pieces each (11-09, user: "배터리 받침과 전장 데크는 한번에 출력하지 말고
두번으로 나눠서 출력해서 조립 또는 이어 붙이게 설계해줘" - the printer bed is 300 x 300 and the one-piece parts needed 304 / 309 mm).

Each part is the v4 solid (torso.tray / torso.deck, v5 layout) cut by a plane y = const (robot axes; the long side of both parts runs
along y between the hip plates).  Both pieces get a flange on the cut face, under the floor; the two flanges butt against each other
and are bolted with 2 x ISO 7380 M3 x 10 + DIN 934 M3 nuts (y-axis bolts, heads on the -y piece), and may be glued as well.
  * battery tray: cut at y 0, the centre rib's place (the two flanges replace that rib, same 8 mm depth and x span between the
    tubes).  Each piece keeps two tube saddles per tube and one strap slot pair, so it also sits on its own.
  * electronics deck: cut at y 80, between the Raspberry Pi (its USB end reaches y 60.3) and the +y end window (y 104); the deck's
    two ribs are cut too, the flanges span the ribs (x +-41.5, DECK_FLANGE_X) and reach 8 mm under the floor (2 mm below the ribs).
Pieces are named after the robot (_L = CAD -y, user 11-08): battery_tray_L / _R, electronics_deck_L (Pi, power board) / _R.

v5run.resplit() applies this to the hub file, build() and relayout() make the pieces directly.  No Fusion import at module level.
"""
TRAY_CUT_Y = 0.0
DECK_CUT_Y = 80.0
FLANGE_T = 3.0               # each flange, along y
FLANGE_D = 8.0               # below the floor underside
HOLE_R = 1.7                 # M3 clearance Ø3.4
BOLTS_X = {'battery_tray': (-22.0, 22.0), 'electronics_deck': (-25.0, 25.0)}
DECK_FLANGE_X = 41.5         # deck flanges end at the ribs' outer faces (torso.deck ribs x +-40, 3 wide): at the full width 55 the
                             # flange's switch-side end cut 1.4 mm into shell_mid's floor, which rises from x -34 towards the switch
                             # face (11-09 interference check); at 41.5 the floor is 1.5 mm below the flange
SCREW_L = 10                 # 2 flanges 6 + nut 2.4 -> 1.6 proud
SCREW = 'FX ISO 7380 M3x%d split' % SCREW_L
NUT = 'FX DIN 934 M3 split'
PIECES = {'battery_tray': ('battery_tray_L', 'battery_tray_R'),
          'electronics_deck': ('electronics_deck_L', 'electronics_deck_R')}
LOOK = ('PETG print', 'dark grey')


def joint(M, whole):
    """(cut y, floor underside z, flange half width x) of the joint of 'battery_tray' or 'electronics_deck' (robot mm)."""
    P = M['params']
    ZH = P.Z_HIP
    if whole == 'battery_tray':
        return TRAY_CUT_Y, ZH + P.TRAY_Z, P.TUBE_X - 8.4        # the tray ribs' x span (torso.tray)
    return DECK_CUT_Y, ZH + P.DECK_Z - P.DECK_T_FLOOR, DECK_FLANGE_X


def pieces(M, whole):
    """{piece name: temporary body}: the v4 solid cut at the joint plane, a flange with 2 bolt holes on each cut face."""
    C, TO = M['cadlib'], M['torso']
    B = C.Builder()
    body = TO.tray() if whole == 'battery_tray' else TO.deck()
    yc, z0, xr = joint(M, whole)
    big = 1000.0
    lo = C.copy(body)
    C.inter(lo, B.abox(-big, big, -big, yc, -big, big))
    hi = C.inter(body, B.abox(-big, big, yc, big, -big, big))
    zb = z0 - FLANGE_D / 2
    for piece, y0, y1 in ((lo, yc - FLANGE_T, yc), (hi, yc, yc + FLANGE_T)):
        C.union(piece, B.abox(-xr, xr, y0, y1, z0 - FLANGE_D, z0 + 0.01))
        for x in BOLTS_X[whole]:
            C.cut(piece, B.ycyl(x, zb, y0 - 1.0, y1 + 1.0, HOLE_R))
    left, right = PIECES[whole]
    return {left: lo, right: hi}


def fastener_masters(M):
    F = M['fasteners']
    return {SCREW: (lambda: F.socket_screw(3, SCREW_L, 'button'), 'Steel', 'black satin'),
            NUT: (lambda: F.nut(3), 'Steel', 'steel')}


def fastener_positions(M, wholes=tuple(PIECES)):
    """fasteners.positions records: the screw head bears on the -y flange, the nut on the +y flange's free face."""
    out = []
    for whole in wholes:
        yc, z0, _xr = joint(M, whole)
        zb = z0 - FLANGE_D / 2
        for x in BOLTS_X[whole]:
            out.append(('torso', SCREW, (x, yc - FLANGE_T, zb), (0.0, 1.0, 0.0)))
            out.append(('torso', NUT, (x, yc + FLANGE_T, zb), (0.0, 1.0, 0.0)))
    return out


def _masters(M):
    C = M['cadlib']
    out = {}
    for name, (geo, mat, look) in fastener_masters(M).items():
        comp = C.find_comp(name)
        if comp is None:
            comp = C.master(name)
            C.add_body(comp, geo(), name, mat, look)
        out[name] = comp
    return out


def add(M, t, wholes=tuple(PIECES)):
    """Pieces and their joint fasteners into the torso occurrence t (robot names).  Returns the pieces' components."""
    C, A, F = M['cadlib'], M['assembly'], M['fasteners']
    comps = []
    for whole in wholes:
        for name, body in pieces(M, whole).items():
            comps.append(A.add_part(t, name, body, *LOOK))
    masters = _masters(M)
    for _link, name, o, ez in fastener_positions(M, wholes):
        C.instance(t, masters[name], C.part_matrix(F._frame(o, ez)))
    return comps


def drop(t, wholes=tuple(PIECES)):
    """Delete the one-piece part, its pieces and their joint fasteners (by position: the nuts / screws of one joint) from the torso."""
    names = set()
    for w in wholes:
        names |= {w} | set(PIECES[w])
    out = []
    for o in list(t.childOccurrences):
        if o.component.name in names:
            out.append(o.component.name)
            o.deleteMe()
    return out


def drop_fasteners(M, t, wholes=tuple(PIECES)):
    """The joint screws / nuts of the given joints (matched by their place on the cut plane)."""
    yc = {joint(M, w)[0] for w in wholes}
    n = 0
    for o in list(t.childOccurrences):
        if o.component.name not in (SCREW, NUT):
            continue
        bb = o.boundingBox
        y_mm = (bb.minPoint.x + bb.maxPoint.x) / 2 * 10.0          # Fusion world X = robot y
        if any(abs(y_mm - y) < 12.0 for y in yc):
            o.deleteMe()
            n += 1
    return n


def apply(M):
    """Hub file: one-piece tray / deck (or earlier pieces) out, pieces + joint fasteners in."""
    C = M['cadlib']
    t = C.root_occ('torso')
    dropped = drop(t)
    nf = drop_fasteners(M, t)
    comps = add(M, t)
    return dict(dropped=sorted(dropped), old_fasteners=nf, pieces=[c.name for c in comps],
                fasteners=len(fastener_positions(M)))
