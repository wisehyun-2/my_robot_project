"""Stills: exploded overview, before / after of every step, the finished joint and two cutaways.
Writes <WORK>/raw/*.png (raw Fusion captures) + raw/stills.json (cameras, label anchors in view pixels)."""
import importlib
import math
import sys
SP = r'C:\Users\User\Desktop\myrobot\my_robot_project\robot_cad_v3\manufacturing\explain\assembly\wheel\scripts'
FUS = r'C:\Users\User\Desktop\myrobot\my_robot_project\robot_cad_v3\fusion'


def run(_context: str):
    for p in (SP, FUS):
        if p not in sys.path:
            sys.path.insert(0, p)
    import adsk.core
    import adsk.fusion
    sys.modules.pop('wxlib', None)               # a same-named module from another folder may be cached
    import wxlib as W
    print('wxlib', W.__file__, 'work', W.WORK)
    d, dd = W.doc()
    import cadlib as C
    G = W.groups(dd)
    allg = list(G)
    vp = adsk.core.Application.get().activeViewport
    vp.visualStyle = adsk.core.VisualStyles.ShadedWithVisibleEdgesOnlyVisualStyle
    W.grid_off()
    js = dict(size=[vp.width, vp.height], stills=[])

    def centre(g):
        bb = None
        for o, _ax in G[g]:
            b = o.boundingBox
            bb = b.copy() if bb is None else (bb.combine(b) and bb)
        return [(bb.minPoint.x + bb.maxPoint.x) * 5, (bb.minPoint.y + bb.maxPoint.y) * 5, (bb.minPoint.z + bb.maxPoint.z) * 5]

    def shot(name, vis, extra=None):
        path = W.snap(name)
        rec = dict(file=name, vis=list(vis), proj=W.project(G, vis), cam=W.cam_dict())
        rec.update(extra or {})
        js['stills'].append(rec)
        return path

    # overview
    W.place(G, W.HOME, visible=set(allg))
    W.set_view(W.VIEW0, target=(0.0, W.AX_Z, 0.0))
    shot('00_overview.png', allg, dict(kind='overview', cam0=True))
    cams = dict(overview=W.cam_dict())
    # steps
    for k, st in enumerate(W.states(), 1):
        vis = set(st['vis'])
        W.place(G, st['before'], visible=vis)
        W.set_view(st['view'], target=(0.0, W.AX_Z, 0.0))
        W.zoom(0.92)
        cams[st['key']] = W.cam_dict()
        arrows = {}
        for g in st['groups']:
            c = centre(g)
            dx = st['after'][g] - st['before'][g]
            arrows[g] = [W.project_pt(c), W.project_pt((c[0] + dx, c[1], c[2]))]
        info = dict(step=k, moving=st['groups'], title=st['title'], text=st['text'])
        shot('%02d_%s_a_before.png' % (k, st['key']), vis, dict(info, kind='before', arrows=arrows))
        W.place(G, st['after'], visible=vis)
        W.set_view(st['view'], target=(0.0, W.AX_Z, 0.0))
        W.zoom(0.9)
        cams[st['key'] + '_after'] = W.cam_dict()
        shot('%02d_%s_b_after.png' % (k, st['key']), vis, dict(info, kind='after'))
    # finished joint
    zero = {g: 0.0 for g in allg}
    W.place(G, zero, visible=set(allg))
    W.set_view(W.VIEW_DONE, target=(0.0, W.AX_Z, 0.0))
    W.zoom(0.95)
    cams['done'] = W.cam_dict()
    n_st = len(W.STEPS)
    shot('%02d_done.png' % (n_st + 1), allg, dict(kind='done'))

    # cutaways: copies of every body with the sector a..b (deg, atan2(z - AX_Z, x) about the wheel axis) removed
    import params as P
    B = C.Builder(1)
    root = dd.rootComponent
    f0 = P.Y_TIBIA_IN - 9.0                        # hub flange face = rim hub face (legs.hub_levels)
    tc = P.tyre_centre_y()

    def sector(a, b):
        ta, tb = math.radians(a), math.radians(b)
        c = (0.0, 0.0, W.AX_Z)
        h1 = B.halfspace(c, (math.sin(ta), 0.0, -math.cos(ta)))      # keeps cross(dir_a, q) >= 0
        h2 = B.halfspace(c, (-math.sin(tb), 0.0, math.cos(tb)))      # keeps cross(dir_b, q) <= 0
        return C.inter(h1, h2)

    feats = {
        'cut1': {'② M3 × 14 + 와셔': (10 * math.cos(math.radians(-5)), 200.0, W.AX_Z + 10 * math.sin(math.radians(-5))),
                 '④ M3 × 14 + 와셔 + 인서트': (0.0, f0 - 4.0, W.AX_Z + 18.0),
                 'A Ø11.8 스피곳': (5.5 * math.cos(math.radians(-5)), 212.3, W.AX_Z + 5.5 * math.sin(math.radians(-5))),
                 'B Ø11 등록 턱 (플랜지 → 림)': (0.0, f0 - 1.5, W.AX_Z + 5.5),
                 'C Ø42 파일럿 ↔ H7 포켓': (21.0 * math.cos(math.radians(-5)), 212.8, W.AX_Z + 21.0 * math.sin(math.radians(-5))),
                 '림 돌기 ↔ 타이어 구멍': (0.0, tc, W.AX_Z + 73.0)},
        'cut2': {'③ 평행핀 Ø3 × 10': (10 * math.cos(math.radians(25)), 209.5, W.AX_Z + 10 * math.sin(math.radians(25))),
                 '① M2.5 × 8 (카운터보어)': (18.25 * math.cos(math.radians(145)), 210.0, W.AX_Z + 18.25 * math.sin(math.radians(145))),
                 '③ 평행핀 (145°)': (10 * math.cos(math.radians(145)), 209.5, W.AX_Z + 10 * math.sin(math.radians(145))),
                 '① M2.5 × 8 (25°)': (18.25 * math.cos(math.radians(25)), 210.0, W.AX_Z + 18.25 * math.sin(math.radians(25)))},
    }
    def onface(t_deg, r, y):
        t = math.radians(t_deg)
        return (r * math.cos(t), y, W.AX_Z + r * math.sin(t))
    for i_cut, cut in enumerate(W.CUTS):
        feats[cut['key']].update({'허브 플랜지': onface(cut['a'], 19.5, 201.0),
                                  '림 허브 10 mm': onface(cut['a'], 27.0, f0 - 5.0),
                                  '정강이 판 8 mm (6061)': onface(cut['a'], 30.0, 210.0), 'AKE60-8 출력부': onface(cut['a'], 9.0, 222.0),
                                  '타이어': onface(cut['a'], 90.0, tc)})
        made = []
        cut_body = sector(cut['a'], cut['b'])
        nfail = 0
        for g in allg:
            for o, _ax in G[g]:
                occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
                occ.component.name = 'CUT_' + o.component.name
                occ.component.attributes.add('wx', 'cut', '1')
                for b in o.component.bRepBodies:
                    t = C.TBM.copy(b)
                    try:
                        C.cut(t, C.TBM.copy(cut_body))
                    except Exception:
                        nfail += 1
                    if t.faces.count == 0:
                        continue
                    nb = occ.component.bRepBodies.add(t)
                    nb.appearance = b.appearance
                made.append(occ)
        for o, _ax in [x for g in allg for x in G[g]]:
            o.isLightBulbOn = False
        W.set_view(cut['view'], target=(0.0, W.AX_Z, 0.0))
        W.recenter((198.0, W.AX_Z, 0.0), 0.62)
        cams[cut['key']] = W.cam_dict()
        pts = {name: W.project_pt((p[1], p[2], p[0])) for name, p in feats[cut['key']].items()}
        path = W.snap('%02d_%s.png' % (n_st + 2 + i_cut, cut['key']))
        js['stills'].append(dict(file=path.split('\\')[-1], kind='cut', key=cut['key'], title=cut['title'],
                                 pts=pts, cam=W.cam_dict(), fails=nfail))
        for occ in made:
            occ.deleteMe()
        print(cut['key'], 'boolean failures', nfail)
    W.place(G, zero, visible=set(allg))
    js['cams'] = cams
    js['names'] = W.NAMES
    W.dump('stills.json', js)
    print('done', len(js['stills']))
