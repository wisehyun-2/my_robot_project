"""Stills of a job: exploded overview, before / after of every step, finished assembly, cutaways.
Writes <WORK>/<job>/raw/*.png + raw/stills.json (cameras, label anchors in view pixels, label texts).

    python fusion_rpc.py f_ax_stills.py 900 <job>
"""
import os
import sys
HERE = r'C:\Users\User\Desktop\myrobot\my_robot_project\robot_cad_v2\manufacturing\explain\assembly\scripts'


def run(_context: str):
    if HERE not in sys.path:
        sys.path.insert(0, HERE)
    sys.modules.pop('axlib', None)
    import adsk.core
    import axlib as X
    job = JOB  # noqa: F821
    cfg, M = X.load_job(job)
    C = M['cadlib']
    d, dd = X.doc(job)
    G = X.groups(dd)
    allg = list(G)
    vp = X.setup_view()
    raw = X.work(job, 'raw')
    for f in os.listdir(raw):
        os.remove(os.path.join(raw, f))
    js = dict(size=[vp.width, vp.height], stills=[], cams={})
    tgt = cfg['target']

    def shot(name, vis, extra):
        X.snap(os.path.join(raw, name))
        rec = dict(file=name, vis=list(vis), proj=X.project(G, vis), cam=X.cam_dict())
        rec.update(extra)
        js['stills'].append(rec)

    X.place(G, cfg['home'], visible=set(allg))
    X.set_view(cfg['view0'], tgt)
    js['cams']['overview'] = X.cam_dict()
    shot('00_overview.png', allg, dict(kind='overview'))
    sts = X.states(cfg)
    for k, st in enumerate(sts, 1):
        vis = set(st['vis'])
        X.place(G, st['before'], visible=vis)
        X.set_view(st['view'], tgt)
        X.recenter(None, 0.92)
        js['cams'][st['key']] = X.cam_dict()
        arrows = {}
        for g in st['groups']:
            c = X.centre(G, g)
            a, b = st['before'][g], st['after'][g]
            w0 = X.rf(a[:3])
            w1 = X.rf(b[:3])
            ax = next((r[1] for r in G[g] if r[1] is not None), None)
            da = (b[3] - a[3])
            dv = [w1[i] - w0[i] - (da * ax[3 + i] if ax else 0.0) for i in range(3)]
            arrows[g] = [X.project_world(c), X.project_world([c[i] + dv[i] for i in range(3)])]
        info = dict(step=k, moving=st['groups'], title=st['title'], text=st['text'])
        shot('%02d_%s_a_before.png' % (k, st['key']), vis, dict(info, kind='before', arrows=arrows))
        X.place(G, st['after'], visible=vis)
        X.set_view(st['view'], tgt)
        X.recenter(None, 0.9)
        js['cams'][st['key'] + '_after'] = X.cam_dict()
        shot('%02d_%s_b_after.png' % (k, st['key']), vis, dict(info, kind='after'))
    n = len(sts)
    zero = {g: X.Z4 for g in allg}
    X.place(G, zero, visible=set(allg))
    X.set_view(cfg['view_done'], tgt)
    X.recenter(None, 0.95)
    js['cams']['done'] = X.cam_dict()
    shot('%02d_done.png' % (n + 1), allg, dict(kind='done'))
    # cutaways: copies of every body minus the removal region
    B = C.Builder()
    root = dd.rootComponent
    for ci, cut in enumerate(cfg['cuts']):
        region = None
        for (p, nrm) in cut['remove']:
            h = B.halfspace(p, nrm)
            region = h if region is None else C.inter(region, h)
        made, nfail = [], 0
        for g in allg:
            for r in G[g]:
                o = r[0]
                occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
                occ.component.name = 'CUT_' + o.component.name
                for b in o.component.bRepBodies:
                    t = C.TBM.copy(b)
                    try:
                        C.cut(t, C.TBM.copy(region))
                    except Exception:
                        nfail += 1
                    if t.faces.count == 0:
                        continue
                    nb = occ.component.bRepBodies.add(t)
                    nb.appearance = b.appearance
                made.append(occ)
        for g in allg:
            for r in G[g]:
                r[0].isLightBulbOn = False
        X.set_view(cut['view'], tgt)
        X.recenter(cut['target'], cut.get('zoom', 0.7))
        js['cams'][cut['key']] = X.cam_dict()
        name = '%02d_%s.png' % (n + 2 + ci, cut['key'])
        X.snap(os.path.join(raw, name))
        pts = {lab: X.project_world(X.rf(p)) for lab, p in cut['feats'].items()}
        js['stills'].append(dict(file=name, kind='cut', key=cut['key'], title=cut['title'], pts=pts,
                                 cam=X.cam_dict(), fails=nfail))
        for occ in made:
            occ.deleteMe()
        print(cut['key'], 'boolean failures', nfail)
    X.place(G, zero, visible=set(allg))
    js['meta'] = {k: cfg[k] for k in ('job', 'title', 'subtitle', 'names', 'short', 'legend', 'major', 'done_labels',
                                      'overview', 'done', 'note')}
    js['meta']['overview_labels'] = cfg.get('overview_labels')
    js['meta']['cuts'] = [dict(key=c['key'], title=c['title']) for c in cfg['cuts']]
    X.dump(os.path.join(raw, 'stills.json'), js)
    print(job, 'stills', len(js['stills']))
