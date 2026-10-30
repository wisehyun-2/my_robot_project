"""Video frames of a job: overview -> steps (parts slide in, fasteners turn in) -> finished assembly orbit.
Writes <WORK>/<job>/frames/fNNNN.png + frames/manifest.json; cameras from raw/stills.json (run f_ax_stills.py first).

    python fusion_rpc.py f_ax_video.py 1800 <job>
"""
import json
import math
import os
import sys
HERE = r'C:\Users\User\Desktop\myrobot\my_robot_project\robot_cad_v2\manufacturing\explain\assembly\scripts'
FPS, CAM = 30, 20


def ease(t):
    return t * t * (3 - 2 * t)


def run(_context: str):
    if HERE not in sys.path:
        sys.path.insert(0, HERE)
    sys.modules.pop('axlib', None)
    import axlib as X
    job = JOB  # noqa: F821
    cfg, M = X.load_job(job)
    d, dd = X.doc(job)
    G = X.groups(dd)
    allg = set(G)
    vp = X.setup_view()
    cams = json.load(open(os.path.join(X.work(job, 'raw'), 'stills.json'), encoding='utf-8'))['cams']
    out = X.work(job, 'frames')
    for f in os.listdir(out):
        os.remove(os.path.join(out, f))
    man = []
    state = dict(cam=cams['overview'])

    def frame(step, caption, hold=1):
        name = 'f%04d.png' % len(man)
        vp.saveAsImageFile(os.path.join(out, name), vp.width, vp.height)
        man.append(dict(file=name, hold=hold, step=step, caption=caption))

    def lerp_cam(a, b, t):
        L = lambda u, v: [u[i] + (v[i] - u[i]) * t for i in range(3)]
        up = L(a['up'], b['up'])
        n = math.sqrt(sum(c * c for c in up))
        return dict(eye=L(a['eye'], b['eye']), target=L(a['target'], b['target']), up=[c / n for c in up],
                    ext=a['ext'] + (b['ext'] - a['ext']) * t)

    def go_cam(cb, step, caption, n=CAM):
        ca = state['cam']
        for i in range(1, n + 1):
            X.set_cam(lerp_cam(ca, cb, ease(i / n)))
            frame(step, caption)
        state['cam'] = cb

    off = {g: tuple(v) for g, v in cfg['home'].items()}
    X.place(G, off, visible=allg)
    X.set_cam(cams['overview'])
    frame(0, cfg['overview'][0].split('  ')[0], hold=int(2.5 * FPS))
    sts = X.states(cfg)
    for k, st in enumerate(sts, 1):
        X.place(G, off, visible=set(st['vis']))
        X.set_cam(cams[st['key']])
        state['cam'] = cams[st['key']]
        frame(k, st['title'], hold=int(1.2 * FPS))
        for mv in st['seq']:
            cap = cfg['captions'].get(next(iter(mv)), st['title'])
            dist = max(math.sqrt(sum((b[i] - a[i]) ** 2 for i in range(3))) + abs(b[3] - a[3]) for a, b in mv.values())
            n = cfg['long_move'] if dist > 250 else cfg['move']
            for i in range(1, n + 1):
                t = ease(i / n)
                cur = dict(off)
                spin = {}
                for g, (a, b) in mv.items():
                    cur[g] = X.lerp4(a, b, t)
                    if g in st.get('spin', []):
                        spin[g] = -2 * math.pi * 3.0 * t
                X.place(G, cur, spin)
                frame(k, cap)
            for g, (a, b) in mv.items():
                off[g] = b
            man[-1]['hold'] = int(0.35 * FPS)
        go_cam(cams[st['key'] + '_after'], k, st['title'] + ' — 완료')
        man[-1]['hold'] = int(1.0 * FPS)
    n_st = len(sts)
    X.place(G, off, visible=allg)
    go_cam(cams['done'], n_st + 1, '조립 완료')
    man[-1]['hold'] = int(1.0 * FPS)
    c0 = cams['done']
    tx, ty, tz = c0['target']
    ex, ey, ez = c0['eye']
    n = 70
    for i in range(1, n + 1):
        a = math.radians(-50.0) * math.sin(math.pi * i / n)
        dx, dz = ex - tx, ez - tz
        X.set_cam(dict(c0, eye=[tx + dx * math.cos(a) - dz * math.sin(a), ey, tz + dx * math.sin(a) + dz * math.cos(a)]))
        frame(n_st + 1, '조립 완료')
    man[-1]['hold'] = int(0.8 * FPS)
    X.set_cam(c0)
    with open(os.path.join(out, 'manifest.json'), 'w', encoding='utf-8') as f:
        json.dump(dict(fps=FPS, size=[vp.width, vp.height], n_steps=n_st, frames=man), f, ensure_ascii=False)
    print(job, 'frames', len(man), 'video s %.1f' % (sum(m['hold'] for m in man) / FPS))
