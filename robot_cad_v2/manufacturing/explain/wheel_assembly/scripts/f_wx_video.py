"""Video frames of the wheel output assembly: overview -> 7 steps (parts slide along the axis, screws turn in) ->
finished joint orbit.  Writes <WORK>/frames/fNNNN.png + frames/manifest.json (hold counts, step, caption).
Cameras come from <WORK>/raw/stills.json so the frames match the stills."""
import importlib
import json
import math
import os
import sys
SP = r'C:\Users\User\Desktop\myrobot\my_robot_project\robot_cad_v2\manufacturing\explain\wheel_assembly\scripts'
FPS = 30
MOVE, LONG_MOVE, CAM = 36, 56, 20
CAPTION = dict(MOTOR='AKE60-8 → 정강이 포켓 (Ø42 파일럿)', RING='SUS304 클램프 링 → 판 안쪽 홈',
               S25='① M2.5 × 8 저두 × 6 조임', INS='황동 인서트 M3 × 6 열압입', PINS='③ 평행핀 Ø3 × 10 × 3 → 출력부 구멍',
               FLANGE='허브 플랜지 → 출력부 (Ø11.8 스피곳)', WASH='평와셔 M3 × 6', S316='② M3 × 16 (12.9) × 6 조임',
               BOLT='⑤ M5 × 50 → 육각 허브', WHEEL='바퀴 → 17 mm 육각', NUT='⑥ M5 플랜지 너트 조임',
               HEX='육각 허브 + 바퀴 → 허브 플랜지 (Ø20 턱)', S38='④ M3 × 8 × 6 → 인서트 (스포크 사이로)')


def ease(t):
    return t * t * (3 - 2 * t)


def run(_context: str):
    if SP not in sys.path:
        sys.path.insert(0, SP)
    import adsk.core
    sys.modules.pop('wxlib', None)               # a same-named module from another folder may be cached
    import wxlib as W
    print('wxlib', W.__file__, 'work', W.WORK)
    d, dd = W.doc()
    G = W.groups(dd)
    allg = set(G)
    vp = adsk.core.Application.get().activeViewport
    vp.visualStyle = adsk.core.VisualStyles.ShadedWithVisibleEdgesOnlyVisualStyle
    W.grid_off()
    cams = json.load(open(os.path.join(W.OUT, 'stills.json'), encoding='utf-8'))['cams']
    out = W.FRAMES
    os.makedirs(out, exist_ok=True)
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
            W.set_cam(lerp_cam(ca, cb, ease(i / n)))
            frame(step, caption)
        state['cam'] = cb

    # overview
    off = dict(W.HOME)
    W.place(G, off, visible=allg)
    W.set_cam(cams['overview'])
    frame(0, '바퀴 출력부 — 분해 상태', hold=int(2.5 * FPS))
    for k, st in enumerate(W.states(), 1):
        vis = set(st['vis'])
        W.place(G, off, visible=vis)
        W.set_cam(cams[st['key']])
        state['cam'] = cams[st['key']]
        frame(k, st['title'], hold=int(1.2 * FPS))
        for mv in st['seq']:
            cap = CAPTION[next(iter(mv))]
            n = LONG_MOVE if max(abs(b - a) for a, b in mv.values()) > 250 else MOVE
            for i in range(1, n + 1):
                t = ease(i / n)
                cur = dict(off)
                spin = {}
                for g, (a, b) in mv.items():
                    cur[g] = a + (b - a) * t
                    if g in st.get('spin', []):
                        spin[g] = -2 * math.pi * 3.0 * t
                W.place(G, cur, spin)
                frame(k, cap)
            for g, (a, b) in mv.items():
                off[g] = b
            man[-1]['hold'] = int(0.35 * FPS)
        go_cam(cams[st['key'] + '_after'], k, st['title'] + ' — 완료')
        man[-1]['hold'] = int(1.0 * FPS)
    # finished joint + orbit about the vertical through the target
    W.place(G, off, visible=allg)
    go_cam(cams['done'], 8, '조립 완료')
    man[-1]['hold'] = int(1.0 * FPS)
    c0 = cams['done']
    tx, ty, tz = c0['target']
    ex, ey, ez = c0['eye']
    n = 70
    for i in range(1, n + 1):
        a = math.radians(-50.0) * math.sin(math.pi * i / n)
        dx, dz = ex - tx, ez - tz
        cd = dict(c0, eye=[tx + dx * math.cos(a) - dz * math.sin(a), ey, tz + dx * math.sin(a) + dz * math.cos(a)])
        W.set_cam(cd)
        frame(8, '조립 완료')
    man[-1]['hold'] = int(0.8 * FPS)
    W.set_cam(c0)
    with open(os.path.join(out, 'manifest.json'), 'w', encoding='utf-8') as f:
        json.dump(dict(fps=FPS, size=[vp.width, vp.height], frames=man), f, ensure_ascii=False)
    print('frames', len(man), 'video s', sum(m['hold'] for m in man) / FPS)
