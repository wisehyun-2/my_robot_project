"""Korean labels / step headers on the raw Fusion captures (<WORK>/raw/*.png + stills.json).

    python annotate.py <raw dir> <out dir>
"""
import json
import math
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

FR, FB = 'C:/Windows/Fonts/malgun.ttf', 'C:/Windows/Fonts/malgunbd.ttf'
HEADER, FOOTER = 175, 46
RED, INK, GREY, GREEN = '#c0392b', '#222222', '#777777', '#2e7d32'
NOTE = '색은 부품 구분용 (실제 허브·정강이는 검정 PETG)  ·  정강이는 바퀴 쪽 아래 부분만  ·  moteus 드라이버 컵은 생략  ·  ' \
       '나사·와셔·핀·인서트는 BOM 규격으로 모델링'
N_STEPS = 7


def font(sz, bold=False):
    return ImageFont.truetype(FB if bold else FR, sz)


def tsize(dr, txt, f):
    b = dr.multiline_textbbox((0, 0), txt, font=f, spacing=6)
    return b[2] - b[0], b[3] - b[1]


def header(dr, w, title, text, step=None, tag=None, tag_col=RED):
    title, text = title.replace('\u2212', '-'), text.replace('\u2212', '-')      # no U+2212 in Malgun Gothic
    dr.rectangle([0, 0, w, HEADER], fill='#f4f5f7')
    dr.line([0, HEADER, w, HEADER], fill='#cfd3d8', width=2)
    x = 34
    if tag:
        f = font(30, True)
        tw, th = tsize(dr, tag, f)
        dr.rounded_rectangle([x, 26, x + tw + 28, 26 + th + 22], 10, fill=tag_col)
        dr.text((x + 14, 30), tag, font=f, fill='white')
        x += tw + 50
    dr.text((x, 24), title, font=font(40, True), fill=INK)
    dr.multiline_text((34, 92), text, font=font(25), fill='#333', spacing=8)
    if step:
        bw, gap = 46, 8
        x0 = w - 34 - N_STEPS * (bw + gap) + gap
        for i in range(1, N_STEPS + 1):
            xx = x0 + (i - 1) * (bw + gap)
            on = i == step
            done = i < step
            dr.rounded_rectangle([xx, 30, xx + bw, 30 + bw], 8, fill=RED if on else ('#9aa5b1' if done else 'white'),
                                 outline='#9aa5b1', width=2)
            f = font(24, True)
            tw, th = tsize(dr, str(i), f)
            dr.text((xx + (bw - tw) / 2, 30 + (bw - th) / 2 - 6), str(i), font=f, fill='white' if (on or done) else GREY)


def footer(dr, w, h):
    dr.rectangle([0, h - FOOTER, w, h], fill='white')
    dr.text((34, h - FOOTER + 10), NOTE, font=font(19), fill=GREY)


def ink_fraction(img, box):
    x0, y0, x1, y1 = (int(v) for v in box)
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(img.width, x1), min(img.height, y1)
    if x1 <= x0 or y1 <= y0:
        return 1.0
    h = img.crop((x0, y0, x1, y1)).convert('L').histogram()
    dark = sum(h[:238])
    return dark / max(1, sum(h))


def overlap(a, b):
    w = min(a[2], b[2]) - max(a[0], b[0])
    h = min(a[3], b[3]) - max(a[1], b[1])
    return w * h if w > 0 and h > 0 else 0


def dashed(dr, p0, p1, col, width=4, dash=16, gap=10):
    L = math.dist(p0, p1)
    if L < 1:
        return
    ux, uy = (p1[0] - p0[0]) / L, (p1[1] - p0[1]) / L
    s = 0.0
    while s < L - 28:
        e = min(s + dash, L - 28)
        dr.line([(p0[0] + ux * s, p0[1] + uy * s), (p0[0] + ux * e, p0[1] + uy * e)], fill=col, width=width)
        s += dash + gap
    tip = p1
    a = math.atan2(uy, ux)
    pts = [tip, (tip[0] - 30 * math.cos(a - 0.4), tip[1] - 30 * math.sin(a - 0.4)),
           (tip[0] - 30 * math.cos(a + 0.4), tip[1] - 30 * math.sin(a + 0.4))]
    dr.polygon(pts, fill=col)


def _inside(p, b, m=4):
    return b[0] - m < p[0] < b[2] + m and b[1] - m < p[1] < b[3] + m


def _cross(p1, p2, q1, q2):
    def o(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    return (o(p1, p2, q1) * o(p1, p2, q2) < 0) and (o(q1, q2, p1) * o(q1, q2, p2) < 0)


def _leader_end(a, box):
    return (min(max(a[0], box[0]), box[2]), min(max(a[1], box[1]), box[3]))


def place_labels(dr, img, labels, area, avoid=()):
    """labels: list of (text, anchor (x, y), style).  Greedy: free white space, no overlaps, leaders that cross
    neither other labels nor other leaders, near the anchor."""
    placed = list(avoid)
    leaders = []
    anchors = [a for _t, a, _s in labels]
    out = []
    for txt, (ax, ay), style in sorted(labels, key=lambda l: (l[1][1], l[1][0])):
        f = font(25, style in ('hot', 'new'))
        tw, th = tsize(dr, txt, f)
        bw, bh = tw + 26, th + 18
        best = None
        for r in (70, 115, 165, 225, 300, 390):
            for ang in range(-180, 180, 15):
                cx = ax + r * math.cos(math.radians(ang))
                cy = ay + r * math.sin(math.radians(ang))
                box = (cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2)
                pen = r * 0.5
                if box[0] < area[0] or box[1] < area[1] or box[2] > area[2] or box[3] > area[3]:
                    continue
                pen += sum(overlap(box, b) for b in placed) * 40
                pen += sum(3000 for p in anchors if _inside(p, box, 8))
                pen += ink_fraction(img, box) * 2600
                e = _leader_end((ax, ay), box)
                seg = [(ax + (e[0] - ax) * t / 16, ay + (e[1] - ay) * t / 16) for t in range(1, 16)]
                pen += sum(250 for p in seg for b in placed if _inside(p, b, 2))
                pen += sum(2500 for (q1, q2) in leaders if _cross((ax, ay), e, q1, q2))
                pen += sum(1500 for (q1, q2) in leaders for t in range(1, 10)
                           if _inside((q1[0] + (q2[0] - q1[0]) * t / 10, q1[1] + (q2[1] - q1[1]) * t / 10), box, 2))
                if best is None or pen < best[0]:
                    best = (pen, box, e)
        box = best[1]
        placed.append(box)
        leaders.append(((ax, ay), best[2]))
        out.append((txt, (ax, ay), style, box, f))
    for txt, (ax, ay), style, box, f in out:
        col = RED if style == 'hot' else (GREEN if style in ('feat', 'new') else '#555')
        qx = min(max(ax, box[0]), box[2])
        qy = min(max(ay, box[1]), box[3])
        dr.line([(ax, ay), (qx, qy)], fill=col, width=3)
        dr.ellipse([ax - 6, ay - 6, ax + 6, ay + 6], fill=col, outline='white', width=2)
    for txt, (ax, ay), style, box, f in out:
        if style in ('hot', 'new'):
            dr.rounded_rectangle(box, 9, fill=RED if style == 'hot' else GREEN)
            dr.multiline_text((box[0] + 13, box[1] + 6), txt, font=f, fill='white', spacing=6)
        else:
            col = GREEN if style == 'feat' else '#555'
            dr.rounded_rectangle(box, 9, fill='white', outline=col, width=3)
            dr.multiline_text((box[0] + 13, box[1] + 6), txt, font=f, fill=INK, spacing=6)


def content_rows(im, margin):
    """Vertical crop of the raw capture: content (non-white) rows +- margin."""
    g = im.convert('L').point(lambda v: 255 if v < 236 else 0)
    bb = g.getbbox()
    if bb is None:
        return 0, im.height
    return max(0, bb[1] - margin), min(im.height, bb[3] + margin)


def compose(raw, rec, names):
    im = Image.open(raw).convert('RGB')
    y0, y1 = content_rows(im, 230)
    im = im.crop((0, y0, im.width, y1))
    W, H = im.size
    W = max(W, 1500)
    cv = Image.new('RGB', (W, H + HEADER + FOOTER), 'white')
    cv.paste(im, (0, HEADER))
    dr = ImageDraw.Draw(cv)
    k = rec['kind']
    off = lambda p: (p[0], p[1] - y0 + HEADER)
    labels, avoid = [], []
    if k == 'overview':
        header(dr, W, '바퀴 출력부 분해도  (왼쪽 다리)',
               '모든 부품이 떨어진 상태 — 오른쪽이 바깥(모터 쪽), 왼쪽이 로봇 중심 쪽.  모두 바퀴 축 위에서 축 방향으로 끼워진다.\n'
               '체결 부품: ① M2.5 × 8 × 6  ② M3 × 16 × 6 + 와셔  ③ 평행핀 × 3  ④ M3 × 8 × 6 + 인서트  ⑤ M5 × 50  ⑥ M5 너트')
    elif k in ('before', 'after'):
        header(dr, W, rec['title'], rec['text'], step=rec['step'], tag='조립 전' if k == 'before' else '조립 후',
               tag_col='#7f8c8d' if k == 'before' else GREEN)
    elif k == 'done':
        header(dr, W, '조립 완료 — 바퀴 출력부',
               '로봇 무게: 바닥 → 타이어 → 17 mm 육각 → 육각 허브 → (④ + Ø20 턱) → 허브 플랜지 → (② + ③ + Ø11.8 스피곳)\n'
               '→ AKE 출력부 → 모터 베어링 → 고정자 → (① + Ø42 파일럿) → 정강이', tag='완성', tag_col=GREEN)
    else:
        header(dr, W, rec['title'], '바퀴 축을 지나는 두 평면 사이를 잘라 냄 — 나사·핀이 어디까지 들어가는지 보인다.',
               tag='단면', tag_col='#2c3e50')
    if k == 'cut':
        for name, p in rec['pts'].items():
            labels.append((name, off(p), 'feat' if name[0] in '①②③④⑤⑥ABC1' else 'part'))
    else:
        mv = set(rec.get('moving', []))
        major = {'SHANK', 'MOTOR', 'FLANGE', 'HEX', 'WHEEL'}
        for g, p in rec['proj'].items():
            if k in ('before', 'after') and g not in mv and g not in major:
                continue                                   # small parts of earlier steps are hidden by now
            if k == 'done' and g not in ('SHANK', 'MOTOR', 'WHEEL', 'NUT'):
                continue
            style = 'part'
            if g in mv:
                style = 'hot' if k == 'before' else 'new'
            labels.append((names[g], off(p), style))
    if k == 'before':
        for g, (p0, p1) in rec.get('arrows', {}).items():
            a, b = off(p0), off(p1)
            if math.dist(a, b) > 40:
                dashed(dr, a, b, RED)
                avoid.append((min(a[0], b[0]) - 8, min(a[1], b[1]) - 8, max(a[0], b[0]) + 8, max(a[1], b[1]) + 8))
    place_labels(dr, cv, labels, (10, HEADER + 10, W - 10, H + HEADER - 10), avoid=())
    footer(dr, W, cv.height)
    return cv


def main():
    src, dst = Path(sys.argv[1]), Path(sys.argv[2])
    dst.mkdir(parents=True, exist_ok=True)
    js = json.load(open(src / 'stills.json', encoding='utf-8'))
    for rec in js['stills']:
        cv = compose(src / rec['file'], rec, js['names'])
        cv.save(dst / rec['file'])
        print(rec['file'])


if __name__ == '__main__':
    main()
