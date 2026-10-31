"""1920 x 1080 video from the Fusion frames: render on the left, step list + description on the right, caption band;
the annotated cutaways at the end.  ffmpeg concat (per-frame durations) -> H.264 mp4.

    python compose_video.py <WORK dir> <dir of the annotated stills> <out mp4>
"""
import json
import subprocess
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

FFMPEG = r'C:\ffmpeg\ffmpeg-2026-10-01-git-0b01ed76aa-essentials_build\bin\ffmpeg.exe'
FR, FB = 'C:/Windows/Fonts/malgun.ttf', 'C:/Windows/Fonts/malgunbd.ttf'
W, H = 1920, 1080
RED, GREEN, INK, GREY = '#c0392b', '#2e7d32', '#222222', '#8a929a'
SHORT = ['모터 → 정강이 포켓', 'SUS 링 + ① M2.5 × 8', '인서트 → 허브 플랜지', '허브 플랜지 → 출력부 (③ ②)',
         '⑤ M5 볼트 → 육각 허브', '바퀴 + ⑥ 너트', '육각 허브 → 플랜지 (④)']
LEGEND = ['① M2.5 × 8 저두 × 6', '② M3 × 14 (12.9) × 6 + 와셔', '③ 평행핀 Ø3 × 10 × 3', '④ M3 × 8 × 6 + 인서트',
          '⑤ M5 × 55', '⑥ M5 플랜지 너트']


def font(sz, bold=False):
    return ImageFont.truetype(FB if bold else FR, sz)


def wrap(dr, text, f, width):
    lines = []
    for para in text.split('\n'):
        cur = ''
        for ch in para:
            if dr.textlength(cur + ch, font=f) > width and cur:
                cut = cur.rfind(' ')
                if cut > len(cur) * 0.5:
                    lines.append(cur[:cut])
                    cur = cur[cut + 1:] + ch
                else:
                    lines.append(cur)
                    cur = ch
            else:
                cur += ch
        lines.append(cur)
    return lines


def panel(dr, x0, step, steps):
    dr.rectangle([x0, 0, W, H], fill='#f4f5f7')
    dr.line([x0, 0, x0, H], fill='#cfd3d8', width=2)
    x = x0 + 28
    dr.text((x, 26), '바퀴 출력부 조립', font=font(34, True), fill=INK)
    dr.text((x, 74), '왼쪽 다리 · Fusion 모델', font=font(20), fill=GREY)
    y = 122
    for i, name in enumerate(SHORT, 1):
        on, done = i == step, (i < step or step >= 8)
        fill = RED if on else ('#9aa5b1' if done else 'white')
        dr.rounded_rectangle([x, y, x + 40, y + 40], 8, fill=fill, outline='#9aa5b1', width=2)
        f = font(22, True)
        tw = dr.textlength(str(i), font=f)
        dr.text((x + 20 - tw / 2, y + 4), str(i), font=f, fill='white' if (on or done) else GREY)
        dr.text((x + 54, y + 6), name, font=font(22, on), fill=INK if (on or not done) else GREY)
        y += 52
    y += 14
    dr.line([x, y, W - 28, y], fill='#cfd3d8', width=2)
    y += 18
    if 1 <= step <= 7:
        title, text = steps[step]
        for ln in wrap(dr, title.split('  ', 1)[-1], font(24, True), W - x - 30):
            dr.text((x, y), ln, font=font(24, True), fill=RED)
            y += 34
        y += 6
        for ln in wrap(dr, text, font(21), W - x - 30):
            dr.text((x, y), ln, font=font(21), fill='#333')
            y += 31
    elif step == 0:
        for ln in wrap(dr, '모든 부품이 떨어진 상태. 오른쪽이 바깥(모터), 왼쪽이 로봇 중심 쪽이고, 모두 바퀴 축 방향으로 끼워진다.',
                       font(21), W - x - 30):
            dr.text((x, y), ln, font=font(21), fill='#333')
            y += 31
    else:
        for ln in wrap(dr, '무게: 타이어 → 17 mm 육각 → 육각 허브 → ④ + Ø20 턱 → 허브 플랜지 → ② + ③ + Ø11.8 스피곳 → '
                           'AKE 출력부 → 고정자 → ① + Ø42 파일럿 → 정강이', font(21), W - x - 30):
            dr.text((x, y), ln, font=font(21), fill='#333')
            y += 31
    y = H - 30 - 30 * len(LEGEND) - 40
    dr.text((x, y), '체결 부품 (바퀴 한쪽)', font=font(21, True), fill=INK)
    y += 36
    for ln in LEGEND:
        dr.text((x, y), ln, font=font(20), fill='#444')
        y += 30


def caption(cv, text, x1):
    band = Image.new('RGBA', (x1, 64), (30, 34, 40, 200))
    cv.paste(band, (0, H - 64), band)
    dr = ImageDraw.Draw(cv)
    f = font(30, True)
    tw = dr.textlength(text, font=f)
    dr.text(((x1 - tw) / 2, H - 56), text, font=f, fill='white')


def main():
    sp, ann, mp4 = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
    man = json.load(open(sp / 'frames' / 'manifest.json', encoding='utf-8'))
    stills = json.load(open(sp / 'raw' / 'stills.json', encoding='utf-8'))['stills']
    steps = {r['step']: (r['title'], r['text']) for r in stills if r.get('kind') == 'before'}
    fps = man['fps']
    out = sp / 'video'
    out.mkdir(exist_ok=True)
    for f in out.glob('*.png'):
        f.unlink()
    rw = round(man['size'][0] * H / man['size'][1])
    items = []
    for k, fr in enumerate(man['frames']):
        im = Image.open(sp / 'frames' / fr['file']).convert('RGB').resize((rw, H), Image.LANCZOS)
        cv = Image.new('RGB', (W, H), 'white')
        cv.paste(im, (0, 0))
        panel(ImageDraw.Draw(cv), rw, fr['step'], steps)
        caption(cv, fr['caption'], rw)
        name = 'v%04d.png' % k
        cv.save(out / name)
        items.append((name, fr['hold'] / fps))
    for k, (fn, cap) in enumerate((('09_cut1.png', '단면 ① — ② M3 × 14 · ④ M3 × 8 · ⑤ M5 볼트 · A/B/C 중심 맞춤'),
                                   ('10_cut2.png', '단면 ② — ③ 평행핀 · ① M2.5 × 8 + SUS 링'))):
        im = Image.open(ann / fn).convert('RGB')
        s = min(rw / im.width, (H - 64) / im.height)
        im = im.resize((round(im.width * s), round(im.height * s)), Image.LANCZOS)
        cv = Image.new('RGB', (W, H), 'white')
        cv.paste(im, ((rw - im.width) // 2, 0))
        panel(ImageDraw.Draw(cv), rw, 9, steps)
        caption(cv, cap, rw)
        name = 'v%04d.png' % (len(man['frames']) + k)
        cv.save(out / name)
        items.append((name, 4.0))
    lst = out / 'list.txt'
    with open(lst, 'w', encoding='utf-8') as f:
        for name, dur in items:
            f.write("file '%s'\nduration %.4f\n" % (name, dur))
        f.write("file '%s'\n" % items[-1][0])
    cmd = [FFMPEG, '-y', '-loglevel', 'error', '-f', 'concat', '-safe', '0', '-i', str(lst), '-vf', 'fps=%d,format=yuv420p' % fps,
           '-c:v', 'libx264', '-preset', 'slow', '-crf', '20', '-movflags', '+faststart', str(mp4)]
    subprocess.run(cmd, check=True)
    print(mp4, 'duration %.1f s' % sum(d for _n, d in items))


if __name__ == '__main__':
    main()
