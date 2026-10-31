"""Contact sheet of the annotated stills (4 per row, file name above each tile) -> all_steps.png.

    python make_all_steps.py <dir of the annotated stills>
"""
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

COLS, TILE_W, CAP_H = 4, 430, 22


def main():
    d = Path(sys.argv[1] if len(sys.argv) > 1 else '..')
    files = sorted(p for p in d.glob('[0-9][0-9]_*.png'))
    f = ImageFont.truetype('C:/Windows/Fonts/malgun.ttf', 13)
    tiles = []
    for p in files:
        im = Image.open(p).convert('RGB')
        im = im.resize((TILE_W, round(im.height * TILE_W / im.width)), Image.LANCZOS)
        tiles.append((p.name, im))
    rows = [tiles[i:i + COLS] for i in range(0, len(tiles), COLS)]
    row_h = [CAP_H + max(im.height for _n, im in r) for r in rows]
    sheet = Image.new('RGB', (COLS * TILE_W, sum(row_h)), 'white')
    dr = ImageDraw.Draw(sheet)
    y = 0
    for r, h in zip(rows, row_h):
        for c, (name, im) in enumerate(r):
            x = c * TILE_W
            dr.text((x + 4, y + 3), name, font=f, fill='#222222')
            sheet.paste(im, (x, y + CAP_H))
            dr.rectangle((x, y, x + TILE_W - 1, y + h - 1), outline='#9aa0a6')
        y += h
    out = d / 'all_steps.png'
    sheet.save(out, optimize=True)
    print(out, sheet.size, len(tiles))


if __name__ == '__main__':
    main()
