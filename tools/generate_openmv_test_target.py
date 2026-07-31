#!/usr/bin/env python3
"""Generate the OpenMV calibration target used by the formal detector."""
from pathlib import Path
from PIL import Image, ImageDraw

OUTER_DIAMETER_MM = 160.0
INNER_DIAMETER_MM = 96.0
CROSS_LENGTH_MM = 64.0
CROSS_LINE_WIDTH_MM = 8.0
PAGE_WIDTH_MM = 210.0
PAGE_HEIGHT_MM = 297.0
MARGIN_MM = 20.0
DPI = 300

ASSETS_DIR = Path(__file__).resolve().parents[1] / 'docs' / 'assets'
SVG_PATH = ASSETS_DIR / 'openmv_test_target.svg'
PNG_PATH = ASSETS_DIR / 'openmv_test_target.png'


def mm_to_px(value_mm):
    return int(round(value_mm / 25.4 * DPI))


def generate_svg(path):
    cx = PAGE_WIDTH_MM / 2.0
    cy = PAGE_HEIGHT_MM / 2.0
    outer_r = OUTER_DIAMETER_MM / 2.0
    inner_r = INNER_DIAMETER_MM / 2.0
    cross_half = CROSS_LENGTH_MM / 2.0
    cross_w = CROSS_LINE_WIDTH_MM / 2.0
    path.write_text(f'''<svg xmlns="http://www.w3.org/2000/svg" width="{PAGE_WIDTH_MM}mm" height="{PAGE_HEIGHT_MM}mm" viewBox="0 0 {PAGE_WIDTH_MM} {PAGE_HEIGHT_MM}">
  <rect width="100%" height="100%" fill="white"/>
  <circle cx="{cx}" cy="{cy}" r="{outer_r}" fill="none" stroke="black" stroke-width="{CROSS_LINE_WIDTH_MM}"/>
  <circle cx="{cx}" cy="{cy}" r="{inner_r}" fill="none" stroke="black" stroke-width="{CROSS_LINE_WIDTH_MM}"/>
  <rect x="{cx - cross_w}" y="{cy - cross_half}" width="{CROSS_LINE_WIDTH_MM}" height="{CROSS_LENGTH_MM}" fill="black"/>
  <rect x="{cx - cross_half}" y="{cy - cross_w}" width="{CROSS_LENGTH_MM}" height="{CROSS_LINE_WIDTH_MM}" fill="black"/>
  <text x="{MARGIN_MM}" y="{PAGE_HEIGHT_MM - MARGIN_MM}" font-size="6" fill="black">OpenMV calibration target only · outer 160 mm · inner 96 mm · ratio 0.60</text>
</svg>\n''', encoding='utf-8')


def generate_png(path):
    width_px = mm_to_px(PAGE_WIDTH_MM)
    height_px = mm_to_px(PAGE_HEIGHT_MM)
    image = Image.new('L', (width_px, height_px), 255)
    draw = ImageDraw.Draw(image)
    cx = width_px // 2
    cy = height_px // 2
    outer_r = mm_to_px(OUTER_DIAMETER_MM) // 2
    inner_r = mm_to_px(INNER_DIAMETER_MM) // 2
    stroke = max(1, mm_to_px(CROSS_LINE_WIDTH_MM))
    cross_half = mm_to_px(CROSS_LENGTH_MM) // 2
    draw.ellipse((cx - outer_r, cy - outer_r, cx + outer_r, cy + outer_r), outline=0, width=stroke)
    draw.ellipse((cx - inner_r, cy - inner_r, cx + inner_r, cy + inner_r), outline=0, width=stroke)
    draw.rectangle((cx - stroke // 2, cy - cross_half, cx + stroke // 2, cy + cross_half), fill=0)
    draw.rectangle((cx - cross_half, cy - stroke // 2, cx + cross_half, cy + stroke // 2), fill=0)
    image.save(path)


def main():
    ASSETS_DIR.mkdir(parents=True, exist_ok=True)
    generate_svg(SVG_PATH)
    generate_png(PNG_PATH)
    print(SVG_PATH)
    print(PNG_PATH)


if __name__ == '__main__':
    main()
