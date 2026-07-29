#!/usr/bin/env python3
"""Generate parameterized synthetic D-target images for regression only."""

import argparse
import math
from pathlib import Path

import cv2
import numpy as np


def generate_target(width=320, height=240, center=(160, 120),
                    outer_diameter=120, ratio=0.6, angle=0.0,
                    gradient=0.0, shadow=False, highlight=False,
                    blur=0, noise=0.0, occlusion=0.0,
                    draw_outer=True, draw_inner=True, draw_cross=True,
                    distractor=False):
    yy, xx = np.mgrid[0:height, 0:width]
    base = 220.0 + gradient * (xx / max(1, width - 1) - 0.5) * 120
    image = np.clip(base, 50, 255).astype(np.uint8)
    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    cx, cy = map(int, center)
    outer_r = max(2, round(outer_diameter / 2))
    inner_r = max(1, round(outer_diameter * ratio / 2))
    thickness = max(3, round(outer_diameter * 0.055))
    if draw_outer:
        cv2.circle(image, (cx, cy), outer_r, (15, 15, 15), thickness)
    if draw_inner:
        cv2.circle(image, (cx, cy), inner_r, (15, 15, 15), thickness)
    if draw_cross:
        arm = max(6, round(inner_r * 0.65))
        for theta in (angle, angle + math.pi / 2):
            dx, dy = round(arm * math.cos(theta)), round(arm * math.sin(theta))
            cv2.line(image, (cx - dx, cy - dy), (cx + dx, cy + dy),
                     (10, 10, 10), max(2, thickness // 2))
    if shadow:
        overlay = image.copy()
        cv2.ellipse(overlay, (width // 3, height // 2),
                    (width // 3, height), 20, 0, 360, (30, 30, 30), -1)
        image = cv2.addWeighted(image, 0.65, overlay, 0.35, 0)
    if highlight:
        cv2.circle(image, (width * 3 // 4, height // 4),
                   max(8, width // 12), (255, 255, 255), -1)
    if occlusion > 0:
        cover = max(1, round(outer_diameter * min(1.0, occlusion)))
        cv2.rectangle(image, (cx, cy - outer_r),
                      (cx + cover, cy + outer_r), (180, 180, 180), -1)
    if distractor:
        cv2.circle(image, (45, 45), 30, (20, 20, 20), 5)
        cv2.line(image, (10, height - 20), (width - 10, height - 50),
                 (20, 20, 20), 6)
    if blur:
        kernel = int(blur) | 1
        image = cv2.GaussianBlur(image, (kernel, kernel), 0)
    if noise:
        rng = np.random.default_rng(2026)
        perturbation = rng.normal(0, noise, image.shape).astype(np.int16)
        image = np.clip(image.astype(np.int16) + perturbation,
                        0, 255).astype(np.uint8)
    return image


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    parser.add_argument('--count', type=int, default=24)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for index in range(args.count):
        angle = index * math.pi / max(1, args.count)
        image = generate_target(
            center=(80 + index * 160 // max(1, args.count - 1),
                    70 + index * 100 // max(1, args.count - 1)),
            outer_diameter=70 + index % 5 * 12, angle=angle,
            gradient=(index % 3) / 2, shadow=index % 4 == 0,
            blur=5 if index % 5 == 0 else 0,
            noise=5 if index % 3 == 0 else 0)
        cv2.imwrite(str(args.output / ('synthetic_%03d.png' % index)), image)


if __name__ == '__main__':
    main()
