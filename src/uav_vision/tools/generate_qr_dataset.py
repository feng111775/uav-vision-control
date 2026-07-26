#!/usr/bin/env python3
"""Generate leak-free synthetic QR shelf detection data and YOLO labels."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


SEED = 20260726


def make_qr(qr_id, pixels=220):
    """Create a standard numeric QR (ECC=M, quiet-zone=4 via OpenCV)."""
    encoder = cv2.QRCodeEncoder_create()
    code = encoder.encode(str(qr_id))
    return cv2.resize(code, (pixels, pixels),
                      interpolation=cv2.INTER_NEAREST)


def shelf_background(rng, width=640, height=480):
    """Create a white multi-level shelf with distractor texture."""
    image = np.full((height, width, 3), rng.integers(215, 256), np.uint8)
    for y in (100, 210, 320, 430):
        cv2.rectangle(image, (15, y), (625, y + 10), (185, 185, 185), -1)
    for x in (15, 165, 315, 465, 625):
        cv2.rectangle(image, (x, 25), (x + 8, 440), (195, 195, 195), -1)
    for _ in range(rng.integers(2, 8)):
        x, y = rng.integers(0, width - 30), rng.integers(0, height - 30)
        cv2.rectangle(image, (x, y), (x + rng.integers(8, 35),
                      y + rng.integers(8, 35)),
                      tuple(int(v) for v in rng.integers(40, 220, 3)), 1)
    return image


def paste_perspective(image, code, rng):
    """Project one code and return its clipped YOLO box."""
    size = int(rng.integers(45, 185))
    x = int(rng.integers(-size // 3, image.shape[1] - 2 * size // 3))
    y = int(rng.integers(-size // 3, image.shape[0] - 2 * size // 3))
    jitter = max(2, int(size * 0.22))
    source = np.float32([[0, 0], [code.shape[1] - 1, 0],
                         [code.shape[1] - 1, code.shape[0] - 1],
                         [0, code.shape[0] - 1]])
    target = np.float32([
        [x + rng.integers(-jitter, jitter), y + rng.integers(-jitter, jitter)],
        [x + size + rng.integers(-jitter, jitter),
         y + rng.integers(-jitter, jitter)],
        [x + size + rng.integers(-jitter, jitter),
         y + size + rng.integers(-jitter, jitter)],
        [x + rng.integers(-jitter, jitter),
         y + size + rng.integers(-jitter, jitter)]])
    transform = cv2.getPerspectiveTransform(source, target)
    warped = cv2.warpPerspective(
        code, transform, (image.shape[1], image.shape[0]),
        borderValue=255)
    mask = cv2.warpPerspective(
        np.full(code.shape, 255, np.uint8), transform,
        (image.shape[1], image.shape[0]), borderValue=0)
    image[mask > 0] = cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)[mask > 0]
    x0, y0 = np.maximum(target.min(axis=0), (0, 0))
    x1, y1 = np.minimum(
        target.max(axis=0), (image.shape[1] - 1, image.shape[0] - 1))
    return [float(x0), float(y0), float(x1 - x0), float(y1 - y0)]


def augment(image, rng, family):
    """Apply deterministic lighting, blur, noise, compression and shadow."""
    alpha = float(rng.uniform(0.55, 1.35))
    beta = float(rng.uniform(-30, 30))
    image = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
    if family % 3 == 0:
        kernel = int(rng.choice([3, 5, 7]))
        image = cv2.GaussianBlur(image, (kernel, kernel), 0)
    if family % 5 == 0:
        kernel = np.zeros((9, 9), np.float32)
        kernel[4, :] = 1.0 / 9
        image = cv2.filter2D(image, -1, kernel)
    if family % 4 == 0:
        noise = rng.normal(0, rng.uniform(4, 18), image.shape)
        image = np.clip(image.astype(float) + noise, 0, 255).astype(np.uint8)
    if family % 7 == 0:
        overlay = image.copy()
        cv2.rectangle(overlay, (0, rng.integers(40, 260)),
                      (640, rng.integers(300, 470)), (30, 30, 30), -1)
        image = cv2.addWeighted(image, 0.72, overlay, 0.28, 0)
    quality = int(rng.integers(35, 96))
    return cv2.imdecode(cv2.imencode(
        '.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, quality])[1],
        cv2.IMREAD_COLOR)


def generate(output, train=160, val=48, test=48, seed=SEED):
    """Generate splits using disjoint family RNG streams."""
    output = Path(output)
    counts = {'train': train, 'val': val, 'test': test}
    stats = {'seed': seed, 'class_names': ['qr_code'],
             'qr_payloads': list(range(1, 25)), 'physical_size_m': 0.19,
             'qr': {'error_correction': 'M', 'quiet_zone_modules': 4},
             'splits': {}, 'conditions': {}}
    for split_index, (split, count) in enumerate(counts.items()):
        (output / 'images' / split).mkdir(parents=True, exist_ok=True)
        (output / 'labels' / split).mkdir(parents=True, exist_ok=True)
        positives = boxes_total = 0
        for index in range(count):
            family = split_index * 100000 + index
            rng = np.random.default_rng(seed + family)
            image = shelf_background(rng)
            boxes = []
            # 15% negatives; otherwise 1..4 codes.
            target_count = 0 if rng.random() < 0.15 else int(
                rng.integers(1, 5))
            for _ in range(target_count):
                qr_id = int(rng.integers(1, 25))
                box = paste_perspective(image, make_qr(qr_id), rng)
                if box[2] >= 8 and box[3] >= 8:
                    boxes.append(box)
            # Partial occlusion / overexposure.
            if boxes and family % 6 == 0:
                x, y, width, height = boxes[0]
                cv2.rectangle(image, (int(x + width * .65), int(y)),
                              (int(x + width), int(y + height * .3)),
                              (255, 255, 255), -1)
            image = augment(image, rng, family)
            stem = f'{split}_{index:04d}'
            cv2.imwrite(str(output / 'images' / split / f'{stem}.jpg'),
                        image)
            lines = []
            for x, y, width, height in boxes:
                lines.append('0 %.6f %.6f %.6f %.6f' % (
                    (x + width / 2) / image.shape[1],
                    (y + height / 2) / image.shape[0],
                    width / image.shape[1], height / image.shape[0]))
            (output / 'labels' / split / f'{stem}.txt').write_text(
                '\n'.join(lines), encoding='utf-8')
            positives += bool(boxes)
            boxes_total += len(boxes)
        stats['splits'][split] = {
            'images': count, 'positive_images': positives,
            'negative_images': count - positives, 'boxes': boxes_total,
            'family_range': [split_index * 100000,
                             split_index * 100000 + count - 1]}
    (output / 'dataset.yaml').write_text(
        'path: %s\ntrain: images/train\nval: images/val\n'
        'test: images/test\nnames:\n  0: qr_code\n' % output.resolve(),
        encoding='utf-8')
    (output / 'statistics.json').write_text(
        json.dumps(stats, indent=2), encoding='utf-8')
    return stats


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='datasets/qr_shelf')
    parser.add_argument('--train', type=int, default=160)
    parser.add_argument('--val', type=int, default=48)
    parser.add_argument('--test', type=int, default=48)
    parser.add_argument('--seed', type=int, default=SEED)
    args = parser.parse_args()
    print(json.dumps(generate(
        args.output, args.train, args.val, args.test, args.seed), indent=2))


if __name__ == '__main__':
    main()
