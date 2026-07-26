#!/usr/bin/env python3
"""Compare OpenCV and learned-localizer hybrid on held-out synthetic images."""

import argparse
import json
from pathlib import Path
import sys
import time

import cv2
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1]))
from uav_vision.qr_core import HybridQRDetector  # noqa: E402


def ground_truth(label, shape):
    height, width = shape[:2]
    boxes = []
    for line in label.read_text().splitlines():
        _, cx, cy, w, h = map(float, line.split())
        boxes.append(((cx - w / 2) * width, (cy - h / 2) * height,
                      w * width, h * height))
    return boxes


def iou(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    intersection = max(0, min(ax + aw, bx + bw) - max(ax, bx)) * \
        max(0, min(ay + ah, by + bh) - max(ay, by))
    union = aw * ah + bw * bh - intersection
    return intersection / union if union else 0


def evaluate(dataset, model_path, limit=48):
    """Measure location and content success separately at image level."""
    dataset = Path(dataset)
    methods = {
        'opencv': HybridQRDetector('opencv'),
        'model_hybrid': HybridQRDetector('hybrid', model_path)}
    summary = {}
    examples = []
    paths = sorted((dataset / 'images' / 'test').glob('*.jpg'))[:limit]
    for name, detector in methods.items():
        truth_count = located = decoded = false_positive = 0
        elapsed = 0.0
        for path in paths:
            image = cv2.imread(str(path))
            truth = ground_truth(
                dataset / 'labels' / 'test' / (path.stem + '.txt'),
                image.shape)
            started = time.perf_counter()
            observations = detector.detect(image)
            elapsed += time.perf_counter() - started
            truth_count += len(truth)
            matched = set()
            for observation in observations:
                overlaps = [iou(observation.bbox, box) for box in truth]
                if overlaps and max(overlaps) >= .25:
                    index = int(np.argmax(overlaps))
                    matched.add(index)
                    decoded += 1
                else:
                    false_positive += 1
            located += len(matched)
            if observations and len(examples) < 2:
                examples.append((name, image.copy(), observations))
        summary[name] = {
            'images': len(paths), 'ground_truth_boxes': truth_count,
            # OpenCV only emits a box after decoding; hybrid may use learned
            # proposals but this public output intentionally counts confirmed
            # decoded boxes for safety.
            'location_success_rate': located / max(1, truth_count),
            'content_decode_success_rate': decoded / max(1, truth_count),
            'false_positive_decodes': false_positive,
            'mean_image_latency_ms': elapsed * 1000 / max(1, len(paths))}
    return summary, examples


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', default='datasets/qr_shelf')
    parser.add_argument(
        '--model', default='src/uav_vision/models/qr_hog_svm.xml')
    parser.add_argument('--output', default='docs/results/qr_shelf_final')
    parser.add_argument('--limit', type=int, default=48)
    args = parser.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    summary, examples = evaluate(args.dataset, args.model, args.limit)
    (output / 'backend_metrics.json').write_text(
        json.dumps(summary, indent=2), encoding='utf-8')
    names = list(summary)
    x = np.arange(len(names))
    fig, axis = plt.subplots(figsize=(7, 4))
    axis.bar(x - .18, [summary[n]['location_success_rate'] for n in names],
             .36, label='location')
    axis.bar(x + .18,
             [summary[n]['content_decode_success_rate'] for n in names],
             .36, label='decode')
    axis.set(xticks=x, xticklabels=names, ylim=(0, 1),
             ylabel='success rate', title='Held-out QR test set')
    axis.legend()
    fig.tight_layout()
    fig.savefig(output / 'backend_comparison.png', dpi=140)
    for index, (name, image, observations) in enumerate(examples):
        for obs in observations:
            x0, y0, w, h = obs.bbox
            cv2.rectangle(image, (x0, y0), (x0 + w, y0 + h),
                          (0, 255, 0), 2)
            cv2.putText(image, str(obs.qr_id), (x0, max(15, y0 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 255), 2)
        cv2.imwrite(str(output / f'detection_{name}_{index}.jpg'), image)
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
