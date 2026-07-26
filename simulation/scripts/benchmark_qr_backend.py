#!/usr/bin/env python3
"""Measure whole-frame QR latency, throughput and decode rate."""

import argparse
import json
from pathlib import Path
import statistics
import sys
import time

import cv2

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src' / 'uav_vision'))
from uav_vision.qr_core import HybridQRDetector  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('images', nargs='+')
    parser.add_argument('--backend', default='opencv',
                        choices=['opencv', 'model', 'hybrid'])
    parser.add_argument('--model', default=str(
        ROOT / 'src/uav_vision/models/qr_hog_svm.xml'))
    args = parser.parse_args()
    detector = HybridQRDetector(args.backend, args.model)
    samples = []
    decoded = 0
    for name in args.images:
        image = cv2.imread(name)
        if image is None:
            continue
        started = time.perf_counter()
        observations = detector.detect(image)
        samples.append((time.perf_counter() - started) * 1000)
        decoded += bool(observations)
    result = {
        'backend': args.backend, 'frames': len(samples),
        'decoded_frames': decoded,
        'decode_rate': decoded / max(1, len(samples)),
        'mean_latency_ms': statistics.mean(samples) if samples else None,
        'p95_latency_ms': sorted(samples)[
            max(0, int(len(samples) * .95) - 1)] if samples else None,
        'worst_latency_ms': max(samples) if samples else None,
        'fps': 1000 / statistics.mean(samples) if samples else 0}
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
