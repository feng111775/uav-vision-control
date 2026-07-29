#!/usr/bin/env python3
"""Evaluate the offline reference detector against optional CSV annotations."""

import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import time

import cv2

from tools.d_task_reference_detector import (
    DTaskReferenceDetector, draw_debug, periodic_angle_error)


IMAGE_SUFFIXES = {'.jpg', '.jpeg', '.png', '.bmp'}


def load_annotations(path):
    if path is None:
        return {}
    with path.open(newline='', encoding='utf-8') as stream:
        return {row['path']: row for row in csv.DictReader(stream)}


def evaluate(dataset, annotations=None, debug_dir=None):
    files = sorted(
        path for path in dataset.rglob('*')
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES)
    labels = load_annotations(annotations)
    detector = DTaskReferenceDetector()
    times = []
    detections = 0
    target_count = no_target_count = false_positive = misses = 0
    center_errors = []
    outer_errors = []
    inner_errors = []
    ratio_errors = []
    angle_errors = []
    lighting = {}
    if debug_dir:
        debug_dir.mkdir(parents=True, exist_ok=True)
    for path in files:
        image = cv2.imread(str(path))
        started = time.perf_counter()
        result = detector.detect(image)
        times.append((time.perf_counter() - started) * 1000.0)
        detections += int(result['valid'])
        key = str(path.relative_to(dataset))
        row = labels.get(key)
        if row is not None and row.get('has_target', '') != '':
            expected = int(row['has_target'])
            category = row.get('lighting', 'unspecified') or 'unspecified'
            bucket = lighting.setdefault(category, {'total': 0, 'detected': 0})
            bucket['total'] += 1
            bucket['detected'] += int(result['valid'])
            if expected:
                target_count += 1
                if not result['valid']:
                    misses += 1
                else:
                    cx, cy = float(row['center_x']), float(row['center_y'])
                    center_errors.append(math.hypot(
                        result['center_x_px'] - cx,
                        result['center_y_px'] - cy))
                    outer = float(row['outer_diameter'])
                    inner = float(row['inner_diameter'])
                    outer_errors.append(abs(result['outer_diameter_px'] - outer))
                    inner_errors.append(abs(result['inner_diameter_px'] - inner))
                    ratio_errors.append(abs(
                        result['inner_diameter_px'] /
                        result['outer_diameter_px'] - inner / outer))
                    angle_errors.append(abs(periodic_angle_error(
                        result['angle_rad'], float(row['angle_rad']))))
            else:
                no_target_count += 1
                false_positive += int(result['valid'])
        if debug_dir:
            cv2.imwrite(str(debug_dir / (path.stem + '_debug.jpg')),
                        draw_debug(image, result))
    ordered = sorted(times)
    p95 = ordered[min(len(ordered) - 1, int(len(ordered) * 0.95))] if ordered else 0
    report = {
        'total_images': len(files), 'candidate_detections': detections,
        'annotations_available': bool(labels), 'target_images': target_count,
        'no_target_images': no_target_count,
        'detection_rate': (
            (target_count - misses) / target_count if target_count else None),
        'false_positive_rate': (
            false_positive / no_target_count if no_target_count else None),
        'center_error_px_mean': (
            statistics.mean(center_errors) if center_errors else None),
        'outer_error_px_mean': (
            statistics.mean(outer_errors) if outer_errors else None),
        'inner_error_px_mean': (
            statistics.mean(inner_errors) if inner_errors else None),
        'ratio_error_mean': (
            statistics.mean(ratio_errors) if ratio_errors else None),
        'angle_periodic_error_rad_mean': (
            statistics.mean(angle_errors) if angle_errors else None),
        'processing_ms_mean': statistics.mean(times) if times else 0,
        'processing_ms_p95': p95, 'lighting': lighting,
    }
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('dataset', type=Path)
    parser.add_argument('--annotations', type=Path)
    parser.add_argument('--debug-dir', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report = evaluate(args.dataset, args.annotations, args.debug_dir)
    text = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        args.output.write_text(text + '\n', encoding='utf-8')
    print(text)


if __name__ == '__main__':
    main()
