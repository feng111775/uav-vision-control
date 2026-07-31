#!/usr/bin/env python3
"""Infer image-axis signs from recorded forward/back/left/right motions."""

import argparse
import csv
import json
from pathlib import Path
import statistics


EXPECTED = {'forward': (1, 0), 'back': (-1, 0),
            'left': (0, 1), 'right': (0, -1)}


def infer(path):
    samples = {key: [] for key in EXPECTED}
    with Path(path).open(newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            samples[row['motion']].append(
                (float(row['delta_x_px']), float(row['delta_y_px'])))
    if any(not samples[key] for key in EXPECTED):
        raise RuntimeError('forward/back/left/right samples are all required')
    forward_dy = statistics.mean(
        y for key in ('forward', 'back') for _, y in samples[key]
        if key == 'forward') - statistics.mean(
            y for _, y in samples['back'])
    left_dx = statistics.mean(
        x for x, _ in samples['left']) - statistics.mean(
            x for x, _ in samples['right'])
    return {
        'camera_y_sign_suggestion': -1.0 if forward_dy < 0 else 1.0,
        'camera_x_sign_suggestion': -1.0 if left_dx < 0 else 1.0,
        'source': str(path),
        'note': 'review signs manually; tool does not edit competition YAML',
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('samples', type=Path)
    args = parser.parse_args()
    print(json.dumps(infer(args.samples), indent=2))


if __name__ == '__main__':
    main()
