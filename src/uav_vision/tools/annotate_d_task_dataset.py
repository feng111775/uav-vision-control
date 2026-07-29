#!/usr/bin/env python3
"""Create/update the D-target CSV annotation manifest."""

import argparse
import csv
from pathlib import Path

from tools.evaluate_d_task_dataset import IMAGE_SUFFIXES


FIELDS = (
    'path', 'has_target', 'center_x', 'center_y', 'outer_diameter',
    'inner_diameter', 'angle_rad', 'lighting', 'height_m', 'notes')


def create_manifest(dataset, output):
    existing = {}
    if output.exists():
        with output.open(newline='', encoding='utf-8') as stream:
            existing = {row['path']: row for row in csv.DictReader(stream)}
    files = sorted(
        path for path in dataset.rglob('*')
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES)
    with output.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        for path in files:
            key = str(path.relative_to(dataset))
            writer.writerow(existing.get(key, {'path': key}))
    return len(files)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('dataset', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    print('manifest images:', create_manifest(args.dataset, args.output))


if __name__ == '__main__':
    main()
