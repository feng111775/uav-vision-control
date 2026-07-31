#!/usr/bin/env python3
"""Fit measured height against reciprocal 50 cm outer-circle diameter."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def fit_scale(path):
    rows = []
    with Path(path).open(newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            rows.append((float(row['outer_diameter_px']),
                         float(row['height_m'])))
    if len(rows) < 5:
        raise RuntimeError('at least 5 measured height samples are required')
    diameter = np.array([row[0] for row in rows])
    height = np.array([row[1] for row in rows])
    if np.any(diameter <= 0) or np.any(height <= 0):
        raise ValueError('diameter and measured height must be positive')
    design = np.column_stack((1.0 / diameter, np.ones_like(diameter)))
    coefficients, _, _, _ = np.linalg.lstsq(design, height, rcond=None)
    predicted = design @ coefficients
    residual = height - predicted
    return {
        'scale_calibrated': True, 'outer_diameter_real_m': 0.5,
        'inner_diameter_real_m': 0.3,
        'model': 'height_m = a / outer_diameter_px + b',
        'a': float(coefficients[0]), 'b': float(coefficients[1]),
        'sample_count': len(rows),
        'rmse_m': float(np.sqrt(np.mean(residual ** 2))),
        'max_abs_residual_m': float(np.max(np.abs(residual))),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('samples', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = fit_scale(args.samples)
    text = json.dumps(result, indent=2)
    if args.output:
        args.output.write_text(text + '\n', encoding='utf-8')
    print(text)


if __name__ == '__main__':
    main()
