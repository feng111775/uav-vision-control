#!/usr/bin/env python3
"""Calibrate camera intrinsics from real checkerboard photographs."""

import argparse
from pathlib import Path

import cv2
import numpy as np

from tools.calibration_io import save_yaml


def calibrate(paths, columns, rows, square_size):
    object_template = np.zeros((columns * rows, 3), np.float32)
    object_template[:, :2] = np.mgrid[
        0:columns, 0:rows].T.reshape(-1, 2) * square_size
    object_points, image_points = [], []
    image_size = None
    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            continue
        image_size = image.shape[::-1]
        found, corners = cv2.findChessboardCorners(
            image, (columns, rows),
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found:
            continue
        corners = cv2.cornerSubPix(
            image, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
             30, 0.001))
        object_points.append(object_template.copy())
        image_points.append(corners)
    if len(object_points) < 8 or image_size is None:
        raise RuntimeError('at least 8 usable checkerboard images are required')
    rms, matrix, distortion, _, _ = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None)
    return {
        'camera_calibrated': True, 'image_width': image_size[0],
        'image_height': image_size[1], 'checkerboard_columns': columns,
        'checkerboard_rows': rows, 'square_size_m': square_size,
        'usable_images': len(object_points), 'rms_reprojection_error': rms,
        'camera_matrix': matrix.tolist(),
        'distortion_coefficients': distortion.ravel().tolist(),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('images', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--columns', type=int, default=9)
    parser.add_argument('--rows', type=int, default=6)
    parser.add_argument('--square-size-m', type=float, required=True)
    args = parser.parse_args()
    paths = sorted(args.images.glob('*'))
    save_yaml(args.output, calibrate(
        paths, args.columns, args.rows, args.square_size_m))


if __name__ == '__main__':
    main()
