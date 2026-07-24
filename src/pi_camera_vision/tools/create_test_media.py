#!/usr/bin/env python3
"""Create deterministic red-target image/video fixtures for desktop checks."""

import argparse
from pathlib import Path

import cv2
import numpy as np


def main():
    """Write one PNG and one MJPEG AVI to the requested directory."""
    parser = argparse.ArgumentParser()
    parser.add_argument('output_dir')
    args = parser.parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    image = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.rectangle(image, (120, 80), (200, 160), (0, 0, 255), -1)
    image_path = output_dir / 'red_target.png'
    if not cv2.imwrite(str(image_path), image):
        raise RuntimeError('failed to write %s' % image_path)

    video_path = output_dir / 'red_target.avi'
    writer = cv2.VideoWriter(
        str(video_path), cv2.VideoWriter_fourcc(*'MJPG'), 15.0, (320, 240))
    if not writer.isOpened():
        raise RuntimeError('failed to open %s' % video_path)
    for index in range(30):
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        left = 40 + index * 5
        cv2.rectangle(
            frame, (left, 90), (left + 60, 150), (0, 0, 255), -1)
        writer.write(frame)
    writer.release()
    print(image_path)
    print(video_path)


if __name__ == '__main__':
    main()
