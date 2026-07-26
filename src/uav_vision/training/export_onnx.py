#!/usr/bin/env python3
"""Validate/export an optional Ultralytics model to ONNX in an isolated env."""

import argparse
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('weights')
    parser.add_argument('--imgsz', type=int, default=320)
    args = parser.parse_args()
    if not Path(args.weights).is_file():
        raise SystemExit('weights not found: %s' % args.weights)
    try:
        from ultralytics import YOLO
    except ImportError as error:
        raise SystemExit(
            'Install the pinned optional training requirements first') from error
    output = YOLO(args.weights).export(
        format='onnx', imgsz=args.imgsz, simplify=True, dynamic=False)
    print(output)


if __name__ == '__main__':
    main()
