"""Command-line image/video test without CSI hardware or ROS graph."""

import argparse
from pathlib import Path

import cv2

from .camera_source import create_source
from .detector import RedTargetDetector


def parse_args(args=None):
    """Parse offline test options."""
    parser = argparse.ArgumentParser()
    parser.add_argument('input', help='image or video file')
    parser.add_argument('--type', choices=('image', 'video'), required=True)
    parser.add_argument('--output', help='annotated image/video path')
    parser.add_argument('--max-frames', type=int, default=0)
    parser.add_argument('--min-area', type=float, default=100.0)
    return parser.parse_args(args)


def run(args):
    """Process an input and return a summary dictionary."""
    source = create_source(args.type, source=args.input)
    detector = RedTargetDetector(min_area=args.min_area)
    writer = None
    frames = detections = 0
    last = None
    try:
        while args.max_frames <= 0 or frames < args.max_frames:
            ok, frame = source.read()
            if not ok:
                break
            last, annotated, _ = detector.detect(frame)
            frames += 1
            detections += int(last[0] == 1.0)
            if args.output and args.type == 'image':
                if not cv2.imwrite(args.output, annotated):
                    raise RuntimeError('cannot write image: %s' % args.output)
            elif args.output:
                if writer is None:
                    height, width = annotated.shape[:2]
                    writer = cv2.VideoWriter(
                        args.output, cv2.VideoWriter_fourcc(*'MJPG'),
                        15.0, (width, height))
                    if not writer.isOpened():
                        raise RuntimeError(
                            'cannot write video: %s' % args.output)
                writer.write(annotated)
    finally:
        source.close()
        if writer is not None:
            writer.release()
    return {'frames': frames, 'detections': detections, 'last': last}


def main(args=None):
    """Run an offline test and print machine-readable field ordering."""
    options = parse_args(args)
    if not Path(options.input).expanduser().is_file():
        raise SystemExit('input does not exist: %s' % options.input)
    summary = run(options)
    print('fields=[valid,cx,cy,width,height,area,confidence]')
    print('frames=%d detections=%d last=%s' % (
        summary['frames'], summary['detections'], summary['last']))
    if summary['frames'] == 0:
        raise SystemExit('no frames decoded')


if __name__ == '__main__':
    main()
