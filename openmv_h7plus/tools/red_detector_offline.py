#!/usr/bin/env python3
"""Offline LAB red-region visual check; it does not connect to ROS or OpenMV."""

import argparse
import sys


def _require_opencv():
    try:
        import cv2
        import numpy
    except ImportError as error:
        raise SystemExit(
            "This optional offline helper needs the existing desktop OpenCV "
            "environment (cv2 and numpy): %s" % error)
    return cv2, numpy


def detect_file(path):
    """Print the largest plausible red region centre from a test image."""
    cv2, numpy = _require_opencv()
    image = cv2.imread(path)
    if image is None:
        raise SystemExit("Cannot read image: %s" % path)

    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    # OpenCV stores L in [0, 255], a/b with a +128 offset.  These ranges map
    # the OpenMV LAB starting bands in red_detector.py for desktop inspection.
    masks = (
        cv2.inRange(lab, numpy.array((51, 153, 128)),
                    numpy.array((255, 255, 255))),
        cv2.inRange(lab, numpy.array((51, 143, 108)),
                    numpy.array((255, 255, 255))),
    )
    mask = cv2.bitwise_or(masks[0], masks[1])
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        print("valid=0,cx=0,cy=0,area=0,confidence=0")
        return 0
    contour = max(contours, key=cv2.contourArea)
    area = int(cv2.contourArea(contour))
    if area <= 0:
        print("valid=0,cx=0,cy=0,area=0,confidence=0")
        return 0
    moments = cv2.moments(contour)
    if moments["m00"] == 0:
        print("valid=0,cx=0,cy=0,area=0,confidence=0")
        return 0
    cx = int(moments["m10"] / moments["m00"])
    cy = int(moments["m01"] / moments["m00"])
    print("valid=1,cx=%d,cy=%d,area=%d,confidence=offline" %
          (cx, cy, area))
    return 0


def main():
    parser = argparse.ArgumentParser(
        description="Inspect a test image for a candidate LAB red region.")
    parser.add_argument("image", help="input image path")
    return detect_file(parser.parse_args().image)


if __name__ == "__main__":
    sys.exit(main())
