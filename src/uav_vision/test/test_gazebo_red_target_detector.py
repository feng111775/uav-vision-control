"""Tests for Gazebo red target detection."""

import math

import cv2
import numpy as np

from uav_vision.gazebo_red_target_detector_node import INVALID_DETECTION
from uav_vision.gazebo_red_target_detector_node import RedTargetDetector


def image_with_rectangles(rectangles):
    """Create a black BGR test image with filled red rectangles."""
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    for start, end in rectangles:
        cv2.rectangle(image, start, end, (0, 0, 255), -1)
    return image


def test_red_target_center_is_correct():
    detector = RedTargetDetector(min_area=20, morphology_kernel=1)
    result, _, _ = detector.detect(
        image_with_rectangles([((140, 100), (180, 140))]))
    assert result[0] == 1.0
    assert abs(result[1] - 160.5) <= 0.5
    assert abs(result[2] - 120.5) <= 0.5


def test_no_target_outputs_all_zero():
    result, _, _ = RedTargetDetector().detect(
        np.zeros((240, 320, 3), dtype=np.uint8))
    assert result == INVALID_DETECTION


def test_small_red_noise_is_filtered():
    detector = RedTargetDetector(min_area=100, morphology_kernel=1)
    result, _, _ = detector.detect(
        image_with_rectangles([((10, 10), (14, 14))]))
    assert result == INVALID_DETECTION


def test_largest_valid_target_is_selected():
    detector = RedTargetDetector(min_area=20, morphology_kernel=1)
    result, _, _ = detector.detect(image_with_rectangles([
        ((10, 10), (20, 20)),
        ((200, 150), (250, 210)),
    ]))
    assert result[0] == 1.0
    assert result[1] > 200
    assert result[2] > 150
    assert result[5] > 2000


def test_edge_target_center_stays_inside_image():
    detector = RedTargetDetector(min_area=1, morphology_kernel=1)
    result, _, _ = detector.detect(
        image_with_rectangles([((300, 220), (319, 239))]))
    assert result[0] == 1.0
    assert 0.0 <= result[1] <= 319.0
    assert 0.0 <= result[2] <= 239.0


def test_empty_and_illegal_images_do_not_raise():
    detector = RedTargetDetector()
    for image in (None, np.array([]), np.zeros((10, 10), dtype=np.uint8)):
        result, _, _ = detector.detect(image)
        assert result == INVALID_DETECTION


def test_output_is_finite_h7_compatible_array():
    detector = RedTargetDetector(min_area=1, morphology_kernel=1)
    result, _, _ = detector.detect(
        image_with_rectangles([((1, 1), (30, 30))]))
    assert len(result) == 7
    assert result[0] in (0.0, 1.0)
    assert all(math.isfinite(value) for value in result)
    assert 0.0 <= result[6] <= 100.0
