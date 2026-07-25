"""Unit tests for red-target detection."""

import math

import cv2
import numpy as np

from pi_camera_vision.detector import INVALID_DETECTION, RedTargetDetector


def test_red_rectangle_h7_field_order():
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.rectangle(image, (120, 80), (200, 160), (0, 0, 255), -1)
    result, annotated, mask = RedTargetDetector(
        min_area=100, morphology_kernel=1).detect(image)
    assert result[0] == 1.0
    assert result[1] == 160.0
    assert result[2] == 120.0
    assert result[3] == 81.0
    assert result[4] == 81.0
    assert result[5] == 6400.0
    assert 0.0 <= result[6] <= 100.0
    assert all(math.isfinite(value) for value in result)
    assert annotated.shape == image.shape
    assert mask.shape == image.shape[:2]


def test_no_red_target_is_all_zero():
    image = np.zeros((100, 100, 3), dtype=np.uint8)
    result, _, _ = RedTargetDetector().detect(image)
    assert result == INVALID_DETECTION


def test_small_noise_is_rejected():
    image = np.zeros((100, 100, 3), dtype=np.uint8)
    cv2.rectangle(image, (1, 1), (4, 4), (0, 0, 255), -1)
    result, _, _ = RedTargetDetector(min_area=100).detect(image)
    assert result == INVALID_DETECTION


def test_custom_hsv_threshold_can_reject_red():
    image = np.zeros((100, 100, 3), dtype=np.uint8)
    cv2.rectangle(image, (10, 10), (90, 90), (0, 0, 255), -1)
    result, _, _ = RedTargetDetector(
        min_area=100, red1_h_min=40, red1_h_max=50,
        red2_h_min=60, red2_h_max=70).detect(image)
    assert result == INVALID_DETECTION
