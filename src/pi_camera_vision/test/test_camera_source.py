"""Tests for desktop acquisition and optional Picamera2 isolation."""

from argparse import Namespace

import cv2
import numpy as np
import pytest

from pi_camera_vision.camera_source import create_source
from pi_camera_vision.offline_test import run
from pi_camera_vision.vision_node import transform_frame


def test_image_source_reads_once(tmp_path):
    path = tmp_path / 'frame.png'
    cv2.imwrite(str(path), np.zeros((20, 30, 3), dtype=np.uint8))
    source = create_source('image', source=str(path))
    try:
        assert source.read()[0]
        assert not source.read()[0]
    finally:
        source.close()


def test_picamera2_missing_has_actionable_error():
    try:
        import picamera2  # noqa: F401
    except ImportError:
        with pytest.raises(RuntimeError, match='only on Raspberry Pi'):
            create_source('picamera2')


def test_offline_image_detects_red(tmp_path):
    source_path = tmp_path / 'red.png'
    output_path = tmp_path / 'annotated.png'
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    cv2.rectangle(image, (40, 30), (120, 90), (0, 0, 255), -1)
    cv2.imwrite(str(source_path), image)
    summary = run(Namespace(
        input=str(source_path), type='image', output=str(output_path),
        max_frames=0, min_area=100.0))
    assert summary['frames'] == 1
    assert summary['detections'] == 1
    assert output_path.is_file()


def test_rotation_and_flip_are_applied():
    image = np.zeros((2, 3, 3), dtype=np.uint8)
    image[0, 0] = (1, 2, 3)
    rotated = transform_frame(image, rotation=90)
    assert rotated.shape == (3, 2, 3)
    assert np.array_equal(rotated[0, 1], (1, 2, 3))
    flipped = transform_frame(image, flip_horizontal=True)
    assert np.array_equal(flipped[0, 2], (1, 2, 3))


def test_invalid_rotation_is_rejected():
    with pytest.raises(ValueError, match='rotation'):
        transform_frame(np.zeros((2, 2, 3), dtype=np.uint8), rotation=45)
