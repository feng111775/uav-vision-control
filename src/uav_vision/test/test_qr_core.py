"""QR shelf core tests covering decoding, safety and mission behavior."""

import cv2
import numpy as np
import pytest

from uav_vision.qr_core import HybridQRDetector, LaserAlignment
from uav_vision.qr_core import QRDecoder, QRInventory
from uav_vision.qr_core import QRObservation
from uav_vision.qr_core import SNAKE_SCAN_ORDER


def code(qr_id, size=240):
    raw = cv2.QRCodeEncoder_create().encode(str(qr_id))
    return cv2.cvtColor(cv2.resize(
        raw, (size, size), interpolation=cv2.INTER_NEAREST),
        cv2.COLOR_GRAY2BGR)


@pytest.mark.parametrize('qr_id', range(1, 25))
def test_generate_and_decode_all_ids(qr_id):
    assert QRDecoder().decode_crop(code(qr_id))[0] == qr_id


@pytest.mark.parametrize('payload', ['0', '25', '-1', 'abc', '01', '', '7.0'])
def test_reject_invalid_payload(payload):
    assert QRDecoder.parse(payload) is None


def test_single_and_multi_qr():
    canvas = np.full((300, 600, 3), 255, np.uint8)
    canvas[30:270, 20:260] = code(3)
    canvas[30:270, 330:570] = code(19)
    ids = {item[0] for item in QRDecoder().decode_full(canvas)}
    assert ids == {3, 19}


def test_small_blurred_and_lighting_variants_do_not_crash():
    for image in (
            cv2.resize(code(6), (60, 60)),
            cv2.GaussianBlur(code(6), (9, 9), 2),
            cv2.convertScaleAbs(code(6), alpha=.45)):
        assert isinstance(HybridQRDetector().detect(image), list)


def test_negative_empty_corrupt_inputs():
    detector = HybridQRDetector()
    assert detector.detect(np.full((200, 200, 3), 127, np.uint8)) == []
    assert detector.detect(None) == []
    assert detector.detect(np.empty((0, 0), np.uint8)) == []


def test_empty_camera_frame_cannot_confirm_expected_qr():
    inventory = QRInventory(confirm_frames=1)
    inventory.set_scan_context(0, True, [-1.25, .78, -2.5])
    observations = HybridQRDetector('opencv').detect(
        np.empty((0, 0, 3), np.uint8))
    inventory.update(observations, 1.0)
    assert inventory.scan_index == 0
    assert inventory.records == {}


def test_camera_frame_without_qr_cannot_confirm():
    inventory = QRInventory(confirm_frames=1)
    inventory.set_scan_context(0, True, [-1.25, .78, -2.5])
    observations = HybridQRDetector('opencv').detect(
        np.full((480, 640, 3), 127, np.uint8))
    inventory.update(observations, 1.0)
    assert observations == []
    assert inventory.scan_index == 0


def test_scan_context_and_position_never_generate_an_observation():
    inventory = QRInventory(confirm_frames=1)
    inventory.set_scan_context(
        scan_index=0, hold=True, scan_position=[-1.25, .78, -2.5])
    inventory.update([], 1.0)
    assert inventory.expected_qr_id == 1
    assert inventory.records == {}
    assert inventory.scan_index == 0


def test_wrong_decoded_id_at_correct_scan_position_cannot_advance():
    inventory = QRInventory(confirm_frames=1)
    inventory.set_scan_context(0, True, [-1.25, .78, -2.5])
    observations = HybridQRDetector('opencv').detect(code(2), 1.0)
    assert [item.qr_id for item in observations] == [2]
    inventory.update(observations, 1.0)
    assert inventory.visible_ids == [2]
    assert inventory.scan_index == 0


def test_crop_clips_edges_and_rejects_outside():
    image = np.zeros((20, 30, 3), np.uint8)
    assert QRDecoder.safe_crop(image, (-4, -3, 10, 10)).size > 0
    assert QRDecoder.safe_crop(image, (100, 100, 2, 2)).size == 0


def test_perspective_variant():
    image = code(12)
    source = np.float32([[0, 0], [239, 0], [239, 239], [0, 239]])
    target = np.float32([[20, 10], [220, 0], [239, 220], [0, 239]])
    warped = cv2.warpPerspective(
        image, cv2.getPerspectiveTransform(source, target), (240, 240),
        borderValue=(255, 255, 255))
    assert isinstance(QRDecoder().decode_crop(warped), tuple)


def observation(qr_id=7, now=1.0, center=(100, 100)):
    return QRObservation(
        qr_id, (80, 80, 40, 40), center, .9, now)


def test_multiframe_confirmation_and_deduplication():
    inventory = QRInventory(confirm_frames=3, target_qr_id=7)
    inventory.set_scan_context(0, True, [1, 2, 3])
    inventory.update([observation(1, now=1)], 1)
    inventory.update([observation(1, now=2)], 2)
    assert not inventory.records
    inventory.update([observation(1, now=3)], 3)
    assert list(inventory.records) == [1]
    inventory.update([observation(1, now=4), observation(1, now=4)], 4)
    assert len(inventory.records) == 1


def test_confirmation_must_be_consecutive():
    inventory = QRInventory(confirm_frames=2, target_qr_id=7)
    inventory.set_scan_context(0, True)
    inventory.update([observation(1)], 1)
    inventory.update([], 2)
    inventory.update([observation(1)], 3)
    assert not inventory.records


def test_strict_snake_order_and_non_current_ids_cannot_advance():
    inventory = QRInventory(confirm_frames=1, target_qr_id=10)
    for index, expected in enumerate(SNAKE_SCAN_ORDER):
        inventory.set_scan_context(index, True, [index, 0, 0])
        other = 24 if expected != 24 else 23
        inventory.update([observation(other)], index + .1)
        assert inventory.scan_index == index
        inventory.update([observation(expected)], index + .2)
        assert inventory.scan_index == index + 1
    assert inventory.complete(30)
    assert list(inventory.records) == list(SNAKE_SCAN_ORDER)


def test_duplicate_frame_cannot_confirm_next_scan_point():
    inventory = QRInventory(confirm_frames=1)
    inventory.set_scan_context(0, True)
    inventory.update([observation(1), observation(2)], 1)
    assert inventory.scan_index == 1
    assert 2 not in inventory.records


def test_inventory_json_exposes_progress_missing_and_record_fields():
    import json
    inventory = QRInventory(confirm_frames=1)
    inventory.set_scan_context(0, True, [0.8, -1.25, -2.5], 1)
    inventory.update([observation(1)], 1)
    data = json.loads(inventory.to_json())
    assert data['confirmed_ids'] == [1]
    assert data['current_expected_qr_id'] == 2
    assert not data['complete']
    assert 2 in data['missing_ids']
    assert {'scan_index', 'qr_id', 'confirmed', 'first_seen_time',
            'confirmed_time', 'scan_position', 'image_center', 'confidence',
            'retry_count'} <= set(data['records'][0])


def test_image_timeout():
    inventory = QRInventory(image_timeout=.5)
    assert inventory.image_stale(0)
    inventory.update([], 1)
    assert not inventory.image_stale(1.4)
    assert inventory.image_stale(1.6)


def test_scan_index_cannot_directly_produce_qr_id():
    inventory = QRInventory(confirm_frames=1)
    for index in range(24):
        inventory.set_scan_context(index, True, [index, 0, 0])
    assert inventory.scan_index == 0
    assert inventory.records == {}


def test_missing_model_falls_back_to_opencv():
    detector = HybridQRDetector('model', '/does/not/exist.xml')
    assert not detector.learned.available
    assert detector.detect(code(8))[0].qr_id == 8


def test_laser_requires_consecutive_aligned_frames():
    logic = LaserAlignment(10, 3)
    assert not logic.update(3, 4)
    assert not logic.update(3, 4)
    assert logic.update(3, 4)
    assert not logic.update(20, 0)


def test_detection_compatibility_array():
    values = observation().detection_array()
    assert len(values) == 7
    assert values[0] == 1.0
    assert 0 <= values[6] <= 100
