import sys
from pathlib import Path
from unittest.mock import Mock
sys.path.insert(0, str(Path(__file__).parent))
from detector_fast import FastV2Detector


class Image:
    def width(self): return 320
    def height(self): return 240


REGION = {
    'blob_bbox': (100, 70, 60, 60),
    'verify_roi': (92, 62, 76, 76),
    'cx': 130,
    'cy': 100,
    'diameter': 60,
    'area': 3600,
    'aspect': 1.0,
    'roundness': 0.9,
    'score': 1.9,
}
VERIFIED = {'valid': 1, 'cx': 130, 'cy': 100, 'outer_diameter_px': 50,
            'inner_diameter_px': 30, 'angle_rad': .1, 'confidence': 80,
            'status': 'TRACKING'}


def ready_detector(mode='FOLLOW'):
    detector = FastV2Detector()
    detector.mode = mode
    detector.track.last = {'valid': 1, 'measurement_valid': 1, 'cx': 130,
                           'cy': 100, 'outer_diameter_px': 50}
    detector.last_verified_success_frame = 1
    detector.last_verified_success_result = VERIFIED
    detector._search_regions = Mock(return_value=[REGION])
    return detector


def test_roi_without_verify_period_uses_current_measurement_only():
    detector = ready_detector()
    detector.frame = 2
    detector._circle_pairs = Mock(side_effect=AssertionError('verify called'))
    result = detector.detect(Image(), 'FOLLOW')
    assert result['valid'] == 1 and result['cx'] == 130
    detector._circle_pairs.assert_not_called()


def test_verify_period_calls_circle_pairs_once_and_scores_once():
    detector = ready_detector()
    detector.frame = 11
    detector._circle_pairs = Mock(return_value=[('outer', 'inner', .6, .01)])
    detector._detect_candidate = Mock(return_value=VERIFIED)
    result = detector.detect(Image(), 'FOLLOW')
    assert result['valid'] == 1
    detector._circle_pairs.assert_called_once()
    detector._detect_candidate.assert_called_once()


def test_missing_current_blob_is_invalid_and_does_not_reuse_old_coordinates():
    detector = ready_detector()
    detector._search_regions = Mock(return_value=[])
    result = detector.detect(Image(), 'FOLLOW')
    assert result['valid'] == 0
    assert (result['cx'], result['cy'], result['outer_diameter_px']) == (0, 0, 0)


def test_strong_verification_expires():
    detector = ready_detector()
    detector.frame = 13
    detector.last_verified_success_frame = 1
    detector._circle_pairs = Mock(side_effect=AssertionError('not due yet'))
    result = detector.detect(Image(), 'FOLLOW')
    assert result['valid'] == 0
    assert result['measurement_valid'] == 1


def test_search_drop_align_and_candidate_limit():
    for mode in ('SEARCH', 'FOLLOW', 'DROP_ALIGN'):
        detector = FastV2Detector(); detector.mode = mode
        detector._search_regions = Mock(return_value=[REGION] * 8)
        detector._circle_pairs = Mock(return_value=[])
        detector.frame = 0
        result = detector.detect(Image(), mode)
        assert result['valid'] == 0
        assert detector._circle_pairs.call_count <= 2


def test_failed_verify_keeps_recent_success_until_age_expires():
    detector = ready_detector()
    detector.frame = 11
    detector._circle_pairs = Mock(return_value=[])
    result = detector.detect(Image(), 'FOLLOW')
    assert result['measurement_valid'] == 1
    assert detector.last_verified_success_result is not None


def test_new_success_refreshes_verification_age():
    detector = ready_detector()
    detector.frame = 11
    detector.last_verified_success_frame = 1
    detector._circle_pairs = Mock(return_value=[('outer', 'inner', .6, .01)])
    detector._detect_candidate = Mock(return_value=VERIFIED)
    detector.detect(Image(), 'FOLLOW')
    assert detector.last_verified_success_frame == 12
    assert detector.consecutive_verify_failures == 0
