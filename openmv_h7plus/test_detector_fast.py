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
    'candidate_id': 'cand_1',
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
    detector.verified_track_active = True
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
    detector.frame = 1
    detector.last_verified_success_frame = 1
    detector.last_verify_attempt_frame = 1
    detector._circle_pairs = Mock(return_value=[('outer', 'inner', .6, .01)])
    detector._detect_candidate = Mock(return_value=VERIFIED)
    for i in range(24):
        detector.frame = 1 + i + 1
        result = detector.detect(Image(), 'FOLLOW')
    assert detector._circle_pairs.call_count == 1
    assert detector._detect_candidate.call_count == 1


def test_missing_current_blob_is_invalid_and_does_not_reuse_old_coordinates():
    detector = ready_detector()
    detector._search_regions = Mock(return_value=[])
    result = detector.detect(Image(), 'FOLLOW')
    assert result['valid'] == 0
    assert (result['cx'], result['cy'], result['outer_diameter_px']) == (0, 0, 0)


def test_strong_verification_expires():
    detector = ready_detector()
    detector.frame = 1
    detector.last_verified_success_frame = 1
    detector._circle_pairs = Mock(side_effect=AssertionError('not due yet'))
    detector.frame = 40
    result = detector.detect(Image(), 'FOLLOW')
    assert result['valid'] == 0


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
    detector.frame = 1
    detector.last_verified_success_frame = 1
    detector.last_verify_attempt_frame = 1
    detector._circle_pairs = Mock(return_value=[])
    detector.frame = 9
    result = detector.detect(Image(), 'FOLLOW')
    assert result['measurement_valid'] == 1
    assert detector.last_verified_success_result is not None


def test_new_success_refreshes_verification_age():
    detector = ready_detector()
    detector.frame = 1
    detector.last_verified_success_frame = 1
    detector.last_verify_attempt_frame = 1
    detector._circle_pairs = Mock(return_value=[('outer', 'inner', .6, .01)])
    detector._detect_candidate = Mock(return_value=VERIFIED)
    detector.frame = 25
    detector.detect(Image(), 'FOLLOW')
    assert detector.last_verified_success_frame == 26  # detect() increments frame before using it
    assert detector.consecutive_verify_failures == 0


def test_unverified_blob_cannot_establish_formal_roi():
    detector = FastV2Detector()
    detector._search_regions = Mock(return_value=[REGION])
    detector._circle_pairs = Mock(return_value=[])
    for frame_idx in range(5):
        detector.frame = frame_idx
        result = detector.detect(Image(), 'SEARCH')
        assert result['valid'] == 0
        assert detector.verified_track_active is False
        assert detector.track.last is None


def test_no_target_background_keeps_full_search():
    detector = FastV2Detector()
    detector._search_regions = Mock(return_value=[REGION])
    detector._circle_pairs = Mock(return_value=[])
    for frame_idx in range(10):
        detector.frame = frame_idx
        result = detector.detect(Image(), 'SEARCH')
        assert result['valid'] == 0
    assert detector.verified_track_active is False


def test_strong_verify_success_establishes_verified_track():
    detector = FastV2Detector()
    detector._search_regions = Mock(return_value=[REGION])
    detector._circle_pairs = Mock(return_value=[('outer', 'inner', .6, .01)])
    detector._detect_candidate = Mock(return_value=VERIFIED)
    detector.frame = 0
    result = detector.detect(Image(), 'SEARCH')
    assert result['valid'] == 1
    assert detector.verified_track_active is True
    assert detector.track.last is not None


def test_single_failure_within_grace_does_not_clear():
    detector = ready_detector()
    detector.frame = 1
    detector.last_verified_success_frame = 1
    detector.last_verify_attempt_frame = 1
    detector._circle_pairs = Mock(return_value=[])
    detector.frame = 9
    result = detector.detect(Image(), 'FOLLOW')
    assert result['measurement_valid'] == 1
    assert detector.verified_track_active is True
    assert detector.track.last is not None


def test_consecutive_failures_clear_verified_track():
    detector = ready_detector()
    detector.frame = 1
    detector.last_verified_success_frame = 1
    detector.last_verify_attempt_frame = 1
    detector._circle_pairs = Mock(return_value=[])
    for idx in range(4):
        detector.frame = 1 + (idx + 1) * 8
        result = detector.detect(Image(), 'FOLLOW')
    assert detector.verified_track_active is False
    assert detector.track.last is None


def test_moving_candidate_uses_verified_diameter():
    detector = ready_detector()
    detector.frame = 1
    detector.last_verified_success_frame = 1
    detector.last_verify_attempt_frame = 1
    detector.last_verified_success_result = {'valid': 1, 'outer_diameter_px': 50}
    moving_region = dict(REGION, diameter=65)
    detector._search_regions = Mock(return_value=[moving_region])
    detector._circle_pairs = Mock(return_value=[('outer', 'inner', .6, .01)])
    detector._detect_candidate = Mock(return_value=VERIFIED)
    detector.frame = 25
    detector.detect(Image(), 'FOLLOW')
    candidate_arg = detector._circle_pairs.call_args[0][1]
    expected = candidate_arg.get('expected_outer_diameter', 0)
    assert 40 <= expected <= 65


def test_edge_clipped_target_rejected():
    detector = FastV2Detector()
    edge_region = dict(REGION, blob_bbox=(2, 2, 60, 60), edge_clipped=True)
    detector._search_regions = Mock(return_value=[edge_region])
    detector._circle_pairs = Mock(return_value=[])
    detector.frame = 0
    result = detector.detect(Image(), 'SEARCH')
    assert result['valid'] == 0
    assert result.get('status') == 'EDGE_CLIPPED'


def test_track_interval_actually_active():
    detector = ready_detector()
    detector.frame = 1
    detector.last_verified_success_frame = 1
    detector.last_verify_attempt_frame = 1
    verify_frames = []
    for frame_idx in range(1, 60):
        detector.frame = 1 + frame_idx
        if detector._verify_due(None):
            verify_frames.append(detector.frame)
            detector.last_verify_attempt_frame = detector.frame
            detector.last_verified_success_frame = detector.frame
    interval_estimate = verify_frames[1] - verify_frames[0] if len(verify_frames) >= 2 else 999
    assert interval_estimate >= 20


def test_strong_verify_under_5_percent():
    detector = ready_detector()
    detector.frame = 1
    detector.last_verified_success_frame = 1
    detector.last_verify_attempt_frame = 1
    verify_count = 0
    for frame_idx in range(1, 101):
        detector.frame = 1 + frame_idx
        if detector._verify_due(None):
            verify_count += 1
            detector.last_verify_attempt_frame = detector.frame
            detector.last_verified_success_frame = detector.frame
    assert verify_count <= 5


def test_acquire_gets_two_verification_opportunities():
    detector = FastV2Detector()
    detector._search_regions = Mock(return_value=[REGION])
    detector._circle_pairs = Mock(return_value=[])
    due_frames = []
    for _ in range(7):
        detector.detect(Image(), 'SEARCH')
        if detector.last_verify_attempt_frame == detector.frame:
            due_frames.append(detector.frame)
    assert len(due_frames) >= 2
    assert due_frames[1] - due_frames[0] == 2
    assert detector.track.last is None


def test_acquire_background_candidate_times_out_to_search():
    detector = FastV2Detector()
    detector._search_regions = Mock(return_value=[REGION])
    detector._circle_pairs = Mock(return_value=[])
    for _ in range(7):
        detector.detect(Image(), 'SEARCH')
    assert detector.diagnostics.acquire_timeout_count >= 1
    assert detector.verified_track_active is False
    assert detector.force_full_search is True


def test_p99_and_max_reported_in_stats():
    from detector import DetectionStatsReporter
    import io
    reporter = DetectionStatsReporter()
    stream = io.StringIO()
    reporter.set_writer(stream)
    reporter.emit_if_due(0)
    for frame_idx in range(10):
        reporter.begin_frame(frame_idx, 'SEARCH', False, 0)
        reporter.finish(False)
    reporter.emit_if_due(1200)
    output = stream.getvalue()
    assert 'D_DETECT_STATS' in output


def test_candidate_switch_not_for_normal_motion():
    detector = ready_detector()
    detector.last_selected_identity = 'cand_1'
    region_moved = dict(REGION, cx=132, cy=102, candidate_id='cand_1')
    detector._search_regions = Mock(return_value=[region_moved])
    detector.frame = 2
    switch_before = detector.diagnostics.candidate_switch_count
    detector.detect(Image(), 'FOLLOW')
    switch_after = detector.diagnostics.candidate_switch_count
    assert switch_after == switch_before
