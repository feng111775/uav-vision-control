"""Regression tests for the formal D-task reference vision and tools."""

import importlib.util
import math
from pathlib import Path
import sys

import cv2
import numpy as np
import pytest


PACKAGE = Path(__file__).parents[1]
TOOLS = PACKAGE / 'tools'
OPENMV = PACKAGE.parents[1] / 'openmv_h7plus'
sys.path.insert(0, str(PACKAGE))

from tools.calibration_io import (  # noqa: E402
    load_yaml, require_metric_calibration, save_yaml)
from tools.d_task_reference_detector import (  # noqa: E402
    DTaskReferenceDetector, DetectorConfig, geometry_score,
    periodic_angle_error)
from tools.generate_synthetic_d_target import generate_target  # noqa: E402


def detect(**kwargs):
    return DTaskReferenceDetector().detect(generate_target(**kwargs))


def test_diameter_ratio_point_six_is_valid_geometry():
    assert geometry_score((10, 10, 100, 1), (10, 10, 60, 1))['valid']


@pytest.mark.parametrize('inner', [40, 50])
def test_ratio_too_small_is_rejected(inner):
    assert geometry_score((10, 10, 100, 1), (10, 10, inner, 1))[
        'status'] == 'RATIO_INVALID'


@pytest.mark.parametrize('inner', [70, 85])
def test_ratio_too_large_is_rejected(inner):
    assert geometry_score((10, 10, 100, 1), (10, 10, inner, 1))[
        'status'] == 'RATIO_INVALID'


def test_concentric_error_is_rejected():
    assert geometry_score((10, 10, 100, 1), (30, 10, 60, 1))[
        'status'] == 'CONCENTRIC_INVALID'


def test_only_outer_circle_is_rejected():
    assert geometry_score((10, 10, 100, 1), None)[
        'status'] == 'RING_INCOMPLETE'


def test_only_inner_circle_is_rejected():
    assert geometry_score(None, (10, 10, 60, 1))[
        'status'] == 'RING_INCOMPLETE'


def test_negative_cross_angle_is_legal():
    result = detect(angle=-0.35)
    assert result['valid'] == 1
    assert math.isfinite(result['angle_rad'])


def test_cross_angle_error_has_ninety_degree_period():
    assert periodic_angle_error(math.radians(89), math.radians(-1)) == pytest.approx(0)


def test_formal_target_is_detected():
    result = detect()
    assert result['valid'] == 1
    assert 0 <= result['confidence'] <= 100
    assert result['inner_diameter_px'] / result[
        'outer_diameter_px'] == pytest.approx(0.6, abs=0.08)


def test_no_cross_rejects_formal_target():
    assert detect(draw_cross=False)['valid'] == 0


def test_single_line_is_not_a_cross():
    image = generate_target(draw_cross=False)
    cv2.line(image, (125, 120), (195, 120), (0, 0, 0), 4)
    assert DTaskReferenceDetector().detect(image)['valid'] == 0


def test_border_truncation_reduces_confidence():
    center = detect()
    border = detect(center=(20, 120))
    assert border['confidence'] < center['confidence']


@pytest.mark.parametrize('kwargs', [
    {'gradient': 1.0},
    {'shadow': True},
    {'blur': 7},
    {'occlusion': 0.15},
    {'highlight': True},
    {'noise': 12.0},
])
def test_degraded_synthetic_frames_never_crash_or_leave_confidence_range(kwargs):
    result = detect(**kwargs)
    assert result['valid'] in (0, 1)
    assert 0 <= result['confidence'] <= 100


def test_wrong_ratio_ring_interference_is_rejected():
    assert detect(ratio=0.82)['valid'] == 0


def test_only_cross_is_rejected():
    assert detect(draw_outer=False, draw_inner=False)['valid'] == 0


def test_distractors_do_not_move_selected_center_far():
    result = detect(distractor=True)
    assert result['valid'] == 1
    assert abs(result['center_x_px'] - 160) < 8
    assert abs(result['center_y_px'] - 120) < 8


def test_roi_tracking_is_used_after_first_detection():
    detector = DTaskReferenceDetector()
    assert detector.detect(generate_target())['valid']
    assert detector.detect(generate_target(center=(165, 122)))['valid']
    assert detector.last_search == 'ROI'


def test_roi_loss_recovers_to_full_frame():
    detector = DTaskReferenceDetector(
        DetectorConfig(roi_failures_before_full=2))
    assert detector.detect(generate_target())['valid']
    blank = np.full((240, 320, 3), 220, np.uint8)
    detector.detect(blank)
    detector.detect(blank)
    assert detector.last_search == 'FULL'
    assert detector.detect(generate_target(center=(60, 60)))['valid']


def test_no_target_heartbeat_serialization():
    spec = importlib.util.spec_from_file_location(
        'openmv_protocol', OPENMV / 'protocol.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.format_target_line(None) == 'D_TARGET,0,0,0,0,0,0,0\n'


def test_d_target_serialization_and_negative_angle():
    spec = importlib.util.spec_from_file_location(
        'openmv_protocol_target', OPENMV / 'protocol.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    line = module.format_target_line({
        'valid': 1, 'cx': 160, 'cy': 120,
        'outer_diameter_px': 100, 'inner_diameter_px': 60,
        'angle_rad': -0.2, 'confidence': 88})
    assert line == 'D_TARGET,1,160,120,100,60,-0.2000,88\n'


def test_calibration_yaml_roundtrip(tmp_path):
    path = tmp_path / 'camera.yaml'
    data = {'camera_calibrated': True,
            'camera_matrix': [[1, 0, 2], [0, 1, 3], [0, 0, 1]],
            'distortion_coefficients': [0, 0, 0, 0, 0]}
    save_yaml(path, data)
    assert load_yaml(path) == data


@pytest.mark.parametrize('data', [
    {}, {'camera_calibrated': True},
    {'scale_calibrated': True},
])
def test_metric_output_forbidden_without_full_calibration(data):
    with pytest.raises(RuntimeError):
        require_metric_calibration(data)


def test_metric_gate_accepts_both_real_calibrations():
    assert require_metric_calibration({
        'camera_calibrated': True, 'scale_calibrated': True})


def test_openmv_main_uses_formal_detector_not_red_threshold():
    main = (OPENMV / 'main.py').read_text()
    detector = (OPENMV / 'detector.py').read_text()
    assert 'DTaskDetector' in main
    assert 'detect_red' not in main
    assert 'RED_THRESHOLDS' not in detector


def test_openmv_detector_has_finite_roi_recovery():
    source = (OPENMV / 'detector.py').read_text()
    assert 'ROI_FAILURE_LIMIT' in source
    assert 'self.roi_failures >= ROI_FAILURE_LIMIT' in source


def test_mission_compatible_schema_remains_seven_fields():
    from uav_vision import d_task_schema
    assert d_task_schema.DETECTION_LENGTH == 7
    assert d_task_schema.validate_detection(
        [1, 160, 120, 100, 60, -0.2, 88]) == [
            1.0, 160.0, 120.0, 100.0, 60.0, -0.2, 88.0]
