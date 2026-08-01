import json
import math

import pytest

from std_msgs.msg import String

from uav_control.mission_controller_node import MissionControllerNode
from uav_control.mission_logic import MissionLogic
from uav_control.mission_schema import MissionState
from uav_control.vision_contract import validate_landing_error, validate_tracked
from uav_control.visual_guidance import map_camera_error, VisualGuidance


def tracked(valid=1.0, confidence=90.0, age=10.0):
    return [valid, 160.0, 120.0, 50.0, 30.0, 0.0, confidence,
            0.0, 0.0, 160.0, 120.0, age]


def landing(valid=1.0, confidence=90.0, age=10.0, error_x=0.0,
            error_y=0.0):
    return [valid, error_x, error_y, error_x * 160.0,
            error_y * 120.0, 0.0, confidence, age]


@pytest.mark.parametrize('values, validator', [
    ([0.0] * 11, validate_tracked),
    ([0.0] * 7, validate_landing_error),
    ([math.nan] * 12, validate_tracked),
    ([math.inf] * 8, validate_landing_error),
])
def test_malformed_visual_arrays_are_rejected(values, validator):
    with pytest.raises(ValueError):
        validator(values)


def test_extra_visual_fields_are_allowed_and_valid_is_thresholded():
    assert len(validate_tracked(tracked() + [123.0])) == 13
    assert validate_tracked(tracked(valid=0.3))[0] == 0.3
    assert validate_landing_error(landing(valid=0.3))[0] == 0.3


def test_mapping_supports_swap_signs_and_target_offset():
    assert map_camera_error(0.2, -0.4, swap_axes=True,
                            x_sign=-1.0, y_sign=1.0,
                            target_x=0.1, target_y=-0.2) == pytest.approx((0.3, 0.4))


def test_mapping_rejects_zero_sign():
    with pytest.raises(ValueError):
        map_camera_error(0.0, 0.0, x_sign=0.0)


def test_guidance_speed_and_acceleration_are_bounded():
    guidance = VisualGuidance(max_speed=0.18, max_acceleration=0.3,
                              camera_x_sign=1.0, camera_y_sign=1.0)
    velocity = guidance.velocity(True, 1.0, 1.0, 90.0, 10.0, 0.0, 0.1)
    assert math.hypot(*velocity) <= 0.18
    assert math.hypot(*velocity) <= 0.03 + 1e-9


def test_invalid_guidance_smoothly_returns_to_zero():
    guidance = VisualGuidance(max_speed=0.18, max_acceleration=0.3)
    guidance.velocity(True, 0.4, 0.4, 90.0, 10.0, 0.0, 1.0)
    first = guidance.velocity(False, 0.4, 0.4, 90.0, 10.0, 0.0, 0.1)
    second = guidance.velocity(False, 0.4, 0.4, 90.0, 10.0, 0.0, 1.0)
    assert math.hypot(*second) < math.hypot(*first)


def test_single_alignment_sample_does_not_release():
    logic = MissionLogic('drop', enable_payload_release=True,
                         visual_stable_seconds=0.8)
    logic.transition(MissionState.ALIGN_FOR_DROP, 0.0)
    logic.position = (0.0, 0.0, 0.0)
    logic.h = (0.0, 0.0, 0.0, 0.0)
    logic.update_visual(True, True, 1.0)
    logic.step(1.1)
    assert not logic.payload_sent
    logic.update_visual(False, False, 1.2)
    logic.step(2.0)
    assert not logic.payload_sent


def test_payload_timeout_is_failure_and_never_success():
    logic = MissionLogic('drop', enable_payload_release=True,
                         payload_ack_timeout=1.0)
    logic.transition(MissionState.WAIT_RELEASE_ACK, 0.0)
    logic.step(1.0)
    assert logic.state == MissionState.FAILSAFE.value
    assert not logic.payload_ack


def test_payload_disabled_never_sets_release_lock():
    logic = MissionLogic('drop', enable_payload_release=False)
    logic.transition(MissionState.ALIGN_FOR_DROP, 0.0)
    logic.position = (0.0, 0.0, 0.0)
    logic.h = (0.0, 0.0, 0.0, 0.0)
    logic.update_visual(True, True, 0.0)
    logic.update_visual(True, True, 1.0)
    logic.step(1.0)
    assert not logic.payload_sent


def test_reset_cannot_clear_completed_payload_lock():
    logic = MissionLogic('drop', enable_payload_release=True)
    logic.state = MissionState.COMPLETE.value
    logic.payload_sent = True
    assert not logic.reset_if_safe()
    assert logic.state == MissionState.COMPLETE.value
    assert logic.payload_sent


def fake_controller_for_ack(state=MissionState.WAIT_RELEASE_ACK.value):
    node = object.__new__(MissionControllerNode)
    node.payload_command_sent = True
    node.payload_request_monotonic = 0.0
    node.payload_ack_seen = False
    node.payload_sequence_id = 7
    node.logic = MissionLogic('drop', payload_ack_timeout=1.0)
    node.logic.state = state
    node.now = lambda: 10.0
    return node


def test_old_or_duplicate_servo_success_cannot_advance():
    node = fake_controller_for_ack()
    node.payload_command_sent = False
    message = String()
    message.data = json.dumps({'sequence_id': 7, 'status': 'SUCCESS'})
    MissionControllerNode._servo_result(node, message)
    assert not node.logic.payload_ack
    node.payload_command_sent = True
    MissionControllerNode._servo_result(node, message)
    assert node.logic.payload_ack
    state = node.logic.state
    MissionControllerNode._servo_result(node, message)
    assert node.logic.state == state


def test_wrong_sequence_ack_cannot_advance():
    node = fake_controller_for_ack()
    message = String()
    message.data = json.dumps({'sequence_id': 6, 'status': 'SUCCESS'})
    MissionControllerNode._servo_result(node, message)
    assert not node.logic.payload_ack


def test_dry_run_confirmed_ack_counts_as_success_for_current_sequence():
    node = fake_controller_for_ack()
    message = String()
    message.data = json.dumps({'sequence_id': 7,
                               'status': 'DRY_RUN_CONFIRMED'})
    MissionControllerNode._servo_result(node, message)
    assert node.logic.payload_ack
    assert node.logic.event == 'SERVO_SUCCESS_THROW'


def test_malformed_ack_cannot_advance():
    node = fake_controller_for_ack()
    message = String()
    message.data = 'SUCCESS:throw'
    MissionControllerNode._servo_result(node, message)
    assert not node.logic.payload_ack


@pytest.mark.parametrize('result', [
    {'sequence_id': 7, 'status': 'FAILED'},
    {'sequence_id': 7, 'status': 'REJECTED'},
    {'sequence_id': 7, 'status': 'DUPLICATE'},
    {'sequence_id': 7, 'status': 'UNKNOWN'}])
def test_non_success_servo_results_never_count_as_ack(result):
    node = fake_controller_for_ack()
    message = String()
    message.data = json.dumps(result)
    MissionControllerNode._servo_result(node, message)
    if result['status'] == 'UNKNOWN':
        assert not node.logic.payload_ack
    else:
        assert node.logic.state == MissionState.FAILSAFE.value
        assert not node.logic.payload_ack
