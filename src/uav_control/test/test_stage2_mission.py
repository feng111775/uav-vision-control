import math
from pathlib import Path

import pytest

from uav_control.mission_logic import MissionLogic
from uav_control.mission_schema import CarProgress, parse_mission_mode
from uav_control.mission_schema import PX4_LOCAL_POSITION_TOPIC
from uav_control.mission_schema import state_allows_flight_setpoint
from uav_control.mission_schema import STATE_ID, TELEMETRY_LENGTH
from uav_control.px4_command_tracker import CommandTracker
from uav_control.touchdown_detector import TouchdownDetector
from uav_control.visual_guidance import VisualGuidance


@pytest.mark.parametrize('mode', ['drop', 'dynamic_land', 'hover_test'])
def test_three_modes(mode): assert parse_mission_mode(mode)


def ready_logic(mode='drop', **kwargs):
    logic = MissionLogic(mode, simulation_mode=True, enable_control=True,
                         enable_auto_arm=True, prestream_seconds=0.1,
                         stable_seconds=0.1, **kwargs)
    logic.update_position(2, 3, 4, 0, 0, 0, 0.2, True)
    logic.update_status(False, False, False)
    logic.step(0)
    logic.step(0.01)
    logic.start_signal = True
    logic.step(0.02)
    return logic


def test_no_h_blocks_start():
    logic = MissionLogic()
    logic.px4_fresh = True
    logic.step(1)
    assert logic.state == 'WAIT_PX4'


def test_h_capture_and_ned_height():
    logic = ready_logic()
    assert logic.h[:3] == (2, 3, 4)
    assert logic.cruise_z == pytest.approx(2.5)


def test_start_gate():
    logic = MissionLogic(simulation_mode=True)
    logic.update_position(0, 0, 0, 0, 0, 0, 0, True)
    logic.step(0)
    logic.step(.1)
    assert logic.state == 'WAIT_START'


def test_real_safety_gate():
    logic = MissionLogic()
    logic.update_position(0, 0, 0, 0, 0, 0, 0, True)
    logic.step(0)
    assert logic.state == 'WAIT_SAFETY'


def test_real_default_never_auto_arms():
    logic = ready_logic('hover_test')
    logic.simulation_mode = False
    logic.competition_mode = False
    assert not logic.auto_arm_allowed()


def test_sitl_auto_arm_conditions(): assert ready_logic().auto_arm_allowed()


def test_car_monotonic_and_idempotent():
    logic = MissionLogic()
    assert logic.update_car_progress(2)
    assert logic.update_car_progress(2)
    assert logic.car_progress == CarProgress.PASSED_B


def test_car_regression_rejected():
    logic = MissionLogic()
    logic.update_car_progress(3)
    assert not logic.update_car_progress(2)
    assert logic.car_regression


def test_b_deadline_risk():
    logic = ready_logic()
    logic.started_at = 0
    logic.state = 'SEARCH_TARGET'
    logic.step(16)
    assert logic.event == 'B_DEADLINE_RISK'


@pytest.mark.parametrize('state', ['DROP_ALIGN', 'DROP_RELEASE',
                         'LANDING_ALIGN', 'DESCEND_ON_CAR'])
def test_d_passed_aborts_actions(state):
    logic = ready_logic()
    logic.state = state
    logic.car_progress = CarProgress.PASSED_D
    logic.step(1)
    assert logic.state == 'RETURN_H'


@pytest.mark.parametrize('state', ['SEARCH_TARGET', 'FOLLOW_TARGET'])
def test_d_passed_aborts_search_and_follow(state):
    logic = ready_logic()
    logic.state = state
    logic.car_progress = CarProgress.PASSED_D
    logic.step(1)
    assert logic.state == 'RETURN_H'


def test_mission_timeout():
    logic = ready_logic()
    logic.started_at = 0
    logic.state = 'SEARCH_TARGET'
    logic.step(91)
    assert logic.state == 'FAILSAFE_LAND'


def test_px4_failsafe_and_tilt():
    logic = ready_logic()
    logic.failsafe = True
    logic.step(1)
    assert logic.state == 'FAILSAFE_LAND'
    logic = ready_logic()
    logic.abnormal_tilt = True
    logic.step(1)
    assert logic.state == 'FAILSAFE_LAND'


def test_manual_offboard_exit():
    logic = ready_logic()
    logic.state = 'TAKEOFF'
    logic.update_status(True, True, False)
    logic.update_status(True, False, False)
    assert logic.state == 'EXTERNAL_CONTROL'


def test_expected_offboard_exit_during_platform_disarm_is_not_manual_takeover():
    for state in ('TOUCHDOWN_VERIFY', 'DISARM_ON_CAR', 'DWELL_ON_CAR',
                  'SECOND_PRESTREAM', 'REQUEST_OFFBOARD', 'SECOND_ARM'):
        logic = ready_logic()
        logic.state = state
        logic.ever_offboard = True
        logic.update_status(False, False, False)
        assert logic.state == state


def test_hover_three_seconds():
    logic = ready_logic()
    logic.state = 'HOVER_CONFIRM'
    logic.state_since = 0
    logic.step(2.9)
    assert logic.state == 'HOVER_CONFIRM'
    logic.step(3)
    assert logic.state == 'SEARCH_TARGET'


def test_hover_test_ignores_visual_and_lands():
    logic = ready_logic('hover_test', hover_test_seconds=10)
    logic.state = 'HOVER_TEST'
    logic.state_since = 0
    logic.target_ok = True
    logic.step(10)
    assert logic.state == 'LAND_H'


def test_payload_once_and_ack():
    logic = ready_logic(enable_payload_release=True)
    logic.state = 'DROP_ALIGN'
    logic.position = (2, 3, logic.cruise_z)
    logic.update_visual(True, True, 0.0)
    logic.step(0.0)
    logic.update_visual(True, True, 0.4)
    logic.step(0.4)
    assert logic.payload_sent and logic.state == 'DROP_RELEASE'
    logic.step(2)
    assert logic.state == 'DROP_RELEASE'
    logic.payload_ack = True
    logic.step(3)
    assert logic.state == 'RETURN_H'


def test_drop_altitude_gate_allows_horizontal_follow_velocity():
    logic = ready_logic(enable_payload_release=True)
    logic.state = 'DROP_ALIGN'
    logic.position = (2, 3, logic.cruise_z)
    logic.velocity = (0.5, 0.0, 0.0)
    logic.update_visual(True, True, 0.0)
    logic.step(0.0)
    logic.update_visual(True, True, 0.4)
    logic.step(0.4)
    assert logic.state == 'DROP_RELEASE'


def test_payload_ack_timeout_aborts():
    logic = ready_logic(enable_payload_release=True, payload_ack_timeout=2)
    logic.state = 'DROP_RELEASE'
    logic.state_since = 0
    logic.step(2)
    assert logic.state == 'ABORT_RETURN_H'


def test_dynamic_descent_needs_alignment():
    logic = ready_logic('dynamic_land', enable_dynamic_landing=True)
    logic.state = 'LANDING_ALIGN'
    logic.aligned = False
    logic.step(1)
    assert logic.state == 'LANDING_ALIGN'


def test_touchdown_to_dwell_and_five_seconds():
    logic = ready_logic('dynamic_land', enable_dynamic_landing=True, enable_second_takeoff=True)
    logic.state = 'DESCEND_ON_CAR'
    logic.touchdown = True
    logic.step(1)
    assert logic.state == 'TOUCHDOWN_VERIFY'
    logic.step(1.9)
    assert logic.state == 'TOUCHDOWN_VERIFY'
    logic.step(2.0)
    assert logic.state == 'DISARM_ON_CAR'
    logic.armed = False
    logic.step(2.1)
    assert logic.state == 'DWELL_ON_CAR'
    logic.step(7.0)
    assert logic.state == 'DWELL_ON_CAR'
    logic.step(7.1)
    assert logic.state == 'SECOND_PRESTREAM'


def test_second_takeoff_returns_h():
    logic = ready_logic('dynamic_land')
    logic.state = 'SECOND_TAKEOFF'
    logic.position = (2, 3, logic.cruise_z)
    logic.velocity = (0, 0, 0)
    logic.step(0)
    logic.step(.11)
    assert logic.state == 'RETURN_H'


def test_platform_landing_records_second_takeoff_origin():
    logic = ready_logic(
        'dynamic_land', enable_dynamic_landing=True,
        enable_second_takeoff=True)
    logic.state = 'DISARM_ON_CAR'
    logic.position = (2.0, -3.0, 0.1)
    logic.armed = False
    logic.step(1.0)
    assert logic.state == 'DWELL_ON_CAR'
    assert logic.second_takeoff_origin == (2.0, -3.0, 0.1)


def test_second_prestream_resets_all_reused_command_trackers():
    source = (Path(__file__).parents[1] / 'uav_control' /
              'mission_controller_node.py').read_text()
    assert "('mode', 'arm', 'land', 'disarm')" in source


def test_at_h_requires_position_velocity_altitude():
    logic = ready_logic()
    logic.position = (2, 3, logic.cruise_z)
    logic.velocity = (0, 0, 0)
    assert logic.at_h()
    logic.velocity = (1, 0, 0)
    assert not logic.at_h()


def test_reset_only_disarmed_nonrunning():
    logic = MissionLogic()
    logic.state = 'COMPLETE'
    assert logic.reset_if_safe()
    logic.state = 'TAKEOFF'
    assert not logic.reset_if_safe()


@pytest.mark.parametrize('result,status', [(0, 'ACCEPTED'),
                         (2, 'DENIED'), (3, 'UNSUPPORTED'), (4, 'FAILED')])
def test_command_ack_results(result, status):
    tracker = CommandTracker()
    assert tracker.request(400, 0)
    assert tracker.acknowledge(400, result)
    assert tracker.status == status


def test_command_timeout_and_finite_retry():
    tracker = CommandTracker(timeout=1, max_attempts=2)
    assert tracker.request(400, 0)
    assert not tracker.request(400, .5)
    assert tracker.request(400, 1)
    assert not tracker.request(400, 2)
    assert tracker.status == 'TIMEOUT'


def test_second_command_tracker_reset():
    tracker = CommandTracker()
    tracker.request(400, 0)
    tracker.acknowledge(400, 0)
    tracker.reset()
    assert tracker.attempts == 0 and tracker.status == 'IDLE'


def test_flu_to_ned_directions():
    assert VisualGuidance.flu_to_ned(1, 0, 0) == pytest.approx((1, 0))
    assert VisualGuidance.flu_to_ned(1, 0, math.pi / 2) == pytest.approx((0, 1), abs=1e-8)


def test_visual_mapping_limit_timeout_and_jump():
    guide = VisualGuidance(max_speed=.2, camera_x_sign=1, camera_y_sign=-1)
    n, e = guide.velocity(True, 1, -1, 90, 0, 0)
    assert math.hypot(n, e) <= .2
    assert guide.velocity(True, 1, 1, 90, 999, 0) == (0, 0)
    guide.velocity(True, 0, 0, 90, 0, 0)
    assert guide.velocity(True, 2, 2, 90, 0, 0) == (0, 0)


def test_touchdown_requires_continuous_confirmation():
    detector = TouchdownDetector(confirm_seconds=1)
    assert not detector.update(0, True, False, 0, 0, 0)
    assert not detector.update(.5, True, False, 0, 0, 0)
    assert detector.update(1, True, False, 0, 0, 0)


def test_touchdown_single_frame_resets():
    detector = TouchdownDetector(confirm_seconds=1)
    detector.update(0, True, False, 0, 0, 0)
    assert not detector.update(.2, False, False, 0, 0, 0)


def test_schema_ids_and_telemetry():
    assert STATE_ID['WAIT_PX4'] == 0 and TELEMETRY_LENGTH >= 22


def test_wrong_command_ack_is_ignored():
    tracker = CommandTracker()
    tracker.request(400, 0)
    assert not tracker.acknowledge(176, 0)


def test_temporary_rejection_can_retry():
    tracker = CommandTracker(timeout=1, max_attempts=2)
    tracker.request(400, 0)
    tracker.acknowledge(400, 1)
    assert tracker.request(400, 0.1)


def test_touchdown_rejects_abnormal_tilt():
    detector = TouchdownDetector(confirm_seconds=0.1)
    detector.update(0, True, False, 0, 1.0, 0)
    assert not detector.update(1, True, False, 0, 1.0, 0)


def test_visual_deadband_stops_both_axes():
    assert VisualGuidance().velocity(
        True, 0.01, 0.01, 90, 0, 0) == (0, 0)


def test_image_right_error_commands_body_right():
    north, east = VisualGuidance().velocity(True, 0.2, 0.0, 90, 0, 0)
    assert north == pytest.approx(0.0)
    assert east > 0.0


def test_visual_invalid_immediately_stops():
    guide = VisualGuidance()
    guide.velocity(True, 0.2, 0.2, 90, 0, 0)
    assert guide.velocity(False, 0.2, 0.2, 90, 0, 0) == (0, 0)


def test_external_control_is_terminal():
    logic = ready_logic()
    logic.state = 'EXTERNAL_CONTROL'
    logic.offboard = True
    logic.step(100)
    assert logic.state == 'EXTERNAL_CONTROL'


def test_formal_dwell_cannot_be_shortened():
    assert MissionLogic(dwell_on_car_seconds=0.1).dwell_on_car_seconds == 5.0


def test_unfresh_px4_cannot_leave_wait_px4():
    logic = MissionLogic(simulation_mode=True)
    logic.px4_fresh = False
    logic.step(1)
    assert logic.state == 'WAIT_PX4'


def test_px4_data_timeout_during_active_mission_enters_failsafe():
    logic = ready_logic()
    logic.state = 'TAKEOFF'
    logic.started_at = 1.0
    logic.px4_fresh = False
    logic.step(2.0)
    assert logic.state == 'FAILSAFE_LAND'
    assert logic.event == 'PX4_DATA_TIMEOUT'


def test_px4_v116_local_position_topic_has_no_version_suffix():
    assert PX4_LOCAL_POSITION_TOPIC == '/fmu/out/vehicle_local_position'


def test_disarm_and_landing_states_publish_no_flight_setpoints():
    for state in ('LAND_H', 'WAIT_DISARM', 'DWELL_ON_CAR'):
        assert not state_allows_flight_setpoint(state)
    assert state_allows_flight_setpoint('TOUCHDOWN_VERIFY')
    assert state_allows_flight_setpoint('DISARM_ON_CAR')
    assert state_allows_flight_setpoint('TAKEOFF')


def test_sitl_touchdown_verification_window_reaches_ground_conservatively():
    config = (Path(__file__).parents[1] / 'config' /
              'sitl_dynamic_land.yaml').read_text()
    assert 'touchdown_verify_seconds: 2.0' in config
    assert 'kinematic_touchdown_height_tolerance: 0.03' in config
    assert 'command_max_attempts: 8' in config
    assert 'sitl_nav_land_after_touchdown: true' in config
    competition = (Path(__file__).parents[1] / 'config' /
                   'competition_dynamic_land.yaml').read_text()
    assert 'sitl_nav_land_after_touchdown: true' not in competition
    source = (Path(__file__).parents[1] / 'uav_control' /
              'mission_controller_node.py').read_text()
    assert "self.get_parameter('near_descent_speed').value" in source


def test_sitl_scenario_uses_px4_best_effort_qos():
    source = (Path(__file__).parents[2] / 'uav_vision' / 'uav_vision' /
              'd_task_sitl_scenario_node.py').read_text()
    assert 'ReliabilityPolicy.BEST_EFFORT' in source


def test_sitl_fault_modes_are_explicit_and_do_not_affect_competition_launch():
    scenario = (Path(__file__).parents[2] / 'uav_vision' / 'uav_vision' /
                'd_task_sitl_scenario_node.py').read_text()
    for mode in ('early_d', 'short_vision_loss',
                 'sustained_vision_loss', 'payload_ack_loss'):
        assert mode in scenario
    competition_launch = (Path(__file__).parents[1] / 'launch' /
                          'd_task_control.launch.py').read_text()
    assert 'fault_mode' not in competition_launch
