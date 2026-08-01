from pathlib import Path

import pytest

from uav_control.hover_matrix import (
    failure_result, HoverMetrics, matrix_cases, mode_recovery_safe,
    mode_recovery_status_valid, PreflightStability, record_mode_recovery,
    RESULT_FIELDS, result_passed, run_until_failure)


PACKAGE_ROOT = Path(__file__).parents[1]


def passing_result():
    return {
        'passed': True,
        'final_state': 'COMPLETE',
        'initial_disarmed': True,
        'auto_disarmed': True,
        'failsafe': False,
    }


def test_matrix_contains_exactly_nine_height_time_pairs():
    assert matrix_cases() == (
        (0.5, 5.0), (0.5, 10.0), (0.5, 15.0),
        (1.0, 5.0), (1.0, 10.0), (1.0, 15.0),
        (1.5, 5.0), (1.5, 10.0), (1.5, 15.0))


def test_timing_fields_have_distinct_boundaries():
    metrics = HoverMetrics(1.0, 0.1)
    metrics.update_state('ARMING', 'TAKEOFF', 2.0)
    metrics.update_state('TAKEOFF', 'HOVER_150CM', 5.0)
    metrics.update_height('HOVER_150CM', 0.95, 5.0)
    metrics.update_height('HOVER_150CM', 1.02, 15.0)
    metrics.update_state('HOVER_150CM', 'FINAL_LAND', 15.0)
    result = metrics.finish(20.2, 10.0)
    assert result['stable_hover_duration_s'] == pytest.approx(10.0)
    assert result['hover_stability_passed'] is True
    assert result['mission_elapsed_s'] == pytest.approx(18.2)
    # Compatibility field includes FINAL_LAND from 15.0 to 20.2.
    assert result['hover_duration_s'] == pytest.approx(15.2)


def test_stable_hover_is_longest_continuous_in_tolerance_segment():
    metrics = HoverMetrics(1.0, 0.1)
    metrics.update_state('TAKEOFF', 'HOVER_150CM', 0.0)
    metrics.update_height('HOVER_150CM', 1.0, 0.0)
    metrics.update_height('HOVER_150CM', 1.2, 4.0)
    metrics.update_height('HOVER_150CM', 1.0, 5.0)
    metrics.update_state('HOVER_150CM', 'FINAL_LAND', 8.0)
    result = metrics.finish(9.0, 8.0)
    assert result['stable_hover_duration_s'] == 4.0
    assert result['hover_stability_passed'] is False


@pytest.mark.parametrize('field,value', [
    ('failsafe', True),
    ('auto_disarmed', False),
    ('final_state', 'FINAL_LAND'),
    ('passed', False),
])
def test_matrix_stops_on_each_terminal_failure(field, value):
    calls = []

    def runner(height, duration):
        calls.append((height, duration))
        result = passing_result()
        if len(calls) == 2:
            result[field] = value
        return result

    results = run_until_failure(matrix_cases(), runner)
    assert len(results) == 2
    assert len(calls) == 2
    assert not result_passed(results[-1])


def test_result_schema_contains_all_required_matrix_fields():
    assert set(RESULT_FIELDS) == {
        'target_height_m', 'commanded_hover_duration_s',
        'stable_hover_duration_s', 'hover_stability_passed',
        'mission_elapsed_s',
        'mean_hover_height_m', 'max_hover_height_m', 'height_error_m',
        'initial_disarmed', 'auto_disarmed', 'failsafe', 'final_state',
        'passed', 'reason'}


def test_launch_passes_matrix_parameters_to_controller_and_recorder():
    text = (
        PACKAGE_ROOT / 'launch' / 'sitl_hover.launch.py').read_text()
    assert "'target_altitude': target_height" in text
    assert "'hover_test_seconds': hover_duration" in text
    assert "'target_height_m': target_height" in text
    assert "'commanded_hover_duration_s': hover_duration" in text
    assert "'altitude_tolerance_m': 0.1" in text


def test_runner_rejects_residual_controller_and_non_sitl_links():
    text = (
        PACKAGE_ROOT / 'scripts' / 'run_sitl_hover.sh').read_text()
    assert text.count(
        "grep -qx '/mission_controller_node'") >= 2
    assert 'px4_sitl_default/bin/px4' in text
    assert 'Gazebo SITL process was not found' in text
    assert 'expected exactly one MicroXRCEAgent instance' in text
    assert 'only a UDP4 SITL MicroXRCEAgent is allowed' in text
    assert 'SITL MicroXRCEAgent must use UDP port 8888' in text
    assert 'expected exactly one SITL vehicle_status publisher' in text


def test_matrix_script_stops_and_summarizes_on_first_failure():
    text = (
        PACKAGE_ROOT / 'scripts' / 'run_hover_matrix.sh').read_text()
    failure = text.index(
        'if "${script_dir}/run_sitl_hover.sh"')
    stop = text.index('exit "${case_rc}"', failure)
    loop_end = text.index('done < <(', failure)
    assert failure < stop < loop_end
    assert 'matrix_summary.csv' in text
    assert 'matrix_summary.json' in text
    assert 'matrix result directory already exists' in text
    assert 'runner failed before result' in text


def test_recorder_uses_controller_relative_home_height_and_monotonic_time():
    text = (
        PACKAGE_ROOT / 'uav_control' /
        'sitl_result_recorder.py').read_text()
    assert "TELEMETRY['relative_h_height']" in text
    assert 'time.monotonic()' in text
    assert 'msg.timestamp' not in text


def test_preflight_false_then_stably_true_passes_within_timeout():
    gate = PreflightStability(3.0, 1.0)
    assert not gate.update(100, False, 0.0)
    assert not gate.update(200, True, 0.5)
    for index, received_at in enumerate(
            (1.0, 1.5, 2.0, 2.5, 3.0), start=3):
        assert not gate.update(index * 100, True, received_at)
    assert gate.update(900, True, 3.5)


def test_preflight_continuously_false_never_passes():
    gate = PreflightStability(3.0, 1.0)
    for index in range(30):
        assert not gate.update(
            100 + index, False, index * 0.5)
    assert gate.reason == 'pre_flight_checks_pass is false'


def test_preflight_rejects_nonincreasing_or_stale_messages():
    gate = PreflightStability(1.0, 0.6)
    assert not gate.update(100, True, 0.0)
    assert not gate.update(100, True, 0.5)
    assert gate.reason == 'VehicleStatus timestamp did not increase'
    assert gate.expired(1.0)
    assert gate.reason == 'VehicleStatus stream is stale'


def test_canonical_preflight_failure_result_does_not_invent_flight_data():
    result = failure_result(
        'nominal', 1.0, 10.0,
        'PX4 preflight readiness timed out')
    assert result['passed'] is False
    assert result['reason'] == 'PX4 preflight readiness timed out'
    assert result['final_state'] == 'NOT_STARTED'
    assert result['initial_disarmed'] is None
    assert result['auto_disarmed'] is False
    assert result['failsafe'] is None
    assert result['mean_hover_height_m'] is None
    assert result['stable_hover_duration_s'] == 0.0


def test_every_runner_refusal_uses_structured_failure_and_checks_residual():
    text = (
        PACKAGE_ROOT / 'scripts' / 'run_sitl_hover.sh').read_text()
    assert 'write_failure_result "${destination}" "${reason}"' in text
    assert 'from uav_control.hover_matrix import failure_result' in text
    assert "grep -qx '/mission_controller_node'" in text
    for reason in (
            'PX4 SITL process was not found',
            'Gazebo SITL process was not found',
            'expected exactly one MicroXRCEAgent instance',
            'only a UDP4 SITL MicroXRCEAgent is allowed',
            'expected exactly one SITL vehicle_status publisher',
            'PX4 preflight readiness timed out',
            'local position ${field} is not true',
            'mission_controller_node already exists'):
        assert reason in text


def test_mode_recovery_is_prohibited_until_landed_disarmed_and_fresh():
    assert mode_recovery_safe(
        armed=False, landed=True, status_fresh=True, land_fresh=True)
    assert not mode_recovery_safe(
        armed=True, landed=True, status_fresh=True, land_fresh=True)
    assert not mode_recovery_safe(
        armed=False, landed=False, status_fresh=True, land_fresh=True)
    assert not mode_recovery_safe(
        armed=False, landed=True, status_fresh=False, land_fresh=True)
    assert not mode_recovery_safe(
        armed=False, landed=True, status_fresh=True, land_fresh=False)


def test_recovery_accepts_false_preflight_after_accepted_mode_change():
    assert mode_recovery_status_valid(
        ack_result=0, accepted_result=0,
        arming_state=1, disarmed_value=1,
        nav_state=2, nav_state_user_intention=2,
        failsafe=False, landed=True, land_fresh=True,
        pre_flight_checks_pass=False)


@pytest.mark.parametrize(
    'override',
    (
        {'ack_result': None},
        {'arming_state': 2},
        {'nav_state': 14},
        {'nav_state_user_intention': 14},
        {'failsafe': True},
        {'landed': False},
        {'land_fresh': False},
    ))
def test_recovery_rejects_each_dangerous_or_unverified_state(override):
    values = {
        'ack_result': 0,
        'accepted_result': 0,
        'arming_state': 1,
        'disarmed_value': 1,
        'nav_state': 2,
        'nav_state_user_intention': 2,
        'failsafe': False,
        'landed': True,
        'land_fresh': True,
        'pre_flight_checks_pass': False,
    }
    values.update(override)
    assert not mode_recovery_status_valid(**values)


def test_recovery_status_must_remain_valid_for_stability_window():
    stable = PreflightStability(3.0, 1.0)
    for index in range(6):
        valid = mode_recovery_status_valid(
            0, 0, 1, 1, 2, 2, False, True, True, False)
        assert valid
        assert not stable.update(index + 1, valid, index * 0.5)
    assert stable.update(7, True, 3.0)


def test_recovery_failure_marks_run_failed_without_erasing_flight_data(
        tmp_path):
    destination = tmp_path / 'nominal_run_1.json'
    original = passing_result()
    original['mean_hover_height_m'] = 0.987
    destination.write_text(
        __import__('json').dumps(original), encoding='utf-8')
    result = record_mode_recovery(
        destination, False, 'PX4 rejected mode recovery')
    assert result['passed'] is False
    assert result['mode_recovery_passed'] is False
    assert 'PX4 rejected mode recovery' in result['reason']
    assert result['mean_hover_height_m'] == 0.987


def test_runner_recovers_only_after_launch_exit_and_residual_check():
    text = (
        PACKAGE_ROOT / 'scripts' / 'run_sitl_hover.sh').read_text()
    launch = text.index(
        'ros2 launch uav_control sitl_hover.launch.py')
    residual = text.index(
        'left a residual mission_controller_node', launch)
    recovery = text.index(
        'ros2 run uav_control sitl_mode_recovery', residual)
    next_run = text.index('done', recovery)
    assert launch < residual < recovery < next_run
    assert 'exit 9' in text[recovery:next_run]
    assert 'record_recovery_result "${result}" false' in text[
        recovery:next_run]


def test_mode_recovery_node_has_airborne_and_ack_fail_closed_guards():
    text = (
        PACKAGE_ROOT / 'uav_control' /
        'sitl_mode_recovery.py').read_text()
    assert 'mode_recovery_safe(' in text
    assert 'airborne or armed state observed' in text
    assert 'PX4 rejected mode recovery' in text
    assert 'VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED' in text
    assert 'mode_recovery_status_valid(' in text
    assert 'msg.pre_flight_checks_pass' in text
    assert 'VEHICLE_CMD_DO_SET_MODE' in text
    assert 'VEHICLE_CMD_COMPONENT_ARM_DISARM' not in text
    assert "self.declare_parameter('simulation_mode', False)" in text
    assert 'simulation_mode=true is required for mode recovery' in text


def test_missing_vehicle_status_graph_fails_through_structured_path():
    text = (
        PACKAGE_ROOT / 'scripts' / 'run_sitl_hover.sh').read_text()
    assignment = text.index('publisher_count="$(')
    refusal = text.index(
        "fail_run 3 'expected exactly one SITL vehicle_status publisher'",
        assignment)
    assert "|| publisher_count=''" in text[assignment:refusal]
