"""Pure-software regression tests for stage 4C-3A safety hardening."""

import math
from pathlib import Path

import pytest

from uav_control.px4_command_tracker import CommandTracker
from uav_control.stage4c_core import (
    MarkerObservation, MissionConfig, MissionFlow, MissionState,
    PersistentPayloadGate, resolve_takeoff_height, StartGate,
    validate_control_interlocks)
import yaml


def airborne_flow(state=MissionState.FOLLOW_CAR):
    flow = MissionFlow()
    flow.initialize(0.0)
    flow.start('session:start', 1.0, (3.0, 4.0, 2.0))
    flow.transition(state, 2.0, 'test')
    flow.follow_since = 2.0
    return flow


@pytest.mark.parametrize('state', [
    MissionState.TAKEOFF, MissionState.HOVER_STABLE,
    MissionState.TRANSIT_TO_INTERCEPT, MissionState.SEARCH_CAR,
    MissionState.ACQUIRE_CAR, MissionState.FOLLOW_CAR,
    MissionState.DROP_ALIGN, MissionState.RELEASE_PAYLOAD,
    MissionState.RETURN_HOME, MissionState.ABORT_RETURN,
])
def test_emergency_land_is_irreversible_from_every_flight_state(state):
    flow = airborne_flow(state)
    flow.update(3.0, failsafe=True)
    assert flow.state == MissionState.EMERGENCY_LAND
    flow.update(
        200.0, observation=MarkerObservation(200.0, True, .9, 0.0, 0.0),
        intercept_reached=True, payload_result='SUCCESS', offboard_lost=True)
    assert flow.state == MissionState.EMERGENCY_LAND
    assert flow.command(200.0)['mode'] == 'LAND'
    assert not flow.start('other:start', 201.0, (0.0, 0.0, 0.0))


def test_emergency_land_only_completes_on_landed():
    flow = airborne_flow()
    flow.update(3.0, failsafe=True)
    flow.update(1000.0, landed=False)
    assert flow.state == MissionState.EMERGENCY_LAND
    flow.update(1001.0, landed=True)
    assert flow.state == MissionState.COMPLETE


@pytest.mark.parametrize('short,long', [
    (-0.1, 1.0), (1.1, 1.0), (math.nan, 1.0), (0.1, math.inf)])
def test_target_loss_thresholds_fail_closed(short, long):
    with pytest.raises(ValueError):
        MissionConfig(
            target_short_loss_s=short, target_long_loss_s=long)


@pytest.mark.parametrize('elapsed,expected', [
    (0.49, MissionState.FOLLOW_CAR),
    (0.50, MissionState.FOLLOW_CAR),
    (0.51, MissionState.FOLLOW_CAR),
    (2.00, MissionState.SEARCH_CAR),
])
def test_short_loss_boundary_is_explicit(elapsed, expected):
    flow = airborne_flow()
    flow.update(3.0, observation=None)
    flow.update(3.0 + elapsed, observation=None, heading=0.0)
    assert flow.state == expected
    if 0.5 <= elapsed < 2.0:
        assert flow.last_reason == 'target_short_loss_hold'


@pytest.mark.parametrize('stamp', [-1.0, math.nan, math.inf, 20.0])
def test_invalid_or_future_marker_timestamp_is_rejected(stamp):
    observation = MarkerObservation(stamp, True, .9, 0.0, 0.0)
    assert not observation.fresh(10.0, .5)


def test_target_recovery_before_short_threshold_resumes_follow():
    flow = airborne_flow()
    flow.update(3.0)
    observation = MarkerObservation(3.4, True, .9, 0.2, 0.0)
    flow.update(3.4, observation=observation)
    assert flow.state == MissionState.FOLLOW_CAR


@pytest.mark.parametrize('values,valid', [
    ((True, True, True, True, False), True),
    ((True, False, True, True, False), False),
    ((False, True, False, False, False), False),
    ((False, False, False, False, True), True),
    ((False, False, False, True, True), False),
])
def test_control_interlock_combinations(values, valid):
    if valid:
        assert validate_control_interlocks(*values)
    else:
        with pytest.raises(ValueError):
            validate_control_interlocks(*values)


def test_height_parameter_has_one_authoritative_value():
    assert resolve_takeoff_height(1.5, 1.5) == 1.5
    with pytest.raises(ValueError):
        resolve_takeoff_height(1.5, 1.0)


def test_ack_requires_pending_fresh_matching_target_and_context():
    tracker = CommandTracker(timeout=1.0, max_attempts=2)
    assert tracker.request(400, 10.0, 'task-a', 1, 1)
    assert not tracker.acknowledge(176, 0, 10.1, 1, 1, 'task-a')
    assert not tracker.acknowledge(400, 0, 10.1, 2, 1, 'task-a')
    assert not tracker.acknowledge(400, 0, 10.1, 1, 1, 'task-b')
    assert not tracker.acknowledge(
        400, 0, 10.1, 1, 1, 'task-a', from_external=False)
    assert not tracker.acknowledge(400, 0, 11.1, 1, 1, 'task-a')
    assert tracker.request(400, 11.1, 'task-a', 1, 1)
    assert tracker.acknowledge(400, 0, 11.2, 1, 1, 'task-a')
    assert not tracker.acknowledge(400, 0, 11.3, 1, 1, 'task-a')


def test_tracker_reset_rejects_prestart_and_previous_task_ack():
    tracker = CommandTracker()
    assert not tracker.acknowledge(400, 0, 1.0, 1, 1, 'old')
    tracker.request(400, 2.0, 'old')
    tracker.reset()
    assert not tracker.acknowledge(400, 0, 2.1, 1, 1, 'old')


def test_start_protocol_is_idempotent_under_retries_and_reconnect(tmp_path):
    path = tmp_path / 'starts.json'
    gate = StartGate(str(path))
    results = [
        gate.accept_protocol('session-a', 'start-1', True, True, False)
        for _ in range(20)]
    assert results == ['START_ACCEPTED'] + ['ALREADY_PROCESSED'] * 19
    restarted = StartGate(str(path))
    assert restarted.accept_protocol(
        'session-a', 'start-1', True, True, False) == 'ALREADY_PROCESSED'
    assert restarted.accept_protocol(
        'session-a', 'start-2', True, True, False) == 'START_ACCEPTED'
    assert restarted.accept_protocol(
        'session-b', 'start-1', True, False, False) == 'NOT_READY'
    assert restarted.accept_protocol(
        'session-b', 'start-1', True, True, True) == 'BUSY'


def test_fifty_start_events_do_not_duplicate():
    gate = StartGate()
    accepted = 0
    for index in range(50):
        status = gate.accept_protocol(
            'wifi-reconnect', str(index // 5), True, True, False)
        accepted += status == 'START_ACCEPTED'
    assert accepted == 10


def test_payload_journal_is_atomic_idempotent_and_restart_safe(tmp_path):
    path = tmp_path / 'payload.json'
    gate = PersistentPayloadGate(str(path))
    assert gate.begin('session', 'release-1', True) == 'EXECUTING'
    restarted = PersistentPayloadGate(str(path))
    assert restarted.state == 'LOCKED'
    assert restarted.begin('session', 'release-1', True) == 'LOCKED'
    assert not restarted.reset_lock('wrong')
    assert restarted.reset_lock('RESET_LOCK')
    assert restarted.begin('session', 'release-1', True) == 'EXECUTING'
    assert restarted.finish('SUCCESS') == 'SUCCESS'
    assert restarted.begin('session', 'release-1', True) == 'REJECTED'


def test_corrupt_payload_journal_locks(tmp_path):
    path = tmp_path / 'payload.json'
    path.write_text('{broken')
    assert PersistentPayloadGate(str(path)).state == 'LOCKED'


def test_payload_cancel_is_terminal_and_idempotent(tmp_path):
    gate = PersistentPayloadGate(str(tmp_path / 'cancel.json'))
    assert gate.begin('session', 'release', True) == 'EXECUTING'
    assert gate.cancel() == 'REJECTED'
    assert gate.begin('session', 'release', True) == 'REJECTED'


def test_payload_atomic_write_failure_locks(tmp_path, monkeypatch):
    gate = PersistentPayloadGate(str(tmp_path / 'failure.json'))

    def fail_replace(_source, _destination):
        raise OSError('injected atomic replace failure')

    monkeypatch.setattr('uav_control.stage4c_core.os.replace', fail_replace)
    with pytest.raises(OSError):
        gate.begin('session', 'release', True)
    assert gate.state == 'LOCKED'


@pytest.mark.parametrize('result', ['SUCCESS', 'FAILED', 'TIMEOUT'])
def test_payload_software_terminal_results_are_persisted(tmp_path, result):
    gate = PersistentPayloadGate(str(tmp_path / (result + '.json')))
    assert gate.begin('session', 'release', True) == 'EXECUTING'
    assert gate.finish(result) == result


def test_hardware_bench_and_sitl_files_are_safely_isolated():
    root = Path(__file__).parents[1]
    bench = yaml.safe_load(
        (root / 'config/mission_stage4c_hardware_bench.yaml').read_text())
    params = bench['/**']['ros__parameters']
    assert params['enable_control'] is False
    assert params['enable_auto_arm'] is False
    assert params['simulation_mode'] is False
    assert params['dry_run'] is True
    assert params['physical_release_enabled'] is False
    launch = (
        root / 'launch/uav_mission_stage4c_hardware_bench.launch.py').read_text()
    assert 'd_task_mock_node' not in launch
    assert 'sitl_acceptance_driver' not in launch
    assert 'servo' not in launch.lower()
    sitl = yaml.safe_load(
        (root / 'config/mission_stage4c_sitl.yaml').read_text())
    controller = sitl['mission_offboard_controller']['ros__parameters']
    assert controller['confirm_sitl_only'] is True


def test_no_gpio_pwm_or_physical_backend_exists():
    root = Path(__file__).parents[1]
    source = '\n'.join(
        path.read_text(errors='ignore')
        for path in (root / 'uav_control').glob('*.py'))
    forbidden = (
        'RPi.GPIO', 'gpiozero', 'pigpio', 'lgpio', '/dev/gpiomem')
    assert all(token not in source for token in forbidden)
    assert 'physical payload output is not implemented' in source


def test_manager_emits_one_fixed_release_identifier():
    source = (
        Path(__file__).parents[1] /
        'uav_control/mission_manager.py').read_text()
    assert "self.flow.task_id + '-release-1'" in source
    assert 'not self.release_request_sent' in source
    assert 'self.flow.state == MissionState.RELEASE_PAYLOAD' in source
