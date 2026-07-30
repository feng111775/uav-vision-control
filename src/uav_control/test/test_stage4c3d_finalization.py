"""Stage 4C-3D restart, adapter and final safety contracts."""

import json
import math
from pathlib import Path

import pytest

from uav_control.car_marker_vision import normalize_marker_error
from uav_control.car_start_gateway import (
    heartbeat_is_current, StartTransport)
from uav_control.stage4c_core import (
    MarkerObservation, MissionConfig, MissionFlow, MissionState,
    PersistentPayloadGate)
import yaml


def test_real_start_transports_fail_closed_without_parameters_or_backend():
    StartTransport('simulation')
    with pytest.raises(ValueError):
        StartTransport('simulation', real_enabled=True)
    with pytest.raises(ValueError):
        StartTransport('serial', real_enabled=True)
    with pytest.raises(ValueError):
        StartTransport(
            'serial', real_enabled=True, serial_device='/undefined',
            serial_baud=115200)
    with pytest.raises(ValueError):
        StartTransport('udp', real_enabled=True)
    with pytest.raises(ValueError):
        StartTransport(
            'udp', real_enabled=True, udp_host='192.0.2.1', udp_port=5000)


@pytest.mark.parametrize('last,now,timeout,expected', [
    (None, 1, 1.0, False),
    (1_000_000_000, 2_000_000_000, 1.0, True),
    (1_000_000_000, 2_000_000_001, 1.0, False),
    (2_000_000_000, 1_000_000_000, 1.0, False),
    (math.nan, 1_000_000_000, 1.0, False),
])
def test_heartbeat_freshness_boundaries(last, now, timeout, expected):
    assert heartbeat_is_current(last, now, timeout) is expected


def test_marker_pixel_normalization_and_invalid_input():
    assert normalize_marker_error(
        320, -240, False, 640, 480) == pytest.approx((1.0, -1.0))
    assert normalize_marker_error(
        0.25, -0.5, True, 640, 480) == pytest.approx((0.25, -0.5))
    for values in (
            (math.nan, 0.0, True, 640, 480),
            (math.inf, 0.0, True, 640, 480),
            (1.0, 1.0, False, 0, 480),
            (3.0, 0.0, True, 640, 480)):
        with pytest.raises(ValueError):
            normalize_marker_error(*values)


def test_payload_ledger_records_canonical_states_and_duplicate_result(tmp_path):
    path = tmp_path / 'payload.json'
    gate = PersistentPayloadGate(str(path))
    assert gate.begin('mission', 'release-1', True) == 'EXECUTING'
    executing = json.loads(path.read_text())
    assert executing['ledger_state'] == 'EXECUTION_STARTED'
    assert PersistentPayloadGate(str(path)).ledger_state == 'UNKNOWN_LOCKED'

    path.unlink()
    gate = PersistentPayloadGate(str(path))
    gate.begin('mission', 'release-1', True)
    assert gate.finish('DRY_RUN_CONFIRMED') == 'DRY_RUN_CONFIRMED'
    restarted = PersistentPayloadGate(str(path))
    assert restarted.historical_execution_count == 1
    assert restarted.previous_result(
        'mission', 'release-1') == 'DRY_RUN_CONFIRMED'
    assert restarted.begin('mission', 'release-1', True) == 'REJECTED'


def test_corrupt_payload_ledger_is_unknown_locked(tmp_path):
    path = tmp_path / 'payload.json'
    path.write_text('not-json')
    gate = PersistentPayloadGate(str(path))
    assert gate.state == 'LOCKED'
    assert gate.ledger_state == 'UNKNOWN_LOCKED'


def test_transport_loss_has_named_safe_exit_and_no_release():
    flow = MissionFlow()
    flow.initialize(0.0)
    assert flow.start('mission', 1.0, (0.0, 0.0, 0.0))
    flow.transition(MissionState.FOLLOW_CAR, 2.0, 'test')
    flow.update(
        2.1, position=(0.0, 0.0, -1.5),
        observation=MarkerObservation(2.1, True, 1.0, 0.0, 0.0),
        communication_lost=True)
    assert flow.state == MissionState.ABORT_RETURN
    assert flow.last_reason == 'start_transport_lost'
    assert not flow.payload_requested


def test_mission_timeout_is_single_transition_with_fresh_inputs():
    config = MissionConfig(
        mission_timeout_s=5.0, follow_timeout_s=20.0,
        release_min_return_time_s=1.0)
    flow = MissionFlow(config)
    flow.initialize(0.0)
    flow.start('mission', 1.0, (0.0, 0.0, 0.0))
    flow.transition(MissionState.FOLLOW_CAR, 2.0, 'test')
    flow.update(
        6.1, position=(1.0, 0.0, -1.5), px4_ok=True,
        communication_lost=False)
    assert flow.state == MissionState.ABORT_RETURN
    assert flow.last_reason == 'mission_timeout'
    entered = flow.state_since
    flow.update(
        6.2, position=(1.0, 0.0, -1.5), px4_ok=True,
        communication_lost=False)
    assert flow.state == MissionState.ABORT_RETURN
    assert flow.state_since == entered


def test_configs_keep_all_real_interfaces_disabled():
    root = Path(__file__).parents[1] / 'config'
    for name in (
            'mission_stage4c.yaml', 'mission_stage4c_sitl.yaml',
            'mission_stage4c_hardware_bench.yaml'):
        params = yaml.safe_load(
            (root / name).read_text())['/**']['ros__parameters']
        assert params['physical_release_enabled'] is False
        assert params['real_transport_enabled'] is False
        assert params['real_vision_enabled'] is False
    bench = yaml.safe_load(
        (root / 'mission_stage4c_hardware_bench.yaml').read_text()
    )['/**']['ros__parameters']
    assert bench['calibration_valid'] is False
    assert bench['enable_control'] is False


def test_frozen_search_speed_is_point_one_eight():
    root = Path(__file__).parents[1] / 'config'
    for name in (
            'mission_stage4c.yaml', 'mission_stage4c_sitl.yaml',
            'mission_stage4c_hardware_bench.yaml'):
        params = yaml.safe_load(
            (root / name).read_text())['/**']['ros__parameters']
        assert params['search_speed_mps'] == pytest.approx(0.18)
        assert params['search_max_speed_mps'] == pytest.approx(0.18)


def test_sitl_launch_exposes_safe_timeout_acceptance_override():
    launch = (
        Path(__file__).parents[1] /
        'launch/uav_mission_stage4c_sitl.launch.py').read_text()
    assert "'mission_timeout_s'" in launch
    assert "'follow_timeout_s'" in launch
    assert "name == 'mission_manager'" in launch


def test_return_uses_dedicated_bounded_velocity_control():
    source = (
        Path(__file__).parents[1] /
        'uav_control/mission_offboard_controller.py').read_text()
    assert "phase in ('RETURN_HOME', 'ABORT_RETURN')" in source
    assert 'self.return_velocity_limiter.update(' in source
    assert "'position_velocity'" in source
