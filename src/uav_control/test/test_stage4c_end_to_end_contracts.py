"""Stage 4C-3C transport, follow and release contract tests."""

import json
import math
from pathlib import Path

import pytest

from uav_control.payload_release import (
    DryRunReleaseActuator, PhysicalReleaseActuator)
from uav_control.stage4c_core import (
    MarkerObservation, MissionConfig, MissionFlow, MissionState, StartGate,
    StartProtocol)
from uav_control.visual_guidance import VisualGuidance
import yaml


def follow_flow(config=None):
    flow = MissionFlow(config)
    flow.initialize(0.0)
    flow.start('session:start', 1.0, (1.0, 2.0, 3.0))
    flow.transition(MissionState.FOLLOW_CAR, 2.0, 'test')
    flow.follow_since = 2.0
    return flow


def packet(now_ns=1_000_000_000, **overrides):
    values = StartProtocol.build(
        'session-a', 'start-1', 1, now_ns, task_number='D')
    values.update(overrides)
    if 'checksum' not in overrides:
        fields = {key: value for key, value in values.items()
                  if key != 'checksum'}
        values['checksum'] = StartProtocol.checksum(fields)
    return values


def test_valid_start_packet_parses_once_and_persists(tmp_path):
    parsed = StartProtocol.parse(packet(), 1_100_000_000, 2.0)
    gate = StartGate(str(tmp_path / 'starts.json'))
    assert gate.accept_protocol(
        parsed['session_id'], parsed['start_id'], True, True, False,
        parsed['sender_counter']) == 'START_ACCEPTED'
    restarted = StartGate(str(tmp_path / 'starts.json'))
    assert restarted.accept_protocol(
        'session-a', 'start-1', True, True, False, 1
    ) == 'ALREADY_PROCESSED'


@pytest.mark.parametrize('change', [
    {'protocol_version': 2},
    {'checksum': '0' * 64},
    {'sender_timestamp_ns': 1},
    {'sender_timestamp_ns': 2_000_000_000},
    {'command': 'ARM'},
])
def test_invalid_version_checksum_age_future_or_command_is_rejected(change):
    with pytest.raises(ValueError):
        StartProtocol.parse(packet(**change), 1_100_000_000, 0.5)


def test_out_of_order_counter_and_busy_are_rejected():
    gate = StartGate()
    assert gate.accept_protocol(
        'session', 'one', True, True, False, 5) == 'START_ACCEPTED'
    assert gate.accept_protocol(
        'session', 'two', True, True, False, 4) == 'START_REJECTED'
    assert gate.accept_protocol(
        'other', 'one', True, True, True, 1) == 'BUSY'


def test_simulation_driver_uses_formal_start_protocol():
    root = Path(__file__).parents[1] / 'uav_control'
    gateway = (root / 'car_start_gateway.py').read_text()
    driver = (root / 'sitl_acceptance_driver.py').read_text()
    assert 'StartProtocol.parse(' in gateway
    assert 'StartProtocol.build(' in driver


def test_start_gateway_requires_px4_aware_manager_readiness():
    root = Path(__file__).parents[1] / 'uav_control'
    gateway = (root / 'car_start_gateway.py').read_text()
    manager = (root / 'mission_manager.py').read_text()
    assert "'/uav_mission/readiness'" in gateway
    assert "'/uav_mission/readiness'" in manager
    assert 'self.px4_ok and not self.failsafe' in manager
    assert 'self.heading is not None' in manager


@pytest.mark.parametrize('heading,expected', [
    (0.0, (0.0, -0.15)),
    (math.pi / 2, (0.15, 0.0)),
    (math.pi, (0.0, 0.15)),
    (-math.pi / 2, (-0.15, 0.0)),
])
def test_camera_error_transform_to_ned_cardinal_headings(heading, expected):
    guide = VisualGuidance(
        kp_left=0.3, kp_forward=0.3, camera_x_sign=1.0,
        camera_y_sign=-1.0, max_speed=0.2)
    assert guide.velocity(
        True, 0.5, 0.0, 90.0, 0.0, heading
    ) == pytest.approx(expected, abs=1e-8)


def test_camera_swap_scale_sign_and_mount_rotation_are_parameterized():
    guide = VisualGuidance(
        kp_left=1.0, kp_forward=1.0, deadband_x=0.0,
        deadband_y=0.0, camera_x_sign=-1.0, camera_y_sign=1.0,
        swap_axes=True, error_scale=2.0,
        camera_mount_yaw_rad=math.pi / 2, max_speed=1.0)
    output = guide.velocity(True, 0.4, 0.2, 90.0, 0.0, 0.0)
    assert all(math.isfinite(value) for value in output)
    assert math.hypot(*output) == pytest.approx(math.hypot(0.1, 0.2))


def test_follow_stability_is_continuous_and_resets_outside_window():
    config = MissionConfig(
        follow_stable_duration_s=1.0, drop_alignment_tolerance=0.08)
    flow = follow_flow(config)
    centered = MarkerObservation(2.1, True, .9, 0.0, 0.0)
    flow.update(2.1, observation=centered)
    flow.update(
        2.9, observation=MarkerObservation(2.9, True, .9, .2, 0.0))
    assert flow.follow_stable_since is None
    flow.update(
        3.0, observation=MarkerObservation(3.0, True, .9, 0.0, 0.0))
    flow.update(
        4.01, observation=MarkerObservation(4.01, True, .9, 0.0, 0.0))
    assert flow.state == MissionState.DROP_ALIGN


def test_short_loss_outputs_height_holding_brake_and_never_release():
    flow = follow_flow()
    flow.update(2.1, observation=None)
    command = flow.command(2.1)
    assert command['mode'] == 'HOLD'
    assert command['target_z'] == pytest.approx(1.5)
    assert not flow.payload_requested


def test_dry_run_actuator_is_explicitly_nonphysical():
    actuator = DryRunReleaseActuator()
    assert actuator.execute('SUCCESS') == ('DRY_RUN_CONFIRMED', False)
    assert actuator.actuator_type == 'DRY_RUN'
    with pytest.raises(RuntimeError):
        PhysicalReleaseActuator()


def test_dry_run_terminal_result_returns_home():
    flow = follow_flow()
    flow.transition(MissionState.RELEASE_PAYLOAD, 3.0, 'test')
    flow.update(3.1, payload_result='DRY_RUN_CONFIRMED')
    assert flow.state == MissionState.RETURN_HOME


def test_release_gate_requires_height_speed_and_safe_return_time():
    flow = follow_flow()
    flow.transition(MissionState.DROP_ALIGN, 70.0, 'test')
    centered = MarkerObservation(70.1, True, .9, 0.0, 0.0)
    flow.update(70.1, position=(1.0, 2.0, 1.5), observation=centered)
    assert flow.state == MissionState.DROP_ALIGN
    assert not flow.payload_requested

    flow.transition(MissionState.DROP_ALIGN, 3.0, 'test')
    flow.update(3.1, position=(1.0, 2.0, 2.0), observation=MarkerObservation(
        3.1, True, .9, 0.0, 0.0))
    assert flow.state == MissionState.DROP_ALIGN
    assert not flow.payload_requested


def test_all_stage4c_configs_keep_release_disabled_and_follow_bounded():
    root = Path(__file__).parents[1] / 'config'
    for name in (
            'mission_stage4c.yaml', 'mission_stage4c_sitl.yaml',
            'mission_stage4c_hardware_bench.yaml'):
        params = yaml.safe_load(
            (root / name).read_text())['/**']['ros__parameters']
        assert params['physical_release_enabled'] is False
        assert params['follow_max_speed_mps'] <= 0.2
        assert params['follow_error_scale'] > 0.0


def test_complete_and_land_commands_are_mutually_exclusive():
    flow = follow_flow()
    flow.transition(MissionState.LAND, 3.0, 'test')
    assert flow.command(3.0)['mode'] == 'LAND'
    flow.update(3.1, landed=True)
    assert flow.state == MissionState.COMPLETE
    assert flow.command(3.1)['mode'] == 'HOLD'


def test_global_timeout_does_not_restart_abort_return_timeout():
    config = MissionConfig(
        mission_timeout_s=10.0, return_timeout_s=5.0,
        release_min_return_time_s=1.0)
    flow = follow_flow(config)
    flow.transition(MissionState.ABORT_RETURN, 11.0, 'mission_timeout')

    flow.update(14.0, position=(10.0, 10.0, 1.5))
    assert flow.state == MissionState.ABORT_RETURN
    assert flow.state_since == 11.0

    flow.update(16.1, position=(10.0, 10.0, 1.5))
    assert flow.state == MissionState.EMERGENCY_LAND
    assert flow.last_reason == 'state_timeout'


def test_protocol_packet_is_json_transport_neutral():
    encoded = json.dumps(packet())
    decoded = json.loads(encoded)
    assert StartProtocol.parse(decoded, 1_100_000_000, 2.0)[
        'command'] == 'CAR_START'
