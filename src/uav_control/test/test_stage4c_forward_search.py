"""Tests for the bounded, heading-locked stage 4C car search."""

import math
from pathlib import Path

import pytest

from uav_control.stage4c_core import (
    HorizontalVelocityLimiter, MarkerObservation, MissionConfig, MissionFlow,
    MissionState)
import yaml


def searching(config=None, heading=0.0):
    flow = MissionFlow(config)
    flow.initialize(0.0)
    flow.start('session:start', 1.0, (4.0, 5.0, 2.0))
    assert flow.enter_search(
        2.0, (4.0, 5.0, 0.5), heading, 'test_search')
    return flow


@pytest.mark.parametrize('heading,expected', [
    (0.0, (0.18, 0.0)),
    (math.pi / 2, (0.0, 0.18)),
    (math.pi, (-0.18, 0.0)),
    (-math.pi / 2, (0.0, -0.18)),
])
def test_body_forward_converts_to_local_ned(heading, expected):
    velocity = searching(heading=heading).search_velocity()
    assert velocity == pytest.approx(expected, abs=1e-9)


@pytest.mark.parametrize(
    'heading', [index * math.pi / 17 for index in range(-34, 35)])
def test_search_horizontal_speed_never_exceeds_point_two(heading):
    assert math.hypot(
        *searching(heading=heading).search_velocity()) <= 0.18 + 1e-12


def test_search_command_holds_relative_home_cruise_height():
    command = searching(heading=0.4).command(2.1)
    assert command['mode'] == 'SEARCH'
    assert command['target_z'] == pytest.approx(0.5)
    assert command['target_z'] == pytest.approx(2.0 - 1.5)


def test_invalid_heading_cannot_start_search():
    for heading in (None, math.nan, math.inf):
        flow = MissionFlow()
        flow.initialize(0.0)
        flow.start('task', 1.0, (0.0, 0.0, 0.0))
        assert not flow.enter_search(
            2.0, (0.0, 0.0, -1.5), heading, 'test')


@pytest.mark.parametrize('speed', [-0.1, math.nan, math.inf, 0.18001])
def test_unsafe_search_speed_is_rejected(speed):
    with pytest.raises(ValueError):
        MissionConfig(search_speed_mps=speed)


def test_velocity_ramps_from_hover_and_remains_bounded():
    limiter = HorizontalVelocityLimiter(0.1, 0.2)
    first = limiter.update((0.2, 0.0), 0.05)
    assert first == pytest.approx((0.005, 0.0))
    for _ in range(100):
        value = limiter.update((0.2, 0.0), 0.05)
        assert math.hypot(*value) <= 0.2 + 1e-12
    assert value == pytest.approx((0.2, 0.0))


def test_velocity_smoothly_leaves_search_for_follow():
    limiter = HorizontalVelocityLimiter(0.1, 0.2)
    for _ in range(40):
        limiter.update((0.2, 0.0), 0.05)
    first_follow = limiter.update((0.0, 0.0), 0.05)
    assert first_follow == pytest.approx((0.195, 0.0))


def test_one_frame_never_enters_follow_and_loss_resumes_search():
    flow = searching()
    observation = MarkerObservation(2.1, True, 0.9, 0.0, 0.0)
    flow.update(2.1, (4.0, 5.0, 0.5), observation=observation, heading=0.0)
    assert flow.state == MissionState.ACQUIRE_CAR
    assert flow.command(2.1)['mode'] == 'SEARCH'
    flow.update(2.2, (4.01, 5.0, 0.5), observation=None, heading=0.0)
    assert flow.state == MissionState.SEARCH_CAR
    assert flow.command(2.2)['mode'] == 'SEARCH'


def test_three_fresh_frames_switch_once_to_follow():
    flow = searching(MissionConfig(target_confirm_frames=3))
    for index in range(3):
        now = 2.1 + index * 0.1
        flow.update(
            now, (4.0, 5.0, 0.5),
            observation=MarkerObservation(now, True, 0.9, 0.0, 0.0),
            heading=0.0)
    assert flow.state == MissionState.FOLLOW_CAR
    assert flow.command(2.31)['mode'] == 'VISION'
    flow.update(
        2.4, (4.0, 5.0, 0.5),
        observation=MarkerObservation(2.4, True, 0.9, 0.0, 0.0),
        heading=0.0)
    assert flow.state == MissionState.FOLLOW_CAR


@pytest.mark.parametrize('observation', [
    MarkerObservation(1.0, True, 0.9, 0.0, 0.0),
    MarkerObservation(2.1, True, 0.59, 0.0, 0.0),
    MarkerObservation(2.1, True, 0.9, math.nan, 0.0),
])
def test_stale_low_confidence_or_nan_detection_is_rejected(observation):
    flow = searching()
    flow.update(
        2.1, (4.0, 5.0, 0.5), observation=observation, heading=0.0)
    assert flow.state == MissionState.SEARCH_CAR


def test_search_timeout_uses_existing_abort_return_path():
    flow = searching(MissionConfig(search_timeout_s=3.0))
    flow.update(5.0, (4.5, 5.0, 0.5), heading=0.0)
    assert flow.state == MissionState.ABORT_RETURN
    assert flow.last_reason == 'search_timeout'


def test_search_distance_limit_uses_existing_abort_return_path():
    flow = searching(MissionConfig(search_max_distance_m=1.0))
    flow.update(3.0, (5.0, 5.0, 0.5), heading=0.0)
    assert flow.state == MissionState.ABORT_RETURN
    assert flow.last_reason == 'search_distance_exceeded'


def test_search_configs_use_point_two_and_controller_reads_it():
    root = Path(__file__).parents[1]
    for name in (
            'mission_stage4c.yaml', 'mission_stage4c_sitl.yaml',
            'mission_stage4c_hardware_bench.yaml'):
        document = yaml.safe_load((root / 'config' / name).read_text())
        assert document['/**']['ros__parameters']['search_speed_mps'] == 0.18
    controller = (
        root / 'uav_control/mission_offboard_controller.py').read_text()
    assert "declare_parameter('search_speed_mps', 0.18)" in controller
    assert "get_parameter('search_speed_mps')" in controller
    core = (root / 'uav_control/stage4c_core.py').read_text()
    assert 'speed = self.config.search_speed_mps' in core


def test_real_intercept_waits_until_horizontal_speed_is_search_safe():
    source = (
        Path(__file__).parents[1] /
        'uav_control/mission_manager.py').read_text()
    assert 'math.hypot(self.velocity[0], self.velocity[1]) <=' in source
    assert 'self.flow.config.search_max_speed_mps' in source


def test_hardware_bench_has_no_px4_control_node():
    root = Path(__file__).parents[1]
    launch = (
        root /
        'launch/uav_mission_stage4c_hardware_bench.launch.py').read_text()
    assert "'mission_offboard_controller'" not in launch
    assert "'mission_controller_node'" not in launch
    assert 'd_task_mock_node' not in launch


def test_sitl_launch_has_exactly_one_stage4c_control_node():
    root = Path(__file__).parents[1]
    launch = (
        root / 'launch/uav_mission_stage4c_sitl.launch.py').read_text()
    assert launch.count("'mission_offboard_controller'") == 1
    assert "'mission_controller_node'" not in launch
