"""Deterministic continuous alignment and release-gate tests."""

from pathlib import Path

import pytest

from uav_control.mission_logic import MissionLogic
from uav_control.mission_schema import MissionState
import yaml


def drop_logic(**kwargs):
    values = {
        'simulation_mode': True,
        'enable_control': True,
        'enable_payload_release': True,
        'enable_visual_follow': True,
        'visual_stable_seconds': 0.4,
        'point_b_progress': 2,
        'point_d_progress': 4,
        'progress_timeout_seconds': 60.0,
    }
    values.update(kwargs)
    logic = MissionLogic('drop', **values)
    logic.state = MissionState.ALIGN_FOR_DROP.value
    logic.h = (0.0, 0.0, 0.0, 0.0)
    logic.position = (0.0, 0.0, -logic.target_altitude)
    return logic


def step_aligned(logic, now, valid=True, aligned=True):
    logic.update_visual(valid, aligned, now)
    logic.step(now)


def test_039_seconds_does_not_release_but_040_does():
    logic = drop_logic()
    step_aligned(logic, 0.0)
    step_aligned(logic, 0.39)
    assert not logic.payload_sent
    step_aligned(logic, 0.4)
    assert logic.payload_sent
    assert logic.state == MissionState.PAYLOAD_RELEASE.value


@pytest.mark.parametrize('break_kind', ['x', 'y', 'invalid', 'loss', 'stale'])
def test_alignment_timer_resets_on_any_invalidating_observation(break_kind):
    logic = drop_logic()
    step_aligned(logic, 0.0)
    if break_kind in ('x', 'y'):
        step_aligned(logic, 0.2, aligned=False)
    else:
        step_aligned(logic, 0.2, valid=False)
    assert logic.drop_alignment_since is None
    step_aligned(logic, 0.3)
    assert not logic.payload_sent
    step_aligned(logic, 0.69)
    assert not logic.payload_sent
    step_aligned(logic, 0.7)
    assert logic.payload_sent


def test_single_alignment_sample_and_state_entry_do_not_release():
    logic = drop_logic()
    step_aligned(logic, 1.0)
    assert not logic.payload_sent
    logic.state = MissionState.VISION_FOLLOW.value
    logic.step(2.0)
    assert not logic.payload_sent


def test_leaving_alignment_state_clears_confirmation_timer():
    logic = drop_logic()
    step_aligned(logic, 0.0)
    assert logic.drop_alignment_since == 0.0
    logic.transition(MissionState.VISION_FOLLOW, 0.1)
    assert logic.drop_alignment_since is None


def test_release_disabled_fails_closed():
    logic = drop_logic(enable_payload_release=False)
    step_aligned(logic, 0.0)
    step_aligned(logic, 0.4)
    step_aligned(logic, 0.8)
    assert not logic.payload_sent
    assert logic.state == MissionState.FAILSAFE.value


def test_competition_and_drop_profiles_use_point_four_alignment_window():
    config_dir = Path(__file__).parents[1] / 'config'
    for name in ('competition_drop.yaml', 'competition_dynamic_land.yaml',
                 'sitl_drop.yaml', 'sitl_dynamic_land.yaml'):
        params = yaml.safe_load((config_dir / name).read_text())[
            'mission_controller_node']['ros__parameters']
        assert params['align_stable_duration_sec'] == 0.4
