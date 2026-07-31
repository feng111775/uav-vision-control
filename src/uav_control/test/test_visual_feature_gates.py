"""Tests for the visual-follow boolean gate without ROS or PX4."""

from uav_control.mission_logic import MissionLogic


def test_visual_follow_disabled_does_not_enter_visual_states():
    logic = MissionLogic(
        mission_mode='drop', enable_visual_follow=False,
        point_b_progress=2, point_d_progress=4)
    logic.state = 'SEARCH_CAR'
    logic.update_visual(True, True, 1.0)
    logic.step(1.0)
    assert logic.state == 'SEARCH_CAR'
    assert not logic.target_ok


def test_visual_follow_disabled_recovers_if_state_is_stale():
    logic = MissionLogic(
        mission_mode='drop', enable_visual_follow=False,
        point_b_progress=2, point_d_progress=4)
    logic.state = 'VISION_FOLLOW'
    logic.update_visual(True, True, 1.0)
    logic.step(1.0)
    assert logic.state == 'SEARCH_CAR'


def test_abort_parameter_is_the_single_runtime_parameter():
    from pathlib import Path
    source = (Path(__file__).parents[1] / 'uav_control' /
              'mission_controller_node.py').read_text()
    assert "'target_loss_abort_seconds').value" in source
    assert 'vision_loss_abort_sec is deprecated' in source
