"""ROS-independent trigger policy tests."""

from uav_control.car_start_gateway import parse_start_text_frame


def test_existing_text_adapter_remains_explicit_and_bounded():
    assert parse_start_text_frame('START,7')['sender_counter'] == 7


def test_formal_gateway_has_single_publisher_and_safety_mode():
    from pathlib import Path
    source = (Path(__file__).parents[1] / 'uav_control' /
              'car_start_gateway.py').read_text()
    assert "'/car/mission_start'" in source
    assert 'communication_only' in source
    assert 'triggered_run_ids' in source
    assert 'allow_state_transition_fallback' in source
    assert "'/car/run_id'" in source
