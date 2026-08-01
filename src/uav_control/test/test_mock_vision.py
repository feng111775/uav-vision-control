"""Static checks for the opt-in simulation-only vision publisher."""

from pathlib import Path


def test_mock_vision_is_explicitly_simulation_only():
    source = (Path(__file__).parents[1] / 'uav_control' /
              'mock_vision_node.py').read_text()
    assert 'requires simulation_mode=true' in source
    for scenario in (
            'no_target', 'acquire', 'follow_converge', 'aligned',
            'intermittent_loss', 'stale_timestamp', 'malformed',
            'hard_loss', 'health_down'):
        assert scenario in source
    assert '/fmu/in/' not in source
