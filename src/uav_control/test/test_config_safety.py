"""Static safety checks for supplied mission profiles."""

from pathlib import Path

import yaml


CONFIG = Path(__file__).parents[1] / 'config'


def mission_params(name):
    return yaml.safe_load((CONFIG / name).read_text())[
        'mission_controller_node']['ros__parameters']


def test_competition_profiles_have_explicit_fail_safe_values():
    for name in ('competition_drop.yaml', 'competition_dynamic_land.yaml',
                 'first_flight_hover.yaml'):
        params = mission_params(name)
        assert params['simulation_mode'] is False
        assert params['enable_auto_arm'] is False
        assert params['enable_auto_disarm'] is False
        assert params['auto_start_hover_test'] is False
        assert params['allow_sitl_heading_quality_bypass'] is False
        assert params['communication_only'] is False
        assert params['enable_payload_release'] is False
        assert params['vision_adapter_mode'] == 'legacy_array'
        if name != 'first_flight_hover.yaml':
            assert params['align_stable_duration_sec'] == 0.4


def test_nonvisual_profile_requires_real_start():
    params = mission_params('sitl_hover_nonvisual.yaml')
    assert params['simulation_mode'] is True
    assert params['auto_start_hover_test'] is False
    assert params['enable_visual_follow'] is False
    assert params['target_altitude'] == 1.5
    assert params['hover_test_seconds'] == 3.0


def test_only_explicit_hover_profiles_allow_automatic_start():
    for name in ('sitl_hover_low_short.yaml', 'sitl_hover_nominal.yaml',
                 'sitl_hover_high_long.yaml'):
        params = mission_params(name)
        assert params['simulation_mode'] is True
        assert params['auto_start_hover_test'] is True
        assert params['enable_auto_arm'] is True
