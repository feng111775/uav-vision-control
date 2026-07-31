"""Pure tests for fail-closed mission parameter combinations."""

import pytest

from uav_control.safety_config import validate_safety_parameters


def test_default_start_policy_is_explicitly_disabled():
    assert validate_safety_parameters({
        'simulation_mode': False,
        'auto_start_hover_test': False,
        'enable_auto_arm': False,
        'allow_sitl_heading_quality_bypass': False,
    }) is True


@pytest.mark.parametrize('name', [
    'auto_start_hover_test', 'enable_auto_arm',
    'allow_sitl_heading_quality_bypass'])
def test_real_mode_rejects_sitl_only_options(name):
    with pytest.raises(ValueError):
        validate_safety_parameters({'simulation_mode': False, name: True})


def test_communication_only_rejects_automatic_start():
    with pytest.raises(ValueError):
        validate_safety_parameters({
            'simulation_mode': True,
            'communication_only': True,
            'auto_start_hover_test': True,
        })
