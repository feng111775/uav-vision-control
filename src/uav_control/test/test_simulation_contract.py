from pathlib import Path

import pytest

from uav_control.stage_catalog import STAGES, validate_stage_target
import yaml


PACKAGE_ROOT = Path(__file__).parents[1]


@pytest.mark.parametrize(
    'scenario,height,duration',
    [('low_short', 0.5, 5.0), ('nominal', 1.0, 10.0),
     ('high_long', 1.5, 15.0)])
def test_hover_scenarios_are_explicitly_sitl_only(
        scenario, height, duration):
    path = PACKAGE_ROOT / 'config' / ('sitl_hover_' + scenario + '.yaml')
    params = yaml.safe_load(path.read_text())[
        'mission_controller_node']['ros__parameters']
    assert params['simulation_mode'] is True
    assert params['allow_sitl_heading_quality_bypass'] is True
    assert params['enable_control'] is True
    assert params['enable_auto_arm'] is True
    assert params['target_altitude'] == height
    assert params['hover_test_seconds'] == duration


def test_first_flight_hardware_profile_was_not_weakened():
    path = PACKAGE_ROOT / 'config'
    params = yaml.safe_load(
        (path / 'first_flight_hover.yaml').read_text())[
            'mission_controller_node']['ros__parameters']
    assert params['simulation_mode'] is False
    assert params['allow_sitl_heading_quality_bypass'] is False
    assert params['enable_auto_arm'] is False


def test_every_real_controller_state_has_a_stage_contract():
    schema = PACKAGE_ROOT / 'uav_control'
    text = (schema / 'mission_schema.py').read_text()
    for state in STAGES:
        assert ("= '" + state + "'") in text
    assert len(STAGES) == 30


@pytest.mark.parametrize('stage', sorted(STAGES))
def test_each_stage_contract_is_complete(stage):
    spec = STAGES[stage]
    assert validate_stage_target(stage) == stage
    assert all((spec.entry, spec.action, spec.success,
                spec.timeout, spec.failure))


def test_unknown_checkpoint_is_rejected():
    with pytest.raises(ValueError):
        validate_stage_target('SKIP_TO_ARM')


def test_stage_launch_uses_full_controller_and_explicit_sitl_bypass():
    text = (
        PACKAGE_ROOT / 'launch' /
        'sitl_d_task_stage.launch.py').read_text()
    assert "executable='mission_controller_node'" in text
    assert "'simulation_mode': True" in text
    assert "'allow_sitl_heading_quality_bypass': True" in text
    assert 'logic.state =' not in text
