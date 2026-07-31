"""Static contracts for the V2-only competition integration path."""

from pathlib import Path

import yaml


ROOT = Path(__file__).parents[1]
CONFIG = ROOT / 'config'


def test_competition_profiles_select_structured_v2_and_safe_gates():
    for name in ('competition_drop.yaml', 'competition_dynamic_land.yaml'):
        params = yaml.safe_load((CONFIG / name).read_text())[
            'mission_controller_node']['ros__parameters']
        assert params['vision_adapter_mode'] == 'v2_structured'
        assert params['enable_auto_arm'] is False
        assert params['enable_payload_release'] is False
        assert params['align_stable_duration_sec'] == 0.4


def test_mock_launch_has_no_px4_input_topic():
    launch = (ROOT / 'launch' / 'mock_vision_v2.launch.py').read_text()
    assert '/fmu/in/' not in launch


def test_formal_vision_launch_has_no_px4_input_topic():
    launch = (ROOT.parents[0] / 'uav_vision' / 'launch' /
              'h7_v2_readonly.launch.py').read_text()
    assert '/fmu/in/' not in launch
