from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_competition_entry_uses_openmv_v2_and_readiness_gate():
    text = (ROOT / 'launch' / 'competition_first_task.launch.py').read_text()
    assert 'openmv_v2_competition.yaml' in text
    assert "executable='readiness_gate'" in text
    assert "executable='h7_bridge_node'" in text
    assert "executable='vision_interface_node'" in text
    for legacy in ('pi_camera_vision_node', 'camera_selector_node',
                   'target_filter_node', 'target_predictor_node',
                   'landing_error_node'):
        assert legacy not in text


def test_profiles_are_fail_closed_except_explicit_real_profile():
    text = (ROOT / 'launch' / 'competition_first_task.launch.py').read_text()
    assert "profile != 'readonly_bench'" in text
    assert "profile == 'real_competition'" in text
    assert "'dry_run': profile != 'real_competition'" in text
    assert "'enable_auto_arm': enable_auto_arm" in text
