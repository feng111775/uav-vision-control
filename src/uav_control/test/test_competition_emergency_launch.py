from pathlib import Path

import yaml


ROOT = Path(__file__).parents[1]


def test_competition_launch_uses_single_safe_chain():
    launch = (ROOT / 'launch' / 'competition_emergency_final.launch.py').read_text()
    assert "executable='car_start_gateway'" in launch
    assert "executable='mission_controller_node'" in launch
    assert "executable='servo_node'" in launch
    assert "executable='pi_camera_vision_node'" in launch
    assert "executable='target_filter_node'" in launch
    assert "executable='target_predictor_node'" in launch
    assert "executable='landing_error_node'" in launch
    assert 'gazebo' not in launch.lower()
    assert "'dry_run': True" in launch


def test_emergency_profile_is_fail_closed_and_search_bounded():
    path = ROOT / 'config' / 'competition_emergency_final.yaml'
    params = yaml.safe_load(path.read_text())['mission_controller_node']['ros__parameters']
    assert params['enable_control'] is False
    assert params['enable_auto_arm'] is False
    assert params['enable_payload_release'] is False
    assert params['enable_dynamic_landing'] is False
    assert params['enable_second_takeoff'] is False
    assert params['target_altitude'] == 1.5
    assert params['hover_confirm_seconds'] == 3.0
    assert params['search_speed_mps'] == 0.18
    assert params['search_max_radius_m'] == 1.0


def test_servo_ack_contract_is_exact_and_fail_closed():
    controller = (ROOT / 'uav_control' / 'mission_controller_node.py').read_text()
    servo = (ROOT.parent / 'servo_control' / 'servo_control' /
             'servo_node.py').read_text()
    assert "command.data = 'throw'" in controller
    assert "result == 'SUCCESS:throw'" in controller
    assert 'result_callback("SUCCESS:throw")' in servo
    assert 'result_callback(f"FAILED:throw:{reason}")' in servo
