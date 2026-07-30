"""Tests for the software-only stage 4C mission framework."""

from pathlib import Path

from uav_control.stage4c_core import (MarkerObservation, MissionConfig,
                                      MissionFlow, MissionState, PayloadGate,
                                      StartGate)
import yaml


def flow_at_takeoff(config=None):
    flow = MissionFlow(config)
    flow.initialize(0.0)
    assert flow.start('task-1', 1.0, (4.0, 5.0, 2.0))
    return flow


def marker(now, ex=0.0, ey=0.0):
    return MarkerObservation(now, True, 0.95, ex, ey)


def test_no_start_cannot_takeoff():
    flow = MissionFlow()
    flow.initialize(0.0)
    flow.update(50.0)
    assert flow.state == MissionState.WAIT_FOR_START


def test_same_task_id_is_accepted_once():
    gate = StartGate()
    assert gate.accept('42', True)
    assert not gate.accept('42', True)
    assert gate.accept('43', True)


def test_only_controller_of_five_may_own_px4_inputs():
    root = Path(__file__).parents[1] / 'uav_control'
    names = ('car_start_gateway.py', 'car_marker_vision.py',
             'payload_release.py', 'mission_manager.py')
    for name in names:
        assert '/fmu/in/' not in (root / name).read_text()
    controller = (root / 'mission_offboard_controller.py').read_text()
    assert 'MissionControllerNode' in controller


def test_height_outside_band_does_not_start_hover():
    flow = flow_at_takeoff()
    flow.update(2.0, (4.0, 5.0, 0.61))
    assert flow.state == MissionState.TAKEOFF
    assert flow.hover_since is None


def test_height_leaving_band_resets_three_second_timer():
    flow = flow_at_takeoff()
    flow.update(2.0, (4.0, 5.0, 0.5))
    assert flow.state == MissionState.HOVER_STABLE
    flow.update(4.0, (4.0, 5.0, 0.7))
    assert flow.state == MissionState.TAKEOFF
    assert flow.hover_since is None


def test_three_second_hover_advances():
    flow = flow_at_takeoff()
    flow.update(2.0, (4.0, 5.0, 0.5))
    flow.update(5.01, (4.0, 5.0, 0.5))
    assert flow.state == MissionState.TRANSIT_TO_INTERCEPT


def test_car_prediction_is_speed_times_elapsed():
    flow = flow_at_takeoff()
    assert flow.predicted_distance(11.0) == 1.0


def test_field_to_local_transform_is_parameterized():
    config = MissionConfig(
        field_yaw_rad=1.5707963267948966,
        field_offset_x_m=10.0,
        field_offset_y_m=-2.0)
    x, y = MissionFlow(config).field_to_local(1.0, 0.0)
    assert abs(x - 10.0) < 1e-9
    assert abs(y + 1.0) < 1e-9


def test_transit_target_is_relative_to_recorded_home():
    flow = flow_at_takeoff(MissionConfig(
        car_speed_mps=0.0, intercept_x_m=2.0, intercept_y_m=-1.0))
    flow.transition(MissionState.TRANSIT_TO_INTERCEPT, 2.0, 'test')
    assert flow.command(2.0)['target'] == (6.0, 4.0, 0.5)


def test_stale_vision_is_invalid():
    flow = flow_at_takeoff()
    assert not flow.target_valid(marker(1.0), 2.0)


def test_consecutive_frames_are_required_for_follow():
    config = MissionConfig(target_confirm_frames=3)
    flow = flow_at_takeoff(config)
    assert flow.enter_search(
        2.0, (4.0, 5.0, 0.5), 0.0, 'test')
    position = (4.0, 5.0, 0.5)
    flow.update(2.1, position, observation=marker(2.1))
    flow.update(2.2, position, observation=marker(2.2))
    assert flow.state == MissionState.ACQUIRE_CAR
    flow.update(2.3, position, observation=marker(2.3))
    assert flow.state == MissionState.FOLLOW_CAR


def test_short_target_loss_holds_follow():
    flow = flow_at_takeoff()
    flow.transition(MissionState.FOLLOW_CAR, 2.0, 'test')
    flow.follow_since = 2.0
    flow.update(2.1, observation=None)
    flow.update(2.5, observation=None)
    assert flow.state == MissionState.FOLLOW_CAR
    assert flow.command(2.5)['mode'] == 'HOLD'


def test_long_target_loss_returns_to_search():
    flow = flow_at_takeoff()
    flow.transition(MissionState.FOLLOW_CAR, 2.0, 'test')
    flow.follow_since = 2.0
    flow.update(2.1, observation=None)
    flow.update(4.2, observation=None, heading=0.0)
    assert flow.state == MissionState.SEARCH_CAR


def test_unstable_follow_cannot_request_payload():
    flow = flow_at_takeoff()
    flow.transition(MissionState.FOLLOW_CAR, 2.0, 'test')
    flow.follow_since = 2.0
    flow.update(2.5, observation=marker(2.5))
    assert not flow.payload_requested


def test_duplicate_payload_request_is_rejected():
    gate = PayloadGate()
    assert gate.request('task', 'release', True) == 'SUCCESS'
    assert gate.request('task', 'release', True) == 'REJECTED'
    assert gate.request('task', 'other', True) == 'REJECTED'


def test_failsafe_disallows_payload_and_forces_emergency_land():
    gate = PayloadGate()
    assert gate.request('task', 'release', False) == 'REJECTED'
    flow = flow_at_takeoff()
    flow.update(2.0, failsafe=True)
    assert flow.state == MissionState.EMERGENCY_LAND


def test_offboard_exit_uses_abort_return():
    flow = flow_at_takeoff()
    flow.transition(MissionState.FOLLOW_CAR, 2.0, 'test')
    flow.update(2.1, offboard_lost=True)
    assert flow.state == MissionState.ABORT_RETURN


def test_every_payload_terminal_result_returns_home():
    for result in ('SUCCESS', 'FAILED', 'REJECTED', 'TIMEOUT'):
        flow = flow_at_takeoff()
        flow.transition(MissionState.RELEASE_PAYLOAD, 2.0, 'test')
        flow.update(2.1, payload_result=result)
        assert flow.state == MissionState.RETURN_HOME


def test_return_command_uses_recorded_home():
    flow = flow_at_takeoff()
    flow.transition(MissionState.RETURN_HOME, 2.0, 'test')
    assert flow.command(2.0)['target'] == (4.0, 5.0, 0.5)


def test_land_requires_continuous_home_arrival():
    config = MissionConfig(home_stable_duration_s=1.0)
    flow = flow_at_takeoff(config)
    flow.transition(MissionState.RETURN_HOME, 2.0, 'test')
    flow.update(2.1, (4.0, 5.0, 0.5))
    flow.update(2.9, (4.0, 5.0, 0.5))
    assert flow.state == MissionState.RETURN_HOME
    flow.update(3.2, (4.0, 5.0, 0.5))
    assert flow.state == MissionState.LAND


def test_all_active_wait_states_have_timeout_or_global_exit():
    exempt = {
        MissionState.INITIALIZING,
        MissionState.WAIT_FOR_START,
        MissionState.HOVER_STABLE,
        MissionState.DROP_ALIGN,
        MissionState.COMPLETE,
        MissionState.EMERGENCY_LAND,
    }
    assert set(MissionState) - exempt <= set(MissionFlow.TIMEOUTS)


def test_complete_cannot_restart():
    flow = flow_at_takeoff()
    flow.transition(MissionState.COMPLETE, 2.0, 'test')
    assert not flow.start('task-2', 3.0, (0.0, 0.0, 0.0))
    flow.update(100.0)
    assert flow.state == MissionState.COMPLETE


def test_dry_run_never_accesses_gpio_or_pwm():
    gate = PayloadGate(dry_run=True)
    assert gate.request('task', 'release', True) == 'SUCCESS'
    assert gate.hardware_access_count == 0
    source = (Path(__file__).parents[1] / 'uav_control' /
              'payload_release.py').read_text().lower()
    assert 'import gpio' not in source
    assert 'import pigpio' not in source
    assert "'simulated_result'" in source


def test_simulated_complete_flow():
    config = MissionConfig(
        stable_hover_duration_s=0.1,
        target_confirm_frames=2,
        follow_stable_duration_s=0.1,
        home_stable_duration_s=0.1)
    flow = flow_at_takeoff(config)
    airborne = (4.0, 5.0, 0.5)
    flow.update(1.1, airborne)
    flow.update(1.21, airborne)
    flow.update(1.22, airborne, intercept_reached=True, heading=0.0)
    flow.update(1.23, airborne, observation=marker(1.23))
    flow.update(1.24, airborne, observation=marker(1.24))
    flow.update(1.35, airborne, observation=marker(1.35))
    flow.update(1.46, airborne, observation=marker(1.46))
    flow.update(1.47, airborne, observation=marker(1.47))
    assert flow.state == MissionState.RELEASE_PAYLOAD
    flow.update(1.48, airborne, payload_result='SUCCESS')
    flow.update(1.49, airborne)
    flow.update(1.60, airborne)
    assert flow.state == MissionState.LAND
    flow.update(1.61, (4.0, 5.0, 2.0), landed=True)
    assert flow.state == MissionState.COMPLETE


def test_launch_starts_exactly_five_nodes_and_safe_controller():
    launch = (Path(__file__).parents[1] / 'launch' /
              'uav_mission_stage4c_sim.launch.py').read_text()
    for name in ('car_start_gateway', 'car_marker_vision',
                 'mission_offboard_controller', 'payload_release',
                 'mission_manager'):
        assert "'%s'" % name in launch
    assert "'enable_control': False" in launch
    assert "'enable_auto_arm': False" in launch


def test_stage4c_topics_use_single_namespace():
    root = Path(__file__).parents[1] / 'uav_control'
    for name in ('car_start_gateway.py', 'car_marker_vision.py',
                 'mission_offboard_controller.py', 'payload_release.py',
                 'mission_manager.py'):
        text = (root / name).read_text()
        for line in text.splitlines():
            if "'/" in line and "'/fmu/out/" not in line:
                assert "'/uav_mission/" in line


def test_default_and_sitl_control_are_safely_isolated():
    root = Path(__file__).parents[1] / 'config'
    normal = yaml.safe_load(
        (root / 'mission_stage4c.yaml').read_text())['/**']['ros__parameters']
    sitl_doc = yaml.safe_load(
        (root / 'mission_stage4c_sitl.yaml').read_text())
    sitl = sitl_doc['mission_offboard_controller']['ros__parameters']
    assert normal['enable_control'] is False
    assert normal['enable_auto_arm'] is False
    assert sitl['enable_control'] is True
    assert sitl['enable_auto_arm'] is True
    assert sitl_doc['/**']['ros__parameters']['use_simulated_px4'] is False
    assert sitl_doc['/**']['ros__parameters']['dry_run'] is True


def test_sitl_launch_has_no_fake_px4_source_or_old_controller():
    launch = (Path(__file__).parents[1] / 'launch' /
              'uav_mission_stage4c_sitl.launch.py').read_text()
    assert 'd_task_mock_node' not in launch
    assert 'sim/position' not in launch
    assert 'sim/px4_status' not in launch
    assert "'mission_controller_node'" not in launch
    assert launch.count("'mission_offboard_controller'") == 1


def test_manager_records_real_start_position_and_px4_outputs():
    source = (Path(__file__).parents[1] / 'uav_control' /
              'mission_manager.py').read_text()
    assert "'/fmu/out/vehicle_local_position'" in source
    assert "'/fmu/out/vehicle_status_v1'" in source
    assert "'/fmu/out/vehicle_land_detected'" in source
    assert 'self.flow.start(' in source
    assert 'self.position)' in source


def test_ned_takeoff_target_is_above_actual_home():
    flow = flow_at_takeoff()
    assert flow.command(1.0)['target'][2] == 0.5
    assert flow.command(1.0)['target'][2] < flow.home[2]


def test_controller_prestream_precedes_mode_and_arm():
    source = (Path(__file__).parents[1] / 'uav_control' /
              'mission_offboard_controller.py').read_text()
    prestream = source.index(
        'self.stage4c_prestream_cycles < self.logic.prestream_cycles')
    mode = source.index("self._publish_command(\n                'mode'")
    arm = source.index("self._publish_command(\n                'arm'")
    assert prestream < mode < arm
    assert "phase == 'TAKEOFF' and self.logic.offboard" in source


def test_stale_visual_command_becomes_hold_but_control_continues():
    flow = flow_at_takeoff()
    flow.transition(MissionState.FOLLOW_CAR, 2.0, 'test')
    flow.follow_since = 2.0
    flow.update(2.1, observation=marker(2.1, 0.2, 0.0))
    assert flow.command(2.1)['mode'] == 'VISION'
    flow.update(2.7, observation=None)
    assert flow.state == MissionState.FOLLOW_CAR
    assert flow.command(2.7)['mode'] == 'HOLD'


def test_visual_guidance_limit_remains_configured():
    source = (Path(__file__).parents[1] / 'uav_control' /
              'mission_offboard_controller.py').read_text()
    assert 'self.guidance.velocity(' in source
    config = yaml.safe_load(
        (Path(__file__).parents[1] / 'config' /
         'mission_stage4c_sitl.yaml').read_text())
    assert config['/**']['ros__parameters']['max_horizontal_speed'] == 0.4
    assert config['/**']['ros__parameters']['max_vertical_speed'] == 0.12


def test_position_target_is_rate_limited_on_all_axes():
    source = (Path(__file__).parents[1] / 'uav_control' /
              'mission_offboard_controller.py').read_text()
    assert 'self._bounded_position_target(target)' in source
    assert "get_parameter('max_horizontal_speed')" in source
    assert "get_parameter('max_vertical_speed')" in source
    assert "get_parameter('control_rate_hz')" in source
    assert 'self.stage4c_position_setpoint = (' in source


def test_nav_land_only_for_land_or_safety_path():
    source = (Path(__file__).parents[1] / 'uav_control' /
              'mission_offboard_controller.py').read_text()
    assert "if mode == 'LAND':" in source
    assert source.count('VehicleCommand.VEHICLE_CMD_NAV_LAND') == 2


def test_emergency_land_can_complete_after_real_landed():
    flow = flow_at_takeoff()
    flow.transition(MissionState.EMERGENCY_LAND, 2.0, 'test')
    flow.update(2.1, landed=True)
    assert flow.state == MissionState.COMPLETE


def test_acceptance_driver_never_publishes_fake_px4_state():
    source = (Path(__file__).parents[1] / 'uav_control' /
              'sitl_acceptance_driver.py').read_text()
    assert "create_publisher(\n            String, '/uav_mission/sim/car_start'" in source
    assert "create_publisher(\n            Float32MultiArray, '/uav_mission/sim/marker'" in source
    assert "'/uav_mission/sim/position'" not in source
    assert "'/uav_mission/sim/px4_status'" not in source
