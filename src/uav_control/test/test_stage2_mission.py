import math
from pathlib import Path

import pytest

from uav_control.mission_logic import (
    control_output_allowed, data_loss_action, descent_speed_for_state,
    MissionLogic,
    payload_ack_is_new, prestream_is_safe, quaternion_is_valid,
    vehicle_command_allowed, vision_is_fresh)
from uav_control.mission_schema import (
    CarProgress, MissionState, parse_mission_mode, STATE_ID,
    TELEMETRY_LENGTH)
from uav_control.px4_command_tracker import CommandTracker
from uav_control.touchdown_detector import TouchdownDetector
from uav_control.vision_contract import (
    validate_landing_error, validate_tracked)
from uav_control.visual_guidance import VisualGuidance
import yaml


S = MissionState
PACKAGE_ROOT = Path(__file__).parents[1]


@pytest.mark.parametrize('mode', ['drop', 'dynamic_land', 'hover_test'])
def test_three_modes_share_schema(mode):
    assert parse_mission_mode(mode)


def ready_logic(mode='drop', **kwargs):
    logic = MissionLogic(
        mode, simulation_mode=True, enable_control=True,
        enable_auto_arm=mode != 'hover_test', prestream_cycles=40,
        stable_seconds=0.1, visual_stable_seconds=0.2, **kwargs)
    logic.update_position(2, 3, 4, 0, 0, 0, 0.2, True)
    logic.odom_ready = True
    logic.attitude_valid = True
    logic.update_status(False, 0, False, False, 0)
    logic.step(0)
    logic.step(0.01)
    logic.start_signal = True
    logic.step(0.02)
    assert logic.state == S.PRESTREAM.value
    return logic


def enter_takeoff(logic, now=1.0):
    for _ in range(logic.prestream_cycles):
        logic.record_prestream_cycle()
    logic.step(now)
    assert logic.state == S.REQUEST_OFFBOARD.value
    logic.update_status(False, 14, True, False, now)
    logic.step(now + 0.1)
    expected = (S.WAIT_MANUAL_ARM.value if logic.mode_name == 'hover_test'
                else S.ARMING.value)
    assert logic.state == expected
    logic.update_status(True, 14, True, False, now + 0.2)
    logic.step(now + 0.2)
    assert logic.state == S.TAKEOFF.value


def request_offboard_logic(mode, simulation_mode, enable_auto_arm,
                           preflight_ok=True):
    logic = MissionLogic(
        mode, simulation_mode=simulation_mode, competition_mode=True,
        enable_control=True, enable_auto_arm=enable_auto_arm)
    logic.state = S.REQUEST_OFFBOARD.value
    logic.start_signal = True
    logic.px4_fresh = True
    logic.h = (0.0, 0.0, 0.0, 0.0)
    logic.safety_ready = True
    logic.attitude_valid = True
    logic.update_status(False, 0, False, False, 0, preflight_ok)
    logic.offboard = True
    return logic


def test_sitl_hover_test_with_auto_arm_enters_arming():
    logic = request_offboard_logic('hover_test', True, True)
    logic.step(1.0)
    assert logic.state == S.ARMING.value


def test_hardware_hover_test_with_auto_arm_requires_manual_arm():
    logic = request_offboard_logic('hover_test', False, True)
    assert logic.auto_arm_allowed()
    logic.step(1.0)
    assert logic.state == S.WAIT_MANUAL_ARM.value


def test_sitl_hover_test_without_auto_arm_requires_manual_arm():
    logic = request_offboard_logic('hover_test', True, False)
    logic.step(1.0)
    assert logic.state == S.WAIT_MANUAL_ARM.value


@pytest.mark.parametrize('mode', ['drop', 'dynamic_land'])
def test_non_hover_auto_arm_behavior_is_unchanged(mode):
    logic = request_offboard_logic(mode, False, True)
    logic.step(1.0)
    assert logic.state == S.ARMING.value


def test_auto_arm_is_blocked_until_preflight_checks_pass():
    logic = request_offboard_logic('drop', False, True, preflight_ok=False)
    assert not logic.auto_arm_allowed()
    logic.step(1.0)
    assert logic.state == S.WAIT_MANUAL_ARM.value


def test_auto_arm_enters_arming_once_preflight_passes():
    logic = request_offboard_logic('drop', False, True, preflight_ok=True)
    assert logic.auto_arm_allowed()
    logic.step(1.0)
    assert logic.state == S.ARMING.value


def reach_cruise(logic, now=2.0):
    logic.update_position(
        logic.h[0], logic.h[1], logic.cruise_z, 0, 0, 0,
        logic.h[3], True)
    logic.step(now)
    logic.step(now + logic.stable_seconds)
    assert logic.state == S.HOVER_150CM.value


def provide_stable_visual(logic, now, aligned=False):
    logic.update_visual(True, aligned, now)
    logic.step(now)
    logic.update_visual(True, aligned, now + logic.visual_stable_seconds)
    logic.step(now + logic.visual_stable_seconds)


def test_hover_test_complete_flow_requires_manual_arm():
    logic = ready_logic('hover_test', hover_test_seconds=2.0)
    enter_takeoff(logic)
    assert not logic.auto_arm_allowed()
    reach_cruise(logic)
    logic.step(4.2)
    assert logic.state == S.FINAL_LAND.value
    logic.landed = True
    logic.update_status(False, 18, False, False, 4.3)
    logic.step(4.3)
    assert logic.state == S.COMPLETE.value
    assert not logic.payload_sent


def test_hover_control_enable_is_explicit_start_without_car_topic():
    logic = MissionLogic(
        'hover_test', simulation_mode=True, enable_control=True)
    logic.update_position(0, 0, 0, 0, 0, 0, 0, True)
    logic.odom_ready = True
    logic.attitude_valid = True
    logic.step(0)
    logic.step(0.1)
    assert logic.state == S.WAIT_START.value
    logic.step(0.2)
    assert logic.state == S.PRESTREAM.value
    assert logic.h == (0.0, 0.0, 0.0, 0.0)


def test_drop_complete_flow_and_b_milestone():
    logic = ready_logic(
        'drop', enable_payload_release=True, hover_confirm_seconds=3.0)
    enter_takeoff(logic)
    reach_cruise(logic)
    logic.step(2.1)
    assert logic.state == S.HOVER_3S.value
    logic.step(5.1)
    assert logic.state == S.SEARCH_CAR.value
    provide_stable_visual(logic, 5.2)
    assert logic.state == S.VISION_FOLLOW.value
    assert logic.formed_follow_before_b
    provide_stable_visual(logic, 5.5, aligned=True)
    assert logic.state == S.ALIGN_FOR_DROP.value
    provide_stable_visual(logic, 5.8, aligned=True)
    assert logic.state == S.WAIT_RELEASE_ACK.value
    assert logic.payload_sent and logic.completed_before_d
    logic.payload_ack = True
    logic.step(6.2)
    assert logic.state == S.RETURN_HOME.value
    logic.step(6.3)
    assert logic.state == S.FINAL_LAND.value
    logic.landed = True
    logic.armed = False
    logic.step(6.4)
    assert logic.state == S.COMPLETE.value


def test_b_milestone_not_claimed_after_b():
    logic = ready_logic('drop')
    logic.state = S.SEARCH_CAR.value
    logic.car_progress = CarProgress.PASSED_B
    provide_stable_visual(logic, 1.0)
    assert logic.state == S.VISION_FOLLOW.value
    assert not logic.formed_follow_before_b


@pytest.mark.parametrize('state', [
    S.SEARCH_CAR.value, S.VISION_FOLLOW.value, S.ALIGN_FOR_DROP.value,
    S.ALIGN_PLATFORM.value, S.DYNAMIC_DESCENT_HIGH.value,
    S.DYNAMIC_DESCENT_NEAR.value])
def test_d_passed_before_completion_forces_return(state):
    logic = ready_logic('drop')
    logic.state = state
    logic.car_progress = CarProgress.PASSED_D
    logic.step(2.0)
    assert logic.state == S.RETURN_HOME.value
    assert not logic.payload_sent


def test_release_is_single_and_ack_timeout_enters_failsafe():
    logic = ready_logic(
        'drop', enable_payload_release=True, payload_ack_timeout=1.0)
    logic.state = S.ALIGN_FOR_DROP.value
    logic.position = (logic.h[0], logic.h[1], logic.cruise_z)
    logic.update_visual(True, True, 1.0)
    logic.update_visual(True, True, 1.3)
    logic.step(1.3)
    assert logic.payload_sent
    logic.step(1.4)
    assert logic.state == S.WAIT_RELEASE_ACK.value
    logic.step(2.4)
    assert logic.state == S.FAILSAFE.value
    assert logic.payload_sent


def test_payload_ack_must_follow_release():
    assert not payload_ack_is_new(None, 1.0, True)
    assert not payload_ack_is_new(2.0, 1.0, True)
    assert not payload_ack_is_new(2.0, 2.0, True)
    assert payload_ack_is_new(2.0, 2.1, True)


def test_ninety_second_deadline_prioritizes_return():
    logic = ready_logic('drop')
    logic.started_at = 0.0
    logic.state = S.VISION_FOLLOW.value
    logic.step(90.0)
    assert logic.state == S.RETURN_HOME.value
    assert logic.event == 'MISSION_DEADLINE_RETURN'


def test_b_deadline_records_missed_follow_without_chase():
    logic = ready_logic('drop', b_deadline_seconds=15.0)
    logic.started_at = 0.0
    logic.state = S.SEARCH_CAR.value
    logic.step(15.0)
    assert logic.event == 'B_FOLLOW_MILESTONE_MISSED'
    assert logic.state == S.SEARCH_CAR.value


def test_dynamic_high_near_and_touchdown_before_d():
    logic = ready_logic(
        'dynamic_land', enable_dynamic_landing=True,
        enable_auto_disarm=True, enable_second_takeoff=True)
    logic.state = S.ALIGN_PLATFORM.value
    logic.update_visual(True, True, 1.0)
    logic.update_visual(True, True, 1.3)
    logic.step(1.3)
    assert logic.state == S.DYNAMIC_DESCENT_HIGH.value
    logic.update_position(2, 3, 3.5, 0, 0, 0.1, 0.2, True)
    logic.step(1.4)
    assert logic.state == S.DYNAMIC_DESCENT_NEAR.value
    logic.touchdown_candidate = True
    logic.step(1.5)
    assert logic.state == S.TOUCHDOWN_CHECK.value
    logic.touchdown = True
    logic.step(1.6)
    assert logic.state == S.LANDED_ON_CAR.value
    assert logic.completed_before_d


def test_descent_phase_speed_switch():
    assert descent_speed_for_state(
        S.DYNAMIC_DESCENT_HIGH.value, .25, .1, .05) == .25
    assert descent_speed_for_state(
        S.DYNAMIC_DESCENT_NEAR.value, .25, .1, .05) == .1
    assert descent_speed_for_state(
        S.TOUCHDOWN_CHECK.value, .25, .1, .05) == .05
    assert descent_speed_for_state(S.RETURN_HOME.value, .25, .1, .05) == 0.0


def test_visual_loss_stops_descent_and_recovery_is_stable():
    logic = ready_logic('dynamic_land', enable_dynamic_landing=True)
    logic.state = S.DYNAMIC_DESCENT_HIGH.value
    logic.update_visual(False, False, 1.0)
    logic.step(1.0)
    assert logic.state == S.DYNAMIC_DESCENT_HIGH.value
    assert not logic.visual_stable(1.0)
    logic.update_visual(True, True, 1.1)
    assert not logic.visual_stable(1.1)
    logic.update_visual(True, True, 1.3)
    assert logic.visual_stable(1.3)


def test_touchdown_requires_continuous_confirmation():
    detector = TouchdownDetector(confirm_seconds=1)
    assert not detector.update(0, True, False, 0, 0, 0)
    assert not detector.update(.5, True, False, 0, 0, 0)
    assert detector.update(1, True, False, 0, 0, 0)
    assert not detector.update(1.1, False, False, 0, 0, 0)


def test_controller_latches_confirmed_touchdown_for_platform_dwell():
    source = Path(
        __file__).parents[1] / 'uav_control' / 'mission_controller_node.py'
    text = source.read_text(encoding='utf-8')
    assert 'if touchdown_confirmed_now:' in text
    assert 'elif self.logic.state in descent_states:' in text


def test_command_failures_use_declared_terminal_failsafe_state():
    source = Path(
        __file__).parents[1] / 'uav_control' / 'mission_controller_node.py'
    text = source.read_text(encoding='utf-8')
    assert 'FAILSAFE_LAND' not in text


def test_sitl_platform_dwell_keeps_safe_contact_descent_only_in_simulation():
    source = Path(
        __file__).parents[1] / 'uav_control' / 'mission_controller_node.py'
    text = source.read_text(encoding='utf-8')
    platform = text.index("'LANDED_ON_CAR', 'DWELL_5S'")
    final_land = text.index("elif self.logic.state == 'FINAL_LAND':", platform)
    branch = text[platform:final_land]
    assert 'if self.logic.simulation_mode:' in branch
    assert "'contact_descent_speed'" in branch
    assert "elif self.logic.state == 'DISARM_ON_CAR':" in text


def test_sitl_disarm_waits_for_px4_independent_landed_signal():
    source = Path(
        __file__).parents[1] / 'uav_control' / 'mission_controller_node.py'
    text = source.read_text(encoding='utf-8')
    assert "VEHICLE_LAND_DETECTED_TOPIC = '/fmu/out/vehicle_land_detected'" in text
    assert 'self.logic.simulation_mode and not self.px4_landed' in text
    assert "'land', VehicleCommand.VEHICLE_CMD_NAV_LAND" in text


def test_five_second_dwell_and_disarm_gate():
    logic = ready_logic(
        'dynamic_land', enable_dynamic_landing=True,
        enable_auto_disarm=True, enable_second_takeoff=True)
    logic.touchdown = True
    logic.state = S.LANDED_ON_CAR.value
    logic.step(1.0)
    assert logic.state == S.DWELL_5S.value
    logic.step(5.9)
    assert logic.state == S.DWELL_5S.value
    logic.step(6.0)
    assert logic.state == S.DISARM_ON_CAR.value
    logic.simulation_mode = False
    assert not logic.auto_disarm_allowed(False)
    assert logic.auto_disarm_allowed(True)
    logic.armed = False
    logic.step(6.1)
    assert logic.state == S.SECOND_PRESTREAM.value
    assert logic.disarm_confirmed


def test_second_takeoff_requires_disarm_dwell_px4_and_vision():
    logic = ready_logic(
        'dynamic_land', enable_second_takeoff=True,
        enable_auto_disarm=True)
    logic.state = S.SECOND_PRESTREAM.value
    logic.second_cycle = True
    logic.disarm_confirmed = False
    logic.prestream_count = 40
    logic.step(10)
    assert logic.state == S.SECOND_PRESTREAM.value
    logic.disarm_confirmed = True
    logic.px4_fresh = True
    logic.update_visual(True, True, 10)
    logic.step(10.1)
    assert logic.state == S.SECOND_PRESTREAM.value
    logic.step(10.3)
    assert logic.state == S.REQUEST_OFFBOARD.value
    logic.offboard = True
    logic.step(10.4)
    assert logic.state == S.SECOND_ARM.value


def test_second_cycle_reenters_offboard_before_second_arm():
    logic = ready_logic(
        'dynamic_land', enable_second_takeoff=True,
        enable_auto_disarm=True)
    logic.second_cycle = True
    logic.disarm_confirmed = True
    logic.state = S.SECOND_PRESTREAM.value
    logic.prestream_count = 40
    logic.update_visual(True, True, 10)
    logic.step(10.6)
    assert logic.state == S.REQUEST_OFFBOARD.value
    assert not logic.armed
    logic.offboard = True
    logic.step(10.7)
    assert logic.state == S.SECOND_ARM.value


def test_second_cycle_offboard_reentry_is_not_abnormal_exit():
    logic = ready_logic('dynamic_land', enable_second_takeoff=True)
    logic.ever_offboard = True
    logic.second_cycle = True
    logic.disarm_confirmed = True
    logic.state = S.REQUEST_OFFBOARD.value
    logic.update_status(False, 0, False, False, 11)
    assert logic.state == S.REQUEST_OFFBOARD.value
    logic.second_cycle = False
    logic.update_status(False, 0, False, False, 12)
    assert logic.state == S.FAILSAFE.value


def test_defaults_disable_arm_disarm_second_takeoff_and_commands():
    logic = MissionLogic()
    assert not logic.enable_auto_arm
    assert not logic.enable_auto_disarm
    assert not logic.enable_second_takeoff
    for state in MissionState:
        assert not vehicle_command_allowed(False, state.value)


def test_final_land_is_not_an_offboard_control_output_state():
    assert control_output_allowed(S.FINAL_LAND.value, False)


def test_final_land_stops_reissuing_nav_land_after_px4_enters_auto_land():
    source = (
        Path(__file__).parents[1] / 'uav_control' /
        'mission_controller_node.py').read_text()
    assert "self.logic.state == 'FINAL_LAND' and self.logic.armed" in source
    assert 'VehicleStatus.NAVIGATION_STATE_AUTO_LAND' in source
    source = Path(
        __file__).parents[1] / 'uav_control' / 'mission_controller_node.py'
    text = source.read_text(encoding='utf-8')
    final_land_branch = text.index("elif self.logic.state == 'FINAL_LAND':")
    next_home_branch = text.index('elif self.logic.h is not None:', final_land_branch)
    assert '_publish_control' not in text[final_land_branch:next_home_branch]


def test_home_locks_once_at_start():
    logic = ready_logic()
    assert logic.h == (2.0, 3.0, 4.0, 0.2)
    logic.update_position(9, 8, 7, 0, 0, 0, 0.3, True)
    assert logic.lock_home()
    assert logic.h == (2.0, 3.0, 4.0, 0.2)
    assert logic.cruise_z == pytest.approx(2.5)


def test_prestream_default_and_minimum():
    assert MissionLogic().prestream_cycles == 40
    assert prestream_is_safe(True, 20)
    assert not prestream_is_safe(True, 19)
    assert prestream_is_safe(False, 0)


def test_px4_timeout_and_failsafe_block_output():
    logic = ready_logic()
    logic.transition(S.DATA_TIMEOUT, 1.0, 'PX4_DATA_TIMEOUT')
    logic.step(2.0)
    assert not control_output_allowed(logic.state, True)
    assert not vehicle_command_allowed(True, S.TAKEOFF.value, True)
    assert S.DATA_TIMEOUT.value in STATE_ID


@pytest.mark.parametrize('lost_input', ['position', 'attitude'])
def test_airborne_critical_data_loss_requests_safe_land(lost_input):
    assert lost_input
    assert data_loss_action(True, False) == S.FINAL_LAND.value


def test_status_loss_cannot_issue_blind_flight_command():
    assert data_loss_action(True, True) == S.DATA_TIMEOUT.value


def test_disarmed_data_loss_never_progresses_or_arms():
    assert data_loss_action(False, False) == S.DATA_TIMEOUT.value


def test_airborne_abort_uses_return_and_normal_land_path():
    logic = ready_logic('hover_test')
    logic.armed = True
    logic.state = S.HOVER_150CM.value
    assert logic.request_safe_abort(5.0)
    assert logic.state == S.RETURN_HOME.value
    assert logic.event == 'MISSION_ABORT_REQUESTED'


def test_disarmed_or_terminal_abort_does_not_advance():
    logic = ready_logic('hover_test')
    before = logic.state
    assert not logic.request_safe_abort(5.0)
    assert logic.state == before
    logic.armed = True
    logic.state = S.FAILSAFE.value
    assert not logic.request_safe_abort(5.1)
    assert logic.state == S.FAILSAFE.value


def test_offboard_exit_enters_failsafe():
    logic = ready_logic()
    logic.state = S.TAKEOFF.value
    logic.update_status(True, 14, True, False, 1.0)
    logic.update_status(True, 2, False, False, 1.1)
    assert logic.state == S.FAILSAFE.value


def test_vision_requires_local_and_source_freshness():
    assert vision_is_fresh(10.0, 9.8, 100.0, 0.5)
    assert not vision_is_fresh(10.0, 9.0, 100.0, 0.5)
    assert not vision_is_fresh(10.0, 9.8, 251.0, 0.5)
    assert not vision_is_fresh(10.0, None, 10.0, 0.5)


@pytest.mark.parametrize('quaternion', [
    [math.nan, 0, 0, 1], [math.inf, 0, 0, 1],
    [0, 0, 0, 0], [2, 0, 0, 0], [1, 0, 0]])
def test_invalid_quaternion_rejected(quaternion):
    assert not quaternion_is_valid(quaternion)


def test_visual_guidance_flu_ned_finite_and_vector_limit():
    guide = VisualGuidance(max_speed=.2, camera_x_sign=1, camera_y_sign=-1)
    north, east = guide.velocity(True, 1, -1, 90, 0, 0)
    assert math.hypot(north, east) <= .2
    assert VisualGuidance.flu_to_ned(
        1, 0, math.pi / 2) == pytest.approx((0, 1), abs=1e-8)
    assert guide.velocity(True, math.nan, 0, 90, 0, 0) == (0, 0)


def test_command_tracker_is_finite_and_ack_correlated():
    tracker = CommandTracker(timeout=1, max_attempts=2)
    assert tracker.request(400, 0)
    assert not tracker.acknowledge(176, 0)
    assert tracker.acknowledge(400, 0)
    assert tracker.status == 'ACCEPTED'


def test_dashboard_has_no_px4_or_payload_control_publishers():
    source = (PACKAGE_ROOT / 'uav_control' /
              'mission_dashboard_node.py').read_text()
    forbidden = (
        'OffboardControlMode', 'TrajectorySetpoint', 'VehicleCommand',
        "'/uav/payload/release'")
    assert all(item not in source for item in forbidden)
    assert source.count('create_publisher(') == 1
    assert "'/uav/mission/debug_canvas'" in source


def test_three_modes_use_one_formal_controller():
    launch = (PACKAGE_ROOT / 'launch' / 'd_task_control.launch.py').read_text()
    assert launch.count("executable='mission_controller_node'") == 1
    assert 'drop' in launch and 'dynamic_land' in launch and 'hover_test' in launch


def test_installed_entry_points_have_one_control_publisher():
    setup = (PACKAGE_ROOT / 'setup.py').read_text()
    assert setup.count("'mission_controller_node = '") == 1
    assert 'offboard_control =' not in setup
    assert 'vision_offboard_controller =' not in setup
    assert not (PACKAGE_ROOT / 'uav_control' / 'offboard_control.py').exists()
    assert not (
        PACKAGE_ROOT / 'uav_control' /
        'vision_offboard_controller.py').exists()


def test_every_installed_launch_uses_only_the_unified_controller():
    launch_text = '\n'.join(
        path.read_text() for path in
        (PACKAGE_ROOT / 'launch').glob('*.launch.py'))
    assert launch_text.count("executable='mission_controller_node'") == 5
    assert "executable='offboard_control'" not in launch_text
    assert 'vision_offboard_controller' not in launch_text


@pytest.mark.parametrize('profile', [
    'competition_drop', 'competition_dynamic_land', 'first_flight_hover',
    'sitl_drop', 'sitl_dynamic_land'])
def test_yaml_safety_boundaries(profile):
    document = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / f'{profile}.yaml').read_text())
    params = document['mission_controller_node']['ros__parameters']
    assert params['control_rate_hz'] == 20.0
    assert params['prestream_cycles'] >= 20
    assert 0.5 <= params['target_altitude'] <= 2.0
    assert (0.0 <= params['contact_descent_speed'] <=
            params['near_descent_speed'] <= params['high_descent_speed'] <=
            params['max_descent_speed'])
    assert params['dwell_on_car_seconds'] >= 5.0
    assert params['mission_timeout_seconds'] == 90.0
    assert not params['enable_auto_disarm']
    assert not params['enable_second_takeoff']
    assert not params['allow_sitl_heading_quality_bypass']


@pytest.mark.parametrize('profile', [
    'competition_drop', 'competition_dynamic_land', 'first_flight_hover'])
def test_hardware_profiles_default_to_monitor_only(profile):
    document = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / f'{profile}.yaml').read_text())
    params = document['mission_controller_node']['ros__parameters']
    assert not params['enable_control']
    assert not params['enable_auto_arm']
    assert not params['enable_auto_disarm']
    assert not params['enable_second_takeoff']
    assert not params['simulation_mode']
    assert not params['allow_sitl_heading_quality_bypass']


def test_first_flight_hover_is_default_safe():
    document = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'first_flight_hover.yaml').read_text())
    params = document['mission_controller_node']['ros__parameters']
    assert params['mission_mode'] == 'hover_test'
    assert params['target_altitude'] == 1.0
    assert not params['enable_control']
    assert not params['enable_auto_arm']
    assert not params['enable_visual_follow']
    assert not params['enable_payload_release']


def test_schema_and_telemetry_are_complete():
    assert STATE_ID[S.WAIT_START.value] >= 0
    assert STATE_ID[S.FAILSAFE.value] >= 0
    assert TELEMETRY_LENGTH >= 35


def test_final_land_requires_landed_and_disarmed():
    logic = MissionLogic('drop')
    logic.transition('FINAL_LAND', 0.0)
    logic.update_status(True, 0, False, False)
    logic.update_landed(True)
    logic.step(1.0)
    assert logic.state == 'FINAL_LAND'
    logic.update_status(False, 0, False, False)
    logic.step(2.0)
    assert logic.state == 'COMPLETE'


def test_frozen_vision_array_contracts():
    tracked = [0.0] * 12
    error = [0.0] * 8
    assert len(validate_tracked(tracked)) == 12
    assert len(validate_landing_error(error)) == 8
    with pytest.raises(ValueError):
        validate_tracked([0.0] * 11)
    with pytest.raises(ValueError):
        validate_landing_error([0.0] * 7)
