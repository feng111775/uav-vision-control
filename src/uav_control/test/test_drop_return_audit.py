"""Safety and timing checks for the calibrated drop-and-return profile."""

from uav_control.mission_logic import MissionLogic
from uav_control.mission_schema import CarProgress, MissionState


S = MissionState


def started_logic(**kwargs):
    logic = MissionLogic(
        'drop', simulation_mode=True, enable_control=True,
        enable_auto_arm=True, prestream_cycles=20, stable_seconds=0.1,
        visual_stable_seconds=0.1, point_b_progress=2, point_d_progress=4,
        **kwargs)
    logic.update_position(0, 0, 0, 0, 0, 0, 0, True)
    logic.odom_ready = True
    logic.attitude_valid = True
    logic.start_signal = True
    logic.step(0.0)
    logic.step(0.1)
    logic.step(0.2)
    assert logic.state == S.PRESTREAM.value
    return logic


def test_drop_old_car_landing_states_are_unreachable():
    for state in (
            S.ALIGN_PLATFORM, S.DYNAMIC_DESCENT_HIGH,
            S.DYNAMIC_DESCENT_NEAR, S.TOUCHDOWN_CHECK,
            S.LANDED_ON_CAR, S.DWELL_5S, S.DISARM_ON_CAR,
            S.SECOND_PRESTREAM, S.SECOND_ARM, S.SECOND_TAKEOFF):
        logic = started_logic()
        logic.state = state.value
        logic.step(1.0)
        assert logic.state == S.RETURN_HOME.value
        assert logic.event == 'DROP_MODE_OLD_LAND_STATE'


def test_b_cutoff_returns_before_follow_is_formed():
    logic = started_logic()
    logic.state = S.SEARCH_CAR.value
    logic.update_car_progress_at(CarProgress.PASSED_B, 1.0)
    logic.step(1.0)
    assert logic.state == S.RETURN_HOME.value
    assert logic.event == 'B_FOLLOW_CUTOFF'
    assert not logic.payload_sent


def test_progress_stale_returns_safely():
    logic = started_logic(progress_timeout_seconds=1.0)
    logic.state = S.SEARCH_CAR.value
    logic.step(1.3)
    assert logic.state == S.RETURN_HOME.value
    assert logic.event == 'CAR_PROGRESS_TIMEOUT'


def test_d_cutoff_blocks_release():
    logic = started_logic(enable_payload_release=True)
    logic.state = S.ALIGN_FOR_DROP.value
    logic.update_position(0, 0, logic.cruise_z, 0, 0, 0, 0, True)
    logic.update_car_progress_at(CarProgress.PASSED_D, 1.0)
    logic.update_visual(True, True, 1.0)
    logic.update_visual(True, True, 1.2)
    logic.step(1.2)
    assert logic.state == S.RETURN_HOME.value
    assert not logic.payload_sent


def test_hover_stability_resets_when_vertical_speed_exceeds_limit():
    logic = started_logic(
        hover_max_vertical_speed_mps=0.15, progress_timeout_seconds=10.0)
    logic.state = S.HOVER_3S.value
    logic.update_position(0, 0, logic.cruise_z, 0, 0, 0, 0, True)
    logic.step(1.0)
    logic.update_position(0, 0, logic.cruise_z, 0, 0, 0.2, 0, True)
    logic.step(2.0)
    assert logic.state == S.HOVER_3S.value
    logic.update_position(0, 0, logic.cruise_z, 0, 0, 0, 0, True)
    logic.update_car_progress_at(CarProgress.STARTED_AT_A, 4.0)
    logic.step(4.0)
    assert logic.state == S.HOVER_3S.value
    logic.step(7.0)
    assert logic.state == S.SEARCH_CAR.value


def test_return_reserve_precedes_new_search():
    logic = started_logic(mission_deadline_s=90.0, return_reserve_s=10.0)
    logic.state = S.SEARCH_CAR.value
    logic.started_at = 0.0
    logic.step(80.0)
    assert logic.state == S.RETURN_HOME.value
    assert logic.event == 'MISSION_DEADLINE_RETURN'


def test_disabled_release_fails_closed_without_marking_sent():
    logic = started_logic(enable_payload_release=False)
    logic.state = S.ALIGN_FOR_DROP.value
    logic.update_position(0, 0, logic.cruise_z, 0, 0, 0, 0, True)
    logic.update_visual(True, True, 1.0)
    logic.update_visual(True, True, 1.2)
    logic.step(1.2)
    assert logic.state == S.FAILSAFE.value
    assert logic.event == 'PAYLOAD_RELEASE_DISABLED'
    assert not logic.payload_sent


def test_b_time_boundary_accepts_follow_at_14_9_seconds():
    logic = started_logic(progress_timeout_seconds=30.0)
    logic.started_at = 0.0
    logic.state = S.SEARCH_CAR.value
    logic.update_visual(True, False, 0.0)
    logic.step(14.9)
    assert logic.state == S.VISION_FOLLOW.value
    assert logic.formed_follow_before_b
    assert not logic.b_deadline_failed


def test_b_time_boundary_rejects_follow_first_seen_at_15_seconds():
    logic = started_logic(progress_timeout_seconds=30.0)
    logic.started_at = 0.0
    logic.state = S.SEARCH_CAR.value
    logic.step(15.0)
    assert logic.b_deadline_failed
    assert not logic.formed_follow_before_b
    logic.update_visual(True, False, 0.0)
    logic.step(15.1)
    assert logic.state == S.VISION_FOLLOW.value
    assert not logic.formed_follow_before_b


def test_throw_at_48_9_and_success_at_51_9_passes_d_deadline():
    logic = started_logic(
        enable_payload_release=True, progress_timeout_seconds=60.0)
    logic.started_at = 0.0
    logic.state = S.ALIGN_FOR_DROP.value
    logic.position = (logic.h[0], logic.h[1], logic.cruise_z)
    logic.update_visual(True, True, 0.0)
    logic.step(48.9)
    assert logic.payload_sent
    assert logic.state == S.PAYLOAD_RELEASE.value
    logic.step(49.0)
    assert logic.state == S.WAIT_RELEASE_ACK.value
    logic.payload_ack = True
    logic.step(51.9)
    assert logic.state == S.RETURN_HOME.value
    assert logic.d_deadline_passed
    assert not logic.d_deadline_failed


def test_latest_command_deadline_forbids_first_throw():
    logic = started_logic(
        enable_payload_release=True, progress_timeout_seconds=60.0)
    logic.started_at = 0.0
    logic.state = S.ALIGN_FOR_DROP.value
    logic.position = (logic.h[0], logic.h[1], logic.cruise_z)
    logic.update_visual(True, True, 0.0)
    logic.step(49.0)
    assert logic.payload_forbidden
    assert not logic.payload_sent
    assert logic.state == S.RETURN_HOME.value


def test_d_deadline_without_ack_returns_without_retry():
    logic = started_logic(
        enable_payload_release=True, progress_timeout_seconds=60.0)
    logic.started_at = 0.0
    logic.payload_sent = True
    logic.state = S.WAIT_RELEASE_ACK.value
    logic.step(52.0)
    assert logic.d_deadline_failed
    assert logic.state == S.RETURN_HOME.value
    logic.step(53.0)
    assert logic.payload_sent


def test_repeated_start_does_not_refresh_any_deadline():
    logic = started_logic(progress_timeout_seconds=60.0)
    logic.started_at = 0.0
    logic.state = S.SEARCH_CAR.value
    logic.start_signal = True
    logic.step(14.0)
    assert logic.started_at == 0.0
    logic.step(15.0)
    assert logic.b_deadline_failed


def test_progress_cutoff_wins_before_time_cutoff():
    logic = started_logic(progress_timeout_seconds=60.0)
    logic.started_at = 0.0
    logic.state = S.SEARCH_CAR.value
    logic.update_car_progress_at(CarProgress.PASSED_B, 10.0)
    logic.step(10.0)
    assert logic.state == S.RETURN_HOME.value
    assert logic.event == 'B_FOLLOW_CUTOFF'


def test_progress_d_cutoff_wins_before_d_time():
    logic = started_logic(
        enable_payload_release=True, progress_timeout_seconds=60.0)
    logic.started_at = 0.0
    logic.state = S.ALIGN_FOR_DROP.value
    logic.update_car_progress_at(CarProgress.PASSED_D, 40.0)
    logic.step(40.0)
    assert logic.state == S.RETURN_HOME.value
    assert not logic.payload_sent
