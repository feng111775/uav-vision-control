from uav_control.first_task_logic import FirstTaskInputs, FirstTaskMissionLogic


def px4(**kwargs):
    values = {
        'now': 0.0, 'readiness': True, 'px4_fresh': True,
        'position_valid': True, 'heading_valid': True, 'x': 0.0,
        'y': 0.0, 'z': 0.0, 'vx': 0.0, 'vy': 0.0, 'vz': 0.0,
        'heading': 0.0,
    }
    values.update(kwargs)
    return FirstTaskInputs(**values)


def test_first_task_home_takeoff_hover_and_search():
    logic = FirstTaskMissionLogic(enable_control=True)
    logic.step(px4(now=0.0))
    assert logic.state == 'WAIT_START'
    logic.step(px4(now=1.0, start_signal=True))
    assert logic.state == 'TAKEOFF'
    logic.step(px4(now=2.0, z=1.5))
    assert logic.state == 'HOVER_150CM'
    logic.step(px4(now=3.0, z=1.5))
    assert logic.state == 'HOVER_3S'
    logic.step(px4(now=6.1, z=1.5))
    assert logic.state == 'SEARCH_CAR'
    assert logic.search_speed <= 0.18


def test_first_task_release_is_once_and_ack_is_required():
    logic = FirstTaskMissionLogic(
        enable_control=True, enable_closed_loop=True,
        enable_payload_release=True,
    )
    logic.step(px4(now=0.0))
    logic.step(px4(now=1.0, start_signal=True))
    logic.state = 'ALIGN_FOR_DROP'
    valid = px4(
        now=2.0, car_progress=4, vision_ready=True, tracked_valid=True,
        landing_valid=True, capture_stamp_valid=True,
        frame_sequence_new=True, vision_fresh=True, confidence_ok=True,
    )
    logic.step(valid)
    assert logic.state == 'PAYLOAD_RELEASE'
    assert logic.release_attempted
    logic.step(px4(now=3.0, release_ack=True))
    assert logic.state == 'WAIT_RELEASE_ACK'
    logic.step(px4(now=4.0, release_ack=True))
    assert logic.state == 'RETURN_HOME'


def test_first_task_complete_requires_landed_and_disarmed():
    logic = FirstTaskMissionLogic(
        land_confirm_timeout_s=1.0, disarm_confirm_timeout_s=1.0,
    )
    logic.started_at = 0.0
    logic.state = 'FINAL_LAND'
    logic.home = (0.0, 0.0, 0.0, 0.0)
    logic.step(px4(now=1.0, landed=True, disarmed=False))
    assert logic.state == 'FINAL_LAND'
    logic.step(px4(now=2.1, landed=True, disarmed=True))
    assert logic.state == 'FINAL_LAND'
    logic.step(px4(now=3.2, landed=True, disarmed=True))
    assert logic.state == 'COMPLETE'


def test_first_task_90_second_timeout_cannot_complete():
    logic = FirstTaskMissionLogic()
    logic.started_at = 0.0
    logic.state = 'SEARCH_CAR'
    logic.step(px4(now=90.1))
    assert logic.state == 'TIMEOUT'
