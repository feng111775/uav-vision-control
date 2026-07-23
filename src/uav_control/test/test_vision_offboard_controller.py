"""视觉速度到PX4 NED速度控制逻辑测试。."""

import math

import pytest

from uav_control.vision_offboard_controller import VisionOffboardLogic


@pytest.mark.parametrize(('heading', 'expected'), [
    (0.0, (1.0, 0.0)),
    (math.pi / 2.0, (0.0, 1.0)),
    (math.pi, (-1.0, 0.0)),
])
def test_flu_forward_to_ned_at_headings(heading, expected):
    """机体前向速度应随NED航向正确旋转。."""
    north, east = VisionOffboardLogic.flu_to_ned(1.0, 0.0, heading)
    assert north == pytest.approx(expected[0], abs=1e-7)
    assert east == pytest.approx(expected[1], abs=1e-7)


def test_left_velocity_becomes_negative_body_right():
    """FLU向左速度在heading为零时应成为负East速度。."""
    north, east = VisionOffboardLogic.flu_to_ned(0.0, 1.0, 0.0)
    assert north == pytest.approx(0.0)
    assert east == pytest.approx(-1.0)


def test_horizontal_velocity_limit_preserves_direction():
    """水平限幅应限制二维模长并保持方向。."""
    north, east = VisionOffboardLogic.limit_horizontal(3.0, 4.0, 0.3)
    assert math.hypot(north, east) == pytest.approx(0.3)
    assert north / east == pytest.approx(3.0 / 4.0)


def test_height_control_has_correct_ned_sign():
    """低于目标时速度为负向上，高于目标时速度为正向下。."""
    logic = VisionOffboardLogic()
    logic.position_valid = True
    logic.target_z = -2.0
    logic.position_z = 0.0
    assert logic.height_velocity() == pytest.approx(-0.5)
    logic.position_z = -2.5
    assert logic.height_velocity() == pytest.approx(0.4)


def test_ground_to_two_metres_commands_negative_ned_velocity():
    """地面z为0且目标高度2米时应命令向上。."""
    logic = prepare_active_logic()
    logic.step(0.0)
    assert logic.target_z == pytest.approx(-2.0)
    assert logic.height_velocity() < 0.0


def test_stale_vision_returns_zero_horizontal_velocity():
    """视觉消息超时后不得保持最后一次水平速度。."""
    logic = VisionOffboardLogic(vision_timeout=0.3)
    logic.update_vision(0.2, 0.0, True, 1.0)
    assert logic.vision_ned_velocity(1.2)[0] == pytest.approx(0.2)
    assert logic.vision_ned_velocity(1.31) == (0.0, 0.0)


def test_real_mode_rejects_auto_arm():
    """simulation_mode为false时自动解锁门必须保持关闭。."""
    logic = VisionOffboardLogic(
        simulation_mode=False,
        enable_offboard=True,
        enable_auto_arm=True,
    )
    assert not logic.auto_arm_allowed()


def prepare_active_logic():
    """创建已收到有效PX4状态的仿真控制逻辑。."""
    logic = VisionOffboardLogic(
        simulation_mode=True,
        enable_offboard=True,
        enable_auto_arm=True,
    )
    logic.update_position(0.0, 0.0, True, 0.0)
    logic.update_status(False, False, False, 0.0)
    return logic


def test_waiting_prestream_takeoff_and_vision_transitions():
    """满足启用、预流、解锁和高度条件后应依次转换状态。."""
    logic = prepare_active_logic()
    assert logic.step(0.0)[3:] == (False, False)
    assert logic.state == logic.PRESTREAM

    logic.update_position(0.0, 0.0, True, 1.0)
    logic.update_status(False, False, False, 1.0)
    assert logic.step(1.0)[3:] == (True, True)
    assert logic.state == logic.PRESTREAM

    logic.update_position(0.0, 0.0, True, 1.1)
    logic.update_status(True, True, False, 1.1)
    logic.step(1.1)
    assert logic.state == logic.TAKEOFF
    assert logic.height_velocity() < 0.0

    logic.update_position(-1.9, 0.0, True, 1.2)
    logic.update_status(True, True, False, 1.2)
    logic.step(1.2)
    assert logic.state == logic.VISION_CONTROL


def enter_takeoff(logic, now=1.1):
    """将测试状态机推进到TAKEOFF。."""
    logic.step(0.0)
    logic.update_position(0.0, 0.0, True, 1.0)
    logic.update_status(False, False, False, 1.0)
    logic.step(1.0)
    logic.update_position(0.0, 0.0, True, now)
    logic.update_status(True, True, False, now)
    logic.step(now)
    assert logic.state == logic.TAKEOFF
    assert logic.takeoff_start == pytest.approx(now)


@pytest.mark.parametrize('current_z', [-2.0, -1.9])
def test_takeoff_reaches_target_within_tolerance(current_z):
    """到达-2米或容差内-1.9米时进入视觉控制。."""
    logic = prepare_active_logic()
    enter_takeoff(logic)
    logic.update_position(current_z, 0.0, True, 1.2)
    logic.update_status(True, True, False, 1.2)
    logic.step(1.2)
    assert logic.state == logic.VISION_CONTROL


def test_reaching_target_prevents_takeoff_timeout():
    """成功到达后不得再触发起飞超时。."""
    logic = prepare_active_logic()
    enter_takeoff(logic)
    logic.update_position(-2.0, 0.0, True, 1.2)
    logic.update_status(True, True, False, 1.2)
    logic.step(1.2)
    logic.update_position(-2.0, 0.0, True, 20.0)
    logic.update_status(True, True, False, 20.0)
    logic.step(20.0)
    assert logic.state == logic.VISION_CONTROL
    assert logic.failsafe_reason is None


def test_real_takeoff_timeout_records_reason():
    """只有持续未到达高度且超时才进入FAILSAFE。."""
    logic = prepare_active_logic()
    enter_takeoff(logic)
    timeout_time = logic.takeoff_start + logic.takeoff_timeout + 0.1
    logic.update_position(-1.0, 0.0, True, timeout_time)
    logic.update_status(True, True, False, timeout_time)
    logic.step(timeout_time)
    assert logic.state == logic.FAILSAFE
    assert logic.failsafe_reason == '起飞超时'


def test_status_age_0504_seconds_does_not_timeout():
    """1.984Hz状态消息的0.504秒周期应被接受。."""
    logic = prepare_active_logic()
    logic.step(0.0)
    logic.update_position(0.0, 0.0, True, 0.504)
    logic.step(0.504)
    assert logic.state == logic.PRESTREAM
    assert logic.failsafe_reason is None


def test_status_age_over_limit_enters_failsafe_with_age():
    """状态消息超过1.5秒应记录实际年龄和限制。."""
    logic = prepare_active_logic()
    logic.step(0.0)
    logic.update_position(0.0, 0.0, True, 1.62)
    logic.step(1.62)
    assert logic.state == logic.FAILSAFE
    assert logic.failsafe_reason == (
        'PX4状态超时：age=1.62s limit=1.50s')


def test_local_position_age_over_limit_enters_failsafe():
    """本地位置消息仍使用0.5秒独立限制。."""
    logic = prepare_active_logic()
    logic.step(0.0)
    logic.update_status(False, False, False, 0.51)
    logic.step(0.51)
    assert logic.state == logic.FAILSAFE
    assert logic.failsafe_reason == (
        '本地位置超时：age=0.51s limit=0.50s')


def test_status_and_position_timeout_parameters_are_independent():
    """两个超时参数不得交叉使用。."""
    logic = VisionOffboardLogic(
        simulation_mode=True,
        enable_offboard=True,
        px4_status_timeout=2.0,
        local_position_timeout=0.25,
    )
    logic.update_position(0.0, 0.0, True, 0.0)
    logic.update_status(False, False, False, 0.0)
    logic.step(0.0)
    logic.update_position(0.0, 0.0, True, 0.3)
    logic.step(0.3)
    assert logic.state == logic.PRESTREAM

    other = VisionOffboardLogic(
        simulation_mode=True,
        enable_offboard=True,
        px4_status_timeout=0.25,
        local_position_timeout=2.0,
    )
    other.update_position(0.0, 0.0, True, 0.0)
    other.update_status(False, False, False, 0.0)
    other.step(0.0)
    other.update_status(False, False, False, 0.3)
    other.step(0.3)
    assert other.state == other.PRESTREAM


def test_invalid_position_enters_failsafe():
    """活动状态下PX4位置无效必须进入FAILSAFE。."""
    logic = prepare_active_logic()
    logic.step(0.0)
    logic.update_position(0.0, 0.0, False, 0.1)
    logic.update_status(False, False, False, 0.1)
    velocity = logic.step(0.1)[:3]
    assert logic.state == logic.FAILSAFE
    assert logic.failsafe_reason == '位置无效'
    assert velocity == (0.0, 0.0, 0.0)


def test_default_configuration_stays_safe_waiting():
    """默认参数只允许零速度且不得请求模式或解锁。."""
    logic = VisionOffboardLogic()
    assert logic.step(10.0) == (0.0, 0.0, 0.0, False, False)
    assert logic.state == logic.WAITING


def test_sitl_accepts_finite_heading_without_good_for_control():
    """SITL允许有限heading替代heading_good_for_control。."""
    logic = VisionOffboardLogic(
        simulation_mode=True,
        enable_offboard=True,
        enable_auto_arm=False,
    )
    accepted, relaxed = logic.position_acceptance(True, True, False, 0.25)
    assert accepted
    assert relaxed
    logic.update_position(0.0, 0.25, accepted, 0.0)
    logic.update_status(False, False, False, 0.0)
    logic.step(0.0)
    assert logic.state == logic.PRESTREAM


def test_real_mode_rejects_heading_not_good_for_control():
    """非SITL必须继续严格要求heading_good_for_control。."""
    logic = VisionOffboardLogic(
        simulation_mode=False,
        enable_offboard=True,
        enable_auto_arm=False,
    )
    accepted, relaxed = logic.position_acceptance(True, True, False, 0.25)
    assert not accepted
    assert not relaxed
    logic.update_position(0.0, 0.25, accepted, 0.0)
    logic.update_status(False, False, False, 0.0)
    logic.step(0.0)
    assert logic.state == logic.WAITING


def test_sitl_rejects_nonfinite_heading():
    """SITL也不能接受NaN heading。."""
    logic = VisionOffboardLogic(
        simulation_mode=True,
        enable_offboard=True,
        enable_auto_arm=False,
    )
    accepted, relaxed = logic.position_acceptance(
        True, True, False, math.nan)
    assert not accepted
    assert not relaxed
    logic.update_position(0.0, math.nan, accepted, 0.0)
    logic.update_status(False, False, False, 0.0)
    logic.step(0.0)
    assert logic.state == logic.WAITING
