"""视觉伺服速度计算测试。"""

import math

import pytest

from uav_vision.visual_servo_node import ZERO_VELOCITY
from uav_vision.visual_servo_node import VisualServoController


def detection(cx=160.0, cy=120.0, valid=1.0):
    """生成一帧滤波检测数据。"""
    return [valid, cx, cy, 50.0, 48.0, 2400.0, 90.0]


def test_center_target_outputs_zero():
    """目标位于图像中心时水平速度必须为零。"""
    controller = VisualServoController()
    assert controller.process(detection(), 1.0) == ZERO_VELOCITY


@pytest.mark.parametrize(('cx', 'cy', 'x_sign', 'y_sign'), [
    (160.0, 80.0, 1, 0),   # 图像上方：向前。
    (160.0, 160.0, -1, 0),  # 图像下方：向后。
    (120.0, 120.0, 0, 1),   # 图像左侧：向左。
    (200.0, 120.0, 0, -1),  # 图像右侧：向右。
])
def test_four_direction_signs(cx, cy, x_sign, y_sign):
    """默认安装方向下四个图像方向应映射到正确FLU符号。"""
    forward, left = VisualServoController().process(
        detection(cx=cx, cy=cy), 1.0)
    assert (forward > 0) - (forward < 0) == x_sign
    assert (left > 0) - (left < 0) == y_sign


def test_deadband_outputs_zero_per_axis():
    """中心死区内的偏差不得产生速度。"""
    controller = VisualServoController(deadband_x=10.0, deadband_y=10.0)
    assert controller.process(
        detection(cx=170.0, cy=110.0), 1.0) == ZERO_VELOCITY


def test_velocity_is_clamped():
    """大偏差速度必须限制在最大速度范围内。"""
    controller = VisualServoController(
        kp_x=1.0, kp_y=1.0, deadband_x=0.0, deadband_y=0.0,
        max_velocity=0.3)
    assert controller.process(detection(cx=320.0, cy=240.0), 1.0) == (
        -0.3, -0.3)


def test_invalid_target_outputs_zero():
    """valid为零时必须立即得到零速度。"""
    controller = VisualServoController()
    assert controller.process(
        detection(valid=0.0), 1.0) == ZERO_VELOCITY


def test_stale_input_outputs_zero():
    """超过stale_timeout后必须返回零速度。"""
    controller = VisualServoController(stale_timeout=0.3)
    velocity = controller.process(detection(cy=80.0), 1.0)
    assert velocity[0] > 0.0
    assert controller.velocity_at(1.31) == ZERO_VELOCITY


@pytest.mark.parametrize('values', [
    [1.0, 160.0],
    [1.0, 160.0, 120.0, 50.0, 48.0, 2400.0, 90.0, 0.0],
    [2.0, 160.0, 120.0, 50.0, 48.0, 2400.0, 90.0],
    [1.0, math.nan, 120.0, 50.0, 48.0, 2400.0, 90.0],
    [1.0, 160.0, 120.0, -1.0, 48.0, 2400.0, 90.0],
    [1.0, 160.0, 120.0, 50.0, 48.0, 2400.0, 101.0],
])
def test_invalid_messages_are_rejected_and_clear_velocity(values):
    """非法消息必须抛出异常并将保存速度清零。"""
    controller = VisualServoController()
    controller.process(detection(cy=80.0), 1.0)
    with pytest.raises(ValueError):
        controller.process(values, 1.1)
    assert controller.velocity_at(1.1) == ZERO_VELOCITY
