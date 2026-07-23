"""H7Plus目标滤波状态机测试。"""

import math

import pytest

from uav_vision.target_filter_node import INVALID_DETECTION, TargetFilter


def detection(cx=160.0, confidence=90.0, valid=1.0):
    """生成一帧合法测试检测。"""
    return [valid, cx, 120.0, 50.0, 48.0, 2400.0, confidence]


def test_confirms_after_three_frames():
    """默认配置必须连续三帧才确认目标。"""
    target_filter = TargetFilter()
    assert target_filter.process(detection()) == INVALID_DETECTION
    assert target_filter.process(detection()) == INVALID_DETECTION
    assert target_filter.process(detection())[0] == 1.0


def test_low_confidence_is_rejected():
    """低于置信度门限的数据不能参与确认。"""
    target_filter = TargetFilter()
    for _ in range(5):
        assert target_filter.process(
            detection(confidence=49.9)) == INVALID_DETECTION


def test_ema_is_correct_and_first_value_is_direct():
    """首帧直接初始化，后续帧按EMA公式更新。"""
    target_filter = TargetFilter(alpha=0.5, confirm_frames=1)
    first = target_filter.process(detection(cx=100.0))
    second = target_filter.process(detection(cx=140.0))
    assert first[1] == 100.0
    assert second[1] == pytest.approx(120.0)
    assert second[6] == pytest.approx(90.0)


def test_short_loss_keeps_filtered_target():
    """不足lost_frames的短暂丢失应保持最后滤波结果。"""
    target_filter = TargetFilter(confirm_frames=1, lost_frames=3)
    confirmed = target_filter.process(detection(cx=150.0))
    first_loss = target_filter.process(detection(valid=0.0, confidence=0.0))
    second_loss = target_filter.process(detection(valid=0.0, confidence=0.0))
    assert first_loss == confirmed
    assert second_loss == confirmed


def test_repeated_loss_becomes_invalid():
    """达到lost_frames时必须发布全零无效目标。"""
    target_filter = TargetFilter(confirm_frames=1, lost_frames=3)
    target_filter.process(detection())
    target_filter.process(detection(valid=0.0, confidence=0.0))
    target_filter.process(detection(valid=0.0, confidence=0.0))
    assert target_filter.process(
        detection(valid=0.0, confidence=0.0)) == INVALID_DETECTION


@pytest.mark.parametrize('values', [
    [1.0, 160.0],
    [1.0, 160.0, 120.0, 50.0, 48.0, 2400.0, 90.0, 1.0],
    [2.0, 160.0, 120.0, 50.0, 48.0, 2400.0, 90.0],
    [1.0, -1.0, 120.0, 50.0, 48.0, 2400.0, 90.0],
    [1.0, 160.0, 120.0, -1.0, 48.0, 2400.0, 90.0],
    [1.0, 160.0, 120.0, 50.0, 48.0, 2400.0, 101.0],
    [1.0, math.nan, 120.0, 50.0, 48.0, 2400.0, 90.0],
    [1.0, math.inf, 120.0, 50.0, 48.0, 2400.0, 90.0],
])
def test_invalid_arrays_are_rejected(values):
    """字段数、范围或有限性不合法的数据必须被拒绝。"""
    with pytest.raises(ValueError):
        TargetFilter().process(values)
