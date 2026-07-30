import math

import pytest

from uav_vision.target_filter_node import TargetFilter


def detection(angle=0.0, confidence=90.0, valid=1.0):
    return [valid, 160.0, 120.0, 50.0, 30.0, angle, confidence]


def test_low_confidence_and_continuous_confirmation():
    target_filter = TargetFilter(confirm_frames=2, min_confidence=50.0)
    assert target_filter.process(detection(confidence=49.0))[0] == 0.0
    assert target_filter.process(detection())[0] == 0.0
    assert target_filter.process(detection())[0] == 1.0


def test_short_loss_holds_then_long_loss_invalidates():
    target_filter = TargetFilter(confirm_frames=1, lost_frames=2)
    assert target_filter.process(detection())[0] == 1.0
    assert target_filter.process(detection(valid=0.0))[0] == 1.0
    assert target_filter.process(detection(valid=0.0))[0] == 1.0
    assert target_filter.process(detection(valid=0.0))[0] == 0.0


def test_ninety_degree_periodic_filter_avoids_false_45_degrees():
    target_filter = TargetFilter(alpha=0.5, angle_alpha=0.5,
                                 confirm_frames=1)
    target_filter.process(detection(math.radians(89.0)))
    result = target_filter.process(detection(math.radians(1.0)))
    assert abs(math.degrees(result[5])) < 2.0


def test_negative_angle_allowed():
    result = TargetFilter(confirm_frames=1).process(detection(-0.2))
    assert result[5] == pytest.approx(-0.2)


def test_reset_immediately_discards_confirmed_target():
    target_filter = TargetFilter(confirm_frames=1)
    assert target_filter.process(detection())[0] == 1.0
    target_filter.reset()
    assert target_filter.process(detection(valid=0.0))[0] == 0.0
