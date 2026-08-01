"""Stage-1 contracts between the retained controller and formal vision chain."""

import math

import pytest

from uav_control.mission_logic import MissionLogic, vision_is_fresh
from uav_control.vision_contract import validate_landing_error, validate_tracked
from uav_vision.landing_error_node import LandingErrorCalculator
from uav_vision.target_filter_node import TargetFilter
from uav_vision.target_predictor_node import TargetPredictor


def detection(confidence=90.0):
    return [1.0, 160.0, 120.0, 50.0, 30.0, 0.0, confidence]


def test_formal_chain_requires_three_frames_and_produces_frozen_lengths():
    filt = TargetFilter(min_confidence=50.0, confirm_frames=3, lost_frames=3)
    assert filt.process(detection())[0] == 0.0
    assert filt.process(detection())[0] == 0.0
    filtered = filt.process(detection())
    assert len(filtered) == 7 and filtered[0] == 1.0
    tracked = TargetPredictor().process(filtered, 10.0)
    assert len(validate_tracked(tracked)) == 12
    error = LandingErrorCalculator().calculate(tracked)
    assert len(validate_landing_error(error)) == 8


def test_invalid_nan_inf_and_wrong_lengths_are_rejected():
    for values in ([1.0] * 6, [1.0] * 8, [math.nan] * 12,
                   [math.inf] * 12):
        with pytest.raises(ValueError):
            validate_tracked(values)
    with pytest.raises(ValueError):
        TargetFilter().process(detection() + [math.nan])


def test_two_second_old_frame_is_not_fresh():
    assert vision_is_fresh(10.0, 8.0, 0.0, 0.5) is False
    assert vision_is_fresh(10.0, 9.9, 100.0, 0.5) is True


def test_controller_requires_continuous_visual_stability():
    logic = MissionLogic(mission_mode='drop', visual_stable_seconds=0.5)
    logic.transition('SEARCH_CAR', 0.0)
    logic.update_visual(True, False, 1.0)
    logic._search_car(1.4)
    assert logic.state == 'SEARCH_CAR'
    logic._search_car(1.5)
    assert logic.state == 'VISION_FOLLOW'


def test_low_confidence_and_loss_reset_follow_gate():
    logic = MissionLogic(mission_mode='drop', visual_stable_seconds=0.5)
    logic.transition('SEARCH_CAR', 0.0)
    logic.update_visual(True, False, 1.0)
    logic.update_visual(False, False, 1.1)
    assert logic.visual_stable(2.0) is False
    logic._search_car(2.0)
    assert logic.state == 'SEARCH_CAR'
