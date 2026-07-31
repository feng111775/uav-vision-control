import pytest

from uav_vision.target_predictor_node import TargetPredictor


def detection(x, y=120.0, valid=1.0):
    return [valid, x, y, 50.0, 30.0, 0.0, 90.0]


def test_velocity_estimate_and_forward_prediction():
    predictor = TargetPredictor(prediction_horizon=0.08)
    predictor.process(detection(100.0), 1.0)
    result = predictor.process(detection(110.0), 1.1)
    assert result[7] == pytest.approx(100.0)
    assert result[9] == pytest.approx(118.0)


def test_huge_jump_is_limited():
    predictor = TargetPredictor(max_jump=20.0, max_velocity=1000.0)
    predictor.process(detection(100.0), 1.0)
    result = predictor.process(detection(300.0), 1.1)
    assert result[1] == pytest.approx(120.0)
    assert abs(result[7]) <= 1000.0


def test_loss_prediction_then_timeout():
    predictor = TargetPredictor(loss_timeout=0.2)
    predictor.process(detection(100.0), 1.0)
    predictor.process(detection(110.0), 1.1)
    assert predictor.process(detection(0.0, valid=0.0), 1.2)[0] == 1.0
    assert predictor.process(detection(0.0, valid=0.0), 1.31)[0] == 0.0


def test_reset_forbids_old_detection_reuse():
    predictor = TargetPredictor(loss_timeout=1.0)
    assert predictor.process(detection(100.0), 1.0)[0] == 1.0
    predictor.reset()
    assert predictor.process(detection(0.0, valid=0.0), 1.01)[0] == 0.0
