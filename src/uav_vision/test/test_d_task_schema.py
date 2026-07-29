import pytest

from uav_vision.d_task_schema import validate_detection


def test_schema_length_and_valid_detection():
    assert validate_detection([1, 160, 120, 50, 30, -0.1, 80])[0] == 1.0
    with pytest.raises(ValueError):
        validate_detection([1, 2])


def test_valid_and_invalid_detection_rules():
    assert validate_detection([0, 0, 0, 0, 0, -2, 0])[0] == 0.0
    with pytest.raises(ValueError):
        validate_detection([1, 0, 0, 0, 0, 0, 50])


def test_outer_smaller_than_inner_rejected():
    with pytest.raises(ValueError):
        validate_detection([1, 10, 10, 20, 30, 0, 50])


def test_negative_angle_is_not_treated_as_geometry():
    assert validate_detection([1, 10, 10, 30, 20, -10, 50])[5] == -10.0
