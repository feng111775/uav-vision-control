from uav_vision.landing_error_node import LandingErrorCalculator


def tracked(x=160.0, y=120.0, age=0.0, valid=1.0):
    return [valid, x, y, 50.0, 30.0, -0.2, 90.0,
            0.0, 0.0, x, y, age]


def test_landing_error_direction():
    result = LandingErrorCalculator().calculate(tracked(180.0, 100.0))
    assert result[1] > 0.0
    assert result[2] < 0.0
    assert result[3:5] == [20.0, -20.0]


def test_invalid_or_timed_out_input_is_immediately_invalid():
    calculator = LandingErrorCalculator(input_timeout_ms=100.0)
    assert calculator.calculate(tracked(valid=0.0))[0] == 0.0
    assert calculator.calculate(tracked(age=101.0))[0] == 0.0
