import math

import pytest

from uav_control.search_guidance import SearchGuidance


def test_heading_rotation_and_speed_bound():
    guidance = SearchGuidance()
    guidance.start(10.0, 20.0, math.pi / 2.0, 0.0)
    velocity = guidance.velocity((10.0, 20.0), 0.1)
    assert velocity[0] == pytest.approx(0.0, abs=1e-9)
    assert velocity[1] == pytest.approx(0.18, abs=1e-9)
    assert math.hypot(*velocity) <= 0.18 + 1e-12


def test_target_pause_and_completion_hover():
    guidance = SearchGuidance()
    guidance.start(0.0, 0.0, 0.0, 0.0)
    assert guidance.velocity((0.0, 0.0), 0.1, paused=True) == (0.0, 0.0)
    assert guidance.velocity((0.0, 0.0), 0.1) == pytest.approx((0.18, 0.0))
    for now in (7.0, 14.0, 21.0):
        guidance.velocity((10.0, 10.0), now)
    assert guidance.complete
    assert guidance.velocity((10.0, 10.0), 22.0) == (0.0, 0.0)


def test_waypoints_stay_inside_radius_and_speed_is_rejected_above_limit():
    guidance = SearchGuidance()
    guidance.start(0.0, 0.0, 0.4, 0.0)
    assert all(math.hypot(*point) <= 1.0 + 1e-9
               for point in guidance.waypoints)
    with pytest.raises(ValueError):
        SearchGuidance(speed_mps=0.18001)
