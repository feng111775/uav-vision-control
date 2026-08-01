"""Pure tests for the replaceable legacy vision adapter."""

import math

import pytest

from uav_control.vision_contract import (
    LegacyVisionAdapter, make_legacy_aligned, make_legacy_no_target)


def valid(adapter, now=1.0):
    tracked, landing = make_legacy_aligned()
    adapter.update_tracked(tracked, now)
    adapter.update_landing(landing, now)
    return adapter.observation(now + 0.01)


def test_valid_observation_is_normalized():
    observation = valid(LegacyVisionAdapter())
    assert observation.control_allowed
    assert observation.target_valid
    assert observation.confidence == 95.0


def test_no_target_is_distinct_from_source_timeout():
    adapter = LegacyVisionAdapter()
    tracked, landing = make_legacy_no_target()
    adapter.update_tracked(tracked, 1.0)
    adapter.update_landing(landing, 1.0)
    no_target = adapter.observation(1.01)
    offline = adapter.observation(2.0)
    assert no_target.source_alive and not no_target.target_valid
    assert not offline.source_alive


@pytest.mark.parametrize('values', [
    ([1.0], [1.0] * 8),
    ([1.0] * 12, [1.0, 0.0, math.nan] + [0.0] * 5),
])
def test_malformed_or_nonfinite_input_is_rejected(values):
    adapter = LegacyVisionAdapter()
    adapter.update_tracked(values[0], 1.0)
    adapter.update_landing(values[1], 1.0)
    observation = adapter.observation(1.01)
    assert not observation.control_allowed
    assert not observation.finite_values


def test_stale_timestamp_is_not_control_eligible():
    adapter = LegacyVisionAdapter()
    tracked, landing = make_legacy_aligned(target_age_ms=5000.0)
    adapter.update_tracked(tracked, 1.0)
    adapter.update_landing(landing, 1.0)
    assert not adapter.observation(1.01).control_allowed


def test_last_error_is_not_reused_after_source_timeout():
    adapter = LegacyVisionAdapter()
    valid(adapter, 1.0)
    observation = adapter.observation(2.0)
    assert not observation.control_allowed
    assert observation.horizontal_error_x == 0.0
