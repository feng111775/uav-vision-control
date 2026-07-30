import json
import math
from pathlib import Path

import pytest

from uav_control.mission_logic import OdomFrameGate


PACKAGE_ROOT = Path(__file__).parents[1]


def feed_valid_window(gate, start_us=1_000_000, start_received=0.0):
    for index in range(21):
        assert gate.update(
            start_us + index * 50_000,
            start_received + index * 0.05,
            True)


def test_a_distinct_increasing_timestamps_pass_count_and_time_window():
    gate = OdomFrameGate()
    feed_valid_window(gate)
    assert gate.consecutive_frames == 21
    assert gate.ready


def test_b_same_timestamp_repeated_100_times_counts_once(capsys):
    gate = OdomFrameGate()
    callbacks = 101
    assert gate.update(1_000_000, 0.0, True)
    for index in range(100):
        assert not gate.update(1_000_000, index * 0.001, True)
    evidence = {
        'callbacks': callbacks,
        'new_frames_counted': gate.consecutive_frames,
        'consecutive_frames': gate.consecutive_frames,
        'ready': gate.ready,
    }
    print(json.dumps(evidence, sort_keys=True))
    assert json.loads(capsys.readouterr().out) == {
        'callbacks': 101,
        'consecutive_frames': 1,
        'new_frames_counted': 1,
        'ready': False,
    }


def test_c_invalid_frame_clears_consecutive_window():
    gate = OdomFrameGate()
    for index in range(10):
        gate.update(1_000_000 + index * 50_000, index * 0.05, True)
    assert not gate.update(1_500_000, 0.5, False)
    assert gate.consecutive_frames == 0
    assert not gate.ready


def test_d_stale_window_expires_and_requires_restart():
    gate = OdomFrameGate(timeout_seconds=0.5)
    for index in range(10):
        gate.update(1_000_000 + index * 50_000, index * 0.05, True)
    assert gate.expire(1.0)
    assert gate.consecutive_frames == 0
    assert not gate.ready


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
def test_e_nonfinite_kinematics_are_invalid_by_contract(value):
    values = (0.0, 0.0, 0.0, 0.0, 0.0, value, 0.0)
    valid = all(math.isfinite(item) for item in values)
    gate = OdomFrameGate()
    gate.update(1_000_000, 0.0, valid)
    assert gate.consecutive_frames == 0


def test_f_recovery_after_invalid_restarts_full_window():
    gate = OdomFrameGate()
    for index in range(19):
        gate.update(1_000_000 + index * 50_000, index * 0.05, True)
    gate.update(2_000_000, 1.0, False)
    feed_valid_window(gate, 3_000_000, 1.1)
    assert gate.consecutive_frames == 21
    assert gate.first_source_timestamp == 3_000_000
    assert gate.ready


def test_g_equal_and_backwards_timestamps_do_not_increment():
    gate = OdomFrameGate()
    assert gate.update(2_000_000, 0.0, True)
    assert not gate.update(2_000_000, 0.1, True)
    assert not gate.update(1_999_999, 0.2, True)
    assert gate.consecutive_frames == 1


def test_h_duplicate_timestamp_does_not_refresh_new_frame_freshness():
    gate = OdomFrameGate(timeout_seconds=0.5)
    gate.update(2_000_000, 0.0, True)
    gate.update(2_000_000, 0.4, True)
    assert gate.last_new_frame_received == 0.0
    gate.update(2_000_000, 0.51, True)
    assert gate.consecutive_frames == 0
    assert gate.last_new_frame_received is None


def test_i_sitl_and_hardware_heading_rules_are_explicit():
    source = (
        PACKAGE_ROOT / 'uav_control' /
        'mission_controller_node.py').read_text(encoding='utf-8')
    assert 'self.logic.simulation_mode and' in source
    assert "'allow_sitl_heading_quality_bypass'" in source
    assert '(msg.heading_good_for_control or bypass)' in source
    assert 'all(math.isfinite(value) for value in values)' in source


def test_j_prestream_loss_participates_in_safe_data_loss_exit():
    source = (
        PACKAGE_ROOT / 'uav_control' /
        'mission_controller_node.py').read_text(encoding='utf-8')
    assert 'not self.logic.odom_ready' in source
    assert 'data_loss_action(self.logic.armed, status_stale)' in source
    assert "'PX4_DATA_TIMEOUT_LAND'" in source


def test_timestamp_sample_preferred_with_timestamp_fallback():
    source = (
        PACKAGE_ROOT / 'uav_control' /
        'mission_controller_node.py').read_text(encoding='utf-8')
    assert 'msg.timestamp_sample if msg.timestamp_sample != 0' in source
    assert 'else msg.timestamp' in source


def test_gate_rejects_weakened_safety_minimums():
    with pytest.raises(ValueError):
        OdomFrameGate(min_frames=19)
    with pytest.raises(ValueError):
        OdomFrameGate(min_source_span_seconds=0.999)
