"""Tests for dynamic vision readiness gating."""

from uav_vision.closed_loop_health import ClosedLoopHealthGate, HealthSample


def sample(now, **kwargs):
    values = {
        'now': now, 'camera_open': True, 'frames_received': True,
        'algorithm_alive': True, 'protocol_ok': True,
        'ready_for_mission': True, 'performance_gate_passed': True,
        'calibration_loaded': True, 'measured_fps': 6.0,
        'last_measurement_age_ms': 20.0, 'receive_age_s': 0.02,
        'recent_protocol_errors': 0, 'recent_duplicate_count': 0,
        'recent_out_of_order_count': 0, 'recent_stalled': False,
    }
    values.update(kwargs)
    return HealthSample(**values)


def test_enable_and_two_second_stable_window_required():
    gate = ClosedLoopHealthGate(min_fps=5.0, stable_seconds=2.0)
    gate.observe_frame(0.0)
    for index in range(1, 10):
        gate.observe_frame(index * 0.1)
    assert not gate.evaluate(sample(1.9), closed_loop_enable=True)
    for index in range(10, 11):
        gate.observe_frame(index * 0.1)
    assert gate.evaluate(sample(2.1), closed_loop_enable=True)


def test_stop_or_error_closes_gate_immediately():
    gate = ClosedLoopHealthGate(min_fps=5.0, stable_seconds=2.0)
    for index in range(21):
        gate.observe_frame(index * 0.1)
    assert gate.evaluate(sample(2.1), closed_loop_enable=True)
    assert not gate.evaluate(sample(2.2, recent_stalled=True),
                             closed_loop_enable=True)


def test_closed_loop_defaults_to_disabled():
    gate = ClosedLoopHealthGate()
    gate.observe_frame(0.0)
    assert not gate.evaluate(sample(3.0), closed_loop_enable=False)
