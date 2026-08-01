from uav_vision.closed_loop_gate import ClosedLoopGate, GateSample


def sample(**kwargs):
    values = dict(
        enabled=True, camera_open=True, frames_received=True,
        algorithm_alive=True, protocol_ok=True, performance_ok=True,
        orientation_ok=True, mapping_ok=True,
        camera_mount_profile='openmv_downward_v1',
        data_age_ok=True, capture_stamp_ok=True,
    )
    values.update(kwargs)
    return GateSample(**values)


def test_gate_requires_warmup_time_and_frames():
    gate = ClosedLoopGate(warmup_sec=3.0, warmup_frames=3)
    assert not gate.update(0.0, sample())
    assert not gate.update(1.0, sample())
    assert not gate.update(2.0, sample())
    assert gate.update(3.0, sample())


def test_gate_hard_fault_closes_immediately():
    gate = ClosedLoopGate(warmup_sec=0.0, warmup_frames=1)
    assert gate.update(0.0, sample())
    assert not gate.update(0.1, sample(stale=True))
    assert not gate.ready


def test_gate_soft_failure_needs_consecutive_failures():
    gate = ClosedLoopGate(
        warmup_sec=0.0, warmup_frames=1, soft_failure_count=3)
    assert gate.update(0.0, sample())
    assert gate.update(0.1, sample(performance_ok=False))
    assert gate.update(0.2, sample(performance_ok=False))
    assert not gate.update(0.3, sample(performance_ok=False))
