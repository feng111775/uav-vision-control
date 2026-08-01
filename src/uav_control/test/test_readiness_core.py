import pytest

from uav_control.readiness_core import ReadinessGate


def healthy_update(gate, now, heading_good=True):
    gate.update_status(now, True, False, True)
    gate.update_position(
        now, True, True, True, True, heading_good, 0.2)
    gate.update_attitude(now, (1.0, 0.0, 0.0, 0.0))


def all_topics(value=True):
    return {name: value for name in ReadinessGate.STREAMS}


def test_no_topics_never_passes():
    gate = ReadinessGate()
    ready, reasons = gate.evaluate(10.0, all_topics(False))
    assert not ready
    assert 'status_topic_missing' in reasons


def test_partial_topics_never_pass():
    gate = ReadinessGate()
    healthy_update(gate, 0.0)
    present = all_topics()
    present['attitude'] = False
    assert not gate.evaluate(3.1, present)[0]


def test_all_topics_must_be_continuously_stable_for_three_seconds():
    gate = ReadinessGate(stable_seconds=3.0)
    for now in (0.0, 1.0, 2.0, 2.9):
        healthy_update(gate, now)
        assert not gate.evaluate(now, all_topics())[0]
    healthy_update(gate, 3.0)
    assert gate.evaluate(3.0, all_topics())[0]


def test_stream_interruption_resets_stability_window():
    gate = ReadinessGate(stable_seconds=3.0)
    healthy_update(gate, 0.0)
    gate.evaluate(0.0, all_topics())
    healthy_update(gate, 2.0)
    gate.evaluate(2.0, all_topics())
    present = all_topics()
    present['position'] = False
    assert not gate.evaluate(2.1, present)[0]
    healthy_update(gate, 2.2)
    assert not gate.evaluate(5.1, all_topics())[0]
    healthy_update(gate, 5.2)
    assert not gate.evaluate(5.2, all_topics())[0]
    healthy_update(gate, 8.21)
    assert gate.evaluate(8.21, all_topics())[0]


@pytest.mark.parametrize(
    'preflight,failsafe,disarmed',
    [(False, False, True), (True, True, True), (True, False, False)])
def test_unsafe_status_never_passes(preflight, failsafe, disarmed):
    gate = ReadinessGate(stable_seconds=0.0)
    gate.update_status(0.0, preflight, failsafe, disarmed)
    gate.update_position(0.0, True, True, True, True, True, 0.0)
    gate.update_attitude(0.0, (1.0, 0.0, 0.0, 0.0))
    assert not gate.evaluate(0.0, all_topics())[0]


def test_hardware_bad_heading_never_passes():
    gate = ReadinessGate(stable_seconds=0.0, simulation_mode=False)
    healthy_update(gate, 0.0, heading_good=False)
    assert not gate.evaluate(0.0, all_topics())[0]


def test_explicit_sitl_heading_bypass_is_allowed():
    gate = ReadinessGate(
        stable_seconds=0.0, simulation_mode=True,
        allow_sitl_heading_quality_bypass=True)
    healthy_update(gate, 0.0, heading_good=False)
    assert gate.evaluate(0.0, all_topics())[0]


def test_hardware_heading_bypass_configuration_is_rejected():
    with pytest.raises(ValueError):
        ReadinessGate(
            simulation_mode=False,
            allow_sitl_heading_quality_bypass=True)


@pytest.mark.parametrize(
    'stream,advance',
    [('status', 1.01), ('position', 0.51), ('attitude', 0.51)])
def test_each_message_age_limit_is_enforced(stream, advance):
    gate = ReadinessGate(stable_seconds=0.0)
    healthy_update(gate, 0.0)
    gate.received[stream] = 0.0
    for other in set(ReadinessGate.STREAMS) - {stream}:
        gate.received[other] = advance
    ready, reasons = gate.evaluate(advance, all_topics())
    assert not ready
    assert stream + '_stale' in reasons


@pytest.mark.parametrize(
    'quaternion',
    [(float('nan'), 0.0, 0.0, 1.0), (2.0, 0.0, 0.0, 0.0)])
def test_invalid_attitude_never_passes(quaternion):
    gate = ReadinessGate(stable_seconds=0.0)
    healthy_update(gate, 0.0)
    gate.update_attitude(0.0, quaternion)
    assert not gate.evaluate(0.0, all_topics())[0]
