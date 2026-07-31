"""Pure tests for the structured V2 flight adapter."""

from types import SimpleNamespace

from uav_control.vision_v2_contract import VisionV2Adapter


def stamp(seconds):
    return SimpleNamespace(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))


def messages(sequence=1, valid=True, confidence=95.0, stamp_value=10.0):
    header = SimpleNamespace(stamp=stamp(stamp_value))
    tracked = SimpleNamespace(
        header=header, frame_sequence=sequence, capture_stamp_valid=True,
        detected=valid, confirmed=valid, measurement_valid=valid,
        confidence=confidence, error_x_norm=0.1, error_y_norm=-0.1,
        metric_valid=False)
    landing = SimpleNamespace(
        header=header, frame_sequence=sequence, capture_stamp_valid=True,
        valid=valid, confidence=confidence, error_x_norm=0.1,
        error_y_norm=-0.1, metric_valid=False)
    health = SimpleNamespace(
        camera_open=True, frames_received=True, algorithm_alive=True,
        protocol_ok=True, ready_for_closed_loop=True)
    return tracked, landing, health


def test_valid_structured_observation_uses_normalized_error_only():
    adapter = VisionV2Adapter()
    tracked, landing, health = messages()
    adapter.update_health(health, 1.0)
    adapter.update_tracked(tracked, 1.0)
    adapter.update_landing(landing, 1.0)
    observation = adapter.observation(1.1, source_now=10.1)
    assert observation.control_allowed
    assert observation.error_x_norm == 0.1
    assert not observation.metric_valid


def test_duplicate_and_out_of_order_sequences_do_not_refresh():
    adapter = VisionV2Adapter()
    tracked, landing, health = messages()
    adapter.update_health(health, 1.0)
    assert adapter.update_tracked(tracked, 1.0)
    assert adapter.update_landing(landing, 1.0)
    assert not adapter.update_landing(landing, 1.1)
    tracked_zero, _, _ = messages(sequence=0)
    assert not adapter.update_tracked(tracked_zero, 1.2)
    assert adapter.sequence.duplicate_count == 1
    assert adapter.sequence.out_of_order_count == 1


def test_invalid_confidence_health_and_source_timestamp_fail_closed():
    adapter = VisionV2Adapter()
    tracked, landing, health = messages(confidence=20.0)
    adapter.update_health(health, 1.0)
    adapter.update_tracked(tracked, 1.0)
    adapter.update_landing(landing, 1.0)
    assert not adapter.observation(1.1, source_now=10.1).control_allowed
    health.ready_for_closed_loop = False
    tracked.frame_sequence = 2
    landing.frame_sequence = 2
    adapter.update_health(health, 1.2)
    adapter.update_tracked(tracked, 1.2)
    adapter.update_landing(landing, 1.2)
    assert not adapter.observation(1.3, source_now=10.1).control_allowed
    health.ready_for_closed_loop = True
    tracked.frame_sequence = 3
    landing.frame_sequence = 3
    tracked.confidence = landing.confidence = 95.0
    tracked.header.stamp = landing.header.stamp = stamp(1.0)
    adapter.update_health(health, 1.4)
    adapter.update_tracked(tracked, 1.4)
    adapter.update_landing(landing, 1.4)
    assert not adapter.observation(1.5, source_now=10.1).control_allowed


def test_sequence_restart_requires_three_new_frames():
    adapter = VisionV2Adapter()
    tracked, landing, health = messages(sequence=9)
    adapter.update_health(health, 1.0)
    adapter.update_tracked(tracked, 1.0)
    adapter.update_landing(landing, 1.0)
    assert adapter.observation(1.1, source_now=10.1).control_allowed
    for sequence in (1, 2):
        tracked, landing, _ = messages(sequence=sequence)
        adapter.update_tracked(tracked, 1.2 + sequence * 0.1)
        adapter.update_landing(landing, 1.2 + sequence * 0.1)
        observation = adapter.observation(
            1.3 + sequence * 0.1, source_now=10.3 + sequence * 0.1)
        assert not observation.control_allowed
    tracked, landing, _ = messages(sequence=3)
    _, _, health = messages(sequence=3)
    adapter.update_health(health, 1.6)
    adapter.update_tracked(tracked, 1.6)
    adapter.update_landing(landing, 1.6)
    assert adapter.observation(1.7, source_now=10.1).control_allowed
