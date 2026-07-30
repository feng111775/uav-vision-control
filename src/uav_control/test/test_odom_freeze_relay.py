"""Tests for the selective odometry relay forwarding gate."""

from px4_msgs.msg import VehicleLocalPosition

from uav_control.odom_freeze_relay import RelayGate


class Clock:
    """Deterministic monotonic clock for relay evidence tests."""

    def __init__(self):
        """Initialize the test clock."""
        self.value = 0.0

    def __call__(self):
        """Advance and return the test clock."""
        self.value += 0.01
        return self.value


def message(timestamp):
    """Create a populated local-position message."""
    msg = VehicleLocalPosition()
    msg.timestamp = timestamp + 1
    msg.timestamp_sample = timestamp
    msg.x, msg.y, msg.z = 1.0, 2.0, -3.0
    msg.vx, msg.vy, msg.vz = 4.0, 5.0, 6.0
    msg.heading = 0.7
    msg.xy_valid = msg.z_valid = True
    msg.v_xy_valid = msg.v_z_valid = True
    msg.xy_reset_counter = 2
    msg.z_reset_counter = 3
    return msg


def test_default_forwards_same_complete_message_object_once():
    """The default gate forwards the exact complete object once."""
    gate = RelayGate(Clock())
    source = message(100)
    output = gate.process(source)
    assert output is source
    assert output == source
    assert gate.input_count == gate.output_count == 1
    assert gate.last_input_timestamp_sample == 100
    assert gate.last_output_timestamp_sample == 100


def test_freeze_counts_inputs_and_stops_all_outputs():
    """A freeze counts raw input while producing no relay output."""
    gate = RelayGate(Clock())
    gate.process(message(100))
    assert gate.set_frozen(True)
    assert gate.process(message(101)) is None
    assert gate.process(message(102)) is None
    assert gate.input_count == 3
    assert gate.output_count == 1
    assert gate.last_input_timestamp_sample == 102
    assert gate.last_output_timestamp_sample == 100


def test_unfreeze_forwards_only_next_new_input_without_backlog():
    """Unfreezing forwards only newly received messages."""
    gate = RelayGate(Clock())
    gate.process(message(100))
    gate.set_frozen(True)
    gate.process(message(101))
    gate.process(message(102))
    gate.set_frozen(False)
    assert gate.output_count == 1
    newest = message(103)
    assert gate.process(newest) is newest
    assert gate.output_count == 2
    assert gate.last_output_timestamp_sample == 103


def test_repeated_freeze_requests_do_not_duplicate_or_change_state():
    """Repeated state requests do not create extra transitions."""
    gate = RelayGate(Clock())
    assert gate.set_frozen(True)
    started = gate.freeze_started
    assert not gate.set_frozen(True)
    assert gate.freeze_started == started
    assert gate.set_frozen(False)
    ended = gate.freeze_ended
    assert not gate.set_frozen(False)
    assert gate.freeze_ended == ended
    assert gate.process(message(200)).timestamp_sample == 200
