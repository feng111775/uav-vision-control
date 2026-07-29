# flake8: noqa
"""Stage B1 protocol, safety, deployment, and configuration regression tests."""
import importlib.util
import os
import pty
import socket
import sys
import tempfile
import time
from pathlib import Path

import pytest

from uav_control.hardware.car_link_protocol import (
    CarFrame, CarProtocolError, CarSequenceGuard, crc16_ccitt,
    decode_car_frame, encode_car_frame,
)
from uav_control.hardware.hardware_schema import (
    HardwareConfigurationError, UnsupportedTransport, validate_hardware_config,
)
from uav_control.hardware.payload_protocol import (
    PayloadAck, PayloadActionTracker, PayloadProtocolError, PayloadResult,
    decode_drop_ack, encode_drop, encode_drop_ack,
)
from uav_control.hardware.transports import BoundedLineBuffer, UdpReceiveTransport
from uav_control.integration.health_logic import (
    DISABLED, OK, STALE, HealthModel, WatchedSource, classify_disk,
    read_cpu_temperature,
)
from uav_control.integration.safety_logic import SafetyInputs, evaluate_safety

ROOT = Path(__file__).resolve().parents[3]


def load_script(relative, name):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def frame(sequence=1, progress=1, flags=1):
    return CarFrame(sequence, 1234, progress, flags)


def test_car_normal_frame():
    assert decode_car_frame(encode_car_frame(frame())) == frame()


def test_car_crc_error():
    with pytest.raises(CarProtocolError, match="CRC"):
        decode_car_frame(encode_car_frame(frame())[:-1] + "0")


def test_car_version_error():
    line = encode_car_frame(frame()).replace("CAR,1,", "CAR,2,", 1)
    with pytest.raises(CarProtocolError, match="version"):
        decode_car_frame(line)


@pytest.mark.parametrize("progress", [-1, 6])
def test_car_progress_range(progress):
    with pytest.raises(CarProtocolError):
        encode_car_frame(frame(progress=progress))


def test_sequence_duplicate():
    guard = CarSequenceGuard()
    guard.accept(frame())
    with pytest.raises(CarProtocolError, match="duplicate"):
        guard.accept(frame())


def test_sequence_backward():
    guard = CarSequenceGuard()
    guard.accept(frame(20))
    with pytest.raises(CarProtocolError, match="out-of-order"):
        guard.accept(frame(19))


def test_sequence_wrap():
    guard = CarSequenceGuard()
    guard.accept(frame(65535))
    guard.accept(frame(0))
    assert guard.last_sequence == 0


def test_progress_regression():
    guard = CarSequenceGuard()
    guard.accept(frame(1, 3))
    with pytest.raises(CarProtocolError, match="progress"):
        guard.accept(frame(2, 2))


@pytest.mark.parametrize("flags,expected", [(0, False), (1, True), (3, True)])
def test_mission_start_flag(flags, expected):
    assert frame(flags=flags).mission_start is expected


def test_car_overlong():
    with pytest.raises(CarProtocolError, match="long"):
        decode_car_frame(b"A" * 257)


def test_udp_source_limit():
    receiver = UdpReceiveTransport("127.0.0.1", 0, "192.0.2.1")
    port = receiver.socket.getsockname()[1]
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sender.sendto(encode_car_frame(frame()).encode(), ("127.0.0.1", port))
    time.sleep(0.01)
    assert receiver.read() is None
    receiver.close()


def test_udp_localhost_receive():
    receiver = UdpReceiveTransport("127.0.0.1", 0, "127.0.0.1")
    port = receiver.socket.getsockname()[1]
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sender.sendto(encode_car_frame(frame()).encode(), ("127.0.0.1", port))
    for _ in range(20):
        item = receiver.read()
        if item:
            break
        time.sleep(0.01)
    assert decode_car_frame(item[0]).progress == 1
    receiver.close()


def test_bounded_serial_reconnect_buffer():
    buffer = BoundedLineBuffer(8)
    assert buffer.feed(b"CAR\n") == [b"CAR"]
    buffer.feed(b"0123456789")
    assert buffer.dropped == 1


def test_pseudo_terminal_line():
    pytest.importorskip("serial")
    import serial
    master, slave = pty.openpty()
    port = serial.Serial(os.ttyname(slave), 115200, timeout=0.2)
    os.write(master, b"HELLO\n")
    assert port.readline() == b"HELLO\n"
    port.close()


def test_payload_drop():
    assert encode_drop(7).startswith("DROP,1,7,")


@pytest.mark.parametrize("result", list(PayloadResult))
def test_payload_ack_roundtrip(result):
    ack = PayloadAck(8, result)
    assert decode_drop_ack(encode_drop_ack(ack)) == ack


def test_payload_bad_ack():
    with pytest.raises(PayloadProtocolError):
        decode_drop_ack("DROP_ACK,1,1,0,0000")


def test_payload_old_sequence():
    tracker = PayloadActionTracker()
    tracker.on_release(True, 0.0, True)
    assert not tracker.on_ack(PayloadAck(0, PayloadResult.SUCCESS))
    assert tracker.status == "STALE_ACK"


def test_payload_sustained_true_once():
    tracker = PayloadActionTracker()
    assert tracker.on_release(True, 0.0, True)
    assert tracker.on_release(True, 0.1, True) is None


def test_payload_false_no_action():
    assert PayloadActionTracker().on_release(False, 0.0, True) is None


def test_payload_timeout():
    tracker = PayloadActionTracker(timeout_seconds=1.0, retry_count=0)
    tracker.on_release(True, 0.0, True)
    assert tracker.poll(1.1) is None
    assert tracker.status == "TIMEOUT"


def test_payload_finite_retries():
    tracker = PayloadActionTracker(timeout_seconds=1.0, retry_count=2)
    tracker.on_release(True, 0.0, True)
    assert tracker.poll(1.1)
    assert tracker.poll(2.2)
    assert tracker.poll(3.3) is None
    assert tracker.attempts == 3


def test_payload_restart_idle():
    assert PayloadActionTracker().active_sequence is None


def test_payload_disabled_no_command():
    tracker = PayloadActionTracker()
    assert tracker.on_release(True, 0.0, False) is None
    assert tracker.status == "DISABLED"


@pytest.mark.parametrize("transport", ["disabled", "mock", "serial"])
def test_payload_supported_transport(transport):
    values = {"transport": transport}
    if transport == "serial":
        values.update(device="/dev/test", baudrate=1)
    assert validate_hardware_config(
        values, supported={"disabled", "mock", "serial"}
    ).transport == transport


def test_unsupported_transport():
    with pytest.raises(UnsupportedTransport):
        validate_hardware_config({"transport": "udp"}, supported={"disabled"})


def test_serial_incomplete_rejected():
    with pytest.raises(HardwareConfigurationError):
        validate_hardware_config({"transport": "serial"})


def test_udp_incomplete_rejected():
    with pytest.raises(HardwareConfigurationError):
        validate_hardware_config({"transport": "udp"})


def test_identity_incomplete_rejected():
    with pytest.raises(HardwareConfigurationError):
        validate_hardware_config({
            "transport": "disabled", "enforce_device_identity": True
        })


def test_health_px4_fresh_and_timeout():
    model = HealthModel({"px4": WatchedSource(1.0)})
    model.observe("px4", 1.0)
    assert model.snapshot(1.5)["px4"] == OK
    assert model.snapshot(2.1)["px4"] == STALE


def test_health_disabled():
    model = HealthModel({"car": WatchedSource(1.0, enabled=False)})
    assert model.snapshot(10.0)["car"] == DISABLED


def test_vision_no_target_is_fresh():
    model = HealthModel({"vision": WatchedSource(1.0)})
    model.observe("vision", 2.0, "valid=0")
    assert model.snapshot(2.5)["vision"] == OK


def test_disk_low_warns():
    assert classify_disk(0.01) == "WARN"


def test_temperature_failure_unknown():
    assert read_cpu_temperature("/does/not/exist")[0] == "UNKNOWN"


def ready_inputs(mode="hover_test"):
    return SafetyInputs(
        mode=mode, operator_enabled=True, operator_fresh=True,
        px4_fresh=True, position_fresh=True, attitude_fresh=True,
        competition_configured=True,
        subsystem={"vision": "OK", "car": "ACTIVE", "payload": "READY"},
    )


def test_safety_default_false():
    assert not evaluate_safety(SafetyInputs())[0]


def test_safety_operator_stale():
    data = ready_inputs()
    data.operator_fresh = False
    assert evaluate_safety(data)[1].startswith("OPERATOR")


def test_hover_allows_disabled_car_payload():
    data = ready_inputs()
    data.subsystem = {"car": "DISABLED", "payload": "DISABLED"}
    assert evaluate_safety(data)[0]


def test_drop_requires_payload():
    data = ready_inputs("drop")
    data.subsystem["payload"] = "DISABLED"
    assert evaluate_safety(data)[1] == "PAYLOAD_NOT_READY"


@pytest.mark.parametrize("missing", ["car", "vision"])
def test_dynamic_requires_links(missing):
    data = ready_inputs("dynamic_land")
    data.subsystem[missing] = "DISABLED"
    assert not evaluate_safety(data)[0]


def test_failsafe_blocks():
    data = ready_inputs()
    data.failsafe = True
    assert evaluate_safety(data)[1] == "PX4_FAILSAFE"


def test_position_timeout_blocks():
    data = ready_inputs()
    data.position_fresh = False
    assert evaluate_safety(data)[1] == "PX4_DATA_STALE"


def test_competition_incomplete_blocks():
    data = ready_inputs("drop")
    data.competition_configured = False
    assert evaluate_safety(data)[1] == "HARDWARE_CONFIGURATION_INCOMPLETE"


def test_observe_never_ready():
    assert evaluate_safety(ready_inputs("observe")) == (False, "OBSERVE_MODE")


def test_manifest_excludes_products_and_data():
    sys.path.insert(0, str(ROOT / "scripts/pi"))
    from deployment_manifest import tracked_manifest
    names = [name for name, _ in tracked_manifest(str(ROOT))]
    assert not any(name.startswith(("build", "install", "log", "datasets/", "models/"))
                   for name in names)


def test_systemd_has_no_auto_arm():
    text = "\n".join(path.read_text() for path in
                     (ROOT / "deploy/systemd").glob("*.service"))
    assert "enable_auto_arm:=true" not in text
    assert "Restart=no" in text


def test_deploy_dry_run_does_not_create_target():
    target = Path(tempfile.gettempdir()) / "d-task-never-create-test"
    if target.exists():
        pytest.skip("fixed dry-run target already exists")
    result = os.system(
        f"{ROOT}/scripts/pi/deploy_workspace.sh --target {target} >/dev/null"
    )
    assert result == 0 and not target.exists()


def test_crc_known_vector():
    assert crc16_ccitt(b"123456789") == 0x29B1
