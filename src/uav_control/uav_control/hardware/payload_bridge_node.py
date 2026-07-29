# flake8: noqa
# noqa: D100,D101,D102,D103,D104,D105,D107
"""Finite-retry payload actuator bridge."""
import json
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .hardware_schema import validate_hardware_config
from .payload_protocol import PayloadActionTracker, PayloadProtocolError, decode_drop_ack
from .transports import BoundedLineBuffer


class PayloadBridge(Node):  # noqa: D101
    def __init__(self) -> None:  # noqa: D107
        super().__init__("payload_bridge_node")
        defaults = {
            "transport": "disabled", "device": "", "baudrate": 0,
            "timeout_seconds": 1.0, "retry_count": 2,
            "allow_actions": False, "enforce_device_identity": False,
            "expected_vid": "", "expected_pid": "", "expected_serial": "",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        values = {name: self.get_parameter(name).value for name in defaults}
        self.cfg = validate_hardware_config(
            values, supported={"disabled", "mock", "serial"}, action_adapter=True
        )
        self.tracker = PayloadActionTracker(
            self.cfg.timeout_seconds, self.cfg.retry_count
        )
        self.serial = None
        self.buffer = BoundedLineBuffer()
        self.ack_pub = self.create_publisher(Bool, "/uav/payload/release_ack", 10)
        self.status_pub = self.create_publisher(String, "/uav/payload/status", 10)
        self.diag_pub = self.create_publisher(
            DiagnosticArray, "/uav/payload/diagnostics", 10
        )
        self.mock_command_pub = self.create_publisher(
            String, "/uav/payload/mock_command", 10
        )
        self.create_subscription(Bool, "/uav/payload/release", self._release, 10)
        self.create_subscription(
            String, "/uav/payload/mock_ack", self._mock_ack, 10
        )
        self._open()
        self.create_timer(0.02, self._poll)
        self.create_timer(0.5, self._status)

    def _open(self) -> None:
        if self.cfg.transport != "serial":
            return
        try:
            import serial
            self.serial = serial.Serial(self.cfg.device, self.cfg.baudrate, timeout=0)
        except Exception as exc:
            self.serial = None
            self.get_logger().warning(f"payload serial unavailable: {exc}")

    def _release(self, msg: Bool) -> None:
        command = self.tracker.on_release(
            msg.data, time.monotonic(),
            self.cfg.allow_actions and self.cfg.transport != "disabled",
        )
        if command:
            self._send(command)

    def _send(self, command: str) -> None:
        if self.cfg.transport == "mock":
            self.mock_command_pub.publish(String(data=command))
        elif self.cfg.transport == "serial" and self.serial is not None:
            self.serial.write((command + "\n").encode("ascii"))

    def _mock_ack(self, msg: String) -> None:
        if self.cfg.transport == "mock":
            self._accept_ack(msg.data)

    def _accept_ack(self, line: str | bytes) -> None:
        try:
            successful = self.tracker.on_ack(decode_drop_ack(line))
        except PayloadProtocolError as exc:
            self.get_logger().warning(f"rejected payload ack: {exc}")
            return
        if successful:
            self.ack_pub.publish(Bool(data=True))

    def _poll(self) -> None:
        command = self.tracker.poll(time.monotonic())
        if command:
            self._send(command)
        if self.cfg.transport == "serial":
            if self.serial is None:
                self._open()
                return
            try:
                for line in self.buffer.feed(self.serial.read(512)):
                    self._accept_ack(line)
            except Exception:
                self.serial = None

    def _status(self) -> None:
        state = "DISABLED" if self.cfg.transport == "disabled" else self.tracker.status
        data = {
            "state": state, "transport": self.cfg.transport,
            "sequence": self.tracker.active_sequence,
            "attempts": self.tracker.attempts,
        }
        self.status_pub.publish(String(data=json.dumps(data, separators=(",", ":"))))
        level = DiagnosticStatus.WARN if state == "DISABLED" else (
            DiagnosticStatus.ERROR if state in {"TIMEOUT", "DENIED", "MECHANICAL_ERROR",
                                                 "SENSOR_ERROR", "UNKNOWN_ERROR"}
            else DiagnosticStatus.OK
        )
        self.diag_pub.publish(DiagnosticArray(status=[DiagnosticStatus(
            level=level, name="payload", message=state, hardware_id="payload",
            values=[KeyValue(key=k, value=str(v)) for k, v in data.items()],
        )]))


def main(args=None) -> None:  # noqa: D103
    rclpy.init(args=args)
    node = PayloadBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
