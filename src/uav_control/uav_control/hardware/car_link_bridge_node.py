# flake8: noqa
# noqa: D100,D101,D102,D103,D104,D105,D107
"""Receive authenticated car progress without ever controlling the car."""
import json
import socket
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from std_msgs.msg import Bool, String, UInt8

from .car_link_protocol import CarProtocolError, CarSequenceGuard, decode_car_frame
from .hardware_schema import validate_hardware_config
from .transports import BoundedLineBuffer, UdpReceiveTransport


class CarLinkBridge(Node):  # noqa: D101
    def __init__(self) -> None:  # noqa: D107
        super().__init__("car_link_bridge_node")
        defaults = {
            "transport": "disabled", "device": "", "baudrate": 0,
            "bind_address": "", "bind_port": 0, "allowed_remote_ip": "",
            "replay_file": "", "timeout_seconds": 1.0, "retry_count": 2,
            "max_line_bytes": 256, "enforce_device_identity": False,
            "expected_vid": "", "expected_pid": "", "expected_serial": "",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        values = {name: self.get_parameter(name).value for name in defaults}
        self.cfg = validate_hardware_config(
            values, supported={"disabled", "serial", "udp", "replay"}
        )
        self.start_pub = self.create_publisher(Bool, "/car/mission_start", 10)
        self.progress_pub = self.create_publisher(UInt8, "/car/progress", 10)
        self.status_pub = self.create_publisher(String, "/car/link/status", 10)
        self.diag_pub = self.create_publisher(
            DiagnosticArray, "/car/link/diagnostics", 10
        )
        self.guard = CarSequenceGuard()
        self.received = self.errors = 0
        self.last_rx = 0.0
        self.last_remote = ""
        self.transport = None
        self.serial = None
        self.buffer = BoundedLineBuffer(int(values["max_line_bytes"]))
        self.replay = []
        self.replay_start = time.monotonic()
        self.replay_index = 0
        self._open()
        self.create_timer(0.02, self._poll)
        self.create_timer(0.5, self._publish_status)

    def _open(self) -> None:
        if self.cfg.transport == "udp":
            self.transport = UdpReceiveTransport(
                self.cfg.bind_address, self.cfg.bind_port, self.cfg.allowed_remote_ip
            )
        elif self.cfg.transport == "serial":
            try:
                import serial
                self.serial = serial.Serial(
                    self.cfg.device, self.cfg.baudrate, timeout=0
                )
            except Exception as exc:  # reconnect occurs from timer
                self.get_logger().warning(f"car serial unavailable: {exc}")
        elif self.cfg.transport == "replay":
            with open(self.cfg.replay_file, encoding="utf-8") as stream:
                for line in stream:
                    item = json.loads(line)
                    self.replay.append((float(item["at_seconds"]), item["frame"]))

    def _poll(self) -> None:
        if self.cfg.transport == "disabled":
            return
        if self.cfg.transport == "udp":
            item = self.transport.read()
            if item:
                self.last_remote = f"{item[1][0]}:{item[1][1]}"
                self._consume(item[0])
        elif self.cfg.transport == "serial":
            if self.serial is None:
                self._open()
                return
            try:
                for line in self.buffer.feed(self.serial.read(512)):
                    self._consume(line)
            except Exception:
                self.serial = None
        elif self.replay_index < len(self.replay):
            elapsed = time.monotonic() - self.replay_start
            while self.replay_index < len(self.replay):
                at, frame = self.replay[self.replay_index]
                if at > elapsed:
                    break
                self._consume(frame)
                self.replay_index += 1

    def _consume(self, line: str | bytes) -> None:
        try:
            frame = decode_car_frame(line, self.buffer.maximum)
            self.guard.accept(frame)
        except CarProtocolError as exc:
            self.errors += 1
            self.get_logger().warning(f"rejected car frame: {exc}")
            return
        self.received += 1
        self.last_rx = time.monotonic()
        self.start_pub.publish(Bool(data=frame.mission_start))
        self.progress_pub.publish(UInt8(data=frame.progress))

    def _publish_status(self) -> None:
        now = time.monotonic()
        fresh = self.last_rx and now - self.last_rx <= self.cfg.timeout_seconds
        state = "DISABLED" if self.cfg.transport == "disabled" else (
            "ACTIVE" if fresh else "STALE"
        )
        text = {
            "state": state, "transport": self.cfg.transport,
            "frames": self.received, "errors": self.errors,
            "progress": self.guard.last_progress, "remote": self.last_remote,
        }
        self.status_pub.publish(String(data=json.dumps(text, separators=(",", ":"))))
        status = DiagnosticStatus(
            level=(DiagnosticStatus.OK if state == "ACTIVE" else
                   DiagnosticStatus.STALE if state == "STALE" else
                   DiagnosticStatus.WARN),
            name="car_link", message=state, hardware_id="car_link",
            values=[KeyValue(key=k, value=str(v)) for k, v in text.items()],
        )
        message = DiagnosticArray(status=[status])
        message.header.stamp = self.get_clock().now().to_msg()
        self.diag_pub.publish(message)


def main(args=None) -> None:  # noqa: D103
    rclpy.init(args=args)
    node = CarLinkBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
