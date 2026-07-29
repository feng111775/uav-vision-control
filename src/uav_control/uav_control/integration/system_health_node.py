# flake8: noqa
# noqa: D100,D101,D102,D103,D104,D105,D107
"""Read-only health monitor for PX4, vision, links, task, and Pi resources."""
import json
import shutil
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, String

from .health_logic import (
    HealthModel, WatchedSource, classify_disk, read_clock_sync,
    read_cpu_temperature, read_memory_usage,
)


class SystemHealthNode(Node):  # noqa: D101
    def __init__(self) -> None:  # noqa: D107
        super().__init__("system_health_node")
        self.declare_parameter("px4_timeout_seconds", 1.0)
        self.declare_parameter("vision_timeout_seconds", 1.0)
        self.declare_parameter("disk_warn_fraction", 0.15)
        px4_timeout = float(self.get_parameter("px4_timeout_seconds").value)
        vision_timeout = float(self.get_parameter("vision_timeout_seconds").value)
        self.model = HealthModel({
            "px4_status": WatchedSource(px4_timeout),
            "px4_position": WatchedSource(px4_timeout),
            "px4_attitude": WatchedSource(px4_timeout),
            "vision_detection": WatchedSource(vision_timeout),
            "vision_tracked": WatchedSource(vision_timeout),
            "vision_error": WatchedSource(vision_timeout),
        })
        self.car_state = self.payload_state = "DISABLED"
        self.vision_h7_state = "UNKNOWN"
        self.target_valid = False
        self.target_age_ms = None
        self.mission_state = self.mission_event = "UNKNOWN"
        self.px4_failsafe = self.px4_armed = self.px4_offboard = False
        self.started = time.monotonic()
        self.diag_pub = self.create_publisher(
            DiagnosticArray, "/system/diagnostics", 10
        )
        self.status_pub = self.create_publisher(String, "/system/status", 10)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )
        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status_v1", self._px4_status, px4_qos
        )
        self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position",
            lambda _: self._seen("px4_position"), px4_qos
        )
        self.create_subscription(
            VehicleAttitude, "/fmu/out/vehicle_attitude",
            lambda _: self._seen("px4_attitude"), px4_qos
        )
        for topic, name in (
            ("/vision/h7/detection", "vision_detection"),
            ("/vision/landing_error", "vision_error"),
        ):
            self.create_subscription(
                Float32MultiArray, topic, lambda _, n=name: self._seen(n), 10
            )
        self.create_subscription(
            Float32MultiArray, "/vision/target/tracked", self._tracked, 10
        )
        self.create_subscription(
            String, "/vision/h7/status",
            lambda msg: setattr(self, "vision_h7_state", msg.data), 10
        )
        self.create_subscription(
            String, "/car/link/status", lambda msg: setattr(
                self, "car_state", self._json_state(msg.data)
            ), 10
        )
        self.create_subscription(
            String, "/uav/payload/status", lambda msg: setattr(
                self, "payload_state", self._json_state(msg.data)
            ), 10
        )
        self.create_subscription(
            String, "/uav/mission/state",
            lambda msg: setattr(self, "mission_state", msg.data), 10
        )
        self.create_subscription(
            String, "/uav/mission/event",
            lambda msg: setattr(self, "mission_event", msg.data), 10
        )
        self.create_timer(0.5, self._publish)

    @staticmethod
    def _json_state(data: str) -> str:
        try:
            return str(json.loads(data).get("state", "UNKNOWN"))
        except (ValueError, AttributeError):
            return data or "UNKNOWN"

    def _seen(self, name: str) -> None:
        self.model.observe(name, time.monotonic())

    def _px4_status(self, msg: VehicleStatus) -> None:
        self._seen("px4_status")
        self.px4_failsafe = bool(msg.failsafe)
        self.px4_armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.px4_offboard = (
            msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

    def _tracked(self, msg: Float32MultiArray) -> None:
        """Record freshness and schema-defined target validity/age."""
        from uav_vision.d_task_schema import TARGET_AGE_MS, TRACKED_LENGTH, VALID
        self._seen("vision_tracked")
        if len(msg.data) == TRACKED_LENGTH:
            self.target_valid = msg.data[VALID] == 1.0
            self.target_age_ms = float(msg.data[TARGET_AGE_MS])

    def _publish(self) -> None:
        now = time.monotonic()
        states = self.model.snapshot(now)
        states["car"] = self.car_state
        states["payload"] = self.payload_state
        temp_state, temperature = read_cpu_temperature()
        memory_state, memory_fraction = read_memory_usage()
        disk = shutil.disk_usage("/")
        disk_fraction = disk.free / disk.total
        states["cpu_temperature"] = temp_state
        states["memory"] = memory_state
        states["clock"] = read_clock_sync()
        states["disk"] = classify_disk(
            disk_fraction, float(self.get_parameter("disk_warn_fraction").value)
        )
        statuses = []
        for name, state in states.items():
            level = DiagnosticStatus.OK if state in {"OK", "ACTIVE", "READY"} else (
                DiagnosticStatus.ERROR if state == "ERROR" else
                DiagnosticStatus.STALE if state == "STALE" else DiagnosticStatus.WARN
            )
            statuses.append(DiagnosticStatus(
                level=level, name=name, message=state, hardware_id="d_task",
                values=[KeyValue(key="state", value=state)],
            ))
        summary = {
            "states": states, "failsafe": self.px4_failsafe,
            "armed": self.px4_armed, "offboard": self.px4_offboard,
            "mission_state": self.mission_state, "mission_event": self.mission_event,
            "vision_h7_status": self.vision_h7_state,
            "target_valid": self.target_valid, "target_age_ms": self.target_age_ms,
            "cpu_temperature_c": temperature, "disk_free_fraction": disk_fraction,
            "memory_used_fraction": memory_fraction,
            "uptime_seconds": now - self.started,
        }
        diagnostic = DiagnosticArray(status=statuses)
        diagnostic.header.stamp = self.get_clock().now().to_msg()
        self.diag_pub.publish(diagnostic)
        self.status_pub.publish(String(data=json.dumps(summary, separators=(",", ":"))))


def main(args=None) -> None:  # noqa: D103
    rclpy.init(args=args)
    node = SystemHealthNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
