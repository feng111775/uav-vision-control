# flake8: noqa
# noqa: D100,D101,D102,D103,D104,D105,D107
"""Read-only-input safety gate for the existing mission controller."""
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from .safety_logic import SafetyInputs, evaluate_safety


class SafetyGateNode(Node):  # noqa: D101
    def __init__(self) -> None:  # noqa: D107
        super().__init__("safety_gate_node")
        for name, value in (
            ("mode", "observe"), ("operator_timeout_seconds", 1.0),
            ("px4_timeout_seconds", 1.0), ("competition_configured", False),
        ):
            self.declare_parameter(name, value)
        self.operator = False
        self.times = {"operator": 0.0, "px4": 0.0, "position": 0.0, "attitude": 0.0}
        self.failsafe = self.armed = False
        self.mission_state = "UNKNOWN"
        self.subsystems = {}
        self.ready_pub = self.create_publisher(Bool, "/uav/readiness/ready", 10)
        self.reason_pub = self.create_publisher(String, "/uav/safety/reason", 10)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )
        self.create_subscription(
            Bool, "/uav/safety/operator_enable", self._operator, 10
        )
        self.create_subscription(
            DiagnosticArray, "/system/diagnostics", self._diagnostics, 10
        )
        self.create_subscription(
            String, "/uav/mission/state",
            lambda msg: setattr(self, "mission_state", msg.data), 10
        )
        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status_v1", self._status, px4_qos
        )
        self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position",
            lambda _: self._stamp("position"), px4_qos
        )
        self.create_subscription(
            VehicleAttitude, "/fmu/out/vehicle_attitude",
            lambda _: self._stamp("attitude"), px4_qos
        )
        self.create_timer(0.1, self._publish)

    def _stamp(self, name: str) -> None:
        self.times[name] = time.monotonic()

    def _operator(self, msg: Bool) -> None:
        self.operator = msg.data
        self._stamp("operator")

    def _status(self, msg: VehicleStatus) -> None:
        self._stamp("px4")
        self.failsafe = bool(msg.failsafe)
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def _diagnostics(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            self.subsystems[status.name.split("_")[0]] = status.message

    def _publish(self) -> None:
        now = time.monotonic()
        operator_timeout = float(
            self.get_parameter("operator_timeout_seconds").value
        )
        px4_timeout = float(self.get_parameter("px4_timeout_seconds").value)
        running = self.mission_state not in {
            "UNKNOWN", "WAIT_PX4", "WAIT_SAFETY", "WAIT_START", "COMPLETE"
        }
        ready, reason = evaluate_safety(SafetyInputs(
            mode=str(self.get_parameter("mode").value),
            operator_enabled=self.operator,
            operator_fresh=now - self.times["operator"] <= operator_timeout,
            px4_fresh=now - self.times["px4"] <= px4_timeout,
            position_fresh=now - self.times["position"] <= px4_timeout,
            attitude_fresh=now - self.times["attitude"] <= px4_timeout,
            failsafe=self.failsafe, armed=self.armed, mission_running=running,
            subsystem=self.subsystems,
            competition_configured=bool(
                self.get_parameter("competition_configured").value
            ),
        ))
        self.ready_pub.publish(Bool(data=ready))
        self.reason_pub.publish(String(data=reason))


def main(args=None) -> None:  # noqa: D103
    rclpy.init(args=args)
    node = SafetyGateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
