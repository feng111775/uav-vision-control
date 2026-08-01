"""Read-only ROS node enforcing continuous PX4 readiness."""

import json

from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, VehicleStatus
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .px4_qos import px4_output_qos
from .readiness_core import ReadinessGate


class ReadinessGateNode(Node):
    def __init__(self):
        super().__init__('readiness_gate')
        defaults = {
            'status_topic': '/fmu/out/vehicle_status_v1',
            'position_topic': '/fmu/out/vehicle_local_position',
            'attitude_topic': '/fmu/out/vehicle_attitude',
            'stable_seconds': 3.0,
            'status_timeout': 1.0,
            'position_timeout': 0.5,
            'attitude_timeout': 0.5,
            'simulation_mode': False,
            'allow_sitl_heading_quality_bypass': False,
            'exit_on_ready': False,
            'overall_timeout_seconds': 60.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.gate = ReadinessGate(
            stable_seconds=self._param('stable_seconds'),
            status_timeout=self._param('status_timeout'),
            position_timeout=self._param('position_timeout'),
            attitude_timeout=self._param('attitude_timeout'),
            simulation_mode=self._param('simulation_mode'),
            allow_sitl_heading_quality_bypass=self._param(
                'allow_sitl_heading_quality_bypass'))
        self.started = self.now()
        self.exit_code = None
        qos = px4_output_qos()
        self.topics = {
            'status': self._param('status_topic'),
            'position': self._param('position_topic'),
            'attitude': self._param('attitude_topic'),
        }
        self.create_subscription(
            VehicleStatus, self.topics['status'], self._status, qos)
        self.create_subscription(
            VehicleLocalPosition, self.topics['position'],
            self._position, qos)
        self.create_subscription(
            VehicleAttitude, self.topics['attitude'],
            self._attitude, qos)
        self.ready_pub = self.create_publisher(
            Bool, '/uav/readiness/ready', 10)
        self.safety_ready_pub = self.create_publisher(
            Bool, '/uav/safety/ready', 10)
        self.status_pub = self.create_publisher(
            String, '/uav/readiness/status', 10)
        self.create_timer(0.1, self._tick)

    def _param(self, name):
        return self.get_parameter(name).value

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _status(self, msg):
        self.gate.update_status(
            self.now(), msg.pre_flight_checks_pass, msg.failsafe,
            msg.arming_state == VehicleStatus.ARMING_STATE_DISARMED)

    def _position(self, msg):
        self.gate.update_position(
            self.now(), msg.xy_valid, msg.z_valid, msg.v_xy_valid,
            msg.v_z_valid, msg.heading_good_for_control, msg.heading)

    def _attitude(self, msg):
        self.gate.update_attitude(self.now(), msg.q)

    def _tick(self):
        now = self.now()
        present = {
            name: bool(self.get_publishers_info_by_topic(topic))
            for name, topic in self.topics.items()
        }
        ready, reasons = self.gate.evaluate(now, present)
        ready_msg = Bool()
        ready_msg.data = ready
        self.ready_pub.publish(ready_msg)
        self.safety_ready_pub.publish(ready_msg)
        status = String()
        status.data = json.dumps({
            'ready': ready, 'reasons': reasons,
            'stable_seconds': (
                0.0 if self.gate.stable_since is None
                else now - self.gate.stable_since),
            'topics': self.topics,
        }, sort_keys=True)
        self.status_pub.publish(status)
        if ready and self._param('exit_on_ready'):
            self.get_logger().info('PX4 readiness stable; gate passed')
            self.exit_code = 0
            rclpy.shutdown()
        elif now - self.started > self._param('overall_timeout_seconds'):
            self.get_logger().error(
                'Readiness timeout: ' + ','.join(reasons))
            self.exit_code = 1
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ReadinessGateNode()
    try:
        rclpy.spin(node)
    finally:
        code = node.exit_code if node.exit_code is not None else 0
        node.destroy_node()
    raise SystemExit(code)
