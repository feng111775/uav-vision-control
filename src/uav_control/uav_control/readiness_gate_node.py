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
            'flight_authorized': False,
            'payload_authorized': False,
            'vision_health_topic': '/vision/health',
            'servo_health_topic': '/servo/health',
            'mission_state_topic': '/uav/mission/state',
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
        self.vision_ok = False
        self.servo_ok = False
        self.mission_state = 'WAIT_PX4'
        self.create_subscription(Bool, self._param('vision_health_topic'),
                                 lambda msg: setattr(self, 'vision_ok', bool(msg.data)), 10)
        self.create_subscription(Bool, self._param('servo_health_topic'),
                                 lambda msg: setattr(self, 'servo_ok', bool(msg.data)), 10)
        self.create_subscription(String, self._param('mission_state_topic'),
                                 lambda msg: setattr(self, 'mission_state', str(msg.data)), 10)
        self.ready_pub = self.create_publisher(
            Bool, '/uav/readiness/ready', 10)
        self.safety_ready_pub = self.create_publisher(
            Bool, '/uav/safety/ready', 10)
        self.status_pub = self.create_publisher(
            String, '/uav_mission/readiness', 10)
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
        reasons = list(reasons)
        if not self.vision_ok:
            reasons.append('vision_not_ready')
        if not self.servo_ok:
            reasons.append('servo_not_ready')
        if self.mission_state != 'WAIT_START':
            reasons.append('mission_not_wait_start')
        if not bool(self._param('flight_authorized')):
            reasons.append('flight_not_authorized')
        if not bool(self._param('payload_authorized')):
            reasons.append('payload_not_authorized')
        base_ready = ready and self.vision_ok and self.servo_ok and bool(
            self._param('flight_authorized')) and bool(self._param('payload_authorized'))
        ready = base_ready and self.mission_state == 'WAIT_START'
        ready_msg = Bool()
        ready_msg.data = ready
        self.ready_pub.publish(ready_msg)
        self.safety_ready_pub.publish(Bool(data=base_ready))
        status = String()
        status.data = json.dumps({
            'ready': ready, 'reasons': reasons,
            'stable_seconds': (
                0.0 if self.gate.stable_since is None
                else now - self.gate.stable_since),
            'topics': self.topics,
        }, sort_keys=True)
        status.data = json.dumps({
            'ready': ready, 'busy': self.mission_state not in (
                'WAIT_START', 'WAIT_PX4', 'WAIT_SAFETY'),
            'state': self.mission_state, 'dds_ok': all(present.values()),
            'px4_status_ok': self.gate.valid['status'],
            'position_ok': self.gate.valid['position'], 'attitude_ok': self.gate.valid['attitude'],
            'preflight_ok': self.gate.valid['status'],
            'failsafe': self.gate.failsafe,
            'disarmed': self.gate.disarmed, 'vision_ok': self.vision_ok,
            'servo_ok': self.servo_ok, 'safety_ready': ready,
            'reasons': reasons, 'stamp_ns': self.get_clock().now().nanoseconds,
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
