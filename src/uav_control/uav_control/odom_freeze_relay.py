"""Selective VehicleLocalPosition relay for SITL fault injection only."""

import json
import time

from px4_msgs.msg import VehicleLocalPosition

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from std_srvs.srv import SetBool

from .px4_qos import px4_output_qos


RAW_TOPIC = '/fmu/out/vehicle_local_position'
OUTPUT_TOPIC = '/phase4b3r/vehicle_local_position'
FREEZE_SERVICE = '/phase4b3r/set_odom_freeze'


class RelayGate:
    """Count inputs while suppressing output during an explicit freeze."""

    def __init__(self, clock=time.monotonic):
        """Initialize a forwarding gate with a monotonic clock."""
        self.clock = clock
        self.frozen = False
        self.input_count = 0
        self.output_count = 0
        self.last_input_timestamp_sample = None
        self.last_output_timestamp_sample = None
        self.last_input_time = None
        self.last_output_time = None
        self.freeze_started = None
        self.freeze_ended = None

    def process(self, message):
        """Record and return one input when forwarding is active."""
        now = self.clock()
        self.input_count += 1
        self.last_input_time = now
        self.last_input_timestamp_sample = int(message.timestamp_sample)
        if self.frozen:
            return None
        self.output_count += 1
        self.last_output_time = now
        self.last_output_timestamp_sample = int(message.timestamp_sample)
        return message

    def set_frozen(self, frozen):
        """Set relay state and return whether it changed."""
        frozen = bool(frozen)
        if frozen == self.frozen:
            return False
        self.frozen = frozen
        now = self.clock()
        if frozen:
            self.freeze_started = now
        else:
            self.freeze_ended = now
        return True

    def snapshot(self):
        """Return JSON-compatible relay evidence."""
        return {
            'frozen': self.frozen,
            'input_count': self.input_count,
            'output_count': self.output_count,
            'last_input_timestamp_sample':
                self.last_input_timestamp_sample,
            'last_output_timestamp_sample':
                self.last_output_timestamp_sample,
            'last_input_time': self.last_input_time,
            'last_output_time': self.last_output_time,
            'freeze_started': self.freeze_started,
            'freeze_ended': self.freeze_ended,
        }


class OdomFreezeRelay(Node):
    """Relay PX4 odometry unchanged, with a freeze service."""

    def __init__(self):
        """Create relay endpoints with the production subscriber QoS."""
        super().__init__('phase4b3r_odom_relay')
        qos = px4_output_qos()
        self.gate = RelayGate(self.now)
        self.publisher = self.create_publisher(
            VehicleLocalPosition, OUTPUT_TOPIC, qos)
        self.create_subscription(
            VehicleLocalPosition, RAW_TOPIC, self._input, qos)
        self.create_service(SetBool, FREEZE_SERVICE, self._set_freeze)
        self.stats = self.create_publisher(
            String, '/phase4b3r/odom_relay_stats', 10)
        self.create_timer(0.5, self._publish_stats)

    def now(self):
        """Return ROS-clock seconds for evidence timestamps."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _input(self, message):
        output = self.gate.process(message)
        if output is not None:
            self.publisher.publish(output)

    def _set_freeze(self, request, response):
        changed = self.gate.set_frozen(request.data)
        snapshot = self.gate.snapshot()
        snapshot['request'] = bool(request.data)
        snapshot['changed'] = changed
        response.success = True
        response.message = json.dumps(snapshot, sort_keys=True)
        self.get_logger().info(response.message)
        self._publish_stats()
        return response

    def _publish_stats(self):
        message = String()
        message.data = json.dumps(self.gate.snapshot(), sort_keys=True)
        self.stats.publish(message)


def main(args=None):
    """Run the selective odometry relay."""
    rclpy.init(args=args)
    node = OdomFreezeRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
