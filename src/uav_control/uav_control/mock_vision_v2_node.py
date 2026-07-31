"""Simulation-only structured V2 vision publisher; never publishes PX4 input."""

import math
import time

import rclpy
from rclpy.node import Node
from uav_interfaces.msg import LandingError, TargetObservation, VisionHealth


class MockVisionV2Node(Node):
    SCENARIOS = {
        'healthy_no_target', 'target_acquired', 'moving_follow', 'align_success',
        'align_0_39_not_release', 'alignment_break_and_restart',
        'low_confidence', 'stale_source_timestamp', 'receive_timeout',
        'duplicate_sequence', 'out_of_order_sequence', 'sequence_restart',
        'health_unhealthy', 'vision_disconnect_reconnect', 'malformed',
    }

    def __init__(self):
        super().__init__('mock_vision_v2_node')
        self.declare_parameter('simulation_mode', True)
        self.declare_parameter('scenario', 'healthy_no_target')
        self.declare_parameter('publish_rate_hz', 20.0)
        if not bool(self.get_parameter('simulation_mode').value):
            raise ValueError('mock_vision_v2_node requires simulation_mode=true')
        self.scenario = str(self.get_parameter('scenario').value)
        if self.scenario not in self.SCENARIOS:
            raise ValueError('unsupported V2 mock scenario')
        self.started = time.monotonic()
        self.sequence = 0
        self.tracked_pub = self.create_publisher(TargetObservation,
                                                 '/vision/target/tracked', 5)
        self.landing_pub = self.create_publisher(LandingError,
                                                 '/vision/landing_error', 5)
        self.health_pub = self.create_publisher(VisionHealth, '/vision/health', 5)
        self.create_timer(1.0 / float(self.get_parameter('publish_rate_hz').value),
                          self._publish)

    def _publish(self):
        elapsed = time.monotonic() - self.started
        scenario = self.scenario
        self.sequence += 1
        sequence = self.sequence
        if scenario == 'duplicate_sequence' and sequence % 4 == 0:
            sequence -= 1
        if scenario == 'out_of_order_sequence' and sequence % 5 == 0:
            sequence = max(1, sequence - 2)
        if scenario == 'sequence_restart' and elapsed > 1.0:
            sequence = 1
            self.sequence = 0
        healthy = scenario not in ('health_unhealthy', 'vision_disconnect_reconnect')
        target = scenario not in ('healthy_no_target', 'receive_timeout', 'malformed')
        if scenario == 'target_acquired' and elapsed < 1.0:
            target = False
        if scenario == 'low_confidence':
            confidence = 20.0
        else:
            confidence = 95.0
        error = 0.0 if scenario in ('align_success', 'align_0_39_not_release') else 0.25
        if scenario == 'moving_follow':
            error = max(0.0, 0.25 - elapsed * 0.08)
        if scenario == 'alignment_break_and_restart' and 1.0 < elapsed < 1.2:
            error = 0.3
        stamp = self.get_clock().now().to_msg()
        health = VisionHealth()
        health.header.stamp = stamp
        health.camera_open = healthy
        health.frames_received = healthy
        health.algorithm_alive = healthy
        health.protocol_ok = healthy
        health.ready_for_closed_loop = healthy
        health.performance_gate_passed = healthy
        health.publish_fps = 20.0
        health.current_mode = 'MOCK'
        health.status_text = 'MOCK'
        self.health_pub.publish(health)
        tracked = TargetObservation()
        tracked.header.stamp = stamp
        tracked.capture_stamp_valid = healthy
        tracked.frame_sequence = sequence & 0xffffffff
        tracked.detected = target
        tracked.confirmed = target
        tracked.measurement_valid = target and scenario != 'malformed'
        tracked.confidence = confidence if target else 0.0
        tracked.image_width = 320
        tracked.image_height = 240
        tracked.error_x_norm = error if target else math.nan
        tracked.error_y_norm = 0.0 if target else math.nan
        tracked.platform_center_valid = target
        tracked.metric_valid = False
        tracked.forward_m = math.nan
        tracked.left_m = math.nan
        self.tracked_pub.publish(tracked)
        landing = LandingError()
        landing.header.stamp = stamp
        landing.capture_stamp_valid = healthy
        landing.valid = target and scenario != 'malformed'
        landing.frame_sequence = tracked.frame_sequence
        landing.confidence = tracked.confidence
        landing.error_x_norm = tracked.error_x_norm
        landing.error_y_norm = tracked.error_y_norm
        landing.metric_valid = False
        landing.forward_m = math.nan
        landing.left_m = math.nan
        self.landing_pub.publish(landing)


def main(args=None):
    rclpy.init(args=args)
    node = MockVisionV2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
