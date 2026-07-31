"""Single synthetic moving-car scenario; never used by competition launch."""

import math

from px4_msgs.msg import VehicleLocalPosition
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray, UInt8

from .d_task_schema import invalid_detection, validate_detection


class DTaskSitlScenarioNode(Node):
    def __init__(self):
        super().__init__('d_task_sitl_scenario_node')
        self.declare_parameter('d_time_seconds', 45.0)
        self.declare_parameter('lap_seconds', 75.0)
        self.declare_parameter('half_fov_m', 9.0)
        self.declare_parameter('pixels_per_meter', 15.0)
        self.declare_parameter('fault_mode', 'none')
        self.start = self.get_clock().now()
        self.uav = None
        self.release = False
        self.start_pub = self.create_publisher(Bool, '/car/mission_start', 10)
        self.progress_pub = self.create_publisher(UInt8, '/car/progress', 10)
        self.detect_pub = self.create_publisher(Float32MultiArray, '/vision/h7/detection', 10)
        self.ack_pub = self.create_publisher(Bool, '/uav/payload/release_ack', 10)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1)
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self._position,
            px4_qos)
        self.create_subscription(Bool, '/uav/payload/release', self._release, 10)
        self.create_timer(0.1, self._tick)

    def _position(self, msg):
        self.uav = (msg.x, msg.y, msg.z, msg.heading)
    def _release(self, msg): self.release = self.release or msg.data

    def _tick(self):
        elapsed = (self.get_clock().now() - self.start).nanoseconds / 1e9
        lap = self.get_parameter('lap_seconds').value
        phase = 2 * math.pi * (elapsed % lap) / lap
        car = (4.0 * math.cos(phase), 3.0 * math.sin(phase))
        start = Bool()
        start.data = elapsed >= 1.0
        self.start_pub.publish(start)
        d_time = self.get_parameter('d_time_seconds').value
        if self.get_parameter('fault_mode').value == 'early_d':
            d_time = 12.0
        progress = 0 if elapsed < 1 else 1 if elapsed < 10 else 2 if elapsed < 25 else 3 if elapsed < d_time else 4 if elapsed < lap else 5
        p = UInt8()
        p.data = progress
        self.progress_pub.publish(p)
        data = invalid_detection()
        if self.uav:
            dx, dy = car[0] - self.uav[0], car[1] - self.uav[1]
            heading = self.uav[3]
            forward = math.cos(heading) * dx + math.sin(heading) * dy
            right = -math.sin(heading) * dx + math.cos(heading) * dy
            half_fov = self.get_parameter('half_fov_m').value
            scale = self.get_parameter('pixels_per_meter').value
            in_view = abs(forward) < half_fov and abs(right) < half_fov
            fault = self.get_parameter('fault_mode').value
            sustained_loss = fault == 'sustained_vision_loss' and elapsed >= 28.0
            short_loss = fault == 'short_vision_loss' and 28.0 <= elapsed < 28.6
            if in_view and not sustained_loss and not short_loss and not (
                    30 < elapsed % 60 < 30.4):
                center_x = max(0.0, min(320.0, 160 + right * scale))
                center_y = max(0.0, min(240.0, 120 - forward * scale))
                data = validate_detection(
                    [1, center_x, center_y, 50, 30, phase, 85])
        msg = Float32MultiArray()
        msg.data = data
        self.detect_pub.publish(msg)
        if (self.release and
                self.get_parameter('fault_mode').value != 'payload_ack_loss'):
            ack = Bool()
            ack.data = True
            self.ack_pub.publish(ack)


def main(args=None):
    rclpy.init(args=args)
    node = DTaskSitlScenarioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
