"""D-task target simulator for hardware-independent integration tests."""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from .d_task_schema import invalid_detection, validate_detection


class FakeH7Node(Node):
    def __init__(self):
        super().__init__('fake_h7_node')
        self.declare_parameter('detection_topic', '/vision/h7/detection')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('enable_temporary_loss', True)
        self.declare_parameter('loss_every_frames', 80)
        self.declare_parameter('loss_duration_frames', 2)
        self.publisher = self.create_publisher(
            Float32MultiArray,
            self.get_parameter('detection_topic').value, 10)
        self.index = 0
        rate = float(self.get_parameter('publish_rate_hz').value)
        if rate <= 0.0:
            raise ValueError('publish_rate_hz must be positive')
        self.create_timer(1.0 / rate, self._publish)

    def _publish(self):
        cycle = int(self.get_parameter('loss_every_frames').value)
        duration = int(self.get_parameter('loss_duration_frames').value)
        loss = (self.get_parameter('enable_temporary_loss').value and cycle > 0
                and self.index % cycle < duration)
        if loss:
            data = invalid_detection()
        else:
            phase = self.index * 0.08
            data = validate_detection([
                1.0, 160.0 + 35.0 * math.sin(phase),
                120.0 + 22.0 * math.cos(phase * 0.7), 50.0, 30.0,
                0.65 * math.sin(phase * 0.4),
                75.0 + 20.0 * math.sin(phase * 0.3)])
        message = Float32MultiArray()
        message.data = data
        self.publisher.publish(message)
        self.index += 1


def main(args=None):
    rclpy.init(args=args)
    node = FakeH7Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
