"""发布模拟H7Plus目标数据，供无硬件环境测试。"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class FakeH7Node(Node):
    """以10 Hz发布变化的模拟检测结果。"""

    def __init__(self):
        super().__init__('fake_h7_node')
        self.declare_parameter(
            'detection_topic', '/vision/h7/detection')
        self.publisher = self.create_publisher(
            Float32MultiArray,
            self.get_parameter('detection_topic').value, 10)
        self.sample_index = 0
        self.timer = self.create_timer(0.1, self.publish_detection)
        self.get_logger().info('H7Plus模拟数据节点已启动，发布频率10 Hz')

    def publish_detection(self):
        """生成一个在图像中心附近缓慢移动的目标。"""
        phase = self.sample_index * 0.1
        message = Float32MultiArray()
        message.data = [
            1.0,
            160.0 + 20.0 * math.sin(phase),
            120.0 + 12.0 * math.cos(phase),
            50.0,
            48.0,
            2400.0,
            90.0,
            320.0,
            240.0,
        ]
        self.publisher.publish(message)
        self.sample_index += 1


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
