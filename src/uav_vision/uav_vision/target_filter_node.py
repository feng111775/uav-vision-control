"""对H7Plus目标检测结果进行确认、丢失保持和指数平滑。"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


INVALID_DETECTION = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
VALUE_COUNT = 7


class TargetFilter:
    """与ROS接口解耦的目标滤波状态机。"""

    def __init__(self, alpha=0.35, min_confidence=50.0,
                 confirm_frames=3, lost_frames=3):
        if not 0.0 < alpha <= 1.0:
            raise ValueError('alpha必须在(0, 1]范围内')
        if not 0.0 <= min_confidence <= 100.0:
            raise ValueError('min_confidence必须在[0, 100]范围内')
        if confirm_frames < 1:
            raise ValueError('confirm_frames必须至少为1')
        if lost_frames < 1:
            raise ValueError('lost_frames必须至少为1')

        self.alpha = float(alpha)
        self.min_confidence = float(min_confidence)
        self.confirm_frames = int(confirm_frames)
        self.lost_frames = int(lost_frames)
        self.confirm_count = 0
        self.lost_count = 0
        self.confirmed = False
        self.filtered_values = None

    @staticmethod
    def validate(values):
        """严格校验七字段检测数组并返回普通浮点数列表。"""
        if len(values) != VALUE_COUNT:
            raise ValueError(
                '检测数组必须正好包含7个元素，实际为%d' % len(values))

        try:
            data = [float(value) for value in values]
        except (TypeError, ValueError) as error:
            raise ValueError('检测数组包含非法数值') from error

        if not all(math.isfinite(value) for value in data):
            raise ValueError('检测数组中的所有数值必须有限')
        if data[0] not in (0.0, 1.0):
            raise ValueError('valid只能是0或1')
        if any(value < 0.0 for value in data[1:6]):
            raise ValueError('坐标、尺寸和面积不能为负数')
        if not 0.0 <= data[6] <= 100.0:
            raise ValueError('confidence必须在0到100之间')
        return data

    def process(self, values):
        """处理一帧数据并返回七字段滤波结果。"""
        data = self.validate(values)
        qualified = data[0] == 1.0 and data[6] >= self.min_confidence
        if not qualified:
            return self._handle_miss()

        current = data[1:]
        if self.filtered_values is None:
            # 第一次合格检测直接初始化，避免从零开始产生偏差。
            self.filtered_values = list(current)
        else:
            previous_weight = 1.0 - self.alpha
            self.filtered_values = [
                self.alpha * new + previous_weight * old
                for new, old in zip(current, self.filtered_values)
            ]

        self.lost_count = 0
        if not self.confirmed:
            self.confirm_count += 1
            if self.confirm_count >= self.confirm_frames:
                self.confirmed = True

        if self.confirmed:
            return [1.0] + list(self.filtered_values)
        return list(INVALID_DETECTION)

    def _handle_miss(self):
        """处理无效或低置信度帧。"""
        if not self.confirmed:
            self.confirm_count = 0
            self.filtered_values = None
            return list(INVALID_DETECTION)

        self.lost_count += 1
        if self.lost_count < self.lost_frames:
            return [1.0] + list(self.filtered_values)

        self.confirmed = False
        self.confirm_count = 0
        self.lost_count = 0
        self.filtered_values = None
        return list(INVALID_DETECTION)


class TargetFilterNode(Node):
    """订阅原始H7Plus检测并发布滤波后的检测。"""

    def __init__(self):
        super().__init__('target_filter_node')
        self.declare_parameter('alpha', 0.35)
        self.declare_parameter('min_confidence', 50.0)
        self.declare_parameter('confirm_frames', 3)
        self.declare_parameter('lost_frames', 3)
        self.declare_parameter(
            'detection_topic', '/vision/h7/detection')
        self.declare_parameter(
            'filtered_detection_topic',
            '/vision/h7/filtered_detection')

        self.filter = TargetFilter(
            alpha=self.get_parameter('alpha').value,
            min_confidence=self.get_parameter('min_confidence').value,
            confirm_frames=self.get_parameter('confirm_frames').value,
            lost_frames=self.get_parameter('lost_frames').value,
        )
        self.publisher = self.create_publisher(
            Float32MultiArray,
            self.get_parameter('filtered_detection_topic').value, 10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            self.get_parameter('detection_topic').value,
            self.detection_callback,
            10,
        )
        self.get_logger().info(
            'H7Plus目标滤波节点已启动：alpha=%.2f，确认=%d帧，丢失=%d帧'
            % (self.filter.alpha, self.filter.confirm_frames,
               self.filter.lost_frames))

    def detection_callback(self, message):
        """校验、滤波并发布一帧检测数据。"""
        try:
            filtered = self.filter.process(message.data)
        except (TypeError, ValueError) as error:
            self.get_logger().warning(
                '忽略非法H7Plus检测数据：%s' % error,
                throttle_duration_sec=5.0,
            )
            return

        output = Float32MultiArray()
        output.data = filtered
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = TargetFilterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
