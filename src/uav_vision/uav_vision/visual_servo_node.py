"""将滤波后的H7Plus目标位置转换为ROS FLU水平速度。"""

import math

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


VALUE_COUNT = 7
ZERO_VELOCITY = (0.0, 0.0)


class VisualServoController:
    """与ROS消息解耦的视觉伺服速度计算和超时状态。"""

    def __init__(self, center_x=160.0, center_y=120.0,
                 kp_x=0.002, kp_y=0.002,
                 deadband_x=10.0, deadband_y=10.0,
                 max_velocity=0.3, stale_timeout=0.3,
                 sign_x=-1.0, sign_y=-1.0):
        parameters = [center_x, center_y, kp_x, kp_y, deadband_x,
                      deadband_y, max_velocity, stale_timeout,
                      sign_x, sign_y]
        if not all(math.isfinite(float(value)) for value in parameters):
            raise ValueError('视觉伺服参数必须是有限数值')
        if kp_x < 0.0 or kp_y < 0.0:
            raise ValueError('kp_x和kp_y不能为负数')
        if deadband_x < 0.0 or deadband_y < 0.0:
            raise ValueError('deadband_x和deadband_y不能为负数')
        if max_velocity <= 0.0:
            raise ValueError('max_velocity必须大于0')
        if stale_timeout <= 0.0:
            raise ValueError('stale_timeout必须大于0')

        self.center_x = float(center_x)
        self.center_y = float(center_y)
        self.kp_x = float(kp_x)
        self.kp_y = float(kp_y)
        self.deadband_x = float(deadband_x)
        self.deadband_y = float(deadband_y)
        self.max_velocity = float(max_velocity)
        self.stale_timeout = float(stale_timeout)
        self.sign_x = float(sign_x)
        self.sign_y = float(sign_y)
        self.last_message_time = None
        self.latest_velocity = ZERO_VELOCITY

    @staticmethod
    def validate(values):
        """严格校验七字段滤波检测数据。"""
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

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))

    def process(self, values, now_seconds):
        """接收一帧检测并保存当前速度；非法数据抛出ValueError。"""
        self.last_message_time = float(now_seconds)
        try:
            data = self.validate(values)
        except (TypeError, ValueError):
            self.latest_velocity = ZERO_VELOCITY
            raise

        if data[0] == 0.0:
            self.latest_velocity = ZERO_VELOCITY
            return self.latest_velocity

        error_x = data[1] - self.center_x
        error_y = data[2] - self.center_y
        left_velocity = 0.0
        forward_velocity = 0.0

        if abs(error_x) > self.deadband_x:
            left_velocity = self.sign_x * self.kp_x * error_x
        if abs(error_y) > self.deadband_y:
            forward_velocity = self.sign_y * self.kp_y * error_y

        self.latest_velocity = (
            self._clamp(forward_velocity, self.max_velocity),
            self._clamp(left_velocity, self.max_velocity),
        )
        return self.latest_velocity

    def velocity_at(self, now_seconds):
        """返回当前速度；输入超时或尚无输入时返回零速度。"""
        if self.last_message_time is None:
            return ZERO_VELOCITY
        if float(now_seconds) - self.last_message_time > self.stale_timeout:
            return ZERO_VELOCITY
        return self.latest_velocity

    def is_stale(self, now_seconds):
        """判断是否尚未收到消息或最近消息已经超时。"""
        if self.last_message_time is None:
            return True
        return float(now_seconds) - self.last_message_time > self.stale_timeout


class VisualServoNode(Node):
    """发布base_link坐标系下的视觉伺服水平速度。"""

    def __init__(self):
        super().__init__('visual_servo_node')
        self.declare_parameter('center_x', 160.0)
        self.declare_parameter('center_y', 120.0)
        self.declare_parameter('kp_x', 0.002)
        self.declare_parameter('kp_y', 0.002)
        self.declare_parameter('deadband_x', 10.0)
        self.declare_parameter('deadband_y', 10.0)
        self.declare_parameter('max_velocity', 0.3)
        self.declare_parameter('stale_timeout', 0.3)
        self.declare_parameter('sign_x', -1.0)
        self.declare_parameter('sign_y', -1.0)

        names = ('center_x', 'center_y', 'kp_x', 'kp_y',
                 'deadband_x', 'deadband_y', 'max_velocity',
                 'stale_timeout', 'sign_x', 'sign_y')
        values = {name: self.get_parameter(name).value for name in names}
        self.controller = VisualServoController(**values)
        self.publisher = self.create_publisher(
            TwistStamped, '/control/vision_velocity', 10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/vision/h7/filtered_detection',
            self.detection_callback,
            10,
        )
        # 20 Hz安全检查，高于实测15.54 Hz输入且远短于默认超时时间。
        self.stale_timer = self.create_timer(0.05, self.check_stale_input)
        self.get_logger().info(
            '视觉伺服节点已启动：frame=base_link，最大速度=%.2f m/s'
            % self.controller.max_velocity)

    def now_seconds(self):
        """返回ROS时钟秒数，兼容仿真时间。"""
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_velocity(self, forward, left):
        """发布仅包含FLU水平线速度的TwistStamped。"""
        output = TwistStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = 'base_link'
        output.twist.linear.x = forward
        output.twist.linear.y = left
        output.twist.linear.z = 0.0
        output.twist.angular.x = 0.0
        output.twist.angular.y = 0.0
        output.twist.angular.z = 0.0
        self.publisher.publish(output)

    def detection_callback(self, message):
        """处理检测数据，任何异常输入都立即发布零速度。"""
        try:
            velocity = self.controller.process(
                message.data, self.now_seconds())
        except (TypeError, ValueError) as error:
            self.get_logger().warning(
                '非法视觉检测数据，发布零速度：%s' % error,
                throttle_duration_sec=5.0,
            )
            velocity = ZERO_VELOCITY
        self.publish_velocity(*velocity)

    def check_stale_input(self):
        """输入超时时定期发布零速度。"""
        now_seconds = self.now_seconds()
        if self.controller.is_stale(now_seconds):
            self.publish_velocity(*ZERO_VELOCITY)
            self.get_logger().warning(
                '视觉检测输入超时，发布零速度',
                throttle_duration_sec=5.0,
            )


def main(args=None):
    """启动视觉伺服速度计算节点。"""
    rclpy.init(args=args)
    node = VisualServoNode()
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
