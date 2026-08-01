"""将正式D-task检测转换为ROS FLU水平速度。"""

import math

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

from .d_task_schema import CENTER_X, CENTER_Y, VALID, validate_detection

ZERO_VELOCITY = (0.0, 0.0)


class VisualServoController:
    """与ROS消息解耦的视觉伺服速度计算和超时状态。"""

    def __init__(self, kp_x=0.15, kp_y=0.15,
                 deadband_x=0.05, deadband_y=0.05,
                 max_velocity=0.3, stale_timeout=0.3,
                 sign_x=-1.0, sign_y=-1.0,
                 camera_mode='down', front_approach_velocity=0.12,
                 image_width=640.0, image_height=480.0):
        parameters = [kp_x, kp_y, deadband_x, deadband_y,
                      max_velocity, stale_timeout,
                      sign_x, sign_y, image_width, image_height]
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
        if camera_mode not in ('front', 'down'):
            raise ValueError('camera_mode必须是front或down')
        if (not math.isfinite(float(front_approach_velocity))
                or front_approach_velocity < 0.0):
            raise ValueError('front_approach_velocity必须是有限非负数')

        self.kp_x = float(kp_x)
        self.kp_y = float(kp_y)
        self.deadband_x = float(deadband_x)
        self.deadband_y = float(deadband_y)
        self.max_velocity = float(max_velocity)
        self.stale_timeout = float(stale_timeout)
        self.sign_x = float(sign_x)
        self.sign_y = float(sign_y)
        self.camera_mode = camera_mode
        self.front_approach_velocity = min(
            float(front_approach_velocity), self.max_velocity)
        if image_width <= 0.0 or image_height <= 0.0:
            raise ValueError('image_width和image_height必须为正数')
        self.image_width = float(image_width)
        self.image_height = float(image_height)
        self.image_center_x = self.image_width / 2.0
        self.image_center_y = self.image_height / 2.0
        self.last_message_time = None
        self.latest_velocity = ZERO_VELOCITY

    @staticmethod
    def validate(values):
        """严格校验正式七字段检测数据。"""
        try:
            data = validate_detection(values)
        except (TypeError, ValueError) as error:
            raise ValueError('检测数组包含非法数值') from error
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

        error_x = ((data[CENTER_X] - self.image_center_x)
                   / self.image_center_x)
        error_y = ((data[CENTER_Y] - self.image_center_y)
                   / self.image_center_y)
        left_velocity = 0.0
        forward_velocity = 0.0

        if data[VALID] == 1.0 and abs(error_x) > self.deadband_x:
            left_velocity = self.sign_x * self.kp_x * error_x
        if self.camera_mode == 'front':
            # 前视图像的纵向像素误差主要受目标高度和俯仰影响，不能用来
            # 判断目标在机体前方还是后方。有效目标始终以受限速度接近。
            forward_velocity = self.front_approach_velocity
        elif data[VALID] == 1.0 and abs(error_y) > self.deadband_y:
            forward_velocity = self.sign_y * self.kp_y * error_y

        self.latest_velocity = (
            self._clamp(forward_velocity, self.max_velocity),
            self._clamp(left_velocity, self.max_velocity),
        )
        return self.latest_velocity

    def set_camera_mode(self, camera_mode):
        """切换相机控制语义；未知来源立即拒绝。"""
        if camera_mode not in ('front', 'down'):
            raise ValueError('camera_mode必须是front或down')
        self.camera_mode = camera_mode

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
        self.declare_parameter('kp_x', 0.15)
        self.declare_parameter('kp_y', 0.15)
        self.declare_parameter('deadband_x', 0.05)
        self.declare_parameter('deadband_y', 0.05)
        self.declare_parameter('max_velocity', 0.3)
        self.declare_parameter('stale_timeout', 0.3)
        self.declare_parameter('sign_x', -1.0)
        self.declare_parameter('sign_y', -1.0)
        self.declare_parameter('camera_mode', 'down')
        self.declare_parameter('front_approach_velocity', 0.12)
        self.declare_parameter('image_width', 640.0)
        self.declare_parameter('image_height', 480.0)
        self.declare_parameter('selected_camera_topic', '')
        self.declare_parameter('camera_source_timeout', 0.3)
        self.declare_parameter(
            'filtered_detection_topic',
            '/vision/h7/filtered_detection')
        self.declare_parameter(
            'vision_velocity_topic', '/control/vision_velocity')

        names = ('kp_x', 'kp_y', 'deadband_x', 'deadband_y', 'max_velocity',
                 'stale_timeout', 'sign_x', 'sign_y', 'camera_mode',
                 'front_approach_velocity', 'image_width', 'image_height')
        values = {name: self.get_parameter(name).value for name in names}
        self.controller = VisualServoController(**values)
        self.publisher = self.create_publisher(
            TwistStamped,
            self.get_parameter('vision_velocity_topic').value, 10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            self.get_parameter('filtered_detection_topic').value,
            self.detection_callback,
            10,
        )
        self.camera_source_time = None
        self.camera_source_timeout = float(
            self.get_parameter('camera_source_timeout').value)
        if (not math.isfinite(self.camera_source_timeout)
                or self.camera_source_timeout <= 0.0):
            raise ValueError('camera_source_timeout必须是有限正数')
        selected_camera_topic = self.get_parameter(
            'selected_camera_topic').value
        self.camera_subscription = None
        if selected_camera_topic:
            self.camera_subscription = self.create_subscription(
                String, selected_camera_topic, self.camera_callback, 10)
        # 20 Hz安全检查，高于实测15.54 Hz输入且远短于默认超时时间。
        self.stale_timer = self.create_timer(0.05, self.check_stale_input)
        self.get_logger().info(
            '视觉伺服节点已启动：frame=base_link，最大速度=%.2f m/s'
            % self.controller.max_velocity)

    def camera_callback(self, message):
        """更新当前选择的相机，非法来源不进入控制。"""
        try:
            self.controller.set_camera_mode(message.data)
        except ValueError as error:
            self.controller.latest_velocity = ZERO_VELOCITY
            self.get_logger().warning(
                '非法相机来源，发布零速度：%s' % error,
                throttle_duration_sec=5.0)
            self.publish_velocity(*ZERO_VELOCITY)
            return
        self.camera_source_time = self.now_seconds()

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
        now_seconds = self.now_seconds()
        if (self.camera_subscription is not None
                and (self.camera_source_time is None
                     or now_seconds - self.camera_source_time
                     > self.camera_source_timeout)):
            self.controller.latest_velocity = ZERO_VELOCITY
            self.publish_velocity(*ZERO_VELOCITY)
            return
        try:
            velocity = self.controller.process(
                message.data, now_seconds)
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
        source_stale = (
            self.camera_subscription is not None
            and (self.camera_source_time is None
                 or now_seconds - self.camera_source_time
                 > self.camera_source_timeout))
        if self.controller.is_stale(now_seconds) or source_stale:
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
