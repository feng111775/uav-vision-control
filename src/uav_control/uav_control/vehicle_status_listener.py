from px4_msgs.msg import VehicleStatus
import rclpy
from rclpy.node import Node

from .px4_qos import px4_output_qos


class VehicleStatusListener(Node):
    """读取并显示PX4飞行状态."""

    def __init__(self):
        super().__init__('vehicle_status_listener')
        self.declare_parameter(
            'vehicle_status_topic',
            '/fmu/out/vehicle_status_v1',
        )
        status_topic = self.get_parameter('vehicle_status_topic').value

        # PX4通过DDS发布状态时使用的QoS配置。
        px4_qos = px4_output_qos()

        # PX4状态topic由vehicle_status_topic参数配置。
        self.status_subscription = self.create_subscription(
            VehicleStatus,
            status_topic,
            self.status_callback,
            px4_qos,
        )

        self.get_logger().info(
            '状态监听节点已启动，正在等待 %s' % status_topic
        )

    def status_callback(self, message):
        """收到PX4状态消息后打印重要信息."""
        self.get_logger().info(
            f'arming_state={message.arming_state}, '
            f'nav_state={message.nav_state}, '
            f'failsafe={message.failsafe}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = VehicleStatusListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
