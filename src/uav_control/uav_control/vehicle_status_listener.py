import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy

from px4_msgs.msg import VehicleStatus


class VehicleStatusListener(Node):
    """读取并显示PX4飞行状态。"""

    def __init__(self):
        super().__init__('vehicle_status_listener')
        self.declare_parameter(
            'vehicle_status_topic',
            '/fmu/out/vehicle_status',
        )

        # PX4通过DDS发布状态时使用的QoS配置。
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # PX4状态topic由vehicle_status_topic参数配置。
        self.status_subscription = self.create_subscription(
            VehicleStatus,
            self.get_parameter('vehicle_status_topic').value,
            self.status_callback,
            px4_qos,
        )

        self.get_logger().info(
            '状态监听节点已启动，当前通过vehicle_status_topic参数配置PX4状态topic'
        )

    def status_callback(self, message):
        """收到PX4状态消息后打印重要信息。"""
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
