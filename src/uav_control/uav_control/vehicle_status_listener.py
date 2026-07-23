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

        # PX4通过DDS发布状态时使用的QoS配置。
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 注意：你的PX4状态话题带有_v1后缀。
        self.status_subscription = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.status_callback,
            px4_qos,
        )

        self.get_logger().info(
            '状态监听节点已启动，正在等待 /fmu/out/vehicle_status_v1'
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
