"""将任务规划轨迹点转换为通用PoseStamped目标。"""

import math
from typing import Optional

from geometry_msgs.msg import PoseStamped
from mission_interfaces.msg import TrajectoryPoint
import rclpy
from rclpy.node import Node


class TrajectoryBridgeNode(Node):
    """桥接任务规划层轨迹消息和控制层位置目标接口。"""

    def __init__(self) -> None:
        """创建参数、轨迹订阅、目标发布和定时器。"""
        super().__init__('trajectory_bridge_node')

        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('publish_rate', 10.0)

        self.target_frame = str(
            self.get_parameter('target_frame').value)
        publish_rate = float(
            self.get_parameter('publish_rate').value)
        if not self.target_frame:
            raise ValueError('target_frame不能为空')
        if not math.isfinite(publish_rate) or publish_rate <= 0.0:
            raise ValueError('publish_rate必须是有限正数')

        self.latest_target: Optional[TrajectoryPoint] = None
        self.target_publisher = self.create_publisher(
            PoseStamped,
            '/control/trajectory_target',
            10,
        )
        self.trajectory_subscription = self.create_subscription(
            TrajectoryPoint,
            '/mission/trajectory_point',
            self.trajectory_callback,
            10,
        )
        self.timer = self.create_timer(
            1.0 / publish_rate,
            self.timer_callback,
        )

        self.get_logger().info('任务轨迹桥接节点已启动')

    def trajectory_callback(self, message: TrajectoryPoint) -> None:
        """保存任务规划层发布的最新轨迹目标。"""
        self.latest_target = message

    def timer_callback(self) -> None:
        """按配置频率发布最新位置目标。"""
        if self.latest_target is None:
            return

        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.target_frame
        message.pose.position.x = self.latest_target.x
        message.pose.position.y = self.latest_target.y
        message.pose.position.z = self.latest_target.z
        message.pose.orientation.w = 1.0
        self.target_publisher.publish(message)


def main(args=None) -> None:
    """启动任务轨迹桥接节点。"""
    rclpy.init(args=args)
    node = TrajectoryBridgeNode()
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
