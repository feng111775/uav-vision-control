"""任务规划层ROS 2闭环测试使用的模拟位置反馈节点。"""

from typing import Optional, Tuple

from geometry_msgs.msg import PoseStamped
from mission_interfaces.msg import TrajectoryPoint
import rclpy
from rclpy.node import Node


class FakePositionNode(Node):
    """以固定比例逐步逼近任务目标，并发布模拟地图坐标位置。"""

    TIMER_PERIOD = 0.1
    MOVE_RATIO = 0.1

    def __init__(self) -> None:
        """创建轨迹订阅、位置发布接口和10 Hz模拟定时器。"""
        super().__init__('fake_position_node')

        self.current_position = [0.0, 0.0, 0.0]
        self.target_position: Optional[Tuple[float, float, float]] = None
        self.target_key: Optional[Tuple[int, int]] = None

        self.position_publisher = self.create_publisher(
            PoseStamped,
            '/mission/current_position',
            10,
        )
        self.target_subscription = self.create_subscription(
            TrajectoryPoint,
            '/mission/trajectory_point',
            self.target_callback,
            10,
        )
        self.timer = self.create_timer(
            self.TIMER_PERIOD,
            self.timer_callback,
        )
        self.get_logger().info('模拟位置节点已启动，等待任务轨迹目标')

    def target_callback(self, message: TrajectoryPoint) -> None:
        """保存最新轨迹目标，并在目标变化时输出目标坐标。"""
        self.target_position = (
            float(message.x),
            float(message.y),
            float(message.z),
        )
        new_target_key = (
            int(message.time_from_start.sec),
            int(message.time_from_start.nanosec),
        )
        if new_target_key != self.target_key:
            self.target_key = new_target_key
            self.get_logger().info(
                '当前目标：x=%.3f, y=%.3f, z=%.3f'
                % self.target_position
            )

    def timer_callback(self) -> None:
        """每周期移动剩余距离的10%，并发布当前位置。"""
        if self.target_position is None:
            return

        for axis in range(3):
            remaining = (
                self.target_position[axis] - self.current_position[axis])
            self.current_position[axis] += remaining * self.MOVE_RATIO

        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.pose.position.x = self.current_position[0]
        message.pose.position.y = self.current_position[1]
        message.pose.position.z = self.current_position[2]
        message.pose.orientation.w = 1.0
        self.position_publisher.publish(message)

        self.get_logger().info(
            '当前位置：x=%.3f, y=%.3f, z=%.3f'
            % tuple(self.current_position),
            throttle_duration_sec=1.0,
        )


def main(args=None) -> None:
    """启动模拟位置反馈节点。"""
    rclpy.init(args=args)
    node = FakePositionNode()
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
