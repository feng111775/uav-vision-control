"""将PX4本地NED位置转换为任务地图位置的ROS 2桥接节点。"""

import math
from typing import Optional

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleLocalPosition
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

from .coordinate_transform import CoordinateTransformer, MapCoordinate


class PX4PositionBridgeLogic:
    """校验PX4本地位置并转换为任务地图坐标。"""

    def __init__(self, transformer: CoordinateTransformer) -> None:
        """保存坐标转换器和最新有效地图位置。"""
        self.transformer = transformer
        self.map_position: Optional[MapCoordinate] = None

    def update_position(
            self,
            north: float,
            east: float,
            down: float,
            xy_valid: bool,
            z_valid: bool,
    ) -> bool:
        """有效时转换并保存位置，无效时清除位置并返回False。"""
        try:
            coordinates = tuple(
                float(value) for value in (north, east, down))
        except (TypeError, ValueError):
            self.map_position = None
            return False

        valid = (
            bool(xy_valid)
            and bool(z_valid)
            and all(math.isfinite(value) for value in coordinates)
        )
        if not valid:
            self.map_position = None
            return False

        self.map_position = self.transformer.px4_to_map_position(
            *coordinates)
        return True


class PX4PositionBridgeNode(Node):
    """以10 Hz发布由PX4 NED转换得到的任务地图位置。"""

    TIMER_PERIOD = 0.1

    def __init__(self) -> None:
        """创建转换参数、PX4订阅、地图位置发布和定时器。"""
        super().__init__('px4_position_bridge_node')

        self.declare_parameter('origin_north', 0.0)
        self.declare_parameter('origin_east', 0.0)
        self.declare_parameter('origin_down', 0.0)
        self.declare_parameter('yaw_offset', 0.0)

        transformer = CoordinateTransformer(
            origin_north=self.get_parameter('origin_north').value,
            origin_east=self.get_parameter('origin_east').value,
            origin_down=self.get_parameter('origin_down').value,
            yaw_offset=self.get_parameter('yaw_offset').value,
        )
        self.logic = PX4PositionBridgeLogic(transformer)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.position_subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.position_callback,
            px4_qos,
        )
        self.position_publisher = self.create_publisher(
            PoseStamped,
            '/mission/current_position',
            10,
        )
        self.timer = self.create_timer(
            self.TIMER_PERIOD,
            self.timer_callback,
        )
        self.get_logger().info('PX4位置反馈桥接节点已启动')

    def position_callback(self, message: VehicleLocalPosition) -> None:
        """接收PX4 NED位置并更新转换结果。"""
        valid = self.logic.update_position(
            north=message.x,
            east=message.y,
            down=message.z,
            xy_valid=message.xy_valid,
            z_valid=message.z_valid,
        )
        if not valid:
            self.get_logger().warning(
                'PX4本地位置无效，停止发布任务位置',
                throttle_duration_sec=5.0,
            )

    def timer_callback(self) -> None:
        """存在有效位置时以10 Hz发布任务地图位置。"""
        position = self.logic.map_position
        if position is None:
            return

        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.pose.position.x = position.x
        message.pose.position.y = position.y
        message.pose.position.z = position.z
        message.pose.orientation.w = 1.0
        self.position_publisher.publish(message)


def main(args=None) -> None:
    """启动PX4位置反馈桥接节点。"""
    rclpy.init(args=args)
    node = PX4PositionBridgeNode()
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
