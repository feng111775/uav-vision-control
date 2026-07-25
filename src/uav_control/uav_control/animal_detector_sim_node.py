"""发布H题固定动物检测结果的ROS 2模拟节点。."""

from mission_interfaces.msg import AnimalDetection
from mission_interfaces.msg import MissionStatus

import rclpy
from rclpy.node import Node


class AnimalDetectorSimNode(Node):
    """进入指定巡查方格时发布对应的模拟动物检测。."""

    ANIMAL_TYPES = (
        'elephant',
        'tiger',
        'wolf',
        'monkey',
        'peacock',
    )
    SIMULATED_DETECTIONS = {
        'A3B2': ('tiger', 0.93, 1),
        'A5B4': ('monkey', 0.88, 1),
    }

    def __init__(self):
        """创建检测发布器和任务状态订阅。."""
        super().__init__('animal_detector_sim_node')
        self.detected_cells = set()
        self.publisher = self.create_publisher(
            AnimalDetection,
            '/animal/detection',
            10,
        )
        self.mission_status_subscription = self.create_subscription(
            MissionStatus,
            '/mission/status',
            self.mission_status_callback,
            10,
        )
        self.get_logger().info(
            '动物检测模拟节点已启动：等待无人机进入动物方格')

    def mission_status_callback(self, status):
        """进入配置的动物方格时发布一次检测结果。."""
        if status.state == MissionStatus.IDLE:
            self.detected_cells.clear()
            return
        if status.state != MissionStatus.EXECUTE:
            return

        cell_id = status.current_cell
        detection = self.SIMULATED_DETECTIONS.get(cell_id)
        if detection is None or cell_id in self.detected_cells:
            return

        animal_type, confidence, count = detection
        message = AnimalDetection()
        message.animal_type = animal_type
        message.cell_id = cell_id
        message.confidence = confidence
        message.count = count
        self.publisher.publish(message)
        self.detected_cells.add(cell_id)
        self.get_logger().info(
            '模拟发现%s：cell=%s confidence=%.2f count=%d'
            % (animal_type, cell_id, confidence, count))


def main(args=None):
    """启动动物检测模拟节点。."""
    rclpy.init(args=args)
    node = AnimalDetectorSimNode()
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
