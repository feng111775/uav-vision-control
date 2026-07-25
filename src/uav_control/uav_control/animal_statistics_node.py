"""统计H题动物检测结果并发布分类汇总。."""

import json

from mission_interfaces.msg import AnimalDetection

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class AnimalStatisticsLogic:
    """按动物类型和方格去重保存最新检测数量。."""

    ANIMAL_NAMES = {
        'elephant': '大象',
        'tiger': '老虎',
        'wolf': '狼',
        'monkey': '猴',
        'peacock': '孔雀',
    }

    def __init__(self):
        """创建空的方格检测记录。."""
        self.cell_detections = {}

    def update(self, animal_type, cell_id, count):
        """校验检测并更新同一动物、同一方格的最新数量。."""
        if animal_type not in self.ANIMAL_NAMES:
            raise ValueError('不支持的动物类型：%s' % animal_type)
        if not cell_id:
            raise ValueError('cell_id不能为空')
        count_value = int(count)
        if count_value < 0:
            raise ValueError('count不能为负数')
        self.cell_detections[(animal_type, cell_id)] = count_value

    def totals(self):
        """返回包含五种动物的中文数量汇总。."""
        totals = {
            chinese_name: 0
            for chinese_name in self.ANIMAL_NAMES.values()
        }
        for (animal_type, _cell_id), count in self.cell_detections.items():
            totals[self.ANIMAL_NAMES[animal_type]] += count
        return totals


class AnimalStatisticsNode(Node):
    """订阅动物检测，并发布五种动物的累计统计。."""

    def __init__(self):
        """创建统计逻辑、检测订阅和统计发布器。."""
        super().__init__('animal_statistics_node')
        self.logic = AnimalStatisticsLogic()
        self.statistics_publisher = self.create_publisher(
            String,
            '/animal/statistics',
            10,
        )
        self.detection_subscription = self.create_subscription(
            AnimalDetection,
            '/animal/detection',
            self.detection_callback,
            10,
        )
        self.get_logger().info('动物统计节点已启动')

    def detection_callback(self, message):
        """接收检测、更新去重统计并立即发布汇总。."""
        try:
            self.logic.update(
                message.animal_type,
                message.cell_id,
                message.count,
            )
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        self.publish_statistics()

    def publish_statistics(self):
        """以JSON字符串发布五种动物的当前数量。."""
        totals = self.logic.totals()
        message = String()
        message.data = json.dumps(totals, ensure_ascii=False)
        self.statistics_publisher.publish(message)
        self.get_logger().info('动物统计：%s' % message.data)


def main(args=None):
    """启动动物统计节点。."""
    rclpy.init(args=args)
    node = AnimalStatisticsNode()
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
