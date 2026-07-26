"""记录H2025任务的计划与实际轨迹，并在任务结束时生成图表和CSV。"""

import csv
from datetime import datetime
import math
from pathlib import Path

from geometry_msgs.msg import PoseStamped
from matplotlib.figure import Figure
from mission_interfaces.msg import MissionStatus
from mission_interfaces.msg import TrajectoryPoint
import rclpy
from rclpy.node import Node


class FlightTrajectoryLogger(Node):
    """订阅任务数据，并将一次飞行的计划和实际轨迹保存到磁盘。"""

    RECORDING_STATES = (
        MissionStatus.TAKEOFF,
        MissionStatus.EXECUTE,
        MissionStatus.RETURN,
        MissionStatus.LAND,
    )
    DEFAULT_OUTPUT_DIRECTORY = (
        '/home/a-corn/2025h/uav-vision-control/log')

    def __init__(self):
        """创建订阅、输出参数和轨迹缓存。"""
        super().__init__('flight_trajectory_logger')

        self.declare_parameter(
            'output_directory', self.DEFAULT_OUTPUT_DIRECTORY)
        self.output_directory = Path(
            self.get_parameter('output_directory').value).expanduser()

        self.recording = False
        self.saved = False
        self.recording_started_at = None
        self.actual_points = []
        self.planned_points = []
        self.planned_waypoint_ids = set()

        self.status_subscription = self.create_subscription(
            MissionStatus,
            '/mission/status',
            self.status_callback,
            10,
        )
        self.position_subscription = self.create_subscription(
            PoseStamped,
            '/mission/current_position',
            self.position_callback,
            10,
        )
        self.trajectory_subscription = self.create_subscription(
            TrajectoryPoint,
            '/mission/trajectory_point',
            self.trajectory_callback,
            10,
        )

        self.get_logger().info(
            '飞行轨迹记录节点已启动，输出目录：%s'
            % self.output_directory)

    def elapsed_seconds(self):
        """返回开始记录后的ROS时钟秒数。"""
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.recording_started_at is None:
            self.recording_started_at = now
        return now - self.recording_started_at

    def status_callback(self, message):
        """根据任务状态开始记录，并在COMPLETE时保存结果。"""
        if message.state in self.RECORDING_STATES and not self.recording:
            self.recording = True
            self.recording_started_at = (
                self.get_clock().now().nanoseconds * 1e-9)
            self.get_logger().info('任务已进入飞行状态，开始记录轨迹')

        if message.state == MissionStatus.COMPLETE:
            self.recording = False
            self.save_if_needed()

    @staticmethod
    def _finite_position(x, y, z):
        """判断三维位置是否均为有限数。"""
        return all(math.isfinite(float(value)) for value in (x, y, z))

    def position_callback(self, message):
        """记录任务坐标系中的实际飞行位置。"""
        if not self.recording:
            return
        position = message.pose.position
        if not self._finite_position(position.x, position.y, position.z):
            self.get_logger().warning(
                '忽略包含非有限数值的实际位置',
                throttle_duration_sec=5.0,
            )
            return
        self.actual_points.append((
            self.elapsed_seconds(),
            float(position.x),
            float(position.y),
            float(position.z),
        ))

    def trajectory_callback(self, message):
        """按waypoint_id去重并记录任务计划轨迹点。"""
        if not self.recording:
            return
        waypoint_id = int(message.waypoint_id)
        if waypoint_id in self.planned_waypoint_ids:
            return
        if not self._finite_position(message.x, message.y, message.z):
            self.get_logger().warning(
                '忽略包含非有限数值的计划轨迹点',
                throttle_duration_sec=5.0,
            )
            return
        self.planned_waypoint_ids.add(waypoint_id)
        self.planned_points.append((
            waypoint_id,
            float(message.x),
            float(message.y),
            float(message.z),
        ))

    @staticmethod
    def _write_csv(path, header, rows):
        """写入带表头的UTF-8 CSV文件。"""
        with path.open('w', newline='', encoding='utf-8') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(header)
            writer.writerows(rows)

    def _plot(self, path):
        """生成XY轨迹和高度曲线两个子图。"""
        figure = Figure(figsize=(14, 6), constrained_layout=True)
        axis_xy, axis_z = figure.subplots(1, 2)

        if self.planned_points:
            planned_x = [point[1] for point in self.planned_points]
            planned_y = [point[2] for point in self.planned_points]
            planned_z = [point[3] for point in self.planned_points]
            axis_xy.plot(
                planned_x, planned_y, 'b--', label='Planned path')
            axis_z.plot(
                range(len(planned_z)), planned_z,
                'b--', label='Planned altitude')

        if self.actual_points:
            actual_x = [point[1] for point in self.actual_points]
            actual_y = [point[2] for point in self.actual_points]
            actual_z = [point[3] for point in self.actual_points]
            axis_xy.plot(
                actual_x, actual_y, 'r-', label='Actual path')
            axis_z.plot(
                range(len(actual_z)), actual_z,
                'r-', label='Actual altitude')

        marker_points = self.actual_points or self.planned_points
        if marker_points:
            axis_xy.plot(
                marker_points[0][1], marker_points[0][2],
                'go', label='Start')
            axis_xy.plot(
                marker_points[-1][1], marker_points[-1][2],
                'kx', markersize=9, markeredgewidth=2, label='End')

        axis_xy.set_title('H2025 Mission Flight Path')
        axis_xy.set_xlabel('X (m)')
        axis_xy.set_ylabel('Y (m)')
        axis_xy.axis('equal')
        axis_xy.grid(True)
        axis_xy.legend()

        axis_z.set_title('Altitude Profile')
        axis_z.set_xlabel('Sample index')
        axis_z.set_ylabel('Z (m)')
        axis_z.grid(True)
        axis_z.legend()

        figure.savefig(path, dpi=150)

    def save_if_needed(self):
        """已有数据且尚未保存时生成PNG及两份CSV。"""
        if self.saved or not (self.actual_points or self.planned_points):
            return False

        self.output_directory.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        image_path = (
            self.output_directory / ('h2025_flight_xy_%s.png' % timestamp))
        actual_csv_path = self.output_directory / (
            'h2025_flight_actual_%s.csv' % timestamp)
        planned_csv_path = self.output_directory / (
            'h2025_flight_planned_%s.csv' % timestamp)

        self._write_csv(
            actual_csv_path,
            ('elapsed_seconds', 'x', 'y', 'z'),
            self.actual_points,
        )
        self._write_csv(
            planned_csv_path,
            ('waypoint_id', 'x', 'y', 'z'),
            self.planned_points,
        )
        self._plot(image_path)
        self.saved = True

        self.get_logger().info('轨迹图保存路径：%s' % image_path)
        self.get_logger().info(
            'actual csv保存路径：%s' % actual_csv_path)
        self.get_logger().info(
            'planned csv保存路径：%s' % planned_csv_path)
        self.get_logger().info(
            '实际记录点数量：%d' % len(self.actual_points))
        self.get_logger().info(
            '计划轨迹点数量：%d' % len(self.planned_points))
        return True


def main(args=None):
    """运行飞行轨迹记录节点，并在退出前保存已有数据。"""
    rclpy.init(args=args)
    node = FlightTrajectoryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_if_needed()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
