"""使用Matplotlib显示H题巡查地图、航线和无人机位置。."""

from geometry_msgs.msg import PoseStamped

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

import rclpy
from rclpy.node import Node

from .coverage_planner import generate_coverage_path
from .map_manager import GridMap


class MissionVisualizer(Node):
    """绘制9×7任务地图并实时更新无人机地图位置。."""

    DEFAULT_NO_FLY_CELLS = ['A3B2', 'A3B3', 'A3B4']

    def __init__(self):
        """创建地图窗口、位置订阅和界面刷新定时器。."""
        super().__init__('mission_visualizer')

        self.declare_parameter(
            'no_fly_cells', self.DEFAULT_NO_FLY_CELLS)
        self.no_fly_cells = list(
            self.get_parameter('no_fly_cells').value)
        self.grid_map = GridMap(
            rows=7,
            cols=9,
            cell_size=0.5,
            no_fly_cells=self.no_fly_cells,
        )
        if self.no_fly_cells:
            self.grid_map.validate_no_fly_cells(
                self.no_fly_cells, require_contiguous=True)
        self.trajectory = generate_coverage_path(self.no_fly_cells)

        self.figure, self.axes = plt.subplots(figsize=(10, 7))
        self.position_artist = None
        self.latest_position = None
        self._draw_map()

        self.position_subscription = self.create_subscription(
            PoseStamped,
            '/mission/current_position',
            self.position_callback,
            10,
        )
        self.refresh_timer = self.create_timer(0.1, self.refresh_plot)

        plt.ion()
        plt.show(block=False)
        self.get_logger().info(
            '任务地图窗口已启动：禁飞区=%s'
            % ','.join(self.no_fly_cells))

    def _draw_map(self):
        """绘制普通区域、禁飞格、起飞点和覆盖航线。."""
        width = self.grid_map.cols * self.grid_map.cell_size
        height = self.grid_map.rows * self.grid_map.cell_size
        self.axes.set_facecolor('#f2f2f2')

        for code in self.no_fly_cells:
            row, col = self.grid_map.parse_cell_code(code)
            x = col * self.grid_map.cell_size
            y = row * self.grid_map.cell_size
            self.axes.add_patch(Rectangle(
                (x, y),
                self.grid_map.cell_size,
                self.grid_map.cell_size,
                facecolor='#e74c3c',
                edgecolor='#922b21',
                alpha=0.75,
                zorder=2,
            ))
            self.axes.text(
                x + self.grid_map.cell_size / 2.0,
                y + self.grid_map.cell_size / 2.0,
                code,
                horizontalalignment='center',
                verticalalignment='center',
                fontsize=8,
                zorder=3,
            )

        route_x = [0.0] + [point.x for point in self.trajectory]
        route_y = [0.0] + [point.y for point in self.trajectory]
        self.axes.plot(
            route_x,
            route_y,
            color='#2471a3',
            linewidth=1.5,
            label='Coverage route',
            zorder=4,
        )
        self.axes.scatter(
            [0.0],
            [0.0],
            color='#27ae60',
            marker='*',
            s=180,
            label='Takeoff point',
            zorder=6,
        )
        self.position_artist, = self.axes.plot(
            [],
            [],
            marker='o',
            markersize=9,
            color='#f39c12',
            markeredgecolor='black',
            linestyle='None',
            label='UAV position',
            zorder=7,
        )

        x_boundaries = [
            index * self.grid_map.cell_size
            for index in range(self.grid_map.cols + 1)
        ]
        y_boundaries = [
            index * self.grid_map.cell_size
            for index in range(self.grid_map.rows + 1)
        ]
        self.axes.set_xticks(x_boundaries)
        self.axes.set_yticks(y_boundaries)
        self.axes.grid(
            visible=True,
            which='major',
            color='#666666',
            linewidth=0.8,
            zorder=1,
        )
        self.axes.set_xticklabels(
            [''] + ['A%d' % index for index in range(1, 10)])
        self.axes.set_yticklabels(
            [''] + ['B%d' % index for index in range(1, 8)])
        self.axes.set_xlim(-0.15, width + 0.05)
        self.axes.set_ylim(-0.15, height + 0.05)
        self.axes.set_aspect('equal', adjustable='box')
        self.axes.set_xlabel('Map X / East (m)')
        self.axes.set_ylabel('Map Y / North (m)')
        self.axes.set_title('H Mission 9x7 Patrol Map')
        self.axes.legend(loc='upper right')
        self.figure.tight_layout()

    def position_callback(self, message):
        """保存map坐标系中的最新无人机位置。."""
        if message.header.frame_id not in ('', 'map'):
            self.get_logger().warning(
                '忽略非map坐标系位置：%s' % message.header.frame_id,
                throttle_duration_sec=5.0,
            )
            return
        self.latest_position = (
            message.pose.position.x,
            message.pose.position.y,
        )

    def refresh_plot(self):
        """以10 Hz刷新无人机位置和Matplotlib窗口事件。."""
        if self.latest_position is not None:
            x, y = self.latest_position
            self.position_artist.set_data([x], [y])
        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()

    def destroy_node(self):
        """关闭地图窗口并销毁ROS节点。."""
        plt.close(self.figure)
        return super().destroy_node()


def main(args=None):
    """启动H题任务地图可视化节点。."""
    rclpy.init(args=args)
    node = MissionVisualizer()
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
