"""2025电赛H题巡查区域的纯Python覆盖路径规划逻辑。"""

from dataclasses import dataclass, field
import math

try:
    from .map_manager import GridMap
except ImportError:
    from map_manager import GridMap


@dataclass
class CoveragePlanResult:
    """保存一次覆盖规划的结果和统计信息。"""

    success: bool
    waypoints: list = field(default_factory=list)
    covered_cells: list = field(default_factory=list)
    path_length: float = 0.0
    estimated_duration: float = 0.0
    failure_reason: str = ''


class CoveragePlanner:
    """生成蛇形全覆盖航点，并检查禁飞区和任务时间。"""

    def __init__(self, patrol_height=1.2, speed=2.0,
                 max_duration=300.0):
        """保存巡航高度、速度和最大允许任务时间。"""
        self.patrol_height = self._positive_float(
            patrol_height, 'patrol_height')
        self.speed = self._positive_float(speed, 'speed')
        self.max_duration = self._positive_float(
            max_duration, 'max_duration')

    @staticmethod
    def _positive_float(value, name):
        """将参数转换为有限正浮点数。"""
        try:
            result = float(value)
        except (TypeError, ValueError) as error:
            raise ValueError('%s必须是正数' % name) from error
        if not math.isfinite(result) or result <= 0.0:
            raise ValueError('%s必须是有限正数' % name)
        return result

    def _generate_waypoints(self, grid_map):
        """按行生成蛇形航点，禁飞格不加入输出。"""
        waypoints = []
        covered_cells = []

        for row in range(grid_map.rows):
            if row % 2 == 0:
                columns = range(grid_map.cols)
            else:
                columns = range(grid_map.cols - 1, -1, -1)

            for col in columns:
                cell = (row, col)
                if grid_map.is_no_fly(cell):
                    continue

                cell_code = grid_map.format_cell_code(row, col)
                x, y = grid_map.cell_center(cell)
                waypoints.append({
                    'cell_code': cell_code,
                    'x': x,
                    'y': y,
                    'z': self.patrol_height,
                })
                covered_cells.append(cell_code)

        return waypoints, covered_cells

    @staticmethod
    def _segment_intersects_rectangle(start, end, bounds):
        """使用线段裁剪法判断二维线段是否接触矩形禁飞区。"""
        x0, y0 = start
        x1, y1 = end
        x_min, x_max, y_min, y_max = bounds
        dx = x1 - x0
        dy = y1 - y0
        lower = 0.0
        upper = 1.0

        # 每组(p, q)表示线段相对矩形一条边的约束。
        constraints = (
            (-dx, x0 - x_min),
            (dx, x_max - x0),
            (-dy, y0 - y_min),
            (dy, y_max - y0),
        )
        for p, q in constraints:
            if p == 0.0:
                if q < 0.0:
                    return False
                continue
            ratio = q / p
            if p < 0.0:
                lower = max(lower, ratio)
            else:
                upper = min(upper, ratio)
            if lower > upper:
                return False
        return True

    @staticmethod
    def _cell_bounds(grid_map, cell):
        """返回禁飞格的二维矩形边界。"""
        row, col = cell
        size = grid_map.cell_size
        map_x_min, _, map_y_min, _ = grid_map.bounds
        return (
            map_x_min + col * size,
            map_x_min + (col + 1) * size,
            map_y_min + row * size,
            map_y_min + (row + 1) * size,
        )

    def _path_crosses_no_fly_cell(self, grid_map, waypoints):
        """检查任意相邻航点的直线是否穿过禁飞格。"""
        if not grid_map.no_fly_cells:
            return False

        for first, second in zip(waypoints, waypoints[1:]):
            start = (first['x'], first['y'])
            end = (second['x'], second['y'])
            for blocked_cell in grid_map.no_fly_cells:
                bounds = self._cell_bounds(grid_map, blocked_cell)
                if self._segment_intersects_rectangle(start, end, bounds):
                    return True
        return False

    def _route_is_clear(self, grid_map, points):
        """判断一组二维折线路段是否全部避开禁飞格。"""
        for start, end in zip(points, points[1:]):
            for blocked_cell in grid_map.no_fly_cells:
                bounds = self._cell_bounds(grid_map, blocked_cell)
                if self._segment_intersects_rectangle(start, end, bounds):
                    return False
        return True

    @staticmethod
    def _route_length(points):
        """计算二维折线长度。"""
        return sum(
            math.hypot(end[0] - start[0], end[1] - start[1])
            for start, end in zip(points, points[1:])
        )

    def _detour_points(self, grid_map, start, end):
        """从禁飞区上下左右四侧选择最短的安全两点绕行路线。"""
        blocked_bounds = [
            self._cell_bounds(grid_map, cell)
            for cell in grid_map.no_fly_cells
        ]
        x_min = min(bounds[0] for bounds in blocked_bounds)
        x_max = max(bounds[1] for bounds in blocked_bounds)
        y_min = min(bounds[2] for bounds in blocked_bounds)
        y_max = max(bounds[3] for bounds in blocked_bounds)
        margin = grid_map.cell_size * 0.1
        map_x_min, map_x_max, map_y_min, map_y_max = grid_map.bounds

        candidates = [
            [(start[0], y_min - margin), (end[0], y_min - margin)],
            [(start[0], y_max + margin), (end[0], y_max + margin)],
            [(x_min - margin, start[1]), (x_min - margin, end[1])],
            [(x_max + margin, start[1]), (x_max + margin, end[1])],
        ]

        valid_routes = []
        for detour in candidates:
            route = [start] + detour + [end]
            inside_map = all(
                map_x_min <= point[0] <= map_x_max
                and map_y_min <= point[1] <= map_y_max
                for point in detour
            )
            if inside_map and self._route_is_clear(grid_map, route):
                valid_routes.append(route)

        if not valid_routes:
            return None
        shortest = min(valid_routes, key=self._route_length)
        return shortest[1:-1]

    def _insert_detours(self, grid_map, waypoints):
        """为穿越禁飞区的相邻蛇形航点插入临时绕行点。"""
        if not grid_map.no_fly_cells or len(waypoints) < 2:
            return waypoints

        routed = [waypoints[0]]
        detour_index = 0
        for waypoint in waypoints[1:]:
            previous = routed[-1]
            segment = [
                (previous['x'], previous['y']),
                (waypoint['x'], waypoint['y']),
            ]
            if self._route_is_clear(grid_map, segment):
                routed.append(waypoint)
                continue

            detour = self._detour_points(
                grid_map, segment[0], segment[1])
            if detour is None:
                raise ValueError('cannot find detour around no-fly cells')
            for x, y in detour:
                routed.append({
                    'cell_code': 'DETOUR_%d' % detour_index,
                    'x': x,
                    'y': y,
                    'z': self.patrol_height,
                })
                detour_index += 1
            routed.append(waypoint)
        return routed

    @staticmethod
    def _path_length(waypoints):
        """计算所有相邻航点之间的三维欧氏距离总和。"""
        total = 0.0
        for first, second in zip(waypoints, waypoints[1:]):
            dx = second['x'] - first['x']
            dy = second['y'] - first['y']
            dz = second['z'] - first['z']
            total += math.sqrt(dx * dx + dy * dy + dz * dz)
        return total

    def plan(self, grid_map):
        """规划并验证覆盖路径，返回``CoveragePlanResult``。"""
        if not isinstance(grid_map, GridMap):
            return CoveragePlanResult(
                success=False,
                failure_reason='grid_map must be a GridMap',
            )

        try:
            # 再次校验地图中的禁飞格，避免外部直接破坏对象状态。
            grid_map.validate_no_fly_cells(grid_map.no_fly_cells)
            waypoints, covered_cells = self._generate_waypoints(grid_map)
            waypoints = self._insert_detours(grid_map, waypoints)
        except (TypeError, ValueError) as error:
            return CoveragePlanResult(
                success=False,
                failure_reason=str(error),
            )

        if self._path_crosses_no_fly_cell(grid_map, waypoints):
            return CoveragePlanResult(
                success=False,
                failure_reason='path crosses no-fly cell',
            )

        path_length = self._path_length(waypoints)
        estimated_duration = path_length / self.speed
        if estimated_duration > self.max_duration:
            return CoveragePlanResult(
                success=False,
                waypoints=waypoints,
                covered_cells=covered_cells,
                path_length=path_length,
                estimated_duration=estimated_duration,
                failure_reason='estimated duration exceeds 300 seconds',
            )

        return CoveragePlanResult(
            success=True,
            waypoints=waypoints,
            covered_cells=covered_cells,
            path_length=path_length,
            estimated_duration=estimated_duration,
        )


def generate_coverage_path(no_fly_cells=[]):
    """生成H题连续覆盖轨迹，可选三个连续禁飞方格。"""
    grid_map = GridMap(
        rows=7,
        cols=9,
        cell_size=5.0,
        no_fly_cells=no_fly_cells,
    )
    if no_fly_cells:
        if len(no_fly_cells) != 3:
            raise ValueError('H题禁飞区必须包含三个方格')
        grid_map.validate_no_fly_cells(
            no_fly_cells, require_contiguous=True)

    planner = CoveragePlanner()
    result = planner.plan(grid_map)
    if not result.success:
        raise RuntimeError('覆盖路径规划失败：%s' % result.failure_reason)

    try:
        from .trajectory_generator import TrajectoryGenerator
    except ImportError:
        from trajectory_generator import TrajectoryGenerator
    return TrajectoryGenerator(speed=planner.speed).generate(
        result.waypoints)


def main():
    """运行默认地图、连续禁飞区和非法地图三个简单测试。"""
    planner = CoveragePlanner()

    # 测试1：默认地图应生成全部63个航点。
    default_result = planner.plan(GridMap())
    assert default_result.success
    assert len(default_result.waypoints) == 63
    assert len(default_result.covered_cells) == 63
    print('测试1通过：默认地图，航点=%d，路径长度=%.2fm，预计时间=%.2fs'
          % (len(default_result.waypoints),
             default_result.path_length,
             default_result.estimated_duration))

    # 测试2：行端三个连续禁飞格可安全跳过，不需要跨越禁飞区。
    blocked_map = GridMap(
        no_fly_cells=['A9B5', 'A9B6', 'A9B7'],
    )
    blocked_map.validate_no_fly_cells(
        blocked_map.no_fly_cells, require_contiguous=True)
    blocked_result = planner.plan(blocked_map)
    assert blocked_result.success
    assert len(blocked_result.waypoints) == 60
    assert len(blocked_result.covered_cells) == 60
    assert all(
        code not in blocked_result.covered_cells
        for code in ('A9B5', 'A9B6', 'A9B7')
    )
    print('测试2通过：连续禁飞区，航点=%d，路径长度=%.2fm，预计时间=%.2fs'
          % (len(blocked_result.waypoints),
             blocked_result.path_length,
             blocked_result.estimated_duration))

    # 测试3：越界禁飞格由GridMap拒绝，并转换为失败结果。
    try:
        invalid_map = GridMap(no_fly_cells=['A10B1'])
        invalid_result = planner.plan(invalid_map)
    except ValueError as error:
        invalid_result = CoveragePlanResult(
            success=False,
            failure_reason=str(error),
        )
    assert not invalid_result.success
    print('测试3通过：非法地图被拒绝，原因=%s'
          % invalid_result.failure_reason)


if __name__ == '__main__':
    main()
