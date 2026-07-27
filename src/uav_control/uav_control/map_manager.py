"""2025电赛H题9×7巡查地图的纯Python管理逻辑。"""

import re


class GridMap:
    """管理方格编号、地图坐标、禁飞区和巡查记录。

    内部行列索引从0开始，``row``对应B1到B7的y方向，``col``对应A1到A9
    的x方向。地图原点位于地图中心，所有坐标单位均为米。
    """

    CELL_CODE_PATTERN = re.compile(r'^A([1-9][0-9]*)B([1-9][0-9]*)$')

    def __init__(self, rows=7, cols=9, cell_size=5.0,
                 no_fly_cells=None):
        """创建地图，并设置可选的禁飞方格列表。"""
        self._validate_dimensions(rows, cols, cell_size)
        self.rows = rows
        self.cols = cols
        self.cell_size = float(cell_size)
        self.no_fly_cells = set()
        self.visited_cells = set()
        self.set_no_fly_cells(no_fly_cells or [])

    @staticmethod
    def _validate_dimensions(rows, cols, cell_size):
        """检查地图行列数和单格尺寸。"""
        if not isinstance(rows, int) or isinstance(rows, bool) or rows <= 0:
            raise ValueError('rows必须是正整数')
        if not isinstance(cols, int) or isinstance(cols, bool) or cols <= 0:
            raise ValueError('cols必须是正整数')
        try:
            size = float(cell_size)
        except (TypeError, ValueError) as error:
            raise ValueError('cell_size必须是正数') from error
        if size <= 0.0:
            raise ValueError('cell_size必须大于0')

    def validate_indices(self, row, col):
        """验证零基行列索引并返回标准整数元组。"""
        if not isinstance(row, int) or isinstance(row, bool):
            raise ValueError('row必须是整数')
        if not isinstance(col, int) or isinstance(col, bool):
            raise ValueError('col必须是整数')
        if not 0 <= row < self.rows or not 0 <= col < self.cols:
            raise ValueError('方格行列索引超出地图范围')
        return row, col

    def parse_cell_code(self, code):
        """将A1B1格式代码转换为零基``(row, col)``。"""
        if not isinstance(code, str):
            raise ValueError('方格代码必须是字符串')
        match = self.CELL_CODE_PATTERN.fullmatch(code.strip().upper())
        if match is None:
            raise ValueError('方格代码必须使用A<列号>B<行号>格式')

        col = int(match.group(1)) - 1
        row = int(match.group(2)) - 1
        return self.validate_indices(row, col)

    def format_cell_code(self, row, col):
        """将零基行列索引转换为A1B1格式代码。"""
        row, col = self.validate_indices(row, col)
        return 'A%dB%d' % (col + 1, row + 1)

    def normalize_cell(self, cell):
        """将方格代码或``(row, col)``统一转换为索引元组。"""
        if isinstance(cell, str):
            return self.parse_cell_code(cell)
        if isinstance(cell, (tuple, list)) and len(cell) == 2:
            return self.validate_indices(cell[0], cell[1])
        raise ValueError('方格必须使用代码或(row, col)表示')

    def cell_center(self, cell):
        """返回指定方格中心的地图坐标``(x, y)``。"""
        row, col = self.normalize_cell(cell)
        x = (col + 0.5) * self.cell_size - self.width / 2.0
        y = (row + 0.5) * self.cell_size - self.height / 2.0
        return x, y

    @property
    def width(self):
        """返回地图东西方向总宽度。"""
        return self.cols * self.cell_size

    @property
    def height(self):
        """返回地图南北方向总高度。"""
        return self.rows * self.cell_size

    @property
    def bounds(self):
        """返回以地图中心为原点的``(x_min, x_max, y_min, y_max)``。"""
        return (
            -self.width / 2.0,
            self.width / 2.0,
            -self.height / 2.0,
            self.height / 2.0,
        )

    def validate_no_fly_cells(self, cells, require_contiguous=False):
        """验证禁飞区范围、重复项及可选的四邻域连续性。"""
        normalized = [self.normalize_cell(cell) for cell in cells]
        if len(normalized) != len(set(normalized)):
            raise ValueError('禁飞区列表包含重复方格')

        blocked = set(normalized)
        if require_contiguous and blocked:
            reached = set()
            pending = [next(iter(blocked))]
            while pending:
                current = pending.pop()
                if current in reached:
                    continue
                reached.add(current)
                row, col = current
                neighbors = (
                    (row - 1, col), (row + 1, col),
                    (row, col - 1), (row, col + 1),
                )
                pending.extend(
                    neighbor for neighbor in neighbors
                    if neighbor in blocked and neighbor not in reached
                )
            if reached != blocked:
                raise ValueError('禁飞区方格必须四邻域连续')
        return blocked

    def set_no_fly_cells(self, cells, require_contiguous=False):
        """替换禁飞区；已巡查记录中的新禁飞格会被移除。"""
        blocked = self.validate_no_fly_cells(cells, require_contiguous)
        self.no_fly_cells = blocked
        self.visited_cells.difference_update(blocked)

    def is_no_fly(self, cell):
        """判断方格是否属于禁飞区。"""
        return self.normalize_cell(cell) in self.no_fly_cells

    def mark_visited(self, cell):
        """记录已巡查方格，禁止将禁飞格标记为已巡查。"""
        normalized = self.normalize_cell(cell)
        if normalized in self.no_fly_cells:
            raise ValueError('禁飞区不能标记为已巡查')
        self.visited_cells.add(normalized)

    def is_visited(self, cell):
        """判断方格是否已经巡查。"""
        return self.normalize_cell(cell) in self.visited_cells

    def visited_codes(self):
        """按行列顺序返回已巡查方格代码。"""
        return [
            self.format_cell_code(row, col)
            for row, col in sorted(self.visited_cells)
        ]

    def unvisited_codes(self):
        """返回所有尚未巡查且不属于禁飞区的方格代码。"""
        result = []
        for row in range(self.rows):
            for col in range(self.cols):
                cell = (row, col)
                if cell not in self.no_fly_cells and cell not in self.visited_cells:
                    result.append(self.format_cell_code(row, col))
        return result


def main():
    """运行地图解析、禁飞区和巡查记录的简单测试。"""
    grid_map = GridMap(
        no_fly_cells=['A4B3', 'A5B3', 'A6B3'],
    )
    grid_map.validate_no_fly_cells(
        grid_map.no_fly_cells, require_contiguous=True)

    assert grid_map.parse_cell_code('A1B1') == (0, 0)
    assert grid_map.parse_cell_code('A9B7') == (6, 8)
    assert grid_map.format_cell_code(4, 2) == 'A3B5'
    assert grid_map.cell_center('A1B1') == (-20.0, -15.0)
    assert grid_map.cell_center('A9B7') == (20.0, 15.0)
    assert grid_map.is_no_fly('A5B3')

    grid_map.mark_visited('A1B1')
    grid_map.mark_visited((0, 1))
    assert grid_map.is_visited('A2B1')
    assert grid_map.visited_codes() == ['A1B1', 'A2B1']
    assert len(grid_map.unvisited_codes()) == 58

    print('地图尺寸：%d×%d，单格：%.1fm'
          % (grid_map.cols, grid_map.rows, grid_map.cell_size))
    print('禁飞区：%s' % sorted(
        grid_map.format_cell_code(*cell)
        for cell in grid_map.no_fly_cells))
    print('已巡查：%s' % grid_map.visited_codes())
    print('剩余可巡查方格：%d' % len(grid_map.unvisited_codes()))
    print('map_manager测试通过')


if __name__ == '__main__':
    main()
