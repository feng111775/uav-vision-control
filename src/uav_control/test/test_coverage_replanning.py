"""验证H题连续禁飞区的覆盖路径重新规划。"""

from uav_control.coverage_planner import generate_coverage_path


NO_FLY_CELLS = ['A3B2', 'A3B3', 'A3B4']


def run_replanning_test():
    """生成原始和绕行轨迹，打印数量并返回绕行是否成功。"""
    original = generate_coverage_path()
    detoured = generate_coverage_path(NO_FLY_CELLS)

    blocked_bounds = (
        -12.5,
        -7.5,
        -12.5,
        2.5,
    )
    x_min, x_max, y_min, y_max = blocked_bounds
    crosses_blocked_area = any(
        x_min <= point.x <= x_max and y_min <= point.y <= y_max
        for point in detoured
    )
    success = len(original) == 1551 and not crosses_blocked_area

    print('原航点数量：%d' % len(original))
    print('绕行后航点数量：%d' % len(detoured))
    print('绕行是否成功：%s' % ('是' if success else '否'))
    return success


def test_contiguous_no_fly_cells_are_detoured():
    """三个纵向连续禁飞格应生成不进入禁飞区的绕行轨迹。"""
    assert run_replanning_test()


if __name__ == '__main__':
    if not run_replanning_test():
        raise SystemExit(1)
