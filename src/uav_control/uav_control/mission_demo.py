"""H题任务规划层完整链路的纯Python集成测试。"""

try:
    from .coverage_planner import CoveragePlanner
    from .map_manager import GridMap
    from .mission_manager import MissionManager, MissionState
    from .trajectory_generator import TrajectoryGenerator, TrajectoryPoint
except ImportError:
    from coverage_planner import CoveragePlanner
    from map_manager import GridMap
    from mission_manager import MissionManager, MissionState
    from trajectory_generator import TrajectoryGenerator, TrajectoryPoint


NO_FLY_CELLS = ('A9B5', 'A9B6', 'A9B7')
EXPECTED_STATE_SEQUENCE = (
    MissionState.IDLE,
    MissionState.TAKEOFF,
    MissionState.EXECUTE,
    MissionState.RETURN,
    MissionState.LAND,
    MissionState.COMPLETE,
)


def record_state(manager, state_history):
    """在任务状态发生变化时记录并输出状态。"""
    if not state_history or state_history[-1] != manager.current_state:
        state_history.append(manager.current_state)
        print('状态变化：%s' % manager.current_state.value)


def run_demo():
    """执行地图、覆盖规划、轨迹生成和任务管理完整测试。"""
    print('=== H题任务规划层集成测试 ===')

    # 1. 创建9×7地图，并设置第一版规划器可安全处理的连续禁飞区。
    grid_map = GridMap(
        rows=7,
        cols=9,
        cell_size=0.5,
        no_fly_cells=NO_FLY_CELLS,
    )
    grid_map.validate_no_fly_cells(
        NO_FLY_CELLS, require_contiguous=True)
    print('地图信息：%d列 × %d行，单格尺寸=%.1fm'
          % (grid_map.cols, grid_map.rows, grid_map.cell_size))
    print('禁飞区：%s' % '、'.join(NO_FLY_CELLS))

    # 2. 生成跳过禁飞格的蛇形覆盖路线。
    planner = CoveragePlanner(
        patrol_height=1.2,
        speed=0.5,
        max_duration=300.0,
    )
    plan_result = planner.plan(grid_map)
    assert plan_result.success, (
        '覆盖规划失败：%s' % plan_result.failure_reason)
    assert len(plan_result.waypoints) == 60
    assert len(plan_result.covered_cells) == 60
    assert all(
        cell not in plan_result.covered_cells for cell in NO_FLY_CELLS)
    print('覆盖规划：成功')
    print('航点数量：%d' % len(plan_result.waypoints))
    print('路径长度：%.2fm，预计时间：%.2fs'
          % (plan_result.path_length, plan_result.estimated_duration))

    # 3. 将覆盖航点插值为带位置、速度、航向和时间的连续轨迹。
    generator = TrajectoryGenerator(speed=0.5, sample_period=0.1)
    trajectory = generator.generate(plan_result.waypoints)
    assert trajectory
    assert all(isinstance(point, TrajectoryPoint) for point in trajectory)
    assert all(
        current.timestamp < following.timestamp
        for current, following in zip(trajectory, trajectory[1:])
    )
    print('轨迹生成：成功')
    print('轨迹点数量：%d' % len(trajectory))
    print('轨迹总时间：%.2fs' % trajectory[-1].timestamp)

    # 4. 加载任务，并记录从IDLE到TAKEOFF的启动过程。
    manager = MissionManager(arrival_radius=0.15)
    state_history = []
    record_state(manager, state_history)
    manager.load_trajectory(trajectory)
    manager.start()
    record_state(manager, state_history)

    # 5. 仿真无人机精确移动到当前轨迹目标，触发任务自动推进。
    while manager.current_state in (
            MissionState.TAKEOFF, MissionState.EXECUTE):
        target = manager.get_current_target()
        assert target is not None
        manager.update_position(target.x, target.y, target.z)
        manager.step()
        record_state(manager, state_history)

    assert manager.current_state == MissionState.RETURN

    # 6. RETURN阶段回到轨迹首点，然后推进LAND和COMPLETE。
    return_target = manager.get_current_target()
    assert return_target is trajectory[0]
    manager.update_position(
        return_target.x, return_target.y, return_target.z)
    manager.step()
    record_state(manager, state_history)
    assert manager.current_state == MissionState.LAND

    manager.step()
    record_state(manager, state_history)

    # 7. 验证完整任务状态顺序及最终结果。
    assert tuple(state_history) == EXPECTED_STATE_SEQUENCE
    assert manager.current_state == MissionState.COMPLETE
    assert manager.current_waypoint_index == len(trajectory)
    print('状态序列：%s'
          % ' → '.join(state.value for state in state_history))
    print('任务结果：完整链路测试通过')
    return {
        'map': grid_map,
        'plan': plan_result,
        'trajectory': trajectory,
        'manager': manager,
        'state_history': state_history,
    }


def main():
    """运行集成演示。"""
    run_demo()


if __name__ == '__main__':
    main()
