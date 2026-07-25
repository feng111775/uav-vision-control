"""测试动物统计的分类、累加和方格去重逻辑。."""

from uav_control.animal_statistics_node import AnimalStatisticsLogic


def test_animal_statistics_by_cell():
    """同一方格重复检测不累加，不同方格需要累加。."""
    logic = AnimalStatisticsLogic()
    logic.update('tiger', 'A3B2', 1)
    logic.update('tiger', 'A3B2', 1)
    logic.update('tiger', 'A4B2', 2)
    logic.update('monkey', 'A5B4', 1)

    assert logic.totals() == {
        '大象': 0,
        '老虎': 3,
        '狼': 0,
        '猴': 1,
        '孔雀': 0,
    }
