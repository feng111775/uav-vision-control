"""H7Plus文本协议解析测试。"""

import unittest

from uav_vision.h7_bridge_node import H7BridgeNode


class TestH7Parser(unittest.TestCase):
    """验证合法数据和各类错误数据。"""

    def test_valid_target_line(self):
        """协议示例应转换为规定的数组顺序。"""
        self.assertEqual(
            H7BridgeNode.parse_line(
                'TARGET,1,160,120,50,48,2400,90'),
            [1.0, 160.0, 120.0, 50.0, 48.0, 2400.0, 90.0],
        )

    def test_invalid_target_lines(self):
        """格式、类型或范围错误的数据必须被拒绝。"""
        invalid_lines = [
            'TARGET,1,160',
            'TARGET,yes,160,120,50,48,2400,90',
            'OTHER,1,160,120,50,48,2400,90',
            'TARGET,2,160,120,50,48,2400,90',
            'TARGET,1,-1,120,50,48,2400,90',
            'TARGET,1,160,120,-50,48,2400,90',
            'TARGET,1,nan,120,50,48,2400,90',
            'TARGET,1,160,120,50,48,2400,101',
        ]
        for line in invalid_lines:
            with self.subTest(line=line):
                with self.assertRaises(ValueError):
                    H7BridgeNode.parse_line(line)
