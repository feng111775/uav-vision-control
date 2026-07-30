#!/usr/bin/env python3
import importlib.util
import math
from pathlib import Path
import sys
import unittest


PACKAGE = Path(sys.argv[1]).resolve()
RUNNER_PATH = PACKAGE / "scripts" / "run_car_robustness_matrix.py"
SPEC = importlib.util.spec_from_file_location("robustness_runner", RUNNER_PATH)
RUNNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER)


def output(**overrides):
    fields = {
        "result": "PASS", "markers": "B,C,D,A", "ab": "5.8", "lap": "34.4",
        "min_v": "0.2", "max_v": "0.25", "max_w": "0.28", "recovery": "0",
        "recovery_time": "0.0", "fault": "no", "final_v": "0.0",
        "recovery_min_v": "0.0"}
    fields.update(overrides)
    result = (
        f"{fields['result']}: markers={fields['markers']} A-B={fields['ab']}s "
        f"lap={fields['lap']}s min_v={fields['min_v']} max_v={fields['max_v']} "
        f"max_w={fields['max_w']} recovery_entries={fields['recovery']} "
        f"recovery_time={fields['recovery_time']}s fault={fields['fault']} "
        f"final_v={fields['final_v']} recovery_min_v={fields['recovery_min_v']}")
    track_overrides = overrides.get("track_overrides", {})
    for segment in ("A_TO_B", "B_TO_C", "C_TO_D", "D_TO_A"):
        metric = {
            "sensor_rms": "0.010", "sensor_max": "0.020",
            "base_rms": "0.030", "base_max": "0.050",
            "lost_max": "0.000", "track_pass": "true"}
        metric.update(track_overrides.get(segment, {}))
        result += (
            f"\nTRACK_METRIC segment={segment} samples=100 "
            f"sensor_rms={metric['sensor_rms']} sensor_max={metric['sensor_max']} "
            f"base_rms={metric['base_rms']} base_max={metric['base_max']} "
            f"lost_samples=0 lost_max={metric['lost_max']} max_line_error=0.5 "
            f"max_angular=0.3 pass={metric['track_pass']}")
    return result


class RobustnessRunnerTest(unittest.TestCase):
    def test_matrix_is_exact_and_unique(self):
        self.assertEqual([item[0] for item in RUNNER.SCENARIOS],
                         [f"S{i}" for i in range(7)])
        self.assertEqual(len({item[1] for item in RUNNER.SCENARIOS}), 7)
        self.assertEqual(RUNNER.SCENARIOS[1][2], 1.470)
        self.assertEqual(RUNNER.SCENARIOS[2][2], 1.530)
        self.assertAlmostEqual(RUNNER.SCENARIOS[3][4], 1.65806278939)
        self.assertAlmostEqual(RUNNER.SCENARIOS[4][4], 1.48352986420)

    def test_runner_only_overrides_pose_and_has_no_publisher(self):
        text = RUNNER_PATH.read_text(encoding="utf-8")
        for key in ("initial_base_x:=", "initial_base_y:=", "initial_yaw:="):
            self.assertIn(key, text)
        for forbidden in (
                "progress.a_", "progress.b_", "progress.c_", "progress.d_",
                "line_pd.", "base_speed_mps:=", "create_publisher", "cmd_vel"):
            self.assertNotIn(forbidden, text)
        self.assertNotIn("scripts/hardware", text)

    def test_valid_real_metrics_pass(self):
        result = RUNNER.parse_result(output())
        self.assertTrue(result["passed"])
        self.assertGreater(result["ab"], 0.0)
        self.assertGreater(result["lap"], 0.0)

    def test_failure_and_nonfinite_propagate(self):
        self.assertFalse(RUNNER.parse_result(output(result="FAIL"))["passed"])
        self.assertFalse(RUNNER.parse_result(output(lap="nan"))["passed"])
        self.assertFalse(RUNNER.parse_result("missing")["passed"])

    def test_strict_time_and_marker_rules(self):
        self.assertFalse(RUNNER.parse_result(output(ab="15.000"))["passed"])
        self.assertFalse(RUNNER.parse_result(output(lap="90.000"))["passed"])
        self.assertFalse(RUNNER.parse_result(output(markers="B,D,C,A"))["passed"])

    def test_speed_and_recovery_rules(self):
        self.assertFalse(RUNNER.parse_result(output(min_v="0.0"))["passed"])
        self.assertFalse(RUNNER.parse_result(
            output(recovery="1", recovery_min_v="0.0"))["passed"])
        self.assertTrue(RUNNER.parse_result(
            output(
                recovery="1", recovery_time="0.2",
                recovery_min_v="0.1"))["passed"])

    def test_node_order_cannot_hide_track_deviation(self):
        result = RUNNER.parse_result(output(track_overrides={
            "C_TO_D": {
                "sensor_rms": "0.080", "sensor_max": "0.120",
                "track_pass": "false"}}))
        self.assertFalse(result["passed"])
        self.assertIn("continuous_track", result["reason"])

    def test_diagonal_d_to_a_cannot_pass(self):
        result = RUNNER.parse_result(output(track_overrides={
            "D_TO_A": {
                "sensor_rms": "0.200", "sensor_max": "0.300",
                "track_pass": "false"}}))
        self.assertFalse(result["passed"])
        self.assertFalse(RUNNER.parse_result(
            output(
                recovery="1", recovery_time="0.0",
                recovery_min_v="0.1"))["passed"])


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]], verbosity=2)
