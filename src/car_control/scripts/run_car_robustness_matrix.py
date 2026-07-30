#!/usr/bin/env python3
"""Run the stage-4B full-lap robustness matrix without publishing commands."""

import argparse
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time


SCENARIOS = (
    ("S0", "standard", 1.500, 1.800, 1.57079632679),
    ("S1", "left_3cm", 1.470, 1.800, 1.57079632679),
    ("S2", "right_3cm", 1.530, 1.800, 1.57079632679),
    ("S3", "yaw_ccw_5deg", 1.500, 1.800, 1.65806278939),
    ("S4", "yaw_cw_5deg", 1.500, 1.800, 1.48352986420),
    ("S5", "combined_one", 1.475, 1.800, 1.62079632679),
    ("S6", "combined_two", 1.525, 1.800, 1.52079632679),
)

RESULT_RE = re.compile(
    r"(?P<result>PASS|FAIL): markers=(?P<markers>\S+) "
    r"A-B=(?P<ab>[-+\w.]+)s lap=(?P<lap>[-+\w.]+)s "
    r"min_v=(?P<min_v>[-+\w.]+) max_v=(?P<max_v>[-+\w.]+) "
    r"max_w=(?P<max_w>[-+\w.]+) recovery_entries=(?P<recovery>\d+) "
    r"recovery_time=(?P<recovery_time>[-+\w.]+)s fault=(?P<fault>\w+) "
    r"final_v=(?P<final_v>[-+\w.]+) recovery_min_v=(?P<recovery_min_v>[-+\w.]+)")
START_RE = re.compile(
    r"line following started: auto=true wait=(?P<wait>[-+\w.]+)s "
    r"odom_sequence=(?P<odom>\d+\.\.\d+) "
    r"sensor_snapshot_sequence=(?P<sensor>\d+\.\.\d+)")
TRACK_RE = re.compile(
    r"TRACK_METRIC segment=(?P<segment>\S+) samples=(?P<samples>\d+) "
    r"sensor_rms=(?P<sensor_rms>[-+\w.]+) sensor_max=(?P<sensor_max>[-+\w.]+) "
    r"base_rms=(?P<base_rms>[-+\w.]+) base_max=(?P<base_max>[-+\w.]+) "
    r"lost_samples=(?P<lost_samples>\d+) lost_max=(?P<lost_max>[-+\w.]+) "
    r"max_line_error=(?P<max_line_error>[-+\w.]+) "
    r"max_angular=(?P<track_max_angular>[-+\w.]+) pass=(?P<track_pass>\w+)")


def parse_result(output):
    match = RESULT_RE.search(output)
    if not match:
        return {"passed": False, "reason": "acceptance result line missing"}
    data = match.groupdict()
    try:
        for key in (
                "ab", "lap", "min_v", "max_v", "max_w",
                "recovery_time", "final_v", "recovery_min_v"):
            data[key] = float(data[key])
        data["recovery"] = int(data["recovery"])
    except ValueError:
        return {"passed": False, "reason": "non-numeric acceptance metric"}
    numeric = [
        data[key] for key in (
            "ab", "lap", "min_v", "max_v", "max_w",
            "recovery_time", "final_v", "recovery_min_v")]
    checks = {
        "node_order": data["markers"] == "B,C,D,A",
        "a_to_b_strict": 0.0 < data["ab"] < 15.0,
        "lap_strict": 0.0 < data["lap"] < 90.0,
        "finite": all(math.isfinite(value) for value in numeric),
        "running_speed": data["min_v"] > 0.0,
        "speed_limit": data["max_v"] <= 0.35 + 1.0e-6 and data["max_w"] <= 3.0 + 1.0e-6,
        "recovery_metrics": (
            data["recovery"] == 0 and data["recovery_time"] == 0.0 or
            data["recovery"] > 0 and data["recovery_time"] > 0.0 and
            0.0 < data["recovery_min_v"] <= 0.35 + 1.0e-6),
        "no_fault": data["fault"] == "no",
        "stopped": abs(data["final_v"]) < 1.0e-3,
        "node_pass": data["result"] == "PASS",
    }
    failed = [name for name, passed in checks.items() if not passed]
    track_metrics = {}
    for match in TRACK_RE.finditer(output):
        metric = match.groupdict()
        for key in (
                "sensor_rms", "sensor_max", "base_rms", "base_max",
                "lost_max", "max_line_error", "track_max_angular"):
            metric[key] = float(metric[key])
        metric["samples"] = int(metric["samples"])
        metric["lost_samples"] = int(metric["lost_samples"])
        track_metrics[metric["segment"]] = metric
    expected_segments = {"A_TO_B", "B_TO_C", "C_TO_D", "D_TO_A"}
    if set(track_metrics) != expected_segments:
        failed.append("track_segments_missing")
    elif any(
            metric["track_pass"] != "true" or
            not all(math.isfinite(metric[key]) for key in (
                "sensor_rms", "sensor_max", "base_rms", "base_max", "lost_max")) or
            metric["sensor_rms"] > 0.025 or metric["sensor_max"] > 0.060 or
            metric["lost_max"] > 0.10
            for metric in track_metrics.values()):
        failed.append("continuous_track")
    data.update(
        passed=not failed,
        reason="ok" if not failed else "failed checks: " + ",".join(failed),
        track_metrics=track_metrics)
    return data


def stop_process_group(process):
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=5.0)
    time.sleep(1.0)


def run_scenario(scenario, log_dir, wall_timeout_s):
    code, name, base_x, base_y, yaw = scenario
    launch_log = log_dir / f"{code}_launch.log"
    acceptance_log = log_dir / f"{code}_acceptance.log"
    command = [
        "ros2", "launch", "d_system_sim", "d_task_car_autonomous.launch.py",
        "headless:=true", "auto_start:=true", "stop_after_finish:=true",
        f"initial_base_x:={base_x:.12f}", f"initial_base_y:={base_y:.12f}",
        f"initial_yaw:={yaw:.12f}"]
    scenario_env = os.environ.copy()
    # Keep Gazebo Transport discovery on loopback.  The robustness runner is
    # strictly local, and multicast attempts on an unavailable interface can
    # otherwise stall simulation long enough to invalidate wall-time bounds.
    scenario_env.setdefault("GZ_IP", "127.0.0.1")
    with launch_log.open("w", encoding="utf-8") as launch_stream:
        launch = subprocess.Popen(
            command, stdout=launch_stream, stderr=subprocess.STDOUT,
            start_new_session=True, text=True, env=scenario_env)
        try:
            time.sleep(4.0)
            completed = subprocess.run(
                ["ros2", "run", "car_control", "car_full_lap_acceptance_node",
                 "--ros-args", "-p", f"wall_timeout_s:={wall_timeout_s}",
                 "-p", f"initial_base_x:={base_x:.12f}",
                 "-p", f"initial_base_y:={base_y:.12f}",
                 "-p", f"initial_yaw:={yaw:.12f}",
                 "-p", f"csv_path:={log_dir / (code + '_track.csv')}"],
                capture_output=True, text=True, timeout=wall_timeout_s + 10.0,
                check=False, env=scenario_env)
            acceptance_output = completed.stdout + completed.stderr
            acceptance_log.write_text(acceptance_output, encoding="utf-8")
            result = parse_result(acceptance_output)
            result["returncode"] = completed.returncode
            if completed.returncode != 0:
                result["passed"] = False
                result["reason"] = f"acceptance exit {completed.returncode}: {result['reason']}"
        except subprocess.TimeoutExpired:
            result = {"passed": False, "reason": "scenario wall timeout"}
            acceptance_log.write_text("scenario wall timeout\\n", encoding="utf-8")
        finally:
            stop_process_group(launch)
    launch_output = launch_log.read_text(encoding="utf-8", errors="replace")
    start = START_RE.search(launch_output)
    if start:
        odom_range = start.group("odom")
        sensor_range = start.group("sensor")
        result.update(
            startup_wait=float(start.group("wait")),
            odom_sequences=odom_range,
            sensor_sequences=sensor_range)
        odom_first, odom_last = (int(value) for value in odom_range.split(".."))
        sensor_first, sensor_last = (int(value) for value in sensor_range.split(".."))
        if odom_last - odom_first < 2 or sensor_last - sensor_first < 2:
            result["passed"] = False
            result["reason"] += "; fewer than three distinct startup inputs"
    else:
        result.update(startup_wait=math.nan, odom_sequences="", sensor_sequences="")
        result["passed"] = False
        result["reason"] += "; auto-start evidence missing"
    result.update(code=code, name=name, base_x=base_x, base_y=base_y, yaw=yaw)
    return result


def print_summary(results):
    print("scenario | A-B | lap | min_v | recovery | fault | result")
    for item in results:
        print(
            f"{item['code']} | {item.get('ab', math.nan):.3f} | "
            f"{item.get('lap', math.nan):.3f} | {item.get('min_v', math.nan):.3f} | "
            f"{item.get('recovery', 0)}/{item.get('recovery_time', 0.0):.3f}s | "
            f"{item.get('fault', 'unknown')} | {'PASS' if item['passed'] else 'FAIL'}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log-dir", type=Path, default=Path("/tmp/car_robustness_matrix"))
    parser.add_argument("--wall-timeout-s", type=float, default=120.0)
    args = parser.parse_args()
    args.log_dir.mkdir(parents=True, exist_ok=True)
    results = []
    for scenario in SCENARIOS:
        print(f"RUN {scenario[0]} {scenario[1]}", flush=True)
        result = run_scenario(scenario, args.log_dir, args.wall_timeout_s)
        results.append(result)
        print_summary(results)
        if not result["passed"]:
            print(f"STOP {result['code']}: {result['reason']}", file=sys.stderr)
            return 1
    print_summary(results)
    return 0


if __name__ == "__main__":
    sys.exit(main())
