#!/usr/bin/env python3
"""Exercise the real ROS H7 pipeline through a reconnectable PTY."""

from __future__ import annotations

import argparse
import math
import os
import pty
import shutil
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String


class Monitor(Node):
    def __init__(self):
        super().__init__("visual_pipeline_pty_monitor")
        self.values = {}
        self.history = {}
        for topic in (
                "/vision/h7/detection", "/vision/target/tracked",
                "/vision/landing_error"):
            self.history[topic] = []
            self.create_subscription(
                Float32MultiArray, topic,
                lambda message, name=topic: self.record(
                    name, list(message.data)), 30)
        self.history["/vision/h7/status"] = []
        self.create_subscription(
            String, "/vision/h7/status",
            lambda message: self.record("/vision/h7/status", message.data), 30)

    def record(self, topic, value):
        self.values[topic] = value
        self.history[topic].append(value)

    def wait(self, predicate, timeout=4.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError("timeout waiting for pipeline condition")


def start_node(executable, parameters=()):
    command = ["ros2", "run", "uav_vision", executable, "--ros-args"]
    for parameter in parameters:
        command.extend(["-p", parameter])
    return subprocess.Popen(
        command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, start_new_session=True)


def send(master, line):
    os.write(master, (line + "\n").encode("ascii"))


def send_until(monitor, master, line, predicate, timeout=4.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        send(master, line)
        rclpy.spin_once(monitor, timeout_sec=0.08)
        if predicate():
            return
    raise AssertionError("timeout while injecting PTY frames")


def target(cx=160, cy=120, angle=0.0, confidence=90):
    return "D_TARGET,1,%d,%d,100,60,%.4f,%d" % (
        cx, cy, angle, confidence)


def stop_process(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=3)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--keep-logs", action="store_true")
    args = parser.parse_args()
    temporary = Path(tempfile.mkdtemp(prefix="d-task-vision-pty."))
    link = temporary / "openmv"
    master, slave = pty.openpty()
    link.symlink_to(os.ttyname(slave))
    processes = [
        start_node("h7_bridge_node", (
            "port:=" + str(link), "data_timeout_sec:=0.3")),
        start_node("target_filter_node"),
        start_node("target_predictor_node"),
        start_node("landing_error_node"),
    ]
    rclpy.init()
    monitor = Monitor()
    try:
        time.sleep(1.0)
        send(master, "D_TARGET,0,0,0,0,0,0,0")
        monitor.wait(
            lambda: monitor.values.get("/vision/h7/detection", [1])[0] == 0)

        send_until(
            monitor, master, target(),
            lambda: monitor.values.get("/vision/target/tracked", [0])[0] == 1)
        assert abs(monitor.values["/vision/landing_error"][1]) < 1e-6
        assert abs(monitor.values["/vision/landing_error"][2]) < 1e-6

        positions = (
            (120, 120, 1, -1), (200, 120, 1, 1),
            (160, 80, 2, -1), (160, 160, 2, 1))
        for cx, cy, index, sign in positions:
            send_until(
                monitor, master, target(cx, cy),
                lambda index=index, sign=sign:
                monitor.values.get("/vision/landing_error", [0, 0, 0])[index] *
                sign > 0)

        for angle in (0.0, 0.785, 1.57):
            send_until(
                monitor, master, target(angle=angle),
                lambda: math.isfinite(
                    monitor.values.get("/vision/target/tracked", [0] * 6)[5]))

        for _ in range(4):
            send(master, target(confidence=49))
            time.sleep(0.04)
        monitor.wait(
            lambda: monitor.values.get("/vision/target/tracked", [1])[0] == 0)

        send(master, "D_STATUS,TRACKING")
        monitor.wait(
            lambda: monitor.values.get("/vision/h7/status") == "TRACKING")
        send(master, "D_STATUS,CROSS_INVALID")
        monitor.wait(
            lambda: monitor.values.get("/vision/h7/status") == "CROSS_INVALID")

        # Garbled and partial input must not create a false detection.
        valid_count = sum(
            item[0] == 1 for item in
            monitor.history["/vision/h7/detection"])
        os.write(master, b"\xffbroken\nD_TARGET,1,160")
        time.sleep(0.1)
        os.write(master, b",120,100,60,0.0000,90\n")
        monitor.wait(
            lambda: sum(item[0] == 1 for item in
                        monitor.history["/vision/h7/detection"]) > valid_count)

        # Silence invalidates the complete downstream chain.
        monitor.wait(
            lambda: monitor.values.get("/vision/h7/status") == "STALE", 2.0)
        monitor.wait(
            lambda: monitor.values.get("/vision/target/tracked", [1])[0] == 0)
        monitor.wait(
            lambda: monitor.values.get("/vision/landing_error", [1])[0] == 0)

        os.close(master)
        os.close(slave)
        link.unlink()
        monitor.wait(
            lambda: monitor.values.get("/vision/h7/status") == "DISCONNECTED",
            3.0)
        monitor.wait(
            lambda: monitor.values.get("/vision/target/tracked", [1])[0] == 0)

        master, slave = pty.openpty()
        link.symlink_to(os.ttyname(slave))
        time.sleep(2.2)
        send_until(
            monitor, master, target(),
            lambda: monitor.values.get("/vision/target/tracked", [0])[0] == 1)

        node_names = set(monitor.get_node_names())
        forbidden = {
            "visual_servo_node", "mission_controller_node",
            "vision_offboard_controller"}
        assert not node_names.intersection(forbidden)
        for topic_name in (
                "/fmu/in/offboard_control_mode",
                "/fmu/in/trajectory_setpoint",
                "/fmu/in/vehicle_command"):
            assert not monitor.get_publishers_info_by_topic(topic_name)
        print("PASS: PTY protocol, filtering, directions, timeout and reconnect")
        return 0
    finally:
        monitor.destroy_node()
        rclpy.shutdown()
        for process in processes:
            stop_process(process)
            output = process.stdout.read() if process.stdout else ""
            (temporary / (str(process.pid) + ".log")).write_text(
                output, encoding="utf-8")
        for descriptor in (master, slave):
            try:
                os.close(descriptor)
            except OSError:
                pass
        if not args.keep_logs:
            shutil.rmtree(temporary, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
