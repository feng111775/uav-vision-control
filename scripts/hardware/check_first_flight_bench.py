#!/usr/bin/env python3
"""Read-only ROS 2 bench checker for first-flight no-prop validation."""
import argparse
import time

import rclpy
from px4_msgs.msg import VehicleAttitude, VehicleCommandAck, VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class BenchChecker(Node):
    def __init__(self):
        super().__init__('first_flight_bench_checker')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.samples = {
            'status': 0,
            'position': 0,
            'attitude': 0,
            'ready': 0,
            'mission_state': 0,
            'ack': 0,
        }
        self.last = {}
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self._mark('status'), qos)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self._mark('position'), qos)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self._mark('position'), qos)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self._mark('attitude'), qos)
        self.create_subscription(Bool, '/uav/safety/ready', self._mark('ready'), 10)
        self.create_subscription(String, '/uav/mission/state', self._mark('mission_state'), 10)
        self.create_subscription(VehicleCommandAck, '/fmu/out/vehicle_command_ack', self._mark('ack'), qos)

    def _mark(self, key):
        def callback(message):
            self.samples[key] += 1
            self.last[key] = message
        return callback

    def snapshot(self):
        state = getattr(self.last.get('mission_state'), 'data', 'N/A')
        ready = getattr(self.last.get('ready'), 'data', False)
        return {
            'samples': dict(self.samples),
            'mission_state': state,
            'ready': ready,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=5.0)
    args = parser.parse_args()
    rclpy.init()
    node = BenchChecker()
    deadline = time.time() + args.duration
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    snapshot = node.snapshot()
    print(snapshot)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
