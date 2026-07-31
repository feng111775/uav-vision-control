#!/usr/bin/env python3
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, VehicleAttitude


class MockPx4(Node):
    def __init__(self, scenario: str):
        super().__init__('mock_first_flight_px4')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.scenario = scenario
        self.status_pub = self.create_publisher(VehicleStatus, '/fmu/out/vehicle_status_v1', qos)
        self.position_pub = self.create_publisher(VehicleLocalPosition, '/fmu/out/vehicle_local_position', qos)
        self.attitude_pub = self.create_publisher(VehicleAttitude, '/fmu/out/vehicle_attitude', qos)
        self.timer = self.create_timer(0.05, self.publish_once)
        self.start = time.time()

    def micros(self):
        if self.scenario == 'zero_timestamp':
            return 0
        return int(time.time_ns() // 1000)

    def publish_once(self):
        now_us = self.micros()

        status = VehicleStatus()
        status.timestamp = now_us
        status.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        status.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
        status.failsafe = False
        if self.scenario == 'armed':
            status.arming_state = VehicleStatus.ARMING_STATE_ARMED
        if self.scenario == 'offboard':
            status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        if self.scenario == 'failsafe':
            status.failsafe = True
        self.status_pub.publish(status)

        if self.scenario != 'position_stale':
            position = VehicleLocalPosition()
            position.timestamp = now_us
            position.x = 0.0
            position.y = 0.0
            position.z = 0.0
            position.vx = 0.0
            position.vy = 0.0
            position.vz = 0.0
            position.heading = 0.0
            position.xy_valid = True
            position.z_valid = True
            position.v_xy_valid = True
            position.v_z_valid = True
            position.heading_good_for_control = True
            if self.scenario == 'position_invalid':
                position.xy_valid = False
            if self.scenario == 'moving':
                position.vx = 0.3
            self.position_pub.publish(position)

        if self.scenario != 'attitude_stale':
            attitude = VehicleAttitude()
            attitude.timestamp = now_us
            attitude.q = [1.0, 0.0, 0.0, 0.0]
            if self.scenario == 'attitude_invalid':
                attitude.q = [math.nan, 0.0, 0.0, 0.0]
            self.attitude_pub.publish(attitude)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', required=True)
    parser.add_argument('--duration', type=float, default=6.0)
    args = parser.parse_args()
    rclpy.init()
    node = MockPx4(args.scenario)
    end = time.time() + args.duration
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
