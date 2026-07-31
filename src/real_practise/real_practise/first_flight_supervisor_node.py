"""Safety gate and explicit start service; never publishes PX4 input topics."""
import math

import rclpy
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

MISSION_WAIT_START = 'WAIT_START'
PX4_LOCAL_POSITION_TOPICS = (
    '/fmu/out/vehicle_local_position',
    '/fmu/out/vehicle_local_position_v1',
)


class FirstFlightSupervisor(Node):
    def __init__(self):
        super().__init__('first_flight_supervisor_node')
        self.declare_parameter('simulation_mode', False)
        self.declare_parameter('status_timeout', 0.5)
        self.declare_parameter('position_timeout', 0.3)
        self.declare_parameter('attitude_timeout', 0.3)
        self.declare_parameter('mission_state_timeout', 0.5)
        self.declare_parameter('max_xy_speed', 0.15)
        self.declare_parameter('max_z_speed', 0.15)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.status = None
        self.position = None
        self.attitude = None
        self.mission_state = None
        self.status_t = None
        self.position_t = None
        self.attitude_t = None
        self.mission_state_t = None
        self.status_px4_timestamp = 0
        self.position_px4_timestamp = 0
        self.attitude_px4_timestamp = 0
        self.position_topic = None
        self.start_sent = False
        self.ready_pub = self.create_publisher(Bool, '/uav/safety/ready', 10)
        self.start_pub = self.create_publisher(Bool, '/car/mission_start', 10)
        self.state_pub = self.create_publisher(String, '/real_practise/status', 10)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self._status, qos)
        for topic in PX4_LOCAL_POSITION_TOPICS:
            self.create_subscription(
                VehicleLocalPosition,
                topic,
                self._position_callback(topic),
                qos,
            )
        self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self._attitude, qos)
        self.create_subscription(
            String, '/uav/mission/state', self._mission_state, 10)
        self.create_service(Trigger, '/real_practise/start', self._start)
        self.create_timer(0.05, self._publish_gate)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _status(self, msg):
        self.status = msg
        self.status_t = self._now()
        self.status_px4_timestamp = int(msg.timestamp)

    def _position_callback(self, topic_name):
        def callback(msg):
            self.position = msg
            self.position_t = self._now()
            self.position_px4_timestamp = int(msg.timestamp)
            self.position_topic = topic_name
        return callback

    def _attitude(self, msg):
        self.attitude = msg
        self.attitude_t = self._now()
        self.attitude_px4_timestamp = int(msg.timestamp)

    def _mission_state(self, msg):
        self.mission_state = str(msg.data)
        self.mission_state_t = self._now()

    def _timed_out(self, stamp, timeout):
        return stamp is None or self._now() - stamp > timeout

    def _fresh_timestamp(self, value):
        return int(value) > 0

    def _base_checks(self):
        if self._timed_out(self.status_t, float(self.get_parameter('status_timeout').value)):
            return False, 'reject: /fmu/out/vehicle_status_v1 stale'
        if self._timed_out(self.position_t, float(self.get_parameter('position_timeout').value)):
            return False, 'reject: vehicle_local_position stale'
        if self._timed_out(self.attitude_t, float(self.get_parameter('attitude_timeout').value)):
            return False, 'reject: /fmu/out/vehicle_attitude stale'
        if not self._fresh_timestamp(self.status_px4_timestamp):
            return False, 'reject: vehicle_status DDS timestamp invalid'
        if not self._fresh_timestamp(self.position_px4_timestamp):
            return False, 'reject: vehicle_local_position DDS timestamp invalid'
        if not self._fresh_timestamp(self.attitude_px4_timestamp):
            return False, 'reject: vehicle_attitude DDS timestamp invalid'
        if self.status.failsafe:
            return False, 'reject: PX4 failsafe active'
        if self.status.arming_state != VehicleStatus.ARMING_STATE_DISARMED:
            return False, 'reject: vehicle not disarmed'
        if self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return False, 'reject: offboard already active'
        position = self.position
        if not (position.xy_valid and position.z_valid):
            return False, 'reject: local position invalid'
        if not position.heading_good_for_control:
            return False, 'reject: heading invalid for control'
        values = (
            position.x, position.y, position.z,
            position.vx, position.vy, position.vz,
            position.heading,
        )
        if any(not math.isfinite(float(value)) for value in values):
            return False, 'reject: local position contains non-finite values'
        if math.hypot(position.vx, position.vy) > float(self.get_parameter('max_xy_speed').value):
            return False, 'reject: vehicle moving laterally'
        if abs(position.vz) > float(self.get_parameter('max_z_speed').value):
            return False, 'reject: vehicle moving vertically'
        attitude = [float(value) for value in self.attitude.q]
        if len(attitude) != 4 or any(not math.isfinite(value) for value in attitude):
            return False, 'reject: vehicle attitude invalid'
        return True, 'base safety ready'

    def _start_checks(self):
        ok, reason = self._base_checks()
        if not ok:
            return ok, reason
        if self.start_sent:
            return False, 'reject: start already accepted for this node instance'
        timeout = float(self.get_parameter('mission_state_timeout').value)
        if self._timed_out(self.mission_state_t, timeout):
            return False, 'reject: mission state stale'
        if self.mission_state != MISSION_WAIT_START:
            return False, 'reject: setpoint prestream gate not ready (%s)' % self.mission_state
        return True, 'ready: manual start allowed'

    def _publish_gate(self):
        base_ok, _ = self._base_checks()
        start_ok, reason = self._start_checks()
        self.ready_pub.publish(Bool(data=base_ok))
        state = 'READY' if start_ok else reason
        if self.position_topic:
            state = '%s | position_topic=%s' % (state, self.position_topic)
        self.state_pub.publish(String(data=state))

    def _start(self, request, response):
        del request
        ok, reason = self._start_checks()
        if not ok:
            response.success = False
            response.message = reason
            return response
        self.start_pub.publish(Bool(data=True))
        self.start_sent = True
        response.success = True
        response.message = 'mission start accepted; request offboard then arm manually'
        return response



def main(args=None):
    rclpy.init(args=args)
    node = FirstFlightSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
