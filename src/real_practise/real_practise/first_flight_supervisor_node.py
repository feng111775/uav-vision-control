"""Safety gate and explicit start service; never publishes PX4 input topics."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, VehicleAttitude
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class FirstFlightSupervisor(Node):
    def __init__(self):
        super().__init__('first_flight_supervisor_node')
        self.declare_parameter('simulation_mode', False)
        self.declare_parameter('status_timeout', 0.5)
        self.declare_parameter('position_timeout', 0.3)
        self.declare_parameter('attitude_timeout', 0.3)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.status = self.position = self.attitude = None
        self.status_t = self.position_t = self.attitude_t = None
        self.start_sent = False
        self.ready_pub = self.create_publisher(Bool, '/uav/safety/ready', 10)
        self.start_pub = self.create_publisher(Bool, '/car/mission_start', 10)
        self.state_pub = self.create_publisher(String, '/real_practise/status', 10)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self._status, qos)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self._position, qos)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self._attitude, qos)
        self.create_service(Trigger, '/real_practise/start', self._start)
        self.create_timer(0.05, self._publish_gate)

    def _now(self): return self.get_clock().now().nanoseconds * 1e-9
    def _status(self, msg): self.status, self.status_t = msg, self._now()
    def _position(self, msg): self.position, self.position_t = msg, self._now()
    def _attitude(self, msg): self.attitude, self.attitude_t = msg, self._now()

    def checks(self):
        now = self._now()
        if self.status is None or now - self.status_t > float(self.get_parameter('status_timeout').value): return False, 'PX4 status stale'
        if self.position is None or now - self.position_t > float(self.get_parameter('position_timeout').value): return False, 'position stale'
        if self.attitude is None or now - self.attitude_t > float(self.get_parameter('attitude_timeout').value): return False, 'attitude stale'
        if self.status.failsafe: return False, 'PX4 failsafe'
        p = self.position
        if not (p.xy_valid and p.z_valid and p.v_xy_valid and p.v_z_valid and p.heading_good_for_control): return False, 'position invalid'
        if any(not math.isfinite(v) for v in (p.x, p.y, p.z, p.vx, p.vy, p.vz, p.heading)): return False, 'non-finite position'
        if math.hypot(p.vx, p.vy) > 0.15 or abs(p.vz) > 0.15: return False, 'vehicle moving'
        q = list(self.attitude.q)
        if len(q) != 4 or any(not math.isfinite(float(v)) for v in q): return False, 'attitude invalid'
        return True, 'OK'

    def _publish_gate(self):
        ok, reason = self.checks()
        self.ready_pub.publish(Bool(data=ok))
        self.state_pub.publish(String(data=('READY' if ok else 'WAIT: ' + reason)))

    def _start(self, request, response):
        ok, reason = self.checks()
        if not ok or self.start_sent:
            response.success = False
            response.message = reason if not ok else 'start already sent'
            return response
        self.start_pub.publish(Bool(data=True))
        self.start_sent = True
        response.success = True
        response.message = 'mission start accepted; manual arm required'
        return response


def main(args=None):
    rclpy.init(args=args); node = FirstFlightSupervisor()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
