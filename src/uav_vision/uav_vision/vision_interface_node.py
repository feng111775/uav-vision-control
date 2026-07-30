"""Formal observation publisher; never publishes PX4 topics."""
import math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from uav_interfaces.msg import TargetObservation, LandingError, VisionHealth, MissionVisionState
from .vision_protocol import parse_target, ClockMapper, Confirmation

NAN=float('nan')
class VisionInterfaceNode(Node):
    def __init__(self):
        super().__init__('vision_interface_node')
        best=QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable=QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        state_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tracked=self.create_publisher(TargetObservation,'/vision/target/tracked',best)
        self.landing=self.create_publisher(LandingError,'/vision/landing_error',best)
        self.health_pub=self.create_publisher(VisionHealth,'/vision/health',reliable)
        self.create_subscription(String,'/vision/internal/h7/raw',self.raw_cb,10)
        self.create_subscription(MissionVisionState,'/uav/mission/state',self.state_cb,state_qos)
        self.mapper=ClockMapper(); self.confirm=Confirmation(); self.mode='SEARCH'; self.last=None; self.last_new=0.; self.errors=0; self.frames=0; self.valid_frames=0; self.processing=[]; self.received=[]
        self.create_timer(.05,self.tick); self.create_timer(.2,self.publish_health)
    def state_cb(self,msg): self.mode={0:'MISSION_IDLE',1:'SEARCH',2:'FOLLOW',3:'DROP_ALIGN'}.get(msg.state,'SEARCH')
    def raw_cb(self,msg):
        try: item=parse_target(msg.data)
        except ValueError: self.errors+=1; return
        now=time.monotonic(); self.received.append(now); self.received=self.received[-400:]
        item['sequence']=item['sequence'] if item['sequence'] is not None else self.frames+1
        item['confidence']=max(0.,min(1.,item['confidence']/100.)); item['raw_confidence']=item['confidence']*100
        valid, confirmed, count=self.confirm.ingest({**item,'confidence':item['raw_confidence']})
        if valid: self.valid_frames+=1
        self.frames+=1; self.last_new=now; item.update(measurement_valid=valid,confirmed=confirmed,count=count,received_ns=self.get_clock().now().nanoseconds)
        if item['version']==2:
            stamp_ns,synced=self.mapper.update(item['ticks'],item['received_ns']); item['capture_stamp_ns']=stamp_ns; item['capture_stamp_valid']=synced; self.processing.append(item['processing_us']/1000.)
        else: item['capture_stamp_ns']=item['received_ns']; item['capture_stamp_valid']=False
        self.processing=self.processing[-400:]; self.last=item
        self.publish(item, stale=False)
    def publish(self,item,stale):
        m=TargetObservation(); m.header.stamp=self._stamp(item['capture_stamp_ns'] if not stale else self.get_clock().now().nanoseconds); m.capture_stamp_valid=item.get('capture_stamp_valid',False) and not stale
        m.target_id=1; m.frame_sequence=item.get('sequence',0); m.image_width=320; m.image_height=240
        valid=item.get('measurement_valid',False) and not stale
        m.detected=valid; m.confirmed=item.get('confirmed',False) and valid; m.measurement_valid=valid; m.predicted=False; m.consecutive_valid_frames=item.get('count',0) if valid else 0; m.confidence=item.get('confidence',0.) if valid else 0.
        if valid:
            m.center_x_px=item['cx'];m.center_y_px=item['cy'];m.error_x_norm=(item['cx']-160)/160;m.error_y_norm=(item['cy']-120)/120;m.outer_diameter_px=item['outer'];m.inner_diameter_px=item['inner'];m.target_angle_rad=item['angle'];m.platform_center_valid=True;m.platform_center_x_px=item['cx'];m.platform_center_y_px=item['cy']
        else:
            for n in ('center_x_px','center_y_px','error_x_norm','error_y_norm','outer_diameter_px','inner_diameter_px','target_angle_rad','platform_center_x_px','platform_center_y_px','forward_m','left_m'): setattr(m,n,NAN)
            m.platform_center_valid=False
        m.metric_valid=False;m.forward_m=NAN;m.left_m=NAN; self.tracked.publish(m)
        e=LandingError();e.header=m.header;e.capture_stamp_valid=m.capture_stamp_valid;e.frame_sequence=m.frame_sequence;e.confidence=m.confidence;e.error_x_norm=m.error_x_norm;e.error_y_norm=m.error_y_norm;e.metric_valid=False;e.forward_m=NAN;e.left_m=NAN;e.valid=m.detected and m.confirmed and m.measurement_valid and not m.predicted and m.confidence>=.6 and m.platform_center_valid;self.landing.publish(e)
    def _stamp(self,ns):
        from builtin_interfaces.msg import Time
        t=Time();t.sec=ns//1000000000;t.nanosec=ns%1000000000;return t
    def tick(self):
        if self.last is None or time.monotonic()-self.last_new>.3:
            self.publish(self.last or {'sequence':0,'capture_stamp_ns':self.get_clock().now().nanoseconds},True)
        elif self.last is not None: self.publish(self.last,False)
    def publish_health(self):
        now=time.monotonic();h=VisionHealth();h.header.stamp=self.get_clock().now().to_msg();h.camera_open=bool(self.last);h.frames_received=bool(self.frames);h.algorithm_alive=bool(self.last) and now-self.last_new<=.3;h.protocol_ok=self.errors==0;h.calibration_loaded=False;h.ready_for_mission=h.algorithm_alive;h.ready_for_closed_loop=False;h.performance_gate_passed=False;h.input_fps=self._rate(self.received);h.new_frame_fps=self._rate(self.received);h.valid_detection_fps=0.;h.publish_fps=20.;h.processing_p50_ms=self._pct(self.processing,.5);h.processing_p95_ms=self._pct(self.processing,.95);h.serial_transport_p95_ms=NAN;h.end_to_end_p95_ms=NAN;h.last_measurement_age_ms=(now-self.last_new)*1000 if self.last else NAN;h.protocol_error_count=self.errors;h.dropped_frame_count=0;h.reconnect_count=0;h.detector_backend='fast_v2';h.current_mode=self.mode;h.status_text='READY_PIXEL_ONLY' if h.algorithm_alive else 'WAITING_FOR_CAMERA';self.health_pub.publish(h)
    def _rate(self,a): return (len(a)-1)/(a[-1]-a[0]) if len(a)>1 and a[-1]>a[0] else 0.
    def _pct(self,a,p): return sorted(a)[int((len(a)-1)*p)] if a else NAN
def main(args=None):
    rclpy.init(args=args);n=VisionInterfaceNode()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.shutdown()
