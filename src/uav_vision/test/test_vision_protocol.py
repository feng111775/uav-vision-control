import math
from uav_vision.vision_protocol import parse_target, Confirmation, ClockMapper

def v2(seq, valid=1, confidence=80):
 return 'D_TARGET_V2,%d,%d,1200,SEARCH,%d,200,100,80,48,0.0,%d' % (seq,seq*20,valid,confidence)
def test_v1_v2_and_confirmation():
 assert parse_target('D_TARGET,1,1,2,3,4,0.0,60')['version']==1
 c=Confirmation()
 assert c.ingest(parse_target(v2(1)))[1] is False
 assert c.ingest(parse_target(v2(2)))[1] is False
 assert c.ingest(parse_target(v2(3)))[1] is True
 assert c.ingest(parse_target(v2(4,0)))[2]==0
 assert c.ingest(parse_target(v2(5,1,59)))[2]==0
def test_malformed_and_ticks_wrap():
 try: parse_target('broken')
 except ValueError: pass
 else: assert False
 m=ClockMapper()
 for i in range(8): stamp,ok=m.update(100+i,1000000000+i*1000000)
 assert ok
 m.update((1<<30)-2,2000000000); m.update(2,2005000000)
 assert m.wraps==1
def test_geometry_signs_and_no_metrics():
 x,y=200,180
 assert (x-160)/160>0 and (y-120)/120>0
 assert math.isnan(float('nan'))
