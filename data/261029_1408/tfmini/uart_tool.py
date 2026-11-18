"""가짜 PTY UART→실제 TFmini/정지 ROS 노드 통합 시험. 물리 센서·모터 사용 안 함."""
import json,math,os,signal,struct,subprocess,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import Bool,String
root=Path(__file__).resolve().parents[2];out=root/'Cartlidar/sim_render/results/followup_261028_1457/tfmini_uart_tool';out.mkdir(exist_ok=False)
master,slave=os.openpty();port=os.ttyname(slave);children=[];logs=[];report={'physical_hardware_used':False,'port':port}
rclpy.init()
class Probe(Node):
 def __init__(self):
  super().__init__('tfmini_uart_probe');self.stops=[];self.ranges=[];self.reasons=[]
  q=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
  self.create_subscription(Bool,'/perception/obstacle_stop',lambda m:self.stops.append((time.monotonic(),m.data)),q)
  self.create_subscription(String,'/perception/obstacle_stop/reason',lambda m:self.reasons.append((time.monotonic(),m.data)),q)
  self.create_subscription(Range,'/tfmini/range',lambda m:self.ranges.append((time.monotonic(),float(m.range),m.header.stamp.sec+m.header.stamp.nanosec*1e-9)),qos_profile_sensor_data)
 def tick(self,seconds):
  end=time.monotonic()+seconds
  while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.005)
 def feed(self,distance_cm,strength,seconds):
  end=time.monotonic()+seconds;last=None
  while time.monotonic()<end:
   data=b'\x59\x59'+struct.pack('<HHH',distance_cm,strength,2048);packet=data+bytes([sum(data)&255]);last=time.monotonic();os.write(master,packet);self.tick(.02)
  return last
 def stopped(self):return self.stops[-1][1] if self.stops else None
n=Probe()
def start(name,args):
 f=(out/(name+'.log')).open('w');logs.append(f);c=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);children.append((name,c))
try:
 lib=root/'261029_1408/install/perception/lib/perception'
 start('driver',[str(lib/'tfmini'),'--ros-args','-p','port:='+port,'-p','publish_rate:=50.0','-p','stale_timeout_s:=0.2'])
 start('stop',[str(lib/'obstacle_stop'),'--ros-args','-p','invalid_stops:=true','-p','stale_s:=0.2','-p','publish_rate:=10.0'])
 n.tick(3.);report['startup_stopped']=n.stopped() is True;report['startup_ranges']=len(n.ranges)
 n.feed(100,500,.5);report['clear_releases']=n.stopped() is False
 n.feed(100,0,.25);report['invalid_stops']=n.stopped() is True
 n.feed(100,500,.5);report['valid_recovery']=n.stopped() is False
 last=n.feed(100,500,.1);range_count=len(n.ranges);n.tick(.6)
 after=[t for t,s in n.stops if s and t>=last];report['silence_stops']=n.stopped() is True;report['silence_stop_delay_s']=after[0]-last if after else None
 report['range_messages_after_silence']=len([r for r in n.ranges[range_count:] if r[0]>last+.1])
 report['reason_values']=sorted(set(v for _,v in n.reasons));report['range_messages']=len(n.ranges)
 report['pass']=all(report[k] for k in ['startup_stopped','clear_releases','invalid_stops','valid_recovery','silence_stops']) and report['startup_ranges']==0 and report['range_messages_after_silence']==0 and report['silence_stop_delay_s'] is not None and report['silence_stop_delay_s']<=.35
finally:
 for name,c in reversed(children):
  try:os.killpg(c.pid,signal.SIGINT)
  except ProcessLookupError:pass
  try:c.wait(timeout=5)
  except subprocess.TimeoutExpired:os.killpg(c.pid,signal.SIGTERM);c.wait(timeout=5)
 report['processes']=[{'name':name,'pid':c.pid,'exit':c.poll()} for name,c in children]
 (out/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));(out/'stop_events.json').write_text(json.dumps(n.stops));print(json.dumps(report,ensure_ascii=False),flush=True)
 n.destroy_node();rclpy.shutdown();os.close(master);os.close(slave)
 for f in logs:f.close()
