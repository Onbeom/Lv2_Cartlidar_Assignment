"""S3: 별도 프로세스 ROS 노드에 TF·PuckArray를 보내 완료 JSON 검증(모의 입력)."""
import json,math,time,sys
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from robot_interfaces.msg import Puck,PuckArray
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'Cartlidar/sim_render/results/puck_survey_261028/ros'
lay=json.loads((ROOT/'Cartlidar/sim_world/worlds/dice_mission/seed_1/layout.json').read_text())
rclpy.init();n=Node('survey_contract_tester');state={}
q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
def receive(m):state.clear();state.update(json.loads(m.data))
sub=n.create_subscription(String,'/perception/puck_survey',receive,q)
pub=n.create_publisher(PuckArray,'/perception/pucks',10);broad=StaticTransformBroadcaster(n)

def until(pred,seconds=5):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  rclpy.spin_once(n,timeout_sec=.02)
  if pred():return
 raise AssertionError(('대기 초과',state))

def service(name):
 cli=n.create_client(Trigger,'/perception/survey/'+name)
 assert cli.wait_for_service(timeout_sec=5),name
 fut=cli.call_async(Trigger.Request());until(fut.done);return fut.result()

def product(a,b):
 x,y,z,w=a;X,Y,Z,W=b
 return (w*X+x*W+y*Z-z*Y,w*Y-x*Z+y*W+z*X,w*Z+x*Y-y*X+z*W,w*W-x*X-y*Y-z*Z)

until(lambda:bool(state));assert service('start').success
assert not service('finish').success
bad_stamp=PuckArray();bad_stamp.header.frame_id='map';bad_stamp.header.stamp.sec=-1;pub.publish(bad_stamp)
until(lambda:state.get('rejected',{}).get('invalid_stamp',0)>0)
bad=PuckArray();bad.header.frame_id='missing_survey_frame';bad.header.stamp=n.get_clock().now().to_msg();pub.publish(bad)
until(lambda:state.get('rejected',{}).get('missing_tf',0)>0);assert sum(t['n_obs'] for t in state['slots'])==0
bad.header.stamp=Time(nanoseconds=n.get_clock().now().nanoseconds-2_000_000_000).to_msg();pub.publish(bad)
until(lambda:state.get('rejected',{}).get('stale_or_future',0)>0)
poses=[(.4,.5,0.),(1.,2.,math.pi/2),(3.,3.,-1.2)];tfs=[]
for i,(bx,by,yaw) in enumerate(poses):
 tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id=f'survey_base_{i}';tf.header.stamp=n.get_clock().now().to_msg()
 tf.transform.translation.x=bx;tf.transform.translation.y=by;tf.transform.rotation.z=math.sin(yaw/2);tf.transform.rotation.w=math.cos(yaw/2);tfs.append(tf)
broad.sendTransform(tfs)
end=time.monotonic()+.5
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
for k in range(6):
 i=k%3;bx,by,yaw=poses[i];msg=PuckArray();msg.header.frame_id=f'survey_base_{i}';msg.header.stamp=n.get_clock().now().to_msg()
 for p in lay['pucks']:
  o=Puck();dx,dy=p['center'][0]-bx,p['center'][1]-by
  o.pose.position.x=math.cos(yaw)*dx+math.sin(yaw)*dy;o.pose.position.y=-math.sin(yaw)*dx+math.cos(yaw)*dy;o.pose.position.z=p['center'][2]
  worldq=(0.,0.,0.,1.) if p['standing'] else (0.,math.sin(math.pi/4),0.,math.cos(math.pi/4))
  pq=product((0.,0.,math.sin(-yaw/2),math.cos(-yaw/2)),worldq)
  o.pose.orientation.x,o.pose.orientation.y,o.pose.orientation.z,o.pose.orientation.w=pq
  o.orientation_valid=True;o.standing=p['standing'];o.tag_id=-1 if k<3 else p['tag_id'];o.camera=0 if k<3 else 2
  o.color={'red':1,'green':2}[p['color']];o.shape={'cylinder':1,'cuboid':2}[p['shape']];o.confidence=.95;msg.pucks.append(o)
 pub.publish(msg);until(lambda:min(t['n_obs'] for t in state['slots'])>=k+1)
until(lambda:state['complete']);assert service('finish').success
until(lambda:not state.get('enabled',True) and state['frozen'])
f=json.loads((OUT/'puck_survey.json').read_text());assert f['slots']==state['slots'];maxerr=0.
for p in lay['pucks']:
 t=next(t for t in f['slots'] if t['box']==p['box'] and t['side']==p['side'])
 assert (t['id'],t['color'],t['shape'],t['pose'])==(p['tag_id'],p['color'],p['shape'],'standing' if p['standing'] else 'lying')
 maxerr=max(maxerr,math.hypot(t['x_mm']-p['center'][0]*1000,t['y_mm']-p['center'][1]*1000))
assert maxerr<.01
result={'pass':True,'slots':12,'complete':f['complete'],'max_position_error_mm':maxerr,'missing_tf_registered':0,'stale_rejected':True,'incomplete_finish_refused':True,'file_topic_slots_identical':True,'frames_tested':poses}
old_id=state['survey_id']
assert service('reset').success;until(lambda:not state['complete'] and sum(t['n_obs'] for t in state['slots'])==0);result['reset']=True;assert state['survey_id']!=old_id;result['survey_id_changes_on_reset']=True
audit=[json.loads(line) for line in (OUT/'puck_survey.observations.jsonl').read_text().splitlines()]
current=[r for r in audit if r.get('survey_id')==f['survey_id']];assert len(current)==72;result['independent_observation_records']=len(current)
(OUT/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps(result,ensure_ascii=False));n.destroy_node();rclpy.shutdown()
