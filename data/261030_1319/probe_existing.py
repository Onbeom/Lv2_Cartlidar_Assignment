"""실제 인지 발행값을 실제 B0로 연결하는 모의 시험. 정답은 채점에만 사용."""
import os,sys,json,time,signal,subprocess,math
from pathlib import Path
from types import SimpleNamespace
import numpy as np,yaml
ROOT=Path('/home/pa4/git/lv2_next_module');OUT=ROOT/'Cartlidar/sim_render/results/integration_261030_1319'
sys.path.insert(0,str(ROOT/'261029_a1_new_model'))
from replay import ArmReplayer,M,pointed
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from robot_interfaces.msg import PuckArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from ament_index_python.packages import get_package_share_directory
class Probe(ArmReplayer):
 def __init__(self,a):
  super().__init__(a);self.report={};self.puck_rows=[];self.transforms={}
  self.create_subscription(String,'/perception/puck_survey',self.survey,QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
  self.create_subscription(PuckArray,'/perception/pucks',self.pucks,10)
  self.broad=StaticTransformBroadcaster(self);tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='odom';tf.transform.rotation.w=1.;self.broad.sendTransform(tf)
 def survey(self,m):self.report=json.loads(m.data)
 def pucks(self,m):
  self.puck_rows.append(dict(stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,frame=m.header.frame_id,pucks=[dict(id=p.tag_id,camera=p.camera,color=p.color,shape=p.shape,orientation_valid=p.orientation_valid,position=[p.pose.position.x,p.pose.position.y,p.pose.position.z]) for p in m.pucks]))
 def publish(self,sc,t,arm=None):
  self.transforms[round(t,6)]=sc['T_world_base_link'];super().publish(sc,t,arm)
 def until(self,pred,seconds=5):
  end=time.monotonic()+seconds
  while time.monotonic()<end:
   self.set_clock(self.now_s);self.spin_for(.02)
   if pred():return
  raise RuntimeError('대기초과: '+str(self.report)[:300])
 def service(self,name):
  cli=self.create_client(Trigger,'/perception/survey/'+name)
  if not cli.wait_for_service(timeout_sec=5):raise RuntimeError(name+' 서비스 없음')
  fut=cli.call_async(Trigger.Request());self.until(fut.done);res=fut.result();self.destroy_client(cli);return res
 def advance(self,seconds):
  for _ in range(math.ceil(seconds/.05)):
   self.set_clock(self.now_s+.05);self.spin_for(.025)
processes={};logs=[]
def start(key,cmd):
 f=(OUT/(key+'.log')).open('a');logs.append(f);p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes[key]=p;return p
 def_unused=None
def stop(key):
 p=processes.get(key)
 if p and p.poll() is None:
  os.killpg(p.pid,signal.SIGINT)
  try:p.wait(timeout=6)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=3)
def mode(n,bits):
 if not n.mode_cli.wait_for_service(timeout_sec=5):raise RuntimeError('인지모드 서비스 없음')
 req=M();req.mode=bits;f=n.mode_cli.call_async(req);n.until(f.done)
 if not f.result().success:raise RuntimeError('인지모드 설정 거부')
def run():
 cfg=OUT/'config';import shutil
 shutil.copytree(Path(get_package_share_directory('perception'))/'config',cfg)
 data=yaml.safe_load((cfg/'perception.yaml').read_text());data['perception_node']['ros__parameters']['image_qos']='reliable';(cfg/'perception.yaml').write_text(yaml.safe_dump(data))
 urdf=ROOT/'261029_1030/urdf_261028/waffle_4wd_description_261028/urdf/waffle_4wd.urdf';rsp=OUT/'rsp.yaml';rsp.write_text(yaml.safe_dump({'robot_state_publisher':{'ros__parameters':{'robot_description':urdf.read_text(),'use_sim_time':True}}}))
 commands={'rsp':['ros2','run','robot_state_publisher','robot_state_publisher','--ros-args','--params-file',str(rsp)],'perception':['ros2','launch','perception','perception.launch.py','use_sim_time:=true','use_tfmini:=false','config_dir:='+str(cfg)],'survey':['ros2','run','perception_survey','puck_survey','--ros-args','-p','use_sim_time:=true','-p','world_frame:=map','-p','output_path:='+str(OUT/'survey.json')]}
 for key,cmd in commands.items():start(key,cmd)
 rclpy.init();a=SimpleNamespace(render=ROOT/'Cartlidar/sim_render/results/a1_new_model_261029/design',out=OUT,t0=1000.,qos_depth=10,joint_delay_s=0.,arm_frame='arm_camera_optical_frame',settle=.05,max_wait=1.,front_depth_empty=False)
 n=Probe(a);n.set_clock(a.t0);n.until(lambda:bool(n.report),10);n.spin_for(1.)
 summary={'pass':False,'detector':'color fallback + AprilTag, not final YOLO engine','pose_source':'stored input FK, map-odom identity; not SLAM performance','scenes':[],'checks':{}}
 try:
  t=1001.;chosen=[s for s in n.meta['scenes'] if s['light_level']==1.]
  import cv2
  for sc in chosen:
   mode(n,0);n.service('start');n.until(lambda:n.report.get('enabled',False));n.d435=(cv2.imread(str(a.render/sc['d435_images']['color'])),np.nan_to_num(np.load(a.render/sc['d435_images']['depth']).astype(np.float32),nan=0.,posinf=0.))
   mode(n,1);obj=None
   for _ in range(4):
    got,t=n.step(sc,t)
    if got and got['object_found']:obj=got['object']
   target=None
   if obj is not None:
    world=(np.array(sc['T_world_base_link'])@np.r_[obj,1])[:3];ds=[(np.linalg.norm(world-np.array(p['center'])),p['name']) for p in sc['pucks']];dist,target=min(ds)
    if dist>.05:target=None
   mode(n,7);rows=[]
   for _ in range(4):
    got,t=n.step(sc,t,sc['arm_frames'][0]);rows.append(got)
   n.spin_for(.25);slots=[s for s in n.report['slots'] if s['n_obs']>0];finish=n.service('finish');out=[pointed(g['offset'],sc['arm_frames'][0]['pucks'])[0] for g in rows if g and g['offset_valid']]
   result=dict(scene=sc['name'],selected=target,registered_slots=len(slots),known_ids=sorted({s['id'] for s in slots if s['id'] is not None}),offset_targets=out,partial_finish_refused=not finish.success,pass_=bool(slots and target and out and all(p==target for p in out) and not finish.success))
   summary['scenes'].append(result);(OUT/(sc['name']+'_survey.json')).write_text(json.dumps(n.report,ensure_ascii=False,indent=2));print(json.dumps(result,ensure_ascii=False),flush=True)
  # 재전송/오래된영상과 입력중단: 새로운 관측이 생기면 안 된다.
  sc=chosen[-1];n.spin_for(.3);before=sum(s['n_obs'] for s in n.report['slots']);last_t=t-.1;n.advance(1.);n.publish(sc,last_t-2,sc['arm_frames'][0]);n.advance(.6)
  summary['checks']['stale_input_no_new_observations']=sum(s['n_obs'] for s in n.report['slots'])==before
  a.arm_frame='missing_integration_optical';t=n.now_s+.1;got,t=n.step(sc,t,sc['arm_frames'][0]);summary['checks']['missing_arm_tf_no_valid_offset']=not(got and got['offset_valid']);a.arm_frame='arm_camera_optical_frame'
  n.service('reset');n.until(lambda:not n.report.get('enabled') and sum(s['n_obs'] for s in n.report['slots'])==0);summary['checks']['reset_clears_registry']=True
  mode(n,0);old=n.report['survey_id'];stop('survey');start('survey',commands['survey']);n.until(lambda:n.report.get('survey_id')!=old and not n.report.get('enabled') and sum(s['n_obs'] for s in n.report.get('slots',[]))==0,10);summary['checks']['survey_process_restart_disabled_empty']=True
  stop('perception');start('perception',commands['perception']);n.advance(.5);n.spin_for(3.);mode(n,7);n.results.clear();n.advance(.6);summary['checks']['perception_restart_no_old_lock']=not any(g['offset_valid'] for g in n.results)
  n.service('start');mode(n,1);t=n.now_s+.1
  for _ in range(4):got,t=n.step(sc,t)
  mode(n,7);restored=[]
  for _ in range(4):got,t=n.step(sc,t,sc['arm_frames'][0]);restored.append(bool(got and got['offset_valid']))
  n.spin_for(.3);summary['checks']['new_input_recovers_after_restart']=any(restored) and any(s['n_obs']>0 for s in n.report['slots'])
  # 실제 수신 PuckArray와 B0 관측감사 좌표를 촬영시각별로 대조한다.
  max_error=0.;compared=0
  raw={p['stamp_ns']:p for p in n.puck_rows}
  for line in (OUT/'survey.observations.jsonl').read_text().splitlines():
   q=json.loads(line);p=raw.get(q['stamp_ns']);T=n.transforms.get(round(q['stamp_ns']/1e9,6))
   if not p or T is None:continue
   ob=q['observation_m'];candidates=[]
   for pp in p['pucks']:
    if pp['id']==ob['id'] and {0:'front_d435',1:'rear_d435',2:'arm'}.get(pp['camera'])==ob['camera']:
     v=(np.array(T)@np.r_[pp['position'],1])[:2];candidates.append(float(np.linalg.norm(v-[ob['x'],ob['y']])))
   if candidates:max_error=max(max_error,min(candidates));compared+=1
  summary['audit_compared']=compared;summary['max_audit_transform_error_m']=max_error;summary['checks']['actual_input_audit_transform_match']=compared>0 and max_error<=1e-6
  summary['checks']['one_perception_publisher']=n.count_publishers('/perception/pucks')==1
  summary['pass']=all(x['pass_'] for x in summary['scenes']) and all(summary['checks'].values())
 except Exception as e:summary['error']=repr(e);raise
 finally:
  (OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2));(OUT/'actual_pucks.jsonl').write_text('\n'.join(json.dumps(p) for p in n.puck_rows)+'\n');(OUT/'input_transforms.json').write_text(json.dumps(n.transforms));n.destroy_node();rclpy.shutdown()
 return 0 if summary['pass'] else 1
if __name__=='__main__':
 def term(signum,frame):raise KeyboardInterrupt('시험 종료 신호')
 signal.signal(signal.SIGTERM,term)
 try:code=run()
 finally:
  for key in list(processes)[::-1]:stop(key)
  for f in logs:f.close()
 sys.exit(code)
