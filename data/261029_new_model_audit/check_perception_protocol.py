"""새 URDF TF와 인지 노드 연결만 확인하는 합성 영상 시험. 실제 렌더/실물 성능 판정 아님."""
import sys,json,math,argparse
from pathlib import Path
import numpy as np,cv2,yaml
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
R=Path('/home/pa4/git/lv2_next_module');O=R/'Cartlidar/sim_render/results/new_model_261029';sys.path.insert(0,str(R/'261028_1459'));sys.path.insert(0,str(R/'Cartlidar/sim_render'));from replay_a1_code import ArmReplayer,tf_msg,info_msg,img_msg,M,to_time
from model_frames import ModelFrames
p=json.loads((O/'perception_model_profile.json').read_text());model=ModelFrames(p['urdf']);q={j:0. for j,d in model.joints.items() if d['kind']!='fixed'};q.update(p['preview_pose']);poses=model.poses(q);spec=yaml.safe_load((R/'261029_1030/urdf_261028/waffle_4wd_description_261028/config/sensors.yaml').read_text());Ks={}
for name,s in [('d435',spec['d435']['sim']),('arm',spec['arm_camera']['sim'])]:
 w,h=s['width'],s['height'];f=w/(2*math.tan(s.get('color_hfov_rad',s.get('hfov_rad'))/2));Ks[name]=dict(w=w,h=h,K=dict(fx=f,fy=f,cx=w/2,cy=h/2))
front='top_camera_front_color_optical_frame';arm='arm_camera_optical_frame'
def project(frame,pt,info):
 c=np.linalg.inv(poses[frame])@np.r_[pt,1];K=info['K'];return np.array([K['fx']*c[0]/c[2]+K['cx'],K['fy']*c[1]/c[2]+K['cy']]),c
# 시험점은 모델의 두 FOV 공통 영역에서 선택. 인지 입력은 합성 영상/깊이뿐.
candidates=[]
for x in np.arange(.4,.91,.025):
 for z in [.10,.13,.16]:
  pt=np.array([x,0,z]);uv1,c1=project(front,pt,Ks['d435']);uv2,c2=project(arm,pt,Ks['arm'])
  if c1[2]>.15 and c2[2]>.02 and all(30<=uv[0]<k['w']-30 and 30<=uv[1]<k['h']-30 for uv,k in [(uv1,Ks['d435']),(uv2,Ks['arm'])]):candidates.append((abs(x-.5),pt))
if not candidates:raise RuntimeError('두 카메라 공통 시험 시야 없음')
pt=min(candidates,key=lambda v:v[0])[1];_,fc=project(front,pt,Ks['d435']);sp=fc[:3]*(1-.015/np.linalg.norm(fc[:3]));surf=(poses[front]@np.r_[sp,1])[:3];uv,dc=project(front,surf,Ks['d435']);f=Ks['d435']['K']['fx'];d=Ks['d435'];color=np.full((d['h'],d['w'],3),170,np.uint8);depth=np.zeros((d['h'],d['w']),np.float32);u,v=map(round,uv);rx=max(5,round(f*.015/dc[2]));ry=max(8,round(f*.03/dc[2]));cv2.rectangle(color,(u-rx,v-ry),(u+rx,v+ry),(0,0,210),-1);depth[v-ry:v+ry+1,u-rx:u+rx+1]=dc[2]
a=Ks['arm'];uv,ac=project(arm,pt,a);image=np.full((a['h'],a['w'],3),170,np.uint8);radius=max(8,round(a['K']['fx']*.015/ac[2]));cv2.circle(image,tuple(map(round,uv)),radius,(0,0,210),-1);expected_offset=(uv-np.array([a['w']/2,a['h']/2]))/np.array([a['w']/2,a['h']/2]);fixture=O/'synthetic_protocol';fixture.mkdir(exist_ok=True);(fixture/'render_a1.json').write_text(json.dumps({'synthetic_only':True,'d435':{'K':d['K']},'arm':{'K':a['K']}}));cv2.imwrite(str(fixture/'front.png'),color);cv2.imwrite(str(fixture/'arm.png'),image)
class Replayer(ArmReplayer):
 def __init__(self,args):
  super().__init__(args);self.joint_pub=self.create_publisher(JointState,'/joint_states',10);self.world=np.eye(4)
 def publish(self,sc,t,arm_data=None):
  st=to_time(t);js=JointState();js.header.stamp=st;js.name=list(q);js.position=list(q.values());self.joint_pub.publish(js);self.tf_pub.publish(TFMessage(transforms=[tf_msg(self.world,'odom','base_link',st)]))
  self.info_pub.publish(info_msg(d['K'],d['w'],d['h'],st,front));self.depth_pub.publish(img_msg(depth,'32FC1',st,front));self.color_pub.publish(img_msg(color,'bgr8',st,front))
  if arm_data is not None:self.arm_info_pub.publish(info_msg(a['K'],a['w'],a['h'],st,arm));self.arm_pub.publish(img_msg(image,'bgr8',st,arm))
args=argparse.Namespace(render=fixture,qos_depth=10,t0=1000.,settle=.05,max_wait=1.,arm_frame=arm,calibrated_d435=False);rclpy.init();n=Replayer(args);n.set_clock(1000.);n.spin_for(1.);t=1001.;rows=[]
try:
 for k in range(2):
  yaw=.4*k;T=np.eye(4);T[:3,:3]=[[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]];T[:3,3]=[.1*k,.2*k,.00675];n.world=T;n.set_mode(M.OFF);n.set_mode(M.SEARCH_FRONT)
  for _ in range(8):got,t=n.step({},t)
  n.set_mode(M.SEARCH_FRONT|M.ARM_ALIGN)
  for j in range(8):
   got,t=n.step({},t,{})
   if j<3:continue
   pe=float(np.linalg.norm(np.array(got['object'])-pt)) if got and got['object_found'] else None;oe=float(np.linalg.norm(np.array(got['offset'][:2])-expected_offset)) if got and got['offset_valid'] else None;rows.append(dict(odom_case=k,frame=j,object_found=bool(got and got['object_found']),offset_valid=bool(got and got['offset_valid']),position_error_m=pe,offset_error=oe,pass_frame=pe is not None and pe<=.03 and oe is not None and oe<=.05))
 report=dict(synthetic_only=True,model=p['urdf'],target_base_link_m=pt.tolist(),image_sizes={k:[v['w'],v['h']] for k,v in Ks.items()},odom_child='base_link',rows=rows,ok=sum(x['pass_frame'] for x in rows),n=len(rows),pass_protocol=len(rows)==10 and all(x['pass_frame'] for x in rows),note='새TF/카메라메시지 연결만 검사. 렌더/실물/A1성능 판정 아님');(O/'perception_protocol_summary.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report));assert report['pass_protocol']
finally:n.destroy_node();rclpy.shutdown()
