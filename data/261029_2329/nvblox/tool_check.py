"""합성 평면 깊이로 nvblox CUDA·ROS 입력·ESDF 출력을 확인한다. 실물/발제 판정이 아니다."""
import json,os,signal,subprocess,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from nvblox_msgs.msg import DistanceMapSlice
here=Path(__file__).resolve().parent;root=here.parents[1]
mode=os.environ.get('NVBLOX_INPUT_MODE','synthetic')
if mode not in ('synthetic','isaac'):raise ValueError('입력 모드 오류')
out=root/'Cartlidar/sim_render/results/followup_261028_1457'/('nvblox_tool' if mode=='synthetic' else 'nvblox_self_filtered_new_model');out.mkdir(exist_ok=False)
frame='nvblox_test_optical';global_frame='odom';K=[385.,0.,320.,0.,385.,240.,0.,0.,1.];T=np.eye(4);T[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]];T[2,3]=.3
encoding='32FC1';image=np.ones((480,640),np.float32);source=None
if mode=='isaac':
 source=root/'Cartlidar/sim_render/results/followup_261028_1457/camera_new_model_spec/nvblox_input.json';metadata=json.loads(source.read_text())
 depth=np.load(root/'Cartlidar/sim_render/results/followup_261028_1457/nvblox_self_filter_cached/top_camera_front_color_optical_frame_filtered_m.npy');valid=np.isfinite(depth)&(depth>=.15)&(depth<=4.)
 image=np.where(valid,np.rint(np.clip(depth,0,4)*1000),0).astype(np.uint16);encoding='16UC1';global_frame='test_world';T=np.array(metadata['T_world_cam']);k=metadata['K'];K=[k['fx'],0.,k['cx'],0.,k['fy'],k['cy'],0.,0.,1.]
 np.save(out/'input_depth_mm.npy',image)
def matrix_quat(R):
 q=np.empty(4);trace=np.trace(R)
 if trace>0:
  v=2*np.sqrt(trace+1);q[:]=[(R[2,1]-R[1,2])/v,(R[0,2]-R[2,0])/v,(R[1,0]-R[0,1])/v,.25*v]
 else:
  i=int(np.argmax(np.diag(R)));j=(i+1)%3;k=(j+1)%3;v=2*np.sqrt(1+R[i,i]-R[j,j]-R[k,k]);q[i]=.25*v;q[j]=(R[j,i]+R[i,j])/v;q[k]=(R[k,i]+R[i,k])/v;q[3]=(R[k,j]-R[j,k])/v
 return q/np.linalg.norm(q)
log=(out/'node.log').open('w');child=None;report={'simulation_only':True,'input':'합성1m평면' if mode=='synthetic' else '새261028모델 drive·near105mm·URDF자기영역제거 정지1시야, 기준자세TF 사용·VO미사용','encoding':encoding,'source':str(source) if source else None,'camera_K':K,'T_global_camera':T.tolist(),'valid_input_pixels':int(np.count_nonzero(image))}
rclpy.init()
class Probe(Node):
 def __init__(self):
  super().__init__('nvblox_input_probe');self.slices=[]
  self.depth=self.create_publisher(Image,'/nvblox_test/depth',qos_profile_sensor_data)
  self.info=self.create_publisher(CameraInfo,'/nvblox_test/camera_info',qos_profile_sensor_data)
  self.create_subscription(DistanceMapSlice,'/nvblox_probe/static_map_slice',self.slices.append,qos_profile_sensor_data)
  self.broadcaster=StaticTransformBroadcaster(self);t=TransformStamped();t.header.frame_id=global_frame;t.child_frame_id=frame;t.header.stamp=self.get_clock().now().to_msg();t.transform.translation.x=float(T[0,3]);t.transform.translation.y=float(T[1,3]);t.transform.translation.z=float(T[2,3]);q=matrix_quat(T[:3,:3])
  t.transform.rotation.x=float(q[0]);t.transform.rotation.y=float(q[1]);t.transform.rotation.z=float(q[2]);t.transform.rotation.w=float(q[3]);self.broadcaster.sendTransform(t)
 def tick(self,seconds):
  end=time.monotonic()+seconds
  while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.02)
 def publish_depth(self):
  now=self.get_clock().now().to_msg();m=Image();m.header.stamp=now;m.header.frame_id=frame;m.height=image.shape[0];m.width=image.shape[1];m.encoding=encoding;m.is_bigendian=False;m.step=image.shape[1]*image.dtype.itemsize;m.data=image.tobytes();self.depth.publish(m)
  info=CameraInfo();info.header=m.header;info.height=image.shape[0];info.width=image.shape[1];info.k=K;info.p=[K[0],0.,K[2],0.,0.,K[4],K[5],0.,0.,0.,1.,0.];info.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.];info.distortion_model='plumb_bob';info.d=[0.]*5;self.info.publish(info)
n=Probe()
try:
 child=subprocess.Popen(['/opt/ros/lyrical/lib/nvblox_ros/nvblox_node','--ros-args','--params-file',str(here/'tool.yaml'),'-r','__node:=nvblox_probe','-p','global_frame:='+global_frame,'-p','map_clearing_frame_id:='+global_frame,'-p','esdf_slice_bounds_visualization_attachment_frame_id:='+global_frame,'-r','camera_0/depth/image:=/nvblox_test/depth','-r','camera_0/depth/camera_info:=/nvblox_test/camera_info'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 n.tick(8.)
 if child.poll() is not None:raise RuntimeError('nvblox 초기화 종료: '+str(child.returncode))
 report['depth_subscribers']=n.depth.get_subscription_count();report['info_subscribers']=n.info.get_subscription_count()
 for _ in range(90):n.publish_depth();n.tick(1/15.)
 n.tick(3.);report['slices']=len(n.slices);report['input_frames']=90
 if n.slices:
  s=n.slices[-1];data=np.array(s.data,np.float32).reshape(s.height,s.width);known=np.isfinite(data)&(data!=s.unknown_value)
  np.savez_compressed(out/'distance_slice.npz',distance=data,known=known,origin=[s.origin.x,s.origin.y,s.origin.z],resolution=s.resolution,unknown=s.unknown_value)
  report.update({'output_frame':s.header.frame_id,'width':s.width,'height':s.height,'resolution_m':s.resolution,'origin':[s.origin.x,s.origin.y,s.origin.z],'unknown_value':s.unknown_value,'known_cells':int(known.sum()),'known_min_m':float(data[known].min()) if known.any() else None,'known_max_m':float(data[known].max()) if known.any() else None})
  samples=[]
  for x,y in ([ (.25,0),(.5,0),(.75,0),(1.,0)] if mode=='synthetic' else []):
   col=int(np.floor((x-s.origin.x)/s.resolution));row=int(np.floor((y-s.origin.y)/s.resolution));valid=0<=row<s.height and 0<=col<s.width and known[row,col];samples.append({'x':x,'y':y,'expected_distance_m':abs(1.-x),'observed':bool(valid),'distance_m':float(data[row,col]) if valid else None})
  report['samples']=samples
 report['tool_pass']=report.get('depth_subscribers',0)>0 and report.get('info_subscribers',0)>0 and report.get('known_cells',0)>0 and report.get('slices',0)>0
finally:
 if child:
  try:os.killpg(child.pid,signal.SIGINT)
  except ProcessLookupError:pass
  try:child.wait(timeout=8)
  except subprocess.TimeoutExpired:
   os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
  report['node_exit']=child.returncode
 (out/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(report,ensure_ascii=False),flush=True);n.destroy_node();rclpy.shutdown();log.close()
