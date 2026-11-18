"""분석식 깊이 입력으로 nvblox 동적 재구성을 확인한다. 실물 시험 아님."""
from pathlib import Path
import json,signal,subprocess,time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo,PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from nvblox_msgs.msg import DistanceMapSlice

ROOT=Path(__file__).resolve().parents[1];HERE=Path(__file__).resolve().parent
OUT=ROOT/'Cartlidar/sim_render/results/followup_261028_1457/dynamic_261030_design';OUT.mkdir(exist_ok=False)
W,H=320,240;FX=200.;v,u=np.indices((H,W));rays=np.stack((np.ones((H,W)),-(u-W/2)/FX,-(v-H/2)/FX),-1);origin=np.array([0.,0.,.45])
def depth_at(t):
    candidates=[np.full((H,W),3.)]
    with np.errstate(divide='ignore',invalid='ignore'):
        for axis,value in [(1,-.5),(1,.5),(2,0.)]:
            d=(value-origin[axis])/rays[:,:,axis];candidates.append(np.where(d>0,d,np.inf))
    depth=np.minimum.reduce(candidates)
    y=None
    if 8<=t<11:y=.9*(11-t)/3
    elif 11<=t<15:y=0.
    elif 15<=t<18:y=-.9*(t-15)/3
    if y is not None:
        lo=np.array([1.025,y-.25,0.]);hi=np.array([1.375,y+.25,1.8]);entry=np.full((H,W),-np.inf);leave=np.full((H,W),np.inf)
        for i in range(3):
            with np.errstate(divide='ignore',invalid='ignore'):
                a=(lo[i]-origin[i])/rays[:,:,i];b=(hi[i]-origin[i])/rays[:,:,i]
            entry=np.maximum(entry,np.minimum(a,b));leave=np.minimum(leave,np.maximum(a,b))
        depth=np.minimum(depth,np.where((leave>=entry)&(entry>0),entry,np.inf))
    return depth.astype(np.float32)

rclpy.init()
class Probe(Node):
    def __init__(self):
        super().__init__('dynamic_input_probe');self.slices={};self.records=[];self.phase='warm';self.points=[]
        self.depth=self.create_publisher(Image,'/dynamic_test/depth',qos_profile_sensor_data);self.info=self.create_publisher(CameraInfo,'/dynamic_test/camera_info',qos_profile_sensor_data)
        for mode in ('static','dynamic','combined'):
            self.create_subscription(DistanceMapSlice,f'/nvblox_probe/{mode}_map_slice',lambda msg,m=mode:self.on_slice(m,msg),qos_profile_sensor_data)
        self.create_subscription(PointCloud2,'/nvblox_probe/dynamic_points',lambda msg:self.points.append((time.monotonic(),msg.width*msg.height)),qos_profile_sensor_data)
        self.broadcaster=StaticTransformBroadcaster(self);tf=TransformStamped();tf.header.frame_id='test_world';tf.child_frame_id='dynamic_optical';tf.header.stamp=self.get_clock().now().to_msg();tf.transform.translation.z=.45;tf.transform.rotation.x=-.5;tf.transform.rotation.y=.5;tf.transform.rotation.z=-.5;tf.transform.rotation.w=.5;self.broadcaster.sendTransform(tf)
    def on_slice(self,mode,msg):
        self.slices[mode]=msg;x,y=1.2,0.;ix=round((x-msg.origin.x)/msg.resolution);iy=round((y-msg.origin.y)/msg.resolution);value=None
        if 0<=ix<msg.width and 0<=iy<msg.height:
            z=msg.data[iy*msg.width+ix]
            if np.isfinite(z) and z!=msg.unknown_value:value=float(z)
        self.records.append({'t':time.monotonic()-self.begin,'mode':mode,'phase':self.phase,'center_distance_m':value})
    def spin_for(self,dt):
        end=time.monotonic()+dt
        while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.005)
    def publish(self,depth):
        stamp=self.get_clock().now().to_msg();m=Image();m.header.stamp=stamp;m.header.frame_id='dynamic_optical';m.width=W;m.height=H;m.encoding='32FC1';m.step=W*4;m.data=depth.tobytes();self.depth.publish(m)
        c=CameraInfo();c.header=m.header;c.width=W;c.height=H;c.k=[FX,0.,W/2,0.,FX,H/2,0.,0.,1.];c.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.];c.p=[FX,0.,W/2,0.,0.,FX,H/2,0.,0.,0.,1.,0.];c.distortion_model='plumb_bob';c.d=[0.]*5;self.info.publish(c)
    def save(self,label,depth):
        np.save(OUT/(label+'_depth_m.npy'),depth)
        for mode,msg in self.slices.items():
            a=np.array(msg.data,np.float32).reshape(msg.height,msg.width);np.savez_compressed(OUT/(label+'_'+mode+'.npz'),distance=a,known=np.isfinite(a)&(a!=msg.unknown_value),origin=[msg.origin.x,msg.origin.y,msg.origin.z],resolution=msg.resolution)

n=Probe();n.begin=time.monotonic();log=(OUT/'node.log').open('w');child=None;report={}
try:
    child=subprocess.Popen(['/opt/ros/lyrical/lib/nvblox_ros/nvblox_node','--ros-args','--params-file',str(HERE/'dynamic.yaml'),'-r','__node:=nvblox_probe','-p','global_frame:=test_world','-p','map_clearing_frame_id:=test_world','-p','esdf_slice_bounds_visualization_attachment_frame_id:=test_world','-r','camera_0/depth/image:=/dynamic_test/depth','-r','camera_0/depth/camera_info:=/dynamic_test/camera_info'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    n.spin_for(8.)
    if child.poll() is not None:raise RuntimeError('nvblox 초기화 종료')
    n.begin=time.monotonic();saved=set();frames=0
    while True:
        t=time.monotonic()-n.begin
        if t>=28:break
        n.phase='warm' if t<8 else 'enter' if t<11 else 'hold' if t<15 else 'leave' if t<18 else 'clear'
        depth=depth_at(t);n.publish(depth);frames+=1;n.spin_for(1/15)
        for label,at in [('warm',7.),('hold',14.),('clear',26.)]:
            if t>=at and label not in saved:n.save(label,depth);saved.add(label)
    def values(mode,start,end):return [v for v in n.records if v['mode']==mode and start<=v['t']<end and v['center_distance_m'] is not None]
    warm=values('combined',5,8);hold=values('combined',11,15);clear=values('combined',18,28)
    detected=next((v['t'] for v in hold if v['center_distance_m']<=.15),None);cleared=next((v['t'] for v in clear if v['center_distance_m']>.2),None)
    dyn=values('dynamic',11,15)
    report={'simulation_only':True,'input':'분석식 복도/직육면체 깊이·고정 기준자세·의미분류 없음','frames':frames,'warm_center_distance_m':warm[-1]['center_distance_m'] if warm else None,'detection_t_s':detected,'clear_t_s':cleared,'detection_delay_after_full_entry_s':detected-11 if detected is not None else None,'clear_delay_after_exit_s':cleared-18 if cleared is not None else None,'dynamic_center_observed':len(dyn),'max_dynamic_points':max((v[1] for v in n.points),default=0),'slice_counts':{m:sum(v['mode']==m for v in n.records) for m in ('static','dynamic','combined')}}
    report['pass']=bool(warm and warm[-1]['center_distance_m']>.2 and detected is not None and detected<=12 and cleared is not None and cleared<=21 and dyn)
finally:
    if child:
        try:child.send_signal(signal.SIGINT);child.wait(timeout=10)
        except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=5)
        report['node_exit']=child.returncode
    report['pass']=report.get('pass',False) and report.get('node_exit')==0
    (OUT/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));(OUT/'timeline.json').write_text(json.dumps(n.records,ensure_ascii=False));print(json.dumps(report,ensure_ascii=False),flush=True);log.close();n.destroy_node();rclpy.shutdown()
