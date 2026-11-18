"""개인 ROS 자기깊이 필터 시제품. GPU 형상 갱신/입력 불일치·시각 초과 시 출력을 보류한다."""
from pathlib import Path
import time
import torch
from collections import Counter, deque
import hashlib,json,sys
import numpy as np

from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Image,CameraInfo,JointState
from tf2_ros import Buffer,TransformListener,TransformException
from rclpy.time import Time
from gpu_filter import filter_depth
from gpu_geometry_tf import load_local,prepare

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'Cartlidar/sim_world'))
from robot_geom import Robot
QOS=QoSProfile(depth=5,reliability=ReliabilityPolicy.RELIABLE)
def seconds(stamp):return stamp.sec+stamp.nanosec*1e-9

class SelfFilter(Node):
    def __init__(self,urdf):
        super().__init__('self_depth_filter_gpu_prototype')
        self.robot=Robot(urdf,Path(urdf).parent.parent)
        self.local=load_local(self.robot)
        self.timings=[]
        digest=hashlib.sha256(Path(urdf).read_bytes())
        for link,items in sorted(self.robot.shapes.items()):
            digest.update(link.encode())
            for transform,vertices in items:digest.update(transform.tobytes());digest.update(vertices.tobytes())
        self.sha=digest.hexdigest()
        self.required=tuple(sorted(j['name'] for j in self.robot.joint_by_name.values() if j['type']!='fixed'))
        self.info=None;self.states=deque(maxlen=200);self.cache=None;self.future=None;self.pending_key=None
        self.counts=Counter()
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.pub=self.create_publisher(Image,'/self_filter/output',QOS)
        self.create_subscription(CameraInfo,'/self_filter/camera_info',self.on_info,QOS)
        self.create_subscription(JointState,'/self_filter/joint_states',self.on_joint,QOS)
        self.create_subscription(Image,'/self_filter/input',self.on_image,QOS)
    def on_info(self,msg):self.info=msg
    def on_joint(self,msg):
        if len(msg.name)!=len(msg.position) or len(set(msg.name))!=len(msg.name):return
        q=dict(zip(msg.name,msg.position))
        if all(n in q and np.isfinite(q[n]) for n in self.required):self.states.append((seconds(msg.header.stamp),q))
    def on_image(self,msg):
        self.counts['input']+=1
        age=self.get_clock().now().nanoseconds*1e-9-seconds(msg.header.stamp)
        if not -.05<=age<=.30:self.counts['stale_image']+=1;return
        info=self.info
        if info is None or not self.states:self.counts['missing_input']+=1;return
        if (msg.width,msg.height)!=(640,480) or (info.width,info.height)!=(msg.width,msg.height) or info.header.frame_id!=msg.header.frame_id:
            self.counts['camera_mismatch']+=1;return
        if any(not np.isfinite(v) or abs(v)>1e-12 for v in info.d):self.counts['unsupported_distortion']+=1;return
        if abs(seconds(info.header.stamp)-seconds(msg.header.stamp))>.05:self.counts['stale_info']+=1;return
        stamp,q=min(self.states,key=lambda v:abs(v[0]-seconds(msg.header.stamp)))
        if abs(stamp-seconds(msg.header.stamp))>.05:self.counts['stale_joint']+=1;return
        try:tf=self.tf.lookup_transform('base_link',msg.header.frame_id,Time.from_msg(msg.header.stamp))
        except TransformException:self.counts['missing_tf']+=1;return
        C=np.eye(4);rot=tf.transform.rotation;tr=tf.transform.translation
        C[:3,:3]=Rotation.from_quat([rot.x,rot.y,rot.z,rot.w]).as_matrix();C[:3,3]=[tr.x,tr.y,tr.z]
        nominal=self.robot.link_poses(q).get(msg.header.frame_id)
        if nominal is None or np.max(np.abs(C-nominal))>1e-4:self.counts['tf_model_mismatch']+=1;return
        k={'fx':float(info.k[0]),'fy':float(info.k[4]),'cx':float(info.k[2]),'cy':float(info.k[5])}
        if not np.isfinite(list(k.values())).all() or min(k['fx'],k['fy'])<=0:self.counts['invalid_k']+=1;return
        if msg.encoding not in ('16UC1','32FC1'):self.counts['encoding']+=1;return
        dt=np.dtype(('>' if msg.is_bigendian else '<')+('u2' if msg.encoding=='16UC1' else 'f4'))
        if msg.step<msg.width*dt.itemsize or len(msg.data)<msg.height*msg.step:self.counts['invalid_buffer']+=1;return
        raw=np.ndarray((msg.height,msg.width),dtype=dt,buffer=msg.data,strides=(msg.step,dt.itemsize))
        depth=raw.astype(float)*(.001 if msg.encoding=='16UC1' else 1.)
        torch.cuda.synchronize();started=time.perf_counter()
        try:
            geometry=prepare(self.robot,self.local,q,msg.header.frame_id,k,camera_pose=C)
            mask=filter_depth(depth,*geometry)
            torch.cuda.synchronize()
        except Exception as exc:
            self.get_logger().error(str(exc));self.counts['gpu_error']+=1;return
        self.timings.append(time.perf_counter()-started)
        age=self.get_clock().now().nanoseconds*1e-9-seconds(msg.header.stamp)
        if not -.05<=age<=.30:self.counts['stale_after_compute']+=1;return
        filtered=raw.copy();filtered[mask]=0
        out=Image();out.header=msg.header;out.height=msg.height;out.width=msg.width;out.encoding=msg.encoding;out.is_bigendian=msg.is_bigendian;out.step=msg.width*dt.itemsize;out.data=filtered.tobytes();self.pub.publish(out);self.counts['output']+=1
    def close(self):pass

if __name__=='__main__':
    rclpy.init();node=SelfFilter(sys.argv[1])
    try:rclpy.spin(node)
    finally:node.close();node.destroy_node();rclpy.shutdown()
