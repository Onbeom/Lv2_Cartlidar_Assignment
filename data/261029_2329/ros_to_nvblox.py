"""실제 로컬 ROS 전송으로 자기필터 캐시 변경·시각 게이트·깊이 보존 확인."""
from pathlib import Path
import json,time,sys,subprocess,signal,os
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image,CameraInfo,JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation
from nvblox_msgs.msg import DistanceMapSlice
from self_filter_node import SelfFilter,QOS

root=Path(__file__).resolve().parents[1];base=root/'Cartlidar/sim_render/results/followup_261028_1457';out=base/'nvblox_self_filter_ros_to_nvblox';out.mkdir(exist_ok=False)
meta=json.loads((base/'camera_new_model_spec/summary.json').read_text());rclpy.init();worker=SelfFilter(meta['urdf']);probe=Node('self_filter_test_input');executor=SingleThreadedExecutor();executor.add_node(worker);executor.add_node(probe)
pub=probe.create_publisher(Image,'/self_filter/input',QOS);info_pub=probe.create_publisher(CameraInfo,'/self_filter/camera_info',QOS);qpub=probe.create_publisher(JointState,'/self_filter/joint_states',QOS);received=[];probe.create_subscription(Image,'/self_filter/output',received.append,QOS);broadcaster=StaticTransformBroadcaster(probe)
frame='top_camera_front_color_optical_frame';C=worker.robot.link_poses(meta['arm_q'])[frame];tf=TransformStamped();tf.header.frame_id='base_link';tf.child_frame_id=frame;tf.header.stamp=probe.get_clock().now().to_msg();tf.transform.translation.x=float(C[0,3]);tf.transform.translation.y=float(C[1,3]);tf.transform.translation.z=float(C[2,3]);qrot=Rotation.from_matrix(C[:3,:3]).as_quat();tf.transform.rotation.x=float(qrot[0]);tf.transform.rotation.y=float(qrot[1]);tf.transform.rotation.z=float(qrot[2]);tf.transform.rotation.w=float(qrot[3]);broadcaster.sendTransform(tf)
def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:executor.spin_once(timeout_sec=.005)
slices=[];probe.create_subscription(DistanceMapSlice,'/nvblox_probe/static_map_slice',slices.append,QOS)
Tcam=np.array(json.loads((base/'camera_new_model_spec/nvblox_input.json').read_text())['T_world_cam']);W=Tcam@np.linalg.inv(C);wt=TransformStamped();wt.header.frame_id='test_world';wt.child_frame_id='base_link';wt.header.stamp=probe.get_clock().now().to_msg();wt.transform.translation.x=float(W[0,3]);wt.transform.translation.y=float(W[1,3]);wt.transform.translation.z=float(W[2,3]);rot=Rotation.from_matrix(W[:3,:3]).as_quat();wt.transform.rotation.x=float(rot[0]);wt.transform.rotation.y=float(rot[1]);wt.transform.rotation.z=float(rot[2]);wt.transform.rotation.w=float(rot[3]);broadcaster.sendTransform([tf,wt])
log=(out/'nvblox.log').open('w')
child=subprocess.Popen(['/opt/ros/lyrical/lib/nvblox_ros/nvblox_node','--ros-args','--params-file',str(Path(__file__).parent/'nvblox/tool.yaml'),'-r','__node:=nvblox_probe','-p','global_frame:=test_world','-p','map_clearing_frame_id:=test_world','-p','esdf_slice_bounds_visualization_attachment_frame_id:=test_world','-r','camera_0/depth/image:=/self_filter/output','-r','camera_0/depth/camera_info:=/self_filter/camera_info'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
spin(8.)
if child.poll() is not None:raise RuntimeError("nvblox 초기화 실패")
phases=[]
try:
    for pose,directory,encoding in [('drive','camera_new_model_spec','16UC1')]:
        m=json.loads((base/directory/'summary.json').read_text());row=next(v for v in m['rows'] if v['frame']==frame and v['visible']);depth=np.load(base/directory/row['depth']);raw=np.rint(depth*1000).astype(np.uint16) if encoding=='16UC1' else depth.astype(np.float32);mask=np.load(base/('nvblox_self_filter_cached' if pose=='drive' else 'nvblox_self_filter_cached_zero_holdout')/(frame+'_mask.npy'));expected=raw.copy();expected[mask]=0;start=len(received);sent={};began=time.monotonic()
        for _ in range(120):
            stamp=probe.get_clock().now().to_msg();key=(stamp.sec,stamp.nanosec);sent[key]=True
            state=JointState();state.header.stamp=stamp;state.name=list(worker.required);state.position=[float(m['arm_q'].get(n,0.)) for n in state.name];qpub.publish(state)
            inf=CameraInfo();inf.header.frame_id=frame;inf.header.stamp=stamp;inf.height=480;inf.width=640;k=row['K'];inf.k=[k['fx'],0.,k['cx'],0.,k['fy'],k['cy'],0.,0.,1.];info_pub.publish(inf)
            image=Image();image.header=inf.header;image.height=480;image.width=640;image.encoding=encoding;image.step=640*raw.dtype.itemsize;image.data=raw.tobytes();pub.publish(image);spin(.05)
            if len(received)-start>=8:break
        spin(.1);results=received[start:];identical=all(np.array_equal(np.frombuffer(v.data,dtype=raw.dtype).reshape(480,640),expected,equal_nan=True) for v in results);headers=all(v.header.frame_id==frame and (v.header.stamp.sec,v.header.stamp.nanosec) in sent and v.encoding==encoding for v in results)
        phases.append({'pose':pose,'encoding':encoding,'received':len(results),'all_depth_identical':identical,'headers_preserved':headers,'elapsed_s':time.monotonic()-began,'pass':len(results)>=8 and identical and headers})
    before=len(received)
    # CameraInfo만 최신일 때 오래된 관절 상태로 출력하지 않는지 확인한다.
    spin(.2)
    for _ in range(8):
        stamp=probe.get_clock().now().to_msg();inf.header.stamp=stamp;info_pub.publish(inf);image.header.stamp=stamp;pub.publish(image);spin(.05)
    stale_outputs=len(received)-before
    before_distortion=len(received)
    for _ in range(4):
        stamp=probe.get_clock().now().to_msg();state.header.stamp=stamp;qpub.publish(state);inf.header.stamp=stamp;inf.d=[.1,0.,0.,0.,0.];info_pub.publish(inf);image.header.stamp=stamp;pub.publish(image);spin(.05)
    distorted_outputs=len(received)-before_distortion
    report={'simulation_only':True,'phases':phases,'stale_joint_outputs':stale_outputs,'unsupported_distortion_outputs':distorted_outputs,'node_counts':dict(worker.counts),'pass':all(p['pass'] for p in phases) and stale_outputs==0 and distorted_outputs==0,'limitations':'정지 drive 한 자세의 ROS→필터→nvblox 연결. 기준TF·실물/VO/전체30Hz 시험 아님.'}
    report['limitations']='정지 drive 한 자세의 ROS→필터→nvblox 연결. 기준TF·실물/VO/전체30Hz 시험 아님.'
    spin(3.)
    known=0
    if slices:
        v=slices[-1];data=np.array(v.data).reshape(v.height,v.width);valid=np.isfinite(data)&(data!=v.unknown_value);known=int(valid.sum());np.savez_compressed(out/'distance_slice.npz',distance=data,known=valid,origin=[v.origin.x,v.origin.y,v.origin.z],resolution=v.resolution)
    report['nvblox_slices']=len(slices);report['nvblox_known_cells']=known;report['pass']=report['pass'] and known>0
    (out/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(report,ensure_ascii=False),flush=True)
finally:
    os.killpg(child.pid,signal.SIGINT)
    try:child.wait(timeout=8)
    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
    report['nvblox_node_exit']=child.returncode;report['pass']=report['pass'] and child.returncode==0
    (out/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));log.close()
    worker.close();executor.shutdown();worker.destroy_node();probe.destroy_node();rclpy.shutdown()
