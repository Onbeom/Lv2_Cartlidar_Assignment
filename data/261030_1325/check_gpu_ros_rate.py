"""워밍업 후 로컬 ROS 30Hz 입력과 정지자세 전환. 실제 관절 운동/전체 nvblox 아님."""
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image,CameraInfo,JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation
from self_filter_gpu_node import SelfFilter,QOS

ROOT=Path('/home/pa4/git/lv2_next_module');BASE=ROOT/'Cartlidar/sim_render/results/followup_261028_1457';OUT=BASE/'improvement_261030_1325/filter_gpu_ros_rate';OUT.mkdir(exist_ok=False)
frame='top_camera_front_color_optical_frame';inputs=[]
for directory,reference in [('camera_new_model_spec','nvblox_self_filter_cached'),('camera_new_model_stow_holdout','nvblox_self_filter_cached_stow_holdout'),('camera_new_model_zero_holdout','nvblox_self_filter_cached_zero_holdout')]:
    m=json.loads((BASE/directory/'summary.json').read_text());row=next(r for r in m['rows'] if r['visible'] and r['frame']==frame)
    raw=np.load(BASE/directory/row['depth']).astype(np.float32)
    expected=raw.copy();expected[np.load(BASE/reference/(frame+'_mask.npy'))]=0
    inputs.append((m,row,raw,expected))
rclpy.init();worker=SelfFilter(inputs[0][0]['urdf']);probe=Node('self_filter_rate_probe');ex=SingleThreadedExecutor();ex.add_node(worker);ex.add_node(probe)
image_pub=probe.create_publisher(Image,'/self_filter/input',QOS);info_pub=probe.create_publisher(CameraInfo,'/self_filter/camera_info',QOS);joint_pub=probe.create_publisher(JointState,'/self_filter/joint_states',QOS)
received=[];probe.create_subscription(Image,'/self_filter/output',lambda m:received.append((m,time.monotonic())),QOS)
tf=TransformStamped();tf.header.frame_id='base_link';tf.child_frame_id=frame;C=worker.robot.link_poses(inputs[0][0]['arm_q'])[frame]
tf.transform.translation.x=float(C[0,3]);tf.transform.translation.y=float(C[1,3]);tf.transform.translation.z=float(C[2,3]);quat=Rotation.from_matrix(C[:3,:3]).as_quat()
tf.transform.rotation.x=float(quat[0]);tf.transform.rotation.y=float(quat[1]);tf.transform.rotation.z=float(quat[2]);tf.transform.rotation.w=float(quat[3]);broadcaster=StaticTransformBroadcaster(probe);broadcaster.sendTransform(tf)
def spin_until(deadline):
    while time.monotonic()<deadline:ex.spin_once(timeout_sec=min(.002,max(0,deadline-time.monotonic())))
def send(index):
    meta,row,raw,expected=inputs[index];stamp=probe.get_clock().now().to_msg()
    state=JointState();state.header.stamp=stamp;state.name=list(worker.required);state.position=[float(meta['arm_q'].get(n,0)) for n in state.name];joint_pub.publish(state)
    info=CameraInfo();info.header.frame_id=frame;info.header.stamp=stamp;info.width=640;info.height=480;k=row['K'];info.k=[k['fx'],0,k['cx'],0,k['fy'],k['cy'],0,0,1.];info_pub.publish(info)
    image=Image();image.header=info.header;image.width=640;image.height=480;image.encoding='32FC1';image.step=2560;image.data=raw.tobytes();image_pub.publish(image)
    return (stamp.sec,stamp.nanosec),expected,time.monotonic()
try:
    spin_until(time.monotonic()+1.)
    for i in range(6):send(i%3);spin_until(time.monotonic()+.1)
    warm_outputs=len(received);received.clear();worker.timings.clear();sent={};start=time.monotonic()
    for i in range(100):
        key,expected,tx=send((i//10)%3);sent[key]=(expected,tx)
        spin_until(start+(i+1)/30.)
    publish_duration=time.monotonic()-start;spin_until(time.monotonic()+.2)
    correct=0;latencies=[];keys=set()
    for msg,rx in received:
        key=(msg.header.stamp.sec,msg.header.stamp.nanosec)
        if key not in sent:continue
        expected,tx=sent[key];keys.add(key);latencies.append(rx-tx)
        actual=np.frombuffer(msg.data,np.float32).reshape(480,640)
        correct+=bool(np.array_equal(actual,expected,equal_nan=True))
    p95=float(np.percentile(worker.timings,95)) if worker.timings else None
    report={'simulation_only':True,'warmup_outputs':warm_outputs,'sent':len(sent),'unique_received':len(keys),'correct_depth_headers':correct,
        'publish_duration_s':publish_duration,'actual_publish_hz':len(sent)/publish_duration,'warm_compute_p95_s':p95,
        'local_publish_to_output_p95_s':float(np.percentile(latencies,95)) if latencies else None,
        'node_counts':dict(worker.counts),'pass':len(keys)>=98 and correct==len(keys) and p95 is not None and p95<=1/30,
        'limitations':'같은프로세스로컬DDS·3정지자세전환·warm100장. 물리팔운동/ROS별도프로세스/Jetson/nvblox전체통합검증아님.'}
    (OUT/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(report,ensure_ascii=False))
finally:worker.close();ex.shutdown();worker.destroy_node();probe.destroy_node();rclpy.shutdown()
