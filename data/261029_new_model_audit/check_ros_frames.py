import sys,json,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer,TransformListener,TransformException
from model_frames import ModelFrames
ROOT=Path('/home/pa4/git/lv2_next_module');O=ROOT/'Cartlidar/sim_render/results/new_model_261029';profile=json.loads((O/'perception_model_profile.json').read_text());model=ModelFrames(profile['urdf']);params=json.loads((ROOT/'261029_1030/urdf_261028/isaac/robot_params.json').read_text())
rclpy.init();node=Node('new_model_tf_check');pub=node.create_publisher(JointState,'/joint_states',10);buf=Buffer();listener=TransformListener(buf,node);rows=[]
try:
 for pose in ['drive','stow','zero']:
  q={j:0. for j,d in model.joints.items() if d['kind']!='fixed'};q.update(params['poses'][pose]['value']);expected=model.poses(q);deadline=time.monotonic()+15;passed=None
  while time.monotonic()<deadline:
   msg=JointState();msg.header.stamp=node.get_clock().now().to_msg();msg.name=list(q);msg.position=list(q.values());pub.publish(msg)
   for _ in range(10):rclpy.spin_once(node,timeout_sec=.02)
   check=[]
   try:
    for origin in ['base_link','base_footprint']:
     for camera in profile['cameras'].values():
      target=camera['frame'];tf=buf.lookup_transform(origin,target,Time.from_msg(msg.header.stamp));t=tf.transform.translation;r=tf.transform.rotation;quat=np.array([r.x,r.y,r.z,r.w]);x,y,z,w=quat/np.linalg.norm(quat);R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]);E=np.linalg.inv(expected[origin])@expected[target];pe=float(np.linalg.norm(np.array([t.x,t.y,t.z])-E[:3,3]));re=float(np.max(abs(R-E[:3,:3])));check.append(dict(pose=pose,source=origin,target=target,position_error_m=pe,rotation_element_error=re,pass_tf=pe<=1e-8 and re<=1e-7))
   except TransformException:continue
   if len(check)==14 and all(r['pass_tf'] for r in check):passed=check;break
  if passed is None:raise RuntimeError(('TF검증시간초과',pose,check))
  rows+=passed;print(pose,'14/14',flush=True)
 report=dict(rows=rows,ok=sum(r['pass_tf'] for r in rows),n=len(rows),pass_tf=len(rows)==42 and all(r['pass_tf'] for r in rows),domain=96,localhost_only=True,motor_commands_sent=False)
 (O/'ros_tf_summary.json').write_text(json.dumps(report,indent=2)+'\n');print('총',report['ok'],'/',report['n'])
finally:node.destroy_node();rclpy.shutdown()
