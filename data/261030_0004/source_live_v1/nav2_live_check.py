"""실제 Nav2 플러그인과 nvblox 저장 슬라이스로 정지/재개 명령 계약을 시험."""
from pathlib import Path
import json,math,signal,subprocess,time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from geometry_msgs.msg import PoseStamped,TransformStamped,TwistStamped
from nav_msgs.msg import OccupancyGrid,Odometry
from std_msgs.msg import Bool,String
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import ComputePathToPose,FollowPath
from nvblox_msgs.msg import DistanceMapSlice
from command_gate import CommandGate
from navigation_coordinator import NavigationCoordinator
from observation_monitor import ObservationMonitor
from sensor_msgs.msg import Image,CameraInfo
from rclpy.qos import qos_profile_sensor_data
from analytic_depth import render,W,H,FX

ROOT=Path(__file__).resolve().parents[1];HERE=Path(__file__).resolve().parent;BASE=ROOT/'Cartlidar/sim_render/results/followup_261028_1457'
OUT=BASE/'nav2_261030_live';OUT.mkdir(exist_ok=False)
SOURCE=BASE/'dynamic_261030_design_v2'
rclpy.init();gate=CommandGate(require_path_valid=True);executor=SingleThreadedExecutor();executor.add_node(gate)
class Probe(Node):
    def __init__(self):
        super().__init__('nav2_contract_probe');self.begin=None;self.x=.35;self.y=0.;self.yaw=0.;self.last=time.monotonic();self.safe=None;self.raw=None;self.events=[];self.reason='startup';self.stage='warm';self.map_published=False
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.maps=self.create_publisher(OccupancyGrid,'/map',qos);self.slices=self.create_publisher(DistanceMapSlice,'/nvblox_probe/combined_map_slice',10);self.odom=self.create_publisher(Odometry,'/odom',10);self.stop=self.create_publisher(Bool,'/perception/obstacle_stop',10)
        self.create_subscription(TwistStamped,'/cmd_vel_nav',self.on_safe,10);self.create_subscription(TwistStamped,'/cmd_vel_nav_raw',self.on_raw,10);self.create_subscription(String,'/navigation/command_gate_state',lambda m:setattr(self,'reason',m.data),10)
        self.tf=TransformBroadcaster(self);self.static=StaticTransformBroadcaster(self);trans=[]
        for child in ['odom','test_world']:
            t=TransformStamped();t.header.frame_id='map';t.child_frame_id=child;t.header.stamp=self.get_clock().now().to_msg();t.transform.rotation.w=1.;trans.append(t)
        camera=TransformStamped();camera.header.frame_id='base_link';camera.child_frame_id='dynamic_optical';camera.header.stamp=self.get_clock().now().to_msg();camera.transform.translation.z=.45;camera.transform.rotation.x=-.5;camera.transform.rotation.y=.5;camera.transform.rotation.z=-.5;camera.transform.rotation.w=.5;trans.append(camera);self.static.sendTransform(trans);self.depth_frames=0;self.depth_pub=self.create_publisher(Image,'/dynamic_test/depth',qos_profile_sensor_data);self.info_pub=self.create_publisher(CameraInfo,'/dynamic_test/camera_info',qos_profile_sensor_data)
        self.create_timer(.02,self.motion);self.create_timer(.1,self.inputs);self.create_timer(1/15,self.sensor)
    def on_safe(self,m):self.safe=m
    def on_raw(self,m):self.raw=m
    def motion(self):
        now=time.monotonic();dt=min(.05,now-self.last);self.last=now
        speed=self.safe.twist.linear.x if self.safe is not None else 0.;turn=self.safe.twist.angular.z if self.safe is not None else 0.
        self.x+=speed*math.cos(self.yaw)*dt;self.y+=speed*math.sin(self.yaw)*dt;self.yaw+=turn*dt
        stamp=self.get_clock().now().to_msg();tf=TransformStamped();tf.header.frame_id='odom';tf.child_frame_id='base_link';tf.header.stamp=stamp;tf.transform.translation.x=self.x;tf.transform.translation.y=self.y;tf.transform.rotation.z=math.sin(self.yaw/2);tf.transform.rotation.w=math.cos(self.yaw/2);self.tf.sendTransform(tf)
        od=Odometry();od.header.frame_id='odom';od.child_frame_id='base_link';od.header.stamp=stamp;od.pose.pose.position.x=self.x;od.pose.pose.position.y=self.y;od.pose.pose.orientation=tf.transform.rotation;od.twist.twist.linear.x=speed;od.twist.twist.angular.z=turn;self.odom.publish(od)
        if self.begin is not None:
            self.events.append({'t':now-self.begin,'x':self.x,'y':self.y,'yaw':self.yaw,'stage':self.stage,'safe_v':speed,'safe_w':turn,'raw_v':self.raw.twist.linear.x if self.raw else None,'reason':self.reason})
    def inputs(self):
        elapsed=time.monotonic()-self.begin if self.begin else -1.
        self.stage='warm' if elapsed<2 else 'hold' if elapsed<26 else 'clear'
        stamp=self.get_clock().now().to_msg();grid=OccupancyGrid();grid.header.frame_id='map';grid.header.stamp=stamp;grid.info.resolution=.05;grid.info.width=100;grid.info.height=40;grid.info.origin.position.x=-1.;grid.info.origin.position.y=-1.;grid.info.origin.orientation.w=1.
        yy,xx=np.indices((40,100));wx=-1+(xx+.5)*.05;wy=-1+(yy+.5)*.05;grid.data=np.where((wx>0)&(wx<3)&(abs(wy)<.5),0,100).astype(np.int8).ravel().tolist();
        if not self.map_published:self.maps.publish(grid);self.map_published=True
        self.stop.publish(Bool(data=34<=elapsed<35))
    def sensor(self):
        elapsed=time.monotonic()-self.begin if self.begin else -1.
        if 32<=elapsed<33:return
        depth=render(self.x,self.y,self.yaw,elapsed);stamp=self.get_clock().now().to_msg();m=Image();m.header.stamp=stamp;m.header.frame_id='dynamic_optical';m.width=W;m.height=H;m.encoding='32FC1';m.step=W*4;m.data=depth.tobytes();self.depth_pub.publish(m);self.depth_frames+=1
        c=CameraInfo();c.header=m.header;c.width=W;c.height=H;c.k=[FX,0.,W/2,0.,FX,H/2,0.,0.,1.];c.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.];c.p=[FX,0.,W/2,0.,0.,FX,H/2,0.,0.,0.,1.,0.];c.distortion_model='plumb_bob';c.d=[0.]*5;self.info_pub.publish(c)
    def pose(self,x,y):
        p=PoseStamped();p.header.frame_id='map';p.header.stamp=self.get_clock().now().to_msg();p.pose.position.x=float(x);p.pose.position.y=float(y);p.pose.orientation.w=1.;return p

n=Probe();executor.add_node(n);coordinator=NavigationCoordinator();executor.add_node(coordinator);monitor=ObservationMonitor();executor.add_node(monitor);children=[];logs=[];report={}
def spin(dt):
    end=time.monotonic()+dt
    while time.monotonic()<end:executor.spin_once(timeout_sec=.005)
def wait_future(f,seconds=15):
    end=time.monotonic()+seconds
    while not f.done() and time.monotonic()<end:spin(.01)
    if not f.done():raise RuntimeError('ROS 응답 대기 초과')
    return f.result()
def transition(name,id):
    client=n.create_client(ChangeState,f'/{name}/change_state');end=time.monotonic()+15
    while not client.service_is_ready() and time.monotonic()<end:spin(.05)
    if not client.service_is_ready():raise RuntimeError('lifecycle 서비스 없음: '+name)
    req=ChangeState.Request();req.transition.id=id
    if not wait_future(client.call_async(req)).success:raise RuntimeError('lifecycle 전환 실패: '+name)
try:
    for package,executable,name in [('nav2_planner','planner_server','planner_server'),('nav2_controller','controller_server','controller_server')]:
        log=(OUT/(name+'.log')).open('w');logs.append(log);child=subprocess.Popen([f'/opt/ros/lyrical/lib/{package}/{executable}','--ros-args','--params-file',str(HERE/'nav2.yaml'),'-r','cmd_vel:=/cmd_vel_nav_raw'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True);children.append(child)
    log=(OUT/'nvblox.log').open('w');logs.append(log);children.append(subprocess.Popen(['/opt/ros/lyrical/lib/nvblox_ros/nvblox_node','--ros-args','--params-file',str(HERE/'dynamic_v2.yaml'),'-r','__node:=nvblox_probe','-p','global_frame:=map','-p','map_clearing_frame_id:=base_link','-p','esdf_slice_bounds_visualization_attachment_frame_id:=map','-r','camera_0/depth/image:=/dynamic_test/depth','-r','camera_0/depth/camera_info:=/dynamic_test/camera_info'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
    spin(8.)
    for name in ['planner_server','controller_server']:transition(name,1);transition(name,3)
    spin(1.)
    goal_pub=n.create_publisher(PoseStamped,'/navigation/goal_pose',10);spin(.5);n.begin=time.monotonic();goal_pub.publish(n.pose(2.5,0))
    while time.monotonic()-n.begin<65:
        spin(.05)
        if coordinator.state=='SUCCEEDED' and time.monotonic()-n.begin>38:break
    report['state']=coordinator.state;report['coordinator_counts']=dict(coordinator.counts);report['plan_pose_counts']=coordinator.plan_sizes;report['goal_error_m']=math.hypot(n.x-2.5,n.y);report['gate_counts']=dict(gate.counts);report['depth_frames']=n.depth_frames;report['observation_counts']=dict(monitor.counts)
    def values(start,end):return [v for v in n.events if start<=v['t']<end]
    blocked=values(3.5,25.5);health=values(32.4,33);urgent=values(34.4,35)
    report['blocked_max_safe_v']=max((v['safe_v'] for v in blocked),default=None);report['health_invalid_max_safe_v']=max((v['safe_v'] for v in health),default=None);report['urgent_stop_max_safe_v']=max((v['safe_v'] for v in urgent),default=None);report['initial_motion']=any(v['safe_v']>.01 for v in values(.2,2));report['resume_motion']=any(v['safe_v']>.01 for v in values(27.5,32))
    report['pass']=coordinator.state=='SUCCEEDED' and coordinator.counts['goals']==1 and coordinator.counts['no_path']>0 and report['initial_motion'] and report['resume_motion'] and report['goal_error_m']<=.10 and all(report[k]==0. for k in ['blocked_max_safe_v','health_invalid_max_safe_v','urgent_stop_max_safe_v'])
except Exception as exc:report['error']=str(exc);report['pass']=False
finally:
    n.stop.publish(Bool(data=True));spin(.2)
    for child in children:
        try:child.send_signal(signal.SIGINT);child.wait(timeout=8)
        except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=5)
    report['process_exit_codes']=[c.returncode for c in children];report['simulation_only']=True;report['limitations']='분석식 이동 카메라 깊이→실제nvblox→Nav2→명령게이트의 동시실행 및 2D운동학. 기준자세TF·잡음없음·자기매쉬미포함, Isaac/실물센서/제동/VO/Jetson 시험 아님.'
    (OUT/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));(OUT/'trajectory.json').write_text(json.dumps(n.events));print(json.dumps(report,ensure_ascii=False),flush=True)
    for log in logs:log.close()
    executor.shutdown();n.destroy_node();gate.destroy_node();coordinator.destroy_node();monitor.destroy_node();rclpy.shutdown()
