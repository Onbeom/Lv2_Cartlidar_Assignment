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

ROOT=Path(__file__).resolve().parents[1];HERE=Path(__file__).resolve().parent;BASE=ROOT/'Cartlidar/sim_render/results/followup_261028_1457'
OUT=BASE/'nav2_261030_design_v3';OUT.mkdir(exist_ok=False)
SOURCE=BASE/'dynamic_261030_design_v2'
rclpy.init();gate=CommandGate();executor=SingleThreadedExecutor();executor.add_node(gate)
class Probe(Node):
    def __init__(self):
        super().__init__('nav2_contract_probe');self.begin=None;self.x=.35;self.y=0.;self.yaw=0.;self.last=time.monotonic();self.safe=None;self.raw=None;self.events=[];self.reason='startup';self.stage='warm';self.map_published=False
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.maps=self.create_publisher(OccupancyGrid,'/map',qos);self.slices=self.create_publisher(DistanceMapSlice,'/nvblox_probe/combined_map_slice',10);self.odom=self.create_publisher(Odometry,'/odom',10);self.health=self.create_publisher(Bool,'/navigation/observations_valid',10);self.stop=self.create_publisher(Bool,'/perception/obstacle_stop',10)
        self.create_subscription(TwistStamped,'/cmd_vel_nav',self.on_safe,10);self.create_subscription(TwistStamped,'/cmd_vel_nav_raw',self.on_raw,10);self.create_subscription(String,'/navigation/command_gate_state',lambda m:setattr(self,'reason',m.data),10)
        self.tf=TransformBroadcaster(self);self.static=StaticTransformBroadcaster(self);trans=[]
        for child in ['odom','test_world']:
            t=TransformStamped();t.header.frame_id='map';t.child_frame_id=child;t.header.stamp=self.get_clock().now().to_msg();t.transform.rotation.w=1.;trans.append(t)
        self.static.sendTransform(trans);self.saved={k:np.load(SOURCE/(k+'_combined.npz')) for k in ['warm','hold','clear']}
        self.create_timer(.02,self.motion);self.create_timer(.1,self.inputs)
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
        self.stage='warm' if elapsed<2 else 'hold' if elapsed<10 else 'clear'
        stamp=self.get_clock().now().to_msg();grid=OccupancyGrid();grid.header.frame_id='map';grid.header.stamp=stamp;grid.info.resolution=.05;grid.info.width=100;grid.info.height=40;grid.info.origin.position.x=-1.;grid.info.origin.position.y=-1.;grid.info.origin.orientation.w=1.
        yy,xx=np.indices((40,100));wx=-1+(xx+.5)*.05;wy=-1+(yy+.5)*.05;grid.data=np.where((wx>0)&(wx<3)&(abs(wy)<.5),0,100).astype(np.int8).ravel().tolist();
        if not self.map_published:self.maps.publish(grid);self.map_published=True
        z=self.saved[self.stage];a=z['distance'];m=DistanceMapSlice();m.header.frame_id='test_world';m.header.stamp=stamp;m.resolution=float(z['resolution']);m.height=a.shape[0];m.width=a.shape[1];m.origin.x=float(z['origin'][0]);m.origin.y=float(z['origin'][1]);m.origin.z=float(z['origin'][2]);m.unknown_value=1000.;m.data=np.where(z['known'],a,1000.).astype(np.float32).ravel().tolist();self.slices.publish(m)
        # 저장 입력 유효성으로 연결을 시험한다. 실물 관측 유효성 판정은 별도 필요.
        self.health.publish(Bool(data=not 16<=elapsed<17));self.stop.publish(Bool(data=18<=elapsed<19))
    def pose(self,x,y):
        p=PoseStamped();p.header.frame_id='map';p.header.stamp=self.get_clock().now().to_msg();p.pose.position.x=float(x);p.pose.position.y=float(y);p.pose.orientation.w=1.;return p

n=Probe();executor.add_node(n);children=[];logs=[];report={}
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
    spin(1.)
    for name in ['planner_server','controller_server']:transition(name,1);transition(name,3)
    spin(1.)
    planner=ActionClient(n,ComputePathToPose,'/compute_path_to_pose');controller=ActionClient(n,FollowPath,'/follow_path');end=time.monotonic()+10
    while (not planner.server_is_ready() or not controller.server_is_ready()) and time.monotonic()<end:spin(.05)
    request=ComputePathToPose.Goal();request.goal=n.pose(2.5,0);request.start=n.pose(n.x,n.y);request.use_start=True;request.planner_id='GridBased'
    handle=wait_future(planner.send_goal_async(request));res=wait_future(handle.get_result_async());path=res.result.path;report['planned_poses']=len(path.poses);report['planner_status']=res.status
    if not path.poses:raise RuntimeError('경로 없음')
    goal=FollowPath.Goal();goal.path=path;goal.controller_id='FollowPath';goal.goal_checker_id='goal_checker';goal.progress_checker_id='progress_checker';goal.path_handler_id='PathHandler';n.begin=time.monotonic();follow=wait_future(controller.send_goal_async(goal));future=follow.get_result_async()
    while time.monotonic()-n.begin<32:
        spin(.05)
        if future.done() and time.monotonic()-n.begin>20:break
    report['follow_status']=future.result().status if future.done() else None;report['goal_error_m']=math.hypot(n.x-2.5,n.y);report['gate_counts']=dict(gate.counts)
    def phase_values(start,end):return [v for v in n.events if start<=v['t']<end]
    blocked=phase_values(3,9.5);health=phase_values(16.4,17);urgent=phase_values(18.4,19)
    report['blocked_max_safe_v']=max((v['safe_v'] for v in blocked),default=None);report['blocked_max_raw_v']=max((v['raw_v'] or 0. for v in blocked),default=None);report['health_invalid_max_safe_v']=max((v['safe_v'] for v in health),default=None);report['urgent_stop_max_safe_v']=max((v['safe_v'] for v in urgent),default=None);report['resume_motion']=any(v['safe_v']>.01 for v in phase_values(10.5,16));report['initial_motion']=any(v['safe_v']>.01 for v in phase_values(.2,2))
    report['pass']=report['initial_motion'] and report['resume_motion'] and report['goal_error_m']<=.10 and all(report[k]==0. for k in ['blocked_max_safe_v','health_invalid_max_safe_v','urgent_stop_max_safe_v']) and report['follow_status']==4
    if not future.done():wait_future(follow.cancel_goal_async())
except Exception as exc:report['error']=str(exc);report['pass']=False
finally:
    n.health.publish(Bool(data=False));n.stop.publish(Bool(data=True));spin(.2)
    for child in children:
        try:child.send_signal(signal.SIGINT);child.wait(timeout=8)
        except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=5)
    report['process_exit_codes']=[c.returncode for c in children];report['simulation_only']=True;report['limitations']='실제Nav2+저장된nvblox거리슬라이스+2D운동학모의. 실제센서/물리/모터/VO/연속동적지도시험아님.'
    (OUT/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));(OUT/'trajectory.json').write_text(json.dumps(n.events));print(json.dumps(report,ensure_ascii=False),flush=True)
    for log in logs:log.close()
    executor.shutdown();n.destroy_node();gate.destroy_node();rclpy.shutdown()
