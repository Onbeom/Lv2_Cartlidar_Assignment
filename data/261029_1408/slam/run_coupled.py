"""고정 bag으로 cuVSLAM odom→Cartographer 연결 확인. 모터·시뮬레이터를 구동하지 않는다."""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer,TransformListener
p=argparse.ArgumentParser();p.add_argument('--record',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--rate',type=float,default=1.);a=p.parse_args()
a.out=a.out.resolve();a.record=a.record.resolve();a.out.mkdir(parents=True,exist_ok=False)
here=Path(__file__).resolve().parent
children=[];streams=[]
def start(name,cmd):
    f=(a.out/(name+'.log')).open('w');streams.append(f)
    child=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);children.append((name,child));return child
rclpy.init()
class Probe(Node):
    def __init__(self):
        super().__init__('cuvslam_carto_probe');self.odom=0;self.edges={};self.chain_ok=0;self.stamps=[]
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.create_subscription(Odometry,'/visual_slam/tracking/odometry',self.on_odom,100)
        self.create_subscription(TFMessage,'/tf',self.on_tf,100)
    def on_odom(self,msg):
        self.odom+=1;self.stamps.append(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
    def on_tf(self,msg):
        for t in msg.transforms:
            key=t.header.frame_id+'->'+t.child_frame_id;self.edges[key]=self.edges.get(key,0)+1
    def tick(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.05)
            try:self.buffer.lookup_transform('map','base_footprint',Time());self.chain_ok+=1
            except Exception:pass
n=Probe();report={}
try:
    start('cuvslam',['ros2','launch',str(here/'cuvslam_odom.launch.py')])
    carto=start('cartographer',[str(Path('/home/pa4/git/ros_test/waffle-lyrical-slam/install/cartographer_ros/lib/cartographer_ros/cartographer_node')),'-configuration_directory',str(here),'-configuration_basename','carto_c6_2lidar.lua','--ros-args','-p','use_sim_time:=true','-r','scan_1:=/scan_front','-r','scan_2:=/scan_rear','-r','odom:=/visual_slam/tracking/odometry'])
    start('mask',['python3',str(here/'mask_relay.py')])
    start('record',['ros2','bag','record','--use-sim-time','-s','mcap','-o',str(a.out/'out_bag'),'--topics','/tf','/visual_slam/tracking/odometry','/visual_slam/status','/clock'])
    n.tick(10.)
    play=start('play',['ros2','bag','play',str(a.record/'bag'),'--rate',str(a.rate),'--disable-keyboard-controls','--remap','/tf:=/tf_wheel','/top_camera_front/infra1/image_rect_raw:=/unmasked/top_camera_front/infra1/image_rect_raw','/top_camera_front/infra2/image_rect_raw:=/unmasked/top_camera_front/infra2/image_rect_raw'])
    deadline=time.monotonic()+720
    while play.poll() is None:
        n.tick(.1)
        if time.monotonic()>deadline:raise RuntimeError('bag 재생 시간 상한 초과')
    n.tick(3.)
    publishers=[{'node':x.node_name,'namespace':x.node_namespace} for x in n.get_publishers_info_by_topic('/tf')]
    report={'record':str(a.record),'rate':a.rate,'variant':'V2 mask220 + ground constraint; odometry-only','odom_messages':n.odom,'tf_edges':n.edges,'map_base_chain_checks':n.chain_ok,'tf_publishers':publishers,'play_exit':play.returncode,'stamp_first':min(n.stamps) if n.stamps else None,'stamp_last':max(n.stamps) if n.stamps else None,'simulation_only':True,'accuracy_evaluated':False}
    report['tool_pass']=n.odom>0 and n.edges.get('map->odom',0)>0 and n.edges.get('odom->base_footprint',0)>0 and n.chain_ok>0 and play.returncode==0
    for name,service,kind,body in [('finish','/finish_trajectory','cartographer_ros_msgs/srv/FinishTrajectory','{trajectory_id: 0}'),('write','/write_state','cartographer_ros_msgs/srv/WriteState',json.dumps({'filename':str(a.out/'map.pbstream'),'include_unfinished_submaps':True}))]:
        with (a.out/(name+'.log')).open('w') as f:
            call=subprocess.run(['ros2','service','call',service,kind,body],stdout=f,stderr=subprocess.STDOUT,timeout=20);report[name+'_exit']=call.returncode
finally:
    for name,c in reversed(children):
        try:os.killpg(c.pid,signal.SIGINT)
        except ProcessLookupError:continue
        try:c.wait(timeout=8)
        except subprocess.TimeoutExpired:
            try:os.killpg(c.pid,signal.SIGTERM);c.wait(timeout=5)
            except (ProcessLookupError,subprocess.TimeoutExpired):
                try:os.killpg(c.pid,signal.SIGKILL);c.wait(timeout=5)
                except (ProcessLookupError,subprocess.TimeoutExpired):pass
    report['processes']=[{'name':name,'pid':c.pid,'exit':c.poll()} for name,c in children]
    report['all_direct_children_exited']=all(c.poll() is not None for _,c in children)
    remaining=[]
    for name,c in children:
        try:os.killpg(c.pid,0);remaining.append({'name':name,'pgid':c.pid})
        except ProcessLookupError:pass
    report['remaining_process_groups']=remaining
    (a.out/'tool_summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(report,ensure_ascii=False),flush=True)
    n.destroy_node();rclpy.shutdown()
    for f in streams:f.close()
