"""실제 ROS 통신에서 명령/관측/정지/경로 입력 중단 및 고장 명령을 주입."""
from pathlib import Path
import json,time,os
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
from command_gate import CommandGate

ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'Cartlidar/sim_render/results/followup_261028_1457'/os.environ.get('GATE_RESULT_TAG','nav2_gate_fault_v1');OUT.mkdir(exist_ok=False)
rclpy.init();gate=CommandGate();probe=Node('gate_fault_injector');executor=SingleThreadedExecutor();executor.add_node(gate);executor.add_node(probe)
topics={'cmd':probe.create_publisher(TwistStamped,'/cmd_vel_nav_raw',10),'health':probe.create_publisher(Bool,'/navigation/observations_valid',10),'stop':probe.create_publisher(Bool,'/perception/obstacle_stop',10),'path':probe.create_publisher(Bool,'/navigation/path_valid',10)}
received=[];probe.create_subscription(TwistStamped,'/cmd_vel_nav',lambda m:received.append((time.monotonic(),m.twist.linear.x,m.twist.angular.z)),10)
def spin(dt):
    end=time.monotonic()+dt
    while time.monotonic()<end:executor.spin_once(timeout_sec=.005)
def inputs(omit=None,bad=None):
    for name,pub in topics.items():
        if name==omit:continue
        if name=='cmd':
            m=TwistStamped();m.header.stamp=probe.get_clock().now().to_msg();m.header.frame_id='base_link';m.twist.linear.x=.12;m.twist.angular.z=.2
            if bad=='nan':m.twist.linear.x=float('nan')
            if bad=='old_stamp':m.header.stamp.sec-=2
        else:m=Bool(data=name!='stop')
        pub.publish(m)
def feed(duration,omit=None,bad=None):
    end=time.monotonic()+duration
    while time.monotonic()<end:inputs(omit,bad);spin(.05)
rows=[]
try:
    spin(.5)
    for case in ['cmd','health','stop','path','nan','old_stamp']:
        feed(.5);moving=bool(received and received[-1][1]>.01);began=time.monotonic();start=len(received);feed(.65,omit=case if case in topics else None,bad=case if case not in topics else None);samples=received[start:]
        zeros=[when-began for when,v,w in samples if v==0 and w==0];delay=zeros[0] if zeros else None;late=[v for v in samples if v[0]-began>=.40];stopped=bool(late) and all(v==0 and w==0 for _,v,w in late)
        feed(.5);resumed=bool(received and received[-1][1]>.01)
        rows.append({'case':case,'moving_before':moving,'first_zero_delay_s':delay,'zero_after_400ms':stopped,'resumed':resumed,'pass':moving and stopped and resumed and delay is not None and delay<=.40})
    report={'simulation_only':True,'rows':rows,'counts':dict(gate.counts),'pass':all(v['pass'] for v in rows)};(OUT/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(report,ensure_ascii=False),flush=True)
finally:
    probe.destroy_node();gate.destroy_node();executor.shutdown();rclpy.shutdown()
