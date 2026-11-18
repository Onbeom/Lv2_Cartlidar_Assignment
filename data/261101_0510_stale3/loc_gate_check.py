"""측량 노드 위치 상태 받기 도구 확인(PC, 모의): 실제 puck_survey 실행 파일 + 가짜 TF · PuckArray · /localization/status.
순서: 상태 없음 → LOST → OK → (상태 끊고 1.5 s) → 관측마다 받았는지(rejected · n_obs)"""
import json, subprocess, sys, time, os, signal
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster
from robot_interfaces.msg import PuckArray, Puck
out = sys.argv[1]
os.makedirs(out, exist_ok=True)
proc = subprocess.Popen(['ros2', 'run', 'perception_survey', 'puck_survey', '--ros-args', '-p', 'world_frame:=map',
                         '-p', f'output_path:={out}/survey.json'], stdout=open(f'{out}/node.log', 'w'), stderr=subprocess.STDOUT, start_new_session=True)
rclpy.init()
n = Node('loc_gate_probe')
lat = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
snap = {}
n.create_subscription(String, '/perception/puck_survey', lambda m: snap.update(json.loads(m.data)), lat)
st_pub = None
pk = n.create_publisher(PuckArray, '/perception/pucks', 10)
tfb = StaticTransformBroadcaster(n)
t = TransformStamped(); t.header.frame_id, t.child_frame_id = 'map', 'base_link'; t.transform.rotation.w = 1.0
tfb.sendTransform(t)
def spin(s):
    e = time.time() + s
    while time.time() < e: rclpy.spin_once(n, timeout_sec=0.02)
cli = n.create_client(Trigger, '/perception/survey/start'); cli.wait_for_service(timeout_sec=10)
f = cli.call_async(Trigger.Request())
while not f.done(): spin(0.05)
def send():
    m = PuckArray(); m.header.stamp = n.get_clock().now().to_msg(); m.header.frame_id = 'base_link'
    p = Puck(); p.tag_id = -1; p.color = 1; p.shape = 1; p.camera = 0; p.confidence = 0.9
    p.pose.position.x, p.pose.position.y = 1.428, 0.95; p.pose.orientation.w = 1.0
    before = (snap['slots'][0]['n_obs'], sum(snap.get('rejected', {}).values()))
    m.pucks.append(p); pk.publish(m)
    e = time.time() + 3.0                       # 결과가 바뀔 때까지(1차: 0.3 s 고정 대기라 한 박자 늦게 읽음)
    while time.time() < e and (snap['slots'][0]['n_obs'], sum(snap.get('rejected', {}).values())) == before: spin(0.02)
    spin(0.1)
    s = snap['slots'][0]; return {'n_obs': s['n_obs'], 'rejected': dict(snap.get('rejected', {}))}
rep = {}
spin(1.0); rep['no_status'] = send()
st_pub = n.create_publisher(String, '/localization/status', lat)
st_pub.publish(String(data='LOST')); spin(0.3); rep['lost'] = send()
st_pub.publish(String(data='CONFIRM')); spin(0.3); rep['confirm'] = send()
st_pub.publish(String(data='OK')); spin(0.3); rep['ok'] = send()
st_pub.publish(String(data='SUSPECT')); spin(0.3); rep['suspect'] = send()
spin(1.5); rep['stale_after_suspect'] = send()
rep['pass'] = (rep['no_status']['n_obs'] == 1 and rep['lost']['n_obs'] == 1 and rep['lost']['rejected'].get('localization_lost') == 1
               and rep['confirm']['rejected'].get('localization_confirm') == 1 and rep['ok']['n_obs'] == 2
               and rep['suspect']['rejected'].get('localization_suspect') == 1 and rep['stale_after_suspect']['n_obs'] == 3)
json.dump(rep, open(f'{out}/summary.json', 'w'), ensure_ascii=False, indent=1); print(json.dumps(rep, ensure_ascii=False))
os.killpg(proc.pid, signal.SIGINT); proc.wait(timeout=10); n.destroy_node(); rclpy.shutdown()
