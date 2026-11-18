"""측정 전용: tf_stub.py 와 같되 팔 joint1~5 는 팔 OpenCR STATE 의 실제 각도(읽기만, 명령 없음), 나머지 관절은 0. 50 Hz.

tf_arm.py <xacro> <rsp.yaml 출력> [포트]
"""
import sys, subprocess, xml.etree.ElementTree as ET, yaml, rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from teleop_common import ROLE_ARM, find_port, tool

xac, out = sys.argv[1], sys.argv[2]
link = tool.Link(find_port(ROLE_ARM, sys.argv[3] if len(sys.argv) > 3 else 'auto'))
xml_s = subprocess.check_output(['xacro', xac], text=True)
yaml.safe_dump({'robot_state_publisher': {'ros__parameters': {'robot_description': xml_s, 'publish_frequency': 50.0}}}, open(out, 'w'))
r = ET.fromstring(xml_s)
kids = {j.find('child').get('link') for j in r.findall('joint')}
root = [l.get('name') for l in r.findall('link') if l.get('name') not in kids][0]
names = [j.get('name') for j in r.findall('joint') if j.get('type') != 'fixed' and j.find('mimic') is None]
arm = {f'joint{k + 1}': k for k in range(5)}
print('뿌리', root, '· 움직이는 관절', names, flush=True)
rclpy.init(); n = rclpy.create_node('tf_arm')
p = n.create_publisher(JointState, '/joint_states', 10)
ps = n.create_publisher(TFMessage, '/tf_static', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
t = TransformStamped(); t.header.stamp = n.get_clock().now().to_msg(); t.header.frame_id = 'odom'; t.child_frame_id = root; t.transform.rotation.w = 1.0
ps.publish(TFMessage(transforms=[t]))
last = {'q': None}


def tick():
    s = link.poll()
    if s is not None:
        last['q'] = tool.arm_pos(s)
    if last['q'] is None:
        return
    m = JointState(); m.header.stamp = n.get_clock().now().to_msg(); m.name = names
    m.position = [float(last['q'][arm[x]]) if x in arm else 0.0 for x in names]
    p.publish(m)


n.create_timer(0.02, tick)
rclpy.spin(n)
