"""측정 전용: URDF 로 robot_state_publisher 입력을 흉내 낸다. 움직이는 관절은 0 고정 50 Hz, odom → URDF 뿌리는 고정."""
import sys, subprocess, xml.etree.ElementTree as ET, yaml, rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
xac, out = sys.argv[1], sys.argv[2]
xml_s = subprocess.check_output(['xacro', xac], text=True)
yaml.safe_dump({'robot_state_publisher': {'ros__parameters': {'robot_description': xml_s, 'publish_frequency': max(50.0, float(sys.argv[3]) if len(sys.argv) > 3 else 50.0)}}}, open(out, 'w'), allow_unicode=True)
r = ET.fromstring(xml_s)
kids = {j.find('child').get('link') for j in r.findall('joint')}
root = [l.get('name') for l in r.findall('link') if l.get('name') not in kids][0]
names = [j.get('name') for j in r.findall('joint') if j.get('type') != 'fixed' and j.find('mimic') is None]
print('뿌리', root, '· 움직이는 관절', len(names), flush=True)
rclpy.init(); n = rclpy.create_node('tf_stub')
p = n.create_publisher(JointState, '/joint_states', 10)
ps = n.create_publisher(TFMessage, '/tf_static', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
t = TransformStamped(); t.header.stamp = n.get_clock().now().to_msg(); t.header.frame_id = 'odom'; t.child_frame_id = root; t.transform.rotation.w = 1.0
ps.publish(TFMessage(transforms=[t]))
def tick():
    m = JointState(); m.header.stamp = n.get_clock().now().to_msg(); m.name = names; m.position = [0.0] * len(names); p.publish(m)
n.create_timer(1.0 / float(sys.argv[3]) if len(sys.argv) > 3 else 0.02, tick)
rclpy.spin(n)
