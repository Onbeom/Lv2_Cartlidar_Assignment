"""도구 확인(모의): 빈 바닥 장면에 정해진 명령 순서를 /cmd_vel_nav(TwistStamped, base_link) 20 Hz 로 보낸다. 분석은 장면의 truth.csv 로(analyze.py).
순서: 정지 2 s → [제자리 회전 w = +0.3 · −0.3 · +0.5 · −0.5 · +0.8 · −0.8, 각 6 s, 사이 정지 2 s] → [직진 v = +0.1 · −0.1 · +0.2 · −0.2, 각 5 s, 사이 정지 2 s] → 정지 2 s."""
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

rclpy.init()
node = Node('probe_cmd', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
pub = node.create_publisher(TwistStamped, '/cmd_vel_nav', 10)
now = lambda: node.get_clock().now().nanoseconds * 1e-9


def hold(v, w, dur):
    t0 = now()
    while now() - t0 < dur:
        m = TwistStamped(); m.header.stamp = node.get_clock().now().to_msg(); m.header.frame_id = 'base_link'
        m.twist.linear.x, m.twist.angular.z = float(v), float(w); pub.publish(m); rclpy.spin_once(node, timeout_sec=0.05)


while now() <= 0.0:
    rclpy.spin_once(node, timeout_sec=0.1)
hold(0, 0, 2)
for w in (0.3, -0.3, 0.5, -0.5, 0.8, -0.8):
    hold(0, w, 6); hold(0, 0, 2)
for v in (0.1, -0.1, 0.2, -0.2):
    hold(v, 0, 5); hold(0, 0, 2)
hold(0, 0, 2)
print('probe_cmd 끝', flush=True)
node.destroy_node(); rclpy.shutdown()
