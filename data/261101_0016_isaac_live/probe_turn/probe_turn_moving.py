"""도구 확인(모의, 판정 아님): 전진하며 돌 때 명령 w 와 참 회전 속도의 관계(4륜 스키드 죽은 구간)를 잰다.
설계 31(live_nav_v31)에서 전진 중 |w| ≤ 0.15 명령은 참 회전 0 이라 RPP 방향 보정이 듣지 않았음 → turn_floor 전진 앞먹임 곡선의 근거.
빈 바닥 장면(probe_turn_moving.sh)에 /cmd_vel_nav(TwistStamped, base_link)를 20 Hz 로 보내고 /sim/truth 로 단계마다 가운데 2.5 s 의 참 회전 · 전진 속도를 구한다.
사용: python3 probe_turn_moving.py --out <json> [--v 0 0.04 0.08 0.12] [--w 0.05 0.1 ...] [--hold 4.0]"""
import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

ap = argparse.ArgumentParser()
ap.add_argument('--out', required=True)
ap.add_argument('--v', type=float, nargs='*', default=[0.0, 0.04, 0.08, 0.12])
ap.add_argument('--w', type=float, nargs='*', default=[0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.45, 0.6, 0.8])
ap.add_argument('--hold', type=float, default=4.0)
a = ap.parse_args()

rclpy.init()
node = Node('probe_turn_moving', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
pub = node.create_publisher(TwistStamped, '/cmd_vel_nav', 10)
truth = []
node.create_subscription(PoseStamped, '/sim/truth', lambda m: truth.append(
    (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.position.x, m.pose.position.y,
     math.atan2(2 * m.pose.orientation.w * m.pose.orientation.z, 1 - 2 * m.pose.orientation.z ** 2))),
    QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))


def now():
    return node.get_clock().now().nanoseconds * 1e-9


def hold(v, w, dur):
    t0 = now()
    while now() - t0 < dur:
        m = TwistStamped()
        m.header.stamp = node.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.linear.x, m.twist.angular.z = v, w
        pub.publish(m)
        rclpy.spin_once(node, timeout_sec=0.05)
    return t0, now()


while not truth or now() <= 0.0:
    rclpy.spin_once(node, timeout_sec=0.1)
rows = []
sign = 1.0
for v in a.v:
    for w in a.w:
        hold(0.0, 0.0, 1.0)
        t0, t1 = hold(v, sign * w, a.hold)
        seg = [p for p in truth if t0 + 1.0 <= p[0] <= t0 + 3.5]
        if len(seg) < 10:
            rows.append(dict(v=v, w=sign * w, n=len(seg)))
            continue
        dyaw = sum(math.remainder(q[3] - p[3], 2 * math.pi) for p, q in zip(seg, seg[1:]))
        dist = sum(math.hypot(q[1] - p[1], q[2] - p[2]) for p, q in zip(seg, seg[1:]))
        dt = seg[-1][0] - seg[0][0]
        r = dict(v=v, w=sign * w, n=len(seg), w_act=round(dyaw / dt, 4), ratio=round(dyaw / dt / (sign * w), 3), v_act=round(dist / dt, 4))
        rows.append(r)
        print(json.dumps(r), flush=True)
        sign = -sign            # 좌우 번갈아 같은 자리 근처에 머물게
hold(0.0, 0.0, 1.0)
json.dump(dict(hold_s=a.hold, window='단계 시작 뒤 1.0~3.5 s', rows=rows), open(a.out, 'w'), indent=1)
node.destroy_node()
rclpy.shutdown()
