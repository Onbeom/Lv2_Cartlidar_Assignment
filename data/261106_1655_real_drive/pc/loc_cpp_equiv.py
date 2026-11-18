#!/usr/bin/env python3
"""Python ↔ C++ 출력 비교 (PC, 11-14): scan_body_mask · scan_merge.
가짜 라이다 앞 · 뒤(10 Hz, 실물처럼 roll π 장착, 점 수 399 · 400 섞음, 차체 상자 안 점 · inf 섞음)와 움직이는 odom → base_link TF 를 내고,
Python 판(/scan_*_filtered, /scan_merged)과 C++ 판(/scan_*_cpp, /scan_merged_cpp)이 같은 입력에서 낸 값을 stamp 로 짝지어 비교한다.
  loc_cpp_equiv.py <초=8> <결과 json>
"""
import json
import math
import random
import sys
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
OUT = sys.argv[2] if len(sys.argv) > 2 else '/tmp/loc_cpp_equiv.json'


def q_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2), math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


class Equiv(Node):
    def __init__(self):
        super().__init__('loc_cpp_equiv')
        self.rng = random.Random(20261114)
        st = StaticTransformBroadcaster(self)
        ts = []
        for name, x, yaw in (('laser_front', 0.1318, 0.0), ('laser_rear', -0.1318, math.pi)):
            t = TransformStamped()
            t.header.frame_id, t.child_frame_id = 'base_link', name
            t.transform.translation.x, t.transform.translation.z = x, 0.0675
            qx, qy, qz, qw = q_rpy(math.pi, 0.0, yaw)
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = qx, qy, qz, qw
            ts.append(t)
        st.sendTransform(ts)
        self.tfb = TransformBroadcaster(self)
        self.pf = self.create_publisher(LaserScan, '/scan_front', qos_profile_sensor_data)
        self.pr = self.create_publisher(LaserScan, '/scan_rear', qos_profile_sensor_data)
        self.got = {k: {} for k in ('/scan_front_filtered', '/scan_front_cpp', '/scan_rear_filtered', '/scan_rear_cpp',
                                     '/scan_merged', '/scan_merged_cpp')}
        for k in self.got:
            self.create_subscription(LaserScan, k, lambda m, k=k: self.rx(k, m), 50)
        self.t0 = time.time()
        self.k = 0
        self.create_timer(0.02, self.tick_tf)
        self.create_timer(0.1, self.tick_scan)

    def rx(self, k, m):
        self.got[k][f'{m.header.stamp.sec}.{m.header.stamp.nanosec:09d}'] = list(m.ranges)

    def tick_tf(self):
        now = self.get_clock().now()
        s = time.time() - self.t0
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id, t.child_frame_id = 'odom', 'base_link'
        t.transform.translation.x, t.transform.translation.y = 0.2 * s, 0.05 * math.sin(s)
        _, _, qz, qw = q_rpy(0, 0, 0.3 * s)
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tfb.sendTransform(t)

    def scan(self, frame, stamp):
        n = 400 if self.rng.random() < 0.7 else 399   # 실물 coin_d4 처럼 점 수가 바뀜
        m = LaserScan()
        m.header.frame_id, m.header.stamp = frame, stamp
        m.angle_min, m.angle_increment = -math.pi, 2 * math.pi / 400
        m.angle_max = m.angle_min + m.angle_increment * (n - 1)
        m.range_min, m.range_max, m.scan_time = 0.05, 12.0, 0.1
        r = []
        for i in range(n):
            u = self.rng.random()
            if u < 0.05:
                r.append(math.inf)
            elif u < 0.15:
                r.append(self.rng.uniform(0.02, 0.25))   # 차체 상자 안 쪽 점
            elif u < 0.17:
                r.append(math.nan)
            else:
                r.append(self.rng.uniform(0.3, 9.0))
        m.ranges = r
        return m

    def tick_scan(self):
        self.k += 1
        st = self.get_clock().now()
        self.pf.publish(self.scan('laser_front', st.to_msg()))
        # 뒤 스캔: 절반은 같은 stamp, 절반은 30 ms 늦게(움직임 보정 길)
        if self.k % 2:
            self.pr.publish(self.scan('laser_rear', st.to_msg()))
        else:
            time.sleep(0.03)
            self.pr.publish(self.scan('laser_rear', self.get_clock().now().to_msg()))


def cmp(a, b):
    import math as m
    keys = sorted(set(a) & set(b))
    worst, nd = 0.0, 0
    for k in keys:
        x, y = a[k], b[k]
        if len(x) != len(y):
            nd += 1
            continue
        for u, v in zip(x, y):
            if (m.isinf(u) and m.isinf(v)) or (m.isnan(u) and m.isnan(v)):
                continue
            if m.isinf(u) or m.isinf(v) or m.isnan(u) or m.isnan(v):
                nd += 1
                worst = float('inf')
                continue
            worst = max(worst, abs(u - v))
    return dict(pairs=len(keys), only_py=len(set(a) - set(b)), only_cpp=len(set(b) - set(a)), max_abs_diff=worst, mismatch=nd)


def main():
    rclpy.init()
    n = Equiv()
    end = time.time() + DUR
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.01)
    res = {
        'scan_body_mask front': cmp(n.got['/scan_front_filtered'], n.got['/scan_front_cpp']),
        'scan_body_mask rear': cmp(n.got['/scan_rear_filtered'], n.got['/scan_rear_cpp']),
        'scan_merge': cmp(n.got['/scan_merged'], n.got['/scan_merged_cpp']),
    }
    json.dump(res, open(OUT, 'w'), indent=1)
    for k, v in res.items():
        print(k, v)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
