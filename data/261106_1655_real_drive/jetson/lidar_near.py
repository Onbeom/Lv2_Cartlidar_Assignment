#!/usr/bin/env python3
"""라이다 가리기 확인 (읽기만): /scan_front 에서 1 s 마다 15 cm 안 점 수 · 가장 가까운 거리. lidar_near.py [초=240]"""
import sys, time, math, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
secs = float(sys.argv[1]) if len(sys.argv) > 1 else 240
rclpy.init(); n = rclpy.create_node('lidar_near'); last = {}
n.create_subscription(LaserScan, '/scan_front', lambda m: last.__setitem__('m', m), qos_profile_sensor_data)
t0 = time.time(); tp = 0
while time.time() - t0 < secs:
    rclpy.spin_once(n, timeout_sec=0.1)
    if 'm' in last and time.time() - tp > 1.0:
        tp = time.time(); m = last['m']
        v = [r for r in m.ranges if math.isfinite(r) and r > 0.0]
        near = sum(1 for r in v if r < 0.15)
        print(f"{time.strftime('%H:%M:%S')}  15 cm 안 점 {near:3d}  가장 가까움 {min(v) if v else float('nan'):.3f} m  (유효 {len(v)})", flush=True)
