#!/usr/bin/env python3
"""S2 기록 (읽기만): /odom(바퀴) · /odometry/filtered(EKF) · /imu/data 를 받아, 멈춤(0.8 s) 사이를 한 판으로 나눠 판마다 거리 · 회전을 낸다.

s2_probe.py <기록 파일> [초]. 판이 끝날 때마다 한 줄씩 화면과 파일에 쓴다.
"""
import json
import math
import signal
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

out = open(sys.argv[1], 'w', encoding='utf-8', buffering=1)
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 1e9
rclpy.init()
node = rclpy.create_node('s2_probe')
STILL_V, STILL_W, STILL_S = 0.005, 0.02, 0.8


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


cur = {}
imu = {'yaw': 0.0, 't': None}
run = {'active': False, 'start': None, 'still_since': None, 'n': 0, 'imu0': 0.0, 't0': None, 'vmax': 0.0, 'wmax': 0.0}


def on_odom(name):
    def cb(m):
        p = m.pose.pose
        cur[name] = (p.position.x, p.position.y, yaw_of(p.orientation))
        if name != 'odom':
            return
        v, w = m.twist.twist.linear.x, m.twist.twist.angular.z
        now = time.monotonic()
        moving = abs(v) > STILL_V or abs(w) > STILL_W
        if moving and not run['active']:
            run.update(active=True, start=dict(cur), imu0=imu['yaw'], t0=now, still_since=None, vmax=0.0, wmax=0.0)
        if run['active']:
            run['vmax'] = max(run['vmax'], abs(v))
            run['wmax'] = max(run['wmax'], abs(w))
            if moving:
                run['still_since'] = None
            elif run['still_since'] is None:
                run['still_since'] = now
            elif now - run['still_since'] > STILL_S:
                finish(now)
    return cb


def unwrap(d):
    return (d + math.pi) % (2 * math.pi) - math.pi


def finish(now):
    run['active'] = False
    run['n'] += 1
    res = {'run': run['n'], 'dur_s': round(now - run['t0'] - STILL_S, 2), 'vmax': round(run['vmax'], 3), 'wmax': round(run['wmax'], 3)}
    for name, a in run['start'].items():
        b = cur[name]
        res[name] = {'dist_m': round(math.hypot(b[0] - a[0], b[1] - a[1]), 4), 'dx_m': round(b[0] - a[0], 4), 'dy_m': round(b[1] - a[1], 4),
                     'dyaw_deg': round(math.degrees(unwrap(b[2] - a[2])), 2), 'turn_deg': round(math.degrees(track[name] - track0[name]), 2)}
    res['imu_turn_deg'] = round(math.degrees(imu['yaw'] - run['imu0']), 2)
    out.write(json.dumps(res, ensure_ascii=False) + '\n')
    print(json.dumps(res, ensure_ascii=False), flush=True)


# 누적 회전(360° 넘게 돌아도 셈)
track, track0, last = {}, {}, {}


def accum(name):
    def cb(m):
        y = yaw_of(m.pose.pose.orientation)
        if name in last:
            track[name] = track.get(name, 0.0) + unwrap(y - last[name])
        else:
            track[name] = 0.0
        last[name] = y
    return cb


def on_imu(m):
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    if imu['t'] is not None and 0 < t - imu['t'] < 0.5:
        imu['yaw'] += m.angular_velocity.z * (t - imu['t'])
    imu['t'] = t


for topic, name in (('/odom', 'odom'), ('/odometry/filtered', 'ekf')):
    node.create_subscription(Odometry, topic, accum(name), 50)
    node.create_subscription(Odometry, topic, on_odom(name), 50)
node.create_subscription(Imu, '/imu/data', on_imu, 100)


_orig_finish = finish


def finish(now):  # noqa: F811  판 시작 때 누적 회전 기준을 같이 저장
    _orig_finish(now)


def mark_start():
    for k, v in track.items():
        track0[k] = v


stop = {'flag': False}
signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))
signal.signal(signal.SIGTERM, lambda *_: stop.update(flag=True))
t_end = time.monotonic() + secs
was = False
while not stop['flag'] and time.monotonic() < t_end:
    rclpy.spin_once(node, timeout_sec=0.05)
    if run['active'] and not was:
        mark_start()
    was = run['active']
node.destroy_node()
rclpy.shutdown()
