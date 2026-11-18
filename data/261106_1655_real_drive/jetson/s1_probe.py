#!/usr/bin/env python3
"""S1 기록 (읽기만, 명령을 내지 않음): /joy · /e_stop · teleop 속도 명령 · 바퀴 관절 속도를 JSON 줄로 남기고 끝에 요약한다.

s1_probe.py <기록 파일> [초]   (Ctrl+C 나 시간이 다 되면 요약)
요약: E-stop 이 잠길 때마다 마지막 /joy 뒤 걸린 시간, 명령 방향별 바퀴 속도 부호.
"""
import json
import signal
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

out = open(sys.argv[1], 'w', encoding='utf-8', buffering=1)
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 1e9
rclpy.init()
node = rclpy.create_node('s1_probe')
state = {'last_joy': None, 'estop': None, 'cmd': (0.0, 0.0), 'locks': [], 'wheel': {}, 'names': None}


def rec(kind, **kw):
    out.write(json.dumps(dict(t=round(time.monotonic(), 4), kind=kind, **kw), ensure_ascii=False) + '\n')


def on_joy(m):
    state['last_joy'] = time.monotonic()
    rec('joy', frame=m.header.frame_id, axes=[round(a, 3) for a in m.axes[:8]], buttons=list(m.buttons[:12]))


def on_estop(m):
    now = time.monotonic()
    if m.data and state['estop'] is False:
        gap = None if state['last_joy'] is None else round(now - state['last_joy'], 3)
        state['locks'].append(gap)
        rec('estop_lock', since_last_joy=gap)
    state['estop'] = bool(m.data)
    rec('estop', data=bool(m.data))


def on_cmd(m):
    tw = m.twist if hasattr(m, 'twist') else m
    state['cmd'] = (round(tw.linear.x, 3), round(tw.angular.z, 3))
    rec('cmd', v=state['cmd'][0], w=state['cmd'][1])


def on_js(m):
    idx = [i for i, n in enumerate(m.name) if 'wheel' in n]
    if not idx or not m.velocity:
        return
    state['names'] = [m.name[i] for i in idx]
    vel = [round(m.velocity[i], 3) for i in idx]
    v, w = state['cmd']
    key = 'stop' if v == 0 and w == 0 else ('forward' if v > 0 and w == 0 else 'back' if v < 0 and w == 0 else 'left' if w > 0 and v == 0 else 'right' if w < 0 and v == 0 else 'mixed')
    if key != 'stop':
        s = state['wheel'].setdefault(key, [[0, 0] for _ in idx])
        for k, x in enumerate(vel):
            if abs(x) > 0.05:
                s[k][0 if x > 0 else 1] += 1
    rec('wheel', cmd=[v, w], vel=vel)


latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(get_message('sensor_msgs/msg/Joy'), '/joy', on_joy, 10)
node.create_subscription(get_message('std_msgs/msg/Bool'), '/e_stop', on_estop, latched)
node.create_subscription(get_message('sensor_msgs/msg/JointState'), '/joint_states', on_js, 10)
types = {}
t_end = time.monotonic() + 5
while time.monotonic() < t_end and '/cmd_vel_teleop' not in types:
    rclpy.spin_once(node, timeout_sec=0.2)
    types = dict(node.get_topic_names_and_types())
if '/cmd_vel_teleop' in types:
    node.create_subscription(get_message(types['/cmd_vel_teleop'][0]), '/cmd_vel_teleop', on_cmd, 10)
    rec('note', text='cmd type ' + types['/cmd_vel_teleop'][0])
else:
    rec('note', text='/cmd_vel_teleop 없음')
stop = {'flag': False}
signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))
signal.signal(signal.SIGTERM, lambda *_: stop.update(flag=True))
t0 = time.monotonic()
while not stop['flag'] and time.monotonic() - t0 < secs:
    rclpy.spin_once(node, timeout_sec=0.1)
summary = dict(kind='summary', wheel_names=state['names'], lock_after_last_joy_s=state['locks'],
               wheel_sign_counts={k: [{'+': a, '-': b} for a, b in v] for k, v in state['wheel'].items()})
out.write(json.dumps(summary, ensure_ascii=False) + '\n')
print(json.dumps(summary, ensure_ascii=False, indent=1))
node.destroy_node()
rclpy.shutdown()
