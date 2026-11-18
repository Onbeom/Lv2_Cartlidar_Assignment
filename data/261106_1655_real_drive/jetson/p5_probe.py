#!/usr/bin/env python3
"""P5 기록 (읽기만): /perception/signal 을 받을 때마다 한 줄 쓰고, 상태 · 순서가 바뀔 때 화면에 낸다. Ctrl+C 면 요약.

p5_probe.py <기록 파일>
요약: 상태별 장 수, 신호등이 보인 장 중 색별 장 수, 확정 · 후보 순서.
t = 받은 시각(Jetson time.time), stamp = 메시지 header.stamp(마지막으로 쓴 영상 촬영 시각).
"""
import collections
import json
import signal
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import SignalSequence as S

out = open(sys.argv[1], 'w', encoding='utf-8', buffering=1)
rclpy.init()
node = rclpy.create_node('p5_probe')
ST = {0: 'SEARCHING', 1: 'COUNTING', 2: 'CANDIDATE', 3: 'CONFIRMED', 4: 'MISMATCH'}
LT = {0: 'OFF', 1: 'R', 2: 'Y', 3: 'G'}
cnt = {'status': collections.Counter(), 'light': collections.Counter(), 'seq': collections.Counter()}
last = {'key': None}


def cb(m):
    st, lt = ST.get(m.status, m.status), LT.get(m.light, m.light)
    cnt['status'][st] += 1
    if m.light_visible:
        cnt['light'][lt] += 1
    if m.sequence:
        cnt['seq'][(st, m.sequence)] += 1
    out.write(json.dumps(dict(t=round(time.time(), 3), stamp=round(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, 4), status=st, seq=m.sequence, r=m.red, y=m.yellow, g=m.green, cycles=m.cycles,
                              light=lt, visible=m.light_visible, pos_ok=m.position_valid,
                              pos=[round(m.position.x, 3), round(m.position.y, 3), round(m.position.z, 3)],
                              t_on=round(m.t_on, 3), t_off=round(m.t_off, 3), t_term=round(m.t_term, 3), camera=m.camera)) + '\n')
    key = (st, m.sequence, m.cycles)
    if key != last['key']:
        last['key'] = key
        print(f'{time.strftime("%H:%M:%S")} {st} 순서 {m.sequence or "-"} (R{m.red} Y{m.yellow} G{m.green}, 묶음 {m.cycles}) '
              f'위치 {"있음" if m.position_valid else "없음"} 켜짐 {m.t_on:.2f} s 꺼짐 {m.t_off:.2f} s 텀 {m.t_term:.2f} s', flush=True)


node.create_subscription(S, '/perception/signal', cb, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
stop = {'flag': False}
signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))
signal.signal(signal.SIGTERM, lambda *_: stop.update(flag=True))
while not stop['flag']:
    rclpy.spin_once(node, timeout_sec=0.1)
summary = {'kind': 'summary', 'status': dict(cnt['status']), 'light_visible': dict(cnt['light']),
           'sequences': {f'{a} {b}': n for (a, b), n in cnt['seq'].items()}}
out.write(json.dumps(summary, ensure_ascii=False) + '\n')
print(json.dumps(summary, ensure_ascii=False))
node.destroy_node()
rclpy.shutdown()
