#!/usr/bin/env python3
"""P3 기록 (읽기만): /perception/barcode · /perception/pucks · /perception/task_marker 에서 새로 나온 값만 화면과 파일에 쓴다. Ctrl+C 면 요약.

p3_probe.py <기록 파일>
"""
import json
import signal
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import Barcode, PuckArray, TaskMarker

out = open(sys.argv[1], 'w', encoding='utf-8', buffering=1)
rclpy.init()
node = rclpy.create_node('p3_probe')
seen = {'barcode': {}, 'puck_tag': {}, 'marker': {}}


def rec(kind, key, extra):
    d = seen[kind]
    first = key not in d
    d[key] = d.get(key, 0) + 1
    out.write(json.dumps(dict(t=round(time.time(), 3), kind=kind, key=key, **extra), ensure_ascii=False) + '\n')
    if first:
        print(f'새 {kind}: {key} {extra}', flush=True)


def on_barcode(m):
    if m.source == Barcode.READ and m.read_ok:
        rec('barcode', m.text.strip(), {'symbols': int(m.symbols)})


def on_pucks(m):
    for p in m.pucks:
        if p.tag_id >= 0:
            rec('puck_tag', int(p.tag_id), {'color': int(p.color), 'shape': int(p.shape), 'camera': int(p.camera)})


def on_marker(m):
    if m.found:
        rec('marker', int(m.marker_id), {'camera': int(m.camera), 'dist': round(m.distance, 3)})


q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(Barcode, '/perception/barcode', on_barcode, q)
node.create_subscription(PuckArray, '/perception/pucks', on_pucks, q)
node.create_subscription(TaskMarker, '/perception/task_marker', on_marker, q)
stop = {'flag': False}
signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))
signal.signal(signal.SIGTERM, lambda *_: stop.update(flag=True))
while not stop['flag']:
    rclpy.spin_once(node, timeout_sec=0.1)
summary = {k: dict(sorted(v.items(), key=lambda x: str(x[0]))) for k, v in seen.items()}
out.write(json.dumps({'kind': 'summary', **{k: {str(a): b for a, b in v.items()} for k, v in summary.items()}}, ensure_ascii=False) + '\n')
print(json.dumps({k: {str(a): b for a, b in v.items()} for k, v in summary.items()}, ensure_ascii=False))
node.destroy_node()
rclpy.shutdown()
