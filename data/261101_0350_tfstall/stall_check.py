"""TFmini 끊김 확인 (PC, 모의 · 도구 확인): 가짜 PTY 시리얼 → 실제 tfmini · obstacle_stop 실행 파일.
물리 센서 · 모터는 쓰지 않는다. 10-29 개인 도구(261029_1408/tfmini/uart_tool.py)를 끊김 경우에 맞춰 줄인 것.

순서: 켬(입력 없음) → 먼 유효 거리 2.0 m 0.6 s(풀림) → 시리얼 끊김 1.0 s → 다시 2.0 m 0.6 s(복구)
실행: python3 stall_check.py <install 폴더> <결과 폴더>
"""
import json
import os
import signal
import struct
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String

install, out = Path(sys.argv[1]), Path(sys.argv[2])
out.mkdir(parents=True, exist_ok=False)
master, slave = os.openpty()
port = os.ttyname(slave)
children, logs = [], []
report = {'physical_hardware_used': False, 'install': str(install)}
rclpy.init()


class Probe(Node):
    def __init__(self):
        super().__init__('tfmini_stall_probe')
        self.stops, self.reasons, self.ranges = [], [], []
        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/perception/obstacle_stop', lambda m: self.stops.append((time.monotonic(), m.data)), q)
        self.create_subscription(String, '/perception/obstacle_stop/reason', lambda m: self.reasons.append((time.monotonic(), m.data)), q)
        self.create_subscription(Range, '/tfmini/range', lambda m: self.ranges.append((time.monotonic(), float(m.range))),
                                 qos_profile_sensor_data)

    def tick(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.005)

    def feed(self, distance_cm, strength, seconds):
        end = time.monotonic() + seconds
        last = None
        while time.monotonic() < end:
            data = b'\x59\x59' + struct.pack('<HHH', distance_cm, strength, 2048)
            os.write(master, data + bytes([sum(data) & 255]))
            last = time.monotonic()
            self.tick(0.02)
        return last

    def stopped(self):
        return self.stops[-1][1] if self.stops else None


def start(name, args):
    f = (out / (name + '.log')).open('w')
    logs.append(f)
    children.append((name, subprocess.Popen(args, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)))


n = Probe()
try:
    lib = install / 'perception/lib/perception'
    start('driver', [str(lib / 'tfmini'), '--ros-args', '-p', 'port:=' + port, '-p', 'publish_rate:=50.0', '-p', 'stale_timeout_s:=0.2'])
    start('stop', [str(lib / 'obstacle_stop'), '--ros-args', '-p', 'stale_s:=0.2', '-p', 'publish_rate:=10.0'])
    n.tick(3.0)
    report['startup_stopped'] = n.stopped() is True
    n.feed(200, 500, 0.6)
    report['far_clear'] = n.stopped() is False
    last = n.feed(200, 500, 0.05)
    i_range = len(n.ranges)
    n.tick(1.0)
    after = [t for t, s in n.stops if s and t >= last]
    report['stall_stops'] = n.stopped() is True
    report['stall_stop_delay_s'] = round(after[0] - last, 3) if after else None
    report['range_msgs_during_stall'] = len([r for r in n.ranges[i_range:] if r[0] > last + 0.1])
    n.feed(200, 500, 0.6)
    report['recovery_clear'] = n.stopped() is False
    report['reason_values'] = sorted(set(v for _, v in n.reasons))
    report['pass'] = (report['startup_stopped'] and report['far_clear'] and report['stall_stops'] and report['recovery_clear']
                      and report['stall_stop_delay_s'] is not None and report['stall_stop_delay_s'] <= 0.35
                      and report['range_msgs_during_stall'] == 0)
finally:
    for name, c in reversed(children):
        try:
            os.killpg(c.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            c.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(c.pid, signal.SIGTERM)
            c.wait(timeout=5)
    report['processes'] = [{'name': name, 'exit': c.poll()} for name, c in children]
    (out / 'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    (out / 'stop_events.json').write_text(json.dumps(n.stops))
    print(json.dumps(report, ensure_ascii=False), flush=True)
    n.destroy_node()
    rclpy.shutdown()
    os.close(master)
    os.close(slave)
    for f in logs:
        f.close()
