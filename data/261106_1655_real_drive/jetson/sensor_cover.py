#!/usr/bin/env python3
"""라이다 · TFmini 포트 가리기 (Jetson, 터미널에서, 엔터로 진행). sensor_cover.py [포트들=/dev/ttyUSB0,/dev/ttyUSB1,/dev/ttyUSB2]

포트마다 라이다 드라이버(sensors.launch, 앞 칸만)를 켜고 스캔이 나오면:
  엔터 ① 아무것도 안 가림(기준 3 s) → 엔터 ② 앞 라이다를 손바닥으로 바짝 → 엔터 ③ 뒤 라이다 → 엔터 ④ TFmini 렌즈
  단계마다 3 s 동안 '기준보다 새로 가까워진 각도 수'(기준 거리의 절반 아래로 줄고 0.25 m 안)를 잼 → 가장 크게 바뀐 단계가 그 포트의 장치.
스캔이 안 나오는 포트는 TFmini 프레임(59 59, 115200)을 읽어 같은 단계에서 거리가 바뀌는지 봄.
드라이버는 새 세션으로 켜서 포트마다 통째로 끔(남는 노드 없음). 결과 ~/real_drive/sensor_cover.log
"""
import math, os, signal, subprocess, sys, time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import serial

PORTS = (sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0,/dev/ttyUSB1,/dev/ttyUSB2').split(',')
STEPS = ['아무것도 안 가림 (기준)', '앞 라이다를 손바닥으로 바짝 가림', '뒤 라이다를 손바닥으로 바짝 가림', 'TFmini 렌즈를 손바닥으로 가림']
log = open(os.path.expanduser('~/real_drive/sensor_cover.log'), 'a', buffering=1)


def say(s):
    print(s, flush=True)
    log.write(s + '\n')


rclpy.init()
node = rclpy.create_node('sensor_cover')
scans = []
node.create_subscription(LaserScan, '/scan_front', lambda m: scans.append(list(m.ranges)), qos_profile_sensor_data)


def collect(sec):
    scans.clear()
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(node, timeout_sec=0.05)
    return list(scans)


def med_per_angle(ss):
    n = min(len(s) for s in ss)
    out = []
    for i in range(n):
        v = sorted(s[i] for s in ss if math.isfinite(s[i]) and s[i] > 0)
        out.append(v[len(v) // 2] if v else float('inf'))
    return out


def tf_read(port, sec):
    try:
        s = serial.Serial(port, 115200, timeout=0.1)
    except Exception as e:
        return None, str(e)
    s.reset_input_buffer(); t0 = time.time(); b = bytearray(); d = []
    while time.time() - t0 < sec:
        b += s.read(256)
        while len(b) >= 9:
            if b[0] == 0x59 and b[1] == 0x59 and (sum(b[:8]) & 0xFF) == b[8]:
                d.append(b[2] | b[3] << 8); del b[:9]
            else:
                del b[0]
    s.close()
    return (sorted(d)[len(d) // 2] if d else None), f'{len(d)} 프레임'


for port in PORTS:
    say(f'\n########## {port} ##########')
    lg = open(os.path.expanduser(f'~/real_drive/sensor_cover_{os.path.basename(port)}.launch.log'), 'w')
    proc = subprocess.Popen(['ros2', 'launch', 'bringup', 'sensors.launch.py', 'use_d435:=false', 'use_arm_camera:=false',
                             'use_lidar:=true', f'lidar_front_port:={port}', 'lidar_rear_port:=/dev/no_such_port'],
                            stdout=lg, stderr=subprocess.STDOUT, start_new_session=True)
    time.sleep(8)
    first = collect(3)
    is_lidar = len(first) >= 5
    say(f'{port}: 스캔 {len(first)} 개 / 3 s → {"라이다" if is_lidar else "스캔 없음 → TFmini 확인"}')
    if not is_lidar:
        os.killpg(proc.pid, signal.SIGINT); time.sleep(3)
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    base = None
    res = []
    for k, st in enumerate(STEPS):
        input(f'  [{k + 1}/4] {st} — 준비되면 엔터 ')
        if is_lidar:
            m = med_per_angle(collect(3))
            if base is None:
                base = m
                say(f'    기준: 각도 {len(m)} 칸, 0.25 m 안 {sum(1 for r in m if r < 0.25)} 칸')
                res.append(0)
                continue
            n = sum(1 for b, r in zip(base, m) if r < 0.25 and (not math.isfinite(b) or r < 0.5 * b))
            res.append(n)
            say(f'    {st}: 새로 가까워진 각도 {n} 칸')
        else:
            d, info = tf_read(port, 3)
            say(f'    {st}: TFmini 거리 중앙 {d} ({info})')
            res.append(d)
    if is_lidar:
        k = max(range(1, 4), key=lambda i: res[i])
        say(f'  ⇒ {port} = {STEPS[k].split("를")[0] if res[k] >= 5 else "판정 못 함(가렸을 때 변화 < 5 칸)"} (칸 수 {res[1:]})')
        os.killpg(proc.pid, signal.SIGINT); time.sleep(4)
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
say('\n끝 — 결과 ~/real_drive/sensor_cover.log')
