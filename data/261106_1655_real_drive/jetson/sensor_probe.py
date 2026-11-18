#!/usr/bin/env python3
"""센서 토픽을 정해진 시간 동안 받아 크기 · 형식 · frame_id · 주기 · 지연 · camera_info 를 적고, 영상은 한 장씩 저장한다.

  python3 sensor_probe.py <초> <저장 폴더> [reliable|sensor_data] [토픽 ...]
  토픽을 주면 그 영상 토픽만 받는다(도구의 동시 수신 한계를 가리는 단독 측정)
  reliable = 드라이버가 낸 주기, sensor_data(best effort) = perception 구독(image_qos: sensor_data)이 받는 주기
"""
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
OUT = sys.argv[2] if len(sys.argv) > 2 else '.'
MODE = sys.argv[3] if len(sys.argv) > 3 else 'reliable'
IMAGES = ['/arm_camera/image_raw',
          '/top_camera_front/color/image_raw', '/top_camera_front/aligned_depth_to_color/image_raw',
          '/top_camera_rear/color/image_raw', '/top_camera_rear/aligned_depth_to_color/image_raw']
INFOS = ['/arm_camera/camera_info', '/top_camera_front/color/camera_info', '/top_camera_rear/color/camera_info']
ONLY = sys.argv[4:]
if ONLY:
    IMAGES, INFOS = ONLY, []

rclpy.init()
node = rclpy.create_node('sensor_probe')
QOS = qos_profile_sensor_data if MODE == 'sensor_data' else QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
stat = {t: {'t': [], 'age': [], 'last': None} for t in IMAGES + INFOS}


def make_cb(topic):
    def cb(m):
        now = time.monotonic()
        s = stat[topic]
        s['t'].append(now)
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        s['age'].append(time.time() - stamp)
        s['last'] = m
    return cb


for t in IMAGES:
    node.create_subscription(Image, t, make_cb(t), QOS)
for t in INFOS:
    node.create_subscription(CameraInfo, t, make_cb(t), QOS)

t0 = time.monotonic()
while time.monotonic() - t0 < SECS:
    rclpy.spin_once(node, timeout_sec=0.05)


def rate_line(s):
    ts = np.array(s['t'])
    if len(ts) < 2:
        return f'{len(ts)} 개'
    d = np.diff(ts)
    age = np.array(s['age']) * 1000
    return (f'{len(ts)} 개 · 평균 {1 / d.mean():.1f} Hz · 간격 최대 {d.max() * 1000:.0f} ms · '
            f'stamp 나이 p50 {np.median(age):.0f} ms p99 {np.percentile(age, 99):.0f} ms')


def save_image(topic, m):
    name = topic.strip('/').replace('/', '_')
    a = np.frombuffer(bytes(m.data), np.uint8)
    if m.encoding in ('16UC1', 'mono16'):
        d = a.view(np.uint16).reshape(m.height, m.width)
        valid = d > 0
        mid = d[m.height // 2 - 20:m.height // 2 + 20, m.width // 2 - 20:m.width // 2 + 20]
        mv = mid[mid > 0]
        vis = cv2.applyColorMap(cv2.convertScaleAbs(d, alpha=255.0 / 3000.0), cv2.COLORMAP_JET)
        vis[~valid] = 0
        cv2.imwrite(os.path.join(OUT, name + '.jpg'), vis)
        return f'유효 화소 {valid.mean() * 100:.0f} % · 가운데 40×40 중앙값 {np.median(mv) if mv.size else 0:.0f} mm'
    img = a.reshape(m.height, m.width, -1)
    if m.encoding == 'rgb8':
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    cv2.imwrite(os.path.join(OUT, name + '.jpg'), img, [cv2.IMWRITE_JPEG_QUALITY, 90])
    return f'밝기 평균 {img.mean():.0f} · 포화(≥250) {(img >= 250).mean() * 100:.1f} %'


print(f'== 토픽 ({SECS:.0f} s, 구독 {MODE})')
for t in IMAGES:
    s = stat[t]
    m = s['last']
    if m is None:
        print(f'{t}: 없음')
        continue
    extra = save_image(t, m) if MODE == 'reliable' and not ONLY else ''
    print(f'{t}: {m.width}x{m.height} {m.encoding} frame_id={m.header.frame_id} · {rate_line(s)} · {extra}')
for t in INFOS:
    s = stat[t]
    m = s['last']
    if m is None:
        print(f'{t}: 없음')
        continue
    k = list(m.k)
    print(f'{t}: {m.width}x{m.height} frame_id={m.header.frame_id} · fx {k[0]:.1f} fy {k[4]:.1f} cx {k[2]:.1f} cy {k[5]:.1f} · '
          f'{m.distortion_model} D {len(m.d)} 개 [{", ".join(f"{v:.4g}" for v in list(m.d)[:3])}…] · {rate_line(s)}')
node.destroy_node()
rclpy.shutdown()
