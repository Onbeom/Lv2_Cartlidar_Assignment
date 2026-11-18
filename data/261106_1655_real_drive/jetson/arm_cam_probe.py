#!/usr/bin/env python3
"""/arm_camera/image_raw 를 받아 크기 · 주기를 재고 한 장을 저장한다."""
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

secs = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
out = sys.argv[2] if len(sys.argv) > 2 else 'arm_cam_sample.jpg'
rclpy.init()
n = rclpy.create_node('arm_cam_probe')
ts, last = [], {}


def cb(m):
    ts.append(time.monotonic())
    last['m'] = m


n.create_subscription(Image, '/arm_camera/image_raw', cb, qos_profile_sensor_data)
t0 = time.monotonic()
while time.monotonic() - t0 < secs + 10 and (not ts or ts[-1] - ts[0] < secs):
    rclpy.spin_once(n, timeout_sec=0.1)
if len(ts) < 2:
    print('결과: 영상 없음')
    sys.exit(1)
m = last['m']
d = np.diff(ts)
print(f'결과: {m.width}x{m.height} {m.encoding} · {len(ts)} 장 · 평균 {1 / d.mean():.1f} Hz · 간격 최대 {d.max() * 1000:.0f} ms')
a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1)
cv2.imwrite(out, cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else a, [cv2.IMWRITE_JPEG_QUALITY, 90])
print('저장:', out)
