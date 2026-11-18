#!/usr/bin/env python3
"""측정 전용: 앞 D435 컬러에서 밝은 화소(V ≥ v_min, S ≥ s_min)의 H · S · V 를 0.2 s 마다 기록하고, 한 주기 동안 장면을 몇 장 저장한다.

signal_hsv.py <출력 폴더> [초]   (영상 위 신호등 자리를 모를 때, 화면에서 가장 큰 밝은 덩어리를 등으로 본다)
"""
import json
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

out = sys.argv[1]
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0
os.makedirs(out, exist_ok=True)
V_MIN, S_MIN = 150, 60          # 인지 기본(200 · 75)보다 조금 넓게 잡아 문턱 근처도 본다
rclpy.init()
n = rclpy.create_node('signal_hsv')
last = {'img': None, 't': 0.0}


def on_img(m):
    a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1)
    last['img'] = cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else a.copy()


n.create_subscription(Image, '/top_camera_front/color/image_raw', on_img, qos_profile_sensor_data)
log = open(os.path.join(out, 'hsv.jsonl'), 'w')
t0 = time.monotonic()
k = 0
next_t = 0.0
while time.monotonic() - t0 < secs:
    rclpy.spin_once(n, timeout_sec=0.02)
    t = time.monotonic() - t0
    if last['img'] is None or t < next_t:
        continue
    next_t = t + 0.2
    img = last['img']
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = ((hsv[..., 2] >= V_MIN) & (hsv[..., 1] >= S_MIN)).astype(np.uint8)
    num, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
    rec = {'t': round(t, 2), 'blob': None}
    if num > 1:
        i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        px = hsv[lab == i]
        h = px[:, 0].astype(int)
        rec['blob'] = {'area': int(stats[i, cv2.CC_STAT_AREA]), 'x': round(float(cent[i][0]), 1), 'y': round(float(cent[i][1]), 1),
                       'h_med': int(np.median(h)), 'h_p10': int(np.percentile(h, 10)), 'h_p90': int(np.percentile(h, 90)),
                       's_med': int(np.median(px[:, 1])), 'v_med': int(np.median(px[:, 2])), 'v_max': int(px[:, 2].max())}
    log.write(json.dumps(rec) + '\n')
    if k % 5 == 0:
        cv2.imwrite(os.path.join(out, f'{k:04d}.jpg'), img, [cv2.IMWRITE_JPEG_QUALITY, 90])
    k += 1
print('기록', k, '장')
