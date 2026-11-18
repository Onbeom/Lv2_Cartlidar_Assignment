#!/usr/bin/env python3
"""P5 판정 bag 의 앞 D435 영상(compressed)에 인지 노드와 같은 HSV 색 판별(detect_light_blobs, perception.yaml signal 값)을 적용해
프레임마다 신호등 자리 색을 CSV 로 낸다 (모의 재생 근사: 깊이 · 높이 거름 · 자리 고정 없음, JPEG 압축 영향 있음).
hue_frames.py <bag> <출력 csv>"""
import sys
import cv2, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage
C = dict(v_min=200, s_min=75, red_h_max=10, red2_h_min=170, yellow_h_min=15, yellow_h_max=67, green_h_min=68, green_h_max=97,
         red_v_min=180, yellow_v_min=218, green_v_min=200, red_warm_h_min=140, red_warm_v_min=150, min_area=4)
def blobs(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV); H, S, V = cv2.split(hsv)
    sat = S >= C['s_min']
    red = ((H <= C['red_h_max']) | (H >= C['red2_h_min'])) & sat & (V >= C['red_v_min'])
    red |= (H >= C['red_warm_h_min']) & (H < C['red2_h_min']) & sat & (V >= C['red_warm_v_min'])
    masks = {'R': red, 'Y': (H >= C['yellow_h_min']) & (H <= C['yellow_h_max']) & sat & (V >= C['yellow_v_min']),
             'G': (H >= C['green_h_min']) & (H <= C['green_h_max']) & sat & (V >= C['green_v_min'])}
    out = []
    for col, m in masks.items():
        n, _, st, cen = cv2.connectedComponentsWithStats(m.astype(np.uint8), connectivity=8)
        for i in range(1, n):
            if st[i, cv2.CC_STAT_AREA] >= C['min_area']:
                out.append((col, int(st[i, cv2.CC_STAT_AREA]), float(cen[i][0]), float(cen[i][1])))
    return out
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id=''), rosbag2_py.ConverterOptions('', ''))
frames = []
while r.has_next():
    topic, data, _ = r.read_next()
    if topic != '/top_camera_front/color/image_raw/compressed': continue
    m = deserialize_message(data, CompressedImage)
    img = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)
    frames.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, blobs(img)))
big = np.array([(u, v) for _, bs in frames for (_, a, u, v) in bs if a >= 100])
u0, v0 = np.median(big, 0)
print(f'신호등 화면 자리 u {u0:.0f} v {v0:.0f} (큰 덩어리 {len(big)})')
with open(sys.argv[2], 'w') as f:
    for t, bs in frames:
        near = [b for b in bs if abs(b[2] - u0) < 40 and abs(b[3] - v0) < 70]
        col = max(near, key=lambda b: b[1])[0] if near else 'OFF'
        f.write(f'{t:.4f},{col}\n')
print('프레임', len(frames))
