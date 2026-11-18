#!/usr/bin/env python3
"""켜진 등 프레임별 검출 요소 (읽기만). lamp_frames.py [초=10]
앞 D435 프레임마다 밝은 색 화소(S ≥ 75, V ≥ 200 = perception signal.s_min · v_min) 중 가장 큰 덩어리의 화소 수,
그 화소 중 깊이 유효(0.1~3 m) 비율, 깊이 짝 유무를 센다. 켜진 등이 있어야 할 프레임에서 무엇이 비는지 보려는 확인 도구."""
import sys, time, collections
import cv2, numpy as np, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
secs = float(sys.argv[1]) if len(sys.argv) > 1 else 10
rclpy.init(); n = rclpy.create_node('lamp_frames')
col, dep = {}, {}
def ns(m): return m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
n.create_subscription(Image, '/top_camera_front/color/image_raw', lambda m: col.__setitem__(ns(m), m), qos_profile_sensor_data)
n.create_subscription(Image, '/top_camera_front/aligned_depth_to_color/image_raw', lambda m: dep.__setitem__(ns(m), m), qos_profile_sensor_data)
t0 = time.time(); rows = []
while time.time() - t0 < secs:
    rclpy.spin_once(n, timeout_sec=0.02)
    for k in sorted(col):
        if k not in dep and time.time() - t0 < secs - 0.2 and len(col) < 5: break
        c = col.pop(k); d = dep.pop(k, None)
        img = np.frombuffer(bytes(c.data), np.uint8).reshape(c.height, c.width, 3)
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV if c.encoding == 'rgb8' else cv2.COLOR_BGR2HSV)
        m = ((hsv[..., 1] >= 75) & (hsv[..., 2] >= 200)).astype(np.uint8)
        nl, lab, stats, _ = cv2.connectedComponentsWithStats(m)
        if nl <= 1: rows.append((0, -1, d is not None)); continue
        i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA])); area = int(stats[i, cv2.CC_STAT_AREA])
        frac = -1
        if d is not None:
            z = np.frombuffer(bytes(d.data), np.uint16).reshape(d.height, d.width)[lab == i] * 0.001
            frac = float(((z > 0.1) & (z < 3)).mean())
        rows.append((area, frac, d is not None))
lit = [r for r in rows if r[0] >= 100]
print(f'프레임 {len(rows)} ({len(rows) / secs:.1f} Hz), 밝은 덩어리 ≥100 px {len(lit)}, 깊이 짝 없음 {sum(not r[2] for r in rows)}')
if lit:
    fr = np.array([r[1] for r in lit if r[1] >= 0])
    print(f'  켜진 프레임 덩어리 화소 중앙 {np.median([r[0] for r in lit]):.0f}, 깊이 유효 비율: 중앙 {np.median(fr):.2f}, 0 인 프레임 {int((fr == 0).sum())}/{len(fr)}, <0.1 {int((fr < 0.1).sum())}, <0.3 {int((fr < 0.3).sum())}')
