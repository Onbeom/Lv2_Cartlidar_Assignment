#!/usr/bin/env python3
"""켜진 등 프레임별 화면 행 · 깊이 · base_link 높이 기록 (읽기만). lamp_rows.py <출력 jsonl> [초=40]
밝은 색 화소(S ≥ 75, V ≥ 200)의 가장 큰 덩어리: 색상 중앙값, 중심 화소(u, v), 깊이 중앙값, base_link 위치."""
import sys, time, json
import cv2, numpy as np, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
import tf2_ros
from rclpy.time import Time
out = open(sys.argv[1], 'w'); secs = float(sys.argv[2]) if len(sys.argv) > 2 else 40
rclpy.init(); n = rclpy.create_node('lamp_rows')
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
col, dep, st = {}, {}, {}
def ns(m): return m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
n.create_subscription(Image, '/top_camera_front/color/image_raw', lambda m: col.__setitem__(ns(m), m), qos_profile_sensor_data)
n.create_subscription(Image, '/top_camera_front/aligned_depth_to_color/image_raw', lambda m: dep.__setitem__(ns(m), m), qos_profile_sensor_data)
n.create_subscription(CameraInfo, '/top_camera_front/color/camera_info', lambda m: st.__setitem__('k', m), qos_profile_sensor_data)
t0 = time.time(); cnt = 0
while time.time() - t0 < secs:
    rclpy.spin_once(n, timeout_sec=0.01)
    for k in sorted(set(col) & set(dep)):
        c = col.pop(k); d = dep.pop(k)
        if 'k' not in st: continue
        K = st['k'].k
        img = np.frombuffer(bytes(c.data), np.uint8).reshape(c.height, c.width, 3)
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV if c.encoding == 'rgb8' else cv2.COLOR_BGR2HSV)
        m = ((hsv[..., 1] >= 75) & (hsv[..., 2] >= 200)).astype(np.uint8)
        nl, lab, stats, cen = cv2.connectedComponentsWithStats(m)
        if nl <= 1: continue
        i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        if stats[i, cv2.CC_STAT_AREA] < 100: continue
        sel = lab == i
        z = np.frombuffer(bytes(d.data), np.uint16).reshape(d.height, d.width)[sel] * 0.001
        z = z[(z > 0.1) & (z < 3)]
        if len(z) < 10: continue
        zz = float(np.median(z)); u, v = cen[i]
        try:
            T = buf.lookup_transform('base_link', c.header.frame_id, Time.from_msg(c.header.stamp))
        except Exception:
            continue
        q = T.transform.rotation; tr = T.transform.translation
        R = np.array([[1 - 2 * (q.y**2 + q.z**2), 2 * (q.x * q.y - q.z * q.w), 2 * (q.x * q.z + q.y * q.w)],
                      [2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.x**2 + q.z**2), 2 * (q.y * q.z - q.x * q.w)],
                      [2 * (q.x * q.z - q.y * q.w), 2 * (q.y * q.z + q.x * q.w), 1 - 2 * (q.x**2 + q.y**2)]])
        p = np.array([(u - K[2]) / K[0] * zz, (v - K[5]) / K[4] * zz, zz]); b = R @ p + np.array([tr.x, tr.y, tr.z])
        h = float(np.median(hsv[..., 0][sel]))
        out.write(json.dumps(dict(t=k * 1e-9, hue=h, area=int(stats[i, cv2.CC_STAT_AREA]), u=round(float(u), 1), v=round(float(v), 1),
                                  depth=round(zz, 4), x=round(float(b[0]), 4), y=round(float(b[1]), 4), z=round(float(b[2]), 4))) + '\n')
        cnt += 1
print('기록', cnt, '장')
