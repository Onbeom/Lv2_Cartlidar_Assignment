#!/usr/bin/env python3
"""앞 D435 에서 밝은 초록 · 빨강/노랑 덩어리의 base_link 높이 (읽기만). light_height.py [초=6]
색 덩어리(HSV: 초록 H 60~100, 따뜻한 색 H 0~35 또는 160~180, S≥100, V≥200) 의 화소 깊이 중앙값으로 광학 좌표를 만들고
촬영 시각 TF 로 base_link 로 옮긴다. 인지 노드의 signal.height_min_m(0.05) 과 비교하려는 확인 도구."""
import sys, time
import cv2, numpy as np, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
import tf2_ros
from rclpy.time import Time
secs = float(sys.argv[1]) if len(sys.argv) > 1 else 6
rclpy.init(); n = rclpy.create_node('light_height')
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
st = {}
n.create_subscription(Image, '/top_camera_front/color/image_raw', lambda m: st.__setitem__('c', m), qos_profile_sensor_data)
n.create_subscription(Image, '/top_camera_front/aligned_depth_to_color/image_raw', lambda m: st.__setitem__('d', m), qos_profile_sensor_data)
n.create_subscription(CameraInfo, '/top_camera_front/color/camera_info', lambda m: st.__setitem__('k', m), qos_profile_sensor_data)
res = {'G': [], 'W': []}
t0 = time.time()
while time.time() - t0 < secs:
    rclpy.spin_once(n, timeout_sec=0.05)
    if not all(k in st for k in 'cdk'): continue
    c, d, k = st.pop('c'), st['d'], st['k']
    if abs((c.header.stamp.sec + c.header.stamp.nanosec * 1e-9) - (d.header.stamp.sec + d.header.stamp.nanosec * 1e-9)) > 0.02: continue
    img = np.frombuffer(bytes(c.data), np.uint8).reshape(c.height, c.width, 3)
    bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if c.encoding == 'rgb8' else img
    dep = np.frombuffer(bytes(d.data), np.uint16).reshape(d.height, d.width) * 0.001
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    H, S, V = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    masks = {'G': (H >= 60) & (H <= 100) & (S >= 100) & (V >= 200), 'W': ((H <= 35) | (H >= 160)) & (S >= 100) & (V >= 200)}
    try:
        T = buf.lookup_transform('base_link', c.header.frame_id, Time.from_msg(c.header.stamp))
    except Exception:
        continue
    q = T.transform.rotation; tr = T.transform.translation
    R = np.array([[1 - 2 * (q.y**2 + q.z**2), 2 * (q.x * q.y - q.z * q.w), 2 * (q.x * q.z + q.y * q.w)],
                  [2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.x**2 + q.z**2), 2 * (q.y * q.z - q.x * q.w)],
                  [2 * (q.x * q.z - q.y * q.w), 2 * (q.y * q.z + q.x * q.w), 1 - 2 * (q.x**2 + q.y**2)]])
    for key, m in masks.items():
        ys, xs = np.nonzero(m)
        if len(xs) < 30: continue
        z = dep[ys, xs]; ok = (z > 0.1) & (z < 3)
        if ok.sum() < 10: continue
        zz = np.median(z[ok]); u = np.median(xs); v = np.median(ys)
        p = np.array([(u - k.k[2]) / k.k[0] * zz, (v - k.k[5]) / k.k[4] * zz, zz])
        b = R @ p + np.array([tr.x, tr.y, tr.z])
        res[key].append((b[0], b[1], b[2], len(xs), zz))
for key, v in res.items():
    if not v: print(key, '없음'); continue
    a = np.array(v)
    print(f"{'초록' if key == 'G' else '따뜻한 색'}: {len(v)} 장, base_link x {np.median(a[:,0]):.3f} y {np.median(a[:,1]):.3f} z {np.median(a[:,2]):.3f} (z 범위 {a[:,2].min():.3f}~{a[:,2].max():.3f}), 화소 {np.median(a[:,3]):.0f}, 깊이 {np.median(a[:,4]):.3f} m")
