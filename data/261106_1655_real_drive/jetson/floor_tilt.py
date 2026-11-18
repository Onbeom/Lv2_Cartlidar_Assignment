#!/usr/bin/env python3
"""앞 D435 깊이로 바닥 면을 맞춰 차체 기울기를 잰다 (읽기만). floor_tilt.py [초=5]
영상 아래쪽 1/3 깊이 점을 base_link 로 옮겨 평면 z = a x + b y + c 로 맞춘다(중앙값 잔차로 두 번 거름).
차체가 바닥과 나란하면 a = b = 0 이고 c = 바닥 높이(base_link 기준). 앞 · 옆 기울기 [°]와 신호등 자리(x 0.42 m) 바닥 높이를 낸다."""
import sys, time, math
import numpy as np, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
import tf2_ros
from rclpy.time import Time
secs = float(sys.argv[1]) if len(sys.argv) > 1 else 5
rclpy.init(); n = rclpy.create_node('floor_tilt')
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
st = {}
n.create_subscription(Image, '/top_camera_front/aligned_depth_to_color/image_raw', lambda m: st.__setitem__('d', m), qos_profile_sensor_data)
n.create_subscription(CameraInfo, '/top_camera_front/color/camera_info', lambda m: st.__setitem__('k', m), qos_profile_sensor_data)
out = []
t0 = time.time()
while time.time() - t0 < secs:
    rclpy.spin_once(n, timeout_sec=0.05)
    if 'd' not in st or 'k' not in st: continue
    d, k = st.pop('d'), st['k']
    try:
        T = buf.lookup_transform('base_link', d.header.frame_id, Time.from_msg(d.header.stamp))
    except Exception:
        continue
    dep = np.frombuffer(bytes(d.data), np.uint16).reshape(d.height, d.width) * 0.001
    v, u = np.mgrid[2 * d.height // 3:d.height:4, 0:d.width:4]
    z = dep[v, u]; ok = (z > 0.15) & (z < 2.0)
    u, v, z = u[ok], v[ok], z[ok]
    P = np.stack([(u - k.k[2]) / k.k[0] * z, (v - k.k[5]) / k.k[4] * z, z], 1)
    q = T.transform.rotation; tr = T.transform.translation
    R = np.array([[1 - 2 * (q.y**2 + q.z**2), 2 * (q.x * q.y - q.z * q.w), 2 * (q.x * q.z + q.y * q.w)],
                  [2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.x**2 + q.z**2), 2 * (q.y * q.z - q.x * q.w)],
                  [2 * (q.x * q.z - q.y * q.w), 2 * (q.y * q.z + q.x * q.w), 1 - 2 * (q.x**2 + q.y**2)]])
    B = P @ R.T + np.array([tr.x, tr.y, tr.z])
    A = np.c_[B[:, 0], B[:, 1], np.ones(len(B))]
    keep = np.ones(len(B), bool)
    for _ in range(3):
        coef, *_ = np.linalg.lstsq(A[keep], B[keep, 2], rcond=None)
        r = np.abs(A @ coef - B[:, 2]); keep = r < max(0.01, 3 * np.median(r[keep]))
    out.append(coef)
    if len(out) >= 20: break
a, b, c = np.median(np.array(out), 0)
print(f'{len(out)} 장: 바닥 기울기 앞뒤 {math.degrees(math.atan(a)):+.1f}° (x 로 갈수록 바닥이 {"높아짐" if a > 0 else "낮아짐"}) · 좌우 {math.degrees(math.atan(b)):+.1f}°, '
      f'base_link 아래 바닥 높이 {c:+.3f} m, x 0.42 m 바닥 {a * 0.42 + c:+.3f} m')
