#!/usr/bin/env python3
"""앞 D435 깊이 N 장을 npz 로 저장 (읽기만, 창 없음). depth_capture.py <폴더> [장 수=15] [카메라 이름=top_camera_front]
깊이 원본(/<이름>/depth/image_rect_raw, 16UC1 mm) · 그 camera_info · <이름>_link ← 깊이 광학 프레임 TF 를 저장한다.
PC 에서 책상 · 바닥 평면을 맞춰 카메라 피치 · 롤을 컬러 태그와 따로 잰다(교차 확인)."""
import json, os, sys, time
import numpy as np, rclpy, tf2_ros
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo

out = sys.argv[1]; N = int(sys.argv[2]) if len(sys.argv) > 2 else 15
cam = sys.argv[3] if len(sys.argv) > 3 else 'top_camera_front'
os.makedirs(out, exist_ok=True)
rclpy.init(); n = rclpy.create_node('depth_capture')
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
st = {'d': [], 'k': None, 'fid': None, 'i': 0}

def on_d(m):
    st['i'] += 1
    if st['i'] % 2 or len(st['d']) >= N:
        return
    st['d'].append(np.frombuffer(bytes(m.data), np.uint16).reshape(m.height, m.width).copy()); st['fid'] = m.header.frame_id

n.create_subscription(CameraInfo, f'/{cam}/depth/camera_info', lambda m: st.__setitem__('k', m), qos_profile_sensor_data)
n.create_subscription(Image, f'/{cam}/depth/image_rect_raw', on_d, qos_profile_sensor_data)
t0 = time.time()
while len(st['d']) < N and time.time() - t0 < 30:
    rclpy.spin_once(n, timeout_sec=0.05)
tf = None
for _ in range(40):
    rclpy.spin_once(n, timeout_sec=0.05)
    try:
        t = buf.lookup_transform(f'{cam}_link', st['fid'] or f'{cam}_depth_optical_frame', Time()).transform
        tf = {'xyz': [t.translation.x, t.translation.y, t.translation.z], 'xyzw': [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]}
        break
    except Exception:
        pass
k = st['k']
np.savez_compressed(os.path.join(out, 'depth.npz'), depth=np.array(st['d']))
json.dump({'frame_id': st['fid'], 'n': len(st['d']), 'K': list(k.k) if k else None, 'D': list(k.d) if k else None,
           'width': k.width if k else None, 'height': k.height if k else None, 'T_link_optical': tf,
           'time': time.strftime('%Y-%m-%d %H:%M:%S')}, open(os.path.join(out, 'meta.json'), 'w'), indent=1)
print('depth saved', len(st['d']), 'K', 'ok' if k else 'NONE', 'TF', 'ok' if tf else 'NONE', 'frame', st['fid'])
