#!/usr/bin/env python3
"""앞 D435 컬러를 N 장 PNG 로 저장하고 K · D · 카메라 안쪽 TF 를 meta.json 에 적는다 (읽기만, 창 없음).
cal_capture.py <폴더> [장 수=30] [카메라 이름=top_camera_front]
- 영상 · camera_info 는 realsense2_camera(sensors.launch.py) 가 낸다. 로봇 TF(robot_state_publisher)는 없어도 된다.
- TF 는 <이름>_link ← 컬러 광학 프레임(realsense 드라이버의 공장 외부값)만 찾는다. base_link ← <이름>_link 는 PC 풀이에서 URDF 값을 쓴다.
- 3 장마다 1 장 저장(약 10 Hz). 끝에 저장 장 수 · 평균 밝기를 출력한다."""
import json, os, sys, time
import cv2, numpy as np, rclpy, tf2_ros
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo

out = sys.argv[1]; N = int(sys.argv[2]) if len(sys.argv) > 2 else 30
cam = sys.argv[3] if len(sys.argv) > 3 else 'top_camera_front'
os.makedirs(out, exist_ok=True)
rclpy.init(); n = rclpy.create_node('cal_capture')
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
st = {'i': 0, 'saved': [], 'k': None}

def on_info(m):
    st['k'] = m

def on_img(m):
    st['i'] += 1
    if st['i'] % 3 or len(st['saved']) >= N:
        return
    a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1)
    img = cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else a.copy()
    f = f'{len(st["saved"]):03d}.png'
    cv2.imwrite(os.path.join(out, f), img)
    st['saved'].append({'file': f, 'stamp': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                        'frame_id': m.header.frame_id, 'mean': float(img.mean())})

n.create_subscription(CameraInfo, f'/{cam}/color/camera_info', on_info, qos_profile_sensor_data)
n.create_subscription(Image, f'/{cam}/color/image_raw', on_img, qos_profile_sensor_data)
t0 = time.time()
while len(st["saved"]) < N and time.time() - t0 < max(30, N / 6 + 10):
    rclpy.spin_once(n, timeout_sec=0.05)
fid = st['saved'][0]['frame_id'] if st['saved'] else f'{cam}_color_optical_frame'
tf = None
for _ in range(40):
    rclpy.spin_once(n, timeout_sec=0.05)
    try:
        t = buf.lookup_transform(f'{cam}_link', fid, Time()).transform
        tf = {'xyz': [t.translation.x, t.translation.y, t.translation.z],
              'xyzw': [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]}
        break
    except Exception:
        pass
k = st['k']
meta = {'camera': cam, 'frame_id': fid, 'n': len(st['saved']), 'frames': st['saved'],
        'K': list(k.k) if k else None, 'D': list(k.d) if k else None,
        'width': k.width if k else None, 'height': k.height if k else None,
        'distortion_model': k.distortion_model if k else None,
        'T_link_optical': tf, 'time': time.strftime('%Y-%m-%d %H:%M:%S')}
json.dump(meta, open(os.path.join(out, 'meta.json'), 'w'), indent=1)
print('saved', len(st['saved']), 'K', 'ok' if k else 'NONE', 'TF', 'ok' if tf else 'NONE',
      'mean', round(np.mean([s['mean'] for s in st['saved']]), 1) if st['saved'] else '-')
