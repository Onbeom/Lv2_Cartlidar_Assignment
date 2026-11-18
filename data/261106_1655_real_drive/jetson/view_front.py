#!/usr/bin/env python3
"""앞 D435 컬러 영상을 Jetson 모니터 창에 띄우고, 인지 노드의 신호등 판단(/perception/signal)을 글자로 얹는다. 읽기만 한다.

DISPLAY=:1 python3 view_front.py   (창에서 q 또는 Esc 로 끝)
"""
import cv2
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from robot_interfaces.msg import SignalSequence as S
from sensor_msgs.msg import Image

ST = {0: 'SEARCHING', 1: 'COUNTING', 2: 'CANDIDATE', 3: 'CONFIRMED', 4: 'MISMATCH'}
LT = {0: 'OFF', 1: 'RED', 2: 'YELLOW', 3: 'GREEN'}
COL = {0: (200, 200, 200), 1: (0, 0, 255), 2: (0, 220, 255), 3: (0, 200, 0)}
rclpy.init()
n = rclpy.create_node('view_front')
st = {'img': None, 'sig': None}


def on_img(m):
    a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1)
    st['img'] = cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else a.copy()


n.create_subscription(Image, '/top_camera_front/color/image_raw', on_img, qos_profile_sensor_data)
n.create_subscription(S, '/perception/signal', lambda m: st.update(sig=m), QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
cv2.namedWindow('front D435', cv2.WINDOW_NORMAL)
cv2.resizeWindow('front D435', 960, 720)
while rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.02)
    if st['img'] is None:
        continue
    img = st['img'].copy()
    s = st['sig']
    if s is not None:
        lt = s.light if s.light_visible else 0
        cv2.rectangle(img, (0, 0), (img.shape[1], 64), (0, 0, 0), -1)
        cv2.putText(img, f"{ST.get(s.status, s.status)}  light {LT.get(lt, lt) if s.light_visible else '-'}  seq {s.sequence or '-'}",
                    (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, COL.get(lt, (255, 255, 255)), 2)
        cv2.putText(img, f"R{s.red} Y{s.yellow} G{s.green}  cycles {s.cycles}  on {s.t_on:.2f}s off {s.t_off:.2f}s term {s.t_term:.2f}s",
                    (8, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    cv2.imshow('front D435', img)
    if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
        break
cv2.destroyAllWindows()
