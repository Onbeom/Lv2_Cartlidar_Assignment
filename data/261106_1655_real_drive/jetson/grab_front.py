#!/usr/bin/env python3
"""D435 컬러 한 장을 JPEG 로 저장 (읽기만, 창 없음). grab_front.py <파일> [카메라 이름=top_camera_front]"""
import sys
import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
rclpy.init()
n = rclpy.create_node('grab_front')
got = {}
def cb(m):
    a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1)
    got['img'] = cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else a.copy()
n.create_subscription(Image, f'/{sys.argv[2] if len(sys.argv) > 2 else "top_camera_front"}/color/image_raw', cb, qos_profile_sensor_data)
while 'img' not in got:
    rclpy.spin_once(n, timeout_sec=0.1)
cv2.imwrite(sys.argv[1], got['img'])
print('saved', got['img'].shape)
