"""적외선 영상 아래쪽 가림 중계 (cuVSLAM 변형 V1 · V2, 모의 재생 전용).

cuVSLAM 5.0 의 img_mask_bottom 은 값이 들어가도(ros2 param get 220 확인) 결과가 V0 과 똑같았다(짧은 기록 도구 확인 2026-10-27 21:2x).
그래서 bag 재생 영상을 받아 아래 MASK_BOTTOM_PX 행을 0 으로 칠해 원래 토픽 이름으로 다시 낸다(header · CameraInfo 는 그대로).
가린 경계(가로 줄)는 카메라와 같이 움직이는 모서리가 되지만, 가로 한 줄이라 모서리 특징점은 화면 양 끝에만 생긴다.
"""
import os

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

ROWS = int(os.environ.get("MASK_BOTTOM_PX", "220"))


def main():
    rclpy.init()
    node = rclpy.create_node("ir_mask_relay")
    pubs = {}
    for c in ("infra1", "infra2"):
        pubs[c] = node.create_publisher(Image, f"/top_camera_front/{c}/image_rect_raw", qos_profile_sensor_data)

        def cb(m, c=c):
            a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step).copy()
            a[max(0, m.height - ROWS):, :] = 0
            m.data = a.tobytes()
            pubs[c].publish(m)

        node.create_subscription(Image, f"/unmasked/top_camera_front/{c}/image_rect_raw", cb, qos_profile_sensor_data)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
