"""가짜 팔 카메라 (팀 저장소 밖, 사진 저장 모의 시험용).

/arm_camera/image_raw (bgr8 640x360, --hz) 와 camera_info 를 내고, 시작 때 진짜 perception_node 의 set_mode
(--mode_srv, 시험에서는 mock_robot 과 겹치지 않게 remap 한 이름) 에 READ_TAG 를 한 번 보내 팔 카메라 모드를 켠다.
영상에는 프레임 번호와 벽시계를 글자로 넣어, 저장된 세 장이 서로 다른 영상인지 눈으로도 볼 수 있게 한다.
  --no_mode   set_mode 를 보내지 않음 (팔 카메라가 꺼진 모드에서 요청 → 경고만 확인)
  --hz 0      영상을 내지 않음 (새 영상이 안 와 포기 확인)
"""
import argparse
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from robot_interfaces.srv import SetPerceptionMode
from sensor_msgs.msg import CameraInfo, Image


class ArmCam(Node):
    def __init__(self, a):
        super().__init__("arm_cam_mock")
        self.a = a
        self.n = 0
        self.img_pub = self.create_publisher(Image, "/arm_camera/image_raw", 2)
        self.info_pub = self.create_publisher(CameraInfo, "/arm_camera/camera_info", 2)
        if a.hz > 0:
            self.create_timer(1.0 / a.hz, self.tick)
        self.mode_sent = a.no_mode
        self.cli = self.create_client(SetPerceptionMode, a.mode_srv)
        self.create_timer(0.5, self.send_mode)

    def send_mode(self):
        if self.mode_sent or not self.cli.service_is_ready():
            return
        req = SetPerceptionMode.Request()
        req.mode = SetPerceptionMode.Request.READ_TAG
        self.cli.call_async(req).add_done_callback(lambda f: self.get_logger().info(f"set_mode READ_TAG: {f.result().message}"))
        self.mode_sent = True

    def tick(self):
        self.n += 1
        img = np.full((360, 640, 3), (40, 90, 40), np.uint8)
        cv2.putText(img, f"frame {self.n}", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (255, 255, 255), 3)
        cv2.putText(img, time.strftime("%H:%M:%S") + f".{int(time.time() * 1000) % 1000:03d}", (20, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 2.0, (255, 255, 255), 3)
        m = Image()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "arm_camera_optical_frame"
        m.height, m.width, m.encoding, m.step = 360, 640, "bgr8", 640 * 3
        m.data = img.tobytes()
        self.img_pub.publish(m)
        ci = CameraInfo()
        ci.header = m.header
        ci.width, ci.height = 640, 360
        ci.k = [432.5, 0.0, 320.0, 0.0, 432.5, 180.0, 0.0, 0.0, 1.0]
        ci.p = [432.5, 0.0, 320.0, 0.0, 0.0, 432.5, 180.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ci.distortion_model = "plumb_bob"
        ci.d = [0.0] * 5
        self.info_pub.publish(ci)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hz", type=float, default=15.0)
    ap.add_argument("--mode_srv", default="/real_perception/set_mode")
    ap.add_argument("--no_mode", action="store_true")
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=[sys.argv[0]] + ros_args)
    rclpy.spin(ArmCam(a))


if __name__ == "__main__":
    main()
