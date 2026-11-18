"""R2: 최소 Python 수신기 (시스템 Lyrical rclpy, best effort). 영상 5개의 stamp와 받은 시각만 rx_min.csv에 쓴다.

사용: python3 rx_min.py --out <폴더>   (SIGINT로 끝냄)
"""

import argparse
import csv
import os
import signal
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

TOPICS = ["/top_camera_front/color/image_raw", "/top_camera_rear/color/image_raw", "/arm_camera/image_raw",
          "/top_camera_front/aligned_depth_to_color/image_raw", "/top_camera_rear/aligned_depth_to_color/image_raw"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node("diag_rx_min")
    f = open(os.path.join(a.out, "rx_min.csv"), "w", newline="", buffering=1)
    w = csv.writer(f)
    w.writerow(["topic", "rx_wall_s", "stamp_s"])
    for t in TOPICS:
        node.create_subscription(Image, t, lambda m, t=t: w.writerow(
            [t, f"{time.time():.6f}", f"{m.header.stamp.sec + m.header.stamp.nanosec * 1e-9:.9f}"]),
            qos_profile_sensor_data)
    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))
    signal.signal(signal.SIGTERM, lambda *_: stop.update(flag=True))
    while not stop["flag"]:
        rclpy.spin_once(node, timeout_sec=0.05)
    f.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
