"""브리지 검증 관찰 노드 (시스템 ROS 2 Lyrical의 rclpy로 실행).

Isaac Sim이 내보내는 토픽을 받아 메시지마다 한 줄씩 rx.csv에 쓰고, 형식별 첫 메시지 전체(영상 데이터 제외)를
first_msgs.json에 남긴다. /cmd_vel에는 정해진 값을 10 Hz로 보낸다. 판정은 analyze_bridge.py가 한다.
"""

import argparse
import csv
import json
import math
import os
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_msgs.msg import TFMessage

# 이진수로 정확히 표현되는 값만 써서, 받은 쪽과 정확히 같아야 한다
TWIST_LIN = (0.25, -0.125, 0.0625)
TWIST_ANG = (0.5, -0.75, 1.0)

TOPICS = [
    ("/clock", Clock), ("/tf", TFMessage), ("/odom", Odometry), ("/scan", LaserScan),
    ("/left/image_raw", Image), ("/right/image_raw", Image),
    ("/left/camera_info", CameraInfo), ("/right/camera_info", CameraInfo),
]


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def front_range(msg):
    """각도 0(정면) 주변 5개 빔의 중앙값. 유효한 값이 없으면 nan"""
    if msg.angle_increment == 0 or not msg.ranges:
        return float("nan")
    i0 = int(round((0.0 - msg.angle_min) / msg.angle_increment))
    vals = [r for r in msg.ranges[max(0, i0 - 2):i0 + 3] if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
    return sorted(vals)[len(vals) // 2] if vals else float("nan")


class Observer(Node):
    def __init__(self, out_dir, reliability):
        super().__init__("bridge_check_observer")
        self.f = open(os.path.join(out_dir, "rx.csv"), "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["topic", "rx_wall_s", "stamp_s", "frame_id", "f1", "f2", "f3", "f4", "f5", "f6", "f7", "f8"])
        self.first = {}
        self.first_path = os.path.join(out_dir, "first_msgs.json")
        self.subs_created_at = {}
        qos = QoSProfile(depth=50, reliability=reliability)
        for name, typ in TOPICS:
            self.create_subscription(typ, name, lambda m, n=name: self.on_msg(n, m), qos)
            self.subs_created_at[name] = time.time()
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(0.1, self.send_twist)
        self.tx_count = 0

    def send_twist(self):
        t = Twist()
        t.linear.x, t.linear.y, t.linear.z = TWIST_LIN
        t.angular.x, t.angular.y, t.angular.z = TWIST_ANG
        self.pub.publish(t)
        self.tx_count += 1

    def keep_first(self, name, d):
        if name not in self.first:
            self.first[name] = d
            with open(self.first_path, "w") as f:
                json.dump(self.first, f, indent=2)

    def on_msg(self, name, m):
        rx = time.time()
        if isinstance(m, Clock):
            row = [rx, m.clock.sec + m.clock.nanosec * 1e-9, ""]
            self.keep_first(name, {"clock": [m.clock.sec, m.clock.nanosec]})
        elif isinstance(m, TFMessage):
            for t in m.transforms:
                tr, q = t.transform.translation, t.transform.rotation
                self.w.writerow([name, f"{rx:.6f}", f"{stamp_s(t.header):.9f}", t.header.frame_id,
                                 t.child_frame_id, tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w])
            self.keep_first(name, [{"parent": t.header.frame_id, "child": t.child_frame_id} for t in m.transforms])
            return
        elif isinstance(m, Odometry):
            p, q, v, a = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear, m.twist.twist.angular
            row = [rx, stamp_s(m.header), m.header.frame_id, m.child_frame_id, p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            self.keep_first(name, {"frame_id": m.header.frame_id, "child_frame_id": m.child_frame_id,
                                   "position": [p.x, p.y, p.z], "orientation": [q.x, q.y, q.z, q.w],
                                   "linear": [v.x, v.y, v.z], "angular": [a.x, a.y, a.z],
                                   "pose_cov_len": len(m.pose.covariance), "twist_cov_len": len(m.twist.covariance)})
        elif isinstance(m, LaserScan):
            row = [rx, stamp_s(m.header), m.header.frame_id, m.angle_min, m.angle_max, m.angle_increment,
                   len(m.ranges), front_range(m), m.range_min, m.range_max]
            self.keep_first(name, {"frame_id": m.header.frame_id, "angle_min": m.angle_min,
                                   "angle_max": m.angle_max, "angle_increment": m.angle_increment,
                                   "range_min": m.range_min, "range_max": m.range_max,
                                   "n_ranges": len(m.ranges), "front_range": front_range(m)})
        elif isinstance(m, Image):
            row = [rx, stamp_s(m.header), m.header.frame_id, m.width, m.height, m.encoding, m.step, len(m.data)]
            self.keep_first(name, {"frame_id": m.header.frame_id, "width": m.width, "height": m.height,
                                   "encoding": m.encoding, "step": m.step, "is_bigendian": m.is_bigendian,
                                   "data_len": len(m.data)})
        elif isinstance(m, CameraInfo):
            row = [rx, stamp_s(m.header), m.header.frame_id, m.width, m.height, m.k[0], m.k[2], m.k[4], m.k[5],
                   m.p[3]]
            self.keep_first(name, {"frame_id": m.header.frame_id, "width": m.width, "height": m.height,
                                   "distortion_model": m.distortion_model, "d": list(m.d), "k": list(m.k),
                                   "r": list(m.r), "p": list(m.p)})
        else:
            return
        rx_s, st = row[0], row[1]
        self.w.writerow([name, f"{rx_s:.6f}", f"{st:.9f}"] + row[2:])

    def close(self, out_dir):
        self.f.close()
        with open(os.path.join(out_dir, "observer_info.json"), "w") as f:
            json.dump({"subs_created_at": self.subs_created_at, "twist_sent": self.tx_count,
                       "twist_lin": TWIST_LIN, "twist_ang": TWIST_ANG,
                       "publishers": {n: [{"node": i.node_name, "reliability": str(i.qos_profile.reliability),
                                           "durability": str(i.qos_profile.durability),
                                           "depth": i.qos_profile.depth}
                                          for i in self.get_publishers_info_by_topic(n)] for n, _ in TOPICS}},
                      f, indent=2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration", type=float, default=1e9, help="[s] 이 시간이 지나면 끝낸다 (SIGINT로도 끝남)")
    ap.add_argument("--reliability", choices=["reliable", "best_effort"], default="reliable",
                    help="구독 QoS. Isaac Sim 발행자가 BEST_EFFORT면 reliable 구독은 아무것도 못 받는다")
    a = ap.parse_args()
    rclpy.init()
    rel = ReliabilityPolicy.RELIABLE if a.reliability == "reliable" else ReliabilityPolicy.BEST_EFFORT
    node = Observer(a.out, rel)
    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))
    signal.signal(signal.SIGTERM, lambda *_: stop.update(flag=True))
    t_end = time.time() + a.duration
    last_info = 0.0
    while not stop["flag"] and time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() - last_info > 10.0:      # 끝나기 전에 Isaac Sim이 꺼져도 발행자 정보가 남게 주기적으로 쓴다
            node.f.flush()
            if any(node.get_publishers_info_by_topic(n) for n, _ in TOPICS):
                with open(os.path.join(a.out, "observer_pubinfo.json"), "w") as f:
                    json.dump({n: [{"node": i.node_name, "reliability": str(i.qos_profile.reliability),
                                    "durability": str(i.qos_profile.durability), "depth": i.qos_profile.depth}
                                   for i in node.get_publishers_info_by_topic(n)] for n, _ in TOPICS}, f, indent=2)
            last_info = time.time()
    node.close(a.out)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
