"""L2 관찰·명령 노드 (시스템 ROS 2 Lyrical rclpy, use_sim_time=true).

받은 메시지를 rx.csv에 한 줄씩 쓰고(TF는 (부모, 자식)별 개수와 odom→base_footprint만),
시뮬레이션 6 s 뒤 처음 받은 영상·깊이·스캔과 base_footprint→센서 frame 변환을 snapshot/에 저장한다.
10 s마다 odom → 인지가 쓰는 frame TF 조회 결과를 tf_lookup.csv에 쓴다.
/cmd_vel(TwistStamped)에 시험 명령을 시뮬레이션 시간표대로 20 Hz로 보낸다(cmd_tx.csv). 판정은 analyze_l2.py.
"""

import argparse
import csv
import json
import os
import signal
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

CAMS = {"front": "/top_camera_front", "rear": "/top_camera_rear", "arm": "/arm_camera"}
TOPICS = {
    "/clock": Clock, "/odom": Odometry, "/joint_states": JointState,
    "/scan_front": LaserScan, "/scan_rear": LaserScan,
    "/top_camera_front/color/image_raw": Image, "/top_camera_rear/color/image_raw": Image,
    "/arm_camera/image_raw": Image,
    "/top_camera_front/aligned_depth_to_color/image_raw": Image,
    "/top_camera_rear/aligned_depth_to_color/image_raw": Image,
    "/top_camera_front/color/camera_info": CameraInfo, "/top_camera_rear/color/camera_info": CameraInfo,
    "/arm_camera/camera_info": CameraInfo,
}
LOOKUP_FRAMES = ["base_link", "top_camera_front_color_optical_frame", "top_camera_rear_color_optical_frame",
                 "camera_optical_frame", "lidar_front_scan", "lidar_rear_scan"]
SNAP_T = 6.0          # [s] 정지 검사 snapshot 시각(팔을 S0로 접은 뒤)
# 명령 시간표 (시뮬레이션 s): (시작, 끝, v, ω). 25 s 뒤로는 보내지 않는다(끊김 → 스스로 정지 확인)
SCHEDULE = [(10.0, 15.0, 0.10, 0.0), (15.0, 17.0, 0.0, 0.0), (17.0, 21.0, 0.0, 0.5),
            (21.0, 23.0, 0.0, 0.0), (23.0, 25.0, 0.10, 0.0)]
CMD_FRAME = "base_footprint"


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class Observer(Node):
    def __init__(self, out):
        super().__init__("l2_observer", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.out = out
        os.makedirs(os.path.join(out, "snapshot"), exist_ok=True)
        self.f = open(os.path.join(out, "rx.csv"), "w", newline="", buffering=1)
        self.w = csv.writer(self.f)
        self.w.writerow(["topic", "rx_wall_s", "rx_clock_s", "stamp_s", "frame_id", "extra"])
        self.lk_f = open(os.path.join(out, "tf_lookup.csv"), "w", newline="", buffering=1)
        self.lk = csv.writer(self.lk_f)
        self.lk.writerow(["clock_s", "target", "source", "ok", "x", "y", "z", "error"])
        self.tx_f = open(os.path.join(out, "cmd_tx.csv"), "w", newline="", buffering=1)
        self.tx = csv.writer(self.tx_f)
        self.tx.writerow(["clock_s", "v", "w"])
        self.tf_pairs = {}
        self.tf_pubs = {"/tf": set(), "/tf_static": set()}   # 실행 중 본 발행 노드를 모두 모은다
        self.snap = {}
        self.clock_s = 0.0
        self.next_lookup = 10.0
        for name, typ in TOPICS.items():
            qos = qos_profile_sensor_data if typ in (Image, LaserScan) else QoSProfile(depth=50)
            self.create_subscription(typ, name, lambda m, n=name: self.on_msg(n, m), qos)
        self.create_subscription(TFMessage, "/tf", lambda m: self.on_tf(m, "/tf"), QoSProfile(depth=200))
        self.create_subscription(TFMessage, "/tf_static", lambda m: self.on_tf(m, "/tf_static"),
                                 QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.buf = Buffer(cache_time=Duration(seconds=30))
        self.tfl = TransformListener(self.buf, self)
        self.pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.create_timer(0.05, self.tick)

    def save_once(self, key, fn):
        if key in self.snap or self.clock_s < SNAP_T:
            return
        self.snap[key] = fn()

    def on_msg(self, name, m):
        rx = time.time()
        if isinstance(m, Clock):
            self.clock_s = m.clock.sec + m.clock.nanosec * 1e-9
            self.w.writerow([name, f"{rx:.6f}", f"{self.clock_s:.6f}", f"{self.clock_s:.9f}", "", ""])
            return
        st, fid, extra = stamp_s(m.header), m.header.frame_id, ""
        if isinstance(m, Image):
            extra = json.dumps([m.width, m.height, m.encoding, m.step, len(m.data)])

            def save():
                dt = np.float32 if m.encoding == "32FC1" else np.uint8
                ch = 1 if m.encoding == "32FC1" else 3
                arr = np.frombuffer(bytes(m.data), dtype=dt).reshape(m.height, m.width, ch)
                fn = name.strip("/").replace("/", "__") + ".npy"
                np.save(os.path.join(self.out, "snapshot", fn), arr)
                return {"file": fn, "stamp": st}
            self.save_once(name, save)
        elif isinstance(m, CameraInfo):
            extra = json.dumps({"w": m.width, "h": m.height, "model": m.distortion_model, "d": list(m.d),
                                "k": list(m.k), "r": list(m.r), "p": list(m.p)})
        elif isinstance(m, LaserScan):
            extra = json.dumps([m.angle_min, m.angle_max, m.angle_increment, m.range_min, m.range_max, len(m.ranges)])
            self.save_once(name, lambda: {"stamp": st, "angle_min": m.angle_min, "angle_increment": m.angle_increment,
                                          "ranges": [r if np.isfinite(r) else None for r in m.ranges]})
        elif isinstance(m, JointState):
            extra = json.dumps({"name": list(m.name), "position": list(m.position)})
        elif isinstance(m, Odometry):
            p, q = m.pose.pose.position, m.pose.pose.orientation
            extra = json.dumps([m.child_frame_id, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        self.w.writerow([name, f"{rx:.6f}", f"{self.clock_s:.6f}", f"{st:.9f}", fid, extra])

    def on_tf(self, m, topic):
        rx = time.time()
        for t in m.transforms:
            key = (topic, t.header.frame_id, t.child_frame_id)
            self.tf_pairs[key] = self.tf_pairs.get(key, 0) + 1
            if t.child_frame_id == "base_footprint":
                tr = t.transform.translation
                self.w.writerow([topic, f"{rx:.6f}", f"{self.clock_s:.6f}", f"{stamp_s(t.header):.9f}",
                                 t.header.frame_id, json.dumps([t.child_frame_id, tr.x, tr.y, tr.z])])

    def lookup(self, target, source):
        try:
            tf = self.buf.lookup_transform(target, source, Time())
            tr, q = tf.transform.translation, tf.transform.rotation
            return True, [tr.x, tr.y, tr.z], [q.x, q.y, q.z, q.w], ""
        except Exception as e:  # tf2 예외 종류가 여럿이라 모두 기록
            return False, None, None, type(e).__name__ + ": " + str(e)[:120]

    def tick(self):
        t = self.clock_s
        seg = next((s for s in SCHEDULE if s[0] <= t < s[1]), None)
        if seg is not None:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = CMD_FRAME
            msg.twist.linear.x, msg.twist.angular.z = seg[2], seg[3]
            self.pub.publish(msg)
            self.tx.writerow([f"{t:.6f}", repr(seg[2]), repr(seg[3])])
        if t >= self.next_lookup:
            self.next_lookup += 10.0
            for fr in LOOKUP_FRAMES:
                ok, p, _, err = self.lookup("odom", fr)
                self.lk.writerow([f"{t:.6f}", "odom", fr, int(ok)] + (p if ok else ["", "", ""]) + [err])
        if t >= SNAP_T and "tf_base" not in self.snap:
            res = {fr: self.lookup("base_footprint", fr) for fr in LOOKUP_FRAMES}
            if all(r[0] for r in res.values()):
                self.snap["tf_base"] = {"clock_s": t, "frames": {fr: {"t": r[1], "q_xyzw": r[2]} for fr, r in res.items()}}

    def write_info(self):
        for tp in self.tf_pubs:
            self.tf_pubs[tp].update(i.node_name for i in self.get_publishers_info_by_topic(tp))
        pubs = {tp: sorted(v) for tp, v in self.tf_pubs.items()}
        with open(os.path.join(self.out, "observer_info.json"), "w") as f:
            json.dump({"tf_pairs": [[k[0], k[1], k[2], v] for k, v in sorted(self.tf_pairs.items())],
                       "tf_publishers": pubs, "snapshot": self.snap, "schedule": SCHEDULE, "cmd_frame": CMD_FRAME,
                       "use_sim_time": self.get_parameter("use_sim_time").value}, f, indent=2)

    def close(self):
        self.write_info()
        for fh in (self.f, self.lk_f, self.tx_f):
            fh.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    rclpy.init()
    node = Observer(a.out)
    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))
    signal.signal(signal.SIGTERM, lambda *_: stop.update(flag=True))
    last_info = time.time()
    while not stop["flag"]:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() - last_info > 15.0:      # Isaac이 중간에 꺼져도 기록이 남게 주기적으로 쓴다
            node.write_info()
            last_info = time.time()
    node.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
