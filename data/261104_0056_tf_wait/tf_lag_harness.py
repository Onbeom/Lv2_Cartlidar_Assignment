"""앞 D435 촬영 시각 TF 기다림 확인 (PC, run_replay.sh 가 rsp · 인지 노드를 띄운 뒤 부른다).

모의 시각을 벽시계와 같은 빠르기로 내며(/clock 500 Hz):
  - 앞 D435 렌더 한 장(컬러 · 깊이 · camera_info)을 30 Hz 로 낸다. stamp = 그 모의 시각. --img-phase 로 처리 주기와 위상을 어긋나게
  - odom → 뿌리 TF 를 50 Hz 로 내되, stamp 보다 --tf-lag 만큼 늦게 낸다(EKF 꼴: 영상 받은 순간 최신 TF 가 5~25 ms 뒤짐)
  - 차체는 제자리에서 --omega rad/s 로 돈다(TF 만. 영상은 그대로라 물리 장면은 아니고, 시각 맞춤만 본다)
  - --tf-stop 초 뒤에는 TF 를 내지 않는다(TF 멈춤에도 영상 처리가 이어지는지)
기록: /perception/pucks 받은 모의 시각 · stamp → <out>/pucks.jsonl, 영상 낸 순간 최신 TF stamp → <out>/img_lag.jsonl
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path.home() / "git/lv2_next_module/Cartlidar/sim_render"))

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster

from robot_interfaces.msg import PuckArray
from robot_interfaces.srv import SetPerceptionMode

from replay_ros import odom_child, quat_from_matrix, stamp_s, to_time


class Harness(Node):
    def __init__(self, a):
        super().__init__("tf_lag_harness")
        self.a = a
        rel = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 100)
        self.img_pub = self.create_publisher(Image, "/top_camera_front/color/image_raw", rel)
        self.info_pub = self.create_publisher(CameraInfo, "/top_camera_front/color/camera_info", rel)
        self.dep_pub = self.create_publisher(Image, "/top_camera_front/aligned_depth_to_color/image_raw", rel)
        self.create_subscription(PuckArray, "/perception/pucks", self.on_pucks, QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
        self.mode_cli = self.create_client(SetPerceptionMode, "/perception/set_mode")
        st = TransformStamped()
        st.header.frame_id, st.child_frame_id = "map", "odom"
        st.transform.rotation.w = 1.0
        StaticTransformBroadcaster(self).sendTransform([st])
        meta = json.loads((a.render / "render_b0.json").read_text())
        pose = [p for p in meta["poses"] if p["kind"] == "d435"][a.pose_index]
        self.pose_id = pose["id"]
        self.cam = meta["d435"]
        self.W, self.H = meta["width"], meta["height"]
        self.bgr = cv2.imread(str(a.render / pose["images"]["color"]), cv2.IMREAD_COLOR)
        dep = np.load(a.render / pose["images"]["depth"]).astype(np.float32)
        dep[~np.isfinite(dep)] = 0.0
        self.dep = dep
        self.child, T_child_bf = odom_child()
        self.T0 = np.array(pose["T_world_base_footprint"]) @ np.linalg.inv(T_child_bf)
        self.now = 0.0
        self.tf_latest = None
        self.pucks = []
        self.lags = []

    def on_pucks(self, m):
        self.pucks.append({"recv": round(self.now, 6), "stamp": round(stamp_s(m.header), 6), "n": len(m.pucks)})

    def set_clock(self, t):
        self.now = t
        c = Clock()
        c.clock = to_time(t)
        self.clock_pub.publish(c)

    def send_tf(self, t):
        yaw = self.a.omega * t
        R = np.array([[math.cos(yaw), -math.sin(yaw), 0, 0], [math.sin(yaw), math.cos(yaw), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        T = self.T0 @ R
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = to_time(t), "odom", self.child
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, T[:3, 3])
        q = quat_from_matrix(T[:3, :3])
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = map(float, q)
        self.tf_pub.publish(TFMessage(transforms=[tf]))
        self.tf_latest = t

    def send_image(self, t):
        st = to_time(t)
        fr = self.cam["frame"]
        K = self.cam["K"]
        info = CameraInfo()
        info.header.stamp, info.header.frame_id = st, fr
        info.width, info.height = self.W, self.H
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [K["fx"], 0.0, K["cx"], 0.0, K["fy"], K["cy"], 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [K["fx"], 0.0, K["cx"], 0.0, 0.0, K["fy"], K["cy"], 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)
        d = Image()
        d.header.stamp, d.header.frame_id = st, fr
        d.height, d.width = self.dep.shape
        d.encoding, d.step = "32FC1", self.dep.shape[1] * 4
        d.data = self.dep.tobytes()
        self.dep_pub.publish(d)
        c = Image()
        c.header.stamp, c.header.frame_id = st, fr
        c.height, c.width = self.bgr.shape[:2]
        c.encoding, c.step = "bgr8", self.bgr.shape[1] * 3
        c.data = self.bgr.tobytes()
        self.img_pub.publish(c)
        self.lags.append({"t": round(t, 6), "tf_latest": None if self.tf_latest is None else round(self.tf_latest, 6)})

    def set_mode(self, mode):
        if not self.mode_cli.wait_for_service(timeout_sec=20.0):
            raise RuntimeError("/perception/set_mode 없음")
        req = SetPerceptionMode.Request()
        req.mode = mode
        fut = self.mode_cli.call_async(req)
        while not fut.done():
            rclpy.spin_once(self, timeout_sec=0.01)
        return fut.result()

    def run(self):
        a = self.a
        t0 = 100.0                     # 모의 시각 시작(0 근처 stamp 피함, 노드 30 s 묶음 로그가 첫 장부터 찍히게)
        dt = 0.002
        next_img, next_tf = t0 + a.img_phase, t0
        pending_tf = []                # (낼 시각, stamp)
        start = time.monotonic()
        self.set_clock(t0)
        self.send_tf(t0)
        r = self.set_mode(SetPerceptionMode.Request.SEARCH_FRONT)
        self.get_logger().info(f"set_mode SEARCH_FRONT: {r.success} {r.message}")
        start = time.monotonic()
        while True:
            t = t0 + (time.monotonic() - start)
            if t - t0 > a.duration:
                break
            self.set_clock(t)
            while next_tf <= t:
                if a.tf_stop is None or next_tf - t0 < a.tf_stop:
                    pending_tf.append((next_tf + a.tf_lag, next_tf))
                next_tf += 1.0 / a.tf_hz
            while pending_tf and pending_tf[0][0] <= t:
                self.send_tf(pending_tf.pop(0)[1])
            while next_img <= t:
                self.send_image(next_img)
                next_img += 1.0 / a.fps
            rclpy.spin_once(self, timeout_sec=0.0)
            rem = dt - ((time.monotonic() - start) % dt)
            time.sleep(rem)
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.01)
        a.out.mkdir(parents=True, exist_ok=True)
        (a.out / "pucks.jsonl").write_text("".join(json.dumps(p) + "\n" for p in self.pucks))
        (a.out / "img_lag.jsonl").write_text("".join(json.dumps(p) + "\n" for p in self.lags))
        (a.out / "harness.json").write_text(json.dumps({"pose": self.pose_id, "t0": t0, **{k: (str(v) if isinstance(v, Path) else v) for k, v in vars(a).items()}}, indent=1))
        self.get_logger().info(f"끝: 영상 {len(self.lags)} · pucks {len(self.pucks)}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--render", type=Path, required=True)
    ap.add_argument("--pose-index", type=int, default=0, help="d435 자세 중 몇 번째")
    ap.add_argument("--duration", type=float, default=40.0, help="[s] 모의 시간")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--tf-hz", type=float, default=50.0)
    ap.add_argument("--tf-lag", type=float, default=0.005, help="[s] TF 를 stamp 보다 늦게 내는 시간(50 Hz 위상 0~20 ms 에 더해짐)")
    ap.add_argument("--img-phase", type=float, default=0.011, help="[s] 영상 시각을 처리 주기(15 Hz) · TF 주기와 어긋나게(실물 · Isaac 은 위상이 제각각)")
    ap.add_argument("--omega", type=float, default=0.5, help="[rad/s] 제자리 회전")
    ap.add_argument("--tf-stop", type=float, default=None, help="[s] 이 모의 시간 뒤 TF 를 내지 않음")
    a = ap.parse_args()
    rclpy.init()
    n = Harness(a)
    try:
        n.run()
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
