"""perception_node 의 /perception/place_box/measure_front 를 앞 D435 렌더 영상(④ judge/)으로 재생 시험 (모의).
노드 하나 띄운 상태에서: 장면마다 정적 TF (odom = base_footprint, base_footprint → top_camera_front_color_optical_frame = 렌더 T_bf_cam),
SEARCH_FRONT 모드, 사전값 /perception/place_box/prior (base_footprint), 서비스 호출 뒤 같은 영상을 새 시각으로 계속 발행 → status · pose 를 참과 비교.
    python3 run_front_replay.py <렌더 폴더> [--every N]
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from robot_interfaces.srv import SetPerceptionMode
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

ap = argparse.ArgumentParser()
ap.add_argument("dir", type=Path)
ap.add_argument("--every", type=int, default=1)
ap.add_argument("--names", type=Path, help="이 파일에 적힌 영상만 (줄마다 하나)")
ap.add_argument("--wait", type=float, default=20.0, help="장마다 결과 기다리는 시간 [s]")
ap.add_argument("--reliable", action="store_true", help="영상 · camera_info 를 reliable 로 보냄 (노드도 image_qos:=reliable)")
ap.add_argument("--tag", default="", help="결과 json 이름 뒤에 붙임")
a = ap.parse_args()
meta = json.loads((a.dir / "render_u3.json").read_text())
K = meta["K"]
W, H = meta["width"], meta["height"]


def rot_to_quat(R):
    q = np.empty(4)
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q[:] = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
        q[i] = 0.25 * s
        q[j] = (R[j, i] + R[i, j]) / s
        q[k] = (R[k, i] + R[i, k]) / s
        q[3] = (R[k, j] - R[j, k]) / s
    return q


class T(Node):
    def __init__(self):
        super().__init__("front_replay")
        iq = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE) if a.reliable else qos_profile_sensor_data
        self.img_pub = self.create_publisher(Image, "/top_camera_front/color/image_raw", iq)
        self.info_pub = self.create_publisher(CameraInfo, "/top_camera_front/color/camera_info", iq)
        lat = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.prior_pub = self.create_publisher(PoseStamped, "/perception/place_box/prior", lat)
        self.st = None
        self.pose = None
        self.create_subscription(String, "/perception/place_box/status", self.on_st, lat)
        self.create_subscription(PoseStamped, "/perception/place_box", lambda m: setattr(self, "pose", m), lat)
        self.tfb = StaticTransformBroadcaster(self)
        self.mode_cli = self.create_client(SetPerceptionMode, "/perception/set_mode")
        self.meas_cli = self.create_client(Trigger, "/perception/place_box/measure_front")

    def on_st(self, m):
        self.st = m


rclpy.init()
n = T()
for c in (n.mode_cli, n.meas_cli):
    if not c.wait_for_service(timeout_sec=20):
        print("서비스 없음", c.srv_name)
        sys.exit(1)
req = SetPerceptionMode.Request()
req.mode = SetPerceptionMode.Request.SEARCH_FRONT
f = n.mode_cli.call_async(req)
rclpy.spin_until_future_complete(n, f, timeout_sec=5)
rows = []
scenes = meta["scenes"][:: a.every]
if a.names:
    want = set(a.names.read_text().split())
    scenes = [sc for sc in meta["scenes"] if sc["image"] in want]
out_json = a.dir.parent / "front_node" / f"replay_{a.dir.name}{a.tag}.json"
for sc in scenes:
    Tm = np.array(sc["T_bf_cam"])
    ts = []
    for parent, child, M in (("odom", "base_footprint", np.eye(4)), ("base_footprint", "top_camera_front_color_optical_frame", Tm)):
        t = TransformStamped()
        t.header.stamp = n.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, M[:3, 3])
        q = rot_to_quat(M[:3, :3])
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = map(float, q)
        ts.append(t)
    n.tfb.sendTransform(ts)
    p = PoseStamped()
    p.header.frame_id = "base_footprint"
    p.header.stamp = n.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y, p.pose.position.z = float(sc["prior"][0]), float(sc["prior"][1]), 0.07
    p.pose.orientation.z, p.pose.orientation.w = math.sin(sc["prior_yaw"] / 2), math.cos(sc["prior_yaw"] / 2)
    n.prior_pub.publish(p)
    bgr = cv2.imread(str(a.dir / sc["image"]))
    img = Image()
    img.height, img.width, img.encoding, img.step = bgr.shape[0], bgr.shape[1], "bgr8", bgr.shape[1] * 3
    img.data = bgr.tobytes()
    img.header.frame_id = "top_camera_front_color_optical_frame"
    info = CameraInfo()
    info.header.frame_id = img.header.frame_id
    info.width, info.height = W, H
    info.k = [K["fx"], 0.0, K["cx"], 0.0, K["fy"], K["cy"], 0.0, 0.0, 1.0]
    info.p = [K["fx"], 0.0, K["cx"], 0.0, 0.0, K["fy"], K["cy"], 0.0, 0.0, 0.0, 1.0, 0.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.distortion_model, info.d = "plumb_bob", [0.0] * 5
    t_wait = time.time()   # 새 정적 TF · 사전값이 노드에 닿을 때까지 (앞 장면 TF 로 새 영상을 재지 않게)
    while time.time() - t_wait < 1.0:
        rclpy.spin_once(n, timeout_sec=0.05)
    fm = n.mode_cli.call_async(req)   # 장면마다 모드를 다시 켬 (다른 노드가 바꿔도 시험이 안 섞이게)
    rclpy.spin_until_future_complete(n, fm, timeout_sec=5)
    n.st = None
    n.pose = None
    f = n.meas_cli.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(n, f, timeout_sec=5)
    resp = f.result()
    t0 = time.time()
    while time.time() - t0 < a.wait and (n.st is None or (json.loads(n.st.data).get("ok") and n.pose is None)):
        st = n.get_clock().now().to_msg()
        img.header.stamp = st
        info.header.stamp = st
        n.info_pub.publish(info)
        n.img_pub.publish(img)
        rclpy.spin_once(n, timeout_sec=0.1)
    st = json.loads(n.st.data) if n.st else {"ok": False, "why": "status 없음"}
    if st.get("ok") and n.pose is not None:
        x, y = n.pose.pose.position.x, n.pose.pose.position.y
        qz, qw = n.pose.pose.orientation.z, n.pose.pose.orientation.w
        yaw = 2 * math.atan2(qz, qw)
        dc = 1000 * math.hypot(x - sc["center"][0], y - sc["center"][1])
        dy = abs(math.degrees((yaw - sc["yaw"] + math.pi / 4) % (math.pi / 2) - math.pi / 4))
    else:
        dc = dy = float("nan")
    ok = bool(st.get("ok")) and dc <= 15 and dy <= 4
    rows.append(dict(image=sc["image"], group=sc.get("group"), theta=sc.get("box_yaw_deg"), srv=resp.success if resp else None,
                     status=st, c_mm=dc, y_deg=dy, d1=ok, camera=st.get("camera")))
    print(f"{sc['image']} θ {sc.get('box_yaw_deg', 0):+.1f} 서비스 {resp.success if resp else None} status ok {st.get('ok')} camera {st.get('camera')} "
          f"중심 {dc:.2f} mm 요 {dy:.2f}°", flush=True)
    out_json.write_text(json.dumps(rows, indent=1, ensure_ascii=False) + "\n")   # 장마다 저장 (중간에 끊겨도 남게)
c = np.array([r["c_mm"] for r in rows])
y = np.array([r["y_deg"] for r in rows])
print(f"노드 재생: {len(rows)} 장, 찾음 {int(np.isfinite(c).sum())}, D1(≤ 15 mm · ≤ 4°) {sum(r['d1'] for r in rows)}/{len(rows)}, "
      f"중심 최대 {np.nanmax(c):.2f} mm, 요 최대 {np.nanmax(y):.2f}°")
out_json.write_text(json.dumps(rows, indent=1, ensure_ascii=False) + "\n")
n.destroy_node()
rclpy.shutdown()
