"""렌더한 신호등 영상을 ROS 토픽으로 재생해 인지 노드(perception_node)에 넣고 /perception/signal 을 기록한다.

Isaac Sim–Lyrical 브리지를 검증하기 전이라, Isaac Sim 이 저장한 영상(render_signal.py 결과)을 이 스크립트가 발행한다.
발행: /clock, /tf(odom → URDF 뿌리(base_footprint 또는 base_link), 렌더 때 로봇 자세), 앞 D435 컬러 · 컬러에 맞춘 깊이(32FC1) · CameraInfo.
base_footprint 아래 고정 변환(카메라 장착)은 run_replay.sh 가 띄운 robot_state_publisher(URDF)가 낸다.

방식 두 가지:
  stream   : 시간표(점멸 순서 · 켜짐 · 꺼짐 · 텀)대로 30 fps, 시계 = 영상 시각. 끝에 확정 순서 · 확정 시각을 본다 (R2 용)
  lockstep : 한 장 내고, 노드가 그 장을 처리한 결과(같은 시각의 /perception/signal)가 올 때까지 시계를 조금씩 민다.
             프레임마다 상태를 정답과 비교한다 (R1 용). 신호등을 고정하려면 점멸이 필요해서 앞에 --lead-blinks 번 깜빡인다
결과: <out>/signal.jsonl(받은 메시지 전부), <out>/frames.jsonl(낸 프레임과 정답), <out>/summary.json

실행 (PC, run_replay.sh 가 노드를 띄운 뒤 부른다):
  python3 replay_ros.py --render <render_signal 결과 폴더> --pose p00_... --out <폴더> stream --sequence RYYYGGGYRRYG
"""
import argparse
import json
import os
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import cv2

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage

from robot_interfaces.msg import SignalSequence
from robot_interfaces.srv import SetPerceptionMode

from urdf_chain import fixed_transform

FPS = 30.0
LIGHT = {SignalSequence.LIGHT_OFF: "off", SignalSequence.LIGHT_RED: "red", SignalSequence.LIGHT_YELLOW: "yellow",
         SignalSequence.LIGHT_GREEN: "green"}
LETTER = {"R": "red", "Y": "yellow", "G": "green"}
STATUS = {0: "SEARCHING", 1: "COUNTING", 2: "CANDIDATE", 3: "CONFIRMED", 4: "MISMATCH"}


def to_time(t):
    m = TimeMsg()
    m.sec = int(t)
    m.nanosec = int(round((t - int(t)) * 1e9))
    if m.nanosec >= 1_000_000_000:
        m.sec += 1
        m.nanosec -= 1_000_000_000
    return m


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


_ODOM_CHILD = None


def odom_child():
    """odom 아래에 둘 링크와 (그 링크 → base_footprint) 변환. URDF 뿌리가 base_footprint 면 그대로(이전 모델),
    아니면(새 모델 261028: 뿌리 base_link, base_footprint 는 자식) 뿌리를 낸다 — base_footprint 에 부모가 둘이면 TF 트리가 끊김.
    URDF 는 run_replay.sh 가 REPLAY_URDF 로 넘긴다(없으면 이전 동작)."""
    global _ODOM_CHILD
    if _ODOM_CHILD is None:
        urdf = os.environ.get("REPLAY_URDF")
        _ODOM_CHILD = ("base_footprint", np.eye(4))
        if urdf:
            root = ET.parse(urdf).getroot()
            children = {j.find("child").get("link") for j in root.findall("joint")}
            top = next(l.get("name") for l in root.findall("link") if l.get("name") not in children)
            if top != "base_footprint":
                _ODOM_CHILD = (top, fixed_transform(urdf, "base_footprint", root=top))
    return _ODOM_CHILD


def quat_from_matrix(R):
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return x, y, z, w


def blink_schedule(sequence, t_on, t_off, t_term, cycles, lead_off):
    """점멸 시간표 → 프레임마다 상태 이름 목록 (30 fps). 처음에 lead_off 초 꺼짐."""
    events = [("off", lead_off)]
    for _ in range(cycles):
        for i, ch in enumerate(sequence):
            events.append((LETTER[ch], t_on))
            events.append(("off", t_off if i < len(sequence) - 1 else t_term))
    frames, t, acc = [], 0.0, 0.0
    for state, dur in events:
        acc += dur
        while t < acc - 1e-9:
            frames.append(state)
            t = len(frames) / FPS
    return frames


class Replayer(Node):
    def __init__(self, a):
        super().__init__("signal_replayer")
        self.a = a
        meta = json.loads((a.render / "render_signal.json").read_text())
        self.meta = meta
        self.pose = next(p for p in meta["poses"] if p["name"] == a.pose)
        self.images = {}
        for state, f in self.pose["images"].items():
            bgr = cv2.imread(str(a.render / f["color"]), cv2.IMREAD_COLOR)
            dep = np.load(a.render / f["depth"]).astype(np.float32)
            dep[~np.isfinite(dep)] = 0.0   # 하늘 · 먼 곳: 깊이 없음(0)으로
            self.images[state] = (bgr, dep)
        rel = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=a.qos_depth, reliability=ReliabilityPolicy.RELIABLE)
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 100)
        ns = a.camera_ns
        self.color_pub = self.create_publisher(Image, f"{ns}/color/image_raw", rel)
        self.depth_pub = self.create_publisher(Image, f"{ns}/aligned_depth_to_color/image_raw", rel)
        self.info_pub = self.create_publisher(CameraInfo, f"{ns}/color/camera_info", rel)
        self.msgs = []
        self.create_subscription(SignalSequence, "/perception/signal", self.on_signal,
                                 QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
        self.mode_cli = self.create_client(SetPerceptionMode, "/perception/set_mode")
        self.now_s = a.t0

    def on_signal(self, m):
        self.msgs.append({"stamp": round(stamp_s(m.header), 6), "status": STATUS.get(m.status, m.status),
                          "sequence": m.sequence, "red": m.red, "yellow": m.yellow, "green": m.green, "cycles": m.cycles,
                          "light": LIGHT.get(m.light, m.light), "light_visible": m.light_visible,
                          "position_valid": m.position_valid,
                          "position": [round(m.position.x, 4), round(m.position.y, 4), round(m.position.z, 4)],
                          "t_on": round(m.t_on, 4), "t_off": round(m.t_off, 4), "t_term": round(m.t_term, 4)})

    def spin_for(self, wall_s):
        end = time.monotonic() + wall_s
        while True:
            rclpy.spin_once(self, timeout_sec=max(0.0, min(0.005, end - time.monotonic())))
            if time.monotonic() >= end:
                return

    def set_clock(self, t):
        self.now_s = t
        c = Clock()
        c.clock = to_time(t)
        self.clock_pub.publish(c)

    def set_mode(self, mode):
        if not self.mode_cli.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/perception/set_mode 없음")
        req = SetPerceptionMode.Request()
        req.mode = mode
        fut = self.mode_cli.call_async(req)
        while not fut.done():
            self.set_clock(self.now_s)   # 서비스 콜백은 시계와 무관하지만, 노드가 시계를 먼저 받게
            self.spin_for(0.05)
        r = fut.result()
        if not r.success:
            raise RuntimeError(f"set_mode {mode} 실패: {r.message}")

    def publish_frame(self, state, t):
        st = to_time(t)
        child, T_child_bf = odom_child()
        T = np.array(self.pose["T_world_base_footprint"]) @ np.linalg.inv(T_child_bf)
        tf = TransformStamped()
        tf.header.stamp = st
        tf.header.frame_id = "odom"   # odom = 월드 (오도메트리 오차 없음)
        tf.child_frame_id = child
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, T[:3, 3])
        q = quat_from_matrix(T[:3, :3])
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = map(float, q)
        self.tf_pub.publish(TFMessage(transforms=[tf]))
        bgr, dep = self.images[state]
        frame = self.meta["frame"]
        K = self.meta["K"]
        info = CameraInfo()
        info.header.stamp, info.header.frame_id = st, frame
        info.width, info.height = self.meta["width"], self.meta["height"]
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [K["fx"], 0.0, K["cx"], 0.0, K["fy"], K["cy"], 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [K["fx"], 0.0, K["cx"], 0.0, 0.0, K["fy"], K["cy"], 0.0, 0.0, 0.0, 1.0, 0.0]
        d = Image()
        d.header.stamp, d.header.frame_id = st, frame
        d.height, d.width = dep.shape
        d.encoding, d.step = "32FC1", dep.shape[1] * 4
        d.data = dep.tobytes()
        c = Image()
        c.header.stamp, c.header.frame_id = st, frame
        c.height, c.width = bgr.shape[:2]
        c.encoding, c.step = "bgr8", bgr.shape[1] * 3
        c.data = bgr.tobytes()
        self.info_pub.publish(info)
        self.depth_pub.publish(d)   # 깊이를 컬러보다 먼저: 노드가 컬러를 처리할 때 짝이 이미 있게
        self.color_pub.publish(c)


def run_stream(n, a, out):
    frames = blink_schedule(a.sequence, a.t_on, a.t_off, a.t_term, a.cycles, a.lead_off)
    log = []
    wall0 = time.monotonic()
    for k, state in enumerate(frames):
        t = a.t0 + a.start_delay + k / FPS
        n.set_clock(t)
        n.publish_frame(state, t)
        log.append({"k": k, "t": round(t, 6), "state": state})
        # 실제 시간 배율 a.rate 로 맞춤
        target = wall0 + (k + 1) / FPS / a.rate
        n.spin_for(max(0.0, target - time.monotonic()))
    t_end = a.t0 + a.start_delay + len(frames) / FPS
    for j in range(int(FPS * 1.0)):   # 끝난 뒤 1 s 시계만 밀어 남은 프레임 처리
        n.set_clock(t_end + j / FPS)
        n.spin_for(1.0 / FPS / a.rate)
    t_start = a.t0 + a.start_delay
    conf = next((m for m in n.msgs if m["status"] == "CONFIRMED"), None)
    last = n.msgs[-1] if n.msgs else None
    period = len(a.sequence) * (a.t_on + a.t_off) - a.t_off + a.t_term
    summary = {"mode": "stream", "truth_sequence": a.sequence, "frames": len(frames), "messages": len(n.msgs),
               "final": last, "confirmed": conf is not None,
               "confirmed_sequence": conf["sequence"] if conf else None,
               "confirmed_correct": bool(conf and conf["sequence"] == a.sequence),
               "wrong_confirmations": sum(1 for m in n.msgs if m["status"] == "CONFIRMED" and m["sequence"] != a.sequence),
               "confirm_after_s": round(conf["stamp"] - t_start, 3) if conf else None,
               "confirm_after_cycles": round((conf["stamp"] - t_start) / period, 3) if conf else None,
               "cycle_period_s": round(period, 4)}
    return log, summary


def run_lockstep(n, a, out):
    lead = []
    for _ in range(a.lead_blinks):   # 신호등 고정용 점멸 (빨강 6장 · 꺼짐 6장)
        lead += ["red"] * 6 + ["off"] * 6
    test = [s for s in a.states.split(",") for _ in range(a.repeat)]
    log, rows = [], []
    t = a.t0 + a.start_delay
    for k, state in enumerate(lead + test):
        is_test = k >= len(lead)
        n.publish_frame(state, t)
        n.spin_for(a.settle)   # 영상이 노드에 닿을 시간 (시계는 아직 그대로)
        got = None
        tc = t
        for _ in range(int(a.max_wait / (1.0 / FPS))):
            tc += 1.0 / FPS
            n.set_clock(tc)
            n.spin_for(a.settle)
            got = next((m for m in reversed(n.msgs) if abs(m["stamp"] - t) < 1e-6), None)
            if got:
                break
        log.append({"k": k, "t": round(t, 6), "state": state, "test": is_test})
        if is_test:
            rows.append({"k": k, "truth": state, "got": got["light"] if got and got["light_visible"] else None,
                         "visible": bool(got and got["light_visible"]), "answered": got is not None,
                         "status": got["status"] if got else None})
        t = tc + 1.0 / FPS
    ok = sum(1 for r in rows if r["got"] == r["truth"])
    confusion = {}
    for r in rows:
        key = f"{r['truth']}->{r['got']}"
        confusion[key] = confusion.get(key, 0) + 1
    locked_at_test = rows[0]["status"] if rows else None
    summary = {"mode": "lockstep", "lead_blinks": a.lead_blinks, "test_frames": len(rows), "correct": ok,
               "accuracy": round(ok / len(rows), 4) if rows else None, "confusion": confusion,
               "unanswered": sum(1 for r in rows if not r["answered"]), "status_at_first_test": locked_at_test,
               "rows": rows}
    return log, summary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--render", type=Path, required=True)
    ap.add_argument("--pose", required=True, help="render_signal.json 의 poses[].name")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--camera-ns", default="/top_camera_front")
    ap.add_argument("--t0", type=float, default=1000.0, help="시작 시계 [s] (0 근처를 피함)")
    ap.add_argument("--start-delay", type=float, default=1.0)
    ap.add_argument("--qos-depth", type=int, default=10)
    sub = ap.add_subparsers(dest="mode", required=True)
    s = sub.add_parser("stream")
    s.add_argument("--sequence", required=True)
    s.add_argument("--t-on", type=float, required=True)
    s.add_argument("--t-off", type=float, required=True)
    s.add_argument("--t-term", type=float, required=True)
    s.add_argument("--cycles", type=int, default=4)
    s.add_argument("--lead-off", type=float, default=0.5)
    s.add_argument("--rate", type=float, default=1.0, help="실제 시간 배율 (1 = 실시간)")
    lk = sub.add_parser("lockstep")
    lk.add_argument("--states", default="off,red,yellow,green")
    lk.add_argument("--repeat", type=int, default=10)
    lk.add_argument("--lead-blinks", type=int, default=4)
    lk.add_argument("--settle", type=float, default=0.03, help="[s] 발행 뒤 기다리는 실제 시간")
    lk.add_argument("--max-wait", type=float, default=1.0, help="[s] 한 장의 결과를 기다리는 시계 시간")
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    n = Replayer(a)
    n.set_clock(a.t0)
    n.spin_for(1.0)
    n.set_mode(SetPerceptionMode.Request.READ_SIGNAL)
    try:
        log, summary = (run_stream if a.mode == "stream" else run_lockstep)(n, a, a.out)
    finally:
        try:
            n.set_mode(SetPerceptionMode.Request.OFF)
        except Exception as e:  # noqa: BLE001
            print("set_mode OFF 실패:", e)
    summary.update({"render": str(a.render), "pose": a.pose, "pose_meta": {k: n.pose[k] for k in (
        "dist", "bearing_deg", "emissive", "robot_xy_yaw", "segments")}})
    with open(a.out / "signal.jsonl", "w") as f:
        for m in n.msgs:
            f.write(json.dumps(m, ensure_ascii=False) + "\n")
    with open(a.out / "frames.jsonl", "w") as f:
        for r in log:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")
    (a.out / "summary.json").write_text(json.dumps(summary, indent=1, ensure_ascii=False))
    print(json.dumps({k: v for k, v in summary.items() if k not in ("rows", "pose_meta")}, ensure_ascii=False))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
