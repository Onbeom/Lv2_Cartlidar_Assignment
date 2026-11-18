"""후방 장애물 재생 · 판정 (모의, 계획_후방장애물.md 2장): render_rear.py 깊이를 실제 rear_obstacle 노드에 넣는다.

장면마다 깊이 --frames 장을 30 Hz 시계로 낸다. 한 장 내고 /perception/rear_obstacle/distance 가 하나 올 때까지 기다린다(lockstep).
CameraInfo = 렌더 K, frame_id = 뒤 컬러 광학 프레임(고정 TF 는 robot_state_publisher 가 URDF 로 냄).
판정: 검출(정답 < 0.40 · 보이는) 프레임 ≥ 99 % 가 거리 < 0.40, 헛검출(정답 ≥ 0.50 또는 빈 통로) 프레임 ≤ 1 %,
      거리 오차(보이는 물체, 정답 ≤ 0.60) 중앙값 ≤ 0.03 m. 사각 · 0.40~0.50 은 기록만.
결과: <out>/rows.jsonl(프레임마다), <out>/summary.json · summary.md
실행: run_replay_rear.sh 가 노드를 띄운 뒤 부른다.
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32

STOP_M, RELEASE_M = 0.40, 0.50


def to_time(t):
    s = int(math.floor(t))
    return TimeMsg(sec=s, nanosec=int(round((t - s) * 1e9)) % 1000000000)


class Rear(Node):
    def __init__(self, a, meta):
        super().__init__("rear_replayer")
        self.a, self.meta = a, meta
        rel = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.depth_pub = self.create_publisher(Image, "/top_camera_rear/aligned_depth_to_color/image_raw", rel)
        self.info_pub = self.create_publisher(CameraInfo, "/top_camera_rear/color/camera_info", rel)
        self.dist = []
        self.stop = None
        self.create_subscription(Float32, "/perception/rear_obstacle/distance", lambda m: self.dist.append(m.data), 50)
        tl = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/perception/rear_obstacle", lambda m: setattr(self, "stop", m.data), tl)
        K = meta["K"]
        self.info = CameraInfo(width=meta["width"], height=meta["height"], distortion_model="plumb_bob", d=[0.0] * 5,
                               k=[K["fx"], 0.0, K["cx"], 0.0, K["fy"], K["cy"], 0.0, 0.0, 1.0],
                               r=[1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0],
                               p=[K["fx"], 0.0, K["cx"], 0.0, 0.0, K["fy"], K["cy"], 0.0, 0.0, 0.0, 1.0, 0.0])
        self.info.header.frame_id = meta["frame"]

    def spin_for(self, s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def clock(self, t):
        self.clock_pub.publish(Clock(clock=to_time(t)))

    def frame(self, dep, t):
        self.clock(t)
        h = to_time(t)
        self.info.header.stamp = h
        img = Image(height=dep.shape[0], width=dep.shape[1], encoding="32FC1", is_bigendian=0, step=dep.shape[1] * 4,
                    data=np.ascontiguousarray(dep, np.float32).tobytes())
        img.header.stamp, img.header.frame_id = h, self.meta["frame"]
        n0 = len(self.dist)
        for _ in range(self.a.retries):
            self.info_pub.publish(self.info)
            self.depth_pub.publish(img)
            end = time.time() + self.a.wait
            while time.time() < end and len(self.dist) == n0:
                rclpy.spin_once(self, timeout_sec=0.01)
            if len(self.dist) > n0:
                return float(self.dist[n0])
        return None


def summarize(rows):
    def frac(xs):
        return (sum(xs) / len(xs)) if xs else None

    det = [r for r in rows if r["truth"] is not None and r["truth"] < STOP_M and r["visible"]]
    neg = [r for r in rows if r["truth"] is None or r["truth"] >= RELEASE_M]
    err = [abs(r["got"] - r["truth"]) for r in rows if r["truth"] is not None and r["truth"] <= 0.60 and r["visible"]
           and r["got"] is not None and np.isfinite(r["got"])]
    miss_frames = [r for r in rows if not r["resp"]]
    s = {
        "frames": len(rows), "frames_no_output": len(miss_frames),
        "detect": {"frames": len(det), "rate": frac([r["got"] is not None and r["got"] < STOP_M for r in det]),
                   "pass": bool(det) and frac([r["got"] is not None and r["got"] < STOP_M for r in det]) >= 0.99},
        "false": {"frames": len(neg), "rate": frac([r["got"] is not None and r["got"] < STOP_M for r in neg]),
                  "pass": bool(neg) and frac([r["got"] is not None and r["got"] < STOP_M for r in neg]) <= 0.01},
        "dist_err": {"n": len(err), "median_m": float(np.median(err)) if err else None,
                     "p95_m": float(np.percentile(err, 95)) if err else None,
                     "pass": bool(err) and float(np.median(err)) <= 0.03},
        "blind_in_zone": sorted({r["scene"] for r in rows if r["truth"] is not None and r["truth"] < STOP_M and not r["visible"]}),
    }
    by = {}
    for r in rows:
        k = (r["kind"], r["zone"])
        g = by.setdefault(f"{k[0]}_{k[1]}", {"frames": 0, "stop": 0, "visible": 0, "err": []})
        g["frames"] += 1
        g["stop"] += int(r["got"] is not None and r["got"] < STOP_M)
        g["visible"] += int(r["visible"])
        if r["truth"] is not None and r["got"] is not None and np.isfinite(r["got"]) and r["visible"]:
            g["err"].append(r["got"] - r["truth"])
    for g in by.values():
        e = g.pop("err")
        g["err_median_m"] = float(np.median(e)) if e else None
    s["by_kind"] = by
    s["pass"] = all(s[k]["pass"] for k in ("detect", "false", "dist_err")) and not miss_frames
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--render", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--frames", type=int, default=5)
    ap.add_argument("--wait", type=float, default=1.0)
    ap.add_argument("--retries", type=int, default=3)
    a = ap.parse_args()
    meta = json.loads((a.render / "render_rear.json").read_text())
    rclpy.init()
    n = Rear(a, meta)
    t = 100.0
    for _ in range(60):          # 노드 · TF 준비 (시계 진행)
        n.clock(t)
        t += 1 / 30
        n.spin_for(0.05)
    rows = []
    with open(a.out / "rows.jsonl", "w") as f:
        for sc in meta["scenes"]:
            dep = np.load(a.render / sc["depth"])
            truth = sc["obj"]["truth_m"] if sc["obj"] else None
            for k in range(a.frames):
                t += 1 / 30
                got = n.frame(dep, t)
                r = {"scene": sc["id"], "k": k, "kind": sc["obj"]["kind"] if sc["obj"] else "none", "zone": sc["zone"],
                     "truth": truth, "visible": bool(sc["visible"]), "obj_px": sc["obj_px"],
                     "resp": got is not None,                                   # False = 노드가 답하지 않음
                     "got": round(got, 4) if got is not None and np.isfinite(got) else None,   # None = 장애물 없음(NaN)
                     "stop": n.stop}
                rows.append(r)
                f.write(json.dumps(r, ensure_ascii=False) + "\n")
            print(sc["id"], "정답", truth, "보임", sc["visible"], "출력", [r["got"] for r in rows[-a.frames:]], flush=True)
    s = summarize(rows)
    (a.out / "summary.json").write_text(json.dumps(s, indent=1, ensure_ascii=False))
    print(json.dumps(s, indent=1, ensure_ascii=False))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
