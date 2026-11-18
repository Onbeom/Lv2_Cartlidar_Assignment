"""R1 · R2 묶음 재생: render_r12.py 결과(render_r12.json)의 여러 자세를 한 번 띄운 인지 노드에 차례로 넣는다.

자세(또는 회)마다 /perception/set_mode 로 READ_SIGNAL 을 껐다 켜서 센 것을 지운 뒤 시작한다. 시계는 계속 앞으로만 간다.
  r1 : lockstep. 고정용 점멸(빨강 · 노랑 · 초록 각 6장 + 꺼짐 6장, --lead-rounds 번) 뒤 시험 4장(off · R · Y · G, 순서 무작위).
       판정(R1R2_설계_가설_261024.md 2장): 거리마다 정답 상태 비율('모름'은 틀림), 퍽을 켜진 등으로 본 수
       (= 정답 꺼짐인데 색을 냄, 또는 고정 위치가 신호등 축에서 0.10 m 넘게 떨어짐)
  r2 : stream. 회마다 12자 순서 · 켜짐 · 꺼짐 · 텀을 U1 범위에서 뽑고(seed 고정) 아무 때나 보기 시작, 최대 --cycles 주기,
       확정되면 멈춤. 판정: 정답 확정 수, 틀린 확정 수
  marker : R5. render_r5.py 결과(render_r5.json)를 한 장씩 내고 /perception/task_marker 를 기다린다(READ_MARKER).
       판정: 1.0 m 이내 번호 맞음 100 %, 위치 오차(base_link 태그 가운데) ≤ 0.03 m
결과: <out>/r1_rows.jsonl · r2_runs.jsonl · r5_rows.jsonl, <out>/signal.jsonl, <out>/summary.json

실행 (PC, run_replay.sh 가 노드를 띄운 뒤 부른다. run_replay.sh 는 replay_ros.py 를 부르므로 REPLAY 환경 변수로 바꿈):
  REPLAY=replay_r12.py Cartlidar/sim_render/run_replay.sh <URDF> <결과> <setup.bash> -- --render <render_r12 결과> r1
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage

from robot_interfaces.msg import SignalSequence, TaskMarker
from robot_interfaces.srv import SetPerceptionMode

from replay_ros import FPS, Replayer, blink_schedule

BASE_LINK_Z = 0.00675   # URDF base_footprint → base_link


class BatchReplayer(Replayer):
    """Replayer 의 발행 · 시계 · 모드 함수를 쓰고, 자세를 바꿔 가며 재생한다."""

    def __init__(self, a):
        Node.__init__(self, "signal_replayer_r12")
        self.a = a
        meta = a.render / ("render_r5.json" if a.mode == "marker" else "render_r12.json")
        self.meta = json.loads(meta.read_text())
        self.markers = []
        if a.mode == "marker":
            self.create_subscription(TaskMarker, "/perception/task_marker", self.on_marker,
                                     QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
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
        self.pose = None
        self.images = {}

    def on_marker(self, m):
        self.markers.append({"stamp": round(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, 6), "found": m.found,
                             "id": m.marker_id, "frame": m.header.frame_id, "distance": round(m.distance, 4),
                             "position": [m.pose.position.x, m.pose.position.y, m.pose.position.z]})

    def load(self, pose):
        self.pose = pose
        self.images = {}
        for state, f in pose["images"].items():
            bgr = cv2.imread(str(self.a.render / f["color"]), cv2.IMREAD_COLOR)
            dep = np.load(self.a.render / f["depth"]).astype(np.float32)
            dep[~np.isfinite(dep)] = 0.0
            self.images[state] = (bgr, dep)

    def restart(self):
        self.set_mode(SetPerceptionMode.Request.OFF)
        self.set_mode(SetPerceptionMode.Request.READ_SIGNAL)

    def lockstep_frame(self, state, t):
        """한 장 내고 그 시각의 결과를 기다린다. 반환 (결과 메시지 또는 None, 다음 시각)."""
        self.publish_frame(state, t)
        self.spin_for(self.a.settle)
        tc = t
        for _ in range(int(self.a.max_wait * FPS)):
            tc += 1.0 / FPS
            self.set_clock(tc)
            self.spin_for(self.a.settle)
            got = next((m for m in reversed(self.msgs) if abs(m["stamp"] - t) < 1e-6), None)
            if got:
                return got, tc + 1.0 / FPS
        return None, tc + 1.0 / FPS


def light_axis_error(pose, msg):
    """메시지의 신호등 위치(base_link)와 신호등 축(초록 아래 ~ 빨강 위) 사이 거리 [m]."""
    if not msg or not msg["position_valid"]:
        return None
    T = np.array(pose["T_world_base_footprint"])
    p_bl = np.array(msg["position"]) + np.array([0, 0, BASE_LINK_Z])
    pw = T[:3, :3] @ p_bl + T[:3, 3]
    seg = pose["segments"]
    x, y = seg["red"]["world"][0], seg["red"]["world"][1]
    z0, z1 = seg["green"]["world"][2] - 0.0175, seg["red"]["world"][2] + 0.0175
    dz = max(z0 - pw[2], 0.0, pw[2] - z1)
    return float(math.hypot(math.hypot(pw[0] - x, pw[1] - y), dz))


def run_r1(n, a, rng):
    rows = []
    t = a.t0 + a.start_delay
    lead = []
    for _ in range(a.lead_rounds):
        for c in ("red", "yellow", "green"):
            lead += [c] * 6 + ["off"] * 6
    for pose in n.meta["poses"]:
        n.load(pose)
        n.restart()
        for state in lead:
            _, t = n.lockstep_frame(state, t)
        for state in rng.permutation(["off", "red", "yellow", "green"]):
            got, t_next = n.lockstep_frame(str(state), t)
            light = got["light"] if got and got["light_visible"] else None
            rows.append({"pose": pose["name"], "dist": pose["dist_nominal"], "light_level": pose["light_level"],
                         "truth": str(state), "got": light, "status": got["status"] if got else None,
                         "answered": got is not None, "pucks": len(pose["pucks"]),
                         "position": got["position"] if got else None,
                         "pos_err_m": light_axis_error(pose, got),
                         "puck_as_light": bool(state == "off" and light not in (None, "off"))})
            t = t_next
        last = rows[-1]
        print(pose["name"], [(r["truth"], r["got"]) for r in rows[-4:]], "pos_err",
              None if last["pos_err_m"] is None else round(last["pos_err_m"], 3), flush=True)
    summary = {"mode": "r1", "frames": len(rows), "by_dist": {}}
    for d in sorted({r["dist"] for r in rows}):
        rs = [r for r in rows if r["dist"] == d]
        far = [r for r in rs if r["pos_err_m"] is not None and r["pos_err_m"] > 0.10]
        conf = {}
        for r in rs:
            conf[f"{r['truth']}->{r['got']}"] = conf.get(f"{r['truth']}->{r['got']}", 0) + 1
        summary["by_dist"][f"{d:g}"] = {
            "frames": len(rs), "correct": sum(r["got"] == r["truth"] for r in rs),
            "accuracy": round(sum(r["got"] == r["truth"] for r in rs) / len(rs), 4),
            "puck_as_light_off_frames": sum(r["puck_as_light"] for r in rs),
            "position_far_frames": len(far), "max_pos_err_m": max((r["pos_err_m"] or 0) for r in rs),
            "confusion": conf}
    return rows, summary


def run_marker(n, a, rng):
    rows = []
    n.set_mode(SetPerceptionMode.Request.READ_MARKER)
    t = a.t0 + a.start_delay
    for pose in n.meta["poses"]:
        n.load(pose)
        n.publish_frame("frame", t)
        n.spin_for(a.settle)
        got, tc = None, t
        for _ in range(int(a.max_wait * FPS)):
            tc += 1.0 / FPS
            n.set_clock(tc)
            n.spin_for(a.settle)
            got = next((m for m in reversed(n.markers) if abs(m["stamp"] - t) < 1e-6), None)
            if got:
                break
        T_w_bl = np.array(pose["T_world_base_footprint"])
        T_w_bl[:3, 3] += T_w_bl[:3, :3] @ np.array([0, 0, BASE_LINK_Z])
        truth = (np.linalg.inv(T_w_bl) @ np.append(pose["tag"]["center"], 1.0))[:3]
        cam = np.array(pose["T_world_cam"])[:3, 3]
        found = bool(got and got["found"])
        err = float(np.linalg.norm(np.array(got["position"]) - truth)) if found else None
        rows.append({"pose": pose["name"], "seed": pose["seed"], "tag_id": pose["tag"]["id"], "dist": pose["dist_nominal"],
                     "tilt_deg": pose["tilt_deg"], "light_level": pose["light_level"], "answered": got is not None,
                     "tag_visible": pose.get("tag_visible"),
                     "found": found, "got_id": got["id"] if found else None,
                     "id_ok": bool(found and got["id"] == pose["tag"]["id"]), "pos_err_m": err,
                     "dist_meas": got["distance"] if found else None,
                     "dist_true": round(float(np.linalg.norm(cam - np.array(pose["tag"]["center"]))), 4)})
        t = tc + 1.0 / FPS
        r = rows[-1]
        print(r["pose"], r["found"], r["got_id"], None if err is None else round(err, 4), flush=True)
    summary = {"mode": "marker", "frames": len(rows), "by_dist": {}}
    for d in sorted({r["dist"] for r in rows}):
        rs = [r for r in rows if r["dist"] == d]
        errs = [r["pos_err_m"] for r in rs if r["pos_err_m"] is not None]
        summary["by_dist"][f"{d:g}"] = {
            "frames": len(rs), "id_ok": sum(r["id_ok"] for r in rs), "wrong_id": sum(r["found"] and not r["id_ok"] for r in rs),
            "not_found": sum(not r["found"] for r in rs), "max_pos_err_m": max(errs) if errs else None,
            "fully_visible": sum((r["tag_visible"] or 0) >= 1.0 for r in rs),
            "id_ok_fully_visible": sum(r["id_ok"] for r in rs if (r["tag_visible"] or 0) >= 1.0),
            "over_30mm": sum(e > 0.03 for e in errs),
            "by_tilt": {f"{a:g}": f"{sum(r['id_ok'] for r in rs if r['tilt_deg'] == a)}/"
                        f"{sum(1 for r in rs if r['tilt_deg'] == a)}" for a in sorted({r['tilt_deg'] for r in rs})}}
    summary["infeasible"] = [p["id"] for p in n.meta.get("infeasible", [])]
    return rows, summary


def random_timing(rng):
    t_on, t_off = rng.uniform(0.1, 1.0), rng.uniform(0.1, 1.0)
    lo = max(3 * t_off, 1.0)
    return "".join(rng.choice(list("RYG"), 12)), float(t_on), float(t_off), float(lo + rng.uniform(0, 1) * (5.0 - lo))


def mission_timing(rng):
    # 색별 1~6, 합 12인 개수 조합과 순서를 무작위로 뽑는다.
    counts = [(r, y, 12-r-y) for r in range(1, 7) for y in range(1, 7) if 1 <= 12-r-y <= 6]
    r, y, g = counts[int(rng.integers(len(counts)))]
    return "".join(rng.permutation(list("R"*r + "Y"*y + "G"*g))), 1.0, 1.0, 3.0


def run_r2(n, a, rng):
    runs = []
    t_base = a.t0 + a.start_delay
    groups = {}
    for pose in n.meta["poses"]:
        groups.setdefault((pose["dist_nominal"], pose["light_level"]), []).append(pose)
    mission_rng_state = rng.bit_generator.state
    for (dist, level), poses in sorted(groups.items()):
        if a.mission:
            rng.bit_generator.state = mission_rng_state   # 두 조명에서 같은 순서·시작 위상
        for k in range(a.runs_per_group):
            pose = poses[k % len(poses)]
            seq, t_on, t_off, t_term = mission_timing(rng) if a.mission else random_timing(rng)
            period = 12 * t_on + 11 * t_off + t_term
            sched = blink_schedule(seq, t_on, t_off, t_term, int(math.ceil(a.cycles)) + 1, 0.0)
            start = int(rng.uniform(0, period) * FPS)
            frames = sched[start:start + int(a.cycles * period * FPS)]
            n.load(pose)
            n.restart()
            n0 = len(n.msgs)
            wall0 = time.monotonic()
            conf = None
            for i, state in enumerate(frames):
                t = t_base + i / FPS
                n.set_clock(t)
                n.publish_frame(state, t)
                n.spin_for(max(0.0, wall0 + (i + 1) / FPS / a.rate - time.monotonic()))
                conf = next((m for m in n.msgs[n0:] if m["status"] == "CONFIRMED"), None)
                if conf:
                    break
            if not conf:   # 끝난 뒤 1 s 시계만 밀어 남은 프레임 처리
                for j in range(int(FPS)):
                    n.set_clock(t_base + (len(frames) + j) / FPS)
                    n.spin_for(1.0 / FPS / a.rate)
                conf = next((m for m in n.msgs[n0:] if m["status"] == "CONFIRMED"), None)
            wrong = sum(1 for m in n.msgs[n0:] if m["status"] == "CONFIRMED" and m["sequence"] != seq)
            r = {"pose": pose["name"], "dist": dist, "light_level": level, "sequence": seq, "t_on": t_on, "t_off": t_off,
                 "t_term": t_term, "period": period, "start_frame": start, "confirmed": conf is not None,
                 "confirmed_sequence": conf["sequence"] if conf else None,
                 "correct": bool(conf and conf["sequence"] == seq), "wrong_confirmations": wrong,
                 "confirm_after_cycles": round((conf["stamp"] - t_base) / period, 3) if conf else None,
                 "t_on_meas": conf["t_on"] if conf else None, "t_off_meas": conf["t_off"] if conf else None,
                 "rate": a.rate, "wall_s": round(time.monotonic() - wall0, 1)}
            r["within_limit"] = bool(conf and conf["stamp"] - t_base <= a.cycles * period + 1e-6)
            runs.append(r)
            with (a.out / "r2_progress.jsonl").open("a") as f:
                f.write(json.dumps(r, ensure_ascii=False) + "\n")
            print(json.dumps({k: r[k] for k in ("pose", "sequence", "correct", "wrong_confirmations",
                                                "confirm_after_cycles", "wall_s")}), flush=True)
            t_base += (len(frames) + 2 * FPS) / FPS
    summary = {"mode": "r2", "runs": len(runs), "correct": sum(r["correct"] for r in runs),
               "wrong_confirmations": sum(r["wrong_confirmations"] for r in runs),
               "not_confirmed": sum(not r["confirmed"] for r in runs), "rate": a.rate,
               "mission": a.mission, "within_limit": sum(r["within_limit"] for r in runs),
               "max_confirm_after_cycles": max((r["confirm_after_cycles"] for r in runs if r["confirmed"]), default=None),
               "pass": all(r["correct"] and r["within_limit"] and not r["wrong_confirmations"] for r in runs)}
    return runs, summary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--render", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--camera-ns", default="/top_camera_front")
    ap.add_argument("--t0", type=float, default=1000.0)
    ap.add_argument("--start-delay", type=float, default=1.0)
    ap.add_argument("--qos-depth", type=int, default=10)
    ap.add_argument("--seed", type=int, default=20261024)
    ap.add_argument("--only", nargs="*", help="이 문자열이 이름에 든 자세만")
    sub = ap.add_subparsers(dest="mode", required=True)
    r1 = sub.add_parser("r1")
    r1.add_argument("--lead-rounds", type=int, default=2)
    r1.add_argument("--settle", type=float, default=0.03)
    r1.add_argument("--max-wait", type=float, default=1.0)
    r2 = sub.add_parser("r2")
    r2.add_argument("--mission", action="store_true", help="발제 1초 켜짐/1초 소등, 마지막 소등 3초, 색별 1~6 합12")
    r2.add_argument("--runs-per-group", type=int, default=1, help="(거리, 조명)마다 회 수")
    r2.add_argument("--cycles", type=float, default=5.0)
    r2.add_argument("--rate", type=float, default=1.0)
    mk = sub.add_parser("marker")
    for p in (r2, mk):
        p.add_argument("--settle", type=float, default=0.03)
        p.add_argument("--max-wait", type=float, default=1.0)
    a = ap.parse_args()
    if a.mode == "r2" and a.mission:
        a.cycles = 3.0
    a.out.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    n = BatchReplayer(a)
    if a.only:
        n.meta["poses"] = [p for p in n.meta["poses"] if any(s in p["name"] for s in a.only)]
    rng = np.random.default_rng(a.seed)
    n.set_clock(a.t0)
    n.spin_for(1.0)
    try:
        rows, summary = {"r1": run_r1, "r2": run_r2, "marker": run_marker}[a.mode](n, a, rng)
    finally:
        try:
            n.set_mode(SetPerceptionMode.Request.OFF)
        except Exception as e:  # noqa: BLE001
            print("set_mode OFF 실패:", e)
    summary.update({"render": str(a.render), "seed": a.seed, "emissive": n.meta.get("emissive")})
    with open(a.out / {"r1": "r1_rows.jsonl", "r2": "r2_runs.jsonl", "marker": "r5_rows.jsonl"}[a.mode], "w") as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")
    with open(a.out / "signal.jsonl", "w") as f:
        for m in n.msgs:
            f.write(json.dumps(m, ensure_ascii=False) + "\n")
    (a.out / "summary.json").write_text(json.dumps(summary, indent=1, ensure_ascii=False))
    print(json.dumps(summary, ensure_ascii=False))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
