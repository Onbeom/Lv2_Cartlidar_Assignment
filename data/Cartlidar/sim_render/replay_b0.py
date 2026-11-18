"""B0′ · P4 재생 · 판정 (2026-10-31): render_b0.py 결과를 측량 순서대로 실제 인지 노드에 넣고, 퍽 등록부(perception_survey)
start → 장면들 → finish 를 한 번의 측량(회)으로 본다. 회 = 렌더 폴더(월드 · 색온도) × 조명.

장면마다: 모드 SEARCH_FRONT(d435) 또는 READ_TAG(arm) → odom → URDF 뿌리 TF(렌더 차체 자리), 팔 관절 /joint_states
(robot_state_publisher 가 팔 TF 로), 카메라 영상 · CameraInfo(d435 는 깊이 먼저)를 촬영 시각으로 내고, 그 시각의 /perception/pucks 를 기다린다.
map → odom 은 항등(정적 TF) — 지도 = 발제 좌표(SLAM 오차 없음, 판정 계획의 범위).
판정(판정 계획 B0′ · P4): 회마다 finish 성공 · complete 이고 12칸 모두 상자 · side · ID · 색 · 모양 · 서있음/누움이 정답과 같고
  중심 오차 ≤ 10 mm 이면 통과(288/288칸). 못 채우면 finish 가 거부해야 함(틀린 완료 0).
  P4: 누운 직육면체 · 서 있는 퍽의 자세 판별 — 칸(등록부) 단위 오류 0, 관측 단위 오류도 센다(기록).
결과: <out>/b0_runs.jsonl(회마다 등록부 · 칸 대조), <out>/pucks.jsonl(받은 /perception/pucks), <out>/summary.json

실행 (PC, run_replay.sh 가 노드를 띄운 뒤 부른다. 등록부 완료 파일은 REPLAY_LAUNCH_ARGS="survey_output:=<out>/survey.json"):
  REPLAY=replay_b0.py REPLAY_LAUNCH_ARGS=... Cartlidar/sim_render/run_replay.sh <URDF> <결과> <setup.bash> -- --render <render_b0 결과> ...
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster

from robot_interfaces.msg import PuckArray
from robot_interfaces.srv import SetPerceptionMode

from replay_ros import FPS, Replayer, odom_child, quat_from_matrix, stamp_s, to_time

COLORS = {1: "red", 2: "green", 3: "blue"}
SHAPES = {1: "cylinder", 2: "cuboid"}
CAMS = {0: "front_d435", 1: "rear_d435", 2: "arm"}


class SurveyReplayer(Replayer):
    def __init__(self, a):
        Node.__init__(self, "survey_replayer_b0")
        self.a = a
        rel = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=a.qos_depth, reliability=ReliabilityPolicy.RELIABLE)
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 100)
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.pub = {"d435": (self.create_publisher(Image, "/top_camera_front/color/image_raw", rel),
                             self.create_publisher(CameraInfo, "/top_camera_front/color/camera_info", rel),
                             self.create_publisher(Image, "/top_camera_front/aligned_depth_to_color/image_raw", rel)),
                    "arm": (self.create_publisher(Image, "/arm_camera/image_raw", rel),
                            self.create_publisher(CameraInfo, "/arm_camera/camera_info", rel), None)}
        self.puck_msgs = []
        self.survey = {}
        self.create_subscription(PuckArray, "/perception/pucks", self.on_pucks, QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(String, "/perception/puck_survey", lambda m: setattr(self, "survey", json.loads(m.data)),
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.mode_cli = self.create_client(SetPerceptionMode, "/perception/set_mode")
        self.static = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = "map", "odom"
        tf.transform.rotation.w = 1.0
        tfs = [tf]
        if a.mission_map is not None:
            # SLAM 꼴(2026-11-01 YOLO 1101 2 통보): mission_map(발제) → map(Cartographer 지도) = A, map → odom = A⁻¹
            #   → mission_map → odom 은 항등(odom = 월드 = 발제 그대로), map 좌표만 지도 원점으로 바뀜
            x, y, yaw = a.mission_map
            mm = TransformStamped()
            mm.header.frame_id, mm.child_frame_id = "mission_map", "map"
            mm.transform.translation.x, mm.transform.translation.y = x, y
            mm.transform.rotation.z, mm.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            c, s_ = math.cos(yaw), math.sin(yaw)
            tf.transform.translation.x, tf.transform.translation.y = -(c * x + s_ * y), -(-s_ * x + c * y)
            tf.transform.rotation.z, tf.transform.rotation.w = -math.sin(yaw / 2), math.cos(yaw / 2)
            tfs.append(mm)
        self.static.sendTransform(tfs)
        self.now_s = a.t0
        self.mode = None

    def on_pucks(self, m):
        self.puck_msgs.append({"stamp": round(stamp_s(m.header), 6), "frame": m.header.frame_id, "pucks": [
            {"tag_id": p.tag_id, "color": COLORS.get(p.color), "shape": SHAPES.get(p.shape), "camera": CAMS.get(p.camera, p.camera),
             "position": [p.pose.position.x, p.pose.position.y, p.pose.position.z],
             "standing": p.standing if p.orientation_valid else None, "confidence": round(p.confidence, 3)} for p in m.pucks]})

    def service(self, name):
        cli = self.create_client(Trigger, f"/perception/survey/{name}")
        if not cli.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"/perception/survey/{name} 없음")
        fut = cli.call_async(Trigger.Request())
        while not fut.done():
            self.set_clock(self.now_s)
            self.spin_for(0.05)
        self.destroy_client(cli)
        r = fut.result()
        return {"success": r.success, "message": r.message}

    def ensure_mode(self, kind):
        m = SetPerceptionMode.Request.SEARCH_FRONT if kind == "d435" else SetPerceptionMode.Request.READ_TAG
        if m != self.mode:
            self.set_mode(m)
            self.mode = m

    def publish(self, meta, render, pose, t):
        st = to_time(t)
        child, T_child_bf = odom_child()
        T = np.array(pose["T_world_base_footprint"]) @ np.linalg.inv(T_child_bf)
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = st, "odom", child
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, T[:3, 3])
        q = quat_from_matrix(T[:3, :3])
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = map(float, q)
        self.tf_pub.publish(TFMessage(transforms=[tf]))
        js = JointState()
        js.header.stamp = st
        js.name = list(pose["arm_q"].keys())
        js.position = [float(v) for v in pose["arm_q"].values()]
        self.js_pub.publish(js)
        self.spin_for(self.a.settle)   # robot_state_publisher 가 이 시각 팔 TF 를 먼저 내게
        kind = pose["kind"]
        cam = meta[kind]
        K = cam["K"]
        img_pub, info_pub, dep_pub = self.pub[kind]
        info = CameraInfo()
        info.header.stamp, info.header.frame_id = st, cam["frame"]
        info.width, info.height = meta["width"], meta["height"]
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [K["fx"], 0.0, K["cx"], 0.0, K["fy"], K["cy"], 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [K["fx"], 0.0, K["cx"], 0.0, 0.0, K["fy"], K["cy"], 0.0, 0.0, 0.0, 1.0, 0.0]
        info_pub.publish(info)
        if dep_pub is not None:
            dep = np.load(render / pose["images"]["depth"]).astype(np.float32)
            dep[~np.isfinite(dep)] = 0.0
            d = Image()
            d.header.stamp, d.header.frame_id = st, cam["frame"]
            d.height, d.width = dep.shape
            d.encoding, d.step = "32FC1", dep.shape[1] * 4
            d.data = dep.tobytes()
            dep_pub.publish(d)   # 깊이를 컬러보다 먼저
        bgr = cv2.imread(str(render / pose["images"]["color"]), cv2.IMREAD_COLOR)
        c = Image()
        c.header.stamp, c.header.frame_id = st, cam["frame"]
        c.height, c.width = bgr.shape[:2]
        c.encoding, c.step = "bgr8", bgr.shape[1] * 3
        c.data = bgr.tobytes()
        img_pub.publish(c)

    def frame(self, meta, render, pose, t):
        """한 장 내고 그 시각 /perception/pucks 를 기다린다. 반환 (받은 메시지 또는 None, 다음 시각)."""
        self.ensure_mode(pose["kind"])
        self.publish(meta, render, pose, t)
        self.spin_for(self.a.settle)
        got, tc = None, t
        for _ in range(int(self.a.max_wait * FPS)):
            tc += 1.0 / FPS
            self.set_clock(tc)
            self.spin_for(self.a.settle)
            got = next((m for m in reversed(self.puck_msgs) if abs(m["stamp"] - t) < 1e-6), None)
            if got:
                break
        return got, tc + 1.0 / FPS


def judge_run(snapshot, finish, truth, obs, a):
    """등록부 칸과 정답 대조. obs = 이 회에 받은 /perception/pucks(관측 단위 자세 오류 세기)."""
    slots = {(s["box"], s["side"]): s for s in snapshot.get("slots", [])}
    cells = []
    for p in truth:
        s = slots.get((p["box"], p["side"]), {})
        want = {"id": p["tag_id"], "color": p["color"], "shape": p["shape"], "pose": "standing" if p["standing"] else "lying"}
        got = {k: s.get(k) for k in want}
        err_mm = math.hypot(s.get("x_mm", 1e9) - p["center"][0] * 1000, s.get("y_mm", 1e9) - p["center"][1] * 1000)
        cells.append({"box": p["box"], "side": p["side"], "name": p["name"], "want": want, "got": got,
                      "fields_ok": {k: got[k] == want[k] for k in want}, "err_mm": round(err_mm, 2),
                      "n_obs": s.get("n_obs"), "seen_by": s.get("seen_by"),
                      "ok": all(got[k] == want[k] for k in want) and err_mm <= a.max_err_mm})
    # 관측 단위: 정답 퍽 중심(map = 월드)과 3D 0.03 m 안의 관측의 서있음/누움
    obs_pose = {"n": 0, "wrong": 0, "unknown": 0, "wrong_examples": []}
    centers = {p["name"]: np.array(p["center"]) for p in truth}
    for m in obs:
        T = np.array(m["T_odom_frame"])
        for g in m["pucks"]:
            w = (T @ np.append(g["position"], 1.0))[:3]
            name, d = min(((n, float(np.linalg.norm(w - c))) for n, c in centers.items()), key=lambda x: x[1])
            if d > 0.03:
                continue
            p = next(x for x in truth if x["name"] == name)
            obs_pose["n"] += 1
            if g["standing"] is None:
                obs_pose["unknown"] += 1
            elif g["standing"] != p["standing"]:
                obs_pose["wrong"] += 1
                if len(obs_pose["wrong_examples"]) < 10:
                    obs_pose["wrong_examples"].append({"stamp": m["stamp"], "camera": g["camera"], "puck": name, "got_standing": g["standing"]})
    lying = [c for c in cells if c["want"]["pose"] == "lying"]
    return {"cells": cells, "cells_ok": sum(c["ok"] for c in cells), "finish": finish, "complete": snapshot.get("complete"),
            "errors": snapshot.get("errors"), "rejected": snapshot.get("rejected"),
            "wrong_complete": bool(finish["success"]) and not all(c["ok"] for c in cells),
            "pass": bool(finish["success"]) and bool(snapshot.get("complete")) and all(c["ok"] for c in cells),
            "p4": {"slot_pose_wrong": sum(c["got"]["pose"] is not None and not c["fields_ok"]["pose"] for c in cells),
                   "slot_pose_missing": sum(c["got"]["pose"] is None for c in cells),
                   "lying_cells": len(lying), "lying_ok": sum(c["fields_ok"]["pose"] for c in lying),
                   "max_err_mm": max(c["err_mm"] for c in cells), "obs": obs_pose}}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--render", type=Path, nargs="+", required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--lights", type=float, nargs="+", default=None, help="이 조명만 (기본 렌더의 전부)")
    ap.add_argument("--t0", type=float, default=1000.0)
    ap.add_argument("--qos-depth", type=int, default=10)
    ap.add_argument("--settle", type=float, default=0.05, help="[s 벽시계] 발행 뒤 처리 기다림")
    ap.add_argument("--max-wait", type=float, default=1.0, help="[s 모의 시계] 한 장 결과 기다림")
    ap.add_argument("--max-err-mm", type=float, default=10.0)
    ap.add_argument("--mission-map", type=float, nargs=3, default=None, metavar=("X", "Y", "YAW"),
                    help="mission_map → map 정적 TF 값(SLAM align.json). 주면 map → odom 은 그 역(기본: map → odom 항등만)")
    a = ap.parse_args()
    rclpy.init()
    n = SurveyReplayer(a)
    child, T_child_bf = odom_child()
    n.set_clock(a.t0)
    for _ in range(100):   # 등록부 노드가 뜨고 첫 상태를 내기까지
        n.set_clock(n.now_s)
        n.spin_for(0.1)
        if n.survey:
            break
    if not n.survey:
        raise RuntimeError("/perception/puck_survey 없음")
    runs_f = (a.out / "b0_runs.jsonl").open("w")
    pucks_f = (a.out / "pucks.jsonl").open("w")
    rows = []
    t = a.t0 + 1.0
    for render in a.render:
        meta = json.loads((render / "render_b0.json").read_text())
        for level in (a.lights or meta["lights"]):
            for world, truth in meta["truth"].items():
                poses = [p for p in meta["poses"] if p["world"] == world and p["light_level"] == level]
                if not poses:
                    continue
                n.mode = None
                n.set_mode(0)
                start = n.service("start")
                old_sid = n.survey.get("survey_id")
                for _ in range(50):
                    n.set_clock(n.now_s)
                    n.spin_for(0.05)
                    if n.survey.get("enabled"):
                        break
                n.puck_msgs.clear()
                frames = []
                for pose in poses:
                    got, t2 = n.frame(meta, render, pose, t)
                    T = np.array(pose["T_world_base_footprint"])
                    frames.append({"name": pose["name"], "kind": pose["kind"], "stamp": round(t, 6), "answered": got is not None,
                                   "n_pucks": len(got["pucks"]) if got else 0})
                    if got:
                        # 결과 frame(base_link 등) → odom: 차체 자리 × (뿌리 → 그 frame). 인지 노드 출력 frame 은 base_link 로 가정하고 확인
                        got["T_odom_frame"] = (T @ np.linalg.inv(T_child_bf)).tolist() if got["frame"] == child else None
                    t = t2
                n.spin_for(0.3)
                for _ in range(10):
                    n.set_clock(t)
                    n.spin_for(0.05)
                snapshot = json.loads(json.dumps(n.survey))
                finish = n.service("finish")
                obs = [m for m in n.puck_msgs if m.get("T_odom_frame")]
                bad_frame = sorted({m["frame"] for m in n.puck_msgs if not m.get("T_odom_frame")})
                res = judge_run(snapshot, finish, truth, obs, a)
                res.update({"render": str(render), "world": world, "light_level": level, "start": start,
                            "survey_id": snapshot.get("survey_id"), "prev_survey_id": old_sid, "frames": frames,
                            "unanswered": sum(not f["answered"] for f in frames), "puck_frames_other_frame": bad_frame,
                            "snapshot": snapshot})
                rows.append(res)
                runs_f.write(json.dumps(res, ensure_ascii=False) + "\n")
                runs_f.flush()
                for m in n.puck_msgs:
                    pucks_f.write(json.dumps({**m, "world": world, "light_level": level}, ensure_ascii=False) + "\n")
                print(Path(world).name, "L", level, "칸", res["cells_ok"], "/ 12", "finish", finish["success"], "complete", res["complete"],
                      "틀린 완료", res["wrong_complete"], "무응답", res["unanswered"], "P4", json.dumps(res["p4"], ensure_ascii=False)[:200],
                      "errors", (res["errors"] or [])[:6], flush=True)
                n.service("reset")
    s = {"runs": len(rows), "pass": sum(r["pass"] for r in rows), "cells_ok": sum(r["cells_ok"] for r in rows),
         "cells": 12 * len(rows), "wrong_complete": sum(r["wrong_complete"] for r in rows),
         "finish_success": sum(bool(r["finish"]["success"]) for r in rows), "unanswered_frames": sum(r["unanswered"] for r in rows),
         "p4_slot_pose_wrong": sum(r["p4"]["slot_pose_wrong"] for r in rows), "p4_slot_pose_missing": sum(r["p4"]["slot_pose_missing"] for r in rows),
         "p4_lying_cells": sum(r["p4"]["lying_cells"] for r in rows), "p4_lying_ok": sum(r["p4"]["lying_ok"] for r in rows),
         "p4_obs": {"n": sum(r["p4"]["obs"]["n"] for r in rows), "wrong": sum(r["p4"]["obs"]["wrong"] for r in rows),
                    "unknown": sum(r["p4"]["obs"]["unknown"] for r in rows)},
         "max_err_mm": max((r["p4"]["max_err_mm"] for r in rows), default=None),
         "field_fail": {k: sum(not c["fields_ok"][k] for r in rows for c in r["cells"]) for k in ("id", "color", "shape", "pose")}}
    s["all_pass"] = s["runs"] > 0 and s["pass"] == s["runs"] and s["wrong_complete"] == 0
    (a.out / "summary.json").write_text(json.dumps(s, indent=1, ensure_ascii=False))
    print(json.dumps(s, ensure_ascii=False), flush=True)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
