"""A1 팔 정렬 목표 유지 재생 · 판정 (검토 문서 6번, 모의): render_a1.py 결과를 장면마다 인지 노드에 넣는다.

장면마다: set_mode OFF → SEARCH_FRONT 로 D435 영상(같은 장면 영상, 시각만 새로) --d435-frames 장 → 마지막 object_pose 에
  가장 가까운 정답 퍽(odom = 월드, 0.05 m 안)을 '추적 목표'로 본다 → SEARCH_FRONT | ARM_ALIGN 으로 켜고 팔 영상을 한 장씩
  (D435 영상도 같은 시각으로 함께) 낸다. 장마다 그 시각의 /perception/result 가 올 때까지 시계를 민다(lockstep).
  TF: odom → base_footprint(로봇 자세, robot_state_publisher 가 URDF 고정 변환), odom → <--arm-frame>(팔 카메라 자세를
  직접. 렌더의 자유 카메라는 URDF 관절로 닿지 않는 자세라 URDF 의 팔 광학 프레임을 쓰지 않음).
판정(사용자 승인 2026-10-27, J1~J4):
  가리킨 퍽 = arm_offset(x, y)에 정답 상자 중심이 가장 가까운 퍽(보이는 퍽 중, 0.15 안. 없으면 '퍽 아님')
  J1 첫 잠금 = 추적 목표: 처음 offset_valid 인 장이 추적 목표를 가리킨 장면 100 %
  J2 목표가 다 보이는 장(in_frac = 1, vis ≥ 0.5)에서 offset_valid 이고 정답 상자 중심과 거리(정규화) ≤ 0.05 인 장 ≥ 95 %
  J3 다른 퍽을 가리킨 장 0
  J4 목표가 다 화면 밖인 장(in_frac = 0)에서 offset_valid = false 100 %, out 장면에서 다시 다 보인 뒤 3장 안에
     목표를 다시 가리킨 장면 ≥ 95 %
  추적 목표가 렌더 때 예상과 다르면 그 장면은 판정에서 빼고 따로 센다.
결과: <out>/a1_rows.jsonl(장마다), <out>/a1_scenes.jsonl, <out>/summary.json

실행 (PC, run_replay.sh 가 노드를 띄운 뒤 부른다):
  REPLAY=replay_a1.py Cartlidar/sim_render/run_replay.sh <URDF> <결과> <setup.bash> -- --render <render_a1 결과>
"""
import sys
sys.path.insert(0,"/home/pa4/git/lv2_next_module/Cartlidar/sim_render")
import argparse
import json
import math
from pathlib import Path

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_msgs.msg import TFMessage

from robot_interfaces.msg import PerceptionResult
from robot_interfaces.srv import SetPerceptionMode

from replay_ros import FPS, Replayer, quat_from_matrix, stamp_s, to_time

BASE_LINK_Z = 0.00675   # URDF base_footprint → base_link
M = SetPerceptionMode.Request


def tf_msg(T, parent, child, st):
    tf = TransformStamped()
    tf.header.stamp, tf.header.frame_id, tf.child_frame_id = st, parent, child
    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, T[:3, 3])
    q = quat_from_matrix(T[:3, :3])
    tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = map(float, q)
    return tf


def info_msg(K, w, h, st, frame):
    info = CameraInfo()
    info.header.stamp, info.header.frame_id = st, frame
    info.width, info.height = w, h
    info.distortion_model = "plumb_bob"
    info.d = [0.0] * 5
    info.k = [K["fx"], 0.0, K["cx"], 0.0, K["fy"], K["cy"], 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [K["fx"], 0.0, K["cx"], 0.0, 0.0, K["fy"], K["cy"], 0.0, 0.0, 0.0, 1.0, 0.0]
    return info


def img_msg(arr, enc, st, frame):
    m = Image()
    m.header.stamp, m.header.frame_id = st, frame
    m.height, m.width = arr.shape[:2]
    m.encoding = enc
    m.step = arr.shape[1] * (4 if enc == "32FC1" else 3)
    m.data = arr.tobytes()
    return m


class ArmReplayer(Replayer):
    def __init__(self, a):
        Node.__init__(self, "arm_replayer_a1")
        self.a = a
        self.meta = json.loads((a.render / "render_a1.json").read_text())
        rel = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=a.qos_depth, reliability=ReliabilityPolicy.RELIABLE)
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 100)
        self.color_pub = self.create_publisher(Image, "/top_camera_front/color/image_raw", rel)
        self.depth_pub = self.create_publisher(Image, "/top_camera_front/aligned_depth_to_color/image_raw", rel)
        self.info_pub = self.create_publisher(CameraInfo, "/top_camera_front/color/camera_info", rel)
        self.arm_pub = self.create_publisher(Image, "/arm_camera/image_raw", rel)
        self.arm_info_pub = self.create_publisher(CameraInfo, "/arm_camera/camera_info", rel)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.pending_joint = None
        self.joint_timer = self.create_timer(.01, self.publish_pending_joint)
        self.results = []
        self.create_subscription(PerceptionResult, "/perception/result", self.on_result,
                                 QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
        self.mode_cli = self.create_client(SetPerceptionMode, "/perception/set_mode")
        self.now_s = a.t0

    def on_result(self, m):
        self.results.append({"stamp": round(stamp_s(m.header), 6), "mode": m.mode, "object_found": m.object_found,
                             "object": [m.object_pose.pose.position.x, m.object_pose.pose.position.y,
                                        m.object_pose.pose.position.z],
                             "offset_valid": m.offset_valid, "offset": [m.arm_offset.x, m.arm_offset.y, m.arm_offset.z],
                             "confidence": round(m.confidence, 3)})

    def publish_pending_joint(self):
        if self.pending_joint and self.now_s >= self.pending_joint[0]:
            self.joint_pub.publish(self.pending_joint[1]); self.pending_joint = None

    def publish(self, sc, t, arm=None):
        st = to_time(t)
        tfs = [tf_msg(np.array(sc["T_world_base_link"]), "odom", "base_link", st)]
        joints = JointState(); joints.header.stamp = st
        q = arm["arm_q"] if arm is not None else sc["initial_q"]
        joints.name, joints.position = list(q), list(map(float,q.values()))
        if arm is not None and self.a.joint_delay_s > 0:
            self.pending_joint = (t+self.a.joint_delay_s, joints)
        else:
            self.joint_pub.publish(joints)
        self.tf_pub.publish(TFMessage(transforms=tfs))
        d = self.meta["d435"]
        frame, W, H = d["frame"], self.meta["width"], self.meta["height"]
        self.info_pub.publish(info_msg(d["K"], W, H, st, frame))
        self.depth_pub.publish(img_msg(self.d435[1], "32FC1", st, frame))   # 깊이를 컬러보다 먼저
        self.color_pub.publish(img_msg(self.d435[0], "bgr8", st, frame))
        if arm is not None:
            self.arm_info_pub.publish(info_msg(self.meta["arm"]["K"], self.meta["arm"]["width"], self.meta["arm"]["height"], st, self.a.arm_frame))
            bgr = cv2.imread(str(self.a.render / arm["image"]), cv2.IMREAD_COLOR)
            self.arm_pub.publish(img_msg(bgr, "bgr8", st, self.a.arm_frame))

    def step(self, sc, t, arm=None):
        """한 장 내고 그 시각 결과가 올 때까지 시계를 민다. (결과 또는 None, 다음 시각)"""
        self.publish(sc, t, arm)
        self.spin_for(self.a.settle)
        tc = t
        for _ in range(int(self.a.max_wait * FPS)):
            tc += 1.0 / FPS
            self.set_clock(tc)
            self.spin_for(self.a.settle)
            got = next((r for r in reversed(self.results) if abs(r["stamp"] - t) < 1e-6), None)
            if got:
                return got, tc + 1.0 / FPS
        return None, tc + 1.0 / FPS


def pointed(offset, pucks, gate=0.15):
    """arm_offset 에 정답 상자 중심이 가장 가까운 퍽 이름과 거리. gate 밖이면 (None, 거리)."""
    best = (None, math.inf)
    for n, p in pucks.items():
        if p["box"] is None:
            continue
        d = math.hypot(offset[0] - p["box"][0], offset[1] - p["box"][1])
        if d < best[1]:
            best = (n, d)
    return best if best[1] <= gate else (None, best[1])


def run_scene(n, a, sc, t):
    n.d435 = (cv2.imread(str(a.render / sc["d435_images"]["color"]), cv2.IMREAD_COLOR),
              np.nan_to_num(np.load(a.render / sc["d435_images"]["depth"]).astype(np.float32), nan=0.0, posinf=0.0,
                            neginf=0.0))
    if a.front_depth_empty:
        n.d435 = (n.d435[0], np.zeros_like(n.d435[1]))
    n.set_mode(M.OFF)
    n.set_mode(M.SEARCH_FRONT)
    obj = None
    search_records = []
    for _ in range(a.d435_frames):
        got, t = n.step(sc, t)
        search_records.append(got)
        if got and got["object_found"]:
            obj = got["object"]
    # 추적 목표: object_pose(base_link) → 월드
    T_bl = np.array(sc["T_world_base_link"])
    tracked, tracked_err = None, None
    if obj is not None:
        w = (T_bl @ np.append(obj, 1.0))[:3]
        d = {p["name"]: float(np.linalg.norm(np.array(p["center"]) - w)) for p in sc["pucks"]}
        nm = min(d, key=d.get)
        tracked, tracked_err = (nm if d[nm] <= 0.05 else None), round(d[nm], 4)
    n.set_mode(M.SEARCH_FRONT | M.ARM_ALIGN)
    rows = []
    for f in sc["arm_frames"]:
        got, t = n.step(sc, t, f)
        tg = f["pucks"].get(sc["target"], {"in_frac": 0.0, "vis": 0.0, "box": None})
        row = {"scene": sc["name"], "k": f["k"], "stage": f["stage"], "answered": got is not None,
               "offset_valid": bool(got and got["offset_valid"]),
               "offset": [round(v, 4) for v in got["offset"]] if got and got["offset_valid"] else None,
               "target_in_frac": tg["in_frac"], "target_vis": tg["vis"], "target_box": tg["box"],
               "neighbor_in_frac": f["pucks"].get(sc["neighbor"], {}).get("in_frac", 0.0)}
        row["seed_object"] = got["object"] if got and got["object_found"] else None
        if row["seed_object"] is not None:
            seed_cam = np.linalg.inv(np.array(f["T_world_cam"])) @ T_bl @ np.append(row["seed_object"],1.)
            if seed_cam[2] > 0:
                K_ = n.meta["arm"]["K"]
                row["seed_projected"] = [float((K_["fx"]*seed_cam[0]/seed_cam[2]+K_["cx"]-n.meta["arm"]["width"]/2)/(n.meta["arm"]["width"]/2)),
                                          float((K_["fy"]*seed_cam[1]/seed_cam[2]+K_["cy"]-n.meta["arm"]["height"]/2)/(n.meta["arm"]["height"]/2))]
        if row["offset_valid"]:
            nm, dist = pointed(got["offset"], f["pucks"])
            row["pointed"], row["pointed_dist"] = nm, round(dist, 4)
            row["target_err"] = (round(math.hypot(got["offset"][0] - tg["box"][0], got["offset"][1] - tg["box"][1]), 4)
                                 if tg["box"] else None)
        rows.append(row)
    n.set_mode(M.OFF)
    first = next((r for r in rows if r["offset_valid"]), None)
    reacq = None
    if sc["kind"] == "out":
        k_back = next((r["k"] for r in rows if r["k"] > 0 and r["target_in_frac"] == 1.0 and
                       any(q["target_in_frac"] == 0.0 for q in rows if q["k"] < r["k"])), None)
        if k_back is not None:
            reacq = any(r["offset_valid"] and r["pointed"] == sc["target"] for r in rows if k_back <= r["k"] < k_back + 3)
    scene = {"scene": sc["name"], "kind": sc["kind"], "light_level": sc["light_level"], "planned_target": sc["target"],
             "tracked": tracked, "tracked_err_m": tracked_err, "valid": tracked == sc["target"],
             "search_records": search_records,
             "neighbor": sc["neighbor"], "neighbor_same_color": sc["neighbor_same_color"],
             "first_lock_k": first["k"] if first else None, "first_lock": first["pointed"] if first else None,
             "reacquired": reacq}
    return rows, scene, t


def rate(k, n):
    return {"ok": k, "n": n, "rate": round(k / n, 4) if n else None}


def summarize(rows, scenes, a):
    valid = {s["scene"] for s in scenes if s["valid"]}
    R = [r for r in rows if r["scene"] in valid]
    S = [s for s in scenes if s["valid"]]
    tgt = {s["scene"]: s["planned_target"] for s in S}
    full = [r for r in R if r["target_in_frac"] == 1.0 and r["target_vis"] >= 0.5]
    j2_ok = [r for r in full if r["offset_valid"] and r["target_err"] is not None and r["target_err"] <= a.j2_tol]
    wrong = [r for r in R if r["offset_valid"] and r["pointed"] not in (None, tgt[r["scene"]])]
    notpuck = [r for r in R if r["offset_valid"] and r["pointed"] is None]
    out = [r for r in R if r["target_in_frac"] == 0.0]
    outs = [s for s in S if s["kind"] == "out" and s["reacquired"] is not None]
    s = {"scenes": len(scenes), "valid_scenes": len(S), "invalid": [(x["scene"], x["planned_target"], x["tracked"],
                                                                      x["tracked_err_m"]) for x in scenes if not x["valid"]],
         "frames": len(R), "unanswered": sum(not r["answered"] for r in R),
         "J1_first_lock_target": rate(sum(x["first_lock"] == x["planned_target"] for x in S), len(S)),
         "J1_fail": [(x["scene"], x["first_lock_k"], x["first_lock"]) for x in S if x["first_lock"] != x["planned_target"]],
         "J2_full_frames": rate(len(j2_ok), len(full)), "J2_tol": a.j2_tol,
         "J2_err_max": max((r["target_err"] for r in j2_ok), default=None),
         "J2_fail": [(r["scene"], r["k"], r["offset_valid"], r.get("target_err"), r.get("pointed")) for r in full
                     if r not in j2_ok][:40],
         "J3_other_puck_frames": len(wrong),
         "J3_examples": [(r["scene"], r["k"], r["stage"], r["pointed"]) for r in wrong][:40],
         "not_puck_frames": len(notpuck),
         "J4_out_frames_invalid": rate(sum(not r["offset_valid"] for r in out), len(out)),
         "J4_out_fail": [(r["scene"], r["k"], r.get("pointed")) for r in out if r["offset_valid"]][:40],
         "J4_reacquire": rate(sum(bool(x["reacquired"]) for x in outs), len(outs)),
         "J4_reacquire_fail": [x["scene"] for x in outs if not x["reacquired"]],
         "partial_frames": sum(0.0 < r["target_in_frac"] < 1.0 for r in R),
         "partial_pointing_target": sum(0.0 < r["target_in_frac"] < 1.0 and r["offset_valid"] and
                                        r.get("pointed") == tgt[r["scene"]] for r in R)}
    s["J1_pass"] = bool(S) and s["J1_first_lock_target"]["ok"] == len(S)
    s["J2_pass"] = bool(full) and s["J2_full_frames"]["rate"] >= 0.95
    s["J3_pass"] = s["J3_other_puck_frames"] == 0
    s["J4_pass"] = (bool(out) and s["J4_out_frames_invalid"]["ok"] == len(out) and bool(outs) and
                    s["J4_reacquire"]["rate"] >= 0.95)
    s["pass"] = all(s[k] for k in ("J1_pass", "J2_pass", "J3_pass", "J4_pass"))
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--render", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--arm-frame", default="arm_camera_optical_frame")
    ap.add_argument("--joint-delay-s", type=float, default=0.)
    ap.add_argument("--front-depth-empty", action="store_true")
    ap.add_argument("--d435-frames", type=int, default=5)
    ap.add_argument("--j2-tol", type=float, default=0.05)
    ap.add_argument("--t0", type=float, default=1000.0)
    ap.add_argument("--start-delay", type=float, default=1.0)
    ap.add_argument("--qos-depth", type=int, default=10)
    ap.add_argument("--settle", type=float, default=0.05)
    ap.add_argument("--max-wait", type=float, default=1.0)
    ap.add_argument("--only", nargs="*")
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    n = ArmReplayer(a)
    n.set_clock(a.t0)
    n.spin_for(1.0)
    rows, scenes = [], []
    t = a.t0 + a.start_delay
    try:
        for sc in n.meta["scenes"]:
            if a.only and not any(s in sc["name"] for s in a.only):
                continue
            r, s, t = run_scene(n, a, sc, t)
            rows += r
            scenes.append(s)
            print(sc["name"], "추적", s["tracked"], "(예상", s["planned_target"] + ")", "첫 잠금", s["first_lock_k"],
                  s["first_lock"], "다시 잡음", s["reacquired"], "다른 퍽",
                  sum(x["offset_valid"] and x["pointed"] not in (None, sc["target"]) for x in r), flush=True)
    finally:
        try:
            n.set_mode(M.OFF)
        except Exception as e:  # noqa: BLE001
            print("set_mode OFF 실패:", e)
    summary = summarize(rows, scenes, a)
    summary["d435_extrinsic_mode"] = "new_model_fk_exact_no_jitter"
    summary["joint_state_delay_s"] = a.joint_delay_s
    summary["front_depth_empty"] = a.front_depth_empty
    summary["render"] = str(a.render)
    with open(a.out / "a1_rows.jsonl", "w") as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")
    with open(a.out / "a1_scenes.jsonl", "w") as f:
        for s in scenes:
            f.write(json.dumps(s, ensure_ascii=False) + "\n")
    (a.out / "summary.json").write_text(json.dumps(summary, indent=1, ensure_ascii=False))
    print(json.dumps(summary, ensure_ascii=False, indent=1))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
