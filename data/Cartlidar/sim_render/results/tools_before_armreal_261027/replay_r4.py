"""R4 퍽 재생 · 판정: render_r4.py 결과(render_r4.json)를 한 장씩 인지 노드에 넣고 /perception/pucks 를 기다린다.

d435 : 모드 SEARCH_FRONT. 앞 D435 컬러 · 깊이 · CameraInfo, odom → base_footprint TF.
arm  : 모드 READ_TAG. /arm_camera/image_raw · camera_info, 팔 자세 S0 의 /joint_states(robot_state_publisher 가 TF 로 바꿈),
       odom → base_footprint = 카메라 자세 × URDF(S0) 역변환.
짝 짓기: 장면마다 정답 퍽 중심(base_link)과 결과 퍽 중심의 3D 거리가 가까운 쌍부터 --gate(0.05 m) 안이면 짝(한 번씩만).
판정(인지 판정 기준 v1 R4, 2026-10-25 변경 반영 — 모양은 YOLO 단계):
  D435 : 대상 = 보이는 비율 ≥ --vis(0.5) 이고 base_footprint 에서 수평 1.0 m 이내인 퍽. 검출 = 짝이 있음 ≥ 95 %,
         색 = 짝의 색이 맞음 ≥ 98 %. 선반 · 바닥 퍽은 따로 센다. 정답 퍽과 짝이 없는 결과(헛검출)는 기록만.
  arm  : 대상 = 장면의 목표 퍽 중 위를 향한 태그가 다 보이는 것(top_tag_visible = 1). 번호 100 %. 틀린 번호(같은 번호의
         정답 퍽이 gate 밖) 0. 목표가 아닌 옆 퍽 · 색 · 모양은 기록만.
결과: <out>/r4_rows.jsonl(장면마다 정답 · 결과), <out>/summary.json

실행 (PC, run_replay.sh 가 노드를 띄운 뒤 부른다):
  REPLAY=replay_r4.py Cartlidar/sim_render/run_replay.sh <URDF> <결과> <setup.bash> -- --render <render_r4 결과> d435
"""
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

from robot_interfaces.msg import PuckArray
from robot_interfaces.srv import SetPerceptionMode

from replay_ros import FPS, Replayer, quat_from_matrix, stamp_s, to_time

BASE_LINK_Z = 0.00675   # URDF base_footprint → base_link
COLORS = {1: "red", 2: "green", 3: "blue"}
SHAPES = {1: "cylinder", 2: "cuboid"}


class PuckReplayer(Replayer):
    """Replayer 의 시계 · 모드 · spin 을 쓰고, 영상 · TF · 관절 발행과 퍽 결과 받기를 따로 둔다."""
    META = "render_r4.json"   # replay_r6.py 는 render_r6.json

    def __init__(self, a, name="puck_replayer_r4"):
        Node.__init__(self, name)
        self.a = a
        self.meta = json.loads((a.render / self.META).read_text())
        rel = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=a.qos_depth, reliability=ReliabilityPolicy.RELIABLE)
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 100)
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        if a.mode == "d435":
            self.color_pub = self.create_publisher(Image, "/top_camera_front/color/image_raw", rel)
            self.depth_pub = self.create_publisher(Image, "/top_camera_front/aligned_depth_to_color/image_raw", rel)
            self.info_pub = self.create_publisher(CameraInfo, "/top_camera_front/color/camera_info", rel)
        else:
            self.color_pub = self.create_publisher(Image, "/arm_camera/image_raw", rel)
            self.info_pub = self.create_publisher(CameraInfo, "/arm_camera/camera_info", rel)
        self.msgs = []
        self.puck_msgs = []
        self.create_subscription(PuckArray, "/perception/pucks", self.on_pucks,
                                 QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
        self.mode_cli = self.create_client(SetPerceptionMode, "/perception/set_mode")
        self.now_s = a.t0

    def on_pucks(self, m):
        self.puck_msgs.append({"stamp": round(stamp_s(m.header), 6), "frame": m.header.frame_id, "pucks": [
            {"tag_id": p.tag_id, "color": COLORS.get(p.color), "shape": SHAPES.get(p.shape), "camera": p.camera,
             "position": [p.pose.position.x, p.pose.position.y, p.pose.position.z],
             "standing": p.standing if p.orientation_valid else None, "confidence": round(p.confidence, 3)}
            for p in m.pucks]})

    def publish(self, pose, t):
        st = to_time(t)
        T = np.array(pose["T_world_base_footprint"])
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = st, "odom", "base_footprint"
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, T[:3, 3])
        q = quat_from_matrix(T[:3, :3])
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = map(float, q)
        self.tf_pub.publish(TFMessage(transforms=[tf]))
        cam = self.meta[self.a.mode]
        if self.a.mode == "arm":
            js = JointState()
            js.header.stamp = st
            js.name = list(self.meta["arm_q"].keys())
            js.position = [float(v) for v in self.meta["arm_q"].values()]
            self.js_pub.publish(js)
            self.spin_for(self.a.settle)   # robot_state_publisher 가 이 시각 팔 TF 를 먼저 내게
        K = cam["K"]
        info = CameraInfo()
        info.header.stamp, info.header.frame_id = st, cam["frame"]
        info.width, info.height = self.meta["width"], self.meta["height"]
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [K["fx"], 0.0, K["cx"], 0.0, K["fy"], K["cy"], 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [K["fx"], 0.0, K["cx"], 0.0, 0.0, K["fy"], K["cy"], 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)
        if self.a.mode == "d435":
            dep = np.load(self.a.render / pose["images"]["depth"]).astype(np.float32)
            dep[~np.isfinite(dep)] = 0.0
            d = Image()
            d.header.stamp, d.header.frame_id = st, cam["frame"]
            d.height, d.width = dep.shape
            d.encoding, d.step = "32FC1", dep.shape[1] * 4
            d.data = dep.tobytes()
            self.depth_pub.publish(d)   # 깊이를 컬러보다 먼저
        bgr = cv2.imread(str(self.a.render / pose["images"]["color"]), cv2.IMREAD_COLOR)
        c = Image()
        c.header.stamp, c.header.frame_id = st, cam["frame"]
        c.height, c.width = bgr.shape[:2]
        c.encoding, c.step = "bgr8", bgr.shape[1] * 3
        c.data = bgr.tobytes()
        self.color_pub.publish(c)


def truth_in_base_link(pose):
    T = np.array(pose["T_world_base_footprint"])
    T[:3, 3] += T[:3, :3] @ np.array([0, 0, BASE_LINK_Z])
    Ti = np.linalg.inv(T)
    return {p["name"]: (Ti @ np.append(p["center"], 1.0))[:3] for p in pose["pucks"]}


def match(truth, got, gate):
    """가까운 쌍부터 gate 안이면 짝. 반환 {정답 이름: 결과 번호}."""
    pairs = sorted((float(np.linalg.norm(np.array(g["position"]) - tp)), n, i)
                   for n, tp in truth.items() for i, g in enumerate(got))
    used_t, used_g, out = set(), set(), {}
    for d, n, i in pairs:
        if d > gate:
            break
        if n in used_t or i in used_g:
            continue
        used_t.add(n)
        used_g.add(i)
        out[n] = (i, d)
    return out


def run(n, a):
    mode_bit = SetPerceptionMode.Request.SEARCH_FRONT if a.mode == "d435" else SetPerceptionMode.Request.READ_TAG
    n.set_mode(mode_bit)
    poses = [p for p in n.meta["poses"] if p["id"].split("_")[1].startswith("d" if a.mode == "d435" else "a")]
    if a.only:
        poses = [p for p in poses if any(s in p["name"] for s in a.only)]
    rows = []
    t = a.t0 + a.start_delay
    for pose in poses:
        n.publish(pose, t)
        n.spin_for(a.settle)
        got, tc = None, t
        for _ in range(int(a.max_wait * FPS)):
            tc += 1.0 / FPS
            n.set_clock(tc)
            n.spin_for(a.settle)
            got = next((m for m in reversed(n.puck_msgs) if abs(m["stamp"] - t) < 1e-6), None)
            if got:
                break
        res = got["pucks"] if got else []
        truth = truth_in_base_link(pose)
        pairs = match(truth, res, a.gate)
        items = []
        for p in pose["pucks"]:
            m = pairs.get(p["name"])
            g = res[m[0]] if m else None
            items.append({"name": p["name"], "tag_id": p["tag_id"], "color": p["color"], "shape": p["shape"],
                          "standing": p["standing"], "location": p["location"], "visible": p["visible"],
                          "top_tag_visible": p.get("top_tag_visible"), "base_hdist_m": p["base_hdist_m"],
                          "cam_dist_m": p["cam_dist_m"], "matched": m is not None,
                          "err_m": round(m[1], 4) if m else None, "got": g})
        used = {m[0] for m in pairs.values()}
        extra = [g for i, g in enumerate(res) if i not in used]
        pos_by_tag = {p["tag_id"]: truth[p["name"]] for p in pose["pucks"]}
        wrong_ids = [g for g in res if g["tag_id"] >= 0 and (g["tag_id"] not in pos_by_tag or np.linalg.norm(
            np.array(g["position"]) - pos_by_tag[g["tag_id"]]) > a.gate)]
        rows.append({"pose": pose["name"], "seed": pose["seed"], "light_level": pose["light_level"],
                     "target": pose.get("target"), "shelf": pose.get("shelf"), "dist_nominal": pose.get("dist_nominal"),
                     "answered": got is not None, "n_result": len(res), "pucks": items, "unmatched_results": extra,
                     "wrong_ids": wrong_ids})
        t = tc + 1.0 / FPS
        if a.mode == "d435":
            jud = [i for i in items if i["visible"] >= a.vis and i["base_hdist_m"] <= 1.0]
            print(pose["name"], f"대상 {len(jud)} 검출 {sum(i['matched'] for i in jud)} 색 "
                  f"{sum(i['matched'] and i['got']['color'] == i['color'] for i in jud)} 헛검출 {len(extra)}", flush=True)
        else:
            tg = next(i for i in items if i["name"] == pose["target"])
            print(pose["name"], "목표", tg["tag_id"], "서 있음" if tg["standing"] else "누움", "태그 보임", tg["top_tag_visible"],
                  "→", tg["got"]["tag_id"] if tg["got"] else None, "틀린 번호", len(wrong_ids), flush=True)
    return rows


def rate(k, n):
    return {"ok": k, "n": n, "rate": round(k / n, 4) if n else None}


def summarize_d435(rows, a):
    s = {"mode": "d435", "frames": len(rows), "unanswered": sum(not r["answered"] for r in rows), "vis": a.vis,
         "gate_m": a.gate}
    allp = [(r, i) for r in rows for i in r["pucks"]]
    for loc in ("shelf", "floor"):
        for band, lo, hi in (("le1.0", 0.0, 1.0), ("gt1.0", 1.0, 9.0)):
            jud = [i for r, i in allp if (i["location"] == "floor") == (loc == "floor") and i["visible"] >= a.vis and
                   lo < i["base_hdist_m"] <= hi]
            det = [i for i in jud if i["matched"]]
            s[f"{loc}_{band}"] = {"detect": rate(len(det), len(jud)),
                                  "color": rate(sum(i["got"]["color"] == i["color"] for i in det), len(det)),
                                  "max_err_m": max((i["err_m"] for i in det), default=None),
                                  "by_light": {f"{lv:g}": rate(sum(i["matched"] for r, i in allp if r["light_level"] == lv and i in jud),
                                                               sum(1 for r, i in allp if r["light_level"] == lv and i in jud))
                                               for lv in sorted({r["light_level"] for r in rows})},
                                  "missed": [f"{r['pose']}:{i['name']}({i['visible']},{i['base_hdist_m']})" for r, i in allp
                                             if i in jud and not i["matched"]],
                                  "wrong_color": [f"{r['pose']}:{i['name']} {i['color']}->{i['got']['color']}" for r, i in allp
                                                  if i in det and i["got"]["color"] != i["color"]]}
    s["partly_visible_detect"] = rate(sum(i["matched"] for r, i in allp if 0 < i["visible"] < a.vis),
                                      sum(1 for r, i in allp if 0 < i["visible"] < a.vis))
    s["unmatched_results"] = sum(len(r["unmatched_results"]) for r in rows)
    s["unmatched_examples"] = [f"{r['pose']}:{g['color']}@{[round(v, 3) for v in g['position']]}" for r in rows
                               for g in r["unmatched_results"]][:30]
    for loc in ("shelf", "floor"):
        k = s[f"{loc}_le1.0"]
        s[f"{loc}_pass"] = bool(k["detect"]["n"] and k["detect"]["rate"] >= 0.95 and k["color"]["rate"] >= 0.98)
    return s


def summarize_arm(rows, a):
    s = {"mode": "arm", "frames": len(rows), "unanswered": sum(not r["answered"] for r in rows), "gate_m": a.gate}
    tg = [(r, next(i for i in r["pucks"] if i["name"] == r["target"])) for r in rows]
    jud = [(r, i) for r, i in tg if i["standing"] and (i["top_tag_visible"] or 0) >= 1.0]
    ok = [(r, i) for r, i in jud if i["got"] and i["got"]["tag_id"] == i["tag_id"]]
    s["target_id"] = rate(len(ok), len(jud))
    s["target_missed"] = [f"{r['pose']}:{i['name']}" for r, i in jud if (r, i) not in ok]
    s["target_err_max_m"] = max((i["err_m"] for r, i in ok), default=None)
    s["target_color"] = rate(sum(i["got"]["color"] == i["color"] for r, i in ok), len(ok))
    s["target_shape"] = rate(sum(i["got"]["shape"] == i["shape"] for r, i in ok), len(ok))
    s["target_shape_wrong"] = [f"{r['pose']}:{i['shape']}->{i['got']['shape']}" for r, i in ok if i["got"]["shape"] != i["shape"]]
    s["lying_targets"] = len([1 for r, i in tg if not i["standing"]])
    s["lying_target_read"] = sum(1 for r, i in tg if not i["standing"] and i["got"] and i["got"]["tag_id"] >= 0)
    nb = [(r, i) for r in rows for i in r["pucks"] if i["name"] != r["target"] and i["standing"] and
          (i["top_tag_visible"] or 0) >= 1.0]
    s["neighbor_full_tag_id"] = rate(sum(1 for r, i in nb if i["got"] and i["got"]["tag_id"] == i["tag_id"]), len(nb))
    s["wrong_ids"] = sum(len(r["wrong_ids"]) for r in rows)
    s["wrong_id_examples"] = [f"{r['pose']}:{g['tag_id']}" for r in rows for g in r["wrong_ids"]][:20]
    s["pass"] = bool(jud and len(ok) == len(jud) and s["wrong_ids"] == 0)
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--render", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--t0", type=float, default=1000.0)
    ap.add_argument("--start-delay", type=float, default=1.0)
    ap.add_argument("--qos-depth", type=int, default=10)
    ap.add_argument("--settle", type=float, default=0.05)
    ap.add_argument("--max-wait", type=float, default=1.0)
    ap.add_argument("--gate", type=float, default=0.05)
    ap.add_argument("--vis", type=float, default=0.5)
    ap.add_argument("--only", nargs="*")
    ap.add_argument("mode", choices=["d435", "arm"])
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    n = PuckReplayer(a)
    n.set_clock(a.t0)
    n.spin_for(1.0)
    try:
        rows = run(n, a)
    finally:
        try:
            n.set_mode(SetPerceptionMode.Request.OFF)
        except Exception as e:  # noqa: BLE001
            print("set_mode OFF 실패:", e)
    summary = summarize_d435(rows, a) if a.mode == "d435" else summarize_arm(rows, a)
    summary["render"] = str(a.render)
    with open(a.out / "r4_rows.jsonl", "w") as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")
    (a.out / "summary.json").write_text(json.dumps(summary, indent=1, ensure_ascii=False))
    print(json.dumps(summary, ensure_ascii=False, indent=1))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
