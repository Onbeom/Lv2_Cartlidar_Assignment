"""SLAM 비교 기록의 도구 확인 T1~T3 (계획 문서 4장). 판정 시험이 아니라 기록이 비교에 쓸 만한지 본다.

T1 적외선 좌·우: 같은 stamp 짝 비율(차 ≤ 1 ms), CameraInfo baseline = −P[3]/fx(오른쪽) ≈ 0.0500 m, 영상 밝기(평균·표준편차)
T2 기록 손실: UDP RcvbufErrors 증가 0, 토픽 개수가 주기 × 기록 시간과 맞는지(±2 %)
T3 오도메트리: 바퀴 오도메트리와 정답(truth.csv)의 이동 거리·회전 비교(첫 직진 구간, 전체)
사용(PC): source /opt/ros/lyrical/setup.bash && python3 check_record.py <결과 폴더>
"""
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

out = Path(sys.argv[1])
meta = json.load(open(out / "meta.json"))
counts = json.load(open(out / "scene_counts.json"))
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(out / "bag"), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
stamps = {}
n = {}
info_p = {}
bright = {"infra1": [], "infra2": []}
k = 0
while reader.has_next():
    topic, data, _ = reader.read_next()
    n[topic] = n.get(topic, 0) + 1
    if topic.endswith("image_rect_raw") or topic.endswith("camera_info"):
        msg = deserialize_message(data, get_message(types[topic]))
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        stamps.setdefault(topic, []).append(st)
        if topic.endswith("camera_info"):
            info_p[topic] = list(msg.p)
        elif n[topic] % 30 == 1:
            a = np.frombuffer(bytes(msg.data), np.uint8)
            bright["infra1" if "infra1" in topic else "infra2"].append((float(a.mean()), float(a.std())))
res = {"bag_counts": n}

# T1
l = np.array(stamps["/top_camera_front/infra1/image_rect_raw"])
r = np.array(stamps["/top_camera_front/infra2/image_rect_raw"])
rs = set(np.round(r, 4))
paired = np.mean([round(x, 4) in rs for x in l])
p2 = info_p["/top_camera_front/infra2/camera_info"]
baseline = -p2[3] / p2[0]
b1 = np.array(bright["infra1"])
res["T1"] = {"left_n": len(l), "right_n": len(r), "paired_ratio_le_1ms": float(paired), "baseline_m": baseline,
             "frame_dt_median_s": float(np.median(np.diff(l))), "frame_dt_max_s": float(np.max(np.diff(l))),
             "ir_mean_brightness": float(b1[:, 0].mean()), "ir_std_brightness": float(b1[:, 1].mean()),
             "pass": bool(paired >= 0.999 and abs(baseline - 0.0500) < 0.0005)}

# T2
rb = dict(row for row in csv.reader(open(out / "rcvbuf_errors.csv")))
d_rcv = int(rb["after"]) - int(rb["before"])
dur = counts["sim_s"]
expect = {"/scan_front": 10, "/scan_rear": 10, "/odom": 30, "/joint_states": 50,
          "/top_camera_front/infra1/image_rect_raw": 30, "/top_camera_front/infra2/image_rect_raw": 30}
rate_ok = {t: n.get(t, 0) / (hz * dur) for t, hz in expect.items()}
sent = {"/scan_front": counts["scan_sent"]["/scan_front"], "/odom": counts["odom_sent"], "/joint_states": counts["js_sent"],
        "/top_camera_front/infra1/image_rect_raw": counts["ir_sent"]["infra1"]}
res["T2"] = {"rcvbuf_errors_delta": d_rcv, "count_over_expected": rate_ok,
             "recorded_over_sent": {t: n.get(t, 0) / max(s, 1) for t, s in sent.items()},
             "pass": bool(d_rcv == 0 and all(abs(v - 1) <= 0.02 for v in rate_ok.values())
                          and all(n.get(t, 0) >= s - 2 for t, s in sent.items()))}

# T3
rows = list(csv.DictReader(open(out / "truth.csv")))
T = np.array([[float(x[c]) for c in ("sim_s", "x", "y", "yaw", "odom_x", "odom_y", "odom_yaw")] for x in rows])
x0, y0, yaw0 = T[0, 1], T[0, 2], T[0, 3]
c, s = math.cos(-yaw0), math.sin(-yaw0)
tx = c * (T[:, 1] - x0) - s * (T[:, 2] - y0)       # 정답을 시작 자세 기준(= odom 원점)으로
ty = s * (T[:, 1] - x0) + c * (T[:, 2] - y0)
tyaw = np.unwrap(T[:, 3] - yaw0)
oyaw = np.unwrap(T[:, 6])
path_truth = float(np.sum(np.hypot(np.diff(T[:, 1]), np.diff(T[:, 2]))))
path_odom = float(np.sum(np.hypot(np.diff(T[:, 4]), np.diff(T[:, 5]))))
turn_truth = float(np.sum(np.abs(np.diff(tyaw))))
turn_odom = float(np.sum(np.abs(np.diff(oyaw))))
res["T3"] = {"path_truth_m": path_truth, "path_odom_m": path_odom, "path_ratio": path_odom / max(path_truth, 1e-9),
             "turn_truth_deg": math.degrees(turn_truth), "turn_odom_deg": math.degrees(turn_odom),
             "turn_ratio": turn_odom / max(turn_truth, 1e-9),
             "end_pos_err_m": float(math.hypot(T[-1, 4] - tx[-1], T[-1, 5] - ty[-1])),
             "end_yaw_err_deg": math.degrees(float(oyaw[-1] - tyaw[-1]))}
# 직진·제자리 회전 구간 속도 비교(직진: 명령 v > 0.15, 회전: |명령 w| > 0.3)
ph = np.array([x["phase"] for x in rows])
cv = np.array([float(x["cmd_v"]) for x in rows])
cw = np.array([float(x["cmd_w"]) for x in rows])
dt = np.diff(T[:, 0])
go = (ph[1:] == "go") & (cv[1:] > 0.15)
tr = (ph[1:] == "turn") & (np.abs(cw[1:]) > 0.3)
v_t = np.hypot(np.diff(T[:, 1]), np.diff(T[:, 2])) / dt
v_o = np.hypot(np.diff(T[:, 4]), np.diff(T[:, 5])) / dt
w_t = np.abs(np.diff(np.unwrap(T[:, 3]))) / dt
w_o = np.abs(np.diff(oyaw)) / dt
res["T3"].update({"go_speed_truth_med": float(np.median(v_t[go])), "go_speed_odom_med": float(np.median(v_o[go])),
                  "turn_rate_truth_med": float(np.median(w_t[tr])), "turn_rate_odom_med": float(np.median(w_o[tr])),
                  "pass_straight": bool(abs(np.median(v_o[go]) / np.median(v_t[go]) - 1) <= 0.02)})
json.dump(res, open(out / "check.json", "w"), indent=2, ensure_ascii=False)
print(json.dumps({k: v for k, v in res.items() if k != "bag_counts"}, indent=1, ensure_ascii=False))
