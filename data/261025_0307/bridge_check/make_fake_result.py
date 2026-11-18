"""analyze_bridge.py 도구 확인용 가짜 결과 폴더를 만든다 (Isaac Sim 없이, 표준 라이브러리만).

사용: python3 make_fake_result.py <폴더> good|bad
  good: 모든 값이 장면 설정과 같음 → 모든 단계 통과여야 한다
  bad:  CameraInfo K/P가 한 칸 밀림, scan intensities 길이 틀림, TwistStamped 못 받음 → 그 항목만 실패여야 한다
"""

import csv
import json
import math
import os
import sys

out, mode = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)
fx = 640 * 1.93 / 3.896
W, H = 640, 480


def yaw_q(deg):
    return (0.0, 0.0, math.sin(math.radians(deg) / 2), math.cos(math.radians(deg) / 2))


bq, mq, oq = yaw_q(90), yaw_q(-45), yaw_q(30)
meta = {"dt": 1 / 60,
        "camera": {"width": W, "height": H, "encoding": "rgb8", "fx": fx, "fy": fx, "cx": 320.0, "cy": 240.0,
                   "frame_skip": 1, "baseline_m": 0.05, "frame_left": "camera_left", "frame_right": "camera_right"},
        "lidar": {"hz": 10, "wall_dist_m": 2.0, "frame": "lidar_link", "scan_rate_hz": 10},
        "tf": {"parent": "world", "child": "base_link", "translation": [1, 2, 0], "rotation_xyzw": bq},
        "tf2": {"parent": "world", "child": "marker_link", "translation": [-0.5, 0.25, 0.75], "rotation_xyzw": mq},
        "odom": {"frame": "odom", "child": "base_link", "position": [1.5, -0.75, 0.125], "orientation_xyzw": oq,
                 "linear": [0.25, -0.125, 0.0625], "angular": [0.5, -0.25, 0.125]},
        "skipped": []}
json.dump(meta, open(f"{out}/meta.json", "w"))
json.dump({"ROS_DISTRO": "lyrical"}, open(f"{out}/isaac_env.json", "w"))
json.dump({"/odom": {"linear": [0.25, -0.125, 0.0625], "angular": [0.5, -0.25, 0.125],
                     "pose_cov_len": 36, "twist_cov_len": 36}}, open(f"{out}/first_msgs.json", "w"))
json.dump({"twist_lin": [0.25, -0.125, 0.0625], "twist_ang": [0.5, -0.75, 1.0],
           "stamped_lin": [0.375, -0.0625, 0.03125], "stamped_ang": [-0.25, 0.125, -1.5],
           "stamped_frame": "base_footprint"}, open(f"{out}/observer_info.json", "w"))
open(f"{out}/proc_maps.txt", "w").write("".join(f"7f 00 00 0 0 /opt/ros/lyrical/lib/{x}\n"
                                                for x in ("librcl.so", "librmw.so", "librmw_cyclonedds_cpp.so")))
open(f"{out}/isaac_stdout.txt", "w").write(
    "System rclpy loaded for ROS distro 'lyrical'\n" if mode == "good"
    else "Could not import system rclpy for 'lyrical': No module named 'em'\n")
open(f"{out}/isaac_exit_code.txt", "w").write("0\n")

t0, dt, n = 1000.0, 1 / 60, 60 * 620
pb, pe = t0 + 20, t0 + 25
K = [fx, 0, 320, 0, fx, 240, 0, 0, 1]
with open(f"{out}/ticks.csv", "w", newline="") as ft, open(f"{out}/rx.csv", "w", newline="") as fr:
    tk, rx = csv.writer(ft), csv.writer(fr)
    tk.writerow(["wall_s", "sim_s", "state"])
    rx.writerow(["topic", "rx_wall_s", "stamp_s", "frame_id"])
    for i in range(n):
        sim = i * dt
        wall = t0 + sim + (5 if t0 + sim > pb else 0)
        if abs(t0 + sim - pb) < dt / 2:
            tk.writerow([pb, "", "pause_begin"])
            tk.writerow([pe, "", "pause_end"])
        tk.writerow([wall, sim, "play"])
        rx.writerow(["/clock", wall, sim, ""])
        rx.writerow(["/tf", wall, sim, "world", "base_link", 1, 2, 0, *bq])
        rx.writerow(["/tf", wall, sim, "world", "marker_link", -0.5, 0.25, 0.75, *mq])
        rx.writerow(["/odom", wall, sim, "odom", "base_link", 1.5, -0.75, 0.125, *oq])
        if i % 2 == 0:
            for side, tx in (("left", 0.0), ("right", -fx * 0.05)):
                rx.writerow([f"/{side}/image_raw", wall, sim, f"camera_{side}", W, H, "rgb8", W * 3, W * 3 * H])
                p = [fx, 0, 320, tx, 0, fx, 240, 0, 0, 0, 1, 0]
                k = K if mode == "good" else [0.0] + K[:-1]
                p = p if mode == "good" else [0.0] + p[:-1]
                rx.writerow([f"/{side}/camera_info", wall, sim, f"camera_{side}", W, H, k[0], k[2], k[4], k[5], p[3],
                             json.dumps({"model": "plumb_bob", "d": [0.0] * 5, "k": k,
                                         "r": [1, 0, 0, 0, 1, 0, 0, 0, 1], "p": p})])
        if i % 6 == 0:
            ni = 1800 if mode == "good" else 3
            rx.writerow(["/scan", wall, sim, "lidar_link", -math.pi, math.pi, 2 * math.pi / 1800, 1800, 2.0005,
                         0.1, 25.0, ni, ni, 0, 255])
with open(f"{out}/twist_rx.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["wall_s", "sim_s", "lx", "ly", "lz", "ax", "ay", "az"])
    w.writerow([t0, 0, 0.25, -0.125, 0.0625, 0.5, -0.75, 1.0])
with open(f"{out}/twist_stamped_rx.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["wall_s", "sim_s", "attr", "value"])
    if mode == "good":
        w.writerow([t0, 0, "outputs:header:frame_id", json.dumps("base_footprint")])
        w.writerow([t0, 0, "outputs:twist:linear", json.dumps([0.375, -0.0625, 0.03125])])
        for c, v in zip("xyz", [-0.25, 0.125, -1.5]):
            w.writerow([t0, 0, f"outputs:twist:angular:{c}", repr(v)])
