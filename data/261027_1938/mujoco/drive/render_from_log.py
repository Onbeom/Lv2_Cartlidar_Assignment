"""결과 폴더의 log.csv(스텝마다 기록한 상태)를 model.xml에 다시 넣어 영상만 새로 그린다. 물리 계산은 하지 않는다.

log.csv에는 차체 위치·자세(roll·pitch·yaw)와 모든 관절 위치가 있어 qpos를 그대로 복원할 수 있다.
카메라: follow(차체를 따라감, mujoco_drive.py와 같음) 또는 fixed(바닥에 고정, 이동 거리가 보임).

실행(PC): MUJOCO_GL=egl .venv/bin/python drive/render_from_log.py <결과 폴더> <출력 mp4> [--camera fixed] [--lookat x y z]
"""
import argparse
import csv
import math
import os
import subprocess
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("folder", type=Path)
parser.add_argument("out", type=Path)
parser.add_argument("--camera", choices=["follow", "fixed"], default="follow")
parser.add_argument("--lookat", type=float, nargs=3, default=None, help="fixed 카메라 중심(m). 주지 않으면 처음과 끝 위치의 가운데")
parser.add_argument("--distance", type=float, default=1.1)
parser.add_argument("--azimuth", type=float, default=135.0)
parser.add_argument("--elevation", type=float, default=-28.0)
parser.add_argument("--fps", type=int, default=30)
parser.add_argument("--size", type=int, nargs=2, default=[640, 480], help="가로 세로(px)")
args = parser.parse_args()


def quat_from_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2), math.cos(y / 2), math.sin(y / 2)
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]


m = mujoco.MjModel.from_xml_path(str(args.folder / "model.xml"))
d = mujoco.MjData(m)
with (args.folder / "log.csv").open(encoding="utf-8") as fh:
    rows = list(csv.DictReader(fh))
joints = [k[2:] for k in rows[0] if k.startswith("q_")]
adr = {n: m.jnt_qposadr[m.joint(n).id] for n in joints}
root = m.jnt_qposadr[m.joint("root").id]
t = np.array([float(r["t_s"]) for r in rows])
bx, by = np.array([float(r["base_x_m"]) for r in rows]), np.array([float(r["base_y_m"]) for r in rows])
lookat = args.lookat or [float((bx[0] + bx[-1]) / 2), float((by[0] + by[-1]) / 2), 0.12]

W, H = args.size
renderer = mujoco.Renderer(m, H, W)
cam = mujoco.MjvCamera()
cam.type = mujoco.mjtCamera.mjCAMERA_FREE
cam.distance, cam.azimuth, cam.elevation = args.distance, args.azimuth, args.elevation
enc = subprocess.Popen(["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}",
                        "-r", str(args.fps), "-i", "-", "-c:v", "libx264", "-profile:v", "main", "-pix_fmt", "yuv420p",
                        "-crf", "20", "-movflags", "+faststart", str(args.out)], stdin=subprocess.PIPE)
# 0 s 상태(기록 첫 줄 이전)는 log에 없으므로 첫 줄부터 프레임 시각마다 가장 가까운 기록을 쓴다
for ft in np.arange(0.0, t[-1] + 1e-9, 1.0 / args.fps):
    r = rows[int(np.argmin(np.abs(t - ft)))]
    d.qpos[root:root + 3] = [float(r["base_x_m"]), float(r["base_y_m"]), float(r["base_z_m"])]
    d.qpos[root + 3:root + 7] = quat_from_rpy(float(r["roll_rad"]), float(r["pitch_rad"]), float(r["yaw_rad"]))
    for n in joints:
        d.qpos[adr[n]] = float(r["q_" + n])
    mujoco.mj_kinematics(m, d)
    mujoco.mj_camlight(m, d)  # 조명·카메라 위치(빠뜨리면 조명이 꺼진 것처럼 어둡다)
    cam.lookat[:] = lookat if args.camera == "fixed" else [float(r["base_x_m"]), float(r["base_y_m"]), 0.12]
    renderer.update_scene(d, camera=cam)
    enc.stdin.write(renderer.render().tobytes())
enc.stdin.close()
enc.wait()
renderer.close()
print(f"{args.out} ({args.camera}, {len(np.arange(0.0, t[-1] + 1e-9, 1.0 / args.fps))} frames)")
