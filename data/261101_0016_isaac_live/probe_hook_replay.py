"""도구 확인(모의, 판정 아님): 설계 25 출발 통로 막힘을 미로 안에서 재현하고 링크별 위치를 기록한다.
probe_robot_width.py 결과: 서 있는 차체의 왼쪽 물리 폭 0.170 m(모델 0.168 과 같음) → 벽 끝이 로봇 좌표 왼쪽 0.210 에 걸린 것은
움직이는 동안 바깥으로 나오는 부분(팔 등)일 수 있음. 설계 25 t 22.5 s 참 자세(발제 (0.833, 0.471, −1.09))에 drive 자세로 놓고
scene_live.py 와 같은 바퀴 명령(제자리 회전 명령 w, v 0)을 주며 0.1 s 마다 root 자세 · 팔 관절 · 모든 링크 위치(로봇 좌표)를 기록.
사용: <IsaacLab python> probe_hook_replay.py --out <폴더> [--w 0.70] [--sim_s 6] [--start x y yaw(발제)]"""
import argparse
import json
import math
import os

from isaaclab.app import AppLauncher

HERE = os.path.dirname(os.path.abspath(__file__))
TOP = os.path.dirname(HERE)
MODEL = os.path.join(TOP, "261029_1030/urdf_261028/isaac")
ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--w", type=float, default=0.70)
ap.add_argument("--sim_s", type=float, default=6.0)
ap.add_argument("--start", type=float, nargs=3, default=[0.833, 0.471, -1.09])
ap.add_argument("--offset", type=float, nargs=2, default=[2.2345, 3.2136])
ap.add_argument("--maze", default=os.path.join(TOP, "261030_1943/maze_src/maze/maze.usda"))
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
args.headless = True
os.makedirs(args.out, exist_ok=True)
app = AppLauncher(args).app

import sys  # noqa: E402

sys.path.insert(0, MODEL)
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402

from waffle_4wd_isaac import robot_cfg as R  # noqa: E402
from waffle_4wd_isaac import sensors_cfg as S  # noqa: E402

ROBOT, DT = "/World/Robot", 1.0 / 120.0
sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=DT, device="cuda:0", physics_material=R.CONTACT_MATERIAL))
g = sim_utils.GroundPlaneCfg(physics_material=R.CONTACT_MATERIAL)
g.func("/World/ground", g)
mz = sim_utils.UsdFileCfg(usd_path=args.maze)
mz.func("/World/maze", mz)
x0, y0, yaw0 = args.start[0] - args.offset[0], args.start[1] - args.offset[1], args.start[2]
cfg = R.WAFFLE_4WD_CFG.replace(prim_path=ROBOT)
cfg.init_state.pos = (x0, y0, R.ROOT_HEIGHT + 0.003)
cfg.init_state.rot = (0.0, 0.0, math.sin(yaw0 / 2), math.cos(yaw0 / 2))
cfg.init_state.joint_pos = R.pose("drive")
robot = Articulation(cfg)
S.spawn_lidar_occluders(ROBOT)
sim.reset()
SEP, RAD = float(R.WHEEL_SEPARATION), float(R.WHEEL_RADIUS)
wl_ids, _ = robot.find_joints(R.WHEEL_JOINTS_LEFT, preserve_order=True)
wr_ids, _ = robot.find_joints(R.WHEEL_JOINTS_RIGHT, preserve_order=True)
wheels = list(wl_ids) + list(wr_ids)
hold_names = R.ARM_JOINTS + R.FINGER_JOINTS
hold, _ = robot.find_joints(hold_names, preserve_order=True)
dp = R.pose("drive")
target = torch.tensor([[dp[n] for n in hold_names]], dtype=torch.float32, device=sim.device)
cmd_buf = robot.actuators.target_command
lim = float(R.WHEEL_SPEED_MAX)
bodies = list(robot.body_names)
rows = []
n = int(args.sim_s / DT)
for i in range(n):
    t = (i + 1) * DT
    w = args.w if t > 0.5 else 0.0                     # 0.5 s 가만히 둔 뒤 회전
    wl, wr = (-w * SEP / 2) / RAD, (w * SEP / 2) / RAD
    k = max(1.0, abs(wl) / lim, abs(wr) / lim)
    cmd_buf.set_velocity_index(value=torch.tensor([[wl / k, wl / k, wr / k, wr / k]], dtype=torch.float32, device=sim.device), joint_ids=wheels)
    cmd_buf.set_position_index(value=target, joint_ids=hold)
    robot.write_data_to_sim()
    sim.step(render=False)
    robot.update(DT)
    if i % 12 == 11:
        d = robot.data
        rp = d.root_pos_w.torch[0].tolist()
        rq = d.root_quat_w.torch[0].tolist()
        yaw = 2 * math.atan2(rq[2], rq[3])
        c, s = math.cos(yaw), math.sin(yaw)
        bp = d.body_pos_w.torch[0].tolist()
        rel = {}
        for b, q in zip(bodies, bp):
            dx, dy = q[0] - rp[0], q[1] - rp[1]
            rel[b] = [round(c * dx + s * dy, 4), round(-s * dx + c * dy, 4), round(q[2], 4)]
        jp = d.joint_pos.torch[0].tolist()
        rows.append({"t": round(t, 3), "mission": [round(rp[0] + args.offset[0], 4), round(rp[1] + args.offset[1], 4), round(yaw, 4)],
                     "arm": {nm: round(jp[j], 4) for nm, j in zip(hold_names, hold)}, "bodies": rel})
json.dump({"args": vars(args), "drive_pose": dp, "rows": rows}, open(os.path.join(args.out, "replay.json"), "w"), indent=1, ensure_ascii=False, default=str)
for r in rows[::5]:
    far = max(r["bodies"].items(), key=lambda kv: abs(kv[1][1]))
    print(r["t"], r["mission"], "가장 옆 링크", far[0], far[1], "joint1", r["arm"].get("joint1"), flush=True)
app.close()
