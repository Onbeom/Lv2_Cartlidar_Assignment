"""도구 확인(모의, 판정 아님): Isaac 물리(PhysX 장면 쿼리)가 보는 로봇 옆선을 잰다.
설계 21 · 22 · 25 에서 출발 통로 벽 끝이 로봇 좌표 (앞 0.08~0.115, 왼쪽 0.210) 에 걸렸는데, USD 충돌 prim 경계는 |y| ≤ 0.1676 이라 맞지 않음.
scene_live.py 와 같게 로봇(가벼운 판, drive 자세) · 라이다 가림 상자를 원점 yaw 0 에 놓고 1 s 물리를 돌린 뒤,
좌 · 우 바깥(y = ±0.6)에서 안쪽으로 광선을 x · z 격자로 쏴서 맞은 y 와 prim 을 기록한다(첫 시도: 맞은 점 0 — 이 설정에선 장면 쿼리가 로봇을 못 봄).
--wall_y W: 왼쪽에 고정 벽(안쪽 면 y = W, x −0.3~0.3, 높이 0.4)을 세우고 로봇을 그 안에 겹치게 둔 뒤 3 s 물리로 밀려난 거리를 잼
→ 왼쪽 물리 폭 = W − 로봇 y(바퀴 마찰로 덜 밀릴 수 있어 '적어도 이만큼').
--fix_wheels: scene_live.py 의 바퀴 충돌 바로잡기(fix_wheel_colliders)를 켠 뒤 같은 측정(고친 뒤 확인용).
사용: <IsaacLab python> probe_robot_width.py --out <폴더> [--fix_wheels]"""
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
ap.add_argument("--fix_wheels", action="store_true")
ap.add_argument("--wall_y", type=float, default=0.0, help="0 = 벽 없음")
ap.add_argument("--wall_x", type=float, nargs=2, default=[-0.3, 0.3])
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
args.headless = True
os.makedirs(args.out, exist_ok=True)
app = AppLauncher(args).app

import sys  # noqa: E402

sys.path.insert(0, MODEL)
sys.path.insert(0, HERE)
import numpy as np  # noqa: E402
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from omni.physx import get_physx_scene_query_interface  # noqa: E402

from waffle_4wd_isaac import robot_cfg as R  # noqa: E402
from waffle_4wd_isaac import sensors_cfg as S  # noqa: E402

ROBOT = "/World/Robot"
sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1.0 / 120.0, device="cuda:0", physics_material=R.CONTACT_MATERIAL))
g = sim_utils.GroundPlaneCfg(physics_material=R.CONTACT_MATERIAL)
g.func("/World/ground", g)
cfg = R.WAFFLE_4WD_CFG.replace(prim_path=ROBOT)
cfg.init_state.pos = (0.0, 0.0, R.ROOT_HEIGHT + 0.003)
cfg.init_state.joint_pos = R.pose("drive")
robot = Articulation(cfg)
S.spawn_lidar_occluders(ROBOT)
if args.wall_y:
    wc = sim_utils.MeshCuboidCfg(size=(args.wall_x[1] - args.wall_x[0], 0.05, 0.4), collision_props=sim_utils.CollisionPropertiesCfg())
    wc.func("/World/probe_wall", wc, translation=((args.wall_x[0] + args.wall_x[1]) / 2, args.wall_y + 0.025, 0.2))
fixed = []
if args.fix_wheels:
    from wheel_collider_fix import fix_wheel_colliders  # noqa: E402
    fixed = fix_wheel_colliders(ROBOT)
sim.reset()
hold_names = R.ARM_JOINTS + R.FINGER_JOINTS
hold, _ = robot.find_joints(hold_names, preserve_order=True)
dp = R.pose("drive")
target = torch.tensor([[dp[n] for n in hold_names]], dtype=torch.float32, device=sim.device)
track = []
for i in range(360 if args.wall_y else 120):
    robot.set_joint_position_target(target, joint_ids=hold)
    robot.write_data_to_sim()
    sim.step(render=False)
    robot.update(1.0 / 120.0)
    if i % 30 == 29:
        rp = robot.data.root_pos_w.torch[0].tolist()
        rq = robot.data.root_quat_w.torch[0].tolist()
        track.append([round((i + 1) / 120.0, 3), round(rp[0], 4), round(rp[1], 4), round(2 * math.atan2(rq[2], rq[3]), 4)])
p = robot.data.root_pos_w.torch[0].tolist()
q = robot.data.root_quat_w.torch[0].tolist()
sq = get_physx_scene_query_interface()
rows = []
for side in (+1, -1):
    for z in (0.005, 0.015, 0.026, 0.040, 0.055, 0.080, 0.12, 0.20, 0.30):
        for x in np.arange(-0.24, 0.30 + 1e-9, 0.005):
            h = sq.raycast_closest((float(x) + p[0], side * 0.6 + p[1], z), (0.0, -float(side), 0.0), 1.2)
            if h["hit"]:
                rows.append({"side": side, "x": round(float(x), 3), "z": z, "y": round(h["position"][1] - p[1], 4), "prim": h["collision"]})
by = {}
for r in rows:
    k = (r["side"], r["prim"])
    if k not in by or abs(r["y"]) > abs(by[k]["y"]):
        by[k] = r
res = {"wall_y": args.wall_y, "wall_x": args.wall_x, "track_t_x_y_yaw": track,
       "left_width_at_least": (round(args.wall_y - p[1], 4) if args.wall_y else None), "root_pos": p, "root_quat_xyzw": q, "fix_wheels": args.fix_wheels, "fixed": fixed,
       "max_abs_y_left": max((r["y"] for r in rows if r["side"] > 0), default=None),
       "max_abs_y_right": max((-r["y"] for r in rows if r["side"] < 0), default=None),
       "widest_per_prim": sorted(by.values(), key=lambda r: -abs(r["y"]))[:12], "n_hits": len(rows)}
json.dump({"summary": res, "rows": rows}, open(os.path.join(args.out, "probe.json"), "w"), indent=1, ensure_ascii=False)
print(json.dumps(res, ensure_ascii=False, indent=1), flush=True)
app.close()
