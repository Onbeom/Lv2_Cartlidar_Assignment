"""pose_robot 도구 확인: USD 링크 prim 의 월드 변환이 URDF FK(base_footprint 기준 → T_w_bf)와 같은지 (Isaac Sim 창 없이).
새 모델 261028 light USD · 이전 모델 261027_1711 USD, 무작위 팔 자세 각 5개. 기준: 위치 차 < 1e-5 m, 회전 행렬 차 < 1e-5."""
import json
import sys
from pathlib import Path

import numpy as np

TOP = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TOP / "Cartlidar/sim_render"))
sys.path.insert(0, str(TOP / "Cartlidar/sim_world"))
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})
import omni.usd  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402

import robot_usd  # noqa: E402
from robot_geom import Robot  # noqa: E402
from urdf_chain import planar_pose  # noqa: E402

CASES = [("new_261028", TOP / "Cartlidar/sim_world/worlds/dice_mission_maze261030_judge/seed_101_T6500",
          TOP / "261029_1030/urdf_261028/isaac/urdf/waffle_4wd_light.urdf",
          TOP / "261029_1030/urdf_261028/isaac/usd/light/waffle_4wd_light/waffle_4wd_light.usda",
          ["joint1", "joint2", "joint3", "joint4", "joint5"], ["link2", "link3", "link4", "link5", "gripper_roll_link", "gripper_left_link"]),
         ("old_261027_1711", TOP / "Cartlidar/sim_world/worlds/dice_mission/seed_1",
          robot_usd.URDF, robot_usd.ROBOT_USD,
          ["RMX52_Base_Yaw", "RMX52_Shoulder", "RMX52_Elbow", "RMX52_Wrist_Pitch", "RMX52_Wrist_Roll_Reversed"],
          ["arm_link_1", "arm_link_2", "arm_link_3", "arm_link_4", "arm_link_5", "gripper_left_link"])]
ctx = omni.usd.get_context()
rng = np.random.default_rng(3)
out = {}
for name, world, urdf, usd, joints, links in CASES:
    ctx.open_stage(str(world / "task_world.usda"))
    for _ in range(30):
        app.update()
    stage = ctx.get_stage()
    robot_usd.use_robot_usd(stage, usd)
    for _ in range(10):
        app.update()
    rb = Robot(urdf, urdf.parent.parent)
    T_bf_root = robot_usd.root_from_footprint(urdf)
    dp, dr = 0.0, 0.0
    for k in range(5):
        q = {j: float(rng.uniform(rb.joint_by_name[j]["lower"], rb.joint_by_name[j]["upper"])) for j in joints}
        q["gripper_left_joint"] = 0.01
        T_w_bf = planar_pose(0.3 + 0.1 * k, 0.5, 0.7 * k)
        robot_usd.pose_robot(stage, urdf, T_w_bf, q)
        P = rb.link_poses(q, T_w_bf @ T_bf_root)
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        for l in links:
            M = np.array(cache.GetLocalToWorldTransform(robot_usd.find_link(stage, l))).T
            dp = max(dp, float(np.abs(M[:3, 3] - P[l][:3, 3]).max()))
            dr = max(dr, float(np.abs(M[:3, :3] - P[l][:3, :3]).max()))
        bf = rb.link_poses(q, T_w_bf @ T_bf_root)["base_footprint"]
        dp = max(dp, float(np.abs(bf - T_w_bf).max()))
    out[name] = {"max_pos_diff_m": dp, "max_rot_diff": dr, "root_z_offset_m": float(T_bf_root[2, 3]),
                 "pass": dp < 1e-5 and dr < 1e-5}
print("RESULT", json.dumps(out, ensure_ascii=False))
app.close()
