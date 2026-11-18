"""세운 직육면체 퍽을 기운 채 상자 위에 내려놓으면 바로 서는지 (2026-11-04, 누운 퍽 A안 확인, 학습 없음).

대마다 기울기 각 하나 (0 ~ 35°, 아래 모서리를 축으로 기울임) x 띄운 높이 (0 / 3 mm) 로 퍽을 두고 1.5 s 뒤 퍽 축 기울기를 잼.
정적으로는 atan(15 / 30) = 26.6° 넘으면 넘어짐.

    ./run_rl.sh setdown_check
"""

from __future__ import annotations

import argparse
import math
import os
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--puck_mu", type=float, default=0.5)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402
from isaaclab.utils.math import quat_apply, quat_from_euler_xyz  # noqa: E402
from waffle_isaac.dice2_cfg import BOX_H, PUCK_H, PUCK_R, box_cfg, puck_cfg  # noqa: E402
from waffle_isaac.robot_cfg import FLOOR_MATERIAL  # noqa: E402

ANGLES = [0, 5, 10, 15, 20, 22, 24, 26, 28, 30, 35]
LIFTS = [0.0, 0.003]


def _puck():
    p = puck_cfg("P", "box", (0.9, 0.2, 0.2), (0.0, 0.0))
    return p.replace(spawn=p.spawn.replace(physics_material=sim_utils.RigidBodyMaterialCfg(
        static_friction=args_cli.puck_mu, dynamic_friction=args_cli.puck_mu, friction_combine_mode="min")))


@configclass
class SceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg(physics_material=FLOOR_MATERIAL))
    boxA: RigidObjectCfg = box_cfg("A", (0.0, 0.0))
    puck: RigidObjectCfg = _puck()
    light = AssetBaseCfg(prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=2000.0))


def main():
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 120, physics_material=FLOOR_MATERIAL))
    cases = [(a, l) for l in LIFTS for a in ANGLES]
    scene = InteractiveScene(SceneCfg(num_envs=len(cases), env_spacing=0.5))
    sim.reset()
    dev = sim.device
    n = len(cases)
    org = scene.env_origins
    box = scene["boxA"]
    bp = box.data.default_root_pose.torch.clone()
    bp[:, :3] = org + torch.tensor([[0.0, 0.0, BOX_H / 2]], device=dev)
    box.write_root_pose_to_sim_index(root_pose=bp, env_ids=torch.arange(n, device=dev))
    pk = scene["puck"]
    ang = torch.tensor([math.radians(a) for a, _ in cases], device=dev)
    lift = torch.tensor([l for _, l in cases], device=dev)
    q = quat_from_euler_xyz(torch.zeros(n, device=dev), ang, torch.zeros(n, device=dev))  # y 축으로 기울임
    # 아래 모서리 (x = −15 mm 쪽이 아니라 기우는 쪽 +x 모서리) 가 상자 윗면 + lift 에 닿게 중심 높이
    corner = torch.tensor([[PUCK_R, 0.0, -PUCK_H / 2]], device=dev).expand(n, 3)
    zc = BOX_H + lift - quat_apply(q, corner)[:, 2]
    pose = torch.cat((org + torch.stack((torch.zeros(n, device=dev), torch.zeros(n, device=dev), zc), dim=1), q), dim=1)
    ids = torch.arange(n, device=dev)
    pk.write_root_pose_to_sim_index(root_pose=pose, env_ids=ids)
    pk.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=ids)
    scene.write_data_to_sim()
    for _ in range(int(1.5 / sim.get_physics_dt())):
        sim.step(render=False)
        scene.update(sim.get_physics_dt())
    up = quat_apply(pk.data.root_quat_w.torch, torch.tensor([[0.0, 0.0, 1.0]], device=dev).expand(n, 3))[:, 2]
    tilt = torch.rad2deg(torch.acos(up.clamp(-1, 1)))
    for i, (a, l) in enumerate(cases):
        st = "섬" if tilt[i] < 3 else ("넘어짐" if tilt[i] > 60 else "기운 채")
        print(f"[SetDown] 처음 기울기 {a:2d}° 띄움 {l * 1000:.0f} mm -> 1.5 s 뒤 {tilt[i]:5.1f}° {st}", flush=True)
    print(f"[SetDown] 마찰 {args_cli.puck_mu}, 정적 넘어짐 각 {math.degrees(math.atan(PUCK_R / (PUCK_H / 2))):.1f}°", flush=True)


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
