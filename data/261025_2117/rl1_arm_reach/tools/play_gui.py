"""RL1 학습한 정책을 Isaac Lab 화면(Kit 창)으로 보기 (모의, 판정 아님).

목표 = 초록 구, TCP(두 손가락 메시 가운데) = 빨간 구. 목표는 학습과 같이 4 s마다 바뀌고 12 s마다 자세를 리셋한다.
창을 닫거나 Ctrl+C로 끝낸다. 학습 코드(waffle_rl)는 읽기만 한다.

사용(PC, IsaacLab-6.1 폴더에서, 화면 있는 터미널):
  uv run --frozen --extra isaacsim python <이 파일> --design v3s3 --checkpoint <model_999.pt> --num_envs 9 --viz kit
"""
import argparse
import sys
import time
from pathlib import Path

import warp as wp

wp.config.enable_backward = False
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from isaaclab.app import add_launcher_args, launch_simulation  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--checkpoint", required=True)
ap.add_argument("--design", choices=["v2", "v3s1", "v3s2", "v3s3", "v4", "v5", "v6", "v7", "v8", "v9", "v10", "v11", "v12", "v13", "v14", "v15", "v16"], required=True)
ap.add_argument("--num_envs", type=int, default=9)
ap.add_argument("--eval_table", action="store_true", help="목표를 평가 표(seed 2)에서 고름")
ap.add_argument("--no_mimic", action="store_true",
                help="화면 모드 전용: 집게 mimic을 뺀 USD 복사본을 쓰고 오른쪽 집게를 강성 1000으로 열린 채 고정"
                     "(Kit 화면 모드에서 PhysxMimicJointAPI 오류로 환경 생성이 실패해서. 학습·평가에는 쓰지 않음)")
add_launcher_args(ap)
args = ap.parse_args()

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import waffle_rl  # noqa: E402,F401
from waffle_rl.agents.rsl_rl_ppo_cfg import RL1ArmReachV2PPORunnerCfg, RL1ArmReachV3S2PPORunnerCfg, RL1ArmReachV8PPORunnerCfg  # noqa: E402
from waffle_rl.reach_env_cfg import (  # noqa: E402
    RESULTS, TCP_OFFSET, RL1ArmReachV2EnvCfg, RL1ArmReachV3S1EnvCfg, RL1ArmReachV3S3EnvCfg, RL1ArmReachV4EnvCfg, RL1ArmReachV5EnvCfg,
    RL1ArmReachV6EnvCfg, RL1ArmReachV7EnvCfg, RL1ArmReachV8EnvCfg, RL1ArmReachV9EnvCfg, RL1ArmReachV10EnvCfg, RL1ArmReachV11EnvCfg, RL1ArmReachV12EnvCfg, RL1ArmReachV13EnvCfg, RL1ArmReachV14EnvCfg, RL1ArmReachV15PlayEnvCfg, RL1ArmReachV16PlayEnvCfg)

TASK, ENV_CFG, AGENT_CFG = {
    "v2": ("Waffle-RL1-ArmReachV2", RL1ArmReachV2EnvCfg, RL1ArmReachV2PPORunnerCfg),
    "v3s1": ("Waffle-RL1-ArmReachV3S1", RL1ArmReachV3S1EnvCfg, RL1ArmReachV2PPORunnerCfg),
    "v3s2": ("Waffle-RL1-ArmReachV3S2", RL1ArmReachV3S1EnvCfg, RL1ArmReachV3S2PPORunnerCfg),
    "v3s3": ("Waffle-RL1-ArmReachV3S3", RL1ArmReachV3S3EnvCfg, RL1ArmReachV2PPORunnerCfg),
    "v4": ("Waffle-RL1-ArmReachV4", RL1ArmReachV4EnvCfg, RL1ArmReachV2PPORunnerCfg),
    "v5": ("Waffle-RL1-ArmReachV5", RL1ArmReachV5EnvCfg, RL1ArmReachV2PPORunnerCfg),
    "v6": ("Waffle-RL1-ArmReachV6", RL1ArmReachV6EnvCfg, RL1ArmReachV2PPORunnerCfg),
    "v7": ("Waffle-RL1-ArmReachV7", RL1ArmReachV7EnvCfg, RL1ArmReachV2PPORunnerCfg),
    "v8": ("Waffle-RL1-ArmReachV8", RL1ArmReachV8EnvCfg, RL1ArmReachV8PPORunnerCfg),
    "v9": ("Waffle-RL1-ArmReachV9", RL1ArmReachV9EnvCfg, RL1ArmReachV8PPORunnerCfg),
    "v10": ("Waffle-RL1-ArmReachV10", RL1ArmReachV10EnvCfg, RL1ArmReachV8PPORunnerCfg),
    "v11": ("Waffle-RL1-ArmReachV11", RL1ArmReachV11EnvCfg, RL1ArmReachV8PPORunnerCfg),
    "v12": ("Waffle-RL1-ArmReachV12", RL1ArmReachV12EnvCfg, RL1ArmReachV8PPORunnerCfg),
    "v13": ("Waffle-RL1-ArmReachV13", RL1ArmReachV13EnvCfg, RL1ArmReachV8PPORunnerCfg),
    "v14": ("Waffle-RL1-ArmReachV14", RL1ArmReachV14EnvCfg, RL1ArmReachV8PPORunnerCfg),
    "v15": ("Waffle-RL1-ArmReachV15", RL1ArmReachV15PlayEnvCfg, RL1ArmReachV8PPORunnerCfg),   # 실행 때 유지 게이트
    "v16": ("Waffle-RL1-ArmReachV16", RL1ArmReachV16PlayEnvCfg, RL1ArmReachV8PPORunnerCfg),   # 실행 때 유지 게이트
}[args.design]

cfg = ENV_CFG()
cfg.scene.num_envs = args.num_envs
cfg.scene.env_spacing = 1.2
cfg.observations.policy.enable_corruption = False
if args.no_mimic:
    cfg.scene.robot.spawn.usd_path = str(RESULTS / "usd" / "waffle_4wd_rl1_fixed_v2_gui_nomimic.usd")
    cfg.scene.robot.actuators["gripper_right_mimic"].stiffness = 1000.0
    cfg.scene.robot.actuators["gripper_right_mimic"].damping = 10.0
if args.eval_table:
    cfg.commands.tcp_target.table_path = str(RESULTS / "targets" / "targets_eval_seed2.npz")
try:  # 첫 환경을 비스듬히 위에서 본다
    cfg.viewer.origin_type = "env"
    cfg.viewer.env_index = 0
    cfg.viewer.eye = (0.95, -0.95, 0.75)
    cfg.viewer.lookat = (0.05, 0.0, 0.25)
except Exception as exc:  # noqa: BLE001
    print("viewer 설정 생략:", exc)

with launch_simulation(cfg, args):
    from importlib.metadata import version

    import isaaclab.sim as sim_utils
    from isaaclab.managers import SceneEntityCfg
    from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
    from isaaclab.utils.math import quat_apply
    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg
    from rsl_rl.runners import OnPolicyRunner

    agent_cfg = handle_deprecated_rsl_rl_cfg(AGENT_CFG(), version("rsl-rl-lib"))
    env = gym.make(TASK, cfg=cfg)
    u = env.unwrapped
    venv = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    runner = OnPolicyRunner(venv, agent_cfg.to_dict(), log_dir=None, device=u.device)
    runner.load(args.checkpoint)
    policy = runner.get_inference_policy(device=u.device)

    markers = VisualizationMarkers(VisualizationMarkersCfg(prim_path="/Visuals/rl1", markers={
        "target": sim_utils.SphereCfg(radius=0.015, visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.1, 0.8, 0.2))),
        "tcp": sim_utils.SphereCfg(radius=0.010, visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.9, 0.1, 0.1))),
    }))
    robot = u.scene["robot"]
    b5 = robot.body_names.index("arm_link_5")
    off = torch.tensor(TCP_OFFSET, device=u.device)
    term = u.command_manager.get_term("tcp_target")
    n = u.num_envs
    idx = torch.cat([torch.zeros(n, dtype=torch.long), torch.ones(n, dtype=torch.long)]).to(u.device)

    obs = venv.get_observations()
    print("[RL1] 화면 재생 시작: 초록 = 목표, 빨강 = TCP. 창을 닫으면 끝", flush=True)
    while u.sim.is_headless_or_exist_active_visualizer():
        t0 = time.time()
        with torch.inference_mode():
            obs, _, dones, _ = venv.step(policy(obs))
            policy.reset(dones) if hasattr(policy, "reset") else None
        rq, rp = robot.data.root_quat_w.torch, robot.data.root_pos_w.torch
        target_w = rp + quat_apply(rq, term.target_b)
        tcp_w = robot.data.body_pos_w.torch[:, b5] + quat_apply(robot.data.body_quat_w.torch[:, b5], off.expand(n, 3))
        markers.visualize(translations=torch.cat([target_w, tcp_w]), marker_indices=idx)
        sleep = u.step_dt - (time.time() - t0)
        if sleep > 0:
            time.sleep(sleep)
    env.close()
