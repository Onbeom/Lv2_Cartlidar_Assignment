"""진단(판정 아님): 학습한 정책을 화면 없이 돌리며 물리 스텝마다 관절별 위치 목표·속도 목표·실제 속도·계산 토크를 기록.

사용(PC, IsaacLab-6.1 폴더에서):
  uv run --frozen --extra isaacsim python <이 파일> --design v5 --checkpoint <model.pt> --out <npz>
"""
import argparse
import json
import sys
from pathlib import Path

import warp as wp

wp.config.enable_backward = False
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from isaaclab.app import add_launcher_args, launch_simulation  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--checkpoint", required=True)
ap.add_argument("--design", required=True)
ap.add_argument("--num_envs", type=int, default=16)
ap.add_argument("--steps", type=int, default=240, help="정책 스텝(30 Hz)")
ap.add_argument("--out", required=True)
add_launcher_args(ap)
args = ap.parse_args()

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

import waffle_rl  # noqa: E402,F401
from waffle_rl.agents import rsl_rl_ppo_cfg as A  # noqa: E402
from waffle_rl import reach_env_cfg as C  # noqa: E402

TASK = {"v4": "Waffle-RL1-ArmReachV4", "v5": "Waffle-RL1-ArmReachV5", "v6": "Waffle-RL1-ArmReachV6", "v7": "Waffle-RL1-ArmReachV7", "v8": "Waffle-RL1-ArmReachV8", "v9": "Waffle-RL1-ArmReachV9", "v10": "Waffle-RL1-ArmReachV10", "v11": "Waffle-RL1-ArmReachV11", "v12": "Waffle-RL1-ArmReachV12", "v13": "Waffle-RL1-ArmReachV13", "v14": "Waffle-RL1-ArmReachV14", "v15": "Waffle-RL1-ArmReachV15", "v16": "Waffle-RL1-ArmReachV16"}[args.design]
ENV = {"v4": "RL1ArmReachV4EnvCfg", "v5": "RL1ArmReachV5EnvCfg", "v6": "RL1ArmReachV6EnvCfg", "v7": "RL1ArmReachV7EnvCfg", "v8": "RL1ArmReachV8EnvCfg", "v9": "RL1ArmReachV9EnvCfg", "v10": "RL1ArmReachV10EnvCfg", "v11": "RL1ArmReachV11EnvCfg", "v12": "RL1ArmReachV12EnvCfg", "v13": "RL1ArmReachV13EnvCfg", "v14": "RL1ArmReachV14EnvCfg", "v15": "RL1ArmReachV15PlayEnvCfg", "v16": "RL1ArmReachV16PlayEnvCfg"}[args.design]
cfg = getattr(C, ENV)()
cfg.scene.num_envs = args.num_envs
cfg.seed = 0
cfg.observations.policy.enable_corruption = False
cfg.commands.tcp_target.table_path = str(C.RESULTS / "targets" / "targets_eval_seed2.npz")

with launch_simulation(cfg, args):
    from importlib.metadata import version

    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg
    from rsl_rl.runners import OnPolicyRunner

    agent_cfg = handle_deprecated_rsl_rl_cfg((A.RL1ArmReachV8PPORunnerCfg() if args.design in ("v8", "v9", "v10", "v11", "v12", "v13", "v14", "v15", "v16") else A.RL1ArmReachV2PPORunnerCfg()), version("rsl-rl-lib"))
    env = gym.make(TASK, cfg=cfg)
    u = env.unwrapped
    venv = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    runner = OnPolicyRunner(venv, agent_cfg.to_dict(), log_dir=None, device=u.device)
    runner.load(args.checkpoint)
    policy = runner.get_inference_policy(device=u.device)
    r = u.scene["robot"]
    arm5 = [i for i, n in enumerate(r.joint_names) if n.startswith("RMX52_")]
    names = [r.joint_names[i] for i in arm5]
    rec = {"q": [], "qd": [], "qt": [], "qdt": [], "tau": [], "act": []}
    orig_step = u.sim.step

    def step_hook(*a, **k):
        out = orig_step(*a, **k)
        d = r.data
        rec["q"].append(d.joint_pos.torch[:, arm5].cpu().numpy())
        rec["qd"].append(d.joint_vel.torch[:, arm5].cpu().numpy())
        rec["qt"].append(d.joint_pos_target.torch[:, arm5].cpu().numpy())
        rec["qdt"].append(d.joint_vel_target.torch[:, arm5].cpu().numpy())
        rec["tau"].append(d.computed_torque.torch[:, arm5].cpu().numpy())
        return out

    u.sim.step = step_hook
    obs = venv.get_observations()
    for k in range(args.steps):
        with torch.inference_mode():
            act = policy(obs)
            rec["act"].append(act.cpu().numpy())
            obs, _, _, _ = venv.step(act)
    arr = {k: np.stack(v) for k, v in rec.items()}
    np.savez_compressed(args.out, names=np.array(names), physics_dt=u.physics_dt, **arr)
    qd = np.abs(arr["qd"])
    summ = {"joint_names": names, "max_abs_vel_per_joint": qd.max((0, 1)).round(2).tolist(),
            "p99_abs_vel_per_joint": np.percentile(qd, 99, axis=(0, 1)).round(2).tolist(),
            "sat_ratio_per_joint": (np.abs(arr["tau"]) >= 3.8 - 1e-6).mean((0, 1)).round(3).tolist(),
            "max_abs_vel_target_per_joint": np.abs(arr["qdt"]).max((0, 1)).round(2).tolist(),
            "action_abs_mean": np.abs(arr["act"]).mean().round(3).item(),
            "action_sign_flip_rate": (np.diff(np.sign(arr["act"]), axis=0) != 0).mean().round(3).item()}
    print("RES", json.dumps(summ), flush=True)
    env.close()
