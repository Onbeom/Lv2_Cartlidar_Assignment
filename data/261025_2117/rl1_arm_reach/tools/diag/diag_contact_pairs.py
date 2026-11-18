"""진단(판정 아님): 평가와 같은 조건(평가 표 앞 n_targets개, 4 s씩, 결정적 정책)으로 돌리며 팔 접촉을 몸체 쌍별로 센다.

사용(PC, IsaacLab-6.1 폴더에서):
  uv run --frozen --extra isaacsim python <이 파일> --design v8 --checkpoint <model.pt> --out <json>
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
ap.add_argument("--num_envs", type=int, default=100)
ap.add_argument("--n_targets", type=int, default=1000)
ap.add_argument("--out", required=True)
add_launcher_args(ap)
args = ap.parse_args()

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

import waffle_rl  # noqa: E402,F401
from waffle_rl.agents import rsl_rl_ppo_cfg as A  # noqa: E402
from waffle_rl import reach_env_cfg as C  # noqa: E402
from waffle_rl.mdp_cfg import ARM_BODIES  # noqa: E402

V = args.design.upper()
TASK = f"Waffle-RL1-ArmReach{V}"
cfg = getattr(C, f"RL1ArmReach{V}EnvCfg")()
AG = A.RL1ArmReachV8PPORunnerCfg if V in ("V8", "V9", "V10", "V11", "V12", "V13", "V14") else A.RL1ArmReachV2PPORunnerCfg
cfg.scene.num_envs = args.num_envs
cfg.seed = 0
cfg.commands.tcp_target.table_path = str(C.RESULTS / "targets" / "targets_eval_seed2.npz")
cfg.commands.tcp_target.resampling_time_range = (1.0e9, 1.0e9)
cfg.episode_length_s = 1.0e5
cfg.terminations.time_out = None
if hasattr(cfg.terminations, "arm_contact"):
    cfg.terminations.arm_contact = None   # v9 학습용 접촉 종료는 평가에서 끔(평가는 리셋 없이 잼)
cfg.observations.policy.enable_corruption = False
K = args.n_targets // args.num_envs

with launch_simulation(cfg, args):
    from importlib.metadata import version

    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg
    from rsl_rl.runners import OnPolicyRunner

    agent_cfg = handle_deprecated_rsl_rl_cfg(AG(), version("rsl-rl-lib"))
    env = gym.make(TASK, cfg=cfg)
    u = env.unwrapped
    venv = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    runner = OnPolicyRunner(venv, agent_cfg.to_dict(), log_dir=None, device=u.device)
    runner.load(args.checkpoint)
    policy = runner.get_inference_policy(device=u.device)
    term = u.command_manager.get_term("tcp_target")
    others = {b: [n for n in C.BODY_PATHS if n != b] for b in ARM_BODIES}
    pair_steps, pair_segs, pair_maxN = {}, {}, {}
    obs = venv.get_observations()
    for k in range(K):
        idx = torch.arange(args.num_envs, device=u.device) * K + k
        term.target_b[:] = term.table_tcp[idx]
        obs = venv.get_observations()
        seg = {}
        for s in range(int(round(4.0 / u.step_dt))):
            with torch.inference_mode():
                obs, _, _, _ = venv.step(policy(obs))
            for b, name in zip(ARM_BODIES, C.CONTACT_SENSORS):
                f = torch.linalg.norm(u.scene.sensors[name].data.force_matrix_w.torch[:, 0], dim=-1)  # (N, F)
                hit = (f > 0.1).cpu().numpy()
                for fi in np.nonzero(hit.any(0))[0]:
                    key = "-".join(sorted([b, others[b][fi]]))
                    pair_steps[key] = pair_steps.get(key, 0) + int(hit[:, fi].sum())
                    pair_maxN[key] = max(pair_maxN.get(key, 0.0), float(f[:, fi].max()))
                    seg.setdefault(key, np.zeros(args.num_envs, bool))
                    seg[key] |= hit[:, fi]
        for key, m in seg.items():
            pair_segs[key] = pair_segs.get(key, 0) + int(m.sum())
    res = {"design": args.design, "checkpoint": args.checkpoint, "n_targets": K * args.num_envs,
           "note": "같은 쌍이 두 센서에서 함께 잡히면 두 번 셈(스텝 수). 구간 수는 쌍별 합",
           "pair_segments": dict(sorted(pair_segs.items(), key=lambda x: -x[1])),
           "pair_env_steps": pair_steps, "pair_max_N": {k: round(v, 2) for k, v in pair_maxN.items()}}
    Path(args.out).write_text(json.dumps(res, ensure_ascii=False, indent=1))
    print("RES", json.dumps(res, ensure_ascii=False), flush=True)
    env.close()
