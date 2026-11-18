"""진단(판정 아님): 평가와 같은 조건으로 돌리며 팔 몸체 위치(base_link 좌표)·몸체별 접촉·TCP 오차·관절 위치를 정책 스텝마다 기록.
J3 접촉이 어디서(차체의 어느 부분, 가는 길 / 목표 근처) 나는지, 갇힌 구간이 어떤 자세인지 보려는 것.

사용(PC, IsaacLab-6.1 폴더에서):
  uv run --frozen --extra isaacsim python <이 파일> --design v8 --checkpoint <model.pt> [--hold_gate 0.006,0.009] --out <npz>
"""
import argparse
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
ap.add_argument("--hold_gate", default="", help="'r_in,r_out'(m)이면 v8 행동을 유지 게이트 행동으로 바꿈")
ap.add_argument("--out", required=True)
add_launcher_args(ap)
args = ap.parse_args()

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

import waffle_rl  # noqa: E402,F401
from waffle_rl import reach_env_cfg as C  # noqa: E402
from waffle_rl.agents import rsl_rl_ppo_cfg as A  # noqa: E402
from waffle_rl.mdp_cfg import ARM_BODIES  # noqa: E402

V = args.design.upper()
TASK = f"Waffle-RL1-ArmReach{V}"
cfg = getattr(C, f"RL1ArmReach{V}EnvCfg")()
AG = A.RL1ArmReachV8PPORunnerCfg
cfg.scene.num_envs = args.num_envs
cfg.seed = 0
cfg.commands.tcp_target.table_path = str(C.RESULTS / "targets" / "targets_eval_seed2.npz")
cfg.commands.tcp_target.resampling_time_range = (1.0e9, 1.0e9)
cfg.episode_length_s = 1.0e5
cfg.terminations.time_out = None
if hasattr(cfg.terminations, "arm_contact"):
    cfg.terminations.arm_contact = None
cfg.observations.policy.enable_corruption = False
if args.hold_gate:
    from waffle_rl.mdp_cfg import HoldGateRelativeJointPositionActionCfg
    r_in, r_out = (float(x) for x in args.hold_gate.split(","))
    a0 = cfg.actions.arm
    cfg.actions.arm = HoldGateRelativeJointPositionActionCfg(
        asset_name=a0.asset_name, joint_names=a0.joint_names, preserve_order=a0.preserve_order, max_rate=a0.max_rate,
        alpha=a0.alpha, max_acc=a0.max_acc, scale_rad=a0.scale_rad, hold_r_in=r_in, hold_r_out=r_out)
K = args.n_targets // args.num_envs

with launch_simulation(cfg, args):
    from importlib.metadata import version

    from isaaclab.managers import SceneEntityCfg
    from isaaclab.utils.math import quat_apply_inverse
    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg
    from rsl_rl.runners import OnPolicyRunner

    from waffle_rl import mdp as rl1

    agent_cfg = handle_deprecated_rsl_rl_cfg(AG(), version("rsl-rl-lib"))
    env = gym.make(TASK, cfg=cfg)
    u = env.unwrapped
    venv = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    runner = OnPolicyRunner(venv, agent_cfg.to_dict(), log_dir=None, device=u.device)
    runner.load(args.checkpoint)
    policy = runner.get_inference_policy(device=u.device)
    robot = u.scene["robot"]
    bids = [robot.body_names.index(b) for b in ARM_BODIES]
    jids = [robot.joint_names.index(n) for n in C.ARM_JOINTS]
    tcp_cfg = SceneEntityCfg("robot", body_names=["arm_link_5"])
    tcp_cfg.resolve(u.scene)
    term = u.command_manager.get_term("tcp_target")
    steps = int(round(4.0 / u.step_dt))
    N = args.num_envs
    pos = np.zeros((N, K, steps, len(bids), 3), np.float32)
    hit = np.zeros((N, K, steps, len(bids)), np.bool_)
    err = np.zeros((N, K, steps), np.float32)
    q = np.zeros((N, K, steps, len(jids)), np.float32)
    tgt = np.zeros((N, K, 3), np.float32)
    clr = np.zeros((N, K, steps), np.float32)   # 팔 ↔ 차체 최소 거리(설계 v15 벌점 함수와 같은 계산)
    clr_cfg = SceneEntityCfg("robot", body_names=C.CLEARANCE_BODIES, preserve_order=True)
    clr_cfg.resolve(u.scene)
    obs = venv.get_observations()
    for k in range(K):
        idx = torch.arange(N, device=u.device) * K + k
        term.target_b[:] = term.table_tcp[idx]
        tgt[:, k] = term.target_b.cpu().numpy()
        obs = venv.get_observations()
        for s in range(steps):
            with torch.inference_mode():
                obs, _, _, _ = venv.step(policy(obs))
            d = robot.data
            pw = d.body_pos_w.torch[:, bids] - d.root_pos_w.torch[:, None]
            rq = d.root_quat_w.torch[:, None].expand(-1, len(bids), -1)
            pos[:, k, s] = quat_apply_inverse(rq.reshape(-1, 4), pw.reshape(-1, 3)).reshape(N, len(bids), 3).cpu().numpy()
            for j, b in enumerate(ARM_BODIES):
                fm = u.scene.sensors[f"contact_{b}"].data.force_matrix_w.torch
                hit[:, k, s, j] = (torch.linalg.norm(fm, dim=-1).amax(dim=(1, 2)) > 0.1).cpu().numpy()
            err[:, k, s] = rl1.tcp_position_error(u, "tcp_target", tcp_cfg, C.TCP_OFFSET).cpu().numpy()
            q[:, k, s] = d.joint_pos.torch[:, jids].cpu().numpy()
            clr[:, k, s] = rl1.arm_chassis_clearance(u, clr_cfg).cpu().numpy()
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.out, clr=clr, pos=pos, hit=hit, err=err, q=q, tgt=tgt, bodies=np.array(ARM_BODIES),
                        table_q=np.load(cfg.commands.tcp_target.table_path)["q"][:N * K].reshape(N, K, -1))
    print("saved", args.out, flush=True)
    env.close()
