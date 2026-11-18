"""RL1 판정 J1~J5 측정 (모의). J6은 seed별 결과를 모아 판정한다(summarize).

평가 조건:
- 목표 = 평가 표(seed 2, 학습에 안 씀) 앞 1000개. 환경 i가 목표 i·K … i·K+K−1을 4 s씩 차례로 받는다(K = 1000 / num_envs).
- 자동 목표 바꿈·시간 종료는 끈다(에피소드 중간 리셋 없음). 시작 자세는 평가 표의 무작위 자세.
- 관찰 잡음은 끈다(정책 성능만 본다). 정책은 결정적(평균 행동).
- 기록은 정책 스텝(30 Hz)마다. 토크 포화는 아래 J4 설명(PhysX 실제 관절 힘 |F| ≥ 3.8 N·m). 관절 속도·토크는 팔 5관절(RMX52_*).

판정(계획_RL1_팔도달.md 2장):
  J1 3.0 s 시점 오차 ≤ 10 mm 비율 ≥ 95 %   J2 처음 10 mm 도달 시간 중앙값 ≤ 1.5 s
  J3 팔–로봇 접촉(> 0.1 N) 0건              J4 관절 속도 최대 ≤ 4.50 rad/s, 토크 포화 시간 ≤ 5 %
  J5 목표 유지 마지막 1 s(3~4 s) 관절 속도 RMS ≤ 0.05 rad/s (목표 구간별 RMS의 최댓값과 95 % 값을 함께 기록)

J4 토크 포화(사용자 결정 2026-10-27): PhysX가 실제로 낸 관절 힘(get_dof_projected_joint_forces, 구동 + 구속의 관절 축 투영)의
|F| ≥ 3.8 N·m 비율로 잰다. 그 전(v14까지)에는 Isaac Lab 계산 토크로 쟀다. 계산 토크는 새 목표와 지난 스텝 상태로 구한
추정값이라 목표를 잘 따라가도 움직이는 동안 크게 나와(계획 문서 14장 T6·24장) 참고값(ref_computed_torque_sat_time_ratio)으로만 남긴다.

사용(PC, IsaacLab-6.1 폴더에서):
  uv run --frozen --extra isaacsim python <이 파일> --checkpoint <model_999.pt> --num_envs 100 --out <폴더>
"""
import argparse
import json
import sys
import time
from pathlib import Path

import warp as wp

wp.config.enable_backward = False

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from isaaclab.app import add_launcher_args, launch_simulation  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--checkpoint", required=True)
ap.add_argument("--design", choices=["v1", "v2", "v3s1", "v3s2", "v3s3", "v4", "v5", "v6", "v7", "v8", "v9", "v10", "v11", "v12", "v13", "v14", "v15", "v16"], required=True, help="학습한 설계(과제·행동 설정)")
ap.add_argument("--num_envs", type=int, default=100)
ap.add_argument("--n_targets", type=int, default=1000)
ap.add_argument("--seed", type=int, default=0, help="시작 자세 무작위 seed")
ap.add_argument("--out", required=True)
ap.add_argument("--diag_hold_gate", default="",
                help="진단 전용(판정에 쓰지 않음, v8 행동만): 'r_in,r_out'(m)이면 유지 게이트 행동으로 바꿔 평가")
ap.add_argument("--diag_act_ema", type=float, default=0.0,
                help="진단 전용(판정에 쓰지 않음): 정책 출력에 지수 이동 평균 a_f = β·a_f + (1−β)·a 를 걸어 env에 넣음. 0이면 끔")
add_launcher_args(ap)
args = ap.parse_args()

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

import waffle_rl  # noqa: E402,F401
from waffle_rl.agents.rsl_rl_ppo_cfg import (  # noqa: E402
    RL1ArmReachPPORunnerCfg, RL1ArmReachV2PPORunnerCfg, RL1ArmReachV3S2PPORunnerCfg, RL1ArmReachV8PPORunnerCfg)
from waffle_rl.reach_env_cfg import (  # noqa: E402
    CONTACT_SENSORS, RESULTS, TCP_OFFSET, RL1ArmReachEnvCfg, RL1ArmReachV2EnvCfg, RL1ArmReachV3S1EnvCfg,
    RL1ArmReachV3S3EnvCfg, RL1ArmReachV4EnvCfg, RL1ArmReachV5EnvCfg, RL1ArmReachV6EnvCfg, RL1ArmReachV7EnvCfg, RL1ArmReachV8EnvCfg, RL1ArmReachV9EnvCfg, RL1ArmReachV10EnvCfg, RL1ArmReachV11EnvCfg, RL1ArmReachV12EnvCfg, RL1ArmReachV13EnvCfg, RL1ArmReachV14EnvCfg, RL1ArmReachV15PlayEnvCfg, RL1ArmReachV16PlayEnvCfg)

TASK, ENV_CFG, AGENT_CFG = {
    "v1": ("Waffle-RL1-ArmReach", RL1ArmReachEnvCfg, RL1ArmReachPPORunnerCfg),
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

EVAL_TABLE = str(RESULTS / "targets" / "targets_eval_seed2.npz")
SEG_S, T_CHECK, TOL, EFFORT = 4.0, 3.0, 0.010, 3.8

cfg = ENV_CFG()
cfg.scene.num_envs = args.num_envs
cfg.seed = args.seed
cfg.commands.tcp_target.table_path = EVAL_TABLE
cfg.commands.tcp_target.resampling_time_range = (1.0e9, 1.0e9)
cfg.episode_length_s = 1.0e5
cfg.terminations.time_out = None
if hasattr(cfg.terminations, "arm_contact"):
    cfg.terminations.arm_contact = None   # v9 학습용 접촉 종료는 평가에서 끔(평가는 리셋 없이 잼)
cfg.observations.policy.enable_corruption = False
if args.diag_hold_gate:
    from waffle_rl.mdp_cfg import HoldGateRelativeJointPositionActionCfg, RelativeAccelLimitedJointPositionActionCfg
    assert type(cfg.actions.arm) is RelativeAccelLimitedJointPositionActionCfg, "유지 게이트 진단은 v8 행동만"
    r_in, r_out = (float(x) for x in args.diag_hold_gate.split(","))
    a0 = cfg.actions.arm
    cfg.actions.arm = HoldGateRelativeJointPositionActionCfg(
        asset_name=a0.asset_name, joint_names=a0.joint_names, preserve_order=a0.preserve_order, max_rate=a0.max_rate,
        alpha=a0.alpha, max_acc=a0.max_acc, scale_rad=a0.scale_rad, hold_r_in=r_in, hold_r_out=r_out)
assert args.n_targets % args.num_envs == 0
K = args.n_targets // args.num_envs

with launch_simulation(cfg, args):
    from isaaclab.managers import SceneEntityCfg
    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg
    from rsl_rl.runners import OnPolicyRunner

    from waffle_rl import mdp as rl1

    from importlib.metadata import version

    agent_cfg = handle_deprecated_rsl_rl_cfg(AGENT_CFG(), version("rsl-rl-lib"))
    env = gym.make(TASK, cfg=cfg)
    u = env.unwrapped
    venv = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    runner = OnPolicyRunner(venv, agent_cfg.to_dict(), log_dir=None, device=u.device)
    runner.load(args.checkpoint)
    policy = runner.get_inference_policy(device=u.device)

    robot = u.scene["robot"]
    arm5 = [i for i, n in enumerate(robot.joint_names) if n.startswith("RMX52_")]
    tcp_cfg = SceneEntityCfg("robot", body_names=["arm_link_5"])
    tcp_cfg.resolve(u.scene)
    term = u.command_manager.get_term("tcp_target")
    steps = int(round(SEG_S / u.step_dt))
    k_check = int(round(T_CHECK / u.step_dt))
    N = args.num_envs
    err = np.zeros((N, K, steps), np.float32)
    contact = np.zeros((N, K, steps), np.int16)
    vmax = np.zeros((N, K, steps), np.float32)
    sat = np.zeros((N, K, steps), np.float32)
    sat_px = np.zeros((N, K, steps), np.float32)
    vrms_last = np.zeros((N, K), np.float32)
    t0 = time.time()
    obs = venv.get_observations()
    a_f = None
    for k in range(K):
        idx = torch.arange(N, device=u.device) * K + k
        term.target_b[:] = term.table_tcp[idx]
        obs = venv.get_observations()
        vel_last = []
        for s in range(steps):
            with torch.inference_mode():
                act = policy(obs)
                if args.diag_act_ema > 0.0:
                    a_f = act.clone() if a_f is None else args.diag_act_ema * a_f + (1.0 - args.diag_act_ema) * act
                    act = a_f
                obs, _, dones, _ = venv.step(act)
            e = rl1.tcp_position_error(u, "tcp_target", tcp_cfg, TCP_OFFSET)
            qd = robot.data.joint_vel.torch[:, arm5]
            tau = robot.data.computed_torque.torch[:, arm5]
            err[:, k, s] = e.cpu().numpy()
            contact[:, k, s] = rl1.arm_self_contact_count(u, CONTACT_SENSORS, 0.1).cpu().numpy().astype(np.int16)
            vmax[:, k, s] = qd.abs().amax(1).cpu().numpy()
            sat[:, k, s] = (tau.abs() >= EFFORT - 1e-6).float().mean(1).cpu().numpy()
            pf = np.asarray(robot.root_view.get_dof_projected_joint_forces().numpy())[:, arm5]
            sat_px[:, k, s] = (np.abs(pf) >= EFFORT - 1e-3).mean(1)
            if s >= k_check:
                vel_last.append(qd)
            assert not bool(dones.any()), "평가 중 리셋이 일어남"
        v = torch.stack(vel_last, 0)                     # (T, N, 5)
        vrms_last[:, k] = torch.sqrt((v ** 2).mean(dim=(0, 2))).cpu().numpy()
        print(f"목표 {k + 1}/{K} 끝, 3 s 오차 중앙값 {np.median(err[:, k, k_check - 1]) * 1000:.2f} mm", flush=True)

    e3 = err[:, :, k_check - 1].ravel()                   # 목표 바뀐 뒤 3.0 s 시점(정책 스텝 90번째 끝)
    reached = err <= TOL
    # 끝내 못 닿은 목표는 무한대로 두고 중앙값을 낸다
    first = np.where(reached.any(2), (reached.argmax(2) + 1) * u.step_dt, np.inf).ravel()
    n_contact_seg = int((contact.max(2) > 0).sum())
    summary = {
        "checkpoint": args.checkpoint, "design": args.design, "diag_act_ema": args.diag_act_ema, "diag_hold_gate": args.diag_hold_gate, "num_envs": N, "n_targets": int(e3.size), "eval_table": EVAL_TABLE,
        "start_pose_seed": args.seed, "wall_s": round(time.time() - t0, 1), "step_dt": u.step_dt,
        "J1_ratio_err3s_le_10mm": float((e3 <= TOL).mean()),
        "J1_err3s_mm": {"median": float(np.median(e3) * 1e3), "p95": float(np.percentile(e3, 95) * 1e3),
                         "max": float(e3.max() * 1e3)},
        "J2_first_10mm_s": {"median": float(np.median(first)), "never_reached": int(np.isinf(first).sum())},
        "J3_contact_segments": n_contact_seg, "J3_contact_steps": int((contact > 0).sum()),
        "J4_joint_vel_max_rad_s": float(vmax.max()), "J4_torque_sat_time_ratio": float(sat_px.mean()),
        "J4_torque_source": "physx_projected_joint_force",
        "ref_computed_torque_sat_time_ratio": float(sat.mean()),
        "J5_last1s_vel_rms_rad_s": {"max": float(vrms_last.max()), "p95": float(np.percentile(vrms_last, 95)),
                                    "median": float(np.median(vrms_last))},
    }
    j2m = summary["J2_first_10mm_s"]["median"]
    summary["verdict"] = {
        "J1": summary["J1_ratio_err3s_le_10mm"] >= 0.95,
        "J2": j2m <= 1.5,
        "J3": n_contact_seg == 0,
        "J4": summary["J4_joint_vel_max_rad_s"] <= 4.50 and summary["J4_torque_sat_time_ratio"] <= 0.05,
        "J5": summary["J5_last1s_vel_rms_rad_s"]["max"] <= 0.05,
    }
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out / "eval_log.npz", err=err, contact=contact, vmax=vmax, sat=sat, vrms_last=vrms_last, sat_px=sat_px)
    (out / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1))
    print("RESULT", json.dumps(summary, ensure_ascii=False), flush=True)
    env.close()
