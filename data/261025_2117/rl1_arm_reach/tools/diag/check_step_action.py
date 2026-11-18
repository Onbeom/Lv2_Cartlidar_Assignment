"""도구 확인 T6(판정 아님): 행동 설계별 계단 명령 응답. 정책 없이 관절 하나의 원하는 목표를 크게 바꿨다가 되돌린다.

환경마다 리셋 자세(평가 표의 무작위 자세)에서 시작해, 관절 j의 원하는 목표를 범위 가운데 쪽으로 step_rad 옮겨 1.5 s → 시작 자세 1.5 s.
접촉(> 0.1 N)이 한 번이라도 난 환경은 통계에서 뺀다(범위 끝까지 보낸 첫 실행은 대부분 차체에 부딪혀 행동 비교가 안 됐음).
나머지 관절은 시작 자세를 유지하는 행동. 물리 스텝마다 팔 5관절 속도·계산 토크·위치/속도 목표를 기록한다.

사용(PC, IsaacLab-6.1 폴더에서):
  uv run --frozen --extra isaacsim python <이 파일> --design v6 --max_acc 10 --num_envs 16 --out <json>
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
ap.add_argument("--design", choices=["v5", "v6"], required=True)
ap.add_argument("--max_acc", type=float, default=None, help="v6 가속도 상한 덮어쓰기(rad/s²)")
ap.add_argument("--num_envs", type=int, default=16)
ap.add_argument("--step_rad", type=float, default=0.8)
ap.add_argument("--max_rate", type=float, default=None, help="목표 속도 상한 덮어쓰기(rad/s)")
ap.add_argument("--hold_current", type=int, default=None, help="v6 위치 목표 한 스텝 늦춤 덮어쓰기(0/1)")
ap.add_argument("--out", required=True)
add_launcher_args(ap)
args = ap.parse_args()

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

import waffle_rl  # noqa: E402,F401
from waffle_rl import reach_env_cfg as C  # noqa: E402
from waffle_rl.robot_cfg import ARM_JOINTS  # noqa: E402

TASK = {"v5": "Waffle-RL1-ArmReachV5", "v6": "Waffle-RL1-ArmReachV6"}[args.design]
cfg = {"v5": C.RL1ArmReachV5EnvCfg, "v6": C.RL1ArmReachV6EnvCfg}[args.design]()
if args.max_acc is not None:
    cfg.actions.arm.max_acc = args.max_acc
if args.max_rate is not None:
    cfg.actions.arm.max_rate = args.max_rate
if args.hold_current is not None:
    cfg.actions.arm.hold_current = bool(args.hold_current)
cfg.scene.num_envs = args.num_envs
cfg.seed = 0
cfg.commands.tcp_target.table_path = str(C.RESULTS / "targets" / "targets_eval_seed2.npz")
cfg.commands.tcp_target.resampling_time_range = (1.0e9, 1.0e9)
cfg.episode_length_s = 1.0e5
cfg.terminations.time_out = None

with launch_simulation(cfg, args):
    env = gym.make(TASK, cfg=cfg)
    u = env.unwrapped
    r = u.scene["robot"]
    arm5 = [i for i, n in enumerate(r.joint_names) if n.startswith("RMX52_")]
    ids = [r.joint_names.index(n) for n in ARM_JOINTS]
    lim = r.data.joint_pos_limits.torch[0, ids]
    lo, hi = lim[:, 0], lim[:, 1]
    rec = {}
    orig_step = u.sim.step

    def step_hook(*a, **k):
        out = orig_step(*a, **k)
        d = r.data
        rec["qd"].append(d.joint_vel.torch[:, arm5].cpu().numpy())
        rec["tau"].append(d.computed_torque.torch[:, arm5].cpu().numpy())
        rec["lead"].append((d.joint_pos_target.torch[:, ids] - d.joint_pos.torch[:, ids]).cpu().numpy())
        rec["q"].append(d.joint_pos.torch[:, arm5].cpu().numpy())
        rec["qt"].append(d.joint_pos_target.torch[:, arm5].cpu().numpy())
        rec["qdt"].append(d.joint_vel_target.torch[:, arm5].cpu().numpy())
        # PhysX가 실제로 낸 관절 힘(구동 + 구속을 관절 축으로 투영). 계산 토크(Isaac Lab 추정)와 비교용
        rec["pf"].append(np.asarray(r.root_view.get_dof_projected_joint_forces().numpy())[:, arm5])
        return out

    u.sim.step = step_hook
    result = {"design": args.design, "step_rad": args.step_rad, "max_acc": getattr(cfg.actions.arm, "max_acc", None), "hold_current": getattr(cfg.actions.arm, "hold_current", None),
              "max_rate": cfg.actions.arm.max_rate, "alpha": cfg.actions.arm.alpha, "per_joint": {}}
    dump = {}
    for j, name in enumerate(ARM_JOINTS):
        env.reset()
        rec.update({"qd": [], "tau": [], "lead": [], "q": [], "qt": [], "qdt": [], "pf": []})
        q0 = r.data.joint_pos.torch[:, ids].clone()
        a_hold = 2.0 * (q0 - lo) / (hi - lo) - 1.0
        sgn = torch.where(q0[:, j] < 0.5 * (lo[j] + hi[j]), 1.0, -1.0)
        goal = (q0[:, j] + sgn * args.step_rad).clamp(lo[j], hi[j])
        a_far = 2.0 * (goal - lo[j]) / (hi[j] - lo[j]) - 1.0
        reach_t, back_t = [], []
        hit = torch.zeros(args.num_envs, dtype=torch.bool, device=u.device)
        for k in range(90):
            a = a_hold.clone()
            if k < 45:
                a[:, j] = a_far
            env.step(a)
            qj = r.data.joint_pos.torch[:, ids[j]]
            tgt = goal if k < 45 else q0[:, j]
            reach_t.append(((qj - tgt).abs() <= 0.01).cpu().numpy())
            for n in C.CONTACT_SENSORS:
                hit |= torch.linalg.norm(u.scene.sensors[n].data.force_matrix_w.torch, dim=-1).amax(dim=(1, 2)) > 0.1
        ok = ~hit.cpu().numpy()
        qd = np.abs(np.stack(rec["qd"]))[:, ok]
        sat = np.abs(np.stack(rec["tau"]))[:, ok] >= 3.8 - 1e-6
        lead = np.abs(np.stack(rec["lead"]))[:, ok, j]
        rt = np.stack(reach_t)[:, ok]     # (90, 접촉 없는 env)
        first_go = np.where(rt[:45].any(0), rt[:45].argmax(0) + 1, np.inf) * u.step_dt
        first_back = np.where(rt[45:].any(0), rt[45:].argmax(0) + 1, np.inf) * u.step_dt
        dump[name] = {k: np.stack(v) for k, v in rec.items()}
        dump[name]["ok"] = ok
        result["per_joint"][name] = {
            "vel_max_rad_s": round(float(qd.max()), 3),
            "sat_ratio_physics_steps": round(float(sat.mean()), 4),
            "sat_ratio_policy_steps": round(float(sat[3::4].mean()), 4),
            "lead_max_rad": round(float(lead.max()), 4),
            "reach_go_s_median": float(np.median(first_go)), "reach_go_never": int(np.isinf(first_go).sum()),
            "reach_back_s_median": float(np.median(first_back)), "reach_back_never": int(np.isinf(first_back).sum()),
            "envs_used_no_contact": int(ok.sum()),
            "sat_ratio_per_arm5_joint": sat.mean((0, 1)).round(4).tolist(),
            "physx_force_abs_max_moved": round(float(np.abs(np.stack(rec["pf"]))[:, ok, j].max()), 3),
            "physx_force_ge_3p8_ratio_moved": round(float((np.abs(np.stack(rec["pf"]))[:, ok, j] >= 3.8 - 1e-3).mean()), 4),
            "computed_tau_ge_3p8_ratio_moved": round(float(sat[:, :, j].mean()), 4),
        }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(Path(args.out).with_suffix(".npz"),
                        **{f"{n}__{k}": v for n, dd in dump.items() for k, v in dd.items()})
    Path(args.out).write_text(json.dumps(result, ensure_ascii=False, indent=1))
    print("RESULT", json.dumps(result, ensure_ascii=False), flush=True)
    env.close()
