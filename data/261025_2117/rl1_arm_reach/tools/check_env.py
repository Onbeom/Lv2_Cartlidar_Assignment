"""RL1 도구 확인 T1~T4 (모의, Isaac Lab + Isaac Sim PhysX, 화면 없음).

  T1  환경 불러오기: 관절 이름·순서, PhysX에 실제로 들어간 이득·armature·최대 힘·최대 속도·한계,
      루트 고정, 자기충돌 속성. 접촉 센서 확인: 알려진 충돌 자세 [0, 1.5, 1.5, 1.7]로 목표를 주고
      팔–로봇 접촉 쌍이 잡히는지(MoveIt 기록: base_link–arm_link_3·arm_link_4, gripper_right–lidar_rear).
  T2  같은 목표 유지: 리셋 자세의 관절값을 5 s 동안 목표로 둠 → 관절·TCP 표류, 정지 떨림.
  T3  num_envs별 GPU 메모리·속도: 무작위 행동 STEPS 스텝(별도 프로세스로 num_envs마다 실행).
  T4  목표 표 FK TCP vs Isaac TCP: 표 자세를 관절에 직접 쓰고 비교(차이 ≤ 1 mm면 통과).
  T5  설계 v2 속도 명령 행동: 관절마다(리셋 = 표의 무작위 자세) +1(4.50 rad/s)을 0.5 s 준 뒤 0을 1 s
      → 최대 속도·움직이는 동안/전체 토크 포화 비율·정지 1 s 뒤 속도. 관절 범위 끝에 닿으면 그 뒤로 멈춘다.

사용(PC, IsaacLab-6.1 폴더에서):
  uv run --frozen --extra isaacsim python <이 파일> --test T1 --num_envs 4 --out <결과.json>
"""
import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import warp as wp

wp.config.enable_backward = False

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from isaaclab.app import add_launcher_args, launch_simulation  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--test", required=True, choices=["T1", "T2", "T3", "T4", "T5"])
ap.add_argument("--num_envs", type=int, default=4)
ap.add_argument("--steps", type=int, default=300, help="T3 무작위 행동 스텝 수(정책 스텝)")
ap.add_argument("--out", required=True)
add_launcher_args(ap)
args = ap.parse_args()

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

import waffle_rl  # noqa: E402,F401
from waffle_rl.reach_env_cfg import CONTACT_SENSORS, TCP_OFFSET, RL1ArmReachEnvCfg, RL1ArmReachV2EnvCfg  # noqa: E402
from waffle_rl.robot_cfg import ARM_JOINTS  # noqa: E402

V2 = args.test == "T5"
cfg = RL1ArmReachV2EnvCfg() if V2 else RL1ArmReachEnvCfg()
cfg.scene.num_envs = args.num_envs
cfg.seed = 0
result = {"test": args.test, "num_envs": args.num_envs}


def gpu_used_mib():
    out = subprocess.run(["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
                         capture_output=True, text=True).stdout
    return int(out.strip().splitlines()[0])


def contacts(u):
    """환경 0의 팔 몸체 × 필터 몸체 접촉 힘(N) 중 0.1 N 넘는 쌍."""
    pairs = {}
    for name in CONTACT_SENSORS:
        s = u.scene.sensors[name]
        fm = torch.linalg.norm(s.data.force_matrix_w.torch[0, 0], dim=-1).cpu().numpy()
        filt = s.cfg.filter_prim_paths_expr
        for f, v in zip(filt, fm):
            if v > 0.1:
                pairs[f"{name[len('contact_'):]}|{f.split('/')[-1]}"] = round(float(v), 3)
    return pairs


t_start = time.time()
with launch_simulation(cfg, args):
    from isaaclab.managers import SceneEntityCfg

    from waffle_rl import mdp as rl1  # Kit가 뜬 뒤에 불러온다

    env = gym.make("Waffle-RL1-ArmReachV2" if V2 else "Waffle-RL1-ArmReach", cfg=cfg)
    u = env.unwrapped
    robot = u.scene["robot"]
    arm_ids = [robot.joint_names.index(j) for j in ARM_JOINTS]
    body5 = robot.body_names.index("arm_link_5")
    tcp_cfg = SceneEntityCfg("robot", body_names=["arm_link_5"])
    tcp_cfg.resolve(u.scene)
    env.reset()
    result["startup_s"] = round(time.time() - t_start, 1)
    act_term = u.action_manager.get_term("arm")

    def action_for(q):
        """관절 목표 q(N, 4) → 행동 [-1, 1]."""
        lo, hi = act_term._lo, act_term._hi
        return 2.0 * (q - lo) / (hi - lo) - 1.0

    if args.test == "T1":
        v = robot.root_view
        names = robot.joint_names
        rd = lambda a: dict(zip(names, [float(x) for x in (a.numpy()[0] if hasattr(a, "numpy") else np.asarray(a)[0])]))
        result["joint_names"] = names
        result["body_names"] = robot.body_names
        result["physx"] = {
            "stiffness": rd(v.get_dof_stiffnesses()), "damping": rd(v.get_dof_dampings()),
            "armature": rd(v.get_dof_armatures()), "max_force": rd(v.get_dof_max_forces()),
            "max_velocity": rd(v.get_dof_max_velocities()),
            "limits": dict(zip(names, np.asarray(v.get_dof_limits().numpy()[0]).astype(float).tolist())),
        }
        result["action_dim"] = int(u.action_manager.total_action_dim)
        result["obs_dim"] = int(u.observation_manager.group_obs_dim["policy"][0])
        result["physics_dt"] = u.physics_dt
        result["step_dt"] = u.step_dt
        result["action_rate_step_rad"] = float(act_term._step)
        result["action_lo_hi"] = [act_term._lo[0].tolist(), act_term._hi[0].tolist()]
        result["root_pos_w_env0"] = robot.data.root_pos_w.torch[0].tolist()
        from pxr import UsdPhysics  # noqa: E402
        stage = u.sim.stage
        rp = stage.GetPrimAtPath("/World/envs/env_0/Robot/Geometry/base_footprint/base_link")
        result["usd_root"] = {a.GetName(): str(a.Get()) for a in rp.GetAttributes()
                              if "elfCollision" in a.GetName() or "articulation" in a.GetName().lower()}
        fixed = [str(p.GetPath()) for p in stage.Traverse()
                 if p.IsA(UsdPhysics.FixedJoint) and str(p.GetPath()).startswith("/World/envs/env_0/")]
        result["fixed_joints_env0"] = fixed
        # 정지 상태 접촉(리셋 자세) 1 s
        hold = action_for(robot.data.joint_pos.torch[:, arm_ids])
        for _ in range(30):
            env.step(hold)
        result["contacts_reset_pose"] = contacts(u)
        # 알려진 충돌 자세로 이동 4 s, 접촉 기록
        q_hit = torch.tensor([[0.0, 1.5, 1.5, 1.7]], device=u.device).expand(args.num_envs, 4)
        seen = {}
        for k in range(120):
            env.step(action_for(q_hit))
            for p, f in contacts(u).items():
                seen[p] = max(seen.get(p, 0.0), f)
        result["contacts_collision_pose_max_N"] = seen
        result["q_collision_pose_end"] = robot.data.joint_pos.torch[0, arm_ids].tolist()
        result["moveit_record"] = ["base_link|arm_link_3 0.0084 m", "base_link|arm_link_4 0.0034 m",
                                   "gripper_right_link|lidar_rear 0.0148 m"]

    elif args.test == "T2":
        q0 = robot.data.joint_pos.torch[:, arm_ids].clone()
        tcp0 = rl1.tcp_pos_b(u, tcp_cfg, TCP_OFFSET).clone()
        hold = action_for(q0)
        log = []
        for k in range(150):   # 5 s
            env.step(hold)
            q = robot.data.joint_pos.torch[:, arm_ids]
            qd = robot.data.joint_vel.torch[:, arm_ids]
            tcp = rl1.tcp_pos_b(u, tcp_cfg, TCP_OFFSET)
            log.append([(k + 1) * u.step_dt, (q - q0).abs().max().item(), torch.linalg.norm(tcp - tcp0, dim=1).max().item(),
                        qd.abs().max().item()])
        a = np.array(log)
        last1 = a[a[:, 0] > 4.0]
        result["q0_env0"] = q0[0].tolist()
        result["max_joint_drift_rad"] = float(a[:, 1].max())
        result["joint_drift_end_rad"] = float(a[-1, 1])
        result["max_tcp_drift_m"] = float(a[:, 2].max())
        result["tcp_drift_end_m"] = float(a[-1, 2])
        result["last1s_max_joint_vel_rad_s"] = float(last1[:, 3].max())
        result["episode_reset_in_window"] = bool(u.episode_length_buf.min().item() < 150)
        result["contacts_end"] = contacts(u)
        gl, gr = robot.joint_names.index("gripper_left_joint"), robot.joint_names.index("gripper_right_joint")
        result["gripper_left_right_end_m"] = [float(robot.data.joint_pos.torch[0, gl]), float(robot.data.joint_pos.torch[0, gr])]

    elif args.test == "T3":
        base = gpu_used_mib()
        peak = base
        t0 = time.time()
        for k in range(args.steps):
            a = 2 * torch.rand(env.action_space.shape, device=u.device) - 1
            env.step(a)
            if k % 50 == 0:
                peak = max(peak, gpu_used_mib())
        torch.cuda.synchronize()
        dt = time.time() - t0
        peak = max(peak, gpu_used_mib())
        result["gpu_used_after_reset_mib"] = base
        result["gpu_peak_mib"] = peak
        result["torch_max_alloc_mib"] = round(torch.cuda.max_memory_allocated() / 2**20, 1)
        result["policy_steps"] = args.steps
        result["wall_s"] = round(dt, 2)
        result["env_steps_per_s"] = round(args.steps * args.num_envs / dt, 1)
        # rsl_rl 1 iteration = 24 정책 스텝 × num_envs
        result["s_per_iteration_sim_only"] = round(24 * dt / args.steps, 3)

    elif args.test == "T5":
        arm5 = [i for i, n in enumerate(robot.joint_names) if n.startswith("RMX52_")]
        out = {}
        for j, name in enumerate(ARM_JOINTS):
            env.reset()
            q_start = robot.data.joint_pos.torch[0, arm_ids].tolist()
            a = torch.zeros(args.num_envs, 4, device=u.device)
            vmax, sat, rows = 0.0, [], []
            for k in range(45):           # 0.5 s 명령 + 1.0 s 정지
                a[:, j] = 1.0 if k < 15 else 0.0
                env.step(a)
                qd = robot.data.joint_vel.torch[0, arm5]
                tau = robot.data.computed_torque.torch[0, arm5]
                vmax = max(vmax, float(qd.abs().max()))
                sat.append(float((tau.abs() >= 3.8 - 1e-6).float().mean()))
                rows.append(float(qd.abs().max()))
            q = robot.data.joint_pos.torch[0, arm_ids].tolist()
            out[name] = {"vel_max_rad_s": round(vmax, 3), "sat_ratio_move": round(float(np.mean(sat[:15])), 3),
                         "sat_ratio_all": round(float(np.mean(sat)), 3), "vel_end_rad_s": round(rows[-1], 4),
                         "q_start": [round(x, 3) for x in q_start], "q_end": [round(x, 3) for x in q],
                         "contacts": contacts(u)}
        result["per_joint"] = out

    elif args.test == "T4":
        term = u.command_manager.get_term("tcp_target")
        n = args.num_envs
        errs = []
        idx_all = torch.arange(0, min(1000, len(term.table_q)), device=u.device)
        for s in range(0, len(idx_all), n):
            idx = idx_all[s:s + n]
            m = len(idx)
            pos = robot.data.default_joint_pos.torch.clone()
            pos[:m][:, arm_ids] = term.table_q[idx]
            robot.write_joint_state_to_sim_index(position=pos, velocity=torch.zeros_like(pos))
            u.scene.write_data_to_sim()
            u.sim.forward()
            u.scene.update(dt=0.0)
            tcp = rl1.tcp_pos_b(u, tcp_cfg, TCP_OFFSET)[:m]
            errs.append(torch.linalg.norm(tcp - term.table_tcp[idx], dim=1).cpu().numpy())
        e = np.concatenate(errs)
        result["n_compared"] = int(len(e))
        result["err_max_m"] = float(e.max())
        result["err_mean_m"] = float(e.mean())
        result["pass_le_1mm"] = bool(e.max() <= 1e-3)

    # Kit 종료가 프로세스를 바로 끝내므로 결과는 with 블록 안에서 저장한다
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(result, ensure_ascii=False, indent=1))
    print("RESULT", json.dumps({k: v for k, v in result.items() if k not in ("physx", "body_names", "joint_names")},
                               ensure_ascii=False), flush=True)
    env.close()
