"""꺼내기 환경 (Waffle-Unload60-Direct) 확인 (2026-11-10): 정책 없이 같은 행동 (잡는 점 직교 이동 + 집게) 으로
퍽 위 4 cm -> 잡는 점 (퍽 중심 + 15 mm) 으로 내려감 -> 닫기 -> 위로. 각본으로도 못 들면 환경 (기하 · 판정) 문제.
    ./run_rl.sh unload_check [환경 수]
"""

import argparse
import os
import sys
from collections import Counter

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=24)
parser.add_argument("--task", default="Waffle-Unload60-Direct")
parser.add_argument("--open", type=float, default=None, help="집게 벌림 (손가락 값, 49.6 mm = 0.004, 37.6 mm = -0.002)")
parser.add_argument("--fill", type=float, default=None, help="다른 칸 퍽 확률 (0 = 빈 팔레트)")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import waffle_isaac  # noqa: E402,F401
from waffle_isaac.unload_env_cfg import WaffleUnload45EnvCfg, WaffleUnload60EnvCfg  # noqa: E402


def main():
    cfg = (WaffleUnload45EnvCfg if "45" in args_cli.task else WaffleUnload60EnvCfg)()
    cfg.scene.num_envs = args_cli.num_envs
    cfg.perc_bias = 0.0
    cfg.ready_noise = 0.0
    if args_cli.open is not None:
        cfg.gripper_open = args_cli.open
    if args_cli.fill is not None:
        cfg.unload_fill_prob = args_cli.fill
        cfg.unload_nb_prob = args_cli.fill
    env = gym.make(args_cli.task, cfg=cfg)
    u = env.unwrapped
    n, dev = u.num_envs, u.device
    env.reset()
    stage = torch.zeros(n, dtype=torch.long, device=dev)  # 0 위, 1 내림, 2 닫기, 3 들기
    cnt = torch.zeros(n, dtype=torch.long, device=dev)
    done = torch.zeros(n, dtype=torch.bool, device=dev)
    rec = {}
    for k in range(int(6.0 / u.step_dt)):
        tgt = u.puck_pos_w() + u._ez * u.cfg.grasp_dz
        g = u.grasp_w()
        goal = tgt + u._ez * torch.where(stage == 0, 0.04, 0.0)[:, None]
        goal = torch.where((stage == 3)[:, None], g + u._ez * 0.01, goal)
        d = u.to_base(goal) - u.to_base(g)
        a = torch.zeros(n, 4, device=dev)
        a[:, :3] = torch.where((stage == 2)[:, None], torch.zeros_like(d), (d / u.cfg.cart_scale).clamp(-1, 1))
        a[:, 3] = torch.where(stage >= 2, -1.0, 1.0)
        cnt += 1
        e = torch.linalg.norm(goal - g, dim=1)
        nx = ((stage == 0) & (e < 0.003)) | ((stage == 1) & (e < 0.002)) | ((stage == 2) & (cnt > 15))
        for s_, nm in ((0, "위 못 감"), (1, "못 내려감")):
            stuck = (stage == s_) & (cnt > 90) & ~done
            for i in torch.nonzero(stuck).flatten().tolist():
                rec.setdefault(i, f"{nm} (남은 오차 {e[i].item() * 1000:.0f} mm)")
        stage = torch.where(nx, stage + 1, stage)
        cnt = torch.where(nx, torch.zeros_like(cnt), cnt)
        ab_f = torch.linalg.norm(u.scene["arm_body_contact"].data.normal_force_matrix_w.torch, dim=-1).amax(dim=2)  # (대, 몸) 이 스텝 전
        prev_ab = getattr(main, "_ab", None)
        main._ab = ab_f
        _, _, term, trunc, _ = env.step(a)
        for i in torch.nonzero((term | trunc) & ~done).flatten().tolist():
            bn = ""
            for key, sens in (("옆 칸 퍽", "arm_nb_contact"), ("다른 칸 퍽", "arm_obs_contact"), ("팔-몸체", "arm_body_contact")):
                f_ = torch.linalg.norm(u.scene[sens].data.normal_force_matrix_w.torch[i], dim=-1).amax(dim=1)
                if f_.max() > 1.0:
                    bn += f" {key}:{u.scene[sens].body_names[int(f_.argmax())]}"
            why = "옆 칸 퍽" if bool(u.term_nb[i]) else "다른 칸 퍽" if bool(u.term_obs[i]) else "팔-몸체" if bool(u.term_arm_body[i]) else "자기 충돌" if bool(u.term_self[i]) else "넘어짐 · 떨어짐" if bool(u.term_topple[i] | u.term_fell[i]) else "기타 종료"
            if why == "팔-몸체":
                f2 = main._ab[i]
                bn += f" 직전 몸 {u.scene['arm_body_contact'].body_names[int(f2.argmax())]} {f2.max().item():.1f} N, 스텝 {k}, 관절 {[round(v, 2) for v in u.robot.data.joint_pos.torch[i, u._arm].tolist()]}"
            rec.setdefault(i, f"종료: {why} (단계 {int(stage[i])}{bn})")
        done |= term | trunc
        stage = torch.where(done, torch.full_like(stage, 9), stage)
    lift = u.lift()
    for i in range(n):
        if i not in rec:
            rec[i] = f"들음 {lift[i].item() * 1000:.0f} mm" if lift[i] > 0.02 else f"못 듦 (단계 {int(stage[i])}, 들림 {lift[i].item() * 1000:.0f} mm, 손가락 {u.robot.data.joint_pos.torch[i, u._gl[0]].item():.4f})"
    slots = u.slot_k.tolist()
    print(f"[UnloadCheck] 성공 판정 {int(u.succeeded.sum())}/{n}, 결과 {dict(Counter(v.split(' (')[0] for v in rec.values()))}", flush=True)
    for i in range(n):
        print(f"[UnloadCheck]   env{i} 칸번호 {slots[i]} 직육면체 {bool(u.is_box[i])}: {rec[i]}", flush=True)
    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
