"""누운 직사각형 퍽: 집기·들기는 학습된 LiePick 정책, 세로로 세우기는 손 IK (2026-10-31, LP-7 까지 정책이 세우기 (φ) 를 못 배움).

단계: 0 정책 (쥐고 3 cm 넘게 든 상태가 0.3 s 이어지면 다음) -> 1 손 IK: 잡는 점을 퍽 처음 높이 + lift_to 로 올리며 집게 요 ψ 를 팔 방향으로
-> 2 집게 숙임 φ 를 0 -> π/2 (phi_rate / 스텝) -> 3 그대로 들고 있음. 결과 = 세움 (긴 축 z > 0.996)·가운데·쥠 유지·팔-상자·옆 퍽.

    ./run_rl.sh liepick_hybrid [환경 수]       (창: liepick_hybrid_play)
"""

import argparse
import glob
import math
import os
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=64)
parser.add_argument("--policy", default=None)
parser.add_argument("--lift_to", type=float, default=0.09, help="[m] 세우기 전 퍽을 이만큼 들어 올림")
parser.add_argument("--phi_rate", type=float, default=0.025, help="[rad / 스텝] 집게 숙임 속도")
parser.add_argument("--gui", action="store_true")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = not args_cli.gui
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import waffle_isaac  # noqa: E402,F401
from isaaclab.utils.math import quat_apply  # noqa: E402
from waffle_isaac.liepick_env_cfg import WaffleLiePickEnvCfg  # noqa: E402


def main():
    path = args_cli.policy or sorted(glob.glob(os.path.join(HERE, "logs/rsl_rl/waffle_liepick/*/exported/policy.pt")))[-1]
    cfg = WaffleLiePickEnvCfg()
    cfg.scene.num_envs = args_cli.num_envs
    cfg.near_start_prob = 0.0
    cfg.episode_length_s = 14.0
    env = gym.make("Waffle-LiePick-Direct", cfg=cfg)
    u = env.unwrapped
    pol = torch.jit.load(path, map_location=u.device).eval()
    obs, _ = env.reset()
    n, dev = u.num_envs, u.device
    z = torch.zeros(n, dtype=torch.bool, device=dev)
    u._get_dones = lambda: (z, z)  # 끝까지 한 에피소드로 봄 (중간 종료·리셋 안 함)
    phase = torch.zeros(n, dtype=torch.long, device=dev)
    cnt = torch.zeros(n, dtype=torch.long, device=dev)
    lost = torch.zeros(n, dtype=torch.bool, device=dev)
    arm_box = torch.zeros(n, dtype=torch.bool, device=dev)
    nb_touch = torch.zeros(n, dtype=torch.bool, device=dev)
    stand_max = torch.zeros(n, device=dev)
    off_at_lift = torch.full((n,), float("nan"), device=dev)
    j1o = torch.tensor([[cfg.joint1_x, 0.0, 0.0]], device=dev).expand(n, 3)
    print(f"[LieHyb] 정책 {path}", flush=True)
    for k in range(int(cfg.episode_length_s / u.step_dt)):
        with torch.inference_mode():
            a = pol(obs["policy"]).clone()
        g = u.grasp_w()
        fing = u.robot.data.joint_pos.torch[:, u._gl[0]]
        holding = (fing > cfg.grasp_finger_range[0]) & (fing < cfg.grasp_finger_range[1]) & (torch.linalg.norm(u.puck_pos_w() - g, dim=1) < 0.03)
        lift = u.lift()
        # 0 -> 1: 쥐고 3 cm 넘게 든 상태 0.3 s
        ok0 = (phase == 0) & holding & (lift > cfg.lift_ok)
        cnt = torch.where(ok0, cnt + 1, torch.where(phase == 0, torch.zeros_like(cnt), cnt + 1))
        go1 = (phase == 0) & (cnt >= 9)
        off_at_lift = torch.where(go1, u.center_off().abs(), off_at_lift)
        phase[go1], cnt[go1] = 1, 0
        # 손 IK 행동 (환경 행동 공간 그대로: 잡는 점 변위 /cart_scale, ψ·φ 변화 /scale, 집게 닫음)
        j1 = u.robot.data.root_link_pos_w.torch + quat_apply(u.robot.data.root_link_quat_w.torch, j1o)
        gb = u.vec_base(g - j1)
        th = torch.atan2(gb[:, 1], gb[:, 0])
        dpsi = torch.atan2(torch.sin(th - u.psi), torch.cos(th - u.psi))
        m12 = (phase == 1) | (phase == 2) | (phase == 3)
        if m12.any():
            act = torch.zeros(n, 6, device=dev)
            dz = (u.puck_z0 + args_cli.lift_to - u.puck_pos_w()[:, 2]).clamp(-0.005, 0.005)
            act[:, 2] = dz / cfg.cart_scale
            act[:, 3] = (dpsi / cfg.psi_scale).clamp(-1, 1)
            act[:, 4] = torch.where(phase == 2, args_cli.phi_rate / cfg.phi_scale, 0.0)
            act[:, 5] = -1.0
            a = torch.where(m12[:, None], act.clamp(-1, 1), a)
            go2 = (phase == 1) & (dpsi.abs() < 0.05) & (lift > args_cli.lift_to - 0.01)
            go2 |= (phase == 1) & (cnt > 90)
            phase[go2], cnt[go2] = 2, 0
            go3 = (phase == 2) & (u.phi > math.pi / 2 - 1e-3) & (cnt > 10)
            phase[go3], cnt[go3] = 3, 0
        lost |= (phase >= 1) & ~holding
        arm_box |= u._contact("arm_box_contact")
        nb_touch |= u._contact("arm_nb_contact")
        stand_max = torch.maximum(stand_max, u.long_axis_w()[:, 2].abs() * holding.float() * (phase >= 2).float())
        obs, _, term, trunc, _ = env.step(a)
        if k % 60 == 59:
            print(f"[LieHyb] t={(k + 1) * u.step_dt:4.1f}s 단계 {phase.tolist()[:16]} 들기 mm {[round(v * 1000) for v in lift.tolist()[:8]]} "
                  f"긴 축 z {[round(v, 2) for v in u.long_axis_w()[:8, 2].abs().tolist()]} φ {[round(v, 2) for v in u.phi[:8].tolist()]}", flush=True)
    stand = u.long_axis_w()[:, 2].abs()
    g = u.grasp_w()
    fing = u.robot.data.joint_pos.torch[:, u._gl[0]]
    holding = (fing > cfg.grasp_finger_range[0]) & (fing < cfg.grasp_finger_range[1]) & (torch.linalg.norm(u.puck_pos_w() - g, dim=1) < 0.03)
    ok = holding & (stand > math.cos(math.radians(5))) & (u.center_off().abs() < 0.006)
    reached = phase >= 1
    offm = off_at_lift[~torch.isnan(off_at_lift)] * 1000
    print(f"[LieHyb] 정책으로 쥐고 듦 {int(reached.sum())}/{n} | 손 IK 로 세움 (5° 안·가운데 6 mm·쥠 유지) **{int(ok.sum())}/{n}** "
          f"(든 것 중 {ok[reached].float().mean().item() * 100 if reached.any() else 0:.0f} %) | 세우는 중 놓침 {int((lost & reached).sum())}", flush=True)
    print(f"[LieHyb] 끝 긴 축 z (든 대) {[round(v, 2) for v in stand[reached][:16].tolist()]} | 최대 {[round(v, 2) for v in stand_max[reached][:16].tolist()]}", flush=True)
    if len(offm):
        print(f"[LieHyb] 들 때 가운데 벗어남 mm 평균 {offm.mean().item():.1f}, 4 mm 안 {int((offm < 4).sum())}/{len(offm)} | 팔-상자 닿음 {int(arm_box.sum())} · 옆 퍽 닿음 {int(nb_touch.sum())}", flush=True)
    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
