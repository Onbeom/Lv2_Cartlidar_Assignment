"""누운 직사각형 퍽 집기·세우기 평가 (창 없이): 학습된 정책 (결정적 행동) 으로 환경마다 에피소드 1 번 ->
성공률 (들고·세우고·가운데·집게 수평), 쥔 대수, 가운데에서 벗어난 거리, 최대 세움 (긴 축 z), 옆 퍽 닿음·쓰러뜨림, 떨어뜨림.

    ./run_rl.sh liepick_eval [환경 수]
"""

import argparse
import glob
import os
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=64)
parser.add_argument("--policy", default=None)
parser.add_argument("--near", type=float, default=0.0, help="역방향 커리큘럼 시작 비율 (0 = 실제 과제)")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import waffle_isaac  # noqa: E402,F401
from waffle_isaac.liepick_env_cfg import WaffleLiePickEnvCfg  # noqa: E402


def main():
    path = args_cli.policy or sorted(glob.glob(os.path.join(HERE, "logs/rsl_rl/waffle_liepick/*/exported/policy.pt")))[-1]
    cfg = WaffleLiePickEnvCfg()
    cfg.scene.num_envs = args_cli.num_envs
    cfg.near_start_prob = args_cli.near
    env = gym.make("Waffle-LiePick-Direct", cfg=cfg)
    u = env.unwrapped
    pol = torch.jit.load(path, map_location=u.device).eval()
    obs, _ = env.reset()
    n, dev = u.num_envs, u.device
    done = torch.zeros(n, dtype=torch.bool, device=dev)
    ok = torch.zeros(n, dtype=torch.bool, device=dev)
    held = torch.zeros(n, dtype=torch.bool, device=dev)
    lifted = torch.zeros(n, dtype=torch.bool, device=dev)
    off_at_grasp = torch.full((n,), float("nan"), device=dev)
    stand = torch.zeros(n, device=dev)
    nb_touch = torch.zeros(n, dtype=torch.bool, device=dev)
    nb_top = torch.zeros(n, dtype=torch.bool, device=dev)
    fell = torch.zeros(n, dtype=torch.bool, device=dev)
    arm_box = torch.zeros(n, dtype=torch.bool, device=dev)
    span_L = torch.full((n,), float("nan"), device=dev)  # 쥔 순간 손가락 높이 방향 (집게 위 축) · 퍽 긴 축 |cos| (1 = 손가락이 긴 축 따라 놓임 = 폭 30 mm 를 쥠)
    close_L = torch.full((n,), float("nan"), device=dev)  # 닫는 축 · 긴 축 |cos| (1 = 긴 축 60 mm 방향으로 닫음 = 끝면을 쥠)
    appr_z = torch.full((n,), float("nan"), device=dev)
    q5_g = torch.full((n,), float("nan"), device=dev)
    nb_pre = torch.zeros(n, dtype=torch.bool, device=dev)  # 쥐기 전에 옆 퍽 닿음
    nb_post = torch.zeros(n, dtype=torch.bool, device=dev)  # 쥔 뒤 (들거나 옮기며) 옆 퍽 닿음
    for _ in range(int(cfg.episode_length_s / u.step_dt)):
        with torch.inference_mode():
            a = pol(obs["policy"]).clone()
        live = ~done
        g = u.grasp_w()
        fing = u.robot.data.joint_pos.torch[:, u._gl[0]]
        h = (fing > cfg.grasp_finger_range[0]) & (fing < cfg.grasp_finger_range[1]) & (torch.linalg.norm(u.puck_pos_w() - g, dim=1) < 0.03) & live
        first = h & ~held
        off_at_grasp = torch.where(first, u.center_off().abs(), off_at_grasp)
        ax_, ay_, az_ = u.a5_axes_w()
        Lw = u.long_axis_w()
        span_L = torch.where(first, ((-ay_) * Lw).sum(1).abs(), span_L)
        close_L = torch.where(first, (ax_ * Lw).sum(1).abs(), close_L)
        appr_z = torch.where(first, (-az_)[:, 2], appr_z)
        q5_g = torch.where(first, u.robot.data.joint_pos.torch[:, u._arm[4]], q5_g)
        nbt = u._contact("arm_nb_contact") & live
        nb_pre |= nbt & ~held
        nb_post |= nbt & held
        held |= h
        lifted |= h & (u.lift() > cfg.lift_ok)
        stand = torch.maximum(stand, u.long_axis_w()[:, 2].abs() * (h & (u.lift() > cfg.lift_ok)).float())
        nb_touch |= u._contact("arm_nb_contact") & live
        nb_top |= u.nb_toppled() & live
        arm_box |= u._contact("arm_box_contact") & live
        obs, _, term, trunc, _ = env.step(a)
        ok |= u.succeeded & live
        fell |= term & live & u._fell()
        done |= (term | trunc) & live
    offm = off_at_grasp[~torch.isnan(off_at_grasp)] * 1000
    print(f"[LieEval] 정책 {path} (역방향 시작 {args_cli.near})", flush=True)
    print(f"[LieEval] 성공 (들고·세우고·가운데·수평) {int(ok.sum())}/{n} ({ok.float().mean().item() * 100:.0f} %) | 쥠 {int(held.sum())} · 3 cm 들어 올림 {int(lifted.sum())} · "
          f"최대 세움 (긴 축 z) 평균 {stand[lifted].mean().item() if lifted.any() else 0:.2f}, 0.996 넘음 {int((stand > 0.996).sum())}", flush=True)
    if len(offm):
        print(f"[LieEval] 쥔 순간 가운데에서 벗어난 거리 mm: 평균 {offm.mean().item():.1f}, 4 mm 안 {int((offm < 4).sum())}/{len(offm)}, 값 {[round(v) for v in offm[:16].tolist()]}", flush=True)
    g = ~torch.isnan(span_L)
    if g.any():
        print(f"[LieEval] 쥔 순간 집게 방향: 손가락 높이 방향·긴 축 |cos| 평균 {span_L[g].mean().item():.2f} (0.9 넘음 {int((span_L[g] > 0.9).sum())}/{int(g.sum())}), "
              f"닫는 축·긴 축 |cos| 평균 {close_L[g].mean().item():.2f}, 집게 방향 z 평균 {appr_z[g].mean().item():.2f} (−1 = 바로 아래), 손목 롤 (관절 5) {[round(v, 2) for v in q5_g[g][:10].tolist()]}", flush=True)
    print(f"[LieEval] 옆 퍽 닿음: 쥐기 전 {int(nb_pre.sum())} · 쥔 뒤 {int(nb_post.sum())} (대 수)", flush=True)
    print(f"[LieEval] 옆 퍽 닿음 {int(nb_touch.sum())} · 쓰러뜨림 {int(nb_top.sum())} | 떨어뜨림 종료 {int(fell.sum())} | 팔-상자 닿음 {int(arm_box.sum())}", flush=True)
    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
