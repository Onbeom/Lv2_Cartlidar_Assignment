"""주사위 눈 놓기 진단 (2026-11-07, 창 없이): 정책 (또는 --script 각본) 으로 대마다 한 번씩 놓아 보고 끝 이유 · 놓은 오차를 셈.

--script: 학습 없이 환경이 맞는지 확인하는 각본 (가정 퍽 위치를 칸 위 4 cm 로 -> 내림 -> 벌림 -> 위로). 같은 행동 (잡는 점 이동 + 집게) 만 씀.

    ./run_place.sh diag [환경 수] [정책 .pt]     ./run_place.sh check [환경 수]   (= --script)
"""

import argparse
import os
import sys
from collections import Counter

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=64)
parser.add_argument("--policy", default=None)
parser.add_argument("--script", action="store_true")
parser.add_argument("--l2", action="store_true", help="2026-11-08 L2 받침 (45° 쥐기, Waffle-Place45L2-Direct)")
parser.add_argument("--seconds", type=float, default=8.0)
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
from waffle_isaac.place_env_cfg import WafflePlace45L2EnvCfg, WafflePlace60EnvCfg  # noqa: E402

NAMES = {0: "시간 끝", 1: "성공", 2: "나쁜 놓기 (어긋남 · 높음)", 3: "놓기 전 빠뜨림", 4: "놓은 퍽 쓰러짐 · 떨어짐", 5: "먼저 놓은 퍽 건드림", 6: "자기 충돌 · 몸체"}


def main():
    cfg = WafflePlace45L2EnvCfg() if args_cli.l2 else WafflePlace60EnvCfg()
    cfg.scene.num_envs = args_cli.num_envs
    env = gym.make("Waffle-Place45L2-Direct" if args_cli.l2 else "Waffle-Place60-Direct", cfg=cfg)
    u = env.unwrapped
    n, dev = u.num_envs, u.device
    pol = None if args_cli.script else torch.jit.load(args_cli.policy, map_location=dev).eval()
    obs, _ = env.reset()
    done_once = torch.zeros(n, dtype=torch.bool, device=dev)
    reason = torch.zeros(n, dtype=torch.long, device=dev)
    err = torch.zeros(n, device=dev)
    rel_h = torch.zeros(n, device=dev)
    stage = torch.zeros(n, dtype=torch.long, device=dev)  # 각본: 0 칸 위로, 1 내림, 2 벌림, 3 위로
    cnt = torch.zeros(n, dtype=torch.long, device=dev)
    steps = int(args_cli.seconds / u.step_dt) + 2
    for k in range(steps):
        if pol is not None:
            with torch.inference_mode():
                a = pol(obs["policy"]).clamp(-1, 1)
        else:
            pb = u.to_base(u.assumed_puck_w())
            tb = u.to_base(u.cell_w + u.cell_bias)
            goal = tb.clone()
            goal[:, 2] = torch.where(stage == 0, tb[:, 2] + 0.04, tb[:, 2] + 0.004)
            d = goal - pb
            up = torch.zeros_like(d); up[:, 2] = 1.0
            d = torch.where((stage >= 3)[:, None], up * 0.01, d)
            a = torch.zeros(n, 4, device=dev)
            a[:, :3] = torch.where((stage == 2)[:, None], torch.zeros_like(d), (d / u.cfg.cart_scale).clamp(-1, 1))
            a[:, 3] = torch.where(stage >= 2, 1.0, -1.0)  # 벌림 = 1 (release_thresh 0.5 넘음)
            cnt += 1
            nx = ((stage == 0) & (torch.linalg.norm(d[:, :2], dim=1) < 0.002) & (d[:, 2].abs() < 0.003)) | \
                 ((stage == 1) & (d.norm(dim=1) < 0.002)) | ((stage == 2) & (cnt > 10))
            stage = torch.where(nx, stage + 1, stage)
            cnt = torch.where(nx, torch.zeros_like(cnt), cnt)
        obs, rew, term, trunc, _ = env.step(a)
        fin = (term | trunc) & ~done_once
        reason = torch.where(fin, u.last_reason, reason)
        err = torch.where(fin, u.last_err, err)
        rel_h = torch.where(fin, u.rel_h, rel_h)  # 리셋 뒤라 0 일 수 있음 (참고만)
        done_once |= term | trunc
        if pol is None:
            stage = torch.where(term | trunc, torch.zeros_like(stage), stage)
        if k == steps // 2 or k == steps - 1:  # 2026-11-09 놓은 뒤 상태 (상자 0/64 원인: 놓기는 2.5 mm 로 잘하는데 성공 0)
            rl = u.released
            if rl.any():
                print(f"[PlaceDbg] step {k}: 놓은 대 {int(rl.sum())}/{n}, 잡는 점-퍽 mm 평균 {u.s_gd[rl].mean().item()*1000:.0f} 최소 {u.s_gd[rl].min().item()*1000:.0f}, "
                      f"칸 오차 mm {u.s_exy[rl].mean().item()*1000:.1f}, 퍽 높이 mm {u.s_h[rl].mean().item()*1000:.1f}, 기울기° {torch.rad2deg(u.s_tilt[rl]).mean().item():.1f}, "
                      f"밀림 mm {u.s_moved[rl].mean().item()*1000:.1f}, ok_cnt 최대 {int(u.ok_cnt[rl].max())}, 집게 행동 평균 {u.actions[rl, 3].mean().item():.2f}", flush=True)
    c = Counter(NAMES[int(v)] for v in reason[done_once].tolist())
    ok = (reason == 1) & done_once
    print(f"[PlaceDiag] {'각본' if pol is None else args_cli.policy}: 끝난 대 {int(done_once.sum())}/{n}, 끝 이유 {dict(c)}", flush=True)
    print(f"[PlaceDiag] **성공 {int(ok.sum())}/{n}**, 놓은 순간 칸 중심 오차 mm (성공 대) 평균 {err[ok].mean().item() * 1000 if ok.any() else float('nan'):.1f} "
          f"최대 {err[ok].max().item() * 1000 if ok.any() else float('nan'):.1f}", flush=True)
    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
