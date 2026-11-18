"""정렬·집기 진단 (창 없이): 학습된 정책을 돌리며 1 s 마다 잡는 점-목표 거리 (수평·높이), 집게 수평도, 손가락, 팔 관절 (한계), 접촉 출력.

    ./run_isaac.sh sidepick_diag [환경 수]
"""

import argparse
import glob
import math
import os
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=8)
parser.add_argument("--policy", default=None)
parser.add_argument("--pitch45", action="store_true", help="2026-11-03: 45° 기울여 쥐기 환경 (Waffle-SidePick45-Direct)")
parser.add_argument("--box45", action="store_true", help="2026-11-04: 세운 직육면체만 45° 쥐기 (Waffle-SidePick45Box-Direct)")
parser.add_argument("--lieroll", action="store_true", help="2026-11-04: 누운 직육면체 45° 쥐기 (Waffle-LieRoll45-Direct)")
parser.add_argument("--lieend", action="store_true", help="2026-11-04: 누운 직육면체 안쪽 끝 수평 쥐기 (Waffle-LieEnd-Direct)")
parser.add_argument("--near", type=float, default=0.0, help="역방향 커리큘럼 (퍽을 손가락 사이에 두고 시작) 비율. 기본 0 = 실제 과제만")
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
from waffle_isaac.sidepick_env_cfg import WaffleLieEndEnvCfg, WaffleLieRoll45EnvCfg, WaffleSidePick45BoxEnvCfg, WaffleSidePick45EnvCfg, WaffleSidePickEnvCfg  # noqa: E402


def main():
    path = args_cli.policy or sorted(glob.glob(os.path.join(HERE, "logs/rsl_rl/waffle_sidepick/*/exported/policy.pt")))[-1]
    cfg = WaffleSidePick45BoxEnvCfg() if args_cli.box45 else WaffleLieRoll45EnvCfg() if args_cli.lieroll else WaffleLieEndEnvCfg() if args_cli.lieend else (WaffleSidePick45EnvCfg() if args_cli.pitch45 else WaffleSidePickEnvCfg())
    cfg.scene.num_envs = args_cli.num_envs
    if hasattr(cfg, "near_start_prob"):
        cfg.near_start_prob = args_cli.near
    if args_cli.lieend or args_cli.lieroll or args_cli.box45:
        cfg.box_shape_prob = 1.0
    env = gym.make("Waffle-SidePick45Box-Direct" if args_cli.box45 else "Waffle-LieRoll45-Direct" if args_cli.lieroll else "Waffle-LieEnd-Direct" if args_cli.lieend else ("Waffle-SidePick45-Direct" if args_cli.pitch45 else "Waffle-SidePick-Direct"), cfg=cfg)
    u = env.unwrapped
    pol = torch.jit.load(path, map_location=u.device).eval()
    obs, _ = env.reset()
    print("[Diag] 정책", path, flush=True)
    # 2026-10-31: 옆 퍽이 목표 퍽 기준 어디 있나 (joint1 축에서 본 반지름 차 dr = 옆 − 목표 (− = 옆 퍽이 로봇 쪽), 접선 차 dt) - 첫 에피소드 배치
    j1 = torch.tensor([cfg.joint1_x, 0.0], device=u.device)
    tb0 = u.to_base(u.puck_pos_w())[:, :2] - j1
    nb0 = u.to_base(u._obj("puckN").data.root_pos_w.torch)[:, :2] - j1
    rad = tb0 / tb0.norm(dim=1, keepdim=True)
    dr0 = ((nb0 - tb0) * rad).sum(1)
    dt0 = ((nb0 - tb0) * torch.stack((-rad[:, 1], rad[:, 0]), dim=1)).sum(1).abs()
    nbt = torch.zeros(u.num_envs, dtype=torch.bool, device=u.device)  # 2026-10-31: 옆 퍽 (목표 아님) 에 한 번이라도 닿은 대
    nbtop = torch.zeros(u.num_envs, dtype=torch.bool, device=u.device)
    ended = torch.zeros(u.num_envs, dtype=torch.bool, device=u.device)  # 2026-10-31: 한 번 시도 기준 (끝난 대는 리셋 뒤 다시 시도해도 실패로 셈)
    toppled1 = torch.zeros(u.num_envs, dtype=torch.bool, device=u.device)
    fell1 = torch.zeros(u.num_envs, dtype=torch.bool, device=u.device)
    # 2026-11-03 SP-45b: 팔-몸체 접촉 (한 번 시도 동안 최대 힘), 든 퍽 기울기 최대, 처음 들었을 때 (lift 3 cm) 잡는 높이
    has_ab = "arm_body_contact" in u.scene.keys()
    ab_max = torch.zeros(u.num_envs, device=u.device)
    ab1 = torch.zeros(u.num_envs, dtype=torch.bool, device=u.device)
    tilt_max = torch.zeros(u.num_envs, device=u.device)
    gdz1 = torch.full((u.num_envs,), float("nan"), device=u.device)
    sd_min = torch.full((u.num_envs,), 9.0, device=u.device)  # 2026-11-04 자기 충돌 근접 최소 거리 · 관절 3 최대
    j3_max = torch.zeros(u.num_envs, device=u.device)
    for t in range(int(cfg.episode_length_s / u.step_dt) - 2):
        with torch.inference_mode():
            a = pol(obs["policy"])
        nbt |= u._contact("arm_nb_contact") & ~ended
        nbtop |= u.nb_toppled() & ~ended
        toppled1 |= u.target_toppled() & ~ended
        if has_ab:
            fab = torch.linalg.norm(u.scene["arm_body_contact"].data.normal_force_matrix_w.torch, dim=-1).amax(dim=(1, 2))
            ab_max = torch.where(ended, ab_max, torch.maximum(ab_max, fab))
        carried = (u.lift() > 0.02) & (torch.linalg.norm(u.puck_pos_w() - u.grasp_w(), dim=1) < 0.03) & ~ended
        tilt_max = torch.where(carried, torch.maximum(tilt_max, torch.rad2deg(torch.acos(u.puck_up().clamp(max=1.0)))), tilt_max)
        first = carried & (u.lift() > 0.03) & torch.isnan(gdz1)
        gdz1 = torch.where(first, (u.grasp_w()[:, 2] - u.puck_pos_w()[:, 2]) * 1000, gdz1)
        sd_min = torch.where(ended, sd_min, torch.minimum(sd_min, u.self_dist()))
        j3_max = torch.where(ended, j3_max, torch.maximum(j3_max, u.robot.data.joint_pos.torch[:, u._arm[2]].abs()))
        obs, _, term, _, _ = env.step(a)
        new_end = term & ~ended
        if has_ab:
            ab1 |= new_end & u.term_arm_body
        nbt |= new_end & u.term_nb
        toppled1 |= new_end & u.term_topple
        fell1 |= new_end & u.term_fell
        ended |= term
        if t % 30 == 29:
            g = u.grasp_w()
            to_t = u.grasp_target_w() - g
            ax, ay, az = u.a5_axes_w()
            appr = -az
            tb = u.to_base(u.puck_pos_w())
            print(f"[Diag] t={(t + 1) * u.step_dt:.1f}s 퍽 (base) {[[round(v, 3) for v in r] for r in tb[:4].tolist()]}", flush=True)
            print(f"[Diag]   목표까지 수평 mm {[round(v * 1000) for v in to_t[:8, :2].norm(dim=1).tolist()]} 높이 mm {[round(v * 1000) for v in to_t[:8, 2].tolist()]}"
                  f" | 집게 방향 z {[round(v, 2) for v in appr[:8, 2].tolist()]} | 집게가 퍽 향함 {[round(v, 2) for v in ((appr[:8, :2] * torch.nn.functional.normalize(to_t[:8, :2], dim=1)).sum(1)).tolist()]}", flush=True)
            print(f"[Diag]   손가락 {[round(v, 4) for v in u.robot.data.joint_pos.torch[:8, u._gl[0]].tolist()]} 집게 행동 {[round(v, 1) for v in a[:8, -1].tolist()]}"
                  f" | 팔-상자 접촉 {u._contact('arm_box_contact')[:8].int().tolist()} | 퍽 직립 {[round(v, 2) for v in u.puck_up()[:8].tolist()]}", flush=True)
            q = u.robot.data.joint_pos.torch[:, u._arm]
            print(f"[Diag]   팔 관절 env0~2 {[[round(v, 2) for v in r] for r in q[:3].tolist()]} 한계 {[round(v, 2) for v in u._lo.tolist()]} ~ {[round(v, 2) for v in u._hi.tolist()]}", flush=True)
    lift = u.lift()
    held = (lift > 0.03) & (torch.linalg.norm(u.grasp_target_w() - u.grasp_w(), dim=1) < 0.03) & (u.puck_up() > 0.95)
    print(f"[Diag] 끝: 쥐고 3 cm 넘게 세워 든 대 {int(held.sum())}/{u.num_envs} ({held.float().mean().item() * 100:.0f} %), 넘어진 퍽 {int((u.puck_up() < 0.7).sum())}, "
          f"들기 mm {[round(v * 1000) for v in lift[:16].tolist()]}", flush=True)
    print(f"[Diag] **한 번 시도 기준**: 쥐고 3 cm 세워 듦 {int((held & ~ended).sum())}/{u.num_envs}, 중간 종료 {int(ended.sum())} (옆 퍽 닿음·쓰러뜨림 {int((ended & nbt).sum())}, 목표 퍽 넘어뜨림 {int(toppled1.sum())}, 상자에서 떨어짐 {int(fell1.sum())})", flush=True)
    for nm, m in (("옆 퍽이 로봇 쪽 (dr < −2 cm)", dr0 < -0.02), ("옆 (|dr| ≤ 2 cm)", dr0.abs() <= 0.02), ("옆 퍽이 바깥 쪽 (dr > 2 cm)", dr0 > 0.02)):
        if m.any():
            print(f"[Diag]   배치 {nm}: {int(m.sum())} 대 -> 옆 퍽 닿음 {int((nbt & m).sum())}, 한 번 시도 성공 {int((held & ~ended & m).sum())} | 접선 차 평균 {dt0[m].mean().item() * 100:.1f} cm", flush=True)
    ok1 = held & ~ended
    if has_ab:
        print(f"[Diag] 팔-몸체 (1 N 넘음, 한 번 시도): {int(((ab_max > 1.0) | ab1).sum())}/{u.num_envs} 대 (그중 닿아서 종료 {int(ab1.sum())}, 종료 전 최대 {ab_max.max().item():.1f} N - 종료 스텝 힘은 리셋으로 못 읽음)", flush=True)
    if ok1.any():
        tm, gz = tilt_max[ok1], gdz1[ok1]
        gz = gz[~torch.isnan(gz)]
        print(f"[Diag] 든 퍽 기울기 최대 (성공 {int(ok1.sum())} 대): 평균 {tm.mean().item():.1f}° · 5° 넘은 대 {int((tm > 5).sum())} · 10° 넘은 대 {int((tm > 10).sum())} (최대 {tm.max().item():.1f}°)"
              f" | 잡는 점-퍽 중심 높이 mm (목표 {cfg.grasp_dz * 1000:.0f}): 평균 {gz.mean().item():.1f} 범위 {gz.min().item():.1f} ~ {gz.max().item():.1f}", flush=True)
    print(f"[Diag] 자기 충돌 근접 (집게 · 쥔 퍽 - 윗팔 · 아랫팔 < {cfg.self_margin * 1000:.0f} mm): {int((sd_min < cfg.self_margin).sum())}/{u.num_envs} 대 (최소 {sd_min.min().item() * 1000:.0f} mm) | 관절 3 최대 {math.degrees(j3_max.max().item()):.0f}°, 80° 넘은 대 {int((j3_max > 1.396).sum())}", flush=True)
    print(f"[Diag] 옆 퍽 (목표 아님): 닿은 대 {int(nbt.sum())}/{u.num_envs}, 쓰러뜨린 대 {int(nbtop.sum())}/{u.num_envs}", flush=True)
    env.close()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
    finally:
        simulation_app.close()
