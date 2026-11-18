"""누운 직사각형 퍽 새 동작 확인 (정책 없이 손 IK, 2026-10-31 사용자와 정한 순서, 기록/1b '설계 결정').

순서: 0 위에서 접근 (손가락 높이 방향 = 퍽 긴 축, 집게 아래) -> 1 낮게 감싸 쥠 (잡는 점 = 퍽 가운데 + --dz) -> 2 들기 (--lift)
-> 3 위에서 쥔 채 롤로 긴 축을 팔 방향과 직각 (접선) 으로 -> 4 집게를 수평으로 (퍽은 자기 긴 축 둘레로만 돌아 누운 채)
-> 5 롤 −90° (손가락 높이 방향 접선 -> 위) = 퍽 세로 -> 6 들고 있기. 단계마다 쥠 유지·세움·가운데·상자 닿음을 잼.

    ./run_rl.sh lie_roll_test [환경 수] [dz]      (창: lie_roll_test_play)
"""

import argparse
import math
import os
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=32)
parser.add_argument("--dz", type=float, default=0.011, help="[m] 잡는 점 = 누운 퍽 가운데 + dz (0.015 = 윗면, 이전 방식)")
parser.add_argument("--lift", type=float, default=0.09, help="[m] 세우기 전 들기 높이")
parser.add_argument("--steps", type=int, default=45, help="3·4·5 단계 각각 걸리는 스텝 (30 Hz)")
parser.add_argument("--mode", default="joint", choices=("joint", "ik"), help="joint: 들고 나서 관절 공간에서 옆 쥐기 자세 (카메라 위) 로 바로 보간 (손가락 높이 방향 = 긴 축이라 끝 자세가 옆 쥐기면 퍽이 세로). ik: 롤로 접선 -> 수평 -> 롤 −90° 를 IK 로 따라감 (2026-10-31 첫 시험, 수평에서 막힘)")
parser.add_argument("--side_q", default="0.30,1.42,-1.72", help="옆 쥐기 끝 자세 관절 2·3·4 (joint1 = 팔 방향, joint5 = 0). 기본 = joint1 축 0.21 m · 퍽 위 9 cm 수평 (MuJoCo lie_ready)")
parser.add_argument("--via_q", default="-0.7,0.25,0.2", help="관절 보간 경유 자세 관절 2·3·4 (어깨 뒤로·팔꿈치 거의 폄: MuJoCo lie_waypoint, 전환 중 손가락-상자 66 mm 넘게). 빈 문자열 = 경유 없음")
parser.add_argument("--gui", action="store_true")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = not args_cli.gui
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import waffle_isaac  # noqa: E402,F401
from isaaclab.utils.math import quat_apply  # noqa: E402
from waffle_isaac.liepick_env_cfg import WaffleLiePickEnvCfg  # noqa: E402


def nrm(v):
    return v / v.norm(dim=1, keepdim=True).clamp(min=1e-9)


def main():
    cfg = WaffleLiePickEnvCfg()
    cfg.scene.num_envs = args_cli.num_envs
    cfg.near_start_prob, cfg.near_low_prob = 1.0, 0.0  # 손가락 방향을 긴 축에 맞춘 채 퍽 위 6 cm 에서 시작
    cfg.grasp_dz_lie = args_cli.dz
    S = args_cli.steps
    T = [45, 65, 100, 100 + S, 100 + 2 * S, 100 + 3 * S, 100 + 3 * S + 45]  # 단계 끝 스텝
    cfg.episode_length_s = (T[-1] + 5) / 30.0
    env = gym.make("Waffle-LiePick-Direct", cfg=cfg)
    u = env.unwrapped
    n, dev = u.num_envs, u.device
    zf = torch.zeros(n, dtype=torch.bool, device=dev)
    u._get_dones = lambda: (zf, zf)  # 중간 종료·리셋 없이 끝까지
    env.reset()
    Rcmd = torch.eye(3, device=dev).repeat(n, 1, 1)
    u.target_R_base = lambda: Rcmd  # 집게 자세 목표를 이 각본이 정함
    ez = torch.tensor([[0.0, 0.0, 1.0]], device=dev).expand(n, 3)
    j1o = torch.tensor([[cfg.joint1_x, 0.0, 0.0]], device=dev).expand(n, 3)

    def frame(a, uu):  # 열 = (집게 방향 a, 닫는 축 c = u x a, 손가락 높이 방향 u)
        a, uu = nrm(a), nrm(uu)
        return torch.stack((a, torch.cross(uu, a, dim=1), uu), dim=2)

    L0 = nrm(u.vec_base(u.long_axis_w()) * torch.tensor([1.0, 1.0, 0.0], device=dev))
    sgn = torch.where((L0 * u.vec_base(-u.a5_axes_w()[1])).sum(1, keepdim=True) >= 0, 1.0, -1.0)
    u0 = L0 * sgn  # 손가락 높이 방향 = 긴 축 (지금 집게와 가까운 부호)
    g_lift = None
    stats = {}
    box_hit = torch.zeros(n, dtype=torch.bool, device=dev)
    nb_hit = torch.zeros(n, dtype=torch.bool, device=dev)
    names = ["0 접근", "1 쥠", "2 들기", "3 롤로 접선", "4 집게 수평", "5 롤 −90°", "6 들고 있기"] if args_cli.mode == "ik" else ["0 접근", "1 쥠", "2 들기", "3 경유 자세", "4 옆 쥐기 자세", "5 들고 있기", "6 들고 있기 2"]
    for k in range(T[-1]):
        ph = sum(k >= t for t in T)
        g = u.grasp_w()
        gb = u.to_base(g)
        j1 = u.to_base(u.robot.data.root_link_pos_w.torch + quat_apply(u.robot.data.root_link_quat_w.torch, j1o))
        r_hat = nrm((gb - j1) * torch.tensor([1.0, 1.0, 0.0], device=dev))
        t_hat = torch.cross(ez, r_hat, dim=1)
        if k == T[2]:  # 3 단계 시작: 지금 손가락 높이 방향에서 가까운 접선 방향 (부호) 을 고름
            cur_u = u.vec_base(-u.a5_axes_w()[1])
            t_sign = torch.where((cur_u * t_hat).sum(1, keepdim=True) >= 0, 1.0, -1.0)
            u1_fixed, a_fixed, r_fixed = t_hat * t_sign, -ez.clone(), r_hat.clone()
            u_start = nrm(cur_u * torch.tensor([1.0, 1.0, 0.0], device=dev))
        act = torch.zeros(n, 6, device=dev)
        if args_cli.mode == "joint" and ph >= 3:
            if k == T[2]:
                q_from = u.robot.data.joint_pos.torch[:, u._arm].clone()
                q_to = q_from.clone()
                q_to[:, 0] = torch.atan2(r_hat[:, 1], r_hat[:, 0])
                q_to[:, 1:4] = torch.tensor([[float(v) for v in args_cli.side_q.split(",")]], device=dev)
                q_to[:, 4] = 0.0
                q_via = None
                if args_cli.via_q:
                    q_via = q_from.clone()
                    q_via[:, 0] = q_to[:, 0]
                    q_via[:, 1:4] = torch.tensor([[float(v) for v in args_cli.via_q.split(",")]], device=dev)
            if q_via is None:
                s = min(1.0, (k - T[2] + 1) / (2 * S))
                s = 0.5 - 0.5 * math.cos(math.pi * s)
                qt = q_from * (1 - s) + q_to * s
            else:  # 들기 -> 경유 (S 스텝) -> 옆 쥐기 끝 자세 (S 스텝)
                s = min(1.0, (k - T[2] + 1) / S)
                s = 0.5 - 0.5 * math.cos(math.pi * s)
                s2 = min(1.0, max(0.0, (k - T[3] + 1) / S))
                s2 = 0.5 - 0.5 * math.cos(math.pi * s2)
                qt = q_from * (1 - s) + q_via * s if k < T[3] else q_via * (1 - s2) + q_to * s2
            u._pre_physics_step = lambda a, qt=qt: (setattr(u, "prev_actions", u.actions.clone()), setattr(u, "actions", a.clamp(-1.0, 1.0)), setattr(u, "arm_target", qt))
            act[:, -1] = -1.0
            box_hit |= u._contact("arm_box_contact")
            nb_hit |= u._contact("arm_nb_contact")
            env.step(act)
            if k + 1 in T:
                fing = u.robot.data.joint_pos.torch[:, u._gl[0]]
                gd = torch.linalg.norm(u.puck_pos_w() - u.grasp_w(), dim=1)
                hold = (fing > cfg.grasp_finger_range[0]) & (fing < cfg.grasp_finger_range[1]) & (gd < 0.03)
                stand = u.long_axis_w()[:, 2].abs()
                ax, ay, az = u.a5_axes_w()
                stats[names[T.index(k + 1)]] = (hold.clone(), stand.clone(), u.center_off().abs().clone(), u.lift().clone(), gd.clone(), (-az)[:, 2].clone())
                print(f"[LieRoll] {names[T.index(k + 1)]:8s} 끝: 쥠 {int(hold.sum())}/{n}, 퍽 들기 평균 {u.lift().mean().item() * 1000:5.0f} mm, 긴 축 z 평균 {stand.mean().item():.2f}, "
                      f"가운데 벗어남 (쥔 대) {(u.center_off().abs() * hold).sum().item() / max(1, int(hold.sum())) * 1000:.1f} mm, 집게 방향 z 평균 {(-az)[:, 2].mean().item():+.2f}, 카메라 위 z {(-ay)[:, 2].mean().item():+.2f}", flush=True)
            continue
        if ph <= 1:
            goal = u.to_base(u.grasp_target_w())
            if k < 25:
                goal[:, 2] += 0.03
            Rcmd[:] = frame(-ez, u0)
            act[:, -1] = 1.0 if ph == 0 else -1.0
        else:
            act[:, -1] = -1.0
            if g_lift is None:
                g_lift = gb.clone()
                g_lift[:, 2] = u.to_base(u.puck_pos_w())[:, 2] + args_cli.lift
            goal = g_lift
            if ph == 2:
                Rcmd[:] = frame(-ez, u0)
            elif ph == 3:
                s = (k - T[2] + 1) / S
                Rcmd[:] = frame(-ez, nrm(u_start * (1 - s) + u1_fixed * s))
            elif ph == 4:
                s = (k - T[3] + 1) / S * math.pi / 2
                Rcmd[:] = frame(-ez * math.cos(s) + r_fixed * math.sin(s), u1_fixed)
            else:
                s = min(1.0, (k - T[4] + 1) / S) * math.pi / 2
                Rcmd[:] = frame(r_fixed, u1_fixed * math.cos(s) + ez * math.sin(s))
        act[:, :3] = ((goal - gb) / cfg.cart_scale).clamp(-1, 1)
        if ph >= 1:
            box_hit |= u._contact("arm_box_contact")
        nb_hit |= u._contact("arm_nb_contact")
        env.step(act)
        if k + 1 in T:
            fing = u.robot.data.joint_pos.torch[:, u._gl[0]]
            gd = torch.linalg.norm(u.puck_pos_w() - u.grasp_w(), dim=1)
            hold = (fing > cfg.grasp_finger_range[0]) & (fing < cfg.grasp_finger_range[1]) & (gd < 0.03)
            stand = u.long_axis_w()[:, 2].abs()
            ax, ay, az = u.a5_axes_w()
            stats[names[T.index(k + 1)]] = (hold.clone(), stand.clone(), u.center_off().abs().clone(), u.lift().clone(), gd.clone(), (-az)[:, 2].clone())
            print(f"[LieRoll] {names[T.index(k + 1)]:8s} 끝: 쥠 {int(hold.sum())}/{n}, 퍽 들기 평균 {u.lift().mean().item() * 1000:5.0f} mm, 긴 축 z 평균 {stand.mean().item():.2f}, "
                  f"가운데 벗어남 (쥔 대) {(u.center_off().abs() * hold).sum().item() / max(1, int(hold.sum())) * 1000:.1f} mm, 집게 방향 z 평균 {(-az)[:, 2].mean().item():+.2f}, "
                  f"손가락 {[round(v, 4) for v in fing[:6].tolist()]}", flush=True)
    hold, stand, off, lift, gd, az = stats[names[-1]]
    ok = hold & (stand > math.cos(math.radians(5))) & (off < 0.006)
    first = stats[names[1]][0]
    print(f"[LieRoll] dz {args_cli.dz * 1000:.0f} mm: 쥠 {int(first.sum())}/{n} -> **세로로 세워 쥐고 있음 (5° 안·가운데 6 mm) {int(ok.sum())}/{n}** "
          f"(쥔 대 중 {ok[first].float().mean().item() * 100 if first.any() else 0:.0f} %) | 상자 닿음 (쥐기부터) {int(box_hit.sum())} · 옆 퍽 닿음 {int(nb_hit.sum())}", flush=True)
    print(f"[LieRoll] 끝 퍽 중심-잡는 점 mm {[round(v * 1000) for v in gd[:12].tolist()]} | 긴 축 z {[round(v, 2) for v in stand[:12].tolist()]}", flush=True)
    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
