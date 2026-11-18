"""스토리지에서 꺼내기 (각본) -> 주사위 눈 놓기 (정책) 연결 (2026-11-09).

꺼내기 = tools_ik/unload_traj.py 경로 (unload_traj.json, 넣기 경로와 별개): 놓기 시작 자세 -> 앞 높이 -> 칸 쪽 높이 (관절1 돌림) -> 칸 위
-> 맞춤 확인 (0.3 s 머묾) -> 내려가기 -> 쥠 (0.6 s) -> 들기 -> 칸 쪽 높이 -> 앞 높이 -> 놓기 시작 자세 -> 놓기 정책 (place60 / place45l2).
환경 = 놓기 환경 그대로 (상자 · 먼저 놓은 퍽 · 판정), 리셋 뒤 손 안 퍽을 칸 안 바닥에 옮겨 둠. 대마다 한 번만 셈.
    ./run_place.sh unload [환경 수] [정책]        (창 없이)   L2=1 이면 L2 (45°, 꺼낼 수 있는 8 칸)
    ./run_place.sh unload_play [환경 수] [정책]   (창)
"""

import argparse
import json
import math
import os
import sys
from collections import Counter

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=16)
parser.add_argument("--policy", default=None)
parser.add_argument("--l2", action="store_true")
parser.add_argument("--slot", default="", help="칸 이름 (예 3-1) 하나만, 비우면 꺼낼 수 있는 칸을 대마다 차례로")
parser.add_argument("--speed", type=float, default=1.0, help="[rad/s] 관절 경로 속도 (내려가기 · 들기는 0.3 배)")
parser.add_argument("--ki", type=float, default=1.0, help="처짐 보정 적분 이득")
parser.add_argument("--render_every", type=int, default=2)
parser.add_argument("--seconds", type=float, default=40.0)
parser.add_argument("--gui", action="store_true")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
GUI = args_cli.gui
args_cli.headless = not GUI
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
from isaaclab.utils.math import quat_apply, quat_from_euler_xyz  # noqa: E402

import waffle_isaac  # noqa: E402,F401
from waffle_isaac.dice2_cfg import PUCK_H  # noqa: E402
from waffle_isaac.place_env_cfg import WafflePlace45L2EnvCfg, WafflePlace60EnvCfg  # noqa: E402
from waffle_isaac.robot_cfg import FINGER_OPEN_TASK  # noqa: E402

NAMES = {0: "시간 끝", 1: "성공", 2: "나쁜 놓기", 3: "놓기 전 빠뜨림", 4: "놓은 퍽 쓰러짐", 5: "먼저 놓은 퍽 건드림", 6: "자기 충돌 · 몸체"}


def main():
    l2 = args_cli.l2
    cfg = WafflePlace45L2EnvCfg() if l2 else WafflePlace60EnvCfg()
    cfg.scene.num_envs = args_cli.num_envs
    cfg.start_table = ""  # 놓기 시작 = 꺼내기 끝 자세 (고정 넘김 자세)
    env = gym.make("Waffle-Place45L2-Direct" if l2 else "Waffle-Place60-Direct", cfg=cfg)
    u = env.unwrapped
    n, dev, r = u.num_envs, u.device, u.robot
    dt = u.step_dt
    pol_path = args_cli.policy or os.path.join(HERE, "policies", "place45l2_latest.pt" if l2 else "place60_latest.pt")
    pol = torch.jit.load(pol_path, map_location=dev).eval()
    tr = json.load(open(os.path.join(HERE, "unload_traj.json")))["pitches"]["45" if l2 else "60"]
    rows = [x for x in tr if x["ok"] and (not args_cli.slot or x["slot"] == args_cli.slot)]
    assert rows, "꺼낼 수 있는 칸 없음"
    pick = [rows[i % len(rows)] for i in range(n)]
    # 경유점: 0 넘김 자세(시작) 1 앞 높이 2 칸 쪽 높이 3 칸 위 4 쥐기 [쥠] 5 들기 6 칸 쪽 높이 7 앞 높이 8 넘김 자세
    W = torch.tensor([[x["q"][5], x["q"][4], x["q"][3], x["q"][0], x["q"][1], x["q"][2], x["q"][3], x["q"][4], x["q"][5]] for x in pick], device=dev)
    NW = W.shape[1]
    slow = torch.tensor([0, 0, 0, 0, 1, 1, 0, 0, 0], dtype=torch.bool, device=dev)  # 이 경유점으로 갈 때 0.3 배
    obs, _ = env.reset()
    # 손 안 퍽 -> 칸 바닥 (base -> 월드: 로봇 루트 위치 + 자세)
    rq, rp_ = r.data.root_link_quat_w.torch, r.data.root_link_pos_w.torch
    pc_b = torch.tensor([[x["x"], x["y"], x["floor_z"] + PUCK_H / 2 + 0.001] for x in pick], device=dev)
    pc_w = rp_ + quat_apply(rq, pc_b)
    yaw = torch.tensor([math.atan2(x["y"], x["x"] - u.cfg.joint1_x) for x in pick], device=dev)
    zero = torch.zeros(n, device=dev)
    for k_, use in (("puckT_box", u.is_box), ("puckT_cyl", ~u.is_box)):
        o = u._obj(k_)
        cur = o.data.root_pose_w.torch.clone()
        pose = torch.cat((pc_w, quat_from_euler_xyz(zero, zero, yaw)), dim=1)
        o.write_root_pose_to_sim_index(root_pose=torch.where(use[:, None], pose, cur), env_ids=torch.arange(n, device=dev))
        o.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=torch.arange(n, device=dev))
    q = r.data.joint_pos.torch[:, u._arm]
    qp = q.clone(); bias = torch.zeros_like(qp)
    idx = torch.ones(n, dtype=torch.long, device=dev)  # 다음 경유점
    cnt = torch.zeros(n, dtype=torch.long, device=dev)
    done_s = torch.zeros(n, dtype=torch.bool, device=dev)
    grip_cl = torch.zeros(n, dtype=torch.bool, device=dev)
    body_hit = torch.zeros(n, dtype=torch.bool, device=dev)
    go0 = u.cfg.gripper_open
    u.cfg.gripper_open = FINGER_OPEN_TASK  # 꺼내기: 49.6 mm (퍽 양옆 약 10 mm, 옆 칸 퍽 안 닿음)
    lead = u.cfg.target_lead_max
    k = 0
    # ---- 1 꺼내기 (각본) ----
    while not bool(done_s.all()) and k * dt < args_cli.seconds:
        q = r.data.joint_pos.torch[:, u._arm]
        w = W[torch.arange(n, device=dev), idx.clamp(max=NW - 1)]
        dq = w - qp
        m = dq.abs().amax(dim=1)
        stp = args_cli.speed * dt * torch.where(slow[idx.clamp(max=NW - 1)], 0.3, 1.0)
        reach = m <= stp
        hold_here = (idx == 4) & reach  # 쥐기 자리: 맞춤 · 닫기 기다림
        qp = torch.where(~done_s[:, None], torch.where(reach[:, None], w, qp + dq * (stp / m.clamp(min=1e-9))[:, None]), qp)
        bias = (bias + args_cli.ki * dt * (qp - q)).clamp(-0.1, 0.1)
        cnt = torch.where(reach, cnt + 1, torch.zeros_like(cnt))
        grip_cl |= (idx == 4) & reach & (cnt > int(0.3 / dt))  # 맞춤 확인 0.3 s 뒤 닫음
        nx = ~done_s & reach & (~hold_here | (cnt > int(0.9 / dt)))  # 쥐기 자리는 닫고 0.6 s 더
        idx = torch.where(nx, idx + 1, idx)
        cnt = torch.where(nx, torch.zeros_like(cnt), cnt)
        done_s |= idx >= NW
        u.arm_target[:] = torch.clamp(torch.clamp(qp + bias, q - lead, q + lead), u._lo, u._hi)
        u.actions[:, -1] = torch.where(grip_cl, -1.0, 1.0)
        draw = GUI and (k % max(1, args_cli.render_every) == 0)
        for j_ in range(cfg.decimation):
            u._apply_action()
            u.scene.write_data_to_sim()
            u.sim.step(render=draw and j_ == cfg.decimation - 1)
            u.scene.update(u.physics_dt)
        u.episode_length_buf[:] = 0
        body_hit |= u._contact("arm_body_contact")
        k += 1
    held = u.held()
    t_unload = k * dt
    print(f"[Unload] {'L2 45°' if l2 else '상자 60°'} 꺼내기 끝 {t_unload:.1f} s: 쥐고 넘김 {int(held.sum())}/{n}, 경로 끝 {int(done_s.sum())}/{n}, 팔-몸체 닿음 {int(body_hit.sum())}", flush=True)
    bad_slot = Counter(pick[i]["slot"] for i in range(n) if not bool(held[i]))
    if bad_slot:
        print(f"[Unload] 못 쥔 칸 {dict(bad_slot)}", flush=True)
    # ---- 2 놓기 (정책) ----
    u.cfg.gripper_open = go0  # 놓기: 조금만 벌림 (FINGER_RELEASED)
    u.released[:] = False; u.phi_ok[:] = False; u.ok_cnt[:] = 0; u.succeeded[:] = False; u.term_reason[:] = 0
    u.actions[:] = 0.0; u.actions[:, -1] = -1.0; u.prev_actions[:] = u.actions
    u.arm_target[:] = r.data.joint_pos.torch[:, u._arm]
    u.episode_length_buf[:] = 0
    obs = u._get_observations()
    done_once = ~held  # 못 쥔 대는 셈에서 뺌 (꺼내기 실패)
    reason = torch.full((n,), -1, dtype=torch.long, device=dev)
    err = torch.zeros(n, device=dev)
    for _ in range(int(cfg.episode_length_s / dt) + 2):
        with torch.inference_mode():
            a = pol(obs["policy"]).clamp(-1, 1)
        obs, rew, term, trunc, _ = env.step(a)
        fin = (term | trunc) & ~done_once
        reason = torch.where(fin, u.last_reason, reason)
        err = torch.where(fin, u.last_err, err)
        done_once |= term | trunc
        if bool(done_once.all()):
            break
    c = Counter(NAMES[int(v)] for v in reason[held & (reason >= 0)].tolist())
    ok = held & (reason == 1)
    print(f"[Unload] 놓기 ({os.path.basename(pol_path)}): 끝 이유 {dict(c)}", flush=True)
    print(f"[Unload] **꺼내 놓기 성공 {int(ok.sum())}/{n}** (꺼내기 {int(held.sum())}/{n}), 놓은 오차 mm 평균 {err[ok].mean().item() * 1000 if ok.any() else float('nan'):.1f} "
          f"최대 {err[ok].max().item() * 1000 if ok.any() else float('nan'):.1f}", flush=True)
    per = Counter(); per_ok = Counter()
    for i in range(n):
        per[pick[i]["slot"]] += 1; per_ok[pick[i]["slot"]] += int(ok[i])
    print(f"[Unload] 칸별 성공 {', '.join(f'{s_} {per_ok[s_]}/{per[s_]}' for s_ in sorted(per))}", flush=True)
    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
