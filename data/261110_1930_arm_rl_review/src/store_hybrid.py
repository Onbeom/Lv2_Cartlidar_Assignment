"""넣기 정책 + 손 IK 내리기 확인 (창 없이): 학습된 넣기 정책이 퍽을 통 축 위에 맞추면 (퍽 바닥-축 ≤ 1.5 mm, 입구 위 12 mm 안, 직립)
그 뒤는 야코비안 IK 로 (집게 방향 유지) 퍽 바닥 중심을 통 축에 맞추며 1 mm / 스텝 내려 입구 아래 18 mm -> 집게 열기 0.5 s -> 집게 위로 4 cm 뺌.
통 안으로 '내리기' 가 물리적으로 되는지 (정책 보상 문제인지, 막힌 것인지) 가르기 위한 것. 결과 = 통별 퍽 깊이·직립·팔-몸체 힘.

    ./run_isaac.sh store_hybrid [환경 수]
"""

import argparse
import glob
import os
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=16)
parser.add_argument("--policy", default=None)
parser.add_argument("--cats", default="AC")
parser.add_argument("--align_mm", type=float, default=2.5)
parser.add_argument("--up_min", type=float, default=0.985)
parser.add_argument("--depth", type=float, default=0.010, help="[m] 쥔 채 내리는 깊이 (퍽 바닥, 벽 윗면 아래). 2026-10-30 퍽 플레이트 컵: 턱 위 1 mm (벽 15 mm), 손가락 밑은 벽 위 2 mm")
parser.add_argument("--collect", default=None, help="넣기 정책 시작 상태 기록 -> 이 json (통 축 6 mm·입구 위 20 mm 안에서 쥔 상태, 3 스텝마다)")
parser.add_argument("--no_policy", action="store_true", help="정책 없이 시작 자세 유지 -> 0.3 s 뒤 바로 손 IK 내리기 (시작 상태 C 에서 컵에 물리적으로 내려 넣어지는지 확인, 2026-10-30)")
parser.add_argument("--gate_clear", type=float, default=0.012, help="[m] 손 IK 내리기 시작: 퍽 바닥이 입구 위 이 안")
parser.add_argument("--desc_steps", type=int, default=60, help="손 IK 내리기 최대 스텝 (1 mm / 스텝)")
parser.add_argument("--occ", type=float, default=None, help="2026-11-01: 나머지 바깥 컵 3 개에 퍽이 미리 들어 있을 확률 (1.0 = 다 참, 연속 채우기 마지막 퍽). 기본 = 환경 설정")
parser.add_argument("--release_open", type=float, default=None, help="놓을 때 손가락 목표 [m] (기본 = 환경 gripper_open 0.004 = 49.6 mm). 옆 컵 퍽을 안 치게 작게 (예 0.0 = 41.6 mm)")
parser.add_argument("--overhead", type=float, default=0.0, help="2026-11-01 연속 채우기: 쥔 퍽이 목표 컵 축 이 [m] 안에 오면 손 IK 가 넘겨받아 (1) 제자리에서 퍽 바닥을 입구 + oh_clear 까지 올리고 (2) 그 높이로 축 위까지 옮긴 뒤 (3) 곧게 내림. 0 = 끔")
parser.add_argument("--oh_clear", type=float, default=0.06, help="[m] 위에서 들어가기 높이 (퍽 바닥 - 입구). 꽂힌 옆 퍽은 입구 위 45 mm")
parser.add_argument("--oh_step", type=float, default=0.003, help="[m / 스텝] 위에서 들어가기 이동 속도")
parser.add_argument("--lift", type=float, default=0.04, help="[m] 놓은 뒤 위로 빼는 높이 (2 mm / 스텝)")
parser.add_argument("--center_release", action="store_true", help="놓기·빼기 동안 집게 가운데 (잡는 점) 를 컵 축에 맞춤 (한쪽 손가락이 옆 퍽에 닿지 않게)")
parser.add_argument("--gui", action="store_true", help="창으로 보기 (run_isaac.sh store_hybrid_play)")
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
from isaaclab.utils.math import quat_apply, quat_box_minus, quat_from_matrix, quat_mul  # noqa: E402
from waffle_isaac.store_env_cfg import WaffleStoreEnvCfg  # noqa: E402


def main():
    fixed = os.path.join(HERE, "policies", "store_latest.pt")  # 기본 = 지금 옮기기 모델 (261028 ST-N5)
    path = args_cli.policy or (fixed if os.path.exists(fixed) else sorted(glob.glob(os.path.join(HERE, "logs/rsl_rl/waffle_store/*/exported/policy.pt")))[-1])
    cfg = WaffleStoreEnvCfg()
    cfg.scene.num_envs = args_cli.num_envs
    cfg.start_cats = tuple(args_cli.cats) if len(args_cli.cats) > 0 else cfg.start_cats
    cfg.cat_weight = tuple(dict(zip(WaffleStoreEnvCfg().start_cats, WaffleStoreEnvCfg().cat_weight)).get(c_, 0.1) for c_ in cfg.start_cats)
    if args_cli.occ is not None:
        cfg.occ_prob = args_cli.occ
    if args_cli.release_open is not None:
        cfg.gripper_open = args_cli.release_open
    env = gym.make("Waffle-Store-Direct", cfg=cfg)
    u = env.unwrapped
    pol = torch.jit.load(path, map_location=u.device).eval()
    env.reset()
    n, dev, r = u.num_envs, u.device, u.robot
    nb = r.num_base_dofs if hasattr(r, "num_base_dofs") else 6
    cols = torch.tensor([nb + j for j in u._arm], device=dev)
    phase = torch.zeros(n, dtype=torch.long, device=dev)  # 0 정책, 1 내리기, 2 열기, 3 빼기, 4 끝
    pt = torch.zeros(n, dtype=torch.long, device=dev)
    ok_cnt = torch.zeros(n, dtype=torch.long, device=dev)
    oh_cnt = torch.zeros(n, dtype=torch.long, device=dev)
    t_align = torch.full((n,), -1.0, device=dev)
    fmax = torch.zeros(n, device=dev)
    ffmax = torch.zeros(n, device=dev)
    fwhen = [""] * n
    dz_goal = torch.zeros(n, device=dev)
    tube = u.tube.clone()
    occ0 = [u.scene[f"occ{j}"].data.root_pos_w.torch.clone() for j in range(3)]
    occ_fmax = torch.zeros(n, device=dev)
    occ_n = sum(((torch.linalg.norm(occ0[j][:, :2] - u.scene.env_origins[:, :2] - torch.tensor([0.75, 0.65 + 0.1 * j], device=dev), dim=1) > 0.05)).long() for j in range(3))
    print(f"[Hybrid] 꽂힌 옆 퍽 수 (대별): 0 개 {int((occ_n == 0).sum())}, 1 개 {int((occ_n == 1).sum())}, 2 개 {int((occ_n == 2).sum())}, 3 개 {int((occ_n == 3).sum())}", flush=True)
    occ_stage = torch.full((n,), -1, dtype=torch.long, device=dev)  # 컵 속 퍽에 처음 1 N 넘게 닿은 단계
    occ_src = torch.full((n,), -1, dtype=torch.long, device=dev)
    occ_dxy = torch.zeros(n, device=dev)  # 처음 닿을 때 쥔 퍽 축 거리
    occ_body = torch.zeros(n, dtype=torch.long, device=dev)
    occ_t = torch.zeros(n, device=dev)
    occ_clr = torch.zeros(n, device=dev)  # 처음 닿을 때 퍽 바닥 - 입구
    print("[Hybrid] 정책", path, "| 통", [u._tube_names[i] for i in tube.tolist()], flush=True)

    def ik_step(dp_w, mask):
        """잡는 점 변위 dp (월드) + 자세 목표 = 팔 방향 수평·카메라 위 (2026-10-31 chain_play 와 같게) -> 감쇠 최소제곱, 관절 한계 반영 다시 풀기."""
        J = r.data.body_link_jacobian_w.torch[:, u._a5]  # (N, 6, dof)
        J = J[:, :, cols]
        q5 = r.data.body_link_quat_w.torch[:, u._a5]
        rr = quat_apply(q5, u._off)  # a5 원점 -> 잡는 점
        Jl = J[:, 0:3] - torch.cross(rr.unsqueeze(-1).expand(-1, -1, 5), J[:, 3:6], dim=1)
        Jf = torch.cat((Jl, J[:, 3:6] * 0.05), dim=1)  # 자세 유지는 약하게
        gb_ = u._state()["gb"]
        ra = torch.stack((gb_[:, 0] - cfg.joint1_x, gb_[:, 1], torch.zeros_like(gb_[:, 0])), dim=1)
        ra = ra / ra.norm(dim=1, keepdim=True).clamp(min=1e-6)
        upv = torch.tensor([[0.0, 0.0, 1.0]], device=dev).expand(n, 3)
        q_tgt = quat_mul(r.data.root_link_quat_w.torch, quat_from_matrix(torch.stack((ra, torch.cross(upv, ra, dim=1), upv), dim=2)))
        err = torch.cat((dp_w, quat_box_minus(q_tgt, q5) * 0.05), dim=1)
        lam = 1e-4
        JJt = Jf @ Jf.transpose(1, 2) + lam * torch.eye(6, device=dev)
        dq = (Jf.transpose(1, 2) @ torch.linalg.solve(JJt, err.unsqueeze(-1))).squeeze(-1)
        qa_ = u.arm_target
        blk = ((qa_ + dq > u._hi - 0.02) & (dq > 0)) | ((qa_ + dq < u._lo + 0.02) & (dq < 0))
        if blk.any():
            Jb = Jf * (~blk).float().unsqueeze(1)
            dq2 = (Jb.transpose(1, 2) @ torch.linalg.solve(Jb @ Jb.transpose(1, 2) + lam * torch.eye(6, device=dev), err.unsqueeze(-1))).squeeze(-1)
            dq = torch.where(blk.any(dim=1, keepdim=True), dq2, dq)
        q = r.data.joint_pos.torch[:, u._arm]
        u.arm_target[mask] = torch.clamp(torch.clamp(qa_ + dq, q - 0.25, q + 0.25), u._lo, u._hi)[mask]

    steps = int(cfg.episode_length_s / u.step_dt) + 60
    rows = []
    for t in range(steps):
        obs = u._get_observations()
        with torch.inference_mode():
            a = pol(obs["policy"]) if not args_cli.no_policy else torch.zeros(n, 6, device=dev)
        if args_cli.overhead > 0:
            a = torch.where((phase != 0)[:, None], torch.zeros_like(a), a)  # 손 IK 단계에서는 정책 행동을 관절 목표에 더하지 않음 (더하면 정책이 IK 와 싸움, 2026-11-01)
        u._pre_physics_step(a)
        s = u._state()
        clear = -s["depth"]
        good = (s["dxy"] < args_cli.align_mm / 1000) & (clear < args_cli.gate_clear) & (clear > -0.002) & (s["u"][:, 2] > args_cli.up_min) & s["held"]
        if args_cli.no_policy:
            good = s["held"] & (t >= 10)
        ok_cnt = torch.where(good & (phase == 0), ok_cnt + 1, torch.zeros_like(ok_cnt))
        if args_cli.overhead > 0:
            ohs = (phase == 0) & s["held"] & (s["dxy"] < args_cli.overhead) & (t >= 3)
            phase[ohs], pt[ohs] = 5, 0
        m5 = phase == 5
        if m5.any():
            # 5 위에서 들어가기: 낮으면 먼저 제자리에서 올리고, 높이가 되면 그 높이로 축 위까지 옮김 (2026-11-01)
            dp = torch.zeros(n, 3, device=dev)
            low = clear < args_cli.oh_clear - 0.005
            vxy = s["t"][:, :2] - s["pb"][:, :2]
            vxy = vxy * torch.clamp(args_cli.oh_step / vxy.norm(dim=1, keepdim=True).clamp(min=1e-6), max=0.5)
            dp[:, :2] = torch.where(low[:, None], torch.zeros_like(vxy), vxy)
            dp[:, 2] = torch.clamp(args_cli.oh_clear - clear, -args_cli.oh_step, args_cli.oh_step)
            ik_step(dp, m5)
            u.actions[m5, 5] = -1.0
            over = m5 & ~low & (s["dxy"] < 0.002) & (s["u"][:, 2] > args_cli.up_min)
            oh_cnt = torch.where(over, oh_cnt + 1, torch.zeros_like(oh_cnt))
            go = m5 & (oh_cnt >= 3)
            phase[go], pt[go] = 1, 0
            t_align[go] = t * u.step_dt
        start = (phase == 0) & (ok_cnt >= 3)
        phase[start], pt[start] = 1, 0
        t_align[start] = t * u.step_dt
        # 1 내리기: 퍽 바닥 xy -> 축 (0.5 배), z 1 mm / 스텝, 입구 아래 18 mm 까지
        m1 = phase == 1
        if m1.any():
            dp = torch.zeros(n, 3, device=dev)
            dp[:, :2] = (s["t"][:, :2] - s["pb"][:, :2]) * 0.5
            dp[:, 2] = -0.001
            ik_step(dp, m1)
            u.actions[m1, 5] = -1.0
            done1 = m1 & ((s["depth"] > args_cli.depth) | (pt > args_cli.desc_steps))
            phase[done1], pt[done1] = 2, 0
        m2 = phase == 2
        if m2.any():
            u.arm_target[m2] = r.data.joint_pos.torch[m2][:, u._arm]
            if args_cli.center_release:
                dp = torch.zeros(n, 3, device=dev)
                dp[:, :2] = (s["t"][:, :2] - s["gb"][:, :2]) * 0.5
                ik_step(dp, m2)
            u.actions[m2, 5] = 1.0
            phase[m2 & (pt >= 15)] = 3
            pt[m2 & (pt >= 15)] = 0
        m3 = phase == 3
        if m3.any():
            dp = torch.zeros(n, 3, device=dev)
            dp[:, 2] = 0.002
            if args_cli.center_release:
                dp[:, :2] = (s["t"][:, :2] - s["gb"][:, :2]) * 0.5
            ik_step(dp, m3)
            u.actions[m3, 5] = 1.0
            phase[m3 & (pt >= int(round(args_cli.lift / 0.002)))] = 4
        m4 = phase == 4
        if m4.any():
            u.arm_target[m4] = r.data.joint_pos.torch[m4][:, u._arm]
            u.actions[m4, 5] = 1.0
        pt += 1
        for _ in range(cfg.decimation):
            u._apply_action()
            u.scene.write_data_to_sim()
            u.sim.step(render=args_cli.gui)
            u.scene.update(u.physics_dt)
            fa, ff = u._force("arm_body_contact"), u._force("finger_body_contact")
            for i in torch.nonzero((ff > ffmax) & (ff > 1.0)).flatten().tolist():
                fwhen[i] = f"단계 {int(phase[i])} {ff[i].item():.1f} N"
            fmax = torch.maximum(fmax, fa)
            fo = torch.maximum(u._force("robot_occ_contact"), u._force("puck_occ_contact"))
            newc = (occ_stage < 0) & (fo > 1.0)
            occ_dxy = torch.where(newc, s["dxy"], occ_dxy)
            occ_clr = torch.where(newc, -s["depth"], occ_clr)
            fb = torch.linalg.norm(u.scene["robot_occ_contact"].data.normal_force_matrix_w.torch, dim=-1).amax(dim=2)  # (N, 몸)
            occ_body = torch.where(newc, fb.argmax(dim=1), occ_body)
            occ_t = torch.where(newc, torch.full_like(occ_t, t * u.step_dt), occ_t)
            occ_stage = torch.where((occ_stage < 0) & (fo > 1.0), phase.long(), occ_stage)
            occ_src = torch.where((occ_src < 0) & (fo > 1.0), (u._force("puck_occ_contact") > 1.0).long(), occ_src)  # 0 팔·손가락, 1 쥔 퍽
            occ_fmax = torch.maximum(occ_fmax, fo)
            ffmax = torch.maximum(ffmax, ff)
        if args_cli.collect and t % 3 == 0:
            s = u._state()
            keep = s["held"] & (s["dxy"] < 0.006) & (-s["depth"] < 0.02) & (phase <= 1)
            jp = r.data.joint_pos.torch
            pq = u.puck.data.root_quat_w.torch
            for i in torch.nonzero(keep).flatten().tolist():
                rows.append({"tube": int(u.tube[i]), "phase": int(phase[i]), "depth": round(float(s["depth"][i]), 5), "dxy": round(float(s["dxy"][i]), 5),
                             "q": [round(float(v), 5) for v in jp[i, u._arm].tolist()], "finger": [float(jp[i, u._gl[0]]), float(jp[i, u._gr[0]])],
                             "puck_b": [round(float(v), 5) for v in s["pc"][i].tolist()], "puck_quat": [float(v) for v in pq[i].tolist()]})
        if t % 30 == 29:
            s = u._state()
            print(f"[Hybrid] t={(t + 1) * u.step_dt:.1f}s 단계 {phase.tolist()} 축 mm {[round(v * 1000, 1) for v in s['dxy'].tolist()]} 입구 아래 mm {[round(v * 1000) for v in s['depth'].tolist()]}"
                  f" 직립 {[round(v, 3) for v in s['u'][:, 2].tolist()]} 퍽-몸체 N {[round(v, 1) for v in u._force('puck_body_contact').tolist()]}", flush=True)
    s = u._state()
    for i in range(n):
        print(f"[Hybrid] env{i} 통 {u._tube_names[int(tube[i])]}: 맞춤 {t_align[i].item():.1f}s, 단계 {int(phase[i])}, 퍽 입구 아래 {s['depth'][i].item() * 1000:.0f} mm, "
              f"축 {s['dxy'][i].item() * 1000:.1f} mm, 직립 {s['u'][i, 2].item():.3f}, 쥠 {int(s['held'][i])}, 팔(링크 2 ~ 5)-몸체 최대 {fmax[i].item():.1f} N, 손가락-몸체 최대 {ffmax[i].item():.1f} N ({fwhen[i]})", flush=True)
    if args_cli.collect:
        import json
        json.dump({"note": "넣기 시작 상태 (Isaac, ST3 맞춤 + 손 IK 내리기 중): 팔 관절, 손가락 (왼, 오), 퍽 중심 (base_link), 퍽 자세 (월드 = 루트 요 0, x,y,z,w), 통, 깊이 (+ = 입구 아래)",
                   "rows": rows}, open(args_cli.collect, "w"))
        print(f"[Hybrid] 시작 상태 {len(rows)} 개 -> {args_cli.collect}", flush=True)
    stored = (s["depth"] > u.cfg.stored_depth) & (s["dxy"] < u.cfg.inside_xy + 0.002) & (phase >= 3) & (phase <= 4)
    occ_moved = torch.zeros(n, dtype=torch.bool, device=dev)
    for j in range(3):
        ob = u.scene[f"occ{j}"]
        occ_moved |= (torch.linalg.norm(ob.data.root_pos_w.torch[:, :2] - occ0[j][:, :2], dim=1) > 0.003) & (occ0[j][:, 2] < 1.0) & (torch.linalg.norm(occ0[j][:, :2] - u.scene.env_origins[:, :2] - torch.tensor([0.75, 0.65 + 0.1 * j], device=dev), dim=1) > 0.05)
    print(f"[Hybrid] **컵 안에 넣고 놓고 뺌 (축 6 mm·턱 위) {int(stored.sum())}/{n}**, 맞춤까지 간 것 {int((t_align >= 0).sum())}/{n}, "
          f"컵 속 퍽에 1 N 넘게 닿은 대 {int((occ_fmax > 1).sum())} (최대 {occ_fmax.max().item():.1f} N), 컵 속 퍽을 3 mm 넘게 밀어낸 대 {int(occ_moved.sum())}", flush=True)
    m5e = torch.nonzero(phase == 5).flatten().tolist()[:6]
    if m5e:
        print(f"[Hybrid] 위에서 들어가기 멈춘 대 관절 (1 ~ 4·롤): {[[round(v, 2) for v in r.data.joint_pos.torch[i, u._arm].tolist()] for i in m5e]} | 하한 {[round(v, 2) for v in u._lo[0].tolist()] if u._lo.dim() > 1 else [round(v, 2) for v in u._lo.tolist()]} 상한 {[round(v, 2) for v in u._hi[0].tolist()] if u._hi.dim() > 1 else [round(v, 2) for v in u._hi.tolist()]}", flush=True)
    sn = ("0 정책 옮기기·맞추기", "1 IK 내리기", "2 놓기", "3 빼기", "4 끝", "5 위에서 들어가기")
    first = {sn[k]: int((occ_stage == k).sum()) for k in range(6) if int((occ_stage == k).sum())}
    print(f"[Hybrid] 컵 속 퍽에 처음 닿은 단계: {first}", flush=True)
    src = {f"{sn[k]} / {w}": int(((occ_stage == k) & (occ_src == v)).sum()) for k in range(6) for v, w in ((0, "팔·손가락"), (1, "쥔 퍽")) if int(((occ_stage == k) & (occ_src == v)).sum())}
    print(f"[Hybrid] 처음 닿은 단계 / 닿은 것: {src}", flush=True)
    bn = u.scene["robot_occ_contact"].body_names
    for k in range(6):
        mk = (occ_stage == k) & (occ_src == 0)
        if mk.any():
            from collections import Counter
            tt = occ_t[mk]
            print(f"[Hybrid]   {sn[k]} 팔·손가락 처음 닿은 몸: {dict(Counter(bn[i] for i in occ_body[mk].tolist()))}, 시각 s 중앙값 {tt.median().item():.2f} (범위 {tt.min().item():.2f} ~ {tt.max().item():.2f})", flush=True)
    mp = (occ_stage == 0) & (occ_src == 1)
    if mp.any():
        print(f"[Hybrid]   정책 단계 쥔 퍽 처음 닿은 시각 s 중앙값 {occ_t[mp].median().item():.2f} (범위 {occ_t[mp].min().item():.2f} ~ {occ_t[mp].max().item():.2f})", flush=True)
    m0 = occ_stage == 0
    if m0.any():
        dd, cc = occ_dxy[m0] * 1000, occ_clr[m0] * 1000
        print(f"[Hybrid] 정책 단계에서 처음 닿을 때 축 거리: < 10 mm {int((dd < 10).sum())}, 10 ~ 25 mm {int(((dd >= 10) & (dd < 25)).sum())}, ≥ 25 mm {int((dd >= 25).sum())} | "
              f"퍽 바닥 - 입구 중앙값 {cc.median().item():.0f} mm (범위 {cc.min().item():.0f} ~ {cc.max().item():.0f})", flush=True)
    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
