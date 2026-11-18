"""2026-11-10 (Onbeom, #192) P1 세우기 평가 (창 없이, 한 번 시도 기준): 대마다 첫 에피소드 결과를 셈.

  python lieupright_eval.py --policy <exported policy.pt> --num_envs 512 [--nb_prob 0.5] [--seed 777] [--starts 1,0,0] [--dump_end 파일]
판정 R1 (세워 놓은 대 / 대), R4 (옆 퍽 · 팔-몸체), R5 (성공 시각) 와 단계별 (쥠 · 잘 쥠 · 10 cm · 세움 · 놓음) · 종료 이유.
--dump_end: 성공한 대의 끝 상태 (세운 퍽 · 상자 · 옆 퍽, base) 를 JSON 으로 = P2 (다시 잡기) 시작 상태.
"""
import argparse
import json
import os
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=512)
parser.add_argument("--policy", required=True)
parser.add_argument("--nb_prob", type=float, default=0.5)
parser.add_argument("--seed", type=int, default=777)
parser.add_argument("--starts", default="1,0,0", help="시작 종류 확률 (처음부터, 쥐고 듦, 세움)")
parser.add_argument("--dump_end", default="")
parser.add_argument("--chain_phys", type=int, default=0, help="2026-11-11 E1a: 1 = chain45 와 같은 물리 (로봇 · 목표 퍽 contact_offset 2 mm · rest 0, 목표 퍽 마찰 0.5 min 결합)")
parser.add_argument("--dump_e1", default="", help="2026-11-11 E1a: 접촉 없이 성공한 대의 끝 상태 (base 좌표, env 번호별) -> chain45 --lie_from_end")
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
from waffle_isaac.lieupright_env_cfg import WaffleLieUprightEnvCfg  # noqa: E402


def main():
    cfg = WaffleLieUprightEnvCfg()
    cfg.scene.num_envs = args_cli.num_envs
    cfg.nb_prob = args_cli.nb_prob
    cfg.start_probs = tuple(float(v) for v in args_cli.starts.split(","))
    cfg.seed = args_cli.seed
    if args_cli.chain_phys:  # chain45_play.py 의 --col_offset 0.002 · --puck_mu 0.5 (기본값) 와 같은 변경 (REGRASP = 목표 퍽 둘 다)
        import isaaclab.sim as sim_utils
        col = sim_utils.CollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0)
        mat = sim_utils.RigidBodyMaterialCfg(static_friction=0.5, dynamic_friction=0.5, friction_combine_mode="min")
        cfg.scene.robot = cfg.scene.robot.replace(spawn=cfg.scene.robot.spawn.replace(collision_props=col))
        for k_ in ("puckT_cyl", "puckT_box"):
            o_ = getattr(cfg.scene, k_)
            setattr(cfg.scene, k_, o_.replace(spawn=o_.spawn.replace(collision_props=col, physics_material=mat)))
    from phys_report import report
    report("E1a" if args_cli.chain_phys else "P1", cfg)
    env = gym.make("Waffle-LieUpright-Direct", cfg=cfg)
    u = env.unwrapped
    pol = torch.jit.load(args_cli.policy, map_location=u.device).eval()
    obs, _ = env.reset()
    n, dev = u.num_envs, u.device
    print(f"[LUEval] 정책 {args_cli.policy}, 대 {n}, 옆 퍽 확률 {args_cli.nb_prob}, seed {args_cli.seed}", flush=True)
    B = lambda: torch.zeros(n, dtype=torch.bool, device=dev)
    ended, succ = B(), B()
    st = {k: B() for k in ("grasp", "center", "lift10", "stood", "placed")}
    why = {k: B() for k in ("drop", "topple", "nb", "self", "arm_body", "body_box", "badspot", "nan", "timeout")}
    t_succ = torch.full((n,), float("nan"), device=dev)
    nolift = B()  # RL1 검토: 쥠 판정 뒤 3 cm 못 든 채 끝난 대
    nb_on = u.nb_present.clone()
    nb_stage = {}
    end_state = [None] * n
    e1 = [None] * n
    j3_hi = torch.zeros(n, device=dev)  # 검토 7: 관절 3 > 1.5 rad 스텝 수 (첫 에피소드)
    steps = torch.zeros(n, device=dev)
    root_yaw_max = torch.zeros(n, device=dev)  # 2026-11-11 RL1: 로봇 root 요 최대 |값| (켤레 순서 잘못의 영향 확인, quat_apply 로만 잼)
    root_tilt_max = torch.zeros(n, device=dev)
    ab_hit = torch.zeros(n, device=dev)  # 2026-11-11 재확인: 팔-상자 접촉 스텝 수 (첫 에피소드, 벽 · 상자 접촉 금지 규칙)
    ml_end = torch.zeros(n, device=dev)  # 끝날 때 최대 든 높이
    T = int(cfg.episode_length_s / u.step_dt)
    for t in range(T + 2):
        ml_ = u.max_lift.clone()
        snap = dict(grasp=u.grasp_seen.clone(), center=u.center_ok.clone(), lift10=u.lift10.clone(), stood=u.stood.clone(), placed=u.released.clone())
        if args_cli.dump_e1:
            rq_, rp_ = u.robot.data.root_link_quat_w.torch, u.robot.data.root_link_pos_w.torch
            from isaaclab.utils.math import quat_mul as _qm
            iq_ = torch.cat((-rq_[:, :3], rq_[:, 3:]), dim=1)  # 켤레 (x, y, z, w) (11-11 04:1x 고침, 전에는 (w, x, y, z) 로 봄 · root 거의 단위라 영향 작음)
            e1_now = {}
            for k_ in ("puckT_box", "boxA", "puckN"):
                o_ = u._obj(k_).data
                e1_now[k_] = (u.to_base(o_.root_pos_w.torch), _qm(iq_, o_.root_quat_w.torch))
            jq_ = u.robot.data.joint_pos.torch
            e1_q, e1_f = jq_[:, u._arm].clone(), torch.stack((jq_[:, u._gl[0]], jq_[:, u._gr[0]]), dim=1)
            e1_t = u.arm_target.clone()
            e1_g = u.to_base(u.grasp_w())  # 팔 관절 목표 (처짐 = 목표 − 측정, chain45 에서 같은 힘 균형으로 이어지게)
        if args_cli.dump_end:
            pb_, nb_ = u.to_base(u.puck_pos_w()), u.to_base(u._obj("puckN").data.root_pos_w.torch)[:, :2]
            rq = u.robot.data.root_link_quat_w.torch
            pq = torch.cat((-rq[:, :3], rq[:, 3:]), dim=1)  # 켤레 (x, y, z, w) (11-11 04:1x 고침)
            from isaaclab.utils.math import quat_mul
            pq = quat_mul(pq, u.puck_quat_w())
        live = ~ended
        j3_hi += (live & (u.robot.data.joint_pos.torch[:, u._arm[2]].abs() > 1.5)).float()
        steps += live.float()
        ab_hit += (live & u._contact("arm_box_contact")).float()
        from isaaclab.utils.math import quat_apply as _qa
        _rq = u.robot.data.root_link_quat_w.torch
        _bx = _qa(_rq, torch.tensor([[1.0, 0.0, 0.0]], device=dev).expand(n, 3))
        _bz = _qa(_rq, torch.tensor([[0.0, 0.0, 1.0]], device=dev).expand(n, 3))
        root_yaw_max = torch.maximum(root_yaw_max, torch.where(live, torch.atan2(_bx[:, 1], _bx[:, 0]).abs(), root_yaw_max))
        root_tilt_max = torch.maximum(root_tilt_max, torch.where(live, torch.acos(_bz[:, 2].clamp(-1, 1)), root_tilt_max))
        with torch.inference_mode():
            a = pol(obs["policy"])
        obs, _, term, trunc, _ = env.step(a)
        done = (term | trunc) & ~ended
        if done.any():
            for k in st:
                st[k] |= done & snap[k]
            nolift |= done & snap["grasp"] & (ml_ < cfg.lift_ok)
            ml_end = torch.where(done, ml_, ml_end)
            s_ = done & u.t_succ
            succ |= s_
            t_succ = torch.where(s_, torch.full_like(t_succ, (t + 1) * u.step_dt), t_succ)
            for k, v in (("drop", u.t_drop), ("topple", u.t_topple), ("nb", u.t_nb), ("self", u.t_self), ("arm_body", u.t_ab), ("body_box", u.t_bb), ("badspot", u.t_badspot), ("nan", u.t_nan)):
                why[k] |= done & v
            why["timeout"] |= done & trunc & ~term
            # 2026-11-11 r2e 준비 (RL1): 옆 퍽 닿음 종료가 난 단계 = 다가가기 (쥐기 전) · 들기 · 세우기 (쥔 뒤 세우기 전) · 놓기 (세운 뒤 벌리기 전) · 손 빼기 (벌린 뒤)
            nbd = done & u.t_nb
            if nbd.any():
                for i in torch.nonzero(nbd).flatten().tolist():
                    stg = "손 빼기" if bool(snap["placed"][i]) else ("놓기" if bool(snap["stood"][i]) else ("들기 · 세우기" if bool(snap["grasp"][i]) else "다가가기"))
                    nb_stage[stg] = nb_stage.get(stg, 0) + 1
            if args_cli.dump_end:
                for i in torch.nonzero(s_).flatten().tolist():
                    end_state[i] = dict(puck_p=pb_[i].tolist(), puck_q=pq[i].tolist(), box_c=u.box_c[i].tolist(), box_yaw=float(u.box_yaw[i]),
                                        nb_p=nb_[i].tolist(), nb_on=bool(nb_on[i]))
            if args_cli.dump_e1:
                for i in torch.nonzero(s_ & (ab_hit == 0)).flatten().tolist():
                    e1[i] = dict(env=i, q=e1_q[i].tolist(), q_tgt=e1_t[i].tolist(), g=e1_g[i].tolist(), fing=e1_f[i].tolist(), nb_on=bool(nb_on[i]),
                                 **{f"{k_}_p": v[0][i].tolist() for k_, v in e1_now.items()}, **{f"{k_}_q": v[1][i].tolist() for k_, v in e1_now.items()})
            ended |= done
        if bool(ended.all()):
            break
    N = n
    pct = lambda m: f"{int(m.sum())}/{N} ({m.float().mean().item() * 100:.1f} %)"
    print(f"[LUEval] **R1 세워 놓기 성공 {pct(succ)}**", flush=True)
    for nm in ("grasp", "center", "lift10", "stood", "placed"):
        print(f"[LUEval]   단계 {nm}: {pct(st[nm])}", flush=True)
    print(f"[LUEval]   옆 퍽 있음 {int(nb_on.sum())} 대 성공 {int((succ & nb_on).sum())}, 없음 {int((~nb_on).sum())} 대 성공 {int((succ & ~nb_on).sum())}", flush=True)
    print(f"[LUEval]   종료 이유: " + ", ".join(f"{k} {int(v.sum())}" for k, v in why.items()), flush=True)
    print(f"[LUEval]   쥠 판정 뒤 {cfg.lift_ok * 100:.0f} cm 못 든 채 끝난 대 {int(nolift.sum())} (쥠 판정 대 {int(st['grasp'].sum())} 중)", flush=True)
    print(f"[LUEval] 옆 퍽 닿음 종료 단계: {nb_stage} (옆 퍽 있는 대 {int(nb_on.sum())}, 그중 닿음 {int((why['nb'] & nb_on).sum())} = {(why['nb'] & nb_on).sum().item() / max(1, int(nb_on.sum())) * 100:.2f} %)", flush=True)
    print(f"[LUEval] R4 옆 퍽 닿음 · 밀림 {int(why['nb'].sum())} 대, 팔-몸체 {int(why['arm_body'].sum())} 대, 자기 충돌 {int(why['self'].sum())} 대", flush=True)
    print(f"[LUEval] 관절 3 > 1.5 rad 스텝 비율 {j3_hi.sum().item() / max(1.0, steps.sum().item()) * 100:.2f} % (그런 스텝이 있었던 대 {int((j3_hi > 0).sum())})", flush=True)
    print(f"[LUEval] 팔-상자 접촉 (> {cfg.contact_force} N) 있었던 대 {int((ab_hit > 0).sum())}, 그중 성공 {int(((ab_hit > 0) & succ).sum())}, 스텝 비율 {ab_hit.sum().item() / max(1.0, steps.sum().item()) * 100:.2f} %", flush=True)
    print(f"[LUEval] 로봇 root 요 최대 |°| {torch.rad2deg(root_yaw_max).max().item():.4f} (중앙 {torch.rad2deg(root_yaw_max).median().item():.4f}), 기울기 최대 ° {torch.rad2deg(root_tilt_max).max().item():.4f} | root 쿼터니언 (x, y, z, w) 대 0: {[round(v, 6) for v in u.robot.data.root_link_quat_w.torch[0].tolist()]}", flush=True)
    print(f"[LUEval] **접촉 없는 성공 (팔-상자 접촉 없음) {pct(succ & (ab_hit == 0))}**", flush=True)
    if succ.any():
        m = ml_end[succ]
        print(f"[LUEval] 성공한 대 최대 든 높이 cm: 최소 {m.min().item() * 100:.1f} 중앙 {m.median().item() * 100:.1f} 최대 {m.max().item() * 100:.1f}, 10 cm 단계 통과 {int((succ & st['lift10']).sum())}, 가운데 쥠 {int((succ & st['center']).sum())}", flush=True)
    ts = t_succ[~torch.isnan(t_succ)]
    if len(ts):
        print(f"[LUEval] R5 성공 시각 s: 중앙 {ts.median().item():.2f} (최소 {ts.min().item():.2f} 최대 {ts.max().item():.2f})", flush=True)
    if args_cli.dump_end:
        L = [s for s in end_state if s is not None]
        with open(args_cli.dump_end, "w") as f:
            json.dump(dict(note="P1 세우기 성공 끝 상태 (base)", starts=L), f)
        print(f"[LUEval] 끝 상태 {len(L)} 개 -> {args_cli.dump_end}", flush=True)
    if args_cli.dump_e1:
        with open(args_cli.dump_e1, "w") as f:
            json.dump(dict(note="E1a 접촉 없는 P1 성공 끝 상태 (성공 판정 한 스텝 전, base 좌표 · base 기준 자세, env 번호별, 나머지 null)",
                           seed=args_cli.seed, nb_prob=args_cli.nb_prob, policy=args_cli.policy, num_envs=n, chain_phys=args_cli.chain_phys, starts=e1), f)
        print(f"[LUEval] E1 끝 상태 {sum(1 for v in e1 if v)} 개 -> {args_cli.dump_e1}", flush=True)
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
