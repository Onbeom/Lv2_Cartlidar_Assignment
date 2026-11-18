"""2026-11-11 (RL 261111 1, 미션 파트 261110 3 요청): r2e 정책 Isaac 에피소드 스텝별 기록 = C++ lieApplyAction · 관측 조립 비교용 정답 벡터.

  python tools/r2e_parity_dump.py --out <폴더> [--seed 777] [--policy handoff/lieupright_r2e.pt]

환경 Waffle-LieUprightB4 설정 (r2e 학습 환경) + 평가와 같은 옆 퍽 월드 판정, 처음부터 시작 (kind 0) 만, 대 3 개:
  ep0 = 옆 퍽 있음, ep1 = 옆 퍽 없음, ep2 = 아무거나 (reset 을 되풀이해 ep0 · ep1 조건이 맞는 배치를 고름).
대마다 첫 에피소드가 끝날 때까지 스텝마다 기록. 기록 시점 = 그 스텝 _pre_physics_step (정책에 넣은 obs 와 같은 물리 상태).
_pre_physics_step · _ik_pose 를 감싸 값만 꺼냄 (계산은 원래 함수 그대로, 감싼 IK 사본이 원래 출력과 같은지 스텝마다 확인).
"""
import argparse
import json
import math
import os
import sys

from isaaclab.app import AppLauncher

W = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAN = os.path.join(W, "ws", "urdf_and_usd", "policy", "manipulator")
parser = argparse.ArgumentParser()
parser.add_argument("--out", required=True)
parser.add_argument("--seed", type=int, default=777)
parser.add_argument("--policy", default=os.path.join(W, "handoff", "lieupright_r2e.pt"))
parser.add_argument("--max_tries", type=int, default=50)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
sys.path.insert(0, MAN)

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
from isaaclab.utils.math import quat_apply, quat_box_minus, quat_from_matrix, quat_mul  # noqa: E402

import waffle_isaac  # noqa: E402,F401
from waffle_isaac.dice2_cfg import BOX_H, BOX_SIZE, GRASP_OFFSET_A5, PUCK_H  # noqa: E402
from waffle_isaac.lieupright_env_cfg import WaffleLieUprightB4EnvCfg  # noqa: E402
from waffle_isaac.robot_cfg import ARM_JOINTS, EE_BODY, GRIPPER_JOINT, GRIPPER_MIMIC  # noqa: E402

L = lambda x: [float(v) for v in x.reshape(-1).tolist()]  # noqa: E731


def main():
    cfg = WaffleLieUprightB4EnvCfg()
    cfg.scene.num_envs = 3
    cfg.nb_prob = 0.5
    cfg.nb_move_world = True  # r2e 평가와 같음
    cfg.start_probs = (1.0, 0.0, 0.0, 0.0, 0.0)
    cfg.seed = args_cli.seed
    env = gym.make("Waffle-LieUprightB4-Direct", cfg=cfg)
    u = env.unwrapped
    n, dev = u.num_envs, u.device
    pol = torch.jit.load(args_cli.policy, map_location=dev).eval()

    # 배치 고르기: 대 0 옆 퍽 있음, 대 1 없음
    obs, _ = env.reset()
    tries = 1
    while not (bool(u.nb_present[0]) and not bool(u.nb_present[1])):
        if tries >= args_cli.max_tries:
            raise SystemExit("배치 조건 못 맞춤")
        obs, _ = env.reset()
        tries += 1
    assert (u.kind == 0).all(), u.kind
    print(f"[Parity] reset {tries} 번, 옆 퍽 {u.nb_present.tolist()}", flush=True)

    # ---- 감싸기 ----
    cap = {}
    orig_ik = u._ik_pose

    def ik_wrap(dp_b):
        c, d = u.cfg, u.robot.data
        rq = d.root_link_quat_w.torch
        J = d.body_link_jacobian_w.torch[:, u._a5][:, :, u._cols]
        q5 = d.body_link_quat_w.torch[:, u._a5]
        rr = quat_apply(q5, u._off)
        Jl = J[:, 0:3] - torch.cross(rr.unsqueeze(-1).expand(-1, -1, 5), J[:, 3:6], dim=1)
        w = c.rot_hold_weight
        Jf = torch.cat((Jl, J[:, 3:6] * w), dim=1)
        Rt = u.target_R_base()
        qt = quat_mul(rq, quat_from_matrix(Rt))
        err = torch.cat((quat_apply(rq, dp_b), quat_box_minus(qt, q5) * w), dim=1)
        out = orig_ik(dp_b)
        JJt = Jf @ Jf.transpose(1, 2) + c.ik_damping * torch.eye(6, device=dev)
        mine = (Jf.transpose(1, 2) @ torch.linalg.solve(JJt, err.unsqueeze(-1))).squeeze(-1)
        cap.update(J_w=J.clone(), rr_w=rr.clone(), q5_w=q5.clone(), Jf=Jf.clone(), R_target_base=Rt.clone(), q_target_w=qt.clone(),
                   err=err.clone(), dp_b=dp_b.clone(), dq=out.clone(), ik_copy_maxdiff=float((mine - out).abs().max()))
        return out

    u._ik_pose = ik_wrap
    orig_pre = u._pre_physics_step

    def pre_wrap(actions):
        d = u.robot.data
        cap.update(act_raw=actions.clone(), beta_before=u.beta.clone(), rho_before=u.rho.clone(), arm_target_before=u.arm_target.clone(),
                   q=d.joint_pos.torch[:, u._arm].clone(), qd=d.joint_vel.torch[:, u._arm].clone(),
                   fing_l=d.joint_pos.torch[:, u._gl[0]].clone(), fing_r=d.joint_pos.torch[:, u._gr[0]].clone(),
                   root_p_w=d.root_link_pos_w.torch.clone(), root_q_w=d.root_link_quat_w.torch.clone(),
                   a5_p_w=d.body_link_pos_w.torch[:, u._a5].clone(), grasp_b=u.to_base(u.grasp_w()).clone(), h_base=u.h_base().clone(),
                   released=u.released.clone(), hold_state=u.hold_state.clone(), hold_on=u.hold_on.clone(), hold_off=u.hold_off.clone(),
                   was_held=u.was_held.clone(), stood=u.stood.clone(),
                   placed=u.placed.clone(), perc_frozen=u.perc_frozen.clone())
        orig_pre(actions)
        cap.update(act_clip=u.actions.clone(), prev_act=u.prev_actions.clone(), beta=u.beta.clone(), rho=u.rho.clone(), arm_target=u.arm_target.clone())

    u._pre_physics_step = pre_wrap

    # 에피소드 상수
    a_ax, c_ax = u.ee_axes_b()
    eps = []
    for i in range(n):
        eps.append(dict(
            env=i, nb_present=bool(u.nb_present[i]), kind=int(u.kind[i]),
            box_c_base=L(u.box_c[i]), box_yaw=float(u.box_yaw[i]), D_base=L(u.D[i]),
            puck_true_base0=L(u.to_base(u.puck_pos_w())[i]), nb_true_base0=L(u.to_base(u._obj("puckN").data.root_pos_w.torch)[i]),
            perc_bias=L(u.perc_bias[i]), box_dh=float(u.box_dh[i]), yaw_bias=float(u.yaw_bias[i]),
            steps=[], end=None))
    ho_on = torch.zeros(n, dtype=torch.long, device=dev)
    ho_off = torch.zeros(n, dtype=torch.long, device=dev)
    ho = torch.zeros(n, dtype=torch.bool, device=dev)
    ended = torch.zeros(n, dtype=torch.bool, device=dev)
    T = int(cfg.episode_length_s / u.step_dt) + 2
    chk = dict(ik_copy_maxdiff=0.0, target_formula_maxdiff=0.0, obs_grasp_maxdiff=0.0, obs_axes_maxdiff=0.0, obs_D_maxdiff=0.0,
               obs_box_maxdiff=0.0, obs_q_maxdiff=0.0, obs_beta_rho_maxdiff=0.0, obs_prevact_maxdiff=0.0, obs_hold_mismatch=0)
    for t in range(T):
        o = obs["policy"].clone()
        d = u.robot.data
        # 기록 도구 계산: 손가락 범위만 쥠 (cfg.obs_hold_mode finger 와 같은 식, 이 스텝 손가락으로 갱신; 첫 스텝은 갱신 전)
        if t > 0:
            fl = d.joint_pos.torch[:, u._gl[0]]
            ro = (fl > cfg.grasp_finger_range[0]) & (fl < cfg.grasp_finger_range[1])
            ho_on = torch.where(ro, ho_on + 1, torch.zeros_like(ho_on))
            ho_off = torch.where(ro, torch.zeros_like(ho_off), ho_off + 1)
            ho = torch.where(ho_on >= cfg.hold_n_on, torch.ones_like(ro), torch.where(ho_off >= cfg.hold_n_off, torch.zeros_like(ro), ho))
        a_ax, c_ax = u.ee_axes_b()
        extra = dict(puck_true_base=u.to_base(u.puck_pos_w()).clone(), puck_true_q_w=u.puck_quat_w().clone(),
                     nb_true_base=u.to_base(u._obj("puckN").data.root_pos_w.torch).clone(), ee_a_base=a_ax.clone(), ee_c_base=c_ax.clone(),
                     D=u.D.clone(), box_c=u.box_c.clone(), box_yaw=u.box_yaw.clone(), ho_finger=ho.clone(), ep_len=u.episode_length_buf.clone())
        with torch.inference_mode():
            a = pol(o)
        a = a.clone()
        obs, rew, term, trunc, _ = env.step(a)
        live = ~ended
        # 자체 확인 (산 대만)
        m = live
        if m.any():
            lead = torch.clamp(cap["arm_target_before"] + cap["dq"], cap["q"] - cfg.target_lead_max, cap["q"] + cfg.target_lead_max)
            tf = torch.clamp(lead, u._lo, u._hi)
            chk["ik_copy_maxdiff"] = max(chk["ik_copy_maxdiff"], cap["ik_copy_maxdiff"])
            chk["target_formula_maxdiff"] = max(chk["target_formula_maxdiff"], float((tf - cap["arm_target"])[m].abs().max()))
            chk["obs_grasp_maxdiff"] = max(chk["obs_grasp_maxdiff"], float((o[:, 16:19] - cap["grasp_b"])[m].abs().max()))
            chk["obs_axes_maxdiff"] = max(chk["obs_axes_maxdiff"], float((o[:, 19:25] - torch.cat((extra["ee_a_base"], extra["ee_c_base"]), 1))[m].abs().max()))
            chk["obs_D_maxdiff"] = max(chk["obs_D_maxdiff"], float((o[:, 25:28] - extra["D"])[m].abs().max()))
            bx = torch.cat((extra["box_c"], torch.cos(extra["box_yaw"])[:, None], torch.sin(extra["box_yaw"])[:, None]), 1)
            chk["obs_box_maxdiff"] = max(chk["obs_box_maxdiff"], float((o[:, 31:35] - bx)[m].abs().max()))
            qd0 = u.robot.data.default_joint_pos.torch[:, u._arm]
            chk["obs_q_maxdiff"] = max(chk["obs_q_maxdiff"], float((o[:, 0:5] - (cap["q"] - qd0))[m].abs().max()), float((o[:, 5:10] - cap["qd"])[m].abs().max()), float((o[:, 10] - cap["fing_l"])[m].abs().max()))
            br = torch.stack((torch.cos(cap["rho_before"]), torch.sin(cap["rho_before"]), cap["beta_before"]), 1)
            chk["obs_beta_rho_maxdiff"] = max(chk["obs_beta_rho_maxdiff"], float((o[:, 36:39] - br)[m].abs().max()))
            chk["obs_prevact_maxdiff"] = max(chk["obs_prevact_maxdiff"], float((o[:, 39:45] - cap["prev_act"])[m].abs().max()))
            chk["obs_hold_mismatch"] += int(((o[:, 35] > 0.5) != cap["hold_state"])[m].sum())
        for i in torch.nonzero(live).flatten().tolist():
            s = dict(t=t, ep_len=int(extra["ep_len"][i]),
                     obs=L(o[i]), action_raw=L(cap["act_raw"][i]), action_clip=L(cap["act_clip"][i]), prev_action=L(cap["prev_act"][i]),
                     beta_before=float(cap["beta_before"][i]), rho_before=float(cap["rho_before"][i]), beta=float(cap["beta"][i]), rho=float(cap["rho"][i]),
                     q=L(cap["q"][i]), qd=L(cap["qd"][i]), finger_left=float(cap["fing_l"][i]), finger_right=float(cap["fing_r"][i]),
                     arm_target_before=L(cap["arm_target_before"][i]), dq_ik=L(cap["dq"][i]), arm_target=L(cap["arm_target"][i]),
                     ik=dict(dp_base=L(cap["dp_b"][i]), h_base=L(cap["h_base"][i]), R_target_base=[L(r) for r in cap["R_target_base"][i]],
                             q_target_w=L(cap["q_target_w"][i]), q5_w=L(cap["q5_w"][i]), err=L(cap["err"][i]),
                             J_w=[L(r) for r in cap["J_w"][i]], rr_w=L(cap["rr_w"][i]), Jf=[L(r) for r in cap["Jf"][i]]),
                     root_p_w=L(cap["root_p_w"][i]), root_q_w=L(cap["root_q_w"][i]), a5_p_w=L(cap["a5_p_w"][i]),
                     grasp_base=L(cap["grasp_b"][i]), ee_a_base=L(extra["ee_a_base"][i]), ee_c_base=L(extra["ee_c_base"][i]),
                     D=L(extra["D"][i]), box_c=L(extra["box_c"][i]), box_yaw=float(extra["box_yaw"][i]),
                     puck_true_base=L(extra["puck_true_base"][i]), puck_true_q_w=L(extra["puck_true_q_w"][i]), nb_true_base=L(extra["nb_true_base"][i]),
                     gripper_cmd=float(cfg.gripper_open if float(cap["act_clip"][i, -1]) > 0.0 else cfg.gripper_closed),
                     hold_state=bool(cap["hold_state"][i]), hold_on=int(cap["hold_on"][i]), hold_off=int(cap["hold_off"][i]),
                     ho_finger=bool(extra["ho_finger"][i]), released=bool(cap["released"][i]),
                     was_held=bool(cap["was_held"][i]), stood=bool(cap["stood"][i]), placed=bool(cap["placed"][i]), perc_frozen=bool(cap["perc_frozen"][i]),
                     reward=float(rew[i]))
            eps[i]["steps"].append(s)
        done = (term | trunc) & live
        for i in torch.nonzero(done).flatten().tolist():
            why = [k for k, v in (("success", u.t_succ), ("drop", u.t_drop), ("topple", u.t_topple), ("nb", u.t_nb), ("self", u.t_self), ("arm_body", u.t_ab),
                                  ("body_box", u.t_bb), ("badspot", u.t_badspot), ("arm_box", u.t_abx), ("nan", u.t_nan)) if bool(v[i])]
            if bool(trunc[i]) and not bool(term[i]):
                why.append("timeout")
            eps[i]["end"] = dict(step=t, time_s=(t + 1) * u.step_dt, reasons=why, success=bool(u.t_succ[i]))
            print(f"[Parity] 대 {i} 끝: 스텝 {t}, {why}", flush=True)
        ended |= done
        if bool(ended.all()):
            break

    meta = dict(
        note="r2e (lieupright_r2e.pt) Isaac 에피소드 스텝별 기록. 기록 시점 = 그 스텝 _pre_physics_step (obs 와 같은 물리 상태). base = 로봇 root = base_link. 쿼터니언 (x, y, z, w) = Isaac Lab 3.0 순서. J_w = body_link_jacobian_w[gripper_roll_link] 의 팔 5 열 (행 = 월드 선속도 3 + 각속도 3). Jf = (Jl, J_ang × rot_hold_weight), Jl = J_lin − rr × J_ang. dq_ik = Jfᵀ (Jf Jfᵀ + ik_damping I)⁻¹ err. arm_target = clamp(clamp(arm_target_before + dq_ik, q ± target_lead_max), lo, hi). ho_finger = 기록 도구 계산 (손가락 범위 + 같은 히스테리시스, 정책 obs 에는 안 씀 — obs 35 칸은 hold_state)",
        task="Waffle-LieUprightB4-Direct", policy=args_cli.policy, seed=args_cli.seed, num_envs=n, nb_move_world=True, obs_hold_mode=cfg.obs_hold_mode,
        reset_tries=tries,
        joints=dict(arm=ARM_JOINTS, gripper=[GRIPPER_JOINT, GRIPPER_MIMIC], arm_lo=L(u._lo), arm_hi=L(u._hi),
                    default_arm=L(u.robot.data.default_joint_pos.torch[0, u._arm]), default_root_z=float(u.robot.data.default_root_pose.torch[0, 2])),
        consts=dict(ee_body=EE_BODY, grasp_offset_a5=list(GRASP_OFFSET_A5), joint1_x=cfg.joint1_x, cart_scale=cfg.cart_scale, beta_scale=cfg.beta_scale,
                    rho_scale=cfg.rho_scale, beta_range=list(cfg.beta_range), rho_max=cfg.rho_max, rot_hold_weight=cfg.rot_hold_weight,
                    ik_damping=cfg.ik_damping, target_lead_max=cfg.target_lead_max, gripper_open=cfg.gripper_open, gripper_closed=cfg.gripper_closed,
                    grasp_finger_range=list(cfg.grasp_finger_range), hold_dist=cfg.hold_dist, hold_n_on=cfg.hold_n_on, hold_n_off=cfg.hold_n_off,
                    perc_freeze_dist=cfg.perc_freeze_dist, sim_dt=cfg.sim.dt, decimation=cfg.decimation, step_dt=u.step_dt,
                    box_h=BOX_H, box_size=BOX_SIZE, puck_h=PUCK_H),
        self_check=chk)
    os.makedirs(args_cli.out, exist_ok=True)
    names = ["ep0_nb1", "ep1_nb0", "ep2_any"]
    for i, ep in enumerate(eps):
        with open(os.path.join(args_cli.out, f"{names[i]}.json"), "w") as f:
            json.dump(dict(meta=meta, episode=ep), f)
        print(f"[Parity] {names[i]}: 옆 퍽 {ep['nb_present']}, 스텝 {len(ep['steps'])}, 끝 {ep['end']}", flush=True)
    print(f"[Parity] 자체 확인 {json.dumps(chk)}", flush=True)
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
