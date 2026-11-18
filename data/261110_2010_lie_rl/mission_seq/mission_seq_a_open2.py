"""[사본 2 · 11-11 RL 261111 1: RL 261110 1 제안 — 든 뒤 처음 벌린 스텝마다 잡는 점 g · 집게 방향 a · 참 퍽 중심 · 참 긴 축 · FK 긴 축 · D 벡터를 기록 (노드 규칙 · 판정은 원본과 같음). 추정점 식 비교는 오프라인]
2026-11-11 (RL 261111 1) 미션 순서 모의 A단계: S0 도착 → S1 target_look (누운 퍽) → S2 상자 보기 → S3 READY_POSE_45 → S4 r2e (노드 규칙) → S5 위로 빼기 → S6 D target_look.
기준 · 순서 = `../미션순서모의_기준초안.md` (RL 261110 1 조건부 승인 반영). 노드 = 미션 파트 `fix/lie-upright-skill` manipulator_node lieTick (11-11).

  python mission_seq_a.py --num_envs 512 --seed 777 --nb_prob 0 --vmax 3.0 --dump <json>

- 환경: Waffle-LieUprightB4 설정 (r2e 학습), 처음부터 (kind 0), 옆 퍽 밀림 월드 판정. 환경의 종료 · 리셋은 쓰지 않고 물리를 직접 돌림 (대마다 단계가 다름).
- 이동 단계 (S1 · S2 · S3 · S5 · S6) = 노드 sendArm: 시간 = max(최소 1 s, 관절 최대 변화 / 0.5 rad/s), 끝 속도 0 3 차 보간 (JTC 한 점 궤적), 30 Hz 로 팔 목표.
- S1 · S6 자세 = 노드 target_look 식 (`tlook_cli`, C++ 그대로), S5 = 노드 위로 빼기 (`retreat_cli`, C++ 그대로).
- S4 = 노드 lieTick 규칙: 퍽 · 긴 축 · 옆 퍽 = S1 끝에 잰 값 고정, 상자 = S2 끝에 잰 값 고정, 세울 자리 D = 그 값으로 (`_stand_spot`, 옆 퍽 없으면 상자 중심 대칭),
  쥠 = 손가락 범위 + 히스테리시스 8 · 3, 들기 = FK 잡는 점 z 가 쥔 순간보다 3 cm 위, 든 뒤 쥠 풀림 = 떨어뜨림 실패,
  든 뒤 처음 벌림 = D 수평 30 mm · 위 60 mm · β ≥ 1.4 안이면 놓음 아니면 실패, 놓은 뒤 집게 늘 벌림, 끝 = 놓고 1 s + 잡는 점 D 3 cm 밖, 15 s 상한.
  관절 속도 제한 = --vmax (환경 cfg.arm_vmax, 노드와 같게 다음 기준 = 자른 값).
- 인식 오차 (--meas 1): 퍽 xy ±5 mm, 긴 축 ±3°, 옆 퍽 xy ±5 mm, 상자 중심 ±3 mm · 요 ±1° (균등, 대마다).
- 단계별 사건 (처음 난 단계): 팔 · 집게-상자 > 1 N, 팔-옆 퍽, 옆 퍽 밀림 3 mm (월드) · 넘어짐, 몸체-상자, 팔-몸체, 자기 충돌,
  목표 퍽 건드림 (이동 단계: 단계 시작보다 3 mm 넘게 움직이거나 5° 넘게 기울어짐), 목표 퍽 넘어짐 (놓은 뒤 20°), 바닥 낙하 (퍽 z < 상자 윗면 − 3 cm).
"""
import argparse
import json
import math
import os
import subprocess
import sys

from isaaclab.app import AppLauncher

HERE = os.path.dirname(os.path.abspath(__file__))
W = os.path.dirname(HERE)
MAN = os.path.join(W, "ws", "urdf_and_usd", "policy", "manipulator")
ap = argparse.ArgumentParser()
ap.add_argument("--num_envs", type=int, default=512)
ap.add_argument("--seed", type=int, default=777)
ap.add_argument("--nb_prob", type=float, default=0.0)
ap.add_argument("--vmax", type=float, default=3.0)
ap.add_argument("--vmode", default="scale", help="속도 제한 방식 clip | scale (노드 lieApplyActionLimited)")
ap.add_argument("--policy", default=os.path.join(W, "handoff", "lieupright_r2e.pt"))
ap.add_argument("--meas", type=int, default=1, help="1 = 인식 오차 켬")
ap.add_argument("--fix_obs", type=int, default=1, help="1 = 노드처럼 퍽 · 긴 축 · 옆 퍽 · 상자 · D 고정 관측, 0 = 학습 관측 그대로 (원인 분해용)")
ap.add_argument("--node_release", type=int, default=1, help="1 = 노드 놓음 · 떨어뜨림 · 끝 규칙, 0 = 환경 성공 판정으로 끝 (원인 분해용)")
ap.add_argument("--release_xy", type=float, default=0.03, help="노드 lie_upright.release_xy (잡는 점 – D 수평)")
ap.add_argument("--release_center_offset", type=float, default=0.0, help="노드 lie_upright.release_center_offset: 수평 판정점 = 잡는 점 + a x 이 값 (퍽 중심 추정, 0 = 잡는 점)")
ap.add_argument("--view", default="now", help="상자 보기 자세: now = [∓0.30, −1.80, 0.50, 1.90, 0], A = [j1, −1.25, 0.15, 1.85, 0], B = [j1, −1.15, 0.15, 1.80, 0] (11-11 미션 파트 후보, j1 ±0.300)")
ap.add_argument("--live", default="", help="고정 관측 중 학습처럼 살릴 칸 (쉼표): axis, puck, held, box (상자 + D), nb (원인 분해용). axis_fk = 노드가 할 수 있는 긴 축: 쥐기 전 잰 값 고정, 쥔 순간 손 좌표 축을 저장하고 그 뒤 FK (손 자세) 로 돌림")
ap.add_argument("--dump", default="")
AppLauncher.add_app_launcher_args(ap)
a = ap.parse_args()
a.headless = True
app = AppLauncher(a).app
sys.path.insert(0, MAN)

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
from isaaclab.utils.math import quat_apply, quat_mul  # noqa: E402

import waffle_isaac  # noqa: E402,F401
from waffle_isaac.dice2_cfg import BOX_H  # noqa: E402
from waffle_isaac.lieupright_env_cfg import WaffleLieUprightB4EnvCfg  # noqa: E402

ALIGN_A = [0.0, -1.854, 0.540, 1.645, 0.0]  # PR #202 (미션 파트 권장)
READY45 = [0.0, -0.338, 0.559, 0.564, 0.0]
VIEWS = {"A": (-1.25, 0.15, 1.85, 0.300), "B": (-1.15, 0.15, 1.80, 0.300)}
VIEW = {"left": [-0.300, -1.80, 0.50, 1.90, 0.0], "right": [0.305, -1.80, 0.50, 1.90, 0.0], "inline": [0.0, -1.80, 0.50, 1.90, 0.0]}
STAGES = ["S1", "S2", "S3", "S4", "S5", "S6"]
EVENTS = ["arm_box", "arm_nb", "nb_move", "nb_topple", "body_box", "arm_body", "self", "puck_touch", "puck_topple", "floor_drop"]
VMAX_MOVE, TMIN = 0.5, 1.0


class Cli:
    def __init__(self, exe):
        self.p = subprocess.Popen([exe], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)

    def __call__(self, line):
        self.p.stdin.write(line + "\n")
        self.p.stdin.flush()
        return self.p.stdout.readline().split()


def main():
    cfg = WaffleLieUprightB4EnvCfg()
    cfg.scene.num_envs = a.num_envs
    cfg.nb_prob = a.nb_prob
    cfg.nb_move_world = True
    cfg.start_probs = (1.0, 0.0, 0.0, 0.0, 0.0)
    cfg.seed = a.seed
    cfg.arm_vmax = a.vmax
    cfg.arm_vlimit_mode = a.vmode
    cfg.episode_length_s = 120.0
    env = gym.make("Waffle-LieUprightB4-Direct", cfg=cfg)
    u = env.unwrapped
    n, dev = u.num_envs, u.device
    pol = torch.jit.load(a.policy, map_location=dev).eval()
    tlook, retreat = Cli(os.path.join(HERE, "tlook_cli")), Cli(os.path.join(HERE, "retreat_cli"))
    g_ = torch.Generator(device=dev).manual_seed(a.seed + 1000)
    U = lambda lo, hi, sh: lo + (hi - lo) * torch.rand(sh, device=dev, generator=g_)  # noqa: E731
    dt_c = u.step_dt
    T_ = lambda v: torch.tensor(v, dtype=torch.float32, device=dev)  # noqa: E731

    env.reset()
    d = u.robot.data
    ids = torch.arange(n, device=dev)
    # S0: 도착 자세 ALIGN_A, 집게 벌림
    q = d.joint_pos.torch.clone()
    q[:, u._arm] = T_(ALIGN_A)
    u.robot.write_joint_position_to_sim_index(position=q, env_ids=ids)
    u.robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), env_ids=ids)
    u.arm_target[:] = T_(ALIGN_A)
    grip = torch.ones(n, device=dev)  # +1 벌림 · −1 닫음 (actions[-1])

    def phys():
        u.actions[:, -1] = grip
        for _ in range(cfg.decimation):
            u._apply_action()
            u.scene.write_data_to_sim()
            u.sim.step(render=False)
            u.scene.update(u.physics_dt)
        u.episode_length_buf += 1
        u._get_dones()  # 환경 단계 · 접촉 계산만 (종료 · 리셋은 안 씀)

    for _ in range(15):  # 0.5 s 자리 잡기 (차체 9.75 → 6.75 mm)
        phys()
    s_side = torch.where(u.to_base(u.puck_pos_w())[:, 1] < 0, 1.0, -1.0)  # s = +1: 누운 퍽 base y < 0 = 계획기 L (turn_p17)
    pri_y = -s_side * 0.0799
    tl_L = [float(v) for v in tlook("0.3565 -0.0799 0.100")[1:6]]
    tl_R = [float(v) for v in tlook("0.3565 0.0799 0.100")[1:6]]
    q_tl1 = torch.where((pri_y < 0)[:, None], T_(tl_L), T_(tl_R))
    box_pri_y = -s_side * 0.1182
    q_view = torch.where((box_pri_y < -0.05)[:, None], T_(VIEW["left"]), torch.where((box_pri_y > 0.05)[:, None], T_(VIEW["right"]), T_(VIEW["inline"])))
    if a.view in VIEWS:
        j2, j3, j4, j1 = VIEWS[a.view]
        VIEW.update(left=[-j1, j2, j3, j4, 0.0], right=[j1, j2, j3, j4, 0.0], inline=[0.0, j2, j3, j4, 0.0])
        q_view = torch.where((box_pri_y < -0.05)[:, None], T_(VIEW["left"]), torch.where((box_pri_y > 0.05)[:, None], T_(VIEW["right"]), T_(VIEW["inline"])))
    q_ready = T_(READY45).expand(n, 5).clone()

    # 단계 상태
    stage = torch.zeros(n, dtype=torch.long, device=dev)  # 0..5 = S1..S6, 6 = 끝 (성공), 7 = 실패
    fail_why = [""] * n
    t_in = torch.zeros(n, device=dev)  # 단계 안 시간
    T_move = torch.zeros(n, device=dev)
    q0_move = d.joint_pos.torch[:, u._arm].clone()
    q1_move = q_tl1.clone()
    settle = torch.zeros(n, device=dev)

    def start_move(m, q1, settle_s):
        qm = d.joint_pos.torch[:, u._arm]
        q0_move[m] = qm[m]
        q1_move[m] = q1[m]
        T_move[m] = torch.clamp((q1[m] - qm[m]).abs().amax(1) / VMAX_MOVE, min=TMIN)
        settle[m] = settle_s
        t_in[m] = 0.0
    all_ = torch.ones(n, dtype=torch.bool, device=dev)
    start_move(all_, q_tl1, 0.3)
    ev_stage = {e: torch.full((n,), -1, dtype=torch.long, device=dev) for e in EVENTS}
    ev_any = {e: torch.zeros(n, 6, dtype=torch.bool, device=dev) for e in EVENTS}  # 단계별 있었는지 (처음 난 단계만이 아니라)
    align_contact = torch.zeros(n, dtype=torch.bool, device=dev)  # 출발 자세 ALIGN_A 팔-몸체 접촉 (S1 첫 0.5 s, 따로 표시 — 미션 파트 요청)
    st_time = torch.zeros(n, 6, device=dev)
    puck_ref_p = u.puck_pos_w().clone()
    puck_ref_up = u.long_axis_w()[:, 2].abs().clone()
    # 노드 S4 상태
    meas_p = torch.zeros(n, 3, device=dev); meas_yaw = torch.zeros(n, device=dev); meas_nb = torch.zeros(n, 2, device=dev)
    meas_bc = torch.zeros(n, 2, device=dev); meas_by = torch.zeros(n, device=dev); D_node = torch.zeros(n, 3, device=dev)
    ho_on = torch.zeros(n, dtype=torch.long, device=dev); ho_off = torch.zeros_like(ho_on); held = torch.zeros(n, dtype=torch.bool, device=dev)
    th_last = torch.zeros(n, device=dev)  # axis_fk: 놓은 뒤 고정할 마지막 θ
    ax_hand = torch.zeros(n, 3, device=dev); ax_have = torch.zeros(n, dtype=torch.bool, device=dev)  # axis_fk: 쥔 순간 손 좌표 긴 축
    grasp_z = torch.zeros(n, device=dev); lifted = torch.zeros_like(held); released = torch.zeros_like(held); clear = torch.zeros(n, device=dev)
    s4_end = [""] * n
    open_log = []
    q_s4end = [None] * n; rise_s5 = [None] * n; rel_dh = [None] * n
    g_end_x = torch.full((n,), float("nan"), device=dev)
    top_b = BOX_H - u.robot.data.root_link_pos_w.torch[:, 2]  # 상자 윗면 base z (차체 자리 잡은 뒤)
    nb_w0 = u._obj("puckN").data.root_pos_w.torch[:, :2].clone()
    max_ticks = int(60.0 / dt_c)
    for tick in range(max_ticks):
        act_mask = stage == 3
        # ---- S4 정책 (노드 lieTick 순서) ----
        qm = d.joint_pos.torch[:, u._arm]
        fing = d.joint_pos.torch[:, u._gl[0]]
        gb = u.to_base(u.grasp_w())
        if act_mask.any():
            ro = (fing > cfg.grasp_finger_range[0]) & (fing < cfg.grasp_finger_range[1])
            ho_on = torch.where(act_mask & ro, ho_on + 1, torch.where(act_mask, torch.zeros_like(ho_on), ho_on))
            ho_off = torch.where(act_mask & ~ro, ho_off + 1, torch.where(act_mask, torch.zeros_like(ho_off), ho_off))
            held = torch.where(act_mask & (ho_on >= cfg.hold_n_on), True, torch.where(act_mask & (ho_off >= cfg.hold_n_off), False, held))
            if a.node_release:
                grasp_z = torch.where(act_mask & held & ~lifted & (grasp_z == 0.0), gb[:, 2], grasp_z)
                grasp_z = torch.where(act_mask & ~held & ~lifted, torch.zeros_like(grasp_z), grasp_z)
                lifted |= act_mask & held & (grasp_z != 0.0) & (gb[:, 2] > grasp_z + cfg.lift_ok)
                drop = act_mask & lifted & ~released & ~held
                for i in torch.nonzero(drop).flatten().tolist():
                    stage[i], fail_why[i], s4_end[i] = 7, "S4 든 뒤 쥠 풀림 (노드 떨어뜨림)", "drop_node"
                act_mask = stage == 3
            obs = u._get_observations()["policy"].clone()
            if a.fix_obs:
                LV = set(a.live.split(",")) if a.live else set()
                if "puck" not in LV:
                    obs[:, 11:14] = meas_p
                if "axis_fk" in LV:  # 쥔 순간 (노드 grasp_z 가 정해지는 스텝) 손 좌표 축 저장, 그 뒤 손 자세로 돌림 (퍽이 손 안에서 안 돈다고 가정)
                    _a, _c = u.ee_axes_b(); _u = torch.cross(_a, _c, dim=1)
                    _R = torch.stack((_a, _c, _u), dim=2)  # base <- 손
                    _e0 = torch.stack((torch.cos(meas_yaw), torch.sin(meas_yaw), torch.zeros_like(meas_yaw)), dim=1)
                    new_g = act_mask & held & (grasp_z != 0.0) & ~ax_have
                    ax_hand = torch.where(new_g[:, None], (_R.transpose(1, 2) @ _e0.unsqueeze(-1)).squeeze(-1), ax_hand)
                    ax_have |= new_g
                    ax_have &= ~(act_mask & ~held & ~lifted)  # 들기 전에 놓치면 다시 (노드 grasp_z 와 같게)
                    _e = torch.where(ax_have[:, None], (_R @ ax_hand.unsqueeze(-1)).squeeze(-1), _e0)
                    _th = torch.atan2(_e[:, 1], _e[:, 0])  # 학습 관측과 같은 식 (서 갈수록 수평 성분이 작아 흔들리는 것도 같게)
                    _th = torch.where(released, th_last, _th)  # 놓은 뒤 = 마지막 값 고정 (RL 261110 1)
                    th_last = torch.where(act_mask, _th, th_last)
                    obs[:, 14] = torch.cos(2 * _th); obs[:, 15] = torch.sin(2 * _th)
                elif "axis" not in LV:
                    obs[:, 14] = torch.cos(2 * meas_yaw); obs[:, 15] = torch.sin(2 * meas_yaw)
                if "box" not in LV:
                    obs[:, 25:28] = D_node
                    obs[:, 31:33] = meas_bc
                    obs[:, 33] = torch.cos(meas_by); obs[:, 34] = torch.sin(meas_by)
                if "nb" not in LV:
                    nbr = torch.where(u.nb_present[:, None], meas_nb - meas_p[:, :2], torch.zeros_like(meas_nb))
                    obs[:, 28:30] = nbr
                if "held" not in LV:
                    obs[:, 35] = held.float()
            with torch.inference_mode():
                act = pol(obs).clamp(-1.0, 1.0)
            act = act.clone()
            if a.node_release:
                opening = act_mask & lifted & ~released & (act[:, 5] > 0)
                _ea, _ = u.ee_axes_b()
                dh = torch.linalg.norm(gb[:, :2] + _ea[:, :2] * a.release_center_offset - D_node[:, :2], dim=1)
                dz = gb[:, 2] - D_node[:, 2]
                bad = opening & ((dh > a.release_xy) | (dz > 0.06) | (u.beta < 1.4))
                for i in torch.nonzero(bad).flatten().tolist():
                    stage[i], fail_why[i], s4_end[i] = 7, f"S4 D 밖에서 벌림 (수평 {dh[i]*1000:.0f} mm 위 {dz[i]*1000:.0f} mm β {float(u.beta[i]):.2f})", "open_away"
                _pt = u.to_base(u.puck_pos_w()); _et = u.vec_base(u.long_axis_w())
                _a2, _c2 = u.ee_axes_b(); _R2 = torch.stack((_a2, _c2, torch.cross(_a2, _c2, dim=1)), dim=2)
                _efk = (_R2 @ ax_hand.unsqueeze(-1)).squeeze(-1)
                for i in torch.nonzero(opening).flatten().tolist():
                    open_log.append(dict(env=i, rejected=bool(bad[i]), g=gb[i].tolist(), a=_a2[i].tolist(), pc=_pt[i].tolist(), e_true=_et[i].tolist(),
                                         e_fk=_efk[i].tolist(), fk_ok=bool(ax_have[i]), D=D_node[i].tolist(), beta=float(u.beta[i])))
                released |= opening & ~bad
                for i in torch.nonzero(opening & ~bad).flatten().tolist():
                    rel_dh[i] = float(dh[i])
                act_mask = stage == 3
                act[:, 5] = torch.where(released, torch.ones_like(act[:, 5]), act[:, 5])
            tgt_keep = u.arm_target.clone(); beta_keep, rho_keep = u.beta.clone(), u.rho.clone()
            u._pre_physics_step(act)
            u.arm_target = torch.where(act_mask[:, None], u.arm_target, tgt_keep)
            u.beta = torch.where(act_mask, u.beta, beta_keep); u.rho = torch.where(act_mask, u.rho, rho_keep)
            grip = torch.where(act_mask, torch.where(u.actions[:, -1] > 0, 1.0, -1.0), grip)
        # ---- 이동 단계 ----
        mv = (stage <= 5) & (stage != 3)
        if mv.any():
            s_ = (t_in / T_move.clamp(min=1e-6)).clamp(0.0, 1.0)
            h = 3 * s_ ** 2 - 2 * s_ ** 3
            qt = q0_move + (q1_move - q0_move) * h[:, None]
            u.arm_target = torch.where(mv[:, None], qt, u.arm_target)
            grip = torch.where(mv, torch.ones_like(grip), grip)
        phys()
        t_in += dt_c
        live = stage <= 5
        for k in range(6):
            st_time[:, k] += ((stage == k) & live).float() * dt_c
        # ---- 사건 (처음 난 단계) ----
        pw = u.puck_pos_w()
        upz = u.long_axis_w()[:, 2].abs()
        evn = {
            "arm_box": u._contact("arm_box_contact"),
            "arm_nb": u._contact("arm_nb_contact") & u.nb_present,
            "nb_move": u.nb_present & (torch.linalg.norm(u._obj("puckN").data.root_pos_w.torch[:, :2] - nb_w0, dim=1) > 0.003),
            "nb_topple": u.nb_present & u.nb_toppled(),
            "body_box": u._contact("body_box_contact"),
            "arm_body": u.arm_body_hit(),
            "self": u.self_bad(),
            "puck_touch": ((stage <= 2) | (stage == 4) | (stage == 5)) & ((torch.linalg.norm(pw - puck_ref_p, dim=1) > 0.003)
                                                                         | (torch.rad2deg(torch.acos(upz.clamp(max=1.0)) - torch.acos(puck_ref_up.clamp(max=1.0))).abs() > 5.0)),
            "puck_topple": released & (torch.acos(upz.clamp(max=1.0)) > cfg.topple_tilt),
            "floor_drop": (pw[:, 2] - u.scene.env_origins[:, 2]) < BOX_H - 0.03,
        }
        early = (stage == 0) & (t_in <= 0.5)
        align_contact |= early & evn["arm_body"]
        evn["arm_body"] = evn["arm_body"] & ~early
        for k, v in evn.items():
            ev_stage[k] = torch.where(live & v & (ev_stage[k] < 0), stage, ev_stage[k])
            hit = live & v
            if hit.any():
                ev_any[k][hit, stage[hit].clamp(max=5)] = True
        # ---- 단계 넘김 ----
        if a.node_release:
            clear += (act_mask & released).float() * dt_c
            fin = act_mask & released & (clear >= 1.0) & (torch.linalg.norm(gb - D_node, dim=1) > 0.03)
        else:
            fin = act_mask & u.succeeded
        to = act_mask & (t_in > 15.0)
        for i in torch.nonzero(to).flatten().tolist():
            stage[i], fail_why[i], s4_end[i] = 7, "S4 15 s 넘음", "timeout"
        done_mv = mv & (t_in >= T_move + settle)
        for i in torch.nonzero(fin).flatten().tolist():
            s4_end[i] = "ok"
            g_end_x[i] = gb[i, 0]
            r = retreat(" ".join(f"{v:.6f}" for v in qm[i].tolist()))
            q_s4end[i] = qm[i].tolist(); rise_s5[i] = float(r[5])
            qu = torch.tensor([float(v) for v in r[:5]], device=dev)
            if float(r[5]) < 0.8 * 0.05 or bool(((qu < u._lo) | (qu > u._hi)).any()):
                stage[i], fail_why[i] = 7, f"S5 위로 빼기 못 함 (rise {float(r[5])*1000:.0f} mm)"
                continue
            stage[i] = 4
            m = torch.zeros(n, dtype=torch.bool, device=dev); m[i] = True
            q1 = q1_move.clone(); q1[i] = qu
            start_move(m, q1, 0.0)
            puck_ref_p[i] = pw[i]; puck_ref_up[i] = upz[i]
        for i in torch.nonzero(done_mv).flatten().tolist():
            k = int(stage[i])
            m = torch.zeros(n, dtype=torch.bool, device=dev); m[i] = True
            puck_ref_p[i] = pw[i]; puck_ref_up[i] = upz[i]
            if k == 0:  # S1 끝: 퍽 · 긴 축 · 옆 퍽 잼 → S2
                pb = u.to_base(pw)[i]
                e = u.vec_base(u.long_axis_w())[i]
                err = U(-0.005, 0.005, (2,)) if a.meas else torch.zeros(2, device=dev)
                meas_p[i] = torch.stack((pb[0] + err[0], pb[1] + err[1], top_b[i] + 0.015))
                meas_yaw[i] = torch.atan2(e[1], e[0]) + (U(-0.0524, 0.0524, (1,))[0] if a.meas else 0.0)
                nbb = u.to_base(u._obj("puckN").data.root_pos_w.torch)[i, :2]
                meas_nb[i] = nbb + (U(-0.005, 0.005, (2,)) if a.meas else 0.0)
                stage[i] = 1; start_move(m, q_view, 0.5)
            elif k == 1:  # S2 끝: 상자 잼 → D → S3
                meas_bc[i] = u.box_c[i] + (U(-0.003, 0.003, (2,)) if a.meas else 0.0)
                meas_by[i] = u.box_yaw[i] + (U(-0.01745, 0.01745, (1,))[0] if a.meas else 0.0)
                nbv = meas_nb[i] if bool(u.nb_present[i]) else 2.0 * meas_bc[i] - meas_p[i, :2]
                Dn = u._stand_spot(meas_bc[i:i + 1], meas_by[i:i + 1], nbv[None], torch.ones(1, dtype=torch.bool, device=dev))[0]
                D_node[i] = torch.stack((Dn[0], Dn[1], top_b[i] + 0.029))
                stage[i] = 2; start_move(m, q_ready, 0.0)
            elif k == 2:  # S3 끝 → S4 (노드: r.target 없음 = 측정값, β π/4 · ρ 0, 행동 (0, .., 1))
                stage[i] = 3; t_in[i] = 0.0
                u.arm_target[i] = qm[i]; u.beta[i] = math.pi / 4; u.rho[i] = 0.0
                u.actions[i] = 0.0; u.actions[i, -1] = 1.0; u.prev_actions[i] = u.actions[i]
                ax_have[i] = False
                ho_on[i] = 0; ho_off[i] = 0; held[i] = False; grasp_z[i] = 0.0; lifted[i] = False; released[i] = False; clear[i] = 0.0
                if not a.node_release:  # 환경 성공 판정을 쓰려면 환경 단계 기록을 처음 상태로
                    for t_ in (u.was_held, u.grasp_seen, u.center_ok, u.lift10, u.stood, u.placed, u.released, u.succ_once, u.succeeded):
                        t_[i] = False
                    u.hold_ok[i] = 0; u.max_lift[i] = 0.0; u.hold_state[i] = False; u.hold_on[i] = 0; u.hold_off[i] = 0
                    u.D[i] = D_node[i]
            elif k == 4:  # S5 끝 → S6 D target_look
                Dbf = D_node[i] + torch.tensor([0.0, 0.0, 0.00675], device=dev)
                r = tlook(" ".join(f"{v:.6f}" for v in Dbf.tolist()))
                if r[0] != "ok":
                    stage[i], fail_why[i] = 7, "S6 target_look 해 없음"
                    continue
                q1 = q1_move.clone(); q1[i] = torch.tensor([float(v) for v in r[1:6]], device=dev)
                stage[i] = 5; start_move(m, q1, 0.3)
            elif k == 5:
                stage[i] = 6
        if bool((stage >= 6).all()):
            break
    for i in torch.nonzero(stage <= 5).flatten().tolist():
        stage[i], fail_why[i] = 7, f"60 s 안에 안 끝남 (단계 {STAGES[int(stage[i])]})"

    # ---- 보고 ----
    nb_on = u.nb_present.clone()
    pct = lambda m: f"{int(m.sum())}/{n} ({m.float().mean().item() * 100:.1f} %)"  # noqa: E731
    ok6 = stage == 6
    s4ok = torch.tensor([v == "ok" for v in s4_end], device=dev)
    print(f"[MSeqA] seed {a.seed}, 대 {n}, 옆 퍽 확률 {a.nb_prob} (있음 {int(nb_on.sum())}), v_max {a.vmax} ({a.vmode}), 인식 오차 {a.meas}, 고정 관측 {a.fix_obs}, 노드 규칙 {a.node_release}, 살린 칸 {a.live or "없음"}, 보기 자세 {a.view}, 놓음 수평 {a.release_xy} · 중심 보정 {a.release_center_offset}", flush=True)
    print(f"[MSeqA] **S6 까지 끝 {pct(ok6)}** | S4 정책 끝 (노드 끝 판정) {pct(s4ok)}", flush=True)
    from collections import Counter
    print(f"[MSeqA] S4 끝 사유: {dict(Counter(s4_end))}", flush=True)
    print(f"[MSeqA] 실패 사유: {dict(Counter(w for w in fail_why if w))}", flush=True)
    for k in EVENTS:
        es = ev_stage[k]
        by = {STAGES[s]: int((es == s).sum()) for s in range(6) if int((es == s).sum())}
        by_any = {STAGES[s]: int(ev_any[k][:, s].sum()) for s in range(6) if int(ev_any[k][:, s].sum())}
        print(f"[MSeqA] 사건 {k}: 대 {int((es >= 0).sum())} 처음 난 단계 {by} | 단계별 있었던 대 {by_any}", flush=True)
    mv_ev = torch.zeros(n, dtype=torch.bool, device=dev)
    for k in EVENTS:
        es = ev_stage[k]
        mv_ev |= (es == 0) | (es == 1) | (es == 2) | (es == 4) | (es == 5)
    mv_any = torch.zeros(n, dtype=torch.bool, device=dev)
    for k in EVENTS:
        mv_any |= ev_any[k][:, [0, 1, 2, 4, 5]].any(1)
    print(f"[MSeqA] 출발 자세 ALIGN_A 팔-몸체 접촉 (S1 첫 0.5 s, 사건에서 뺌) {pct(align_contact)}", flush=True)
    print(f"[MSeqA] 이동 단계 (S1 · S2 · S3 · S5 · S6) 사건 아무거나 있는 대 {pct(mv_any)}", flush=True)
    s4_clean = s4ok & (ev_stage["arm_box"] != 3)
    print(f"[MSeqA] S4 접촉 없는 끝 (팔-상자 접촉 없음) {pct(s4_clean)}", flush=True)
    for k in range(6):
        m = st_time[:, k] > 0
        if m.any():
            print(f"[MSeqA] 단계 {STAGES[k]} 시간 s 중앙 {st_time[m, k].median().item():.2f} 최대 {st_time[m, k].max().item():.2f}", flush=True)
    _rd = sorted(v for v in rel_dh if v is not None)
    if _rd:
        print(f"[MSeqA] 놓음 순간 잡는 점 – D 수평 mm: 중앙 {_rd[len(_rd)//2]*1000:.1f} 90 % {_rd[int(len(_rd)*0.9)]*1000:.1f} 최대 {_rd[-1]*1000:.1f} (놓음 {len(_rd)} 대, 기준 {a.release_xy*1000:.0f} mm)", flush=True)
    gx = g_end_x[~torch.isnan(g_end_x)]
    if len(gx):
        print(f"[MSeqA] S4 끝 잡는 점 base x m: 중앙 {gx.median().item():.3f} 90 % {gx.quantile(0.9).item():.3f} 최대 {gx.max().item():.3f}", flush=True)
    if a.dump:
        rq, rp = d.root_link_quat_w.torch, d.root_link_pos_w.torch
        iq = torch.cat((-rq[:, :3], rq[:, 3:]), dim=1)
        objs = {}
        for k_ in ("puckT_box", "boxA", "puckN"):
            o_ = u._obj(k_).data
            objs[k_] = (u.to_base(o_.root_pos_w.torch), quat_mul(iq, o_.root_quat_w.torch))
        jq = d.joint_pos.torch
        out = []
        for i in range(n):
            ev = {k: [STAGES[s] for s in range(6) if bool(ev_any[k][i, s])] for k in EVENTS}
            row = dict(env=i, ok=bool(ok6[i]), fail=fail_why[i], s4_end=s4_end[i], nb_on=bool(nb_on[i]), events=ev, align_contact=bool(align_contact[i]), q_s4end=q_s4end[i], rise_s5=rise_s5[i], rel_dh=rel_dh[i])
            if bool(ok6[i]):
                row.update(q=jq[i, u._arm].tolist(), q_tgt=u.arm_target[i].tolist(), fing=[float(jq[i, u._gl[0]]), float(jq[i, u._gr[0]])],
                           g=u.to_base(u.grasp_w())[i].tolist(), D_node=D_node[i].tolist(),
                           **{f"{k_}_p": v[0][i].tolist() for k_, v in objs.items()}, **{f"{k_}_q": v[1][i].tolist() for k_, v in objs.items()})
            out.append(row)
        with open(a.dump, "w") as f:
            json.dump(dict(note="미션 순서 모의 A단계 (S0 ~ S6) 끝 상태, base 좌표 · base 기준 자세 (x, y, z, w)", seed=a.seed, nb_prob=a.nb_prob, vmax=a.vmax,
                           meas=a.meas, fix_obs=a.fix_obs, node_release=a.node_release, num_envs=n, rows=out), f)
        print(f"[MSeqA] 끝 상태 -> {a.dump} (S6 까지 끝 {int(ok6.sum())})", flush=True)
    if a.dump:
        json.dump(open_log, open(a.dump.replace(".json", "_open.json"), "w"))
        print(f"[MSeqA] 벌림 기록 {len(open_log)} 개", flush=True)
    env.close()


if __name__ == "__main__":
    main()
    app.close()
