"""주사위 눈 놓기 환경 (2026-11-07, 설정 place_env_cfg.py). 45° 집기 환경 (sidepick_env) 의 IK · 행동 · 자기 충돌 · 팔-몸체 판정을 그대로 쓰고
배치 · 관측 · 보상 · 종료만 바꿈. 보상은 참 퍽 위치, 관측은 실물에서 얻을 수 있는 값 (놓기 가정 퍽 위치, 인식 칸 위치)."""

from __future__ import annotations

import json
import math
import os
from collections.abc import Sequence

import torch
from isaaclab.envs import DirectRLEnv
from isaaclab.utils.math import quat_from_euler_xyz, sample_uniform

from .dice2_cfg import BOX_H, BOX_SIZE, PUCK_H
from .place_env_cfg import DICE, GRID, L2_GRID_FROM_FACE, L2_SIZE, N_OBS, WafflePlace60EnvCfg
from .sidepick_env import WaffleSidePickEnv


class WafflePlace60Env(WaffleSidePickEnv):
    cfg: WafflePlace60EnvCfg

    def __init__(self, cfg: WafflePlace60EnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        n, dev = self.num_envs, self.device
        self.cell_w = torch.zeros(n, 3, device=dev)  # 목표 칸 중심 (참, 월드, 퍽 중심 높이)
        self.cell_bias = torch.zeros(n, 3, device=dev)
        self.obs_on = torch.zeros(n, N_OBS, dtype=torch.bool, device=dev)
        self.obs_xy0 = torch.zeros(n, N_OBS, 2, device=dev)
        self.box_top = torch.zeros(n, device=dev)
        self.released = torch.zeros(n, dtype=torch.bool, device=dev)
        self.rel_xy = torch.zeros(n, device=dev)
        self.rel_h = torch.zeros(n, device=dev)
        self.pk_rel_xy = torch.zeros(n, 2, device=dev)
        self.ok_cnt = torch.zeros(n, dtype=torch.long, device=dev)
        self.term_reason = torch.zeros(n, dtype=torch.long, device=dev)  # 진단: 1 성공, 2 나쁜 놓기, 3 빠뜨림, 4 쓰러짐, 5 장애물, 6 자기 · 몸체
        self.last_reason = torch.zeros(n, dtype=torch.long, device=dev)
        self.last_err = torch.zeros(n, device=dev)
        self.phi_prev = torch.zeros(n, device=dev)  # 퍼텐셜 (리셋 뒤 첫 스텝은 차이 0)
        self.phi_ok = torch.zeros(n, dtype=torch.bool, device=dev)
        # 경우 표 (주사위 눈, 모양) -> 칸 순서 (먼 줄부터)
        # 잘못된 DICE 정의가 학습 중 조용히 섞이지 않도록 환경 생성 시 검사한다.
        cases = []
        case_faces = []
        case_shapes = []
        if not DICE:
            raise ValueError("DICE 패턴 정의가 비어 있습니다.")

        for face, shapes in DICE.items():
            face_count = int(face)
            if not shapes:
                raise ValueError(f"DICE[{face!r}]에 패턴이 없습니다.")
            for shape_idx, cells in enumerate(shapes):
                cells = [tuple(map(int, cell)) for cell in cells]
                if len(cells) != face_count:
                    raise ValueError(
                        f"DICE[{face!r}][{shape_idx}]의 칸 수가 {len(cells)}개입니다. "
                        f"주사위 {face_count}의 칸 수와 일치해야 합니다."
                    )
                if len(set(cells)) != len(cells):
                    raise ValueError(f"DICE[{face!r}][{shape_idx}]에 중복 좌표가 있습니다.")
                if any(u not in (-1, 0, 1) or v not in (-1, 0, 1) for u, v in cells):
                    raise ValueError(
                        f"DICE[{face!r}][{shape_idx}] 좌표는 3x3 범위(-1, 0, 1)여야 합니다."
                    )
                cells = sorted(cells, key=lambda c: (-c[0], -c[1]))
                cases.append(cells)
                case_faces.append(face_count)
                case_shapes.append(shape_idx)

        max_required_obs = max(len(cells) - 1 for cells in cases)
        if N_OBS < max_required_obs:
            raise ValueError(
                f"N_OBS={N_OBS}이지만 가장 큰 패턴은 장애물 {max_required_obs}개가 필요합니다."
            )

        self._cases = cases
        self._case_faces = torch.tensor(case_faces, dtype=torch.long, device=dev)
        self._case_shapes = torch.tensor(case_shapes, dtype=torch.long, device=dev)
        self.case_face = torch.zeros(n, dtype=torch.long, device=dev)
        self.case_shape = torch.zeros(n, dtype=torch.long, device=dev)
        self.target_grid_uv = torch.zeros(n, 2, dtype=torch.long, device=dev)
        # 2026-11-09 시작 자세 표 (스토리지에서 꺼내 위로 가져온 직후 흉내, tools_ik/place_starts.py)
        self._st_q = self._st_g = None
        if cfg.start_table:
            fn = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), cfg.start_table)
            with open(fn, "r", encoding="utf-8") as f:
                st = json.load(f)["starts"]
            if not st:
                raise ValueError(f"시작 자세 표에 starts 항목이 없습니다: {fn}")
            self._st_q = torch.tensor([e["q"] for e in st], device=dev)
            self._st_g = torch.tensor([e["g"] for e in st], device=dev)
            print(f"[Place] 시작 자세 표 {cfg.start_table}: {len(st)} 개, 쓰는 확률 {cfg.start_table_prob}", flush=True)

    # ---- 행동: 집게는 행동 > release_thresh 일 때만 벌림 (처음 학습 때 아무렇게나 벌려 바로 끝나는 것 줄이기) ----
    def _apply_action(self):
        cmd = self.robot.actuators.target_command
        # 2026-11-09 한 번 놓으면 계속 벌림 (상자 60° 학습이 놓은 뒤 다시 닫아 퍽을 쥔 채 시간 끝 = 성공 0/64 의 원인)
        opn = (self.actions[:, -1:] > self.cfg.release_thresh) | self.released[:, None]
        g = torch.where(opn, self.cfg.gripper_open, self.cfg.gripper_closed)
        cmd.set_position_index(value=self.arm_target.contiguous(), joint_ids=self._arm)
        cmd.set_position_index(value=g.expand(-1, 2).contiguous(), joint_ids=self._gl + self._gr)
        cmd.set_velocity_index(value=torch.zeros(self.num_envs, len(self._wheels), device=self.device), joint_ids=self._wheels)

    # ---- 공용 ----
    def assumed_puck_w(self):
        """놓기 가정 (실물): 퍽 중심 = 잡는 점 15 mm 아래 (수직)."""
        return self.grasp_w() - self._ez * self.cfg.grasp_dz

    def held(self):
        return torch.linalg.norm(self.puck_pos_w() - self.grasp_w(), dim=1) < 0.035

    def _obs_pos(self):
        return torch.stack([self._obj(f"obs{j}").data.root_pos_w.torch for j in range(N_OBS)], dim=1)

    # ---- 관측 ----
    def _get_observations(self):
        d = self.robot.data
        pb = self.to_base(self.assumed_puck_w())
        tb = self.to_base(self.cell_w + self.cell_bias)
        ax, _, az = self.a5_axes_w()
        ob = (self._obs_pos()[..., :2] - (self.cell_w + self.cell_bias)[:, None, :2]) * self.obs_on[..., None].float()
        obs = torch.cat((
            d.joint_pos.torch[:, self._arm] - d.default_joint_pos.torch[:, self._arm], d.joint_vel.torch[:, self._arm],
            d.joint_pos.torch[:, self._gl], pb, tb, tb - pb, self.vec_base(-az), self.vec_base(ax),
            ob.flatten(1), self.obs_on.float(), self.is_box.float().unsqueeze(1), self.actions,
        ), dim=-1)
        return {"policy": obs}

    # ---- 상태 (DirectRLEnv.step 은 _get_dones -> _get_rewards 순서라 여기서 한 번 계산) ----
    def _update_state(self):
        c = self.cfg
        p = self.puck_pos_w()
        self.s_p = p
        self.s_exy = torch.linalg.norm(p[:, :2] - self.cell_w[:, :2], dim=1)
        self.s_h = p[:, 2] - PUCK_H / 2 - self.box_top  # 퍽 바닥 - 상자 윗면
        up = self.puck_up()
        self.s_tilt = torch.acos(up.clamp(max=1.0))
        held_now = self.held()
        self.s_held = held_now & ~self.released
        newrel = (self.actions[:, -1] > self.cfg.release_thresh) & ~self.released & held_now
        self.s_newrel = newrel
        self.rel_xy = torch.where(newrel, self.s_exy, self.rel_xy)
        self.rel_h = torch.where(newrel, self.s_h, self.rel_h)
        self.s_good_rel = newrel & (self.s_exy < c.release_xy_max) & (self.s_h < c.release_h_max)
        self.bad_rel = newrel & ~self.s_good_rel
        self.released |= newrel
        self.pk_rel_xy = torch.where(newrel[:, None], p[:, :2], self.pk_rel_xy)
        self.dropped = ~self.released & ~held_now
        self.s_gd = torch.linalg.norm(self.grasp_w() - p, dim=1)
        self.s_moved = torch.linalg.norm(p[:, :2] - self.pk_rel_xy, dim=1) * self.released.float()
        resting = self.released & (self.s_h.abs() < 0.003) & (up > 0.98) & (self.s_exy < c.place_tol) & (self.s_gd > c.retract_dist)
        self.ok_cnt = torch.where(resting, self.ok_cnt + 1, torch.zeros_like(self.ok_cnt))
        self.succeeded |= self.ok_cnt >= c.success_hold_steps
        self.toppled_p = self.released & ((up < 0.9) | (self.s_h < -0.01))
        obs_moved = (torch.linalg.norm(self._obs_pos()[..., :2] - self.obs_xy0, dim=-1) * self.obs_on.float()).amax(dim=1)
        self.obs_hit = self._contact("arm_obs_contact") | (obs_moved > c.obs_move_max)

    # ---- 보상 ----
    def _get_rewards(self):
        c = self.cfg
        e_xy, h, tilt, held, gd, moved = self.s_exy, self.s_h, self.s_tilt, self.s_held, self.s_gd, self.s_moved
        good_rel, newrel = self.s_good_rel, self.s_newrel
        aligned = (e_xy < 0.01).float()
        far = e_xy > 0.02
        # 퍼텐셜 차이 (머물면 0): 수평 오차 + 칸에 가까울수록 무게를 두는 높이
        phi = -(torch.clamp(e_xy / 0.01, max=10.0) + torch.exp(-e_xy / 0.01) * torch.clamp(h.clamp(min=0.0) / 0.01, max=10.0))
        dphi = torch.where(self.phi_ok & held, phi - self.phi_prev, torch.zeros_like(phi))
        self.phi_prev = phi
        self.phi_ok = held.clone()
        rew = (
            c.rew_phi * dphi + c.rew_time * (~self.released).float()
            + held.float() * (c.rew_xy * (1 - torch.tanh(e_xy / 0.05)) + c.rew_xy_fine * (1 - torch.tanh(e_xy / 0.008))
                            + c.rew_lower * aligned * (1 - torch.tanh(h.clamp(min=0.0) / 0.02))
                            + c.rew_low_far * (far & (h < 0.02)).float()
                            + c.rew_held_tilt_p * torch.clamp(tilt / math.radians(5.0), max=3.0))
            + c.rew_release_good * good_rel.float() * (1.5 - e_xy / c.place_tol).clamp(min=0.0)
            + c.rew_release_bad * self.bad_rel.float() + c.rew_drop * self.dropped.float()
            + self.released.float() * c.rew_retract * torch.clamp((gd - 0.03) / 0.03, 0.0, 1.0)
            + c.rew_after_move * moved * 100.0 * (moved > 0.002).float()
            + c.rew_success * (self.ok_cnt == c.success_hold_steps).float()
            + c.rew_topple_p * self.toppled_p.float() + c.rew_obs_hit * self.obs_hit.float()
            + c.rew_arm_box * (self._contact("arm_box_contact") | self._contact("arm_plat_contact")).float() + c.rew_body_box * self._contact("body_box_contact").float()
            + c.rew_action_rate * torch.sum(torch.square(self.actions - self.prev_actions), dim=1)
            + c.rew_joint_vel * torch.sum(torch.square(self.robot.data.joint_vel.torch[:, self._arm]), dim=1)
        )
        sb = self.self_bad()
        j3 = self.robot.data.joint_pos.torch[:, self._arm[2]].abs()
        rew = rew + c.rew_self_end * sb.float() + c.rew_j3_bend * torch.clamp((j3 - c.j3_bend_start) / 0.1, 0.0, 3.0)
        rew = rew + c.rew_arm_body_end * self.arm_body_hit().float()
        lg = self.extras.setdefault("log", {})
        if newrel.any():
            lg["Metrics/release_xy_mm"] = (e_xy[newrel] * 1000).mean().item()
            lg["Metrics/release_h_mm"] = (h[newrel] * 1000).mean().item()
        lg["Metrics/obs_hit_ratio"] = self.obs_hit.float().mean().item()
        lg["Metrics/held_err_xy_mm"] = (e_xy[held] * 1000).mean().item() if held.any() else 0.0
        return rew

    def _get_dones(self):
        self._update_state()
        tipped = self.robot.data.projected_gravity_b.torch[:, 2] > -0.6
        self_hit = self.self_bad() | self.arm_body_hit() | self._contact("body_box_contact")
        reason = torch.zeros_like(self.term_reason)
        reason = torch.where(self_hit | tipped, 6, reason)
        reason = torch.where(self.obs_hit, 5, reason)
        reason = torch.where(self.toppled_p, 4, reason)
        reason = torch.where(self.dropped, 3, reason)
        reason = torch.where(self.bad_rel, 2, reason)
        reason = torch.where(self.succeeded, 1, reason)
        self.term_reason = reason
        end = reason > 0
        return end, self.episode_length_buf >= self.max_episode_length - 1

    # ---- 초기화 ----
    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        if len(env_ids) > 0:
            lg = self.extras.setdefault("log", {})
            rs = self.term_reason[env_ids]
            lg["Metrics/success_rate"] = self.succeeded[env_ids].float().mean().item()
            for k, nm in ((2, "bad_release"), (3, "drop"), (4, "topple"), (5, "obs_hit"), (6, "self_body")):
                lg[f"Metrics/end_{nm}"] = (rs == k).float().mean().item()
            self.last_reason[env_ids] = rs
            self.last_err[env_ids] = self.rel_xy[env_ids]
        DirectRLEnv._reset_idx(self, env_ids)
        c, r, n, dev = self.cfg, self.robot, len(env_ids), self.device
        org = self.scene.env_origins[env_ids]

        def U(lo, hi, shape):
            return sample_uniform(lo, hi, shape, dev)

        root = r.data.default_root_pose.torch[env_ids].clone()
        root[:, :3] += org
        r.write_root_pose_to_sim_index(root_pose=root, env_ids=env_ids)
        r.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=env_ids)
        self.root0[env_ids] = root[:, :3]
        q = r.data.default_joint_pos.torch[env_ids].clone()
        g_b = torch.tensor([c.start_grasp_b], device=dev).expand(n, 3).clone()
        if self._st_q is not None:
            use = torch.rand(n, device=dev) < c.start_table_prob
            k = torch.randint(0, len(self._st_q), (n,), device=dev)
            qa = q[:, self._arm]
            q[:, self._arm] = torch.where(use[:, None], self._st_q[k], qa)
            g_b = torch.where(use[:, None], self._st_g[k], g_b)
        r.write_joint_position_to_sim_index(position=q, env_ids=env_ids)
        r.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), env_ids=env_ids)
        self.arm_target[env_ids] = q[:, self._arm]
        # 상자
        yaw = U(-c.place_yaw, c.place_yaw, (n,))
        face = torch.stack((c.robot_front + U(*c.place_gap, (n,)), U(*c.place_lat, (n,))), dim=1)  # 상자 · 받침 앞면 가운데
        fwd = torch.stack((torch.cos(yaw), torch.sin(yaw)), dim=1)
        if c.surface == "l2":  # L2 받침: 격자 가운데 = 앞면에서 0.21 m, 받침 가운데 = 앞면에서 0.41 m, 205 상자는 치움
            bc = face + fwd * L2_GRID_FROM_FACE
            self._place("platformL2", env_ids, face + fwd * (L2_SIZE / 2), yaw, torch.full((n,), BOX_H / 2, device=dev))
            self._place("boxA", env_ids, torch.tensor([0.0, 0.0], device=dev).expand(n, 2), yaw, torch.full((n,), -0.5, device=dev))
        else:
            bc = face + fwd * (BOX_SIZE / 2)
            self._place("boxA", env_ids, bc, yaw, torch.full((n,), BOX_H / 2, device=dev))
        self.box_top[env_ids] = org[:, 2] + BOX_H  # 상자 윗면 (월드)
        # 주사위 눈 · 칸 순서 -> 목표 = k 번째, 장애물 = 앞의 것들
        ci = torch.randint(0, len(self._cases), (n,), device=dev)
        # 확인용 고정 (play 에서 완성된 모양을 보려고): DICE_FACE=5 [DICE_SHAPE=0] [DICE_K=5]  (K 는 1 부터, 큰 값 = 마지막 칸, 없으면 무작위)
        fix_face = int(os.environ.get("DICE_FACE", "0"))
        fix_shape = int(os.environ.get("DICE_SHAPE", "-1"))
        fix_k = int(os.environ.get("DICE_K", "0"))
        if fix_face:
            ok = [i for i, (f, sh) in enumerate(zip(self._case_faces.tolist(), self._case_shapes.tolist())) if f == fix_face and (fix_shape < 0 or sh == fix_shape)]
            if not ok:
                raise ValueError(f"DICE_FACE={fix_face} DICE_SHAPE={fix_shape} 에 맞는 패턴이 없습니다.")
            ci = torch.tensor(ok, device=dev)[torch.randint(0, len(ok), (n,), device=dev)]
        cy, sy = torch.cos(yaw), torch.sin(yaw)
        obs_on = torch.zeros(n, N_OBS, dtype=torch.bool, device=dev)
        cell = torch.zeros(n, 2, device=dev)
        obs_xy = torch.zeros(n, N_OBS, 2, device=dev)
        target_uv = torch.zeros(n, 2, dtype=torch.long, device=dev)
        for i in range(n):  # 경우 수가 적어 환경 리셋 때만 파이썬으로 처리
            case_idx = int(ci[i].item())
            cells = self._cases[case_idx]
            kk = int(torch.randint(0, len(cells), (1,), device=dev).item()) if fix_k <= 0 else min(fix_k - 1, len(cells) - 1)
            self.case_face[env_ids[i]] = self._case_faces[case_idx]
            self.case_shape[env_ids[i]] = self._case_shapes[case_idx]
            for j, (uu, vv) in enumerate(cells[:kk + 1]):
                du, dv = uu * GRID, vv * GRID
                xy = torch.stack((
                    bc[i, 0] + cy[i] * du - sy[i] * dv,
                    bc[i, 1] + sy[i] * du + cy[i] * dv,
                ))
                if j < kk:
                    obs_on[i, j] = True
                    obs_xy[i, j] = xy
                else:
                    cell[i] = xy
                    target_uv[i] = torch.tensor((uu, vv), dtype=torch.long, device=dev)
        self.target_grid_uv[env_ids] = target_uv
        zc = BOX_H + PUCK_H / 2 + 0.001
        for j in range(N_OBS):
            park = torch.tensor([-0.5, 0.40 + 0.07 * j], device=dev).expand(n, 2)
            self._place(f"obs{j}", env_ids, torch.where(obs_on[:, j:j + 1], obs_xy[:, j], park), U(0.0, math.pi / 2, (n,)), torch.full((n,), zc, device=dev))
        self.obs_on[env_ids] = obs_on
        self.obs_xy0[env_ids] = obs_xy + org[:, None, :2]  # 켜진 것만 씀 (obs_on 으로 가림)
        self.cell_w[env_ids] = torch.cat((cell + org[:, :2], (org[:, 2] + zc)[:, None]), dim=1)
        self.cell_bias[env_ids] = torch.cat((U(-c.cell_perc_bias, c.cell_perc_bias, (n, 2)), torch.zeros(n, 1, device=dev)), dim=1)
        # 실제 배치가 의도한 주사위 패턴과 다른지 추적할 수 있도록 선택 정보를 기록한다.
        self.extras.setdefault("log", {})["Metrics/dice_face"] = self.case_face[env_ids].float().mean().item()
        self.extras["log"]["Metrics/dice_shape"] = self.case_shape[env_ids].float().mean().item()
        self.extras["log"]["Metrics/target_grid_u"] = self.target_grid_uv[env_ids, 0].float().mean().item()
        self.extras["log"]["Metrics/target_grid_v"] = self.target_grid_uv[env_ids, 1].float().mean().item()
        # 쥔 퍽: 시작 자세의 잡는 점 15 mm 아래 + 손 안 어긋남 (집게 방향 수평 · 위아래)
        is_box = torch.rand(n, device=dev) < c.box_puck_prob
        self.is_box[env_ids] = is_box
        g0 = g_b + root[:, :3]
        pc = g0.clone()
        pc[:, 0] += U(-c.inhand_err, c.inhand_err, (n,))
        pc[:, 2] += -c.grasp_dz + U(-c.inhand_err, c.inhand_err, (n,))
        park = torch.tensor([0.0, 0.7], device=dev).expand(n, 2)
        zero = torch.zeros(n, device=dev)
        for k_, use in (("puckT_box", is_box), ("puckT_cyl", ~is_box)):
            o = self._obj(k_)
            xy = torch.where(use[:, None], pc[:, :2], park + org[:, :2] + (0.1 if k_ == "puckT_cyl" else 0.0))
            z = torch.where(use, pc[:, 2], org[:, 2] + zc)
            pose = torch.cat((xy, z[:, None], quat_from_euler_xyz(zero, zero, zero)), dim=1)
            o.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
            o.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=env_ids)
        self._place("puckN", env_ids, torch.tensor([0.2, 0.7], device=dev).expand(n, 2), zero, torch.full((n,), zc, device=dev))
        self.puck_z0[env_ids] = pc[:, 2]
        self.perc_bias[env_ids] = 0.0
        self.nb_present[env_ids] = False
        self.actions[env_ids] = 0.0
        self.actions[env_ids, -1] = -1.0  # 쥔 채 시작
        self.prev_actions[env_ids] = self.actions[env_ids]
        self.released[env_ids] = False
        self.phi_ok[env_ids] = False
        self.rel_xy[env_ids] = 0.0
        self.rel_h[env_ids] = 0.0
        self.ok_cnt[env_ids] = 0
        self.succeeded[env_ids] = False
        self.term_reason[env_ids] = 0
        self.close_cnt[env_ids] = 0
