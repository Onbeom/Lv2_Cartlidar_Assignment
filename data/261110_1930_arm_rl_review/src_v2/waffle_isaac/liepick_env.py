"""누운 직사각형 퍽 집기·세우기 `Waffle-LiePick-Direct` 구현. 설정과 설명은 liepick_env_cfg.py.

정렬·집기 (SidePick) 환경을 이어받음: 장면 (로봇·상자·목표 퍽 원기둥/상자·옆 퍽), 접촉 센서 (상자·옆 퍽), 옆 퍽 벌점은 같음.
여기서는 목표 퍽 = 상자 모양 (30 x 30 x 60) 을 **눕혀** 두고, 집게 자세 목표 (ψ, φ) 를 행동으로 바꿈.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import torch

from isaaclab.utils.math import quat_apply, quat_box_minus, quat_from_euler_xyz, quat_from_matrix, quat_mul, sample_uniform

from .dice2_cfg import BOX_H, BOX_SIZE
from .sidepick_env import WaffleSidePickEnv

LOW_TABLE = [  # (joint1 축 거리 r, 관절 2, 3, 4): 잡는 점 = 누운 퍽 윗면 + 12 mm, 집게 아래 방향, 손가락 높이 방향 = 팔 방향 (MuJoCo IK)
    (0.180, 0.297, -0.030, 1.304), (0.190, 0.374, -0.149, 1.346), (0.200, 0.456, -0.281, 1.396), (0.210, 0.544, -0.429, 1.455),
    (0.220, 0.645, -0.603, 1.529), (0.230, 0.768, -0.828, 1.631), (0.240, 1.001, -1.278, 1.848),
]


class WaffleLiePickEnv(WaffleSidePickEnv):

    def __init__(self, cfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        n, dev = self.num_envs, self.device
        self.is_box[:] = True
        self.psi = torch.zeros(n, device=dev)
        self.phi = torch.zeros(n, device=dev)
        self.was_grasped = torch.zeros(n, dtype=torch.bool, device=dev)
        self.succ_once = torch.zeros(n, dtype=torch.bool, device=dev)
        self.prev_gd = torch.zeros(n, device=dev)
        self.max_stand = torch.zeros(n, device=dev)
        self.off_steps = torch.zeros(n, device=dev)
        self.puck_z0[:] = cfg.lie_z

    # ---- 상태 ----
    def long_axis_w(self):
        """누운 퍽의 긴 축 (상자 모양 퍽 몸 z)."""
        return quat_apply(self.puck_quat_w(), self._ez)

    def center_off(self):
        """잡는 점 - 퍽 중심 의 긴 축 방향 성분 [m] (가운데 잡기 판정)."""
        return ((self.grasp_w() - self.puck_pos_w()) * self.long_axis_w()).sum(1)

    def grasp_target_w(self):
        p = self.puck_pos_w().clone()
        p[:, 2] += self.cfg.grasp_dz_lie
        return p

    def target_R_base(self):
        """집게 자세 목표 (base 좌표, 열 = 집게 방향 a, 닫는 축 c, 손가락 높이 방향 u)."""
        cp, sp, cf, sf = torch.cos(self.psi), torch.sin(self.psi), torch.cos(self.phi), torch.sin(self.phi)
        z = torch.zeros_like(cp)
        a = torch.stack((sf * cp, sf * sp, -cf), dim=1)
        c = torch.stack((-sp, cp, z), dim=1)
        u = torch.cross(a, c, dim=1)
        return torch.stack((a, c, u), dim=2)

    # ---- 행동 ----
    def _pre_physics_step(self, actions):
        c = self.cfg
        self.prev_actions = self.actions.clone()
        self.actions = actions.clamp(-1.0, 1.0)
        self.psi = self.psi + self.actions[:, 3] * c.psi_scale
        self.phi = torch.clamp(self.phi + self.actions[:, 4] * c.phi_scale, 0.0, math.pi / 2)
        q = self.robot.data.joint_pos.torch[:, self._arm]
        dq = self._ik_pose(self.actions[:, :3] * c.cart_scale)
        t = torch.clamp(self.arm_target + dq, q - c.target_lead_max, q + c.target_lead_max)
        self.arm_target = torch.clamp(t, self._lo, self._hi)

    def _ik_pose(self, dp_b):
        c, d = self.cfg, self.robot.data
        rq = d.root_link_quat_w.torch
        J = d.body_link_jacobian_w.torch[:, self._a5][:, :, self._cols]
        q5 = d.body_link_quat_w.torch[:, self._a5]
        rr = quat_apply(q5, self._off)
        Jl = J[:, 0:3] - torch.cross(rr.unsqueeze(-1).expand(-1, -1, 5), J[:, 3:6], dim=1)
        w = c.rot_hold_weight
        Jf = torch.cat((Jl, J[:, 3:6] * w), dim=1)
        qt = quat_mul(rq, quat_from_matrix(self.target_R_base()))
        err = torch.cat((quat_apply(rq, dp_b), quat_box_minus(qt, q5) * w), dim=1)
        JJt = Jf @ Jf.transpose(1, 2) + c.ik_damping * torch.eye(6, device=self.device)
        return (Jf.transpose(1, 2) @ torch.linalg.solve(JJt, err.unsqueeze(-1))).squeeze(-1)

    # ---- 관측·보상·종료 ----
    def _get_observations(self):
        d = self.robot.data
        perc = self.puck_pos_w() + self.perc_bias + (torch.rand(self.num_envs, 3, device=self.device) * 2 - 1) * self.cfg.perc_jitter
        pb = self.to_base(perc)
        gb = self.to_base(self.grasp_w())
        ax, _, az = self.a5_axes_w()
        nb = self.to_base(self._obj("puckN").data.root_pos_w.torch)[:, :2] - pb[:, :2]
        obs = torch.cat((
            d.joint_pos.torch[:, self._arm] - d.default_joint_pos.torch[:, self._arm], d.joint_vel.torch[:, self._arm],
            d.joint_pos.torch[:, self._gl], pb, gb, pb - gb, self.vec_base(-az), self.vec_base(ax), self.vec_base(self.long_axis_w()), nb,
            torch.stack((torch.cos(self.psi), torch.sin(self.psi), self.phi), dim=1), self.actions,
        ), dim=-1)
        return {"policy": obs}

    def _get_rewards(self):
        c = self.cfg
        g = self.grasp_w()
        dist = torch.linalg.norm(self.grasp_target_w() - g, dim=1)
        pc = self.puck_pos_w()
        gd = torch.linalg.norm(pc - g, dim=1)  # 잡는 점-퍽 중심
        lift = self.lift()
        close = (dist < 0.06).float()
        ax, ay, az = self.a5_axes_w()
        appr, upv = -az, -ay
        L = self.long_axis_w()
        stand = L[:, 2].abs()
        off = self.center_off().abs()
        fing = self.robot.data.joint_pos.torch[:, self._gl[0]]
        closed_ok = (fing > c.grasp_finger_range[0]) & (fing < c.grasp_finger_range[1])
        holding = closed_ok & (gd < 0.03)
        grasped = holding & (lift < c.lift_ok)
        self.was_grasped |= holding
        lifted = holding & (lift > c.lift_ok)
        off_bad = holding & (off > c.center_tol)
        off_w = torch.clamp((off - c.center_tol) / c.center_tol, 0.0, 1.0) * holding.float()  # LP-3: 비례 벌점
        height = torch.clamp(lift, 0.0, c.lift_full) / c.lift_full * holding.float()
        j1 = self.robot.data.root_link_pos_w.torch + quat_apply(self.robot.data.root_link_quat_w.torch, torch.tensor([[c.joint1_x, 0.0, 0.0]], device=self.device).expand(self.num_envs, 3))
        gb = self.vec_base(g - j1)
        th_arm = torch.atan2(gb[:, 1], gb[:, 0])
        psi_arm = torch.clamp(torch.cos(self.psi - th_arm), min=0.0)
        level = 1 - appr[:, 2].abs()
        success = lifted & (stand > c.stand_cos) & (off < c.center_tol) & (level > 0.98)
        self.hold = torch.where(success, self.hold + 1, torch.zeros_like(self.hold))
        succ_now = (self.hold >= int(c.success_hold_s / (c.sim.dt * c.decimation))) & ~self.succ_once
        self.succ_once |= succ_now
        self.succeeded |= succ_now
        slip = torch.clamp(gd - self.prev_gd, min=0.0) * holding.float() / 0.005
        self.prev_gd = gd
        nb_touch = self._contact("arm_nb_contact")
        nb_topple = self.nb_toppled()
        arm_box = self._contact("arm_box_contact")
        body_box = self._contact("body_box_contact")
        base_move = torch.linalg.norm(self.robot.data.root_link_pos_w.torch[:, :2] - self.root0[:, :2], dim=1)
        dropped = self.was_grasped & ~holding & (lift < c.lift_ok)
        pre = ~holding
        rew = (
            pre.float() * (c.rew_reach * (1 - torch.tanh(dist / 0.1)) + c.rew_reach_fine * (1 - torch.tanh(dist / 0.015))
                           + close * (c.rew_yaw_align * (upv * L).sum(1).abs() + c.rew_center * (1 - torch.tanh(off / 0.003))))
            + c.rew_grasp * grasped.float() + c.rew_off_center * off_w
            + c.rew_close_near * ((dist < 0.01) & (self.actions[:, -1] < 0.0)).float()
            + c.rew_lifted * lifted.float() + c.rew_lift_height * height
            + lifted.float() * (c.rew_stand * stand + c.rew_level_after * level * (stand > 0.9).float() + c.rew_psi_arm * psi_arm + c.rew_pitch * level)
            + c.rew_success * succ_now.float() + c.rew_puck_slip * slip
            + c.rew_nb_touch * nb_touch.float() + c.rew_nb_topple * nb_topple.float()
            + c.rew_nb_touch_end * (nb_touch & torch.tensor(c.nb_touch_terminate, device=self.device)).float()
            + c.rew_nb_near * torch.clamp(1 - self.nb_near() / c.nb_near_m, 0.0, 1.0)
            + c.rew_arm_box * arm_box.float() + c.rew_body_box * body_box.float() + c.rew_base_move * (base_move > c.base_move_max).float()
            + c.rew_drop * dropped.float() + c.rew_base_tilt * self.base_tilt()
            + c.rew_action_rate * torch.sum(torch.square(self.actions - self.prev_actions), dim=1)
            + c.rew_joint_vel * torch.sum(torch.square(self.robot.data.joint_vel.torch[:, self._arm]), dim=1)
        )
        self.max_lift = torch.maximum(self.max_lift, lift * holding.float())
        self.max_stand = torch.maximum(self.max_stand, stand * lifted.float())
        self.off_steps += off_bad.float()
        self.arm_box_steps += arm_box.float()
        log = self.extras.setdefault("log", {})
        log["Metrics/grasp_distance"] = dist.mean().item()
        log["Metrics/center_off_mm"] = (off * holding.float()).sum().item() / max(1.0, holding.float().sum().item()) * 1000
        log["Metrics/grasped_ratio"] = holding.float().mean().item()
        log["Metrics/lifted_ratio"] = lifted.float().mean().item()
        log["Metrics/stand_when_lifted"] = (stand * lifted.float()).sum().item() / max(1.0, lifted.float().sum().item())
        log["Metrics/phi"] = self.phi.mean().item()
        log["Metrics/nb_touch_ratio"] = nb_touch.float().mean().item()
        log["Metrics/nb_topple_ratio"] = nb_topple.float().mean().item()
        log["Metrics/arm_box_contact_ratio"] = arm_box.float().mean().item()
        log["Metrics/body_box_contact_ratio"] = body_box.float().mean().item()
        return rew

    def _fell(self):
        lift = self.lift()
        g = self.grasp_w()
        holding = torch.linalg.norm(self.puck_pos_w() - g, dim=1) < 0.03
        return self.was_grasped & ~holding & (lift < self.cfg.lift_ok) | (self.puck_pos_w()[:, 2] < self.puck_z0 - 0.03)

    # ---- 초기화 ----
    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        if len(env_ids) > 0:
            log = self.extras.setdefault("log", {})
            log["Metrics/success_rate"] = self.succeeded[env_ids].float().mean().item()
            log["Metrics/max_lift"] = self.max_lift[env_ids].mean().item()
            log["Metrics/max_stand"] = self.max_stand[env_ids].mean().item()
            log["Metrics/off_center_steps"] = self.off_steps[env_ids].mean().item()
        super(WaffleSidePickEnv, self)._reset_idx(env_ids)
        c, r, n, dev = self.cfg, self.robot, len(env_ids), self.device
        org = self.scene.env_origins[env_ids]

        def U(lo, hi, shape):
            return sample_uniform(lo, hi, shape, dev)

        root = r.data.default_root_pose.torch[env_ids].clone()
        root[:, :3] += org
        r.write_root_pose_to_sim_index(root_pose=root, env_ids=env_ids)
        r.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=env_ids)
        self.root0[env_ids] = root[:, :3]

        # 퍽 (누움): joint1 축 r, 방향 ang, 긴 축 요 무작위
        rr, ang = U(*c.r_range, (n,)), U(-c.ang_max, c.ang_max, (n,))
        p = torch.stack((c.joint1_x + rr * torch.cos(ang), rr * torch.sin(ang)), dim=1)
        lyaw = U(-math.pi / 2, math.pi / 2, (n,))
        # 상자: 퍽이 안에 들고 (긴 축 끝 + 5 mm), 가까운 면이 로봇 앞 끝 + gap_min 넘게
        byaw = U(-0.2, 0.2, (n,))
        half = BOX_SIZE / 2
        slack = half - 0.035
        bc = p + torch.stack((U(0.0, slack, (n,)), U(-slack, slack, (n,))), dim=1)
        bc[:, 0] = torch.maximum(bc[:, 0], torch.full_like(bc[:, 0], c.robot_front + c.gap_min + half * (torch.cos(byaw).abs() + torch.sin(byaw).abs()).max().item()))
        bc[:, 0] = torch.minimum(bc[:, 0], p[:, 0] + slack)
        self._place("boxA", env_ids, bc, byaw, torch.full((n,), BOX_H / 2, device=dev))
        # 누운 상자 퍽: 몸 z (긴 축) 를 수평으로 (x 축 둘레 90°) 그 뒤 요
        zero = torch.zeros(n, device=dev)
        qlie = quat_mul(quat_from_euler_xyz(zero, zero, lyaw), quat_from_euler_xyz(torch.full((n,), math.pi / 2, device=dev), zero, zero))
        self._place_q("puckT_box", env_ids, p, qlie, torch.full((n,), c.lie_z, device=dev))
        park = torch.tensor([0.0, 0.7], device=dev).expand(n, 2)
        self._place("puckT_cyl", env_ids, park + torch.tensor([0.1, 0.0], device=dev), zero, torch.full((n,), 0.031, device=dev))
        # 옆 퍽 (세운 원기둥): 목표에서 nb_dist, 로봇에서 먼 쪽 반원, 상자 안
        na = U(-math.pi / 2, math.pi / 2, (n,)) + ang
        nd = U(*c.nb_dist, (n,))
        nbr = p + torch.stack((torch.cos(na), torch.sin(na)), dim=1) * nd.unsqueeze(1)
        lo = bc - (half - 0.02)
        hi = bc + (half - 0.02)
        nbr = torch.maximum(torch.minimum(nbr, hi), lo)
        self._place("puckN", env_ids, nbr, zero, torch.full((n,), BOX_H + 0.031, device=dev))
        self.nb_prev[env_ids] = nbr + org[:, :2]
        self.puck_z0[env_ids] = c.lie_z + org[:, 2]

        # 팔: 시작 자세 (퍽 위 6 cm, 집게 아래) + joint1 = 퍽 방향. 역방향 커리큘럼: 요를 퍽 긴 축에 맞춰 둠
        q = r.data.default_joint_pos.torch[env_ids].clone()
        rp = torch.tensor([list(c.ready_pose.values())], device=dev).expand(n, 5).clone()
        rp[:, 0] = ang
        q[:, self._arm] = torch.clamp(rp + U(-1.0, 1.0, (n, 5)) * c.ready_noise, self._lo, self._hi)
        r.write_joint_position_to_sim_index(position=q, env_ids=env_ids)
        r.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), env_ids=env_ids)
        self.arm_target[env_ids] = q[:, self._arm]
        near = torch.rand(n, device=dev) < c.near_start_prob
        # ψ: 손가락 높이 방향 u = (cos ψ, sin ψ) 가 팔 방향 (= ang) 으로 시작, near 면 퍽 긴 축 (요 mod π 중 가까운 것)
        lp = lyaw + math.pi / 2  # 긴 축 수평 방향 (x 축 90° 뒤 몸 z = (0, −1, 0) -> 요 회전) 의 각
        lp = lp - math.pi * torch.round((lp - ang) / math.pi)
        self.psi[env_ids] = torch.where(near, lp, ang)
        # LP-2: near 중 일부는 집게를 퍽 윗면 1.2 cm 위 (잡는 점 목표 + 12 mm, 아래 방향) 에서 시작 = 닫기 직전. 관절 2·3·4 는 joint1 축 거리별
        #   MuJoCo IK 표 (`lie_low_table.py`, 손가락 밑-상자 17 mm) 를 선형 보간
        low = near & (torch.rand(n, device=dev) < c.near_low_prob)
        if low.any():
            tab = torch.tensor(LOW_TABLE, device=dev)
            k = torch.clamp((rr - tab[0, 0]) / 0.01, 0.0, len(LOW_TABLE) - 1.001)
            k0 = k.long()
            fr = (k - k0).unsqueeze(1)
            qq = tab[k0, 1:] * (1 - fr) + tab[k0 + 1, 1:] * fr
            q2 = q.clone()
            q2[:, self._arm[1]], q2[:, self._arm[2]], q2[:, self._arm[3]] = qq[:, 0], qq[:, 1], qq[:, 2]
            q2[:, self._arm[0]] = ang
            q2[:, self._arm[4]] = torch.atan2(torch.sin(lp - ang), torch.cos(lp - ang))  # 손목 롤 = 긴 축 - 팔 방향 (부호는 IK 가 바로잡음)
            q = torch.where(low[:, None], q2, q)
            r.write_joint_position_to_sim_index(position=q, env_ids=env_ids)
            self.arm_target[env_ids] = q[:, self._arm]
        self.phi[env_ids] = 0.0
        self.is_box[env_ids] = True
        self.perc_bias[env_ids] = U(-c.perc_bias, c.perc_bias, (n, 3))
        self.actions[env_ids] = 0.0
        self.actions[env_ids, -1] = 1.0
        self.prev_actions[env_ids] = self.actions[env_ids]
        self.hold[env_ids] = 0
        self.succeeded[env_ids] = False
        self.succ_once[env_ids] = False
        self.was_grasped[env_ids] = False
        self.prev_gd[env_ids] = 0.0
        self.max_lift[env_ids] = 0.0
        self.max_stand[env_ids] = 0.0
        self.off_steps[env_ids] = 0.0
        self.arm_box_steps[env_ids] = 0.0

    def _place_q(self, k, env_ids, xy, quat, z):
        o = self._obj(k)
        pose = o.data.default_root_pose.torch[env_ids].clone()
        pose[:, :2] = xy + self.scene.env_origins[env_ids, :2]
        pose[:, 2] = z + self.scene.env_origins[env_ids, 2]
        pose[:, 3:7] = quat
        o.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
        o.write_root_velocity_to_sim_index(root_velocity=torch.zeros(len(env_ids), 6, device=self.device), env_ids=env_ids)
