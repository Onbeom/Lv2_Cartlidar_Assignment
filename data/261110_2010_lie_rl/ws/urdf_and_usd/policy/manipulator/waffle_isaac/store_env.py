"""스토리지 넣기 `Waffle-Store-Direct` 구현. 설정과 설명은 store_env_cfg.py."""

from __future__ import annotations

import json
import math
import os
from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation, RigidObject
from isaaclab.envs import DirectRLEnv
from isaaclab.utils.math import quat_apply, quat_apply_inverse

from .dice2_cfg import GRASP_OFFSET_A5, PUCK_H
from .robot_cfg import ARM_JOINTS, EE_BODY, FINGER_HOLD_30, GRIPPER_JOINT, GRIPPER_MIMIC, ROOT_TO_BASE_LINK, WHEEL_REGEX, ee_axes

if TYPE_CHECKING:
    from .store_env_cfg import WaffleStoreEnvCfg

STARTS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "store_starts.json")



def tube_code_from(t, joint1_x):
    """목표 자리 (통 · 컵, (N, 3) 축 xy·입구 z) 를 관측 3 칸으로: joint1 축에서 본 방향 cos, 방향 각 / π, 거리 − 0.19 m.
    2026-10-30: 이전 원-핫 3 칸 (통 3 개) 은 컵 4 개를 못 담아 (cos, sin, 거리) 로 바꿈. ST-C4: sin -> 각 / π
    (바깥 컵 1·2 는 각 ∓3.0 rad 라 sin 이 ∓0.14 뿐 -> 둘을 헷갈려 관절 1 을 반대로 돌려 ±π 한계에 막힘. 각 / π = ∓0.95)."""
    dx, dy = t[:, 0] - joint1_x, t[:, 1]
    th = torch.atan2(dy, dx)
    return torch.stack((torch.cos(th), th / math.pi, torch.sqrt(dx * dx + dy * dy) - 0.19), dim=1)

OCC_PARK = (0.75, 0.65)  # 컵 속 장애물 퍽을 안 쓸 때 치워 두는 곳 (환경 원점 기준)


class WaffleStoreEnv(DirectRLEnv):
    cfg: WaffleStoreEnvCfg

    def __init__(self, cfg: WaffleStoreEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        r = self.robot
        self._arm, _ = r.find_joints(ARM_JOINTS, preserve_order=True)
        self._gl, _ = r.find_joints([GRIPPER_JOINT])
        self._gr, _ = r.find_joints([GRIPPER_MIMIC])
        self._wheels, _ = r.find_joints([WHEEL_REGEX])
        self._a5 = r.find_bodies(EE_BODY)[0][0]
        lim = r.data.soft_joint_pos_limits.torch[0, self._arm].clone()
        self._lo, self._hi = lim[:, 0], lim[:, 1]
        n, dev = self.num_envs, self.device
        S = json.load(open(STARTS, encoding="utf-8"))
        assert S["arm_joints"] == list(ARM_JOINTS)
        allr = list(S["samples"])
        hand = os.path.join(os.path.dirname(STARTS), "store_handover.json")  # 집기 모델이 실제로 끝난 상태 (chain_play --collect_handover)
        if os.path.exists(hand):
            allr += json.load(open(hand, encoding="utf-8"))["samples"]
        hand_b = os.path.join(os.path.dirname(STARTS), "store_handover_box.json")  # ST-D7 (2026-11-01): 누운 직육면체를 세운 뒤 상태 (퍽 가운데 쥠), 시작 종류 'HB'
        if os.path.exists(hand_b):
            allr += json.load(open(hand_b, encoding="utf-8"))["samples"]
        smp = [s for s in allr if s["cat"] in cfg.start_cats and (s["tube"] < 0 or s["tube"] in cfg.tubes_used)]
        self._sq = torch.tensor([s["q"] for s in smp], device=dev)
        self._spk = torch.tensor([s["puck_b"] for s in smp], device=dev)
        self._spq = torch.tensor([s["puck_quat_b"] for s in smp], device=dev)
        self._stube = torch.tensor([s["tube"] for s in smp], device=dev)
        cats = [s["cat"] for s in smp]
        cw = dict(zip(cfg.start_cats, cfg.cat_weight))
        cw = {k: v for k, v in cw.items() if k in cats}  # 없는 종류 (예: H 파일 없음) 는 빼고 나머지 비율로
        w = torch.tensor([cw[c] / cats.count(c) for c in cats], device=dev)
        print(f"[Store] 시작 상태 { {k: cats.count(k) for k in cw} } 비율 {cw}", flush=True)
        self._sw = w / w.sum()
        self._tubes = torch.tensor([[t["axis"][0], t["axis"][1], t["rim_z"]] for t in S["tubes"]], device=dev)
        self._seats = torch.tensor([t.get("seat_z", t["rim_z"] - 0.015) for t in S["tubes"]], device=dev)
        self._tube_names = [t["name"] for t in S["tubes"]]  # base_link 좌표
        self._bl = torch.tensor([[0.0, 0.0, ROOT_TO_BASE_LINK]], device=dev)
        self._off = torch.tensor([GRASP_OFFSET_A5], device=dev).repeat(n, 1)
        self._ex = torch.tensor([[1.0, 0.0, 0.0]], device=dev).repeat(n, 1)
        self._ey = torch.tensor([[0.0, 1.0, 0.0]], device=dev).repeat(n, 1)
        self._ez = torch.tensor([[0.0, 0.0, 1.0]], device=dev).repeat(n, 1)
        self.actions = torch.zeros(n, 6, device=dev)
        self.prev_actions = torch.zeros_like(self.actions)
        self.arm_target = r.data.default_joint_pos.torch[:, self._arm].clone()
        self.tube = torch.zeros(n, dtype=torch.long, device=dev)
        self.root0 = torch.zeros(n, 3, device=dev)
        self.ever_inserted = torch.zeros(n, dtype=torch.bool, device=dev)
        self.hold = torch.zeros(n, dtype=torch.long, device=dev)
        self.succeeded = torch.zeros(n, dtype=torch.bool, device=dev)
        self.ready_done = torch.zeros(n, dtype=torch.bool, device=dev)  # ST-D1: 넣은 뒤 다음 퍽 집기 자세로 돌아옴
        self.was_open = torch.zeros(n, dtype=torch.bool, device=dev)
        self._q_ready = torch.tensor([list(self.cfg.ready_pose)], device=dev)
        self.contact_steps = torch.zeros(n, device=dev)
        self.best_xy = torch.ones(n, device=dev)
        self.max_bottom = torch.zeros(n, device=dev)

    @property
    def robot(self) -> Articulation:
        return self.scene["robot"]

    @property
    def puck(self) -> RigidObject:
        return self.scene["puck"]

    # ---- 좌표 (base_link 좌표 = 루트 + ROOT_TO_BASE_LINK, 261028 부터 루트 = base_link) ----
    def to_bl(self, p_w):
        d = self.robot.data
        return quat_apply_inverse(d.root_link_quat_w.torch, p_w - d.root_link_pos_w.torch) - self._bl

    def vec_bl(self, v_w):
        return quat_apply_inverse(self.robot.data.root_link_quat_w.torch, v_w)

    def grasp_w(self):
        d = self.robot.data
        return d.body_link_pos_w.torch[:, self._a5] + quat_apply(d.body_link_quat_w.torch[:, self._a5], self._off)

    def a5_axes_w(self):
        """(닫는 축, −집게 방향) = 이전 arm_link_5 의 (x, z) 축 뜻 (robot_cfg.ee_axes)."""
        close, _, neg_appr = ee_axes(quat_apply, self.robot.data.body_link_quat_w.torch[:, self._a5], self._ex, self._ey, self._ez)
        return close, neg_appr

    def puck_axis_bl(self):
        """퍽 축 (위쪽 부호)."""
        u = self.vec_bl(quat_apply(self.puck.data.root_quat_w.torch, self._ez))
        return u * torch.where(u[:, 2:3] < 0, -1.0, 1.0)

    def tube_code(self):
        """목표 자리 (통 · 컵) 를 관측 3 칸으로 (`tube_code_from`)."""
        return tube_code_from(self._tubes[self.tube], self.cfg.joint1_x)

    def _yaw_err(self):
        """관절 1 이 목표 통 방향 각 (joint1 축에서 통 축으로, 새 팔은 joint1 각 = 팔 방향) 에서 얼마나 먼지 [rad].
        joint1 한계가 ±π 라 P 통 (+2.80) 은 + 쪽으로, N 통 (−2.80) 은 − 쪽으로 돌아야 닿음 (반대로 돌면 ±π 한계에 막힘)."""
        t = self._tubes[self.tube]
        th = torch.atan2(t[:, 1], t[:, 0] - self.cfg.joint1_x)
        return (self.robot.data.joint_pos.torch[:, self._arm[0]] - th).abs()

    def base_tilt(self):
        g = self.robot.data.projected_gravity_b.torch
        return torch.acos((-g[:, 2]).clamp(-1.0, 1.0))

    def _force(self, k):
        f = self.scene[k].data.normal_force_matrix_w.torch  # (N, 몸, 상대, 3)
        return torch.linalg.norm(f, dim=-1).amax(dim=(1, 2))

    def _state(self):
        c = self.cfg
        pc = self.to_bl(self.puck.data.root_pos_w.torch)
        u = self.puck_axis_bl()
        pb = pc - u * (PUCK_H / 2)  # 퍽 바닥 중심
        t = self._tubes[self.tube]
        dxy = torch.linalg.norm(pb[:, :2] - t[:, :2], dim=1)
        depth = t[:, 2] - pb[:, 2]  # + = 입구 아래
        gb = self.to_bl(self.grasp_w())
        fing0 = self.robot.data.joint_pos.torch[:, self._gl[0]]
        # ST-D2: 손가락이 닫혀 있어야 쥔 것 (ST-D1 은 거리만 봐서, 컵 옆 판 위에 퍽을 내려놓고 집게를 연 채 머물러도 '쥠' -> 보상을 계속 받음)
        held = (torch.linalg.norm(pc - gb, dim=1) < c.held_dist) & (fing0 < c.released_finger)
        fing = self.robot.data.joint_pos.torch[:, self._gl[0]]
        inside = (dxy < c.inside_xy) & (depth > c.inserted_depth)
        stored = (dxy < c.inside_xy) & (depth > c.stored_depth) & (u[:, 2] > 0.95) & (fing > c.released_finger) & ~held
        return dict(pc=pc, u=u, pb=pb, t=t, dxy=dxy, depth=depth, gb=gb, held=held, fing=fing, inside=inside, stored=stored)

    # ---- 행동 ----
    def _pre_physics_step(self, actions):
        self.prev_actions = self.actions.clone()
        self.actions = actions.clamp(-1.0, 1.0)
        q = self.robot.data.joint_pos.torch[:, self._arm]
        lead = self.cfg.target_lead_max
        s = self._state()
        fine = (s["dxy"] < self.cfg.fine_xy) & (-s["depth"] < self.cfg.zone_clear)
        scale = self.cfg.action_scale * torch.where(fine, self.cfg.fine_scale, 1.0).unsqueeze(1)
        t = torch.clamp(self.arm_target + scale * self.actions[:, :5], q - lead, q + lead)
        self.arm_target = torch.clamp(t, self._lo, self._hi)
        if self.cfg.lock_roll:  # 손목 롤 (관절 5) 0 고정: 집게 닫는 축 수평 = 퍽 세움 (261028 ST-N5)
            self.arm_target[:, 4] = 0.0

    def _apply_action(self):
        cmd = self.robot.actuators.target_command
        g = torch.where(self.actions[:, 5:6] > 0.0, self.cfg.gripper_open, self.cfg.gripper_closed)
        cmd.set_position_index(value=self.arm_target.contiguous(), joint_ids=self._arm)
        cmd.set_position_index(value=g.expand(-1, 2).contiguous(), joint_ids=self._gl + self._gr)  # 두 손가락 같은 값
        cmd.set_velocity_index(value=torch.zeros(self.num_envs, len(self._wheels), device=self.device), joint_ids=self._wheels)

    # ---- 관측·보상·종료 ----
    def _get_observations(self):
        d = self.robot.data
        s = self._state()
        ax, az = self.a5_axes_w()
        obs = torch.cat((
            d.joint_pos.torch[:, self._arm] - d.default_joint_pos.torch[:, self._arm], d.joint_vel.torch[:, self._arm],
            d.joint_pos.torch[:, self._gl], s["gb"], s["pc"] - s["gb"], s["u"], s["t"], s["pb"] - s["t"],
            self.vec_bl(-az), self.vec_bl(ax), self.tube_code(), self.actions,
        ), dim=-1)
        return {"policy": obs}

    def _get_rewards(self):
        c = self.cfg
        s = self._state()
        dxy, depth, held, u, fing = s["dxy"], s["depth"], s["held"], s["u"], s["fing"]
        up = u[:, 2]
        self.ever_inserted |= s["inside"]
        pre = held & ~self.ever_inserted  # 넣기 전 쥐고 옮기는 중
        near_xy = dxy < c.inside_xy
        # 축에 맞고 입구 근처 (입구 위 12 mm 아래) 일 때만 높이·hover 보상을 다 줌. ST-C3 (2026-10-30): 이전엔 축만 맞으면 높이 상관없이 다 줘서
        # C1·C2 가 입구 위 21 cm 에서 축만 맞추고 멈춤 (통 시절엔 통 안으로 내려갈 때 깎이지 않게 하려던 것)
        near = near_xy & (-depth < c.clear_margin)
        # 높이 들기: 퍽 바닥이 (입구 − 30 cm) -> 입구 + 12 mm, 축에 맞으면 다 받은 것으로 (내려 넣을 때 깎이지 않게)
        clear = -depth  # 퍽 바닥 − 입구
        lift = torch.where(near, torch.ones_like(dxy), torch.clamp((clear + 0.30) / (0.30 + c.clear_margin), 0.0, 1.0))
        # 다가가기: 높이 들었을 때만 (입구 아래 10 cm 부터 켜짐)
        high = torch.where(near, torch.ones_like(dxy), torch.clamp((clear + c.high_band) / c.high_band, 0.0, 1.0))
        approach = high * (c.rew_approach * (1 - torch.tanh(dxy / 0.15)) + c.rew_approach_mid * (1 - torch.tanh(dxy / 0.03))
                           + c.rew_approach_fine * (1 - torch.tanh(dxy / 0.008)) + c.rew_precise * (1 - torch.tanh(dxy / 0.002)))
        # 통 5 cm 안: 퍽 바닥을 입구 + 8 mm 로 (넣기 직전 높이), 축에 맞으면 다 받은 것으로
        hover = torch.where(near, torch.ones_like(dxy), (1 - torch.tanh((clear - c.hover_clear).abs() / 0.01)) * (dxy < 0.05).float())
        hover_c = torch.where(near, torch.ones_like(dxy), torch.clamp(1 - (clear - c.hover_clear).abs() / 0.25, min=0.0) * (dxy < 0.08).float())
        # ST-C4 원뿔 높이: 목표 퍽 바닥 높이 = 입구 + cone_base + cone_slope x (축 거리 − 1 cm), 최대 cone_max. 축에 가까울수록 낮게
        # (C3 까지 높이 보상은 입구 + 12 mm 에서 다 차서 높이 떠도 손해가 없었음 -> 입구 위 23 cm 에 머묾)
        h_goal = c.hover_clear + torch.clamp(c.cone_slope * (dxy - 0.01), 0.0, c.cone_max)
        h_err = (clear - h_goal).abs()
        cone = torch.clamp(1 - h_err / 0.30, min=0.0) + (1 - torch.tanh(h_err / 0.02))
        aligned = ((dxy < c.align_xy) & (up > c.upright_min) & (clear < c.zone_clear)).float()  # ST-C2: 입구 위 3 cm 안 (C1 은 21 cm 위도 셈)
        insert = (1 - torch.tanh(dxy / 0.003)) * (up > c.upright_min).float() * torch.clamp((depth + 0.01) / 0.04, 0.0, 1.0)
        released = s["inside"] & (fing > c.released_finger)
        retract = (self.ever_inserted & ~held).float() * torch.clamp(torch.linalg.norm(s["gb"] - s["t"], dim=1) / 0.06, max=1.0)
        f_arm = self._force("arm_body_contact")
        f_fing = self._force("finger_body_contact")
        f_puck = self._force("puck_body_contact")
        zone = (dxy < c.zone_xy) & (clear < c.zone_clear)  # 넣는 자리: 손가락이 통 위에 닿는 건 약하게
        fing_c = f_fing > c.contact_force
        body = (f_arm > c.contact_force) | (fing_c & ~zone)
        fing_zone = fing_c & zone
        crash = (f_arm > c.crash_force) | ((f_fing > c.crash_force) & ~zone)
        puck_body = (f_puck > c.contact_force) & ~(near_xy & (depth > -0.003))
        dropped = ~held & ~self.ever_inserted
        # ST-D7: 관절 1 ~ 4 가 한계 joint_margin 안이면 (가까울수록) 약한 음 - 누운 퍽을 세운 뒤 컵 위에서 팔꿈치 (관절 3) 상한에 걸려 손 IK 가 못 내리던 것
        qj = self.robot.data.joint_pos.torch[:, self._arm[:4]]
        dlim = torch.minimum(qj - self._lo[:4], self._hi[:4] - qj)
        jm = torch.clamp((c.joint_margin - dlim) / c.joint_margin, 0.0, 1.0).sum(dim=1)
        # ST-D1 (2026-10-30 사용자 요청) ① 쓸데없이 팔을 높게·멀리 펴서 옮기면 약한 음의 보상: 퍽 바닥이 원뿔 목표 + 3 cm 넘게 높음, 잡는 점이 joint1 축에서 reach_max 넘게 멂
        too_high = torch.clamp((clear - h_goal - 0.03) / 0.10, 0.0, 3.0)
        gb = s["gb"]
        reach = torch.sqrt((gb[:, 0] - c.joint1_x) ** 2 + gb[:, 1] ** 2)
        too_far = torch.clamp((reach - c.reach_max) / 0.10, 0.0, 3.0)
        # ② 컵에 놓을 때 세로가 아니면 강한 음의 보상: 넣는 자리 (축 8 mm·입구 위 3 cm 안) 에서 퍽 기울기 > place_tilt_deg, 그 자리에서 집게를 연 순간 기울었으면 한 번 더
        tilt_bad = up < math.cos(math.radians(c.place_tilt_deg))
        at_place = (dxy < c.zone_xy) & (clear < c.zone_clear)
        opening = (self.actions[:, 5] > 0.0) & ~self.was_open
        self.was_open = self.actions[:, 5] > 0.0
        place_tilt = (at_place & tilt_bad).float()
        release_tilt = (at_place & tilt_bad & opening & held).float()
        # ③ 넣은 뒤 이어서 다음 퍽을 집을 자세 (정렬·집기 시작 자세) 로 돌아오면 강한 양의 보상
        q_err = torch.linalg.norm(self.robot.data.joint_pos.torch[:, self._arm[:4]] - self._q_ready[:, :4], dim=1)
        after = self.succeeded & ~held
        back = after.float() * (1 - torch.tanh(q_err / 0.5))
        ready_now = after & (q_err < c.ready_tol) & ~self.ready_done
        self.ready_done |= ready_now
        base_move = torch.linalg.norm(self.robot.data.root_link_pos_w.torch[:, :2] - self.root0[:, :2], dim=1) > c.base_move_max
        f_occ = torch.maximum(self._force("robot_occ_contact"), self._force("puck_occ_contact"))  # ST-D8
        rew = (
            pre.float() * (c.rew_hold + c.rew_lift * lift + approach + c.rew_hover * hover + c.rew_hover_coarse * hover_c + c.rew_cone * cone
                           + c.rew_upright * (1 - torch.tanh(torch.clamp((1 - up) - c.upright_dead, min=0.0) / c.upright_scale))
                           + c.rew_level * (1 - torch.tanh(torch.clamp(self.vec_bl(-self.a5_axes_w()[1])[:, 2].abs() - c.level_dead, min=0.0) / 0.03))
                           + c.rew_yaw * (1 - self._yaw_err() / math.pi)
                           + c.rew_yaw_fine * (1 - torch.tanh(self._yaw_err() / 0.1)) + c.rew_yaw_wrong * (self._yaw_err() > math.pi / 2).float())
            + c.rew_insert * insert * held.float() + c.rew_inserted * s["inside"].float() + c.rew_released * released.float()
            + c.rew_stored * s["stored"].float() + c.rew_retract * retract
            + c.rew_puck_tilt * (1 - up) * held.float()
            + c.rew_body_contact * body.float() + c.rew_finger_zone * fing_zone.float() + c.rew_body_crash * crash.float() + c.rew_puck_body * puck_body.float()
            + c.rew_drop * dropped.float() + c.rew_base_move * base_move.float()
            + held.float() * c.rew_joint_margin * jm
            + c.rew_occ_hit * (f_occ > c.contact_force).float() + 0.5 * c.rew_body_crash * (f_occ > c.occ_crash).float()
            + held.float() * (c.rew_too_high * too_high + c.rew_too_far * too_far)
            + c.rew_place_tilt * place_tilt + c.rew_release_tilt * release_tilt
            + c.rew_back * back + c.rew_next_ready * ready_now.float()
            + c.rew_action_rate * torch.sum(torch.square(self.actions - self.prev_actions), dim=1)
            + c.rew_joint_vel * torch.sum(torch.square(self.robot.data.joint_vel.torch[:, self._arm]), dim=1)
            + c.rew_base_tilt * self.base_tilt()
        )
        self.hold = torch.where(s["stored"], self.hold + 1, torch.zeros_like(self.hold))
        self.succeeded |= self.hold >= int(c.success_hold_s / (c.sim.dt * c.decimation))
        self.contact_steps += body.float()
        self.best_xy = torch.minimum(self.best_xy, torch.where(held, dxy, self.best_xy))
        self.max_bottom = torch.maximum(self.max_bottom, torch.where(held, clear, self.max_bottom))
        log = self.extras.setdefault("log", {})
        log["Metrics/xy_to_tube_mm"] = dxy.mean().item() * 1000
        log["Metrics/bottom_above_rim_mm"] = clear.mean().item() * 1000
        log["Metrics/held_ratio"] = held.float().mean().item()
        log["Metrics/aligned_ratio"] = aligned.mean().item()
        log["Metrics/inserted_ratio"] = s["inside"].float().mean().item()
        log["Metrics/released_ratio"] = released.float().mean().item()
        log["Metrics/stored_ratio"] = s["stored"].float().mean().item()
        log["Metrics/joint_margin_pen"] = jm.mean().item()
        log["Metrics/occ_hit_ratio"] = (f_occ > c.contact_force).float().mean().item()
        log["Metrics/too_high_mean"] = (too_high * held.float()).mean().item()
        log["Metrics/reach_m"] = reach.mean().item()
        log["Metrics/place_tilt_ratio"] = place_tilt.mean().item()
        log["Metrics/next_ready_ratio"] = self.ready_done.float().mean().item()
        log["Metrics/body_contact_ratio"] = body.float().mean().item()
        log["Metrics/puck_body_contact_ratio"] = puck_body.float().mean().item()
        log["Metrics/finger_zone_contact_ratio"] = fing_zone.float().mean().item()
        log["Metrics/puck_upright"] = up.mean().item()
        return rew

    def _get_dones(self):
        s = self._state()
        tipped = self.robot.data.projected_gravity_b.torch[:, 2] > -0.6
        dropped = ~s["held"] & ~self.ever_inserted & ~s["inside"]
        zone = (s["dxy"] < self.cfg.zone_xy) & (-s["depth"] < self.cfg.zone_clear)
        crash = (self._force("arm_body_contact") > self.cfg.crash_force) | ((self._force("finger_body_contact") > self.cfg.crash_force) & ~zone)
        crash = crash | (torch.maximum(self._force("robot_occ_contact"), self._force("puck_occ_contact")) > self.cfg.occ_crash)  # ST-D8
        base_move = torch.linalg.norm(self.robot.data.root_link_pos_w.torch[:, :2] - self.root0[:, :2], dim=1) > self.cfg.base_move_max
        z = torch.zeros_like(self.tube)
        self.term_reason = torch.where(dropped, 1, torch.where(crash, 2, torch.where(base_move, 3, torch.where(tipped, 4, z))))  # 진단용
        self.term_arm_f = self._force("arm_body_contact")
        self.term_fing_f = self._force("finger_body_contact")
        return tipped | dropped | crash | base_move, self.episode_length_buf >= self.max_episode_length - 1

    # ---- 초기화 ----
    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        if len(env_ids) > 0:
            log = self.extras.setdefault("log", {})
            log["Metrics/success_rate"] = self.succeeded[env_ids].float().mean().item()
            for i, nm in enumerate(self._tube_names):
                m = self.tube[env_ids] == i
                if m.any():
                    log[f"Metrics/success_{nm}"] = self.succeeded[env_ids][m].float().mean().item()
            log["Metrics/ever_inserted"] = self.ever_inserted[env_ids].float().mean().item()
            log["Metrics/contact_steps"] = self.contact_steps[env_ids].mean().item()
            log["Metrics/best_xy_mm"] = self.best_xy[env_ids].mean().item() * 1000
            log["Metrics/max_bottom_above_rim_mm"] = self.max_bottom[env_ids].mean().item() * 1000
        super()._reset_idx(env_ids)
        c, r, n, dev = self.cfg, self.robot, len(env_ids), self.device
        org = self.scene.env_origins[env_ids]
        root = r.data.default_root_pose.torch[env_ids].clone()
        root[:, :3] += org
        r.write_root_pose_to_sim_index(root_pose=root, env_ids=env_ids)
        r.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=env_ids)
        self.root0[env_ids] = root[:, :3]
        k = torch.multinomial(self._sw, n, replacement=True)
        q = r.data.default_joint_pos.torch[env_ids].clone()
        q[:, self._arm] = torch.clamp(self._sq[k], self._lo, self._hi)
        q[:, self._gl[0]] = FINGER_HOLD_30
        q[:, self._gr[0]] = FINGER_HOLD_30
        r.write_joint_position_to_sim_index(position=q, env_ids=env_ids)
        r.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), env_ids=env_ids)
        self.arm_target[env_ids] = q[:, self._arm]
        st = self._stube[k]
        used = torch.tensor(c.tubes_used, device=dev)
        self.tube[env_ids] = torch.where(st < 0, used[torch.randint(0, len(c.tubes_used), (n,), device=dev)], st)
        # 쥔 퍽 (루트 요 0 -> base_link 좌표 = 월드 방향)
        pose = torch.cat((root[:, :3] + self._bl + self._spk[k], self._spq[k]), dim=1)
        self.puck.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
        self.puck.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=env_ids)
        # ST-D8: 나머지 바깥 컵에 퍽을 occ_prob 로 미리 넣어 둠 (장애물), 아니면 멀리 치움
        nt = self._tubes.shape[0]
        others = torch.stack([torch.tensor([cc for cc in range(nt) if cc != t] + [0] * max(0, 4 - nt), device=dev)[:3] for t in range(nt)])  # (nt, 3)
        oth = others[self.tube[env_ids]]
        for j in range(3):
            ob = self.scene[f"occ{j}"]
            cj = oth[:, j]
            # 시작 때 쥔 퍽·손가락이 그 컵 퍽과 겹치거나 옆에 낮게 있으면 그 컵은 비움 (C 시작 = 컵 옆 2 ~ 3 cm·입구 바로 위 -> 생성 순간 겹침, ST-D8·D9 오염 원인)
            pk = self._spk[k][:, :3]
            near = (torch.linalg.norm(pk[:, :2] - self._tubes[cj, :2], dim=1) < self.cfg.occ_free_xy) & (pk[:, 2] - PUCK_H / 2 < self._tubes[cj, 2] + self.cfg.occ_free_z)
            on = (torch.rand(n, device=dev) < self.cfg.occ_prob) & ~near
            pb_ = torch.stack((self._tubes[cj, 0], self._tubes[cj, 1], self._seats[cj] + PUCK_H / 2 + 0.001), dim=1)
            # 치우는 곳 = 네 환경 사이 모서리 (간격 1.5 m 라 (0, 2.0) 은 이웃 환경 로봇 옆이었음 -> ST-D8 학습 오염, 2026-11-01 수정)
            park = torch.tensor([[OCC_PARK[0], OCC_PARK[1] + 0.1 * j, 0.031]], device=dev).expand(n, 3)
            pos = torch.where(on[:, None], root[:, :3] + self._bl + pb_, park + self.scene.env_origins[env_ids])
            ob.write_root_pose_to_sim_index(root_pose=torch.cat((pos, ob.data.default_root_pose.torch[env_ids, 3:7]), dim=1), env_ids=env_ids)
            ob.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=env_ids)
        self.actions[env_ids] = 0.0
        self.actions[env_ids, 5] = -1.0  # 쥔 채로 시작
        self.prev_actions[env_ids] = self.actions[env_ids]
        self.ever_inserted[env_ids] = False
        self.hold[env_ids] = 0
        self.succeeded[env_ids] = False
        self.ready_done[env_ids] = False
        self.was_open[env_ids] = False
        self.contact_steps[env_ids] = 0.0
        self.best_xy[env_ids] = 1.0
        self.max_bottom[env_ids] = -1.0
