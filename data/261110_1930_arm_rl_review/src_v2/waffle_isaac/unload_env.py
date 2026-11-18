"""스토리지 (팔레트 12 칸) 에서 퍽 꺼내기 강화학습 (2026-11-10, 설정 unload_env_cfg.py).

45° 집기 환경 (sidepick_env) 의 행동 (잡는 점 직교 이동 + 집게) · IK · 쥠 / 듦 / 성공 판정 · 자기 충돌 · 팔-몸체 판정을 그대로 쓰고 리셋만 바꿈:
  - 팔 시작 = 꺼낼 칸 위 6 cm (tools_ik/unload_traj.py 경유점 0, 피치 60° / 45°) + 잡음
  - 목표 퍽 = 칸 바닥 (칸 모양 = 퍽 모양, 직육면체는 칸 방향), 옆 칸 퍽 (puckN, 닿으면 종료) + 다른 칸 퍽 최대 5 개 (obs0 ~ 4, 닿으면 벌점 · 종료)
  - 정책 관측 퍽 좌표 = 칸 설계 좌표 + 에피소드 치우침 (perc_bias, 실물 보정 오차 흉내)
성공 = 퍽을 쥔 채 success_lift (칸 벽 위 3 cm) 넘게 들고 세운 채 success_hold_s. 뒤 운반 (앞으로 돌리기 -> 놓기 자세) 은 각본 (unload_place_play.py).
"""

from __future__ import annotations

import json
import math
import os
from collections.abc import Sequence

import torch
from isaaclab.utils.math import sample_uniform

from .dice2_cfg import PUCK_H
from .place_env_cfg import N_OBS
from .sidepick_env import WaffleSidePickEnv

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class WaffleUnloadEnv(WaffleSidePickEnv):
    def __init__(self, cfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        dev = self.device
        tr = json.load(open(os.path.join(HERE, "unload_traj.json")))["pitches"][cfg.unload_pitch]
        slots = json.load(open(os.path.join(HERE, "pallet_place_traj.json")))["slots"]
        th = {f"{s['row']}-{s['index']}": math.radians(s["theta_deg"]) for s in slots}
        rows = [x for x in tr if x["ok"]]
        self._u_q0 = torch.tensor([x["q"][0] for x in rows], device=dev)  # 칸 위 6 cm
        self._u_xy = torch.tensor([[x["x"], x["y"]] for x in rows], device=dev)  # base
        self._u_fl = torch.tensor([x["floor_z"] for x in rows], device=dev)
        self._u_box = torch.tensor([x["shape"] == "box" for x in rows], device=dev)
        self._u_yaw = torch.tensor([th[x["slot"]] for x in rows], device=dev)
        allxy = torch.tensor([[s["x"], s["y"]] for s in slots], device=dev)
        self._all_xy = allxy
        self._all_fl = torch.tensor([s["floor_z"] for s in slots], device=dev)
        self._all_yaw = torch.tensor([math.radians(s["theta_deg"]) for s in slots], device=dev)
        # 칸 (꺼낼 수 있는 것) -> 전체 칸 번호, 가장 가까운 옆 칸
        d = torch.cdist(self._u_xy, allxy)
        self._u_self = d.argmin(dim=1)
        d2 = d.clone(); d2[torch.arange(len(rows)), self._u_self] = 9.0
        self._u_nb = d2.argmin(dim=1)
        self.slot_k = torch.zeros(self.num_envs, dtype=torch.long, device=dev)
        print(f"[Unload] 꺼내기 학습 칸 {len(rows)} 개 (피치 {cfg.unload_pitch}°)", flush=True)

    def _get_dones(self):
        end, to = super()._get_dones()
        self.term_obs = self._contact("arm_obs_contact")
        return end | self.term_obs, to

    def _get_rewards(self):
        return super()._get_rewards() + self.cfg.rew_obs_hit * self._contact("arm_obs_contact").float()

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        super()._reset_idx(env_ids)
        c, r, n, dev = self.cfg, self.robot, len(env_ids), self.device

        def U(lo, hi, shape):
            return sample_uniform(lo, hi, shape, dev)

        k = torch.randint(0, len(self._u_q0), (n,), device=dev)
        self.slot_k[env_ids] = k
        q = r.data.default_joint_pos.torch[env_ids].clone()
        q[:, self._arm] = torch.clamp(self._u_q0[k] + U(-1.0, 1.0, (n, 5)) * c.ready_noise, self._lo, self._hi)
        r.write_joint_position_to_sim_index(position=q, env_ids=env_ids)
        r.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), env_ids=env_ids)
        self.arm_target[env_ids] = q[:, self._arm]
        rp = r.data.default_root_pose.torch[env_ids, :3]  # 환경 원점 기준 로봇 루트 (자세 = 정면)

        def loc(xy_b, fl):
            return rp[:, :2] + xy_b, rp[:, 2] + fl + PUCK_H / 2 + 0.001

        # 목표 퍽 (칸 모양), 칸 안 틈 ±0.5 mm
        is_box = self._u_box[k]
        self.is_box[env_ids] = is_box
        xy, z = loc(self._u_xy[k] + U(-5e-4, 5e-4, (n, 2)), self._u_fl[k])
        park = torch.tensor([0.6, 0.7], device=dev).expand(n, 2)
        zp = torch.full((n,), 0.031, device=dev)
        self._place("puckT_box", env_ids, torch.where(is_box[:, None], xy, park), self._u_yaw[k], torch.where(is_box, z, zp))
        self._place("puckT_cyl", env_ids, torch.where(is_box[:, None], park + 0.1, xy), self._u_yaw[k], torch.where(is_box, zp, z))
        self.puck_xy0[env_ids] = xy + self.scene.env_origins[env_ids, :2]
        self.puck_z0[env_ids] = z + self.scene.env_origins[env_ids, 2]
        self.target_xy[env_ids] = xy
        # 옆 칸 퍽
        nb_on = torch.rand(n, device=dev) < c.unload_nb_prob
        j = self._u_nb[k]
        nxy, nz = loc(self._all_xy[j], self._all_fl[j])
        self._place("puckN", env_ids, torch.where(nb_on[:, None], nxy, park + torch.tensor([0.0, 0.2], device=dev)), self._all_yaw[j], torch.where(nb_on, nz, zp))
        self.nb_present[env_ids] = nb_on
        self.nb_prev[env_ids] = torch.where(nb_on[:, None], nxy, park + torch.tensor([0.0, 0.2], device=dev)) + self.scene.env_origins[env_ids, :2]
        # 다른 칸 퍽 (꽉 찬 팔레트 흉내): 남은 칸에서 무작위 N_OBS 개, 칸마다 unload_fill_prob
        sc = torch.rand(n, len(self._all_xy), device=dev)
        sc[torch.arange(n, device=dev), self._u_self[k]] = -1.0
        sc[torch.arange(n, device=dev), j] = -1.0
        pick = sc.argsort(dim=1, descending=True)[:, :N_OBS]
        for t in range(N_OBS):
            jj = pick[:, t]
            on = torch.rand(n, device=dev) < c.unload_fill_prob
            oxy, oz = loc(self._all_xy[jj], self._all_fl[jj])
            pk = torch.tensor([-0.5, 0.40 + 0.07 * t], device=dev).expand(n, 2)
            self._place(f"obs{t}", env_ids, torch.where(on[:, None], oxy, pk), self._all_yaw[jj], torch.where(on, oz, zp))
        # 상자 · L2 판 치움
        far = torch.tensor([-1.0, -1.0], device=dev).expand(n, 2)
        self._place("boxA", env_ids, far, torch.zeros(n, device=dev), torch.full((n,), -0.5, device=dev))
        self.perc_bias[env_ids] = U(-c.perc_bias, c.perc_bias, (n, 3))
