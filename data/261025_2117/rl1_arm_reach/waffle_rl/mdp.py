"""RL1 전용 MDP 실행 항목: 속도 제한 관절 위치 행동, 목표 표 명령, 팔 자기충돌 접촉.

설정 클래스는 mdp_cfg.py에 있다(이 모듈은 Kit가 뜬 뒤에만 불러온다).
"""
from __future__ import annotations

import json
import numpy as np
import torch

from isaaclab.envs.mdp.actions.joint_actions import JointPositionAction
from isaaclab.managers import CommandTerm, SceneEntityCfg
from isaaclab.utils.math import quat_apply, quat_apply_inverse

from .mdp_cfg import ARM_BODIES  # noqa: F401


# ---------- 행동 ----------
class RateLimitedJointPositionAction(JointPositionAction):
    """행동 [-1, 1]을 관절 범위 [lower, upper]로 옮겨 위치 목표로 쓰되, 물리 스텝마다 목표 변화를 max_rate·dt로 제한한다.

    v12 물리 시험의 제어기 속도 한계(위치 목표를 한 스텝에 한계 × dt까지만 바꿈)와 같은 방식이다.
    apply_actions는 물리 스텝(decimation)마다 불린다.
    """


    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        lim = self._asset.data.joint_pos_limits.torch[:, self._joint_ids].clone()
        self._lo, self._hi = lim[..., 0], lim[..., 1]
        self._step = cfg.max_rate * env.physics_dt
        self._target = self._asset.data.joint_pos.torch[:, self._joint_ids].clone()

    def process_actions(self, actions: torch.Tensor):
        self._raw_actions[:] = actions
        a = actions.clamp(-1.0, 1.0)
        self._processed_actions[:] = self._lo + (a + 1.0) * 0.5 * (self._hi - self._lo)

    def apply_actions(self):
        delta = (self._processed_actions - self._target).clamp(-self._step, self._step)
        self._target += delta
        self._asset.set_joint_position_target_index(target=self._target, joint_ids=self._joint_ids)

    def reset(self, env_ids=None):
        super().reset(env_ids)
        ids = slice(None) if env_ids is None else env_ids
        self._target[ids] = self._asset.data.joint_pos.torch[ids][:, self._joint_ids]

    @property
    def applied_target(self) -> torch.Tensor:
        return self._target


class VelocityCommandJointAction(JointPositionAction):
    """설계 v2: 행동 [-1, 1] × max_rate = 관절 속도 명령(rad/s).

    물리 스텝마다 위치 목표에 속도 × dt를 더하고(관절 범위 안으로 제한), 같은 속도를 drive 속도 목표로도 넣는다.
    implicit drive 토크는 kp·(목표 위치 − 위치) + kd·(목표 속도 − 속도)라서, 속도 목표가 0이면(v1) 관절이
    3.8 / kd 36 = 0.11 rad/s만 넘어도 토크가 포화된다. 범위 끝에 닿은 관절은 속도 목표를 0으로 둔다.
    위치 목표 변화가 한 스텝에 max_rate·dt를 넘지 않으므로 v12 제어기 속도 한계와 같은 상한을 지킨다.
    """

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        lim = self._asset.data.joint_pos_limits.torch[:, self._joint_ids].clone()
        self._lo, self._hi = lim[..., 0], lim[..., 1]
        self._dt = env.physics_dt
        self._target = self._asset.data.joint_pos.torch[:, self._joint_ids].clone()
        self._vel = torch.zeros_like(self._target)

    def process_actions(self, actions: torch.Tensor):
        self._raw_actions[:] = actions
        self._processed_actions[:] = actions.clamp(-1.0, 1.0) * self.cfg.max_rate

    def apply_actions(self):
        new = (self._target + self._processed_actions * self._dt).clamp(self._lo, self._hi)
        self._vel[:] = (new - self._target) / self._dt
        self._target[:] = new
        self._asset.set_joint_position_target_index(target=self._target, joint_ids=self._joint_ids)
        self._asset.set_joint_velocity_target_index(target=self._vel, joint_ids=self._joint_ids)

    def reset(self, env_ids=None):
        super().reset(env_ids)
        ids = slice(None) if env_ids is None else env_ids
        self._target[ids] = self._asset.data.joint_pos.torch[ids][:, self._joint_ids]
        self._vel[ids] = 0.0

    @property
    def applied_target(self) -> torch.Tensor:
        return self._target



class RateLimitedFeedforwardJointPositionAction(RateLimitedJointPositionAction):
    """설계 v3 S3: v1과 같은 절대 위치 행동(행동 [-1, 1] → 관절 범위, 목표 변화 ≤ max_rate·dt)에
    목표가 움직인 속도를 drive 속도 목표로 넣는다(v1은 속도 목표 0이라 0.11 rad/s만 넘어도 포화)."""

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self._vel = torch.zeros_like(self._target)
        self._dt = env.physics_dt

    def apply_actions(self):
        delta = (self._processed_actions - self._target).clamp(-self._step, self._step)
        self._target += delta
        self._vel[:] = delta / self._dt
        self._asset.set_joint_position_target_index(target=self._target, joint_ids=self._joint_ids)
        self._asset.set_joint_velocity_target_index(target=self._vel, joint_ids=self._joint_ids)

    def reset(self, env_ids=None):
        super().reset(env_ids)
        ids = slice(None) if env_ids is None else env_ids
        self._vel[ids] = 0.0


class FilteredFeedforwardJointPositionAction(RateLimitedFeedforwardJointPositionAction):
    """설계 v4: S3 행동의 원하는 목표를 정책 스텝마다 지수 이동 평균(EMA)으로 거른 뒤 속도 제한·피드포워드.

    v3 S3는 정책 출력이 스텝마다 흔들려 목표가 최대 속도로 오락가락했다(포화 79 %, 정지 RMS 1.1 rad/s).
    filtered = alpha × 원하는 목표 + (1 − alpha) × 직전 filtered. 리셋 때는 현재 관절 위치로 맞춘다.
    """

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self._filtered = self._target.clone()

    def process_actions(self, actions: torch.Tensor):
        self._raw_actions[:] = actions
        desired = self._lo + (actions.clamp(-1.0, 1.0) + 1.0) * 0.5 * (self._hi - self._lo)
        self._filtered[:] = self.cfg.alpha * desired + (1.0 - self.cfg.alpha) * self._filtered
        self._processed_actions[:] = self._filtered

    def reset(self, env_ids=None):
        super().reset(env_ids)
        ids = slice(None) if env_ids is None else env_ids
        self._filtered[ids] = self._target[ids]


class AccelLimitedJointPositionAction(FilteredFeedforwardJointPositionAction):
    """설계 v6: v4 행동(EMA로 거른 원하는 목표)을 물리 스텝마다 가속도·속도 제한 궤적으로 따라간다.

    v5 진단(계획 문서 14장): 목표가 정책 스텝 첫 물리 스텝에만 움직여 속도 목표가 펄스로 걸리고(포화 순간 kd 항 중앙값
    51 N·m), 목표 속도가 즉시 max_rate가 되어 위치 목표가 실제 관절보다 최대 0.34 rad 앞서 나가 실제 속도가 5.8 rad/s까지
    넘쳤다. 여기서는 궤적 속도 v를 물리 스텝마다 ±max_acc·dt까지만 바꾸고, 남은 거리 e에서 멈출 수 있는 속도
    sqrt(2·max_acc·|e|)와 max_rate를 넘지 않게 한다. 위치 목표 += v·dt, 속도 목표 = v.
    """

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self._dv = cfg.max_acc * env.physics_dt

    def apply_actions(self):
        e = self._processed_actions - self._target
        v_ref = torch.sign(e) * torch.minimum(torch.sqrt(2.0 * self.cfg.max_acc * e.abs()), e.abs() / self._dt)
        v_ref = v_ref.clamp(-self.cfg.max_rate, self.cfg.max_rate)
        v = self._vel + (v_ref - self._vel).clamp(-self._dv, self._dv)
        new = (self._target + v * self._dt).clamp(self._lo, self._hi)
        self._vel[:] = (new - self._target) / self._dt
        # hold_current: 위치 목표 = 이번 물리 스텝 시작 시점의 궤적 위치(한 스텝 늦춤). 앞당긴 목표(new)는 따라가기만
        # 해도 kp·v·dt(= 3.33·v N·m)의 계산 토크가 생긴다(T6). 속도 목표는 같은 v.
        q_cmd = self._target.clone() if self.cfg.hold_current else new
        self._target[:] = new
        self._asset.set_joint_position_target_index(target=q_cmd, joint_ids=self._joint_ids)
        self._asset.set_joint_velocity_target_index(target=self._vel, joint_ids=self._joint_ids)


class DeltaAccelLimitedJointPositionAction(AccelLimitedJointPositionAction):
    """설계 v7: 행동 = 원하는 목표의 변화량. desired += clamp(a, −1, 1) × delta_max (정책 스텝마다), 그 뒤는 v6 궤적.

    v6 진단(계획 문서 15장): 절대 위치 행동은 출력 0.01이 관절 약 0.025 rad(TCP 약 6 mm)라, 목표 근처에서 출력이 스텝마다
    ±0.3씩 흔들려 정지 구간에서도 계속 움직였다(J5 1000구간 모두 실패). 변화량 행동은 멈추기 = 출력 0이다.
    원하는 목표는 관절 범위 안, 실제 관절에서 ±window 안으로 제한한다(부딪혀 막혔을 때 목표가 멀리 달아나지 않게).
    """

    def process_actions(self, actions: torch.Tensor):
        self._raw_actions[:] = actions
        q = self._asset.data.joint_pos.torch[:, self._joint_ids]
        d = self._filtered + actions.clamp(-1.0, 1.0) * self.cfg.delta_max
        d = torch.maximum(torch.minimum(d, q + self.cfg.window), q - self.cfg.window)
        self._filtered[:] = torch.maximum(torch.minimum(d, self._hi), self._lo)
        self._processed_actions[:] = self._filtered

    @property
    def desired(self) -> torch.Tensor:
        return self._filtered


class RelativeAccelLimitedJointPositionAction(AccelLimitedJointPositionAction):
    """설계 v8: 행동 = 지금 관절 기준 상대 목표. desired = clamp(q + clamp(a, −1, 1) × scale, 범위), 그 뒤는 v6 궤적.

    v6(절대 위치)은 출력 0.01이 관절 약 0.025 rad라 목표 근처에서 떨렸고, v7(목표 변화량을 쌓음)은 학습이 105 mm에서
    멈췄다(계획 문서 16장). 상대 목표는 정책 스텝마다 '지금에서 어디로'를 바로 주면서 출력 0.01 = scale × 0.01 rad.
    desired 속성은 관찰용으로 궤적 위치 목표(실제 관절이 따라가는 값)를 돌려준다.
    """

    def process_actions(self, actions: torch.Tensor):
        self._raw_actions[:] = actions
        q = self._asset.data.joint_pos.torch[:, self._joint_ids]
        d = q + actions.clamp(-1.0, 1.0) * self.cfg.scale_rad
        self._filtered[:] = torch.maximum(torch.minimum(d, self._hi), self._lo)
        self._processed_actions[:] = self._filtered

    @property
    def desired(self) -> torch.Tensor:
        return self._target


class RelativeCmdAccelLimitedJointPositionAction(RelativeAccelLimitedJointPositionAction):
    """설계 v9: v8 상대 목표의 기준을 측정 관절이 아니라 지금 명령 중인 궤적 위치 목표로 바꾼다.

    v8 진단(계획 문서 17장): 기준이 측정 관절이면 출력 0일 때 목표 = 처진 위치라 중력 처짐(약 0.0035 rad)만큼 매 스텝
    미끄러져, 정책이 정지 구간에서도 ±0.17을 번갈아 내며 버텼다(J5 998구간 실패). 명령 목표 기준이면 출력 0 = 그 자리 유지
    (모터 목표 위치 레지스터와 같음). 원하는 목표는 관절 범위 안, 실제 관절 ±window 안으로 제한한다.
    """

    def process_actions(self, actions: torch.Tensor):
        self._raw_actions[:] = actions
        q = self._asset.data.joint_pos.torch[:, self._joint_ids]
        d = self._target + actions.clamp(-1.0, 1.0) * self.cfg.scale_rad
        d = torch.maximum(torch.minimum(d, q + self.cfg.window), q - self.cfg.window)
        self._filtered[:] = torch.maximum(torch.minimum(d, self._hi), self._lo)
        self._processed_actions[:] = self._filtered


class DeadbandRelativeCmdJointPositionAction(RelativeCmdAccelLimitedJointPositionAction):
    """설계 v11: v9 행동(명령 목표 기준 상대 목표)에 불감대. a' = sign(a)·max(|a| − db, 0)/(1 − db)(연속).

    v8~v10 정지 떨림: 궤적 생성기만으로는 멈추지만(계획 문서 19장), 상대 목표는 적분기라 위치 유지를 정책 혼자 맡고
    고이득 정책과 30 Hz 지연이 작은 진동을 만든다. 명령 목표 기준 + 불감대면 출력이 0 근처일 때 그대로 멈춘다.
    """

    def process_actions(self, actions: torch.Tensor):
        a = actions.clamp(-1.0, 1.0)
        db = self.cfg.deadband
        a = torch.sign(a) * (a.abs() - db).clamp(min=0.0) / (1.0 - db)
        super().process_actions(a)
        self._raw_actions[:] = actions


class HoldGateRelativeJointPositionAction(RelativeAccelLimitedJointPositionAction):
    """설계 v14: v8 상대 목표 + 유지 게이트(규칙). TCP 오차가 hold_r_in 안에 들어오면 원하는 목표를 지금 명령 중인 궤적
    위치로 고정해 정책 출력을 무시하고, hold_r_out 밖으로 나가면(새 목표 등) 다시 정책 출력을 따른다(히스테리시스).

    v5~v13: 정밀 보상을 넣은 뒤로 모든 설계가 목표 유지 중 정책 출력 부호를 매 스텝 바꾸는 떨림(J5 중앙 0.16~0.34)을
    보였고, 행동 변화 벌점·불감대·정밀 보상 꼴·출력 EMA로는 줄지 않았다(계획 문서 19~22장). 정책 출력 0이 유지인 행동이라
    고정은 궤적 생성기만으로 멈춘다(T6 재계산 0.001 rad/s).
    """

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self._tcp_cfg = SceneEntityCfg(cfg.asset_name, body_names=[cfg.tcp_body])
        self._tcp_cfg.resolve(env.scene)
        self._hold = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)

    def process_actions(self, actions: torch.Tensor):
        super().process_actions(actions)
        e = tcp_position_error(self._env, self.cfg.command_name, self._tcp_cfg, self.cfg.tcp_offset)
        self._hold[:] = torch.where(self._hold, e <= self.cfg.hold_r_out, e <= self.cfg.hold_r_in)
        h = self._hold.unsqueeze(1)
        self._filtered[:] = torch.where(h, self._target, self._filtered)
        self._processed_actions[:] = self._filtered

    def reset(self, env_ids=None):
        super().reset(env_ids)
        self._hold[slice(None) if env_ids is None else env_ids] = False


# ---------- 목표 표 ----------
def load_table(path):
    d = np.load(path)
    info = json.loads(str(d["info"]))
    return torch.tensor(d["q"], dtype=torch.float32), torch.tensor(d["tcp"], dtype=torch.float32), info


def tcp_pos_b(env, asset_cfg: SceneEntityCfg, tcp_offset):
    """TCP 위치(로봇 루트 = base_link 좌표, m). TCP = arm_link_5 좌표의 tcp_offset."""
    asset = env.scene[asset_cfg.name]
    bid = asset_cfg.body_ids[0]
    pos_w = asset.data.body_pos_w.torch[:, bid]
    quat_w = asset.data.body_quat_w.torch[:, bid]
    off = torch.tensor(tcp_offset, device=env.device).expand(pos_w.shape[0], 3)
    tcp_w = pos_w + quat_apply(quat_w, off)
    return quat_apply_inverse(asset.data.root_quat_w.torch, tcp_w - asset.data.root_pos_w.torch)


class TableTargetCommand(CommandTerm):
    """목표 표(make_targets.py)에서 TCP 목표 위치를 고른다. 명령 = 루트(base_link) 좌표 위치 (N, 3)."""


    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        q, tcp_fp, self.table_info = load_table(cfg.table_path)
        # 표의 TCP는 base_footprint 기준 → 루트(base_link)는 base_footprint 위 BASE_LINK_Z
        self.table_q = q.to(self.device)
        self.table_tcp = (tcp_fp - torch.tensor([0.0, 0.0, cfg.base_link_z])).to(self.device)
        self.robot = env.scene[cfg.asset_name]
        self.body_cfg = SceneEntityCfg(cfg.asset_name, body_names=[cfg.body_name])
        self.body_cfg.resolve(env.scene)
        self.target_b = torch.zeros(self.num_envs, 3, device=self.device)
        self.target_idx = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self.metrics["position_error"] = torch.zeros(self.num_envs, device=self.device)

    @property
    def command(self) -> torch.Tensor:
        return self.target_b

    def _resample_command(self, env_ids):
        idx = torch.randint(0, len(self.table_tcp), (len(env_ids),), device=self.device)
        self.target_idx[env_ids] = idx
        self.target_b[env_ids] = self.table_tcp[idx]

    def _update_command(self):
        pass

    def _update_metrics(self):
        cur = tcp_pos_b(self._env, self.body_cfg, self.cfg.tcp_offset)
        self.metrics["position_error"] = torch.linalg.norm(cur - self.target_b, dim=1)



# ---------- 관찰·보상 ----------
def tcp_error_vec_b(env, command_name: str, asset_cfg: SceneEntityCfg, tcp_offset) -> torch.Tensor:
    """관찰(설계 v4): 목표 − TCP 위치(루트 좌표, m). 정책이 남은 거리를 직접 보게 한다."""
    return env.command_manager.get_command(command_name) - tcp_pos_b(env, asset_cfg, tcp_offset)


def arm_desired_minus_pos(env, action_name: str, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """관찰(설계 v7): 원하는 목표 − 실제 관절 위치(rad). 변화량 행동에서 정책이 쌓인 목표를 보게 한다."""
    term = env.action_manager.get_term(action_name)
    return term.desired - env.scene[asset_cfg.name].data.joint_pos.torch[:, asset_cfg.joint_ids]


def tcp_position_error(env, command_name: str, asset_cfg: SceneEntityCfg, tcp_offset) -> torch.Tensor:
    cmd = env.command_manager.get_command(command_name)
    return torch.linalg.norm(tcp_pos_b(env, asset_cfg, tcp_offset) - cmd, dim=1)


def tcp_position_error_tanh(env, std: float, command_name: str, asset_cfg: SceneEntityCfg, tcp_offset):
    return 1.0 - torch.tanh(tcp_position_error(env, command_name, asset_cfg, tcp_offset) / std)


def tcp_position_error_tanh_sq(env, std: float, command_name: str, asset_cfg: SceneEntityCfg, tcp_offset):
    """1 − tanh((d/std)²). 목표점(d=0)에서 기울기가 0이라 오차가 0 근처일 때 더 줄일 이득이 작다(v13)."""
    d = tcp_position_error(env, command_name, asset_cfg, tcp_offset)
    return 1.0 - torch.tanh((d / std) ** 2)


_CLEARANCE_GEOM = {}


def _clearance_geom(device):
    """tools/make_clearance_geom.py 결과(차체 거리 격자, 팔 몸체 점)를 장치별로 한 번만 읽는다."""
    if device not in _CLEARANCE_GEOM:
        from pathlib import Path
        g = np.load(Path(__file__).resolve().parents[1] / "results" / "geom" / "clearance_geom.npz")
        _CLEARANCE_GEOM[device] = {
            "grid": torch.tensor(g["grid"], device=device), "lo": torch.tensor(g["grid_lo"], device=device),
            "h": float(g["grid_m"]), "pts": torch.tensor(g["arm_points"], device=device),
            "idx": torch.tensor(g["arm_body_index"], device=device, dtype=torch.long),
            "bodies": [str(b) for b in g["arm_bodies"]]}
    return _CLEARANCE_GEOM[device]


def arm_chassis_clearance(env, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """팔 몸체(arm_link_3·4·5, 집게 2) 충돌 형상 점과 차체(base_link·라이다) 충돌 형상 사이 최소 거리(m), 환경별.
    asset_cfg.body_names는 clearance_geom.npz의 arm_bodies와 같은 순서여야 한다."""
    g = _clearance_geom(env.device)
    asset = env.scene[asset_cfg.name]
    bid = asset_cfg.body_ids
    pos = asset.data.body_pos_w.torch[:, bid]                     # (N, B, 3)
    quat = asset.data.body_quat_w.torch[:, bid]                   # (N, B, 4)
    n, p = pos.shape[0], g["pts"].shape[0]
    pw = pos[:, g["idx"]] + quat_apply(quat[:, g["idx"]].reshape(-1, 4),
                                       g["pts"].expand(n, p, 3).reshape(-1, 3)).reshape(n, p, 3)
    rq = asset.data.root_quat_w.torch[:, None].expand(n, p, 4).reshape(-1, 4)
    pb = quat_apply_inverse(rq, (pw - asset.data.root_pos_w.torch[:, None]).reshape(-1, 3))
    ijk = torch.round((pb - g["lo"]) / g["h"]).long()
    shape = torch.tensor(g["grid"].shape, device=env.device)
    ijk = torch.minimum(torch.clamp(ijk, min=0), shape - 1)
    d = g["grid"][ijk[:, 0], ijk[:, 1], ijk[:, 2]].reshape(n, p)
    return d.min(dim=1).values


def arm_chassis_clearance_penalty(env, margin: float, gate_err: float, command_name: str, asset_cfg: SceneEntityCfg,
                                  tcp_cfg: SceneEntityCfg, tcp_offset) -> torch.Tensor:
    """설계 v15: (margin − 최소 거리)/margin(0~1). TCP 오차가 gate_err보다 클 때(가는 길)만. 목표 자세의 일부는
    차체와 가까워(평가 표 7 %가 20 mm 안) 목표 근처에서는 벌점을 주지 않는다. 실제 접촉 벌점(self_contact)은 따로 있다."""
    d = arm_chassis_clearance(env, asset_cfg)
    far = tcp_position_error(env, command_name, tcp_cfg, tcp_offset) > gate_err
    return ((margin - d).clamp(min=0.0) / margin) * far.float()


def arm_self_contact_count(env, sensor_names: list[str], threshold: float) -> torch.Tensor:
    """팔 몸체마다 (필터한 로봇 몸체와의) 접촉 힘이 threshold N을 넘으면 1. 환경별 합."""
    total = torch.zeros(env.num_envs, device=env.device)
    for name in sensor_names:
        fm = env.scene.sensors[name].data.force_matrix_w.torch   # (N, 1, F, 3)
        total += (torch.linalg.norm(fm, dim=-1).amax(dim=(1, 2)) > threshold).float()
    return total


def joint_vel_l2_near_target(env, radius: float, command_name: str, asset_cfg: SceneEntityCfg,
                             tcp_cfg: SceneEntityCfg, tcp_offset) -> torch.Tensor:
    """보상(설계 v10): TCP 오차가 radius(m) 안일 때만 팔 관절 속도 제곱합. 목표에 닿은 뒤 멈춰 있게 한다(J5)."""
    near = tcp_position_error(env, command_name, tcp_cfg, tcp_offset) < radius
    qd = env.scene[asset_cfg.name].data.joint_vel.torch[:, asset_cfg.joint_ids]
    return (qd ** 2).sum(dim=1) * near.float()


def arm_contact_after(env, sensor_names: list[str], threshold: float, start_step: int) -> torch.Tensor:
    """종료(설계 v9): 정책 스텝 start_step 뒤로 팔–로봇 접촉(> threshold N)이 나면 에피소드를 끝낸다."""
    if env.common_step_counter < start_step:
        return torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
    return arm_self_contact_count(env, sensor_names, threshold) > 0


def torque_saturation_ratio(env, asset_cfg: SceneEntityCfg, effort: float) -> torch.Tensor:
    """팔 관절 중 계산 토크(자르기 전)가 effort 이상인 비율(0~1). 판정 J4와 같은 정의."""
    tau = env.scene[asset_cfg.name].data.computed_torque.torch[:, asset_cfg.joint_ids]
    return (tau.abs() >= effort - 1e-6).float().mean(dim=1)


def reset_joints_from_table(env, env_ids, command_name: str, asset_cfg: SceneEntityCfg):
    """리셋 자세 = 목표 표의 무작위 자세(충돌 없음·바닥 위). roll·집게·바퀴는 기본값."""
    term = env.command_manager.get_term(command_name)
    asset = env.scene[asset_cfg.name]
    idx = torch.randint(0, len(term.table_q), (len(env_ids),), device=env.device)
    pos = asset.data.default_joint_pos.torch[env_ids].clone()
    pos[:, asset_cfg.joint_ids] = term.table_q[idx]
    vel = torch.zeros_like(pos)
    asset.write_joint_state_to_sim_index(position=pos, velocity=vel, env_ids=env_ids)
    asset.set_joint_position_target_index(target=pos, env_ids=env_ids)
