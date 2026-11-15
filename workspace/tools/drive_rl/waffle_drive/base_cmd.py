"""정책 행동 → 바퀴 속도. 실물 명령이 지나가는 길의 한계를 흉내 낸다 (torch 만 씀, Isaac 없이 시험 가능).

  정책 출력 [-1, 1]² × drive_node skill 한계 (0.3 m/s, 1.5 rad/s)     ros2_ws/src/control/src/drive_node.cpp skill.max_*
  → diff_drive: |vx| ≤ 0.45, |wz| ≤ 2.0, vx 가속 · 감속 ≤ 1.5 m/s²    opencr_hardware/config/controllers_4wd.yaml · drive_belt.yaml
    (각속도 가속 한계는 설정에 없음)
  → 바퀴 ω = (vx ∓ wz · 좌우 간격 / 2) / 반지름
  → 한 바퀴라도 한계를 넘으면 좌우를 같은 비율로 줄임                  펌웨어 비례 축소 (PR #10). controller_manager 는 바퀴를
                                                                        따로 자르지 않음 (opencr.ros2_control.xacro limits enable false)
바퀴 한계는 시뮬 바퀴 한계 (팀 robot_params.json 14.661 rad/s, v7) 를 쓴다. 실물 펌웨어 한계 (모터 7.51 rad/s × 벨트 v7 2.0 = 15.02) 보다
작아서, 시뮬 관절이 바퀴마다 따로 잘려 회전 반경이 바뀌는 일이 없다.
"""

from __future__ import annotations

import math

import torch

SKILL_VX = 0.3  # drive_node skill.max_linear [m/s] = 정책 vx 출력 범위
SKILL_WZ = 1.5  # drive_node skill.max_angular [rad/s] = 정책 wz 출력 범위
DD_VX = 0.45  # diff_drive linear.x.max_velocity (drive_belt.yaml, v7 2026-11-14 · 이전 0.34) [m/s]
DD_WZ = 2.0  # diff_drive angular.z.max_velocity [rad/s]
DD_AX = 1.5  # diff_drive linear.x.max_acceleration = -max_deceleration [m/s²]
MOTOR_M_PER = 0.0335 * 1.5  # 실물 모터 1 rad/s → 바퀴 둘레 [m/s] (바퀴 반지름 controllers_4wd.yaml 0.0335 × 벨트 36T→24T 1.5)
TURNING_W = 0.05  # [rad/s] 이보다 빠르게 돌면 '도는 중' (멈춘 채면 제자리 회전, 주행기록 HTML 판정과 같음)


class BaseCmd:
    """환경마다 diff_drive 가속 한계 상태 (직전 vx 명령) 를 가진다. step() 은 물리 스텝마다 부른다.

    mode "twist" (기본): 위 사슬. "wheels": 행동 = 왼쪽 · 오른쪽 바퀴 속도 × 바퀴 한계 (예전 D1 방식, 비교용).
    mode "curvature" (2026-11-13 팀 결정: 제자리 회전 금지 — 뒤 바퀴 벨트가 빠짐): 행동 = (vx, 곡률).
      vx = a0 × (앞 SKILL_VX | 뒤 vx_rev), 곡률 κ = a1 / min_turn_r, wz = (가속 한계를 지난) vx × κ
      → |wz| ≤ |vx| / min_turn_r, vx 가 0 이면 wz 도 0 (멈춘 채 못 돎). min_turn_r ≥ 좌우 간격 / 2 라 두 바퀴가 늘 같은 방향.
    규칙 v2 (2026-11-14 사용자 승인, curvature 모드에서 아래 값을 주면 켜짐 — 기본값은 11-13 규칙 그대로):
      spin_max > 0: 멈춘 채 (|vx| < spin_v) 제자리 회전 허용. 속도 ≤ spin_wz, 돈 각도 합 ≤ spin_max (0.15 m = reset_dist 움직여야
        다시 셈 → 90° + 90° 같은 이어 돌기 막음). wz = a1 × (vx / R + spin_wz × (1 − |vx| / spin_v)⁺) 라 spin_v 경계에서 끊기지 않음.
      rev_max < ∞: 이어진 후진 ≤ rev_max (reset_dist 만큼 전진해야 다시 셈). 남은 후진이 멈추는 거리 이하면 뒤로 가는 명령을 0 으로.
      turn_motor_max < ∞: 꺾는 중 가장 빠른 모터 ≤ 상한 (곡률 turn_k0 부터 줄어 turn_k1 부터 turn_motor_max, 제자리 회전도 이 값).
        넘으면 vx · wz 를 같은 비율로 줄임 = 네 바퀴 같은 비율 · 회전 반경 그대로 (벨트 빠짐 대책).
      넘는 명령은 행동 단계에서 잘림 (벌점이 아니라 못 함) — 실물에서도 cmd_vel 앞 같은 필터로 옮길 수 있게.
    """

    def __init__(self, num_envs: int, device, wheel_radius: float, wheel_separation: float, wheel_speed_max: float,
                 dt: float, mode: str = "twist", min_turn_r: float = 0.20, vx_rev: float = 0.15,
                 spin_max: float = 0.0, spin_wz: float = 0.5, spin_v: float = 0.03, reset_dist: float = 0.15,
                 rev_max: float = math.inf, turn_motor_max: float = math.inf, turn_k0: float = 0.5, turn_k1: float = 1.0):
        if mode not in ("twist", "wheels", "curvature"):
            raise ValueError(f"action_mode 는 twist · wheels · curvature: {mode}")
        if mode == "curvature" and not min_turn_r >= wheel_separation * 0.5:
            raise ValueError(f"min_turn_r {min_turn_r} < 좌우 간격 / 2 ({wheel_separation * 0.5:.4f}) — 안쪽 바퀴가 거꾸로 돎 (제자리 회전 성분)")
        if not 0.0 <= vx_rev <= DD_VX:
            raise ValueError(f"vx_rev 는 0 ~ {DD_VX}: {vx_rev}")
        if mode != "curvature" and (spin_max > 0 or rev_max < math.inf or turn_motor_max < math.inf):
            raise ValueError("규칙 v2 한계 (spin_max · rev_max · turn_motor_max) 는 curvature 모드에서만")
        if not (0.0 <= spin_max <= math.pi and spin_wz > 0 and spin_v > 0 and reset_dist > 0 and rev_max > 0
                and turn_motor_max > 0 and 0 <= turn_k0 < turn_k1):
            raise ValueError("규칙 v2 값이 잘못됨")
        self.mode = mode
        self.min_turn_r, self.vx_rev = min_turn_r, vx_rev
        self.r, self.sep, self.wheel_max, self.dt = wheel_radius, wheel_separation, wheel_speed_max, dt
        self.spin_max, self.spin_wz, self.spin_v, self.reset_dist = spin_max, spin_wz, spin_v, reset_dist
        self.rev_max, self.turn_motor_max, self.turn_k0, self.turn_k1 = rev_max, turn_motor_max, turn_k0, turn_k1
        # 곡률 turn_k0 에서 전진 최고 속도일 때 바깥 모터 = 상한이 줄기 시작하는 값 (직진 · 완만한 꺾기는 안 건드림)
        self._m_top = max(SKILL_VX * (1 + turn_k0 * wheel_separation / 2) / MOTOR_M_PER, turn_motor_max)
        self.vx = torch.zeros(num_envs, device=device)
        """한계를 적용한 뒤의 vx 명령 [m/s] (관측에도 씀 — 실물에서는 호스트가 아는 값)"""
        z = lambda: torch.zeros(num_envs, device=device)  # noqa: E731
        self.spin_used, self.moved = z(), z()
        """멈춘 채 돈 각도 합 [rad] · 마지막 제자리 회전 뒤 움직인 거리 [m]"""
        self.rev_run, self.fwd = z(), z()
        """이어진 후진 거리 [m] · 마지막 후진 뒤 전진 거리 [m]"""

    def reset(self, env_ids) -> None:
        for t in (self.vx, self.spin_used, self.moved, self.rev_run, self.fwd):
            t[env_ids] = 0.0

    def limits_obs(self) -> torch.Tensor:
        """(N, 2) = 남은 제자리 회전 비율 · 남은 후진 비율 (0 ~ 1, 한계가 꺼져 있으면 0 · 1) — 정책 관측"""
        spin = (1 - self.spin_used / self.spin_max).clamp(0.0, 1.0) if self.spin_max > 0 else torch.zeros_like(self.vx)
        rev = (1 - self.rev_run / self.rev_max).clamp(0.0, 1.0) if self.rev_max < math.inf else torch.ones_like(self.vx)
        return torch.stack((spin, rev), dim=1)

    def step(self, actions: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """actions (N, 2) → (왼쪽, 오른쪽) 바퀴 각속도 [rad/s], 각각 (N,)"""
        a = actions.clamp(-1.0, 1.0)
        if self.mode == "wheels":
            wl, wr = a[:, 0] * self.wheel_max, a[:, 1] * self.wheel_max
            self.vx = (wl + wr) * 0.5 * self.r
            return wl, wr
        if self.mode == "curvature":
            vx_target = torch.where(a[:, 0] >= 0, a[:, 0] * SKILL_VX, a[:, 0] * self.vx_rev).clamp(-DD_VX, DD_VX)
            if self.rev_max < math.inf:   # 남은 후진 ≤ 멈추는 거리 (+ 두 스텝 여유) 면 뒤로 가는 목표를 0 으로
                back = (-self.vx).clamp(min=0.0)
                stop_d = back * back / (2 * DD_AX) + 2 * back * self.dt + DD_AX * self.dt * self.dt
                vx_target = torch.where(self.rev_run + stop_d >= self.rev_max, vx_target.clamp(min=0.0), vx_target)
        else:
            vx_target = (a[:, 0] * SKILL_VX).clamp(-DD_VX, DD_VX)
        dv = DD_AX * self.dt
        self.vx = self.vx + (vx_target - self.vx).clamp(-dv, dv)
        if self.mode == "curvature":   # 가속 한계를 지난 vx 로 → 늘 |wz| ≤ |vx| / min_turn_r (+ 멈춘 채 제자리 회전 몫)
            wz = self.vx * a[:, 1] / self.min_turn_r
            if self.spin_max > 0:
                speed = self.vx.abs()
                wz = wz + a[:, 1] * self.spin_wz * (1 - speed / self.spin_v).clamp(min=0.0)
                left = (self.spin_max - self.spin_used).clamp(min=0.0) / self.dt
                wz = torch.where(speed < self.spin_v, torch.maximum(torch.minimum(wz, left), -left), wz)
            wz = wz.clamp(-DD_WZ, DD_WZ)
            if self.turn_motor_max < math.inf:
                speed = self.vx.abs()
                f = ((wz.abs() / speed.clamp(min=1e-6) - self.turn_k0) / (self.turn_k1 - self.turn_k0)).clamp(0.0, 1.0)
                allow = self._m_top - (self._m_top - self.turn_motor_max) * f
                allow = torch.where(speed < self.spin_v, torch.full_like(allow, self.turn_motor_max), allow)
                s = ((speed + wz.abs() * self.sep * 0.5) / MOTOR_M_PER / allow).clamp(min=1.0)
                self.vx, wz = self.vx / s, wz / s
            self._count(wz)
        else:
            wz = (a[:, 1] * SKILL_WZ).clamp(-DD_WZ, DD_WZ)
        half = wz * self.sep * 0.5
        wl, wr = (self.vx - half) / self.r, (self.vx + half) / self.r
        s = (torch.maximum(wl.abs(), wr.abs()) / self.wheel_max).clamp(min=1.0)
        return wl / s, wr / s

    def _count(self, wz: torch.Tensor) -> None:
        """규칙 v2 셈 (이번 물리 스텝의 최종 vx · wz 로)"""
        speed, zero = self.vx.abs(), torch.zeros_like(self.vx)
        if self.spin_max > 0:
            slow = speed < self.spin_v
            self.spin_used = torch.where(slow, self.spin_used + wz.abs() * self.dt, self.spin_used)
            self.moved = torch.where(slow & (wz.abs() > TURNING_W), zero, self.moved + speed * self.dt)
            self.spin_used = torch.where(self.moved >= self.reset_dist, zero, self.spin_used)
        if self.rev_max < math.inf:
            back = self.vx < 0
            self.rev_run = torch.where(back, self.rev_run - self.vx * self.dt, self.rev_run)
            self.fwd = torch.where(back, zero, self.fwd + self.vx.clamp(min=0.0) * self.dt)
            self.rev_run = torch.where(self.fwd >= self.reset_dist, zero, self.rev_run)
