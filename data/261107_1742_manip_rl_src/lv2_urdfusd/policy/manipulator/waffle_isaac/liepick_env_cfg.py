"""누운 직사각형 퍽 집기·세우기 `Waffle-LiePick-Direct` (Direct RL, 2026-10-30 사용자 요청).

사용자 요청: "직사각형 퍽 (30 x 30 x 60) 이 누워 있을 때, 중앙을 잡고, 세로로 돌려서 퍽 스토리지에 위치시키기
(정확하게 퍽의 중앙을 잡지 않으면 강력한 음의 보상)". 정책_학습기록 4.16 절.

MuJoCo 확인 (`sim/mujoco/lie_grasp_check*.py`, 새 URDF):
- 옆에서 (집게 수평) 누운 퍽 잡기는 안 됨: 손가락이 집게 축 아래로 약 37 mm 라 손가락 밑이 상자를 뚫음.
- **위에서 (집게 아래 방향) 가운데 잡기**: joint1 축 0.18 ~ 0.24 m 에서 수직 그대로 닿음 (잡는 점 = 퍽 윗면 = 가운데 + 15 mm,
  손가락 밑-상자 5 mm). 0.26 m 부터 수직이 안 됨 (5 ~ 48° 기울어 손가락이 상자에 닿음). 퍽 요는 손목 롤 (joint5) 로 맞춤.
- 들어 올린 뒤 집게를 90° 세움 (닫는 축 = 접선, 집게 방향 아래 -> joint1 축 바깥 수평): 손가락 높이 방향 (퍽 긴 축) 이 세로가 됨
  = 옮기기 (Store) 가 받는 '옆에서 쥔 세운 퍽' 자세 (단 잡는 점이 퍽 가운데). 위 3 ~ 12 cm 에서 수평 자세 닿음 (팔꿈치 방향이 반대라 사이 경로는 학습).
- 퍽 가운데를 쥔 채 세우면 손가락 (높이 ±29 mm) 밑이 퍽 바닥 위 1 mm 라 컵 안으로 밀어 넣지 못함 -> 컵 입구 위에서 놓음.
  팀 `puck_plate_check.py --shape box --drop 16/20` = 12/12 컵에 바르게 앉음 (세운 채 축에 맞춰 놓으면).

행동 6 = 잡는 점 변위 (base x, y, z) x 5 mm, 집게 요 ψ 변화 x 0.06 rad, 집게 숙임 φ 변화 x 0.05 rad, 집게 (> 0 벌림).
  집게 자세 목표 R(ψ, φ): 집게 방향 a = −cos φ z + sin φ (cos ψ, sin ψ, 0), 닫는 축 c = (−sin ψ, cos ψ, 0), 손가락 높이 방향 u = a x c.
  φ = 0 은 아래 방향 (u = (cos ψ, sin ψ, 0) 를 퍽 긴 축에 맞춤), φ = π/2 는 수평 (u = 위). 감쇠 최소제곱 IK (집기와 같은 방식).
관측 40 = 팔 위치 5 · 속도 5, 손가락 1, 인식 퍽 좌표 3, 잡는 점 3, 퍽 − 잡는 점 3, 집게 방향 3, 닫는 축 3, 퍽 긴 축 3,
  옆 퍽 − 퍽 (x, y) 2, (cos ψ, sin ψ, φ) 3, 이전 행동 6.
"""

from __future__ import annotations

from isaaclab.utils import configclass

from .sidepick_env_cfg import WaffleSidePickEnvCfg

LIE_READY = {"joint1": 0.0, "joint2": 0.60, "joint3": -0.92, "joint4": 1.89, "joint5": 0.0}
"""시작 자세 = joint1 축 0.21 m, 퍽 윗면 위 6 cm 에서 집게 아래 방향 (MuJoCo `lie_ready.py`)."""


@configclass
class WaffleLiePickEnvCfg(WaffleSidePickEnvCfg):
    episode_length_s = 8.0
    action_space = 6
    observation_space = 40
    cart_scale = 0.005
    psi_scale = 0.06  # [rad / 스텝]
    phi_scale = 0.05
    rot_hold_weight = 1.0  # 자세가 중요 (집기는 0.3)
    ready_pose = LIE_READY
    ready_noise = 0.08
    gripper_open = 0.010  # 61.6 mm (퍽 30 mm 양쪽 15 mm)

    # 배치: 집을 퍽 = joint1 축 r_range, 방향 ±ang_max, 누운 퍽 긴 축 요 무작위. 상자는 퍽이 안에 들게 + 로봇 앞 끝에서 gap_min 넘게
    r_range = (0.185, 0.235)
    ang_max = 0.25  # [rad]
    gap_min = 0.03
    lie_z = 0.070 + 0.015 + 0.001  # 누운 퍽 중심 (상자 위 + 15 mm + 1 mm)
    nb_dist = (0.06, 0.08)  # 옆 퍽 (세운 원기둥, 목표 아님) 거리
    topple_terminate = False  # 누운 퍽은 원래 누워 있음 (집기의 '넘어뜨림' 판정 끔)
    grasp_dz_lie = 0.011  # LP-8 (2026-10-31 사용자와 정함): 바닥 살짝 위까지 감싸 쥠 (패드가 바닥 위 약 11 mm 부터). 손 IK 각본 lie_roll_test 로 세워 쥠 32/32 확인. 이전 0.015 =  # 잡는 점 = 퍽 가운데 + 15 mm (퍽 윗면 높이, 손가락 밑 5 mm 여유)
    near_start_prob = 0.5  # 역방향 커리큘럼: 집게 요를 퍽 긴 축에 맞춰 시작 (LP-3: 0.3 -> 0.5)
    near_low_prob = 0.6  # LP-3: 0.5 -> 0.6. LP-2: 그중 이 비율은 집게를 퍽 윗면 바로 위 (1 ~ 2 cm, 벌림) 까지 내려 시작 (닫기만 하면 쥠)

    # 판정
    grasp_finger_range = (-0.0108, -0.003)  # LP-2: 누운 퍽을 위에서 쥐면 손가락 −0.0100 ~ −0.0105 (손 IK 각본 `lie_script.py`, 퍽 들림),
    #   세운 원기둥 옆 쥐기 (−0.0058) 범위 (−0.010, −0.003) 를 그대로 써서 LP-1 은 쥐어도 '쥠' 으로 안 셈 (쥠 0 %). 빈손 끝까지 = −0.011
    center_tol = 0.004  # [m] 긴 축 방향으로 가운데에서 이만큼 안 = 가운데 잡음
    stand_cos = 0.996  # 퍽 긴 축 z (≈ 5°) 넘으면 세움
    lift_ok = 0.03
    success_hold_s = 0.5

    # 보상
    rew_reach = 1.0
    rew_reach_fine = 2.0
    rew_yaw_align = 1.0  # 가까울 때 손가락 높이 방향 u 를 퍽 긴 축에 (|u·L|)
    rew_center = 2.0  # 가까울 때 잡는 점이 긴 축 방향 가운데 (1 − tanh(|d|/0.003))
    rew_grasp = 5.0  # LP-5: 2 -> 5 다시 (LP-4 는 쥠 보상을 낮추자 아예 안 쥠). LP-4: 5 -> 2 (LP-3 은 상자 위에서 쥐고만 있어도 +5 라 3 cm 들기 (+4 + 높이) 와 차이가 없어 안 듦). LP-3: 3 -> 5
    rew_close_near = 1.0  # LP-3: 잡는 점이 목표 1 cm 안에서 집게를 닫는 행동 (LP-1·2 는 한 번도 안 닫음)
    perc_bias = 0.0015  # LP-3: 인식 치우침 ±4 -> ±1.5 mm (가운데 4 mm 안을 요구하는데 ±4 mm 치우침이면 맞출 수 없음 -> 쥐면 −20 이라 아예 안 쥠)
    rew_off_center = -20.0  # **강한 음: 쥐었는데 가운데가 아님, 스텝마다**. LP-3: center_tol 넘은 만큼 비례 (4 -> 8 mm 에서 0 -> −20)
    rew_lifted = 15.0  # LP-5: 10 -> 15. LP-4: 4 -> 10
    rew_lift_height = 15.0  # LP-5: 8 -> 15.  # LP-4: 4 -> 8 (쥔 채 0 -> 6 cm 연속)
    rew_psi_arm = 3.0  # LP-7: 들었을 때 집게 요 ψ 를 팔 방향 (joint1 축 -> 잡는 점) 으로 (cos 차, 0 아래 자름) = 세우기 전 준비 (손 IK 각본 순서)
    rew_pitch = 4.0  # LP-7: 들었을 때 실제 집게 방향 수평 성분 (1 − |a_z|), 퍽 세움 여부와 상관없이 (LP-6 은 φ ≈ 0 에서 안 움직임)
    lift_full = 0.10  # LP-7: 들기 높이 보상이 다 차는 높이 6 -> 10 cm (손 IK 각본은 5 ~ 14 cm 에서 세움)
    rew_stand = 10.0  # LP-4: 6 -> 10.  # 들었을 때 퍽 긴 축 z (세로로 돌림)
    rew_level_after = 2.0  # 세운 뒤 집게 수평 (1 − |a_z|)
    rew_success = 20.0  # 들고·세우고·가운데·집게 수평 (한 번)
    rew_puck_slip = -5.0  # 쥔 뒤 퍽이 손에서 미끄러짐 (잡는 점-퍽 거리 증가)
    rew_drop = -20.0  # 들었다 떨어뜨림 + 종료
