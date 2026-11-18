# r2e 정답 벡터 (Isaac 모의, 2026-11-11)

C++ `lieApplyAction`(β · ρ 적분 + 감쇠 최소제곱 IK)과 관측 45칸 조립이 학습 환경과 같은 값을 내는지 로봇 없이 맞춰 보려고 만든 기록입니다.

- 만든 도구: `../../tools/r2e_parity_dump.py`
- 실행 기록: `../../results/r2e_parity_run.log`
- 환경: `Waffle-LieUprightB4-Direct`(r2e 학습 설정)이고, 옆 퍽 밀림은 평가와 같게 월드 좌표로 판정합니다.
- 정책: `../lieupright_r2e.pt`(sha256 b728a7a6…, `results/p1_r2e_policy.pt` 와 같은 파일)
- 조건: seed 777, 대 3개, 모두 처음부터 시작(kind 0)

| 파일 | 옆 퍽 | 스텝 | 끝 |
|---|---|---|---|
| `ep0_nb1.json` | 있음 | 108 | 성공 3.60 s |
| `ep1_nb0.json` | 없음 | 99 | 성공 3.30 s |
| `ep2_any.json` | 없음(무작위로 뽑힘) | 106 | 성공 3.53 s |

## 기록 시점

값은 그 스텝 `_pre_physics_step` 안에서 기록했습니다. 이때 물리 상태는 정책에 넣은 `obs` 를 만들 때와 같습니다.

- `q`, `qd`, `finger_*`, 몸체 자세: 그 스텝에서 잰 값
- `beta_before` · `rho_before` · `arm_target_before`: 갱신하기 전 값
- `beta` · `rho` · `arm_target`: 행동을 반영한 뒤 값
- `action_raw`: 정책 출력 그대로
- `action_clip`: [−1, 1] 로 자르고, 놓은 뒤에는 집게 칸을 1 로 바꾼 값
- `prev_action`: 직전 스텝의 `action_clip`. obs 39~44 칸과 같습니다.

## 키

- `meta`: 관절 이름, 관절 한계(`arm_lo`, `arm_hi`), 기본 자세, 상수(cart_scale, beta_scale, rho_scale, beta_range, rho_max, ik_damping, target_lead_max, 잡는 점 오프셋, joint1_x 등), 자체 확인 결과
- `episode`: 옆 퍽 있음 여부, 상자 중심 · 요, 세울 자리 D, 인식 치우침(perc_bias, box_dh, yaw_bias), `steps[]`, `end`
- `steps[].ik`: IK 중간값
  - `dp_base`: 잡는 점 변위
  - `h_base`
  - `R_target_base`: 열이 a, c, u 인 행렬
  - `q_target_w`, `q5_w`
  - `err`: 6칸
  - `J_w`: gripper_roll_link 의 월드 야코비안. 6행(선속도 3 + 각속도 3) × 팔 5열
  - `rr_w`: 잡는 점 오프셋의 월드 벡터
  - `Jf`: (J_lin − rr × J_ang, J_ang × w)
- 식:
  - `dq_ik` = Jfᵀ (Jf Jfᵀ + 1e-4 I)⁻¹ err
  - `arm_target` = clamp(clamp(arm_target_before + dq_ik, q ± 0.25), lo, hi)
- 상태 칸:
  - `gripper_cmd`: 이 스텝에 손가락 관절로 보낸 목표 [m]. `action_clip[5] > 0` 이면 0.010(벌림), 아니면 −0.011(닫음)
  - `hold_state`: 학습 판정 쥠. obs 35 칸에 들어간 값입니다.
  - `hold_on` · `hold_off`: 쥠 히스테리시스 카운터. 직전 스텝 물리 뒤 갱신한 값으로, 이 스텝 `hold_state` 를 만든 값입니다. 연속 8 스텝이면 쥠, 연속 3 스텝이면 놓음.
  - `ho_finger`: 손가락 범위만 보는 쥠. 기록 도구가 같은 히스테리시스로 따로 계산했고, 정책에는 넣지 않았습니다.
  - `released`, `was_held`, `stood`, `placed`, `perc_frozen`
- 비교용 참값: `puck_true_base`, `nb_true_base`. obs 11~13 칸은 지연 · 잡음이 들어간 인식값입니다.

## 좌표 · 단위

- base = 로봇 root = `base_link`
- 쿼터니언 순서 = (x, y, z, w). Isaac Lab 3.0 순서입니다.
- 단위: m, rad

## 자체 확인 (세 에피소드, 모든 스텝)

- 관측 조각(관절, 잡는 점, 집게 축, D, 상자, β · ρ, 이전 행동)과 기록 값의 차이: 0
- obs 35 칸과 `hold_state` 가 다른 스텝: 0
- `arm_target` 을 위 식으로 다시 계산한 값과의 차이: 0
- 감싼 IK 사본과 원래 `_ik_pose` 출력의 차이: 최대 2.2e-6 rad (float32 연산 차)

## 비교할 때 주의

- 정책 출력은 C++ ONNX 와 조금 다를 수 있습니다. TorchScript 대비 최대 1.5e-4 입니다. 그래서 행동 → 관절 목표 비교는 기록된 `action_raw` 를 넣어서 합니다.
- `J_w` 는 Isaac USD 에서 나온 값이고, C++ 은 URDF FK 로 계산합니다. 모델 차이가 있으면 IK 결과도 조금 달라질 수 있습니다. 그럴 때는 `J_w` · `q5_w` · `rr_w` 를 따로 비교하면 원인을 나눌 수 있습니다.
- 모의 기록입니다. 실물에서 잰 값이 아닙니다.
