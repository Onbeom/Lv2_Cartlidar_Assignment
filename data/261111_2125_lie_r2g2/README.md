# 261111_2125 누운 퍽 정책 r2e → r2g2 배포 교체 (#193)

- 브랜치 `fix/lie-upright-r2g2`(origin/dev b6a2ab0), 작업 폴더 `~/git/Lv2_Cartlidar_Assignment_lier2g2`.
- 바꾼 것: `robot_policy/config/models/lieupright_r2e.onnx` → `lieupright_r2g2.onnx`(md5 0c6e20cabce0, RL 261111 1 넘김 `261110_2010_lie_rl/handoff/`), `policies.yaml` lie_upright `model` 과 머리 주석(B7 = r2g2, 노드 관측으로 학습, 속도 비례 축소 3.0 · 놓음 규칙이 학습에 들어 있음).
- 노드 기본값은 이미 학습 조건과 같음(PR #212): `lie_upright.max_joint_velocity` 3.0, `velocity_limit` scale, `release_xy` 0.03 · `release_z` 0.06 · `release_beta` 1.4 · `release_center_offset` 0.011.

## 도구
- `parity_tool.cpp` → `parity_tool`: dev 의 `control/lieupright.{hpp,cpp}` · `arm_diff_ik.cpp` 로 빌드. 한 스텝 입력 → 관측 45 · β · ρ · 관절 목표(lieApplyActionLimited, max_step = 3.0/30) · 고른 배율 · 잡는 점 · 집게 축. 세울 자리 D xy 는 노드 규칙(lieStandSpot, 옆 퍽 없으면 잰 상자 중심 대칭), 쥠은 C++ LieHoldFilter(에피소드마다 초기화).
  빌드: `g++ -O2 -std=c++17 -I<control>/include -I/usr/include/eigen3 parity_tool.cpp <control>/src/lieupright.cpp <control>/src/arm_diff_ik.cpp -o parity_tool`
- `compare.py` → `compare.log`: r2g2 정답 벡터 3 에피소드(315 스텝)와 비교.
- `onnx_check.py` → `onnx_check.log`: 배포 ONNX 에 기록 관측을 넣어 기록 action_raw 와 비교(IsaacLab venv 의 onnxruntime).
- 빌드 `build.txt`, 시험 `test.txt`(이 폴더 build · install, COLCON_IGNORE).

## 시험 표 (모의 기록 대 C++ · ONNX, 실물 아님)
판정 기준 P1 ~ P7 은 검토 261110 3 승인(11-11). **기준은 도구를 한 번 돌린 뒤 계산 분해능으로 정함**(결과 값에 맞춘 것이 아님): float32 기록 대 double 계산은 1e-5, 배율 이분법(8 번) 한 단계 = 1/256. P5 1e-3 rad 는 갈린 스텝의 배율 차 1/256 × 스텝 한계 0.1 rad(3.0 rad/s ÷ 30 Hz) ≈ 3.9e-4 rad 의 약 2.5 배.

| # | 확인 | 구분 | 예상 (값의 의미 · 단위, 예상과 이유) | 기준 안 | 결과 |
|---|---|---|---|---|---|
| P1 | 관측 45 칸 최대 차이 (C++ − Isaac) | 모의 기록 대 C++ | 노드 관측 방식으로 학습했으므로 float32 반올림 수준(< 1e-5) | ≤ 1e-5 | 1.15e-6(approach), 고정 퍽 · 긴 축 · 옆 퍽 · 잰 상자 · D · 쥠 칸 모두 ≤ 6e-8 |
| P2 | 쥠 칸(손가락만 히스테리시스 8 · 3) 다른 스텝 수 | 모의 기록 대 C++ | 같은 필터라 0 | 0 | 0 / 315 |
| P3 | 세울 자리 D (노드 규칙) | 모의 기록 대 C++ | 학습이 노드 규칙을 씀 → 0 에 가까움 | ≤ 1e-6 m | 7.5e-9 m |
| P4 | 고른 배율 vl_scale 차이 | 모의 기록 대 C++ | 학습 float32 · 노드 double 이분법이라 경계에서 한 단계(1/256) 갈릴 수 있음 | 한 단계(0.0039) 이하, 갈린 스텝 ≤ 1 % | 최대 0.0039, 갈린 스텝 2 / 315(0.6 %): ep0:50 0.8008 대 0.8047, ep1:54 0.6055 대 0.6016 |
| P5 | 관절 목표 · β · ρ 최대 차이 | 모의 기록 대 C++ | 배율이 같으면 1e-5 수준, 한 단계 갈리면 그만큼(관절 한 스텝 최대 0.1 rad × 1/256 ≈ 4e-4) | ≤ 1e-3 rad | 관절 6.7e-4 rad · β 2.0e-4 · ρ 2.3e-4 (모두 P4 의 두 스텝) |
| P6 | 배포 ONNX 출력 대 기록 action_raw | 모의 기록 대 ONNX | 같은 파일(md5) 이라 float32 반올림 수준 | ≤ 1e-4 | 8.6e-6, 입력 obs [1, 45] · 출력 actions [1, 6] |
| P7 | 빌드 · 단위 시험 (control · robot_policy) | 도구 | 설정 · 모델 파일만 바뀜 → 모두 통과 | 경고 0 · 실패 0 | 코드 경고 0(파이썬 바이트 컴파일 안내 4), 79 시험 통과. 이 시험들은 누운 퍽 모델 파일을 읽지 않음 |
