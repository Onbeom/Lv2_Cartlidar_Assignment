# 261111_1300 누운 직육면체 세워 놓기 스킬 본체 (#193 3번, manipulator_node)

- 작업 트리 `~/git/Lv2_Cartlidar_Assignment_lieskill`, 브랜치 `fix/lie-upright-skill`(origin/dev 7cb49b0). 커밋 전이다.
- 정책은 r2e(`261110_2010_lie_rl/handoff/lieupright_r2e.onnx`)다. 세워 놓고 손을 빼는 데까지만 한다. 다시 집기는 기존 고정 동작 toppick_tilt가 한다(사용자 11-11).
- 계획기 쪽은 `fix/lie-pick-order`(커밋 전)다. 누운 직육면체 스킬 문장을 `lie_upright:base_footprint:<상자 x>:<상자 y>:<요 deg>`로 낸다. 상자 사전값은 두 퍽 자리의 가운데, 요는 −turn이다.

## 만든 것
1. `control/include/control/lieupright.hpp` · `src/lieupright.cpp`: ROS 없는 계산이다. 학습 `lieupright_env.py`와 같은 식이다.
   - 목표 집게 자세(β 숙임 · ρ 롤 → 열 a · c · u)
   - 한 스텝 행동(β · ρ 적분 + 축-각 오차 감쇠 최소제곱 IK, 앞섬 ±0.25, 관절 한계)
   - 쥠(손가락 범위 + 히스테리시스 8 · 3)
   - 세울 자리 D(학습 `_stand_spot`)
   - 관측 45 조립
2. `manipulator_node`: 미션 skill `lie_upright:<frame>:x:y:yaw_deg`. `skill.enabled`와 따로 `lie_upright.enabled`로 정책을 로드한다. 흐름은 다음과 같다.
   1. 바로 앞 target_look 값(누운 퍽)과 그 자리의 `/perception/pucks`(긴 축 · 옆 퍽)를 읽는다.
   2. 상자 보기 자세(`lie_upright.view_*`, `261111_1130_pickbox_view`에서 고른 값)로 간다. `/perception/place_box/prior`에 사전값을 낸 뒤 `/perception/place_box/measure`를 부르고 상자를 받는다(못 재면 `box_required`면 실패).
   3. READY_POSE_45로 가서 벌린다.
   4. 30 Hz로 정책을 돌린다.
   5. 쥐고 3 cm 든 뒤 집게를 벌리면 놓은 것으로 본다. 그 뒤 집게는 늘 벌린다. 놓고 1 s가 지났고 잡는 점이 D에서 3 cm 넘게 떨어지면 끝이다.
   6. D에서 target_look을 한다. 그 값을 다음 toppick_tilt가 쓴다.
   - 실패로 끝나는 경우: 시간(15 s), 든 뒤 놓기 전에 쥠이 풀림(떨어뜨림), 긴 축을 못 읽음, 관측 · 추론 오류.
   - 옆 퍽이 없으면 세울 자리는 '있었다면' 자리(상자 중심에 대해 누운 퍽 반대편)로 계산한다.
3. `perception_node`: `/perception/place_box/prior`(PoseStamped)를 받는다. 측정 요청 전 5 s 안에 온 값이면 그 측정에만 사전값으로 쓴다. 없으면 기존 파라미터(장소 상자)를 쓴다.
4. `robot_policy/config/policies.yaml`에 `lie_upright` 항목(관측 45 이름 · 순서, `tcp_delta(6)`)을 더하고, `models/lieupright_r2e.onnx`(md5 80640ec1…, handoff와 같음)를 넣었다.

## 시험 (PC, 단위 · 기록 비교 · 기동. 모의 미션 · 실물 아님)

| # | 시험 | 예상 (실행 전 시험 코드에 적은 허용값) | 결과 |
|---|---|---|---|
| U1 | 빌드: robot_policy · control · perception | 경고 · 오류 0 | 통과. 컴파일 경고 0 (`build4.txt` · `build5.txt`) |
| U2 | gtest `LieUpright.*` 7개. 학습 식(torch로 옮긴 `gen_ref.py`)으로 낸 목표 자세 · 세울 자리와 비교, 행동 0 유지, x 한 스텝 5 mm, β 0.5 rad 뒤 자세 맞춤, 앞섬 · 한계, 히스테리시스, 관측 45 배치 | 자세 1e-8, D 1e-7 m, 한 스텝 x 5 ± 0.7 mm | 7/7 통과 |
| U3 | 세 패키지 기존 시험 전체(`colcon test`) | 모두 통과 | 303/303 통과 (`test4.txt`) |
| P1 | Isaac 기록 비교 (`parity/compare.py`): RL 261111 1이 뽑은 r2e 에피소드 3개, 313 스텝. 기록의 상태 · 행동을 C++에 넣어 관측 45 · β · ρ · 관절 목표 · FK를 비교 | 실행 전에 적지 못함(아래 결과만 적는다) | 관절 목표 최대 1.35e-5 rad, 관측 칸 최대 1.25e-6, FK 잡는 점 4.2e-7 m, β · ρ 6e-8. 손가락만 쓰는 쥠은 기록 `ho_finger`와 313/313 같음. 옆 퍽 없을 때 대칭 근사 D는 학습 D와 0.00 mm (`parity/compare.log`) |
| P2 | 노드 단계 판정을 같은 기록에 대어 봄 (`parity/phases.py`) | 실행 전에 적지 못함 | 놓음 판정은 학습 released보다 1~3 스텝 앞. 그 사이 정책도 계속 벌림이라 같다. 끝 판정은 학습 성공(에피소드 끝)보다 3~5 스텝 앞이라 학습에 없던 상태로 정책을 돌리지 않는다(`phases.log`). 처음 판정(손이 D에서 3 cm 넘게 1 s)은 끝이 학습 성공보다 늦거나 오지 않아서 바꿨다 |
| S1 | 노드 기동 + `execute_skill lie_upright:base_footprint:0.3448:-0.1182:-17.06` (도메인 141) | PC는 ONNX Runtime이 없어 정책 로드 실패 로그를 남기고 노드는 뜸. 목표는 받고 이유를 붙여 실패 | 그대로 나옴 (`smoke2.txt` · `goal.txt`) |

## 한계 · 남은 것
- **노드 안 정책 흐름은 실행해 보지 못했다.** PC robot_policy는 ONNX Runtime C++ 헤더가 없어 정책을 돌리지 않는다. 상자 보기 → place_box → 시작 자세 → 정책 → target_look 흐름은 빌드와 계산 비교만 했다. Jetson ONNX Runtime 빌드 여부는 확인 전이다.
- 쥠 판정은 손가락 범위만 쓴다. r2e는 학습 판정(손가락 + 퍽-잡는 점 3 cm)으로 학습됐다. 기록에서는 같았지만, 실물 손가락 값 대응(`toppick_rl.finger_m_at_*`)은 가정값(★)이다. 실물 판정은 #201(팔 담당)이 정한다.
- 상자 윗면 높이는 TF(base_footprint → base_link)로 넣는다. Isaac base_link 기본 높이(9.75 mm)와 URDF(6.75 mm)가 달라 관측 z가 3 mm 다를 수 있다.
- 퍽은 target_look 값을 고정해서 쓴다(학습은 지연 · 끊김 있는 갱신 뒤, 가까워지면 고정).
- 상자 보기 자세로 움직일 때 팔-상자 · 옆 퍽 충돌은 보지 않았다(렌더만).
- 실물에서 팔 처짐(r2e 21 %가 x 0.38 m 넘게 뻗음)은 실물 항목이다.

## 11-11 검토 반려 뒤 고친 것 (RL 261110 1 필수 1 ~ 3)
- 필수 1: 다음 스텝 기준(`r.target`)을 자르기 전 IK 값이 아니라 **실제로 보낸(속도 제한으로 자른) 값**으로 바꿨다. 학습 arm_target과 같은 뜻이다. 2.0 rad/s 제한이 성공률에 주는 영향은 RL 261111 1에 Isaac 평가를 요청했다(2.0 · 4.0 rad/s, 512 × 3 seed). 결과를 기다린다.
- 필수 2: 놓음은 잡는 점이 D 수평 30 mm 안 · D 위 60 mm 아래 · β ≥ 1.4일 때 처음 벌린 것만 친다. 밖에서 벌리면 '떨어뜨림' 실패다. 기록 세 에피소드(`parity/phases.log`)는 모두 놓음으로 잡혔다(수평 8.3 · 9.1 · 13.3 mm, 위 37.9 · 39.3 · 20.1 mm, β 1.75 · 1.75 · 1.55). 든 직후 누운 채로 벌린다고 가정하면 위 51 ~ 61 mm, β 0.8 ~ 1.0이라 β 조건으로 떨어뜨림이 된다(`parity/open_midair.log`). β 조건은 검토 요청(60 mm만으로는 든 직후 둘 중 둘이 놓음으로 잡힘).
- 필수 3(가): 끝난 뒤 지금 집게 자세에서 β · ρ를 다시 정하고(`lieBetaRhoFrom`, 학습 `set_beta_rho_from`), 같은 IK를 측정 없이 10 스텝 이어 잡는 점을 5 cm 위로 뺀 뒤 D target_look으로 간다. 3(나) 미션 순서 모의는 RL 261111 1이 한다.
  - 처음에는 정책 β · ρ를 그대로 썼다. 그러자 기록 끝 자세 세 개에서 집게가 3.5° 돌아(0.061 rad) 시험 기준 0.05 rad를 넘었다. 정책 β 1.75는 범위 끝이라 실제 자세와 조금 다르기 때문이다. FK로 다시 정한 뒤 통과했다.

| # | 시험 | 예상 | 결과 |
|---|---|---|---|
| U4 | gtest `LieUpright.RetreatUpFromRecordedEndPoses` (기록 끝 자세 3개) | 위 5 ± 0.5 cm, 수평 5 mm 안, 집게 자세 0.05 rad 안, 관절 한계 안 | 처음 불통과(자세 0.061 rad) → `lieBetaRhoFrom` 뒤 통과. LieUpright 8/8 |
| U5 | control 시험 전체 | 모두 통과 | 304/304 (`test7.txt`) |
| P3 | 놓음 조건을 기록에 대어 봄 | 세 에피소드 모두 놓음, 떨어뜨림 0 | 그대로 (`parity/phases.log`) |
- 세울 자리 D 높이: 학습은 실제 base_link 기준 **상자 윗면 + 29 mm**로 학습됐다(RL 261111 1 확인: _stand_spot이 생성 높이 9.75 mm로 뺐고 차체는 6.75 mm로 내려앉음). 노드 기본값 `lie_upright.spot_dz`를 32 → 29 mm로 바꿨다. 다른 관측(잡는 점, 인식 퍽 z)은 실제 base_link 기준이라 그대로다. 빌드 경고 0, robot_policy · control 시험 모두 통과(`test8.txt`).
