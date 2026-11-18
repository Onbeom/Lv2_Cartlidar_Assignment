## 무엇
- manipulator_node 에 미션 skill `lie_upright:<frame>:<상자 x>:<상자 y>:<요 deg>` 를 더했습니다(#193). 상자 위 누운 직육면체를 r2e 정책(#192)으로 세워 놓고 손을 뺀 뒤, 세운 자리를 target_look 으로 다시 잽니다. 다시 집기는 기존 toppick_tilt 가 합니다.
- 흐름: 상자 보기 자세 → 인식 place_box 로 상자 중심 · 요 측정 → READY_POSE_45 → 정책 30 Hz → 놓고 1 s · 손이 세울 자리에서 3 cm → 세울 자리 target_look.
- 계산은 ROS 없는 `control/lieupright.hpp` 에 두었습니다(관측 45, β · ρ 집게 자세 + 감쇠 최소제곱 IK, 세울 자리, 쥠 히스테리시스). 인식 `/perception/place_box/prior` 토픽, policies.yaml `lie_upright` 와 `models/lieupright_r2e.onnx` 도 더했습니다.

## 왜
- 계획기가 누운 직육면체 앞에 이 스킬을 부르게 바꾸고 있습니다(따로 PR). 팔 노드에 이 스킬이 없으면 미션이 그 단계에서 멈춥니다.
- 상자 중심 · 요는 정책 관측에 들어갑니다. 집는 상자도 205 규격이라 장소 상자용 place_box 를 사전값만 바꿔 씁니다.

## 시험 (PC)
- 빌드 경고 0. 새 gtest 7 개와 robot_policy · control · perception 기존 시험을 합쳐 303 개 모두 통과했습니다.
- Isaac r2e 에피소드 3 개(313 스텝) 기록과 비교했습니다. 관절 목표는 1.35e-5 rad, 관측 칸은 1.3e-6 안에서 같았습니다.
- 집는 상자 place_box 를 모의 렌더 120 장으로 판정했습니다. 중심 최대 0.79 mm, 요 최대 0.30° 로 기준(3 mm · 1°) 안이었습니다.
- 노드를 띄워 skill 목표를 보냈습니다. ONNX Runtime 이 없는 PC 에서는 이유를 붙여 실패하는 것을 확인했습니다.

## 한계
- PC 에 ONNX Runtime C++ 가 없어, 노드 안에서 정책을 돌리는 흐름은 실행하지 못했습니다. 모의 미션 · 실물도 아직입니다.
- 쥠은 손가락 범위만 봅니다. 실물 손가락 대응값(`toppick_rl.finger_m_at_*`)은 가정값이고, 실물 판정은 #201 에서 정합니다.
- manipulator_node(LCY · kanichong 님 코드)와 policies.yaml(kanichong 님)에 더한 부분이라 리뷰를 부탁드립니다. 기존 줄은 Task 목록 · skill 이름 해석 · onTick · skillTick · endJob 의 연결 몇 줄만 바꿨습니다.
