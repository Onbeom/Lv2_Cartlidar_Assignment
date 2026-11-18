# Nav2 진행 이 한 일 — 보고서 재료 (작성: 2026-11-17)

세션 id `local_408a2591-b818-4469-b839-01a9fb2eb3a3`, 시작 2026-10-19. 10-13~10-15의 Gazebo · Cartographer 일은 이 세션이 열리기 전 세션에서 했고, 같은 흐름이라 2-1·2-2에 함께 적는다. 숫자 출처는 각 줄의 결과 파일과 `.claude/STATUS.md`.

## 1. 맡은 일 · 기간
- 10-13~10-15: Gazebo 미로 + Cartographer 지도 만들기, Cartographer 설정 비교(이전 세션).
- 10-19: Nav2 설정 가설 사본(수업에서 Nav2 실습이 나오기 전이라 보류).
- 10-20: 수업 Isaac Sim · Isaac Lab 실습(낙하 · 카메라 · Cartpole).
- 10-21~10-22: 새 로봇(Waffle 4WD + 팔) URDF 1단계 ROS 확인, MoveIt 충돌 확인, USD 만들기, 261022 리비전 이동. 3단계 물리 시험은 시뮬레이션 연습 세션에 넘기고 결과만 받음.

## 2. 한 일
### 2-1. Gazebo 미로 지도 (모의)
- 왜: 운영 측 예제(waffle-lyrical-slam)로 SLAM 흐름을 먼저 익힘.
- 어떻게: Gazebo + Cartographer + 키보드 조종, Waffle Pi SDF(`sim/gazebo_maze_waffle_pi.launch.py`).
- 실험 · 조건: 미로를 손으로 몰며 지도 저장.
- 결과(모의): `maps/waffle_pi_maze.pgm`·`.yaml`, 104×81칸, 0.05 m.
- 결론: 저장 뒤 로봇이 벽에 밀려 Cartographer 상태가 망가져 `.pbstream`은 남기지 않음.

### 2-2. Cartographer 설정 비교 (모의, 같은 bag 오프라인 처리)
- 왜: 지도 품질과 CPU 부담을 함께 보고 설정을 고르기 위해.
- 어떻게: bag 하나(`results/cartographer_ab/bag_20261014_1724`)를 8설정(해상도 0.05/0.025 m × ceres 가중치 10/40·1/4 × 회전 삽입 각도 0.1°/1°)으로 처리, 정답 = `maze.stl` 높이 0.131 m 단면.
- 결과(모의, `results/cartographer_ab/offline_20261014/summary.md`): 기준 c1 유령 벽 0.269 · 벽 위치 오차 8.07 cm · CPU 290.1 s. 최선 c6(0.025 m, 10/40, 1°) 유령 벽 0.035 · 오차 2.28 cm · CPU 384.0 s. 변수 효과: 회전 각도 1° 유령 벽 −0.192 · CPU −82.6 s, 해상도 0.025 오차 −3.91 cm · CPU +162.6 s, 가중치 1/4는 유령 벽 +0.064로 오히려 나빠짐(가설과 반대).
- 루프 폐합 주기(`results/cartographer_ab/live_period_20261015/summary.md`, 실시간 재생 3번씩): 90 / 30 → 도중 유령 벽 0.053 / 0.056, 오차 2.84 / 2.82 cm, CPU 277.8 / 292.3 s.
- 결론: c6 채택, 주기는 차이가 없어 기본값 90 유지. 해상도 · 회전 각도는 Jetson에서 부담되면 되돌리기로 함(사용자 결정).

### 2-3. Nav2 설정 가설 사본 (실행 안 함)
- 왜: Nav2를 바로 시작할 수 있게 준비.
- 어떻게: nav2_bringup 1.5.0 `nav2_params.yaml` 사본에 승인된 가설 7개 반영(`nav2/nav2_params_waffle_pi.yaml`, `nav2/nav2_maze.launch.py`): footprint 다각형, costmap 0.025, MPPI consider_footprint, 속도 0.20/−0.10 m/s · 1.0 rad/s, AMCL laser_max_range 3.5 등.
- 결과: 실행 · 판정 없음.
- 결론: 수업 Nav2 실습 방식이 나오면 다시 하기로 보류(사용자). 실제 Nav2 작업은 뒤의 Nav2 세션들(`261031_1325_nav2`, `261102_1646_nav2_B` 등)이 함.

### 2-4. 수업 Isaac Sim · Isaac Lab 실습 (모의)
- 왜: 수업 실습(`~/pa-course/windows_lab`, Isaac Sim 6.1, Isaac Lab ae37b028).
- 카메라 예제가 조용히 실패 → 진단 사본으로 원인 확인: `CameraSensor`의 resolution은 (세로, 가로)인데 예제는 (320, 240)을 넘김. `resolution=(240, 320)`, 깊이 미리보기 5~15 m로 고침. 결과(모의): seed 7 · 42 모두 RGB (240, 320, 3) · 깊이 (240, 320, 1), 깊이 유효값 99.7 %, 범위 seed 7 5.164~10.357 m · seed 42 5.609~10.357 m.
- Cartpole PPO(모의): num_envs 1, seed 42, 300 반복 학습 끝, `policy.pt`·`policy.onnx` 내보냄(`~/pa-course/IsaacLab-6.1/logs/rsl_rl/cartpole/2026-10-20_12-31-29/`).
- 실습 자료 문제: `isaaclab list_envs`가 고정 커밋에 없음(→ `scripts/environments/list_envs.py`), 예외가 나도 종료 코드 0.

### 2-5. Waffle 4WD URDF 1단계 ROS 확인 (ROS 정적 · 표시 검사, 실물 아님)
- 어떻게: check_urdf, xacro 비교, colcon, display.launch(RViz).
- 결과: 23링크 단일 트리, xacro와 수치 차 0, 메시 504개 모두 있음, 오류 · 경고 0, /joint_states 11개(mimic 집게 포함), 카메라 광축 Arducam 아래 40.8° · RealSense 아래 35.3°.
- 결론: 카메라 방향은 실물과 같음(사용자 확인). 팔 연동 제한은 URDF로 못 나타내 → 자기충돌로 막기로 함.

### 2-6. MoveIt 2 충돌 확인 (ROS 정적 충돌 검사, 모의)
- 어떻게: 새 패키지 `sim_ws/src/waffle_4wd_moveit_config`, 도구 `sim/tools/check_arm_collision.py`(/check_state_validity).
- 결과: 관절 0 자세 · 접은 자세 [0, 1.4992, −1.3998, 1.7000, 0] 충돌 0쌍. 극단 자세 [0, 1.5, 1.5, 1.7, 0]은 base_link–arm_link_3(0.0084 m), base_link–arm_link_4(0.0034 m), gripper_right–lidar_rear(0.0148 m) 검출. RViz 빨간 표시 사용자 확인.
- 결론: 충돌 제외 쌍은 인접 13쌍만. collisions_updater가 낸 "Never" 49쌍은 실제 충돌을 숨길 수 있어 넣지 않음.

### 2-7. USD 만들기 (모의, 정적 검사)
- 어떻게: `sim/isaac/import_waffle_4wd.py`(Isaac Sim 6.1 URDF importer, 자기충돌 켬, 고정 관절 안 합침, 바퀴 = 속도 · 팔 = 위치 목표).
- 결과: importer 오류 0. 판정 ②~⑩(2단계 세션이 실행, `sim/isaac/logs/check_usd_r1.json`): 총질량 4.8911940 kg, 가동 관절 11, 센서 위치 오차 3.9e-9 m, 충돌 형상 542, mimic 계수 −1.
- 결론: 성공, 단 ⑤ 속도 한계 · ⑩ 자기충돌은 newton 속성으로만 기록돼 조건부.

### 2-8. 261022 리비전 이동 · 3단계 결과 받음
- URDF · USD를 `261022_1243/`로 옮기고 패키지 이름 바꿈(`package://` 504곳). 이동 검사 `261022_1243/relocation_check_after_move.json` ok.
- 3단계 물리 시험(모의, 시뮬레이션 연습 세션이 실행, `261022_1243/phys_tests/결과_v2_261022.md`): P1 · P2 · P4 · P5 실패, P3 · P6 · P8 성공, P7은 기준 문구상 통과지만 실제 회전 0.0002°.

## 3. 실패 · 다시 한 것
- 카메라 예제 조용한 실패 → 진단 사본으로 원인(해상도 순서) 찾고 고침.
- MoveIt RViz 설정 파일의 잘못된 Panels 항목 → 지움.
- 문서에 "가동 관절 13개"로 잘못 적음(고정 라이다 관절 2개 포함) → 11개로 바로잡음.
- USD 이동은 Isaac Sim이 파일을 열고 있어 미뤘다가 닫은 뒤 옮김.

## 4. 결과물 위치
- `maps/waffle_pi_maze.*`, `results/cartographer_ab/`, `experiments/cartographer_ab/`
- `nav2/nav2_params_waffle_pi.yaml`, `nav2/nav2_maze.launch.py`
- `sim_ws/src/waffle_4wd_moveit_config/`, `sim/tools/check_arm_collision.py`, `sim/isaac/import_waffle_4wd.py`
- `handoff_teammate/`(USD zip · 명령 모음), `261022_1243/`(URDF · USD · 검증계획 · phys_tests)
- 브랜치 · PR · Issue 없음(학습 폴더, git 저장소 아님).

## 5. 보고서에 꼭 넣으라고 사용자가 정한 것
- 이 세션에서 사용자가 '보고서에 꼭 넣으라'고 한 항목은 없음.

## 6. 영상 · 사진
- `~/pa-course/til_20261020/drop_heights.png`: 낙하 실습 시간-높이 그래프(모의).
- `~/pa-course/til_20261020/cartpole_training.png`: Cartpole 보상 곡선(모의).
- `results/cartographer_ab/live_period_20261015/period_timeseries.png`: 루프 폐합 주기 90/30 도중 지도 품질(모의).
- `photo/261022_arm_link_5_나사제거_전후.png`: 261022 리비전 변경 부분.
- 영상 없음.

## 7. 한계 · 남은 일
- 모두 모의 · 정적 검사. 실물 시험 없음.
- 물성 · 토크 · 관절 한계는 추정값. 3단계 미해결 3건(보고 속도와 위치 변화 불일치, Elbow 반대 방향, 제자리 회전 안 됨)은 뒤 리비전에서 다시 검사함(이 세션 밖).
- Nav2 사본은 실행하지 않음. Cartographer 비교는 Gazebo bag 하나 기준.
