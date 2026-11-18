# 261103_1502_loc_pkg — 팀 저장소 localization 패키지 연결 · 모의 확인

## 무엇
- 팀 저장소 worktree `~/git/Lv2_Cartlidar_Assignment_loc`(브랜치 `feat/localization`, origin/dev c27d100에서 시작)에 사용자 소유 새 패키지 `workspace/ros2_ws/src/localization`.
  - Cartographer 순수 위치 추정(기본) · AMCL(인자 `localization:=amcl`, 기본으로는 안 뜸) · map_odom_relay · loc_monitor(두 방식 지원) · scan_body_mask · scan_merge(amcl) · map_server 2개.
  - 팀 규약에 맞춘 것: `map` = 발제 좌표(팀 mission_planner 지도와 같은 원점, 벽 칸 비교 중앙값 1칸 0.025 m), Cartographer 지도 좌표는 `carto_map`(정적 TF map → carto_map),
    map_odom_relay 가 맞춤 값을 곱해 `map → odom` 을 냄, odom → base_link 는 팀 EKF(`bringup/config/ekf.yaml`, 바퀴 vx + IMU yaw rate), loc_monitor 게이트는 기본 끔(`/localization/lock` 만).
  - 남의 파일(kanichong bringup · mission_planner AMCL, control drive_node)은 안 고침.
- 빌드는 저장소 밖 `ws/`(colcon --symlink-install). 단위 시험 `python3 -m pytest -q test/` 3 통과(도구 확인).

## 모의 확인 구성(설계 45 판정 판과 다른 점)
- launch `launch/test_stack.launch.py` = 패키지 launch + `launch/nav2_only.launch.py`(261031_1325_nav2 nav2_mission_loc2 사본 + `localization:=external`: 위치 추정 · 지도 · loc_monitor 를 띄우지 않음).
- EKF = 팀 `bringup/config/ekf.yaml`(icr_odom 없음, vy 안 씀). 설계 45 판정은 icr_odom + ekf_sim_imu_icr(vx · vy).
- Nav2 params `config/nav2_params_v45_B_map.yaml` = v45_B 에서 `mission_map` → `map` 만.
- 실행 `run_k01_pkg.sh`(→ `run_live_pkg.sh`), 채점 evaluate.py · `tools/evaluate_kidnap_map.py`(MM 항등만 다름).
- 팀 bringup 전체(sim:=true)는 쓰지 않음: 팀 Isaac 브리지는 ros2_control TopicBasedSystem · 빈 바닥이라 이 미로 장면과 맞지 않음. 패키지가 닿는 부분(EKF 설정 · 토픽 · 프레임)만 팀 것으로 맞춤.

## 판정 기준(이전 K0 · K1 그대로, 내가 정함)
- K0: G1~G5 통과, SUSPECT 0 · LOST 0(R6).
- K1: R1~R5 통과(evaluate_kidnap), G1 7/7, G3 > 0.

## 결과 표

| 판 | 값(단위) | 예상 | 결과 |
|---|---|---|---|
| K0 carto | G4 RMSE · 최대(m), SUSPECT · LOST, 검사 Hz | 통과. RMSE 0.008~0.015(이전 0.0075, 팀 EKF 는 제자리 회전 때 base_link 옆 이동을 몰라 Cartographer 사전값이 조금 나빠짐), 검사 ≈ 10 Hz. 위험: 잔차 최대가 문턱 0.034 근처라 SUSPECT 날 수 있음 | **불통과**(G3 · SUSPECT). 목표 7/7, G4 RMSE 0.0147 · 최대 0.035 m(이전 0.0075 · 0.021), G3 최소 −0.001 m(겹침 23 표본, 이전 +0.0175), SUSPECT 1(t 74.4 s, 잔차 0.035, 0.3 s 뒤 OK) · LOST 0, 검사 ≈ 10 Hz(n_check 97~100/10 s), 잔차 최대 0.0402. Isaac 139 1번 → 다시 돌림 |
| K1 carto | 감지(s) · 다시 찾은 오차(m) · R1~R5 | 통과. 감지 ≤ 0.2 s(점프), 오차 ≤ 0.02 | **통과**. 감지 0.19 s(점프 0.057 m · 10°), 다시 찾은 오차 0.0097 m · 0.22°, LOST → OK 1.52 s, R1~R5 통과, G1 7/7, G3 0.0105 m. (G2 · G4 불통과는 강제 이동 자체, K1 기준 밖) |
| K0 amcl | G4 RMSE · 최대(m), SUSPECT · LOST | 통과. RMSE 0.016~0.025(이전 0.0154~0.0162, EKF vy 없음은 옴니 모델이 덜 받음). 위험: AMCL 오차가 잔차 문턱 0.034 를 넘겨 SUSPECT | **불통과**(SUSPECT). G1~G5 통과, G4 RMSE 0.0221 · 최대 0.051 m(이전 0.0154~0.0162), G3 0.0064 m, SUSPECT 2(t 224.5 s 잔차 0.0375 1.4 s, t 293.2 s 잔차 0.0391 0.6 s) · LOST 0, 잔차 최대 0.0398 |
| K1 amcl | 감지(s) · 다시 찾은 오차(m) · R1~R5 | 통과. 감지는 점프가 아니라 잔차로 0.5~1 s(AMCL 은 강제 이동을 odom 으로 모름), /initialpose 다시 넣기 뒤 오차 ≤ 0.03 | **통과**. 감지 0.52 s(잔차, SUSPECT → 2.0 s 뒤 LOST), /initialpose 다시 넣기 오차 0.0156 m · 0.14°, 감지 → OK 3.02 s, R1~R5 통과, G1 7/7, G3 0.0048 m. Isaac 139 1번 → 다시 돌림 |
| K0 carto · ICR EKF(원인 확인) | 위 K0 carto 와 같고 EKF 만 이전 icr_odom + ekf_sim_imu_icr | SUSPECT 0 · G3 > 0 · RMSE ≈ 0.008 이면 팀 EKF(vy 없음)가 원인. 그대로 SUSPECT 나면 EKF 아닌 다른 차이(패키지 연결) 의심 | 위치 추정은 예상대로: RMSE 0.0083 · 최대 0.016, 잔차 최대 0.0169, SUSPECT 0 · LOST 0 → **원인 = 팀 EKF vy 없음(모의 1 판)**. 판 자체는 **불통과 G1**: 첫 목표 start_box_1 에서 출발 칸 모퉁이 앞뒤 흔들기 25 s → 'Failed to make progress', 그 자리(여유 0.051, 겹침 없음)가 costmap 상 'Start occupied' 라 뒤 목표 6 개 계획 실패. 위치 추정과 무관한 Nav2 약점(아래) |
| K0 amcl · ICR EKF(원인 확인) | 같음 | SUSPECT 0 · RMSE ≈ 0.016 이면 원인 같음 | **통과**. G1~G5 통과, RMSE 0.0157 · 최대 0.028, G3 0.0088, 잔차 최대 0.0335, SUSPECT 0 · LOST 0 → 원인 같음 |

## 결론(모의)
- K0 SUSPECT 의 원인 = 팀 `bringup/config/ekf.yaml` 이 바퀴 vx 만 받아 스키드 회전 때 base_link 옆 이동(vy = −x_icr · w)을 모름. ICR 보정 EKF 로 바꾸면 두 방식 모두 SUSPECT 0, RMSE 가 설계 45 판정 값으로 돌아옴.
  → bringup(kanichong) 에 EKF ICR 보정 Issue. 패키지 쪽에서는 고칠 수 없음(odom → base_link 는 bringup EKF).
- Nav2 약점(위치 추정과 무관, Nav2 인계 때 같이 넘길 것): 출발 칸 → 첫 목표 구간에서 14 판 모두 모퉁이 앞뒤 흔들기(출발 → x > 1.0 m 까지 27.7~41.6 s, 최소 여유 0.009~0.044 m). 이번 1 판(K0_pkg_icr_carto)은 진행 검사 시간을 넘겨 중단, 그 자리가 costmap 상 점유라 이후 계획 전부 실패(복구 동작 없음).

## 18:2x 변경(사용자 결정: 팀은 AMCL, Nav2 · AMCL 노드는 주행 쪽에 인계, loc_monitor 는 우리)
- 패키지 launch 기본 `localization:=amcl`, 새 인자 `use_amcl_node`(기본 false — AMCL 노드는 주행 쪽 launch). carto 는 남겨 두되 기본에서 끔.
- 시험: 노드 목록 확인(도구 확인) — 기본: scan_body_mask · scan_merge · map_server 2 · lifecycle · loc_monitor(amcl), `use_amcl_node:=true` 면 + amcl · lifecycle_manager_amcl, `localization:=carto` 면 carto 묶음. 모의 판은 다시 안 돌림(누가 AMCL 을 띄우는지만 바뀜). `launch/test_stack.launch.py` 는 use_amcl_node true.
- 인계 묶음 `~/git/lv2_next_module/261103_1820_handoff_nav2_amcl(.zip)`. PR · Issue 본문 `gh/`.
