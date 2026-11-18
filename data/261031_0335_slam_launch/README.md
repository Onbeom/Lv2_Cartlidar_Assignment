# SLAM 후보 실시간 launch · 2026-10-31 03:35 (모의 확인)

사용자 지시 "launch 만들기"(2026-10-31). 후보 = 261030_2058 에서 고정한 Cartographer c6 + TF 구성 B + V4(근거 `261030_2058/README.md`). 팀 저장소는 건드리지 않음(옮기는 일은 인지 코드 세션이 사용자 허가 뒤).

## 구성
| 파일 | 하는 일 |
|---|---|
| `launch/slam_carto.launch.py` | 아래 노드 전부. 인자: use_sim_time · use_ekf(팀 bringup 이 EKF 를 내면 false) · ekf_params · use_rsp · urdf · carto_config · map_resolution(0.025) · future_dt(0.1) · mask_file · mask_save |
| `nodes/scan_body_mask.py` | /scan_front · /scan_rear → `*_filtered`. 차체 가림 칸 = ① 출발 전 정지 0.5~1.9 s 에 몸체 상자(x −0.175~0.285, \|y\| ≤ 0.19) 안에 든 적 있는 칸(또는 mask_file) ∪ ② 매 스캔 상자 안 점 |
| `config/carto_candidate.lua` | c6 + tracking base_link · use_odometry false · use_pose_extrapolator false · publish_to_tf false · publish_tracked_pose true |
| cartographer_node · cartographer_occupancy_grid_node | waffle-lyrical-slam 빌드. 입력 scan_1 · scan_2 = `*_filtered`, 출력 /tracked_pose(map→base_link, 스캔 시각) · /map(0.025 m, 1 s) |
| `nodes/map_odom_relay.py` | map→odom = T(map←base, 스캔 시각) · T(odom←base, 같은 시각, EKF)⁻¹ → /tf 에 stamp '지금 + 0.1 s'(20 Hz · 새 값마다), 1 s 넘게 끊기면 안 냄 |
| ekf_node(robot_localization) · `config/ekf_sim.yaml` | odom→base_link(child base_link, 50 Hz). 모의엔 IMU 가 없어 회전 속도도 보정 바퀴 odom 에서(실물은 팀 ekf.yaml: vx + IMU yaw rate) |
| `tools/save_and_align_map.py` | finish_trajectory · write_state → pbstream → map_carto.pgm/yaml → 미로 도면(maze.zip z 0.08, 발제 = USD + (2.2345, 3.2136))에 맞춤 → align.json · map_mission.pgm/yaml(발제 frame) · 정적 TF 예 |
| `tools/tf_probe.py`, `test/*` | 확인용: TF 조회 탐침, 시험 bag 만들기 · 재생 · 채점 |
TF: map→odom = 중계, odom→base_link = EKF, base_link→센서 · base_footprint = robot_state_publisher(정적). 같은 TF 를 두 노드가 내지 않음.

## 발견: 모의 라이다의 차체 가림(03:4x, 시험 bag 분석)
몸체 상자 안 칸이 출발 뒤 0~2 s 앞 126 · 뒤 176칸, 2~5 s 92 · 151칸, **5 s 뒤로는 13~25칸**(주행 내내). 녹화 확인 check.json 의 own_body_deg_median 앞 15.3° · 뒤 12.6° 와 같음(그때 놓침). 모델 문서는 앞 139.5° · 뒤 195.3° 를 못 봄.
→ 모의 라이다는 주행 중 차체를 거의 못 봄(원인 미확인). 261030_2058 의 마스크 v2(정지 구간 칸 고정, 앞 141.3° · 뒤 197.1°)가 실물 가림을 흉내 낸 셈이라 그쪽 SLAM 결과는 실물 조건에 가까움. 이 launch 도 같은 규칙(①)을 씀. 매 스캔 상자 검사(②)만 쓰면 모의에선 실물보다 훨씬 많이 봄.

## 확인(test/run_replay.sh · test/evaluate_launch.py, 밝음 · 어두움 녹화, 도메인 117 · 118)
시험 bag(`test/make_test_bag.py`): 녹화 원본 라이다(마스크 전) · 관절 TF · /clock · camera_info · /odom(회전 배수 1.15 보정), odom TF 없음.
| 항목 | 예상(실행 전) |
|---|---|
| 학습한 고정 칸 | 앞 약 157칸(141°) · 뒤 약 219칸(197°), v2 와 같음 |
| A 실시간 자세(받은 TF 만) | 위치 RMSE 밝음 0.046 · 어두움 0.007 m(±0.01, 261030_2058 B + V4 실시간 채점과 같은 수준), yaw RMSE 0.6~0.8° |
| B 도면 맞춤 뒤(도구 변환으로 USD 좌표 직접 비교) | 밝음 0.010~0.017 · 어두움 0.006~0.012 m |
| 도면 맞춤 보정량 | 밝음 dyaw −0.4~−0.9°, 어두움 −0.2~+0.2°, 이동 ≤ 2 cm. 벽 오차(10 cm 자름) 맞춤 뒤 ≤ 2 cm |
| C 영상 stamp − EKF 최신 stamp | 중앙 ≤ 0.02 s, 최대 < 0.1 s(0.1 s 넘는 것 0) — 인지 세션 조건 1 |
| EKF · 중계 주기 | EKF 0.02 s, 중계 0.05 s 이하, 중계 stamp − 수신 ≈ +0.1 s |
| D TF 탐침 | TimePointZero 성공 ≥ 99 %, 나이 중앙 ≤ 0.03 s · p95 ≤ 0.05 s(중계 없으면 ≈ 0.1 s). 지금 시각 조회 성공 비율은 기록만 |
- 도구 오류(03:4x): 짧은 기동 확인(smoke)에서 띄운 launch 가 정리되지 않고 도메인 117 에 남아, 같은 도메인으로 띄운 밝음 확인과 Cartographer 두 개가 섞임('Requested submap … trimmed' 오류 수백 회). 밝음 결과는 `test/results/bright_TOOLFAIL_smoke_domain117_clash/` 로 보존 · 판정 제외, 남은 프로세스 정리 뒤 도메인 119 로 다시 돌림. 어두움(118)은 영향 없음.

## 확인 결과(04:0x, `test/results/{bright,dark}/evaluate.json`, 모의)
도구 오류 2건(판정 전 고침, 원본 보존): ① 실행 스크립트가 녹화기를 강제 종료해 out_bag 의 metadata.yaml 이 없음 → `ros2 bag reindex` 로 복구(508 s, 끝 4.6 s 빠짐), 스크립트는 녹화기 정상 종료를 기다리게 고침 ② 채점이 정답 끝(512.7 s)까지 계산해 기록 없는 끝 4.6 s 에서 오래된 TF 로 0.2 m 오차 → 받은 TF 끝까지만 채점하게 고침(이전 결과 `evaluate_TOOLFAIL_*`).
| 항목 | 밝음 | 어두움 | 예상과 비교 |
|---|---|---|---|
| 고정 칸(학습) / 실제 가린 각(고정 + 매 스캔) | 157 · 219칸 / 143.6° · 197.9° | 157 · 219칸 / 143.7° · 197.9° | 예상대로(v2 와 같음) |
| A 실시간 자세 위치 RMSE / p95 / 최대 | 0.0424 / 0.0661 / 0.0818 m | 0.0408 / 0.0636 / 0.0737 m | 밝음 예상대로, **어두움은 예상(0.007)보다 나쁨** — 지도 전체 회전 −0.625°(아래). 녹화 5개 범위(0.0085~0.047) 안 |
| A yaw RMSE / p95 | 0.69° / 1.44° | 0.56° / 1.16° | 예상(0.6~0.8°)대로 |
| B 도면 맞춤 뒤(도구 변환, USD 직접 비교) RMSE / p95 / 최대 | **0.0089 / 0.0155 / 0.0299 m** | **0.0083 / 0.0146 / 0.0250 m** | 밝음 예상보다 좋음, 어두움 예상 안 |
| 도면 맞춤 보정 dx · dy · dyaw / 벽 오차 전 → 뒤 / ghost | 0.005 · 0.005 m · −0.625° / 2.68 → 1.23 cm / 1.9 % | 0.005 · 0.010 m · −0.625° / 2.62 → 1.23 cm / 1.9 % | 예상 안(어두움 dyaw 는 예상 ±0.2° 밖 — 이번엔 회전함) |
| C 영상 stamp − EKF 최신 stamp 중앙 / p95 / 최대 / 0.1 s 넘음 | 0 / 0.033 / 0.067 s / **0** | 0 / 0.033 / 0.067 s / **0** | 조건 1 통과 |
| EKF TF 주기 / 중계 주기 / 중계 stamp − 수신 | 0.033 / 0.033 / +0.100 s | 같음 | EKF 는 예상(0.02)보다 느림 — 모의 /odom 30 Hz 에 맞춰 냄(설정 50 Hz) |
| D TimePointZero 성공 / 나이 중앙 · p95 · 최대 | 100 % / 0.033 · 0.067 · 0.083 s | 같음 | 예상(≤ 0.03 · ≤ 0.05)보다 조금 큼 — EKF 30 Hz 간격만큼. 중계가 없을 때(≥ 0.1 s)보다 작음 |
| D 지금 시각 조회 성공(기록만) | 13 % | 11 % | EKF 최신 stamp 가 '지금'보다 조금 앞서지 않으면 실패 — Nav2 는 TimePointZero · transform_tolerance 로 받아야 함 |
- 어두움이 이번엔 −0.625° 돌아간 것: launch 마스크(고정 칸 + 매 스캔)가 v2 보다 칸이 조금 많아(앞 143.6° vs 141.3°) 입력이 달라짐 → Cartographer 가 입력에 민감(녹화 5개에서 본 것과 같은 현상). 도면 맞춤으로 둘 다 0.008~0.009 m.
- 한계: 모의 · 녹화 2개 · EKF 는 IMU 없이 보정 바퀴 odom 만(실물 EKF · IMU 미검증) · 실시간 Isaac 연결(bag 아님)은 안 해 봄 · /map 토픽 QoS 는 확인 안 함.
- 고침(2026-10-31 13:5x, Nav2 모의에서 발견): `scan_body_mask.py` · `map_odom_relay.py` 의 경고 줄이 `logger.warn`(rclpy 에 없음)이라, 정적 TF 가 스캔보다 늦게 오면 마스크 노드가 죽음(bag 재생 확인 땐 /tf_static 이 먼저 있어 이 경로를 안 탐). `warning` 으로 바꿈. 확인한 동작 경로는 바뀌지 않음(위 결과의 source_sha256 은 고치기 전 값).
- 알려진 문제(Nav2 에서 발견): 저장 지도(map_mission)의 출발 자리 footprint 안에 점유 칸 40개(자기 차체로 보임, 원인 미확인 — 마스크 학습 전 첫 스캔 또는 상자 밖 몸체 부품). Nav2 는 그 자리를 지운 복사본을 씀(261031_1325_nav2/tools/clean_start_footprint.py).
