# Isaac 실시간 연결 확인 · 2026-11-01 00:16 (모의)

사용자 승인(2026-11-01): 1단계 SLAM launch 실시간, 2단계 Nav2 v11 실시간 1회. 팀 저장소 · 원본 설정은 바꾸지 않음. ROS 도메인 101, GPU 잠금 /tmp/lv2_gpu.lock.
지금까지 SLAM launch(`261031_0335_slam_launch`)는 녹화 bag 재생으로만, Nav2(`261031_1325_nav2`)는 2D 운동학 모의로만 확인했다. 이번엔 Isaac 장면이 내는 토픽을 바로 받는다.

## 장면(`scene_live.py` = `261030_1943/record_slam_lab.py` 사본 + 실시간 선택)
- 같은 것: 새 모델 urdf_261028 Isaac light(생성 파일 수정 없음), 새 미로 maze.zip(벽만, 상자 · 신호등 · 퍽 없음), 라이다 LDS-03 400칸 10 Hz 2개(차체 가림 포함), 앞 D435 IR 좌 · 우 30 Hz, 정답은 truth.csv 에만.
- 바꾼 것: `/odom` 50 Hz(실물 팀 `controllers_4wd.yaml` publish_rate 50 · `sensors.yaml` odom_rate_hz 50 과 같게, 녹화 땐 30 Hz), 바퀴 odom 회전 1/1.15 보정(시험 bag 과 같은 값), odom→base_link TF 안 냄(EKF 가 냄), 2단계용 `--mode cmd_vel`(`/cmd_vel_nav` TwistStamped, 0.5 s 끊기면 정지).
- 장면 브리지는 Isaac Lab 안의 rclpy(Isaac Sim native ROS 2 브리지 아님).

## SLAM launch 변경(`261031_0335_slam_launch/launch/slam_carto.launch.py`, 사용자 결정 2026-11-01)
- 정적 TF `mission_map → map` 추가(인자 use_mission_tf · mission_frame · mission_x/y/yaw, 기본 = 출발 자세 공칭 0.734, 0.950, π, 지도 저장 뒤엔 align.json 값). 고치기 전 사본 `slam_carto.launch.py.before_mission_map_261101`(sha256 3a5d3c2d…7502).
- 저장 지도로 AMCL 을 돌리는 단계(Nav2)는 map 이 곧 발제 좌표라 이 TF 를 쓰지 않음.

## 1단계 SLAM 실시간(`run_live_slam.sh` → `evaluate_live.py`)
경로 = 녹화와 같은 route_maze.json(경유점 91, 50.3 m), 조명 1.0(밝음). 판정 기준(사용자 승인)과 예상:

| 항목 | 기준 | 예상(실행 전) |
|---|---|---|
| S1 /clock · use_sim_time | /clock 앞으로만, 모든 노드 true | 통과(장면 60 Hz, launch 의 모든 노드에 use_sim_time 인자) |
| S2 라이다 2개 · /odom 끊김 | 받은 수 ≥ 낸 수 99 %, stamp 간격 최대 ≤ 주기 × 2 | 통과. 위험: 렌더 프레임(4 스텝마다)에 물리 · 발행이 같이 늦어지지만 stamp 는 sim 시각이라 간격은 일정할 것 |
| S3 mission_map → 앞 카메라 TF | TimePointZero ≥ 99 %, 나이 p95 ≤ 0.05 s | 통과. bag 확인 땐 p95 0.067 s 였는데 원인은 /odom 30 Hz(EKF 가 측정마다 냄) → 50 Hz 로 p95 약 0.04 s |
| S4 영상 stamp − EKF 최신 stamp | 0.1 s 넘는 것 0 | 통과(bag 땐 최대 0.067 s, 50 Hz 면 더 작음) |
| 기록 A 실시간 위치 RMSE · B 도면 맞춤 뒤 | (판정 아님) | A 0.01~0.05 m, B 0.008~0.016 m(녹화 5개 범위). 입력이 bag 과 조금 달라 Cartographer 결과는 녹화마다 흩어지는 범위 안 |

### 도구 확인(경유점 4, `results/smoke_wp4_TOOLCHECK/`, 판정 아님)
- 장면 · launch · 지도 저장 · 도면 맞춤은 정상(Isaac 종료 0, 노드 10개 use_sim_time true).
- 도구 오류 3건(고친 뒤 판정 실행): ① 녹화기가 SIGINT 를 못 받아 강제 종료 → metadata 없음 · 끝 약 2 s 빠짐(→ S2 받은 비율 93 % 로 보인 원인). 스크립트 배경 작업은 SIGINT 무시로 시작하므로 TERM 으로 바꿈(녹화 스크립트 run_record_lab.sh 와 같음). ② tf2 가 탐침 안에 만드는 `transform_listener_impl_*` 노드는 매개변수 서비스가 없어 use_sim_time 조회 실패 → 판정에서 뺌. ③ 시스템 python3 에 scipy 없음(옛 eval_slam.py 가 불러옴) → 쓰는 함수만 `bagutil.py` 로 옮김(설치 변경 없음).
- 고친 채점으로 다시 본 값(참고): S3 TimePointZero 100 % · 나이 p95 0.033 s, S4 최대 0.025 s · 넘음 0, A 0.007 m · B 0.006 m(29 s 짧은 구간).

### 1단계 결과(`results/live_bright/evaluate.json`, 모의) — **판정 통과**
| 항목 | 기준 | 결과 | 예상과 비교 |
|---|---|---|---|
| S1 | /clock 앞으로만 · 노드 모두 use_sim_time true | /clock 30761개(0.017~512.68 s), 뒤로 0 · 반복 0, 노드 10개 모두 true | 예상대로 |
| S2 | ≥ 99 % · 간격 ≤ 주기 × 2 | 라이다 앞 · 뒤 5126/5127, /odom 25634/25635, 간격 최대 0.100 · 0.100 · 0.025 s | 예상대로(빠진 1개는 장면 끝 마지막 발행) |
| S3 | ≥ 99 % · p95 ≤ 0.05 s | mission_map → 앞 카메라 100 %, 나이 중앙 · p95 · 최대 0.033 s | 예상(약 0.04)보다 작음. EKF TF 주기 0.0167 s(/clock 60 Hz 마다) |
| S4 | 0.1 s 넘음 0 | 중앙 0.017 · 최대 0.025 s, 넘음 0 | 예상대로 |
| 기록 A · B | — | A RMSE 0.0136 · 최대 0.031 m, yaw 0.86°. B 0.0094 · 최대 0.027 m | 녹화 5개 범위 안(bag 밝음 A 0.042 · B 0.009). 도면 맞춤 dx −0.005 · dy 0.010 m · dyaw 0.125°, 벽 오차 1.45 → 1.13 cm |
- 도구 오류(판정 결과에는 영향 없음, 기록 보존): ① 실행 스크립트가 S1 조회(`ros2 param get`)에서 00:24 부터 멈춰, 01:02 바깥 timeout 으로 끝남 — 장면(91/91, 종료 0)과 launch · 녹화기는 정상으로 남아 있었음. 남은 단계(남은 노드 use_sim_time 조회 · 지도 저장 · 녹화기 TERM · 채점)를 손으로 이어서 함(`exit_codes.txt`). S1 의 노드 10개 값은 /clock 이 멈춘 뒤 조회(시작 때 정해지는 값, 부분 기록 `use_sim_time_partial_0024.txt` 3개도 true). 이후 스크립트 두 개에 조회마다 timeout 20 s. ② 손으로 지도 저장할 때 상대 경로로 불러 지도 그림 경로가 겹침 → `--skip-services`(pbstream 은 첫 호출 것)로 다시. 실패 기록 `*_relpath_TOOLFAIL*`.
- 한계: 모의 · 녹화와 같은 경로 1회 · 밝음만 · 장면 브리지는 Isaac Lab rclpy(native 브리지 아님) · EKF 는 IMU 없음.

## 2단계 Nav2 v11 실시간(`run_live_nav2.sh` → `make_truth_nav.py` → `261031_1325_nav2/test/evaluate.py`)
2D 모의 `run_sim.sh` 에서 모의기(sim2d.py)만 Isaac 장면(cmd_vel 모드, 출발 발제 (0.734, 0.95, π) = USD (−1.5005, −2.2636))으로 바꿈. launch 기본값 = 설계 11(nav2_params_v11 · map_mission_nav · map_objects_fine), 목표 goals_seed1_v9.json 7개, 1회.
장면에는 벽만 있음: 상자 · 신호등 · 퍽은 costmap 정적 층(map_objects_fine)에만 있고, 참값 여유(G3)는 배치 seed_1 물체까지 계산(2D 모의와 같은 geom.clearance). 라이다는 신호등 · 퍽을 못 봄(2D 모의는 봄).

| 항목 | 기준(2D 모의와 같음, 사용자 승인 2026-10-31) | 예상(실행 전) |
|---|---|---|
| G1 | 7개 모두 SUCCEEDED | 위험 큼. 2D 모의는 1차 지연 0.1 s 이상적 구동, Isaac 은 4륜 스키드라 작은 회전 명령(< 약 0.35 rad/s)에서 차체가 멈춤(녹화 T1 첫 시도 0.08 rad/s 정지). RotationShim · MPPI 의 끝무렵 작은 회전이 멈추면 진행 검사로 실패할 수 있음 |
| G2 끝 위치 · yaw | ≤ 0.10 m · ≤ 0.15 rad | 도착한 것은 0.03~0.08 m(2D 와 비슷) |
| G3 최소 여유 | > 0(겹침 0) | 0.01~0.03 m. 스키드 미끄럼으로 명령보다 덜 돌거나 밀리면 2D(0.016~0.025)보다 작아질 수 있음 |
| G4 위치 추정 | RMSE ≤ 0.05 m, 최대 ≤ 0.15 m | RMSE 0.02~0.04 m. 바퀴 odom 은 회전 1/1.15 보정 뒤에도 스키드 오차가 남아 2D(0.020~0.026)보다 큼 |
| G5 명령 | base_link, \|vx\| ≤ 0.20, \|wz\| ≤ 0.8 | 통과 |
| 함께 기록 | S1 · S2 (1단계와 같은 뜻) | 통과 |

### 2단계 첫 시도 — 도구 오류(시작 못 함, `results/live_nav_v11_TOOLFAIL_gpu_lock_leftover/`)
- 1단계를 손으로 정리할 때 static_transform_publisher(mission_map_tf) 하나를 놓침 → 실행 스크립트의 잠금 fd(8 · 9)를 물려받은 채 남아 GPU 잠금을 00:42~01:15 쥠 → 2단계가 600 s 기다리다 'GPU 잠금 실패'로 끝남. 같은 동안 인지 코드 세션의 GPU 작업(261101_0040_yolopost/equiv)도 약 33분 기다림(그쪽에 알림).
- 고침: 두 스크립트의 배경 프로세스에 `8>&- 9>&-`(자식이 잠금을 물려받지 않음), GPU 대기 600 → 3600 s. 기준 · 예상은 그대로 두고 다시 실행(GPU 잠금 순서대로).

### 2단계 결과 v11(`results/live_nav_v11/`, 모의) — **불통과**
| 항목 | 기준 | 결과 | 예상과 비교 |
|---|---|---|---|
| G1 | 7/7 SUCCEEDED | 0/7 — 6개 ABORTED 'Failed to make progress'(각 97~106 s), 마지막 TIMEOUT | 예상한 위험(작은 회전 명령에서 정지)이 그대로, 그보다 나쁨 |
| G2 | ≤ 0.10 m · 0.15 rad | 끝 위치 오차 0.17~5.6 m(목표에 못 감) | — |
| G3 | > 0 | 최소 0.028 m · 겹침 0 | 통과(거의 안 움직임) |
| G4 | RMSE ≤ 0.05 · 최대 ≤ 0.15 m | RMSE 0.084 · 최대 0.106 m, yaw RMSE 0.17 rad | 불통과. 바퀴만 돌고 차체가 안 돌아 odom yaw 가 참값과 벌어짐(참값 −2.37 rad 일 때 odom 0.69 rad 등) |
| G5 | base_link, \|vx\| ≤ 0.20, \|wz\| ≤ 0.8 | 통과(vx 최대 0.055) | — |
| 함께 | use_sim_time | Nav2 노드 모두 true. 장면 노드(isaac_live_bridge)는 false — /clock 을 내는 쪽이라 맞음 | — |
- 원인(`scene/truth.csv`): 명령 각속도가 거의 내내 ±0.04 rad/s. 참값 yaw 는 100 s 마다 한 번씩 0.5 rad 씩 툭 튐(멈춤 – 미끄러짐). RotationShim `closed_loop: true` 가 '측정 각속도 + max_angular_accel 0.8 × 0.05 s' 로 명령을 올리는데 스키드 마찰 때문에 0.04 rad/s 로는 차체가 안 돌아 측정이 0 근처 → 교착. 속도 다듬기는 OPEN_LOOP 라 원인 아님.
- Isaac 장면 종료 139(segfault, 끝난 뒤 정리 중 — truth.csv 32204줄은 온전함). 이전 녹화에서도 본 것과 같은 꼴(원인 미확인).

### 설계 12(기준 그대로) — RotationShim open loop
`261031_1325_nav2/config/nav2_params_v12.yaml` = v11 + `closed_loop: false`(이유 `nav2_params_v12_changes.md`). 1회.
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 큰 회전은 명령이 0.5 rad/s 까지 올라 넘어감. 위험: 회전 끝무렵 감속 · MPPI 의 미세 조정에서 작은 회전(< 약 0.35 rad/s)이 또 멈춰, yaw 허용 0.1 안으로 못 들어가는 목표가 생길 수 있음 → 7/7 은 반반 |
| G2 | 도착한 것 0.03~0.08 m |
| G3 | 0.01~0.03 m, 겹침 0 |
| G4 | RMSE 0.03~0.05 m(멈춤 – 미끄러짐이 줄면 odom 이 덜 벌어짐) |
| G5 | 통과 |

### 설계 12 결과(`results/live_nav_v12/`, 모의) — **불통과**
| 항목 | 기준 | 결과 |
|---|---|---|
| G1 | 7/7 | 2/7(start_box_3 · L3 성공). 4개 ABORTED 'Failed to make progress', 출발 복귀는 경로 못 만듦(추정 자세가 벽 쪽으로 밀림) |
| G2 | ≤ 0.10 m · 0.15 rad | 성공한 것 0.078 · 0.140 m(L3 는 위치 초과) |
| G3 | > 0 | **최소 −0.001 m · 겹침 334 표본** |
| G4 | RMSE ≤ 0.05 · 최대 ≤ 0.15 m | RMSE 0.089 · 최대 0.160 m |
| G5 | | 통과(vx 최대 0.194) |
- 예상과 비교: 큰 회전은 넘어감(예상대로), 작은 회전 정지 위험이 예상보다 훨씬 큼. 명령 크기별(`scene/truth.csv`, 제자리, \|v\| < 0.02):

| \|cmd_w\| (rad/s) | 시간 비율 | 참 회전 / 명령(중앙) | odom 회전 / 참 회전 |
|---|---|---|---|
| 0.02~0.10 | 45 % | 0.00 | 24.5 |
| 0.10~0.20 | 11 % | 0.03 | 1.33 |
| 0.20~0.35 | 3 % | 0.40 | 1.11 |
| ≥ 0.35 | 9 % | 0.65~0.75 | 1.0 |
  전진 명령은 그대로 따름(참 v / 명령 1.00~1.02). → MPPI 가 방향을 작은 제자리 회전으로 다듬는데 차체가 안 돌고 바퀴만 돌아 odom 회전이 부풀어 위치 추정이 밀림(G4 · G3).

### 설계 13(기준 그대로) — 제자리 회전 명령 바닥값
`261031_1325_nav2/nodes/turn_floor.py`(launch 인자 `turn_floor:=true`, 기본 꺼짐 — 고치기 전 launch 사본 `nav2_mission.launch.py.before_turn_floor_261101`). 흐름 /cmd_vel_ctrl → turn_floor → velocity_smoother. 전진 < 0.02 m/s 일 때 \|w\| < 0.05 → 0, 0.05~0.35 → ±0.35, 그 밖 그대로. 설정 = v12(closed_loop false). 1회.
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 5~7/7. 회전 정지가 거의 없어짐. 위험: ±0.35 로 올린 회전이 목표 yaw 근처에서 넘쳐(0.35 × 0.05 s = 0.0175 rad/주기, 속도 다듬기 가속 1.5 rad/s²) 왔다 갔다 할 수 있음(허용 0.1 rad 이라 크진 않을 것) |
| G2 | 0.03~0.08 m · ≤ 0.1 rad |
| G3 | 0.01~0.03 m, 겹침 0 |
| G4 | RMSE 0.02~0.05 m(바퀴만 도는 시간이 없어져 odom 회전 오차가 1.0~1.1 배로) |
| G5 | 통과(\|wz\| 0.35~0.8) |

### 설계 13 결과(`results/live_nav_v13/`, 모의) — **불통과**
| 항목 | 기준 | 결과 | 예상과 비교 |
|---|---|---|---|
| G1 | 7/7 | 3/7(start_box_1 · L1_via · L2). L1 ABORTED(진행 없음), start_box_3 · L3 · 출발 'Controller patience exceeded' | 예상(5~7)보다 나쁨 |
| G2 | ≤ 0.10 m | 성공 3개 중 2개 넘음(0.155 · 0.190 m, L2 0.028) — Nav2 는 도착이라 보지만 참값은 멀음(위치 추정 오차) | 예상보다 나쁨 |
| G3 | > 0 | **겹침 1938 표본**: 130~136 s L1 칸 place_L1 상자, 294~327 s center_stand_2 받침대(둘 다 높이 0.07 m, 라이다에 안 보이고 지도에만 있는 물체 — Isaac 장면에는 없어서 몸이 막히진 않음) | 예상(겹침 0) 밖 |
| G4 | RMSE ≤ 0.05 · 최대 ≤ 0.15 m | RMSE 0.051 · 최대 0.122 m, yaw RMSE 0.058 rad | 기준 바로 위 |
| G5 | | 통과 | |
- turn_floor: 받음 5259 · 0 으로 754 · 0.35 로 올림 1319. 제자리 회전 정지는 없어짐(명령 ≥ 0.3 때 odom/참 회전 1.04).
- 남은 원인: 위치 추정 오차가 움직일 때 큼(정지 중앙 0.027 · p95 0.077, 제자리 회전 0.056 · 0.109, 전진 0.044 · 0.098 m). **전진하며 돌 때 바퀴 odom 회전이 참값의 1.40 배**(1.15 보정은 제자리 회전에서 잰 값). 실물 팀 ekf.yaml 은 회전 속도를 바퀴가 아니라 IMU 에서만 받음 — 모의 EKF 만 바퀴 회전을 씀.

### 설계 14(기준 그대로) — EKF 를 실물 구성(바퀴 vx + IMU yaw rate)으로
- 장면: `--imu_topic /imu/data`(100 Hz, frame imu_link, 차체 각속도 z + 잡음 σ 0.0017 rad/s — 팀 모델 sensors.yaml imu.sim 값, 둘 다 가정, 바이어스 없음).
- EKF: `261031_1325_nav2/config/ekf_sim_imu.yaml`(팀 dev ekf.yaml 과 같은 입력). 나머지는 설계 13 그대로(v12 + turn_floor). 1회.
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 5~7/7. 남는 위험: L1 · L3 칸처럼 여유 0.07 m 남짓인 곳 |
| G2 | 0.03~0.08 m |
| G3 | 0.005~0.03 m. 겹침 0 기대, 다만 지도에만 있는 낮은 상자 옆은 위치 오차가 그대로 여유를 먹음 |
| G4 | RMSE 0.02~0.035 m(odom 회전 오차가 IMU 잡음 수준으로 줄어 2D 모의 0.020~0.026 에 가까워짐) |
| G5 | 통과 |
- 한계: 장면 IMU 가 이상적(바이어스 · 온도 표류 없음)이라 실물보다 좋은 조건. 통과해도 실물 IMU 바이어스 영향은 따로 확인해야 함.

### 설계 14 결과(`results/live_nav_v14/`, 모의) — **불통과**
| 항목 | 기준 | 결과 | 예상과 비교 |
|---|---|---|---|
| G1 | 7/7 | 2/7(start_box_1 · L1_via), L1 부터 'Controller patience exceeded' | 예상보다 나쁨 |
| G2 | ≤ 0.10 m | 성공 2개도 0.153 · 0.174 m | 예상보다 나쁨 |
| G3 | > 0 | 겹침 3215 표본 | 예상 밖 |
| G4 | RMSE ≤ 0.05 · 최대 ≤ 0.15 m | RMSE 0.088 · 최대 0.152 m, **yaw RMSE 0.038 rad(설계 13 0.058 보다 좋아짐)** | 위치는 예상보다 나쁨 |
- 원인 분리(`out_bag`): EKF 10 s 구간 상대 이동 오차 중앙 0.061 m(설계 13 0.029), 회전 오차는 비슷(0.019 rad). 참값 분석: **제자리 회전 때 base_link 가 옆으로 0.025 m/s 미끄러짐**(w 0.32 rad/s) → 회전 중심 x = **−0.0786 m**(설계 12 · 13 · 14 모두 같음, p10~p90 1 mm 안, 뒤 바퀴 축 −0.08 근처). 바퀴는 base_link 앞뒤 대칭(±0.08)이지만 차체가 뒤가 무거워(base_link 질량 중심 x −0.054) 뒤 바퀴를 축으로 돎. 제자리 회전 중 참 이동 합 2.0 m 를 바퀴 odom 은 0.15 m 로 봄. 설계 13 에선 바퀴 회전 과대가 우연히 일부를 메웠던 것으로 보임(확인 안 함).
- 실물도 같은지 모름 → 실물 제자리 회전 시험으로 회전 중심을 재야 함(요청 목록).

### 설계 15(기준 그대로) — 회전 중심(ICR) 보정 odom
`261031_1325_nav2/nodes/icr_odom.py`(launch `icr_odom:=true`, 기본 꺼짐): /odom 의 vx + IMU yaw rate → /odom_icr(vy = −x_icr · w, x_icr = −0.0786 — 설계 12 실행에서 잰 값, 판정 실행과 다른 실행). EKF `config/ekf_sim_imu_icr.yaml`(vx · vy + IMU yaw rate). 나머지 설계 14 그대로. 1회.
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 5~7/7 |
| G2 | 0.03~0.08 m |
| G3 | 0.005~0.03 m, 겹침 0 기대 |
| G4 | RMSE 0.02~0.035 m(EKF 10 s 상대 오차가 0.06 → 0.01~0.02 로) |
| G5 | 통과 |

### 설계 15 결과(`results/live_nav_v15/`, 모의) — **불통과**
| 항목 | 기준 | 결과 |
|---|---|---|
| G1 | 7/7 | 2/7(start_box_1 · L1_via), L1 부터 'Controller patience exceeded' |
| G2 | ≤ 0.10 m | 성공 2개도 0.175 · 0.195 m |
| G3 | > 0 | 겹침 3771 표본 |
| G4 | RMSE ≤ 0.05 · 최대 ≤ 0.15 m | RMSE 0.081 · 최대 0.128 m, yaw RMSE 0.032 rad |
| G5 | | 통과 |
- 원인 분리: 첫 제자리 회전(12~16 s)에서 **odom 만 쓴 위치 오차 0.012 m, AMCL 은 0.082 m** — odom 보정은 됐고 AMCL 이 틀린 쪽으로 끌어감.
- 스캔-지도 정합 진단(`diag_scan_map.py`, 0~100 s): 참값 자세에서 스캔 끝점 – 지도 거리 앞 1.00 · 뒤 0.82 cm(중앙), 추정 자세에서 3.58 · 3.20 cm → 지도 · 원 스캔은 맞고 AMCL 이 더 나쁜 자세에 머묾.
- **버그 발견(`261031_1325_nav2/nodes/scan_merge.py`, 이쪽이 만든 노드)**: AMCL 입력 `/scan_merged` 에 뒤 스캔이 거의 안 붙음 — Isaac v11 4972/6300 · v13 3000/3000 · v15 2076/2100 못 붙임, **2D 모의 v11_a · b 도 2233/3600 · 2400/3600**. 원인 ① 콜백 안에서 TF 를 timeout 으로 기다리는데 TF 수신이 같은 executor 라 기다리는 동안 안 들어옴 ② 뒤 스캔이 앞 스캔 직후에 와서 늘 한 주기 전 것을 붙이려 함. → AMCL 이 대부분 앞 라이다만으로 위치를 잡았음.
- 고침: TF 수신 스레드 분리(spin_thread), 같은 stamp 뒤 스캔을 0.06 s 기다려 붙임(같은 시각이면 TF 보정 없음), 못 오면 예전처럼. 고치기 전 사본 `scan_merge.py.before_spin_thread_261101`. 도구 확인(같은 bag 110 s 재현, `results/diag_v15_scan_merged_fixed2/`): **900/900 같은 시각 뒤 스캔, 못 붙임 0**(고치기 전 재현 `diag_v15_scan_merged/` 891/900 못 붙임). 중간 재현 하나는 앞 재현 노드가 남아 섞여 버림(`*_TOOLFAIL_two_instances`).
- **영향**: 2D 모의 설계 11 '3회 통과'(10-31)도 이 버그를 안은 결과 → 고친 노드로 2D 모의를 다시 확인해야 함.

### 설계 16(기준 그대로) — 설계 15 + 고친 scan_merge
나머지 설정 설계 15 그대로. 1회.
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 6~7/7. 남는 위험: L1 · L3 좁은 칸 |
| G2 | 0.03~0.07 m |
| G3 | 0.005~0.03 m, 겹침 0 |
| G4 | RMSE 0.015~0.03 m(뒤 라이다가 더해져 앞만 볼 때보다 좋아짐, 2D 모의 0.020~0.026 이하) |
| G5 | 통과 |
- 설계 16 첫 시도 도구 오류(`results/live_nav_v16_TOOLFAIL_isaac_segv/`): Isaac 장면이 시작 약 4 s(참값 197줄)에 segfault(139) — 이전 녹화에서도 가끔 나던 것, 충돌 전 단서 없음. 목표 0개 보냄. 같은 설계로 다시 실행.

### 설계 16 결과(`results/live_nav_v16/`, 모의) — **불통과**(크게 나아짐)
| 항목 | 기준 | 결과 |
|---|---|---|
| G1 | 7/7 | 5/7(L1 'Failed to make progress', 출발 복귀 'Controller patience exceeded') |
| G2 | ≤ 0.10 m | 성공 5개 중 L2(0.043)만 안, 나머지 0.129~0.184 m |
| G3 | > 0 | 겹침 662 표본 |
| G4 | RMSE ≤ 0.05 · 최대 ≤ 0.15 m | RMSE 0.063 · 최대 0.211 m, yaw 0.029 rad |
| G5 | | 통과 |
- scan_merge 고침 확인: 4200/4200 같은 시각 뒤 스캔, 못 붙임 0.
- 진단(`diag_scan_map.py` 0~450 s): 참값 자세 스캔 – 지도 0.81 cm(중앙), 추정 자세 2.56 cm → AMCL 이 덜 맞는 자세에 머묾. AMCL 레이저 모델이 기본값(sigma_hit 0.2, z_hit 0.5, z_rand 0.5)이라 cm 단위를 못 가리는 것으로 봄.

### 설계 17(기준 그대로) — AMCL 레이저 모델 좁힘
`261031_1325_nav2/config/nav2_params_v17.yaml` = v12 + sigma_hit 0.05 · z_hit 0.8 · z_rand 0.2(이유 `nav2_params_v17_changes.md`). 실행 구성 설계 16 그대로. 1회.
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 6~7/7 |
| G2 | 0.02~0.06 m |
| G3 | 0.005~0.03 m, 겹침 0 |
| G4 | RMSE 0.015~0.03 m |
| G5 | 통과 |
- 위험: σ 를 좁히면 입자가 한쪽에 몰려 잘못 수렴했을 때 못 빠져나올 수 있음(입자 500~2000, recovery 꺼짐). 실물에서는 지도에 없는 퍽 · 사람이 같은 영향을 줌.

### 설계 17 결과(`results/live_nav_v17/`, 모의) — **불통과**
| 항목 | 기준 | 결과 |
|---|---|---|
| G1 | 7/7 | 3/7(start_box_1 · L1_via · L2), L1 'Failed to make progress', 뒤로 'Controller patience exceeded' |
| G2 | ≤ 0.10 m | start_box_1 0.183 · L1_via 0.190 · L2 0.051 |
| G3 | > 0 | 겹침 3055 표본 |
| G4 | RMSE ≤ 0.05 · 최대 ≤ 0.15 m | RMSE 0.050 · 최대 0.131 m(RMSE 는 기준 경계), yaw 0.022 rad |
| G5 | | 통과 |
- start_box_1 도착 오차가 설계 13~17 모두 x −0.12~−0.18 m 로 같은 방향(체계 오차). 첫 목표 동안 바퀴 전진 거리는 참값과 같음(1.004~1.007).
- 시간별(설계 17, 0~80 s): 추정 오차는 **제자리 회전 때 생김** — 출발 첫 회전(13~16 s) 0.003 → 0.07 m, 목표에서 방향 맞추는 회전(73~79 s) 0.02 → 0.11 m. 직진(46~70 s)은 0.01 m 안팎. 같은 때 odom 만 쓴 오차는 0.01~0.05 m. AMCL yaw 가 1~2° 어긋난 채 위치와 맞바꿈.
- 정리: 원인이 odom · 스캔에서 'AMCL 의 제자리 회전 때 동작'으로 좁혀짐. 같은 Isaac 장면에서 Cartographer 는 실시간 0.0136 m(1단계).

## 설계 11~17 요약(2단계, 모두 모의 · 1회씩, 기준 그대로)
| 설계 | 바꾼 것 | G1 | G4 RMSE(m) | 찾은 원인 |
|---|---|---|---|---|
| 11 | (2D 모의 통과 설정) | 0/7 | 0.084 | RotationShim closed_loop: 측정 각속도 0 → 명령 0.04 rad/s 에 묶임 |
| 12 | closed_loop false | 2/7 | 0.089 | 제자리 회전 명령 ≤ 0.1 rad/s 가 45 %, 차체 안 돎 |
| 13 | + turn_floor(제자리 \|w\| < 0.05 → 0, 0.05~0.35 → 0.35) | 3/7 | 0.051 | 전진하며 돌 때 바퀴 odom 회전 1.40 배 |
| 14 | + IMU yaw rate EKF(팀 ekf.yaml 구성) | 2/7 | 0.088 | 제자리 회전 중심 x −0.0786 m(뒤 바퀴 축) |
| 15 | + icr_odom(vy = −x_icr · w) | 2/7 | 0.081 | scan_merge 버그(뒤 스캔 거의 못 붙임, 2D 모의 v11 도) |
| 16 | + scan_merge 고침 | 5/7 | 0.063 | AMCL 레이저 모델 기본값 |
| 17 | + sigma_hit 0.05 · z_hit 0.8 · z_rand 0.2 | 3/7 | 0.050 | AMCL 이 제자리 회전 때 5~10 cm 밀림 |
도구 오류(판정 제외, 보존): v11 첫 시도 GPU 잠금 남은 프로세스, v16 첫 시도 Isaac segfault.

## 설계 18(기준 G1~G5 그대로) — Cartographer 순수 위치 추정(사용자 결정 2026-11-01, AMCL 대신)
- 위치 추정: Cartographer(`261031_1325_nav2/config/carto_localization_v18.lua` = 1단계 carto_candidate.lua + pure_localization_trimmer 3 · optimize_every_n_nodes 20, 공식 예제 backpack_2d_localization 값), 지도 = 1단계 `results/live_bright/map/map.pbstream`(frozen). 시작 자세 = `nodes/carto_start.py` 가 /start_trajectory(기준 궤적 0 첫 노드에 대한 상대 자세)로 출발 발제 (0.734, 0.95, π) 를 align.json(0.729, 0.960, 3.1438)으로 바꿔 줌. map→odom = map_odom_relay(1단계와 같은 노드). AMCL · scan_merge 안 띄움.
- TF = 1단계와 같은 꼴: mission_map →(정적) map →(relay) odom →(EKF) base_link. Nav2 global frame · 지도 frame = mission_map(`nav2_params_v18.yaml` = v12 + frame 5곳, 이유 `nav2_params_v18_changes.md`). 목표 frame mission_map(`run_goals.py --frame`).
- 유지: v12(closed_loop false), turn_floor, icr_odom, IMU EKF(ekf_sim_imu_icr.yaml). launch 인자 `localization:=carto`(기본 amcl, 고치기 전 사본 `nav2_mission.launch.py.before_carto_loc_261101`), run_goals 사본 `.before_frame_261101`, 실행 스크립트 사본 `run_live_nav2.sh.before_carto_loc_261101`(7번째 인자 run_goals 추가 · Cartographer install source · /tracked_pose 녹화).
- 도구 확인(장면 없이 40 s, `results/TOOLCHECK_carto_start_noscene/`, 판정 아님): pbstream 노드 2194개 읽음, 새 궤적 1 시작 성공(상대 (−0.003, 0.022, −0.006)). 첫 시도는 gflags bool 인자를 `-flag false` 로 써서 기본 궤적이 먼저 시작 → 'Topics are already used'(`*_TOOLFAIL_gflags_bool`), `-flag=false` 로 고침.

| 항목 | 예상(실행 전) |
|---|---|
| G1 7/7 | 5~7/7. 위치 추정이 1단계 수준(약 1~2 cm)이면 설계 16 의 실패 원인(도착 오차 · 회전 때 밀림)이 사라짐. 남는 위험: L1 좁은 칸 회전 여유, 최적화 때 map→odom 이 몇 cm 툭 바뀌어 controller 가 흔들릴 수 있음 |
| G2 끝 위치 ≤ 0.10 m · yaw ≤ 0.15 rad | 0.02~0.06 m(도착 허용 0.06 안팎) |
| G3 최소 여유 > 0 | 0.005~0.03 m, 겹침 0 |
| G4 RMSE ≤ 0.05 · 최대 ≤ 0.15 m | RMSE 0.01~0.03 m(1단계 실시간 0.0136 m, 같은 Cartographer 설정 · 같은 장면), 최대 0.03~0.08 m |
| G5 | 통과(명령 경로 안 바뀜) |

### 설계 18 결과(`results/live_nav_v18/`, 모의) — **불통과**
| 항목 | 기준 | 결과 |
|---|---|---|
| G1 | 7/7 | 0/7(3개 'Resulting plan has 0 poses', 4개 'Controller patience exceeded') |
| G2 | ≤ 0.10 m | 목표에 못 감(0.11~5.6 m) |
| G3 | > 0 | 최소 0.085 m · 겹침 0(거의 안 움직임) |
| G4 | RMSE ≤ 0.05 · 최대 ≤ 0.15 m | RMSE 3.82 · 최대 5.08 m, yaw RMSE 3.06 rad |
| G5 | | 통과 |
- 원인(`out_bag` /tracked_pose vs /sim/truth): 0~12 s 는 맞음(시작 (−0.009, 0.010, −0.002), 첫 회전 +0.40 rad 참값 +0.43). **14 s 에 Cartographer map (−3.015, −2.482, −2.32) 로 튐** — yaw 가 π 뒤집힌 엉뚱한 자리. 참 자세는 출발 상자 안 그대로.
- 소스 확인(cartographer `pose_graph_2d.cc` 276~291): 저장 궤적과의 제약은 '두 궤적의 마지막 연결 + global_constraint_search_after_n_seconds' 안이면 지역 탐색(현재 추정 둘레 창), 아니면 지도 전체 탐색. **초기 자세(/start_trajectory initial_pose)는 연결로 치지 않아**(연결 시각 기본값 0) 첫 연결은 늘 지도 전체 탐색 → 출발 상자처럼 단순한 칸은 다른 자리와 점수 0.7 을 넘겨 잘못 맞음. 예상 표에 없던 위험.

### 설계 19(기준 그대로) — 지도 전체 탐색 끔 · 지역 탐색 창 1 m
`261031_1325_nav2/config/carto_localization_v19.lua` = v18 + `POSE_GRAPH.global_constraint_search_after_n_seconds = 1e11`(노드 시각은 0001 년 기준 약 6.4e10 s → 늘 지역 탐색) · `constraint_builder.fast_correlative_scan_matcher.linear_search_window = 1.0`(기본 7 m, 각도 창 ±30° 그대로). 위치를 잃었을 때 다시 찾는 일은 Cartographer 전역 탐색에 맡기지 않고 따로 만듦(다음 할 일 2). 나머지 설계 18 그대로. 도구 확인(장면 없이, `results/TOOLCHECK_carto_v19_noscene/`): 설정 읽힘 · 새 궤적 1 시작.
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 5~7/7. 남는 위험: L1 좁은 칸, 최적화 때 map→odom 몇 cm 바뀜 |
| G2 | 0.02~0.06 m |
| G3 | 0.005~0.03 m, 겹침 0 |
| G4 | RMSE 0.01~0.03 m, 최대 0.03~0.08 m(1단계 실시간 0.0136) |
| G5 | 통과 |

### 설계 19 결과(`results/live_nav_v19/`, 모의) — **불통과**(위치 추정은 통과 수준)
| 항목 | 기준 | 결과 |
|---|---|---|
| G1 | 7/7 | 2/7(start_box_1 · L1_via). L1 'Controller patience exceeded' 뒤 그 자리에 멈춰 나머지 4개 실패 |
| G2 | ≤ 0.10 m | 성공 2개 0.170 · 0.188 m |
| G3 | > 0 | 최소 −0.001 m · 겹침 3070 표본 |
| G4 | RMSE ≤ 0.05 · 최대 ≤ 0.15 m | **RMSE 0.0051 · 최대 0.0106 m, yaw 0.0074 rad**(예상 0.01~0.03 보다 좋음) |
| G5 | | 통과 |
- 위치 추정 확인: 목표 끝 시각마다 추정(mission_map→base_link)과 참값 차이 3~5 mm. 지도 전체 탐색을 끈 뒤 튐 없음.
- 끝 위치 오차 원인(`/plan` 끝점은 목표와 0.000 m): Nav2 가 '도착'이라 한 순간 추정 자세가 이미 목표에서 0.169 · 0.191 m. 도착 판정이 stateful(위치 0.06 m 안을 잡아 둠) → 방향 맞추는 제자리 회전 동안 회전 중심(x −0.0786 m, 설계 14)을 축으로 base_link 가 최대 0.157 m 밀림. AMCL 설계 13~17 의 start_box_1 체계 오차(x −0.12~−0.18 m)도 같은 원인으로 설명됨.

### 설계 20(기준 그대로) — Nav2 제어점을 회전 중심으로
`nav2_params_v20.yaml` = v18 + robot_base_frame rotation_center(4곳) · footprint +0.0786 m(이유 `nav2_params_v20_changes.md`). launch `rotation_center_tf:=true turn_floor_out_frame:=base_link`, 목표 `--base_offset_x -0.0786`. Cartographer 는 설계 19(carto_localization_v19.lua) 그대로. 고치기 전 사본: launch `.before_rotation_center_261101`, turn_floor `.before_out_frame_261101`, run_goals `.before_base_offset_261101`.
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 5~7/7. 남는 위험: 회전 중심 기준으로 돌면 앞 끝이 더 크게 휘돌아(반지름 0.316 → 0.382 m) L1 · L3 좁은 칸에서 막힐 수 있음 |
| G2 | 0.02~0.07 m(회전으로 밀리던 0.157 m 가 없어짐) |
| G3 | 0~0.03 m. 위험: L1 · L3 회전 구간 규칙은 base_link 중심으로 계산 → 겹침이 생길 수 있음 |
| G4 | RMSE 0.005~0.015 m |
| G5 | 통과(header 는 turn_floor 가 base_link 로) |

### 설계 20 결과(`results/live_nav_v20/`, 모의) — **불통과**
| 항목 | 기준 | 결과 |
|---|---|---|
| G1 | 7/7 | 1/7(L2 만). start_box_1 · L1_via · L1 'Failed to make progress', 나머지 'Controller patience exceeded' |
| G2 | ≤ 0.10 m | L2 0.043 m(성공 1개는 안) |
| G3 | > 0 | 최소 0.0045 m · 겹침 0 |
| G4 | RMSE ≤ 0.05 · 최대 ≤ 0.15 m | RMSE 0.0046 · 최대 0.0133 m |
| G5 | | 통과 |
- 원인: 최종 /cmd_vel_nav 제자리 회전 명령의 **69 %가 0.35 rad/s 아래**(설계 19 26 %) → 차체가 안 돌고 출발 상자에서 진행 없음. turn_floor 는 속도 다듬기 앞에 있고, 그 뒤 collision_monitor(approach, 1.2 s)가 좁은 상자에서 회전을 줄임. 또 collision_monitor 의 base_frame 을 base_link 로 남겨 회전 예측이 실제 회전 중심과 어긋남 — v20_changes.md 의 '바꿀 것 없음'은 틀린 판단.

### 설계 21(기준 그대로) — 충돌 감시도 회전 중심 기준 · turn_floor 맨 끝
`nav2_params_v21.yaml` = v20 + `collision_monitor.base_frame_id: rotation_center` · `cmd_vel_out_topic: cmd_vel_cm`. launch `turn_floor_last:=true`(흐름: controller → /cmd_vel_ctrl → velocity_smoother → collision_monitor → /cmd_vel_cm → turn_floor → /cmd_vel_nav, 사본 `.before_turn_floor_last_261101`). turn_floor 는 0.05 아래는 0 으로 두므로 충돌 감시의 정지(0)는 그대로 남고, 0.05~0.35 로 줄인 회전만 0.35 로 올림. 위험: 충돌 감시가 줄인 회전을 다시 올리므로 그 1.2 s 예측보다 빨리 닿을 수 있음 — 차체가 0.35 아래로는 안 돌기 때문에 줄여도 회전이 아니라 정지가 되어 진행이 막히던 것. 도구 확인(장면 없이 세 경우, `results/TOOLCHECK_turn_floor_last_noscene/`): last = 속도 다듬기 /cmd_vel_ctrl · turn_floor /cmd_vel_cm → /cmd_vel_nav, 기존 turn_floor:=true · false 는 예전과 같은 연결. 첫 시도는 PythonExpression 에 true 가 그대로 들어가 오류(`*_TOOLFAIL_pyexpr`).
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 4~7/7. 남는 위험: L1 · L3 좁은 칸(회전 구간 규칙이 base_link 중심 계산) |
| G2 | 0.02~0.07 m |
| G3 | 0~0.03 m. 겹침 위험은 L1 · L3 |
| G4 | RMSE 0.005~0.015 m |
| G5 | 통과 |

### 설계 21 결과(`results/live_nav_v21/`, 모의) — **불통과**
| 항목 | 기준 | 결과 |
|---|---|---|
| G1 | 7/7 | 2/7(start_box_1 0.040 m · L1 0.052 m). 나머지 5개 'Failed to make progress' |
| G2 | ≤ 0.10 m · 0.15 rad | 성공 2개는 안(yaw 0.004 · 0.001 rad) — 회전으로 밀리던 끝 위치 오차가 없어짐 |
| G3 | > 0 | 최소 0.011 m · 겹침 0 |
| G4 | RMSE ≤ 0.05 · 최대 ≤ 0.15 m | RMSE 0.0049 · 최대 0.021 m |
| G5 | | 통과 |
- 막힌 곳은 모두 L1 칸 둘레(참 여유 0.04~0.07 m): L1_via 입구에서 돌다가, L1 에서 나가려다(L2 · 이후 목표) MPPI 회전 명령이 |w| 0.01~0.07 rad/s 로 거의 0 → 차체 정지. 예상 표의 위험(회전 중심으로 돌면 앞 끝이 더 크게 휘돎) 그대로.
- 지금 목표의 한 바퀴 회전 여유(회전 중심 축 / base_link 축): L1_via −0.001 / 0.082, L1 −0.001 / 0.004, L3 −0.001 / 0.008, 출발 복귀 −0.001 / 0.031 m(−0.001 = 겹침). 설계 8 · 9 의 회전 구간 규칙은 base_link 축으로 계산한 것이라 실제 회전에서는 돌 자리가 없음.

### 설계 22(기준 그대로) — 좁은 자리 목표를 회전 중심 규칙으로 다시
`261031_1325_nav2/tools/make_goals_v22.py` → `config/goals_seed1_v22.json`: L1_via(0 → π/2) · L1 · L3(들어올 때 π/2 → 태그, 나갈 때 태그 → 3π/2) · 출발 복귀(π ± 45°) 를 회전 중심 축 회전 구간 여유 ≥ 0.04 m 로 다시 고름(0.07 → 0.04 이유: 0.07 은 AMCL 최대 오차 0.066 기준, Cartographer 최대 0.021 m 의 약 2배). 나머지 설계 21 그대로.
- 문턱값 정정(실행 전): 0.04 로 돌리니 L1 에서 법선 ≤ 15° 를 지키는 자세가 없음(0.04 이상은 법선 22~24°, `results/make_goals_v22.log`). 법선 15° 는 태그 읽기 조건이라 넓히지 않고, 문턱을 위치 추정 최대 오차(0.021 m) 이상인 **0.025 m** 로 둠. 결과: L1 (4.158, 0.895, 3.000) → (4.183, 0.950, π) 회전 여유 들어올 때 0.0285 · 나갈 때 0.0277, L3 (4.163, 5.354, π) → (4.183, 5.354, π) 0.0285 · 0.0363. L1_via · 출발 복귀는 그대로(회전 중심 여유 0.092 · 0.114).
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 4~7/7. 남는 위험: L1 · L3 회전 여유 0.028 m 가 MPPI costmap 팽창(inflation)보다 작아 controller 가 회전을 꺼릴 수 있음, L1_via 근처에서 지나친 뒤 도는 경우 |
| G2 | 0.02~0.06 m |
| G3 | 0~0.03 m, 겹침 0 |
| G4 | RMSE 0.005~0.015 m |
| G5 | 통과 |
- 설계 22 첫 시도 도구 오류(`results/live_nav_v22_TOOLFAIL_isaac_segv/`): Isaac 장면이 약 34 s 에 segfault(139, crashreporter 기록) → 목표 실행기를 손으로 끝냄(INT). 같은 설계로 다시 실행.
- 함께 찾은 도구 문제: 04:13 장면 없이 한 도구 확인 중 launch 가 PythonExpression 예외로 죽으며 자식 노드 21개(ROS 도메인 103, use_sim_time false)가 남아 있었음(05:2x 정리). 도메인이 달라(LOCALHOST · 103 ≠ 102) 설계 21 · 22 토픽에는 섞이지 않았고, GPU 잠금도 쥐지 않음. 이후 도구 확인 뒤에는 도메인별 남은 프로세스를 확인.
- 설계 22 두 번째 시도도 도구 오류(`results/live_nav_v22_TOOLFAIL_isaac_segv2/`, 모의 약 377 s 에 segfault 139). 두 시도 모두 같은 시간에 이쪽이 CPU 를 많이 쓰는 오프라인 계산 · bag 재생(재위치 도구 확인)을 돌리고 있었음 → 이후 Isaac 시험 중엔 무거운 작업을 같이 돌리지 않음. 부분 기록(판정 아님): start_box_1 ABORTED(출발 회전 뒤 여유 0.018 m 에서 MPPI 명령 |w| 0.02 · v 0.003 으로 멈춤), L1_via 성공 0.078 m, L1 ABORTED(회전 중 여유 0.045 → 0.001 m, 그 뒤 MPPI 멈춤).
- 참고 측정(참값, 설계 19~21): 제자리 회전 중심 x −0.0777 ~ −0.0788 m(회전 속도 0.1~0.9 rad/s 모두 같음), 실제 회전 속도 / 명령 = 0.53(|w| 0.1~0.3) · 0.65(0.3~0.4) · 0.72~0.75(≥ 0.4).
- 참고 계산(make_goals_v22 규칙, 회전 중심 축): L1 최대 회전 여유 = 법선 ≤ 15° 0.028 · ≤ 20° 0.035 · ≤ 25° 0.043 m, L3 0.036 · 0.043 · 0.048 m. 카메라 거리 상한을 0.75 m 까지 넓혀도 같음(칸 모양이 막음).

### 설계 22 결과(`results/live_nav_v22/`, 세 번째 시도, 다른 작업 없이, 모의) — **불통과**
| 항목 | 기준 | 결과 |
|---|---|---|
| G1 | 7/7 | 2/7(start_box_1 0.041 m · L2 0.043 m). L1_via · L1 · L3 · 출발 'Failed to make progress', start_box_3 'Controller patience exceeded' |
| G2 | ≤ 0.10 m · 0.15 rad | 성공 2개는 안 |
| G3 | > 0 | 최소 −0.001 · 겹침 7 표본 |
| G4 | RMSE ≤ 0.05 · 최대 ≤ 0.15 m | RMSE 0.0062 · 최대 0.0215 m |
| G5 | | 통과 |
- 실패는 모두 좁은 곳(L1 칸 둘레 · 출발 상자 · L3 칸)에서 MPPI 명령이 거의 0 이 되는 멈춤. 예상 표의 위험(회전 여유 0.028 m 가 작음) 그대로. 실제 회전 속도가 명령의 53~75 % 라 MPPI 예측과 실제 움직임이 어긋나 회전 중 몸체가 물체 쪽으로 밀림(부분 기록 L1: 여유 0.045 → 0.001 m).

### 설계 23(기준 그대로) — 회전 속도 되먹임 + 태그 각도 25°
- turn_floor `yaw_loop:=true`: IMU yaw rate 되먹임 PI(kp 0.5 · ki 3.0 · 적분 ±0.4 rad/s · |w| ≤ 0.8), 명령 w 가 0 이거나 부호가 바뀌면 적분 0(사본 `turn_floor.py.before_yaw_loop_261101` · launch `.before_yaw_loop_261101`). 바퀴 각속도 되먹임은 원래 구동(제어 파트) 계층 일 — 제어 Issue 에 넣을 것.
- 목표 `goals_seed1_v23.json`(make_goals_v22.py --max_off 25): 인지 세션 합성 확인(261101_0600_tagangle, 30° 까지 읽힘 · 25° 가 여유 있는 선). L1 (4.173, 0.800, 2.783) 법선 20.6° · 카메라 0.488 m · 회전 여유 0.038 / 0.041, L3 (4.183, 5.204, 2.791) 20.1° · 0.497 m · 0.048 / 0.044.
- 두 가지를 한 번에 바꾼 이유: 한 번 실행에 약 25 분, 두 원인(회전 속도 어긋남 · 작은 회전 여유)이 모두 기록으로 확인됨. 나머지 설계 22 그대로.
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 4~7/7. 위험: PI 가 MPPI 의 빠른 명령 변화와 엇갈려 회전이 넘침(목표 yaw 근처 왔다 갔다), 출발 상자 첫 회전 |
| G2 | 0.02~0.06 m |
| G3 | 0~0.03 m, 겹침 0 |
| G4 | RMSE 0.005~0.015 m |
| G5 | 통과(|wz| ≤ 0.8 로 묶음) |

### 설계 23 결과(`results/live_nav_v23/`, 모의) — **불통과(중단)**
- 목표 2개(start_box_1 · L1_via) 모두 'Controller patience exceeded' 로 G1 통과가 불가능해져 약 120 s 에 목표 실행기를 끝냄(run_goals_exit 130, Isaac 정상 종료 0). 채점은 2개 기준: G3 겹침 290 표본, G4 RMSE 0.0057 · 최대 0.024 m, G5 통과.
- 원인(`scene/truth.csv`): yaw_loop PI 가 회전 명령을 ±0.8 rad/s 한계로 몰고 부호가 뒤집힘(12 s +0.8 → 15 s −0.77 → 18 s +0.8, 54~66 s 도 같음) — 예상 표의 위험(PI 와 MPPI 의 빠른 명령 변화가 엇갈려 넘침) 그대로. 실제 회전은 명령보다 늦게 따라가 출발 상자 · L1_via 둘레에서 왔다 갔다 함.
- 다음 후보(설계 24): PI 대신 정적 앞먹임(제자리 회전 명령 ÷ 측정 비 0.65 근처, 상한 0.8) 또는 PI 게인을 크게 낮춤(kp 0 · ki 0.5, 제자리 회전에만). 태그 각도 25° 목표(goals_seed1_v23)는 그대로 씀.

### 설계 24(기준 그대로) — PI 대신 제자리 회전 정적 앞먹임
- turn_floor `turn_ff:=true` · `yaw_loop:=false` · `turn_floor_w_min:=0.19`(사본 `turn_floor.py.before_ff_261101` · launch `.before_ff_261101`): 제자리(|v| < 0.02)일 때 바닥값을 거친 원하는 회전 |w| 를 측정 곡선의 역함수로 명령으로 바꿈. 곡선 = 설계 21 · 22 참값(`scene/truth.csv`)에서 명령이 1 s 동안 같은 자유 회전 구간의 실제/명령 중앙값: 명령 0.35 → 실제 0.186 · 0.55 → 0.358 · 0.8 → 0.60(설계 21 · 22 같음). 대응: 원하는 0.19 → 명령 0.355 · 0.35 → 0.54 · 0.5 → 0.70 · ≥ 0.6 → 0.8(상한). w_min 0.19 = '원하는 실제 회전' 바닥(명령으로는 지금까지와 같은 0.35).
- 참고: 설계 21 의 명령 0.4~0.5 · 실제 0 구간(63~116 s, 54 s)은 출발 상자 옆 (−1.27, −2.88)에서 몸체가 막혀 못 돈 것 → 곡선에서 뺌(자유 회전 아님). 이 막힘은 앞먹임으로 안 풀림.
- 나머지 설계 23 그대로(goals_seed1_v23 태그 각도 25°, nav2_params_v21, Cartographer v19).
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 3~7/7. 실제 회전이 MPPI 예측과 맞아 회전 중 밀림이 줄 것. 위험: 곡선이 정상 상태 값이라 회전 시작 · 멈춤 지연은 남음, 출발 상자 막힘(설계 21 같은 접촉)은 안 풀림 |
| G2 | 0.02~0.06 m |
| G3 | 0~0.03 m, 겹침 ≤ 설계 22 |
| G4 | RMSE 0.005~0.015 m |
| G5 | 통과(|wz| ≤ 0.8) |

### 설계 24 결과(`results/live_nav_v24/`, 모의) — **불통과(중단)**
| 항목 | 기준 | 결과 |
|---|---|---|
| G1 | 7/7 | 1/4(start_box_1 0.045 m). L1_via · L1 · L2 'Failed to make progress' → G1 통과 불가라 목표 4개 뒤 목표 실행기를 끝냄(run_goals_exit 130) |
| G3 | > 0 | 최소 0.0036 m, 겹침 0 |
| G4 | RMSE ≤ 0.05 · 최대 ≤ 0.15 m | RMSE 0.0060 · 최대 0.0162 m |
| G5 | | 통과(|wz| ≤ 0.8) |
- 앞먹임은 뜻대로 됨: 원하는 0.5(RotationShim 회전 속도) → 명령 0.70 → 실제 약 0.5 rad/s(참값 yaw 변화). 설계 23 같은 명령 흔들림 없음.
- **L1_via 실패 원인 = 목표 생성 도구(make_goals_v22.py) 기하 오류.** sweep_rc 는 base_link 가 (x, y) 에 '도착 방향' y0 으로 있을 때의 회전 중심을 축으로 회전 여유를 셌는데, 목표 파일의 (x, y, yaw) 는 최종 자세이고 run_goals `--base_offset_x` 가 최종 yaw 로 회전 중심 목표를 만듦 → 두 회전 중심이 0.0786·|e^{i·y0} − e^{i·yaw}| 어긋남(L1_via 0.111 m, L1 · L3 약 0.09 m > 도착 허용 0.06 m). 기록: 회전 중심은 (4.01, 0.40) 에 거의 고정된 채 돌기만 했고 목표 회전 중심 (4.08, 0.33) 까지 0.098 m 를 옮기지 못함 → Shim(목표 방향) · MPPI(목표 위치)가 번갈아 ±0.5 회전(4~5 s 주기). 같은 자리 기하 회전 여유는 + 쪽 ≥ 0.093 m, − 쪽은 −1.0 rad 에서 0.021 m.
- 설계 21~23 의 L1 · L3 · L1_via 실패에도 같은 어긋남이 섞여 있었을 수 있음(설계 22 · 23 목표 모두 이 도구).

### 설계 25(기준 그대로) — 목표를 '최종 자세 회전 중심' 규칙으로 다시
- `261031_1325_nav2/tools/make_goals_v25.py --max_off 25` → `config/goals_seed1_v25.json`(로그 `tools/make_goals_v25.log`). 회전 여유를 최종 자세의 회전 중심(= run_goals 가 보내는 목표 점)을 축으로 셈. 문턱 0.025 m · 법선 ≤ 25° · 카메라 0.35~0.55 m 그대로.
  - L1_via (4.08, 0.41) → (4.06, 0.41, π/2): 0 → π/2 회전 여유 0.033 m(v22 계산 0.092 는 다른 회전 중심 기준이라 틀린 값).
  - L1 (4.173, 0.800, 2.783) → (4.143, 0.800, 2.756): 법선 22.1° · 카메라 0.46 m · 회전 여유 들어올 때 0.068 · 나갈 때 0.068 m.
  - L3 (4.183, 5.204, 2.791) → (4.153, 5.204, 2.766): 21.5° · 0.469 m · 0.074 · 0.074 m. 출발 복귀 · 나머지 목표 그대로.
- 나머지 설계 24 그대로(turn_ff 앞먹임 유지).
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 4~7/7. L1 · L3 는 회전 중심이 목표에 놓인 채 돌면 끝나 나아질 것. 위험: L1_via 회전 여유 0.033 m 가 작음, 회전 중심을 목표 점(허용 0.06 m)까지 옮기는 마지막 전진 · 후진, L2 · start_box_3 는 이번 변경과 무관 |
| G2 | 0.02~0.06 m |
| G3 | 0~0.03 m, 겹침 0 |
| G4 | RMSE 0.005~0.015 m |
| G5 | 통과 |

### 설계 25 결과(`results/live_nav_v25/`, 모의) — **불통과(중단)**
- start_box_1 TIMEOUT(240 s, 끝 오차 1.29 m) → 목표 실행기 중단(run_goals_exit 130). 27.5 s 부터 발제 (0.894, 0.407) 에서 몸체가 멈추고 바퀴만 돎(바퀴 odom 0.1 m/s 로 늘어남, 참 자세 그대로) = 물리 막힘. 이번 변경(L1_via · L1 · L3 목표)과 무관한 자리.
- **(정정 — 아래 '설계 25 원인 확인' 절. 이 줄의 '바퀴 폭' 추정은 측정으로 틀린 것으로 확인됨)** 처음 추정: Isaac 물리의 바퀴 충돌 폭이 footprint 보다 넓음. 출발 통로의 wall_d0 끝 (1.07, 0.543)(두께 3 mm, 높이 0.005~0.385 전부 같은 자리, USD 충돌 메시 확인)이 로봇 좌표 (앞 0.08~0.115, 왼쪽 **0.210**) 에 고정된 채 로봇이 그 점을 축으로 끌려 돎 — 설계 21(55~70 s) · 22(40~52 s) · 25(23~27 s) 세 번 같은 자리. 이 x 범위는 왼쪽 앞바퀴(x 0.047~0.113). 그러나 로봇 USD 충돌 prim 의 경계 상자 최대 |y| 는 0.1676(바퀴 tire_cylinder, 정확한 Cylinder · 폭 ±0.0091) · URDF 도 같음, 팔(집게 끝 반경 0.167 m) 로도 못 닿음, 로봇 · 미로 USD 에 rest/contact offset 지정 없음. → PhysX 가 바퀴 원기둥을 다루는 방식에서 옆으로 약 0.04 m 더 차지하는 것으로 보임(직접 측정은 아직).
- 영향: footprint(±0.171) 기준 여유 0.03~0.04 m 인 곳(출발 통로 · L1_via 회전 0.033 m)은 Isaac 에서 실제로 닿음. 실물 바퀴 바깥면은 0.168(URDF, AGENTS.md 바퀴 폭 가운데 ±0.1585) 이라 이 0.04 는 모의 모델 쪽 문제.

### 설계 25 원인 확인(도구 확인, `results/TOOLCHECK_robot_width/`, 모의) — 바퀴 폭 추정 틀림, 원인은 미로 기둥
- 차체 물리 폭 직접 측정(`probe_robot_width.py`): 왼쪽에 고정 벽(안쪽 면 y 0.15)을 겹치게 세우자 로봇이 y −0.0202 로 밀려남 → 왼쪽 물리 폭 **0.170 m**(모델 0.168 과 같음, 전체 약 0.34 m). 벽 0.19 에선 안 밀림. 'Isaac 바퀴 폭 ±0.21(전체 0.42)' 추정은 틀림(사용자 지적 2026-11-01). PhysX 광선 쿼리는 이 설정에서 로봇을 못 봄(맞은 점 0).
- 막힘 재현(`probe_hook_replay.py`, 설계 25 t 22.5 s 자세 · 회전 명령 0.70): 같은 끌림(실제 회전 약 0.1 rad/s, 5.6 s 에 튐) 재현, 팔은 그대로(joint1 ≈ 0).
- **원인 = 미로 충돌 원기둥(벽 끝 · 모서리 프로파일 기둥) 110 개(자리 70, 반지름 0.0303 m, 높이 0~0.39)가 geom 벽 자료에 없었음.** `261030_1943/slice_maze.py` 가 충돌 Mesh 만 잘라 `maze_slice_z*.json` 에 원기둥이 빠짐. 걸린 것 = wall_d0 끝 기둥 (1.0705, 0.5305)(판 끝 0.543 보다 아래 0.500 까지). 차체 0.170 + 기둥 반지름 0.030 ≈ 기록의 0.207~0.210.
- 라이다 · SLAM 지도(map_mission)에는 기둥이 칸(0.025 m) 만큼 깎여 들어가 있음(벽 끝 점유 y ≈ 0.515). 물체 지도(map_objects_fine)에는 없음.
- 고친 것: `261030_1943/slice_maze_posts.py` → `maze_posts.json`, `261031_1325_nav2/nodes/geom.py` 에 기둥(외접 16각형, 라이다 · 몸체, 사본 `geom.py.before_posts_261101`), `tools/make_objects_fine.py --posts` → `config/map_objects_fine_posts.yaml`(사본 `.before_posts_261101`, 기둥 없이 만들면 옛 지도와 바이트 같음).
- **G3 정정(기둥 포함 다시 계산, scene/truth.csv 0.1 s 표본)**: 설계 21 최소 −0.001 · 겹침 670(전 판정 '겹침 0' 틀림), 22 겹침 161, 24 겹침 96(전 판정 G3 통과 틀림), 25 겹침 2634. 설계 22~25 의 목표 회전 여유(make_goals_v22 · v25)도 기둥 없이 계산된 값.

### 설계 26(기준 그대로) — costmap 물체 지도와 목표에 기둥 넣기
- launch `objects_map:=…/config/map_objects_fine_posts.yaml`(전역 · 지역 costmap 둘째 정적 층에 기둥 70 자리 정확히). 목표 `goals_seed1_v26.json`(make_goals_v25.py --max_off 25, 기둥 포함 geom, 로그 `tools/make_goals_v26.log`):
  L1_via (4.05, 0.39, π/2) 회전 여유 0.025 m(문턱과 같음) · L1 (4.133, 0.800, 2.747) 법선 22.6° · 0.451 m · 0.055 / 0.055 · L3 (4.143, 5.464, −2.853) 16.6° · 0.447 m · 0.064 / 0.065. 나머지 설계 25 그대로.
- 채점 G3 는 기둥 포함 geom 으로 계산(make_truth_nav 가 geom 을 씀).
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 3~7/7. 출발 통로 기둥을 costmap 이 정확히 알아 MPPI 가 피할 것. 위험: L1_via 회전 여유 0.025 m(위치 추정 최대 오차 ~0.02 와 비슷), MPPI 가 모서리를 깎는 버릇(footprint 는 중심 비용 높을 때만 검사) |
| G2 | 0.02~0.06 m |
| G3 | 0~0.03 m, 겹침 0 (기둥 포함) |
| G4 | RMSE 0.005~0.015 m |
| G5 | 통과 |

### 설계 26 결과(`results/live_nav_v26/`, 모의) — **불통과(중단)**
| 항목 | 기준 | 결과 |
|---|---|---|
| G1 | 7/7 | 0/3(start_box_1 'Controller patience exceeded', 뒤 목표는 막힌 자리에서 시작해 실패) → 목표 실행기 중단(INT 가 action 요청 중에 들어가 run_goals_exit 1 · TypeError, 도구 쪽, 채점 영향 없음) |
| G3 | > 0 | 최소 −0.001 · 겹침 3479 표본(기둥 포함 채점) |
| G4 | RMSE ≤ 0.05 · 최대 ≤ 0.15 m | RMSE 0.0038 · 최대 0.0109 m |
| G5 | | 통과 |
- 물체 지도는 기둥 포함판이 실렸음(launch.log `map_objects_fine_posts.pgm`). 그래도 같은 기둥에 닿음: 남향 통로(x ≈ 0.81)를 내려오다 base y ≈ 0.53 에서 왼쪽(동쪽)으로 제자리 회전 시작(19 s~), 여유 0.054 → 0.024(22.0 s) → 닿음(22.5 s, 명령 0.70 = RotationShim 0.5 앞먹임). behavior spin · backup 은 'Collision Ahead' 로 멈춤.
- 해석: 전역 계획기가 NavFn(점 로봇, 차체 모양 · 방향 모름)이라 기둥 쪽 안쪽 모서리를 깎는 경로를 냄. 이 ㄱ자 모퉁이(통로 폭 약 0.46 m → 틈 높이 0.48 m, 기둥이 안쪽 모서리)는 제자리 회전 지름(약 0.76 m)보다 좁아 전진 · 회전을 섞어야 함 → 차체 모양으로 통과 길이 있는지 먼저 계산(`tools/corner_feasibility.py`).

### 장애물 자료 일치 검사(도구, `261031_1325_nav2/tools/check_world_consistency.py`, 판정 아님)
- 대조: ① 장면 미로 USD 의 모든 충돌 prim(Mesh 9 · Cylinder 110, 높이 0~0.5) ② geom(목표 · G3 · 분석) ③ Nav2 지도(벽 지도 map_mission_nav + 물체 지도). ①의 경계 칸마다 ② · ③ 의 가장 가까운 점유까지 거리, 0.02 m 넘으면 빠짐.
- 결과: 옛 geom(기둥 없음) 빠짐 113 덩어리 · 최대 0.045 m(기둥 누락을 잡음) → 고친 geom 빠짐 0(최대 0.005). Nav2 지도: 기둥판 물체 지도로도 SLAM 지도의 얇은 벽(3 mm 판) 구멍 12 덩어리(최대 (1.175, 1.275) 0.055 · (3.392, 5.12) 0.038 m) → 도면 벽 · 기둥을 그린 `config/map_objects_fine_posts_walls.yaml`(make_objects_fine.py --posts --walls)로 빠짐 0(최대 0.011) **통과**. Nav2 에만 있는 점유: SLAM 지도 바깥 테두리가 최대 0.055 m 바깥으로(보수 쪽).
- 실물: ① 대신 발제 도면 · ③ 대신 실측 SLAM 지도로 같은 대조를 해 차이를 먼저 잼(차이가 크면 도면 대신 실측을 따름).

### 출발 모퉁이 통과 가능성(기하 계산, `261031_1325_nav2/tools/corner_feasibility.py`, 판정 아님)
- 남향 통로 (0.81, 0.75, −π/2) → 기둥 아래 동쪽 틈(x ≥ 1.25, 동향 ±15°), 전진 · 후진 0.02 m · 회전 중심 축 ±5°, 가장 좁은 곳 여유 최대 경로: **통과 가능, 병목 여유 0.052 m**(154 동작, `tools/corner_feasibility.json`).
- 그 길은 왼쪽 90° 가 아니라 **오른쪽으로 약 270°**(남 → 서 → 북 → 동) 돌며 전진 · 후진을 섞음. 왼쪽으로 꺾으면 앞모서리가 기둥 쪽으로 쓸림(설계 21~26 막힘 방향). 점 로봇 계획기(NavFn) + MPPI 는 이런 동작을 못 찾음.
- 다음 설계 27 후보: 물체 지도 = posts_walls + 전역 계획기 Smac Hybrid-A*(footprint 로 모든 자세 검사, REEDS_SHEPP · 작은 회전 반경 · 후진 허용) + MPPI 후진(enforce_path_inversion) · RotationShim 관계 정리.

### 설계 27(기준 그대로) — 전역 계획기 Smac Hybrid-A*(차체 모양 · 후진) + 전역 footprint 여백
- `261031_1325_nav2/config/nav2_params_v27.yaml`(생성 `tools/make_nav2_params_v27.py`, 바꾼 값 · 이유 `nav2_params_v27_changes.md`, v21 그대로): NavFn → `SmacPlannerHybrid`(REEDS_SHEPP, 최소 회전 반경 0.10, 72 방향, 후진 벌점 2.0, 해석 연장 최대 1.0 m, 거리 표 6 m), 전역 costmap `footprint_padding 0.04`(계획에만, 지역 · MPPI · 충돌 감시는 실제 차체), PathHandler `enforce_path_inversion true`(끊는 점 허용 0.06 m · 0.3 rad), RotationShim `use_path_orientations true`.
- launch `objects_map:=…/config/map_objects_fine_posts_walls.yaml`(도면 벽 · 기둥, 일치 검사 통과판). 나머지 설계 26 그대로(목표 goals_seed1_v26.json, Cartographer v19, turn_floor · turn_ff · icr_odom · rotation_center).
- 도구 확인(장면 없이 계획기만, `261031_1325_nav2/test/plan_only.sh` → `results/TOOLCHECK_plan_v27*/`, 판정 아님): 7 구간 경로를 받아 경로 자세마다 차체 여유(기둥 포함 geom).
  - 첫 시도 도구 실패: Smac 시작 때 거리 표 20 m 계산 26 s → 활성화 전 요청이 거절(`_TOOLFAIL_inactive`). 표 6 m 로 줄이고(시작 약 2 s) 받아들일 때까지 다시 보냄.
  - 여백 없이: 7 구간 모두 경로 · 겹침 0, 최소 여유 0.0075~0.036 m. 출발 모퉁이는 오른쪽 270° 가 아니라 기둥 아래로 왼쪽으로 꺾는 길(0.018 m). cost_penalty 4 · 8, 반경 0.05, 2차 벌점으로도 0.004~0.045 m 에서 오르내림.
  - 전역 여백 0.02 / 0.03 / 0.04: 최소 0.018 / 0.023 / **0.034 m**(0.04 는 두 번 실행 0.034~0.070 · 0.039~0.051, 출발 모퉁이는 통로 끝에서 앞뒤로 2~3번 끊어 돌고 기둥 아래로 빠짐). 0.04 로 정함.
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 3~7/7. 출발 모퉁이는 계획 경로가 기둥에서 0.05 m 떨어져 지금까지보다 나아질 것. 위험: ① BT 가 1 Hz 로 다시 계획하며 끊는 점 위치가 바뀌어 MPPI 가 앞뒤로 흔들림 ② MPPI 가 짧은 후진 구간(PreferForward 는 0.5 m 안에선 꺼짐)을 못 따라 진행 검사 실패 ③ 회전 반경 0.10 호를 스키드가 실제로는 제자리 회전 + 전진으로 따름 |
| G2 | 도착한 것 0.02~0.06 m |
| G3 | 0~0.04 m, 겹침 0(기둥 포함). 계획 여유 0.034 보다 위치 추정 · 추종 오차만큼 작음 |
| G4 | RMSE 0.004~0.015 m |
| G5 | 통과(후진 vx ≥ −0.10) |

### 설계 27 결과(`results/live_nav_v27/`, 모의) — **불통과(중단)**
| 항목 | 기준 | 결과 |
|---|---|---|
| G1 | 7/7 | 0/2 — start_box_1 'Failed to make progress'(148 s), L1_via 'Start occupied'(계획 거절) → 목표 실행기 중단(run_goals_exit 130, 도구 쪽) |
| G3 | > 0 | 최소 0.018 m · 겹침 0(기둥 포함) — 모퉁이까지 가지 못해서 |
| G4 | RMSE ≤ 0.05 · 최대 ≤ 0.15 m | RMSE 0.0086 · 최대 0.0185 m |
| G5 | | 통과(vx −0.10~0.085) |
- 148 s 동안 전진 명령이 거의 0 이고 제자리 회전만 좌우로 반복하며 출발 자리 서쪽(base x 0.44~0.63)에서 맴돎. 통로로 내려가지 못함.
- 원인: launch.log `Path validation failed` 수십 번(무효 자세 번호가 매번 다름, 0번 34회), 로봇이 서 있던 59~63 s 에도 → BT 가 약 0.7 s 마다 다시 계획, 계획마다 끊는 점 · 0.5 m 앞 방향이 크게 바뀜(−105° → 170° → −30° → 55° …, out_bag /plan) → RotationShim(use_path_orientations)이 그 방향으로 제자리 회전. 유효성 검사는 경로 자세의 (여백 포함) footprint 가 치명 칸에 닿는지 보는데, 전역 costmap 의 라이다 장애물 층이 스캔마다 다른 칸을 찍어 여백 경계에 붙은 경로가 계속 무효가 된 것으로 봄.
- 둘째 목표: 복구 후진(0.30 m)으로 남쪽 벽 0.04 m 까지 간 뒤 여백 0.04 footprint 로는 출발 자세가 점유 → Smac 'Start occupied'. 여백의 부작용(벽에 붙으면 계획 못 함).

### 설계 28(기준 그대로) — 전역 costmap 에서 라이다 층 빼기
- `nav2_params_v28.yaml`(같은 생성 스크립트 `--variant v28`, 표 `nav2_params_v28_changes.md`) = v27 + 전역 costmap plugins 에서 obstacle_layer 뺌(정적 층 = SLAM 벽 · 물체 지도(도면 벽 · 기둥 · 상자) + 팽창). 지역 costmap · MPPI · 충돌 감시는 라이다 그대로. 장면 없는 도구 확인(plan_only)은 처음부터 이 구성이라 경로는 그 결과(최소 여유 0.034~0.070 m)와 같은 조건.
- 나머지 설계 27 그대로.
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 2~7/7. 목표마다 한 번 계획한 경로가 유지되어 맴돌기는 없어질 것. 위험: ① MPPI 가 끊는 점(출발 모퉁이 2~3번)과 짧은 후진 구간을 못 따라 진행 검사 실패 ② 복구 후진 0.30 m 로 벽에 붙으면 여백 때문에 'Start occupied' ③ 경로를 다시 계획하지 않아 많이 벗어나면 되돌아가려 함 |
| G2 | 도착한 것 0.02~0.06 m |
| G3 | 0~0.04 m, 겹침 0(기둥 포함) |
| G4 | RMSE 0.004~0.015 m |
| G5 | 통과 |

### 설계 28 결과(`results/live_nav_v28/`, 모의) — **불통과(중단)**
| 항목 | 기준 | 결과 |
|---|---|---|
| G1 | 7/7 | 0/4 — 모두 'Failed to make progress'(start_box_1 116 s 등) → 목표 실행기 중단(run_goals_exit 130, 도구 쪽) |
| G3 | > 0 | 최소 0.0039 m · 겹침 0(기둥 포함) |
| G4 | RMSE ≤ 0.05 · 최대 ≤ 0.15 m | RMSE 0.0069 · 최대 0.0181 m |
| G5 | | 통과(vx −0.10~0.109) |
- 다시 계획은 줄어듦(경로 무효 19회, 설계 27 은 수십 번 · 0.7 s 마다): start_box_1 동안 /plan 은 11~17 s · 51~56 s · 130 s~ 세 묶음뿐(전역 라이다 층 뺀 효과).
- 그러나 첫 경로(출발 자리에서 남쪽 통로 x≈0.75 로 내려가 y≈0.42 에서 끊고 동쪽으로)를 MPPI 가 따르지 않음: Shim 이 −150° 로 돌린 뒤 MPPI 가 서남쪽 출발 방 안으로 0.4 m 전진(base (0.41, 0.71)) → 24~51 s 거의 정지 → 진행 실패. 뒤 목표들도 같은 자리 근처에서 제자리 회전만.
- 해석: 전역 경로가 차체 모양 · 여백으로 이미 검사된 좁은 길인데 MPPI 는 표본 궤적 비용으로 경로에서 벗어날 수 있고, 끊는 점 · 짧은 후진 구간을 비용으로만 다뤄 따르지 못함.

### 설계 29(기준 그대로) — 지역 제어기 MPPI → RPP(후진 허용)
- `nav2_params_v29.yaml`(`--variant v29`, 표 `nav2_params_v29_changes.md`) = v28 + RotationShim 안의 primary_controller 를 RegulatedPurePursuit 로: allow_reversing, 전방 주시 0.2(0.15~0.4) m, 목표 속도 0.12 · 곡률 크면 0.04 까지, 충돌 검사(지역 costmap footprint, 1 s), 속도 한계 MPPI 와 같음. use_rotate_to_heading 은 끄고 제자리 회전은 RotationShim 이 함. 나머지 설계 28 그대로.
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 1~7/7. 경로를 그대로 따라 출발 모퉁이를 지날 것. 위험: ① Isaac 스키드는 실제 회전이 명령의 53~75 % 라 반경 0.10 호에서 바깥으로 밀림(되먹임은 전방 주시 0.2 m 만) ② 곡률 큰 곳에서 0.04 m/s · 회전 0.3 rad/s 대 → 전진 중 작은 회전은 turn_floor 가 안 올려 줌 ③ 끊는 점 허용 0.06 m · 0.3 rad 에 못 들어가 멈춤 ④ RPP 충돌 검사가 좁은 곳에서 멈춤 |
| G2 | 도착한 것 0.02~0.06 m |
| G3 | 0~0.04 m, 겹침 0(기둥 포함) |
| G4 | RMSE 0.004~0.015 m |
| G5 | 통과 |

### 설계 29 결과(`results/live_nav_v29/`, 모의) — **불통과(중단)**
| 항목 | 기준 | 결과 |
|---|---|---|
| G1 | 7/7 | 0/1 — start_box_1 'Controller patience exceeded'(52 s) → 목표 실행기 중단 |
| G3 | > 0 | 최소 0.053 m · 겹침 0 |
| G4 | | RMSE 0.0074 · 최대 0.0148 m |
| G5 | | 통과(vx −0.04~0.12) |
- RPP 는 적재됨(`desired_linear_vel` 폐기 예정 경고, 실제 0.12 로 달림). Shim 이 −120° 로 돌린 뒤 통로 입구로 0.12 m/s 들어가다 30° 비스듬한 채로 RPP 'collision ahead' 42회 → 멈춤 · 제자리 회전 반복. 반경 0.10 호를 스키드 차체가 못 따름(실제 회전이 명령의 53~75 %, 0.35 미만 회전은 거의 안 돎).

### 설계 30(기준 그대로) — 계획기 Hybrid → Lattice '직선 + 제자리 회전'
- 원시 동작 `261031_1325_nav2/config/lattice_straight_rot_0025.json`(`tools/make_lattice_straight_rot.py`, 0.025 m 칸, 16 방향, 직선 짧은 것 · 약 0.1 m, 이웃 방향 제자리 회전(회전 중심 축, 5° 간격 자세로 쓸림 검사), 호 없음). 설치 예제는 5 cm · 반경 0.5 m 호라 안 맞음.
- `nav2_params_v30.yaml`(`--variant v30`) = v29 + SmacPlannerLattice(후진 허용, 회전 벌점 5, 다듬기 끔, 해석 연장 0.05 m 안), PathHandler `enforce_path_rotation true` · `minimum_rotation_angle 0.25`(제자리 회전 자리에서 경로를 끊음), Shim `angular_dist_threshold 0.25`(구간 시작에서 제자리로 돎), RPP `max_linear_vel 0.12`(폐기 예정 이름 대신). 전역 여백 0.04 · 전역 라이다 층 없음 그대로.
- 도구 확인(장면 없이, `results/TOOLCHECK_plan_v30/`, 판정 아님): 7 구간 모두 경로 · 겹침 0, 최소 여유 0.025(L1_via → L1)~0.065 m. 첫 구간 = 출발 자리에서 26° · 18° · 18° · 26° 나눠 돌며 조금씩 나아가 남향 → 통로 0.32 m → 바닥에서 돌고 후진 0.12 → 동향 0.56 m(기둥 아래) → 27° 대각 0.8 m → 목표. 대각 구간 끝에 후진 0.013 · 전진 0.036 잔떨림 2번(16 방향 격자 탓).
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 1~7/7. 처음으로 출발 모퉁이를 지날 것. 위험: ① 18~27° 작은 제자리 회전이 많음(앞먹임 turn_ff 로 0.19 rad/s 이상이면 돎) ② 끊는 점 · 회전 자리 허용 0.06 m · 0.3 rad 에 못 들어가 멈춤 ③ 잔떨림 후진 0.013 m 에서 앞뒤 흔들림 ④ L1 칸 안 회전(여유 0.025) |
| G2 | 도착한 것 0.02~0.06 m |
| G3 | 0~0.04 m, 겹침 0 |
| G4 | RMSE 0.004~0.015 m |
| G5 | 통과 |

### 설계 30 결과(`results/live_nav_v30/`, 모의) — **불통과(중단)**
| 항목 | 기준 | 결과 |
|---|---|---|
| G1 | 7/7 | 0/1 — start_box_1 'Failed to make progress'(136 s) → 목표 실행기 중단 |
| G3 | > 0 | 최소 0.080 m · 겹침 0 |
| G4 | | RMSE 0.0066 · 최대 0.0161 m |
| G5 | | 통과(vx −0.04~0.112) |
- 처음으로 출발 통로를 남쪽으로 내려감(회전 중심 y 0.95 → 0.565, 21 s). 그러나 ① 출발 자리의 작은 제자리 회전(26° · 18° · 18° · 26°)을 Shim 이 하지 않고 RPP 가 호로 돌아(15 s v 0.074 · w 0.30) 계획 통로 x 0.673 이 아닌 x 0.74 로 내려감 ② 계획 회전 자리 (0.673, 0.46) 보다 0.1 m 위 · 0.06 m 동쪽에서 제자리 회전을 시작해 북서(138~146°)를 본 채 23~150 s 앞뒤 ±0.04 m 만 떪, RPP 'collision ahead' 88회.
- 가설(확인 못 함, 경로 처리기 소스가 설치본에 없음): 원시 동작에 5° 간격 중간 자세를 넣어 이웃 자세 사이 회전이 `minimum_rotation_angle` 0.25 보다 작아 회전 자리에서 경로를 끊지 못함 → Shim 이 아니라 RPP 가 회전 구간을 처리.

### 설계 31(기준 그대로) — 회전 자리에서 실제로 돌기(경로 끊는 규칙에 맞춘 원시 동작 · 허용값)
- 원인 확인(도구, `261031_1325_nav2/test/probe_constraint/` → `261031_1325_nav2/results/TOOLCHECK_constraint_v31/`, 판정 아님): 설치된 `nav2_util::findFirstPathConstraint` 를 경로 파일에 그대로 불러 끊는 자리를 봄(소스: Nav2 main 의 path_utils.cpp · feasible_path_handler.cpp · rotation_shim_controller.cpp · smac_planner_lattice_impl.hpp).
  - 규칙: 제자리 회전의 **누적** 각이 minimum_rotation_angle 을 넘는 자세 직전까지를 한 구간으로, 다음 구간은 그 자세부터. 로봇이 구간 마지막 자세의 허용(xy · yaw) 안에 들면 다음 구간.
  - 설계 30: 5° 간격 중간 자세 때문에 회전 도중(13.2°)에서 끊김 → 허용 0.3 rad(17°) 안에 이미 있어 회전 없이 다음 구간 → RPP 가 남은 회전을 호로 처리(관찰과 같음). 가설 '안 끊김'은 틀렸고, '끊지만 회전 도중 · 허용이 큼'이 맞음.
  - 중간 자세만 빼면 회전이 연달아 있을 때(90° = 26.6 · 18.4 · 18.4 · 26.6°) 자세 1개짜리 구간이 생김(Shim 은 2개 미만이면 예외 → RPP 가 0.04 m/s 로 그냥 나아갈 위험). Smac Lattice 는 xy · yaw 차이 1e-4 미만 이웃 자세를 지워 끝 자세를 그냥 두 번 넣는 것도 안 됨 → 둘째 끝 자세를 같은 방향 5e-4 rad 더 돈 자세로 넣음. 결과: 7 구간 모두 자세 1개짜리 구간 없음(구간 맨 앞 출발 자세 1개짜리는 로봇이 이미 그 자리라 바로 넘어감).
- 바꾼 것(`261031_1325_nav2/config/nav2_params_v31.yaml`, 생성 `tools/make_nav2_params_v27.py --variant v31`, 표 `nav2_params_v31_changes.md`, v27~v30 바이트 같음 확인):
  - 원시 동작 `config/lattice_straight_rot_0025_nomid.json`(`tools/make_lattice_straight_rot.py --no_rot_mid`, 회전 = 시작 · 끝 · 끝+5e-4 rad, 직선 그대로).
  - PathHandler `inversion_yaw_tolerance` 0.3 → 0.15(가장 작은 회전 0.32 rad 보다 작고 Shim 해제 0.1 보다 큼), `inversion_xy_tolerance` 0.06 → 0.03(회전 자리 0.06 m 앞에서 미리 돌지 않게).
  - Shim `forward_sampling_distance` 0.5 → 0.03(짧은 직선 구간에서 끝 자세 방향을 미리 보고 돌지 않게, 회전 구간 시작에선 회전한 방향을 봄).
- 도구 확인(장면 없이, `261031_1325_nav2/results/TOOLCHECK_plan_v31/`, 판정 아님 — 계획기 매개변수는 xy 허용 추가 전후 같음): 7 구간 모두 경로 · 겹침 0, 자세 최소 여유 0.034~0.087 m, **쓸림 여유**(plan_only 새 항목, 이웃 자세 사이를 2° · 5 mm 로 채움) 0.023(L3 → start)~0.087 m · 쓸림 겹침 0. 중간 자세 없는 회전의 쓸림은 전역 여백 0.04 가 덮음(26.6° 회전의 끝 자세 밖 쓸림 ≈ 0.382 × (1 − cos 13.3°) = 0.010 m). 옛 시도: 끝 자세 하나만(`_nodup`) · 그냥 두 번(`_dup_removed`) — 둘 다 자세 1개짜리 구간 1~12개.
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 1~7/7. 출발 모퉁이를 회전 자리마다 제자리로 돌며 지날 것. 위험: ① 구간 끝(회전 직전)에서 RPP 가 0.03 m · 0.15 rad 허용에 못 들어감(방향이 직선에서 8.6° 넘게 틀어지면 멈춤) ② 회전 뒤 Shim 해제 0.1 rad 오차를 RPP 가 호로 고치며 좁은 곳에서 'collision ahead' ③ 90° 회전이 4 번 끊어 돌아 느림(진행 검사 0.25 m / 15 s) ④ L1 칸 안 회전(쓸림 0.024) |
| G2 | 도착한 것 0.02~0.06 m |
| G3 | 0~0.04 m, 겹침 0 |
| G4 | RMSE 0.004~0.015 m |
| G5 | 통과 |

### 설계 31 결과(`results/live_nav_v31/`, 모의) — **불통과**(처음으로 출발 모퉁이 통과, 5/7 도착)
- **구성 정정(2026-11-01, 설계 32 준비 중 발견)**: 생성 스크립트의 판별 적용 조건 실수로 v31 실행본에는 V28(전역 라이다 층 뺌) · V29(RPP)가 **빠져 있었음**. 실제로 달린 것 = 설계 27 + V30 + V31: 지역 제어기 **MPPI**(RPP 아님), 전역 costmap 라이다 층 **있음**, 계획기 Lattice(중간 자세 없는 회전), 경로 처리기 · Shim 은 위 설계 31 값. V30 의 RPP 용 값(max_linear_vel 0.12)은 MPPI 매개변수 묶음에 들어가 쓰이지 않음. 아래 결과 · 원인의 'RPP' 는 'MPPI' 로 읽을 것(전진 중 죽은 구간은 차체 성질이라 그대로). 생성 스크립트는 판별 묶음을 명시하도록 고침(v27~v31 실행본과 바이트 같음 확인).
| 항목 | 기준 | 결과 |
|---|---|---|
| G1 | 7/7 | 5/7 — start_box_3 'Failed to make progress' 뒤 다시 계획하자 'Resulting plan has 0 poses'(103), 마지막 start 'Goal was in lethal cost'(206) |
| G2 | 도착 ≤ 0.10 m · yaw ≤ 0.15 rad | 불통과 — 위치 0.011~0.098 m, yaw 0.009(L2)~1.19 rad(start_box_1). 'SUCCEEDED' 인데 yaw 가 크게 틀림 |
| G3 | > 0 | −0.001 m · 겹침 317 표본 — 모두 L3 도착 뒤 281.7~288 s 같은 자세(4.078, 5.504, 155°)에 서 있는 동안 |
| G4 | RMSE ≤ 0.05 · 최대 ≤ 0.15 m | RMSE 0.0075 · 최대 0.0192 m |
| G5 | | 통과(vx −0.084~0.124) |
- 좋아진 것: 출발 모퉁이(start_box_1)를 64 s 에 지남. 회전 자리마다 Shim 이 제자리로 돎(설계 30 의 호 처리 없어짐).
- 원인 ① 도착 방향: Lattice 경로의 끝 방향이 목표 방향과 다름(장면 없는 확인에서도 같음: start_box_1 목표 2.786 → 경로 끝 1.571, L3 −2.853 → 2.678). 제어기는 경로 끝 자세를 목표로 보므로 그 방향에서 'SUCCEEDED'. Smac A*(a_star_impl.hpp)는 허용(0.05 m) 안 노드가 나온 뒤 `max_on_approach_iterations`(1000) 만 더 찾고 못 찾으면 방향과 상관없이 가장 가까운 노드로 끝냄. 또 16 방향 칸이라 맞춰도 목표와 최대 11° 차이(L3 10°).
- 원인 ② L3 겹침: 경로 끝(목표 아닌 자세)에서 위치 오차 0.035 m 로 기둥에 1 mm 닿은 채 섬.
- 원인 ③ start_box_3: 63° → 90° 제자리 회전이 78° 에서 끝남(추정 방향이 참보다 앞섬, 206.6~206.8 s). 이후 전진하며 RPP 가 w 0.06~0.11 로 고치려 했지만 참 방향이 78.2° 그대로 → 경로에서 옆으로 0.18 m 벗어나 구간 끝 허용(0.03 m)에 못 들어가고 멈춤(진행 실패). 이 bag 의 0.5 s 창(전진 ≥ 0.03 m/s, w 일정) 집계: 명령 |w| < 0.15 → 참 회전 0.000~0.002, 0.15~0.20 → 0.03(설계 30 · 29 bag: 0.2~0.3 → 0.12, 0.3~0.4 → 0.17). **전진 중에는 |w| 약 0.17 까지 죽은 구간** — turn_floor 는 제자리(v < 0.02)만 고침.
- 원인 ④ '0 poses': 진행 실패 뒤 다시 계획한 경로가 제자리 회전으로 시작 → 첫 구간이 출발 자세 1개 → 로봇이 이미 0.03 m · 0.15 rad 허용 밖이면 잘라 낸 뒤 자세 0개(추정).

### 전진 중 회전 곡선(도구 확인, `results/TOOLCHECK_turn_moving/`, `probe_turn/`, 모의 · 판정 아님)
- 빈 바닥(`probe_turn/empty_maze.usda`, 라이다 대상용 먼 기둥 하나)에 scene_live.py(cmd_vel 모드)를 띄우고 /cmd_vel_nav 로 v 0 · 0.04 · 0.08 · 0.12 m/s × |w| 0.05~0.8 rad/s 를 4 s 씩(가운데 2.5 s 참값). 첫 시도는 미로에 벽 prim 이 없어 장면 실패(`_TOOLFAIL_no_wall`).
- 결과(전진 3 속도 평균, 실제 |w|): 명령 0.05 → 0.0003, 0.10 → 0.0013, 0.15 → 0.018, 0.20 → 0.050, 0.25 → 0.088, 0.30 → 0.129, 0.35 → 0.172, 0.45 → 0.261, 0.60 → 0.400, 0.80 → 0.582. 전진 속도에 거의 무관하고 제자리(v 0)와도 같은 꼴(제자리 0.35 → 0.187 · 0.8 → 0.600, 설계 24 곡선과 맞음). **명령 0.1 rad/s 까지 차체가 안 돎.** 곡선 `results/TOOLCHECK_turn_moving/move_ff_curve.json`.

### 설계 32(기준 그대로) — 실제 v31 구성(MPPI) + 정확한 도착 자세 · 실패 뒤 다시 계획 · 회전 끝 정밀 · 전진 중 회전 앞먹임
- `261031_1325_nav2/config/nav2_params_v32.yaml`(`tools/make_nav2_params_v27.py --variant v32` = 설계 27 + V30 + V31 + V32, 표 `nav2_params_v32_changes.md`):
  - 계획기: `max_on_approach_iterations` 1000 → 100000(정확한 목표 칸 · 방향 칸까지), `analytic_expansion_max_length` 0.05 → 0.15(정확한 목표 xy), `max_planning_time` 5 → 10 s(6.5 m 구간 7.3 s).
  - BT `config/bt/navigate_to_pose_replan_after_failure.xml`(설치본 사본): FollowPath 실패 뒤 · 복구 동작 뒤 현재 자리에서 다시 계획, 목표 가까움 4 → 20 m(주기적 다시 계획 대신 경로 무효 · 실패 때만).
  - PathHandler `inversion_yaw_tolerance` 0.15 → 0.2(다시 계획한 경로 첫 자세 = 로봇 방향에 가까운 칸, 최대 0.196 rad), Shim `angular_disengage_threshold` 0.1 → 0.05.
- launch: 설계 31 인자 + `move_ff:=true move_ff_cmd:=[0.1,0.15,0.2,0.25,0.3,0.35,0.45,0.6,0.8] move_ff_act:=[0.0,0.0179,0.0496,0.0875,0.129,0.1719,0.261,0.3999,0.5824]`(turn_floor 전진 중 앞먹임, 새 기능 · 기본 꺼짐).
- 목표 실행기: `--final_spin_tol 0.03`(도착 뒤 추정 방향 차 > 0.03 rad 면 Nav2 Spin 으로 맞춤 — Lattice 끝 방향은 16 방향 칸이라 목표와 최대 11.25°). 채점(G2)은 Spin 뒤 참값.
- 도구 확인(장면 없이, 계획기 매개변수 v32 와 같음 확인): `261031_1325_nav2/results/TOOLCHECK_plan_v32_a15/` — 7 구간 모두 경로 · 겹침 0, 끝 xy 0.000, 끝 방향 차 0~10°, 끝 호 0.004~0.043 m, 자세 여유 0.034~0.048, 쓸림 0.0089(L1 → L2)~0.043 m. 참고: `TOOLCHECK_plan_v32_TOOLFAIL_hybrid_chain`(생성 조건 실수로 Hybrid 가 나온 첫 시도), `_v32`(해석 0.05 · 5 s: 끝 xy 0.004~0.015, 마지막 구간 45°), `_a30`(호 0.04~0.10).
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 5~7/7. 설계 31 의 실패 둘(옛 경로 재전송 · 위험 칸 목표)을 다시 계획 · 정확한 목표로 넘길 것. 위험: ① 전진 앞먹임이 MPPI 와 엇갈려 좌우로 흔들림(명령 0.02 → 0.15 처럼 크게 바뀜) ② 전역 라이다 층 깜빡임으로 경로 무효 → 다시 계획 → 첫 구간 문제 ③ 끝 호(≤ 0.043 m)를 못 따라 끝 허용(0.03 m)에 못 들어감 |
| G2 | 0.01~0.05 m · yaw 0.01~0.06 rad(Spin 뒤) |
| G3 | 0~0.04 m, 겹침 0(목표 자리에서 끝남, L1 → L2 쓸림 0.009 가 가장 좁음) |
| G4 | RMSE 0.005~0.015 m |
| G5 | 통과 |

### 설계 32 결과(`results/live_nav_v32/`, 모의) — **불통과(0/7)**
| 항목 | 기준 | 결과 |
|---|---|---|
| G1 | 7/7 | 0/7 — start_box_1 출발 5 s 뒤 FollowPath 실패 → 새 BT 가 다시 계획 → 'Start occupied'(205), 이후 목표 모두 같은 자리에서 'Start occupied' · 마지막은 'Goal was in lethal cost' |
| G3 | > 0 | 0.0705 m · 겹침 0(출발 자리 근처만) |
| G4 | | RMSE 0.0051 · 최대 0.0086 m |
| G5 | | 통과 |
- 원인: 설계 27 과 같음 — 전역 costmap 라이다 층이 스캔마다 다른 칸을 찍어 BT 의 ValidatePath 가 거의 1 s 마다 실패('Path validation failed. Invalid pose indices: [71]' · [61] · [66] · [59] …) → 1 s 마다 다시 계획 → 경로 처리기가 매번 첫 구간부터 → 진행 못 하고 실패 → 다시 계획하는 사이 로봇이 벽 쪽(0.785, 0.845, −94°)으로 붙어 전역 여백 0.04 footprint 가 점유 → 'Start occupied'. 설계 31 은 같은 구성(라이다 층 있음)이었지만 출발 구간에서 이 깜빡임을 피했음(경로가 달라서로 봄, 확인 안 함).
- V28(전역 라이다 층 뺌)은 원래 이 문제의 해결책이었는데 설계 31 · 32 에 빠져 있었음.

### 설계 33(기준 그대로) — 설계 32 + V28(전역 costmap 라이다 층 뺌)
- `nav2_params_v33.yaml`(`--variant v33`) = v32 와 `global_costmap.plugins` 하나만 다름(static · objects · inflation). 지역 costmap · MPPI · 충돌 감시는 라이다 그대로. 나머지(launch · move_ff · run_goals --final_spin_tol 0.03) 설계 32 그대로. 장면 없는 경로 확인(TOOLCHECK_plan_v32_a15)은 처음부터 라이다 층 없이 했으므로 그대로 해당.
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 4~7/7. 경로가 목표마다 한 번(실패 뒤만 다시). 위험: ① 전진 앞먹임과 MPPI 가 엇갈려 흔들림 ② 실패 뒤 다시 계획할 때 벽에 붙어 있으면 여백 0.04 로 'Start occupied' ③ 끝 호(≤ 0.043 m) 끝 허용 0.03 m |
| G2 | 0.01~0.05 m · yaw 0.01~0.06 rad(Spin 뒤) |
| G3 | 0~0.04 m, 겹침 0 |
| G4 | RMSE 0.005~0.015 m |
| G5 | 통과 |

### 설계 33 결과(`results/live_nav_v33/`, 모의) — **불통과(6/7, 지금까지 가장 좋음)**
| 항목 | 기준 | 결과 |
|---|---|---|
| G1 | 7/7 | 6/7 — 마지막 start 'Start occupied'(L3 도착 자리에서 계획 못 함) |
| G2 | ≤ 0.10 m · ≤ 0.15 rad | 불통과 — 위치 0.039~0.057 m(모두 통과), yaw 0.003~0.227 rad(start_box_1 0.151 · L3 0.227 넘음) |
| G3 | > 0 | **통과** 최소 0.0156 m · 겹침 0 |
| G4 | RMSE ≤ 0.05 · 최대 ≤ 0.15 m | **통과** RMSE 0.0057 · 최대 0.0147 m |
| G5 | | 통과(vx −0.10~0.123) |
- 출발 모퉁이부터 L3 까지 모두 도착. 다시 계획 반복 없어짐(V28 효과).
- 원인(yaw): 도착 뒤 Spin 이 지나침. L3: 남은 차 −0.222 → Spin 이 0.8 rad/s 로 돌고 velocity_smoother 각감속 1.5 rad/s² 로 0.8²/(2·1.5) = 0.21 rad 더 돎 → −0.438 돌아 반대쪽 0.227. 제어 출력이 0 인데 최종 명령이 0.3~0.5 s 남는 것을 bag(/cmd_vel_ctrl · /cmd_vel_nav)에서 확인. start_box_1 도 같은 꼴(0.083 → 넘어가 0.151).
- 원인(마지막 목표): L3 에서 방향이 0.23 rad 넘어가 차체가 기둥 쪽으로 돈 자리에서 여백 0.04 footprint 가 점유 → 'Start occupied'(추정, 위 지나침이 풀리면 같이 풀릴 것).

### 설계 34(기준 그대로) — 설계 33 + 회전 멈춤 지나침 줄이기
- `nav2_params_v34.yaml`(`--variant v34`) = v33 과 세 값만 다름: velocity_smoother 각가속 · 각감속 ±1.5 → ±3.2 rad/s²(behavior_server rotational_acc_lim · Spin 감속 곡선과 같게), behavior_server `max_rotational_vel` 0.8 → 0.5(멈춤 거리 0.04 rad).
- 목표 실행기 `--final_spin_max 3`(새 인자): 도착 뒤 방향 차가 0.03 rad 안에 들 때까지 Spin 을 3 번까지. 나머지 설계 33 그대로.
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 6~7/7. 마지막 목표는 L3 방향이 맞으면 출발 가능. 위험: 각감속을 키워 MPPI · Shim 회전이 덜 부드러워짐(Isaac 차체 흔들림), Spin 최소 0.35 · turn_ff 로 작은 각(0.03~0.05)에서 다시 지나침 |
| G2 | 위치 0.03~0.06 m · yaw 0.01~0.06 rad |
| G3 | 0~0.04 m, 겹침 0 |
| G4 | RMSE 0.005~0.015 m |
| G5 | 통과 |

### 설계 34 결과(`results/live_nav_v34/`, 모의) — **불통과(6/7)**
| 항목 | 기준 | 결과 |
|---|---|---|
| G1 | 7/7 | 6/7 — 마지막 start 'Start occupied'(설계 33 과 같음) |
| G2 | ≤ 0.10 m · ≤ 0.15 rad | 도착한 6개는 모두 안(위치 0.049~0.068 m, yaw 0.003~0.083 rad). 마지막 목표 미도착으로 불통과 |
| G3 | > 0 | **통과** 0.0197 m · 겹침 0 |
| G4 | | **통과** RMSE 0.0062 · 최대 0.0158 m |
| G5 | | 통과(vx −0.098~0.150) |
- Spin 지나침은 줄었지만 남음: start_box_1 0.104 → −0.087 → 0.092, L1 0.076 → −0.085 → 0.088, L3 −0.182 → 0.084 → −0.089(3 번 모두 약 0.09 rad 넘어감). 최소 0.35 로 끝까지 돌다 멈추는 것 + 차체 · 추정 지연으로 봄.
- 위치 오차가 모든 목표에서 0.05 m 안팎 = 목표 확인기 xy 허용 0.06 경계에서 멈춤. L3 는 좁은 칸이라 덜 간 자리(+ 남은 방향 차)에서 여백 0.04 footprint 가 점유 → 'Start occupied'(추정).

### 설계 35(기준 그대로) — 설계 34 + 목표 xy 허용 0.025 · Spin 끝 감속
- `nav2_params_v35.yaml`(`--variant v35`) = v34 와 세 값만 다름: `general_goal_checker.xy_goal_tolerance` 0.06 → 0.025, behavior_server `min_rotational_vel` 0.35 → 0.19(turn_floor 제자리 바닥과 같게), `rotational_acc_lim` 3.2 → 1.0(목표 앞 0.1 rad 에서 0.45 rad/s 로 미리 줄임). 나머지(launch · run_goals --final_spin_tol 0.03 --final_spin_max 3) 설계 34 그대로.
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 6~7/7. 위험: 허용 0.025 에 MPPI 가 못 들어가 끝에서 맴돌다 진행 실패 · 끝 호(≤ 0.043 m)를 못 따라감, Spin 0.19 바닥에서도 0.03~0.05 지나침 |
| G2 | 위치 0.01~0.03 m · yaw 0.01~0.05 rad |
| G3 | 0~0.04 m, 겹침 0 |
| G4 | RMSE 0.005~0.015 m |
| G5 | 통과 |

### 설계 35 결과(`results/live_nav_v35/`, 모의) — **불통과(6/7)**
| 항목 | 기준 | 결과 |
|---|---|---|
| G1 | 7/7 | 6/7 — 마지막 start 'Failed to make progress'(176 s, 출발점 0.79 m 앞) |
| G2 | ≤ 0.10 m · ≤ 0.15 rad | 도착 6개 위치 0.016~0.032 m · yaw 0.006~0.058 rad(모두 안), 마지막 미도착으로 불통과 |
| G3 | > 0 | **통과** 0.0177 m · 겹침 0 |
| G4 | | **통과** RMSE 0.0061 · 최대 0.0191 m |
| G5 | | 통과(vx −0.10~0.158) |
- 'Start occupied' 해결(L3 에 0.032 m 까지 들어가 다음 구간 출발). Spin 은 0.106 → −0.029, L3 −0.202 → 0.062 → −0.016 으로 수렴.
- 마지막 구간(6.5 m)에서 세 번 멈춤: ① 382~397 s · ② 417~432 s — 로봇이 구간 끝(회전 직전 자세)에서 0.035 · 0.037 m 앞, 방향 차 0.2~0.5° 인데 **MPPI 가 거기서 멈춰** 허용 0.03 에 못 들어감 → 진행 검사 15 s 뒤 실패 → 다시 계획해야 회전. ③ 447~512 s — 경로 위(0.004 m)에서 앞으로 0.53 m 곧게 가면 되는데 MPPI 출력이 ≈ 0(통로 여유 0.068 m, 최종 목표 0.55 m 앞이라 GoalCritic(1.4 m) 쪽으로 바뀐 구간, 로그 경고 없음 — 정확한 원인 확인 못 함) → 진행 실패 3 번.
- 해석: Lattice 경로는 이미 차체 모양 · 여백으로 검사된 길인데 MPPI 는 비용으로 경로를 '참고'만 해서 구간 끝 · 좁은 곳에서 멈춤(설계 28 과 같은 성질).

### 설계 36(기준 그대로) — 설계 35 + 지역 제어기 MPPI → RPP
- `nav2_params_v36.yaml`(`--variant v36`) = v35 와 `FollowPath.primary_controller` 만 다름(설계 29 · 30 의 RPP 값 그대로: 후진 허용, 전방 주시 0.15~0.4 m, 최대 0.12 m/s, 끝 0.3 m 에서 0.04 m/s 까지 줄임, 충돌 검사 1 s).
- RPP 가 설계 29 · 30 에서 실패한 원인은 호 경로(→ Lattice), 회전 건너뜀(→ 설계 31), 전진 중 작은 w 가 안 듦(→ move_ff, 설계 32~) — 모두 지금은 고쳐진 상태. RPP 는 구간 끝까지 0.04 m/s 로 다가가므로 허용 0.03 에 들어갈 것.
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 6~7/7. 위험: ① 좁은 곳에서 RPP 'collision ahead'(지역 costmap footprint 1 s 앞, 여백 없음) ② move_ff 로 커진 w 가 RPP 와 흔들림 ③ 후진 구간 끝 · 끝 호 |
| G2 | 위치 0.01~0.03 m · yaw 0.01~0.06 rad |
| G3 | 0~0.04 m, 겹침 0 |
| G4 | RMSE 0.005~0.015 m |
| G5 | 통과 |

### 설계 36 결과(`results/live_nav_v36/`, 모의) — **불통과(6/7)**
| 항목 | 기준 | 결과 |
|---|---|---|
| G1 | 7/7 | 6/7 — 마지막 start 'Start occupied'(L3 출발 30 s 뒤) |
| G2 | ≤ 0.10 m · ≤ 0.15 rad | 도착 6개 위치 0.018~0.024 m · yaw 0.001~0.062 rad(모두 안), 마지막 미도착으로 불통과 |
| G3 | > 0 | **통과** 0.0157 m · 겹침 0 |
| G4 | | **통과** RMSE 0.0054 · 최대 0.0183 m |
| G5 | | 통과(vx −0.090~0.120) |
- RPP 로 구간이 빨라짐(start_box_1 54 s, MPPI 77~110 s). 구간 끝 멈춤 없음. L3 셋째 Spin 은 behavior_server 'Collision Ahead' 로 멈춤(좁은 칸, yaw 0.033 로 끝남).
- 마지막 구간: 출발 직후부터 BT 의 ValidatePath 가 거의 1 s 마다 'Path validation failed'(무효 자세 124~131, 앞 구간에도 [0] 몇 번) → 다시 계획 반복 → 경로 처리기가 매번 첫 구간부터 → L3 칸 안에서 v ±0.04 로 앞뒤로 떨고 돌다 다시 계획 때 여백 footprint 점유 → 'Start occupied'. 전역 costmap 은 정적 지도뿐(라이다 층 없음)이라 Smac 이 받아들인 경로를 검사가 무효로 보는 것은 검사 방식 차이로 추정(확인 안 함).

### 설계 37(기준 그대로) — 설계 36 + BT 경로 유효 검사 뺌
- `nav2_params_v37.yaml`(`--variant v37`) = v36 과 `default_nav_to_pose_bt_xml` 만 다름: `config/bt/navigate_to_pose_replan_only_after_failure.xml`(설계 32 사본에서 TruncatePathLocal + ValidatePath 를 뺌, 트리 ID 따로). 다시 계획은 목표 바뀜 · FollowPath 실패 뒤 · 복구 뒤만. 움직이는 장애물은 지역 costmap · RPP 충돌 검사 · collision_monitor. 나머지 설계 36 그대로.
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 6~7/7. 마지막 구간이 한 번 계획한 경로로 감. 위험: L3 칸 출발 회전 · 출발점 앞 좁은 통로에서 RPP 'collision ahead', 출발 상자 목표 자세(여유 0.19) 도착 회전 |
| G2 | 위치 0.01~0.03 m · yaw 0.01~0.06 rad |
| G3 | 0~0.04 m, 겹침 0 |
| G4 | RMSE 0.005~0.015 m |
| G5 | 통과 |

### 설계 37 결과(`results/live_nav_v37_TOOLFAIL_isaac_segv_last_goal/`, 모의) — **불통과(5/7, 마지막 목표는 도구 실패)**
| 항목 | 기준 | 결과 |
|---|---|---|
| G1 | 7/7 | 5/7 — L3 'Controller patience exceeded'(두 번), 마지막 start 는 **Isaac 비정상 종료(isaac_exit 139)** 로 TIMEOUT(도구 실패, 판정에서 뺌) |
| G2 | | 도착 5개 위치 0.010~0.025 m · yaw 0.017~0.056 rad |
| G3 | > 0 | 0.0102 m · 겹침 0 |
| G4 | | RMSE 0.0052 · 최대 0.0164 m |
| G5 | | 통과 |
- ValidatePath 를 빼 다시 계획 반복은 없어짐. L3 칸 안(실제 여유 0.036~0.08 m)에서 RPP 'collision ahead' 7 번씩 → 실패 → 다시 계획 → 같은 실패. 설계 36 은 같은 RPP 로 L3 를 지났으므로 경계선. RPP 충돌 검사는 지역 costmap(0.025 m 칸)에 실제 차체를 대 보므로 여유 0.04 m 쯤에서 칸 반올림 · 라이다 잡음으로 닿음이 될 수 있다고 봄(확인 안 함).
- Spin 은 매번 약 0.06 rad 지나침(0.05 → −0.05 → 0.05 왕복, 허용 0.03 에 못 들고 3 번 다 씀).

### 설계 38(기준 그대로) — 설계 37 + RPP 자체 충돌 검사 끔(안전은 collision_monitor) · 도착 Spin 허용 0.07
- `nav2_params_v38.yaml`(`--variant v38`) = v37 과 RPP `use_collision_detection` 만 다름(true → false). 경로는 여백 0.04 로 검사됐고, 움직이는 장애물 안전은 collision_monitor(실제 차체 · 라이다 점 6 개 이상 · 1.2 s 앞 접근 감속, 칸 반올림 없음).
- 목표 실행기 `--final_spin_tol 0.07`(0.03 → 0.07): Spin 이 매번 약 0.06 지나쳐 0.03 에서는 왕복만 함. 0.07 은 G2 0.15 의 절반 아래. 나머지 설계 37 그대로.
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 6~7/7. 위험: collision_monitor 접근 감속이 좁은 L3 칸에서 멈춰 세움, 충돌 검사 없이 경로를 벗어나 벽에 닿음(G3) |
| G2 | 위치 0.01~0.03 m · yaw 0.01~0.07 rad |
| G3 | 0~0.04 m, 겹침 0 |
| G4 | RMSE 0.005~0.015 m |
| G5 | 통과 |

### 설계 38 결과(`results/live_nav_v38/`, 모의) — **불통과(6/7)**
| 항목 | 기준 | 결과 |
|---|---|---|
| G1 | 7/7 | 6/7 — 마지막 start 'Failed to make progress'(159 s, 출발점 0.199 m 앞) |
| G2 | ≤ 0.10 m · ≤ 0.15 rad | 도착 6개 위치 0.015~0.022 m · yaw 0.006~0.051 rad(모두 안), 마지막 미도착 |
| G3 | > 0 | **불통과** −0.001 m · 겹침 20 표본 — L3 칸 안 269.6~269.9 s, (4.06, 5.51, 156°)에서 후진 −0.04 + 회전 0.72(RPP 충돌 검사 끔 뒤) |
| G4 | RMSE ≤ 0.05 · 최대 ≤ 0.15 m | 통과(RMSE 0.0227 · **최대 0.1246 m**, 지금까지 최대 0.02 대) |
| G5 | | 통과 |
- L3 · 구간 끝 멈춤은 없어짐(L3 43.6 s, 가장 빠름). Spin 허용 0.07 로 왕복 줄어듦(1~2 번).
- 마지막 구간: 남북 통로(x 0.82)를 내려와 출발 상자로 들어서는 413 → 415 s 에 **Cartographer 위치가 통로 방향으로 0.11 m 튐**(/tracked_pose 와 참값의 y 합 0.950 → 0.840, x 합은 0.73~0.745 로 그대로). 그 뒤 추정 기준으로 목표에 가려다 참값으로는 0.16 m 지나쳐(0.84, 0.79) 벽 옆에서 돌다 진행 실패. 이 무렵 마지막 노드(1269~1276)의 제약이 원래 지도 submap 과 0.05~0.07 m 어긋남(점수 66~87 %). 출발 통로 근처는 장애물 자료 일치 검사에서 SLAM 지도의 얇은 벽 구멍(최대 (1.175, 1.275) 0.055 m)이 있던 곳 — 지도 · 위치 추정 쪽 원인으로 봄(확인 안 함).

### Cartographer 재생 확인(도구, `261031_1325_nav2/test/replay_carto.sh` · `replay_carto_compare.py`, 모의 · 판정 아님)
- live_nav_v38 녹화의 /clock · 라이다 2개 · tf_static(필요하면 /odometry/filtered)만 다시 틀어 launch 와 같은 가림 · Cartographer(저장 pbstream frozen) · carto_start 를 다른 lua 로 돌리고, /tracked_pose 를 참값 /sim/truth 와 비교(발제 좌표, align.json). 결과 `261031_1325_nav2/results/TOOLCHECK_replay_carto_*_on_v38/compare.json`.
- 도구 실패 3번(폴더 `_TOOLFAIL_gflags_order` · `_record_args` · `_record_sigint`): gflags 인자는 --ros-args 앞, 녹화는 --topics, 녹화기 종료는 TERM(스크립트 배경 작업은 SIGINT 무시).
| lua(v19 은 그대로, 나머지는 사본) | 바꾼 것 | RMSE | 최대(시각) | 2 s 안 0.05 m 넘는 튐 |
|---|---|---|---|---|
| 실시간 녹화 그대로(v19) | — | 0.0223 | 0.124(418 s) | 415.0 s |
| v19 재생 | — | 0.0486 | 0.246(415 s) | 415.0 s(재현됨) |
| v20 | min_score 0.65 → 0.75 | 0.0576 | 0.247 | 415.0 s |
| v21 | min_score 0.80 | 0.0563 | 0.247 | 415.0 s |
| **v22** | **use_odometry true(/odometry/filtered = EKF 바퀴 ICR + IMU 각속도)** | **0.0073** | **0.033(414 s)** | **없음** |
| v23 | ceres translation · rotation 가중치 ×10 | 0.529 | 1.56 | 59 번 |
- 원인: 참값 자세에선 스캔이 지도와 0.4~0.5 cm 로 맞는데 Cartographer 추정 자세에선 3.3 cm(diag_scan_map 412~420 s) → 지도 구멍이 아니라 **라이다만 쓰는 지역 맞춤이 남북 통로(x 0.82)를 내려와 출발 구역으로 들어설 때 통로 방향으로 미끄러짐**(414 s 방향 오차 −0.08 rad 가 먼저, 이어 위치). 낮은 점수 제약 거부(v20 · v21)로는 안 막힘. IMU 를 Cartographer 에 넣는 후보는 장면 IMU 의 선가속도가 0(중력 없음)이라 뺌.
- 한계: 재생은 열린 고리(로봇은 녹화 경로 그대로 움직임). 닫힌 고리 효과는 설계 39 실시간에서.

### 설계 38 L3 닿음 원인(v38 bag /plan · 명령, 판정 아님)
- L3 마지막 구간 = 후진 0.07 m + **해석 연장 후진 호**(0.04 m 에 0.94 rad, Reeds-Shepp 반경 0.1). 구간이 RPP 전방 주시(최소 0.15 m)보다 짧아 끝점을 바로 겨눠 268.6~270.2 s 후진 −0.04 · 회전 0.46~0.52 rad/s → 차체가 쓸려 벽에 1 mm(G3 −0.001).
- 해석 연장은 거리 √2 칸(0.035 m) 미만이면 시도하지 않음(`nav2_smac_planner/analytic_expansion_impl.hpp:227`) → 최대 길이 0.01 이면 꺼지고 경로는 원시 동작(직선 + 제자리 회전)만으로 목표 칸 · 방향 칸에서 끝남.
- 장면 없는 경로 확인(`261031_1325_nav2/results/TOOLCHECK_plan_v39/`): 7 구간 경로 · 겹침 0, 자세 여유 0.026~0.048 · 쓸림 0.009~0.043 m, 끝 호(이동 + 회전 동시 자세쌍) 0(설계 32~38 판 TOOLCHECK_plan_v32_a15 는 구간마다 2~3쌍), 끝 xy 는 a15 끝(= 정확한 목표)과 0.004~0.011 m 차이, 마지막 'start' 끝 방향 −2.357(목표 π, 0.785 rad 차 → 도착 뒤 Spin).

### 설계 39(기준 그대로) — 설계 38 + 해석 연장 호 끔 + Cartographer 에 EKF odom
- `nav2_params_v39.yaml`(`--variant v39`) = v38 과 `planner_server.GridBased.analytic_expansion_max_length` 0.15 → 0.01 만 다름.
- launch: 설계 38 인자에서 `carto_config:=carto_localization_v22.lua`(v19 + `options.use_odometry = true`) + `carto_odom_topic:=/odometry/filtered`(새 인자, 기본 `odom` = 이전과 같음, 고치기 전 사본 `nav2_mission.launch.py.before_carto_odom_261101`). 목표 실행기 · 장면 인자는 설계 38 그대로.
- TF 는 그대로(Cartographer 는 TF 를 안 내고 /tracked_pose 만, map→odom = map_odom_relay). EKF(odom→base_link)는 Cartographer 를 입력으로 받지 않으므로 되먹임 고리 없음.
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 6~7/7. 마지막 구간 위치 튐이 없으면 출발 상자 도착. 위험: ① 호 없이 끝 칸에서 멈춰 끝 xy 가 목표와 ≤ 0.011 m 다름(허용 0.025 안) ② 'start' 끝 방향 0.785 rad → Spin 3 번 안에 0.07 로 못 들 수 있음 ③ EKF odom 의 스키드 회전 오차가 Cartographer 사전값을 끌어 다른 자리에서 새 튐 |
| G2 | 위치 0.01~0.03 m · yaw 0.01~0.07 rad(Spin 뒤) |
| G3 | 0~0.04 m, 겹침 0(L3 후진 호가 없어 후진하며 도는 동작 없음) |
| G4 | RMSE 0.005~0.015 m, 최대 ≤ 0.04 m(재생 v22 최대 0.033) |
| G5 | 통과 |

### 설계 39 결과(`results/live_nav_v39/`, 모의) — **불통과(G1 7/7 처음, G2 · G3 불통과)**
| 항목 | 기준 | 결과 | 예상과 비교 |
|---|---|---|---|
| G1 | 7/7 | **통과 7/7**(처음) — 마지막 start 도 SUCCEEDED(170 s) | 예상 범위 위 끝 |
| G2 | ≤ 0.10 m · ≤ 0.15 rad | 불통과 — 6 개 위치 0.009~0.036 · yaw 0.004~0.062, **start yaw 0.596**(위치 0.054) | 위험 ② 대로: 경로 끝 −2.357(목표 π) → 도착 뒤 Spin 3 번 모두 즉시 'Collision Ahead' |
| G3 | > 0 | 불통과 −0.001 · 17 표본 — 281.7~282.0 s L3 칸 (4.06, 5.51, 2.71 → 2.91) 후진 −0.04 · 회전 0.72~0.8(설계 38 과 같은 자리 · 같은 동작) | 예상 틀림: 호가 없어도 같은 쓸림 |
| G4 | RMSE ≤ 0.05 · 최대 ≤ 0.15 m | **통과 RMSE 0.0059 · 최대 0.017 m**(설계 38 최대 0.125) | 예상대로(재생 v22 0.033 보다 작음) — 마지막 구간 위치 튐 없어짐 |
| G5 | | 통과(vx −0.086~0.12) | |
- G3 원인: L3 마지막 후진 구간(0.07 m)이 RPP 전방 주시(최소 0.15 m)보다 짧아 주시점이 구간 끝에 붙음 → 곡률 2y/L² 이 L 작아 부풀어 회전 명령이 커짐(옆 벗어남 수 cm 에도). 호 여부와 무관.
- G2 원인: 해석 연장을 끈 뒤 Smac 이 허용(0.05 m) 안 노드를 찾은 뒤 100000 번 안에 정확한 방향 칸을 못 찾아 가장 가까운 노드(−2.357)로 끝냄. Spin 거부 이유는 확인 못 함 — 참값 형상으로 그 자리 회전 쓸림 여유 0.086 m 이상, 전역 · 물체 지도 점유까지 0.08~0.18 m(지도 탓 아님). 지역 costmap(라이다 층) 쪽으로 추정.
- 장면 없는 확인 `261031_1325_nav2/results/TOOLCHECK_plan_v39_approach1e6/`(마지막 두 구간, 탐색 1000000): start 끝 방향 −3.142(π), 8.9 s.

### 설계 40(기준 그대로) — 설계 39 + 끝 방향 탐색 늘림 · RPP 곡률 주시점 연장
- `nav2_params_v40.yaml`(`--variant v40`) = v39 와 다섯 값만 다름: Smac `max_on_approach_iterations` 100000 → 1000000 · `max_iterations` 1000000 → 5000000 · `max_planning_time` 10 → 15 s, RPP `use_fixed_curvature_lookahead` true · `interpolate_curvature_after_goal` true(곡률은 0.3 m 고정 주시점, 구간 끝을 넘으면 끝 두 자세 직선 연장 위 — 옆 벗어남 0.01 m 에 w 0.009 rad/s).
- 장면 없는 확인 `261031_1325_nav2/results/TOOLCHECK_plan_v40/`: 7 구간 경로 · 겹침 0, 자세 여유 0.026~0.048, 끝 호 0, start 끝 방향 π(7.7 s). launch · 목표 실행기 · 장면 인자는 설계 39 그대로(Carto v22 + /odometry/filtered).
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 6~7/7. 위험: ① 곡률을 0.3 m 앞에서 재 굽은 곳(회전 끊음 직전)에서 늦게 꺾어 경로를 벗어나 끝 허용 0.03 에 못 들어 진행 실패 ② start 끝 회전을 Shim 이 하다 지역 costmap 충돌로 멈춤 |
| G2 | 위치 0.01~0.04 m · yaw 0.01~0.07 rad(start 도 경로 끝이 π 라 Spin 몫 작음) |
| G3 | 0~0.04 m, 겹침 0(L3 후진 구간 회전 명령 작아짐) |
| G4 | RMSE 0.005~0.01 m · 최대 ≤ 0.04 m |
| G5 | 통과 |

### 설계 40 결과(`results/live_nav_v40/`, 모의) — **불통과(6/7, G3 통과)**
| 항목 | 기준 | 결과 | 예상과 비교 |
|---|---|---|---|
| G1 | 7/7 | 6/7 — 마지막 start 'Failed to make progress'(187 s) | 위험 ② 대로 |
| G2 | ≤ 0.10 m · ≤ 0.15 rad | 도착 6 개 위치 0.017~0.026 · yaw 0.002~0.061(모두 안), start 미도착(0.126 · 1.45) | 6 개는 예상대로 |
| G3 | > 0 | **통과 0.0041 m · 겹침 0**(L3 후진 쓸림 없어짐) | 예상대로 |
| G4 | | **통과** RMSE 0.0109 · 최대 0.033 m | 예상대로 |
| G5 | | 통과(vx −0.052~0.12, \|wz\| ≤ 0.70) | |
- 마지막 구간: 남북 통로 끝(회전 중심 ≈ (0.81, 0.93), 남향)까지 내려와 π 로 돌아야 하는데 RotationShim '회전 충돌'이 이어지고 복구 BackUp 도 'Collision Ahead' → 진행 실패.

### 도구 결함 발견 — 장면의 라이다 차체 가림 상자가 출발 자세 자리에 굳음(모의 장면 문제, 판정 아님)
- 찾은 것: 녹화 스캔(가림 상자 x −0.175~0.285 · |y| ≤ 0.19 밖, 참값 벽에서 0.05 m 넘게 떨어진 점)을 발제 좌표에 찍으면 **x 0.64~0.89 · y 0.80~1.10 한 자리에 고정**(로봇이 2~3 m 떨어져 있어도 같은 자리, v39 · v40 같음) = 출발 자세(0.734, 0.95, π)에 놓인 가림 상자들(sensors.json occluder_boxes)의 자리. 로봇이 출발 구역에 돌아오면 이 허상이 지역 costmap 장애물이 되어 제자리 회전 · Spin · BackUp 을 막음(설계 33~40 마지막 목표 'start' 실패와 같은 자리).
- 원인(추정): 레이캐스터는 가림 상자의 첫 강체 조상(`Geometry/base_link`, 강체이면서 관절 루트)의 PhysX 강체 view 로 자세를 따라가는데, 관절 링크라 자세가 갱신되지 않는 것으로 봄(Isaac Lab base_multi_mesh_ray_caster.py · isaaclab_physx ray_caster.py). 라이다 원점 자체는 따라감(스캔이 참값 자세로 벽과 0.4~0.6 cm). 예전 기록 '출발 뒤 약 5 s 부터 차체를 거의 못 봄'(scan_body_mask.py 설명)도 같은 현상.
- 고침(도구): `scene_live.py --occluder_mode analytic`(새 인자, 기본 raycaster = 옛 방식 그대로, 사본 `scene_live.py.before_occluder_mode_261101`) + 새 `lidar_occlusion.py`: 가림 상자와 라이다가 모두 base_link 에 붙어 있으므로 센서 frame 의 칸(0.9°)마다 상자 · 원기둥 교차로 가림 거리를 한 번 구해 스캔에 min. 레이캐스터 대상에서 가림 상자는 뺌.
- 도구 확인(장면 없이): 계산한 가림 칸과 v40 녹화의 출발 직후 1.9 s(상자가 아직 제자리) 스캔 비교 — 앞 153/154 칸 일치 · 거리차 중앙 0.8 mm · 최대 9 mm, 뒤 215/216 칸 · 0.3 mm · 4 mm(녹화에만 있는 뒤 95 칸은 출발 자리 뒤 0.22~0.30 m 의 실제 벽).
- 영향 · 한계: 1단계 SLAM 지도(live_bright pbstream)와 Nav2 지도도 이 허상이 있는 장면에서 만들어짐(Nav2 지도는 이미 '출발 자리 자기 차체 지운 복사본'). Cartographer 저장 지도에는 출발 자리에 허상 상자가 남아 있을 수 있음 — 고친 장면에서는 그 자리에 아무것도 안 보이므로 출발 구역 근처 위치 추정이 조금 달라질 수 있음(설계 40 다시 실행의 G4 로 봄). 지금까지의 설계 결과는 옛 장면 그대로 둠.

### 설계 40 다시(도구 고친 장면, `results/live_nav_v40_occfix/`) — 매개변수 · launch · 목표 실행기는 설계 40 그대로, 장면 인자에 `--occluder_mode analytic` 만 더함
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 6~7/7. 마지막 start 의 허상이 없어져 끝 회전 · 도착. 위험: Cartographer 저장 지도의 출발 자리 허상과 실제 스캔이 달라 출발 구역 근처 위치 추정이 흔들림, 출발 직후 첫 구간도 장면이 달라짐 |
| G2 | 위치 0.01~0.04 m · yaw 0.01~0.07 rad |
| G3 | 0~0.04 m, 겹침 0 |
| G4 | RMSE 0.005~0.015 m · 최대 ≤ 0.05 m |
| G5 | 통과 |
- 첫 시도 도구 실패(`results/live_nav_v40_occfix_TOOLFAIL_isaac_segv_last_goal/`): 마지막 구간 중 sim 390 s 에 Isaac 비정상 종료(exit 139, 설계 22 · 37 과 같은 현상) → start TIMEOUT. 판정에서 뺌(참고: 앞 6 개 도착 · 위치 0.016~0.026 · yaw 0.003~0.063, 390 s 까지 G3 0.022 겹침 0 · G4 RMSE 0.0058 · 최대 0.013). 같은 인자로 다시 실행.

### 설계 40 다시 결과(`results/live_nav_v40_occfix/`, 고친 장면, 모의) — **판정 통과(처음)**
| 항목 | 기준 | 결과 | 예상과 비교 |
|---|---|---|---|
| G1 | 7/7 | **통과 7/7**(start 90.9 s) | 예상대로 |
| G2 | ≤ 0.10 m · ≤ 0.15 rad | **통과** 위치 0.007~0.027 · yaw 0.004~0.060 | 예상대로 |
| G3 | > 0 | **통과** 0.0078 m · 겹침 0 | 예상대로 |
| G4 | RMSE ≤ 0.05 · 최대 ≤ 0.15 m | **통과** RMSE 0.0070 · 최대 0.020 m | 예상대로(Cartographer 지도의 출발 자리 허상 영향 안 보임) |
| G5 | | 통과(vx −0.052~0.12, \|wz\| ≤ 0.70) | |
- 허상 확인: 상자 밖 · 참값 벽에서 0.05 m 넘는 라이다 점 v40 107,348 개(그중 출발 자리 107,343) → 0 개.
- 남은 것: 반복. 선례(설계 7~11, 3회 모두 통과해야 설계 통과)대로 장면 잡음 seed 만 바꿔 2 회 더(`--imu_seed 1`, `--imu_seed 2`; 이번 판 = 기본 20261101). 매개변수 · launch · 목표 · 기준 그대로. 예상: 각 G1~G5 통과, 값은 이번 판 범위(G3 0.005~0.03, G4 최대 ≤ 0.04). 위험: Isaac 세그폴트(도구 실패로 다시), 출발 모퉁이 · L3 칸 경계 동작.

### 설계 40 반복(고친 장면, 장면 잡음 seed 만 다름, 모의) — **설계 불통과(3 회 중 2 회 통과)**
| 회 | 결과 | G1 | G2 | G3 | G4 RMSE · 최대 |
|---|---|---|---|---|---|
| `live_nav_v40_occfix`(seed 20261101) | 통과 | 7/7 | 0.027 · 0.060 | 0.0078 | 0.0070 · 0.020 |
| `live_nav_v40_occfix_r1`(--imu_seed 1) | **불통과** | 5/7 — L3 'Failed to make progress'(yaw 0.85 남음) → start 'Start occupied'(L3 칸 안) | — | 0.0226 | 0.0054 · 0.015 |
| `live_nav_v40_occfix_r2`(--imu_seed 2) | 통과 | 7/7 | 0.023 · 0.063 | 0.0181 | 0.0070 · 0.017 |
- r1 L3 원인: 칸 안에서 두 번 제자리 회전한 뒤 회전 중심이 0.09 m 후진 구간 선에서 옆으로 0.031 m(차체 회전축이 회전 중심과 조금 달라 회전마다 1~2 cm 밀림). 곡률 주시 0.3 m 라 회전 명령이 약해(w ≈ 0.03) 옆 오차를 못 고치고, 구간 끝까지 거리 0.031(허용 0.03)에서 앞뒤로 떨다 15 s 진행 검사 실패(266~270 s v ±0.04).

### 설계 41(기준 그대로) — 설계 40 + 곡률 주시 거리 0.3 → 0.15 m
- `nav2_params_v41.yaml`(`--variant v41`) = v40 과 RPP `curvature_lookahead_dist` 만 다름(0.15 = 전방 주시 최소와 같게). 옆 0.03 m 에 w ≈ 0.11 rad/s(0.3 이면 0.03, 설계 38 · 39 의 주시점 0.05 m 붙음이면 0.5~0.8). 구간 끝 너머 연장(V40)은 그대로. launch · 목표 실행기 · 장면(--occluder_mode analytic) 그대로.
- 반복: 처음부터 3 회(장면 seed 20261101 · 1 · 2), 3 회 모두 G1~G5 통과해야 설계 통과.
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 3 회 모두 7/7. 위험: 곡선 구간(전진 중 회전)에서 더 세게 꺾어 흔들림, L3 후진 구간에서 쓸림이 다시 생김(설계 38 · 39 보다는 약함) |
| G2 | 위치 0.01~0.03 m · yaw 0.01~0.07 rad |
| G3 | 0.005~0.03 m, 겹침 0 |
| G4 | RMSE 0.005~0.01 · 최대 ≤ 0.04 m |
| G5 | 통과 |

### 설계 41 결과(모의) — **불통과**(반복 중단)
| 회 | 결과 |
|---|---|
| `live_nav_v41_s20261101_TOOLFAIL_isaac_segv_first_goal` | 도구 실패: sim 약 30 s(첫 목표 중) Isaac 비정상 종료 |
| `live_nav_v41_s1`(--imu_seed 1) | **불통과**: L1 'Failed to make progress'(129 s) · G3 −0.001(128 s, L1 칸) → 이어 'Start occupied'. Isaac 은 그 뒤 약 137 s 에 비정상 종료(L1 실패는 그 전이라 실제 결과로 봄) |
| s2 | 실행 안 함(s1 불통과로 설계 41 불통과가 정해짐) |
- s1 L1: 회전 뒤(2.33) RotationShim 이 다음 회전 구간 방향으로 더 돌며 2.68 → 2.82 로 0.14 rad 지나치고, 명령 0 인 125.5~127.5 s 에도 2 cm 미끄러져 벽에 닿음. 곡률 주시(V41)와는 직접 관계없어 보임(경로가 회마다 조금씩 다름 — L1_via 도착 자세 차이로 Lattice 경로가 바뀜).
- 계획 쪽 근거: 장면 없는 확인에서 L1 구간 쓸림 여유 0.0103 m(TOOLCHECK_plan_v40) — 설계 31 의 '중간 자세 없는 회전 원시 동작'은 회전 쓸림을 검사하지 않아, 여백 0.04 가 자세에만 걸림. 회전 지나침 0.1 rad · 미끄럼 2 cm 면 닿는 구조.
- Isaac 비정상 종료가 잦아짐(최근 5 판 중 3 판). 충돌 직전 기록에 공통 원인 없음(옛 장면 v37 도 같음).

### 설계 41 s1 L1 원인 정정(bag 명령 · Nav2 소스, 판정 아님)
- 위 '0.14 rad 지나침 · 명령 0 인데 2 cm 미끄러짐'은 잘못 읽은 것. /cmd_vel_ctrl 을 50 ms 단위로 보면 125~128 s 에 Shim 회전(가속 제한으로 0.014 → 0.054 → 0.096 → 0.136)과 RPP(전진 0.04 · w −0.03)가 2~4 주기마다 번갈아 나옴. Nav2 RotationShim 은 회전 시뮬레이션이 충돌이면 그 주기를 주 제어기(RPP, 자체 충돌 검사 끔 = V38)로 넘김(ros-navigation/navigation2 main `nav2_rotation_shim_controller.cpp`). turn_floor 가 0.054 같은 작은 회전을 0.355 로 올려 실제로 2.68 → 2.82 돌았고, 번갈아 나온 RPP 전진(최종 /cmd_vel_nav 0.025)으로 2 cm 나가 벽에 닿음(G3 −0.001).
- 회전이 막힌 까닭: 제자리 회전 시작 자리(참값 회전 중심)가 경로의 회전 자리보다 진행 방향으로 앞에 있음. 도구 `261031_1325_nav2/test/cut_err.py <결과 폴더>`(판정 아님): v41_s1 31 번 중앙 0.022 · 최대 0.029 m, v40_occfix_r1 45 번 중앙 0.023 · 최대 0.050 m, 대부분 앞뒤 성분 = 끊는 점 허용 inversion_xy_tolerance 0.03 에 들어서자마자 끊음. L1 칸 계획 쓸림 여유는 0.0103 m 라 이만큼 덜 가서 돌면 닿음. 설계 40 r1 L3 의 옆 0.031 도 앞 회전을 덜 간 자리에서 한 결과(덜 간 거리 × sin 회전각)로 봄.

### 설계 42(기준 그대로) — 설계 41 + 끊는 점 도착 허용 0.03 → 0.015 · 전역 팽창 기울기 6 → 3
- `nav2_params_v42.yaml`(`--variant v42`, 사본 `make_nav2_params_v27.py.before_v42_261101`, v27~v41 생성 파일 바이트 같음 확인) = v41 과 두 값만 다름: PathHandler `inversion_xy_tolerance` 0.015, 전역 costmap `inflation_layer.cost_scaling_factor` 3.0. 곡률 주시 0.15(V41)는 허용을 좁히면 옆 오차를 고칠 힘이 더 필요해 유지.
- 장면 없는 확인(판정 아님): 비용 사본 비교 `261031_1325_nav2/results/TOOLCHECK_plan_v42_{cp5,cp10,cs3,cs3cp5}/`(사본 생성 `TOOLCHECK_plan_v42_cs3/make_variant_copies.py`) — cost_penalty 5 · 10 은 v40 과 거의 같음, 기울기 3 이 열린 구간 쓸림을 늘림. 실제 v42 파일 `TOOLCHECK_plan_v42/`: 7 구간 겹침 0, 쓸림 여유 0.0485~0.0588(L1 → L2 0.0089 → 0.0485), 좁은 칸 L1_via → L1 0.0103 · L3 0.0268 은 그대로(칸에 다른 길 없음).
- launch · 목표 실행기 · 장면 인자는 설계 40 다시와 같음(장면 `--imu_topic /imu/data --occluder_mode analytic --imu_seed N`). 3 회(seed 20261101 · 1 · 2) 모두 통과해야 설계 통과.
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 3 회 모두 7/7. 위험: 옆 오차가 0.015 를 넘는 구간 끝에서 앞뒤로 떨다 진행 실패(설계 40 r1 꼴), 구간 끝 접근이 느려져 시간이 늘어남 |
| G2 | 위치 0.01~0.03 m · yaw 0.01~0.07 rad |
| G3 | 0.005~0.03 m, 겹침 0(회전 시작이 회전 자리에 0.015 안으로 붙음) |
| G4 | RMSE 0.005~0.01 · 최대 ≤ 0.04 m |
| G5 | 통과 |
| 도구 확인 cut_err | 회전 시작 자리 오차 중앙 ≤ 0.015 m(v40 · v41 0.022) |

### 설계 42 결과(모의) — **불통과**(첫 판에서 중단)
| 회 | 결과 |
|---|---|
| `live_nav_v42_s20261101` | **불통과**: start_box_1 'Failed to make progress'(출발 모퉁이 끝), 이어 6 개 'Start occupied'. G3 0.0443 · 겹침 0, G4 RMSE 0.0072 · 최대 0.012, G5 통과 |
| s1 · s2 | 실행 안 함(run_reps.sh 규칙) |
- 예상과 비교: cut_err(회전 시작 자리 오차) 3 번 중앙 0.008 · 최대 0.013 m = 예상대로(v40 · v41 0.022). G1 은 예상 밖.
- 실패 모습: 모퉁이 마지막 회전 자리 P2 (0.6668, 0.2876) 에서 동쪽(0)으로 돈 뒤 RPP 가 0.12 m/s 로 0.07 m 나가다 34.7 s 부터 제어기 출력이 정확히 0(경고 없음) → 15 s 진행 검사 실패. 그동안 제어기 경고 '구간 끝이 목표에서 1.68 m' = 구간 끝이 P2 에 머묾(다음 구간 끝 193 번이면 1.04 m). 
- 확인한 것: TF 기준 회전 중심은 P2 에서 0.008~0.011 m · 방향 0.2 rad 안이었음. Nav2 소스(main `feasible_path_handler.cpp`)대로면 구간 끝(up-to-constraint 마지막 자세)과 xy · yaw 허용 안이면 넘어가야 함. 남은 경로를 설치 함수(probe)에 넣으면 P2 다음 구간은 193 번까지 긴 직선. **왜 안 넘어갔는지는 지금 bag(전역 경로 · 명령 · TF)으로 확인 못 함.**
- 함께 찾은 것: 이 판의 경로에는 회전 자리 사이 0.010 m 후진 구간이 있었음(P1 → P2). 새 도구 `261031_1325_nav2/test/short_segments.py <plan_only.json>`(판정 아님)로 보면 이런 0.03 m 미만 구간은 v40 · v42 · v43 경로 모두 1~5 개, 자리는 계획할 때마다 다름 — Smac 경로가 같은 설정에서도 실행마다 달라짐(TOOLCHECK_plan_v40 과 v43 은 계획기 설정이 같은데 L1 구간 쓸림 0.0103 vs 0.0327). 그래서 '팽창 기울기 3 이 짧은 구간을 만들었다'는 해석은 근거가 약함.

### 설계 43(기준 그대로) — 설계 41 + 끊는 점 도착 허용 0.015(설계 42 에서 팽창 기울기 3 은 뺌) + 제어기 구간 기록
- `nav2_params_v43.yaml`(`--variant v43`, 사본 `.before_v43_261101`, v27~v42 바이트 같음) = v41 과 PathHandler `inversion_xy_tolerance` 0.015 만 다름. 팽창 기울기를 뺀 이유: 설계 42 실패 자리를 바꾼 값 두 개 중 어느 것이 만들었는지 가르기 위해(기울기 3 의 쓸림 이득은 경로가 실행마다 달라 확실하지 않음).
- 도구 확인 `261031_1325_nav2/results/TOOLCHECK_plan_v43/`: 7 구간 겹침 0, 쓸림 0.0268~0.0574.
- 기록 추가(도구): `run_live_nav2.sh` 에 환경 변수 `EXTRA_TOPICS`(사본 `.before_extra_topics_261101`, 기본은 예전 그대로). 이번엔 `/received_global_plan /lookahead_point /local_plan /transformed_global_plan /cmd_vel_smoothed /cmd_vel_cm` 를 더 녹화해 제어기가 받은 구간을 봄.
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 3 회 모두 7/7. 위험: 설계 42 와 같은 출발 모퉁이 멈춤(허용 0.015 가 원인이면 다시 남), 옆 오차 > 0.015 구간 끝에서 앞뒤로 떪 |
| G2 | 위치 0.01~0.03 m · yaw 0.01~0.07 rad |
| G3 | 0.005~0.03 m, 겹침 0 |
| G4 | RMSE 0.005~0.01 · 최대 ≤ 0.04 m |
| G5 | 통과 |
| cut_err | 회전 시작 자리 오차 중앙 ≤ 0.015 m |

### 기록 정정 — 설계 30 이후 '직선 + 제자리 회전' 위주가 사용자 결정(2026-10-24)과 다름
- 사용자 결정(STATUS.md, 2026-10-24): 제자리 회전 최소화가 기본(모퉁이는 호 회전, 막다른 길은 후진), 제자리 회전이 필요하면 돈 뒤 위치 정상화 후 재주행.
- 설계 30 에서 계획기를 '직선 + 제자리 회전'(호 없음)으로 바꾼 근거는 설계 29 결과(반경 0.10 호를 스키드가 못 따름)와 생성기 V30 설명에 있으나, 위 결정과 다르다는 점은 적지 않았고 사용자 확인도 받지 않았음(2026-11-02 사용자 질문으로 발견). 호를 시험한 것은 설계 27(Hybrid · MPPI) · 29(Hybrid r 0.10 · RPP) 두 번뿐이고, 둘 다 전진 중 회전 앞먹임(move_ff, 설계 32) 전이며 반경도 0.10 하나뿐.
- 돈 뒤 위치 정상화는 지금 구성에 없음 — 설계 40 r1 · 41 s1 의 좁은 칸 실패(회전마다 1~3 cm 어긋남이 쌓임)와 관련.

### 설계 43 결과(모의) — **불통과**(3 회 중 1 회 통과, s2 안 돌림)
| 회 | 결과 | G1 | G2 위치 · yaw 최대 | G3 | G4 RMSE · 최대 | cut_err 중앙 · 최대 |
|---|---|---|---|---|---|---|
| `live_nav_v43_s20261101` | **통과** | 7/7 | 0.029 · 0.063 | 0.0216 · 겹침 0 | 0.0069 · 0.018 | — |
| `live_nav_v43_s1_TOOLFAIL_isaac_segv_try1` | 도구 실패(Isaac exit 139) — 같은 seed 로 다시 | | | | | |
| `live_nav_v43_s1` | **불통과**(G3 만) | 7/7 | 0.028 · 0.052 | **−0.001 · 겹침 22**(131.9~133.9 s, L1 칸) | 0.0060 · 0.017 | 0.0058 · 0.0225(91 번) |
- 예상과 비교: G1 · G2 · G4 · G5 · cut_err(중앙 ≤ 0.015) 는 예상대로. 설계 42 의 출발 모퉁이 멈춤은 두 판 모두 안 남 → 설계 42 실패는 끊는 점 허용 0.015 탓이 아님(팽창 기울기 3 또는 그 판 경로 탓, 확정 못 함).
- s1 L1 닿음(새로 녹화한 /transformed_global_plan · /lookahead_point 로 확인): 제어기 구간 = 회전 끝 자리 한 점에 자세 2 개. 회전 중심은 그 점에서 xy 0.004~0.010 m(허용 0.015 안)인데 방향이 0.30~0.32 rad 모자람(허용 0.2 밖) → 회전해야 하나 Shim 회전이 충돌 검사로 막히고, 그 주기마다 RPP 가 ±0.04 m/s 앞뒤로 떨어(설계 41 s1 과 같은 구조) 벽에 1 mm. 약 10 s 뒤 돌고 목표 도착.
- 판단: 좁은 L1 칸의 제자리 회전이 남은 실패의 공통 원인(설계 40 r1 · 41 s1 · 43 s1). 사용자 결정(2026-11-02): 설계 44 는 2026-10-24 결정대로 '호 회전 기본 · 꼭 필요한 곳만 제자리 회전 · 돈 뒤 위치 정상화'.

### 설계 44(기준 그대로) — 설계 43 + 호 원시 동작 · 회전 벌점 20(사용자 결정 '제자리 회전 최소화'로 되돌림, 2026-11-02)
- `nav2_params_v44.yaml`(`--variant v44`, 사본 `make_nav2_params_v27.py.before_v44_261102`, v27~v43 바이트 같음) = v43 과 두 값만 다름: Smac Lattice `lattice_filepath` → `config/lattice_straight_rot_arc25_0025.json`(직선 + 제자리 회전 + 방향 ±1 · ±2 칸 호 64 개, 반지름 0.25~0.34 m, `make_lattice_straight_rot.py --no_rot_mid --arc_r_min 0.25 --arc_r_max 0.6`, 사본 `.before_arc_261102`, 호 없는 옛 출력은 그대로), `rotation_penalty` 5 → 20.
- 반지름 근거(도구 확인, 새로 안 잼): `results/TOOLCHECK_turn_moving/` 정상 상태 호 — 0.08~0.12 m/s 에서 base 반지름 0.23~0.32 m(명령 w 0.6~0.8, 실제 0.40~0.58). 설계 29 의 호 실패는 반지름 0.10 · 전진 앞먹임(move_ff) 없을 때.
- 장면 없는 확인(판정 아님, `261031_1325_nav2/results/TOOLCHECK_plan_v44_rp{5,20,50}/`, 통계 `test/path_stats.py`):
| 구성 | 제자리 회전 | 호 | 짧은 이동 구간 | 최소 쓸림 여유 |
|---|---|---|---|---|
| 설계 43(호 없음, TOOLCHECK_plan_v43) | 90 번 · 2033° | 0 m | 4 | 0.0268 |
| 호 + 벌점 5 | 19 번 · 432° | 5.11 m | 0 | 0.0164 |
| **호 + 벌점 20(설계 44)** | **4 번 · 90°**(출발 모퉁이 · L1 칸 0) | 5.81 m | 0 | **0.0331**(L1 구간 0.0103 → 0.0341) |
| 호 + 벌점 50 | 4 번 · 90° | 6.15 m | 0 | 0.0164 |
- 돈 뒤 위치 정상화: 회전 전 자리 맞춤 = 설계 43 끊는 점 허용 0.015(cut_err 중앙 0.006), 돈 뒤 옆 어긋남 = RPP 곡률 주시 0.15 가 고침. 따로 되돌아가는 단계는 넣지 않음(Nav2 에 없는 기능, 제자리 회전 자체를 4 번으로 줄임).
- launch · 목표 실행기 · 장면 인자는 설계 40 다시와 같음, 녹화에 `EXTRA_TOPICS`(제어기 구간) 그대로. 3 회 모두 통과해야 설계 통과.
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 3 회 모두 7/7. 위험: ① RPP 가 반지름 0.25~0.34 호를 덜 돌아 바깥으로 밀림(곡률 큰 곳 속도 0.04 까지 줄어 move_ff 곡선 아래쪽, 전진 중 \|w\| ≤ 0.1 은 안 돎) ② 후진 호 구간을 RPP 가 못 따름 ③ Shim 이 호 시작에서 0.25 rad 넘게 틀어졌다고 보고 제자리 회전 |
| G2 | 위치 0.01~0.03 m · yaw 0.01~0.07 rad |
| G3 | 0.01~0.04 m, 겹침 0(호 경로 쓸림 여유 0.033 이상) |
| G4 | RMSE 0.005~0.012 · 최대 ≤ 0.04 m |
| G5 | 통과 |
| 제자리 회전(경로) | 목표마다 0~1 번(도착 뒤 Spin 방향 맞춤은 별도) |

### 설계 44 결과(모의) — **불통과**(첫 판에서 중단)
| 회 | 결과 |
|---|---|
| `live_nav_v44_s20261101` | **불통과**: start_box_1 · L1_via 도착(호 경로로 출발 모퉁이 통과), L1 'Failed to make progress'(22.9 s), 이어 4 개 'Start occupied'. G3 0.0199 · 겹침 0, G4 RMSE 0.0067 · 최대 0.018, G5 통과(vx −0.070~0.12) |
| s1 · s2 | 실행 안 함 |
- L1 실패(녹화한 /transformed_global_plan 과 TF 로 확인): L1 칸 전진 ↔ 후진 끊는 점(제어기 구간 = 자세 2 개, 0.005 m)에서 앞뒤 오차 −0.002~0.008 · **옆 0.016~0.021 m(허용 0.015 밖)** · 방향 차 0.196~0.215 rad(허용 0.2 경계)로 서서 구간이 안 넘어감 → RPP 가 ±0.04 m/s 앞뒤로 떨다 진행 실패. 호를 따라온 끝점에 옆 오차가 남고, 앞뒤 오차가 0 이라 RPP 가 옆을 고치지 못함. L1_via(55 s)도 같은 떨림이 섞임.
- 예상 위험 ① · ② 쪽(호 · 후진 호 추종 오차)이 끊는 점 허용 0.015 와 겹친 것.

### 설계 45(기준 그대로) — 설계 44 + 구간 끝 허용 xy 0.015 → 0.03 · yaw 0.2 → 0.28
- `nav2_params_v45.yaml`(`--variant v45`, 사본 `.before_v45_261102`, v27~v44 바이트 같음) = v44 와 PathHandler `inversion_xy_tolerance` 0.03 · `inversion_yaw_tolerance` 0.28 만 다름. 0.015 는 좁은 칸 제자리 회전 시작 자리를 맞추려던 값인데 설계 44 경로에는 좁은 칸 제자리 회전이 없음. yaw 0.28 < 제자리 회전 한 칸 0.321 rad(회전 건너뜀 방지). 계획기는 그대로라 장면 없는 확인은 TOOLCHECK_plan_v44_rp20 과 같음.
| 항목 | 예상(실행 전) |
|---|---|
| G1 | 3 회 모두 7/7. 위험: 끊는 점을 0.03 앞에서 넘겨 좁은 L1 · L3 칸에서 다음 후진 호가 옆 0.03 에서 시작 → 벽 쪽 쓸림, 남은 제자리 회전(경로 4 번)에서 설계 41 s1 꼴 |
| G2 | 위치 0.01~0.03 m · yaw 0.01~0.07 rad |
| G3 | 0.005~0.03 m, 겹침 0 |
| G4 | RMSE 0.005~0.012 · 최대 ≤ 0.04 m |
| G5 | 통과 |

### 기록 정정 — 2단계 Nav2 위치 추정에 cuVSLAM 이 빠진 것(사용자 질문 2026-11-02)
- 사용자 구성(STATUS.md 10-29 · STATUS_yolo 10-27): cuVSLAM 오도메트리를 Cartographer 입력으로(OpenCR IMU 를 믿기 어렵다는 이유). 실물 계획은 EKF odom1 = cuVSLAM 속도, 고르는 것은 실물 bag 비교(B4).
- 2단계(설계 1~45)는 EKF = 바퀴 vx + 장면 IMU yaw rate(설계 14, 팀 ekf.yaml 꼴), 설계 39 부터 Cartographer 가 그 EKF odom 을 받음. cuVSLAM 은 한 번도 넣지 않았고, 뺀다는 결정 · 기록도 없었음.
- 정황(결정 아님): 10-29 결합 모의(옛 모델 · 옛 맵)에서 cuVSLAM→Carto 위치 RMSE 0.293 / 0.370 m(밝음 / 어두움, 바퀴 odom Carto 0.203 / 0.204), 헤딩 8.2°, 튐 7/11 로 기준 미달. cuVSLAM 단독 1.5~2.9 m(접은 팔이 앞 IR 아래를 가림 등). PC GPU 8 GB 라 Isaac 실시간과 동시에 돌리기도 어려움.
- 한계: 2단계 결과는 '이상적 장면 IMU 를 믿는 구성'의 결과라, 사용자 구성(IMU 대신 cuVSLAM)에서 같은 성능이 나온다는 근거는 아님.

### 설계 45 결과(모의, 진행 중)
| 회 | 결과 | G1 | G2 위치 · yaw 최대 | G3 | G4 RMSE · 최대 |
|---|---|---|---|---|---|
| `live_nav_v45_s20261101` | **통과** | 7/7 | 0.033 · 0.066 | 0.0123 · 겹침 0 | 0.0072 · 0.023 |
| `live_nav_v45_s1` | **통과** | 7/7 | 0.031 · 0.059 | 0.0102 · 겹침 0 | 0.0057 · 0.012 |
| `live_nav_v45_s2_TOOLFAIL_isaac_segv_try1` | 도구 실패: 마지막 목표 중 Isaac exit 139(앞 6 개 도착) | | | | |
| `live_nav_v45_s2_TOOLFAIL_no_map_odom_try2` | 도구 실패: map→odom 한 번도 안 나옴 → 목표 실행기 'mission_map→base_link TF 없음(60 s)'. map_odom_relay 가 /tracked_pose(905 개) 시각의 odom→base_link 를 못 찾음(계산 0 · 못 찾음 349, 요청 시각이 EKF TF 최신보다 0.4~1.0 s 앞섬). 설계 값과 무관한 시작 시 시각 어긋남, 원인 미확인 | | | | |
| `live_nav_v45_s2_TOOLFAIL_isaac_segv_try3` | 도구 실패: sim 259 s(마지막 목표 중) Isaac exit 139, 앞 6 개 도착 | | | | |
| `live_nav_v45_s2_TOOLFAIL_isaac_segv_try4` | 도구 실패: sim 29 s Isaac exit 139 | | | | |
| `live_nav_v45_s2`(5 번째 시도) | **통과** | 7/7 | 0.038 · 0.073 | 0.0170 · 겹침 0 | 0.0066 · 0.017 |
- Isaac 비정상 종료 시각은 sim 29~390 s 로 제각각(v37 · v40 · v41 · v43 · v45 기록), 특정 목표 · 장면과 무관해 보임. GPU 메모리 · 시스템 메모리 여유는 실행 사이에 정상(GPU 532/8151 MiB 사용, RAM 가용 21 GB). 원인 미확인.
- 도구 결함 고침: run_reps.sh 가 실패 폴더 이름을 시도 번호로만 지어 앞 실행 폴더와 겹쳤음(3 번째 시도 폴더가 1 번째 안으로 들어가고 콘솔 기록을 덮어씀) → 비어 있는 번호를 고르게 고침.

**설계 45 판정: 통과(3 회 모두 G1~G5 통과, 모의).** 예상과 비교: G1 · G2 · G4 · G5 예상대로, G3 0.010~0.017(예상 0.005~0.03 안). 위험으로 적은 '끊는 점을 0.03 앞에서 넘겨 좁은 칸 쓸림'은 안 나옴. seed 2 는 도구 실패 4 번 뒤 5 번째에 끝까지 돎.
- 2단계 고정 구성 = 설계 45: params `261031_1325_nav2/config/nav2_params_v45.yaml`, launch 인자 = `results/live_nav_v45_s2/exit_codes.txt` 마지막 줄, 목표 실행기 `--frame mission_map --base_offset_x -0.0786 --final_spin_tol 0.07 --final_spin_max 3`.
- 한계: 모의(Isaac 장면 IMU 이상적 · 장면 브리지는 Isaac Lab rclpy), seed 1 목표 배치 하나, 밝음만, cuVSLAM 미포함, loc_monitor(/localization/status) 끈 상태(C1 문턱 미정), 실물 값(회전 곡선 · 회전 중심) 아님.

### C1 문턱 근거 ① 정상 주행 잔차(도구, 판정 아님) — 설계 45 세 판 bag(고친 장면)
- 도구 `261031_1325_nav2/test/check_reloc_offline.py <결과 폴더>`(시스템 python3, `source /opt/ros/lyrical/setup.bash`), 결과 각 `results/live_nav_v45_*/reloc_offline.json · .log`. 옛 장면(설계 19 · 20, 출발 자리 허상) bag 대신 고정 구성 bag 을 씀.
| 판 | 스캔 쌍 | 추정 자세 잔차 p50 · p95 · p99 · 최대 | 참값 자세 잔차 최대 | 0.20 m · 10° 어긋난 자세 잔차 | 근처 · 전체 탐색 성공 | 전체 탐색 2등−1등 점수 p50 · 최소 쪽 |
|---|---|---|---|---|---|---|
| s20261101 | 3256 | 0 · 0 · 0.025 · 0.025 | 0.025 | 0.100 | 30/30 · 30/30 | 0.020 · p95 0.043 |
| s1 | 3068 | 0 · 0 · 0.025 · 0.025 | 0.025 | 0.100 | 30/30 · 30/30 | 0.022 · p95 0.042 |
| s2 | 3098 | 0 · 0.025 · 0.025 · 0.025 | 0.025 | 0.100 | 30/30 · 30/30 | 0.022 · p95 0.047 |
- 해석: 잔차(스캔 끝점 거리 중앙값)는 거리장 칸 0.025 m 단위로 나와 정상 주행은 최대 한 칸(0.025), 0.20 m · 10° 어긋나면 0.100. loc_monitor 기본 `res_thr` 0.03 은 정상 최대보다 0.005 위라 여유가 매우 작음 → 지도에 없는 물체(퍽 · 상자)가 스캔에 들어오면 정상 주행에서도 넘을 수 있음 → ② 물체 장면 실행으로 확인해야 문턱을 정할 수 있음.

### C1 문턱 근거 ② 물체 장면(도구, 판정 아님) — `results/TOOLCHECK_c1_objects_v45/`
- 설계 45 고정 구성 그대로(loc_monitor 끔), 장면에만 `--objects …/dice_mission_maze261030/seed_1/layout.json`(상자 · 퍽 · 신호등 — Cartographer 저장 지도에는 없음, Nav2 물체 지도에는 있음). 녹화 bag 으로 ① 과 같은 잔차 분석.
| 항목 | 예상(실행 전) |
|---|---|
| 정상 잔차(추정 자세) | 최대 0.025~0.05 m(물체 점은 지도 벽에서 멀어 잔차를 키우지만, 중앙값이라 스캔의 일부면 덜 움직임) |
| 주행 | 7/7(물체는 Nav2 물체 지도에 있음), 위험: 라이다가 물체를 보며 Cartographer 위치 추정이 흔들림 |
- ② 결과(`TOOLCHECK_c1_objects_v45`): 7/7 · G3 0.0106 · G4 RMSE 0.0061, 잔차 최대 0.025(물체 없을 때와 같음). **한계: 상자 · 받침대(0.07 m) · 퍽은 라이다 높이(약 0.082 m)보다 낮아 스캔에 거의 안 들어옴** → '지도에 없는 물체가 스캔에 들어온 경우' 시험이 아님.
- ②-b 다리 기둥(`results/TOOLCHECK_c1_legs_v45/`): 위 + `--legs 1.585 2.485 2.965 1.465 1.425 4.565 3.825 6.085`(지름 0.1 · 높이 0.8 m, 라이다에 보임, Nav2 물체 지도에는 없음). 자리 = 장면 없는 계획 경로(TOOLCHECK_plan_v44_rp20)에서 0.57~0.63 m · 벽에서 ≥ 0.19 m 인 열린 칸(판정 K0 는 다른 자리로).
| 항목 | 예상(실행 전) |
|---|---|
| 정상 잔차(추정 자세) | 기둥이 보이는 동안 0.025~0.05 m(기둥 점은 스캔 400 칸 중 수 칸이라 중앙값은 거의 안 움직임) |
| 주행 | 7/7, 위험: 지역 costmap 이 기둥을 피하며 경로에서 벗어남 |
- ②-b 결과(`TOOLCHECK_c1_legs_v45`): 7/7 · G3 0.0095 · 겹침 0 · G4 RMSE 0.0058, 잔차 p95 0.025 · 최대 0.025 — 기둥이 보여도 중앙값은 안 움직임(예상대로).

### C1 문턱 근거 ③ 점프 · 정한 문턱(도구, 판정 아님)
- 점프 도구 `261031_1325_nav2/test/jump_stats.py <결과 폴더...>`: /tracked_pose 연속 표본 사이 이동 − 같은 구간 EKF 이동. 다섯 판(설계 45 세 판 · 물체 · 기둥): xy p99 0.0043~0.0048 · **최대 0.0073~0.0094 m**, yaw p99 0.86~0.96 · **최대 1.10~1.46°**.
- 잔차 다섯 판 합계 스캔 쌍 16,025 개에서 **최대 0.025**(한 칸), 0.20 m · 10° 어긋난 자세 0.100.
| 문턱 | 값 | 이유 |
|---|---|---|
| `res_thr`(잔차) | **0.03 그대로** | 잔차가 0.025 칸 단위라 (0.025, 0.05) 안 어떤 값이든 같게 동작 = 두 칸(0.05) 이상이면 넘음. 정상 최대 0.025 는 한 번도 안 넘음, 0.20 m · 10° 어긋남(0.100)은 넘음 |
| `suspect_s` · `lost_s` | 0.5 · 2.0 s 그대로 | 정상 주행에서 잔차가 문턱을 넘은 적이 없어 지속 시간은 실제 잃음에만 걸림. 2 s × 0.12 m/s = 0.24 m 이내에 LOST |
| `jump_xy` · `jump_yaw_deg` | 0.10 m · 5° 그대로 | 정상 최대 0.0094 m · 1.46° 의 약 10 배 · 3.4 배. 설계 38 무렵 Cartographer 미끄럼 튐 0.11~0.25 m(옛 lua v19)는 잡힘 |
- 한계: 모의만, 어긋남은 0.20 m · 10° 한 가지만 봄(0.1 m 미만 어긋남은 잔차 한 칸 안이라 못 잡을 수 있음), 다리 기둥은 고정 4 개, 판정 K0 는 다른 기둥 자리 · loc_monitor:=true 로 따로.

### C1 ④ 잡을 수 있는 가장 작은 어긋남(도구, 판정 아님, 인지 세션 권함 2026-11-02)
- 도구 `261031_1325_nav2/test/min_offset.py <결과 폴더> 0 [res_thr]`(앞부분은 check_reloc_offline.py 와 같음): 참값 자세를 이동만(방향 무작위) · 회전만 어긋나게 해 스캔 한 장마다 잔차가 res_thr 0.03 을 넘는 비율. 결과 각 `min_offset.json`.
| 어긋남 | v45_s1 넘는 비율 · 잔차 p50 | 기둥 장면 넘는 비율 · p50 |
|---|---|---|
| 이동 0.025 · 0.035 m | 0 · 0.002 | 0 · 0 |
| 이동 0.05 m | 0.121 · 0.025 | 0.157 · 0.025 |
| 이동 0.075 m | 0.723 · 0.05 | 0.752 · 0.05 |
| 이동 0.10 m | 0.852 · 0.0525 | 0.845 · 0.0625 |
| 이동 0.15 · 0.20 m | 0.907 · 0.943 | 0.927 · 0.963 |
| 회전 1 · 2° | 0 · 0.003 | 0 · 0.016 |
| 회전 3° | 0.132 · 0.025 | 0.135 · 0.025 |
| 회전 5° | 0.537 · 0.035 | 0.542 · 0.035 |
| 회전 7° | 0.796 · 0.05 | 0.800 · 0.05 |
| 회전 10° | 0.984 · 0.075 | 0.978 · 0.075 |
- 해석: 스캔 한 장 기준으로 이동 ≥ 0.10 m · 회전 ≥ 7° 는 80~98 % 넘음. 이동 0.035~0.05 m · 회전 2~3° 는 0~16 % 라 suspect_s 0.5 s 동안 이어서 넘기 어려움 → **이 범위 어긋남은 감지 못 한다고 봐야 함**(이동 0.075 m · 회전 5° 는 반쯤). 지속 시간 조건까지 넣은 감지 확률은 K0 판정 실행에서 봄.

### K0 판정(위치 잃음 시험, 모의) — `results/K0_v45/`
- 기준(사용자 승인 2026-11-01): **R6 K0(정상 주행) 동안 '잃음'(LOST) 0 번**. 인지 조건 A1: 지도에 없는 물체(발제 배치 + 다리 기둥) 포함, 기둥은 문턱 정한 실행과 다른 자리.
- 구성: 설계 45 고정 + launch `loc_monitor:=true loc_event_log:=…/loc_events.jsonl`(문턱 기본값 res_thr 0.03 · suspect 0.5 s · lost 2.0 s · jump 0.10 m · 5°, 인지 동의 2026-11-02), 장면 `--objects …/seed_1/layout.json --legs 2.525 5.285 4.285 4.365 0.145 3.125`, 녹화에 /localization/status · /localization/lock.
| 항목 | 예상(실행 전) |
|---|---|
| R6 LOST 횟수 | 0(정상 잔차 최대 0.025 < 두 칸, 점프 최대 0.0094 m) |
| SUSPECT 횟수(참고) | 0~2(회전 중 스캔 시각 어긋남으로 잠깐 넘을 수 있음) |
| 주행(참고, G1~G5) | 7/7, loc_monitor 가 명령을 막지 않으면 설계 45 와 같음 |
- **K0 결과: R6 통과**(모의). LOST 0 · SUSPECT 0 · RECOVERING 0, /localization/status 는 처음부터 끝까지 OK, /localization/lock 은 늘 false(3,322 표본). loc_monitor 검사 351 번(건너뜀 1) · 잔차 최대 0.025(예상대로). 참고 G1~G5 통과: 7/7 · G3 0.0154 · 겹침 0 · G4 RMSE 0.0074 · 최대 0.018.
- 알게 된 것: loc_monitor 잔차 검사는 약 1.1 Hz(10 s 에 7~17 번)라 suspect_s 0.5 s 는 사실상 '연속 두 번 넘음' 쯤으로 동작함. 지속 시간을 넣은 실제 감지는 K1~K3(강제 이동)에서 봄.

### K1 판정(정지 중 0.20 m · 10° 강제 이동, 모의) — `results/K1_v45/`
- 기준 R1~R5(사용자 승인 2026-11-01, B1 포함). 채점 새 도구 `261031_1325_nav2/test/evaluate_kidnap.py <결과 폴더> [--k3]`(감지 전 위치 오차도 기록 — 인지 요청).
- 장면: K0 장면(물체 + 기둥 3 개) + `--kidnap_by 45 0.1875 -0.0696 0.1745 --kidnap_still_s 2.5`(sim 45 s 뒤 처음 2.5 s 멈춘 때 = start_box_1 도착 뒤 대기 예상, 상자 면 법선 방향으로 0.20 m 뒤로 · +10°, 그 자리 여유 0.41 m).
| 항목 | 예상(실행 전) |
|---|---|
| R1 | 통과. 3 s 안 SUSPECT(잔차 0.100 ≫ 두 칸, 검사 1.1 Hz 라 1~2 s), 또는 Cartographer 가 먼저 되돌림(근처 탐색 창 1.0 m 안이라 가능 — B1 표시) |
| R2 | LOST 까지 가면 통과(gate 0), 안 가면 해당 없음 |
| R3 · R4 | 다시 찾은 순간 ≤ 0.02 m · ≤ 1°(오프라인 근처 탐색 30/30 · ≤ 0.016 m), 10 s 안 > 0.10 m 0 번 |
| R5 | ≤ 10 s |
- **K1 결과: R1~R5 모두 통과**(모의). 강제 이동 sim 85.03 s(start_box_1 도착 뒤, 실제 이동 0.20 m · 10°) → 0.17 s 뒤 감지(사유 '점프' dxy 0.058 · dyaw 10.3°, 잔차 아님) → LOST → RECOVERING → 근처 탐색(0.22 s 계산, 점수 0.0074 · 2등 0.046) → 새 궤적 → CONFIRM → OK(감지 → 다시 찾기 4.63 s).
| R1 감지 지연 | R2 LOST 동안 명령 0 | R3 다시 찾은 순간 | R4 10 s 최대 오차 | R5 |
|---|---|---|---|---|
| 0.17 s | 159 표본 중 0 아닌 것 0 | 0.0139 m · 0.03° | 0.014 m | 4.63 s |
- 감지 전 위치 오차(인지 요청): 이동 순간부터 감지까지 0.17 s 동안 최대 0.196 m · 10.7°(추정이 옛 자리에 머문 시간). 참고 G1~G5: 7/7 · G3 0.0163 · 겹침 0.
- 채점 도구 결함 고침: 같은 /clock 에 'OK' → 'LOST' 가 함께 찍혀 첫 채점이 다시 찾은 시각을 옛 OK 로 잡음(R3 0.18 m 로 잘못 실패) → 메시지 순서로 찾게 고친 뒤 다시 채점.

### K2 판정(1.0 m · 90° 다른 통로 강제 이동, 모의) — `results/K2_v45/`
- 장면: K1 과 같고 `--kidnap_to 45 2.2261 1.5888 1.16 --kidnap_still_s 2.5`(start_box_1 도착 뒤, 북쪽 통로 자세 — K1 판 참값 궤적이 지난 자리, 차체 여유 0.19 m(geom), 이동 0.91 m · 90° — 승인 설명 1.0 m 보다 0.09 m 짧음).
| 항목 | 예상(실행 전) |
|---|---|
| R1 | 통과, 감지 ≤ 0.3 s(점프) |
| R2 | 통과(LOST 동안 명령 0) |
| R3 · R4 | 근처 탐색 창 ±0.5 m · ±30° 밖이라 전체 탐색으로 찾음, 다시 찾은 순간 ≤ 0.02 m · ≤ 1°(오프라인 전체 탐색 30/30), 위험: 비슷한 통로를 고름(2등−1등 점수 차 p50 0.02) |
| R5 | ≤ 20 s |
- **K2 결과: R1~R5 모두 통과**(모의). 강제 이동 sim 81.1 s(실제 0.91 m · 90°) → 점프는 안 걸림(Cartographer 추정이 옛 자리에 머묾) → 잔차 0.100 으로 SUSPECT 1.22 s · LOST 2.82 s → 근처 탐색 실패(점수 0.071) → 지도 전체 탐색(1.01 s 계산, 점수 0.0077 · 2등 0.034 = 다른 통로 (2.26, 4.84))로 찾음 → CONFIRM → OK.
| R1 감지 지연 | R2 LOST 동안 명령 0 | R3 다시 찾은 순간 | R4 10 s 최대 오차 | R5 |
|---|---|---|---|---|
| 1.22 s(SUSPECT) | 124 표본 중 0 아닌 것 0 | 0.0086 m · 1.05° | 0.0153 m | 4.92 s |
- 감지 전 위치 오차: 이동부터 SUSPECT 까지 1.22 s 동안 최대 0.909 m · 95.3°(그동안 /localization/status 는 OK).
- **판정 밖 발견(주행)**: G1 불통과 — 강제 이동 3 s 뒤 목표 실행기가 L1_via 를 보냈는데 그때 LOST(명령 차단) → OK 뒤에도 15.7 s 만에 'Failed to make progress', 이어 (2.21, 1.49) 방향 2.35 에서 'Start occupied' 로 나머지 5 개 실패. 위치를 다시 찾은 뒤 주행을 이어 가는 방법(목표 다시 보내기 · 진행 검사 시간 · 좁은 자리 여백)은 미정 — mission 쪽과 정할 일.

### K3 판정(비슷한 칸으로 강제 이동, 모의) — `results/K3_v45/`
- 자리: start_box_1 부근 스캔의 지도 전체 탐색 2등 자세(live_nav_v45_s1 reloc_offline 표본 t 56.7 · 64.6, 1등−2등 점수 차 0.018~0.019) = (2.223, 5.76, −0.652), 차체 여유 0.0215 m(geom). 장면 = K1 과 같고 `--kidnap_to 45 2.223 5.76 -0.652 --kidnap_still_s 2.5`(start_box_1 도착 뒤).
- 기준 R1~R5, K3 는 못 찾으면 안전 정지 + 기록도 통과(채점 `evaluate_kidnap.py <폴더> --k3`). '찾았는데 다른 자리'는 R3 · R4 실패(인지 A4).
| 항목 | 예상(실행 전) |
|---|---|
| R1 | 통과, 잔차 0.1 이상으로 1~3 s 안 SUSPECT(위치가 크게 바뀌어 점프 또는 잔차) |
| R3 · R4 | 반반: 전체 탐색이 맞는 자리(새 자리)를 찾거나, 모호해 ambiguity_margin 0.005 에 걸려 못 찾고 안전 정지 |
| R5 | 찾으면 ≤ 20 s, 못 찾으면 max_recover_s 60 s 뒤 안전 정지(통과) |
- **해석은 다음 세션에서**(사용량 한도). 할 일: `source /opt/ros/lyrical/setup.bash; /usr/bin/python3 261031_1325_nav2/test/evaluate_kidnap.py 261101_0016_isaac_live/results/K3_v45 --k3` → `loc_events.jsonl` 사건 · evaluate.json 확인 → 이 절에 결과 표 · 인지에 감지 시간 · 감지 전 오차 보내기.
- **K3 결과: R1~R5 모두 통과**(모의, `--k3` 채점). 강제 이동 sim 74.08 s(start_box_1 부근 (2.215, 0.646, 2.860) → (2.223, 5.760, −0.652), 5.1 m · 177°) → 0.12 s 뒤 감지(점프 dyaw −17.9°) → 근처 탐색 실패(점수 0.051) → 전체 탐색 1.03 s(1등 0.0117 = 맞는 자리 · 2등 0.0288 = (0.61, 3.92)) → CONFIRM → OK.
| R1 감지 지연 | R2 LOST 동안 명령 0 | R3 다시 찾은 순간 | R4 10 s 최대 오차 | R5 |
|---|---|---|---|---|
| 0.12 s | 47 표본 중 0 아닌 것 0 | 0.0031 m · 0.73° | 0.0041 m | 2.52 s |
- 해석: 고른 자리는 '출발 부근 스캔의 2등 후보'였지만, 옮긴 뒤의 스캔으로는 1등 · 2등 차가 0.017(모호 문턱 0.005 의 3 배 넘음)이라 헷갈리지 않음 → '못 찾음 + 안전 정지' 경로는 이번에 시험되지 않음. 감지 전 오차 최대 5.14 m · 177°(0.12 s). 판정 밖: K2 처럼 다시 찾은 뒤 주행 이어 가기 실패(G1 불통과, G3 0.0204 · 겹침 0).

### K2 · K3 자리 바꿔 1 회 더(승인 기준, 모의) — `results/K2b_v45/` · `K3b_v45/`(실행 `run_k_repeat.sh`, 끝나면 `results/K_repeat_done.txt`)
- K2b: `--kidnap_to 45 3.2478 0.4378 1.16`(start_box_1 도착 뒤 → 동쪽 통로, 약 1.05 m · 97°, K1 판 참값 궤적이 지난 자리, 차체 여유 0.202 m).
- K3b: `--kidnap_to 100 1.435 0.42 -1.1`(sim 100 s 뒤 처음 2.5 s 멈춘 때, L1 칸 쪽 예상 → L1 부근 스캔(v45_s1 t 129.7)의 전체 탐색 2등 자리, 차체 여유 0.084 m). 채점 `--k3`.
| 항목 | 예상(실행 전) |
|---|---|
| K2b R1~R5 | 통과(K2 와 비슷: 잔차 또는 점프로 1~3 s 안 감지, 전체 탐색) |
| K3b R1~R5 | 통과 — 찾거나(K3 처럼 옮긴 뒤 스캔으로는 덜 헷갈림), 모호하면 60 s 뒤 안전 정지 |
- **해석은 다음 세션**: 각 `evaluate_kidnap.log` · `loc_events.jsonl`(grep -v report) · evaluate.json, 실제 이동 자리는 `scene/kidnap.json`(K3b 는 언제 · 어디서 옮겼는지 먼저 확인).
- **K2b 결과: R1~R5 모두 통과**(모의). 강제 이동 sim 91.58 s(약 1.05 m · 97°) → 0.12 s 뒤 감지 → OK 3.73 s, LOST 동안 명령 0(120 표본), 다시 찾은 순간 0.0175 m · 0.02°, 10 s 최대 0.0198 m, 감지 전 오차 최대 1.046 m · 96.1°. 사건 순서 · G1 은 다음 세션에서 확인.

### B2 기록 실행(판정 아님, 사용자 승인 B2) — `results/B2a_v45/`(K3b 끝나면 `run_b2a.sh` 가 자동 시작, 끝나면 `results/B2a_done.txt`)
- 주행 중 강제 이동: `--kidnap_by 60 0 0 0.26 --kidnap_still_s 0`(sim 60 s 에 움직이는 채로 제자리 15° 회전만 — 벽에 박히지 않게 이동 없음). 나머지 K 판정과 같은 장면 · 구성. tracked_pose 끊김 · odom 만 틀림 기록은 도구가 없어 아직 안 함.
| 항목 | 예상(실행 전) |
|---|---|
| 감지 | 점프(dyaw 15° > 5°)로 0.2 s 안 LOST, 또는 Cartographer 가 먼저 되돌림 |
| 다시 찾기 | 근처 탐색으로 ≤ 0.02 m · ≤ 1°, ≤ 5 s |
- 해석은 다음 세션: `evaluate_kidnap.py results/B2a_v45`(기준 R1~R5 를 참고로 적용) · loc_events.jsonl.
- **K3b 결과: R1~R5 모두 통과**(모의). 강제 이동 sim 105.84 s, L1_via 도착 뒤 (4.061, 0.411, 1.552) → (1.435, 0.420, −1.100)(L1 부근 스캔의 전체 탐색 2등 자리, 2.6 m · 152°) → 0.16 s 뒤 감지(점프 dyaw −17.2°) → 근처 탐색 실패(0.021) → 전체 탐색 1.05 s(1등 0.0094 = 맞는 자리 · 2등 0.0216 = 옮기기 전 L1 칸 쪽 (4.11, 0.88)) → OK.
| R1 | R2 | R3 | R4 | R5 |
|---|---|---|---|---|
| 0.16 s | 100 표본 중 0 아닌 것 0 | 0.0076 m · 0.58° | 0.0101 m | 3.83 s |
- 감지 전 오차 최대 2.62 m · 152°(0.16 s). 판정 밖: G1 불통과(다시 찾은 뒤 주행 이어 가기, K2 · K3 와 같음), G3 0.0196 · 겹침 0.
- **강제 이동 판정 정리(모의): K0 · K1 · K2 · K3 · K2b · K3b 모두 통과(승인 기준 R1~R6, 각 1 회 + K2 · K3 자리 바꿔 1 회).** 한계: 모의, 감지 하한(이동 0.035~0.05 m · 회전 2~3° 는 못 잡음), '모호해서 못 찾음 → 안전 정지' 경로는 두 K3 판 모두 1등−2등 차가 커서 시험 안 됨, 다시 찾은 뒤 주행 이어 가기는 미해결.
- **B2a 결과(기록 실행, 모의)**: 주행 중 sim 60.0 s 제자리 15° 강제 회전 → 0.10 s 뒤 감지(점프 15.1°) → 근처 탐색(0.0035 · 2등 0.068)으로 찾음 → OK 3.43 s. 감지 전 오차 최대 0.0145 m · 15.3°. R1~R5 참고 적용 모두 통과(R3 0.0128 m · 1.06°, R4 0.0184 m, LOST 동안 명령 0 · 130 표본). **G1~G5 도 통과(7/7, G3 0.0218 · 겹침 0, G4 RMSE 0.0065)** — 근처 탐색으로 찾은 경우는 주행을 이어 감(K2 · K3 처럼 멀리 옮긴 경우만 이어 가기 실패).

### K2c(다음 실험, 모의) — 위치 다시 찾은 뒤 주행 이어 가기: `results/K2c_retry_v45/`(`run_k2_retry.sh`, 끝나면 `results/K2c_done.txt`)
- K2 와 같은 강제 이동 + 목표 실행기 `--loc_retry 2`(새 인자, 사본 `run_goals.py.before_loc_retry_261102`: 목표 실패 + 그 사이 /localization/status 가 OK 아니었으면 OK 로 돌아올 때까지(≤ 60 s) 기다려 같은 목표를 2 번까지 다시 보냄).
| 항목 | 예상(실행 전) |
|---|---|
| R1~R5 | K2 와 같이 통과 |
| G1 | 7/7(L1_via 를 OK 뒤 다시 보내 이어 감). 위험: 다시 찾은 자리(북쪽 통로 여유 0.19)에서 'Start occupied' 로 다시 보내도 실패 |
- 해석은 다음 세션: evaluate_kidnap.log · evaluate.json(G1) · run_goals.json 의 loc_retries.
- K2c 첫 시도 **도구 실패**(`results/K2c_retry_v45_TOOLFAIL_isaac_segv_try1/`): 강제 이동 전 Isaac exit 139(목표 실행기 시간 초과, kidnap.json 없음) → 재주행 시험 안 됨. K3c(`run_k3_retry.sh`) 는 예약대로 실행, K2c 는 그 뒤 `run_k2_retry_again.sh` 로 다시(같은 인자, 결과 `results/K2c_retry_v45/`, 끝나면 `results/K2c_done.txt`).
- **K2c · K3c 결과(모의, 재주행 `--loc_retry 2`)**: 위치 잃음 판정은 둘 다 R1~R5 통과(K2c 감지 0.13 s · OK 5.02 s · R3 0.0113 m, K3c 감지 0.10 s · OK 3.10 s · R3 0.0037 m). **재주행은 실패(G1 불통과)**: OK 뒤 목표를 다시 보냈으나(K2c 첫 시도 'Failed to make progress'(LOST 동안 명령 차단), K3c 첫 시도 'Resulting plan has 0 poses') 두 번째 시도에서 전역 계획기가 **'Start occupied'**. 그때 참값 차체 여유 K3c 0.020 · K2c 0.037 m < 전역 costmap footprint 여백 0.04(V27) → 계획기가 출발 자세를 막힌 칸으로 봄. 다음 후보(다음 세션): 다시 보내기 전에 빠져나오기(BackUp · 여유 큰 쪽으로), 또는 출발 칸만 여백 없이 검사.

### K2d · K3d(모의) — 좁은 자리에서 다시 찾은 뒤 빠져나와 재주행: `results/K2d_escape_v45/` · `K3d_escape_v45/`(`run_k_escape.sh`, 끝나면 `results/K_escape_done.txt`)
- 바꾼 것: 목표 실행기 `--escape 2`(사본 `261031_1325_nav2/test/run_goals.py.before_escape_261102`). 목표가 'Start occupied'(205)로 실패하면 바로 다시 계획하지 않고 ① 추정 자세 · 전역 costmap 치명 칸으로 앞 · 뒤 직선 후보(0.025 m 간격, 최대 0.30 m)를 보고, 가는 길이 막히지 않으면서 차체 여유 ≥ 0.08 m(여백 0.04 + 칸 0.025 + 0.015)가 되는 가장 짧은 쪽으로 움직임(같으면 뒤, Nav2 BackUp / DriveOnHeading 0.1 m/s) ② 그 자리에서 회전 중심 → 치명 칸 거리 ≥ footprint 외접 반지름 0.382 + 0.03 이면 한 바퀴(Spin 2π) 돌고 위치 상태 OK 확인, 아니면 회전 생략 ③ 같은 목표 다시. 참값은 쓰지 않음. 나머지(장면 · 강제 이동 · launch 인자 · `--loc_retry 2`)는 K2c · K3c 와 같음.
- 실행 전 참값 형상 래스터로 고르기 확인(scratchpad): K2c 멈춘 자리 → 뒤 0.10 m(여유 0.096), 회전 불가(0.317 < 0.412) · K3c 자리 → 뒤 0.175 m(여유 0.083), 회전 가능(0.436 ≥ 0.412).
- 판정 기준: R1~R5(K 판정과 같음, K3d 는 `--k3`) + G1 7/7. G2~G5 는 참고.
| 항목 | 예상(실행 전) |
|---|---|
| R1~R5 | K2c · K3c 와 같이 통과(빠져나오기는 다시 찾은 뒤에만 일어남) |
| 빠져나오기(escape 기록) | K2d: 뒤 약 0.10 m · 회전 생략, K3d: 뒤 약 0.15~0.20 m · 한 바퀴 회전 뒤 OK — 실제 자세는 추정이라 ±0.025 m 다를 수 있음 |
| G1 | 7/7 — 빠져나온 뒤 다시 보낸 L1_via 부터 이어 감. 위험: BackUp 0.1 m/s 가 바닥 마찰로 안 움직임(move_ff 곡선은 회전만), 충돌 감시(approach)가 멈춤, 한 번 빠져나와도 Lattice 첫 동작이 다시 좁은 쪽 |
- **K3d 결과(모의): R1~R5 통과 · G1 7/7 — 빠져나오기 동작 확인.** 강제 이동 sim 98.24 s → 0.16 s 뒤 감지 → OK 5.03 s(R3 0.0027 m · 0.93°). L1_via 첫 시도 'Resulting plan has 0 poses'(LOST 중) → OK 기다려 다시 → 'Start occupied' → 빠져나오기: 추정 여유 0.021, 후보 뒤 −0.15 m(0.088) · 앞 0.075 m(0.021) → BackUp 0.15 m 성공(여유 0.106) → 회전 중심 여유 0.452 ≥ 0.412 라 Spin 2π 성공 · OK → 다시 보낸 L1_via 부터 7 개 모두 성공(위치 ≤ 0.041 m). 빠져나오기 전체 16.6 s. 참고: G3 0.0073 · 겹침 0, G4 는 강제 이동 오차(최대 5.1 m)로 불통과(K 판정에선 참고).
- **K2d 결과(모의): R1~R5 통과 · G1 7/7 — 그러나 빠져나오기는 일어나지 않음**(L1_via 첫 시도에 바로 성공, K2c 처럼 좁은 쪽으로 돌지 않음 → K2 조건의 빠져나오기는 미시험). 감지 0.17 s · OK 2.92 s · R3 0.0095 m. **G3 불통과: 다시 찾은 뒤 L1_via 로 가며 후진 + 회전(cmd −0.1 m/s · −0.37 rad/s, sim 84.2~88.8 s) 중 오른쪽 모서리가 미로 기둥(약 (2.61, 1.28), 반지름 0.03)에 겹침 229 표본(참값 여유 ≤ 0)**. 그동안 위치 추정 오차 ≤ 0.008 m 라 위치 잃음 탓은 아님 — 후진 회전 경로 추종이 기둥 모서리를 깎은 것으로 보임(원인 미확인). 이전 K2 · K2c 는 이 후진이 없었음(여유 0.030 · 0.029).
- 정리: 빠져나오기 자체는 K3d 에서 설계대로 동작. 남은 것 ① K2 조건에서 빠져나오기가 실제로 일어나는 판(K2c 처럼 좁은 쪽 회전)은 아직 못 봄 ② 다시 찾은 뒤 후진 회전이 기둥에 닿는 문제(Nav2 후진 경로 · 추종, 정상 주행 설계 45 에서는 안 나왔던 동작).

### C1 ⑤ 감지 개선 오프라인 비교(도구, 판정 아님, 11-02 사용자 확정 '5 cm 어긋남 감지 개선' ①)
- 도구 `261031_1325_nav2/test/min_offset_variants.py`, 입력 설계 45 세 판 + 기둥 장면(스캔 쌍 12,640, 그중 1/5 사용), 결과 `results/live_nav_v45_s1/min_offset_variants.json` · `.log`. 문턱 = 방식마다 '추정 자세 정상 최대 + 0.005'(같은 자료에서 정함 — 표본 안 값이라 실제 여유는 더 작을 수 있음).
| 방식 | 정상 최대(추정) | 문턱 | 이동 0.035 · 0.05 · 0.075 m 넘는 비율 | 회전 2 · 3 · 5° 넘는 비율 |
|---|---|---|---|---|
| A 지금(칸 값 · 중앙값) | 0.025 | 0.030 | 0 · 0.125 · 0.713 | 0.009 · 0.129 · 0.544 |
| B 보간 · 중앙값 | 0.0195 | 0.0245 | 0.175 · 0.630 · 0.901 | 0.149 · 0.395 · 0.825 |
| **C 보간 · p75** | 0.0292 | 0.0342 | **0.301 · 0.798 · 0.994** | **0.342 · 0.736 · 0.986** |
| D 보간 · 0.05 자른 평균 | 0.0198 | 0.0248 | 0.138 · 0.645 · 0.943 | 0.175 · 0.455 · 0.869 |
| E 양쪽 거리장 · 중앙값 | 0.0125 | 0.0175 | 0.020 · 0.385 · 0.832 | 0.065 · 0.240 · 0.715 |
| F 양쪽 거리장 · p75 | 0.0175 | 0.0225 | 0.268 · 0.772 · 0.994 | 0.324 · 0.709 · 0.983 |
- 해석: 보간 + p75(C)가 가장 좋음 — 0.05 m 를 스캔당 12.5 % → 79.8 %, 3° 를 12.9 % → 73.6 %. 목표(정상은 문턱 밑 · 0.035~0.05 m 를 스캔당 ≥ 80 %)는 **0.05 m 는 경계(0.798), 0.035 m 는 어느 방식도 못 함(최대 0.30)**. 통로 방향 이동은 벽과 나란해 스캔으로 원래 안 보이는 성분이 있음.
- 다음(아직 안 함): ② C 를 loc_core 사본(`residual` 보간 · p75)으로 넣고 문턱 0.034 · 검사 주기 1.1 Hz → 더 빠르게 → ③ K0 · K1 다시(모의) → 인지에 C1 다시 보냄(계산 · 문턱 바뀜).
