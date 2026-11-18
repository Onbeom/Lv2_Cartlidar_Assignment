제목: Jetson 정지 상태 CPU 85~95 % — 담당마다 줄일 곳 확인 부탁 (mission_planner 시작 계산 약 10 점, 인지 결과 300 ms 초과 13~75 %)

## 무엇을 봤나 (실물, Jetson Orin Nano Super, 정지 상태)
`bringup.launch.py use_teleop:=false use_localization:=true use_nav2:=true` + 인지 모드 7, 바퀴 막고 움직임 명령 없이 5 분. 설치본 14087c4, CPU 1728 MHz (25W).

| 측정 | 조건 | CPU 평균 | 인지 결과 | > 300 ms |
|---|---|---|---|---|
| 11-13 17:26 | 그대로 | 94.8 % | 3.1 Hz | 74.9 % |
| 11-13 18:00 | `use_planner:=false` | **84.9 %** | 9.0 Hz | **12.8 %** |
| 11-13 18:07 | `use_planner:=false` + `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` | 91.9 % | 4.5 Hz | 58.1 % |

- 세 번 다 로봇이 **미로 밖**이라 위치 추정은 실패함(약 85 s 에 FAILED). 시험 조건 탓이라 이 이슈에서 다루지 않음.
- LOCALHOST 는 CPU 를 줄이지 못함(한 번씩 잰 값).

## mission_planner 시작 계산
- `World(sdf)`(지도 · 접근 자세 · 경로 비용, 시작할 때 한 번): PC 24.6 s, **Jetson 145.1 s**(bringup 없이 따로 잼).
- bringup 안에서는 더 걸림. 10:09 · 17:26 둘 다 5 분 동안 `ready` 줄이 **없음**. 17:26 은 시작 뒤 215 s 에 누적 CPU 1:59 로 145 s 를 다 쓰지 못함 → 측정 내내 계산 중이었다고 봄. 그 순간 21 % 인 것은 다른 노드와 CPU 를 나눠 써서로 봄.
- 계획기를 끄면 CPU −10 점, 인지 결과 3.1 → 9.0 Hz.
- 코드상 `/mission_planner/plan` 서비스는 World 다음에 생김 → plan 모드 mission_node 는 `allServersReady()` 가 그동안 false(코드를 읽고 판단, 실물로는 확인 안 함).

## 계획기를 끈 뒤 남은 85 % (18:00, 측정 구간 평균, 코어 하나 = 100, 6 코어 = 600)
| 묶음 | CPU (코어 하나 = 100) | 6 코어 대비 | 내용 |
|---|---|---|---|
| perception_container | 104 | 17.3 % | perception_node · scan_merger · rear_obstacle · tfmini_node |
| 카메라 드라이버 3 | 101 | 16.8 % | 팔 Arducam 41 · 앞 D435 31 · 뒤 D435 29 |
| control_container | 48 | 8.0 % | mission_node · drive_node · manipulator_node · tfmini_node · obstacle_stop |
| ros2_control_node | 28 | 4.7 % | OpenCR 2 대 |
| Python 보조 노드 | 120 | 20.0 % | puck_survey 28 · odom_icr 28 · loc_monitor 24 · scan_merge 22 · scan_body_mask 18 |
| 위치 추정 · Nav2 | 약 63 | 10.5 % | EKF 13 · AMCL 10 · Nav2 서버들 약 40 |
| 그 밖 | 약 45 | 7.5 % | 작은 노드들(robot_state_publisher · lifecycle_manager · map_server 등) 약 18 · 프로세스 밖(커널 · 시스템, tegrastats 509 − 프로세스 합 482) 약 27 |
| 합 | 약 509 | 84.8 % | tegrastats 84.9 % × 600 |

## 요청 — 담당마다 자기 줄을 확인해 줄일 수 있는 것을 댓글로
정지 상태에서 이미 85 % 라 주행 · 미션 때 여유가 없습니다(controller_manager 100 Hz 루프 overrun 경고 5 분에 104 줄 — EKF 'Failed to meet update rate' 는 따로 7 번, 17:26 의 335 번은 EKF 쪽. 인지 결과의 12.8 % 가 300 ms 를 넘어 manipulator `stale_timeout: 0.3` 에서 버려짐 — 8 번에 1 번). 줄일 수 있는 것(주기 낮추기 · 필요할 때만 켜기 · Python → C++ 등)과 대략의 효과를 댓글로 남겨 주세요. 숫자는 18:00 측정(코어 하나 = 100)입니다.

| 담당 | 프로세스 (CPU) | 눈에 띄는 점 |
|---|---|---|
| Onbeom (인지 · 위치 추정) | perception_container 104 · 팔 카메라 41 · D435 31 · 29 · puck_survey 28 · loc_monitor 24 · scan_merge 22 · scan_body_mask 18 | 팔 카메라는 30 → 15 Hz 로 줄이는 중(따로 PR). puck_survey 는 조사 전(enabled false)에도 28 — Python TF 리스너로 봄(추정) |
| kanichong (계획기 · bringup) | mission_planner 시작 계산(Jetson 145 s, 측정 중 약 60) · odom_icr 28 | odom_icr 는 vy 한 칸을 채우는데 EKF(13)의 두 배. Python 이 `/odom` · `/imu/data`(100 Hz) 메시지마다 받고 다시 냄(코드를 읽고 추정) |
| WindForce08 (바퀴 · Nav2 · AMCL) | ros2_control_node 28(OpenCR 2 대, 팔과 같이) · Nav2 서버들 약 40 · EKF 13 · AMCL 10 | 정지 상태에서도 controller_server · planner · behavior · collision_monitor 등이 돎 |
| developlcy-oss (팔) | control_container 48(manipulator · mission · drive · tfmini · obstacle_stop 같이) · ros2_control_node(위와 같음) | 컨테이너 안 노드별 몫은 아직 못 가름 |

- mission_planner 는 World 결과를 빌드 · 첫 실행 때 파일로 저장해 두고 다음부터 읽는 방법을 검토해 주세요(위 '시작 계산').

## 한계
- 미로 밖이라 위치 추정이 실패한 상태의 부하. 미로 안 출발 자리에서 다시 재야 미션 때 부하가 됨.
- 조건마다 한 번씩 잼. 같은 조건(계획기 끔)에서도 측정 사이 차이가 있음(17:44 는 컨트롤러가 안 떠 비교 제외).
- 결과: Jetson `~/real_drive/p7a_out/1113_1726`, `1113_1800`, `1113_1807`.
