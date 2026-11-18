# 01. 경계: localization 패키지(인지 파트) ↔ Nav2 · AMCL(받는 쪽)

## 누가 무엇을 내나
| 토픽 · TF | 내는 쪽 | 받는 쪽 | 비고 |
|---|---|---|---|
| `/scan_front_filtered` · `/scan_rear_filtered` | localization `scan_body_mask` | Nav2 costmap(원하면), loc_monitor | 차체 · 팔이 가린 라이다 칸을 지운 스캔 |
| `/scan_merged`(LaserScan, frame base_link) | localization `scan_merge` | **AMCL** | 앞 · 뒤를 한 스캔으로, 스캔 사이 움직임 보정. AMCL 최적화는 이 입력으로 함 |
| `/map`(SLAM 벽, 0.01 m) · `/map_objects`(라이다에 안 보이는 고정 물체, 0.01 m) | localization `map_server` 2 개 | AMCL(`/map`), Nav2 전역 costmap 두 정적 층 | 둘 다 frame `map`, 같은 0.01 m 격자(다르면 costmap 이 늦게 온 쪽 해상도로 다시 만들어 앞 층을 지움 — 실제로 겪음) |
| `map → odom` · `/amcl_pose` | **AMCL**(받는 쪽 launch) | Nav2, loc_monitor | |
| `odom → base_link` · `/odometry/filtered` | bringup EKF(kanichong) | 전부 | 04 의 ICR 문제 참고 |
| `base_link → rotation_center`(정적, x = x_icr) | **Nav2 쪽 launch** | Nav2(`robot_base_frame: rotation_center`) | 스키드 회전 중심. B안 모의 x_icr −0.0658 m |
| `/localization/status`(std_msgs/String, Reliable · TransientLocal, 10 Hz) | localization `loc_monitor` | drive_node(loc_gate), perception_survey | `OK · SUSPECT · LOST · RECOVERING · CONFIRM · FAILED` |
| `/localization/lock`(std_msgs/Bool, 10 Hz) | localization `loc_monitor` | twist_mux lock(아직 연결 안 됨) | OK · SUSPECT 가 아니면 true |
| `/initialpose`(frame map) | loc_monitor(위치 다시 찾았을 때) | **AMCL** | AMCL 기본 구독 그대로 두면 됨 |

- frame `map` = 발제 좌표(팀 `mission_planner/maps` 문서 좌표 지도와 같은 원점 · 방향). 모의 문서의 `mission_map` 은 같은 것의 옛 이름.
- localization 패키지 실행: `ros2 launch localization localization.launch.py`(기본 `localization:=amcl`, AMCL 노드는 띄우지 않음). `use_amcl_node:=true` 면 패키지가 `config/amcl.yaml` 로 AMCL 도 띄움 — 단독 시험용. **실제 운용에서는 둘 중 하나만** AMCL 을 띄운다.
- `mission_planner/launch/localization.launch.py`(kanichong, AMCL + `/map`)와 localization 패키지를 함께 띄우지 않는다(`/map` · `map → odom` 이 겹침).

## 받는 쪽이 지켜 줄 규칙
1. **loc_gate(drive_node 가 Nav2 목표를 보낼 때)** — 모의 근거 있음
   - 목표를 보내기 전 `/localization/status == OK` 를 기다린다(모의 도구는 최대 60 s).
   - 주행 중 OK 가 아니었다가 OK 로 돌아오면 지금 목표를 취소하고 같은 목표를 다시 보낸다(위치를 다시 찾으면 추정 자세가 뛰므로 그 전 자세로 만든 경로를 버림).
   - 근거: 이 규칙 없이 강제 이동 시험(K2d)에서 낡은 경로를 쫓아 실패, 규칙을 넣은 K2e 는 통과(Cartographer 구성, 감지 0.17 s, 다시 찾은 오차 0.012 m, 기다림 2.98 s 한 번). 구현 예: `reference/sim_tools/run_goals.py` 의 `--loc_gate`.
2. **속도 명령 잠금** — 모의는 loc_monitor 게이트(명령을 0 으로)로 확인함. 팀 구조에서는 twist_mux lock 이 자연스러움:
   `opencr_hardware/config/twist_mux.yaml` locks 에 `localization: {topic: /localization/lock, timeout: 0.5, priority: 20}` 같은 줄(nav 10 만 막고 drive 50 · teleop 100 은 통과).
   lock 토픽이 끊겨도 잠기므로 loc_monitor 를 안 띄우는 실행은 lock 줄이 없는 설정을 써야 함. twist_mux.yaml 은 kanichong · LCY 파일 → 그쪽에 요청.
   대안: localization 패키지 `gate_in:=<Nav2 마지막 출력> gate_out:=/cmd_vel_nav` (모의에서 쓴 방식).
3. Nav2 체인의 마지막 출력은 `/cmd_vel_nav`(TwistStamped, frame base_link) — 팀 twist_mux nav 입력.
