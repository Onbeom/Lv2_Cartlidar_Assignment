# 설계 18 = 설계 12 설정에서 바꾼 값(Isaac 실시간 설계 17 불통과 뒤, 기준 그대로)

위치 추정을 AMCL 에서 Cartographer 순수 위치 추정으로 바꿈(사용자 결정 2026-11-01). launch `localization:=carto` 로 AMCL · scan_merge 를 띄우지 않으므로 `amcl` 절은 남아 있어도 쓰이지 않음(v17 의 AMCL 값은 가져오지 않음).

| 항목 | v12 | v18 | 이유 |
|---|---|---|---|
| `bt_navigator.global_frame` | `map` | `mission_map` | TF 를 1단계 SLAM 과 같은 꼴로: mission_map(발제 좌표) →(정적, align.json) map(Cartographer) →(map_odom_relay) odom →(EKF) base_link. Cartographer map 은 출발 자세가 원점이라 발제 좌표가 아님 |
| `global_costmap.global_frame` | `map` | `mission_map` | 같은 이유 |
| `behavior_server.global_frame` | `map` | `mission_map` | 같은 이유 |
| `map_server.frame_id` | (기본 `map`) | `mission_map` | 벽 지도 map_mission_nav.yaml 은 발제 좌표로 그린 것 |
| `map_objects_server.frame_id` | `map` | `mission_map` | 고정 물체 지도도 발제 좌표 |

- 지도를 Cartographer map 좌표로 다시 그리는 방법 대신 frame 이름을 바꾼 이유: 다시 그리면 0.025 · 0.01 m 칸을 회전 재표본해 지도 오차가 더해지고, 1단계와 2단계의 TF 모양이 달라짐(인지 · 미션 쪽이 frame 하나만 알면 되게).
- 목표(goals json) 좌표는 그대로(발제 좌표), 보내는 frame 만 mission_map(run_goals.py `--frame`).
