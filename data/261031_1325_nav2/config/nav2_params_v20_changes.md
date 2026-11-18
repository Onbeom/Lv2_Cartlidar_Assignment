# 설계 20 = 설계 18 설정(v18)에서 바꾼 값(Isaac 실시간 설계 19 불통과 뒤, 기준 그대로)

설계 19: 위치 추정은 맞는데(G4 RMSE 0.005 m) 목표 끝이 0.17~0.19 m 어긋남. 원인 = 제자리 회전 중심이 base_link 가 아니라 x −0.0786 m(뒤 바퀴 축 근처, 설계 14 에서 잼) →
도착 판정 stateful 이 '위치 0.06 m 안'을 잡아 둔 뒤 방향 맞추는 제자리 회전 동안 base_link 가 반지름 0.079 m 원을 따라 최대 0.157 m 밀림. 그래서 Nav2 가 제어하는 점을 실제로 안 움직이는 회전 중심으로 둠.

| 항목 | v18 | v20 | 이유 |
|---|---|---|---|
| `bt_navigator` · `local_costmap` · `global_costmap` · `behavior_server` 의 `robot_base_frame` | `base_link` | `rotation_center` | 제자리 회전 때 이 점은 그대로라 도착 판정 · 경로 추종이 회전으로 깨지지 않음. frame = 정적 TF base_link → rotation_center(x_icr −0.0786, 0), launch `rotation_center_tf:=true` |
| local · global costmap `footprint` | x −0.155 ~ 0.263 | x −0.0764 ~ 0.3416 | 같은 차체를 rotation_center 기준으로(+0.0786) |
| `collision_monitor.base_frame_id` | `base_link` | 그대로 | footprint 는 토픽(published_footprint)으로 받아 바꿀 것 없음 |

- 함께 바꾸는 것: 목표를 base_link 자세 ∘ (−0.0786, 0) 점으로 보냄(`run_goals.py --base_offset_x -0.0786`, 채점 G2 는 base_link 참값 그대로). 명령 header 는 rotation_center 로 나오므로 turn_floor 가 base_link 로 이름만 바꿈(`turn_floor_out_frame:=base_link`, 스키드는 옆 속도를 못 내서 (vx, wz) 바퀴 명령은 같음).
- 한계: x_icr 은 Isaac 모의에서 잰 값. 실물 회전 중심은 실측해야 함(요청 목록). 목표의 회전 구간 규칙(goals_seed1_v9)은 base_link 중심 회전으로 계산한 것이라 좁은 칸(L1 · L3) 여유가 달라질 수 있음.
