## 요청 (navigation `gazebo_sim.launch.py`)
`use_loc_monitor` 기본값 false 의 이유였던 거짓 LOST(gap)를 localization 쪽에서 고쳤습니다(#129). 필요하면 Gazebo 시험에서 loc_monitor 를 켜도 됩니다. 켜고 끄는 것은 주행 쪽에서 정해 주세요.

## 켤 때 해야 할 것
`gazebo_sim.launch.py` 는 `map` · `objects_map` 만 넘깁니다. loc_monitor 를 켜면 `loc_map` 도 같은 SDF 지도로 줘야 합니다.
```bash
ros2 launch navigation gazebo_sim.launch.py use_loc_monitor:=true loc_map:=<mission_planner share>/maps/map_amcl.yaml
```
`loc_map` 을 안 주면 loc_monitor 가 Isaac `maze.usda` 지도(`slam_walls`)를 봅니다. Gazebo 모의에서 잔차 중앙값이 0.021 → 0.077 m 로 커지고 165 s 에 FAILED 로 끝났습니다.

## 원인 (Gazebo 모의, 실물 아님)
- 움직인 뒤 멈추면 Gazebo `/odom` 변화가 정확히 0 이 되고, 그동안 AMCL 이 `/amcl_pose` 를 내지 않습니다. loc_monitor 는 이것을 0.5 s 끊김으로 보고 LOST 로 판단했습니다.
- 고친 뒤 12 곳 주행 2 판에서 LOST:gap 40 → 0 · 0, `/initialpose` 47 → 6 · 6.
- README 의 '시작하자마자 LOST' 자체는 Gazebo 를 60 s 켜 둔 조건에서 재현되지 않았습니다. 더 오래 켠 조건은 안 봤습니다.

## 켜도 남는 것
- 12 곳 주행에서 LOST:residual 이 6 번 남습니다. 문턱 0.034 m 는 Isaac 기준입니다. 참값이 없어 거짓인지 모릅니다. 이 구성은 EKF · IMU 없이 스키드 odom 을 그대로 씁니다.

## 참고: 12/12 재현 안 됨
같은 dev(30c2d1f) · 같은 SDF 지도 · `approach_poses.yaml` 12 곳을 `gui:=false rviz:=false` 로 돌렸을 때 도착 수가 README 의 12/12 와 달랐습니다. 목표 전송은 README 와 다른 도구(`run_goals.py`, frame map)를 썼고, 어떤 차이 때문인지는 가리지 않았습니다.

| 조건 | 도착 | 중단 |
|---|---|---|
| loc_monitor 끔 | 8/12 | B3:S · B3:N · B4:S · B4:N, error 104 |
| 잔차 문턱 0.10 (LOST 0) | 9/12 | B3:S · B3:N · B4:S, error 104 |
| 고친 판 2 번 | 9/12 · 9/12 | L2 · L3 · HOME, error 104 |

error 104 는 `Controller patience exceeded` 입니다. loc_monitor 를 꺼도 나므로 원인은 위치 잃음 판단이 아닌 것으로 봅니다. 그 앞 원인은 보지 않았습니다.

관련: #114 #116 #121
