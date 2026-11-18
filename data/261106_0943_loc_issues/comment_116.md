인지 몫 답입니다. 표의 '#30 정정(2026-11-02)' 열은 그 뒤 바뀌었고, 지금 코드 쪽이 맞습니다.

## 확정
| 항목 | 값 | 비고 |
|---|---|---|
| 위치 추정 | **AMCL** | `localization.launch.py` 기본값이 이미 `amcl` 이라 코드는 고칠 것이 없습니다. Cartographer 는 `localization:=carto` 로 남겨 둡니다. |
| 목표 프레임 | **`map`** (= 발제 좌표) | `mission_map` 은 같은 좌표의 옛 이름입니다. 정적 변환을 두지 않습니다(#90 병합). |

## 근거 (모두 Isaac 모의, 실물 값 아님)
- 설계 45 Nav2 로 AMCL 3 판 위치 오차 RMSE 0.0154~0.0162 m · 최대 0.030 m 이하, Cartographer 3 판 RMSE 0.0072~0.0081 m. Cartographer 가 더 정확하지만 AMCL 도 통과했고, 팀이 무게를 이유로 AMCL 을 원해 AMCL 로 갑니다.
- 정지 중 0.20 m · 10° 강제 이동(K1, AMCL): 감지 0.52 s, 다시 찾은 오차 0.0156 m.
- 실물 값은 실물 SLAM 뒤에 다시 잽니다.

## 이 이슈에서 인지 몫이 아닌 것
- Nav2 `robot_base_frame` 과 회전 중심 TF 는 Nav2 를 인계받은 주행 담당 몫입니다. 지금 dev 는 `base_link` 입니다.

## 통합 launch 주의 (코드를 읽고 본 것, 실행 확인 안 함)
`bringup use_localization:=true` 와 `navigation.launch.py` 를 기본값으로 함께 띄우면 localization 이 두 번 뜹니다(`/map` · loc_monitor 중복). 하나만 띄우는 조합은 다음 둘 중 하나입니다.
- `bringup use_localization:=false use_loc_lock:=true` + `navigation.launch.py` (AMCL 은 navigation 쪽)
- `bringup use_localization:=true use_amcl_node:=true` + `navigation.launch.py use_localization:=false`

`use_loc_lock` 기본값은 `use_localization` 을 따르므로, 첫 조합은 직접 켜야 잠금이 붙습니다.

## 지도 주의 (자료 비교로 확인, 실행 확인 안 함)
- dev 기본 `/map`(`slam_walls_001`)과 loc_monitor 지도(`slam_walls`)는 인지 Isaac 미로(`maze.usda`)에서 만든 것입니다. 이 미로와는 맞지만(중앙 0.010 m, p95 0.031 m), 발제 도면 SDF 와는 동쪽 · 북쪽 벽이 7~10 cm 다릅니다(최대 0.14 m). SLAM 오차가 아니라 세계 모델 차이입니다.
- 팀 Isaac 장면(`scenes/dice_mission.usda`)은 SDF 에서 옮긴 것이라 SDF 쪽입니다. 그래서 Isaac 브리지 미션 전체 1 회는 기본 지도로 돌리면 안 되고, `gazebo_sim.launch.py` 처럼 `mission_planner/maps` 를 줘야 합니다.
- 이때 loc_monitor 를 켜면 `loc_map:=` 도 같은 지도로 줘야 합니다. `map:=` 만 바꾸면 AMCL 과 loc_monitor 가 서로 다른 지도를 봐서 거짓 LOST 가 날 수 있습니다.
- 실물 지도는 실물 SLAM 뒤 인지가 패키지에서 바꿉니다.
