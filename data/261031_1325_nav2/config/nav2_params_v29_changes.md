# 설계 29 = v21 에서 바꾼 값(설계 27 = 설계 26 불통과 뒤, 설계 28 = 설계 27 + 전역 라이다 층 뺌, 설계 29 = 설계 28 + RPP, 설계 30 = 설계 29 + Lattice 직선 · 제자리 회전, 기준 그대로)

설계 26: 물체 지도에 기둥을 넣어도 출발 통로 ㄱ자 모퉁이에서 같은 기둥(wall_d0 끝 (1.0705, 0.5305))에 닿음. 원인 = 전역 계획기 NavFn 이 점 로봇 경로를 내고, 이 모퉁이는 차체로 지나려면 전진 · 후진을 섞은 오른쪽 약 270° 동작이 필요(기하 계산).

| 항목 | v21 | v27 | 이유 |
|---|---|---|---|
| `planner_server.ros__parameters.GridBased` | nav2_navfn_planner::NavfnPlanner | nav2_smac_planner::SmacPlannerHybrid | NavFn 은 점 로봇이라 차체 모양 · 방향을 모르고 안쪽 모서리를 깎음(설계 21~26 같은 기둥에 막힘). Hybrid-A* 는 자세마다 footprint 를 검사. 출발 모퉁이는 차체로 지날 수 있지만 오른쪽 약 270° + 전진 · 후진이 필요(tools/corner_feasibility.json, 병목 0.052 m) |
| `planner_server.ros__parameters.GridBased.motion_model_for_search` | (없음) | REEDS_SHEPP | 전진 · 후진 둘 다 허용(모퉁이에서 후진 필요) |
| `planner_server.ros__parameters.GridBased.minimum_turning_radius` | (없음) | 0.1 | 스키드라 실제로는 제자리 회전도 되지만 Hybrid 는 반경 0 이 없음 → 작게. 시작 0.10, 모퉁이 못 찾으면 줄임 |
| `planner_server.ros__parameters.GridBased.angle_quantization_bins` | (없음) | 72 | 5° 칸(corner_feasibility 의 회전 칸과 같게) |
| `planner_server.ros__parameters.GridBased.reverse_penalty` | (없음) | 2.0 | 필요할 때만 후진(라이다 뒤 시야는 있지만 앞 카메라 쪽 주행을 우선) |
| `planner_server.ros__parameters.GridBased.change_penalty` | (없음) | 0.0 | 기본값(방향 바꿈 벌점 없음) |
| `planner_server.ros__parameters.GridBased.non_straight_penalty` | (없음) | 1.2 | 기본값 |
| `planner_server.ros__parameters.GridBased.cost_penalty` | (없음) | 2.0 | 기본값. costmap 비용을 피함 = 벽에서 떨어지게 |
| `planner_server.ros__parameters.GridBased.retrospective_penalty` | (없음) | 0.015 | 기본값 |
| `planner_server.ros__parameters.GridBased.analytic_expansion_ratio` | (없음) | 3.5 | 기본값 |
| `planner_server.ros__parameters.GridBased.analytic_expansion_max_length` | (없음) | 1.0 | 좁은 미로에서 긴 해석 경로(Reeds-Shepp 한 번에 잇기)가 벽 옆을 지나지 않게 기본 3.0 보다 짧게 |
| `planner_server.ros__parameters.GridBased.analytic_expansion_max_cost` | (없음) | 200.0 | 기본값 |
| `planner_server.ros__parameters.GridBased.analytic_expansion_max_cost_override` | (없음) | False | 기본값 |
| `planner_server.ros__parameters.GridBased.tolerance` | 0.1 | 0.05 | 목표가 막혔을 때 가장 가까운 자세 허용 거리. 도착 허용 0.06 m 안 |
| `planner_server.ros__parameters.GridBased.downsample_costmap` | (없음) | False | costmap 0.025 m 그대로(통로 여유가 수 cm) |
| `planner_server.ros__parameters.GridBased.allow_unknown` | False | False | 모르는 칸으로 계획하지 않음(NavFn 설정과 같음) |
| `planner_server.ros__parameters.GridBased.max_iterations` | (없음) | 1000000 | 기본값 |
| `planner_server.ros__parameters.GridBased.max_on_approach_iterations` | (없음) | 1000 | 기본값 |
| `planner_server.ros__parameters.GridBased.max_planning_time` | (없음) | 5.0 | 좁은 모퉁이 탐색 시간(기본 5.0) |
| `planner_server.ros__parameters.GridBased.terminal_checking_interval` | (없음) | 5000 | 기본값 |
| `planner_server.ros__parameters.GridBased.cache_obstacle_heuristic` | (없음) | False | 출발 자세가 목표마다 바뀌어 캐시 이득이 적음(기본값) |
| `planner_server.ros__parameters.GridBased.lookup_table_size` | (없음) | 6.0 | 기본 20 m 는 0.025 m 칸에서 시작 때 표 계산 26 s(도구 확인 TOOLCHECK_plan_v27 첫 시도) → 6 m(경기장 칸 단위 구간 길이). 표 밖은 장애물 휴리스틱만 |
| `planner_server.ros__parameters.GridBased.smooth_path` | (없음) | True | Smac 내장 다듬기(후진 끊는 점은 유지) |
| `planner_server.ros__parameters.GridBased.debug_visualizations` | (없음) | False | 기본값 |
| `planner_server.ros__parameters.GridBased.use_quadratic_cost_penalty` | (없음) | False | 기본값 |
| `planner_server.ros__parameters.GridBased.downsample_obstacle_heuristic` | (없음) | True | 기본값 |
| `planner_server.ros__parameters.GridBased.coarse_search_resolution` | (없음) | 1 | 좁은 곳에서 자세를 건너뛰지 않음 |
| `global_costmap.global_costmap.ros__parameters.footprint_padding` | (없음) | 0.04 | Smac 은 footprint 가 치명 칸에 닿는지만 봄 → 여백 없이는 경로 최소 여유 0.004~0.045 m(cost_penalty 2~8 · 회전 반경 0.05 바꿔도 같음). 전역 costmap 에만 여백을 줘 계획 경로가 벽에서 떨어지게(도구 확인 TOOLCHECK_plan_v27_variants: 0.02 → 최소 0.018, 0.03 → 0.023, 0.04 → 0.034~0.070 m, 7 구간 모두 경로 있음). 지역 costmap · MPPI · 충돌 감시는 여백 없음(실제 차체 그대로) |
| `controller_server.ros__parameters.PathHandler.enforce_path_inversion` | False | True | 후진 구간이 있는 경로를 끊는 점마다 잘라서 따름(MPPI 가 앞 · 뒤 구간을 한꺼번에 보고 지름길로 깎지 않게) |
| `controller_server.ros__parameters.PathHandler.inversion_xy_tolerance` | 0.2 | 0.06 | 끊는 점 도착 허용. 기본 0.2 는 모퉁이 여유(0.05 m)보다 커서 미리 다음 구간으로 넘어감 → 도착 허용 0.06 과 같게 |
| `controller_server.ros__parameters.PathHandler.inversion_yaw_tolerance` | 0.4 | 0.3 | 같은 이유로 기본 0.4 보다 좁게 |
| `controller_server.ros__parameters.FollowPath.use_path_orientations` | False | True | RotationShim 이 경로 앞점 방향(후진 구간이면 뒤쪽)으로 제자리 회전하지 않고 Smac 경로 자세 방향을 씀 → 후진 구간과 싸우지 않음. Shim 을 빼는 것보다 단순(도착 방향 회전 rotate_to_goal_heading 은 그대로 씀) |
| `global_costmap.global_costmap.ros__parameters.plugins` | ['static_layer', 'objects_layer', 'obstacle_layer', 'inflation_layer'] | ['static_layer', 'objects_layer', 'inflation_layer'] | 설계 27(live_nav_v27): 라이다 장애물 층이 벽 · 기둥 점을 스캔마다 조금씩 다르게 찍어, 여백 경계에 붙은 Smac 경로가 계속 무효(Path validation failed, 로봇이 서 있어도) → 약 0.7 s 마다 다시 계획 → 끊는 점 · 앞 방향이 바뀌어 RotationShim 이 좌우로 제자리 회전만 반복. 전역은 정적 지도(SLAM 벽 + 도면 벽 · 기둥 · 물체, 일치 검사 통과)만 써서 목표마다 한 번 계획한 경로를 유지. 움직이는 장애물은 지역 costmap(라이다) · MPPI · 충돌 감시가 그대로 봄. 도구 확인 plan_only 는 처음부터 이 구성 |
| `controller_server.ros__parameters.FollowPath.primary_controller` | nav2_mppi_controller::MPPIController | nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController | 설계 28(live_nav_v28): 경로는 안정됐는데(다시 계획 줄어듦) MPPI 가 끊는 점이 있는 Smac 경로를 따르지 않고 출발 방 서남쪽으로 0.4 m 들어가 멈춤(진행 실패). 전역 경로가 이미 차체 모양 · 여백 0.04 로 검사됐으므로 경로를 그대로 따르는 RPP 로 바꿈: allow_reversing(후진 구간을 뒤로), 전방 주시 0.15~0.4 m(회전 반경 0.10 경로), 곡률이 크면 속도를 0.04 까지 줄임(반경 0.4 기준), 충돌 검사는 지역 costmap footprint 로 1 s 앞까지. 속도 한계는 MPPI 와 같음(0.2 / −0.1 / 0.8). use_rotate_to_heading 은 allow_reversing 과 함께 못 쓰므로 끄고, 제자리 회전(처음 방향 · 도착 방향)은 바깥 RotationShim 이 그대로 함 |

- 함께 바꾸는 것(launch 인자): `objects_map:=config/map_objects_fine_posts_walls.yaml`(도면 벽 · 기둥 포함, check_world_consistency 통과판).
- 그대로: global costmap footprint(회전 중심 기준 x −0.0764~0.3416 · y ±0.171, Smac 이 이 footprint 로 검사), 팽창 0.45 · 6.0, MPPI 비평가, 속도 한계.
