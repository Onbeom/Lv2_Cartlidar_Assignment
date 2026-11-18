# 설계 37 = v21 에서 바꾼 값(설계 27 = 설계 26 불통과 뒤, 설계 28 = 설계 27 + 전역 라이다 층 뺌, 설계 29 = 설계 28 + RPP, 설계 30 = 설계 29 + Lattice 직선 · 제자리 회전, 설계 36 = 설계 35 + 지역 제어기 MPPI → RPP(V29), 설계 37 = 설계 36 + BT 경로 유효 검사 뺌, 기준 그대로)

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
| `planner_server.ros__parameters.GridBased` | nav2_navfn_planner::NavfnPlanner | nav2_smac_planner::SmacPlannerLattice | 설계 29(live_nav_v29): RPP 가 반경 0.10 호 경로를 따르다 통로 입구에서 30° 비스듬히 들어가 'collision ahead' 42회로 멈춤. Isaac 4륜 스키드는 전진하며 도는 호를 정확히 못 따름(실제 회전 53~75 %, 0.35 미만 회전은 거의 안 돎). 차체가 잘하는 직선 + 회전 중심 제자리 회전만으로 경로를 만듦 |
| `planner_server.ros__parameters.GridBased.lattice_filepath` | (없음) | /home/pa4/git/lv2_next_module/261031_1325_nav2/config/lattice_straight_rot_0025.json | tools/make_lattice_straight_rot.py 로 만든 0.025 m 칸 원시 동작(16 방향, 직선 짧은 것 · 약 0.1 m, 이웃 방향 제자리 회전 5° 간격 자세) |
| `planner_server.ros__parameters.GridBased.allow_reverse_expansion` | (없음) | True | 후진 직선 허용(좁은 곳 앞뒤 끊기) |
| `planner_server.ros__parameters.GridBased.reverse_penalty` | (없음) | 2.0 | v27 과 같음 |
| `planner_server.ros__parameters.GridBased.rotation_penalty` | (없음) | 5.0 | 기본값(제자리 회전을 필요할 때만) |
| `planner_server.ros__parameters.GridBased.change_penalty` | (없음) | 0.0 | 기본값 |
| `planner_server.ros__parameters.GridBased.non_straight_penalty` | (없음) | 1.05 | 기본값 |
| `planner_server.ros__parameters.GridBased.cost_penalty` | (없음) | 2.0 | v27 과 같음 |
| `planner_server.ros__parameters.GridBased.retrospective_penalty` | (없음) | 0.015 | 기본값 |
| `planner_server.ros__parameters.GridBased.analytic_expansion_ratio` | (없음) | 3.5 | 기본값 |
| `planner_server.ros__parameters.GridBased.analytic_expansion_max_length` | (없음) | 0.05 | 해석 연장은 호(반경 0.1)라 목표 바로 앞 0.05 m 안에서만 허용 |
| `planner_server.ros__parameters.GridBased.analytic_expansion_max_cost` | (없음) | 200.0 | 기본값 |
| `planner_server.ros__parameters.GridBased.analytic_expansion_max_cost_override` | (없음) | False | 기본값 |
| `planner_server.ros__parameters.GridBased.tolerance` | 0.1 | 0.05 | v27 과 같음 |
| `planner_server.ros__parameters.GridBased.allow_unknown` | False | False | v27 과 같음 |
| `planner_server.ros__parameters.GridBased.max_iterations` | (없음) | 1000000 | 기본값 |
| `planner_server.ros__parameters.GridBased.max_on_approach_iterations` | (없음) | 1000 | 기본값 |
| `planner_server.ros__parameters.GridBased.max_planning_time` | (없음) | 5.0 | v27 과 같음 |
| `planner_server.ros__parameters.GridBased.terminal_checking_interval` | (없음) | 5000 | 기본값 |
| `planner_server.ros__parameters.GridBased.cache_obstacle_heuristic` | (없음) | False | v27 과 같음 |
| `planner_server.ros__parameters.GridBased.lookup_table_size` | (없음) | 6.0 | v27 과 같음(시작 시간) |
| `planner_server.ros__parameters.GridBased.smooth_path` | (없음) | False | 다듬기는 제자리 회전 자리를 호로 바꿔 버림 → 끔 |
| `planner_server.ros__parameters.GridBased.debug_visualizations` | (없음) | False | 기본값 |
| `planner_server.ros__parameters.GridBased.use_quadratic_cost_penalty` | (없음) | False | 기본값 |
| `planner_server.ros__parameters.GridBased.downsample_obstacle_heuristic` | (없음) | True | 기본값 |
| `planner_server.ros__parameters.GridBased.coarse_search_resolution` | (없음) | 1 | v27 과 같음 |
| `controller_server.ros__parameters.PathHandler.enforce_path_rotation` | False | True | 경로 안의 제자리 회전 자리에서 경로를 끊어, 다음 구간 시작에서 RotationShim 이 제자리로 돌게 |
| `controller_server.ros__parameters.PathHandler.minimum_rotation_angle` | 0.785 | 0.25 | 원시 동작 한 칸 회전(18.4~26.6°)도 끊게 기본 0.785 보다 작게 |
| `controller_server.ros__parameters.FollowPath.angular_dist_threshold` | 0.785 | 0.25 | 같은 이유로 Shim 이 0.25 rad 넘게 틀어진 구간 시작에서 제자리 회전(기본 0.785) |
| `controller_server.ros__parameters.FollowPath.primary_controller.desired_linear_vel` | (없음) | None | 이 버전에서 폐기 예정(경고) → max_linear_vel 로 |
| `controller_server.ros__parameters.FollowPath.primary_controller.max_linear_vel` | (없음) | 0.12 | v29 의 목표 속도 0.12 를 이 이름으로(실제로 0.12 로 달렸음) |
| `planner_server.ros__parameters.GridBased.lattice_filepath` | (없음) | /home/pa4/git/lv2_next_module/261031_1325_nav2/config/lattice_straight_rot_0025_nomid.json | 설계 30(live_nav_v30): 출발 자리의 작은 제자리 회전을 Shim 이 하지 않고 RPP 가 호로 돎. 설치된 nav2_util::findFirstPathConstraint 를 v30 경로에 그대로 불러 보니(test/probe_constraint, 판정 아님) 누적 회전이 0.25 rad 를 넘는 자세(회전 도중 약 14°)에서 끊어, 끊은 자세가 처음 방향과 13° 차이 → 허용 0.3 rad 안이라 회전 없이 다음 구간으로 넘어감. 중간 자세를 뺀 원시 동작(make_lattice_straight_rot.py --no_rot_mid)으로 회전 한 번 = 한 단계(18.4~26.6°)가 되어 회전 끝 자세에서 끊김. 회전 중 쓸림은 전역 여백 0.04 가 덮음(바깥 모서리 반지름 0.382 m, 26.6° 회전의 끝 자세 밖 쓸림 ≈ 0.382 × (1 − cos 13.3°) = 0.010 m) — plan_only 쓸림 여유로 확인 |
| `controller_server.ros__parameters.PathHandler.inversion_xy_tolerance` | 0.2 | 0.03 | 끊는 점(회전 직전 자세)에서 0.06 m 앞에서 넘어가면 그만큼 일찍 돌아 좁은 곳(쓸림 여유 0.023~0.05 m)에서 벽 쪽으로 감. RPP 는 끝 0.3 m 에서 0.04 m/s 로 줄여 20 Hz 에 2 mm 씩 다가가므로 0.03 이면 닿음 |
| `controller_server.ros__parameters.PathHandler.inversion_yaw_tolerance` | 0.4 | 0.15 | 회전 자리 도착 허용을 가장 작은 회전(18.4° = 0.32 rad)보다 작게, Shim 해제 문턱(0.1)보다 크게 → 회전을 실제로 한 뒤에만 다음 구간 |
| `controller_server.ros__parameters.FollowPath.forward_sampling_distance` | 0.5 | 0.03 | Shim 은 경로 첫 자세에서 이 거리 이상 떨어진 첫 자세의 방향을 봄. 0.5 면 0.1~0.3 m 짧은 직선 구간에서 끝(회전한) 자세를 미리 보고 직선 전에 돌아 버림 → 자세 간격(0.025)보다 조금 크게 |
| `planner_server.ros__parameters.GridBased.max_on_approach_iterations` | (없음) | 100000 | 설계 31: 허용(0.05 m) 안 노드가 나온 뒤 1000 번만 더 찾고 못 찾으면 방향 상관없이 가장 가까운 노드로 끝냄 → 경로 끝 방향이 목표와 최대 120° 다름(start_box_1 2.786 → 1.571), 제어기는 경로 끝을 목표로 봐서 그 방향에서 SUCCEEDED(G2 yaw 1.19 rad). 정확한 목표 칸 · 방향 칸까지 찾게 크게 |
| `bt_navigator.ros__parameters.default_nav_to_pose_bt_xml` | (없음) | /home/pa4/git/lv2_next_module/261031_1325_nav2/config/bt/navigate_to_pose_replan_after_failure.xml | 설계 31: FollowPath 실패 뒤 원본 BT 가 옛 경로를 다시 보내 경로 처리기가 첫 구간(지나온 출발 쪽)부터 시작 → 'Resulting plan has 0 poses'. 실패 · 복구 뒤 현재 자리에서 다시 계획하는 사본(config/bt/) |
| `planner_server.ros__parameters.GridBased.analytic_expansion_max_length` | (없음) | 0.15 | 정확한 목표 xy 로 끝내기: 0.05 면 허용 안 칸 노드에서 끝나 목표와 0.004~0.015 m 다름. 0.15 면 7 구간 모두 xy 0.000, 끝 호 0.004~0.043 m(도구 확인 TOOLCHECK_plan_v32_a15, 0.30 은 호 0.04~0.10 m 로 길어짐). 방향은 목표 칸 방향(16 방향)으로 끝나므로 남은 방향은 도착 뒤 Spin(run_goals --final_spin_tol) |
| `planner_server.ros__parameters.GridBased.max_planning_time` | (없음) | 10.0 | 마지막 긴 구간(6.5 m)은 계획 7.3~7.8 s → 5 s 에서 끊겨 끝 방향이 45° 틀렸음(TOOLCHECK_plan_v32) |
| `controller_server.ros__parameters.PathHandler.inversion_yaw_tolerance` | 0.4 | 0.2 | 다시 계획한 경로의 첫 자세 방향은 로봇 방향에 가장 가까운 칸(16 방향, 최대 11.25° = 0.196 rad 차이) → 첫 구간이 제자리 회전이면 허용 안에 들어야 넘어감. 가장 작은 회전 0.32 rad 보다는 작게(회전을 건너뛰지 않게) |
| `controller_server.ros__parameters.FollowPath.angular_disengage_threshold` | 0.1 | 0.05 | 설계 31 start_box_3: 63 → 90° 회전이 78° 에서 끝남(해제 0.1 rad + 추정 방향이 회전 중 참보다 앞섬). 전진 중 작은 회전은 거의 안 들어 남은 오차가 옆 벗어남(0.18 m)이 됨 → 더 맞추고 끝냄 |
| `velocity_smoother.ros__parameters.max_accel` | [0.5, 0.0, 1.5] | [0.5, 0.0, 3.2] | 각가속 1.5 → 3.2(behavior_server rotational_acc_lim 과 같게). 선속도는 그대로 |
| `velocity_smoother.ros__parameters.max_decel` | [-0.5, 0.0, -1.5] | [-0.5, 0.0, -3.2] | 설계 33(live_nav_v33): 도착 뒤 Spin 이 0.8 rad/s 로 돌다 멈출 때 smoother 각감속 1.5 로 묶여 0.8²/(2·1.5) = 0.21 rad 더 돎(L3 관찰 0.216 rad 넘어감, yaw 오차 0.22 → 반대쪽 0.23). Spin 자체 감속 곡선(3.2)을 따르게 |
| `behavior_server.ros__parameters.max_rotational_vel` | 0.8 | 0.5 | Spin(도착 뒤 방향 맞춤 · 복구 회전) 최대 0.8 → 0.5. 멈춤 거리 0.5²/(2·3.2) = 0.04 rad, 측정 곡선에서 명령 0.5 ≈ 실제 0.3 |
| `controller_server.ros__parameters.general_goal_checker.xy_goal_tolerance` | 0.06 | 0.025 | 설계 34(live_nav_v34): 도착 위치 오차가 모든 목표에서 0.049~0.068 m(허용 0.06 경계에 들어서자 멈춤). 좁은 L3 칸에서는 0.05 m 덜 간 자리에서 여백 0.04 footprint 가 점유 → 다음 구간 'Start occupied'. 경로 끝은 목표 xy 와 같으므로(해석 연장 0.15) 더 가까이 |
| `behavior_server.ros__parameters.min_rotational_vel` | 0.35 | 0.19 | 설계 34: 도착 뒤 Spin 이 0.104 → −0.087 → 0.092 처럼 매번 약 0.09 rad 지나침. 최소 0.35 로 끝까지 돌다 멈춤 + 차체 · 추정 지연. turn_floor 제자리 바닥(turn_floor_w_min 0.19, turn_ff 로 실제 0.19)과 같게 |
| `behavior_server.ros__parameters.rotational_acc_lim` | 3.2 | 1.0 | Spin 감속 곡선 √(2·a·남은 각): 3.2 면 남은 0.04 rad 까지 최대 속도 → 1.0 이면 남은 0.1 rad 에서 0.45, 0.02 에서 0.2 로 미리 줄임. smoother 각감속 3.2 보다 작아 그대로 따라감 |
| `bt_navigator.ros__parameters.default_nav_to_pose_bt_xml` | (없음) | /home/pa4/git/lv2_next_module/261031_1325_nav2/config/bt/navigate_to_pose_replan_only_after_failure.xml | 설계 36(live_nav_v36): 전역 costmap 이 정적 지도뿐인데 ValidatePath 가 Smac 경로를 1 s 마다 무효로 봐 다시 계획 반복 → 좁은 L3 칸에서 앞뒤로 떨다 'Start occupied'. 유효 검사를 뺀 BT 사본(목표 바뀜 · 실패 뒤 · 복구 뒤만 다시 계획) |

- 함께 바꾸는 것(launch 인자): `objects_map:=config/map_objects_fine_posts_walls.yaml`(도면 벽 · 기둥 포함, check_world_consistency 통과판).
- 그대로: global costmap footprint(회전 중심 기준 x −0.0764~0.3416 · y ±0.171, Smac 이 이 footprint 로 검사), 팽창 0.45 · 6.0, MPPI 비평가, 속도 한계.
