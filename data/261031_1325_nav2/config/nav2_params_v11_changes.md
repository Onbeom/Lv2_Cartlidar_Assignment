# 기본 설정에서 바꾼 값

기본 = ROS 2 Lyrical `nav2_bringup/params/nav2_params.yaml`(복사본 `nav2_params_default_lyrical.yaml`). 이번에 띄우지 않는 서버(docking · route · 필터 · loopback · waypoint_follower)와 costmap 필터(keepout · speed)는 뺐다.

| 항목 | 기본 | 바꾼 값 | 이유 |
|---|---|---|---|
| `amcl.ros__parameters.base_frame_id` | `base_footprint` | `base_link` | 로봇 frame(팀 dev 규약 odom→base_link) |
| `amcl.ros__parameters.scan_topic` | `scan` | `/scan_merged` | 앞 · 뒤 가림 제거 스캔을 base_link 기준으로 합친 것(AMCL 은 토픽 하나) |
| `amcl.ros__parameters.laser_max_range` | `100.0` | `12.0` | 모의 · Isaac 스캔 range_max 와 같게(실물 LDS 는 그 장비 값) |
| `amcl.ros__parameters.set_initial_pose` | `(없음)` | `True` | 발제 출발 자세(0.734, 0.95, π)에서 시작 |
| `amcl.ros__parameters.initial_pose` | `(없음)` | `…` | 발제 출발 자세 |
| `bt_navigator.ros__parameters.odom_topic` | `odom` | `/odometry/filtered` | EKF 출력(odom→base_link 와 같은 출처) |
| `behavior_server.ros__parameters.local_footprint_topic` | `local_costmap/published_footprint` | `local_costmap/published_footprint` | 그대로(확인용) |
| `behavior_server.ros__parameters.max_rotational_vel` | `1.0` | `0.8` | 모의 보수 속도(실물 운용값 아님). 바퀴 한계 10.996 rad/s × 0.033 m = 0.36 m/s 안 |
| `behavior_server.ros__parameters.min_rotational_vel` | `0.4` | `0.35` | 모의 녹화에서 회전이 멈추지 않은 최소 회전 속도(261030_1943 T1) |
| `controller_server.ros__parameters.odom_topic` | `(없음)` | `/odometry/filtered` | EKF 출력 |
| `controller_server.ros__parameters.enable_stamped_cmd_vel` | `(없음)` | `True` | 팀 twist_mux use_stamped: true |
| `controller_server.ros__parameters.general_goal_checker.xy_goal_tolerance` | `0.25` | `0.06` | 판정 위치 오차 ≤ 0.10 m(위치 추정 오차 몫 0.04 남김) |
| `controller_server.ros__parameters.general_goal_checker.yaw_goal_tolerance` | `0.25` | `0.1` | 판정 yaw ≤ 0.15 rad |
| `controller_server.ros__parameters.FollowPath.vx_max` | `0.5` | `0.2` | 모의 보수 속도(실물 운용값 아님). 바퀴 한계 10.996 rad/s × 0.033 m = 0.36 m/s 안 |
| `controller_server.ros__parameters.FollowPath.vx_min` | `-0.35` | `-0.1` | 모의 보수 속도(실물 운용값 아님). 바퀴 한계 10.996 rad/s × 0.033 m = 0.36 m/s 안 |
| `controller_server.ros__parameters.FollowPath.wz_max` | `1.9` | `0.8` | 모의 보수 속도(실물 운용값 아님). 바퀴 한계 10.996 rad/s × 0.033 m = 0.36 m/s 안 |
| `controller_server.ros__parameters.FollowPath.ax_max` | `3.0` | `0.5` | 모의 보수 속도(실물 운용값 아님). 바퀴 한계 10.996 rad/s × 0.033 m = 0.36 m/s 안 |
| `controller_server.ros__parameters.FollowPath.ax_min` | `-3.0` | `-0.5` | 모의 보수 속도(실물 운용값 아님). 바퀴 한계 10.996 rad/s × 0.033 m = 0.36 m/s 안 |
| `controller_server.ros__parameters.FollowPath.az_max` | `3.5` | `1.5` | 모의 보수 속도(실물 운용값 아님). 바퀴 한계 10.996 rad/s × 0.033 m = 0.36 m/s 안 |
| `controller_server.ros__parameters.FollowPath.vx_std` | `0.2` | `0.1` | 속도 상한을 줄인 만큼 표본 폭도 줄임 |
| `controller_server.ros__parameters.FollowPath.wz_std` | `0.4` | `0.3` | 같은 이유 |
| `controller_server.ros__parameters.FollowPath.CostCritic.consider_footprint` | `False` | `True` | 사각 footprint(앞 0.263 · 옆 0.171)라 원으로 보면 옆 · 앞을 놓침 |
| `controller_server.ros__parameters.FollowPath.TrajectoryValidator.consider_footprint` | `False` | `True` | 같은 이유 |
| `local_costmap.local_costmap.ros__parameters.robot_radius` | `0.22` | `None` | footprint 로 바꿈 |
| `local_costmap.local_costmap.ros__parameters.footprint` | `(없음)` | `[[-0.155, -0.171], [-0.155, 0.171], [0.263, 0.171], [0.263, -0.171]]` | 차체 x −0.155~0.263(drive 그리퍼) · y ±0.171, 실제보다 작게 하지 않음 |
| `local_costmap.local_costmap.ros__parameters.resolution` | `0.05` | `0.025` | SLAM 지도 해상도와 같게(미로 칸 0.734 m 에 회전 반경 0.314 m) |
| `local_costmap.local_costmap.ros__parameters.plugins` | `['voxel_layer', 'inflation_layer']` | `['obstacle_layer', 'objects_layer', 'inflation_layer']` | 라이다 2개 + 발제 고정 물체(라이다에 안 보이는 0.07 m 상자) |
| `local_costmap.local_costmap.ros__parameters.voxel_layer` | `…` | `None` | 2D 라이다뿐이라 obstacle_layer 로 바꿈 |
| `local_costmap.local_costmap.ros__parameters.obstacle_layer` | `(없음)` | `…` | 라이다 2개(가림 제거 토픽) |
| `local_costmap.local_costmap.ros__parameters.objects_layer` | `(없음)` | `…` | 발제 고정 물체 층 |
| `local_costmap.local_costmap.ros__parameters.use_maximum` | `(없음)` | `True` | 정적 층 2개(벽 · 물체)를 큰 값으로 합침(costmap 단위 파라미터, 물체 지도의 모름 칸은 건너뜀) |
| `local_costmap.local_costmap.ros__parameters.inflation_layer.inflation_radius` | `0.7` | `0.45` | 미로 통로 0.73 m 에서 가운데 비용이 너무 높지 않게(외접 반경 0.314 m 보다 큼) |
| `local_costmap.local_costmap.ros__parameters.inflation_layer.cost_scaling_factor` | `3.0` | `6.0` | 같은 이유 |
| `global_costmap.global_costmap.ros__parameters.robot_radius` | `0.22` | `None` | footprint 로 바꿈 |
| `global_costmap.global_costmap.ros__parameters.footprint` | `(없음)` | `[[-0.155, -0.171], [-0.155, 0.171], [0.263, 0.171], [0.263, -0.171]]` | 위와 같음 |
| `global_costmap.global_costmap.ros__parameters.resolution` | `0.05` | `0.025` | SLAM 지도 해상도와 같게 |
| `global_costmap.global_costmap.ros__parameters.plugins` | `['static_layer', 'obstacle_layer', 'inflation_layer']` | `['static_layer', 'objects_layer', 'obstacle_layer', 'inflation_layer']` | 벽 = SLAM 지도, 물체 = 발제 배치 |
| `global_costmap.global_costmap.ros__parameters.objects_layer` | `(없음)` | `…` | 발제 고정 물체 층 |
| `global_costmap.global_costmap.ros__parameters.obstacle_layer` | `…` | `…` | 라이다 2개 |
| `global_costmap.global_costmap.ros__parameters.use_maximum` | `(없음)` | `True` | 위와 같음 |
| `global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius` | `0.7` | `0.45` | 위와 같음 |
| `global_costmap.global_costmap.ros__parameters.inflation_layer.cost_scaling_factor` | `3.0` | `6.0` | 위와 같음 |
| `planner_server.ros__parameters.GridBased.tolerance` | `0.5` | `0.1` | 목표가 막혔을 때 0.5 m 옆으로 바꿔 가지 않게 |
| `planner_server.ros__parameters.GridBased.allow_unknown` | `True` | `False` | 지도 밖 모름 칸으로 경로를 내지 않게 |
| `velocity_smoother.ros__parameters.enable_stamped_cmd_vel` | `(없음)` | `True` | 팀 twist_mux use_stamped: true |
| `velocity_smoother.ros__parameters.odom_topic` | `odom` | `/odometry/filtered` | EKF 출력 |
| `velocity_smoother.ros__parameters.max_velocity` | `[0.5, 0.0, 2.0]` | `[0.2, 0.0, 0.8]` | 모의 보수 속도(실물 운용값 아님). 바퀴 한계 10.996 rad/s × 0.033 m = 0.36 m/s 안 |
| `velocity_smoother.ros__parameters.min_velocity` | `[-0.5, 0.0, -2.0]` | `[-0.1, 0.0, -0.8]` | 모의 보수 속도(실물 운용값 아님). 바퀴 한계 10.996 rad/s × 0.033 m = 0.36 m/s 안 |
| `velocity_smoother.ros__parameters.max_accel` | `[2.5, 0.0, 3.2]` | `[0.5, 0.0, 1.5]` | 모의 보수 속도(실물 운용값 아님). 바퀴 한계 10.996 rad/s × 0.033 m = 0.36 m/s 안 |
| `velocity_smoother.ros__parameters.max_decel` | `[-2.5, 0.0, -3.2]` | `[-0.5, 0.0, -1.5]` | 모의 보수 속도(실물 운용값 아님). 바퀴 한계 10.996 rad/s × 0.033 m = 0.36 m/s 안 |
| `collision_monitor.ros__parameters.enable_stamped_cmd_vel` | `(없음)` | `True` | 팀 twist_mux use_stamped: true |
| `collision_monitor.ros__parameters.base_frame_id` | `base_footprint` | `base_link` | 로봇 frame(팀 dev 규약 odom→base_link) |
| `collision_monitor.ros__parameters.cmd_vel_out_topic` | `cmd_vel` | `cmd_vel_nav` | 팀 twist_mux nav 입력 = nav2 체인의 마지막 출력(collision_monitor) |
| `collision_monitor.ros__parameters.observation_sources` | `['scan']` | `['scan_front', 'scan_rear']` | 라이다 2개 |
| `collision_monitor.ros__parameters.scan` | `…` | `None` | 라이다 2개로 바꿈 |
| `collision_monitor.ros__parameters.scan_front` | `(없음)` | `…` | 가림 제거 토픽 |
| `collision_monitor.ros__parameters.scan_rear` | `(없음)` | `…` | 가림 제거 토픽 |
| `behavior_server.ros__parameters.enable_stamped_cmd_vel` | `(없음)` | `True` | 팀 twist_mux use_stamped: true |
| `amcl.ros__parameters.update_min_d` | `0.25` | `0.05` | run1 위치 추정 RMSE 0.046 m(기준 0.05 에 가까움) → 더 자주 갱신 |
| `amcl.ros__parameters.update_min_a` | `0.2` | `0.05` | 같은 이유 |
| `amcl.ros__parameters.max_beams` | `60` | `120` | 같은 이유(합친 스캔 720칸 중 120개) |
| `controller_server.ros__parameters.FollowPath.GoalAngleCritic.threshold_to_consider` | `0.5` | `0.15` | run1: 목표 0.5 m 안에서 목표 방향으로 먼저 돌아 옆으로 못 감(차동 구동) → 위치 먼저 |
| `controller_server.ros__parameters.progress_checker.required_movement_radius` | `0.5` | `0.25` | 좁은 칸 마무리 움직임이 10 s 에 0.5 m 못 됨 |
| `controller_server.ros__parameters.progress_checker.movement_time_allowance` | `10.0` | `15.0` | 같은 이유 |
| `controller_server.ros__parameters.general_goal_checker.yaw_goal_tolerance` | `0.1` | `0.05` | v2b: 끝 yaw 오차 0.145~0.173 rad(기준 0.15) — 0.10 에서 멈출 때 남은 회전 속도로 더 돎 + 위치 추정 yaw 오차 |
| `local_costmap.local_costmap.ros__parameters.resolution` | `0.025` | `0.0125` | v9_c: 참값 회전 여유 0.029 m 인데 costmap 상 충돌 — 격자 반올림(물체 지도 덧댐 · local 재표본 · footprint 윤곽)을 줄임. 물체 지도도 0.01 m(map_objects_fine) |
| `controller_server.ros__parameters.FollowPath` | MPPIController | RotationShimController(primary_controller = 위 MPPI) | RotationShim(MPPI 를 primary_controller 로): 경로 방향과 0.785 rad 넘게 다르면 제자리 회전, 목표 허용 오차 안에 들면 제자리에서 목표 방향으로 회전 — 차동 구동이 위치 먼저 · 방향 나중(run1 실패 원인) |
| `controller_server.ros__parameters.FollowPath.rotate_to_heading_angular_vel` | `0.8` | `0.5` | 같은 이유: 제자리 회전을 느리게 · 부드럽게 끝내 넘어 돎을 줄임 |
| `controller_server.ros__parameters.FollowPath.max_angular_accel` | `1.5` | `0.8` | 같은 이유: 제자리 회전을 느리게 · 부드럽게 끝내 넘어 돎을 줄임 |
| `controller_server.ros__parameters.FollowPath.angular_disengage_threshold` | `0.3925` | `0.1` | v5 · v6: 목표 근처에서 경로 방향 회전을 0.39 rad 남기고 MPPI 에 넘기자 1점 경로가 잘려 “plan has 0 poses” 로 끝남 → 0.10 rad 까지 직접 회전 |
