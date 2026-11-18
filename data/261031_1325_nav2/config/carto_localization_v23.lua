-- 설계 19(2026-11-01) = carto_localization_v18.lua + 맨 끝 '설계 19' 두 줄. 설계 18 은 아래 설명대로.
-- 설계 18(2026-11-01) Cartographer 순수 위치 추정 설정. 261031_0335_slam_launch/config/carto_candidate.lua(1단계 SLAM, 모의 고정)를 그대로 복사하고 맨 끝 두 줄만 더함:
--   TRAJECTORY_BUILDER.pure_localization_trimmer(새 궤적의 submap 은 최근 3개만 남김) · POSE_GRAPH.optimize_every_n_nodes 20
--   (둘 다 cartographer_ros 공식 예제 backpack_2d_localization.lua 와 같은 값). 지도는 -load_state_filename … -load_frozen_state true 로 읽음.
--   optimize 20: SLAM 은 90(10-14 비교, 90 · 30 차이 없음)이지만 위치 추정에선 저장 지도에 대한 보정이 최적화 때만 반영되므로 공식 예제처럼 자주.
-- SLAM 후보(모의 고정 2026-10-31) Cartographer 설정. 261030_2058/carto_c6_2lidar_frozen.lua(c6, sha256 1e272fab…)에서 아래만 바꿈:
--   tracking_frame base_link(새 URDF 루트) · use_odometry false(위치 추정은 라이다만) · published_frame base_link · provide_odom_frame false
--   · use_pose_extrapolator false(스캔 시각 자세 · stamp) · publish_to_tf false · publish_tracked_pose true
--   → Cartographer 는 TF 를 내지 않고 /tracked_pose(map→base_link, 스캔 시각)만 냄. map→odom 은 nodes/map_odom_relay.py 가
--     같은 시각의 EKF odom→base_link 로 계산해 지금 시각으로 앞당겨 냄. 근거 261030_2058/README.md('TF 구성 B' · 'V4' · '실시간 관점 채점').
-- SLAM 비교(모의) Cartographer 설정: 2026-10-14 비교 최선 c6 (results/cartographer_ab/offline_20261014/configs/c6_res0.025_w10-40_ang1.lua) 를 그대로 쓰고 라이다 개수만 2로 바꿈(사용자 결정 2026-10-27 "c6 재사용").
-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- /* Author: Darby Lim */

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  use_pose_extrapolator = false,
  publish_to_tf = false,
  publish_tracked_pose = true,
  publish_frame_projected_to_2d = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 2,   -- 라이다 2개(앞 · 뒤), c6 에서 바꾼 유일한 줄
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 3.5
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)  -- 실험 변수 (원본: math.rad(0.1))

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- POSE_GRAPH.optimize_every_n_nodes = 0

-- ---- cartographer_ab 실험 변수 (원본 파일에는 없는 줄, Cartographer 기본값은 주석) ----
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.025  -- 기본값 0.05
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0  -- 기본값 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0  -- 기본값 40.

-- ---- 설계 18 순수 위치 추정(위 설명) ----
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}
POSE_GRAPH.optimize_every_n_nodes = 20

-- ---- 설계 19: 지도 전체 탐색 끔 · 지역 탐색 창 좁힘(이유 261101_0016_isaac_live/README.md '설계 18 결과') ----
-- 초기 자세를 줘도 새 궤적은 저장 궤적과 '연결'되지 않은 상태(연결 시각 기본값 0)라 지도 전체 탐색만 함(pose_graph_2d.cc 280~291) →
-- 미로의 비슷한 자리와 잘못 맞음(live_nav_v18: 14 s 에 yaw π 뒤집힘). 아래 값이면 '마지막 연결 + n 초'가 늘 노드 시각보다 커서
-- 지역 탐색(현재 추정 자세 둘레 창)만 함. 노드 시각은 0001 년 기준 약 6.4e10 s 라 1e11 로 둠. 위치를 잃었을 때 다시 찾는 일은 따로(loc_monitor).
POSE_GRAPH.global_constraint_search_after_n_seconds = 1e11
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 1.0  -- 기본 7 m. 초기 자세가 cm 수준이라 넓은 창은 비슷한 칸과 잘못 맞을 위험만 키움


-- ---- 도구 확인용(2026-11-01, 이 사본 v23 = v19 + 아래 두 줄. v19 는 그대로) ----
-- live_nav_v38 413~415 s 지역 맞춤 미끄러짐 → 지역 맞춤이 앞 자세(외삽)에서 벗어나는 것에 벌점 ×10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 100.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 400.0

return options
