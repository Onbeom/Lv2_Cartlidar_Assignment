"""설치 버전(ROS 2 Lyrical nav2_bringup) 기본 nav2_params.yaml 에서 바꾼 값만 적용해 config/nav2_params.yaml 을 만든다.
바꾼 값 목록 = CHANGES(경로, 값, 이유). 결과 파일과 함께 config/nav2_params_changes.md 에 표로 남긴다.
--variant v2: CHANGES 뒤에 V2(설계 2, run1 실패 뒤)를 더 적용하고 MPPI 를 RotationShim 의 primary_controller 로 옮겨 nav2_params_v2.yaml 로 쓴다."""
import argparse
import copy
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent.parent
src = yaml.safe_load((HERE / 'config' / 'nav2_params_default_lyrical.yaml').read_text())
FP = '[[-0.155, -0.171], [-0.155, 0.171], [0.263, 0.171], [0.263, -0.171]]'
SCANS = {'observation_sources': 'scan_front scan_rear'}
for k, t in (('scan_front', '/scan_front_filtered'), ('scan_rear', '/scan_rear_filtered')):
    SCANS[k] = {'topic': t, 'data_type': 'LaserScan', 'clearing': True, 'marking': True, 'max_obstacle_height': 2.0,
                'raytrace_max_range': 3.5, 'raytrace_min_range': 0.0, 'obstacle_max_range': 3.0, 'obstacle_min_range': 0.0}
OBJ_LAYER = {'plugin': 'nav2_costmap_2d::StaticLayer', 'map_topic': '/map_objects', 'map_subscribe_transient_local': True}
ROBOT = '로봇 frame(팀 dev 규약 odom→base_link)'
SPEED = '모의 보수 속도(실물 운용값 아님). 바퀴 한계 10.996 rad/s × 0.033 m = 0.36 m/s 안'
CHANGES = [
    # AMCL
    ('amcl.ros__parameters.base_frame_id', 'base_link', ROBOT),
    ('amcl.ros__parameters.scan_topic', '/scan_merged', '앞 · 뒤 가림 제거 스캔을 base_link 기준으로 합친 것(AMCL 은 토픽 하나)'),
    ('amcl.ros__parameters.laser_max_range', 12.0, '모의 · Isaac 스캔 range_max 와 같게(실물 LDS 는 그 장비 값)'),
    ('amcl.ros__parameters.set_initial_pose', True, '발제 출발 자세(0.734, 0.95, π)에서 시작'),
    ('amcl.ros__parameters.initial_pose', {'x': 0.734, 'y': 0.95, 'z': 0.0, 'yaw': 3.141593}, '발제 출발 자세'),
    # BT · 행동
    ('bt_navigator.ros__parameters.odom_topic', '/odometry/filtered', 'EKF 출력(odom→base_link 와 같은 출처)'),
    ('behavior_server.ros__parameters.local_footprint_topic', 'local_costmap/published_footprint', '그대로(확인용)'),
    ('behavior_server.ros__parameters.max_rotational_vel', 0.8, SPEED),
    ('behavior_server.ros__parameters.min_rotational_vel', 0.35, '모의 녹화에서 회전이 멈추지 않은 최소 회전 속도(261030_1943 T1)'),
    # controller · MPPI
    ('controller_server.ros__parameters.odom_topic', '/odometry/filtered', 'EKF 출력'),
    ('controller_server.ros__parameters.enable_stamped_cmd_vel', True, '팀 twist_mux use_stamped: true'),
    ('controller_server.ros__parameters.general_goal_checker.xy_goal_tolerance', 0.06, '판정 위치 오차 ≤ 0.10 m(위치 추정 오차 몫 0.04 남김)'),
    ('controller_server.ros__parameters.general_goal_checker.yaw_goal_tolerance', 0.10, '판정 yaw ≤ 0.15 rad'),
    ('controller_server.ros__parameters.FollowPath.vx_max', 0.20, SPEED),
    ('controller_server.ros__parameters.FollowPath.vx_min', -0.10, SPEED),
    ('controller_server.ros__parameters.FollowPath.wz_max', 0.8, SPEED),
    ('controller_server.ros__parameters.FollowPath.ax_max', 0.5, SPEED),
    ('controller_server.ros__parameters.FollowPath.ax_min', -0.5, SPEED),
    ('controller_server.ros__parameters.FollowPath.az_max', 1.5, SPEED),
    ('controller_server.ros__parameters.FollowPath.vx_std', 0.1, '속도 상한을 줄인 만큼 표본 폭도 줄임'),
    ('controller_server.ros__parameters.FollowPath.wz_std', 0.3, '같은 이유'),
    ('controller_server.ros__parameters.FollowPath.CostCritic.consider_footprint', True, '사각 footprint(앞 0.263 · 옆 0.171)라 원으로 보면 옆 · 앞을 놓침'),
    ('controller_server.ros__parameters.FollowPath.TrajectoryValidator.consider_footprint', True, '같은 이유'),
    # costmap
    ('local_costmap.local_costmap.ros__parameters.robot_radius', None, 'footprint 로 바꿈'),
    ('local_costmap.local_costmap.ros__parameters.footprint', FP, '차체 x −0.155~0.263(drive 그리퍼) · y ±0.171, 실제보다 작게 하지 않음'),
    ('local_costmap.local_costmap.ros__parameters.resolution', 0.025, 'SLAM 지도 해상도와 같게(미로 칸 0.734 m 에 회전 반경 0.314 m)'),
    ('local_costmap.local_costmap.ros__parameters.plugins', ['obstacle_layer', 'objects_layer', 'inflation_layer'], '라이다 2개 + 발제 고정 물체(라이다에 안 보이는 0.07 m 상자)'),
    ('local_costmap.local_costmap.ros__parameters.voxel_layer', None, '2D 라이다뿐이라 obstacle_layer 로 바꿈'),
    ('local_costmap.local_costmap.ros__parameters.obstacle_layer', dict(plugin='nav2_costmap_2d::ObstacleLayer', enabled=True, **SCANS), '라이다 2개(가림 제거 토픽)'),
    ('local_costmap.local_costmap.ros__parameters.objects_layer', OBJ_LAYER, '발제 고정 물체 층'),
    ('local_costmap.local_costmap.ros__parameters.use_maximum', True, '정적 층 2개(벽 · 물체)를 큰 값으로 합침(costmap 단위 파라미터, 물체 지도의 모름 칸은 건너뜀)'),
    ('local_costmap.local_costmap.ros__parameters.inflation_layer.inflation_radius', 0.45, '미로 통로 0.73 m 에서 가운데 비용이 너무 높지 않게(외접 반경 0.314 m 보다 큼)'),
    ('local_costmap.local_costmap.ros__parameters.inflation_layer.cost_scaling_factor', 6.0, '같은 이유'),
    ('global_costmap.global_costmap.ros__parameters.robot_radius', None, 'footprint 로 바꿈'),
    ('global_costmap.global_costmap.ros__parameters.footprint', FP, '위와 같음'),
    ('global_costmap.global_costmap.ros__parameters.resolution', 0.025, 'SLAM 지도 해상도와 같게'),
    ('global_costmap.global_costmap.ros__parameters.plugins', ['static_layer', 'objects_layer', 'obstacle_layer', 'inflation_layer'], '벽 = SLAM 지도, 물체 = 발제 배치'),
    ('global_costmap.global_costmap.ros__parameters.objects_layer', OBJ_LAYER, '발제 고정 물체 층'),
    ('global_costmap.global_costmap.ros__parameters.obstacle_layer', dict(plugin='nav2_costmap_2d::ObstacleLayer', enabled=True, **SCANS), '라이다 2개'),
    ('global_costmap.global_costmap.ros__parameters.use_maximum', True, '위와 같음'),
    ('global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius', 0.45, '위와 같음'),
    ('global_costmap.global_costmap.ros__parameters.inflation_layer.cost_scaling_factor', 6.0, '위와 같음'),
    # planner
    ('planner_server.ros__parameters.GridBased.tolerance', 0.10, '목표가 막혔을 때 0.5 m 옆으로 바꿔 가지 않게'),
    ('planner_server.ros__parameters.GridBased.allow_unknown', False, '지도 밖 모름 칸으로 경로를 내지 않게'),
    # 속도 다듬기 · 충돌 감시
    ('velocity_smoother.ros__parameters.enable_stamped_cmd_vel', True, '팀 twist_mux use_stamped: true'),
    ('velocity_smoother.ros__parameters.odom_topic', '/odometry/filtered', 'EKF 출력'),
    ('velocity_smoother.ros__parameters.max_velocity', [0.20, 0.0, 0.8], SPEED),
    ('velocity_smoother.ros__parameters.min_velocity', [-0.10, 0.0, -0.8], SPEED),
    ('velocity_smoother.ros__parameters.max_accel', [0.5, 0.0, 1.5], SPEED),
    ('velocity_smoother.ros__parameters.max_decel', [-0.5, 0.0, -1.5], SPEED),
    ('collision_monitor.ros__parameters.enable_stamped_cmd_vel', True, '팀 twist_mux use_stamped: true'),
    ('collision_monitor.ros__parameters.base_frame_id', 'base_link', ROBOT),
    ('collision_monitor.ros__parameters.cmd_vel_out_topic', 'cmd_vel_nav', '팀 twist_mux nav 입력 = nav2 체인의 마지막 출력(collision_monitor)'),
    ('collision_monitor.ros__parameters.observation_sources', ['scan_front', 'scan_rear'], '라이다 2개'),
    ('collision_monitor.ros__parameters.scan', None, '라이다 2개로 바꿈'),
    ('collision_monitor.ros__parameters.scan_front', {'type': 'scan', 'topic': '/scan_front_filtered', 'enabled': True}, '가림 제거 토픽'),
    ('collision_monitor.ros__parameters.scan_rear', {'type': 'scan', 'topic': '/scan_rear_filtered', 'enabled': True}, '가림 제거 토픽'),
    ('behavior_server.ros__parameters.enable_stamped_cmd_vel', True, '팀 twist_mux use_stamped: true'),
]


V2 = [
    ('amcl.ros__parameters.update_min_d', 0.05, 'run1 위치 추정 RMSE 0.046 m(기준 0.05 에 가까움) → 더 자주 갱신'),
    ('amcl.ros__parameters.update_min_a', 0.05, '같은 이유'),
    ('amcl.ros__parameters.max_beams', 120, '같은 이유(합친 스캔 720칸 중 120개)'),
    ('controller_server.ros__parameters.FollowPath.GoalAngleCritic.threshold_to_consider', 0.15,
     'run1: 목표 0.5 m 안에서 목표 방향으로 먼저 돌아 옆으로 못 감(차동 구동) → 위치 먼저'),
    ('controller_server.ros__parameters.progress_checker.required_movement_radius', 0.25, '좁은 칸 마무리 움직임이 10 s 에 0.5 m 못 됨'),
    ('controller_server.ros__parameters.progress_checker.movement_time_allowance', 15.0, '같은 이유'),
]
V3 = [('controller_server.ros__parameters.general_goal_checker.yaw_goal_tolerance', 0.05,
       'v2b: 끝 yaw 오차 0.145~0.173 rad(기준 0.15) — 0.10 에서 멈출 때 남은 회전 속도로 더 돎 + 위치 추정 yaw 오차')]
V7_SHIM = {'angular_disengage_threshold': 0.10}
V11 = [('local_costmap.local_costmap.ros__parameters.resolution', 0.0125,
        'v9_c: 참값 회전 여유 0.029 m 인데 costmap 상 충돌 — 격자 반올림(물체 지도 덧댐 · local 재표본 · footprint 윤곽)을 줄임. 물체 지도도 0.01 m(map_objects_fine)')]
V10 = [('controller_server.ros__parameters.general_goal_checker.xy_goal_tolerance', 0.03,
        'v7 반복 r3 · v9_c: 위치 허용 0.06 안에 든 뒤 목표 방향 회전 중 밀려 추정 자세가 목표에서 0.064 · 0.083 m — 회전 여유 좁은 태그 칸에서 costmap 상 충돌로 못 나감'),
       ('controller_server.ros__parameters.general_goal_checker.stateful', False, '같은 이유: 위치 도달을 한 번 잡은 뒤에도 끝날 때 위치를 다시 확인')]
V7_SHIM_WHY = ('v5 · v6: 목표 근처에서 경로 방향 회전을 0.39 rad 남기고 MPPI 에 넘기자 1점 경로가 잘려 “plan has 0 poses” 로 끝남 → 0.10 rad 까지 직접 회전')
V3_SHIM = {'rotate_to_heading_angular_vel': 0.5, 'max_angular_accel': 0.8}
V3_SHIM_WHY = '같은 이유: 제자리 회전을 느리게 · 부드럽게 끝내 넘어 돎을 줄임'
SHIM = {'plugin': 'nav2_rotation_shim_controller::RotationShimController', 'angular_dist_threshold': 0.785, 'angular_disengage_threshold': 0.3925,
        'forward_sampling_distance': 0.5, 'rotate_to_heading_angular_vel': 0.8, 'max_angular_accel': 1.5, 'simulate_ahead_time': 1.0,
        'rotate_to_goal_heading': True, 'rotate_to_heading_once': False, 'closed_loop': True, 'use_path_orientations': False}
SHIM_WHY = ('RotationShim(MPPI 를 primary_controller 로): 경로 방향과 0.785 rad 넘게 다르면 제자리 회전, 목표 허용 오차 안에 들면 제자리에서 목표 방향으로 회전'
            ' — 차동 구동이 위치 먼저 · 방향 나중(run1 실패 원인)')


def put(d, path, value):
    keys = path.split('.')
    for k in keys[:-1]:
        d = d.setdefault(k, {})
    old = d.get(keys[-1], '(없음)')
    if value is None:
        d.pop(keys[-1], None)
    else:
        d[keys[-1]] = value
    return old


ap = argparse.ArgumentParser()
ap.add_argument('--variant', default='v1', choices=['v1', 'v2', 'v3', 'v7', 'v10', 'v11'])
a = ap.parse_args()
out = copy.deepcopy(src)
rows = []
for path, value, why in (CHANGES + (V2 if a.variant in ('v2', 'v3', 'v7', 'v10', 'v11') else []) + (V3 if a.variant in ('v3', 'v7', 'v10', 'v11') else [])
                         + (V10 if a.variant == 'v10' else []) + (V11 if a.variant == 'v11' else [])):
    old = put(out, path, value)
    rows.append(f'| `{path}` | `{old if not isinstance(old, dict) else "…"}` | `{value if not isinstance(value, dict) else "…"}` | {why} |')
if a.variant in ('v2', 'v3', 'v7', 'v10', 'v11'):
    cs = out['controller_server']['ros__parameters']
    mppi = cs['FollowPath']
    cs['FollowPath'] = dict(SHIM, primary_controller=mppi)
    rows.append(f'| `controller_server.ros__parameters.FollowPath` | MPPIController | RotationShimController(primary_controller = 위 MPPI) | {SHIM_WHY} |')
    if a.variant in ('v3', 'v7', 'v10', 'v11'):
        for k, v in (V3_SHIM | (V7_SHIM if a.variant in ('v7', 'v10', 'v11') else {})).items():
            rows.append(f'| `controller_server.ros__parameters.FollowPath.{k}` | `{cs["FollowPath"][k]}` | `{v}` | {V7_SHIM_WHY if k in V7_SHIM else V3_SHIM_WHY} |')
            cs['FollowPath'][k] = v
# 이번에 띄우지 않는 서버 설정은 뺌(docking · route · 필터 마스크 · loopback)
for k in ('docking_server', 'route_server', 'keepout_filter_mask_server', 'keepout_costmap_filter_info_server', 'speed_filter_mask_server',
          'speed_costmap_filter_info_server', 'loopback_simulator', 'waypoint_follower'):
    out.pop(k, None)
for cm in ('local_costmap', 'global_costmap'):
    out[cm][cm]['ros__parameters'].pop('filters', None)
    for f in ('keepout_filter', 'speed_filter'):
        out[cm][cm]['ros__parameters'].pop(f, None)
out['map_objects_server'] = {'ros__parameters': {'topic_name': 'map_objects', 'frame_id': 'map'}}
class NoAlias(yaml.SafeDumper):
    def ignore_aliases(self, data):   # ROS params 파서는 YAML 별칭(&, *)을 못 읽음
        return True


head = '# 생성 파일(tools/make_nav2_params.py) — 직접 고치지 말고 CHANGES 를 고친 뒤 다시 만든다. 기본 = ROS 2 Lyrical nav2_bringup/params/nav2_params.yaml\n'
suffix = '' if a.variant == 'v1' else '_' + a.variant
(HERE / 'config' / f'nav2_params{suffix}.yaml').write_text(head + yaml.dump(out, Dumper=NoAlias, sort_keys=False, allow_unicode=True))
(HERE / 'config' / f'nav2_params{suffix}_changes.md').write_text(
    '# 기본 설정에서 바꾼 값\n\n기본 = ROS 2 Lyrical `nav2_bringup/params/nav2_params.yaml`(복사본 `nav2_params_default_lyrical.yaml`). '
    '이번에 띄우지 않는 서버(docking · route · 필터 · loopback · waypoint_follower)와 costmap 필터(keepout · speed)는 뺐다.\n\n'
    '| 항목 | 기본 | 바꾼 값 | 이유 |\n|---|---|---|---|\n' + '\n'.join(rows) + '\n')
print(len(rows), '개 바꿈')
