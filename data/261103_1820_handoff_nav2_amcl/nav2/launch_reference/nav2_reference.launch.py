# 11-03 사본(261031_1325_nav2/launch/nav2_mission_loc2.launch.py): localization:=external 을 더함 — 위치 추정 · 지도 · loc_monitor 를 이 launch 에서 띄우지 않음(팀 저장소 localization 패키지가 냄). 그 밖은 같음.
# 원본 머리말: 11-02 사본(nav2_mission.launch.py): 바꾼 것 = loc_monitor 에 loc_res_mode · loc_check_mode 인자 전달, 11-03 costmap_map 인자(기본값이면 원본과 같음)
"""저장 지도로 주행(Nav2): 차체 가림 마스크 → 스캔 합치기 → AMCL(map→odom) · EKF(odom→base_link) → planner · MPPI controller · BT → 속도 다듬기 → 충돌 감시 → /cmd_vel_nav.
map = 발제 좌표(map_mission.yaml = SLAM 지도를 도면에 맞춘 것). 벽 = SLAM 지도(/map), 라이다에 안 보이는 고정 물체 = 발제 배치 지도(/map_objects).
속도 명령 흐름: controller · behavior → /cmd_vel_ctrl → [turn_floor:=true 면 제자리 회전 바닥값 → /cmd_vel_turn_floor] → velocity_smoother → /cmd_vel_smoothed
→ collision_monitor → /cmd_vel_nav(TwistStamped, 팀 twist_mux nav 입력).
사용(PC · 로봇 공통, ROS 2 Lyrical 을 source 한 셸):
  ros2 launch <이 폴더>/launch/nav2_mission.launch.py [use_sim_time:=true] [use_ekf:=true] [use_rsp:=true urdf:=<urdf>]
목표: NavigateToPose 동작 /navigate_to_pose (frame map).
위치 추정 선택(설계 18, 2026-11-01): localization:=carto 면 AMCL · 스캔 합치기 대신 Cartographer 순수 위치 추정(저장 pbstream frozen,
nodes/carto_start.py 가 /start_trajectory 로 출발 자세 지정) → /tracked_pose → map_odom_relay(map→odom), 정적 TF mission_map → map(align.json).
이때 Nav2 global frame · 지도 frame 은 mission_map(params:=config/nav2_params_v18.yaml), 목표도 frame mission_map. 셸에 waffle-lyrical-slam install 을 source."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, EqualsSubstitution, LaunchConfiguration as LC, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue

HERE = Path('/home/pa4/git/lv2_next_module/261031_1325_nav2')   # 11-03 사본: 노드 · 설정은 원래 폴더 것을 씀
SLAM = Path('/home/pa4/git/lv2_next_module/261031_0335_slam_launch')
LOC = ['map_server', 'map_objects_server', 'amcl']
LOC_CARTO = ['map_server', 'map_objects_server']
LIVE_MAP = Path('/home/pa4/git/lv2_next_module/261101_0016_isaac_live/results/live_bright/map')
NAV = ['controller_server', 'smoother_server', 'planner_server', 'behavior_server', 'bt_navigator', 'velocity_smoother', 'collision_monitor']


def generate_launch_description():
    sim = {'use_sim_time': ParameterValue(LC('use_sim_time'), value_type=bool)}
    params = ParameterFile(LC('params'), allow_substs=True)   # 기본 YAML 의 $(find-pkg-share …) 를 풀기 위해
    args = [DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('params', default_value=str(HERE / 'config' / 'nav2_params_v11.yaml'), description='설계 11 에서 3회 확인한 설정(README)'),
            DeclareLaunchArgument('map', default_value=str(HERE / 'config' / 'map_mission_nav.yaml'), description='벽 지도(발제 좌표, 출발 자리 자기 차체 지운 복사본)'),
            DeclareLaunchArgument('objects_map', default_value=str(HERE / 'config' / 'map_objects_fine.yaml'), description='발제 고정 물체 지도(0.01 m, 덧대지 않음)'),
            DeclareLaunchArgument('mask_file', default_value='none', description='차체 고정 칸 yaml(없으면 출발 전 0.5~1.9 s 정지 스캔에서 학습)'),
            DeclareLaunchArgument('use_ekf', default_value='true', description='odom→base_link 를 낼 EKF(팀 bringup 이 내면 false)'),
            DeclareLaunchArgument('ekf_params', default_value=str(SLAM / 'config' / 'ekf_sim.yaml')),
            DeclareLaunchArgument('use_rsp', default_value='false', description='robot_state_publisher(팀 bringup 이 내면 false)'),
            DeclareLaunchArgument('urdf', default_value=''),
            DeclareLaunchArgument('turn_floor', default_value='false', description='제자리 회전 명령 바닥값 노드(nodes/turn_floor.py, 설계 13)'),
            DeclareLaunchArgument('turn_floor_w_min', default_value='0.35'),
            DeclareLaunchArgument('turn_floor_last', default_value='false',
                                  description='true 면 turn_floor 를 맨 끝(collision_monitor → /cmd_vel_cm → turn_floor → /cmd_vel_nav)에, params 의 collision_monitor 출력은 cmd_vel_cm(설계 21)'),
            DeclareLaunchArgument('icr_odom', default_value='false', description='회전 중심 보정 odom(nodes/icr_odom.py → /odom_icr, 설계 15). ekf_params 도 /odom_icr 를 받는 것으로'),
            DeclareLaunchArgument('x_icr', default_value='-0.0786'),
            DeclareLaunchArgument('rotation_center_tf', default_value='false',
                                  description='정적 TF base_link → rotation_center(x_icr, 0) — 설계 20, Nav2 제어점(params nav2_params_v20.yaml)'),
            DeclareLaunchArgument('loc_monitor', default_value='false',
                                  description='위치 잃음 감지 · 다시 찾기(nodes/loc_monitor.py, localization:=carto · turn_floor_last:=true 전제). turn_floor → /cmd_vel_gate_in → loc_monitor → /cmd_vel_nav'),
            DeclareLaunchArgument('loc_event_log', default_value='none', description='loc_monitor 사건 기록 jsonl'),
            DeclareLaunchArgument('loc_res_thr', default_value='0.03'),
            DeclareLaunchArgument('costmap_map', default_value=LC('map'), description='11-03: map_server(/map → 전역 costmap 정적 층)만 이 지도로. 기본 = map. 물체 지도와 같은 0.01 m 격자면 두 정적 층 도착 순서에 상관없이 같은 costmap'),
            DeclareLaunchArgument('loc_res_mode', default_value='cell_median', description='11-02: interp_p75 = 보간 · 75 백분위(문턱 0.034)'),
            DeclareLaunchArgument('loc_check_mode', default_value='front', description='11-02: pair = 앞 · 뒤 짝 검사(스캔 주기 가까이)'),
            DeclareLaunchArgument('yaw_loop', default_value='false', description='turn_floor 안 IMU yaw rate PI 되먹임(설계 23)'),
            DeclareLaunchArgument('turn_ff', default_value='false', description='turn_floor 안 제자리 회전 정적 앞먹임(설계 24, 바닥값 뒤 측정 곡선 역함수)'),
            DeclareLaunchArgument('move_ff', default_value='false', description='turn_floor 안 전진 중 회전 정적 앞먹임(설계 32, 전진 중 측정 곡선 역함수)'),
            DeclareLaunchArgument('move_ff_cmd', default_value='[0.15, 0.80]', description='전진 중 곡선: 명령 |w|(move_ff_act 와 같은 길이)'),
            DeclareLaunchArgument('move_ff_act', default_value='[0.0, 0.60]', description='전진 중 곡선: 실제 |w|'),
            DeclareLaunchArgument('turn_floor_out_frame', default_value='none', description='turn_floor 가 명령 header.frame_id 를 바꿀 이름(설계 20 base_link)'),
            DeclareLaunchArgument('localization', default_value='amcl', description='amcl | carto(설계 18)'),
            DeclareLaunchArgument('carto_config_dir', default_value=str(HERE / 'config')),
            DeclareLaunchArgument('carto_config', default_value='carto_localization_v18.lua'),
            DeclareLaunchArgument('carto_state', default_value=str(LIVE_MAP / 'map.pbstream'), description='1단계 실시간 지도(live_bright)'),
            DeclareLaunchArgument('carto_odom_topic', default_value='odom',
                                  description='Cartographer odometry 입력(lua 의 use_odometry true 일 때만 구독, 설계 39 후보는 /odometry/filtered)'),
            DeclareLaunchArgument('mission_x', default_value='0.729', description='mission_from_carto_map = live_bright/map/align.json'),
            DeclareLaunchArgument('mission_y', default_value='0.960'),
            DeclareLaunchArgument('mission_yaw', default_value='3.143774'),
            DeclareLaunchArgument('start_x', default_value='0.734', description='출발 자세(발제 좌표)'),
            DeclareLaunchArgument('start_y', default_value='0.950'),
            DeclareLaunchArgument('start_yaw', default_value='3.14159265'),
            DeclareLaunchArgument('future_dt', default_value='0.1', description='map_odom_relay 가 map→odom 을 앞당겨 낼 시간[s]'),
            DeclareLaunchArgument('python', default_value='/usr/bin/python3')]
    py = LC('python')
    ros_sim = ['--ros-args', '-p', ['use_sim_time:=', LC('use_sim_time')]]
    ctrl = [('cmd_vel', '/cmd_vel_ctrl')]
    carto = IfCondition(EqualsSubstitution(LC('localization'), 'carto'))
    amcl = IfCondition(EqualsSubstitution(LC('localization'), 'amcl'))
    own = UnlessCondition(EqualsSubstitution(LC('localization'), 'external'))

    def nav(pkg, exe, name=None, remap=()):
        return Node(package=pkg, executable=exe, name=name or exe, output='screen', parameters=[params, sim], remappings=list(remap))
    return LaunchDescription(args + [
        ExecuteProcess(cmd=[py, str(SLAM / 'nodes' / 'scan_body_mask.py')] + ros_sim + ['-p', ['mask_file:=', LC('mask_file')]],
                       name='scan_body_mask', output='screen', condition=own),
        ExecuteProcess(cmd=[py, str(HERE / 'nodes' / 'scan_merge.py')] + ros_sim, name='scan_merge', output='screen', condition=amcl),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[params, sim, {'yaml_filename': LC('costmap_map')}], condition=own),
        Node(package='nav2_map_server', executable='map_server', name='map_objects_server', output='screen',
             parameters=[params, sim, {'yaml_filename': LC('objects_map')}], condition=own),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen', parameters=[params, sim], condition=amcl),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization', output='screen',
             parameters=[sim, {'autostart': True, 'node_names': LOC}], condition=amcl),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization', output='screen',
             parameters=[sim, {'autostart': True, 'node_names': LOC_CARTO}], condition=carto),
        Node(package='cartographer_ros', executable='cartographer_node', name='cartographer_node', output='screen', parameters=[sim],
             arguments=['-configuration_directory', LC('carto_config_dir'), '-configuration_basename', LC('carto_config'),
                        '-load_state_filename', LC('carto_state'), '-load_frozen_state=true', '-start_trajectory_with_default_topics=false'],
             remappings=[('scan_1', '/scan_front_filtered'), ('scan_2', '/scan_rear_filtered'), ('odom', LC('carto_odom_topic'))], condition=carto),
        ExecuteProcess(cmd=[py, str(HERE / 'nodes' / 'carto_start.py')] + ros_sim + [
                           '-p', ['carto_config_dir:=', LC('carto_config_dir')], '-p', ['carto_config:=', LC('carto_config')],
                           '-p', ['align_x:=', LC('mission_x')], '-p', ['align_y:=', LC('mission_y')], '-p', ['align_yaw:=', LC('mission_yaw')],
                           '-p', ['start_x:=', LC('start_x')], '-p', ['start_y:=', LC('start_y')], '-p', ['start_yaw:=', LC('start_yaw')]],
                       name='carto_start', output='screen', condition=carto),
        ExecuteProcess(cmd=[py, str(SLAM / 'nodes' / 'map_odom_relay.py')] + ros_sim + ['-p', ['future_dt:=', LC('future_dt')]],
                       name='map_odom_relay', output='screen', condition=carto),
        Node(package='tf2_ros', executable='static_transform_publisher', name='mission_map_tf', output='screen', parameters=[sim], condition=carto,
             arguments=['--x', LC('mission_x'), '--y', LC('mission_y'), '--yaw', LC('mission_yaw'), '--frame-id', 'mission_map', '--child-frame-id', 'map']),
        nav('nav2_controller', 'controller_server', remap=ctrl),
        nav('nav2_smoother', 'smoother_server'),
        nav('nav2_planner', 'planner_server'),
        nav('nav2_behaviors', 'behavior_server', remap=ctrl),
        nav('nav2_bt_navigator', 'bt_navigator'),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother', output='screen', parameters=[params, sim],
             remappings=ctrl, condition=IfCondition(PythonExpression(["'", LC('turn_floor'), "' != 'true' or '", LC('turn_floor_last'), "' == 'true'"]))),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother', output='screen', parameters=[params, sim],
             remappings=[('cmd_vel', '/cmd_vel_turn_floor')], condition=IfCondition(PythonExpression(["'", LC('turn_floor'), "' == 'true' and '", LC('turn_floor_last'), "' != 'true'"]))),
        ExecuteProcess(cmd=[py, str(HERE / 'nodes' / 'turn_floor.py')] + ros_sim + ['-p', ['w_min:=', LC('turn_floor_w_min')], '-p', ['out_frame:=', LC('turn_floor_out_frame')], '-p', ['yaw_loop:=', LC('yaw_loop')], '-p', ['turn_ff:=', LC('turn_ff')], '-p', ['move_ff:=', LC('move_ff')], '-p', ['move_ff_cmd:=', LC('move_ff_cmd')], '-p', ['move_ff_act:=', LC('move_ff_act')]],
                       name='turn_floor', output='screen', condition=IfCondition(PythonExpression(["'", LC('turn_floor'), "' == 'true' and '", LC('turn_floor_last'), "' != 'true'"]))),
        ExecuteProcess(cmd=[py, str(HERE / 'nodes' / 'turn_floor.py')] + ros_sim + ['-p', ['w_min:=', LC('turn_floor_w_min')], '-p', ['out_frame:=', LC('turn_floor_out_frame')], '-p', ['yaw_loop:=', LC('yaw_loop')], '-p', ['turn_ff:=', LC('turn_ff')], '-p', ['move_ff:=', LC('move_ff')], '-p', ['move_ff_cmd:=', LC('move_ff_cmd')], '-p', ['move_ff_act:=', LC('move_ff_act')],
                            '-p', 'in_topic:=/cmd_vel_cm', '-p', 'out_topic:=/cmd_vel_nav'],
                       name='turn_floor', output='screen',
                       condition=IfCondition(PythonExpression(["'", LC('turn_floor'), "' == 'true' and '", LC('turn_floor_last'), "' == 'true' and '", LC('loc_monitor'), "' != 'true'"]))),
        ExecuteProcess(cmd=[py, str(HERE / 'nodes' / 'turn_floor.py')] + ros_sim + ['-p', ['w_min:=', LC('turn_floor_w_min')], '-p', ['out_frame:=', LC('turn_floor_out_frame')], '-p', ['yaw_loop:=', LC('yaw_loop')], '-p', ['turn_ff:=', LC('turn_ff')], '-p', ['move_ff:=', LC('move_ff')], '-p', ['move_ff_cmd:=', LC('move_ff_cmd')], '-p', ['move_ff_act:=', LC('move_ff_act')],
                            '-p', 'in_topic:=/cmd_vel_cm', '-p', 'out_topic:=/cmd_vel_gate_in'],
                       name='turn_floor', output='screen',
                       condition=IfCondition(PythonExpression(["'", LC('turn_floor'), "' == 'true' and '", LC('turn_floor_last'), "' == 'true' and '", LC('loc_monitor'), "' == 'true'"]))),
        ExecuteProcess(cmd=[py, str(HERE / 'nodes' / 'loc_monitor.py')] + ros_sim + [
                           '-p', ['map:=', LC('map')], '-p', ['carto_config_dir:=', LC('carto_config_dir')], '-p', ['carto_config:=', LC('carto_config')],
                           '-p', ['align_x:=', LC('mission_x')], '-p', ['align_y:=', LC('mission_y')], '-p', ['align_yaw:=', LC('mission_yaw')],
                           '-p', ['res_thr:=', LC('loc_res_thr')], '-p', ['res_mode:=', LC('loc_res_mode')], '-p', ['check_mode:=', LC('loc_check_mode')], '-p', ['event_log:=', LC('loc_event_log')],
                           '-p', 'gate_in:=/cmd_vel_gate_in', '-p', 'gate_out:=/cmd_vel_nav'],
                       name='loc_monitor', output='screen', condition=IfCondition(PythonExpression(["'", LC('loc_monitor'), "' == 'true' and '", LC('localization'), "' != 'external'"]))),
        ExecuteProcess(cmd=[py, str(HERE / 'nodes' / 'icr_odom.py')] + ros_sim + ['-p', ['x_icr:=', LC('x_icr')]],
                       name='icr_odom', output='screen', condition=IfCondition(LC('icr_odom'))),
        nav('nav2_collision_monitor', 'collision_monitor'),
        Node(package='tf2_ros', executable='static_transform_publisher', name='rotation_center_tf', output='screen', parameters=[sim],
             condition=IfCondition(LC('rotation_center_tf')),
             arguments=['--x', LC('x_icr'), '--frame-id', 'base_link', '--child-frame-id', 'rotation_center']),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation', output='screen',
             parameters=[sim, {'autostart': True, 'node_names': NAV}]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
             parameters=[LC('ekf_params'), sim], condition=IfCondition(LC('use_ekf'))),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', condition=IfCondition(LC('use_rsp')),
             parameters=[sim, {'robot_description': ParameterValue(Command(['cat ', LC('urdf')]), value_type=str)}]),
    ])
