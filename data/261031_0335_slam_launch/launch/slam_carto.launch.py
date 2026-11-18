"""SLAM 후보 실시간 launch(지도 만들기): 차체 가림 마스크 → Cartographer(라이다 2개만, /tracked_pose) → map→odom 중계, EKF(odom→base_link),
점유 지도(/map), (선택) robot_state_publisher. TF: map→odom = 중계(지금 + future_dt), odom→base_link = EKF, 나머지 = rsp(정적),
mission_map→map = 정적(발제 좌표, 기본은 출발 자세 공칭값, 지도 저장 뒤엔 align.json 값). 저장 지도로 AMCL 을 돌릴 땐 map 이 곧 발제 좌표라 이 TF 를 쓰지 않음.
사용(PC · 로봇 공통, ROS 2 Lyrical + waffle-lyrical-slam install 을 source 한 셸):
  ros2 launch <이 폴더>/launch/slam_carto.launch.py [use_sim_time:=true] [use_ekf:=true ekf_params:=<yaml>] [use_rsp:=true urdf:=<urdf>]
지도 저장 · 도면 맞춤: python3 <이 폴더>/tools/save_and_align_map.py --out <폴더>"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

HERE = Path(__file__).resolve().parent.parent


def generate_launch_description():
    sim = {'use_sim_time': ParameterValue(LC('use_sim_time'), value_type=bool)}
    args = [DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('use_ekf', default_value='true', description='odom→base_link 를 낼 EKF(팀 bringup 이 내면 false)'),
            DeclareLaunchArgument('ekf_params', default_value=str(HERE / 'config' / 'ekf_sim.yaml')),
            DeclareLaunchArgument('use_rsp', default_value='false', description='robot_state_publisher(팀 bringup 이 내면 false)'),
            DeclareLaunchArgument('urdf', default_value=''),
            DeclareLaunchArgument('carto_config_dir', default_value=str(HERE / 'config')),
            DeclareLaunchArgument('carto_config', default_value='carto_candidate.lua'),
            DeclareLaunchArgument('map_resolution', default_value='0.025'),
            DeclareLaunchArgument('future_dt', default_value='0.1', description='map→odom 을 앞당겨 낼 시간[s]'),
            DeclareLaunchArgument('mask_file', default_value='none', description='차체 고정 칸 yaml(없으면 출발 전 0.5~1.9 s 정지 스캔에서 학습)'),
            DeclareLaunchArgument('mask_save', default_value='none', description='학습한 고정 칸을 저장할 yaml'),
            DeclareLaunchArgument('use_mission_tf', default_value='true',
                                  description='발제 좌표 frame → Cartographer map 정적 TF(사용자 결정 2026-11-01, frame 이름 mission_map)'),
            DeclareLaunchArgument('mission_frame', default_value='mission_map'),
            DeclareLaunchArgument('mission_x', default_value='0.734', description='지도 저장 뒤엔 align.json mission_from_carto_map 값'),
            DeclareLaunchArgument('mission_y', default_value='0.950'),
            DeclareLaunchArgument('mission_yaw', default_value='3.14159265'),
            DeclareLaunchArgument('python', default_value='/usr/bin/python3')]
    py = LC('python')
    ros_sim = ['--ros-args', '-p', ['use_sim_time:=', LC('use_sim_time')]]
    return LaunchDescription(args + [
        ExecuteProcess(cmd=[py, str(HERE / 'nodes' / 'scan_body_mask.py')] + ros_sim + ['-p', ['mask_file:=', LC('mask_file')], '-p', ['mask_save:=', LC('mask_save')]],
                       name='scan_body_mask', output='screen'),
        Node(package='cartographer_ros', executable='cartographer_node', name='cartographer_node', output='screen', parameters=[sim],
             arguments=['-configuration_directory', LC('carto_config_dir'), '-configuration_basename', LC('carto_config')],
             remappings=[('scan_1', '/scan_front_filtered'), ('scan_2', '/scan_rear_filtered')]),
        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node', name='cartographer_occupancy_grid_node',
             output='screen', parameters=[sim], arguments=['-resolution', LC('map_resolution'), '-publish_period_sec', '1.0']),
        ExecuteProcess(cmd=[py, str(HERE / 'nodes' / 'map_odom_relay.py')] + ros_sim + ['-p', ['future_dt:=', LC('future_dt')]],
                       name='map_odom_relay', output='screen'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
             parameters=[LC('ekf_params'), sim], condition=IfCondition(LC('use_ekf'))),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', condition=IfCondition(LC('use_rsp')),
             parameters=[sim, {'robot_description': ParameterValue(Command(['cat ', LC('urdf')]), value_type=str)}]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='mission_map_tf', output='screen',
             parameters=[sim], condition=IfCondition(LC('use_mission_tf')),
             arguments=['--x', LC('mission_x'), '--y', LC('mission_y'), '--yaw', LC('mission_yaw'),
                        '--frame-id', LC('mission_frame'), '--child-frame-id', 'map']),
    ])
