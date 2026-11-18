"""Jetson 부하 2단계(녹화 재생, 주행 없음): 위치 추정 묶음 = 팀 EKF(ekf.yaml) + localization 패키지(localization:=amcl, use_amcl_node:=true).
노드: ekf_filter_node · scan_body_mask · scan_merge · amcl(+ lifecycle) · map_server 2(+ lifecycle) · loc_monitor. 모두 use_sim_time(bag play --clock).
base_link → 라이다 정적 TF 는 재생 bag 의 /tf_static 이 낸다(robot_state_publisher 안 띄움).
사용: ros2 launch <이 파일> [loc_event_log:=<jsonl>]"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

HERE = Path(__file__).resolve().parent.parent


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('loc_event_log', default_value='none'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
             parameters=[str(HERE / 'config' / 'ekf.yaml'), {'use_sim_time': True}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('localization'), 'launch', 'localization.launch.py'])),
            launch_arguments={'use_sim_time': 'true', 'localization': 'amcl', 'use_amcl_node': 'true',
                              'loc_event_log': LC('loc_event_log')}.items()),
    ])
