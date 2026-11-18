"""B4 모의: 11-03 test_stack(localization 패키지 + Nav2 설계 45)에 dev bringup odom_icr 노드만 더함(팀 bringup 전체는 Isaac 브리지 · ros2_control 이 필요해 이 장면과 맞지 않음). EKF 는 ekf_params 로 dev ekf.yaml."""
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
T = '/home/pa4/git/lv2_next_module/261106_0946_b4_k0/team_src/workspace/ros2_ws/src'
def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource('/home/pa4/git/lv2_next_module/261103_1502_loc_pkg/launch/test_stack.launch.py')),
        ExecuteProcess(name='odom_icr', output='screen', cmd=['/usr/bin/python3', T + '/bringup/scripts/odom_icr.py', '--ros-args', '--params-file', T + '/control/config/robot_geometry.yaml', '-p', 'use_sim_time:=true']),
    ])
