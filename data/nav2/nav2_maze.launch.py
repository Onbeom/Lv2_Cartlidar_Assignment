"""Twin Closed Maze B + TurtleBot3 Waffle Pi(SDF) Nav2 실행 (Gazebo).

nav2_bringup 1.5.0의 bringup_launch.py·rviz_launch.py를 불러 쓰고, 설정 파일·지도·위치 추정 방식만 인자로 넘긴다.
Gazebo는 sim/gazebo_maze_waffle_pi.launch.py로 따로 띄운다.

사용: ros2 launch ~/git/lv2_next_module/nav2/nav2_maze.launch.py map:=<지도 yaml 경로>
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    bringup = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    localization = LaunchConfiguration('localization')

    def include(file, arguments, condition=None):
        kwargs = {'condition': condition} if condition is not None else {}
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, file)),
            launch_arguments=arguments.items(), **kwargs)

    return LaunchDescription([
        DeclareLaunchArgument('map', description='지도 yaml 경로(필수)'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(HERE, 'nav2_params_waffle_pi.yaml')),
        DeclareLaunchArgument('localization', default_value='amcl', choices=['amcl'],
                              description='위치 추정 방식. amcl이면 nav2_bringup이 map_server·AMCL을 띄운다'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        include('bringup_launch.py', {
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'true',
            'use_localization': PythonExpression(["'", localization, "' == 'amcl'"]),
            # keepout·speed 영역 마스크 지도가 없으므로 끈다(기본값 True면 빈 마스크 경로로 서버가 뜬다)
            'use_keepout_zones': 'false',
            'use_speed_zones': 'false',
        }),
        include('rviz_launch.py', {'use_sim_time': 'true'}, IfCondition(LaunchConfiguration('rviz'))),
    ])
