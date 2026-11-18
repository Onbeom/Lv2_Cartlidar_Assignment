"""Twin Closed Maze B + TurtleBot3 Waffle Pi (SDF) Gazebo 실행.

waffle-lyrical-slam 저장소에서 빌드한 패키지(waffle_slam_course: 미로 월드·모델,
turtlebot3_gazebo: Waffle Pi SDF·브리지 설정)를 그대로 불러 쓴다. 저장소 파일은 고치지 않는다.

robot_state_publisher(URDF)는 쓰지 않는다. Gazebo DiffDrive가 odom→base_footprint를 내보내고,
나머지 고정 변환은 Waffle Pi model.sdf의 joint pose 값으로 static_transform_publisher가 내보낸다.

사용: ros2 launch ~/git/lv2_next_module/sim/gazebo_maze_waffle_pi.launch.py gui:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 미로 시작 위치 [m]: waffle_slam_course/models/twin_closed_maze/source.json의 spawn_xy_m
SPAWN_X, SPAWN_Y = '-1.835', '2.757'

# turtlebot3_waffle_pi/model.sdf의 고정 joint pose (부모, 자식, x, y, z) [m]
STATIC_TF = [
    ('base_footprint', 'base_link', 0.0, 0.0, 0.010),   # base_joint
    ('base_link', 'base_scan', -0.064, 0.0, 0.121),     # lidar_joint
    ('base_link', 'imu_link', -0.032, 0.0, 0.068),      # imu_joint
]


def generate_launch_description():
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    gz = get_package_share_directory('ros_gz_sim')
    course = get_package_share_directory('waffle_slam_course')

    def include(folder, file, arguments, condition=None):
        kwargs = {'condition': condition} if condition is not None else {}
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(folder, 'launch', file)),
            launch_arguments=arguments.items(), **kwargs)

    static_tf = [
        Node(package='tf2_ros', executable='static_transform_publisher', name=f'static_tf_{child}',
             parameters=[{'use_sim_time': True}],
             arguments=['--x', str(x), '--y', str(y), '--z', str(z),
                        '--frame-id', parent, '--child-frame-id', child])
        for parent, child, x, y, z in STATIC_TF
    ]

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('gz_partition', default_value='pa4_26',
                              description='Gazebo transport 파티션. 같은 네트워크의 다른 Gazebo와 섞이지 않게 한다'),
        SetEnvironmentVariable('GZ_PARTITION', LaunchConfiguration('gz_partition')),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi'),   # spawn_turtlebot3.launch.py가 이 값으로 SDF·브리지 설정을 고른다
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(tb3, 'models')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(course, 'models')),
        include(gz, 'gz_sim.launch.py', {
            'gz_args': ['-r -s --headless-rendering -v 2 ', os.path.join(course, 'worlds', 'twin_closed_maze.sdf')],
            'on_exit_shutdown': 'true'}),
        include(gz, 'gz_sim.launch.py', {'gz_args': '-g -v 2'}, IfCondition(LaunchConfiguration('gui'))),
        include(tb3, 'spawn_turtlebot3.launch.py', {'x_pose': SPAWN_X, 'y_pose': SPAWN_Y}),
        *static_tf,
    ])
