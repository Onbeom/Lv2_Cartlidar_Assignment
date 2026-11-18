"""RL1 목표 표용 MoveIt 충돌 검사(화면 없음).

sim_ws/src/waffle_4wd_moveit_config/launch/collision_check.launch.py의 복사본에서 RViz만 뺐다(원본은 그대로).
move_group의 /check_state_validity를 make_targets.py가 부른다.

사용: ros2 launch <이 파일 경로>
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder('waffle_4wd', package_name='waffle_4wd_moveit_config')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[moveit_config.robot_description], output='screen'),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[moveit_config.robot_description], output='screen'),
        Node(package='moveit_ros_move_group', executable='move_group',
             parameters=[moveit_config.to_dict(), {'allow_trajectory_execution': False}], output='screen'),
    ])
