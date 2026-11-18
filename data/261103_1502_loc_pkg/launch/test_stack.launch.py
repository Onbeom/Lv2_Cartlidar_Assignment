"""모의 확인용 묶음(11-03): 팀 저장소 localization 패키지(localization.launch.py, localization:=<loc>) + Nav2 설계 45(nav2_only.launch.py localization:=external).
명령줄 인자는 두 launch 가 함께 받는다. 다른 것만 그룹 안에서 덮어씀: Nav2 쪽 localization=external, 패키지 쪽 localization=<loc>, gate_in=/cmd_vel_gate_in.
팀 EKF(bringup/config/ekf.yaml)는 nav2_only 의 use_ekf · ekf_params 로 띄운다(팀 bringup 전체는 Isaac 브리지 · ros2_control 이 필요해 이 장면과 맞지 않음)."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

HERE = Path(__file__).resolve().parent


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('loc', default_value='carto', description='carto | amcl'),
        GroupAction(scoped=True, actions=[
            SetLaunchConfiguration('localization', LC('loc')),
            SetLaunchConfiguration('gate_in', '/cmd_vel_gate_in'),
            SetLaunchConfiguration('gate_out', '/cmd_vel_nav'),
            SetLaunchConfiguration('use_amcl_node', 'true'),   # 18:2x 패키지 기본이 AMCL 노드를 안 띄움(주행 쪽이 띄움) → 모의 확인은 여기서 띄움
            IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('localization'), 'launch', 'localization.launch.py'])))]),
        GroupAction(scoped=True, actions=[
            SetLaunchConfiguration('localization', 'external'),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(HERE / 'nav2_only.launch.py')))]),
    ])
