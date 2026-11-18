"""loc_cpp 비교 (PC, 11-16): 같은 재생 입력 · 같은 AMCL 에 Python 판(원래 토픽)과 C++ 판(/cpp/… 로 이름 바꿈)을 함께 띄운다.
C++ 판의 /initialpose 도 /cpp/initialpose 로 돌려 AMCL 에 닿지 않게 함 (다시 찾기는 Python 판만 실제로 함)."""
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, SetRemap
from launch_ros.substitutions import FindPackageShare

LOC = PathJoinSubstitution([FindPackageShare('localization'), 'launch', 'localization.launch.py'])
REMAP = ['/scan_front_filtered', '/scan_rear_filtered', '/scan_merged', '/localization/status', '/localization/lock', '/initialpose']


def generate_launch_description():
    py = IncludeLaunchDescription(PythonLaunchDescriptionSource(LOC), launch_arguments={
        'use_sim_time': 'true', 'use_amcl_node': 'true', 'loc_cpp': 'false'}.items())
    cpp = GroupAction([PushRosNamespace('cpp')] + [SetRemap(src=t, dst='/cpp' + t) for t in REMAP] + [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(LOC), launch_arguments={
            'use_sim_time': 'true', 'use_amcl_node': 'false', 'use_map_server': 'false', 'loc_cpp': 'true'}.items())])
    return LaunchDescription([py, cpp])
