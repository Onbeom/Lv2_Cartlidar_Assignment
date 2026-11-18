"""외부 지도/TF/보정된 깊이를 받는 PC 격리 도메인99 연결 시제품."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess,SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    here=Path(__file__).resolve().parent
    actions=[SetEnvironmentVariable('ROS_DOMAIN_ID','99'),SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE','LOCALHOST')]
    for package,executable in [('nav2_planner','planner_server'),('nav2_controller','controller_server')]:
        actions.append(Node(package=package,executable=executable,name=executable,parameters=[str(here/'nav2.yaml')],remappings=[('cmd_vel','/cmd_vel_nav_raw')],output='screen'))
    actions.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='navigation_lifecycle_manager',parameters=[{'autostart':True,'node_names':['planner_server','controller_server'],'bond_timeout':4.}],output='screen'))
    actions.append(ExecuteProcess(cmd=['flock','/tmp/lv2_gpu.lock','timeout','--kill-after=20s','1800s','/opt/ros/lyrical/lib/nvblox_ros/nvblox_node','--ros-args','--params-file',str(here/'dynamic_v2.yaml'),'-r','__node:=nvblox_probe','-p','global_frame:=map','-p','map_clearing_frame_id:=base_link','-p','esdf_slice_bounds_visualization_attachment_frame_id:=map','-r','camera_0/depth/image:=/self_filter/output','-r','camera_0/depth/camera_info:=/self_filter/camera_info'],output='screen'))
    for kind in ['gate','coordinator','monitor']:
        arguments=['python3',str(here/'run_node.py'),kind]
        if kind=='monitor':arguments+=['--ros-args','-r','/dynamic_test/depth:=/self_filter/output','-r','/dynamic_test/camera_info:=/self_filter/camera_info']
        actions.append(ExecuteProcess(cmd=arguments,output='screen'))
    return LaunchDescription(actions)
