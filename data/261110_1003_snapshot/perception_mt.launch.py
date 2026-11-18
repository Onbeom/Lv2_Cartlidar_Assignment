"""시험용: perception_node 를 실물과 같이 component_container_mt 에 띄움. remap:=true 면 mock_robot 과 겹치는 이름을 /real_perception/* 로"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def nodes(context):
    rp = '/real_perception'
    names = ['/perception/set_mode', '/perception/check_dice', '/perception/result', '/perception/pucks', '/perception/task_marker',
             '/perception/signal', '/perception/barcode', '/perception/place_box', '/perception/place_box/cells',
             '/perception/place_box/status', '/perception/place_box/measure', '/arm_range', '/perception/arm_target',
             '/perception/arm_range']
    remaps = [(n, rp + n.replace('/perception', '')) for n in names] if LaunchConfiguration('remap').perform(context) == 'true' else []
    cfg = os.path.join(get_package_share_directory('perception'), 'config', 'perception.yaml')
    return [ComposableNodeContainer(
        name='perception_container', namespace='', package='rclcpp_components', executable='component_container_mt', output='screen',
        composable_node_descriptions=[ComposableNode(package='perception', plugin='perception::PerceptionNode', name='perception_node',
                                                     parameters=[cfg, {'image_qos': 'reliable'}], remappings=remaps)])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('remap', default_value='false'), OpaqueFunction(function=nodes)])
