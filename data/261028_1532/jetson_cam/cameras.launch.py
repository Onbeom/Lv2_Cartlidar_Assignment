"""실물 카메라 3대 드라이버를 perception.yaml 이 기다리는 토픽 이름으로 띄운다.

ros2 launch perception cameras.launch.py                         # 앞 · 뒤 D435 + 팔 Arducam
ros2 launch perception cameras.launch.py use_rear:=false         # 일부만
ros2 launch perception cameras.launch.py arm_device:=/dev/video0 # udev(/dev/arm_cam) 규칙을 아직 안 넣었을 때

발행 토픽(perception.yaml cameras 절과 같음):
  /top_camera_front/color/image_raw · /top_camera_front/aligned_depth_to_color/image_raw · /top_camera_front/color/camera_info
  /top_camera_rear/...(같은 구조) · /arm_camera/image_raw · /arm_camera/camera_info
카메라 TF 는 URDF(robot_state_publisher)가 낸다 → realsense2_camera 의 publish_tf 는 끈다(같은 TF 를 둘이 내지 않게).
팔 카메라는 4:3 모드(640×480 · 320×240)에서 세로 줄무늬가 생겨(실물 확인 2026-10-28) 1280×720 MJPG 로 받는다.
usb_cam 0.8.1 은 이 카메라 MJPG 를 풀다 세그폴트로 죽어서 scripts/arm_camera_node.py(OpenCV V4L2)로 띄운다.
arm_script:=<경로> 를 주면 패키지를 빌드 · 설치하지 않은 컴퓨터에서도 그 파일을 python3 로 직접 실행한다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGS = {
    'use_front': ('true', '앞 D435 실행'),
    'use_rear': ('true', '뒤 D435 실행'),
    'use_arm': ('true', '팔 Arducam 실행'),
    'front_serial': ('261822072247', '앞 D435 시리얼 ★'),
    'rear_serial': ('261822079801', '뒤 D435 시리얼 ★'),
    'rs_profile': ('640,480,15', 'D435 컬러 · 깊이 해상도와 fps (가로,세로,fps)'),
    'rs_sync': ('false', 'D435 컬러 · 깊이를 한 묶음으로 맞춰 내기(enable_sync)'),
    'arm_device': ('/dev/arm_cam', '팔 카메라 장치 (udev/99-sensors.rules 의 이름)'),
    'arm_width': ('1280', '팔 카메라 가로'),
    'arm_height': ('720', '팔 카메라 세로'),
    'arm_fps': ('30', '팔 카메라 fps'),
    'arm_camera_info_url': ('', '팔 카메라 보정 파일 (비우면 보정 전 빈 camera_info)'),
    'arm_script': ('', '비우면 perception 의 arm_camera_node.py, 경로를 주면 그 파일을 직접 실행'),
}


def as_bool(text):
    return str(text).lower() in ('true', '1', 'yes')


def _setup(context):
    a = {k: context.launch_configurations[k] for k in ARGS}

    def realsense(name, serial):
        return Node(
            package='realsense2_camera', executable='realsense2_camera_node',
            namespace='', name=name, output='screen',
            parameters=[{
                'camera_name': name,
                'serial_no': ParameterValue(serial, value_type=str),   # 숫자로 읽히지 않게
                'rgb_camera.color_profile': a['rs_profile'],
                'depth_module.depth_profile': a['rs_profile'],
                'align_depth.enable': True,
                'enable_sync': as_bool(a['rs_sync']),
                'publish_tf': False,
                'enable_infra1': False,
                'enable_infra2': False,
            }])

    nodes = []
    if as_bool(a['use_front']):
        nodes.append(realsense('top_camera_front', a['front_serial']))
    if as_bool(a['use_rear']):
        nodes.append(realsense('top_camera_rear', a['rear_serial']))
    if as_bool(a['use_arm']):
        params = {
            'device': a['arm_device'],
            'width': int(a['arm_width']),
            'height': int(a['arm_height']),
            'fps': float(a['arm_fps']),
            'frame_id': 'camera_optical_frame',     # URDF 팔 카메라 광학 프레임
            'camera_info_url': a['arm_camera_info_url'],
        }
        if a['arm_script']:
            args = ['--ros-args', '-r', '__ns:=/arm_camera', '-r', '__node:=arm_camera']
            for k, v in params.items():
                args += ['-p', f'{k}:={v}'] if v != '' else []
            nodes.append(ExecuteProcess(cmd=['python3', a['arm_script']] + args, output='screen'))
        else:
            nodes.append(Node(
                package='perception', executable='arm_camera_node.py',
                namespace='arm_camera', name='arm_camera', output='screen', parameters=[params]))
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in ARGS.items()]
        + [OpaqueFunction(function=_setup)])
