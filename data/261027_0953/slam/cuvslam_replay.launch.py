"""cuVSLAM(Isaac ROS 5.0) — SLAM 비교 기록 bag 재생용. 앞 RealSense 적외선 좌·우(실물 D435 형식 mono8 640×480), IMU 없음.

TF: cuVSLAM이 map → odom → base_footprint를 모두 낸다(계획 문서 구성). bag의 /tf(바퀴 오도메트리)는 재생 때 다른 이름으로 바꿔 넣는다.
기준 프레임·카메라 프레임은 기록 때 URDF(robot_state_publisher가 낸 /tf_static, bag에 들어 있음).
변형(환경 값 CUVSLAM_VARIANT, 사용자 승인 2026-10-27):
  V0 기본 · V1 아래쪽 220 px 가림(접은 팔이 앞 IR 영상 아래 가운데 행 약 265~480 을 가림, photo/261027_SLAM_앞IR_팔가림.png)
     · V2 = V1 + 평면 주행 제약(enable_ground_constraint_in_odometry/slam).
  가림은 run_cuvslam.sh 가 mask_relay.py 로 영상에 칠해 넣는다. img_mask_bottom 은 값이 들어가도(param get 220) 결과가 V0 과
  똑같아 쓰지 않는다(도구 확인 2026-10-27 21:2x).
매개변수는 Isaac ROS 예제(isaac_ros_visual_slam_realsense.launch.py)에서 IMU 관련을 뺀 것이다.
"""
import os

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


VARIANTS = {
    "V0": {},
    "V1": {},
    "V2": {"enable_ground_constraint_in_odometry": True,
           "enable_ground_constraint_in_slam": True},
}


def generate_launch_description():
    extra = VARIANTS[os.environ.get("CUVSLAM_VARIANT", "V0")]
    vslam = ComposableNode(
        name="visual_slam_node",
        package="isaac_ros_cuvslam",
        plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",
        remappings=[("visual_slam/image_0", "/top_camera_front/infra1/image_rect_raw"),
                    ("visual_slam/camera_info_0", "/top_camera_front/infra1/camera_info"),
                    ("visual_slam/image_1", "/top_camera_front/infra2/image_rect_raw"),
                    ("visual_slam/camera_info_1", "/top_camera_front/infra2/camera_info")],
        parameters=[{
            "use_sim_time": True,
            "enable_image_denoising": False,
            "rectified_images": True,
            "image_jitter_threshold_ms": 35.0,   # 30 Hz 영상(33.3 ms 간격). 기본 19 는 60 Hz 기준이라 매 장 경고(도구 확인 T4)
            "base_frame": "base_footprint",
            "odom_frame": "odom",
            "map_frame": "map",
            "publish_odom_to_base_tf": True,
            "publish_map_to_odom_tf": True,
            "enable_slam_visualization": False,
            "enable_landmarks_view": False,
            "enable_observations_view": False,
            "camera_optical_frames": ["top_camera_front_infra1_optical_frame",
                                      "top_camera_front_infra2_optical_frame"],
            **extra,
        }])
    container = ComposableNodeContainer(
        name="visual_slam_launch_container", namespace="", package="rclcpp_components",
        executable="component_container", composable_node_descriptions=[vslam], output="screen")
    return launch.LaunchDescription([container])
