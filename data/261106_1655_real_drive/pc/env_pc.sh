# PC 에서 P5 (신호등) 를 돌릴 때: ROS + 팀 dev 빌드(작업 폴더 ~/git/Lv2_Cartlidar_Assignment_lab), 통신은 PC 안으로만
source /opt/ros/lyrical/setup.bash
source ~/git/Lv2_Cartlidar_Assignment_lab/workspace/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=26
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # 같은 네트워크의 Jetson · 다른 사람 ROS 와 섞이지 않게
