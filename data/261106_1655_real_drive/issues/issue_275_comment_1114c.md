@kanichong 같은 날 다른 bringup(미로 주행용)에서 노드끼리 서로 못 찾는 듯한 일이 더 있어 함께 확인 부탁드립니다.

## 11-14 13:03 ~ 13:26 (실물 Jetson, 재부팅 뒤, `use_teleop:=true rc100:=true use_perception:=false use_localization:=true use_amcl_node:=true use_nav2:=false use_planner:=false use_loc_lock:=false use_loc_gate:=false`)
- spawner: 바퀴 spawner 가 `Could not contact service /controller_manager/list_controllers` 를 되풀이하며 잠금을 쥔 채 멈추고, 나머지 spawner 는 `Failed to acquire lock` 으로 포기. 같은 때 밖에서 `ros2 control list_controllers` 는 응답함(나중에는 15 s 동안 응답 없음도 한 번).
- rc100_joy: bringup 이 띄운 `rc100_joy` 가 `/rc100/raw`(17 Hz)를 받지 못해 `/joy` 를 한 번도 안 냄. 같은 노드를 따로 띄우니 바로 20 Hz 로 냄.
- **/tf 미녹화**: 같은 bringup 에서 `ros2 bag record` 로 9 분 녹화했는데 `/tf` 가 0 개(`/tf_static` 3 개, `/odom` · `/joint_states` · `/odometry/filtered` 는 정상 개수). robot_state_publisher · EKF · AMCL 이 /tf 를 내는 구성.
  - 녹화 명령: `ros2 bag record -s mcap -o ~/real_drive/maze_$(date +%m%d_%H%M) -e "^/(scan.*|odom.*|odometry/.*|imu/.*|tf|tf_static|joint_states|localization/.*|amcl_pose|joy|cmd_vel.*)$"` — 정규식 목록에 `/tf` 있음(-a 아님). 녹화기는 bringup 이 뜨고 약 14 분 뒤(13:17) 별도 터미널에서 시작.
  - 녹화 중 /tf 가 나왔다는 근거(간접): AMCL 이 13:05 에 `createLaserObject`(레이저 TF 를 찾아야 만듦) 뒤 `/amcl_pose` 를 녹화 동안 5567 개 냄, 그 전에는 `Message Filter dropping message … earlier than all the data in the transform cache` (TF 버퍼에 데이터가 있음). 녹화 중 `ros2 topic hz /tf` 는 재지 않음. → /tf 는 AMCL 에는 들어오는데 녹화기는 0 개 = 녹화기가 /tf 발행자를 못 찾은 것으로 봄(추정).
- 공통점으로 보이는 것: bringup 안의 일부 노드가 다른 노드의 서비스 · 토픽을 못 봄(밖의 CLI 는 봄). 원인은 확인 못 함.

## 부탁
- bringup 쪽에서 위 세 가지가 같은 원인인지 확인 부탁드립니다(발견 범위 `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` · 실행 순서 · 프로세스 수 등).
- 확인 방법(제안): 같은 인자로 bringup 뒤 60 s 에 `ros2 topic hz /tf` · `ros2 control list_controllers` · `ros2 topic hz /joy`(리모컨 버튼 누름) 를 10 번 반복해 성공 수.
- 기록: Jetson `~/.ros/log/2026-11-14-13-03-33-*`, bag `~/real_drive/maze_1114_1317`(metadata.yaml 에 토픽별 개수).
