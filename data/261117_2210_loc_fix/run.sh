#!/usr/bin/env bash
# 고친 loc_cpp 확인 (PC). A: 그대로 재생 + 120 s 에 수동 /initialpose, B: 앞 스캔 빼고 60 s. 따로 도메인 · LOCALHOST.
set -u
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG=/home/pa4/git/lv2_next_module/261116_0002_loccpu_replay/bag_replay
PQ=/home/pa4/git/lv2_next_module/261116_0002_loccpu_replay/play_qos.yaml
W=/home/pa4/git/Lv2_Cartlidar_Assignment/workspace/ros2_ws/install
E="env -i HOME=$HOME USER=$USER PATH=/usr/local/bin:/usr/bin:/bin LANG=en_US.UTF-8 ROS_DOMAIN_ID=91 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"
S="source /opt/ros/lyrical/setup.bash && source $W/setup.bash"
one() {  # $1 이름, $2 재생 인자, $3 수동 initialpose 보낼 재생 초 (0 = 안 보냄)
  local O="$D/out_$1"; rm -rf "$O"; mkdir -p "$O"
  $E bash -c "$S && exec ros2 launch localization localization.launch.py use_sim_time:=true use_amcl_node:=true loc_cpp:=true" > "$O/launch.log" 2>&1 &
  local L=$!; sleep 12
  $E bash -c "$S && exec ros2 topic hz /scan_merged --use-sim-time --window 50" > "$O/hz.log" 2>&1 &
  local H=$!
  $E bash -c "$S && exec ros2 bag play $BAG --clock 100 --disable-keyboard-controls --qos-profile-overrides-path $PQ $2" < /dev/null > "$O/play.log" 2>&1 &
  local P=$!
  if [ "$3" != 0 ]; then
    sleep "$3"
    timeout 30 $E bash -c "$S && ros2 topic pub --once -w 1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {frame_id: map}, pose: {pose: {position: {x: 0.694, y: 0.95}, orientation: {z: 1.0, w: 0.0}}, covariance: [0.0025,0,0,0,0,0, 0,0.0025,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0076]}}'" > "$O/manual_pose.log" 2>&1
    date +%s.%N > "$O/manual_pose_wall.txt"
  fi
  wait $P
  sleep 3
  kill -TERM $H 2>/dev/null; kill -TERM $L; sleep 6
  pkill -TERM -f "Lv2_Cartlidar_Assignment/workspace/ros2_ws/install|nav2_map_server/map_server|nav2_amcl/amcl" 2>/dev/null
  echo "$1 끝 $(date +%T)"
}
one A "" 120
one B "--playback-duration 60 --topics /tf /tf_static /scan_rear /odom /imu/data" 0
