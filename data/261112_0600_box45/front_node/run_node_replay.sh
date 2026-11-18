#!/bin/bash
# 사용: ./run_node_replay.sh <렌더 폴더> <이름> [--every N]  — 인지 노드를 ROS_DOMAIN_ID 77 에서 띄우고 재생 시험, 끝나면 노드 끔
# (set -u 는 ROS setup.bash 와 맞지 않아 뺌)
cd "$(dirname "$0")"
source /opt/ros/lyrical/setup.bash
source ~/git/Lv2_Cartlidar_Assignment_pbfront/workspace/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=78
ros2 run perception perception --ros-args --params-file ~/git/Lv2_Cartlidar_Assignment_pbfront/workspace/ros2_ws/install/perception/share/perception/config/perception.yaml $NODE_ARGS > "node_$2.log" 2>&1 &
NPID=$!
sleep 4
timeout ${TOTAL_TIMEOUT:-1800} python3 run_front_replay.py "$1" "${@:3}" > "replay_$2.log" 2>&1
kill $NPID
tail -1 "replay_$2.log"
