#!/usr/bin/env bash
set -euo pipefail
cd /home/pa4/git/lv2_next_module
set +u
source /opt/ros/lyrical/setup.bash
source 261028_1440/perception_build_mission/install/setup.bash
source 261028_1459/install/setup.bash
set -u
export ROS_DOMAIN_ID=89 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
OUT=/home/pa4/git/lv2_next_module/Cartlidar/sim_render/results/puck_survey_261028/ros
mkdir -p "$OUT"
ros2 run perception_survey puck_survey --ros-args -p output_path:="$OUT/puck_survey.json" > "$OUT/node.log" 2>&1 &
SURVEY_PID=$!
trap 'kill -INT "$SURVEY_PID" 2>/dev/null || true; wait "$SURVEY_PID" 2>/dev/null || true' EXIT
python3 261028_1459/test_survey_ros.py > "$OUT/test.log" 2>&1
