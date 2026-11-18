#!/usr/bin/env bash
set -eo pipefail
cd /home/pa4/git/lv2_next_module
source /opt/ros/lyrical/setup.bash
set -u
export ROS_DOMAIN_ID=94
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export CYCLONEDDS_URI=file:///home/pa4/.config/cyclonedds/cyclonedds_rx.xml
export ROS_LOG_DIR=/home/pa4/git/lv2_next_module/261029_2125/ros_logs
export NVBLOX_INPUT_MODE=isaac
flock /tmp/lv2_gpu.lock timeout --kill-after=10s 90s python3 /home/pa4/git/lv2_next_module/261029_2125/nvblox/tool_check.py
