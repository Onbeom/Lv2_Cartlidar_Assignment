#!/usr/bin/env bash
set -eo pipefail
cd /home/pa4/git/lv2_next_module
source /opt/ros/lyrical/setup.bash
source /home/pa4/git/lv2_next_module/261029_1408/install/setup.bash
set -u
export ROS_DOMAIN_ID=95
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export CYCLONEDDS_URI=file:///home/pa4/.config/cyclonedds/cyclonedds_rx.xml
export ROS_LOG_DIR=/home/pa4/git/lv2_next_module/261029_1408/ros_logs
timeout --kill-after=10s 60s python3 /home/pa4/git/lv2_next_module/261029_1408/tfmini/uart_tool.py
