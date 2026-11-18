#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/lyrical/setup.bash
set -u
export ROS_DOMAIN_ID=100
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/pa4/.config/cyclonedds/cyclonedds_rx.xml
export ROS_LOG_DIR=/home/pa4/git/lv2_next_module/261030_0004/ros_logs
export OPENBLAS_NUM_THREADS=1
GATE_RESULT_TAG=nav2_gate_fault_v2 timeout --kill-after=20s 600s python3 /home/pa4/git/lv2_next_module/261030_0004/gate_fault_check.py
