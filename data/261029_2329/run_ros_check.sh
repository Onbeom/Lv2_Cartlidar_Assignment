#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/lyrical/setup.bash
set -u
export ROS_DOMAIN_ID=97
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/pa4/.config/cyclonedds/cyclonedds_rx.xml
export ROS_LOG_DIR=/home/pa4/git/lv2_next_module/261029_2329/ros_logs
export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH
export OPENBLAS_NUM_THREADS=1
timeout --kill-after=10s 90s /home/pa4/isaacsim-venv/bin/python /home/pa4/git/lv2_next_module/261029_2329/ros_check.py
