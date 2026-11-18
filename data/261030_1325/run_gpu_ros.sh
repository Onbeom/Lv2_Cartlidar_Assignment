#!/bin/bash
set -eo pipefail
source /opt/ros/lyrical/setup.bash
set -u
ROOT=/home/pa4/git/lv2_next_module
export ROS_DOMAIN_ID=102
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOG_DIR="$ROOT/261030_1325/ros_gpu_logs"
export PYTHONPATH=/usr/lib/python3/dist-packages:${PYTHONPATH:-}
export TRITON_CACHE_DIR="$ROOT/261030_1325/triton_cache"
exec 8>/tmp/lv2_ros_domain_102.lock
flock -w 60 8
exec 9>/tmp/lv2_gpu.lock
flock -w 60 9
timeout --signal=INT --kill-after=30s 180 /home/pa4/isaacsim-venv/bin/python "$ROOT/261030_1325/check_gpu_ros.py" > "$ROOT/Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325/filter_gpu_ros_v1.log" 2>&1
