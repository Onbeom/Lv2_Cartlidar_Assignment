#!/usr/bin/env bash
set -eo pipefail
ROOT=/home/pa4/git/lv2_next_module
export ROS_DOMAIN_ID=121 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOG_DIR=$ROOT/Cartlidar/sim_render/results/integration_261030_1319/probe_full_v2/ros_logs
exec 9>/tmp/lv2_ros121.lock
flock 9
source /opt/ros/lyrical/setup.bash
source "$ROOT/261030_0701/install/setup.bash"
exec timeout --signal=TERM --kill-after=20s 1800s python3 "$ROOT/261030_1319/probe_full_v2.py"
