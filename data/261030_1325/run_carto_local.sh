#!/bin/bash
set -eo pipefail
source /opt/ros/lyrical/setup.bash
source /home/pa4/git/ros_test/waffle-lyrical-slam/install/setup.bash
set -u
export ROS_DOMAIN_ID=101
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONPATH=/usr/lib/python3/dist-packages:${PYTHONPATH:-}
export ROS_LOG_DIR=/home/pa4/git/lv2_next_module/261030_1325/ros_logs
ROOT=/home/pa4/git/lv2_next_module
exec 8>/tmp/lv2_ros_domain_101.lock
flock -w 3600 8
for light in bright dark; do
  for rotation in 4 1; do
    timeout --signal=INT --kill-after=30s 900 /home/pa4/isaacsim-venv/bin/python "$ROOT/261030_1325/replay_carto_local.py" "$light" 0 "$rotation"
  done
done
