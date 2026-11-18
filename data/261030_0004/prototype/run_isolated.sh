#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/lyrical/setup.bash
set -u
prototype_dir="$(dirname "$(realpath "${BASH_SOURCE[0]}")")"
export ROS_DOMAIN_ID=99
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOG_DIR="$prototype_dir/logs"
exec ros2 launch "$prototype_dir/navigation.launch.py"
