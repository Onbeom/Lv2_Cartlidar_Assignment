#!/bin/bash
# 새 녹화 대조(PC, CPU Cartographer). 사용: run_all.sh <녹화 밝음> <녹화 어두움>
set -eo pipefail
source /opt/ros/lyrical/setup.bash
source /home/pa4/git/ros_test/waffle-lyrical-slam/install/setup.bash
set -u
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONPATH=/usr/lib/python3/dist-packages:${PYTHONPATH:-}
HERE=/home/pa4/git/lv2_next_module/261030_2058
PY=/home/pa4/isaacsim-venv/bin/python
mkdir -p $HERE/ros_logs $HERE/results
export ROS_LOG_DIR=$HERE/ros_logs
sha256sum $HERE/*.py $HERE/*.lua $HERE/run_all.sh > $HERE/results/source_sha256.txt
[ -d $HERE/results/bag_lidar_only_bright ] || $PY $HERE/prepare_bags.py "$1" bright
[ -d $HERE/results/bag_lidar_only_dark ] || $PY $HERE/prepare_bags.py "$2" dark
chain() {
  local light=$1 dom=$2
  ( export ROS_DOMAIN_ID=$dom
    exec 8>/tmp/lv2_ros_domain_$dom.lock; flock -w 60 8
    for kind in lidar_only wheel_odom truth_odom; do
      timeout --signal=INT --kill-after=30s 1500 $PY $HERE/replay.py $kind $light || echo "fail $kind $light $?"
    done ) > $HERE/results/chain_$light.log 2>&1
}
chain bright 109 & p1=$!
chain dark 110 & p2=$!
wait $p1; wait $p2
$PY $HERE/evaluate.py "$1" "$2" > $HERE/results/evaluate.log 2>&1
echo done
