#!/bin/bash
# 대조 재생(PC, CPU Cartographer만). 밝음·어두움을 다른 도메인에서 동시에, 각 도메인 잠금.
set -eo pipefail
source /opt/ros/lyrical/setup.bash
source /home/pa4/git/ros_test/waffle-lyrical-slam/install/setup.bash
set -u
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONPATH=/usr/lib/python3/dist-packages:${PYTHONPATH:-}
HERE=/home/pa4/git/lv2_next_module/261030_1820
PY=/home/pa4/isaacsim-venv/bin/python
mkdir -p $HERE/ros_logs
export ROS_LOG_DIR=$HERE/ros_logs
sha256sum $HERE/*.py $HERE/*.lua $HERE/run_controls.sh > $HERE/results_source_sha256.txt
[ -d $HERE/results/bag_lidar_only_bright ] || $PY $HERE/prepare_control_bags.py
chain() {
  local light=$1 dom=$2
  (
    export ROS_DOMAIN_ID=$dom
    exec 8>/tmp/lv2_ros_domain_$dom.lock
    flock -w 60 8
    for kind in lidar_only truth_odom; do
      timeout --signal=INT --kill-after=30s 900 $PY $HERE/replay_control.py $kind $light || echo "fail $kind $light $?"
    done
  ) > $HERE/results/chain_$light.log 2>&1
}
chain bright 106 & p1=$!
chain dark 107 & p2=$!
wait $p1; wait $p2
$PY $HERE/evaluate_controls.py > $HERE/results/evaluate.log 2>&1
echo done
