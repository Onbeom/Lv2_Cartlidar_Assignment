#!/bin/bash
# TF 구성 B 재생(lidar_tfodom_scaled · raw × 밝음 · 어두움). PC, CPU, 도메인 111 · 112.
set -eo pipefail
set +u
source /opt/ros/lyrical/setup.bash
source /home/pa4/git/ros_test/waffle-lyrical-slam/install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONPATH=/usr/lib/python3/dist-packages:${PYTHONPATH:-}
HERE=/home/pa4/git/lv2_next_module/261030_2058
PY=/home/pa4/isaacsim-venv/bin/python
export ROS_LOG_DIR=$HERE/ros_logs
sha256sum $HERE/*.py $HERE/*.lua $HERE/run_tf_b.sh > $HERE/results/source_sha256_tf_b.txt
chain() {
  local light=$1 dom=$2
  ( export ROS_DOMAIN_ID=$dom
    exec 8>/tmp/lv2_ros_domain_$dom.lock; flock -w 60 8
    for kind in lidar_tfodom_scaled lidar_tfodom_raw; do
      timeout --signal=INT --kill-after=30s 1500 $PY $HERE/replay.py $kind $light || echo "fail $kind $light $?"
    done ) > $HERE/results/chain_tf_b_$light.log 2>&1
}
chain bright 111 & p1=$!
chain dark 112 & p2=$!
wait $p1; wait $p2
$PY $HERE/evaluate_runs.py summary_tf_b.json bright="$1" dark="$2" > $HERE/results/evaluate_tf_b.log 2>&1; echo "evaluate $?"
echo done
