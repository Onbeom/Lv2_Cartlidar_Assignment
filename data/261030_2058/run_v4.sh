#!/bin/bash
# V4(use_pose_extrapolator false) 재생: lidar_tfodom_scaled · lidar_only × 밝음 · 어두움. PC, CPU, 도메인 111 · 112.
set -eo pipefail
set +u
source /opt/ros/lyrical/setup.bash
source /home/pa4/git/ros_test/waffle-lyrical-slam/install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONPATH=/usr/lib/python3/dist-packages:${PYTHONPATH:-}
HERE=/home/pa4/git/lv2_next_module/261030_2058
PY=/home/pa4/isaacsim-venv/bin/python
export ROS_LOG_DIR=$HERE/ros_logs
sha256sum $HERE/*.py $HERE/*.lua $HERE/variants/*.lua $HERE/run_v4.sh > $HERE/results/source_sha256_v4.txt
chain() {
  local light=$1 dom=$2
  ( export ROS_DOMAIN_ID=$dom
    exec 8>/tmp/lv2_ros_domain_$dom.lock; flock -w 60 8
    for kind in lidar_tfodom_scaled lidar_only; do
      VARIANT_LUA=$HERE/variants/V4_no_pose_extrapolator.lua REPLAY_TAG=_V4_no_pose_extrapolator timeout --signal=INT --kill-after=30s 1500 $PY $HERE/replay.py $kind $light || echo "fail $kind $light $?"
    done ) > $HERE/results/chain_v4_$light.log 2>&1
}
chain bright 111 & p1=$!
chain dark 112 & p2=$!
wait $p1; wait $p2
$PY $HERE/evaluate_runs.py summary_v4.json bright="$1" dark="$2" > $HERE/results/evaluate_v4.log 2>&1; echo "evaluate $?"
echo done
