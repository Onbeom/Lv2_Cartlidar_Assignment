#!/bin/bash
# 회전 배수 보정 바퀴 odom 재생(c6 · V3 × 밝음 · 어두움). PC, CPU, 도메인 111 · 112.
set -eo pipefail
source /opt/ros/lyrical/setup.bash
source /home/pa4/git/ros_test/waffle-lyrical-slam/install/setup.bash
set -u
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONPATH=/usr/lib/python3/dist-packages:${PYTHONPATH:-}
HERE=/home/pa4/git/lv2_next_module/261030_2058
PY=/home/pa4/isaacsim-venv/bin/python
export ROS_LOG_DIR=$HERE/ros_logs
sha256sum $HERE/*.py $HERE/*.lua $HERE/variants/*.lua $HERE/run_wheel_scaled.sh > $HERE/results/source_sha256_wheel_scaled.txt
chain() {
  local light=$1 dom=$2
  ( export ROS_DOMAIN_ID=$dom
    exec 8>/tmp/lv2_ros_domain_$dom.lock; flock -w 60 8
    timeout --signal=INT --kill-after=30s 1500 $PY $HERE/replay.py wheel_odom_scaled $light || echo "fail c6 $light $?"
    VARIANT_LUA=$HERE/variants/V3_ceres_rot400.lua REPLAY_TAG=_V3_ceres_rot400 timeout --signal=INT --kill-after=30s 1500 $PY $HERE/replay.py wheel_odom_scaled $light || echo "fail V3 $light $?"
  ) > $HERE/results/chain_wheel_scaled_$light.log 2>&1
}
chain bright 111 & p1=$!
chain dark 112 & p2=$!
wait $p1; wait $p2
$PY $HERE/evaluate_runs.py summary_wheel_scaled.json bright="$1" dark="$2" > $HERE/results/evaluate_wheel_scaled.log 2>&1
echo done
