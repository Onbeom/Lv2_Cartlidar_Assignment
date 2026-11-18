#!/bin/bash
# Cartographer 설정 복사본 시험(V1~V3 × lidar_only · truth_odom × 밝음 · 어두움). PC, CPU, 도메인 111~114 (4줄 동시).
set -eo pipefail
source /opt/ros/lyrical/setup.bash
source /home/pa4/git/ros_test/waffle-lyrical-slam/install/setup.bash
set -u
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONPATH=/usr/lib/python3/dist-packages:${PYTHONPATH:-}
HERE=/home/pa4/git/lv2_next_module/261030_2058
PY=/home/pa4/isaacsim-venv/bin/python
export ROS_LOG_DIR=$HERE/ros_logs
sha256sum $HERE/*.py $HERE/*.lua $HERE/variants/*.lua $HERE/run_variants.sh > $HERE/results/source_sha256_variants.txt
chain() {
  local kind=$1 light=$2 dom=$3
  ( export ROS_DOMAIN_ID=$dom
    exec 8>/tmp/lv2_ros_domain_$dom.lock; flock -w 60 8
    for v in V1_csm_cost V2_no_csm V3_ceres_rot400; do
      VARIANT_LUA=$HERE/variants/$v.lua REPLAY_TAG=_$v timeout --signal=INT --kill-after=30s 1500 $PY $HERE/replay.py $kind $light || echo "fail $kind $light $v $?"
    done ) > $HERE/results/chain_variants_${kind}_${light}.log 2>&1
}
chain lidar_only bright 111 & p1=$!
chain lidar_only dark 112 & p2=$!
chain truth_odom bright 113 & p3=$!
chain truth_odom dark 114 & p4=$!
wait $p1; wait $p2; wait $p3; wait $p4
$PY $HERE/evaluate_variants.py "$1" "$2" > $HERE/results/evaluate_variants.log 2>&1
echo done
