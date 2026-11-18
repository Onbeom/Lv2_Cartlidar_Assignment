#!/bin/bash
# 반복 재생(같은 bag, 비결정성 폭 확인) + 도구 오류 1회 재실행. PC, CPU Cartographer, 도메인 109/110.
set -eo pipefail
source /opt/ros/lyrical/setup.bash
source /home/pa4/git/ros_test/waffle-lyrical-slam/install/setup.bash
set -u
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONPATH=/usr/lib/python3/dist-packages:${PYTHONPATH:-}
HERE=/home/pa4/git/lv2_next_module/261030_2058
PY=/home/pa4/isaacsim-venv/bin/python
export ROS_LOG_DIR=$HERE/ros_logs
sha256sum $HERE/*.py $HERE/*.lua $HERE/run_repeat.sh > $HERE/results/source_sha256_repeat.txt
mv $HERE/results/carto_truth_odom_bright $HERE/results/carto_truth_odom_bright_TOOLFAIL_play_segv
chain() {
  local light=$1 dom=$2
  ( export ROS_DOMAIN_ID=$dom
    exec 8>/tmp/lv2_ros_domain_$dom.lock; flock -w 60 8
    if [ "$light" = bright ]; then REPLAY_TAG= timeout --signal=INT --kill-after=30s 1500 $PY $HERE/replay.py truth_odom bright || echo "fail truth_odom bright r1 $?"; fi
    for tag in _r2 _r3; do
      for kind in lidar_only truth_odom wheel_odom; do
        REPLAY_TAG=$tag timeout --signal=INT --kill-after=30s 1500 $PY $HERE/replay.py $kind $light || echo "fail $kind $light $tag $?"
      done
    done ) > $HERE/results/chain_repeat_$light.log 2>&1
}
chain bright 109 & p1=$!
chain dark 110 & p2=$!
wait $p1; wait $p2
$PY $HERE/evaluate.py "$1" "$2" > $HERE/results/evaluate_repeat.log 2>&1
echo done
