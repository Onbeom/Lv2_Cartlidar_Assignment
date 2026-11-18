#!/bin/bash
# 반복 녹화 rep1~3 후처리: 녹화가 끝나는 대로 확인 → bag 준비 → lidar_only c6 · V3 재생, 셋 다 끝나면 다섯 녹화 채점. PC, CPU, 도메인 113 · 114 · 116.
set -o pipefail
TOP=/home/pa4/git/lv2_next_module
HERE=$TOP/261030_2058
REC_DIR=$TOP/261030_1943
PY=/home/pa4/isaacsim-venv/bin/python
source /opt/ros/lyrical/setup.bash
source /home/pa4/git/ros_test/waffle-lyrical-slam/install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONPATH=/usr/lib/python3/dist-packages:${PYTHONPATH:-}
export ROS_LOG_DIR=$HERE/ros_logs
sha256sum $HERE/*.py $HERE/*.lua $HERE/variants/*.lua $HERE/run_reps_post.sh > $HERE/results/source_sha256_reps.txt
one() {
  local i=$1 dom=$2
  until grep -q "끝: .*n261028_rep${i}_" $REC_DIR/results_reps.log 2>/dev/null; do sleep 30; done
  local rec=$(ls -d $REC_DIR/results/n261028_rep${i}_* | tail -1)
  if [ ! -s $rec/check.json ] || grep -q '바인딩 해제한 변수' $rec/check_stdout.txt 2>/dev/null; then
    [ -f $rec/check_stdout.txt ] && mv $rec/check_stdout.txt $rec/check_stdout_TOOLFAIL_set_u.txt
    $PY $REC_DIR/check_record_lab.py $rec > $rec/check_stdout.txt 2>&1; echo "[post] rep$i check $?"
  fi
  $PY $HERE/prepare_bags.py $rec rep$i > $HERE/results/prepare_rep$i.log 2>&1; echo "[post] rep$i prepare $?"
  ( export ROS_DOMAIN_ID=$dom
    exec 8>/tmp/lv2_ros_domain_$dom.lock; flock -w 60 8
    timeout --signal=INT --kill-after=30s 1500 $PY $HERE/replay.py lidar_only rep$i || echo "fail c6 rep$i $?"
    VARIANT_LUA=$HERE/variants/V3_ceres_rot400.lua REPLAY_TAG=_V3_ceres_rot400 timeout --signal=INT --kill-after=30s 1500 $PY $HERE/replay.py lidar_only rep$i || echo "fail V3 rep$i $?"
  ) > $HERE/results/chain_reps_rep$i.log 2>&1
  echo "[post] rep$i replay 끝"
}
one 1 113 & p1=$!
one 2 114 & p2=$!
one 3 116 & p3=$!
wait $p1; wait $p2; wait $p3
args="bright=$REC_DIR/results/n261028_bright_261030_211830 dark=$REC_DIR/results/n261028_dark_261030_213001"
for i in 1 2 3; do args="$args rep$i=$(ls -d $REC_DIR/results/n261028_rep${i}_* | tail -1)"; done
$PY $HERE/evaluate_runs.py summary_reps.json $args > $HERE/results/evaluate_reps.log 2>&1; echo "[post] evaluate $?"
echo done
