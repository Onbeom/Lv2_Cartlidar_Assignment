#!/bin/bash
# launch 확인(모의): 시험 bag 재생 → launch 전체(마스크 · Cartographer · 중계 · EKF · 점유 지도) → 지도 저장 · 도면 맞춤. PC, CPU.
#   사용: test/run_replay.sh <bright|dark> <ROS 도메인>
set -o pipefail
LIGHT=$1; DOM=$2
HERE=/home/pa4/git/lv2_next_module/261031_0335_slam_launch
OUT=$HERE/test/results/$LIGHT
[ -e $OUT ] && { echo "$OUT 이미 있음"; exit 1; }
mkdir -p $OUT
set +u
source /opt/ros/lyrical/setup.bash
source /home/pa4/git/ros_test/waffle-lyrical-slam/install/setup.bash
export ROS_DOMAIN_ID=$DOM ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOG_DIR=$HERE/test/ros_logs
exec 8>/tmp/lv2_ros_domain_$DOM.lock; flock -w 60 8 || { echo "도메인 $DOM 잠금 실패"; exit 1; }
sha256sum $HERE/config/* $HERE/nodes/*.py $HERE/tools/*.py $HERE/launch/*.py $HERE/test/*.py $HERE/test/*.sh > $OUT/source_sha256.txt
setsid ros2 launch $HERE/launch/slam_carto.launch.py use_sim_time:=true mask_save:=$OUT/body_mask.yaml > $OUT/launch.log 2>&1 & LP=$!
setsid ros2 bag record --use-sim-time -s mcap -o $OUT/out_bag --topics /tf /tracked_pose /odometry/filtered /top_camera_front/infra1/camera_info /scan_front_filtered /scan_rear_filtered > $OUT/record.log 2>&1 & RP=$!
setsid /usr/bin/python3 $HERE/tools/tf_probe.py --ros-args -p use_sim_time:=true -p out:=$OUT/tf_probe.csv > $OUT/tf_probe.log 2>&1 & PP=$!
sleep 8
ros2 bag play $HERE/test/bags/$LIGHT --rate 1.0 --disable-keyboard-controls > $OUT/play.log 2>&1; echo "play_exit $?" > $OUT/exit_codes.txt
sleep 3
/usr/bin/python3 $HERE/tools/save_and_align_map.py --out $OUT/map > $OUT/save_align.log 2>&1; echo "save_align_exit $?" >> $OUT/exit_codes.txt
kill -INT -$RP 2>/dev/null          # 녹화기 먼저 정상 종료(끝 몇 초 · metadata.yaml 이 빠지지 않게)
for i in $(seq 60); do kill -0 $RP 2>/dev/null || break; sleep 0.5; done
for p in $PP $LP; do kill -INT -$p 2>/dev/null; done
sleep 8
for p in $PP $RP $LP; do kill -KILL -$p 2>/dev/null; done
[ -f $OUT/out_bag/metadata.yaml ] || echo "out_bag metadata 없음 — ros2 bag reindex 필요" >> $OUT/exit_codes.txt
echo "[run] 끝 $LIGHT"
