#!/bin/bash
# 도구 확인(판정 아님, GPU 없음): Isaac 실시간 Nav2 녹화(out_bag)의 라이다 · tf_static · /clock 만 다시 틀어 Cartographer 순수 위치 추정을
# 다른 lua 로 돌리고 /tracked_pose 를 녹화한다. launch(localization:=carto)와 같은 노드: scan_body_mask → cartographer_node(저장 pbstream frozen) · carto_start.
# 비교는 replay_carto_compare.py. PC, ROS 도메인 103(환경 변수 DOM 으로 바꿈, flock, 실시간 실행 102 와 따로).
#   사용: replay_carto.sh <이름> <결과 폴더(out_bag 있는 곳)> <lua 이름(config/ 안)> [재생 속도, 기본 1.0]
#   환경 변수: PLAY_EXTRA(더 틀 토픽, 예 /odometry/filtered) · CARTO_REMAP(cartographer_node 추가 remap, 예 "-r odom:=/odometry/filtered")
set -o pipefail
NAME=$1; SRC=$2; LUA=$3; RATE=${4:-1.0}; DOM=${DOM:-103}
HERE=/home/pa4/git/lv2_next_module/261031_1325_nav2
SLAM=/home/pa4/git/lv2_next_module/261031_0335_slam_launch
STATE=/home/pa4/git/lv2_next_module/261101_0016_isaac_live/results/live_bright/map/map.pbstream
OUT=$HERE/results/$NAME
[ -e $OUT ] && { echo "$OUT 이미 있음"; exit 1; }
[ -d $SRC/out_bag ] || { echo "$SRC/out_bag 없음"; exit 1; }
mkdir -p $OUT
exec 8>/tmp/lv2_ros_domain_$DOM.lock; flock -w 60 8 || { echo "도메인 $DOM 잠금 실패"; exit 1; }
set +u
source /opt/ros/lyrical/setup.bash
source /home/pa4/git/ros_test/waffle-lyrical-slam/install/setup.bash
export ROS_DOMAIN_ID=$DOM ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOG_DIR=$HERE/test/ros_logs
sha256sum $HERE/config/$LUA $STATE $SLAM/nodes/scan_body_mask.py $HERE/nodes/carto_start.py $HERE/test/replay_carto.sh $SRC/out_bag/*.mcap > $OUT/source_sha256.txt
SIM="-p use_sim_time:=true"
setsid /usr/bin/python3 $SLAM/nodes/scan_body_mask.py --ros-args $SIM > $OUT/scan_body_mask.log 2>&1 8>&- & P1=$!
setsid ros2 run cartographer_ros cartographer_node \
  -configuration_directory $HERE/config -configuration_basename $LUA -load_state_filename $STATE -load_frozen_state=true \
  -start_trajectory_with_default_topics=false --ros-args $SIM -r scan_1:=/scan_front_filtered -r scan_2:=/scan_rear_filtered $CARTO_REMAP > $OUT/cartographer.log 2>&1 8>&- & P2=$!
setsid ros2 bag record --use-sim-time -s mcap -o $OUT/replay_bag --topics /tracked_pose > $OUT/record.log 2>&1 8>&- & P3=$!
timeout 180 /usr/bin/python3 $HERE/nodes/carto_start.py --ros-args $SIM -p carto_config_dir:=$HERE/config -p carto_config:=$LUA \
  -p align_x:=0.729 -p align_y:=0.960 -p align_yaw:=3.143774 -p start_x:=0.734 -p start_y:=0.950 -p start_yaw:=3.14159265 > $OUT/carto_start.log 2>&1
CS=$?; echo "carto_start_exit $CS" > $OUT/exit_codes.txt
if [ $CS -ne 0 ]; then for p in $P3 $P2 $P1; do kill -KILL -$p 2>/dev/null; done; echo "carto_start 실패 — 재생 안 함" >> $OUT/exit_codes.txt; exit 1; fi
sleep 3
ros2 bag play $SRC/out_bag --rate $RATE --topics /clock /scan_front /scan_rear /tf_static $PLAY_EXTRA --disable-keyboard-controls > $OUT/play.log 2>&1 8>&-
echo "play_exit $?" >> $OUT/exit_codes.txt
sleep 5
kill -TERM -$P3 2>/dev/null; for p in $P2 $P1; do kill -INT -$p 2>/dev/null; done   # 녹화기는 TERM(스크립트 배경 작업은 SIGINT 무시로 시작)
sleep 6
for p in $P3 $P2 $P1; do kill -KILL -$p 2>/dev/null; done
echo "src $SRC lua $LUA rate $RATE play_extra $PLAY_EXTRA carto_remap $CARTO_REMAP" >> $OUT/exit_codes.txt
touch $OUT/done.txt
