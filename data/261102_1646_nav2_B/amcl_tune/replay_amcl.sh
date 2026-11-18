#!/bin/bash
# 도구(판정 아님, GPU 없음): Isaac 실시간 녹화(out_bag)의 /clock · 라이다 앞뒤 · tf_static · /odometry/filtered · /sim/truth 를 다시 틀어
# 라이브 AMCL 구성과 같은 노드(scan_body_mask → scan_merge → map_server · amcl, odom→base_link 는 odom_to_tf)로 AMCL 을 돌리고
# loc_eval 로 참값 대비 추정 자세를 csv 로 남긴다. 녹화의 /tf(그때 위치 추정 TF)는 틀지 않음.
#   사용: replay_amcl.sh <결과 폴더> <원본 실행 폴더(out_bag · run_goals.json)> <amcl params yaml> [재생 속도 1.0] [ROS 도메인 110]
OUT=$1; SRC=$2; PAR=$3; RATE=${4:-1.0}; DOM=${5:-110}
SLAM=/home/pa4/git/lv2_next_module/261031_0335_slam_launch; N=/home/pa4/git/lv2_next_module/261031_1325_nav2; T=$(dirname $(readlink -f $0))
[ -e $OUT ] && { echo "$OUT 이미 있음"; exit 1; }
mkdir -p $OUT
exec 8>/tmp/lv2_ros_domain_$DOM.lock; flock -w 600 8 || { echo "도메인 $DOM 잠금 실패"; exit 1; }
set +u; source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=$DOM ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOG_DIR=$OUT/ros_logs
cp $PAR $OUT/params.yaml
sha256sum $PAR $T/*.py $T/replay_amcl.sh $SLAM/nodes/scan_body_mask.py $N/nodes/scan_merge.py $SRC/out_bag/*.mcap > $OUT/source_sha256.txt
S="--ros-args -p use_sim_time:=true"
PIDS=()
run() { local log=$1; shift; setsid "$@" > $OUT/$log 2>&1 8>&- & PIDS+=($!); }
run odom_to_tf.log /usr/bin/python3 $T/odom_to_tf.py $S
run scan_body_mask.log /usr/bin/python3 $SLAM/nodes/scan_body_mask.py $S
run scan_merge.log /usr/bin/python3 $N/nodes/scan_merge.py $S
run map_server.log ros2 run nav2_map_server map_server $S --params-file $PAR
run amcl.log ros2 run nav2_amcl amcl $S --params-file $PAR
run lifecycle.log ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=lifecycle_manager_localization -p use_sim_time:=true -p autostart:=true -p "node_names:=[map_server,amcl]"
run loc_eval.log /usr/bin/python3 $T/loc_eval.py $OUT/loc_eval.csv $S
sleep 6
ros2 bag play $SRC/out_bag --rate $RATE --topics /clock /scan_front /scan_rear /tf_static /odometry/filtered /sim/truth > $OUT/play.log 2>&1 8>&-
echo "play_exit $?" > $OUT/exit_codes.txt
sleep 3
for p in "${PIDS[@]}"; do kill -INT -- -$p 2>/dev/null; done; sleep 3
for p in "${PIDS[@]}"; do kill -KILL -- -$p 2>/dev/null; done
grep -c "Setting pose\|initialPoseReceived\|Received initial pose" $OUT/amcl.log >> $OUT/exit_codes.txt
/usr/bin/python3 $T/score.py $OUT $SRC > $OUT/score.json
cat $OUT/score.json
