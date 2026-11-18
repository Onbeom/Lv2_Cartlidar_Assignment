#!/bin/bash
# Nav2 모의 확인(2D 운동학 모의기, Isaac 아님): 모의기 → launch 전체(마스크 · 합치기 · AMCL · EKF · Nav2 · rsp) → 목표 차례로 → 채점. PC, CPU.
#   사용: test/run_sim.sh <결과 이름> <ROS 도메인> [goals json] [nav2 params yaml] [모의기 잡음 seed] [고정 물체 지도 yaml]
set -o pipefail
NAME=$1; DOM=$2
HERE=/home/pa4/git/lv2_next_module/261031_1325_nav2
GOALS=${3:-$HERE/config/goals_seed1.json}
PARAMS=${4:-$HERE/config/nav2_params.yaml}
SEED=${5:-20261031}
OBJ=${6:-$HERE/config/map_objects.yaml}
OUT=$HERE/results/$NAME
URDF=/home/pa4/git/lv2_next_module/261029_1030/urdf_261028/waffle_4wd_description_261028/urdf/waffle_4wd.urdf
[ -e $OUT ] && { echo "$OUT 이미 있음"; exit 1; }
mkdir -p $OUT
set +u
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=$DOM ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOG_DIR=$HERE/test/ros_logs
exec 8>/tmp/lv2_ros_domain_$DOM.lock; flock -w 60 8 || { echo "도메인 $DOM 잠금 실패"; exit 1; }
sha256sum $HERE/config/* $HERE/nodes/*.py $HERE/tools/*.py $HERE/launch/*.py $HERE/test/*.py $HERE/test/*.sh \
  /home/pa4/git/lv2_next_module/261031_0335_slam_launch/nodes/scan_body_mask.py /home/pa4/git/lv2_next_module/261031_0335_slam_launch/config/ekf_sim.yaml > $OUT/source_sha256.txt
setsid /usr/bin/python3 $HERE/nodes/sim2d.py --ros-args -p log:=$OUT/truth.csv -p seed:=$SEED > $OUT/sim.log 2>&1 & SP=$!
sleep 1
setsid ros2 launch $HERE/launch/nav2_mission.launch.py use_sim_time:=true use_rsp:=true urdf:=$URDF params:=$PARAMS objects_map:=$OBJ > $OUT/launch.log 2>&1 & LP=$!
setsid ros2 bag record --use-sim-time -s mcap -o $OUT/out_bag --topics /tf /tf_static /sim/truth /cmd_vel_nav /cmd_vel_ctrl /amcl_pose /plan /odometry/filtered > $OUT/record.log 2>&1 & RP=$!
sleep 10
timeout -s INT 2400 /usr/bin/python3 $HERE/test/run_goals.py --goals $GOALS --out $OUT/run_goals.json > $OUT/run_goals.log 2>&1; echo "run_goals_exit $?" > $OUT/exit_codes.txt
kill -INT -$RP 2>/dev/null
for i in $(seq 60); do kill -0 $RP 2>/dev/null || break; sleep 0.5; done
for p in $LP $SP; do kill -INT -$p 2>/dev/null; done
sleep 8
for p in $RP $LP $SP; do kill -KILL -$p 2>/dev/null; done
[ -f $OUT/out_bag/metadata.yaml ] || echo "out_bag metadata 없음 — ros2 bag reindex 필요" >> $OUT/exit_codes.txt
/usr/bin/python3 $HERE/test/evaluate.py --run $OUT > $OUT/evaluate.log 2>&1; echo "evaluate_exit $?" >> $OUT/exit_codes.txt
echo "goals $GOALS params $PARAMS seed $SEED objects $OBJ" >> $OUT/exit_codes.txt
echo "[run] 끝 $NAME"
