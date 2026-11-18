#!/bin/bash
# 2단계 Nav2 실시간 연결 확인(모의): Isaac 장면(scene_live.py, cmd_vel 모드, 출발 자세 발제 (0.734, 0.95, π))
# → 261031_1325_nav2 launch 전체(마스크 · 합치기 · AMCL · EKF · Nav2 · rsp, 기본값 = 설계 11) → 목표 차례로(run_goals.py) → 채점(evaluate.py).
# 2D 모의 run_sim.sh 에서 모의기만 Isaac 장면으로 바꿈. PC, ROS 도메인 102(flock), GPU 잠금.
#   사용: run_live_nav2.sh <이름> [goals json] [조명 0~1] [nav2 params yaml, 기본 v11] [launch 추가 인자, 예 "turn_floor:=true"] [장면 추가 인자, 예 "--imu_topic /imu/data"] [run_goals 추가 인자, 예 "--frame mission_map"]
#   환경 변수 EXTRA_TOPICS(선택): 녹화에 더할 토픽(빈칸으로 구분, 기본 없음 = 예전 그대로)
#   설계 18(localization:=carto) 때문에 waffle-lyrical-slam install(Cartographer)도 source 함 — AMCL 구성에는 영향 없음
set -o pipefail
NAME=$1; DOM=102
TOP=/home/pa4/git/lv2_next_module
HERE=$TOP/261101_0016_isaac_live
NV=$TOP/261031_1325_nav2
GOALS=${2:-$NV/config/goals_seed1_v9.json}
LIGHT=${3:-1.0}
PARAMS=${4:-$NV/config/nav2_params_v11.yaml}
EXTRA=${5:-}
SCENE_EXTRA=${6:-}
GOALS_EXTRA=${7:-}
OUT=$HERE/results/$NAME
URDF=$TOP/261029_1030/urdf_261028/waffle_4wd_description_261028/urdf/waffle_4wd.urdf
LAB_PY=$HOME/pa-course/IsaacLab-6.1/.venv/bin/python
PYLINKS=$TOP/261025_0307/bridge_check/pylinks
[ -e $OUT ] && { echo "$OUT 이미 있음"; exit 1; }
mkdir -p $OUT
exec 8>/tmp/lv2_ros_domain_$DOM.lock; flock -w 60 8 || { echo "도메인 $DOM 잠금 실패"; exit 1; }
exec 9>/tmp/lv2_gpu.lock; flock -w 3600 9 || { echo "GPU 잠금 실패"; exit 1; }
set +u
source /opt/ros/lyrical/setup.bash
source /home/pa4/git/ros_test/waffle-lyrical-slam/install/setup.bash
export ROS_DOMAIN_ID=$DOM ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOG_DIR=$HERE/ros_logs
sha256sum $HERE/*.py $HERE/*.sh $NV/config/* $NV/nodes/*.py $NV/launch/*.py $NV/test/*.py \
  $TOP/261031_0335_slam_launch/nodes/scan_body_mask.py $TOP/261031_0335_slam_launch/nodes/map_odom_relay.py $HERE/results/live_bright/map/map.pbstream $TOP/261031_0335_slam_launch/config/ekf_sim.yaml $URDF > $OUT/source_sha256.txt
STOP=$OUT/stop_scene
env -u AMENT_PREFIX_PATH bash -c "export PYTHONPATH='$PYLINKS':\$PYTHONPATH && exec '$LAB_PY' '$HERE/scene_live.py' --out '$OUT/scene' --light_level $LIGHT --mode cmd_vel \
    --start -1.5005 -2.2636 3.14159265 --truth_topic /sim/truth --stop_file '$STOP' --max_sim_s 3000 $SCENE_EXTRA" > $OUT/isaac_stdout.txt 2>&1 8>&- 9>&- & SCP=$!
for i in $(seq 240); do grep -q '\[slam\] start' $OUT/isaac_stdout.txt 2>/dev/null && break; kill -0 $SCP 2>/dev/null || break; sleep 1; done
echo "scene_ready_after_s $i" > $OUT/exit_codes.txt
setsid ros2 launch $NV/launch/nav2_mission.launch.py use_sim_time:=true use_rsp:=true urdf:=$URDF params:=$PARAMS $EXTRA > $OUT/launch.log 2>&1 8>&- 9>&- & LP=$!
setsid ros2 bag record --use-sim-time -s mcap -o $OUT/out_bag --topics /clock /tf /tf_static /sim/truth /cmd_vel_nav /cmd_vel_ctrl /amcl_pose /plan \
    /odometry/filtered /odom /odom_icr /imu/data /scan_front /scan_rear /tracked_pose ${EXTRA_TOPICS:-} > $OUT/record.log 2>&1 8>&- 9>&- & RP=$!
( sleep 60; for n in $(timeout 20 ros2 node list 2>/dev/null); do echo "$n $(timeout 20 ros2 param get $n use_sim_time 2>&1 | tr '\n' ' ')"; done > $OUT/use_sim_time.txt ) 8>&- 9>&- & SP=$!
sleep 15
timeout -s INT 3600 /usr/bin/python3 $NV/test/run_goals.py --goals $GOALS --out $OUT/run_goals.json $GOALS_EXTRA > $OUT/run_goals.log 2>&1; echo "run_goals_exit $?" >> $OUT/exit_codes.txt
wait $SP
touch $STOP
wait $SCP; echo "isaac_exit $?" >> $OUT/exit_codes.txt
flock -u 9
kill -TERM -$RP 2>/dev/null          # 배경 작업은 SIGINT 무시(smoke_wp4) → TERM
for i in $(seq 120); do kill -0 $RP 2>/dev/null || break; sleep 0.5; done
kill -INT -$LP 2>/dev/null
sleep 8
for p in $RP $LP; do kill -KILL -$p 2>/dev/null; done
[ -f $OUT/out_bag/metadata.yaml ] || echo "out_bag metadata 없음 — ros2 bag reindex 필요" >> $OUT/exit_codes.txt
/usr/bin/python3 $HERE/make_truth_nav.py $OUT > $OUT/make_truth.log 2>&1; echo "make_truth_exit $?" >> $OUT/exit_codes.txt
/usr/bin/python3 $NV/test/evaluate.py --run $OUT > $OUT/evaluate.log 2>&1; echo "evaluate_exit $?" >> $OUT/exit_codes.txt
echo "goals $GOALS light $LIGHT params $PARAMS extra $EXTRA scene $SCENE_EXTRA goals_extra $GOALS_EXTRA" >> $OUT/exit_codes.txt
echo "[live-nav] 끝 $NAME" | tee $OUT/done.txt
