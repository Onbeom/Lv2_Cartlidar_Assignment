#!/bin/bash
# 11-03 A-2 영상 시각 측정(모의): 261103_1502_loc_pkg/run_live_pkg.sh 사본. 바꾼 것: 결과 폴더 = 이 폴더 results/, 탐침 3 개를 launch 와 함께 띄움
#   (tf_probe map → 앞 카메라 infra1 optical · map → base_link 10 Hz, tools/img_lag_probe.py 영상 받을 때마다), sha 목록에 이 폴더 파일 더함. 나머지 같음.
#   loc_pkg 의 launch · ws(install) · config 는 그대로 불러 씀(고치지 않음).
# 261102 B안 사본(261101_0016_isaac_live/run_live_nav2.sh). 바꾼 것: 결과 폴더 = 이 폴더 results/, robot_state_publisher URDF = B안(261102_1440_urdf_B light),
#   장면 = 261102_1450_isaac_B_probe/scene_live.py + LV2_ISAAC_MODEL=B안 isaac 폴더, sha 목록에 B안 파일 더함, 환경 변수 LAUNCH(선택)로 launch 파일을 바꿀 수 있음. 나머지(make_truth · evaluate · launch)는 원본 그대로 불러 씀.
#   사용: run_a2.sh <이름> [goals json] [조명 0~1] [nav2 params yaml] [launch 추가 인자] [장면 추가 인자] [run_goals 추가 인자]
#   환경 변수 EXTRA_TOPICS(선택): 녹화에 더할 토픽(빈칸으로 구분)
set -o pipefail
NAME=$1; DOM=102
TOP=/home/pa4/git/lv2_next_module
HERE=$TOP/261103_1909_cam_timing
LOCPKG=$TOP/261103_1502_loc_pkg
PKG=/home/pa4/git/Lv2_Cartlidar_Assignment_loc/workspace/ros2_ws/src/localization
TEAM_EKF=/home/pa4/git/Lv2_Cartlidar_Assignment_loc/workspace/ros2_ws/src/bringup/config/ekf.yaml
LIVE=$TOP/261101_0016_isaac_live
SCENE=$TOP/261102_1450_isaac_B_probe/scene_live.py
MODEL_B=$TOP/261102_1440_urdf_B/isaac
NV=$TOP/261031_1325_nav2
PROBE=$TOP/261031_0335_slam_launch/tools/tf_probe.py
GOALS=${2:-$NV/config/goals_seed1_v9.json}
LIGHT=${3:-1.0}
PARAMS=${4:-$NV/config/nav2_params_v11.yaml}
EXTRA=${5:-}
SCENE_EXTRA=${6:-}
GOALS_EXTRA=${7:-}
OUT=$HERE/results/$NAME
URDF=$MODEL_B/urdf/waffle_4wd_light.urdf
LAB_PY=$HOME/pa-course/IsaacLab-6.1/.venv/bin/python
PYLINKS=$TOP/261025_0307/bridge_check/pylinks
[ -e $OUT ] && { echo "$OUT 이미 있음"; exit 1; }
mkdir -p $OUT
exec 8>/tmp/lv2_ros_domain_$DOM.lock; flock -w 60 8 || { echo "도메인 $DOM 잠금 실패"; exit 1; }
exec 9>/tmp/lv2_gpu.lock; flock -w 21600 9 || { echo "GPU 잠금 실패"; exit 1; }
set +u
source /opt/ros/lyrical/setup.bash
source /home/pa4/git/ros_test/waffle-lyrical-slam/install/setup.bash
source $LOCPKG/ws/install/setup.bash
export ROS_DOMAIN_ID=$DOM ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOG_DIR=$HERE/ros_logs
sha256sum $PKG/localization/*.py $PKG/launch/*.py $PKG/config/* $PKG/maps/* $TEAM_EKF $LOCPKG/launch/*.py $SCENE $MODEL_B/urdf/sensors.json $HERE/*.sh $HERE/tools/*.py $PROBE $LOCPKG/config/* $LOCPKG/tools/*.py $TOP/261102_1646_nav2_B/config/* $LIVE/*.py $NV/config/* $NV/nodes/*.py $NV/launch/*.py $NV/test/*.py \
  $URDF > $OUT/source_sha256.txt
STOP=$OUT/stop_scene
env -u AMENT_PREFIX_PATH LV2_ISAAC_MODEL=$MODEL_B bash -c "export PYTHONPATH='$PYLINKS':\$PYTHONPATH && exec '$LAB_PY' '$SCENE' --out '$OUT/scene' --light_level $LIGHT --mode cmd_vel \
    --start -1.5005 -2.2636 3.14159265 --truth_topic /sim/truth --stop_file '$STOP' --max_sim_s 3000 $SCENE_EXTRA" > $OUT/isaac_stdout.txt 2>&1 8>&- 9>&- & SCP=$!
for i in $(seq 240); do grep -q '\[slam\] start' $OUT/isaac_stdout.txt 2>/dev/null && break; kill -0 $SCP 2>/dev/null || break; sleep 1; done
echo "scene_ready_after_s $i" > $OUT/exit_codes.txt
setsid ros2 launch ${LAUNCH:-$LOCPKG/launch/test_stack.launch.py} use_sim_time:=true use_rsp:=true urdf:=$URDF params:=$PARAMS $EXTRA > $OUT/launch.log 2>&1 8>&- 9>&- & LP=$!
setsid ros2 bag record --use-sim-time -s mcap -o $OUT/out_bag --topics /clock /tf /tf_static /sim/truth /cmd_vel_nav /cmd_vel_ctrl /amcl_pose /plan \
    /odometry/filtered /odom /odom_icr /imu/data /scan_front /scan_rear /tracked_pose ${EXTRA_TOPICS:-} > $OUT/record.log 2>&1 8>&- 9>&- & RP=$!
setsid /usr/bin/python3 $PROBE --ros-args -p out:=$OUT/tf_probe_cam.csv -p use_sim_time:=true -p target:=map -p source:=top_camera_front_infra1_optical_frame > $OUT/probe_cam.log 2>&1 8>&- 9>&- & P1=$!
setsid /usr/bin/python3 $PROBE --ros-args -p out:=$OUT/tf_probe.csv -p use_sim_time:=true -p target:=map -p source:=base_link -r __node:=tf_probe_base > $OUT/probe_base.log 2>&1 8>&- 9>&- & P2=$!
setsid /usr/bin/python3 $HERE/tools/img_lag_probe.py --ros-args -p out:=$OUT/img_lag.csv -p use_sim_time:=true > $OUT/probe_img.log 2>&1 8>&- 9>&- & P3=$!
( sleep 60; for n in $(timeout 20 ros2 node list 2>/dev/null); do echo "$n $(timeout 20 ros2 param get $n use_sim_time 2>&1 | tr '\n' ' ')"; done > $OUT/use_sim_time.txt ) 8>&- 9>&- & SP=$!
sleep 15
timeout -s INT 3600 /usr/bin/python3 $NV/test/run_goals.py --goals $GOALS --out $OUT/run_goals.json $GOALS_EXTRA > $OUT/run_goals.log 2>&1; echo "run_goals_exit $?" >> $OUT/exit_codes.txt
wait $SP
touch $STOP
wait $SCP; echo "isaac_exit $?" >> $OUT/exit_codes.txt
flock -u 9
kill -TERM -$RP 2>/dev/null          # 배경 작업은 SIGINT 무시(smoke_wp4) → TERM
for i in $(seq 120); do kill -0 $RP 2>/dev/null || break; sleep 0.5; done
for p in $P1 $P2 $P3; do kill -TERM -$p 2>/dev/null; done
kill -INT -$LP 2>/dev/null
sleep 8
for p in $RP $LP $P1 $P2 $P3; do kill -KILL -$p 2>/dev/null; done
[ -f $OUT/out_bag/metadata.yaml ] || echo "out_bag metadata 없음 — ros2 bag reindex 필요" >> $OUT/exit_codes.txt
echo "global_costmap_resize $(grep 'global_costmap.global_costmap\]: StaticLayer: Resizing costmap' $OUT/launch.log | grep -o 'at [0-9.]*' | tr '\n' ' ')" >> $OUT/exit_codes.txt
/usr/bin/python3 $LIVE/make_truth_nav.py $OUT > $OUT/make_truth.log 2>&1; echo "make_truth_exit $?" >> $OUT/exit_codes.txt
/usr/bin/python3 $NV/test/evaluate.py --run $OUT > $OUT/evaluate.log 2>&1; echo "evaluate_exit $?" >> $OUT/exit_codes.txt
(source /opt/ros/lyrical/setup.bash; /usr/bin/python3 $HERE/tools/evaluate_a2.py $OUT > $OUT/evaluate_a2.log 2>&1); echo "evaluate_a2_exit $?" >> $OUT/exit_codes.txt
echo "goals $GOALS light $LIGHT params $PARAMS extra $EXTRA scene $SCENE_EXTRA goals_extra $GOALS_EXTRA" >> $OUT/exit_codes.txt
echo "[live-nav] 끝 $NAME" | tee $OUT/done.txt
