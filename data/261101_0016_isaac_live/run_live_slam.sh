#!/bin/bash
# 1단계 SLAM 실시간 연결 확인(모의): Isaac 장면(scene_live.py, 경로 모드)을 실시간으로 띄우고 261031_0335 SLAM launch 가 bag 대신 바로 받는다.
# 지도 저장 · 도면 맞춤 → evaluate_live.py. PC, ROS 도메인 101(flock), GPU 잠금 /tmp/lv2_gpu.lock.
#   사용: run_live_slam.sh <이름> <조명 0~1> [경유점 개수, 0 = 전체]
set -o pipefail
NAME=$1; LIGHT=$2; NWP=${3:-0}; DOM=101
HERE=/home/pa4/git/lv2_next_module/261101_0016_isaac_live
TOP=/home/pa4/git/lv2_next_module
SL=$TOP/261031_0335_slam_launch
OUT=$HERE/results/$NAME
[ -e $OUT ] && { echo "$OUT 이미 있음"; exit 1; }
mkdir -p $OUT
URDF=$TOP/261029_1030/urdf_261028/waffle_4wd_description_261028/urdf/waffle_4wd.urdf
LAB_PY=$HOME/pa-course/IsaacLab-6.1/.venv/bin/python
PYLINKS=$TOP/261025_0307/bridge_check/pylinks
exec 8>/tmp/lv2_ros_domain_$DOM.lock; flock -w 60 8 || { echo "도메인 $DOM 잠금 실패"; exit 1; }
exec 9>/tmp/lv2_gpu.lock; flock -w 3600 9 || { echo "GPU 잠금 실패"; exit 1; }
set +u
source /opt/ros/lyrical/setup.bash
source /home/pa4/git/ros_test/waffle-lyrical-slam/install/setup.bash
export ROS_DOMAIN_ID=$DOM ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOG_DIR=$HERE/ros_logs
sha256sum $HERE/*.py $HERE/*.sh $SL/config/* $SL/nodes/*.py $SL/tools/*.py $SL/launch/slam_carto.launch.py $URDF > $OUT/source_sha256.txt
awk '/^Udp:/ {getline; print "before," $6}' /proc/net/snmp > $OUT/rcvbuf_errors.csv
setsid ros2 launch $SL/launch/slam_carto.launch.py use_sim_time:=true use_rsp:=true urdf:=$URDF mask_save:=$OUT/body_mask.yaml > $OUT/launch.log 2>&1 8>&- 9>&- & LP=$!
setsid ros2 bag record --use-sim-time -s mcap -o $OUT/out_bag --topics /clock /tf /tf_static /tracked_pose /odometry/filtered /odom /scan_front /scan_rear \
    /scan_front_filtered /scan_rear_filtered /top_camera_front/infra1/camera_info > $OUT/record.log 2>&1 8>&- 9>&- & RP=$!
setsid /usr/bin/python3 $SL/tools/tf_probe.py --ros-args -r __node:=tf_probe_base -p use_sim_time:=true -p out:=$OUT/tf_probe.csv > $OUT/tf_probe.log 2>&1 8>&- 9>&- & PP=$!
setsid /usr/bin/python3 $SL/tools/tf_probe.py --ros-args -r __node:=tf_probe_cam -p use_sim_time:=true -p out:=$OUT/tf_probe_cam.csv \
    -p target:=mission_map -p source:=top_camera_front_infra1_optical_frame > $OUT/tf_probe_cam.log 2>&1 8>&- 9>&- & PC=$!
sleep 8
( sleep 90; for n in $(timeout 20 ros2 node list 2>/dev/null); do echo "$n $(timeout 20 ros2 param get $n use_sim_time 2>&1 | tr '\n' ' ')"; done > $OUT/use_sim_time.txt ) 8>&- 9>&- & SP=$!
env -u AMENT_PREFIX_PATH bash -c "export PYTHONPATH='$PYLINKS':\$PYTHONPATH && exec '$LAB_PY' '$HERE/scene_live.py' --out '$OUT/scene' --light_level $LIGHT --max_waypoints $NWP --mode route" \
    > $OUT/isaac_stdout.txt 2>&1; echo "isaac_exit $?" > $OUT/exit_codes.txt
wait $SP
flock -u 9
sleep 3
/usr/bin/python3 $SL/tools/save_and_align_map.py --out $OUT/map > $OUT/save_align.log 2>&1; echo "save_align_exit $?" >> $OUT/exit_codes.txt
kill -TERM -$RP 2>/dev/null          # 스크립트의 배경 작업은 SIGINT 를 무시한 채 시작함(도구 확인 smoke_wp4 에서 발견) → TERM
for i in $(seq 120); do kill -0 $RP 2>/dev/null || break; sleep 0.5; done
for p in $PP $PC $LP; do kill -INT -$p 2>/dev/null; done
sleep 8
for p in $PP $PC $RP $LP; do kill -KILL -$p 2>/dev/null; done
awk '/^Udp:/ {getline; print "after," $6}' /proc/net/snmp >> $OUT/rcvbuf_errors.csv
[ -f $OUT/out_bag/metadata.yaml ] || echo "out_bag metadata 없음 — ros2 bag reindex 필요" >> $OUT/exit_codes.txt
/usr/bin/python3 $HERE/evaluate_live.py $OUT > $OUT/evaluate.log 2>&1; echo "evaluate_exit $?" >> $OUT/exit_codes.txt
echo "[live] 끝 $NAME" | tee $OUT/done.txt
