#!/bin/bash
# Python ↔ C++ loc_monitor 출력 비교 (PC, 모의 bag 재생, 11-14). 같은 bag · 같은 가림 스캔(C++ scan_body_mask 하나)을 두 판에 넣고
# 사건 기록(jsonl, 모의 시각)을 비교. 두 판은 토픽 이름만 바꿔 서로 안 겹치게 함.
#   loc_mon_equiv.sh <bag> <결과 폴더> [재생 배속=1.0]
BAG=$1; OUT=$2; RATE=${3:-1.0}
WS=~/git/Lv2_Cartlidar_Assignment_loccpu/workspace/ros2_ws
source /opt/ros/lyrical/setup.bash; source $WS/install/setup.bash
export ROS_DOMAIN_ID=87 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # 다른 시험 · 팀원과 안 섞이게
mkdir -p $OUT; rm -f $OUT/py.jsonl $OUT/cpp.jsonl
MAP=$WS/install/localization/share/localization/maps/slam_walls.yaml
COMMON="-p use_sim_time:=true -p backend:=amcl -p map:=$MAP -p res_mode:=interp_p75 -p res_thr:=0.034 -p check_mode:=pair -p gate_in:=none"
ros2 run perception scan_body_mask --ros-args -p use_sim_time:=true > $OUT/mask.log 2>&1 &
P1=$!
ros2 run localization loc_monitor --ros-args $COMMON -p event_log:=$OUT/py.jsonl -r __node:=loc_monitor_py \
  -r /localization/status:=/loc_py/status -r /localization/lock:=/loc_py/lock -r /initialpose:=/loc_py/initialpose > $OUT/py.log 2>&1 &
P2=$!
ros2 run perception loc_monitor --ros-args $COMMON -p event_log:=$OUT/cpp.jsonl -r __node:=loc_monitor_cpp \
  -r /localization/status:=/loc_cpp/status -r /localization/lock:=/loc_cpp/lock -r /initialpose:=/loc_cpp/initialpose > $OUT/cpp.log 2>&1 &
P3=$!
sleep 4
ros2 bag play $BAG --clock 100 -r $RATE --topics /scan_front /scan_rear /tf /tf_static /amcl_pose > $OUT/play.log 2>&1
sleep 3
kill -INT $P1 $P2 $P3; sleep 2; kill $P1 $P2 $P3 2>/dev/null
echo LOC_MON_EQUIV_DONE
