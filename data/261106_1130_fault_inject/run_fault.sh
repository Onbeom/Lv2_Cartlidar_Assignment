#!/usr/bin/env bash
# 교안 L7 장애 주입 실행(PC). run_fault.sh <결과 폴더> [color] [--cases F1,F2]
# color: 엔진 없이 색 검출기로 도구만 확인(GPU 안 씀, 판정 아님). 그 밖에는 E3 PC 엔진으로 판정한다.
set -u
ROOT=/home/pa4/git/lv2_next_module; W=$ROOT/261106_1130_fault_inject; E=$ROOT/261105_2015_yolo_e3
OUT=$1; MODE=${2:-engine}; shift; [ $# -gt 0 ] && shift
export ROS_DOMAIN_ID=146 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export REPLAY_URDF=$ROOT/261029_1030/urdf_261028/waffle_4wd_description_261028/urdf/waffle_4wd.urdf
set +u; source /opt/ros/lyrical/setup.bash; source $W/install/setup.bash; set -u
ARGS=(--out "$OUT" --render $ROOT/261031_1718_r3port/b0/judge/render_101_T3000_awb --share $W/install/perception/share/perception/config --urdf "$REPLAY_URDF")
[ "$MODE" = engine ] && ARGS+=(--engine480 $W/trt/yolo11n_task4_E3_480_fp16.engine --engine384 $W/trt/yolo11n_task4_E3_384_fp16.engine)
exec python3 $W/fault_driver.py "${ARGS[@]}" "$@"
