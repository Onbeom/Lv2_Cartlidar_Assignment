#!/bin/bash
# Jetson: 정밀도 비교 한 번에(11-04). ① 정확도(카메라 없음, 약 10분) ② 실물 부하(카메라 3대 + YOLO 15 Hz × 3) 엔진마다 180 초, 사이 30 초 쉼.
# 시스템 python3 로: bash run_all.sh   (카메라 · 다른 YOLO 프로그램은 끄고)
set -u; cd "$(dirname "$0")"; H=$HOME; R=$PWD/results; mkdir -p $R
python3 acc_check.py || exit 1
run() { echo "== 부하 $1"; python3 load_test.py --engine "$2" ${3:+--engine-arm "$3"} --seconds 180 --out $R/load_$1 || echo "부하 $1 실패"; sleep 30; }
run P32 $H/yolo_D/deploy/yolo11n_task4_D_fp32_notf32.engine
run P16 $H/fp16_check/results/yolo11n_task4_D_fp16.engine
run I8 $H/fp16_check/int8/yolo11n_task4_D_int8_qdq.engine
run I8F16 $R/yolo11n_task4_D_int8_qdq_fp16.engine
run N16 $H/yolo_D/deploy/yolo11n_task4_D_480_fp16.engine $H/yolo_D/deploy/yolo11n_task4_D_384_fp16.engine
echo "모두 끝 → $R"
