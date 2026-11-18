#!/bin/bash
# 팀 검출기(브랜치 빌드의 perception_detector) ↔ ONNX Runtime + decode_cli. PC, GPU 잠금 안에서
set -o pipefail
E=/home/pa4/git/lv2_next_module/261101_0040_yolopost/equiv
B=/home/pa4/git/lv2_next_module/261101_0040_yolopost/build_branch/build/perception
SRC=$HOME/git/Lv2_Cartlidar_Assignment_yolopost/workspace/ros2_ws/src/perception
ENGINE=/home/pa4/git/lv2_next_module/261031_0100_trt/results/yolo11n_task4_B_fp32_notf32.engine
g++ -std=c++17 -O2 -o $E/trt_team $E/trt_team.cpp -I$SRC/include $B/libperception_detector.a \
  $(pkg-config --cflags --libs opencv4) -lnvinfer -L/usr/local/cuda/lib64 -lcudart || exit 1
ls /home/pa4/git/lv2_next_module/261030_1325/B_negative/images/eval/*.png | sort > $E/images.txt
sha256sum $E/trt_team.cpp $E/compare.py $E/run_equiv.sh $ENGINE > $E/source_sha256.txt
exec 9>/tmp/lv2_gpu.lock; flock 9
$E/trt_team $ENGINE $E/images.txt > $E/trt_team.jsonl 2> $E/trt_team.err || { echo "trt_team 실패 $?"; exit 1; }
flock -u 9
$HOME/yolo-venv/bin/python -I $E/compare.py $E/trt_team.jsonl $E
