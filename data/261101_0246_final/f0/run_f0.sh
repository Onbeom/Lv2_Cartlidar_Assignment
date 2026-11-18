#!/bin/bash
# F0 도구 확인 (PC): 합친 트리 + #32 빌드의 팀 TensorRT 검출기로 B0′ 판정 렌더(AWB) D435 영상 288장 검출 → 정답 투영과 대조
set -o pipefail
F=/home/pa4/git/lv2_next_module/261101_0246_final/f0
J=/home/pa4/git/lv2_next_module/261031_1718_r3port/b0/judge
B=/home/pa4/git/lv2_next_module/261101_0040_yolopost/build_merged/build/perception
SRC=$HOME/git/Lv2_Cartlidar_Assignment_merge9yp/workspace/ros2_ws/src/perception
ENGINE=/home/pa4/git/lv2_next_module/261031_0100_trt/results/yolo11n_task4_B_fp32_notf32.engine
g++ -std=c++17 -O2 -o $F/trt_team /home/pa4/git/lv2_next_module/261101_0040_yolopost/equiv/trt_team.cpp -I$SRC/include \
  $B/libperception_detector.a $(pkg-config --cflags --libs opencv4) -lnvinfer -L/usr/local/cuda/lib64 -lcudart || exit 1
RENDERS=$(ls -d $J/render_*_T????_awb | sort)
for r in $RENDERS; do ls $r/w*_d*_L?/frame.png; done | sort > $F/images.txt
wc -l < $F/images.txt
sha256sum $F/eval_f0.py $F/run_f0.sh $ENGINE > $F/source_sha256.txt
exec 9>/tmp/lv2_gpu.lock; flock 9
$F/trt_team $ENGINE $F/images.txt > $F/trt_team.jsonl 2> $F/trt_team.err || { echo "trt_team 실패 $?"; exit 1; }
flock -u 9
python3 -I $F/eval_f0.py $F/trt_team.jsonl $RENDERS > $F/summary.json && echo done > $F/done.txt
