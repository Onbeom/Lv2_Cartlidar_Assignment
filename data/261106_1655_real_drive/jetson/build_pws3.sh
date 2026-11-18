#!/usr/bin/env bash
# pws2(GPU 왜곡 보정 + 단계 시간) + 위치 추정 C++ 판 (시험용 덧씌우기, 팀 설치본은 안 건드림). bringup 이 꺼진 때 실행 (코어 3 개, 약 5 분)
set -e
source /opt/ros/lyrical/setup.bash
source ~/Lv2_Cartlidar_Assignment/workspace/ros2_ws/install/setup.bash
cd ~/real_drive/pws3
cat VERSION
t0=$(date +%s)
MAKEFLAGS=-j3 nice -n 10 colcon build --packages-select perception localization perception_survey \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DONNXRUNTIME_ROOT=$HOME/onnxruntime 2>&1 | tail -5
grep -h "perception: VPI\|perception: TensorRT" log/latest_build/perception/stdout_stderr.log | head -3
ls install/perception/lib/perception | grep -E "^(loc_monitor|loc_scan_merge|scan_body_mask)$"
echo "빌드 $(( $(date +%s) - t0 )) s"
