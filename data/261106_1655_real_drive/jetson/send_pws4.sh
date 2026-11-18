#!/usr/bin/env bash
# PC 에서 실행: 합친 시험 판(~/git/Lv2_Cartlidar_Assignment_overlay, exp/overlay-1115 = dev 8cafcc7 + fix/localization-cpu + fix/perception-gpu-undistort
#   + fix/perception-arm-cam-pose-table, 커밋 전)의 perception · localization · perception_survey 를 Jetson ~/real_drive/pws4/src 로 보냄.
#   팀 설치본은 안 건드림. 빌드는 Jetson 에서 build_pws4.sh (bringup 이 꺼진 때).
set -e
SRC=~/git/Lv2_Cartlidar_Assignment_overlay/workspace/ros2_ws/src
SOCK=$HOME/.ssh/lv2_jetson.sock
J="ssh -o ControlPath=$SOCK cart11@cart11.local"
H=$(cd ~/git/Lv2_Cartlidar_Assignment_overlay && { git diff HEAD; git ls-files --others --exclude-standard | xargs -r cat; } | sha256sum | cut -c1-12)
$J 'mkdir -p ~/real_drive/pws4/src'
for p in perception localization perception_survey; do
  rsync -a --delete -e "ssh -o ControlPath=$SOCK" $SRC/$p/ cart11@cart11.local:real_drive/pws4/src/$p/
done
$J "printf '%s\n%s\n' 'dev 8cafcc7 + fix/localization-cpu(위치 추정 C++) + fix/perception-gpu-undistort(VPI 왜곡 보정 · CUDA YOLO 입력 · 단계 시간) + fix/perception-arm-cam-pose-table(#296, 팔 카메라 자세별 보정, 기본 꺼짐) — exp/overlay-1115, 커밋 전' '$H' > ~/real_drive/pws4/VERSION; cat ~/real_drive/pws4/VERSION"
