#!/bin/bash
# 실물 R1 촬영 (Jetson): bash ~/front_cal/r1_run.sh <놓기 이름 A1|A2|A3|A4>
# - 앞 D435 컬러 토픽이 이미 있으면(다른 사람이 띄운 스택) 그대로 쓰고, 없으면 sensors.launch.py 로 D435 만 띄웠다가 끝나면 끈다.
# - 결과: ~/front_cal/<이름>/000.png ~ 029.png, meta.json. 이름이 D 로 시작하면 깊이 15 장(depth.npz). 끝 줄 'R1_DONE <이름>'.

NAME=${1:?놓기 이름 (A1 A2 A3 A4)}
OUT=~/front_cal/$NAME
source ~/real_drive/env.sh
set -u
STARTED=0
if ! timeout 6 ros2 topic list 2>/dev/null | grep -qx /top_camera_front/color/image_raw; then
  echo "앞 D435 토픽 없음 → sensors.launch.py 로 띄움 (라이다 · 팔 카메라 끔)"
  setsid ros2 launch bringup sensors.launch.py use_lidar:=false use_arm_camera:=false > ~/front_cal/sensors_$NAME.log 2>&1 &
  LP=$!; STARTED=1; sleep 10
else
  echo "앞 D435 토픽 있음 → 띄워진 스택 사용"
fi
rm -rf "$OUT"
if [[ $NAME == D* ]]; then python3 ~/front_cal/depth_capture.py "$OUT" 15 top_camera_front; else python3 ~/front_cal/cal_capture.py "$OUT" 30 top_camera_front; fi
if [ $STARTED = 1 ]; then
  kill -INT -- -$LP 2>/dev/null; sleep 4; pkill -INT -f 'realsense2_camera_node.*top_camera' 2>/dev/null
fi
echo "R1_DONE $NAME"
