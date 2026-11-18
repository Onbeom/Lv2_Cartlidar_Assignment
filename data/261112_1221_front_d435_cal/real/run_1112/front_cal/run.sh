#!/bin/bash
# 남은 실물 촬영 한 도구 (Jetson): bash ~/front_cal/run.sh <단계>
#   DT      주행 멈춤 확인 (1 초 뒤로 → 멈춤)
#   Y1 Y2   앞 D435 · 판 고정 · 자동 앞뒤 주행(drive_auto.py) 동안 360 장(약 45 s)   (앞 요 + 정지 구간 롤 · 피치)
#   R2_*    앞 D435 · 실제 장소 상자 · 10 장 (R2_none R2_0a R2_0b R2_45a R2_45b)  (measure_front 실물)
#   RY1 RY2 뒤 D435 · 판 고정 · 자동 앞뒤 주행 동안 360 장                  (뒤 요 + 정지 구간 롤 · 피치)
#   RD      뒤 D435 깊이 15 장 (판 · 상자 치움)                             (뒤 피치 교차 확인)
#   J1      place_box 처리 시간 (렌더 영상 80 장) — 판정: 카메라 3 대 + perception 노드 부하 상태
#   J1idle  같은 측정을 빈 상태로 (기록)
# 앞 · 뒤 D435 토픽이 없으면 sensors.launch.py 로 띄웠다가(라이다 · 팔 카메라 끔) 끝나면 끈다. 끝 줄 'RUN_DONE <단계>'.
NAME=${1:?단계 이름 (Y1 R2_0a RY RD J1 ...)}
cd ~/front_cal || exit 1
source ~/real_drive/env.sh
set -u
if [ "$NAME" = DT ]; then   # 멈춤 확인: 0.05 m/s 로 1 초 뒤로 → 멈춤
  python3 ~/front_cal/drive_auto.py test; echo "RUN_DONE DT"; exit 0
fi
if [ "$NAME" = J1 ] || [ "$NAME" = J1idle ]; then
  g++ -O3 -DNDEBUG -std=c++17 -I pb_src/include pb_src/src/place_box_check_main.cpp $(pkg-config --cflags --libs opencv4) -o place_box_check || exit 1
  OUTF=~/front_cal/${NAME}_result.txt; : > $OUTF
  if [ "$NAME" = J1 ]; then   # 판정: 카메라 3 대 + perception 노드를 띄운 부하 (검토 조건)
    pgrep -f realsense2_camera_node >/dev/null || { setsid ros2 launch bringup sensors.launch.py use_lidar:=false > ~/front_cal/sensors_J1.log 2>&1 & echo $! > /tmp/j1_s.pid; }
    pgrep -f perception_container >/dev/null || { setsid ros2 launch perception perception.launch.py use_survey:=false > ~/front_cal/perception_J1.log 2>&1 & echo $! > /tmp/j1_p.pid; }
    sleep 25
    echo "부하: realsense $(pgrep -fc realsense2_camera_node) · arm_camera $(pgrep -fc arm_camera_node) · perception_container $(pgrep -fc perception_container)" | tee -a $OUTF
  else
    echo "빈 상태 (기록)" | tee -a $OUTF
  fi
  (cd j1 && python3 j1_time.py ../place_box_check) | tee -a $OUTF
  uptime | tee -a $OUTF
  for f in /tmp/j1_p.pid /tmp/j1_s.pid; do [ -f $f ] && { kill -INT -- -$(cat $f) 2>/dev/null; rm -f $f; }; done
  sleep 4; echo "RUN_DONE $NAME"; exit 0
fi
case $NAME in
  Y[0-9]*|R2_*) CAM=top_camera_front ;;
  RY[0-9]*|RD)  CAM=top_camera_rear ;;
  *) echo "모르는 단계 $NAME"; exit 1 ;;
esac
STARTED=0
cleanup() { [ $STARTED = 1 ] && { kill -INT -- -$LP 2>/dev/null; sleep 4; pkill -INT -f 'realsense2_camera_node.*top_camera' 2>/dev/null; }; STARTED=0; }
trap 'echo "멈춤 (Ctrl-C)"; cleanup; exit 130' INT HUP TERM
if ! timeout 6 ros2 topic list 2>/dev/null | grep -qx /$CAM/color/image_raw; then
  echo "$CAM 토픽 없음 → sensors.launch.py 로 띄움 (라이다 · 팔 카메라 끔)"
  setsid ros2 launch bringup sensors.launch.py use_lidar:=false use_arm_camera:=false > ~/front_cal/sensors_$NAME.log 2>&1 &
  LP=$!; STARTED=1; sleep 10
else
  echo "$CAM 토픽 있음 → 띄워진 스택 사용"
fi
OUT=~/front_cal/$NAME; rm -rf "$OUT"
case $NAME in
  Y[0-9]*|RY[0-9]*)   # 찍는 동안 자동 주행 (키 안 누름): 3 s 멈춤 → 판에서 멀어지게 5 s → 3 s → 돌아오게 5 s → 3 s → 한 번 더 → 멈춤
         SIDE=front; [[ $NAME == RY* ]] && SIDE=rear
         python3 ~/front_cal/cal_capture.py "$OUT" 360 $CAM > ~/front_cal/capture_$NAME.log 2>&1 &
         CP=$!; sleep 3
         python3 ~/front_cal/drive_auto.py run $SIDE
         wait $CP; cat ~/front_cal/capture_$NAME.log ;;
  R2_*)  python3 ~/front_cal/cal_capture.py "$OUT" 10 $CAM ;;
  RD)    python3 ~/front_cal/depth_capture.py "$OUT" 15 $CAM ;;
esac
cleanup
echo "RUN_DONE $NAME"
