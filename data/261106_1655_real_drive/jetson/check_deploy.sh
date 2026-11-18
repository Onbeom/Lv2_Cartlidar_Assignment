#!/usr/bin/env bash
# 배포 전 · 후 점검 (Jetson). 읽기만 한다. 설치 · 수정 · sudo 없음. apt 는 -s(모의)만 쓴다.
#   실행: bash ~/real_drive/check_deploy.sh 2>&1 | tee ~/real_drive/check_result.txt
set -u
REPO=$HOME/Lv2_Cartlidar_Assignment
WS=$REPO/workspace/ros2_ws
ENGINE=$HOME/yolo_D/deploy
line() { echo; echo "== $*"; }
export LC_ALL=C

line "보드"
uname -m; head -1 /etc/nv_tegra_release 2>/dev/null | cut -c1-70
nvpmodel -q 2>/dev/null | head -2
free -g | sed -n 2p; df -h "$HOME" | tail -1
id -nG | tr ' ' '\n' | grep -E '^(dialout|video|input|plugdev)$' | tr '\n' ' '; echo "(그룹)"

line "ROS"
ls /opt/ros/ 2>/dev/null
[ -f /opt/ros/lyrical/setup.bash ] && set +u && source /opt/ros/lyrical/setup.bash && set -u
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-없음} RMW=${RMW_IMPLEMENTATION:-기본}"

line "apt 패키지 (없음 = 설치 필요)"
PK="twist-mux joy joy-teleop teleop-twist-joy ros2-control ros2-controllers controller-manager hardware-interface xacro robot-state-publisher robot-localization cv-bridge nav2-amcl nav2-map-server navigation2 nav2-smac-planner nav2-mppi-controller usb-cam realsense2-camera coin-d4-driver topic-based-ros2-control cartographer-ros-msgs"
MISSING=""
for p in $PK; do
  n=ros-lyrical-$p
  if dpkg-query -W -f='${Status}' "$n" 2>/dev/null | grep -q "ok installed"; then echo "있음 $p"; else echo "없음 $p"; MISSING="$MISSING $n"; fi
done
for n in libzbar-dev liblapack-dev libopencv-dev v4l-utils python3-yaml; do
  if dpkg-query -W -f='${Status}' "$n" 2>/dev/null | grep -q "ok installed"; then echo "있음 $n"; else echo "없음 $n"; MISSING="$MISSING $n"; fi
done
line "없는 것 모의 설치 (실제로 설치하지 않음)"
if [ -n "$MISSING" ]; then
  echo "목록:$MISSING"
  apt-get install -s $MISSING 2>&1 | grep -E "^(E:|The following packages have unmet|[0-9]+ upgraded|[0-9]+ to upgrade)" | cut -c1-160
else echo "없는 패키지 없음"; fi

line "YOLO 엔진 (TensorRT)"
ls -la "$ENGINE"/yolo11n_task4_E3_* 2>&1 | awk '{print $5,$9}'
for f in "$ENGINE"/yolo11n_task4_E3_{480,384}_fp16.engine; do (cd "$(dirname "$f")" && sha256sum -c "$(basename "$f").sha256" 2>&1); done
ls /usr/include/aarch64-linux-gnu/NvInfer.h /usr/lib/aarch64-linux-gnu/libnvinfer.so 2>&1 | cut -c1-90

line "ONNX Runtime (강화학습 정책)"
ls "$HOME"/onnxruntime/lib/libonnxruntime.so "$HOME"/onnxruntime/include/onnxruntime_cxx_api.h 2>&1 | cut -c1-90

line "저장소 · 빌드"
if [ -d "$REPO/.git" ]; then
  git -C "$REPO" log -1 --format='%h %s' | cut -c1-80; git -C "$REPO" status -sb | head -3
  ls -d "$WS"/install/* 2>/dev/null | wc -l | sed 's/$/ 개 패키지 설치됨/'
  grep -rh "robot_policy: ONNX\|perception: TensorRT" "$WS"/log/latest_build/*/stdout.log 2>/dev/null | sort -u | cut -c1-110
else echo "저장소 없음: $REPO"; fi

line "장치 (로봇이 연결됐을 때만 보임)"
ls -l /dev/ttyACM* /dev/ttyUSB* /dev/arm_cam /dev/tfmini /dev/lidar_front /dev/lidar_rear /dev/input/js* 2>/dev/null | awk '{print $1,$3,$4,$NF}'
rs-enumerate-devices -s 2>/dev/null | cut -c1-70 | head -5
ls /etc/udev/rules.d/ 2>/dev/null | grep -E "opencr|sensors|realsense" | tr '\n' ' '; echo "(udev)"
echo; echo "== 끝. 이 파일을 알려 주세요: ~/real_drive/check_result.txt"
