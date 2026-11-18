#!/usr/bin/env bash
# 할 순서 2 (Jetson): bringup sensors.launch.py 로 팔 Arducam(1280x720, 보정 파일) · D435 두 대를 띄워 토픽을 잰다. 라이다는 끈다.
#   실행: bash ~/real_drive/check_sensors.sh [앞 D435 시리얼] [뒤 D435 시리얼]
#   시리얼을 안 주면 rs-enumerate-devices 로 찾은 순서대로 넣는다 (어느 쪽이 앞인지는 저장한 그림으로 확인).
#   카메라가 사람 얼굴을 향하지 않게 둔다. 결과: ~/real_drive/sensors_out/
source "$HOME/real_drive/env.sh"
export ROS_DOMAIN_ID=139 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
O=$HOME/real_drive/sensors_out; rm -rf "$O"; mkdir -p "$O"
R=$O/result.txt
exec > >(tee "$R") 2>&1
echo "== $(date '+%F %T') · 소스 $(git -C ~/Lv2_Cartlidar_Assignment log -1 --format='%h %s' | cut -c1-70)"

DEV=
for p in $(ls -d /sys/class/video4linux/video* | sort -V); do
  [ "$(cat $p/index)" = 0 ] && grep -qE "USB 2.0 Camera|Arducam" $p/name && { DEV=/dev/$(basename $p); break; }
done
[ -z "$DEV" ] && { echo "Arducam 못 찾음"; v4l2-ctl --list-devices 2>/dev/null | head; exit 1; }
echo "Arducam $DEV ($(readlink -f /dev/arm_cam 2>/dev/null || echo '/dev/arm_cam 없음'))"

FRONT=${1:-}; REAR=${2:-}
if [ -z "$FRONT" ] || [ -z "$REAR" ]; then
  if command -v rs-enumerate-devices >/dev/null; then
    mapfile -t SN < <(rs-enumerate-devices -s 2>/dev/null | grep D435 | grep -oE '[0-9]{9,}')
  else
    mapfile -t SN < <(python3 -c "import pyrealsense2 as rs; [print(d.get_info(rs.camera_info.serial_number)) for d in rs.context().devices]" 2>/dev/null)
  fi
  FRONT=${FRONT:-${SN[0]:-}}; REAR=${REAR:-${SN[1]:-}}
fi
echo "D435 앞 ${FRONT:-?} · 뒤 ${REAR:-?}"
[ -z "$FRONT" ] || [ -z "$REAR" ] && { echo "D435 시리얼 두 개를 못 찾음 — 인자로 주세요"; lsusb | grep -i 8086; exit 1; }
echo "== USB 속도 (D435 는 5000M 이어야 함)"; lsusb -t | grep -iE "uvcvideo|5000M|480M" | head -12

L=$O/launch.log
ros2 launch ${SENSORS_LAUNCH:-bringup sensors.launch.py} use_lidar:=false front_serial:="$FRONT" rear_serial:="$REAR" \
  arm_device:="$DEV" > "$L" 2>&1 &   # 보정 파일은 launch 기본값(auto, #146)을 그대로 시험
P=$!
sleep 15
# CPU 는 구독이 있을 때 잰다(구독이 없으면 드라이버가 일을 덜 한다)
( sleep 8; PIDS=$(pgrep -f 'realsense2_camera_node|arm_camera_node.py' | paste -sd,)
  top -b -n 2 -d 4 -p "$PIDS" | awk '/^top -/{n++} n==2' > "$O/cpu_top.txt"
  for p in ${PIDS//,/ }; do echo "$p $(ps -o args= -p $p | grep -oE 'top_camera_[a-z]+|arm_camera' | head -1)"; done > "$O/cpu_names.txt" ) &
C=$!
python3 "$HOME/real_drive/sensor_probe.py" 20 "$O" reliable
wait $C
echo "== CPU (구독 중, top %, 100 = 코어 하나)"
grep -E "^%Cpu" "$O/cpu_top.txt" | cut -c1-90
awk 'NR==FNR{n[$1]=$2; next} $1 in n{print n[$1], $9}' "$O/cpu_names.txt" "$O/cpu_top.txt"
python3 "$HOME/real_drive/sensor_probe.py" 10 "$O" sensor_data
echo "== 단독 측정 (토픽 하나만 reliable 로, 8 s)"
for t in /top_camera_front/aligned_depth_to_color/image_raw /top_camera_rear/aligned_depth_to_color/image_raw; do
  python3 "$HOME/real_drive/sensor_probe.py" 8 "$O" reliable "$t" | grep -v '^=='
done
echo "== tf_static 의 카메라 프레임"
timeout 5 ros2 topic echo /tf_static --once --full-length 2>/dev/null | grep -oE "child_frame_id: \S+" | sort -u | head -12
if kill -0 $P 2>/dev/null; then echo "launch 살아 있음 (약 50 s)"; else wait $P; echo "launch 끝남 rc=$?"; fi
kill -INT $P 2>/dev/null; sleep 4; pkill -INT -f realsense2_camera_node; pkill -INT -f arm_camera_node.py; sleep 1
echo "== 로그 오류 · 경고 (앞 12 줄)"; grep -iE "error|fatal|traceback|segfault|warn|★" "$L" | cut -c1-180 | head -12
echo "== 팔 카메라 열림 줄"; grep -oE "열림 .*|요청 .*다름.*" "$L" | head -3
echo "그림: $O/*.jpg"
