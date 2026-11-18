#!/usr/bin/env bash
# #119 확인 (Jetson): usb_cam 을 sensors.launch.py 와 같은 값(1280x720 mjpeg2rgb)으로 띄워 세그폴트 · 크기 · 주기 · CPU 를 본다.
#   실행: bash ~/real_drive/check_arm_cam.sh   (사람 얼굴이 안 찍히게 카메라를 바닥 쪽으로)
source "$HOME/real_drive/env.sh"
export ROS_DOMAIN_ID=139 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
DEV=${1:-}
if [ -z "$DEV" ]; then
  for p in $(ls -d /sys/class/video4linux/video* | sort -V); do
    [ "$(cat $p/index)" = 0 ] && grep -qE "USB 2.0 Camera|Arducam" $p/name && { DEV=/dev/$(basename $p); break; }
  done
fi
[ -z "$DEV" ] && { echo "Arducam 못 찾음"; v4l2-ctl --list-devices 2>/dev/null | head; exit 1; }
echo "장치 $DEV"; v4l2-ctl -d "$DEV" --list-formats-ext 2>/dev/null | grep -E "MJPG|1280x720" | head -3
L=$HOME/real_drive/arm_cam.log
ros2 run usb_cam usb_cam_node_exe --ros-args -r __ns:=/arm_camera -r __node:=arm_camera_driver -p video_device:=$DEV -p framerate:=30.0 -p image_width:=1280 -p image_height:=720 -p pixel_format:=mjpeg2rgb -p io_method:=mmap -p frame_id:=arm_camera_optical_frame -p camera_name:=arm_camera > "$L" 2>&1 &
P=$!
sleep 5
python3 "$HOME/real_drive/arm_cam_probe.py" 15 "$HOME/real_drive/arm_cam_sample.jpg" &
Q=$!
sleep 8
N=$(pgrep -f usb_cam_node_exe | head -1)
[ -n "$N" ] && top -b -n 2 -d 3 -p "$N" | tail -1 | awk '{print "usb_cam CPU " $9 " %"}'
wait $Q
if kill -0 $P 2>/dev/null; then echo "usb_cam 살아 있음"; else wait $P; echo "usb_cam 죽음 rc=$? (139 = 세그폴트)"; fi
pkill -INT -f usb_cam_node_exe; sleep 1
grep -iE "error|segfault|fatal" "$L" | head -5 | cut -c1-160
echo "그림: ~/real_drive/arm_cam_sample.jpg"
