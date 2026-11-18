#!/usr/bin/env bash
# perception_node 사진 요청 단독 시험. 사용: run_perception_only.sh <이름> [arm_cam_mock 인자...]
#   /perception/snapshot 를 한 번 보내고 5 s 뒤 로그 · 파일을 본다
set +u
W="$(cd "$(dirname "$0")" && pwd)"
NAME="${1:-P}"; shift
OUT="$W/p_$NAME"
source /opt/ros/lyrical/setup.bash
source "$W/install/setup.bash"
export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
rm -rf "$OUT"; mkdir -p "$OUT"
PER=$W/install/perception/share/perception/config
ros2 launch "$W/perception_mt.launch.py" remap:=false > "$OUT/perception.log" 2>&1 &
P1=$!
python3 "$W/arm_cam_mock.py" --mode_srv /perception/set_mode "$@" > "$OUT/armcam.log" 2>&1 &
P2=$!
for i in $(seq 1 30); do grep -q 'set_mode READ_TAG\|^.*$' "$OUT/armcam.log" 2>/dev/null; ros2 service list 2>/dev/null | grep -q /perception/set_mode && break; sleep 1; done
sleep 3
ros2 topic pub --once -w 1 /perception/snapshot std_msgs/msg/String "{data: '$OUT/images/01_test'}" > /dev/null
sleep 6
kill -INT $P1 $P2 2>/dev/null; sleep 3; kill -9 $P1 $P2 2>/dev/null; pkill -f "__node:=perception_container" 2>/dev/null
grep -h "사진" "$OUT/perception.log"; ls "$OUT/images" 2>/dev/null
true
