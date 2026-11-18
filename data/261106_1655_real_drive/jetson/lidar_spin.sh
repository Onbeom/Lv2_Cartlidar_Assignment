#!/usr/bin/env bash
# 라이다 찾기 (Jetson): 포트 하나만 라이다 드라이버로 켜서(모터가 돎) 어느 라이다인지 눈으로 가린다. 다른 bringup 이 없을 때.
#   bash lidar_spin.sh /dev/ttyUSB0 [초=90] → 앞 라이다 자리로만 켜고 /scan_front 가 나오는지 보고, 끝나면 끔
P=$1; S=${2:-90}
source ~/real_drive/env.sh
L=~/real_drive/lidar_spin_$(basename $P).log
ros2 launch bringup sensors.launch.py use_d435:=false use_arm_camera:=false use_lidar:=true \
  lidar_front_port:=$P lidar_rear_port:=/dev/no_such_port > $L 2>&1 &
B=$!
sleep 8
echo "== $P: $(date +%T) 켬, /scan_front:"
timeout 6 ros2 topic hz /scan_front 2>&1 | grep -m1 "average rate" || echo "  /scan_front 없음"
grep -m3 -E "lidar_front|Failed|wrong" $L | cut -c1-150
sleep $((S - 14))
kill -INT $B; sleep 4; kill -TERM $B 2>/dev/null; sleep 1
echo "== $P: $(date +%T) 끔"
