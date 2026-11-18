#!/usr/bin/env bash
# 부모가 사라져 남은 ROS 노드(PPID 1, 내 계정, bringup 설치본 · /opt/ros 실행 파일) 끄기 (Jetson). bash ~/real_drive/cleanup_orphans.sh
# ros2 데몬은 남김. 끄기 전에 목록을 보여 주고 엔터를 기다림. pkill -f 를 쓰지 않음(부른 셸까지 죽는 일이 있었음).
L=$(ps -u "$USER" -o pid=,ppid=,args= | awk '$2==1' | grep -E "/opt/ros/lyrical/lib/|Lv2_Cartlidar_Assignment/workspace/ros2_ws/install/" | grep -v ros2-daemon | awk '{print $1}')
if [ -z "$L" ]; then echo "남은 노드 없음"; exit 0; fi
echo "끌 노드 $(echo $L | wc -w) 개:"; ps -o pid,lstart,args -p $(echo $L | tr ' ' ',') | cut -c1-130
read -r -p "엔터 = 끄기, Ctrl+C = 그만 " _
kill -INT $L 2>/dev/null; sleep 5; kill -TERM $L 2>/dev/null; sleep 3; kill -KILL $L 2>/dev/null
echo "--- 아직 쓰는 장치(없으면 비어 있음)"
for p in /dev/video* /dev/ttyACM* /dev/ttyUSB*; do f=$(fuser "$p" 2>/dev/null); [ -n "$f" ] && echo "$p $f"; done
echo 끝
