#!/usr/bin/env bash
# 인지만 30 초 띄워 시작 로그를 본다 (Jetson, 카메라 없어도 됨). 다른 노드와 섞이지 않게 도메인을 나눈다.
source "$HOME/real_drive/env.sh"
L=$HOME/real_drive/boot.log
ROS_DOMAIN_ID=139 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST timeout -s INT 30 \
  ros2 launch perception perception.launch.py use_survey:=false use_obstacle_stop:=false use_rear_obstacle:=false > "$L" 2>&1
echo "== 검출기 줄"; grep -o "검출기: .*" "$L" | cut -c1-200
echo "== 해시"; grep -oE "해시 일치 [0-9a-f]+|해시 파일 없음|엔진 해시 불일치[^\"]*|엔진 해시 파일 형식이 틀림" "$L" | sort | uniq -c
echo "== 색 검출기 경고"; grep -cE "색 검출기로 대신|engine_path 를 쓰지 못해" "$L"
echo "== FATAL 수"; grep -c FATAL "$L"
