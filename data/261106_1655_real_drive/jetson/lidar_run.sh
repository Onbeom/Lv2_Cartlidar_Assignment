#!/usr/bin/env bash
# 라이다 방향 시험 (Jetson, 11-15). bringup 을 띄우고 lidar_dir_test.py(엔터로 한 자리씩)를 돌린 뒤 bringup 을 끔.
#   bash ~/real_drive/lidar_run.sh → ~/real_drive/lidar_out/<시각>/test.txt (끝에 '단계별 요약')
# 전제: 12 V 먼저 · 팔 STOW 또는 HOLD · 다른 bringup · 남은 노드 없음 · 로봇 둘레 60 cm 안에 물건 없음 (사람은 상자를 댈 때만).
# 사람: 화면이 말하는 자리를 상자로 가리고 엔터 (6 자리). 판정은 요약 줄 (가린 쪽 라이다 반응 · 맞으면 각 ±30°).
source ~/real_drive/env.sh
O=~/real_drive/lidar_out/$(date +%m%d_%H%M); mkdir -p $O
PORTS=$(ls /dev/ttyACM* 2>/dev/null | paste -sd,)
ARGS="use_teleop:=false use_perception:=false use_localization:=true use_amcl_node:=true use_nav2:=false use_planner:=false use_loc_lock:=false use_loc_gate:=false ports:=$PORTS"
{ echo "설치본: $(git -C ~/Lv2_Cartlidar_Assignment log --oneline -1)"; echo "시작 $(date)"; echo "인자: $ARGS"; echo "링크: $(ls -l /dev/lidar_front /dev/lidar_rear 2>&1 | awk '{print $(NF-2), $NF}' | tr '\n' ' ')"; } | tee $O/summary.txt
if pgrep -f "ros2 launch bringup" > /dev/null; then
  echo "다른 bringup 이 떠 있어 하지 않음: $(pgrep -af 'ros2 launch bringup' | cut -c1-100)" | tee -a $O/summary.txt; exit 1
fi
if pgrep -f "lib/robot_state_publisher/robot_state_publisher|lib/controller_manager/ros2_control_node|opencr_hardware/rc100_joy.py|realsense2_camera_node|multi_coin_d4_node" > /dev/null; then
  echo "bringup 없이 남은 노드가 있어 하지 않음 (끈 뒤 다시):" | tee -a $O/summary.txt
  ps -eo pid,pgid,lstart,args | grep -E "robot_state_publisher|ros2_control_node|rc100_joy|realsense2_camera_node|multi_coin_d4_node" | grep -v grep | cut -c1-140 | tee -a $O/summary.txt; exit 1
fi
setsid ros2 launch bringup bringup.launch.py $ARGS > $O/bringup.log 2>&1 &
B=$!
trap 'kill -INT -- -$B 2>/dev/null; sleep 10; kill -KILL -- -$B 2>/dev/null; exit 130' INT TERM   # 11-15: Ctrl+C 로 스크립트만 끝나 bringup 이 남던 것
echo "bringup 띄움 (pgid $B), 45 s 기다림 — 아직 상자를 대지 마세요"
sleep 45
for k in /scan_front /scan_rear; do
  echo "$k $(timeout 15 ros2 topic hz $k 2>&1 | grep 'average rate' | tail -1)" | tee -a $O/summary.txt
done
python3 -u ~/real_drive/lidar_dir_test.py $O 2>&1 | tee $O/test.txt   # 엔터로 한 자리씩 (11-15 사용자 요청)
kill -INT -- -$B 2>/dev/null; sleep 10; kill -TERM -- -$B 2>/dev/null; sleep 3; kill -KILL -- -$B 2>/dev/null
echo "LIDAR 끝 $(date +%H:%M:%S) — 기록 $O" | tee -a $O/summary.txt; printf '\a'
