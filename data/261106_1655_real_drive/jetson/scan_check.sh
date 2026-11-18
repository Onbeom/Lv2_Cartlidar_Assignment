#!/usr/bin/env bash
# 라이다 앞 · 뒤 배정 · TF 확인 (Jetson, 11-15 밤). 설치본 그대로 (#304 반영판). 허브 · 케이블을 옮긴 뒤 한 번.
#   bash ~/real_drive/scan_check.sh → ~/real_drive/scan_check_out/<시각>/ (tf.txt · test.txt)
# 전제: 팔 STOW 또는 HOLD · 다른 bringup · 남은 노드 없음. 사람: 화면이 말할 때 상자를 앞 · 뒤에 둠.
source ~/real_drive/env.sh
O=~/real_drive/scan_check_out/$(date +%m%d_%H%M); mkdir -p $O
PORTS=$(ls /dev/ttyACM* 2>/dev/null | paste -sd,)
ARGS="use_teleop:=false use_perception:=false use_localization:=true use_amcl_node:=true use_nav2:=false use_planner:=false use_loc_lock:=false use_loc_gate:=false ports:=$PORTS"
{ echo "설치본: $(git -C ~/Lv2_Cartlidar_Assignment log --oneline -1)"; echo "링크: $(ls -l /dev/lidar_front /dev/lidar_rear 2>&1 | awk "{print \$(NF-2), \$NF}" | tr "\n" " ")"; echo "시작 $(date)"; } | tee $O/summary.txt
if pgrep -f "[r]os2 launch bringup" > /dev/null || pgrep -f "[l]ib/robot_state_publisher/robot_state_publisher|[l]ib/controller_manager/ros2_control_node|[m]ulti_coin_d4_node" > /dev/null; then
  echo "다른 bringup · 남은 노드가 있어 하지 않음" | tee -a $O/summary.txt; ps -eo pid,pgid,args | grep -E "[r]os2 launch bringup|[r]obot_state_publisher|[r]os2_control_node|[m]ulti_coin" | cut -c1-120; exit 1
fi
setsid ros2 launch bringup bringup.launch.py $ARGS > $O/bringup.log 2>&1 &
B=$!
trap 'kill -INT -- -$B 2>/dev/null; sleep 8; kill -KILL -- -$B 2>/dev/null' EXIT
trap 'exit 130' INT TERM HUP
echo "bringup 띄움 (pgid $B), 45 s — 아직 상자를 두지 마세요"
sleep 45
for f in laser_front laser_rear; do
  echo "== base_link → $f" | tee -a $O/tf.txt
  timeout 6 ros2 run tf2_ros tf2_echo base_link $f 2>&1 | grep -m3 -E "Translation|RPY \(degree\)" | tee -a $O/tf.txt
done
python3 -u ~/real_drive/scan_x_check.py $O 2>&1 | tee $O/test.txt
echo "SCAN 확인 끝 $(date +%H:%M:%S) — 기록 $O" | tee -a $O/summary.txt; printf '\a'
