#!/usr/bin/env bash
# #275 실물 확인 (Jetson, 11-15). kanichong 고침(bd1b4a0 · 6577f89 spawner 하나, b020f52 · f0759ad 단계로 띄움)이 든 설치본에서
# 11-14 실패 때와 같은 인자로 bringup 을 N 번 띄우고 판마다 잰다:
#   컨트롤러 활성 수(로그 'Configured and activated' · list_controllers), /rc100/raw · /joy Hz, /tf Hz, /tf 30 s 녹화 메시지 수
#   bash ~/real_drive/c275.sh [판=3] → ~/real_drive/c275_out/<시각>/summary.txt
# 전제: 12 V 먼저 켠 뒤 USB(아니면 보드 RESET) · 팔 STOW(대기 범위) · 바퀴 띄움 · RC-100 켬 · 다른 bringup 없음.
#   팔은 리셋 버튼(REARM)을 누르기 전까지 HOLD 라 움직이지 않음. 버튼 · 리모컨은 누르지 않음.
N=${1:-3}
source ~/real_drive/env.sh
O=~/real_drive/c275_out/$(date +%m%d_%H%M); mkdir -p $O
PORTS=$(ls /dev/ttyACM* 2>/dev/null | paste -sd,)
ARGS="use_teleop:=true rc100:=true use_perception:=false use_localization:=true use_amcl_node:=true use_nav2:=false use_planner:=false use_loc_lock:=false use_loc_gate:=false ports:=$PORTS"
{ echo "설치본: $(git -C ~/Lv2_Cartlidar_Assignment log --oneline -1)"; echo "시작 $(date)"; echo "인자: $ARGS"; } | tee $O/summary.txt
if pgrep -f "ros2 launch bringup" > /dev/null; then
  echo "다른 bringup 이 떠 있어 하지 않음: $(pgrep -af 'ros2 launch bringup' | cut -c1-100)" | tee -a $O/summary.txt; exit 1
fi
# 11-15 16:00: 앞 판 bringup 이 남긴 고아 robot_state_publisher(옛 /robot_description, 래치)를 새 ros2_control_node 가 받아 옛 대기 범위로 검사함
if pgrep -f "lib/robot_state_publisher/robot_state_publisher|lib/controller_manager/ros2_control_node|opencr_hardware/rc100_joy.py|realsense2_camera_node" > /dev/null; then
  echo "bringup 없이 남은 노드가 있어 하지 않음 (끈 뒤 다시):" | tee -a $O/summary.txt
  ps -eo pid,ppid,lstart,args | grep -E "robot_state_publisher|ros2_control_node|rc100_joy" | grep -v grep | cut -c1-140 | tee -a $O/summary.txt; exit 1
fi
hz() { timeout $2 ros2 topic hz $1 2>&1 | grep "average rate" | tail -1 | awk '{print $3}'; }
for i in $(seq 1 $N); do
  L=$O/bringup_$i.log
  setsid ros2 launch bringup bringup.launch.py $ARGS > $L 2>&1 &
  B=$!
  trap 'kill -INT -- -$B 2>/dev/null; sleep 10; kill -KILL -- -$B 2>/dev/null; exit 130' INT TERM   # 11-15: Ctrl+C 로 스크립트만 끝나 bringup 이 남던 것
  sleep 90
  NC=$(grep -c 'Configured and activated' $L)
  LC=$(timeout 15 ros2 control list_controllers 2>&1 | grep -c ' active')
  RAW=$(hz /rc100/raw 12); JOY=$(hz /joy 12); TF=$(hz /tf 12)
  rm -rf $O/tf_$i
  # Lyrical: 토픽은 --topics 로 (11-15 16:03 첫 3 판은 위치 인자라 녹화가 시작도 안 됨 → 0 개)
  # 11-15 19:29: 터미널 stdin 이면 rosbag2 키보드 처리 때문에 SIGINT 에도 안 끝남 → 키보드 끔 · stdin 끊음 · 10 s 뒤 KILL
  timeout -s INT -k 10 35 ros2 bag record -o $O/tf_$i --disable-keyboard-controls --topics /tf /tf_static < /dev/null > $O/bag_$i.log 2>&1
  TFN=$(ros2 bag info $O/tf_$i 2>/dev/null | grep "Topic: /tf |" | grep -o "Count: [0-9]*" | awk '{print $2}')
  STBY=$(grep -o "팔 대기 자세 확인[^\"]*\|팔이 대기 자세가 아님\|팔 HOLD[^\"]*건너뜀" $L | head -1)
  FAIL=$(grep -c "Failed to acquire lock\|Could not contact service" $L)
  echo "판 $i: 컨트롤러 로그 $NC/4 · list_controllers active $LC · spawner 실패 줄 $FAIL · /rc100/raw ${RAW:-없음} Hz · /joy ${JOY:-없음} Hz · /tf ${TF:-없음} Hz · /tf 30 s 녹화 ${TFN:-0} 개 · 팔 검사: ${STBY:-로그 없음}" | tee -a $O/summary.txt
  kill -INT -- -$B 2>/dev/null; sleep 10; kill -TERM -- -$B 2>/dev/null; sleep 3; kill -KILL -- -$B 2>/dev/null
  sleep 20
done
echo "C275 끝 $(date +%H:%M:%S)" | tee -a $O/summary.txt; printf '\a'
