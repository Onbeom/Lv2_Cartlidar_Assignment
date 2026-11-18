#!/usr/bin/env bash
# 미로 재녹화 (Jetson, 11-15 준비). bringup(리모컨 · 위치 추정 · 라이다, 인지 · Nav2 끔) 을 띄우고 지도용 bag 을 녹화. Ctrl+C 로 끝 → bag 닫고 bringup 끔.
#   bash ~/real_drive/rec_maze.sh [이름=maze_1116] → ~/real_drive/bags/<이름>_<시각>/
# 전제: 미로 출발 자리, 팔 STOW(시작 전 arm_standby_check), 12 V 먼저, 바퀴 닿음, RC-100 켬, 다른 bringup · 남은 노드 없음.
# 11-14 와 달라진 점: #304 라이다 TF (앞 yaw π · 뒤 0), /tf · /tf_static 을 --topics 로 바로 녹화, 원래 스캔 + _filtered 둘 다, 키보드 처리 끔.
N=${1:-maze_1116}
source ~/real_drive/env.sh
O=~/real_drive/bags/${N}_$(date +%m%d_%H%M); mkdir -p ~/real_drive/bags
# 팔이 토크 켜진 채 HOLD(예: DRIVE 자세)면 하드웨어가 대기 검사를 건너뛰므로 SKIP_ARM_CHECK=1 로 넘길 수 있음 (11-16)
[ -n "${SKIP_ARM_CHECK:-}" ] || python3 ~/real_drive/arm_standby_check.py || { echo "팔 대기 범위 밖 — 녹화 안 함 (팔이 HOLD 면 SKIP_ARM_CHECK=1)"; exit 1; }
if pgrep -f "[r]os2 launch bringup" > /dev/null || pgrep -f "[l]ib/robot_state_publisher/robot_state_publisher|[l]ib/controller_manager/ros2_control_node|[m]ulti_coin_d4_node" > /dev/null; then
  echo "다른 bringup · 남은 노드가 있어 하지 않음"; exit 1
fi
PORTS=$(ls /dev/ttyACM* 2>/dev/null | paste -sd,)
setsid ros2 launch bringup bringup.launch.py use_teleop:=true rc100:=true use_perception:=false use_localization:=true use_amcl_node:=true use_nav2:=false use_planner:=false use_loc_lock:=false use_loc_gate:=false ports:=$PORTS > /tmp/rec_maze_bringup.log 2>&1 &
B=$!
trap 'kill -INT -- -$B 2>/dev/null; sleep 8; kill -KILL -- -$B 2>/dev/null; cp /tmp/rec_maze_bringup.log $O/ 2>/dev/null' EXIT
trap 'exit 130' TERM HUP
echo "bringup 띄움 (pgid $B), 60 s 뒤 확인"
sleep 60
NC=$(grep -c 'Configured and activated' /tmp/rec_maze_bringup.log)
hz() { timeout 6 ros2 topic hz $1 2>&1 | grep -o 'average rate: [0-9.]*' | tail -1 | awk '{print $3}'; }
SF=$(hz /scan_front); SR=$(hz /scan_rear); JY=$(hz /joy)
echo "컨트롤러 $NC/4 · /scan_front ${SF:-없음} · /scan_rear ${SR:-없음} · /joy ${JY:-없음} Hz"
[ "$NC" -ge 4 ] || { echo "컨트롤러가 다 안 떠 녹화 안 함"; exit 1; }
[ -n "$SF" ] && [ -n "$SR" ] || { echo "라이다가 안 들어와 녹화 안 함 (뒤 라이다 TF 를 고친 뒤 첫 녹화 — 링크 /dev/lidar_* 확인)"; exit 1; }
echo "▶ 정지 상태로 10 s 둔 뒤 리모컨으로 천천히 미로 전체를 돌고, 출발 자리로 돌아와 10 s 정지. 끝나면 Ctrl+C"
ros2 bag record -o $O --disable-keyboard-controls --topics /tf /tf_static /scan_front /scan_rear /scan_front_filtered /scan_rear_filtered /scan_merged \
  /odom /odom_icr /odometry/filtered /imu/data /joint_states /cmd_vel /joy < /dev/null || true
echo "녹화 끝: $O"; ros2 bag info $O 2>/dev/null | grep -E "Duration|Topic: /(tf|scan_front|scan_rear|odom|imu)" | cut -c1-110
