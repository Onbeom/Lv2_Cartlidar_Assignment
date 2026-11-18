#!/usr/bin/env bash
# 라이다 점 보기 (11-15). 창은 PC 로 (ssh -X) 또는 Jetson 모니터로.
#   PC:     ssh -X cart11@cart11.local 'bash ~/real_drive/lidar_view.sh'
#   Jetson: bash ~/real_drive/lidar_view.sh   (모니터에 cart11 로 로그인해 있어야 함)
# bringup 이 이미 떠 있으면 창만 띄움. 없으면 bringup 을 띄우고, 창을 닫거나 Ctrl+C 하면 그 bringup 도 끔.
source ~/real_drive/env.sh
# bringup 을 새로 띄울 때만 팔 확인 (떠 있는 bringup 이 포트를 쓰는 동안 읽지 않음)
if ! pgrep -f "[r]os2 launch bringup" > /dev/null && ! python3 ~/real_drive/arm_standby_check.py; then
  echo "팔이 STOW 범위 밖 — 위 안내대로 옮긴 뒤 다시"; exit 1
fi
if [ -z "$DISPLAY" ]; then   # ssh -X 가 아니면 이 사용자의 로컬 화면
  U=$(id -u)
  for s in /tmp/.X11-unix/X*; do [ -S "$s" ] && [ "$(stat -c %u "$s")" = "$U" ] && { export DISPLAY=":${s##*/X}"; break; }; done
  for x in /run/user/$U/gdm/Xauthority $HOME/.Xauthority; do [ -f "$x" ] && { export XAUTHORITY=$x; break; }; done
fi
[ -z "$DISPLAY" ] && { echo "화면 없음: PC 에서 ssh -X 로 실행하거나 Jetson 모니터에 로그인하세요"; exit 1; }
echo "화면 DISPLAY=$DISPLAY"
B=
if pgrep -f "[r]os2 launch bringup" > /dev/null; then
  echo "떠 있는 bringup 을 그대로 씀: $(pgrep -af '[r]os2 launch bringup' | cut -c1-80)"
else
  if pgrep -f "[l]ib/robot_state_publisher/robot_state_publisher|[l]ib/controller_manager/ros2_control_node|[m]ulti_coin_d4_node" > /dev/null; then
    echo "bringup 없이 남은 노드가 있어 하지 않음 (끈 뒤 다시):"; ps -eo pid,pgid,args | grep -E "[r]obot_state_publisher|[r]os2_control_node|[m]ulti_coin_d4_node" | cut -c1-120; exit 1
  fi
  # 11-16 02:0x: '하드웨어 끔 + robot_state_publisher 따로' 판은 실물에서 laser TF 가 안 나와 되돌림 → 팔이 STOW 범위 안이어야 함 (arm_standby_check.py)
  PORTS=$(ls /dev/ttyACM* 2>/dev/null | paste -sd,)
  setsid ros2 launch bringup bringup.launch.py use_teleop:=false use_perception:=false use_localization:=true use_amcl_node:=true use_nav2:=false use_planner:=false use_loc_lock:=false use_loc_gate:=false ports:=$PORTS > /tmp/lidar_view_bringup.log 2>&1 &
  B=$!
  trap 'kill -INT -- -$B 2>/dev/null; sleep 8; kill -KILL -- -$B 2>/dev/null' EXIT
  trap 'exit 130' INT TERM HUP   # Ctrl+C · ssh 끊김에도 EXIT 정리가 돌게
  echo "bringup 띄움 (pgid $B, 로그 /tmp/lidar_view_bringup.log), 30 s 기다림 (하드웨어 · 컨트롤러 · 라이다 예열)"
  sleep 30
fi
python3 ~/real_drive/lidar_view.py "$@"
