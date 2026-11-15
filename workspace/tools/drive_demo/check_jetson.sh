#!/usr/bin/env bash
# 시연 전 Jetson 점검 (읽기만 — 아무것도 바꾸거나 움직이지 않음). 로봇 쪽 launch 3 개를 띄운 뒤 실행.
#   ./check_jetson.sh
set -u
set +u; source /opt/ros/lyrical/setup.bash; [ -f ~/Lv2_Cartlidar_Assignment/workspace/ros2_ws/install/setup.bash ] && source ~/Lv2_Cartlidar_Assignment/workspace/ros2_ws/install/setup.bash; set -u
ok() { echo "  ✓ $*"; }; bad() { echo "  ✗ $*"; FAIL=1; }; FAIL=0
echo "1) Nav2 패키지 (시연 노드)"
for p in nav2_controller nav2_behaviors nav2_regulated_pure_pursuit_controller nav2_lifecycle_manager nav2_msgs nav2_costmap_2d; do
  ros2 pkg prefix $p >/dev/null 2>&1 && ok $p || bad "$p 없음 → sudo apt install ros-lyrical-${p//_/-} (승인 필요)"; done
python3 -c "from nav2_msgs.action import FollowPath, Spin" 2>/dev/null && ok "python nav2_msgs" || bad "python nav2_msgs 못 읽음"
echo "2) 로봇 쪽 노드 · 토픽 (터미널 1 ~ 3)"
NODES=$( (ros2 node list 2>/dev/null; timeout 15 ros2 node list --no-daemon 2>/dev/null) | sort -u)
for n in twist_mux controller_manager amcl ekf_filter_node map_server lidar_node; do
  if echo "$NODES" | grep -q "/$n$"; then ok "노드 $n"
  elif pgrep -f "__node:=$n( |$)" >/dev/null || pgrep -f "lib/[a-z0-9_]*/$n( |$)" >/dev/null; then ok "노드 $n (프로세스 있음 — 노드 목록엔 늦게 보임)"
  else bad "노드 $n 안 보임"; fi; done
# 11-15 20:3x: 라이다 180° 는 URDF (PR #304, yaw π) 또는 scan_rotate180 중 하나만 — 실제 TF 로 판단 (둘 다 = 두 번 돌림, 둘 다 없음 = 안 돌림)
QF=$(timeout 6 ros2 run tf2_ros tf2_echo base_link laser_front 2>/dev/null | grep -m1 "Quaternion" | grep -o "\[.*\]")
ROTN=0; echo "$NODES" | grep -q "/scan_rotate180$" && ROTN=1
URDFFIX=$(python3 -c "import sys;q=[abs(float(v)) for v in sys.argv[1].strip('[]').split(',')];print(1 if q[1]>0.7 else 0 if q[0]>0.7 else '')" "$QF" 2>/dev/null)
case "$URDFFIX$ROTN" in
  10) ok "라이다 방향: URDF 보정 (yaw π) · scan_rotate 안 돌림" ;;
  01) ok "라이다 방향: 옛 URDF · scan_rotate180 이 돌림" ;;
  11) bad "라이다 두 번 돌림 — URDF 에 보정이 있는데 scan_rotate180 도 돎 → 터미널 2 를 SCAN_ROT=0 으로 다시" ;;
  00) bad "라이다 안 돌림 — 옛 URDF 인데 scan_rotate 가 그대로 냄 → 터미널 2 를 SCAN_ROT=1 로 다시" ;;
  *) echo "  ⚠ base_link → laser_front TF 를 못 읽음 ($QF) — 라이다 방향은 RViz 로 확인" ;;
esac
# 11-15: 위치 감시 (loc_monitor) 는 시연 때 끔 (잃었다고 보면 /initialpose 로 자세를 바꿔 넣음 — 11-14 1 회차). 떠 있으면 알려 줌
LOCMON=0; echo "$NODES" | grep -q "/loc_monitor$" && LOCMON=1
[ $LOCMON = 1 ] && echo "  ⚠ loc_monitor 가 떠 있음 — 시연 권장은 끄기: pkill -INT -f lib/localization/loc_monitor (출발 확인은 상태 OK 만 통과)" \
                || ok "loc_monitor 꺼짐 (출발 확인 = AMCL σ + RViz 눈 확인 y)"
echo "$NODES" | grep -qE "^/(controller_server|behavior_server|bt_navigator)$" && bad "Nav2 노드가 이미 있음 (use_nav2:=false 로)" || ok "다른 Nav2 없음"
for t in /scan_front /scan_rear /scan_merged /odometry/filtered /map /e_stop; do
  timeout 8 ros2 topic echo --once $t >/dev/null 2>&1 && ok "토픽 $t" || bad "토픽 $t 값이 안 옴 (8 s)"; done
if [ $LOCMON = 1 ]; then echo "   위치 감시: $(timeout 4 ros2 topic echo --once /localization/status 2>/dev/null | grep data | head -1)"; fi
AP=$(timeout 4 ros2 topic echo --once /amcl_pose 2>/dev/null | grep -m2 -E "^ +(x|y):" | tr -s " " | tr "\n" " ")
[ -n "$AP" ] && ok "AMCL 자세 /amcl_pose: $AP" || echo "  ⚠ /amcl_pose 가 안 옴 — 로봇이 멈춰 있으면 안 낼 수 있음. 출발 자리에 놓고 ./run_demo.sh real slow step init 0-0"
echo "   E-stop: $(timeout 4 ros2 topic echo --once /e_stop 2>/dev/null | grep data | head -1)  (true 면 리모컨 2+3)"
ros2 topic info /cmd_vel_nav 2>/dev/null | grep -q "Subscription count: [1-9]" && ok "/cmd_vel_nav 를 twist_mux 가 받음" || bad "/cmd_vel_nav 받는 쪽 없음"
echo "3) 위치 (map → base_link)"
timeout 6 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null | grep -m2 -E "Translation|RPY \(degree\)" | sed 's/^/   /'
echo "   계획 출발 자세 = (0.694, 0.950) · 180° (출발 구역, 앞이 −x 쪽)"
echo "4) 배터리"
for b in /battery/wheel /battery/arm; do V=$(timeout 6 ros2 topic echo --once $b 2>/dev/null | grep -m1 voltage)
  case "$V" in *nan*|"") bad "$b: ${V:-값 없음} — OpenCR 하드웨어가 안 켜짐 (터미널 1 의 부팅 점검 · FAULT → 배터리 켠 채 두 보드 RESET)";; *) echo "   $b: $V";; esac; done
[ $FAIL = 0 ] && echo "== 점검 통과" || echo "== ✗ 위 항목 확인 뒤 시연"
