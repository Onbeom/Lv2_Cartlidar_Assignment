#!/usr/bin/env bash
# PC 에서 실행. Jetson 조건을 먼저 보고, 맞을 때만 팔 OpenCR 읽기 · RSP · 팔 카메라 · 인지를 띄워 태그 퍽을 읽는다.
# bringup 을 쓰지 않고 이 스크립트는 OpenCR 에 아무것도 쓰지 않는다. 토크가 필요하면 팀 opencr_arm_tool.py 로 먼저 켠다.
# 사용: --j1_offset 3.1477 (joint1 영점 수리 전 임시 값, 기본 0)
# 사용: bash run_check.sh [--preflight] [--tag 번호]
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
SSH="ssh -S $HOME/.ssh/lv2_jetson.sock -o BatchMode=yes -o ConnectTimeout=8 cart11@cart11.local"
ONLY=0; TAG=-1; J1=0.0; OV=""; SEC=15
while [ $# -gt 0 ]; do case "$1" in --preflight) ONLY=1;; --tag) TAG=$2; shift;; --j1_offset) J1=$2; shift;; --overlay) OV=$2; shift;; --seconds) SEC=$2; shift;; *) echo "모르는 인자 $1"; exit 2;; esac; shift; done

$SSH true || { echo "중단: Jetson ssh 공유 연결이 없음"; exit 2; }
scp -o ControlPath=$HOME/.ssh/lv2_jetson.sock -q "$HERE/puck_probe.py" "$HERE/arm_state_bridge.py" cart11@cart11.local:/tmp/ || exit 2
$SSH 'bash -s' <<'R' || exit 2
WS=~/Lv2_Cartlidar_Assignment/workspace/ros2_ws
bad=0
echo "소스: $(git -C ~/Lv2_Cartlidar_Assignment log --oneline -1 | cut -c1-90)"
if grep -rqiE "rotate_?180" $WS/src/bringup/launch $WS/src/perception/scripts $WS/src/perception/launch $WS/src/perception/config; then echo "통과: 180° 회전 코드 있음"; else echo "중단: 180° 회전 코드가 아직 없음 (PR 병합 · 빌드 전)"; bad=1; fi
n=$(ls /dev/ttyACM* 2>/dev/null | wc -l); if [ "$n" = 1 ]; then echo "통과: OpenCR 포트 1개"; else echo "중단: OpenCR 포트 $n 개 (팔 보드 1개여야 함)"; bad=1; fi
[ -f /tmp/arm_state_bridge.py ] && { python3 /tmp/arm_state_bridge.py --check || bad=1; }
[ -e /dev/arm_cam ] && echo "통과: /dev/arm_cam" || { echo "중단: /dev/arm_cam 없음"; bad=1; }
busy=$(ps -eo args= | grep -E "^[^ ]*(python3? [^ ]*ros2 launch|component_container|ros2_control_node)|colcon build" | grep -v grep | cut -c1-100); if [ -n "$busy" ]; then echo "중단: 다른 ROS 실행이나 빌드가 돌고 있음: $busy"; bad=1; else echo "통과: 다른 ROS 실행 · 빌드 없음"; fi
exit $bad
R
[ "$ONLY" = 1 ] && { echo "사전 점검 통과"; exit 0; }

$SSH "TAG=$TAG J1=$J1 OV=$OV SEC=$SEC bash -s" <<'R'
source /opt/ros/lyrical/setup.bash; source ~/Lv2_Cartlidar_Assignment/workspace/ros2_ws/install/setup.bash
[ -n "$OV" ] && source "$OV/local_setup.bash" && echo "덧씌움: $(ros2 pkg prefix perception)"
export ROS_DOMAIN_ID=140 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 - <<'Y'
import subprocess, yaml, os
x = subprocess.check_output(['xacro', subprocess.check_output(['ros2', 'pkg', 'prefix', '--share', 'opencr_hardware'], text=True).strip() + '/urdf/robot.urdf.xacro'], text=True)
yaml.safe_dump({'robot_state_publisher': {'ros__parameters': {'robot_description': x}}}, open('/tmp/puck_rsp.yaml', 'w'))
Y
P=""
setsid nohup python3 /tmp/arm_state_bridge.py --j1_offset "$J1" > /tmp/puck_bridge.log 2>&1 < /dev/null & P="$P $!"
setsid nohup ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file /tmp/puck_rsp.yaml > /tmp/puck_rsp.log 2>&1 < /dev/null & P="$P $!"
setsid nohup ros2 launch bringup sensors.launch.py use_d435:=false use_lidar:=false > /tmp/puck_sensors.log 2>&1 < /dev/null & P="$P $!"
setsid nohup ros2 launch perception perception.launch.py use_survey:=false use_obstacle_stop:=false use_rear_obstacle:=false > /tmp/puck_perception.log 2>&1 < /dev/null & P="$P $!"
stop() { for p in $P; do kill -INT -- -$p 2>/dev/null; done; python3 -c "import time;time.sleep(5)"; for p in $P; do kill -KILL -- -$p 2>/dev/null; done; }   # 내가 띄운 묶음만 끈다
trap stop EXIT
timeout 170 python3 /tmp/puck_probe.py --tag "$TAG" --seconds "$SEC"; rc=$?
echo "== 인지 경고 상위"; grep -E "perception_node" /tmp/puck_perception.log | grep -E "WARN|ERROR" | sed -E 's/.*perception_node\]: //' | cut -c1-120 | sort | uniq -c | sort -rn | head -6
exit $rc
R
rc=$?
scp -o ControlPath=$HOME/.ssh/lv2_jetson.sock -q cart11@cart11.local:/tmp/puck_probe.json "$HERE/result.json" 2>/dev/null
echo "결과 파일: $HERE/result.json, 종료 코드 $rc"
exit $rc
