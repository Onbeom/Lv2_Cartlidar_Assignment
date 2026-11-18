#!/usr/bin/env bash
# 후방 장애물 재생 판정 실행 (PC, Isaac Sim 없이): robot_state_publisher(URDF) + 인지 launch(rear_obstacle 포함) + replay_rear.py
#   run_replay_rear.sh <render_rear 결과> <결과 폴더> <perception install setup.bash> [URDF]
# 설정은 설치된 perception/config 그대로(복사본을 결과 폴더에 남김). 도메인 88, 같은 PC 안 통신만. 끝나면 띄운 프로세스를 모두 끈다.
set -u
RENDER=${1:?render_rear 결과}
OUT=${2:?결과 폴더}
SETUP=${3:?perception install setup.bash}
URDF=${4:-$HOME/git/lv2_next_module/261027_1711/urdf/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf}
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$OUT"
[ -n "$(ls -A "$OUT")" ] && { echo "빈 폴더가 아님: $OUT"; exit 1; }
export ROS_DOMAIN_ID=126 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/.config/cyclonedds/cyclonedds_rx.xml"
set +u
source /opt/ros/lyrical/setup.bash
source "$SETUP"
set -u
cp -r "$(ros2 pkg prefix perception)/share/perception/config" "$OUT/config"
python3 - "$URDF" "$OUT/rsp_params.yaml" <<'PY'
import sys, yaml
yaml.safe_dump({'robot_state_publisher': {'ros__parameters': {
    'robot_description': open(sys.argv[1]).read(), 'use_sim_time': True}}}, open(sys.argv[2], 'w'))
PY
set -m
ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file "$OUT/rsp_params.yaml" > "$OUT/rsp.log" 2>&1 &
RSP=$!
ros2 launch perception perception.launch.py use_sim_time:=true config_dir:="$OUT/config" > "$OUT/perception_launch.log" 2>&1 &
LAUNCH=$!
cleanup() {
  kill -INT -"$LAUNCH" -"$RSP" 2>/dev/null
  for _ in $(seq 1 50); do
    kill -0 -"$LAUNCH" 2>/dev/null || kill -0 -"$RSP" 2>/dev/null || break
    sleep 0.1
  done
  kill -KILL -"$LAUNCH" -"$RSP" 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM
sleep 4
python3 "/home/pa4/git/lv2_next_module/261027_2058/replay_rear.py" --render "$RENDER" --out "$OUT" 2>&1 | tee "$OUT/replay.out"
