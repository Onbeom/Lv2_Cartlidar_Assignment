#!/usr/bin/env bash
# 비스듬히 서기 mock 미션 한 번 (주행 · 팔 · 인식 가짜 = mock_robot_oblique.py, 계획기는 진짜).
# 사용: run_mock.sh <이름> <oblique 0|1> <box_rot deg> <box_shift dx,dy> [base]
#   base = origin/dev 그대로 빌드한 mission (base_install) 로 돌림 (끈 상태 비교 기준)
set +u
W="$(cd "$(dirname "$0")" && pwd)"
NAME="$1"; OBL="${2:-0}"; ROT="${3:-0}"; SHIFT="${4:-0,0}"; WHICH="${5:-new}"
OUT="$W/run_$NAME"
WS=~/git/Lv2_Cartlidar_Assignment_oblique/workspace/ros2_ws
source /opt/ros/lyrical/setup.bash
source "$WS/install/setup.bash"
if [ "$WHICH" = base ]; then source "$W/base_install/setup.bash"; fi
# 도메인 87: 같은 PC 의 다른 세션이 77 에서 perception 재생을 돌려 /perception/place_box · TF 가 섞였음 (261112 첫 시도)
export ROS_DOMAIN_ID=87 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
rm -rf "$OUT"; mkdir -p "$OUT/log"
MIS=$(ros2 pkg prefix mission)/share/mission/config
MPL=$(ros2 pkg prefix mission_planner)/share/mission_planner/config
echo "mission: $(ros2 pkg prefix mission)  oblique=$OBL rot=$ROT shift=$SHIFT" > "$OUT/run.txt"
EXTRA=()
if [ "$WHICH" != base ]; then EXTRA=(-p oblique.enabled:=$([ "$OBL" = 1 ] && echo true || echo false)); fi
setsid ros2 run mission mission_node --ros-args -r __node:=mission_node --params-file $MIS/mission.yaml --params-file $MIS/mission_table.yaml \
  --params-file $MIS/mission_plan.yaml -p survey.enabled:=false -p log_dir:="$OUT/log" "${EXTRA[@]}" > "$OUT/mission.log" 2>&1 &
P1=$!
setsid ros2 launch mission_planner planner.launch.py pucks_file:=$MPL/problem_q1_example.yaml log_dir:="$OUT/log/planner" > "$OUT/planner.log" 2>&1 &
P2=$!
python3 "$W/mock_robot_oblique.py" --approach $MPL/approach_poses.yaml --box_shift="$SHIFT" --box_rot="$ROT" \
  --out "$OUT/mock_out.json" --timeout 300 > "$OUT/mock.log" 2>&1 &
MOCK=$!
sleep 8
ros2 service call /mission/start std_srvs/srv/Trigger | tail -1
wait $MOCK
tail -1 "$OUT/mock.log"
kill -INT -- -$P1 -$P2 2>/dev/null; sleep 2; kill -9 -- -$P1 -$P2 2>/dev/null
true
