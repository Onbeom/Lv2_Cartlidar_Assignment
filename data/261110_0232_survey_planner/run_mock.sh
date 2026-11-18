#!/usr/bin/env bash
# survey → planner 한 번 실행 (mock). 사용: run_mock.sh <이름> [survey_pucks_mock 인자...]
set +u
W="$(cd "$(dirname "$0")" && pwd)"
NAME="${1:-A}"; shift
OUT="$W/mock_$NAME"
REPO=~/git/Lv2_Cartlidar_Assignment_verify/workspace/ros2_ws/src
source /opt/ros/lyrical/setup.bash
source "$W/install/setup.bash"
export ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
rm -rf "$OUT"; mkdir -p "$OUT/log"
MIS=$W/install/mission/share/mission/config
MPL=$W/install/mission_planner/share/mission_planner/config
PER=$REPO/perception/config
PUCKS="$OUT/log/pucks_survey.yaml"
ros2 run mission mission_node --ros-args -r __node:=mission_node --params-file $MIS/mission.yaml --params-file $MIS/mission_table.yaml \
  --params-file $MIS/mission_plan.yaml -p survey.pucks_out:="$PUCKS" -p log_dir:="$OUT/log" > "$OUT/mission.log" 2>&1 &
P1=$!
ros2 run perception_survey puck_survey --ros-args --params-file $PER/puck_survey.yaml -p output_path:="$OUT/puck_survey.json" > "$OUT/survey.log" 2>&1 &
P2=$!
ros2 launch mission_planner planner.launch.py pucks_file:="$PUCKS" log_dir:="$OUT/log" > "$OUT/planner.log" 2>&1 &
P3=$!
python3 "$W/survey_pucks_mock.py" --truth "$W/truth.yaml" "$@" > "$OUT/helper.log" 2>&1 &
P4=$!
python3 "$REPO/mission_planner/test/mock_robot.py" --approach $MPL/approach_poses.yaml --box_shift 0,0 --box_rot 0 \
  --out "$OUT/mock_out.json" --timeout 300 > "$OUT/mock.log" 2>&1 &
MOCK=$!
for i in $(seq 1 60); do grep -q 'ready (mode' "$OUT/planner.log" && break; sleep 1; done   # 계획기 지도 읽기 (약 12 s) 뒤에 시작해야 신호를 받음
sleep 2
ros2 service call /mission/start std_srvs/srv/Trigger | tail -1
wait $MOCK
tail -1 "$OUT/mock.log"
kill -INT $P1 $P2 $P3 $P4 2>/dev/null; sleep 2; kill -9 $P1 $P2 $P3 $P4 2>/dev/null
true
