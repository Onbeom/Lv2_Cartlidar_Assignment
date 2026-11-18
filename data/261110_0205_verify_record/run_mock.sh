#!/usr/bin/env bash
# mock 미션 (주행 · 팔 · 인식 가짜) + verify 퍽 보조 노드로 재검증 기록 확인. 사용: run_mock.sh <ok|dup|unread>
set +u
W="$(cd "$(dirname "$0")" && pwd)"
SC="${1:-ok}"
OUT="$W/mock_$SC"
REPO=~/git/Lv2_Cartlidar_Assignment_verify/workspace/ros2_ws/src
source /opt/ros/lyrical/setup.bash
source "$W/install/setup.bash"
export ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
rm -rf "$OUT"; mkdir -p "$OUT/log"
MIS=$W/install/mission/share/mission/config
MPL=$W/install/mission_planner/share/mission_planner/config
ros2 run mission mission_node --ros-args -r __node:=mission_node --params-file $MIS/mission.yaml --params-file $MIS/mission_table.yaml \
  --params-file $MIS/mission_plan.yaml -p survey.enabled:=false -p log_dir:="$OUT/log" > "$OUT/mission.log" 2>&1 &
P1=$!
ros2 launch mission_planner planner.launch.py pucks_file:=$MPL/problem_q1_example.yaml log_dir:="$OUT/log" > "$OUT/planner.log" 2>&1 &
P2=$!
python3 "$W/verify_pucks_mock.py" --scenario "$SC" > "$OUT/helper.log" 2>&1 &
P3=$!
python3 "$REPO/mission_planner/test/mock_robot.py" --approach $MPL/approach_poses.yaml --box_shift 0,0 --box_rot 0 \
  --out "$OUT/mock_out.json" --timeout 300 > "$OUT/mock.log" 2>&1 &
MOCK=$!
sleep 8
ros2 service call /mission/start std_srvs/srv/Trigger | tail -1
wait $MOCK
tail -1 "$OUT/mock.log"
kill -INT $P1 $P2 $P3 2>/dev/null; sleep 2; kill -9 $P1 $P2 $P3 2>/dev/null
true
