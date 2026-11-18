#!/usr/bin/env bash
# R2 · R3 모의 미션 한 번 (Q1 예제, mock_robot.py). PC 에서 실행.
#   run_mock.sh <결과 폴더> <start_box.enabled true|false> <start_box.restand true|false> [mock_robot.py 인자 ...]
#   OLD=1 이면 #247 빌드 (startbox worktree) 만 씀 (R3 바탕, restand 인자는 안 넘김). MISSION_ARGS="-p a:=b ..." 로 mission 인자 더함.
#   mock_robot.py · approach_poses.yaml 은 언제나 이 worktree (restand) 것 (같은 mock).
# 결과: <결과 폴더>/mock_out.json · log/ (mission_log.jsonl) · mission.log · planner.log · mock.log
set +u
NEW=~/git/Lv2_Cartlidar_Assignment_restand/workspace/ros2_ws
BASE=~/git/Lv2_Cartlidar_Assignment_startbox/workspace/ros2_ws
OUT="$1"; EN="$2"; RS="$3"; shift 3
source /opt/ros/lyrical/setup.bash
source $BASE/install/setup.bash
EXTRA="-p start_box.restand:=$RS"
if [ -n "$OLD" ]; then EXTRA=""; else source $NEW/install/setup.bash; fi
export ROS_DOMAIN_ID=${SB_DOMAIN:-142}            # 다른 세션 · 장비와 섞이지 않게
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
MIS=$(ros2 pkg prefix mission)/share/mission/config
MPL=$NEW/src/mission_planner/config
HERE=$NEW/src/mission_planner/test
rm -rf "$OUT"; mkdir -p "$OUT/log"
echo "mission $(ros2 pkg prefix mission) $EXTRA $MISSION_ARGS" > "$OUT/which.txt"
setsid ros2 run mission mission_node --ros-args -r __node:=mission_node --params-file $MIS/mission.yaml --params-file $MIS/mission_table.yaml \
  --params-file $MIS/mission_plan.yaml -p survey.enabled:=false -p log_dir:="$OUT/log" -p start_box.enabled:=$EN $EXTRA $MISSION_ARGS > "$OUT/mission.log" 2>&1 &
MPID=$!
setsid ros2 launch mission_planner planner.launch.py pucks_file:=$MPL/problem_q1_example.yaml log_dir:="$OUT/log/planner_run" > "$OUT/planner.log" 2>&1 &
PPID_=$!
python3 "$HERE/mock_robot.py" --approach $MPL/approach_poses.yaml --out "$OUT/mock_out.json" --timeout ${SB_TIMEOUT:-240} "$@" > "$OUT/mock.log" 2>&1 &
MOCK=$!
sleep 6
for i in $(seq 1 20); do
  r=$(timeout 10 ros2 service call /mission/start std_srvs/srv/Trigger 2>/dev/null | grep -o "success=True" | head -1)
  [ -n "$r" ] && break
  sleep 1
done
wait $MOCK
tail -1 "$OUT/mock.log"
for pg in $MPID $PPID_; do kill -INT -- -$pg 2>/dev/null; done; sleep 3
for pg in $MPID $PPID_; do kill -KILL -- -$pg 2>/dev/null; done
true
