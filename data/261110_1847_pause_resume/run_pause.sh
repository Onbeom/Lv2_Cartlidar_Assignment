#!/usr/bin/env bash
# PAUSED 재개 · 정지 뒤 새 시작 모의 시험. 사용: run_pause.sh <이름> <pause|stop>:<사진 접두어 조각> ...
#   구성은 261110_1003_snapshot/run_mock.sh 와 같음 (설치본 · 가짜 노드 그 폴더 것) + pause_ctl.py
set +u
W=/home/pa4/git/lv2_next_module/261110_1003_snapshot; H="$(cd "$(dirname "$0")" && pwd)"
NAME="${1:-A}"; shift
OUT="$H/run_$NAME"; CTL_ARGS=("$@"); set --
REPO=~/git/Lv2_Cartlidar_Assignment_snapshot/workspace/ros2_ws/src
source /opt/ros/lyrical/setup.bash
source "$W/install/setup.bash"
export ROS_DOMAIN_ID=78 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
rm -rf "$OUT"; mkdir -p "$OUT"
LOG="$OUT/mission_log"            # 기록 루트 (bringup log_dir 과 같은 구조: 계획기 = $LOG/planner, 등록표 = $LOG/pucks_survey.yaml)
MIS=$W/install/mission/share/mission/config
MPL=$W/install/mission_planner/share/mission_planner/config
PER=$W/install/perception/share/perception/config
RP=/real_perception
ros2 run mission mission_node --ros-args -r __node:=mission_node --params-file $MIS/mission.yaml \
  --params-file $MIS/mission_plan.yaml -p log_dir:="$LOG" > "$OUT/mission.log" 2>&1 &
P1=$!
ros2 run perception_survey puck_survey --ros-args --params-file $PER/puck_survey.yaml -p output_path:="$OUT/puck_survey.json" > "$OUT/survey.log" 2>&1 &
P2=$!
ros2 launch mission_planner planner.launch.py pucks_file:="$LOG/pucks_survey.yaml" log_dir:="$LOG/planner" > "$OUT/planner.log" 2>&1 &
P3=$!
python3 "$W/survey_pucks_mock.py" --truth "$W/truth.yaml" "$@" > "$OUT/helper.log" 2>&1 &
P4=$!
ros2 launch "$W/perception_mt.launch.py" remap:=true > "$OUT/perception.log" 2>&1 &
P5=$!
python3 "$W/arm_cam_mock.py" > "$OUT/armcam.log" 2>&1 &
P6=$!
python3 "$REPO/mission_planner/test/mock_robot.py" --approach $MPL/approach_poses.yaml --box_shift 0,0 --box_rot 0 \
  --out "$OUT/mock_out.json" --timeout 400 > "$OUT/mock.log" 2>&1 &
MOCK=$!
for i in $(seq 1 60); do grep -q 'ready (mode' "$OUT/planner.log" && break; sleep 1; done   # 계획기 지도 읽기 뒤에 시작해야 신호를 받음
for i in $(seq 1 30); do grep -q 'set_mode READ_TAG' "$OUT/armcam.log" && break; sleep 1; done
sleep 2
ros2 service call /mission/start std_srvs/srv/Trigger | tail -1
python3 "$H/pause_ctl.py" "$LOG" "${CTL_ARGS[@]}" > "$OUT/ctl.log" 2>&1 &
CTL=$!
wait $MOCK; kill $CTL 2>/dev/null
tail -1 "$OUT/mock.log"
sleep 3   # 마지막 사진 쓰기
kill -INT $P1 $P2 $P3 $P4 $P5 $P6 2>/dev/null; sleep 3; kill -9 $P1 $P2 $P3 $P4 $P5 $P6 2>/dev/null; pkill -f "__node:=perception_container" 2>/dev/null; pkill -f "$W/install/mission_planner/lib" 2>/dev/null
true
