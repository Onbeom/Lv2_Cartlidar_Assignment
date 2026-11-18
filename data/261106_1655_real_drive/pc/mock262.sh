#!/usr/bin/env bash
# #262 mock 미션 (PC, 모의): 계획기 + mission_node + mock_robot.py. start_box · oblique 를 켜고 끔.
#   bash mock262.sh <판 이름> <start_box true|false> <oblique true|false> [mock_robot 추가 인자 …]
#   결과: ~/git/lv2_next_module/261106_1655_real_drive/pc/mock262_out/<판>/ (mission.log · planner.log · mock_out.json · log/)
# (set -u 는 ROS setup.bash 와 안 맞음)
NAME=$1; SB=$2; OB=$3; shift 3; MOCK_ARGS="$*"
WS=~/git/Lv2_Cartlidar_Assignment_mock262/workspace/ros2_ws
OUT=~/git/lv2_next_module/261106_1655_real_drive/pc/mock262_out/$NAME
rm -rf "$OUT"; mkdir -p "$OUT/log"
source /opt/ros/lyrical/setup.bash
source $WS/install/setup.bash
export ROS_DOMAIN_ID=93 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # 로봇 · 다른 시험과 안 섞이게
MPL=$WS/install/mission_planner/share/mission_planner/config
MIS=$WS/install/mission/share/mission/config
CTL=$WS/src/control/config
HERE=$WS/src/mission_planner/test
git -C $WS log --oneline -1 > "$OUT/version.txt"
echo "판 $NAME: start_box $SB · oblique $OB · mock $MOCK_ARGS · geom ${GEOM:-원본}" | tee "$OUT/summary.txt"
ros2 launch mission_planner planner.launch.py pucks_file:=$MPL/problem_q1_example.yaml log_dir:="$OUT/log/planner" > "$OUT/planner.log" 2>&1 &
P1=$!
# -p 로는 mission_plan.yaml 의 값이 덮이지 않아 (11-14 R0: survey 가 켜진 채) 마지막 params 파일로 덮어씀
KEY=mission_node   # mission_plan.yaml 의 노드 이름 칸
cat > "$OUT/override.yaml" <<YAML
$KEY:
  ros__parameters:
    survey:
      enabled: false
    start_box:
      enabled: $SB
    oblique:
      enabled: $OB
    log_dir: $OUT/log
YAML
ros2 run mission mission_node --ros-args -r __node:=mission_node --params-file ${GEOM:-$CTL/robot_geometry.yaml} --params-file $MIS/mission.yaml \
  --params-file $MIS/mission_table.yaml --params-file $MIS/mission_plan.yaml --params-file "$OUT/override.yaml" > "$OUT/mission.log" 2>&1 &
P2=$!
python3 "$HERE/mock_robot.py" --approach $MPL/approach_poses.yaml --out "$OUT/mock_out.json" --timeout 600 $MOCK_ARGS > "$OUT/mock.log" 2>&1 &
P3=$!
# 계획기 World 준비(PC 약 3 s)와 plan 서비스를 기다림
for i in $(seq 1 60); do ros2 service list 2>/dev/null | grep -q /mission_planner/plan && break; sleep 1; done
sleep 3
echo "[mock262] /mission/start" | tee -a "$OUT/summary.txt"
timeout 30 ros2 service call /mission/start std_srvs/srv/Trigger 2>&1 | tail -1 | tee -a "$OUT/summary.txt"
wait $P3
tail -2 "$OUT/mock.log" | tee -a "$OUT/summary.txt"
kill -INT $P1 $P2 2>/dev/null; sleep 2; kill -KILL $P1 $P2 2>/dev/null
kill -INT $(pgrep -f "lib/mission_planner/mission_planner_node|lib/mission/mission_node") 2>/dev/null
echo "[mock262] 끝 → $OUT"
