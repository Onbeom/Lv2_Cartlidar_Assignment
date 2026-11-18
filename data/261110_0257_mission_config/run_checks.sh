#!/usr/bin/env bash
# L1 ~ L5 (mission_bringup 실제 실행) · B1 B2 (bringup 구성만) · M1 (미션표 없이 plan mock 미션). 결과 = checks/
set +u
W="$(cd "$(dirname "$0")" && pwd)"
SRC=~/git/Lv2_Cartlidar_Assignment_missioncfg/workspace/ros2_ws/src
source /opt/ros/lyrical/setup.bash
source "$W/install/setup.bash"
export ROS_DOMAIN_ID=78 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
O="$W/checks"; rm -rf "$O"; mkdir -p "$O"
# 이 스크립트가 띄운 것만 정리: 각 프로세스를 setsid 로 새 그룹에 두고 그 그룹만 끈다
PG=()
bg() { setsid "$@" & PG+=($!); }
cleanup() { for g in "${PG[@]}"; do kill -INT -- -$g 2>/dev/null; done; sleep 3
            for g in "${PG[@]}"; do kill -KILL -- -$g 2>/dev/null; done; PG=(); true; }
params() {  # $1 이름 $2... launch 인자
  local n=$1; shift
  bg ros2 launch mission mission_bringup.launch.py "$@" > "$O/$n.launch.log" 2>&1
  for i in $(seq 1 30); do ros2 node list 2>/dev/null | grep -q '^/mission_node$' && break; sleep 1; done
  { for k in mission_source missions.tag_ids locations.names survey.enabled verify.arm_pose align.coarse_distance; do
      echo "$k: $(ros2 param get /mission_node $k 2>&1 | tr '\n' ' ')"; done; } > "$O/$n.params.txt"
  cleanup
}
params L1_default
params L4_tag source:=tag
timeout -s INT 20 ros2 launch mission mission_bringup.launch.py source:=foo > "$O/L5_foo.launch.log" 2>&1; echo "exit $?" >> "$O/L5_foo.launch.log"
python3 "$W/bringup_static.py" $SRC/bringup/launch/bringup.launch.py > "$O/B1_bringup_default.txt" 2>&1
python3 "$W/bringup_static.py" $SRC/bringup/launch/bringup.launch.py tag > "$O/B2_bringup_tag.txt" 2>&1
# M1
MIS=$W/install/mission/share/mission/config; MPL=$W/install/mission_planner/share/mission_planner/config
mkdir -p "$O/M1/log"
bg ros2 run mission mission_node --ros-args -r __node:=mission_node --params-file $MIS/mission.yaml --params-file $MIS/mission_plan.yaml \
  -p survey.enabled:=false -p log_dir:="$O/M1/log" > "$O/M1/mission.log" 2>&1
bg ros2 launch mission_planner planner.launch.py pucks_file:=$MPL/problem_q1_example.yaml log_dir:="$O/M1/log" > "$O/M1/planner.log" 2>&1
bg python3 "$SRC/mission_planner/test/mock_robot.py" --approach $MPL/approach_poses.yaml --out "$O/M1/mock_out.json" --timeout 300 > "$O/M1/mock.log" 2>&1
MOCK=${PG[-1]}
sleep 8
ros2 service call /mission/start std_srvs/srv/Trigger > "$O/M1/start.txt" 2>&1
wait $MOCK
tail -1 "$O/M1/mock.log"
cleanup
