#!/bin/bash
# 도구 확인(판정 아님): 장면 · 위치 추정 없이 지도 2개 + 정적 TF(mission_map → rotation_center) + planner_server 만 띄워 plan_only.py 로 경로를 받는다.
# 매개변수는 <params> 사본에서 global costmap 의 obstacle_layer 만 뺀 것(라이다가 없어서, 원본은 그대로). PC, ROS 도메인 102(flock).
#   환경 변수 MAP(선택, 11-03): 벽 지도 yaml(기본 config/map_mission_nav.yaml)
#   환경 변수 RC_X(선택, 11-02 B안): 출발 회전 중심 x(기본 0.8126 = 0.734 + 0.0786, B안 0.7998)
#   사용: plan_only.sh <이름> [params yaml] [goals json] [objects_map yaml] [plan_only.py 추가 인자]
set -o pipefail
NAME=$1; DOM=102
HERE=/home/pa4/git/lv2_next_module/261031_1325_nav2
PARAMS=${2:-$HERE/config/nav2_params_v27.yaml}
GOALS=${3:-$HERE/config/goals_seed1_v26.json}
OBJ=${4:-$HERE/config/map_objects_fine_posts_walls.yaml}
EXTRA=${5:-}
OUT=$HERE/results/$NAME
[ -e $OUT ] && { echo "$OUT 이미 있음"; exit 1; }
mkdir -p $OUT
exec 8>/tmp/lv2_ros_domain_$DOM.lock; flock -w 60 8 || { echo "도메인 $DOM 잠금 실패"; exit 1; }
set +u
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=$DOM ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOG_DIR=$HERE/test/ros_logs
/usr/bin/python3 - "$PARAMS" "$OUT/params_plan_only.yaml" <<'E'
import sys, yaml
d = yaml.safe_load(open(sys.argv[1]))
g = d['global_costmap']['global_costmap']['ros__parameters']
g['plugins'] = [p for p in g['plugins'] if p != 'obstacle_layer']
class D(yaml.SafeDumper):
    def ignore_aliases(self, data): return True
open(sys.argv[2], 'w').write(yaml.dump(d, Dumper=D, allow_unicode=True, sort_keys=False))
E
P=$OUT/params_plan_only.yaml
sha256sum $PARAMS $P $GOALS $OBJ $HERE/config/map_mission_nav.yaml $HERE/test/plan_only.py $HERE/nodes/geom.py > $OUT/source_sha256.txt
setsid ros2 run nav2_map_server map_server --ros-args -r __node:=map_server --params-file $P -p yaml_filename:=${MAP:-$HERE/config/map_mission_nav.yaml} > $OUT/map_server.log 2>&1 8>&- & M1=$!
setsid ros2 run nav2_map_server map_server --ros-args -r __node:=map_objects_server --params-file $P -p yaml_filename:=$OBJ > $OUT/map_objects_server.log 2>&1 8>&- & M2=$!
setsid ros2 run tf2_ros static_transform_publisher --x ${RC_X:-0.8126} --y 0.95 --yaw 3.14159265 --frame-id mission_map --child-frame-id rotation_center > $OUT/tf.log 2>&1 8>&- & T1=$!
setsid ros2 run nav2_planner planner_server --ros-args --params-file $P > $OUT/planner_server.log 2>&1 8>&- & PL=$!
setsid ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=lifecycle_manager_plan -p autostart:=true -p "node_names:=[map_server, map_objects_server, planner_server]" > $OUT/lifecycle.log 2>&1 8>&- & LM=$!
sleep 12
timeout -s INT 900 /usr/bin/python3 $HERE/test/plan_only.py --goals $GOALS --out $OUT/plan_only.json $EXTRA > $OUT/plan_only.log 2>&1; echo "plan_only_exit $?" > $OUT/exit_codes.txt
for p in $LM $PL $T1 $M2 $M1; do kill -INT -$p 2>/dev/null; done
sleep 4
for p in $LM $PL $T1 $M2 $M1; do kill -KILL -$p 2>/dev/null; done
echo "params $PARAMS goals $GOALS objects $OBJ extra $EXTRA" >> $OUT/exit_codes.txt
cat $OUT/plan_only.log
