#!/bin/bash
# 11-06 Gazebo 확인 한 판(PC). 사용: run_one.sh <이름> stop|drive <loc_map yaml, 빈 값이면 기본>. 도메인 102 · GPU 잠금.
NAME=$1; MODE=$2; LOCMAP=$3
HERE=/home/pa4/git/lv2_next_module/261106_1117_loc_gazebo; OUT=$HERE/results/$NAME
[ -e $OUT ] && { echo "$OUT 이미 있음"; exit 1; }
mkdir -p $OUT
exec 8>/tmp/lv2_ros_domain_102.lock; flock -w 60 8 || { echo 도메인 잠금 실패; exit 1; }
exec 9>/tmp/lv2_gpu.lock; flock -w 3600 9 || { echo GPU 잠금 실패; exit 1; }
set +u; source /opt/ros/lyrical/setup.bash; source $HERE/ws/install/setup.bash
export ROS_DOMAIN_ID=102 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOG_DIR=$HERE/ros_logs GZ_PARTITION=pa4_26
MP=$HERE/ws/install/mission_planner/share/mission_planner/maps
ARGS="map:=$MP/map_amcl.yaml objects_map:=$MP/map_nav.yaml start_x:=0.69365 loc_event_log:=$OUT/loc_events.jsonl"
[ -n "$LOCMAP" ] && ARGS="$ARGS loc_map:=$LOCMAP"
echo "mode $MODE args $ARGS start $(date +%T)" > $OUT/info.txt
setsid ros2 bag record --use-sim-time -s mcap -o $OUT/bag --topics /clock /amcl_pose /initialpose /localization/status /localization/lock /odom /cmd_vel /tf > $OUT/record.log 2>&1 8>&- 9>&- & RP=$!
if [ "$MODE" = stop ]; then
  setsid ros2 launch navigation gazebo_sim.launch.py gui:=false rviz:=false nav:=false > $OUT/gazebo.log 2>&1 8>&- 9>&- & GP=$!
  sleep 85        # 팔 자세 약 25 s + 켜 둔 시간 60 s
  echo "nav_start $(date +%T)" >> $OUT/info.txt
  setsid ros2 launch navigation navigation.launch.py use_sim_time:=true odom_topic:=/odom cmd_vel_out:=/cmd_vel $ARGS > $OUT/launch.log 2>&1 8>&- 9>&- & LP=$!
  sleep 150
else
  setsid ros2 launch navigation gazebo_sim.launch.py gui:=false rviz:=false use_loc_monitor:=true $ARGS > $OUT/gazebo.log 2>&1 8>&- 9>&- & GP=$!; LP=
  for i in $(seq 240); do grep -q 'lifecycle_manager_navigation.*Managed nodes are active' $OUT/gazebo.log && break; sleep 1; done
  echo "nav_active_after_s $i $(date +%T)" >> $OUT/info.txt
  sleep 5
  timeout -s INT 1500 /usr/bin/python3 /home/pa4/git/lv2_next_module/261031_1325_nav2/test/run_goals.py --goals $HERE/goals_stage12.json --out $OUT/run_goals.json --frame map > $OUT/run_goals.log 2>&1; echo "run_goals_exit $?" >> $OUT/info.txt
fi
echo "end $(date +%T)" >> $OUT/info.txt
kill -TERM -$RP 2>/dev/null; for i in $(seq 40); do kill -0 $RP 2>/dev/null || break; sleep 0.5; done
for p in $LP $GP; do kill -INT -$p 2>/dev/null; done
sleep 10
for p in $RP $LP $GP; do kill -KILL -$p 2>/dev/null; done
pkill -KILL -f "gz sim.*dice_mission" ; pkill -KILL -f "[g]z-sim" ; sleep 3
flock -u 9
echo "[끝] $NAME" | tee $OUT/done.txt
