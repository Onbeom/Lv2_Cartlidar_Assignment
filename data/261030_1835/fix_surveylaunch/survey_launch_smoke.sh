#!/usr/bin/env bash
# perception.launch.py 가 B0 노드를 띄우는지 확인 (PC 도구 확인, domain 121, 30초 안)

B=/tmp/claude-1000/-home-pa4-git-lv2-next-module/361a5b23-e559-4019-8fe6-b8fd742202c9/scratchpad/b_survey
OUT=$B/smoke
mkdir -p $OUT
source /opt/ros/lyrical/setup.bash
source $B/install/setup.bash
export ROS_DOMAIN_ID=121 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
exec 9>/tmp/lv2_ros121.lock
flock -n 9 || { echo "domain 121 사용 중"; exit 1; }
setsid ros2 launch perception perception.launch.py survey_output:=$OUT/puck_survey.json > $OUT/launch.log 2>&1 &
LP=$!
sleep 8
ros2 node list > $OUT/nodes.txt 2>&1
ros2 param get /puck_survey tagged_min_confidence > $OUT/param_conf.txt 2>&1
ros2 param get /puck_survey output_path > $OUT/param_out.txt 2>&1
ros2 param get /puck_survey world_frame > $OUT/param_frame.txt 2>&1
timeout 5 ros2 topic echo --once /perception/puck_survey std_msgs/msg/String > $OUT/survey_msg.txt 2>&1
ros2 service call /perception/survey/finish std_srvs/srv/Trigger > $OUT/finish.txt 2>&1
kill -INT -- -$LP 2>/dev/null; sleep 3; kill -KILL -- -$LP 2>/dev/null
wait $LP 2>/dev/null
echo "--- nodes"; cat $OUT/nodes.txt
echo "--- params"; cat $OUT/param_conf.txt $OUT/param_out.txt $OUT/param_frame.txt
echo "--- msg"; head -c 300 $OUT/survey_msg.txt; echo
echo "--- finish"; tail -3 $OUT/finish.txt
