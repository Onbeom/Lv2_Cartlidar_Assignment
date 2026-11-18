source ~/real_drive/env.sh
export ROS_DOMAIN_ID=139 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
if pgrep -f 'arm_camera_node|realsense2_camera_node|ros2 launch|perception_container|perception_node' >/dev/null; then echo 실행중_중단; exit 1; fi
O=~/real_drive/p9a_out; rm -rf $O; mkdir -p $O
ros2 launch bringup sensors.launch.py use_lidar:=false front_serial:=261822072247 rear_serial:=261822079801 > $O/sensors.log 2>&1 &
A=$!
XA=$(ros2 pkg prefix opencr_hardware)/share/opencr_hardware/urdf/robot.urdf.xacro
python3 ~/real_drive/tf_stub.py $XA $O/rsp.yaml > $O/stub.log 2>&1 &
TS=$!
sleep 4; cat $O/stub.log | cut -c1-80
ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file $O/rsp.yaml > $O/rsp.log 2>&1 &
RS=$!
sleep 6
ros2 launch perception perception.launch.py use_survey:=false use_obstacle_stop:=false use_rear_obstacle:=false > $O/perception.log 2>&1 &
B=$!
sleep 24
echo "TF 경고 수 $(grep -cE 'TF_PENDING|Invalid frame' $O/perception.log)"
echo "== 검출기"; grep -oE "검출기: .*|해시 일치 [0-9a-f]{8}|engine_path 를 쓰지 못해.*|색 검출기로 대신.*" $O/perception.log | cut -c1-150 | sort -u | head -6
ST=$(ros2 service type /perception/set_mode); RT=$(ros2 topic info /perception/result 2>/dev/null | awk '/^Type/{print $2}'); echo "서비스 $ST · 결과 $RT"
PP=$(pgrep -f 'perception_node|perception_container' | head -1)
for spec in "ARM_ALIGN:2" "READ_TAG:4" "ARM+TAG:6" "SEARCH앞뒤+ARM:11"; do
  name=${spec%%:*}; m=${spec##*:}
  ros2 service call /perception/set_mode $ST "{mode: $m}" 2>&1 | grep -oE "success=[A-Za-z]+|message='[^']*'" | tr '\n' ' ' | cut -c1-110; echo
  sleep 3
  echo "== $name (mode $m)"
  timeout 20 tegrastats --interval 500 > "$O/t_$m.log" 2>&1 &
  T1=$!
  ( sleep 8; top -b -n 2 -d 5 -p $PP | awk '/^top -/{n++} n==2 && $1 ~ /^[0-9]+$/ {print "  인지 노드 CPU", $9, "· RSS", $6}' ) &
  T2=$!
  python3 ~/real_drive/perc_probe.py 20 $RT
  wait $T1 $T2 2>/dev/null; sleep 1
done
ros2 service call /perception/set_mode $ST "{mode: 0}" >/dev/null 2>&1
kill -0 $A && echo 센서_살아있음; kill -0 $B && echo 인지_살아있음
kill -TERM $TS $RS $B $A 2>/dev/null; sleep 5
pkill -TERM -f "component_container_mt.*perception_container"; pkill -TERM -f realsense2_camera_node; pkill -TERM -f arm_camera_node.py; sleep 2
echo "남은 프로세스 $(pgrep -fc 'component_container_mt|realsense2_camera_node|arm_camera_node.py|tf_stub|robot_state_publisher')"
echo "== 오류"; grep -cE "process has died|FATAL|Traceback" $O/perception.log $O/sensors.log
grep -E "ERROR|FATAL|process has died" $O/perception.log | cut -c1-150 | head -6
