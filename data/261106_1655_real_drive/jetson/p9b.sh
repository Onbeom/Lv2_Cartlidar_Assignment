source ~/real_drive/env.sh
export ROS_DOMAIN_ID=139 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
SH=~/Lv2_Cartlidar_Assignment/workspace/ros2_ws/install/perception/share/perception/config
XA=$(ros2 pkg prefix opencr_hardware)/share/opencr_hardware/urdf/robot.urdf.xacro
R=~/real_drive/p9b_out; rm -rf $R; mkdir -p $R
clean() { kill -TERM $TS $RS $B $A $RP 2>/dev/null; sleep 5; pkill -TERM -f "component_container_mt.*perception_container"; pkill -TERM -f realsense2_camera_node; pkill -TERM -f arm_camera_node.py; sleep 2; }
run() { # 이름 관절Hz 팔(real|replay) 모드들
  O=$R/$1; mkdir -p $O; RP=
  echo "#### $1 (관절 $2 Hz, 팔 $3) 시작 전 idle $(top -b -n 2 -d 1 | awk '/^%Cpu/{l=$8} END{print l}') %"
  if [ "$3" = real ]; then ARM=true; else ARM=false; fi
  ros2 launch bringup sensors.launch.py use_lidar:=false use_arm_camera:=$ARM front_serial:=261822072247 rear_serial:=261822079801 > $O/sensors.log 2>&1 &
  A=$!
  if [ "$3" != real ]; then FASTDDS_DEFAULT_PROFILES_FILE=$SH/fastdds_arm_camera.xml python3 ~/real_drive/arm_replay.py ~/real_drive/tagframe.png $SH/arm_camera_1280x720.yaml > $O/replay.log 2>&1 & RP=$!; fi
  python3 ~/real_drive/tf_stub.py $XA $O/rsp.yaml $2 > $O/stub.log 2>&1 &
  TS=$!; sleep 4
  ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file $O/rsp.yaml > $O/rsp.log 2>&1 &
  RS=$!; sleep 6
  ros2 launch perception perception.launch.py use_survey:=false use_obstacle_stop:=false use_rear_obstacle:=false > $O/perception.log 2>&1 &
  B=$!; sleep 24
  ST=robot_interfaces/srv/SetPerceptionMode; RT=robot_interfaces/msg/PerceptionResult; PP=$(pgrep -f 'component_container_mt.*perception_container' | head -1)
  shift 3
  for spec in "$@"; do
    name=${spec%%:*}; m=${spec##*:}
    ros2 service call /perception/set_mode $ST "{mode: $m}" >/dev/null 2>&1; sleep 3
    echo "== $name (mode $m)"
    timeout 20 tegrastats --interval 500 > "$O/t_$m.log" 2>&1 & T1=$!
    ( sleep 8; top -b -n 2 -d 5 -p $PP | awk '/^top -/{n++} n==2 && $1 ~ /^[0-9]+$/ {print "  인지 노드 CPU", $9}' ) & T2=$!
    python3 ~/real_drive/perc_probe.py 20 $RT
    wait $T1 $T2 2>/dev/null; sleep 1
  done
  timeout 5 ros2 topic echo --once --field tag_found /perception/result 2>/dev/null | head -1 | sed 's/^/  마지막 tag_found: /'
  ros2 service call /perception/set_mode $ST "{mode: 0}" >/dev/null 2>&1
  clean
  echo "  죽음 $(grep -cE 'process has died|FATAL|Traceback' $O/perception.log $O/sensors.log | tr '\n' ' ')"
}
run R3_태그사진 50 replay TAG:4
echo "남은 $(pgrep -fc 'component_container_mt|realsense2_camera_node|arm_camera_node.py|tf_stub|robot_state_publisher|arm_replay')"
