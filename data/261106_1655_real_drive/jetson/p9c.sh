# P9c: 인지 수정 전후 비교 (관절 50 Hz). 변형: BASE(설치본 dev 4a2f8c3) · A(팔 TF 대기) · B(태그 검색 1 회) · AB(둘 다)
source ~/real_drive/env.sh
export ROS_DOMAIN_ID=139 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
SH=~/Lv2_Cartlidar_Assignment/workspace/ros2_ws/install/perception/share/perception/config
XA=$(ros2 pkg prefix opencr_hardware)/share/opencr_hardware/urdf/robot.urdf.xacro
R=~/real_drive/p9c_out; rm -rf $R; mkdir -p $R
if pgrep -f 'arm_camera_node|realsense2_camera_node|perception_container|robot_state_publisher' >/dev/null; then echo 실행중_중단; echo P9C_DONE; exit 1; fi
clean() { kill -TERM $TS $RS $B $A $RP 2>/dev/null; sleep 5; pkill -TERM -f "component_container_mt.*perception_container"; pkill -TERM -f realsense2_camera_node; pkill -TERM -f arm_camera_node.py; sleep 2; }
run() { # 이름 관절Hz 팔(real|replay) 모드들
  O=$R/$1; mkdir -p $O; RP=
  echo "#### $1 (관절 $2 Hz, 팔 $3) 시작 전 idle $(top -b -n 2 -d 1 | awk '/^%Cpu/{l=$8} END{print l}') % · perception $(ros2 pkg prefix perception)"
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
  echo "  불러온 라이브러리: $(grep -oE '/[^ ]*libperception[^ ]*\.so' /proc/$PP/maps | sort -u | head -2 | tr '\n' ' ')"
  echo "  arm_tf_wait_s: $(timeout 8 ros2 param get /perception_node arm_tf_wait_s 2>&1 | head -1)"
  echo "  D435 Node Is Up: 앞 $(grep -c 'top_camera_front\].*Node Is Up' $O/sensors.log) · 뒤 $(grep -c 'top_camera_rear\].*Node Is Up' $O/sensors.log) (둘 다 1 이어야 유효) · NOT found $(grep -c 'NOT found' $O/sensors.log) · HW 알림 $(grep -c 'XXX Hardware Notification' $O/sensors.log)"
  shift 3
  for spec in "$@"; do
    name=${spec%%:*}; m=${spec##*:}
    ros2 service call /perception/set_mode $ST "{mode: $m}" >/dev/null 2>&1; sleep 3
    echo "== $name (mode $m)"
    timeout 20 tegrastats --interval 500 > "$O/t_$m.log" 2>&1 & T1=$!
    ( sleep 8; top -b -n 2 -d 5 -p $PP | awk '/^top -/{n++} n==2 && $1 ~ /^[0-9]+$/ {print "  인지 노드 CPU", $9}' ) & T2=$!
    python3 ~/real_drive/perc_probe.py 20 $RT
    wait $T1 $T2 2>/dev/null; sleep 1
    python3 ~/real_drive/tsum.py "$O/t_$m.log"
  done
  timeout 5 ros2 topic echo --once --field tag_found /perception/result 2>/dev/null | head -1 | sed 's/^/  마지막 tag_found: /'
  ros2 service call /perception/set_mode $ST "{mode: 0}" >/dev/null 2>&1
  clean
  echo "  ARM_TF_PENDING $(grep -c ARM_TF_PENDING $O/perception.log) 줄 · 죽음 $(grep -cE 'process has died|FATAL|Traceback' $O/perception.log $O/sensors.log | sed 's#.*/##' | tr '\n' ' ')"
}
for V in BASE A B AB; do
  (
    if [ $V != BASE ]; then source ~/real_drive/fix_ws/$V/install/local_setup.bash; fi
    run ${V}_실장면 50 real ARM:2 TAG:4 ARM+TAG:6 탐색+ARM:11
    run ${V}_태그사진 50 replay TAG:4 ARM+TAG:6
  )
done
echo "남은 $(pgrep -fc 'component_container_mt|realsense2_camera_node|arm_camera_node.py|tf_stub|robot_state_publisher|arm_replay')"
echo P9C_DONE
