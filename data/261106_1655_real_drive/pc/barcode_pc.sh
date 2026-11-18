# 바코드 거리별 판독 스택 (PC): Arducam(팔 카메라 노드, 1280x720, 180° 돌림) + 인지 노드, 모드 READ_BARCODE(64), 창 없음.
#   D435 · 라이다 끔. 끝내기: bash barcode_pc.sh stop
H=/home/pa4/git/lv2_next_module/261106_1655_real_drive
source $H/pc/env_pc.sh
O=$H/pc/barcode_out
DEV=${ARM_DEV:-$(ls /dev/v4l/by-id/*Arducam*video-index0 2>/dev/null | head -1)}
XA=$(ros2 pkg prefix opencr_hardware)/share/opencr_hardware/urdf/robot.urdf.xacro
stop_all() { K=$H/pc/kill_procs.sh; bash $K TERM perception_container arm_camera_node.py tf_stub.py robot_state_publisher sensors.launch.py "launch perception"; sleep 3; }
if [ "${1:-}" = stop ]; then stop_all; echo BARCODE_STOPPED; exit 0; fi
[ -n "$DEV" ] || { echo "Arducam 장치를 못 찾음 (/dev/v4l/by-id/*Arducam*)"; exit 1; }
mkdir -p $O
ros2 launch ~/git/Lv2_Cartlidar_Assignment_lab/workspace/ros2_ws/src/bringup/launch/sensors.launch.py use_d435:=false use_lidar:=false use_arm_camera:=true arm_device:=$DEV > $O/sensors.log 2>&1 &
python3 $H/jetson/tf_stub.py $XA $O/rsp.yaml 50 > $O/tf_stub.log 2>&1 &
sleep 4
ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file $O/rsp.yaml > $O/rsp.log 2>&1 &
sleep 4
ros2 launch perception perception.launch.py use_survey:=false use_obstacle_stop:=false use_rear_obstacle:=false > $O/perception.log 2>&1 &
sleep 20
ros2 service call /perception/set_mode robot_interfaces/srv/SetPerceptionMode "{mode: 64}" 2>&1 | grep -o "success=[A-Za-z]*\|message='[^']*'"
echo "Arducam $DEV"
grep -m2 -i "error\|열지 못\|fail" $O/sensors.log
echo BARCODE_READY
