# P3: 카메라 3대 + 인지 노드 + 팔 실제 각도 TF(읽기만). 모드 READ_TAG | READ_MARKER | READ_BARCODE = 84. 끝내기: bash ~/real_drive/p3.sh stop
source ~/real_drive/env.sh
SH=~/Lv2_Cartlidar_Assignment/workspace/ros2_ws/install/perception/share/perception/config
XA=$(ros2 pkg prefix opencr_hardware)/share/opencr_hardware/urdf/robot.urdf.xacro
O=~/real_drive/p3_out
stop_all() { pkill -INT -f "[p]3_probe.py"; sleep 2; pkill -TERM -f "component_container_mt.*perception_container"; pkill -TERM -f "[r]ealsense2_camera_node"; pkill -TERM -f "[a]rm_camera_node.py"; pkill -TERM -f "[t]f_arm.py"; pkill -TERM -f "[r]obot_state_publisher"; pkill -TERM -f "[r]os2 launch"; sleep 3; }
if [ "${1:-}" = stop ]; then stop_all; echo P3_STOPPED; exit 0; fi
mkdir -p $O
ros2 launch bringup sensors.launch.py use_lidar:=false use_arm_camera:=true front_serial:=261822072247 rear_serial:=261822079801 > $O/sensors.log 2>&1 &
python3 ~/real_drive/tf_arm.py $XA $O/rsp.yaml > $O/tf_arm.log 2>&1 &
sleep 4
ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file $O/rsp.yaml > $O/rsp.log 2>&1 &
sleep 6
ros2 launch perception perception.launch.py use_survey:=false use_obstacle_stop:=false use_rear_obstacle:=false > $O/perception.log 2>&1 &
sleep 24
ros2 service call /perception/set_mode robot_interfaces/srv/SetPerceptionMode "{mode: 84}" 2>&1 | tail -n 1
echo "D435 Node Is Up: 앞 $(grep -c 'top_camera_front\].*Node Is Up' $O/sensors.log) · 뒤 $(grep -c 'top_camera_rear\].*Node Is Up' $O/sensors.log) · 팔 $(grep -c '열림' $O/sensors.log)"
python3 -u ~/real_drive/p3_probe.py $O/p3.jsonl > $O/probe.txt 2>&1 &
echo P3_READY
