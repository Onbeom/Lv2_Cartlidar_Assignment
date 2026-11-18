# P5 스택 (PC): D435 두 대(노출 80 · gain 128, 앞 261822072247 · 뒤 261822079801) + 인지 노드, 관절 0 고정 흉내, 창 없음.
#   jetson/p5.sh 와 같은 구성을 PC 에서. 끝내기: bash p5_pc.sh stop
H=/home/pa4/git/lv2_next_module/261106_1655_real_drive
source $H/pc/env_pc.sh
O=$H/pc/p5_out
XA=$(ros2 pkg prefix opencr_hardware)/share/opencr_hardware/urdf/robot.urdf.xacro
stop_all() { K=$H/pc/kill_procs.sh; bash $K INT p5_probe.py; sleep 2; bash $K TERM perception_container realsense2_camera_node tf_stub.py robot_state_publisher sensors.launch.py "launch perception"; sleep 3; }   # 자기 · 부른 셸은 안 건드림
if [ "${1:-}" = stop ]; then stop_all; echo P5_STOPPED; exit 0; fi
mkdir -p $O
ros2 launch ~/git/Lv2_Cartlidar_Assignment_lab/workspace/ros2_ws/src/bringup/launch/sensors.launch.py use_lidar:=false use_arm_camera:=false front_serial:=${FRONT:-261822072247} rear_serial:=${REAR:-261822079801} > $O/sensors.log 2>&1 &
python3 $H/jetson/tf_stub.py $XA $O/rsp.yaml 50 > $O/tf_stub.log 2>&1 &
sleep 4
ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file $O/rsp.yaml > $O/rsp.log 2>&1 &
sleep 6
ros2 launch perception perception.launch.py use_survey:=false use_obstacle_stop:=false use_rear_obstacle:=false > $O/perception.log 2>&1 &
sleep 20
ros2 service call /perception/set_mode robot_interfaces/srv/SetPerceptionMode "{mode: 32}" 2>&1 | grep -o "success=[A-Za-z]*"
echo "D435 Node Is Up: 앞 $(grep -c 'top_camera_front\].*Node Is Up' $O/sensors.log) · 뒤 $(grep -c 'top_camera_rear\].*Node Is Up' $O/sensors.log)"
timeout 8 ros2 param get /top_camera_front rgb_camera.exposure 2>&1 | head -n 1
timeout 8 ros2 param get /top_camera_front rgb_camera.gain 2>&1 | head -n 1
python3 -u $H/jetson/p5_probe.py $O/p5.jsonl > $O/probe.txt 2>&1 &
echo P5_READY
