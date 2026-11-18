# P5: D435 두 대(노출 80 · gain 128, sensors.launch 기본) + 인지 노드, 모드 READ_SIGNAL(32). 팔 카메라 · 라이다 끔.
#   관절은 0 고정 흉내(tf_stub, D435 는 차체 고정이라 관절과 무관). 끝내기: bash ~/real_drive/p5.sh stop
source ~/real_drive/env.sh
[ -n "${OVERLAY:-}" ] && source $OVERLAY/install/local_setup.bash   # 시험용 인지 오버레이 (예: OVERLAY=~/real_drive/fix_ws/SIG)
XA=$(ros2 pkg prefix opencr_hardware)/share/opencr_hardware/urdf/robot.urdf.xacro
O=~/real_drive/p5_out
stop_all() { pkill -INT -f "[p]5_probe.py"; sleep 2; pkill -TERM -f "component_container_mt.*perception_container"; pkill -TERM -f "[r]ealsense2_camera_node"; pkill -TERM -f "[t]f_stub.py"; pkill -TERM -f "[r]obot_state_publisher"; pkill -TERM -f "[r]os2 launch"; sleep 3; }
if [ "${1:-}" = stop ]; then stop_all; echo P5_STOPPED; exit 0; fi
mkdir -p $O
ros2 launch bringup sensors.launch.py use_lidar:=false use_arm_camera:=false front_serial:=${FRONT:-261822072247} rear_serial:=${REAR:-261822079801} > $O/sensors.log 2>&1 &
python3 ~/real_drive/tf_stub.py $XA $O/rsp.yaml 50 > $O/tf_stub.log 2>&1 &
sleep 4
ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file $O/rsp.yaml > $O/rsp.log 2>&1 &
sleep 6
ros2 launch perception perception.launch.py use_survey:=false use_obstacle_stop:=false use_rear_obstacle:=false > $O/perception.log 2>&1 &
sleep 24
ros2 service call /perception/set_mode robot_interfaces/srv/SetPerceptionMode "{mode: 32}" 2>&1 | grep -o "success=[A-Za-z]*"
echo "D435 Node Is Up: 앞 $(grep -c 'top_camera_front\].*Node Is Up' $O/sensors.log) · 뒤 $(grep -c 'top_camera_rear\].*Node Is Up' $O/sensors.log)"
timeout 8 ros2 param get /top_camera_front rgb_camera.exposure 2>&1 | head -n 1
timeout 8 ros2 param get /top_camera_front rgb_camera.gain 2>&1 | head -n 1
python3 -u ~/real_drive/p5_probe.py $O/p5.jsonl > $O/probe.txt 2>&1 &
echo P5_READY
