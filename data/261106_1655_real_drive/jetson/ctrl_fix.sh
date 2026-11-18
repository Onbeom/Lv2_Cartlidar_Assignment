#!/usr/bin/env bash
# bringup 뒤 안 켜진 컨트롤러만 다시 켬 (11-14 미로 주행: spawner 가 서비스를 못 찾은 채 잠금을 쥐어 나머지가 실패, #275).
# base_bringup.launch.py 의 spawner 인자와 같게. bringup 이 떠 있고 하드웨어가 active 일 때 실행. 팀 설치본 · 설정은 안 바꿈.
source ~/Lv2_Cartlidar_Assignment/workspace/ros2_ws/install/setup.bash
C=~/Lv2_Cartlidar_Assignment/workspace/ros2_ws/install/opencr_hardware/share/opencr_hardware/config
hw=$(timeout 10 ros2 control list_hardware_components 2>/dev/null | grep -o "label=[a-z]*")
echo "하드웨어 $hw"
[ "$hw" = "label=active" ] || { echo "하드웨어가 active 가 아님 → bringup 기록을 먼저 봄(포트 · 팔 대기 자세)"; exit 1; }
active() { timeout 10 ros2 control list_controllers 2>/dev/null | grep -E "^$1 " | grep -q " active"; }
spawn() {   # 이름, 추가 인자...
  local n=$1; shift
  if active "$n"; then echo "$n 이미 active"; return; fi
  pkill -f "lib/controller_manager/spawner $n" && sleep 2   # 멈춘 spawner(잠금을 쥔 것)만
  timeout 60 ros2 run controller_manager spawner "$n" --controller-manager /controller_manager --param-file "$C/controllers_4wd.yaml" "$@" 2>&1 | tail -1
}
spawn joint_state_broadcaster
spawn imu_sensor_broadcaster --controller-ros-args "-r imu_sensor_broadcaster/imu:=/imu/data"
spawn diff_drive_controller --param-file "$C/drive_belt.yaml" --controller-ros-args "-r diff_drive_controller/cmd_vel:=/cmd_vel -r diff_drive_controller/odom:=/odom"
spawn arm_controller
timeout 10 ros2 control list_controllers 2>/dev/null
