# P5c: p5.sh 스택이 떠 있는 채로 인지 노드만 오버레이(인자, 기본 fix_ws/SIGDBG)로 바꿔 띄움. 카메라 · TF 는 그대로.
#   [P5c] 줄(5 s 마다): 고정 자리의 프레임별 판정 이유(켜짐 · 꺼짐 · 화면 밖 · 가림) 계수. 끝내기: bash ~/real_drive/p5.sh stop
source ~/real_drive/env.sh
source ~/real_drive/fix_ws/${1:-SIGDBG}/install/local_setup.bash   # 인자: 오버레이 이름 (기본 SIGDBG)
O=~/real_drive/p5_out
pkill -INT -f "[p]5_probe.py"; sleep 2
pkill -TERM -f "ros2 launch perception"; pkill -TERM -f "component_container_mt.*perception_container"; sleep 4
ros2 launch perception perception.launch.py use_survey:=false use_obstacle_stop:=false use_rear_obstacle:=false > $O/perception_${1:-SIGDBG}.log 2>&1 &
sleep 24
ros2 service call /perception/set_mode robot_interfaces/srv/SetPerceptionMode "{mode: 32}" 2>&1 | grep -o "success=[A-Za-z]*"
python3 -u ~/real_drive/p5_probe.py $O/p5c_${1:-SIGDBG}.jsonl > $O/probe_p5c.txt 2>&1 &
echo P5C_READY
