#!/usr/bin/env bash
# P7a 정지 상태 전체 구성 부하 (Jetson, ssh): 로봇 전체 bringup(OpenCR 2 · 라이다 · D435 2 · Arducam · TFmini · EKF · AMCL · Nav2 · 미션 대기)
#   + 인지 모드 7(SEARCH_FRONT | ARM_ALIGN | READ_TAG, 미션 정밀 정렬 · 집기 때 가장 무거움) 을 정지 상태로 5 분.
#   bash ~/real_drive/p7a.sh [초=300] [bringup 추가 인자 …] → 결과 ~/real_drive/p7a_out/<시각>/ (summary.txt 가 요약)
# 전제: 팔 STOW(대기 자세 범위 안, 아니면 하드웨어가 시작 거부) · 12 V 켬 · 바퀴 막아 둠(움직임 명령은 보내지 않음) · 다른 bringup 없음.
#   조이스틱은 안 씀(use_teleop:=false). 지도는 bringup 기본(map_amcl.yaml). 끝나면 bringup 을 끔.
# 기록(판정은 해석 때, 'CPU 는 증상이 있을 때만' 원칙): tegrastats(CPU 코어 평균 · 코어별 · GPU · RAM · 온도), 프로세스별 CPU 상위,
#   인지 결과 나이(>300 ms 비율), 로그의 제어 주기 놓침 · CMD_TIMEOUT · LOST · 노드 죽음, 노드 목록 앞뒤 비교.
S=${1:-300}; shift; X="$*"   # 추가 인자 예: use_planner:=false (11-13 P7b)
source ~/real_drive/env.sh
[ -n "$P7_OVERLAY" ] && source $P7_OVERLAY/install/setup.bash   # 시험용 덧씌우기 빌드 (11-13 P7e: 인지 단계별 시간)
O=~/real_drive/p7a_out/$(date +%m%d_%H%M); mkdir -p $O
{ echo "설치본: $(git -C ~/Lv2_Cartlidar_Assignment log --oneline -1)"; echo "시작 $(date)"; echo "추가 인자: ${X:-없음}"; echo "덧씌우기: ${P7_OVERLAY:-없음} $(cat ${P7_OVERLAY:-/nonexist}/VERSION 2>/dev/null) · PERCEPTION_PROFILE=${PERCEPTION_PROFILE:-} · PERCEPTION_CV_THREADS=${PERCEPTION_CV_THREADS:-} · PERCEPTION_UNDISTORT_GPU=${PERCEPTION_UNDISTORT_GPU:-}"; echo "다른 bringup: $(pgrep -af bringup.launch | grep -v $$ | cut -c1-80 | tr "\n" " ")"; echo "발견 범위 ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-없음} · RMW=${RMW_IMPLEMENTATION:-기본}"; } | tee $O/summary.txt
if pgrep -f "ros2 launch bringup" > /dev/null; then   # 11-14: 팀원 bringup 이 떠 있는 동안 세 번이 컨트롤러 0/4 로 실패 → 재지 않고 끝냄 (남의 bringup 은 끄지 않음)
  echo "다른 bringup 이 떠 있어 재지 않음: $(pgrep -af 'ros2 launch bringup' | cut -c1-100)" | tee -a $O/summary.txt; exit 1
fi
# 11-15: 앞 bringup 이 남긴 고아 노드(옛 /robot_description 래치 · 카메라 점유)가 있으면 재지 않음
if pgrep -f "lib/robot_state_publisher/robot_state_publisher|lib/controller_manager/ros2_control_node|opencr_hardware/rc100_joy.py|realsense2_camera_node" > /dev/null; then
  echo "bringup 없이 남은 노드가 있어 재지 않음 (끈 뒤 다시):" | tee -a $O/summary.txt
  ps -eo pid,pgid,lstart,args | grep -E "robot_state_publisher|ros2_control_node|rc100_joy|realsense2_camera_node" | grep -v grep | cut -c1-140 | tee -a $O/summary.txt; exit 1
fi
if [ -n "$P7_ARM_DRIVER" ]; then   # 팔 카메라 드라이버 복사본으로 바꿔 잼 (11-13 P7d: 15 Hz). bringup 에는 use_arm_camera:=false 를 같이 줄 것
  I=~/Lv2_Cartlidar_Assignment/workspace/ros2_ws/install/perception/share/perception/config
  FASTDDS_DEFAULT_PROFILES_FILE=$I/fastdds_arm_camera.xml setsid python3 $P7_ARM_DRIVER --ros-args -r __ns:=/arm_camera -r __node:=arm_camera_driver \
    -p device:=/dev/arm_cam -p fps:=${P7_ARM_FPS:-15.0} -p width:=1280 -p height:=720 -p frame_id:=arm_camera_optical_frame \
    -p camera_info_url:=file://$I/arm_camera_1280x720.yaml > $O/arm_driver.log 2>&1 &
  AD=$!; echo "팔 카메라 드라이버 복사본: $P7_ARM_DRIVER fps ${P7_ARM_FPS:-15.0} (pid $AD)" | tee -a $O/summary.txt
fi
# 컨트롤러가 안 뜨면(11-13: spawner 가 controller_manager 서비스를 못 찾음, 5 번 중 2 번) /joint_states 가 없음 → bringup 을 한 번 다시 띄움
for try in 1 2 3 4; do   # 11-14: spawner 가 controller_manager 서비스를 못 잡는 일이 첫 판 6/9 → 4 번까지
  setsid ros2 launch bringup bringup.launch.py use_teleop:=false use_localization:=true use_nav2:=true $X > $O/bringup.log 2>&1 &   # 새 프로세스 그룹: 끌 때 노드까지 한꺼번에 (11-13 버그: & 로 띄운 launch 는 SIGINT 를 무시해 노드가 남았음)
  B=$!
  sleep 60
  NC=$(grep -c 'Configured and activated' $O/bringup.log)   # 11-13 P7d: /joint_states 는 나왔지만 diff_drive · imu 가 죽어 odom TF 가 없었음 → 4 개 모두 확인
  if [ "$NC" -ge 4 ] && timeout 30 ros2 topic echo --once /joint_states sensor_msgs/msg/JointState > /dev/null 2>&1; then
    echo "컨트롤러 확인: 4 개 활성 · /joint_states 나옴 ($try 번째 bringup)" | tee -a $O/summary.txt; break
  fi
  echo "컨트롤러 확인: 활성 $NC / 4 ($try 번째 bringup) — 다시 띄움" | tee -a $O/summary.txt
  cp $O/bringup.log $O/bringup_fail_$try.log
  kill -INT -- -$B 2>/dev/null; sleep 10; kill -TERM -- -$B 2>/dev/null; sleep 3; kill -KILL -- -$B 2>/dev/null
  if [ $try = 4 ]; then
    echo "네 번 다 컨트롤러가 안 떠 측정하지 않고 끝냄" | tee -a $O/summary.txt
    [ -n "$AD" ] && { kill -INT -- -$AD 2>/dev/null; sleep 3; kill -KILL -- -$AD 2>/dev/null; }
    exit 1
  fi
  sleep 10
done
ros2 node list 2>/dev/null | sort > $O/nodes_before.txt
echo "노드 $(wc -l < $O/nodes_before.txt) 개 (60 s 뒤)" | tee -a $O/summary.txt
# 인지 모드 7 (11-13 버그: 서비스 답이 안 오면 ros2 service call 이 끝없이 기다려 측정이 시작되지 않았음 → 20 s 제한 · 3 번)
MODE_OK=
for k in 1 2 3; do
  r=$(timeout 20 ros2 service call /perception/set_mode robot_interfaces/srv/SetPerceptionMode "{mode: 7}" 2>&1 | grep -o "success=[A-Za-z]*")
  echo "인지 모드 7 ($k 번째): ${r:-답 없음 (20 s)}" | tee -a $O/summary.txt
  [ "$r" = "success=True" ] && { MODE_OK=1; break; }
  sleep 5
done
if [ -z "$MODE_OK" ]; then
  echo "인지 모드를 못 바꿈 — 측정하지 않고 끔" | tee -a $O/summary.txt
  kill -INT -- -$B 2>/dev/null; sleep 10; kill -TERM -- -$B 2>/dev/null; sleep 3; kill -KILL -- -$B 2>/dev/null
  [ -n "$AD" ] && { kill -INT -- -$AD 2>/dev/null; sleep 3; kill -KILL -- -$AD 2>/dev/null; }
  exit 1
fi
sleep 5
ros2 run tf2_ros static_transform_publisher --frame-id idle_probe_a --child-frame-id idle_probe_b --ros-args -r __node:=idle_probe > /dev/null 2>&1 & IP=$!   # 빈 노드 하나(홀로 띄우면 0 %, 11-13): bringup 안에서 얼마 쓰는지
sleep 5
envof() { tr '\0' '\n' < /proc/$1/environ 2>/dev/null | grep -E '^ROS_AUTOMATIC_DISCOVERY_RANGE=|^RMW_IMPLEMENTATION=|^ROS_DOMAIN_ID=' | tr '\n' ' '; }
echo "환경: bringup(pid $B) [$(envof $B)] · idle_probe(pid $IP) [$(envof $IP)] (빈 칸 = 설정 없음)" | tee -a $O/summary.txt
python3 ~/real_drive/cpu_snap.py snap $O/cpu_a.json   # 측정 구간 전체의 프로세스별 CPU (11-13: top 한 번으로는 95 % 구성을 못 가름)
timeout $((S + 2)) tegrastats --interval 1000 > $O/tegrastats.log 2>&1 & TG=$!
( sleep $((S / 2)); top -b -n 2 -d 5 | awk '/^top -/{n++} n==2 && $1 ~ /^[0-9]+$/' | sort -k9 -nr | head -15 > $O/top.txt ) & TOPP=$!
( timeout 25 ros2 topic hz /arm_camera/camera_info 2>&1 | grep "average rate" | tail -1 | sed 's/^/팔 영상(camera_info, 영상과 같이 발행) /' > $O/arm_hz.txt ) &   # 인지가 실제로 쓴 수는 아래 '인지 결과 arm_range' Hz
( sleep 30; timeout 40 ros2 topic hz /scan_merged --window 300 2>&1 | grep "average rate" | tail -1 | sed 's/^/합친 스캔 \/scan_merged /' > $O/scan_hz.txt ) &   # 11-15: 위치 추정 C++ 판 기준 '/scan_merged 10 Hz 유지' (README '다음 측정 기준')
python3 ~/real_drive/perc_probe.py $S robot_interfaces/msg/PerceptionResult > $O/perc_probe.txt 2>&1
python3 ~/real_drive/cpu_snap.py snap $O/cpu_b.json
kill -INT $IP 2>/dev/null
wait $TG $TOPP   # bringup(백그라운드)은 기다리지 않음 (11-12 버그: 그냥 wait 는 bringup 까지 기다려 안 끝남)
ros2 node list 2>/dev/null | sort > $O/nodes_after.txt
{
  echo "== tegrastats"; python3 ~/real_drive/tsum.py $O/tegrastats.log
  echo "== 인지 결과"; cat $O/perc_probe.txt; cat $O/arm_hz.txt 2>/dev/null | sed "s/^/  /"
  cat $O/scan_hz.txt 2>/dev/null | sed "s/^/  /"
  echo "== 프로세스 CPU 상위 (top, 코어 하나 = 100)"; awk '{printf "  %s %s%%\n", $12, $9}' $O/top.txt
  echo "== 프로세스별 CPU (측정 구간 평균, 코어 하나 = 100)"; python3 ~/real_drive/cpu_snap.py diff $O/cpu_a.json $O/cpu_b.json | head -40
  echo "== Nav2 상태 (11-14: 다 뜨면 CPU 약 +10 점 → 같은 상태끼리 비교)"; echo "  Managed nodes are active: $(grep -c 'Managed nodes are active' $O/bringup.log) · Aborting bringup: $(grep -c 'Aborting bringup' $O/bringup.log) · 실패 노드: $(grep -o 'Failed to change state for node: [a-z_]*' $O/bringup.log | awk '{print $NF}' | sort -u | tr '\n' ' ')"
  echo "== 컨트롤러 (spawner)"; grep -o "Configured and activated [a-z_]*\|Failed to acquire lock after multiple attempts\|Could not contact service [^ ]*" $O/bringup.log | sort | uniq -c | sed 's/^/  /'
  echo "== 컨테이너에 담긴 노드"; grep -o "Loaded node '[^']*' in container '[^']*'" $O/bringup.log | sed 's/^/  /'
  echo "== 로그 경고 수"
  for p in "overrun\|주기 놓침\|missed its desired rate" "CMD_TIMEOUT" "LOST" "process has died\|exit code" "ERROR"; do echo "  '$p': $(grep -ci "$p" $O/bringup.log)"; done
  echo "== 사라진 노드"; comm -23 $O/nodes_before.txt $O/nodes_after.txt | sed 's/^/  /'
  echo "끝 $(date)"
} | tee -a $O/summary.txt
kill -INT -- -$B 2>/dev/null; sleep 10; kill -TERM -- -$B 2>/dev/null; sleep 3; kill -KILL -- -$B 2>/dev/null
[ -n "$AD" ] && { kill -INT -- -$AD 2>/dev/null; sleep 3; kill -KILL -- -$AD 2>/dev/null; grep -m1 "열림" $O/arm_driver.log | sed 's/^/팔 드라이버: /' | tee -a $O/summary.txt; }
pgrep -g $B >/dev/null && echo '주의: bringup 노드가 남음 (pgrep -g '$B')' || echo 'bringup 노드 모두 꺼짐'
echo "결과 $O/summary.txt (bringup 끔)"
