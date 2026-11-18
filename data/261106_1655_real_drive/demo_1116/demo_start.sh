#!/bin/bash
# 시연 · 리허설 시작 (Jetson). bringup 을 띄우고, 미션 노드가 준비되면 /mission/start 를 대신 부른다.
#   쓰기: bash ~/real_drive/demo_start.sh            (bringup 인자를 뒤에 붙일 수 있음)
#   전제: 이 터미널에서 `ros2 launch bringup bringup.launch.py` 가 되는 환경 (ROS · 작업 공간 source 끝)
#   끄기: Ctrl-C (bringup 이 내려감). 로그: ~/real_drive/demo_logs/demo_<시각>.log
#   팀 코드는 바꾸지 않음. /mission/start 는 준비 확인이 모두 OK 일 때만 받아들여지므로 받아들여질 때까지 3 s 마다 다시 부른다.
mkdir -p ~/real_drive/demo_logs
LOG=~/real_drive/demo_logs/demo_$(date +%y%m%d_%H%M%S).log
echo "[demo_start] 로그 $LOG"
(
  sleep 5
  until ros2 service list 2>/dev/null | grep -qx /mission/start; do sleep 2; done
  echo "[demo_start] /mission/start 서비스 보임 → 시작 요청" | tee -a "$LOG"
  for i in $(seq 1 100); do
    out=$(timeout 10 ros2 service call /mission/start std_srvs/srv/Trigger 2>&1 | tail -1)
    echo "[demo_start] 시도 $i: $out" | tee -a "$LOG"
    echo "$out" | grep -q "success=True" && { echo "[demo_start] ★ 미션 시작됨" | tee -a "$LOG"; exit 0; }
    sleep 3
  done
  echo "[demo_start] ✗ 100 번 불러도 시작 안 됨 — 위 message 의 준비 항목을 볼 것" | tee -a "$LOG"
) &
STARTER=$!
trap 'kill $STARTER 2>/dev/null' EXIT
ros2 launch bringup bringup.launch.py "$@" 2>&1 | tee -a "$LOG"
