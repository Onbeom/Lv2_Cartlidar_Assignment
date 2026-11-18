#!/usr/bin/env bash
# 카메라 3대 동시 실행 시험 (Jetson 에서 실행). 결과: ~/cam_test_<시각>/
#   bash run_cam_test.sh [측정 초(기본 60)] [launch 인자 ...]
#   예) bash run_cam_test.sh 60 arm_device:=/dev/video0
# 시작 전: 이전 실행이 남긴 카메라 노드가 있으면 멈춤(같은 장치를 두 노드가 열면 USB 재연결이 반복됨)
# 순서: dmesg 줄 수 기록 → tegrastats 시작 → 카메라 launch(백그라운드, 자기 프로세스 그룹) → 15 초 뒤 reliable 로 측정 → best effort 로 20 초 더 → 정리 → 요약
# 측정 QoS 를 둘로 나눈 이유: PC 모의에서 Fast DDS(Jetson 기본)는 best effort 구독에 큰 영상이 거의 안 들어옴(2026-10-28)
HERE=$(cd "$(dirname "$0")" && pwd)
DUR=${1:-60}; shift || true
OUT=~/cam_test_$(date +%y%m%d_%H%M%S); mkdir -p "$OUT"
source /opt/ros/lyrical/setup.bash
set -u                                             # ROS setup.bash 는 빈 변수를 쓰므로 source 뒤에 켬
export ROS_DOMAIN_ID=26 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # 시험 트래픽을 이 컴퓨터 안으로만
echo "launch 인자: $*" > "$OUT/args.txt"

if pgrep -f "realsense2_camera_node|usb_cam_node_exe" > /dev/null; then   # 이름이 15자를 넘어 -x 로는 못 찾음
  echo "이전 카메라 노드가 남아 있음 — 먼저 정리하세요:"; pgrep -af "realsense2_camera_node|usb_cam_node_exe"; exit 1
fi
sudo -v || exit 1                                  # dmesg 를 읽으려고 미리 비밀번호를 받음
D0=$(sudo dmesg | wc -l)
tegrastats --interval 1000 --logfile "$OUT/tegrastats.log" &
TS=$!
# 팔 카메라는 패키지 설치 없이 같은 폴더의 arm_camera_node.py 를 직접 실행(뒤에 준 인자가 앞을 덮음)
setsid ros2 launch "$HERE/cameras.launch.py" arm_script:="$HERE/arm_camera_node.py" "$@" > "$OUT/launch.log" 2>&1 &
LP=$!
cleanup() {                                        # Ctrl+C · 오류로 끝나도 카메라 노드를 남기지 않음(남으면 다음 실행이 장치를 못 엶)
  kill -INT -- -"$LP" 2>/dev/null
  for i in $(seq 20); do kill -0 "$LP" 2>/dev/null || break; sleep 0.5; done
  kill -KILL -- -"$LP" 2>/dev/null
  kill "$TS" 2>/dev/null
}
trap 'cleanup; exit 130' INT TERM
sleep 15
python3 "$HERE/measure_topics.py" "$DUR" "$OUT/topics.json" rel > /dev/null
python3 "$HERE/measure_topics.py" 20 "$OUT/topics_be.json" be > /dev/null
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-기본(rmw_fastrtps_cpp)}" >> "$OUT/args.txt"
ros2 topic list > "$OUT/topic_list.txt" 2>&1
cleanup; trap - INT TERM
sudo dmesg | tail -n +"$((D0 + 1))" > "$OUT/dmesg_new.txt"
python3 "$HERE/summarize.py" "$OUT" | tee "$OUT/summary.txt"
echo "결과 폴더: $OUT"
