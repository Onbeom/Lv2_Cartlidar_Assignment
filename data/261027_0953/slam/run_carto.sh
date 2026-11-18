#!/usr/bin/env bash
# Cartographer 재생 한 번 (PC, 모의): 기록 bag 을 실제 시간으로 재생하며 cartographer_node 를 주행 때처럼 돌리고,
# 출력 /tf(map → odom)를 bag 으로 남긴 뒤 궤적을 끝내 pbstream · 지도(pgm)를 저장한다.
#   사용: 261027_0953/slam/run_carto.sh <기록 결과 폴더> [재생 속도, 기본 1.0]
# 결과: <기록 결과 폴더>/carto_<시각>/ (out_bag, map.pbstream, map.pgm/yaml, 노드 로그, CPU 시간 · 최대 메모리)
# 구성(계획 문서): map → odom = Cartographer, odom → base_footprint = bag 의 바퀴 오도메트리(/tf 그대로 재생).
# 설정 carto_c6_2lidar.lua(c6 + 라이다 2개). 도메인은 기록(26)과 겹치지 않게 27(REPLAY_DOMAIN). GPU 를 안 써서 잠금 없음.
set -u
REC=$(cd "$1" && pwd)
RATE=${2:-1.0}
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$REC/carto_$(date +%y%m%d_%H%M%S)"
WS=${WAFFLE_WS:-$HOME/git/ros_test/waffle-lyrical-slam}
CFG=carto_c6_2lidar.lua
RES=$(grep -oP 'grid_options_2d.resolution = \K[0-9.]+' "$HERE/$CFG")
mkdir -p "$OUT"
cp "$HERE/$CFG" "$OUT/"
COMMON_ENV=(HOME="$HOME" USER="$USER" LOGNAME="$USER" LANG=en_US.UTF-8
            PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
            ROS_DOMAIN_ID=${REPLAY_DOMAIN:-27} RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
            CYCLONEDDS_URI="file://$HOME/.config/cyclonedds/cyclonedds_rx.xml")
[ -n "${XDG_RUNTIME_DIR:-}" ] && COMMON_ENV+=(XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR")
ros() { env -i "${COMMON_ENV[@]}" bash -c "source /opt/ros/lyrical/setup.bash && source '$WS/install/setup.bash' && $1"; }
# 백그라운드용: exec 로 서브셸을 바꿔 $! 가 실제 ROS 프로세스가 되게 한다(그냥 함수를 & 로 띄우면 $! 는 서브셸이라
# TERM 을 보내도 녹화기가 남아 out_bag 이 비었음 — 도구 확인 2026-10-27 20:35)
ros_bg() { exec env -i "${COMMON_ENV[@]}" bash -c "source /opt/ros/lyrical/setup.bash && source '$WS/install/setup.bash' && $1"; }
stop_pid() { kill -${2:-INT} "$1" 2>/dev/null; for _ in $(seq 40); do kill -0 "$1" 2>/dev/null || return 0; sleep 0.5; done; kill -9 "$1" 2>/dev/null; }

ros_bg "exec ros2 run cartographer_ros cartographer_node -configuration_directory '$HERE' -configuration_basename $CFG \
     --ros-args -p use_sim_time:=true -r scan_1:=/scan_front -r scan_2:=/scan_rear -r odom:=/odom" > "$OUT/cartographer_stdout.txt" 2>&1 &
CART=$!
ros_bg "exec ros2 bag record --use-sim-time -s mcap -o '$OUT/out_bag' --topics /tf" > "$OUT/record_stdout.txt" 2>&1 &
REC_PID=$!
sleep 6
START=$(date +%s.%N)
ros "exec ros2 bag play '$REC/bag' --rate $RATE --disable-keyboard-controls" > "$OUT/play_stdout.txt" 2>&1
echo "play_wall_s,$(echo "$(date +%s.%N) - $START" | bc)" > "$OUT/play_time.csv"
sleep 2
CNODE="^$WS/install/cartographer_ros/lib/cartographer_ros/cartographer_node -configuration_directory $HERE"   # 실행 파일 경로로 시작하는 것만(다른 셸 명령줄에 같은 글자가 있어도 안 잡게)
CPID=$(pgrep -f "$CNODE" | head -1)
ros "ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory '{trajectory_id: 0}'" > "$OUT/finish_stdout.txt" 2>&1
ros "ros2 service call /write_state cartographer_ros_msgs/srv/WriteState '{filename: \"$OUT/map.pbstream\", include_unfinished_submaps: true}'" \
    > "$OUT/write_stdout.txt" 2>&1
[ -n "$CPID" ] && { echo "cpu_s,$(awk '{printf "%.2f", ($14+$15)/100}' /proc/$CPID/stat)"; grep VmHWM /proc/$CPID/status | awk '{print "max_rss_kb,"$2}'; } > "$OUT/proc_usage.csv"   # C1
stop_pid "$REC_PID" TERM
stop_pid "$CART" INT
pkill -INT -f "$CNODE" 2>/dev/null   # ros2 run 아래 실제 노드
sleep 2; pkill -KILL -f "$CNODE" 2>/dev/null
wait "$REC_PID" "$CART" 2>/dev/null
ros "ros2 run cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename '$OUT/map.pbstream' -map_filestem '$OUT/map' -resolution $RES" \
    > "$OUT/to_map_stdout.txt" 2>&1
echo "[carto] 끝: $OUT"
