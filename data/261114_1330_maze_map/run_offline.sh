#!/usr/bin/env bash
# 실물 미로 bag → Cartographer 오프라인 지도 (PC). experiments/cartographer_ab/run_offline.sh 와 같은 방식(별도 도메인 · LOCALHOST).
#   사용: 261114_1330_maze_map/run_offline.sh [설정 이름=maze_c6] [결과 이름=설정 이름]
#   입력: bag/carto_in (앞 · 뒤 라이다 차체 가린 스캔 = scan_1 · scan_2, /odom, /tf_static)
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CFG=${1:-maze_c6}; NAME=${2:-$CFG}
BAG=${BAG:-$HERE/bag/carto_in}   # maze_1114_1317 에서 /scan_front_filtered → /scan_1 · /scan_rear_filtered → /scan_2 로 이름만 바꾼 사본(오프라인 노드는 --ros-args 이름 바꾸기를 bag 에 안 씀) + /odom · /imu · /tf_static
WS=${WAFFLE_WS:-$HOME/git/ros_test/waffle-lyrical-slam}
RUN=$HERE/run/$NAME; rm -rf "$RUN"; mkdir -p "$RUN"
RES=$(grep -o "resolution = [0-9.]*" "$HERE/configs/$CFG.lua" | head -1 | awk '{print $3}')
ros2_env() { env -i HOME="$HOME" USER="$USER" PATH=/usr/local/bin:/usr/bin:/bin LANG=en_US.UTF-8 \
    ROS_DOMAIN_ID=${OFFLINE_DOMAIN:-87} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    bash -c "source /opt/ros/lyrical/setup.bash && source '$WS/install/setup.bash' && $1"; }
{ echo "bag: $BAG"; echo "config: $CFG (해상도 $RES)"; echo "start: $(date -Iseconds)"; } > "$RUN/run_info.txt"
ros2_env "/usr/bin/time -v -o '$RUN/time.txt' ros2 run cartographer_ros cartographer_offline_node \
    -configuration_directory '$HERE/configs' -configuration_basenames '$CFG.lua' \
    -bag_filenames '$BAG' -save_state_filename '$RUN/state.pbstream'" > "$RUN/offline_stdout.txt" 2>&1
echo $? > "$RUN/exit_code.txt"
ros2_env "ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
    -pbstream_filename '$RUN/state.pbstream' -map_filestem '$RUN/map' -resolution $RES" > "$RUN/to_map_stdout.txt" 2>&1
echo "end: $(date -Iseconds)" >> "$RUN/run_info.txt"
ls -la "$RUN"; tail -3 "$RUN/offline_stdout.txt"
