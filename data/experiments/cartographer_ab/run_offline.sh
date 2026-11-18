#!/usr/bin/env bash
# 녹화한 bag 하나를 Cartographer 설정 8가지로 각각 REPEAT번 오프라인 처리해 지도를 만든다 (PC).
#   사용: experiments/cartographer_ab/run_offline.sh <bag 폴더> [결과 폴더] [반복 횟수, 기본 3]
#   이미 끝난 실행(map.pgm, 종료 코드 0)은 건너뛴다.
#   결과: <결과 폴더>/configs/(설정 8개, manifest.json), runs/<설정>_r<n>/(pbstream, pgm, yaml, 처리 로그)
# 오프라인 노드도 /tf·/clock을 발행하므로, 돌고 있는 Gazebo(도메인 26)와 섞이지 않게 별도 도메인·LOCALHOST로 실행한다.
set -u
BAG=$(realpath "${1:?bag 폴더}")
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$(dirname "$HERE")")"
OUT=$(realpath -m "${2:-$ROOT/results/cartographer_ab/$(basename "$BAG")}")
REPEAT=${3:-3}
WS=${WAFFLE_WS:-$HOME/git/ros_test/waffle-lyrical-slam}

mkdir -p "$OUT/runs"
python3 "$HERE/make_configs.py" "$OUT" || exit 1
ros2_env() { env -i HOME="$HOME" USER="$USER" PATH=/usr/local/bin:/usr/bin:/bin LANG=en_US.UTF-8 \
    ROS_DOMAIN_ID=${OFFLINE_DOMAIN:-87} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    bash -c "source /opt/ros/lyrical/setup.bash && source '$WS/install/setup.bash' && $1"; }

{ echo "bag: $BAG"; echo "start: $(date -Iseconds)"; echo "repeat: $REPEAT"
  git -C "$WS" rev-parse HEAD 2>/dev/null | sed 's/^/waffle-lyrical-slam: /'
  for d in cartographer cartographer_ros; do git -C "$WS/src/$d" rev-parse HEAD 2>/dev/null | sed "s/^/$d: /"; done
} >> "$OUT/run_info.txt"

for name in $(python3 -c "import json;print(' '.join(c['name'] for c in json.load(open('$OUT/configs/manifest.json'))['configs']))"); do
    res=$(python3 -c "import json;print([c['resolution'] for c in json.load(open('$OUT/configs/manifest.json'))['configs'] if c['name']=='$name'][0])")
    for r in $(seq 1 "$REPEAT"); do
        RUN="$OUT/runs/${name}_r$r"
        if [ -f "$RUN/map.pgm" ] && [ "$(cat "$RUN/exit_code.txt" 2>/dev/null)" = "0" ]; then
            echo "[run] $name r$r 이미 있음, 건너뜀"; continue
        fi
        rm -rf "$RUN"; mkdir -p "$RUN"
        echo "[run] $name r$r"
        ros2_env "/usr/bin/time -v -o '$RUN/time.txt' ros2 run cartographer_ros cartographer_offline_node \
            -configuration_directory '$OUT/configs' -configuration_basenames '$name.lua' \
            -bag_filenames '$BAG' -save_state_filename '$RUN/state.pbstream'" > "$RUN/offline_stdout.txt" 2>&1
        echo $? > "$RUN/exit_code.txt"
        ros2_env "ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
            -pbstream_filename '$RUN/state.pbstream' -map_filestem '$RUN/map' -resolution $res" > "$RUN/to_map_stdout.txt" 2>&1
        [ -f "$RUN/map.pgm" ] || echo "[run] 경고: $name r$r 지도 없음 (offline_stdout.txt 확인)"
    done
done
echo "end: $(date -Iseconds)" >> "$OUT/run_info.txt"
echo "[run] 완료: $OUT"
