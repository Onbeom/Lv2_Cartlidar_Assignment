#!/usr/bin/env bash
# 루프 폐합 주기(POSE_GRAPH.optimize_every_n_nodes) 비교: bag을 실제 시간으로 재생해 cartographer_node를
# 실제 주행 때처럼 돌리고, 주행 도중 /map을 SNAP초마다 저장한다 (PC).
#   사용: experiments/cartographer_ab/run_live_period.sh <bag 폴더> <기준 설정 lua> [결과 폴더] [반복, 기본 3]
#   환경 변수: PERIODS(기본 "90 30"), RATE(재생 배속, 기본 1), SNAP(저장 주기 s, 기본 30),
#             PLAY_ARGS(ros2 bag play 추가 인자, 예: "--playback-duration 70"으로 짧은 동작 확인)
# 오프라인 처리와 달리 끝에 전체 최적화를 하지 않으므로, 주기의 효과가 도중 지도에 남는다.
set -u
BAG=$(realpath "${1:?bag 폴더}")
BASE=$(realpath "${2:?기준 설정 lua}")
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$(dirname "$HERE")")"
OUT=$(realpath -m "${3:-$ROOT/results/cartographer_ab/live_period_$(date +%Y%m%d_%H%M)}")
REPEAT=${4:-3}
PERIODS=${PERIODS:-"90 30"}
RATE=${RATE:-1}
SNAP=${SNAP:-30}
export WS=${WAFFLE_WS:-$HOME/git/ros_test/waffle-lyrical-slam}
RES=$(grep -oP 'grid_options_2d.resolution = \K[0-9.]+' "$BASE" || echo 0.05)

mkdir -p "$OUT/configs"
ros2_env() { env -i HOME="$HOME" USER="$USER" PATH=/usr/local/bin:/usr/bin:/bin LANG=en_US.UTF-8 \
    ROS_DOMAIN_ID=${LIVE_DOMAIN:-88} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    bash -c "source /opt/ros/lyrical/setup.bash && source '$WS/install/setup.bash' && $1"; }
{ echo "bag: $BAG"; echo "base: $BASE"; echo "periods: $PERIODS"; echo "rate: $RATE"; echo "snap_s: $SNAP"
  echo "resolution: $RES"; echo "play_args: ${PLAY_ARGS:-}"; echo "start: $(date -Iseconds)"; } > "$OUT/run_info.txt"

for P in $PERIODS; do
    NAME="$(basename "$BASE" .lua)_opt$P"
    sed "s/^return options/-- cartographer_ab 루프 폐합 주기 실험 (Cartographer 기본값 90)\nPOSE_GRAPH.optimize_every_n_nodes = $P\n\nreturn options/" \
        "$BASE" > "$OUT/configs/$NAME.lua"
    for r in $(seq 1 "$REPEAT"); do
        RUN="$OUT/runs/${NAME}_r$r"; mkdir -p "$RUN"
        echo "[live] $NAME r$r"
        setsid bash -c "$(declare -f ros2_env); ros2_env \"/usr/bin/time -v -o '$RUN/time.txt' ros2 run cartographer_ros cartographer_node \
            -configuration_directory '$OUT/configs' -configuration_basename '$NAME.lua' --ros-args -p use_sim_time:=true\"" \
            > "$RUN/cartographer_stdout.txt" 2>&1 &
        CART=$!
        setsid bash -c "$(declare -f ros2_env); ros2_env \"ros2 run cartographer_ros cartographer_occupancy_grid_node \
            -resolution $RES -publish_period_sec 1.0 --ros-args -p use_sim_time:=true\"" > "$RUN/grid_stdout.txt" 2>&1 &
        GRID=$!
        setsid bash -c "$(declare -f ros2_env); ros2_env \"python3 '$HERE/snapshot_map.py' '$RUN/snapshots' $SNAP\"" \
            > "$RUN/snapshot_stdout.txt" 2>&1 &
        SNAPPID=$!
        sleep 5
        ros2_env "ros2 bag play '$BAG' --rate $RATE --disable-keyboard-controls ${PLAY_ARGS:-}" > "$RUN/play_stdout.txt" 2>&1
        sleep 5
        for p in $SNAPPID $GRID $CART; do kill -INT -- -"$p" 2>/dev/null; done
        for p in $SNAPPID $GRID $CART; do wait "$p" 2>/dev/null; done
        ls "$RUN/snapshots" 2>/dev/null | grep -c '\.pgm$' | sed 's/^/[live] snapshots: /'
    done
done
echo "end: $(date -Iseconds)" >> "$OUT/run_info.txt"
echo "[live] 완료: $OUT"
