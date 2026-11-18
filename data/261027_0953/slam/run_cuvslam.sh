#!/usr/bin/env bash
# cuVSLAM 재생 한 번 (PC, 모의): 기록 bag을 그대로 재생하고 cuVSLAM 출력을 bag으로 남긴다.
#   사용: [CUVSLAM_VARIANT=V0|V1|V2] 261027_0953/slam/run_cuvslam.sh <기록 결과 폴더> [재생 속도, 기본 1.0]  (변형은 cuvslam_replay.launch.py)
# 결과: <기록 결과 폴더>/cuvslam_<시각>/ (out_bag, 노드 로그, GPU 메모리, 자원 사용)
# 도메인은 기록(26)과 겹치지 않게 27(REPLAY_DOMAIN). bag의 /tf(바퀴 오도메트리 odom→base_footprint)는 /tf_wheel로 바꿔 재생한다(cuVSLAM이 odom→base_footprint를 냄).
set -u
REC=$(cd "$1" && pwd)
RATE=${2:-1.0}
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VARIANT=${CUVSLAM_VARIANT:-V0}
OUT="$REC/cuvslam_${VARIANT}_$(date +%y%m%d_%H%M%S)"
mkdir -p "$OUT"
COMMON_ENV=(HOME="$HOME" USER="$USER" LOGNAME="$USER" LANG=en_US.UTF-8
            PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
            ROS_DOMAIN_ID=${REPLAY_DOMAIN:-27} RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST CUVSLAM_VARIANT="$VARIANT"
            CYCLONEDDS_URI="file://$HOME/.config/cyclonedds/cyclonedds_rx.xml")
[ -n "${XDG_RUNTIME_DIR:-}" ] && COMMON_ENV+=(XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR")
stop_pid() { kill -${2:-INT} "$1" 2>/dev/null; for _ in $(seq 40); do kill -0 "$1" 2>/dev/null || return 0; sleep 0.5; done; kill -9 "$1" 2>/dev/null; }
OUT_TOPICS=(/visual_slam/tracking/odometry /visual_slam/tracking/vo_pose /visual_slam/tracking/slam_pose
            /visual_slam/status /tf)

exec 9>/tmp/lv2_gpu.lock
flock 9
( echo "wall_s,used_mib"
  while true; do echo "$(date +%s.%N),$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits | head -1)"; sleep 1; done
) > "$OUT/gpu_mem.csv" 2>/dev/null &
GPU_PID=$!
env -i "${COMMON_ENV[@]}" bash -c "source /opt/ros/lyrical/setup.bash && exec /usr/bin/time -v ros2 launch '$HERE/cuvslam_replay.launch.py'" \
    > "$OUT/cuvslam_stdout.txt" 2>&1 &
VS_PID=$!
env -i "${COMMON_ENV[@]}" bash -c "source /opt/ros/lyrical/setup.bash && exec ros2 bag record --use-sim-time -s mcap -o '$OUT/out_bag' --topics ${OUT_TOPICS[*]}" \
    > "$OUT/record_stdout.txt" 2>&1 &
REC_PID=$!
sleep 8
# V1 · V2: 적외선 아래 MASK_BOTTOM_PX(220) 행을 칠해 넣음(mask_relay.py). cuVSLAM img_mask_bottom 은 효과가 없었음(도구 확인 21:2x)
REMAP="/tf:=/tf_wheel"
MASK_PID=""
if [ "$VARIANT" != "V0" ]; then
  REMAP="$REMAP /top_camera_front/infra1/image_rect_raw:=/unmasked/top_camera_front/infra1/image_rect_raw /top_camera_front/infra2/image_rect_raw:=/unmasked/top_camera_front/infra2/image_rect_raw"
  env -i "${COMMON_ENV[@]}" MASK_BOTTOM_PX=${MASK_BOTTOM_PX:-220} bash -c "source /opt/ros/lyrical/setup.bash && exec python3 '$HERE/mask_relay.py'" > "$OUT/mask_stdout.txt" 2>&1 &
  MASK_PID=$!
  echo "mask_bottom_px,${MASK_BOTTOM_PX:-220}" > "$OUT/mask.csv"
  sleep 3
fi
START=$(date +%s.%N)
env -i "${COMMON_ENV[@]}" bash -c "source /opt/ros/lyrical/setup.bash && exec ros2 bag play '$REC/bag' --rate $RATE --remap $REMAP" \
    > "$OUT/play_stdout.txt" 2>&1
echo "play_wall_s,$(echo "$(date +%s.%N) - $START" | bc)" > "$OUT/play_time.csv"
sleep 3
VNODE="^/opt/ros/lyrical/lib/rclcpp_components/component_container .*__node:=visual_slam_launch_container"   # 실행 파일 경로로 시작하는 것만
CPID=$(pgrep -f "$VNODE" | head -1)
[ -n "$CPID" ] && { echo "cpu_s,$(awk '{printf "%.2f", ($14+$15)/100}' /proc/$CPID/stat)"; grep VmHWM /proc/$CPID/status | awk '{print "max_rss_kb,"$2}'; } > "$OUT/proc_usage.csv"   # C1(컨테이너 CPU 시간 · 최대 메모리)
stop_pid "$REC_PID" TERM
[ -n "$MASK_PID" ] && stop_pid "$MASK_PID" TERM
stop_pid "$VS_PID" TERM
pkill -TERM -f "$VNODE" 2>/dev/null   # ros2 launch 를 끝내도 컨테이너가 남는 문제(도구 확인 T4)
sleep 2; pkill -KILL -f "$VNODE" 2>/dev/null
wait "$REC_PID" "$VS_PID" 2>/dev/null
kill "$GPU_PID" 2>/dev/null; wait "$GPU_PID" 2>/dev/null
flock -u 9
echo "[cuvslam] 끝: $OUT"
