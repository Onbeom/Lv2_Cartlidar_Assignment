#!/usr/bin/env bash
# SLAM 비교 기록 한 번 (PC, 모의). 4단계 L2 실행 방식(261025_0307/l2/run_l2.sh)과 같은 환경: 조건 A′, 도메인 26, CycloneDDS, LOCALHOST.
#   사용: 261027_0953/slam/run_record.sh <조명 0~1> <이름> [경유점 개수, 0 = 전체]
# 결과: 261027_0953/results/<이름>_<시각>/ (bag, truth.csv, meta.json, scene_counts.json, 수신 버퍼 오류 수, GPU 메모리)
# GPU는 /tmp/lv2_gpu.lock 잠금을 잡고 쓴다. 수신기(bag 기록)는 ~/.config/cyclonedds/cyclonedds_rx.xml(16 MB)을 쓴다.
set -u
LIGHT=$1
NAME=$2
NWP=${3:-0}
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK="$(dirname "$HERE")"
TOP="$(dirname "$WORK")"
ISAAC_PY=${ISAAC_PYTHON:-$HOME/isaacsim-venv/bin/python}
PYLINKS="$TOP/261025_0307/bridge_check/pylinks"
URDF="$TOP/261027_1711/urdf/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf"   # 261022_2119 기록(main_*)은 그 리비전
OUT="$WORK/results/${NAME}_$(date +%y%m%d_%H%M%S)"
mkdir -p "$OUT"
TOPICS=(/clock /tf /tf_static /odom /joint_states /scan_front /scan_rear
        /top_camera_front/infra1/image_rect_raw /top_camera_front/infra1/camera_info
        /top_camera_front/infra2/image_rect_raw /top_camera_front/infra2/camera_info)

COMMON_ENV=(HOME="$HOME" USER="$USER" LOGNAME="$USER" LANG=en_US.UTF-8
            PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
            ROS_DOMAIN_ID=26 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST)
[ -n "${XDG_RUNTIME_DIR:-}" ] && COMMON_ENV+=(XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR")
RX_ENV=(CYCLONEDDS_URI="file://$HOME/.config/cyclonedds/cyclonedds_rx.xml")
stop_pid() { kill -${2:-INT} "$1" 2>/dev/null; for _ in $(seq 40); do kill -0 "$1" 2>/dev/null || return 0; sleep 0.5; done; kill -9 "$1" 2>/dev/null; }
rcvbuf() { awk '/^Udp:/ {getline; print $6}' /proc/net/snmp; }

exec 9>/tmp/lv2_gpu.lock
flock 9
echo "[rec] 조명 $LIGHT, 경유점 ${NWP}(0=전체), 결과 $OUT"
sysctl -n net.core.rmem_max > "$OUT/rmem_max.txt"
echo "before,$(rcvbuf)" > "$OUT/rcvbuf_errors.csv"

python3 -c "import json,sys; u=open(sys.argv[1]).read(); print('robot_state_publisher:\n  ros__parameters:\n    use_sim_time: true\n    robot_description: ' + json.dumps(u))" \
    "$URDF" > "$OUT/rsp_params.yaml"
( echo "wall_s,used_mib"
  while true; do echo "$(date +%s.%N),$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits | head -1)"; sleep 1; done
) > "$OUT/gpu_mem.csv" 2>/dev/null &
GPU_PID=$!

env -i "${COMMON_ENV[@]}" bash -c "source /opt/ros/lyrical/setup.bash && exec /opt/ros/lyrical/lib/robot_state_publisher/robot_state_publisher --ros-args --params-file '$OUT/rsp_params.yaml'" \
    > "$OUT/rsp_stdout.txt" 2>&1 &
RSP_PID=$!
env -i "${COMMON_ENV[@]}" "${RX_ENV[@]}" bash -c "source /opt/ros/lyrical/setup.bash && exec ros2 bag record --use-sim-time -s mcap --storage-preset-profile zstd_fast -o '$OUT/bag' --topics ${TOPICS[*]}" \
    > "$OUT/bag_stdout.txt" 2>&1 &
BAG_PID=$!
sleep 4

env -i "${COMMON_ENV[@]}" bash -c "source /opt/ros/lyrical/setup.bash && export PYTHONPATH='$PYLINKS':\$PYTHONPATH && exec '$ISAAC_PY' '$HERE/scene_slam.py' --out '$OUT' --light_level $LIGHT --max_waypoints $NWP --/crashreporter/url=" \
    > "$OUT/isaac_stdout.txt" 2>&1 &
ISAAC_PID=$!
echo "[rec] Isaac Sim PID $ISAAC_PID. 진행: tail -f '$OUT/isaac_stdout.txt'"
wait "$ISAAC_PID"
echo $? > "$OUT/isaac_exit_code.txt"
sleep 2
stop_pid "$BAG_PID" TERM   # 백그라운드(&) 프로세스는 SIGINT를 무시하므로 SIGTERM
stop_pid "$RSP_PID"
wait "$BAG_PID" "$RSP_PID" 2>/dev/null
kill "$GPU_PID" 2>/dev/null; wait "$GPU_PID" 2>/dev/null
echo "after,$(rcvbuf)" >> "$OUT/rcvbuf_errors.csv"
flock -u 9
env -i "${COMMON_ENV[@]}" bash -c "source /opt/ros/lyrical/setup.bash && ros2 bag info '$OUT/bag'" > "$OUT/bag_info.txt" 2>&1
echo "[rec] 끝: Isaac 종료 코드 $(cat "$OUT/isaac_exit_code.txt"), $OUT"
