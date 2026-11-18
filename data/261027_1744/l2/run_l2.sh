#!/usr/bin/env bash
# L2 로봇 장면 검사 한 번 실행 (PC). 조건 A′: Lyrical source + empy·lark 링크(bridge_check/pylinks).
#   사용: 261027_1744/l2/run_l2.sh [시뮬레이션 시간 s, 기본 600] [결과 상위 폴더, 기본 261027_1744/results]
# 바깥 노드(Lyrical, use_sim_time): robot_state_publisher(URDF 261027_1711), 관찰·명령 노드.
# 모두 env -i로 같은 환경에서 시작한다(도메인 26, CycloneDDS, LOCALHOST).
set -u
DURATION=${1:-600}
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK="$(dirname "$HERE")"
TOP="$(dirname "$WORK")"
PARENT=${2:-$WORK/results}
ISAAC_PY=${ISAAC_PYTHON:-$HOME/isaacsim-venv/bin/python}
KIT_LOG_DIR="$(dirname "$ISAAC_PY")/../lib/python3.12/site-packages/isaacsim/kit/logs"
PYLINKS="$TOP/261025_0307/bridge_check/pylinks"   # L1 때 만든 em·lark 링크를 같이 씀
URDF="$TOP/261027_1711/urdf/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf"
OUT="$PARENT/L2_$(date +%y%m%d_%H%M%S)"
mkdir -p "$OUT"

COMMON_ENV=(HOME="$HOME" USER="$USER" LOGNAME="$USER" LANG=en_US.UTF-8
            PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
            ROS_DOMAIN_ID=26 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST)
[ -n "${XDG_RUNTIME_DIR:-}" ] && COMMON_ENV+=(XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR")
ros() { env -i "${COMMON_ENV[@]}" bash -c "source /opt/ros/lyrical/setup.bash && $1"; }
# 끝낼 때: SIGINT 뒤 10 s 안에 안 끝나면 강제 종료
stop_pid() { kill -INT "$1" 2>/dev/null; for _ in $(seq 20); do kill -0 "$1" 2>/dev/null || return 0; sleep 0.5; done; kill -9 "$1" 2>/dev/null; }

echo "[run] L2, 시뮬레이션 ${DURATION}s, 결과 $OUT"

# robot_state_publisher 매개변수 파일 (URDF 문자열은 JSON 문자열로 넣는다: YAML이 그대로 읽음)
python3 -c "import json,sys; u=open(sys.argv[1]).read(); print('robot_state_publisher:\n  ros__parameters:\n    use_sim_time: true\n    robot_description: ' + json.dumps(u))" \
    "$URDF" > "$OUT/rsp_params.yaml"

( echo "wall_s,used_mib"
  while true; do echo "$(date +%s.%N),$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits | head -1)"; sleep 1; done
) > "$OUT/gpu_mem.csv" 2>/dev/null &
GPU_PID=$!

# 함수가 아니라 env를 직접 띄운다: env → bash → exec 로 PID가 그대로 노드 프로세스가 되어 신호가 닿는다
env -i "${COMMON_ENV[@]}" bash -c "source /opt/ros/lyrical/setup.bash && exec /opt/ros/lyrical/lib/robot_state_publisher/robot_state_publisher --ros-args --params-file '$OUT/rsp_params.yaml'" \
    > "$OUT/rsp_stdout.txt" 2>&1 &
RSP_PID=$!
# OBS_EXTRA_ENV: 관찰 노드에만 줄 환경 변수(NAME=VALUE, 진단 실험 D2의 CycloneDDS 수신 버퍼 설정)
env -i "${COMMON_ENV[@]}" ${OBS_EXTRA_ENV:-} bash -c "source /opt/ros/lyrical/setup.bash && exec python3 '$HERE/observe_l2.py' --out '$OUT'" \
    > "$OUT/observer_stdout.txt" 2>&1 &
OBS_PID=$!
sleep 3

START_TS=$(date +%s)
env -i "${COMMON_ENV[@]}" bash -c "source /opt/ros/lyrical/setup.bash && export PYTHONPATH='$PYLINKS':\$PYTHONPATH && exec '$ISAAC_PY' '$HERE/scene_l2.py' --out '$OUT' --duration $DURATION --/crashreporter/url=" \
    > "$OUT/isaac_stdout.txt" 2>&1 &
ISAAC_PID=$!
( echo "wall_s,rss_mib"
  while kill -0 "$ISAAC_PID" 2>/dev/null; do
      rss=$(awk '/VmRSS/ {print $2}' "/proc/$ISAAC_PID/status" 2>/dev/null)
      [ -n "$rss" ] && echo "$(date +%s.%N),$((rss / 1024))"
      sleep 5
  done ) > "$OUT/isaac_rss.csv" &
RSS_PID=$!
echo "[run] Isaac Sim PID $ISAAC_PID 실행 중. 진행: tail -f '$OUT/isaac_stdout.txt'"
wait "$ISAAC_PID"
echo $? > "$OUT/isaac_exit_code.txt"
echo "[run] Isaac Sim 종료 코드 $(cat "$OUT/isaac_exit_code.txt")"

# C1-2: 바깥 노드의 use_sim_time (노드가 살아 있을 때 확인)
{ for n in /robot_state_publisher /l2_observer; do
      echo "$n $(ros "ros2 param get --no-daemon $n use_sim_time" 2>&1 | tail -1)"
  done; } > "$OUT/use_sim_time.txt"
ros "ros2 node list --no-daemon" > "$OUT/node_list.txt" 2>&1

sleep 1
stop_pid "$OBS_PID"
stop_pid "$RSP_PID"
wait "$OBS_PID" "$RSP_PID" 2>/dev/null
kill "$GPU_PID" "$RSS_PID" 2>/dev/null; wait "$GPU_PID" "$RSS_PID" 2>/dev/null

KIT_LOG=$(find "$KIT_LOG_DIR" -name 'kit_*.log' -newermt "@$START_TS" -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2-)
[ -n "$KIT_LOG" ] && cp "$KIT_LOG" "$OUT/kit_log.txt"

python3 "$HERE/analyze_l2.py" "$OUT" > /dev/null
echo "[run] 판정: $OUT/summary.md"
grep -E '^## ' "$OUT/summary.md"
