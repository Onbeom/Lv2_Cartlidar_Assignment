#!/usr/bin/env bash
# Isaac Sim ROS 2 브리지 검증 한 번 실행 (PC).
#   사용: bridge_check/run_bridge_check.sh A|B [재생 시간 s, 기본 600] [결과 상위 폴더, 기본 results/bridge_check]
#   A: ROS 2 Lyrical을 source한 깨끗한 환경에서 Isaac Sim 실행 (시험 대상)
#   B: source 없이 실행 → python.sh가 Isaac Sim 내장 Jazzy 라이브러리를 씀 (비교 기준)
# 두 조건 모두 env -i로 부모 셸 환경을 비우고 시작해 같은 조건을 만든다. 관찰 노드는 항상 Lyrical.
set -u
COND=${1:?A 또는 B}
DURATION=${2:-600}
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$HERE")"
PARENT=${3:-$ROOT/results/bridge_check}
ISAAC=${ISAAC_SIM_ROOT:-$HOME/isaacsim}
OUT="$PARENT/${COND}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"

COMMON_ENV=(HOME="$HOME" USER="$USER" LOGNAME="$USER" LANG=en_US.UTF-8
            PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
            ROS_DOMAIN_ID=28 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST)
[ -n "${XDG_RUNTIME_DIR:-}" ] && COMMON_ENV+=(XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR")
PAUSE_TOTAL=5   # scene_bridge_check.py 기본 일시정지 길이와 같게

echo "[run] 조건 $COND, 재생 ${DURATION}s, 결과 $OUT"

# GPU 메모리 1초 간격 기록
( echo "wall_s,used_mib"
  while true; do echo "$(date +%s.%N),$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits | head -1)"; sleep 1; done
) > "$OUT/gpu_mem.csv" 2>/dev/null &
GPU_PID=$!

# 관찰 노드 (Lyrical). Isaac Sim보다 먼저 띄워 처음 메시지부터 받는다
env -i "${COMMON_ENV[@]}" bash -c "source /opt/ros/lyrical/setup.bash && exec python3 '$HERE/observe_bridge.py' --out '$OUT' ${OBS_ARGS:-}" \
    > "$OUT/observer_stdout.txt" 2>&1 &
OBS_PID=$!
sleep 3

START_TS=$(date +%s)
if [ "$COND" = "A" ]; then
    ISAAC_CMD="source /opt/ros/lyrical/setup.bash && exec '$ISAAC/python.sh' '$HERE/scene_bridge_check.py' --out '$OUT' --duration $DURATION"
else
    ISAAC_CMD="exec '$ISAAC/python.sh' '$HERE/scene_bridge_check.py' --out '$OUT' --duration $DURATION"
fi
env -i "${COMMON_ENV[@]}" bash -c "$ISAAC_CMD" > "$OUT/isaac_stdout.txt" 2>&1 &
ISAAC_PID=$!

# Isaac Sim 프로세스 메모리 5초 간격 기록 (python.sh가 exec하므로 같은 PID)
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

sleep 2
kill -INT "$OBS_PID" 2>/dev/null; wait "$OBS_PID" 2>/dev/null
kill "$GPU_PID" "$RSS_PID" 2>/dev/null; wait "$GPU_PID" "$RSS_PID" 2>/dev/null

# 이번 실행의 Kit 로그 복사
KIT_LOG=$(find "$HOME/.nvidia-omniverse/logs/Kit" -name 'kit_*.log' -newermt "@$START_TS" -printf '%T@ %p\n' 2>/dev/null \
          | sort -n | tail -1 | cut -d' ' -f2-)
[ -n "$KIT_LOG" ] && cp "$KIT_LOG" "$OUT/kit_log.txt" && echo "$KIT_LOG" > "$OUT/kit_log_source.txt"

python3 "$HERE/analyze_bridge.py" "$OUT" --cond "$COND" > /dev/null
echo "[run] 판정: $OUT/summary.md"
grep -E '^## B[0-3]' "$OUT/summary.md"
