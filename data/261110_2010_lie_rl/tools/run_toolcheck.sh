#!/usr/bin/env bash
# 도구 확인 (모의): T1 n5 진단 64 대, T2 · T3 2048 대 학습 속도 · GPU 메모리. GPU 잠금 안에서 차례로.
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"
MAN="$W/ws/urdf_and_usd/policy/manipulator"
OUT="$W/results"; mkdir -p "$OUT"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"
SUM="$OUT/toolcheck_summary.txt"
exec 9>/tmp/lv2_gpu.lock
echo "$(date '+%F %T') GPU 잠금 기다림" | tee -a "$SUM"
if ! flock -w 1800 9; then echo "$(date '+%F %T') 잠금 대기 30 분 초과, 건너뜀" | tee -a "$SUM"; exit 2; fi
echo "$(date '+%F %T') 잠금 잡음" | tee -a "$SUM"
cd "$MAN"

# T1
timeout -k 30s -s INT 20m ./run_rl.sh sidepick45_diag 64 "$MAN/policies/sidepick45_n5.pt" > "$OUT/toolcheck_T1_diag_n5.log" 2>&1
echo "$(date '+%F %T') T1 끝 (exit $?)" | tee -a "$SUM"
pkill -9 -f "[s]idepick_diag.py" 2>/dev/null; sleep 5

# T2 · T3
( while true; do nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits; sleep 1; done ) > "$OUT/toolcheck_T3_mem.txt" 2>&1 &
MP=$!
NS=2048 bash ./bench_speed.sh > "$OUT/toolcheck_T2_bench.txt" 2>&1
kill $MP 2>/dev/null
mv -f "$MAN/bench_speed_2048.log" "$OUT/toolcheck_T2_train_2048.log" 2>/dev/null
echo "$(date '+%F %T') T2 끝: $(cat "$OUT/toolcheck_T2_bench.txt" | tail -1)" | tee -a "$SUM"
echo "$(date '+%F %T') T3 GPU 메모리 최대 $(sort -n "$OUT/toolcheck_T3_mem.txt" | tail -1) MiB" | tee -a "$SUM"
grep -aiE "out of memory|Traceback|nan" "$OUT/toolcheck_T1_diag_n5.log" "$OUT/toolcheck_T2_train_2048.log" | head -5 | tee -a "$SUM"
echo "$(date '+%F %T') 끝" | tee -a "$SUM"
