#!/usr/bin/env bash
# 2026-11-06 학습 속도 비교: 원기둥 45° 집기를 환경 수 256 · 512 · 1024 로 각 15 반복 (fit1 에서), 반복당 시간 · 초당 스텝.
# 로그는 logs/rsl_rl/bench_speed/ (다른 학습의 '가장 최근' 과 안 섞임). 다른 학습 · GUI 를 끈 상태에서 (약 8 분).
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"; cd "$HERE"
CK="$HERE/logs/rsl_rl/waffle_sidepick45/2026-11-06_13-10-50/model_6348.pt"
for N in 256 512 1024; do
  timeout -s INT 6m ./run_rl.sh sidepick45 "$N" 15 "$CK" --experiment_name bench_speed > "bench_speed_$N.log" 2>&1
  pkill -f "[t]rain.py --task Waffle-SidePick45" 2>/dev/null; sleep 5
  it=$(grep -a "Iteration time" "bench_speed_$N.log" | tail -5 | awk '{s+=$3} END {if (NR) printf "%.2f", s/NR}')
  err=$(grep -aiE "out of memory|Traceback" "bench_speed_$N.log" | head -1)
  if [ -n "$it" ]; then sps=$(awk -v n="$N" -v t="$it" 'BEGIN{printf "%.0f", n*32/t}'); else sps=-; fi
  echo "환경 $N: 반복당 ${it:-?} s, 초당 스텝 $sps ${err:+(에러: $err)}"
done
