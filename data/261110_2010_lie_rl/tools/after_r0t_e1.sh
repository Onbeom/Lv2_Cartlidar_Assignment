#!/usr/bin/env bash
# R0t (r2c_verify pid $1) 끝 -> E1 코드 시험 (r2c 8 대) -> r2d 학습 · 3 seed -> E1 (채택 정책 8 대 시험 -> 512 x 2)
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"
tail --pid="$1" -f /dev/null
E1_SMOKE=1 bash "$W/tools/run_e1.sh"
bash "$W/tools/run_r2d.sh"
bash "$W/tools/run_e1.sh"
