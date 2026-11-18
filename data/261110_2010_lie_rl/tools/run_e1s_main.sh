#!/usr/bin/env bash
# E1-S (모의, RL1 승인 r2e 보다 먼저): run_e1s_check (pid $1) 끝 -> S-T8 에 Traceback 없고 '끝 상태 넣음' 줄 있으면 E1-S 64 (r2d E1a N0 끝 상태)
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"; D="$W/ws_dev/workspace/urdf_and_usd/policy/manipulator"; OUT="$W/results"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"; PY="$ISAACLAB_DIR/.venv/bin/python"
SUM="$OUT/e1_summary.txt"
[ -n "${1:-}" ] && tail --pid="$1" -f /dev/null
if grep -aq Traceback "$OUT/e1s_T8.log" || ! grep -aq "E1-scripted 끝 상태 넣음 8 대" "$OUT/e1s_T8.log"; then echo "$(date '+%F %T') S-T8 불통과 -> E1-S 안 돌림" | tee -a "$SUM"; exit 1; fi
exec 9>/tmp/lv2_gpu.lock; flock -w 3600 9 || exit 2
cd "$D"
timeout -k 30s -s INT 30m "$PY" chain_pallet_eval_e1.py --num_envs 64 --scripted_pick 2 --grip_open 0.0042 --seed 2 --pick_only 1 --from_end "$OUT/e1a_N0.json" --out "$OUT/e1s_main.json" > "$OUT/e1s_main.log" 2>&1
echo "$(date '+%F %T') E1-S (r2d E1a N0 끝 상태 64, 고정 동작 집기)" | tee -a "$SUM"; grep -aE "\[Chain\]|Traceback" "$OUT/e1s_main.log" | sed 's/^/    /' | cut -c1-500 | tee -a "$SUM"
