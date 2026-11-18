#!/usr/bin/env bash
# E1-S (모의, RL1 승인 r2e 보다 먼저): E1-S-r2e (RL1 승인 11-11 08:1x): r2e E1a N0 s777 끝 상태 64 대
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"; D="$W/ws_dev/workspace/urdf_and_usd/policy/manipulator"; OUT="$W/results"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"; PY="$ISAACLAB_DIR/.venv/bin/python"
SUM="$OUT/e1_summary.txt"
exec 9>/tmp/lv2_gpu.lock; flock -w 3600 9 || exit 2
cd "$D"
timeout -k 30s -s INT 30m "$PY" chain_pallet_eval_e1.py --num_envs 64 --scripted_pick 2 --grip_open 0.0042 --seed 2 --pick_only 1 --from_end "$OUT/e1a_r2e_N0_s777.json" --out "$OUT/e1s_r2e_diag.json" > "$OUT/e1s_r2e_diag.log" 2>&1
echo "$(date '+%F %T') E1-S-r2e 진단 (r2e E1a N0 s777 끝 상태 64, 고정 동작 집기)" | tee -a "$SUM"; grep -aE "\[Chain\]|Traceback" "$OUT/e1s_r2e_diag.log" | sed 's/^/    /' | cut -c1-500 | tee -a "$SUM"
