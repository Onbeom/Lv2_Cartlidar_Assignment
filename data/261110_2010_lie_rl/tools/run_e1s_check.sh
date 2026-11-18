#!/usr/bin/env bash
# E1-scripted 준비 (모의): 앞 사슬 (pid $1) 끝 -> S-base 64 (기본 배치) -> S-T8 8 (r2d E1a N0 끝 상태, 코드 확인)
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"; D="$W/ws_dev/workspace/urdf_and_usd/policy/manipulator"; OUT="$W/results"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"; PY="$ISAACLAB_DIR/.venv/bin/python"
SUM="$OUT/e1_summary.txt"
[ -n "${1:-}" ] && tail --pid="$1" -f /dev/null
exec 9>/tmp/lv2_gpu.lock; flock -w 3600 9 || exit 2
cd "$D"
O="--scripted_pick 2 --grip_open 0.0042 --seed 2 --pick_only 1"
timeout -k 30s -s INT 20m "$PY" chain_pallet_eval_e1.py --num_envs 64 $O --out "$OUT/e1s_base.json" > "$OUT/e1s_base.log" 2>&1
echo "$(date '+%F %T') S-base (dev 사본 기본 배치 64)" | tee -a "$SUM"; grep -aE "\[Chain\]|Traceback" "$OUT/e1s_base.log" | sed 's/^/    /' | cut -c1-400 | tee -a "$SUM"
timeout -k 30s -s INT 20m "$PY" chain_pallet_eval_e1.py --num_envs 8 $O --from_end "$OUT/e1a_N0.json" --out "$OUT/e1s_T8.json" > "$OUT/e1s_T8.log" 2>&1
echo "$(date '+%F %T') S-T8 (r2d E1a N0 끝 상태 8)" | tee -a "$SUM"; grep -aE "\[Chain\]|Traceback" "$OUT/e1s_T8.log" | sed 's/^/    /' | cut -c1-400 | tee -a "$SUM"
grep -a -A8 Traceback "$OUT/e1s_T8.log" "$OUT/e1s_base.log" | head -20 | tee -a "$SUM"
