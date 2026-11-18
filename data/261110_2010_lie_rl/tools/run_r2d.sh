#!/usr/bin/env bash
# r2d (모의): r2c_verify (pid $1, R0t 포함) 끝 -> r2c model_999 에서 500 반복 (B3) -> 512 평가 seed 777 · 778 · 779
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"; MAN="$W/ws/urdf_and_usd/policy/manipulator"; OUT="$W/results"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"; PY="$ISAACLAB_DIR/.venv/bin/python"
[ -n "${1:-}" ] && tail --pid="$1" -f /dev/null
CK="$MAN/logs/rsl_rl/waffle_lieupright/2026-11-11_00-43-38/model_999.pt"
TASK=Waffle-LieUprightB3-Direct bash "$W/tools/run_p1_round.sh" r2d "$CK" 500
SUM="$OUT/p1_rounds.txt"
exec 9>/tmp/lv2_gpu.lock; flock -w 1800 9 || exit 2
cd "$MAN"
for sd in 777 778 779; do
  L="$OUT/p1_r2dv_s$sd.log"
  timeout -k 30s -s INT 20m "$PY" lieupright_eval.py --policy "$OUT/p1_r2d_policy.pt" --num_envs 512 --seed $sd --nb_prob 0.5 > "$L" 2>&1
  echo "$(date '+%F %T') [r2d-v] seed $sd" | tee -a "$SUM"
  grep -aE "\[LUEval\]" "$L" | grep -v "정책" | sed "s/^/    /" | tee -a "$SUM"
  grep -aq Traceback "$L" && grep -a -A8 Traceback "$L" | head -12 | tee -a "$SUM"
done
echo "$(date '+%F %T') [r2d] 끝" | tee -a "$SUM"
