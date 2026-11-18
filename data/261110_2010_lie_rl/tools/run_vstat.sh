#!/usr/bin/env bash
# 2026-11-11 VS (RL 261111 1, RL 261110 1 요청): r2e 제한 없음 관절 속도 통계 512 x 3 seed (VL 끝난 뒤 잠금 대기)
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"; MAN="$W/ws/urdf_and_usd/policy/manipulator"; OUT="$W/results"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"; PY="$ISAACLAB_DIR/.venv/bin/python"
exec 9>/tmp/lv2_gpu.lock; flock -w 3600 9 || { echo "GPU 잠금 대기 실패" | tee -a "$OUT/vl_summary.txt"; exit 2; }
cd "$MAN"
for sd in 777 778 779; do
  L="$OUT/p1_r2e_vstat_s$sd.log"
  timeout -k 30s -s INT 20m "$PY" lieupright_eval_vstat.py --policy "$OUT/p1_r2e_policy.pt" --num_envs 512 --seed $sd --nb_prob 0.5 --nb_world 1 > "$L" 2>&1
  echo "== VS seed $sd (제한 없음)" | tee -a "$OUT/vl_summary.txt"; grep -a "R1 세워\|접촉 없는 성공\|관절 속도\|Traceback" "$L" | tee -a "$OUT/vl_summary.txt"
done
echo "VS 끝 $(date '+%F %T')" | tee -a "$OUT/vl_summary.txt"
