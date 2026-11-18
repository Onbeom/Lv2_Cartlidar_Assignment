#!/usr/bin/env bash
# 2026-11-11 VL (RL 261111 1, RL 261110 1 승인): r2e 팔 관절 속도 제한 평가, seed 마다 제한 없음 · 4.0 · 2.0 짝
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"; MAN="$W/ws/urdf_and_usd/policy/manipulator"; OUT="$W/results"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"; PY="$ISAACLAB_DIR/.venv/bin/python"
exec 9>/tmp/lv2_gpu.lock; flock -w 1800 9 || { echo "GPU 잠금 30 분 대기 실패 - 건너뜀" | tee -a "$OUT/vl_summary.txt"; exit 2; }
cd "$MAN"
for sd in 777 778 779; do for v in 0 4.0 3.0 2.0; do
  L="$OUT/p1_r2e_vl${v}_s$sd.log"
  timeout -k 30s -s INT 20m "$PY" lieupright_eval.py --policy "$OUT/p1_r2e_policy.pt" --num_envs 512 --seed $sd --nb_prob 0.5 --nb_world 1 --vmax $v > "$L" 2>&1
  echo "== r2e seed $sd vmax $v" | tee -a "$OUT/vl_summary.txt"
  grep -a "R1 세워\|단계 \|종료 이유\|그중 닿음 [0-9]* =\|접촉 없는 성공\|떨어뜨림 종료\|성공 순간\|R5\|v_max\|Traceback" "$L" | tee -a "$OUT/vl_summary.txt"
done; done
echo "VL 끝 $(date '+%F %T')" | tee -a "$OUT/vl_summary.txt"
