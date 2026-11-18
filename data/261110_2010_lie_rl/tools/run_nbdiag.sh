#!/usr/bin/env bash
# 2026-11-11 r2f 뒤: 옆 퍽 닿음 배치 진단 (r2e · r2f 정책, seed 777 · 778 · 779, 판정과 같은 평가 + --nb_diag)
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"; MAN="$W/ws/urdf_and_usd/policy/manipulator"; OUT="$W/results"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"; PY="$ISAACLAB_DIR/.venv/bin/python"
exec 9>/tmp/lv2_gpu.lock; flock -w 1800 9 || exit 2
cd "$MAN"
for nm in r2e r2f; do for sd in 777 778 779; do
  L="$OUT/p1_${nm}_nbdiag_s$sd.log"
  timeout -k 30s -s INT 20m "$PY" lieupright_eval.py --policy "$OUT/p1_${nm}_policy.pt" --num_envs 512 --seed $sd --nb_prob 0.5 --nb_diag "$OUT/p1_${nm}_nbdiag_s$sd.json" > "$L" 2>&1
  echo "== $nm $sd"; grep "옆 퍽 진단\|닿은 대 원인\|비교\|그중 닿음 [0-9]* =" "$L"
done; done
