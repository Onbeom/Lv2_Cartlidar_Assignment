#!/usr/bin/env bash
# r2e 준비 (모의): 앞 사슬 (pid $1) 끝 -> r2d 512 평가 seed 777 · 778 · 779 (옆 퍽 반반) 로 옆 퍽 닿음 단계 세기
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"; MAN="$W/ws/urdf_and_usd/policy/manipulator"; OUT="$W/results"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"; PY="$ISAACLAB_DIR/.venv/bin/python"
[ -n "${1:-}" ] && tail --pid="$1" -f /dev/null
exec 9>/tmp/lv2_gpu.lock; flock -w 3600 9 || exit 2
cd "$MAN"
for sd in 777 778 779; do
  L="$OUT/p1_r2d_nbstage_s$sd.log"
  timeout -k 30s -s INT 20m "$PY" lieupright_eval.py --policy "$OUT/p1_r2d_policy.pt" --num_envs 512 --seed $sd --nb_prob 0.5 > "$L" 2>&1
  echo "seed $sd: $(grep -a '옆 퍽 닿음 종료 단계' "$L")"
done
