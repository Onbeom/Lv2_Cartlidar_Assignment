#!/usr/bin/env bash
# 2026-11-11 (RL1 승인 뒤에만 실행): r2e 관측 쥠을 손가락 범위 + 히스테리시스만 (hold_dist 없음) 으로 바꿔 3 seed, 같은 회에 학습 판정 (truth) 도 짝으로
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"; MAN="$W/ws/urdf_and_usd/policy/manipulator"; OUT="$W/results"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"; PY="$ISAACLAB_DIR/.venv/bin/python"
exec 9>/tmp/lv2_gpu.lock; flock -w 1800 9 || exit 2
cd "$MAN"
for sd in 777 778 779; do for oh in truth finger; do
  L="$OUT/p1_r2e_oh${oh}_s$sd.log"
  timeout -k 30s -s INT 20m "$PY" lieupright_eval.py --policy "$OUT/p1_r2e_policy.pt" --num_envs 512 --seed $sd --nb_prob 0.5 --nb_world 1 --obs_hold $oh > "$L" 2>&1
  echo "== r2e $sd $oh"; grep "R1 세워\|종료 이유\|그중 닿음 [0-9]* =\|접촉 없는 성공\|관측 쥠" "$L"
done; done
