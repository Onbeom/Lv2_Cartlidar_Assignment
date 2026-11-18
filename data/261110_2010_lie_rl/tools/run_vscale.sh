#!/usr/bin/env bash
# 2026-11-11 VC (clip 원인 확인) · VK (노드 비례 축소 scale) — RL 261111 1, 사용자 지시 · 미션 파트 요청
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"; MAN="$W/ws/urdf_and_usd/policy/manipulator"; OUT="$W/results"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"; PY="$ISAACLAB_DIR/.venv/bin/python"
S="$OUT/vk_summary.txt"
exec 9>/tmp/lv2_gpu.lock; flock -w 3600 9 || { echo "GPU 잠금 대기 실패" | tee -a "$S"; exit 2; }
cd "$MAN"
run() { # 이름 v 방식 seed
  L="$OUT/p1_r2e_$1_s$4.log"
  timeout -k 30s -s INT 25m "$PY" lieupright_eval.py --policy "$OUT/p1_r2e_policy.pt" --num_envs 512 --seed $4 --nb_prob 0.5 --nb_world 1 --vmax $2 --vmode $3 > "$L" 2>&1
  echo "== $1 seed $4 (v $2, $3)" | tee -a "$S"
  grep -a "R1 세워\|단계 stood\|종료 이유\|접촉 없는 성공\|떨어뜨림 종료\|성공 순간\|R5\|v_max\|원인 확인\|그중 닿음 [0-9]* =\|Traceback" "$L" | tee -a "$S"
}
for v in 2.0 3.0; do run vc$v $v clip 777; done
for sd in 777 778 779; do for v in 4.0 3.0 2.0; do run vk$v $v scale $sd; done; done
echo "VK 끝 $(date '+%F %T')" | tee -a "$S"
