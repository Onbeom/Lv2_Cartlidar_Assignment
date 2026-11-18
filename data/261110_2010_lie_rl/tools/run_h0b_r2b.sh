#!/usr/bin/env bash
# H0' (고친 판정, r1 정책, 쥐고 든 상태 시작만 512) -> r2' (B2 + 판정 고침, 중간 확인 300:s4_grasp:0.2)
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"
MAN="$W/ws/urdf_and_usd/policy/manipulator"
OUT="$W/results"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"
PY="$ISAACLAB_DIR/.venv/bin/python"
SUM="$OUT/p1_rounds.txt"
(
exec 9>/tmp/lv2_gpu.lock
flock -w 3600 9 || { echo "$(date '+%F %T') H0' 잠금 대기 초과" | tee -a "$SUM"; exit 2; }
cd "$MAN"
echo "$(date '+%F %T') [H0'] 시작 (r1 정책, 고친 판정, 쥐고 든 상태 시작만 512)" | tee -a "$SUM"
timeout -k 30s -s INT 20m "$PY" lieupright_eval.py --policy "$OUT/p1_r1_policy.pt" --num_envs 512 --seed 777 --nb_prob 0.5 --starts 0,1,0 > "$OUT/h0b_eval.log" 2>&1
grep -aE "\[LUEval\]" "$OUT/h0b_eval.log" | grep -v "정책" | sed "s/^/    /" | tee -a "$SUM"
grep -a -A8 "^Traceback" "$OUT/h0b_eval.log" | head -12 | tee -a "$SUM"
)
CHECK=300:s4_grasp:0.2 TASK=Waffle-LieUprightB2-Direct bash "$W/tools/run_p1_round.sh" r2b
