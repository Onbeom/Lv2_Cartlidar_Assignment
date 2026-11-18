#!/usr/bin/env bash
# H0 (모의): r1 정책을 쥐고 든 상태 시작만 512 대 평가 (혼합안 기준값) -> B2 (시작 상태 v2 · 도구 확인 · 2 회차)
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"
MAN="$W/ws/urdf_and_usd/policy/manipulator"
OUT="$W/results"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"
PY="$ISAACLAB_DIR/.venv/bin/python"
SUM="$OUT/p1_rounds.txt"
(
exec 9>/tmp/lv2_gpu.lock
flock -w 3600 9 || { echo "$(date '+%F %T') H0 잠금 대기 초과" | tee -a "$SUM"; exit 2; }
cd "$MAN"
echo "$(date '+%F %T') [H0] 시작 (r1 정책, 쥐고 든 상태 시작만 512)" | tee -a "$SUM"
timeout -k 30s -s INT 20m "$PY" lieupright_eval.py --policy "$OUT/p1_r1_policy.pt" --num_envs 512 --seed 777 --nb_prob 0.5 --starts 0,1,0 > "$OUT/h0_eval.log" 2>&1
grep -aE "\[LUEval\]" "$OUT/h0_eval.log" | grep -v "정책" | sed "s/^/    /" | tee -a "$SUM"
grep -a -A8 "^Traceback" "$OUT/h0_eval.log" | head -12 | tee -a "$SUM"
)
bash "$W/tools/run_b2.sh"
