#!/usr/bin/env bash
# (pid $1 끝) -> P1 도구 확인 64 x 5 (GPU 잠금) -> Traceback 없고 'Learning iteration' 있으면 1 회차 (2048 x 1000)
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"
MAN="$W/ws/urdf_and_usd/policy/manipulator"
OUT="$W/results"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"
PY="$ISAACLAB_DIR/.venv/bin/python"
SUM="$OUT/after_r0_summary.txt"
[ -n "${1:-}" ] && tail --pid="$1" -f /dev/null
(
exec 9>/tmp/lv2_gpu.lock
flock -w 3600 9 || { echo "$(date '+%F %T') 도구 확인 잠금 대기 초과" | tee -a "$SUM"; exit 2; }
cd "$MAN"
mv -f "$OUT/p1_smoke.log" "$OUT/p1_smoke_try1.log" 2>/dev/null
timeout -k 30s -s INT 15m "$PY" train.py --task Waffle-LieUpright-Direct --num_envs 64 --max_iterations 5 --viz none \
  --experiment_name lieupright_smoke > "$OUT/p1_smoke.log" 2>&1
echo "$(date '+%F %T') P1 도구 확인 (2 번째) 끝 (exit $?)" | tee -a "$SUM"
grep -aE "Learning iteration|Mean reward|Metrics/s0_grasp_any|Metrics/s1_success|Metrics/s2_success|Iteration time|Episode_End" "$OUT/p1_smoke.log" | tail -30 | cut -c1-200 | tee -a "$SUM"
grep -a -A12 "^Traceback" "$OUT/p1_smoke.log" | head -30 | tee -a "$SUM"
)
if grep -aq Traceback "$OUT/p1_smoke.log" || ! grep -aq "Learning iteration" "$OUT/p1_smoke.log"; then
  echo "$(date '+%F %T') 도구 확인 (2 번째) 실패 -> 1 회차 안 함" | tee -a "$OUT/p1_rounds.txt"; exit 1; fi
bash "$W/tools/run_p1_round.sh" r1
