#!/usr/bin/env bash
# P1 2 회차 B2 (모의): 각본 시작 상태 v2 모으기 (256 대, above · at · lifted · stood) -> 도구 확인 64 x 5 -> 2 회차 r2 (2048 x 1000)
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"
MAN="$W/ws/urdf_and_usd/policy/manipulator"
OUT="$W/results"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"
PY="$ISAACLAB_DIR/.venv/bin/python"
SUM="$OUT/b2_summary.txt"
(
exec 9>/tmp/lv2_gpu.lock
flock -w 3600 9 || { echo "$(date '+%F %T') 잠금 대기 초과" | tee -a "$SUM"; exit 2; }
echo "$(date '+%F %T') 잠금 잡음, 시작 상태 v2 모으기" | tee -a "$SUM"
cd "$MAN"
timeout -k 30s -s INT 60m ./run_rl.sh chain45_eval 256 --lie_regrasp --lie_layout mission --lie_nb_prob 0.5 --est --seconds 25 \
  --dump_starts "$MAN/lieupright_starts_v2.json" --dump_pre 1 > "$OUT/collect_starts_v2.log" 2>&1
pkill -9 -f "[c]hain45_play.py" 2>/dev/null; sleep 5
grep -aE "mission 배치|경로 계산|시작 상태|Traceback" "$OUT/collect_starts_v2.log" | cut -c1-300 | tee -a "$SUM"
timeout -k 30s -s INT 15m "$PY" train.py --task Waffle-LieUprightB2-Direct --num_envs 64 --max_iterations 5 --viz none \
  --experiment_name lieupright_smoke > "$OUT/p1_smoke_b2.log" 2>&1
echo "$(date '+%F %T') B2 도구 확인 끝 (exit $?)" | tee -a "$SUM"
grep -aE "LieUpright\]|Mean reward|Metrics/s[34]_|Iteration time" "$OUT/p1_smoke_b2.log" | tail -14 | cut -c1-200 | tee -a "$SUM"
grep -a -A12 "^Traceback" "$OUT/p1_smoke_b2.log" | head -30 | tee -a "$SUM"
)
if grep -aq Traceback "$OUT/p1_smoke_b2.log" || ! grep -aq "Learning iteration" "$OUT/p1_smoke_b2.log" || [ ! -f "$MAN/lieupright_starts_v2.json" ]; then
  echo "$(date '+%F %T') B2 도구 확인 실패 -> 2 회차 안 함" | tee -a "$OUT/p1_rounds.txt"; exit 1; fi
CHECK=300:s4_grasp:0.2 TASK=Waffle-LieUprightB2-Direct bash "$W/tools/run_p1_round.sh" r2
