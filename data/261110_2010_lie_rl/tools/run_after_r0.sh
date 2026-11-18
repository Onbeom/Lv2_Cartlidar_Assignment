#!/usr/bin/env bash
# R0 다음 (모의): C 각본 시작 상태 모으기 (256 대, 옆 퍽 반반) -> P1 도구 확인 (64 대 x 5 반복, 처음부터만 / 모은 상태 섞어)
#   bash run_after_r0.sh <R0 스크립트 pid>
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"
MAN="$W/ws/urdf_and_usd/policy/manipulator"
OUT="$W/results"; mkdir -p "$OUT"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"
PY="$ISAACLAB_DIR/.venv/bin/python"
SUM="$OUT/after_r0_summary.txt"
[ -n "${1:-}" ] && { echo "$(date '+%F %T') R0 (pid $1) 끝 기다림" | tee -a "$SUM"; tail --pid="$1" -f /dev/null; }
exec 9>/tmp/lv2_gpu.lock
if ! flock -w 1800 9; then echo "$(date '+%F %T') 잠금 대기 30 분 초과, 건너뜀" | tee -a "$SUM"; exit 2; fi
echo "$(date '+%F %T') 잠금 잡음" | tee -a "$SUM"
cd "$MAN"
# C: 시작 상태
timeout -k 30s -s INT 60m ./run_rl.sh chain45_eval 256 --lie_regrasp --lie_layout mission --lie_nb_prob 0.5 --est --seconds 25 \
  --dump_starts "$MAN/lieupright_starts.json" > "$OUT/collect_starts.log" 2>&1
pkill -9 -f "[c]hain45_play.py" 2>/dev/null; sleep 5
grep -aE "mission 배치|경로 계산|시작 상태|Traceback" "$OUT/collect_starts.log" | cut -c1-300 | tee -a "$SUM"
# P1 도구 확인 (시작 상태 파일이 있으면 섞임)
timeout -k 30s -s INT 15m "$PY" train.py --task Waffle-LieUpright-Direct --num_envs 64 --max_iterations 5 --viz none \
  --experiment_name lieupright_smoke > "$OUT/p1_smoke.log" 2>&1
echo "$(date '+%F %T') P1 도구 확인 끝 (exit $?)" | tee -a "$SUM"
grep -aE "LieUpright\]|Traceback|Error|Mean reward|Metrics/s0_grasp_any|Metrics/s1_success|Metrics/s2_success|Iteration time" "$OUT/p1_smoke.log" | tail -25 | cut -c1-250 | tee -a "$SUM"
grep -a -A8 Traceback "$OUT/p1_smoke.log" | head -30 | tee -a "$SUM"
echo "$(date '+%F %T') 끝" | tee -a "$SUM"
