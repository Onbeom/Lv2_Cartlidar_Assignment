#!/usr/bin/env bash
# R0s (모의): 세운 퍽 기준 512 -> r2c: r2b model_250 이어 학습 (B2, 750 반복 더, 감시 규칙만, 중간 확인 없음)
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"
MAN="$W/ws/urdf_and_usd/policy/manipulator"
OUT="$W/results"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"
SUM="$OUT/r0_summary.txt"
(
exec 9>/tmp/lv2_gpu.lock
flock -w 3600 9 || { echo "$(date '+%F %T') R0s 잠금 대기 초과" | tee -a "$SUM"; exit 2; }
cd "$MAN"
echo "$(date '+%F %T') R0s 시작 (세운 퍽 기준 512, 옆 퍽 0)" | tee -a "$SUM"
timeout -k 30s -s INT 100m ./run_rl.sh chain45_eval 512 --lie_regrasp --lie_layout mission --lie_nb_prob 0.0 --lie_stand_start 1 --est --seconds 60 > "$OUT/r0_R0s.log" 2>&1
rc=$?; pkill -9 -f "[c]hain45_play.py" 2>/dev/null; sleep 5
echo "$(date '+%F %T') R0s 끝 (exit $rc)" | tee -a "$SUM"
grep -aE "세운 퍽 기준|경로 계산|끝 분류|\*\*성공|팔-몸체 1 N|A dock: 쥐는" "$OUT/r0_R0s.log" | sed 's/^/    /' | cut -c1-400 | tee -a "$SUM"
grep -aiE -m3 -A3 "Traceback|out of memory" "$OUT/r0_R0s.log" | tee -a "$SUM"
)
CK=$(ls "$MAN"/logs/rsl_rl/waffle_lieupright/2026-11-10_23-48-29/model_250.pt)
TASK=Waffle-LieUprightB2-Direct bash "$W/tools/run_p1_round.sh" r2c "$CK" 750
