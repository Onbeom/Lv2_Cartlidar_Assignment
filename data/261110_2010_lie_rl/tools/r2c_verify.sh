#!/usr/bin/env bash
# r2c 재확인 (모의): 512 평가 seed 777 · 778 · 779 -> 이어 R0t 512 (8 대 시험의 3 대는 세워 놓기 IK 경로 없음 = 코드 문제 아님)
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"; MAN="$W/ws/urdf_and_usd/policy/manipulator"; OUT="$W/results"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"; PY="$ISAACLAB_DIR/.venv/bin/python"
SUM="$OUT/p1_rounds.txt"
exec 9>/tmp/lv2_gpu.lock; flock -w 1800 9 || exit 2
cd "$MAN"
for sd in 777 778 779; do
  L="$OUT/p1_r2cv_s$sd.log"
  timeout -k 30s -s INT 20m "$PY" lieupright_eval.py --policy "$OUT/p1_r2c_policy.pt" --num_envs 512 --seed $sd --nb_prob 0.5 > "$L" 2>&1
  echo "$(date '+%F %T') [r2c-v] seed $sd" | tee -a "$SUM"
  grep -aE "\[LUEval\]" "$L" | grep -v "정책" | sed "s/^/    /" | tee -a "$SUM"
  grep -aq Traceback "$L" && grep -a -A8 Traceback "$L" | head -12 | tee -a "$SUM"
done
S2="$OUT/r0_summary.txt"
echo "$(date '+%F %T') R0t 시작 (세운 퍽 자리 기준 512, 8 대 시험 5/8 시작 = 3 대 세워 놓기 IK 없음)" | tee -a "$S2"
timeout -k 30s -s INT 100m ./run_rl.sh chain45_eval 512 --lie_regrasp --lie_layout mission --lie_nb_prob 0.0 --lie_stand_start 1 --lie_stand_at_nb 1 --est --seconds 60 > "$OUT/r0_R0t.log" 2>&1
pkill -9 -f "[c]hain45_play.py" 2>/dev/null; sleep 5
echo "$(date '+%F %T') R0t 끝" | tee -a "$S2"
grep -aE "세운 퍽 기준|경로 계산|끝 분류|\*\*성공|팔-몸체 1 N|A dock:" "$OUT/r0_R0t.log" | sed 's/^/    /' | cut -c1-400 | tee -a "$S2"
grep -aiE -m3 -A3 "Traceback|out of memory" "$OUT/r0_R0t.log" | tee -a "$S2"
