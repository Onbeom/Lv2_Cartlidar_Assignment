#!/usr/bin/env bash
# P1 한 회차 (모의, 설계 6-1): 2048 대 학습 (최대 1000 반복, p1_monitor 가 정체 · 오류면 멈춤) -> 정책 내보내기 -> 512 대 평가 (seed 777, 옆 퍽 반반, 처음부터).
#   bash run_p1_round.sh <회차 이름> [시작 체크포인트 .pt] [반복 1000] [환경 2048]
# 결과: results/p1_<이름>_{train,eval}.log, 요약 한 줄씩 results/p1_rounds.txt, 정책 results/p1_<이름>_policy.pt
set -u
NM="$1"; CK="${2:-}"; IT="${3:-1000}"; NE="${4:-2048}"
W="$(cd "$(dirname "$0")/.." && pwd)"
MAN="$W/ws/urdf_and_usd/policy/manipulator"
OUT="$W/results"; mkdir -p "$OUT"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"
PY="$ISAACLAB_DIR/.venv/bin/python"
SUM="$OUT/p1_rounds.txt"
TASK="${TASK:-Waffle-LieUpright-Direct}"
say() { echo "$(date '+%F %T') [$NM] $*" | tee -a "$SUM"; }
exec 9>/tmp/lv2_gpu.lock
if ! flock -w 1800 9; then say "잠금 대기 30 분 초과, 건너뜀"; exit 2; fi
say "시작 (과제 $TASK, 반복 $IT, 환경 $NE, 시작 ${CK:-처음})"
cd "$MAN"
TL="$OUT/p1_${NM}_train.log"
"$PY" train.py --task "$TASK" --num_envs "$NE" --max_iterations "$IT" --viz none ${CK:+--checkpoint "$CK"} > "$TL" 2>&1 &
TP=$!
CHECK="${CHECK:-}" python3 "$W/tools/p1_monitor.py" "$TL" "$TP" "$SUM" 250 0.02 > "$OUT/p1_${NM}_monitor.log" 2>&1
wait $TP 2>/dev/null
sleep 5; pkill -9 -f "[t]rain.py --task Waffle-LieUpright" 2>/dev/null
RUN=$(ls -d "$MAN"/logs/rsl_rl/waffle_lieupright/*/ 2>/dev/null | sort | tail -1)
LAST=$(ls "$RUN"/model_*.pt 2>/dev/null | sort -t_ -k2 -n | tail -1)
say "학습 끝: $RUN $(basename "${LAST:-없음}")"
[ -z "$LAST" ] && { say "체크포인트 없음, 멈춤"; exit 1; }
timeout 300 "$PY" play.py --task "$TASK" --num_envs 1 --checkpoint "$LAST" --viz none > "$OUT/p1_${NM}_export.log" 2>&1 &
EP=$!; until [ -f "$RUN/exported/policy.pt" ] || ! kill -0 $EP 2>/dev/null; do sleep 2; done; sleep 3
pkill -f "[p]lay.py --task Waffle-LieUpright" 2>/dev/null; wait $EP 2>/dev/null
[ -f "$RUN/exported/policy.pt" ] || { say "내보내기 실패"; exit 1; }
cp -f "$RUN/exported/policy.pt" "$OUT/p1_${NM}_policy.pt"
EL="$OUT/p1_${NM}_eval.log"
timeout -k 30s -s INT 20m "$PY" lieupright_eval.py --policy "$OUT/p1_${NM}_policy.pt" --num_envs 512 --seed 777 --nb_prob 0.5 \
  --dump_end "$OUT/p1_${NM}_end.json" > "$EL" 2>&1
grep -aE "\[LUEval\]" "$EL" | grep -v "정책" | sed "s/^/    /" | tee -a "$SUM"
grep -aqE "Traceback" "$EL" && grep -a -A6 Traceback "$EL" | head -12 | tee -a "$SUM"
say "회차 끝 (체크포인트 $LAST)"
