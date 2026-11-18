#!/usr/bin/env bash
# 2026-11-07 직육면체 연결 0/48 고침: 잡는 높이 조건 (grasp_dz_tol 6 mm) + 시작 잡음 0.08 으로 직육면체 45° 집기 이어 학습 -> 진단 -> 상자당 2 개 48.
#   시작 = b2048 (잡는 높이 평균 16 mm 일 때, 표준편차 0.5 로 맞춘 사본), 2048 x 300 (약 25 분). 요약 boxfix_summary.log
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"; cd "$HERE"
SUM="$HERE/boxfix_summary.log"; NENV="${NENV:-2048}"; ITERS="${ITERS:-300}"
CK="${CK:-$HERE/logs/rsl_rl/waffle_sidepick45box/2026-11-06_20-18-47/model_28939_std05.pt}"
cleanup() { pkill -f "[t]rain.py --task Waffle-SidePick45" 2>/dev/null; pkill -f "[s]idepick_diag.py" 2>/dev/null; pkill -f "[c]hain45_play.py" 2>/dev/null; sleep 5; }
echo "===== $(date '+%F %T') 직육면체 고침 학습 ($CK, $NENV x $ITERS) =====" | tee -a "$SUM"
timeout -s INT 60m ./run_rl.sh sidepick45box "$NENV" "$ITERS" "$CK" > boxfix_train.log 2>&1; cleanup
grep -aq "Traceback" boxfix_train.log && { echo "에러: boxfix_train.log" | tee -a "$SUM"; exit 1; }
echo "[$(date '+%T')] 학습 끝: $(grep -a 'Learning iteration' boxfix_train.log | tail -1 | tr -s ' ') $(grep -a 'Metrics/success_rate' boxfix_train.log | tail -1 | tr -s ' ') $(grep -a 'grasp_dz_err_mm' boxfix_train.log | tail -1 | tr -s ' ')" | tee -a "$SUM"
timeout -s INT 8m ./run_rl.sh sidepick45box_diag 64 > boxfix_diag.log 2>&1; cleanup
grep -aE "한 번 시도 기준|팔-몸체 \(1 N|잡는 점-퍽 중심 높이" boxfix_diag.log | sed "s/^/    /" | cut -c1-220 | tee -a "$SUM"
cp -f policies/sidepick45box_latest.pt policies/sidepick45box_fix1.pt
timeout -s INT 10m ./run_rl.sh chain45_eval 48 --per_box 2 --fill_order lr --occ 1.0 --box_prob 1.0 --est --seconds 40 --sidepick "$HERE/policies/sidepick45box_fix1.pt" > boxfix_chain.log 2>&1; cleanup
grep -aE "다 채운 대|끝 분류|\*\*성공|팔끼리" boxfix_chain.log | sed 's/대마다 연속으로 앉힌 수 \[[^]]*\], //' | sed "s/^/    /" | cut -c1-220 | tee -a "$SUM"
echo "===== $(date '+%F %T') 끝 =====" | tee -a "$SUM"
