#!/usr/bin/env bash
# 2026-11-06 정지 거리 5 ~ 6 cm 학습 한 번에:
#   1 · 2) run_near.sh = 원기둥 · 직육면체 45° 집기 (2048 x 300 씩) -> 진단 -> 상자당 2 개 -> 누운 직육면체 세로 · 가로 (다시 집기 = 새 직육면체 정책)
#   3) 상자 앞 띠 (관절1 축 0.12 ~ 0.20 m) 세운 퍽 70° 집기 (원기둥 · 직육면체 반반, 원기둥 fit1 에서 2048 x 300) -> 진단 64
#   요약 near_summary.log (약 1 시간 50 분)
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"; cd "$HERE"
SUM="$HERE/near_summary.log"
./run_near.sh || { echo "run_near.sh 에러로 멈춤" | tee -a "$SUM"; exit 1; }
echo "[$(date '+%T')] 3) 상자 앞 띠 70° 집기 학습 시작" | tee -a "$SUM"
timeout -s INT 60m ./run_rl.sh sidepicknear70 2048 300 > near70_train.log 2>&1
pkill -f "[t]rain.py --task Waffle-SidePickNear70" 2>/dev/null; sleep 5
grep -aq "Traceback" near70_train.log && { echo "에러: near70_train.log" | tee -a "$SUM"; exit 1; }
echo "[$(date '+%T')] 70° 학습 끝: $(grep -a 'Learning iteration' near70_train.log | tail -1 | tr -s ' ') | $(grep -a 'Metrics/success_rate' near70_train.log | tail -1 | tr -s ' ')" | tee -a "$SUM"
timeout -s INT 8m ./run_rl.sh sidepicknear70_diag 64 > near70_diag.log 2>&1; pkill -f "[s]idepick_diag.py" 2>/dev/null; sleep 5
grep -aE "한 번 시도 기준|팔-몸체 \(1 N|자기 충돌 근접|옆 퍽 \(목표" near70_diag.log | sed "s/^/    /" | cut -c1-230 | tee -a "$SUM"
cp -f policies/sidepicknear70_latest.pt policies/sidepicknear70_near1.pt 2>/dev/null
echo "===== $(date '+%F %T') 전체 끝 =====" | tee -a "$SUM"
