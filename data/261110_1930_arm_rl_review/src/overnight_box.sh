#!/usr/bin/env bash
# 밤새 반복 학습 (2026-11-04): 세운 직육면체 다시 집기 (sidepick45box) 를 500 반복씩 이어 학습하고, 회차마다
#   진단 (sidepick45box_diag 64) + 누운 직육면체 세로 · 가로 연결 평가 (chain45_eval 16 --lie_regrasp --est) 를 돌려 요약에 남김.
# 끝나는 시각 (기본 내일 13:00) 이 되기 전까지 한 회차 (약 23 분) 가 끝날 수 있을 때만 다음 회차를 시작함.
# 회차마다 정책을 policies/sidepick45box_r<회차>.pt 로 남김 -> 아침에 overnight_summary.log 를 보고 가장 좋은 회차를 고름.
#
#   ./overnight_box.sh [끝나는 시각 "YYYY-mm-dd HH:MM"] [최대 회차]
#   예) ./overnight_box.sh "2026-11-05 13:00" 40
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"
END="${1:-$(date -d 'tomorrow 13:00' '+%Y-%m-%d %H:%M')}"
MAX="${2:-40}"
ROUND_MIN=25  # 한 회차 예상 [분] (학습 500 약 12 + 진단 2 + 연결 평가 2 x 5)
END_S=$(date -d "$END" +%s)
SUM="$HERE/overnight_summary.log"
echo "===== $(date '+%F %T') 시작, 끝 $END, 최대 $MAX 회차 =====" | tee -a "$SUM"

cleanup() {  # 시간 제한으로 끊긴 뒤 남은 Isaac python 정리 (다음 회차 GPU 메모리 부족 방지)
  pkill -f "[t]rain.py --task Waffle-SidePick45Box" 2>/dev/null
  pkill -f "[s]idepick_diag.py --box45" 2>/dev/null
  pkill -f "[c]hain45_play.py --num_envs 16 --lie_regrasp" 2>/dev/null
  sleep 5
}

for i in $(seq 1 "$MAX"); do
  NOW=$(date +%s)
  if [ $((NOW + ROUND_MIN * 60)) -gt "$END_S" ]; then
    echo "[$(date '+%T')] 끝 시각 전 한 회차를 못 끝냄 -> 멈춤 (회차 $((i - 1)) 까지)" | tee -a "$SUM"; break
  fi
  L=$(ls -d logs/rsl_rl/waffle_sidepick45box/*/ | sort | tail -1)
  M=$(ls "$L"model_*.pt | sort -V | tail -1)
  echo "[$(date '+%T')] 회차 $i 학습 시작 ($M)" | tee -a "$SUM"
  timeout -s INT 40m ./run_rl.sh sidepick45box 256 500 "$HERE/$M" > "overnight_train_r$i.log" 2>&1; cleanup
  timeout -s INT 8m ./run_rl.sh sidepick45box_diag 64 > "overnight_diag_r$i.log" 2>&1; cleanup
  cp -f policies/sidepick45box_latest.pt "policies/sidepick45box_r$i.pt"
  D1=$(grep -E '^\[Diag\] \*\*한 번 시도' "overnight_diag_r$i.log" | sed 's/\[Diag\] //')
  D2=$(grep -E '^\[Diag\] 든 퍽 기울기' "overnight_diag_r$i.log" | sed 's/\[Diag\] //' | cut -c1-120)
  D3=$(grep -E '^\[Diag\] 팔-몸체' "overnight_diag_r$i.log" | sed 's/\[Diag\] //' | cut -c1-60)
  for y in radial side; do
    timeout -s INT 10m ./run_rl.sh chain45_eval 16 --lie_regrasp --lie_yaw "$y" --est --seconds 60 > "overnight_eval_r${i}_$y.log" 2>&1; cleanup
  done
  E1=$(grep -oE '성공 \(칸에 앉음[^)]*\) [0-9]+/[0-9]+' "overnight_eval_r${i}_radial.log" | grep -oE '[0-9]+/[0-9]+$' | head -1)
  E2=$(grep -oE '성공 \(칸에 앉음[^)]*\) [0-9]+/[0-9]+' "overnight_eval_r${i}_side.log" | grep -oE '[0-9]+/[0-9]+$' | head -1)
  echo "[$(date '+%T')] 회차 $i 끝 | 진단: $D1 | $D3 | $D2 | 연결 세로 ${E1:-?} 가로 ${E2:-?} | 정책 policies/sidepick45box_r$i.pt" | tee -a "$SUM"
done
echo "===== $(date '+%F %T') 끝 =====" | tee -a "$SUM"
