#!/usr/bin/env bash
# 2026-11-05 밤 반복 학습 (주행 정지 기준 배치 + 상자당 2 개): run_dock2.sh 가 끝나길 기다렸다가
#   원기둥 (sidepick45) · 직육면체 (sidepick45box) 를 가장 최근 체크포인트에서 500 반복씩 동시에 이어 학습 ->
#   진단 64 대 x 2 -> 상자당 2 개 평가 24 대 x 2 를 끝 시각까지 반복. 회차마다 정책을 policies/*_d<회차>.pt 로 남김.
#   요약 overnight_dock2_summary.log -> 아침에 가장 좋은 회차를 고름.
#
#   ./overnight_dock2.sh ["끝 시각 YYYY-mm-dd HH:MM"] [최대 회차]
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"
END="${1:-$(date -d 'tomorrow 09:00' '+%Y-%m-%d %H:%M')}"
MAX="${2:-30}"
ROUND_MIN=40  # 한 회차 최대 예상 [분]
END_S=$(date -d "$END" +%s)
SUM="$HERE/overnight_dock2_summary.log"
KEYS='한 번 시도|팔-몸체 \(1 N|연속|끝 분류|\*\*성공'

cleanup() {
  pkill -f "[t]rain.py --task Waffle-SidePick45" 2>/dev/null
  pkill -f "[s]idepick_diag.py" 2>/dev/null
  pkill -f "[c]hain45_play.py" 2>/dev/null
  sleep 5
}
latest_ckpt() {  # 로그 폴더 중 체크포인트가 있는 가장 최근 것의 마지막 model
  for d in $(ls -d "$HERE/logs/rsl_rl/$1"/*/ | sort -r); do
    m=$(ls "$d"model_*.pt 2>/dev/null | sort -V | tail -1)
    [ -n "$m" ] && { echo "$m"; return; }
  done
}

while pgrep -f "[r]un_dock2.sh" > /dev/null; do sleep 30; done  # 앞 실행 (run_dock2.sh) 이 끝날 때까지
echo "===== $(date '+%F %T') 시작, 끝 $END, 최대 $MAX 회차 =====" | tee -a "$SUM"
for i in $(seq 1 "$MAX"); do
  if [ $(( $(date +%s) + ROUND_MIN * 60 )) -gt "$END_S" ]; then
    echo "[$(date '+%T')] 끝 시각 전에 한 회차를 못 끝냄 -> 멈춤 (회차 $((i - 1)) 까지)" | tee -a "$SUM"; break
  fi
  C1=$(latest_ckpt waffle_sidepick45); C2=$(latest_ckpt waffle_sidepick45box)
  echo "[$(date '+%T')] 회차 $i 학습 시작 ($C1 | $C2)" | tee -a "$SUM"
  timeout -s INT 40m ./run_rl.sh sidepick45 256 500 "$C1" > "od2_train_cyl_r$i.log" 2>&1 &
  P1=$!; sleep 60
  timeout -s INT 39m ./run_rl.sh sidepick45box 256 500 "$C2" > "od2_train_box_r$i.log" 2>&1 &
  P2=$!; wait $P1 $P2; cleanup
  timeout -s INT 8m ./run_rl.sh sidepick45_diag 64 > "od2_diag_cyl_r$i.log" 2>&1; cleanup
  timeout -s INT 8m ./run_rl.sh sidepick45box_diag 64 > "od2_diag_box_r$i.log" 2>&1; cleanup
  cp -f policies/sidepick45_latest.pt "policies/sidepick45_d$i.pt"
  cp -f policies/sidepick45box_latest.pt "policies/sidepick45box_d$i.pt"
  timeout -s INT 8m ./run_rl.sh chain45_eval 24 --per_box 2 --fill_order lr --occ 1.0 --est --seconds 40 \
    --sidepick "$HERE/policies/sidepick45_d$i.pt" > "od2_chain_cyl_r$i.log" 2>&1; cleanup
  timeout -s INT 8m ./run_rl.sh chain45_eval 24 --per_box 2 --fill_order lr --occ 1.0 --est --seconds 40 --box_prob 1.0 \
    --sidepick "$HERE/policies/sidepick45box_d$i.pt" > "od2_chain_box_r$i.log" 2>&1; cleanup
  {
    echo "[$(date '+%T')] 회차 $i 끝 (정책 policies/sidepick45_d$i.pt · sidepick45box_d$i.pt)"
    for f in diag_cyl diag_box chain_cyl chain_box; do
      grep -aE "$KEYS" "od2_${f}_r$i.log" | grep -av "단계 \[" | sed "s/^/    $f | /" | cut -c1-220
    done
  } | tee -a "$SUM"
done
echo "===== $(date '+%F %T') 끝 =====" | tee -a "$SUM"
