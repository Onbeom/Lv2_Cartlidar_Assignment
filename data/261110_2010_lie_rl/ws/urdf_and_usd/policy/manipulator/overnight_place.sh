#!/usr/bin/env bash
# 2026-11-08 주사위 눈 놓기 밤샘: 회차마다 KINDS (기본 "box l2") 를 차례로 환경 2048 x 500 반복 이어 학습 -> 진단 64.
#   box = L1 · L3 상자 (Waffle-Place60-Direct, 60° 쥐기), l2 = L2 받침 (Waffle-Place45L2-Direct, 45° 쥐기, 첫 학습은 box 의 최근 체크포인트에서 시작).
#   정책 policies/place60_p<회차>.pt · place45l2_p<회차>.pt, 요약 overnight_place_summary.log. 회차 약 2 시간 10 분 (둘 다일 때)
#   ./overnight_place.sh ["끝 시각 YYYY-mm-dd HH:MM"]     KINDS="box" ./overnight_place.sh ...  (하나만)
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"; cd "$HERE"
END="${1:-$(date -d 'tomorrow 09:00' '+%Y-%m-%d %H:%M')}"; END_S=$(date -d "$END" +%s)
KINDS="${KINDS:-box l2}"; NK=$(echo $KINDS | wc -w); ROUND_MIN=$((NK * 65))
SUM="$HERE/overnight_place_summary.log"
cleanup() { pkill -f "[t]rain.py --task Waffle-Place" 2>/dev/null; pkill -f "[p]lace_diag.py" 2>/dev/null; pkill -f "[p]lay.py --task Waffle-Place" 2>/dev/null; sleep 5; }
latest_ckpt() { for d in $(ls -d "$HERE/logs/rsl_rl/$1"/*/ 2>/dev/null | sort -r); do m=$(ls "$d"model_[0-9]*.pt 2>/dev/null | sort -V | tail -1); [ -n "$m" ] && { echo "$m"; return; }; done; }
echo "===== $(date '+%F %T') 시작, 끝 $END, 종류 $KINDS =====" | tee -a "$SUM"
for i in $(seq 1 40); do
  [ $(( $(date +%s) + ROUND_MIN * 60 )) -gt "$END_S" ] && { echo "[$(date '+%T')] 끝 시각 전에 한 회차를 못 끝냄 -> 멈춤" | tee -a "$SUM"; break; }
  for k in $KINDS; do
    if [ $k = l2 ]; then L2V=1; EXP=waffle_place45l2; PN=place45l2; else L2V=0; EXP=waffle_place60; PN=place60; fi
    C=$(latest_ckpt $EXP); [ -z "$C" ] && [ $k = l2 ] && C=$(latest_ckpt waffle_place60)  # L2 첫 학습은 상자 쪽에서 시작 (관측 · 행동 같음)
    echo "[$(date '+%T')] 회차 $i $k 학습 시작 (${C:-처음부터})" | tee -a "$SUM"
    L2=$L2V timeout -s INT 75m ./run_place.sh train 2048 500 $C > "op_${k}_train_r$i.log" 2>&1; cleanup
    if grep -aq "Traceback" "op_${k}_train_r$i.log"; then echo "에러 (op_${k}_train_r$i.log) -> 멈춤" | tee -a "$SUM"; grep -a -A3 "Traceback" "op_${k}_train_r$i.log" | tail -4 | tee -a "$SUM"; exit 1; fi
    echo "    학습 끝: $(grep -a 'Learning iteration' op_${k}_train_r$i.log | tail -1 | tr -s ' ') | $(grep -a 'Metrics/success_rate' op_${k}_train_r$i.log | tail -1 | tr -s ' ') | $(grep -a 'Metrics/held_err_xy_mm' op_${k}_train_r$i.log | tail -1 | tr -s ' ')" | tee -a "$SUM"
    L2=$L2V timeout -s INT 10m ./run_place.sh diag 64 > "op_${k}_diag_r$i.log" 2>&1; cleanup
    cp -f "policies/${PN}_latest.pt" "policies/${PN}_p$i.pt" 2>/dev/null
    grep -aE "PlaceDiag" "op_${k}_diag_r$i.log" | sed "s/^/    /" | tee -a "$SUM"
  done
done
echo "===== $(date '+%F %T') 끝 =====" | tee -a "$SUM"
