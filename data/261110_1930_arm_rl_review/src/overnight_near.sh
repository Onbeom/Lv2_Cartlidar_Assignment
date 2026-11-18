#!/usr/bin/env bash
# 2026-11-06 밤샘 (정지 거리 5 ~ 6 cm): run_all_near.sh 한 번 (원기둥 · 직육면체 45° · 상자 앞 띠 70° 학습 + 평가) 뒤,
#   끝 시각까지 회차 반복 = 세 정책을 가장 최근 체크포인트에서 2048 x 300 씩 차례로 이어 학습 -> 진단 x 3 -> 상자당 2 개 48 x 2 -> 누운 직육면체 24 x 2.
#   회차 정책 policies/{sidepick45,sidepick45box,sidepicknear70}_n<회차>.pt, 요약 overnight_near_summary.log (회차 약 1 시간 40 분)
#   ./overnight_near.sh ["끝 시각 YYYY-mm-dd HH:MM"]
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"; cd "$HERE"
END="${1:-$(date -d 'tomorrow 12:00' '+%Y-%m-%d %H:%M')}"; END_S=$(date -d "$END" +%s); ROUND_MIN=105
SUM="$HERE/overnight_near_summary.log"
KEYS='한 번 시도 기준|팔-몸체 \(1 N|다 채운 대|끝 분류|\*\*성공|팔끼리 충돌|A \(세워'
cleanup() { pkill -f "[t]rain.py --task Waffle-SidePick" 2>/dev/null; pkill -f "[s]idepick_diag.py" 2>/dev/null; pkill -f "[c]hain45_play.py" 2>/dev/null; sleep 5; }
latest_ckpt() { for d in $(ls -d "$HERE/logs/rsl_rl/$1"/*/ | sort -r); do m=$(ls "$d"model_[0-9]*.pt 2>/dev/null | grep -v std05 | sort -V | tail -1); [ -n "$m" ] && { echo "$m"; return; }; done; }
show() { grep -aE "$KEYS" "$2" | sed 's/대마다 연속으로 앉힌 수 \[[^]]*\], //' | sed "s/^/    $1 | /" | cut -c1-220; }
echo "===== $(date '+%F %T') 시작, 끝 $END =====" | tee -a "$SUM"
./run_all_near.sh > overnight_near_first.out 2>&1; cleanup
echo "[$(date '+%T')] 첫 실행 (run_all_near.sh) 끝 -> near_summary.log" | tee -a "$SUM"
for i in $(seq 2 40); do
  [ $(( $(date +%s) + ROUND_MIN * 60 )) -gt "$END_S" ] && { echo "[$(date '+%T')] 끝 시각 전에 한 회차를 못 끝냄 -> 멈춤" | tee -a "$SUM"; break; }
  echo "[$(date '+%T')] 회차 $i 시작" | tee -a "$SUM"
  for t in sidepick45:waffle_sidepick45 sidepick45box:waffle_sidepick45box sidepicknear70:waffle_sidepicknear70; do
    T=${t%%:*}; E=${t##*:}; C=$(latest_ckpt $E)
    timeout -s INT 60m ./run_rl.sh $T 2048 300 "$C" > "on_${T}_r$i.log" 2>&1; cleanup
    echo "    학습 $T ($C) -> $(grep -a 'Learning iteration' on_${T}_r$i.log | tail -1 | tr -s ' ') $(grep -a 'Metrics/success_rate' on_${T}_r$i.log | tail -1 | tr -s ' ')" | tee -a "$SUM"
  done
  for T in sidepick45 sidepick45box sidepicknear70; do
    timeout -s INT 8m ./run_rl.sh ${T}_diag 64 > "on_${T}_diag_r$i.log" 2>&1; cleanup
    cp -f policies/${T}_latest.pt policies/${T}_n$i.pt
    show "진단 $T" "on_${T}_diag_r$i.log" | tee -a "$SUM"
  done
  timeout -s INT 10m ./run_rl.sh chain45_eval 48 --per_box 2 --fill_order lr --occ 1.0 --est --seconds 40 --sidepick "$HERE/policies/sidepick45_n$i.pt" > on_chain_cyl_r$i.log 2>&1; cleanup; show "원기둥 2 개" on_chain_cyl_r$i.log | tee -a "$SUM"
  timeout -s INT 10m ./run_rl.sh chain45_eval 48 --per_box 2 --fill_order lr --occ 1.0 --box_prob 1.0 --est --seconds 40 --sidepick "$HERE/policies/sidepick45box_n$i.pt" > on_chain_box_r$i.log 2>&1; cleanup; show "직육면체 2 개" on_chain_box_r$i.log | tee -a "$SUM"
  for y in radial side; do
    timeout -s INT 10m ./run_rl.sh chain45_eval 24 --lie_regrasp --lie_yaw $y --est --seconds 60 --sidepick "$HERE/policies/sidepick45box_n$i.pt" > on_lie_${y}_r$i.log 2>&1; cleanup; show "누운 $y" on_lie_${y}_r$i.log | tee -a "$SUM"
  done
  echo "[$(date '+%T')] 회차 $i 끝 (정책 policies/*_n$i.pt)" | tee -a "$SUM"
done
echo "===== $(date '+%F %T') 끝 =====" | tee -a "$SUM"
