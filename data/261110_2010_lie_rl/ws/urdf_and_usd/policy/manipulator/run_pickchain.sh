#!/usr/bin/env bash
# 2026-11-09 세운 퍽 45° 집기 다시 학습 (연결 기준, Waffle-SidePick45Chain-Direct): 인식 치우침 ±1 mm, 성공 = 손 안 퍽 가정 오차 4 mm 안.
#   1) 기준: 원기둥 n5 연결 상자당 2 개 (원기둥 · 직육면체 48 대, --perc_bias 1 = 학습과 같은 조건)
#   2) 회차 반복 (끝 시각까지): 가장 최근 체크포인트 (첫 회차 = n5) 에서 2048 x 300 -> 내보냄 -> 연결 상자당 2 개 원기둥 · 직육면체 48 대
#   정책 policies/sidepick45chain_k<회차>.pt, 요약 pickchain_summary.log (회차 약 30 분)
#   ./run_pickchain.sh ["끝 시각 YYYY-mm-dd HH:MM"]   ITERS=300 바꿀 수 있음
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"; cd "$HERE"
END="${1:-$(date -d '+3 hours' '+%Y-%m-%d %H:%M')}"; END_S=$(date -d "$END" +%s); ITERS="${ITERS:-300}"; ROUND_MIN=35
SUM="$HERE/pickchain_summary.log"
exec 9>"$HERE/.pickchain.lock"; flock -n 9 || { echo "run_pickchain.sh 가 이미 돌고 있음 -> 끝냄 (두 개가 돌면 서로의 학습을 끔)"; exit 1; }  # 2026-11-09
KEYS='다 채운 대|끝 분류|손 안 퍽 가정 오차 \|mm\||팔-몸체 \(1 N'
cleanup() { pkill -9 -f "[t]rain.py --task Waffle-SidePick45Chain" 2>/dev/null; pkill -9 -f "[p]lay.py --task Waffle-SidePick45Chain" 2>/dev/null; pkill -9 -f "[c]hain45_play.py" 2>/dev/null; sleep 5; }
latest_ckpt() { for d in $(ls -d "$HERE/logs/rsl_rl/$1"/*/ 2>/dev/null | sort -r); do m=$(ls "$d"model_[0-9]*.pt 2>/dev/null | sort -V | tail -1); [ -n "$m" ] && { echo "$m"; return; }; done; }
show() { grep -aE "$KEYS" "$2" | sed 's/대마다 연속으로 앉힌 수 \[[^]]*\], //' | sed "s/^/    $1 | /" | cut -c1-220; }
chain2() {  # $1 정책, $2 이름
  for b in 0 1; do
    timeout -k 30s -s INT 10m ./run_rl.sh chain45_eval 48 --per_box 2 --fill_order lr --occ 1.0 --box_prob $b --est --perc_bias 1 --seconds 40 --sidepick "$1" > "pc_${2}_b$b.log" 2>&1; cleanup
    show "$2 $([ $b = 0 ] && echo 원기둥 || echo 직육면체)" "pc_${2}_b$b.log" | tee -a "$SUM"
  done
}
echo "===== $(date '+%F %T') 시작, 끝 $END, 회차마다 $ITERS 반복 =====" | tee -a "$SUM"
cleanup
echo "[$(date '+%T')] 기준 n5 (치우침 ±1 mm)" | tee -a "$SUM"
chain2 "$HERE/policies/sidepick45_n5.pt" base_n5
for i in $(seq 1 40); do
  [ $(( $(date +%s) + ROUND_MIN * 60 )) -gt "$END_S" ] && { echo "[$(date '+%T')] 끝 시각 전에 한 회차를 못 끝냄 -> 멈춤" | tee -a "$SUM"; break; }
  C=$(latest_ckpt waffle_sidepick45chain); C="${C:-$HERE/logs/rsl_rl/waffle_sidepick45/2026-11-07_03-20-58/model_7843.pt}"
  echo "[$(date '+%T')] 회차 $i 학습 ($C)" | tee -a "$SUM"
  timeout -k 30s -s INT 45m ./run_rl.sh sidepick45chain 2048 "$ITERS" "$C" > "pc_train_k$i.log" 2>&1; cleanup
  if grep -aq "Traceback" "pc_train_k$i.log"; then echo "에러 (pc_train_k$i.log) -> 멈춤" | tee -a "$SUM"; grep -a -A3 "Traceback" "pc_train_k$i.log" | tail -4 | tee -a "$SUM"; exit 1; fi
  echo "    학습 끝: $(grep -a 'Learning iteration' pc_train_k$i.log | tail -1 | tr -s ' ') | $(grep -a 'Metrics/success_rate' pc_train_k$i.log | tail -1 | tr -s ' ') | $(grep -a 'Metrics/grasp_fit_mm' pc_train_k$i.log | tail -1 | tr -s ' ')" | tee -a "$SUM"
  timeout -k 30s 6m ./run_rl.sh sidepick45chain_export > "pc_export_k$i.log" 2>&1; cleanup
  cp -f policies/sidepick45chain_latest.pt "policies/sidepick45chain_k$i.pt"
  chain2 "$HERE/policies/sidepick45chain_k$i.pt" k$i
done
echo "===== $(date '+%F %T') 끝 =====" | tee -a "$SUM"
