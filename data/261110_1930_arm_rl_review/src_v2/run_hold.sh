#!/usr/bin/env bash
# 2026-11-06 '멈춘 뒤 닫기' 규칙 (close_hold_steps 12 = 닫는 명령 시작 뒤 0.4 s 팔 이동 0) + 퍽 자리 무작위 + 밀기 벌점.
# fit1 에서 환경 1024 x 250 반복 (bench: 256 의 2.2 배 처리량), 원기둥 -> 직육면체 차례로 -> 진단 -> 상자당 2 개 48 대.
# 비교 기준 (같은 무작위 배치, run_push.sh): fit1 원기둥 22/48 · 직육면체 12/48, push1 17 · 15.
#   ./run_hold.sh   (약 50 분, 요약 fit_summary.log)   NENV=1024 ITERS=250 TAG=hold1 바꿀 수 있음
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"; cd "$HERE"
SUM="$HERE/fit_summary.log"; TAG="${TAG:-hold1}"; NENV="${NENV:-1024}"; ITERS="${ITERS:-250}"
CK_CYL="${CK_CYL:-$HERE/logs/rsl_rl/waffle_sidepick45/2026-11-06_13-10-50/model_6348.pt}"      # fit1
CK_BOX="${CK_BOX:-$HERE/logs/rsl_rl/waffle_sidepick45box/2026-11-06_13-11-51/model_28790.pt}"  # fit1
KEYS='한 번 시도 기준|팔-몸체 \(1 N|다 채운 대|끝 분류|손 안 퍽 가정 오차 \|mm\||팔끼리 충돌'
cleanup() { pkill -f "[t]rain.py --task Waffle-SidePick45" 2>/dev/null; pkill -f "[s]idepick_diag.py" 2>/dev/null; pkill -f "[c]hain45_play.py" 2>/dev/null; sleep 5; }
show() { grep -aE "$KEYS" "$1" | sed 's/대마다 연속으로 앉힌 수 \[[^]]*\], //' | sed "s/^/    /" | cut -c1-230 | tee -a "$SUM"; }
met() { for k in grasp_fit_mm pre_push_mm close_speed_cms success_rate; do printf "%s " "$(grep -a "Metrics/$k" "$1" | tail -1 | tr -s ' ')"; done; }
echo "===== $(date '+%F %T') $TAG 시작 (환경 $NENV x $ITERS 반복, 차례로) =====" | tee -a "$SUM"
for k in cyl box; do
  if [ $k = cyl ]; then T=sidepick45; C="$CK_CYL"; else T=sidepick45box; C="$CK_BOX"; fi
  echo "[$(date '+%T')] 학습 $k 시작 ($C)" | tee -a "$SUM"
  timeout -s INT 40m ./run_rl.sh $T "$NENV" "$ITERS" "$C" > "fit_${TAG}_train_$k.log" 2>&1; cleanup
  grep -aq "Traceback" "fit_${TAG}_train_$k.log" && { echo "에러: fit_${TAG}_train_$k.log" | tee -a "$SUM"; exit 1; }
  echo "[$(date '+%T')] 학습 $k 끝: $(grep -a 'Learning iteration' fit_${TAG}_train_$k.log | tail -1 | tr -s ' ') | $(met fit_${TAG}_train_$k.log)" | tee -a "$SUM"
done
timeout -s INT 8m ./run_rl.sh sidepick45_diag 64 > fit_${TAG}_diag_cyl.log 2>&1; cleanup; show fit_${TAG}_diag_cyl.log
timeout -s INT 8m ./run_rl.sh sidepick45box_diag 64 > fit_${TAG}_diag_box.log 2>&1; cleanup; show fit_${TAG}_diag_box.log
cp -f policies/sidepick45_latest.pt policies/sidepick45_$TAG.pt; cp -f policies/sidepick45box_latest.pt policies/sidepick45box_$TAG.pt
echo "[$(date '+%T')] 원기둥 상자당 2 개 (가정)" | tee -a "$SUM"
timeout -s INT 10m ./run_rl.sh chain45_eval 48 --per_box 2 --fill_order lr --occ 1.0 --est --seconds 40 --sidepick "$HERE/policies/sidepick45_$TAG.pt" > fit_${TAG}_chain_cyl.log 2>&1; cleanup; show fit_${TAG}_chain_cyl.log
echo "[$(date '+%T')] 직육면체 상자당 2 개 (가정)" | tee -a "$SUM"
timeout -s INT 10m ./run_rl.sh chain45_eval 48 --per_box 2 --fill_order lr --occ 1.0 --box_prob 1.0 --est --seconds 40 --sidepick "$HERE/policies/sidepick45box_$TAG.pt" > fit_${TAG}_chain_box.log 2>&1; cleanup; show fit_${TAG}_chain_box.log
echo "===== $(date '+%F %T') $TAG 끝 =====" | tee -a "$SUM"
