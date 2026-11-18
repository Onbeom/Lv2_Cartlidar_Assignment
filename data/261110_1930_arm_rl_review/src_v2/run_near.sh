#!/usr/bin/env bash
# 2026-11-06 정지 거리 5 ~ 6 cm (사용자 확정) 학습: 퍽 자리 무작위 (관절1 축 0.18 ~ 0.365 중 상자 위), 시작 자세 퍽 위로 올림.
#   원기둥 fit1 -> 2048 x 300, 직육면체 b2048 (행동 표준편차 21 -> 0.5 로 맞춘 사본) -> 2048 x 300, 차례로
#   -> 진단 64 x 2 -> 상자당 2 개 48 x 2 -> 누운 직육면체 세로 · 가로 24 x 2. 요약 near_summary.log (약 1 시간 20 분)
#   NENV=2048 ITERS=300 TAG=near1 바꿀 수 있음
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"; cd "$HERE"
SUM="$HERE/near_summary.log"; TAG="${TAG:-near1}"; NENV="${NENV:-2048}"; ITERS="${ITERS:-300}"
CK_CYL="${CK_CYL:-$HERE/logs/rsl_rl/waffle_sidepick45/2026-11-06_13-10-50/model_6348.pt}"            # fit1
CK_BOX="${CK_BOX:-$HERE/logs/rsl_rl/waffle_sidepick45box/2026-11-06_20-18-47/model_28939_std05.pt}"  # b2048, 표준편차 0.5
KEYS='한 번 시도 기준|팔-몸체 \(1 N|다 채운 대|끝 분류|\*\*성공|팔끼리 충돌|dock|A \(세워|자기 충돌 근접'
cleanup() { pkill -f "[t]rain.py --task Waffle-SidePick45" 2>/dev/null; pkill -f "[s]idepick_diag.py" 2>/dev/null; pkill -f "[c]hain45_play.py" 2>/dev/null; sleep 5; }
show() { grep -aE "$KEYS" "$1" | sed 's/대마다 연속으로 앉힌 수 \[[^]]*\], //' | sed "s/^/    /" | cut -c1-230 | tee -a "$SUM"; }
met() { for k in success_rate grasp_fit_mm pre_push_mm nb_touch_ratio toppled_ratio arm_body_ratio self_bad_ratio; do printf "%s " "$(grep -a "Metrics/$k" "$1" | tail -1 | tr -s ' ')"; done; printf "std %s" "$(grep -a 'Mean action std' "$1" | tail -1 | tr -s ' ')"; }
echo "===== $(date '+%F %T') $TAG 시작 (정지 5 ~ 6 cm, 환경 $NENV x $ITERS, 차례로) =====" | tee -a "$SUM"
for k in cyl box; do
  if [ $k = cyl ]; then T=sidepick45; C="$CK_CYL"; else T=sidepick45box; C="$CK_BOX"; fi
  echo "[$(date '+%T')] 학습 $k 시작 ($C)" | tee -a "$SUM"
  timeout -s INT 60m ./run_rl.sh $T "$NENV" "$ITERS" "$C" > "${TAG}_train_$k.log" 2>&1; cleanup
  grep -aq "Traceback" "${TAG}_train_$k.log" && { echo "에러: ${TAG}_train_$k.log" | tee -a "$SUM"; exit 1; }
  echo "[$(date '+%T')] 학습 $k 끝: $(grep -a 'Learning iteration' ${TAG}_train_$k.log | tail -1 | tr -s ' ') | $(met ${TAG}_train_$k.log)" | tee -a "$SUM"
done
timeout -s INT 8m ./run_rl.sh sidepick45_diag 64 > ${TAG}_diag_cyl.log 2>&1; cleanup; show ${TAG}_diag_cyl.log
timeout -s INT 8m ./run_rl.sh sidepick45box_diag 64 > ${TAG}_diag_box.log 2>&1; cleanup; show ${TAG}_diag_box.log
cp -f policies/sidepick45_latest.pt policies/sidepick45_$TAG.pt; cp -f policies/sidepick45box_latest.pt policies/sidepick45box_$TAG.pt
for a in cyl box; do
  echo "[$(date '+%T')] $a 상자당 2 개 (가정)" | tee -a "$SUM"
  if [ $a = cyl ]; then X="--sidepick $HERE/policies/sidepick45_$TAG.pt"; else X="--box_prob 1.0 --sidepick $HERE/policies/sidepick45box_$TAG.pt"; fi
  timeout -s INT 10m ./run_rl.sh chain45_eval 48 --per_box 2 --fill_order lr --occ 1.0 --est --seconds 40 $X > ${TAG}_chain_$a.log 2>&1; cleanup; show ${TAG}_chain_$a.log
done
for y in radial side; do
  echo "[$(date '+%T')] 누운 직육면체 $y" | tee -a "$SUM"
  timeout -s INT 10m ./run_rl.sh chain45_eval 24 --lie_regrasp --lie_yaw $y --est --seconds 60 > ${TAG}_lie_$y.log 2>&1; cleanup; show ${TAG}_lie_$y.log
done
echo "===== $(date '+%F %T') $TAG 끝 =====" | tee -a "$SUM"
